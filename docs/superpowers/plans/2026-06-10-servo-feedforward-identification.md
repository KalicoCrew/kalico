# Servo Feedforward + Identification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement the layers-3/4 design in
`docs/superpowers/specs/2026-06-10-servo-feedforward-identification-design.md`:
host-computed velocity/torque feedforward streamed to the A6-EC servo over
CiA402 offset objects (60B1h/60B2h), and an offline dynamics-identification
toolkit that produces the dynamics profile the runtime consumes.

**Architecture:** Part A extends the EtherCAT endpoint: PDO remap to variable
mapping (adds the two offset objects outbound and 606Ch velocity-actual
inbound), acceleration evaluation from the already-streamed cubic pieces, a
validated TOML dynamics profile, and a per-cycle FF computation with clamping.
Part B is a new std-only Rust crate `servo-ident` with two binaries: a G-code
excitation generator and a least-squares fitter that turns telemetry captures
into dynamics profiles. The parts are independent; the TOML profile format is
their only shared contract.

**Tech Stack:** Rust (workspace at `rust/`, test with `cargo nextest run`),
C (SOEM-based `bench/libecrt.c`, builds only on the Pi), Python (klippy
config plumbing). New deps: `serde` + `toml` for `kalico-ethercat-rt` only.

---

## Context primer (read first)

- **Units convention (from the spec, used everywhere):** motion in mm of the
  motor stream (mm/s, mm/s²); torque in 0.1%-of-rated units (the native unit
  of 6077h/60B2h, i16); velocity offset 60B1h in encoder counts/s
  (`mm/s × counts_per_mm`).
- **Pieces:** `runtime::piece_ring::PieceEntry` = `start_time` (ns), Bézier
  `coeffs: [f32; 4]`, `duration` (s). `entry.to_monomial()` → monomial
  position coeffs `[f32; 4]` and velocity coeffs `[f32; 3]` in seconds since
  piece start. Acceleration is therefore `vel[1] + 2·vel[2]·t`.
- **Endpoint:** `rust/kalico-ethercat-rt/`. The `hw` feature gates FFI +
  the real binary; the lib and stub build anywhere. The hw binary's DC loop
  is `src/bin/kalico-ethercat-rt.rs` (`'dc: loop`). `AxisRing::sample(now)`
  in `src/curves.rs` walks/evaluates the armed piece.
- **C boundary:** `bench/libecrt.c`/`bench/libecrt.h` — SOEM bring-up, PDO
  structs, cycle exchange. **Builds only on the Pi** (`make -f
  Makefile.kalico ethercat-endpoint-hw`); there is no local compile check.
  Follow `docs/kalico-rewrite/mcu-c-rust-boundary.md` rules at the seam.
- **2000h object mapping convention (manual ch. 11):** parameter
  `Cgg.xx` (xx hex) ↔ object index `0x20gg`, subindex `xx + 1`. So
  C01.13 → 0x2001:14h, C01.14 → 0x2001:15h, C01.16 → 0x2001:17h,
  C01.17 → 0x2001:18h. All U16.
- **Tests:** unit tests live in separate files (project rule). Run scoped:
  `cargo nextest run -p <crate>` from `rust/`. Doc-tests need
  `cargo test --doc` separately.
- **Comments:** project rule — comments are a failure of expression; write
  code that says it. Brief comments are acceptable only in `libecrt.c`
  (existing C style) for wire-layout/manual-reference facts.
- **Commits:** no Claude/Anthropic trailers. Conventional prefixes
  (`feat:`, `test:`, `docs:`) as in `git log`.

---

# Part A — runtime feedforward (layer 4)

### Task 1: Acceleration evaluation in `runtime::motion_core`

A new pub function only — existing functions untouched, so MCU stepper
codegen is unaffected (an uncalled function is dropped at link time; the
existing disasm check in CI/bench confirms).

**Files:**
- Create: `rust/runtime/tests/motion_core_accel.rs`
- Modify: `rust/runtime/src/motion_core.rs` (append function)

- [ ] **Step 1: Write the failing test**

```rust
// rust/runtime/tests/motion_core_accel.rs
use runtime::motion_core::{eval_accel, eval_horner};
use runtime::piece_ring::PieceEntry;

fn entry() -> PieceEntry {
    PieceEntry {
        start_time: 5_000_000,
        coeffs: [1.0, 2.5, -3.0, 4.0],
        duration: 0.5,
        _reserved: 0,
    }
}

#[test]
fn accel_matches_velocity_finite_difference() {
    let (mono, vel) = entry().to_monomial();
    let cps = 1.0e9_f32;
    let start = 5_000_000_u64;
    for &t_s in &[0.0_f32, 0.05, 0.2, 0.45] {
        let now = start + (t_s * cps) as u64;
        let h_cycles = 1_000_u64;
        let h_s = h_cycles as f32 / cps;
        let (_, v0) = eval_horner(&mono, &vel, start, now, cps);
        let (_, v1) = eval_horner(&mono, &vel, start, now + h_cycles, cps);
        let fd = (v1 - v0) / h_s;
        let a = eval_accel(&vel, start, now, cps);
        assert!(
            (a - fd).abs() <= 0.05 * fd.abs().max(1.0),
            "t={t_s}: accel {a} vs finite-diff {fd}"
        );
    }
}

#[test]
fn accel_is_linear_in_time() {
    let (_, vel) = entry().to_monomial();
    let cps = 1.0e9_f32;
    let a0 = eval_accel(&vel, 0, 0, cps);
    let a1 = eval_accel(&vel, 0, (0.1 * cps) as u64, cps);
    let a2 = eval_accel(&vel, 0, (0.2 * cps) as u64, cps);
    assert!((a2 - a1 - (a1 - a0)).abs() < 1e-3, "{a0} {a1} {a2}");
}

#[test]
fn accel_clamps_before_piece_start() {
    let (_, vel) = entry().to_monomial();
    assert_eq!(eval_accel(&vel, 1000, 500, 1.0e9), vel[1]);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p runtime -E 'test(accel)'`
Expected: compile FAIL — `eval_accel` not found.

- [ ] **Step 3: Implement**

Append to `rust/runtime/src/motion_core.rs`:

```rust
#[inline]
pub fn eval_accel(
    vel: &[f32; 3],
    piece_start_cycles: u64,
    now: u64,
    cycles_per_second: f32,
) -> f32 {
    let elapsed_cycles = now.saturating_sub(piece_start_cycles);
    let t = if cycles_per_second > 0.0 {
        elapsed_cycles as f32 / cycles_per_second
    } else {
        0.0_f32
    };
    vel[1] + 2.0 * t * vel[2]
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd rust && cargo nextest run -p runtime -E 'test(accel)'`
Expected: 3 PASS.

- [ ] **Step 5: Run the full runtime suite (hot-path safety net)**

Run: `cd rust && cargo nextest run -p runtime`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/motion_core.rs rust/runtime/tests/motion_core_accel.rs
git commit -m "feat(runtime): second-derivative evaluation of armed cubic pieces"
```

### Task 2: `AxisRing::sample` returns acceleration

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/curves.rs` (`sample`)
- Modify: `rust/kalico-ethercat-rt/src/curves/tests.rs` (existing callers)
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs:254`
  (destructure 3-tuple)

- [ ] **Step 1: Extend an existing sample test to assert acceleration**

In `rust/kalico-ethercat-rt/src/curves/tests.rs`, find the existing tests
calling `ring.sample(...)` (they currently destructure `(pos, vel)`). Update
all call sites to the 3-tuple. Add one new test (adapt the piece-construction
helper the file already uses — push a piece with known coeffs, then):

```rust
#[test]
fn sample_reports_acceleration_consistent_with_velocity_slope() {
    let mut ring = AxisRing::new();
    // push one piece exactly as the existing position/velocity tests do,
    // with start_time T0 and a duration covering the probes below
    let t0 = /* piece start ns, same value used when pushing */;
    let dt = 1_000_000_u64;
    let (_, v0, a0) = ring.sample(t0 + 10 * dt).expect("in piece");
    let (_, v1, _) = ring.sample(t0 + 11 * dt).expect("in piece");
    let fd = (v1 - v0) / (dt as f32 / 1.0e9);
    assert!((a0 - fd).abs() <= 0.05 * fd.abs().max(1.0), "{a0} vs {fd}");
}
```

(The literal piece-push lines must be copied from the neighboring test in
that file so the entry layout stays consistent with `parse_piece_entry`.)

- [ ] **Step 2: Run to verify it fails**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt`
Expected: compile FAIL — `sample` returns a 2-tuple.

- [ ] **Step 3: Implement**

In `rust/kalico-ethercat-rt/src/curves.rs`:

```rust
use runtime::motion_core::{eval_accel, get_position_and_velocity, ArmedPiece};
```

```rust
    pub fn sample(&mut self, now_ns: u64) -> Option<(f32, f32, f32)> {
        let AxisRing {
            ref mut armed,
            ref mut desc,
            ref storage,
            ref fault,
            ..
        } = *self;
        let sink = EtherCatFaultSink { reg: fault };
        let (pos, vel) = get_position_and_velocity(
            armed,
            desc,
            storage,
            now_ns,
            EC_DC_PERIOD_NS,
            CLOCK_FREQ_HZ,
            EC_AXIS_IDX,
            &sink,
        )?;
        let p = armed
            .as_ref()
            .expect("sample yielded a value with no armed piece");
        let acc = eval_accel(&p.vel_coeffs, p.piece_start_cycles, now_ns, CLOCK_FREQ_HZ);
        Some((pos, vel, acc))
    }
```

In the hw binary (`src/bin/kalico-ethercat-rt.rs:254`), change the
destructure to `if let Some((pos_mm, _vel_mm_s, _acc_mm_s2)) = ring.sample(now)`
(the FF consumption arrives in Task 6).

- [ ] **Step 4: Run to verify it passes**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt`
Expected: all PASS. (The hw binary itself only compiles on the Pi with
`--features hw`; the lib tests cover `sample`.)

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-ethercat-rt/src/curves.rs rust/kalico-ethercat-rt/src/curves/tests.rs rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs
git commit -m "feat(ethercat): AxisRing::sample yields acceleration"
```

### Task 3: Dynamics profile — parse, validate, evaluate

**Files:**
- Create: `rust/kalico-ethercat-rt/src/dynamics.rs`
- Create: `rust/kalico-ethercat-rt/src/dynamics/tests.rs`
- Modify: `rust/kalico-ethercat-rt/src/lib.rs` (add `pub mod dynamics;`)
- Modify: `rust/kalico-ethercat-rt/Cargo.toml` (deps)

- [ ] **Step 1: Add dependencies**

In `rust/kalico-ethercat-rt/Cargo.toml` `[dependencies]`:

```toml
serde = { version = "1", features = ["derive"] }
toml = "0.8"
```

- [ ] **Step 2: Write the failing tests**

```rust
// rust/kalico-ethercat-rt/src/dynamics/tests.rs
use super::*;

const SCALAR: &str = r#"
version = 1
axes = ["x"]
mass = [[0.0123]]
viscous = [0.0045]
coulomb_fwd = [1.2]
coulomb_rev = [-1.1]
coulomb_deadband_mm_s = 0.5
fit_rms_residual = [0.8]
"#;

const COREXY: &str = r#"
version = 1
axes = ["a", "b"]
mass = [[0.030, -0.010], [-0.010, 0.030]]
viscous = [0.004, 0.004]
coulomb_fwd = [1.0, 1.0]
coulomb_rev = [-1.0, -1.0]
coulomb_deadband_mm_s = 0.5
fit_rms_residual = [0.5, 0.5]
"#;

#[test]
fn parses_scalar_profile() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    assert_eq!(m.n, 1);
}

#[test]
fn torque_ff_scalar() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let tau = m.torque_ff(0, &[1000.0], &[100.0]);
    let expect = 0.0123 * 1000.0 + 0.0045 * 100.0 + 1.2;
    assert!((tau - expect).abs() < 1e-4, "{tau} vs {expect}");
}

#[test]
fn torque_ff_reverse_coulomb_and_deadband() {
    let m = DynamicsModel::from_toml_str(SCALAR).unwrap();
    let rev = m.torque_ff(0, &[0.0], &[-100.0]);
    assert!((rev - (0.0045 * -100.0 + -1.1)).abs() < 1e-4);
    let dead = m.torque_ff(0, &[0.0], &[0.1]);
    assert!((dead - 0.0045 * 0.1).abs() < 1e-4, "no coulomb inside deadband");
}

#[test]
fn corexy_effective_inertia_is_direction_dependent() {
    let m = DynamicsModel::from_toml_str(COREXY).unwrap();
    let x_move = m.torque_ff(0, &[1000.0, 1000.0], &[0.0, 0.0]);
    let y_move = m.torque_ff(0, &[1000.0, -1000.0], &[0.0, 0.0]);
    assert!((x_move - 20.0).abs() < 1e-3); // (0.030 - 0.010) * 1000
    assert!((y_move - 40.0).abs() < 1e-3); // (0.030 + 0.010) * 1000
}

#[test]
fn rejects_each_invariant_violation() {
    let bad_version = SCALAR.replace("version = 1", "version = 2");
    assert!(matches!(
        DynamicsModel::from_toml_str(&bad_version),
        Err(ProfileError::Version(2))
    ));
    let bad_dim = SCALAR.replace("viscous = [0.0045]", "viscous = [0.0045, 1.0]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&bad_dim),
        Err(ProfileError::Dim(_))
    ));
    let asym = COREXY.replace("[-0.010, 0.030]", "[-0.011, 0.030]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&asym),
        Err(ProfileError::NotSymmetric)
    ));
    let not_pd = SCALAR.replace("mass = [[0.0123]]", "mass = [[-0.0123]]");
    assert!(matches!(
        DynamicsModel::from_toml_str(&not_pd),
        Err(ProfileError::NotPositiveDefinite)
    ));
    let nan = SCALAR.replace("viscous = [0.0045]", "viscous = [nan]");
    assert!(DynamicsModel::from_toml_str(&nan).is_err());
    assert!(matches!(
        DynamicsModel::from_toml_str("not toml ["),
        Err(ProfileError::Parse(_))
    ));
}

#[test]
fn clamp_counts_saturation() {
    let mut sat = 0u32;
    assert_eq!(clamp_torque(50.0, 300, &mut sat), 50);
    assert_eq!(sat, 0);
    assert_eq!(clamp_torque(450.7, 300, &mut sat), 300);
    assert_eq!(clamp_torque(-450.7, 300, &mut sat), -300);
    assert_eq!(sat, 2);
}
```

- [ ] **Step 3: Run to verify failure**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt -E 'test(dynamics) or test(torque_ff) or test(clamp) or test(corexy) or test(profile)'`
Expected: compile FAIL — module missing.

- [ ] **Step 4: Implement**

```rust
// rust/kalico-ethercat-rt/src/dynamics.rs
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct ProfileFile {
    version: u32,
    axes: Vec<String>,
    mass: Vec<Vec<f64>>,
    viscous: Vec<f64>,
    coulomb_fwd: Vec<f64>,
    coulomb_rev: Vec<f64>,
    coulomb_deadband_mm_s: f64,
    #[allow(dead_code)]
    fit_rms_residual: Vec<f64>,
}

#[derive(Debug)]
pub enum ProfileError {
    Parse(String),
    Version(u32),
    Dim(&'static str),
    NotFinite(&'static str),
    NotSymmetric,
    NotPositiveDefinite,
}

#[derive(Debug)]
pub struct DynamicsModel {
    pub n: usize,
    pub axes: Vec<String>,
    mass: Vec<f32>,
    viscous: Vec<f32>,
    coulomb_fwd: Vec<f32>,
    coulomb_rev: Vec<f32>,
    deadband: f32,
}

impl DynamicsModel {
    pub fn from_toml_str(s: &str) -> Result<Self, ProfileError> {
        let f: ProfileFile =
            toml::from_str(s).map_err(|e| ProfileError::Parse(e.to_string()))?;
        if f.version != 1 {
            return Err(ProfileError::Version(f.version));
        }
        let n = f.axes.len();
        if n == 0 {
            return Err(ProfileError::Dim("axes is empty"));
        }
        if f.mass.len() != n || f.mass.iter().any(|row| row.len() != n) {
            return Err(ProfileError::Dim("mass must be n x n"));
        }
        if f.viscous.len() != n {
            return Err(ProfileError::Dim("viscous length"));
        }
        if f.coulomb_fwd.len() != n {
            return Err(ProfileError::Dim("coulomb_fwd length"));
        }
        if f.coulomb_rev.len() != n {
            return Err(ProfileError::Dim("coulomb_rev length"));
        }
        let mass: Vec<f64> = f.mass.iter().flatten().copied().collect();
        let all = mass
            .iter()
            .chain(&f.viscous)
            .chain(&f.coulomb_fwd)
            .chain(&f.coulomb_rev)
            .chain(std::iter::once(&f.coulomb_deadband_mm_s));
        if all.into_iter().any(|v| !v.is_finite()) {
            return Err(ProfileError::NotFinite("profile contains non-finite value"));
        }
        for i in 0..n {
            for j in (i + 1)..n {
                let (a, b) = (mass[i * n + j], mass[j * n + i]);
                if (a - b).abs() > 1e-9 * a.abs().max(b.abs()).max(1e-12) {
                    return Err(ProfileError::NotSymmetric);
                }
            }
        }
        if !cholesky_is_pd(&mass, n) {
            return Err(ProfileError::NotPositiveDefinite);
        }
        Ok(Self {
            n,
            axes: f.axes,
            mass: mass.iter().map(|&v| v as f32).collect(),
            viscous: f.viscous.iter().map(|&v| v as f32).collect(),
            coulomb_fwd: f.coulomb_fwd.iter().map(|&v| v as f32).collect(),
            coulomb_rev: f.coulomb_rev.iter().map(|&v| v as f32).collect(),
            deadband: f.coulomb_deadband_mm_s as f32,
        })
    }

    pub fn torque_ff(&self, axis: usize, acc_mm_s2: &[f32], vel_mm_s: &[f32]) -> f32 {
        assert_eq!(acc_mm_s2.len(), self.n);
        assert_eq!(vel_mm_s.len(), self.n);
        assert!(axis < self.n);
        let inertial: f32 = (0..self.n)
            .map(|j| self.mass[axis * self.n + j] * acc_mm_s2[j])
            .sum();
        let v = vel_mm_s[axis];
        let coulomb = if v > self.deadband {
            self.coulomb_fwd[axis]
        } else if v < -self.deadband {
            self.coulomb_rev[axis]
        } else {
            0.0
        };
        inertial + self.viscous[axis] * v + coulomb
    }
}

fn cholesky_is_pd(m: &[f64], n: usize) -> bool {
    let mut l = m.to_vec();
    for k in 0..n {
        for j in 0..k {
            l[k * n + k] -= l[k * n + j] * l[k * n + j];
        }
        if l[k * n + k] <= 0.0 {
            return false;
        }
        l[k * n + k] = l[k * n + k].sqrt();
        for i in (k + 1)..n {
            for j in 0..k {
                l[i * n + k] -= l[i * n + j] * l[k * n + j];
            }
            l[i * n + k] /= l[k * n + k];
        }
    }
    true
}

pub fn clamp_torque(raw_tenths_pct: f32, limit_tenths_pct: i16, saturation_count: &mut u32) -> i16 {
    let lim = f32::from(limit_tenths_pct);
    if raw_tenths_pct > lim {
        *saturation_count += 1;
        limit_tenths_pct
    } else if raw_tenths_pct < -lim {
        *saturation_count += 1;
        -limit_tenths_pct
    } else {
        raw_tenths_pct as i16
    }
}

#[cfg(test)]
mod tests;
```

Add `pub mod dynamics;` to `rust/kalico-ethercat-rt/src/lib.rs`.

- [ ] **Step 5: Run to verify pass**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/kalico-ethercat-rt/Cargo.toml rust/kalico-ethercat-rt/src/lib.rs rust/kalico-ethercat-rt/src/dynamics.rs rust/kalico-ethercat-rt/src/dynamics/tests.rs rust/Cargo.lock
git commit -m "feat(ethercat): dynamics profile parsing, validation, torque FF evaluation"
```

### Task 4: StatusHeartbeat carries the FF saturation counter

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs:324` (struct + encode/decode)
- Modify: `rust/kalico-protocol/src/messages/tests.rs` (roundtrip)
- Modify: `rust/kalico-ethercat-rt/src/wire.rs:120` (frame builder)
- Modify: callers of `status_heartbeat_frame`:
  `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs` (2 sites),
  `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt-stub.rs` (3 sites:
  lines ~203, ~221, ~233), plus any `wire/tests.rs` uses.
- Check: `grep -rn "StatusHeartbeat {" rust/` for other constructors
  (host-rt only decodes; decoding sites need no change).

- [ ] **Step 1: Update the protocol roundtrip test first**

In `rust/kalico-protocol/src/messages/tests.rs`, find the existing
`StatusHeartbeat` roundtrip/decode tests and add the field with a nonzero
value, e.g. `ff_saturation_count: 7`, asserting it survives
encode→decode. If a test constructs the struct literally, the missing field
is the compile failure we want.

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p kalico-protocol`
Expected: compile FAIL — missing field.

- [ ] **Step 3: Implement**

`messages.rs` — add `pub ff_saturation_count: u32` as the last struct field;
in `Encode::encode` append `put_u32(out, self.ff_saturation_count);` after
the counts loop; in `Decode::decode_from` read
`let ff_saturation_count = get_u32(c)?;` after the counts loop and include
it in `Ok(Self { ... })`. The field is mandatory — both ends version
together; a short frame is a decode error (fail loudly).

`wire.rs`:

```rust
pub fn status_heartbeat_frame(
    engine_state: u8,
    retired_counts: &[u32],
    ff_saturation_count: u32,
) -> Vec<u8> {
    let hb = StatusHeartbeat {
        engine_state,
        fault_code: 0,
        retired_counts: retired_counts.to_vec(),
        ff_saturation_count,
    };
    // rest unchanged
```

Callers: stub passes `0` at all 3 sites; hw binary passes `0` for now
(Task 6 threads the real counter). Fix any wire/integration tests that call
the builder (`grep -rn "status_heartbeat_frame" rust/`).

- [ ] **Step 4: Run the workspace suite**

Run: `cd rust && cargo nextest run`
Expected: all PASS (this catches every decoder/constructor in
kalico-host-rt, motion-bridge, and the endpoint tests).

- [ ] **Step 5: Commit**

```bash
git add -A rust/
git commit -m "feat(protocol): StatusHeartbeat carries FF saturation counter"
```

### Task 5: libecrt — variable PDO mapping, FF objects, accessors

C-side change; **compiles only on the Pi**. Local verification is the
static asserts + careful diff review; the Pi build happens in Task 14.

**Files:**
- Modify: `bench/libecrt.c`
- Modify: `bench/libecrt.h`
- Modify: `rust/kalico-ethercat-rt/src/ffi.rs`

- [ ] **Step 1: Update the PDO structs and layout comment**

Replace the layout comment and structs in `bench/libecrt.c`:

```c
/*
 * PDO layout must match the variable mapping written in pdo_remap().
 *
 * RxPDO 0x1600 (18 bytes):
 *   controlword      6040  uint16
 *   target_position  607A  int32
 *   touch_probe_fn   60B8  uint16
 *   phys_outputs     60FE:01 uint32
 *   velocity_offset  60B1  int32   (counts/s, speed FF when C01.13=5)
 *   torque_offset    60B2  int16   (0.1% rated, torque FF when C01.16=5)
 *
 * TxPDO 0x1A00 (32 bytes):
 *   error_code       603F  uint16
 *   statusword       6041  uint16
 *   position_actual  6064  int32
 *   torque_actual    6077  int16
 *   following_error  60F4  int32
 *   tp_status        60B9  uint16
 *   tp1_pos          60BA  int32
 *   tp2_pos          60BC  int32
 *   digital_inputs   60FD  uint32
 *   velocity_actual  606C  int32
 */
#pragma pack(push, 1)
typedef struct {
    uint16_t controlword;
    int32_t  target_position;
    uint16_t touch_probe_fn;
    uint32_t phys_outputs;
    int32_t  velocity_offset;
    int16_t  torque_offset;
} out_t;
typedef struct {
    uint16_t error_code;
    uint16_t statusword;
    int32_t  position_actual;
    int16_t  torque_actual;
    int32_t  following_error;
    uint16_t tp_status;
    int32_t  tp1_pos;
    int32_t  tp2_pos;
    uint32_t digital_inputs;
    int32_t  velocity_actual;
} in_t;
#pragma pack(pop)
_Static_assert(sizeof(out_t) == 18, "RxPDO 0x1600 mapping is 18 bytes");
_Static_assert(sizeof(in_t)  == 32, "TxPDO 0x1A00 mapping is 32 bytes");
```

- [ ] **Step 2: Add the remap + FF-routing SDO writes to bring-up**

Add above `ec_rt_bringup`:

```c
/* Variable PDO mapping (manual 8.3.1): only in PRE-OP, not EEPROM-retained,
 * so it is rewritten on every bring-up. Sequence: clear SM assignment ->
 * clear map -> write entries -> write entry count -> reassign SM. */
static int pdo_remap(void) {
    static const uint32_t rx[6] = {
        0x60400010, 0x607A0020, 0x60B80010, 0x60FE0120, 0x60B10020, 0x60B20010,
    };
    static const uint32_t tx[10] = {
        0x603F0010, 0x60410010, 0x60640020, 0x60770010, 0x60F40020,
        0x60B90010, 0x60BA0020, 0x60BC0020, 0x60FD0020, 0x606C0020,
    };
    uint8_t  zero = 0, cnt;
    uint16_t pdo;
    int ok = 1, i;

    ok &= ec_SDOwrite(1, 0x1C12, 0x00, FALSE, sizeof zero, &zero, EC_TIMEOUTRXM) > 0;
    ok &= ec_SDOwrite(1, 0x1600, 0x00, FALSE, sizeof zero, &zero, EC_TIMEOUTRXM) > 0;
    for (i = 0; i < 6; i++)
        ok &= ec_SDOwrite(1, 0x1600, (uint8_t)(i + 1), FALSE, sizeof rx[i], (void *)&rx[i], EC_TIMEOUTRXM) > 0;
    cnt = 6;
    ok &= ec_SDOwrite(1, 0x1600, 0x00, FALSE, sizeof cnt, &cnt, EC_TIMEOUTRXM) > 0;
    pdo = 0x1600;
    ok &= ec_SDOwrite(1, 0x1C12, 0x01, FALSE, sizeof pdo, &pdo, EC_TIMEOUTRXM) > 0;
    cnt = 1;
    ok &= ec_SDOwrite(1, 0x1C12, 0x00, FALSE, sizeof cnt, &cnt, EC_TIMEOUTRXM) > 0;

    ok &= ec_SDOwrite(1, 0x1C13, 0x00, FALSE, sizeof zero, &zero, EC_TIMEOUTRXM) > 0;
    ok &= ec_SDOwrite(1, 0x1A00, 0x00, FALSE, sizeof zero, &zero, EC_TIMEOUTRXM) > 0;
    for (i = 0; i < 10; i++)
        ok &= ec_SDOwrite(1, 0x1A00, (uint8_t)(i + 1), FALSE, sizeof tx[i], (void *)&tx[i], EC_TIMEOUTRXM) > 0;
    cnt = 10;
    ok &= ec_SDOwrite(1, 0x1A00, 0x00, FALSE, sizeof cnt, &cnt, EC_TIMEOUTRXM) > 0;
    pdo = 0x1A00;
    ok &= ec_SDOwrite(1, 0x1C13, 0x01, FALSE, sizeof pdo, &pdo, EC_TIMEOUTRXM) > 0;
    cnt = 1;
    ok &= ec_SDOwrite(1, 0x1C13, 0x00, FALSE, sizeof cnt, &cnt, EC_TIMEOUTRXM) > 0;

    return ok ? 0 : -6;
}

/* FF sources to "communication" (60B1h/60B2h) at 100.0% scale.
 * C01.13 -> 0x2001:14h, C01.14 -> 0x2001:15h, C01.16 -> 0x2001:17h,
 * C01.17 -> 0x2001:18h (group C01 = index 2001h, subindex = param + 1). */
static int ff_routing(void) {
    uint16_t src = 5, pct = 1000;
    int ok = 1;
    ok &= ec_SDOwrite(1, 0x2001, 0x14, FALSE, sizeof src, &src, EC_TIMEOUTRXM) > 0;
    ok &= ec_SDOwrite(1, 0x2001, 0x15, FALSE, sizeof pct, &pct, EC_TIMEOUTRXM) > 0;
    ok &= ec_SDOwrite(1, 0x2001, 0x17, FALSE, sizeof src, &src, EC_TIMEOUTRXM) > 0;
    ok &= ec_SDOwrite(1, 0x2001, 0x18, FALSE, sizeof pct, &pct, EC_TIMEOUTRXM) > 0;
    return ok ? 0 : -7;
}
```

In `ec_rt_bringup`, after the existing five SDO writes (opmode + sync) and
**before** `ec_configdc()`:

```c
    int rc = pdo_remap();
    if (rc != 0) { ec_close(); return rc; }
    rc = ff_routing();
    if (rc != 0) { ec_close(); return rc; }
```

Update the zeroing block after `g_out`/`g_in` assignment:

```c
    g_out->velocity_offset = 0;
    g_out->torque_offset   = 0;
```

and add the same two lines inside `ec_rt_disable`'s loop body (offsets must
not persist across a disable).

- [ ] **Step 3: Add accessors**

`bench/libecrt.c` (bottom, next to the existing accessors):

```c
void ec_rt_set_velocity_offset(int32_t counts_per_s) { g_out->velocity_offset = counts_per_s; }
void ec_rt_set_torque_offset(int16_t tenths_pct)     { g_out->torque_offset  = tenths_pct; }
int32_t ec_rt_get_velocity_actual(void)              { return g_in->velocity_actual; }
int16_t ec_rt_get_torque_actual(void)                { return g_in->torque_actual; }
```

`bench/libecrt.h` (after `ec_rt_get_following_error`; also extend the
bring-up doc comment's rc list with `-6 PDO remap, -7 FF routing`):

```c
/* Stage CiA402 offsets for the next cycle's send (zeroed at bring-up and
 * on disable). Velocity in encoder counts/s, torque in 0.1% of rated. */
void ec_rt_set_velocity_offset(int32_t counts_per_s);
void ec_rt_set_torque_offset(int16_t tenths_pct);
int32_t ec_rt_get_velocity_actual(void);
int16_t ec_rt_get_torque_actual(void);
```

`rust/kalico-ethercat-rt/src/ffi.rs` (inside the `extern "C"` block):

```rust
    pub fn ec_rt_set_velocity_offset(counts_per_s: i32);

    pub fn ec_rt_set_torque_offset(tenths_pct: i16);

    pub fn ec_rt_get_velocity_actual() -> i32;

    pub fn ec_rt_get_torque_actual() -> i16;
```

- [ ] **Step 4: Local check + commit**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt` (lib unaffected,
must stay green; the C file and `--features hw` build are Pi-verified in
Task 14).

```bash
git add bench/libecrt.c bench/libecrt.h rust/kalico-ethercat-rt/src/ffi.rs
git commit -m "feat(ethercat): variable PDO mapping with 60B1/60B2 FF offsets + 606C velocity actual"
```

### Task 6: Endpoint — flags, profile load, per-cycle FF

**Files:**
- Modify: `rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs`

No local test runner covers the hw binary (it needs `--features hw` + Pi
libs); correctness here leans on Task 3's tested dynamics module and the
Pi build in Task 14. Keep this diff minimal and mechanical.

- [ ] **Step 1: Args and profile load (before `ec_rt_bringup`)**

Extend the usage doc-comment at the top with the new flags, then after the
existing arg parsing:

```rust
    let velocity_ff = args.iter().any(|a| a == "--velocity-ff");
    let torque_clamp_tenths: i16 = arg_val(&args, "--torque-clamp-pct")
        .and_then(|s| s.parse::<f64>().ok())
        .map(|pct| (pct * 10.0) as i16)
        .unwrap_or(300);
    let dynamics = arg_val(&args, "--dynamics-profile").map(|path| {
        let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
            eprintln!("ec-rt: dynamics profile {path}: {e}");
            std::process::exit(1);
        });
        let model = DynamicsModel::from_toml_str(&text).unwrap_or_else(|e| {
            eprintln!("ec-rt: dynamics profile {path} invalid: {e:?}");
            std::process::exit(1);
        });
        if model.n != NUM_AXES {
            eprintln!(
                "ec-rt: dynamics profile {path} has {} axes, endpoint drives {NUM_AXES}",
                model.n
            );
            std::process::exit(1);
        }
        model
    });
```

with `use kalico_ethercat_rt::dynamics::{clamp_torque, DynamicsModel};`
added to the imports. Exiting pre-bind is the loud-failure path: klippy's
claim fails with the endpoint's stderr in its log.

Extend the startup banner eprintln with
`velocity_ff={velocity_ff} dynamics={} clamp={torque_clamp_tenths}` (use
`dynamics.is_some()`).

- [ ] **Step 2: Per-cycle FF in the DC loop**

Add `let mut ff_saturation = 0u32;` next to `let mut prdiv = 0u64;`.
Replace the Enabled sampling block:

```rust
        if gate.state() == TorqueState::Enabled {
            if let Some((pos_mm, vel_mm_s, acc_mm_s2)) = ring.sample(now) {
                let map = cmap.get_or_insert_with(|| {
                    let actual = unsafe { ffi::ec_rt_get_position_actual() };
                    CountMap::new(counts_per_mm, actual, f64::from(pos_mm))
                });
                let counts = map.target_counts(f64::from(pos_mm));
                let vel_offset = if velocity_ff {
                    (f64::from(vel_mm_s) * counts_per_mm).round() as i32
                } else {
                    0
                };
                let torque_offset = match &dynamics {
                    Some(model) => {
                        let raw = model.torque_ff(0, &[acc_mm_s2], &[vel_mm_s]);
                        if !raw.is_finite() {
                            eprintln!(
                                "ec-rt: FAULT non-finite torque FF (acc={acc_mm_s2} vel={vel_mm_s}) — disabling"
                            );
                            server.respond(&status_heartbeat_frame(
                                ENGINE_STATE_FAULT,
                                &[ring.retired_count()],
                                ff_saturation,
                            ));
                            unsafe {
                                ffi::ec_rt_disable();
                                ffi::ec_rt_shutdown();
                            }
                            std::process::exit(1);
                        }
                        clamp_torque(raw, torque_clamp_tenths, &mut ff_saturation)
                    }
                    None => 0,
                };
                unsafe {
                    ffi::ec_rt_set_target_position(counts);
                    ffi::ec_rt_set_velocity_offset(vel_offset);
                    ffi::ec_rt_set_torque_offset(torque_offset);
                }
            } else {
                cmap = None;
                unsafe {
                    ffi::ec_rt_set_velocity_offset(0);
                    ffi::ec_rt_set_torque_offset(0);
                }
            }
        }
```

Thread `ff_saturation` into both `status_heartbeat_frame` calls in this file
(replacing the Task 4 placeholder `0`), and extend the periodic telemetry
eprintln with `ff_sat={ff_saturation}` plus
`vel_act={}` / `tq_act={}` from the two new getters.

- [ ] **Step 3: Verify lib + stub still build/test, commit**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt && cargo check -p kalico-ethercat-rt --bins`
Expected: PASS / clean check (the hw bin is feature-gated out locally; the
stub must compile).

```bash
git add rust/kalico-ethercat-rt/src/bin/kalico-ethercat-rt.rs
git commit -m "feat(ethercat): per-cycle velocity/torque feedforward in the DC loop"
```

### Task 7: klippy + bridge plumbing for the new flags

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:248` (`spawn_ethercat_endpoint`)
  and `:740` (`claim_ethercat_node` pyo3 signature)
- Modify: `klippy/extras/servo_axis.py` (config keys)
- Modify: `klippy/extras/ethercat_node.py` (`_claim`, derive + pass through)

- [ ] **Step 1: Bridge spawn args**

`spawn_ethercat_endpoint` gains parameters and forwards them:

```rust
fn spawn_ethercat_endpoint(
    binary: &str,
    interface: &str,
    socket_path: &str,
    counts_per_mm: f64,
    velocity_ff: bool,
    dynamics_profile: Option<&str>,
    torque_clamp_pct: f64,
) -> Result<std::process::Child, String> {
    let mut cmd = std::process::Command::new(binary);
    cmd.arg(interface)
        .arg("--socket")
        .arg(socket_path)
        .arg("--counts-per-mm")
        .arg(counts_per_mm.to_string())
        .arg("--torque-clamp-pct")
        .arg(torque_clamp_pct.to_string());
    if velocity_ff {
        cmd.arg("--velocity-ff");
    }
    if let Some(p) = dynamics_profile {
        cmd.arg("--dynamics-profile").arg(p);
    }
    cmd.spawn().map_err(|e| format!("spawn {binary}: {e}"))
}
```

(Keep the existing arg order for `interface`/`--socket`/`--counts-per-mm` —
copy whatever the current body does, the snippet above must be reconciled
with the literal current code at `bridge.rs:254-261`.)

`claim_ethercat_node` pyo3 signature gains
`velocity_ff: bool, dynamics_profile: Option<String>, torque_clamp_pct: f64`
(update `#[pyo3(signature = (...))]` accordingly) and passes them through.

- [ ] **Step 2: klippy config keys**

`klippy/extras/servo_axis.py`, next to the existing config reads:

```python
        self.velocity_ff = config.getboolean("velocity_ff", False)
        self.dynamics_profile = config.get("dynamics_profile", None)
        self.ff_torque_clamp = config.getfloat(
            "ff_torque_clamp", 30.0, above=0.0, maxval=400.0
        )
```

and add a getter mirroring `get_counts_per_mm`:

```python
    def get_ff_config(self):
        return (self.velocity_ff, self.dynamics_profile, self.ff_torque_clamp)
```

`klippy/extras/ethercat_node.py`: in `_derive_counts_per_mm`'s rail-lookup
pattern, also fetch the FF config from the same `[servo_*]` section (factor
the section lookup so both derivations share it), then pass the three values
to `bridge.claim_ethercat_node(...)` in `_claim` and include them in the
`logging.info` line.

- [ ] **Step 3: Build + test**

Run: `cd rust && cargo nextest run -p motion-bridge && cargo check -p motion-bridge`
Expected: PASS. Then `python3 -m py_compile klippy/extras/servo_axis.py klippy/extras/ethercat_node.py` — clean.

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs klippy/extras/servo_axis.py klippy/extras/ethercat_node.py
git commit -m "feat(servo): velocity_ff / dynamics_profile / ff_torque_clamp config plumbing"
```

---

# Part B — identification toolkit (layer 3, independent of Part A)

New workspace crate `rust/servo-ident` (std-only, no external deps). Two
binaries: `servo-excite` (G-code generator) and `servo-ident` (fitter).

### Task 8: Crate scaffold + small linear algebra

**Files:**
- Modify: `rust/Cargo.toml` (add `"servo-ident"` to `members`)
- Create: `rust/servo-ident/Cargo.toml`
- Create: `rust/servo-ident/src/lib.rs`
- Create: `rust/servo-ident/src/linalg.rs`
- Create: `rust/servo-ident/tests/linalg.rs`

- [ ] **Step 1: Scaffold**

`rust/servo-ident/Cargo.toml`:

```toml
[package]
name = "servo-ident"
version = "0.1.0"
edition = "2021"
publish = false
license.workspace = true

[[bin]]
name = "servo-ident"
path = "src/bin/servo-ident.rs"

[[bin]]
name = "servo-excite"
path = "src/bin/servo-excite.rs"

[lints]
workspace = true
```

`src/lib.rs`:

```rust
pub mod capture;
pub mod fit;
pub mod gcode_gen;
pub mod linalg;
pub mod model;
pub mod profile_out;
```

(Each module lands in its own task; create empty files
`capture.rs`/`fit.rs`/`gcode_gen.rs`/`model.rs`/`profile_out.rs` now so the
crate compiles, filled in Tasks 9–12.)

- [ ] **Step 2: Failing linalg tests**

```rust
// rust/servo-ident/tests/linalg.rs
use servo_ident::linalg::{solve_spd, sym_eig_extremes};

#[test]
fn solves_known_spd_system() {
    // A = [[4,1],[1,3]], y = [1,2] -> x = [1/11, 7/11]
    let a = vec![4.0, 1.0, 1.0, 3.0];
    let x = solve_spd(&a, &[1.0, 2.0], 2).unwrap();
    assert!((x[0] - 1.0 / 11.0).abs() < 1e-12);
    assert!((x[1] - 7.0 / 11.0).abs() < 1e-12);
}

#[test]
fn rejects_non_pd() {
    let a = vec![1.0, 2.0, 2.0, 1.0];
    assert!(solve_spd(&a, &[1.0, 1.0], 2).is_none());
}

#[test]
fn eig_extremes_of_diagonal() {
    let a = vec![9.0, 0.0, 0.0, 0.0, 4.0, 0.0, 0.0, 0.0, 1.0];
    let (lo, hi) = sym_eig_extremes(&a, 3);
    assert!((lo - 1.0).abs() < 1e-9 && (hi - 9.0).abs() < 1e-9);
}

#[test]
fn eig_extremes_of_rotated_matrix() {
    // eigenvalues of [[2,1],[1,2]] are 1 and 3
    let a = vec![2.0, 1.0, 1.0, 2.0];
    let (lo, hi) = sym_eig_extremes(&a, 2);
    assert!((lo - 1.0).abs() < 1e-9 && (hi - 3.0).abs() < 1e-9);
}
```

- [ ] **Step 3: Run to verify failure**

Run: `cd rust && cargo nextest run -p servo-ident`
Expected: compile FAIL.

- [ ] **Step 4: Implement `linalg.rs`**

```rust
// rust/servo-ident/src/linalg.rs

/// Cholesky solve of A·x = y for symmetric positive-definite A (row-major
/// n×n). None when A is not PD.
pub fn solve_spd(a: &[f64], y: &[f64], n: usize) -> Option<Vec<f64>> {
    assert_eq!(a.len(), n * n);
    assert_eq!(y.len(), n);
    let mut l = vec![0.0; n * n];
    for i in 0..n {
        for j in 0..=i {
            let mut s = a[i * n + j];
            for k in 0..j {
                s -= l[i * n + k] * l[j * n + k];
            }
            if i == j {
                if s <= 0.0 {
                    return None;
                }
                l[i * n + i] = s.sqrt();
            } else {
                l[i * n + j] = s / l[j * n + j];
            }
        }
    }
    let mut z = vec![0.0; n];
    for i in 0..n {
        let mut s = y[i];
        for k in 0..i {
            s -= l[i * n + k] * z[k];
        }
        z[i] = s / l[i * n + i];
    }
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut s = z[i];
        for k in (i + 1)..n {
            s -= l[k * n + i] * x[k];
        }
        x[i] = s / l[i * n + i];
    }
    Some(x)
}

/// Smallest and largest eigenvalue of a symmetric matrix via cyclic Jacobi.
pub fn sym_eig_extremes(a: &[f64], n: usize) -> (f64, f64) {
    assert_eq!(a.len(), n * n);
    let mut m = a.to_vec();
    for _sweep in 0..64 {
        let mut off = 0.0;
        for p in 0..n {
            for q in (p + 1)..n {
                off += m[p * n + q] * m[p * n + q];
            }
        }
        if off < 1e-24 {
            break;
        }
        for p in 0..n {
            for q in (p + 1)..n {
                let apq = m[p * n + q];
                if apq.abs() < 1e-300 {
                    continue;
                }
                let theta = (m[q * n + q] - m[p * n + p]) / (2.0 * apq);
                let t = theta.signum() / (theta.abs() + (theta * theta + 1.0).sqrt());
                let c = 1.0 / (t * t + 1.0).sqrt();
                let s = t * c;
                for k in 0..n {
                    let akp = m[k * n + p];
                    let akq = m[k * n + q];
                    m[k * n + p] = c * akp - s * akq;
                    m[k * n + q] = s * akp + c * akq;
                }
                for k in 0..n {
                    let apk = m[p * n + k];
                    let aqk = m[q * n + k];
                    m[p * n + k] = c * apk - s * aqk;
                    m[q * n + k] = s * apk + c * aqk;
                }
            }
        }
    }
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for i in 0..n {
        lo = lo.min(m[i * n + i]);
        hi = hi.max(m[i * n + i]);
    }
    (lo, hi)
}
```

- [ ] **Step 5: Run to verify pass, commit**

Run: `cd rust && cargo nextest run -p servo-ident`
Expected: 4 PASS.

```bash
git add rust/Cargo.toml rust/Cargo.lock rust/servo-ident/
git commit -m "feat(servo-ident): crate scaffold with SPD solve and symmetric eigen extremes"
```

### Task 9: Model structures and regressor rows

**Files:**
- Create: `rust/servo-ident/src/model.rs` (replace empty file)
- Create: `rust/servo-ident/tests/model.rs`

- [ ] **Step 1: Failing tests**

```rust
// rust/servo-ident/tests/model.rs
use servo_ident::model::{Structure, COULOMB_DEADBAND_MM_S};

#[test]
fn scalar_row_layout() {
    let s = Structure::CartesianScalar;
    assert_eq!(s.param_count(), 4);
    let row = s.row(0, &[1000.0], &[100.0]);
    assert_eq!(row, vec![1000.0, 100.0, 1.0, 0.0]);
    let row_rev = s.row(0, &[1000.0], &[-100.0]);
    assert_eq!(row_rev, vec![1000.0, -100.0, 0.0, 1.0]);
    let row_dead = s.row(0, &[1000.0], &[COULOMB_DEADBAND_MM_S / 2.0]);
    assert_eq!(row_dead[2], 0.0);
    assert_eq!(row_dead[3], 0.0);
}

#[test]
fn corexy_rows_share_mass_params() {
    let s = Structure::CoreXY;
    assert_eq!(s.param_count(), 8);
    let ra = s.row(0, &[100.0, 50.0], &[10.0, -10.0]);
    assert_eq!(ra, vec![100.0, 50.0, 10.0, 1.0, 0.0, 0.0, 0.0, 0.0]);
    let rb = s.row(1, &[100.0, 50.0], &[10.0, -10.0]);
    assert_eq!(rb, vec![50.0, 100.0, 0.0, 0.0, 0.0, -10.0, 0.0, 1.0]);
}

#[test]
fn params_to_profile_blocks() {
    let s = Structure::CoreXY;
    let theta = vec![0.030, -0.010, 0.004, 1.0, -1.1, 0.005, 0.9, -0.8];
    let p = s.unpack(&theta);
    assert_eq!(p.mass, vec![vec![0.030, -0.010], vec![-0.010, 0.030]]);
    assert_eq!(p.viscous, vec![0.004, 0.005]);
    assert_eq!(p.coulomb_fwd, vec![1.0, 0.9]);
    assert_eq!(p.coulomb_rev, vec![-1.1, -0.8]);
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p servo-ident -E 'binary(model)'`
Expected: compile FAIL.

- [ ] **Step 3: Implement `model.rs`**

```rust
// rust/servo-ident/src/model.rs

pub const COULOMB_DEADBAND_MM_S: f64 = 0.5;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Structure {
    CartesianScalar,
    CoreXY,
}

#[derive(Debug, PartialEq)]
pub struct PhysicalParams {
    pub mass: Vec<Vec<f64>>,
    pub viscous: Vec<f64>,
    pub coulomb_fwd: Vec<f64>,
    pub coulomb_rev: Vec<f64>,
}

fn coulomb_cols(v: f64) -> (f64, f64) {
    if v > COULOMB_DEADBAND_MM_S {
        (1.0, 0.0)
    } else if v < -COULOMB_DEADBAND_MM_S {
        (0.0, 1.0)
    } else {
        (0.0, 0.0)
    }
}

impl Structure {
    pub fn axis_count(self) -> usize {
        match self {
            Structure::CartesianScalar => 1,
            Structure::CoreXY => 2,
        }
    }

    /// Scalar: theta = [m, b, c_fwd, c_rev].
    /// CoreXY: theta = [m_diag, m_off, b_a, cf_a, cr_a, b_b, cf_b, cr_b]
    /// (equal diagonals and symmetric off-diagonal are baked into the rows).
    pub fn param_count(self) -> usize {
        match self {
            Structure::CartesianScalar => 4,
            Structure::CoreXY => 8,
        }
    }

    /// Regression row for one torque sample of `motor`:
    /// tau_motor = row(motor, acc, vel) · theta
    pub fn row(self, motor: usize, acc: &[f64], vel: &[f64]) -> Vec<f64> {
        match self {
            Structure::CartesianScalar => {
                assert_eq!(motor, 0);
                let (cf, cr) = coulomb_cols(vel[0]);
                vec![acc[0], vel[0], cf, cr]
            }
            Structure::CoreXY => {
                assert!(motor < 2);
                let other = 1 - motor;
                let (cf, cr) = coulomb_cols(vel[motor]);
                let mut r = vec![acc[motor], acc[other], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
                let base = 2 + 3 * motor;
                r[base] = vel[motor];
                r[base + 1] = cf;
                r[base + 2] = cr;
                r
            }
        }
    }

    pub fn unpack(self, theta: &[f64]) -> PhysicalParams {
        assert_eq!(theta.len(), self.param_count());
        match self {
            Structure::CartesianScalar => PhysicalParams {
                mass: vec![vec![theta[0]]],
                viscous: vec![theta[1]],
                coulomb_fwd: vec![theta[2]],
                coulomb_rev: vec![theta[3]],
            },
            Structure::CoreXY => PhysicalParams {
                mass: vec![vec![theta[0], theta[1]], vec![theta[1], theta[0]]],
                viscous: vec![theta[2], theta[5]],
                coulomb_fwd: vec![theta[3], theta[6]],
                coulomb_rev: vec![theta[4], theta[7]],
            },
        }
    }
}
```

- [ ] **Step 4: Run to verify pass, commit**

Run: `cd rust && cargo nextest run -p servo-ident`

```bash
git add rust/servo-ident/src/model.rs rust/servo-ident/tests/model.rs
git commit -m "feat(servo-ident): regression structures for scalar and corexy dynamics"
```

### Task 10: Fitter with refusal diagnostics + synthetic-recovery tests

**Files:**
- Create: `rust/servo-ident/src/fit.rs` (replace empty file)
- Create: `rust/servo-ident/tests/synthetic.rs`

- [ ] **Step 1: Failing tests**

```rust
// rust/servo-ident/tests/synthetic.rs
use servo_ident::fit::{fit, FitError, FitInput, FitOptions};
use servo_ident::model::Structure;

/// Triangle-stroke kinematics: accelerate at `a` for `t1`, decelerate for
/// `t1`, mirror back. Returns (acc, vel) per motor over `n` samples at `dt`.
fn triangle(a: f64, t1: f64, dt: f64, reps: usize) -> (Vec<f64>, Vec<f64>) {
    let mut acc = Vec::new();
    let mut vel = Vec::new();
    let mut v = 0.0;
    for _ in 0..reps {
        for phase in [a, -a, -a, a] {
            let steps = (t1 / dt) as usize;
            for _ in 0..steps {
                acc.push(phase);
                v += phase * dt;
                vel.push(v);
            }
        }
    }
    (acc, vel)
}

fn noisy(x: f64, k: usize) -> f64 {
    // deterministic pseudo-noise, +/-0.5 of a 0.1% torque unit
    let h = (k.wrapping_mul(2654435761)) as u32;
    x + ((h % 1000) as f64 / 1000.0 - 0.5)
}

#[test]
fn recovers_scalar_truth() {
    let (m, b, cf, cr) = (0.0123, 0.0045, 1.2, -1.1);
    let (acc, vel) = triangle(2000.0, 0.08, 0.001, 6);
    let torque: Vec<f64> = acc
        .iter()
        .zip(&vel)
        .enumerate()
        .map(|(k, (&a, &v))| {
            let c = if v > 0.5 { cf } else if v < -0.5 { cr } else { 0.0 };
            noisy(m * a + b * v + c, k).round() // i16 quantization
        })
        .collect();
    let input = FitInput {
        structure: Structure::CartesianScalar,
        acc: vec![acc],
        vel: vec![vel],
        torque: vec![torque],
    };
    let r = fit(&input, &FitOptions::default()).unwrap();
    let p = &r.params;
    assert!((p.mass[0][0] - m).abs() < 0.1 * m, "m: {}", p.mass[0][0]);
    assert!((p.coulomb_fwd[0] - cf).abs() < 0.5, "cf: {}", p.coulomb_fwd[0]);
    assert!(r.rms_residual < 2.0);
}

#[test]
fn recovers_corexy_coupling() {
    let (md, mo) = (0.030, -0.010);
    let (acc_x, vel_x) = triangle(1500.0, 0.06, 0.001, 4); // X strokes: a=b
    let (acc_y, vel_y) = triangle(1500.0, 0.06, 0.001, 4); // Y strokes: a=-b
    let acc_a: Vec<f64> = acc_x.iter().chain(&acc_y).copied().collect();
    let vel_a: Vec<f64> = vel_x.iter().chain(&vel_y).copied().collect();
    let acc_b: Vec<f64> = acc_x
        .iter()
        .map(|&v| v)
        .chain(acc_y.iter().map(|&v| -v))
        .collect();
    let vel_b: Vec<f64> = vel_x
        .iter()
        .map(|&v| v)
        .chain(vel_y.iter().map(|&v| -v))
        .collect();
    let tq = |acc_s: &f64, acc_o: &f64, v: &f64, k: usize| {
        let c = if *v > 0.5 { 1.0 } else if *v < -0.5 { -1.0 } else { 0.0 };
        noisy(md * acc_s + mo * acc_o + 0.004 * v + c, k).round()
    };
    let torque_a: Vec<f64> = acc_a
        .iter()
        .zip(&acc_b)
        .zip(&vel_a)
        .enumerate()
        .map(|(k, ((a, b), v))| tq(a, b, v, k))
        .collect();
    let torque_b: Vec<f64> = acc_b
        .iter()
        .zip(&acc_a)
        .zip(&vel_b)
        .enumerate()
        .map(|(k, ((b, a), v))| tq(b, a, v, k + 7))
        .collect();
    let input = FitInput {
        structure: Structure::CoreXY,
        acc: vec![acc_a, acc_b],
        vel: vec![vel_a, vel_b],
        torque: vec![torque_a, torque_b],
    };
    let r = fit(&input, &FitOptions::default()).unwrap();
    assert!((r.params.mass[0][0] - md).abs() < 0.1 * md);
    assert!((r.params.mass[0][1] - mo).abs() < 0.1 * mo.abs());
}

#[test]
fn refuses_insufficient_excitation() {
    // constant velocity only: acceleration column is all zeros
    let n = 2000;
    let input = FitInput {
        structure: Structure::CartesianScalar,
        acc: vec![vec![0.0; n]],
        vel: vec![vec![100.0; n]],
        torque: vec![vec![1.0; n]],
    };
    assert!(matches!(
        fit(&input, &FitOptions::default()),
        Err(FitError::InsufficientExcitation { .. })
    ));
}

#[test]
fn refuses_saturated_torque() {
    let (acc, vel) = triangle(2000.0, 0.08, 0.001, 4);
    let n = acc.len();
    let mut torque = vec![100.0; n];
    for t in torque.iter_mut().take(n / 10) {
        *t = 3995.0; // at the i16 0.1% ceiling
    }
    let input = FitInput {
        structure: Structure::CartesianScalar,
        acc: vec![acc],
        vel: vec![vel],
        torque: vec![torque],
    };
    assert!(matches!(
        fit(&input, &FitOptions::default()),
        Err(FitError::SaturatedTorque { .. })
    ));
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p servo-ident -E 'binary(synthetic)'`
Expected: compile FAIL.

- [ ] **Step 3: Implement `fit.rs`**

```rust
// rust/servo-ident/src/fit.rs
use crate::linalg::{solve_spd, sym_eig_extremes};
use crate::model::{PhysicalParams, Structure};

pub struct FitInput {
    pub structure: Structure,
    /// Per motor, same length per motor: acc[motor][sample] (mm/s²).
    pub acc: Vec<Vec<f64>>,
    pub vel: Vec<Vec<f64>>,
    /// Measured torque per motor (0.1% rated units).
    pub torque: Vec<Vec<f64>>,
}

pub struct FitOptions {
    pub max_condition: f64,
    pub saturation_abs: f64,
    pub max_saturated_fraction: f64,
    pub max_rms_residual: f64,
}

impl Default for FitOptions {
    fn default() -> Self {
        Self {
            max_condition: 1.0e8,
            saturation_abs: 3900.0,
            max_saturated_fraction: 0.001,
            max_rms_residual: 50.0,
        }
    }
}

#[derive(Debug)]
pub enum FitError {
    ShapeMismatch(&'static str),
    SaturatedTorque { fraction: f64 },
    InsufficientExcitation { condition: f64 },
    ResidualTooLarge { rms: f64 },
}

#[derive(Debug)]
pub struct FitResult {
    pub params: PhysicalParams,
    pub rms_residual: f64,
    pub condition: f64,
    pub samples: usize,
}

pub fn fit(input: &FitInput, opts: &FitOptions) -> Result<FitResult, FitError> {
    let s = input.structure;
    let n_motors = s.axis_count();
    if input.acc.len() != n_motors
        || input.vel.len() != n_motors
        || input.torque.len() != n_motors
    {
        return Err(FitError::ShapeMismatch("motor count"));
    }
    let n_samples = input.acc[0].len();
    for m in 0..n_motors {
        if input.acc[m].len() != n_samples
            || input.vel[m].len() != n_samples
            || input.torque[m].len() != n_samples
        {
            return Err(FitError::ShapeMismatch("sample count"));
        }
    }

    let saturated = input
        .torque
        .iter()
        .flatten()
        .filter(|t| t.abs() >= opts.saturation_abs)
        .count();
    let fraction = saturated as f64 / (n_motors * n_samples) as f64;
    if fraction > opts.max_saturated_fraction {
        return Err(FitError::SaturatedTorque { fraction });
    }

    let p = s.param_count();
    let mut ata = vec![0.0; p * p];
    let mut aty = vec![0.0; p];
    let mut col_norm2 = vec![0.0; p];
    for k in 0..n_samples {
        let acc_k: Vec<f64> = (0..n_motors).map(|m| input.acc[m][k]).collect();
        let vel_k: Vec<f64> = (0..n_motors).map(|m| input.vel[m][k]).collect();
        for motor in 0..n_motors {
            let row = s.row(motor, &acc_k, &vel_k);
            let y = input.torque[motor][k];
            for i in 0..p {
                aty[i] += row[i] * y;
                col_norm2[i] += row[i] * row[i];
                for j in 0..p {
                    ata[i * p + j] += row[i] * row[j];
                }
            }
        }
    }

    let scale: Vec<f64> = col_norm2
        .iter()
        .map(|&c| if c > 0.0 { c.sqrt() } else { 0.0 })
        .collect();
    if scale.iter().any(|&sc| sc == 0.0) {
        return Err(FitError::InsufficientExcitation { condition: f64::INFINITY });
    }
    let mut ata_s = vec![0.0; p * p];
    for i in 0..p {
        for j in 0..p {
            ata_s[i * p + j] = ata[i * p + j] / (scale[i] * scale[j]);
        }
    }
    let (lo, hi) = sym_eig_extremes(&ata_s, p);
    let condition = if lo > 0.0 { hi / lo } else { f64::INFINITY };
    if condition > opts.max_condition {
        return Err(FitError::InsufficientExcitation { condition });
    }

    let aty_s: Vec<f64> = (0..p).map(|i| aty[i] / scale[i]).collect();
    let theta_s = solve_spd(&ata_s, &aty_s, p)
        .ok_or(FitError::InsufficientExcitation { condition })?;
    let theta: Vec<f64> = (0..p).map(|i| theta_s[i] / scale[i]).collect();

    let mut sq_sum = 0.0;
    for k in 0..n_samples {
        let acc_k: Vec<f64> = (0..n_motors).map(|m| input.acc[m][k]).collect();
        let vel_k: Vec<f64> = (0..n_motors).map(|m| input.vel[m][k]).collect();
        for motor in 0..n_motors {
            let row = s.row(motor, &acc_k, &vel_k);
            let pred: f64 = row.iter().zip(&theta).map(|(r, t)| r * t).sum();
            let e = input.torque[motor][k] - pred;
            sq_sum += e * e;
        }
    }
    let rms = (sq_sum / (n_motors * n_samples) as f64).sqrt();
    if rms > opts.max_rms_residual {
        return Err(FitError::ResidualTooLarge { rms });
    }

    Ok(FitResult {
        params: s.unpack(&theta),
        rms_residual: rms,
        condition,
        samples: n_samples,
    })
}
```

- [ ] **Step 4: Run to verify pass, commit**

Run: `cd rust && cargo nextest run -p servo-ident`
Expected: all PASS.

```bash
git add rust/servo-ident/src/fit.rs rust/servo-ident/tests/synthetic.rs
git commit -m "feat(servo-ident): least-squares dynamics fit with refusal diagnostics"
```

### Task 11: Capture adapter + profile output + C00.06 report

**Files:**
- Create: `rust/servo-ident/src/capture.rs` (replace empty file)
- Create: `rust/servo-ident/src/profile_out.rs` (replace empty file)
- Create: `rust/servo-ident/tests/capture_profile.rs`

The CSV format is the **interim layer-2 contract**: header row, column `t`
(seconds) plus per axis `<name>` from `--axes`: `target_<name>` (mm,
commanded), `torque_<name>` (0.1% rated). Optional: `pos_<name>`,
`vel_<name>`. ω and α are central differences of `target_<name>`.

- [ ] **Step 1: Failing tests**

```rust
// rust/servo-ident/tests/capture_profile.rs
use servo_ident::capture::parse_capture_csv;
use servo_ident::model::PhysicalParams;
use servo_ident::profile_out::{c0006_recommendation, render_profile};

#[test]
fn parses_and_differentiates() {
    // x(t) = 0.5 * 1000 * t^2 -> vel = 1000 t, acc = 1000
    let mut csv = String::from("t,target_x,torque_x\n");
    for k in 0..100 {
        let t = k as f64 * 0.001;
        csv.push_str(&format!("{t},{},{}\n", 0.5 * 1000.0 * t * t, 12.0));
    }
    let cap = parse_capture_csv(&csv, &["x"]).unwrap();
    assert_eq!(cap.torque[0][50], 12.0);
    assert!((cap.vel[0][50] - 1000.0 * 0.050).abs() < 1.0);
    assert!((cap.acc[0][50] - 1000.0).abs() < 5.0);
    assert_eq!(cap.acc[0].len(), 100);
}

#[test]
fn rejects_missing_column() {
    assert!(parse_capture_csv("t,target_x\n0,0\n", &["x"]).is_err());
}

#[test]
fn renders_loadable_profile() {
    let p = PhysicalParams {
        mass: vec![vec![0.0123]],
        viscous: vec![0.0045],
        coulomb_fwd: vec![1.2],
        coulomb_rev: vec![-1.1],
    };
    let toml_text = render_profile(&p, &["x"], &[0.8]);
    assert!(toml_text.contains("version = 1"));
    assert!(toml_text.contains("coulomb_deadband_mm_s = 0.5"));
    assert!(toml_text.contains("mass = [[0.0123]]"));
}

#[test]
fn c0006_matches_hand_calculation() {
    // M = 0.0123 (0.1% rated)/(mm/s²), rated 1.27 N·m, rot_dist 40 mm,
    // rotor 0.269e-4 kg·m².
    // J_total = 0.0123 * (1.27/1000) * 0.040/(2π) = 9.945e-8... in SI the
    // mm of rot_dist and the mm/s² cancel; keep rot_dist in meters:
    let j_total = 0.0123 * (1.27 / 1000.0) * 0.040 / (2.0 * std::f64::consts::PI);
    let rotor = 0.269e-4;
    let expect = (j_total - rotor) / rotor * 100.0;
    let got = c0006_recommendation(0.0123, 1.27, 40.0, rotor);
    assert!((got - expect).abs() < 1e-9, "{got} vs {expect}");
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p servo-ident -E 'binary(capture_profile)'`
Expected: compile FAIL.

- [ ] **Step 3: Implement**

```rust
// rust/servo-ident/src/capture.rs

#[derive(Debug)]
pub struct Capture {
    pub t: Vec<f64>,
    pub acc: Vec<Vec<f64>>,
    pub vel: Vec<Vec<f64>>,
    pub torque: Vec<Vec<f64>>,
}

#[derive(Debug)]
pub enum CaptureError {
    MissingColumn(String),
    Malformed { line: usize, what: String },
    TooShort,
}

pub fn parse_capture_csv(text: &str, axes: &[&str]) -> Result<Capture, CaptureError> {
    let mut lines = text.lines().enumerate();
    let (_, header) = lines.next().ok_or(CaptureError::TooShort)?;
    let cols: Vec<&str> = header.split(',').map(str::trim).collect();
    let col = |name: &str| {
        cols.iter()
            .position(|c| *c == name)
            .ok_or_else(|| CaptureError::MissingColumn(name.to_string()))
    };
    let t_col = col("t")?;
    let target_cols: Vec<usize> = axes
        .iter()
        .map(|a| col(&format!("target_{a}")))
        .collect::<Result<_, _>>()?;
    let torque_cols: Vec<usize> = axes
        .iter()
        .map(|a| col(&format!("torque_{a}")))
        .collect::<Result<_, _>>()?;

    let mut t = Vec::new();
    let mut target: Vec<Vec<f64>> = vec![Vec::new(); axes.len()];
    let mut torque: Vec<Vec<f64>> = vec![Vec::new(); axes.len()];
    for (lineno, line) in lines {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').map(str::trim).collect();
        let num = |idx: usize| -> Result<f64, CaptureError> {
            fields
                .get(idx)
                .and_then(|f| f.parse().ok())
                .ok_or(CaptureError::Malformed {
                    line: lineno + 1,
                    what: format!("column {idx}"),
                })
        };
        t.push(num(t_col)?);
        for (a, (&tc, &qc)) in target_cols.iter().zip(&torque_cols).enumerate() {
            target[a].push(num(tc)?);
            torque[a].push(num(qc)?);
        }
    }
    let n = t.len();
    if n < 5 {
        return Err(CaptureError::TooShort);
    }

    let diff = |x: &[f64]| -> Vec<f64> {
        let mut d = vec![0.0; n];
        for k in 1..n - 1 {
            let dt = t[k + 1] - t[k - 1];
            d[k] = if dt > 0.0 { (x[k + 1] - x[k - 1]) / dt } else { 0.0 };
        }
        d[0] = d[1];
        d[n - 1] = d[n - 2];
        d
    };
    let vel: Vec<Vec<f64>> = target.iter().map(|x| diff(x)).collect();
    let acc: Vec<Vec<f64>> = vel.iter().map(|v| diff(v)).collect();
    Ok(Capture { t, acc, vel, torque })
}
```

```rust
// rust/servo-ident/src/profile_out.rs
use crate::model::{PhysicalParams, COULOMB_DEADBAND_MM_S};

pub fn render_profile(p: &PhysicalParams, axes: &[&str], rms_residual: &[f64]) -> String {
    let fmt_vec = |v: &[f64]| {
        let inner: Vec<String> = v.iter().map(|x| format!("{x}")).collect();
        format!("[{}]", inner.join(", "))
    };
    let mass_rows: Vec<String> = p.mass.iter().map(|row| fmt_vec(row)).collect();
    let axes_q: Vec<String> = axes.iter().map(|a| format!("\"{a}\"")).collect();
    format!(
        "version = 1\naxes = [{}]\nmass = [{}]\nviscous = {}\ncoulomb_fwd = {}\n\
         coulomb_rev = {}\ncoulomb_deadband_mm_s = {COULOMB_DEADBAND_MM_S}\n\
         fit_rms_residual = {}\n",
        axes_q.join(", "),
        mass_rows.join(", "),
        fmt_vec(&p.viscous),
        fmt_vec(&p.coulomb_fwd),
        fmt_vec(&p.coulomb_rev),
        fmt_vec(rms_residual),
    )
}

/// Drive load-inertia-ratio (C00.06, percent) implied by a fitted diagonal
/// mass entry. `rot_dist_mm` per motor rev; `rated_torque_nm`; rotor inertia
/// in kg·m². The mm in M's denominator cancels rot_dist's mm.
pub fn c0006_recommendation(
    m_diag: f64,
    rated_torque_nm: f64,
    rot_dist_mm: f64,
    rotor_inertia_kgm2: f64,
) -> f64 {
    let j_total =
        m_diag * (rated_torque_nm / 1000.0) * (rot_dist_mm / 1000.0) / (2.0 * std::f64::consts::PI);
    (j_total - rotor_inertia_kgm2) / rotor_inertia_kgm2 * 100.0
}
```

**Note the units fix vs the test comment:** `rot_dist` enters in meters
(`/1000.0`) *and* M's per-mm/s² denominator must be converted to per-m/s²
(×1000) — the two factors cancel, so the formula above multiplies by
`rot_dist_mm/1000` once and is consistent with the test's hand calculation
(`0.040`). Keep test and implementation agreeing; if the test was written
with the other convention, fix the test to the meters form shown.

- [ ] **Step 4: Run, fix any units disagreement deliberately, verify pass**

Run: `cd rust && cargo nextest run -p servo-ident`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/servo-ident/src/capture.rs rust/servo-ident/src/profile_out.rs rust/servo-ident/tests/capture_profile.rs
git commit -m "feat(servo-ident): capture CSV adapter, profile renderer, C00.06 report"
```

### Task 12: Excitation generator + the two CLI binaries

**Files:**
- Create: `rust/servo-ident/src/gcode_gen.rs` (replace empty file)
- Create: `rust/servo-ident/tests/gcode_gen.rs`
- Create: `rust/servo-ident/src/bin/servo-ident.rs`
- Create: `rust/servo-ident/src/bin/servo-excite.rs`

- [ ] **Step 1: Failing generator test**

```rust
// rust/servo-ident/tests/gcode_gen.rs
use servo_ident::gcode_gen::{generate, Excitation};

#[test]
fn strokes_stay_in_bounds_and_reach_peak_speed() {
    let e = Excitation {
        axis: "X".into(),
        min_mm: 10.0,
        max_mm: 210.0,
        accels_mm_s2: vec![1000.0, 3000.0],
        speeds_mm_s: vec![100.0, 300.0],
        reps: 3,
    };
    let g = generate(&e).unwrap();
    assert!(g.contains("SET_VELOCITY_LIMIT ACCEL=1000"));
    assert!(g.contains("SET_VELOCITY_LIMIT ACCEL=3000"));
    assert!(g.contains("F18000")); // 300 mm/s
    assert!(g.contains("M400"));
    for line in g.lines().filter(|l| l.starts_with("G1 X")) {
        let x: f64 = line[4..]
            .split_whitespace()
            .next()
            .unwrap()
            .parse()
            .unwrap();
        assert!((10.0..=210.0).contains(&x), "{line}");
    }
}

#[test]
fn refuses_stroke_too_short_for_peak_speed() {
    let e = Excitation {
        axis: "X".into(),
        min_mm: 0.0,
        max_mm: 20.0,
        accels_mm_s2: vec![500.0],
        speeds_mm_s: vec![300.0], // needs v²/a = 180 mm > 20 mm
        reps: 1,
    };
    assert!(generate(&e).is_err());
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p servo-ident -E 'binary(gcode_gen)'`

- [ ] **Step 3: Implement `gcode_gen.rs`**

```rust
// rust/servo-ident/src/gcode_gen.rs
use std::fmt::Write as _;

pub struct Excitation {
    pub axis: String,
    pub min_mm: f64,
    pub max_mm: f64,
    pub accels_mm_s2: Vec<f64>,
    pub speeds_mm_s: Vec<f64>,
    pub reps: usize,
}

#[derive(Debug)]
pub enum GenError {
    BadBounds,
    StrokeTooShort { accel: f64, speed: f64, needed_mm: f64 },
}

pub fn generate(e: &Excitation) -> Result<String, GenError> {
    let span = e.max_mm - e.min_mm;
    if span <= 0.0 {
        return Err(GenError::BadBounds);
    }
    let mut g = String::new();
    let _ = writeln!(g, "; servo-excite: axis {} strokes {}..{} mm", e.axis, e.min_mm, e.max_mm);
    for &a in &e.accels_mm_s2 {
        for &v in &e.speeds_mm_s {
            let needed = v * v / a;
            if needed > span {
                return Err(GenError::StrokeTooShort { accel: a, speed: v, needed_mm: needed });
            }
            let f = (v * 60.0).round();
            let _ = writeln!(g, "SET_VELOCITY_LIMIT ACCEL={a} ACCEL_TO_DECEL={a}");
            for _ in 0..e.reps {
                let _ = writeln!(g, "G1 {}{} F{f}", e.axis, e.max_mm);
                let _ = writeln!(g, "G1 {}{} F{f}", e.axis, e.min_mm);
            }
            let _ = writeln!(g, "M400");
        }
    }
    Ok(g)
}
```

- [ ] **Step 4: CLI binaries**

```rust
// rust/servo-ident/src/bin/servo-excite.rs
//! Usage: servo-excite --axis X --min 10 --max 210 \
//!   --accels 1000,3000,6000 --speeds 100,200,300 [--reps 4] [--out f.gcode]
use servo_ident::gcode_gen::{generate, Excitation};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
}

fn list(s: &str) -> Vec<f64> {
    s.split(',').map(|v| v.trim().parse().expect("numeric list")).collect()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let e = Excitation {
        axis: arg(&args, "--axis").expect("--axis"),
        min_mm: arg(&args, "--min").expect("--min").parse().expect("--min"),
        max_mm: arg(&args, "--max").expect("--max").parse().expect("--max"),
        accels_mm_s2: list(&arg(&args, "--accels").expect("--accels")),
        speeds_mm_s: list(&arg(&args, "--speeds").expect("--speeds")),
        reps: arg(&args, "--reps").map(|r| r.parse().expect("--reps")).unwrap_or(4),
    };
    let g = generate(&e).unwrap_or_else(|err| {
        eprintln!("servo-excite: {err:?}");
        std::process::exit(1);
    });
    match arg(&args, "--out") {
        Some(p) => std::fs::write(&p, g).unwrap_or_else(|e| {
            eprintln!("servo-excite: write {p}: {e}");
            std::process::exit(1);
        }),
        None => print!("{g}"),
    }
}
```

```rust
// rust/servo-ident/src/bin/servo-ident.rs
//! Usage: servo-ident --capture run.csv --structure scalar|corexy \
//!   --axes x[,b] --out profile.toml \
//!   [--rated-torque-nm T --rotor-inertia-kgm2 J --rotation-distance-mm D]
use servo_ident::capture::parse_capture_csv;
use servo_ident::fit::{fit, FitInput, FitOptions};
use servo_ident::model::Structure;
use servo_ident::profile_out::{c0006_recommendation, render_profile};

fn arg(args: &[String], key: &str) -> Option<String> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1).cloned())
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let capture_path = arg(&args, "--capture").expect("--capture");
    let structure = match arg(&args, "--structure").expect("--structure").as_str() {
        "scalar" => Structure::CartesianScalar,
        "corexy" => Structure::CoreXY,
        other => {
            eprintln!("servo-ident: unknown structure {other}");
            std::process::exit(1);
        }
    };
    let axes_arg = arg(&args, "--axes").expect("--axes");
    let axes: Vec<&str> = axes_arg.split(',').map(str::trim).collect();
    if axes.len() != structure.axis_count() {
        eprintln!(
            "servo-ident: {} axes given, structure needs {}",
            axes.len(),
            structure.axis_count()
        );
        std::process::exit(1);
    }

    let text = std::fs::read_to_string(&capture_path).unwrap_or_else(|e| {
        eprintln!("servo-ident: read {capture_path}: {e}");
        std::process::exit(1);
    });
    let cap = parse_capture_csv(&text, &axes).unwrap_or_else(|e| {
        eprintln!("servo-ident: capture invalid: {e:?}");
        std::process::exit(1);
    });
    let input = FitInput { structure, acc: cap.acc, vel: cap.vel, torque: cap.torque };
    let r = fit(&input, &FitOptions::default()).unwrap_or_else(|e| {
        eprintln!("servo-ident: refusing to emit a profile: {e:?}");
        std::process::exit(2);
    });

    eprintln!(
        "fit: {} samples, rms residual {:.2} (0.1% rated), condition {:.1e}",
        r.samples, r.rms_residual, r.condition
    );
    if let (Some(t), Some(j), Some(d)) = (
        arg(&args, "--rated-torque-nm").and_then(|v| v.parse::<f64>().ok()),
        arg(&args, "--rotor-inertia-kgm2").and_then(|v| v.parse::<f64>().ok()),
        arg(&args, "--rotation-distance-mm").and_then(|v| v.parse::<f64>().ok()),
    ) {
        let n = r.params.mass.len();
        let m_light = if n == 2 {
            r.params.mass[0][0] + r.params.mass[0][1]
        } else {
            r.params.mass[0][0]
        };
        eprintln!(
            "recommended C00.06 (light direction): {:.0}%",
            c0006_recommendation(m_light, t, d, j)
        );
        if n == 2 {
            let m_heavy = r.params.mass[0][0] - r.params.mass[0][1];
            eprintln!(
                "heavy-direction equivalent (reference only): {:.0}%",
                c0006_recommendation(m_heavy, t, d, j)
            );
        }
    }

    let rms = vec![r.rms_residual; axes.len()];
    let profile = render_profile(&r.params, &axes, &rms);
    let out = arg(&args, "--out").expect("--out");
    std::fs::write(&out, profile).unwrap_or_else(|e| {
        eprintln!("servo-ident: write {out}: {e}");
        std::process::exit(1);
    });
    eprintln!("profile written to {out}");
}
```

- [ ] **Step 5: Run crate suite + build bins, commit**

Run: `cd rust && cargo nextest run -p servo-ident && cargo build -p servo-ident --bins`
Expected: PASS, clean build.

```bash
git add rust/servo-ident/
git commit -m "feat(servo-ident): excitation generator and fitter CLIs"
```

### Task 13: Cross-crate contract test — fitter output loads in the endpoint

**Files:**
- Create: `rust/kalico-ethercat-rt/tests/profile_contract.rs`
- Modify: `rust/kalico-ethercat-rt/Cargo.toml` (dev-dependency)

- [ ] **Step 1: Add dev-dependency**

In `rust/kalico-ethercat-rt/Cargo.toml` `[dev-dependencies]`:
`servo-ident = { path = "../servo-ident" }`

- [ ] **Step 2: Write the contract test**

```rust
// rust/kalico-ethercat-rt/tests/profile_contract.rs
use kalico_ethercat_rt::dynamics::DynamicsModel;
use servo_ident::model::PhysicalParams;
use servo_ident::profile_out::render_profile;

#[test]
fn fitter_rendered_profile_loads_and_evaluates() {
    let p = PhysicalParams {
        mass: vec![vec![0.030, -0.010], vec![-0.010, 0.030]],
        viscous: vec![0.004, 0.005],
        coulomb_fwd: vec![1.0, 0.9],
        coulomb_rev: vec![-1.1, -0.8],
    };
    let text = render_profile(&p, &["a", "b"], &[0.5, 0.6]);
    let m = DynamicsModel::from_toml_str(&text).expect("fitter output must load");
    assert_eq!(m.n, 2);
    let tau = m.torque_ff(0, &[1000.0, -1000.0], &[0.0, 0.0]);
    assert!((tau - 40.0).abs() < 1e-3); // heavy direction: (0.030+0.010)*1000
}
```

- [ ] **Step 3: Run, commit**

Run: `cd rust && cargo nextest run -p kalico-ethercat-rt -E 'test(contract)'`
Expected: PASS.

```bash
git add rust/kalico-ethercat-rt/Cargo.toml rust/kalico-ethercat-rt/tests/profile_contract.rs rust/Cargo.lock
git commit -m "test: profile contract between servo-ident and the endpoint loader"
```

---

# Part C — docs and bench

### Task 14: Documentation + full-suite gate

**Files:**
- Create: `docs/kalico-rewrite/servo-feedforward.md`
- Modify: `docs/kalico-rewrite/ethercat-bench-bringup.md` (config sample +
  new flags)
- Modify: `docs/superpowers/specs/2026-06-10-servo-feedforward-identification-design.md`
  (add `coulomb_deadband_mm_s` to the profile example — implementation
  promoted the deadband into the profile so fitter and runtime share one
  source)

- [ ] **Step 1: Write `docs/kalico-rewrite/servo-feedforward.md`**

Content (prose, concise): the motor-space dynamics model and units table;
the four `[servo_*]` keys (`velocity_ff`, `dynamics_profile`,
`ff_torque_clamp`, existing `rotation_distance`/`encoder_counts_per_rev`);
the identification workflow (servo-excite → run under capture → servo-ident
→ profile → config); the rollout ladder from the spec (remap soak →
velocity FF → ident → torque FF); the capture CSV interim contract; what
the saturation counter in StatusHeartbeat means and the NaN fault. Link the
spec for rationale.

- [ ] **Step 2: Update the bring-up doc's sample config**

Add to the `[servo_x]` sample in `ethercat-bench-bringup.md`:

```ini
#velocity_ff: True              # stream 60B1h velocity feedforward
#dynamics_profile: dynamics_x.toml  # enables 60B2h torque feedforward
#ff_torque_clamp: 30.0          # torque-offset clamp, % of rated
```

and a one-line note that bring-up now performs the variable PDO remap
(bringup rc -6) and FF routing writes (rc -7).

- [ ] **Step 3: Full gate**

Run: `cd rust && cargo nextest run && cargo test --doc`
Expected: entire workspace PASS.

- [ ] **Step 4: Commit**

```bash
git add docs/
git commit -m "docs: servo feedforward + identification workflow"
```

### Task 15: Bench validation (Pi + user-supervised; no G-code without explicit per-command permission)

Follow the bench flow: commit → push → pull on the Pi → build there → flash
if MCU configs changed (they did not — this work is host/endpoint-side
only; `make -f Makefile.kalico ethercat-endpoint-hw` + `setcap-ethercat`
rebuild the endpoint).

- [ ] **Step 1: Pi build** — `make -f Makefile.kalico ethercat-endpoint-hw`
  must compile `libecrt.c` (first real C compile of Task 5) and link; then
  `make -f Makefile.kalico setcap-ethercat`.
- [ ] **Step 2: Remap soak (rollout 1)** — no new config keys; restart
  klippy, claim must succeed (PRE-OP remap + FF routing happen inside
  bring-up). Confirm `ready`, telemetry line shows `vel_act`/`tq_act`
  moving, jog behaves identically to pre-change (user runs the motion
  commands).
- [ ] **Step 3: Velocity FF (rollout 2)** — user adds `velocity_ff: True`;
  compare following error during identical strokes with/without (capture or
  the `ferr` telemetry). Verify sign/scale: `vel_act` tracks commanded
  velocity, following error shrinks, drive does not fault.
- [ ] **Step 4: Identification (rollout 3)** — `servo-excite` output
  reviewed, run under layer-2 capture (user-triggered), `servo-ident` fit;
  cross-check recommended C00.06 against the drive's F30.10 auto-tune.
- [ ] **Step 5: Torque FF (rollout 4)** — user adds `dynamics_profile`;
  following-error comparison again; watch `ff_sat` stays 0 on normal moves.
- [ ] **Step 6: Record results** in
  `docs/kalico-rewrite/ethercat-bench-bringup.md` (dated section, as prior
  bench results are recorded). Commit.

---

## Self-review notes (already applied)

- Spec coverage: model/units → Task 3; excitation → Task 12; fit + refusal →
  Task 10; profile + C00.06 → Task 11; PDO remap + FF routing → Task 5;
  accel eval → Tasks 1–2; per-cycle FF + clamp + NaN fault → Task 6;
  saturation in heartbeat → Tasks 4/6; spawn/config plumbing → Task 7;
  contract seam → Task 13; docs + rollout → Tasks 14–15. Shadow axes and
  multi-slave: out of scope per spec.
- The coulomb deadband moved into the profile file (single source for fitter
  and runtime); Task 14 syncs the spec example.
- Type checks: `clamp_torque(f32, i16, &mut u32) -> i16` consistent across
  Tasks 3/6; `sample → Option<(f32, f32, f32)>` across Tasks 2/6;
  `status_heartbeat_frame(u8, &[u32], u32)` across Tasks 4/6;
  `render_profile(&PhysicalParams, &[&str], &[f64]) -> String` across
  Tasks 11/13.
