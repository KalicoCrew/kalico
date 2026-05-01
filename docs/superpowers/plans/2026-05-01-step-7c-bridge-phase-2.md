# Step 7-C-bridge Phase 2 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire `motion_toolhead.move()` end-to-end through the Rust planner pipeline to produce correct step events on the MCU for single-axis test moves.

**Architecture:** Bridge constructs `CubicSegment` from control points (via `compat::collinear::to_collinear_bezier`), enqueues to a background planner thread that runs temporal TOPP-RA → trajectory shape_batch, then pushes shaped per-axis scalar curves to MCUs. Two MCUs: Octopus H723 (X/Y, CoreXY) and F446 bottom (Z, Cartesian). Full smooth-ZV/MZV shaper + β-medium active from the start.

**Tech Stack:** Rust (motion-bridge PyO3 crate, kalico-host-rt, trajectory, temporal, geometry, compat, nurbs), Python (klippy motion_toolhead.py)

**Spec:** `docs/superpowers/specs/2026-05-01-step-7c-bridge-phase-2-design.md`

---

## File Map

### New files

| File | Responsibility |
|------|---------------|
| `rust/motion-bridge/src/planner.rs` | Planner thread: channel loop, window accumulation, pipeline orchestration, per-MCU dispatch |
| `rust/motion-bridge/src/classify.rs` | Move classification (XY-only, Z-only, Travel) and `CubicSegment` construction |
| `rust/motion-bridge/src/config.rs` | `PlannerConfig` / `PlannerLimits` / `ShaperConfig` structs parsed from Python dicts |
| `rust/motion-bridge/src/dispatch.rs` | Per-MCU axis mapping, curve loading, segment push sequencing |
| `rust/motion-bridge/tests/sim_motion.rs` | kalico-sim integration tests for single-axis moves + shaper validation |

### Modified files

| File | Changes |
|------|---------|
| `rust/compat/src/collinear.rs` | Add `to_collinear_bezier()` returning `[[f64; 3]; 4]` |
| `rust/kalico-host-rt/src/wire.rs` | Add `encode_load_curve_scalar()` for per-axis 1D scalar curves |
| `rust/kalico-host-rt/src/producer.rs` | Add `load_curve()` transport function |
| `rust/motion-bridge/src/lib.rs` | Register new modules |
| `rust/motion-bridge/src/bridge.rs` | Add motion submission PyO3 methods (`submit_move`, `wait_moves`, `submit_dwell`, etc.), planner thread lifecycle |
| `rust/motion-bridge/Cargo.toml` | Add `crossbeam-channel` dependency |
| `klippy/motion_toolhead.py` | Un-stub `move()`, `manual_move()`, `dwell()`, `wait_moves()`, `get_last_move_time()`, `set_position()`, add `SET_INPUT_SHAPER` |

---

## Stage A — Bottom-up foundation: compat + wire encoder + curve loader

### Task 1: `compat::collinear::to_collinear_bezier`

**Files:**
- Modify: `rust/compat/src/collinear.rs`

- [ ] **Step 1: Write failing test**

Add to the bottom of `rust/compat/src/collinear.rs`, inside a `#[cfg(test)] mod tests` block:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn collinear_bezier_matches_g5line() {
        let start = [10.0, 20.0, 0.0];
        let end = [40.0, 50.0, 0.0];
        let bezier = to_collinear_bezier(start, end);

        // P0 = start
        assert_eq!(bezier[0], start);
        // P3 = end
        assert_eq!(bezier[3], end);
        // P1 = start + (end-start)/3
        let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
        assert!((bezier[1][0] - (start[0] + d[0] / 3.0)).abs() < 1e-12);
        assert!((bezier[1][1] - (start[1] + d[1] / 3.0)).abs() < 1e-12);
        assert!((bezier[1][2] - (start[2] + d[2] / 3.0)).abs() < 1e-12);
        // P2 = start + 2*(end-start)/3
        assert!((bezier[2][0] - (start[0] + 2.0 * d[0] / 3.0)).abs() < 1e-12);
        assert!((bezier[2][1] - (start[1] + 2.0 * d[1] / 3.0)).abs() < 1e-12);
        assert!((bezier[2][2] - (start[2] + 2.0 * d[2] / 3.0)).abs() < 1e-12);
    }

    #[test]
    fn collinear_bezier_z_axis_only() {
        let start = [0.0, 0.0, 5.0];
        let end = [0.0, 0.0, 10.0];
        let bezier = to_collinear_bezier(start, end);
        assert_eq!(bezier[0], start);
        assert_eq!(bezier[3], end);
        assert!((bezier[1][2] - (5.0 + 5.0 / 3.0)).abs() < 1e-12);
        assert!((bezier[2][2] - (5.0 + 10.0 / 3.0)).abs() < 1e-12);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo test -p compat -- collinear_bezier_matches_g5line`
Expected: FAIL — `to_collinear_bezier` not found.

- [ ] **Step 3: Implement `to_collinear_bezier`**

Add to `rust/compat/src/collinear.rs` above the `#[cfg(test)]` block:

```rust
/// Structured variant of `to_collinear_g5` for bridge callers.
///
/// Returns the 4 cubic Bézier control points [P0, P1, P2, P3] directly
/// as `[[f64; 3]; 4]`. Same 1/3-2/3 lerp math as `to_collinear_g5`,
/// without the G5Line text intermediary.
pub fn to_collinear_bezier(start: [f64; 3], end: [f64; 3]) -> [[f64; 3]; 4] {
    let d = [
        end[0] - start[0],
        end[1] - start[1],
        end[2] - start[2],
    ];
    let p1 = [
        start[0] + d[0] / 3.0,
        start[1] + d[1] / 3.0,
        start[2] + d[2] / 3.0,
    ];
    let p2 = [
        start[0] + 2.0 * d[0] / 3.0,
        start[1] + 2.0 * d[1] / 3.0,
        start[2] + 2.0 * d[2] / 3.0,
    ];
    [start, p1, p2, end]
}
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p compat -- collinear_bezier`
Expected: 2 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/compat/src/collinear.rs
git commit -m "feat(compat): add to_collinear_bezier() structured API for bridge"
```

### Task 2: Scalar wire encoder

**Files:**
- Modify: `rust/kalico-host-rt/src/wire.rs`

- [ ] **Step 1: Write failing test**

Add to the `#[cfg(test)] mod tests` block in `rust/kalico-host-rt/src/wire.rs`:

```rust
#[test]
fn scalar_encoder_header_and_length() {
    let knots = [0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let cps = [0.0_f32, 3.33, 6.67, 10.0];
    let blob = encode_load_curve_scalar(3, &knots, &cps);
    assert_eq!(blob[0], FORMAT_VERSION_V1);
    assert_eq!(blob[1], 3, "degree");
    assert_eq!(blob[2], 4, "num_cps");
    assert_eq!(blob[3], 8, "num_knots");
    assert_eq!(blob[4], 0, "num_weights (always 0 for scalar)");
    // 5-byte header + 4 cps × 4 bytes + 8 knots × 4 bytes = 5 + 16 + 32 = 53
    assert_eq!(blob.len(), 53);
}

#[test]
fn scalar_encoder_values_are_le() {
    let knots = [0.0_f32, 1.0];
    let cps = [1.5_f32];
    let blob = encode_load_curve_scalar(0, &knots, &cps);
    // CP at offset 5
    let cp_bytes: [u8; 4] = blob[5..9].try_into().unwrap();
    assert_eq!(f32::from_le_bytes(cp_bytes), 1.5);
    // Knot 0 at offset 9
    let k0_bytes: [u8; 4] = blob[9..13].try_into().unwrap();
    assert_eq!(f32::from_le_bytes(k0_bytes), 0.0);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo test -p kalico-host-rt -- scalar_encoder`
Expected: FAIL — `encode_load_curve_scalar` not found.

- [ ] **Step 3: Implement `encode_load_curve_scalar`**

Add to `rust/kalico-host-rt/src/wire.rs` below `encode_load_curve_v1`:

```rust
/// Encode a `kalico_load_curve` blob for per-axis scalar curves (Step 7-B+).
///
/// Wire layout (V1 format, scalar variant):
///
/// ```text
/// [u8 format_version=0x01]
/// [u8 degree]
/// [u8 num_cps]
/// [u8 num_knots]
/// [u8 num_weights=0]
/// [num_cps × f32_le]   // scalar control points
/// [num_knots × f32_le] // knot vector
/// ```
pub fn encode_load_curve_scalar(degree: u8, knots: &[f32], cps: &[f32]) -> Vec<u8> {
    debug_assert!(u8::try_from(cps.len()).is_ok());
    debug_assert!(u8::try_from(knots.len()).is_ok());
    let mut out = Vec::with_capacity(5 + cps.len() * 4 + knots.len() * 4);
    out.push(FORMAT_VERSION_V1);
    out.push(degree);
    out.push(cps.len() as u8);
    out.push(knots.len() as u8);
    out.push(0); // num_weights — always 0 for polynomial scalar curves
    for &v in cps {
        out.extend_from_slice(&v.to_le_bytes());
    }
    for &k in knots {
        out.extend_from_slice(&k.to_le_bytes());
    }
    out
}
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt -- scalar_encoder`
Expected: 2 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-host-rt/src/wire.rs
git commit -m "feat(host-rt): add encode_load_curve_scalar for per-axis scalar curves"
```

### Task 3: Host-side `load_curve` transport function

**Files:**
- Modify: `rust/kalico-host-rt/src/producer.rs`

This task adds the `load_curve` function that encodes a scalar NURBS, sends the `kalico_load_curve` wire command, waits for `kalico_load_curve_response`, and returns a `CurveHandle`.

- [ ] **Step 1: Write failing test**

Add to `rust/kalico-host-rt/src/producer.rs` in a `#[cfg(test)] mod tests` block:

```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn curve_load_params_encodes_correctly() {
        let params = CurveLoadParams {
            degree: 3,
            knots_f32: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            cps_f32: vec![0.0, 3.33, 6.67, 10.0],
        };
        let blob = params.encode();
        assert_eq!(blob[0], crate::wire::FORMAT_VERSION_V1);
        assert_eq!(blob[1], 3);
        assert_eq!(blob[2], 4); // num_cps
        assert_eq!(blob[3], 8); // num_knots
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo test -p kalico-host-rt -- curve_load_params`
Expected: FAIL — `CurveLoadParams` not found.

- [ ] **Step 3: Implement `CurveLoadParams` and `load_curve`**

Add to `rust/kalico-host-rt/src/producer.rs`:

```rust
/// Parameters for loading a single scalar curve into an MCU's curve pool.
#[derive(Debug, Clone)]
pub struct CurveLoadParams {
    pub degree: u8,
    pub knots_f32: Vec<f32>,
    pub cps_f32: Vec<f32>,
}

impl CurveLoadParams {
    /// Encode this curve as a V1 scalar wire blob.
    pub fn encode(&self) -> Vec<u8> {
        crate::wire::encode_load_curve_scalar(self.degree, &self.knots_f32, &self.cps_f32)
    }

    /// Construct from a `ScalarNurbs<f64>`, truncating to f32.
    pub fn from_scalar_nurbs(nurbs: &nurbs::ScalarNurbs<f64>) -> Self {
        Self {
            degree: nurbs.degree(),
            knots_f32: nurbs.knots().iter().map(|&k| k as f32).collect(),
            cps_f32: nurbs.control_points().iter().map(|&v| v as f32).collect(),
        }
    }
}

/// Load a scalar curve into the MCU's curve pool.
///
/// Sends `kalico_load_curve` and waits for `kalico_load_curve_response`.
/// Returns the packed handle `(generation << 16) | slot_idx`.
pub fn load_curve<T: Transport>(
    io: &T,
    params: &CurveLoadParams,
    timeout: Duration,
) -> Result<u32, ProducerError> {
    let blob = params.encode();
    let cmd = format!("kalico_load_curve data={}", hex::encode(&blob));
    let resp = io.send_with_response(&cmd, timeout)?;

    let slot = resp
        .get_u16("slot")
        .ok_or(ProducerError::McuRejected(-1))?;
    let generation = resp
        .get_u16("generation")
        .ok_or(ProducerError::McuRejected(-2))?;

    Ok((generation as u32) << 16 | slot as u32)
}
```

Note: The exact `Transport::send_with_response` API and response-parsing shape may differ from above. The implementer must check the actual `Transport` trait in `rust/kalico-host-rt/src/transport.rs` and adapt the `load_curve` body to match. The key contract is: send the blob, get back `(slot, generation)`, return the packed handle. If `Transport` does not have a `send_with_response` method, use the existing `send` + notify pattern from the passthrough router.

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-host-rt -- curve_load_params`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-host-rt/src/producer.rs
git commit -m "feat(host-rt): add CurveLoadParams + load_curve for scalar curve loading"
```

---

## Stage B — Bridge internals: classify + config + planner thread

### Task 4: Move classifier (`classify.rs`)

**Files:**
- Create: `rust/motion-bridge/src/classify.rs`
- Modify: `rust/motion-bridge/src/lib.rs`

- [ ] **Step 1: Write failing test**

Create `rust/motion-bridge/src/classify.rs` with test at bottom:

```rust
//! Move classification and CubicSegment construction.

use compat::collinear::to_collinear_bezier;
use geometry::segment::{CubicSegment, EMode, SourceRange};
use nurbs::VectorNurbs;

#[derive(Debug)]
pub enum MoveClass {
    /// XY travel (no Z, no E). Includes pure-X and pure-Y.
    XyTravel,
    /// Z-only move.
    ZOnly,
}

#[derive(Debug)]
pub struct ClassifiedMove {
    pub segment: CubicSegment,
    pub class: MoveClass,
}

/// Classify a G1-style delta move and construct a CubicSegment.
///
/// Returns `Err` if `de != 0` (Phase 2 does not support extrusion) or
/// if the move has zero displacement.
pub fn classify_and_build(
    start: [f64; 3],
    dx: f64,
    dy: f64,
    dz: f64,
    de: f64,
    feedrate_mm_s: f64,
) -> Result<ClassifiedMove, ClassifyError> {
    if de.abs() > 1e-9 {
        return Err(ClassifyError::ExtrusionNotSupported);
    }
    let end = [start[0] + dx, start[1] + dy, start[2] + dz];
    let has_xy = dx.abs() > 1e-9 || dy.abs() > 1e-9;
    let has_z = dz.abs() > 1e-9;

    if !has_xy && !has_z {
        return Err(ClassifyError::ZeroDisplacement);
    }

    let class = if has_xy { MoveClass::XyTravel } else { MoveClass::ZOnly };

    let cps = to_collinear_bezier(start, end);
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
        None,
    )
    .map_err(|e| ClassifyError::NurbsConstruction(format!("{e:?}")))?;

    let segment = CubicSegment::try_new(
        xyz,
        EMode::Travel,
        0.0,       // extrusion_per_xy_mm
        None,      // e_independent
        feedrate_mm_s,
        SourceRange { start_line: 0, end_line: 0 },
        None,      // split_info
    )
    .map_err(|e| ClassifyError::SegmentConstruction(format!("{e:?}")))?;

    Ok(ClassifiedMove { segment, class })
}

#[derive(Debug)]
pub enum ClassifyError {
    ExtrusionNotSupported,
    ZeroDisplacement,
    NurbsConstruction(String),
    SegmentConstruction(String),
}

impl std::fmt::Display for ClassifyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ExtrusionNotSupported => write!(f, "extrusion not yet supported (Phase 2)"),
            Self::ZeroDisplacement => write!(f, "zero displacement move"),
            Self::NurbsConstruction(e) => write!(f, "NURBS construction: {e}"),
            Self::SegmentConstruction(e) => write!(f, "segment construction: {e}"),
        }
    }
}

impl std::error::Error for ClassifyError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xy_travel_classifies_correctly() {
        let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 0.0, 100.0).unwrap();
        assert!(matches!(m.class, MoveClass::XyTravel));
        assert_eq!(m.segment.e_mode, EMode::Travel);
        assert_eq!(m.segment.feedrate_mm_s, 100.0);
        let cps = m.segment.xyz.control_points();
        assert_eq!(cps.len(), 4);
        assert_eq!(cps[0], [0.0, 0.0, 0.0]);
        assert!((cps[3][0] - 10.0).abs() < 1e-12);
    }

    #[test]
    fn z_only_classifies_correctly() {
        let m = classify_and_build([0.0, 0.0, 5.0], 0.0, 0.0, 5.0, 0.0, 50.0).unwrap();
        assert!(matches!(m.class, MoveClass::ZOnly));
    }

    #[test]
    fn extrusion_rejected() {
        let r = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 1.0, 100.0);
        assert!(matches!(r, Err(ClassifyError::ExtrusionNotSupported)));
    }

    #[test]
    fn zero_displacement_rejected() {
        let r = classify_and_build([0.0; 3], 0.0, 0.0, 0.0, 0.0, 100.0);
        assert!(matches!(r, Err(ClassifyError::ZeroDisplacement)));
    }
}
```

- [ ] **Step 2: Register module in `lib.rs`**

Add to `rust/motion-bridge/src/lib.rs`:

```rust
mod classify;
```

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p motion-bridge -- classify`
Expected: 4 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/classify.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(motion-bridge): move classifier with CubicSegment construction"
```

### Task 5: Planner config types (`config.rs`)

**Files:**
- Create: `rust/motion-bridge/src/config.rs`
- Modify: `rust/motion-bridge/src/lib.rs`

- [ ] **Step 1: Create `config.rs` with types and tests**

```rust
//! Planner configuration types. Parsed from klippy's printer.cfg values
//! passed through PyO3 at bridge init and runtime updates.

use temporal::Limits;
use trajectory::{ShaperConfig, RequiredShaper, AxisShaper, ELimits};

/// Full planner configuration snapshot.
#[derive(Debug, Clone)]
pub struct PlannerConfig {
    pub limits: PlannerLimits,
    pub shaper: ShaperConfig,
    pub e_limits: ELimits,
    pub window_capacity: usize,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub fit_tolerance_mm: f64,
    pub worker_threads: usize,
}

/// Dynamic velocity/acceleration limits (updateable at runtime).
#[derive(Debug, Clone, Copy)]
pub struct PlannerLimits {
    pub max_velocity: f64,
    pub max_accel: f64,
    pub max_z_velocity: f64,
    pub max_z_accel: f64,
    pub square_corner_velocity: f64,
}

impl PlannerLimits {
    /// Convert to temporal's `Limits` struct.
    pub fn to_temporal_limits(&self) -> Limits {
        Limits::new(
            [self.max_velocity, self.max_velocity, self.max_z_velocity],
            [self.max_accel, self.max_accel, self.max_z_accel],
            // Jerk limits: use 2× accel as a reasonable default.
            // The β-medium loop will further constrain if needed.
            [self.max_accel * 2.0, self.max_accel * 2.0, self.max_z_accel * 2.0],
            self.square_corner_velocity.powi(2) / (self.max_accel * 0.5),
        )
    }
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            limits: PlannerLimits {
                max_velocity: 300.0,
                max_accel: 3000.0,
                max_z_velocity: 15.0,
                max_z_accel: 100.0,
                square_corner_velocity: 5.0,
            },
            shaper: ShaperConfig {
                x: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
                y: RequiredShaper::SmoothMzv { frequency_hz: 50.0 },
                z: AxisShaper::Passthrough,
            },
            e_limits: ELimits { v_max: 50.0, a_max: 5000.0 },
            window_capacity: 32,
            beta_max_iters: 10,
            beta_convergence_ratio: 0.05,
            fit_tolerance_mm: 0.005,
            worker_threads: 3,
        }
    }
}

/// Parse a shaper type string into a `RequiredShaper`.
pub fn parse_required_shaper(name: &str, freq: f64) -> Result<RequiredShaper, String> {
    match name {
        "smooth_zv" | "smooth-zv" => Ok(RequiredShaper::SmoothZv { frequency_hz: freq }),
        "smooth_mzv" | "smooth-mzv" => Ok(RequiredShaper::SmoothMzv { frequency_hz: freq }),
        other => Err(format!("unsupported shaper type for MVP: '{other}'. Use smooth_zv or smooth_mzv")),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_has_sensible_values() {
        let c = PlannerConfig::default();
        assert_eq!(c.window_capacity, 32);
        assert_eq!(c.beta_max_iters, 10);
    }

    #[test]
    fn temporal_limits_converts() {
        let l = PlannerLimits {
            max_velocity: 300.0,
            max_accel: 3000.0,
            max_z_velocity: 15.0,
            max_z_accel: 100.0,
            square_corner_velocity: 5.0,
        };
        let tl = l.to_temporal_limits();
        assert_eq!(tl.v_max[0], 300.0);
        assert_eq!(tl.v_max[2], 15.0);
        assert_eq!(tl.a_max[0], 3000.0);
    }

    #[test]
    fn parse_shaper_types() {
        assert!(matches!(
            parse_required_shaper("smooth_mzv", 50.0),
            Ok(RequiredShaper::SmoothMzv { frequency_hz }) if (frequency_hz - 50.0).abs() < 1e-9
        ));
        assert!(parse_required_shaper("ei", 50.0).is_err());
    }
}
```

- [ ] **Step 2: Register module in `lib.rs`**

Add to `rust/motion-bridge/src/lib.rs`:

```rust
mod config;
```

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p motion-bridge -- config`
Expected: 3 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/config.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(motion-bridge): planner config types with temporal/trajectory conversion"
```

### Task 6: Planner thread core (`planner.rs`)

**Files:**
- Create: `rust/motion-bridge/src/planner.rs`
- Modify: `rust/motion-bridge/src/lib.rs`
- Modify: `rust/motion-bridge/Cargo.toml`

This is the largest single task. The planner thread receives `PlannerMsg` messages, accumulates moves in a window, runs the pipeline (temporal → trajectory), and dispatches shaped segments. For this task, dispatch is a callback — the actual MCU push logic comes in Task 7.

- [ ] **Step 1: Add `crossbeam-channel` dependency**

Add to `rust/motion-bridge/Cargo.toml` under `[dependencies]`:

```toml
crossbeam-channel = "0.5"
```

- [ ] **Step 2: Create `planner.rs` with channel types, thread loop, and unit tests**

Create `rust/motion-bridge/src/planner.rs`. The file should contain:

1. `PlannerMsg` enum (Move, Dwell, Flush, UpdateLimits, UpdateShaper, Shutdown)
2. `PendingMove` struct wrapping `ClassifiedMove`
3. `PlannerHandle` struct with `sender: crossbeam_channel::Sender<PlannerMsg>` and `join_handle: Option<JoinHandle<()>>`
4. `PlannerHandle::spawn(config, dispatch_fn)` — spawns the thread
5. `PlannerHandle::submit_move(move)`, `flush()`, `dwell()`, `update_limits()`, `update_shaper()`, `shutdown()`
6. Internal `run_loop` function
7. Internal `run_pipeline(segments, config) -> Vec<ShapedSegment>` calling `trajectory::shape_batch`
8. `PlannerError` enum
9. Shared error state: `Arc<Mutex<Option<PlannerError>>>`

The planner thread loop (from spec §2.3):
- Block on `recv`.
- `try_recv` to drain all immediately-available messages.
- If buffer reaches `window_capacity` or a Flush/Dwell/Shutdown is seen, run the pipeline.
- Call `dispatch_fn` for each `ShapedSegment`.
- For Flush: wait until dispatch completes, then sleep until `t_end` has elapsed, then wake the notify.
- For Dwell: same as Flush but additionally advance internal `print_time_offset` by the dwell duration.
- Store errors in the shared mutex.

Key pipeline call:

```rust
fn run_pipeline(
    segments: &[CubicSegment],
    config: &PlannerConfig,
) -> Result<Vec<trajectory::ShapedSegment>, PlannerError> {
    let seg_inputs: Vec<trajectory::ShapeSegmentInput<'_>> = segments
        .iter()
        .map(|seg| trajectory::ShapeSegmentInput {
            temporal: temporal::multi::SegmentInput {
                curve: &seg.xyz,
                limits: config.limits.to_temporal_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: seg.e_mode,
            extrusion_per_xy_mm: seg.extrusion_per_xy_mm,
            e_independent: seg.e_independent.as_ref(),
            feedrate_mm_s: seg.feedrate_mm_s,
        })
        .collect();

    let input = trajectory::ShapeBatchInput {
        segments: &seg_inputs,
        grid_strategy: temporal::multi::GridStrategy::Adaptive {
            min_n: 20,
            max_n: 200,
            target_grid_spacing_mm: 0.5,
        },
        worker_threads: config.worker_threads,
        shaper: config.shaper.clone(),
        fit_tolerance_mm: config.fit_tolerance_mm,
        beta_max_iters: config.beta_max_iters,
        beta_convergence_ratio: config.beta_convergence_ratio,
        e_limits: config.e_limits,
    };

    let output = trajectory::shape_batch(&input)
        .map_err(PlannerError::Shape)?;

    Ok(output.segments)
}
```

Unit tests for the channel protocol — verify:
- Submit + flush returns shaped segments via the dispatch callback.
- Shutdown joins the thread cleanly.
- Error in pipeline surfaces on next submit_move.
- UpdateLimits applies to the next batch.

The implementer must write the full thread loop. The spec §2.3 has the pseudocode; translate it to Rust with `crossbeam_channel::Receiver::recv()` + `try_recv()` drain.

- [ ] **Step 3: Register module**

Add to `rust/motion-bridge/src/lib.rs`:

```rust
mod planner;
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p motion-bridge -- planner`
Expected: All planner unit tests PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/planner.rs rust/motion-bridge/src/lib.rs rust/motion-bridge/Cargo.toml
git commit -m "feat(motion-bridge): planner thread with streaming window and shape_batch pipeline"
```

### Task 7: Per-MCU dispatch (`dispatch.rs`)

**Files:**
- Create: `rust/motion-bridge/src/dispatch.rs`
- Modify: `rust/motion-bridge/src/lib.rs`

This module maps shaped segments to MCUs, loads curves, and pushes segments.

- [ ] **Step 1: Create `dispatch.rs` with types and tests**

```rust
//! Per-MCU axis mapping and segment dispatch.

use kalico_host_rt::producer::{CurveLoadParams, SegmentPushParams};

/// UNUSED_SENTINEL packed value — MCU ignores axes with this handle.
pub const UNUSED_HANDLE: u32 = 0xFFFE_FFFE;

/// Axis index constants matching ShapedSegment.axes[].
pub const AXIS_X: usize = 0;
pub const AXIS_Y: usize = 1;
pub const AXIS_Z: usize = 2;

/// Which MCU owns which axes.
#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    pub mcu_id: u32,
    pub axes: Vec<usize>,       // indices into ShapedSegment.axes[]
    pub kinematics: u8,         // 0 = CoreXyAndE, 1 = CartesianXyzAndE
}

/// Build the per-MCU push params for one shaped segment.
///
/// For each MCU, returns the list of `(axis_idx, CurveLoadParams)` to load
/// and the `SegmentPushParams` skeleton (handles filled as UNUSED until
/// the caller fills them from load_curve responses).
pub fn build_push_params(
    shaped: &trajectory::ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    t_start_clock: u64,
    t_end_clock: u64,
) -> Vec<McuPushPlan> {
    let mut plans = Vec::new();

    for mcu_cfg in mcu_configs {
        let mut curves_to_load: Vec<(usize, CurveLoadParams)> = Vec::new();

        for &axis_idx in &mcu_cfg.axes {
            let nurbs = &shaped.axes[axis_idx];
            // Skip trivially constant curves (all control points identical)
            let cps = nurbs.control_points();
            let is_trivial = cps.len() > 1
                && cps.iter().all(|&v| (v - cps[0]).abs() < 1e-12);
            if !is_trivial {
                curves_to_load.push((axis_idx, CurveLoadParams::from_scalar_nurbs(nurbs)));
            }
        }

        if curves_to_load.is_empty() {
            continue;
        }

        let params_skeleton = SegmentPushParams {
            id: 0, // filled by caller
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: t_start_clock,
            t_end: t_end_clock,
            kinematics: mcu_cfg.kinematics,
            e_mode: 2, // Travel
            extrusion_ratio: 0.0,
        };

        plans.push(McuPushPlan {
            mcu_id: mcu_cfg.mcu_id,
            curves_to_load,
            params: params_skeleton,
        });
    }

    plans
}

/// Plan for pushing one segment to one MCU.
#[derive(Debug)]
pub struct McuPushPlan {
    pub mcu_id: u32,
    pub curves_to_load: Vec<(usize, CurveLoadParams)>,
    pub params: SegmentPushParams,
}

impl McuPushPlan {
    /// Set the handle for a given axis after loading the curve.
    pub fn set_handle(&mut self, axis_idx: usize, packed_handle: u32) {
        match axis_idx {
            AXIS_X => self.params.x_handle_packed = packed_handle,
            AXIS_Y => self.params.y_handle_packed = packed_handle,
            AXIS_Z => self.params.z_handle_packed = packed_handle,
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_trivial_nurbs(value: f64) -> nurbs::ScalarNurbs<f64> {
        nurbs::ScalarNurbs::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![value, value, value, value],
            None,
        )
        .unwrap()
    }

    fn make_nontrivial_nurbs() -> nurbs::ScalarNurbs<f64> {
        nurbs::ScalarNurbs::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 3.33, 6.67, 10.0],
            None,
        )
        .unwrap()
    }

    #[test]
    fn x_move_dispatches_to_octopus_only() {
        let shaped = trajectory::ShapedSegment {
            axes: [make_nontrivial_nurbs(), make_trivial_nurbs(0.0), make_trivial_nurbs(0.0)],
            e_mode: geometry::segment::EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        };
        let configs = vec![
            McuAxisConfig { mcu_id: 0, axes: vec![AXIS_X, AXIS_Y], kinematics: 0 },
            McuAxisConfig { mcu_id: 1, axes: vec![AXIS_Z], kinematics: 1 },
        ];
        let plans = build_push_params(&shaped, &configs, 1000, 2000);

        assert_eq!(plans.len(), 1, "only Octopus should get a push");
        assert_eq!(plans[0].mcu_id, 0);
        assert_eq!(plans[0].curves_to_load.len(), 1); // only X is non-trivial
        assert_eq!(plans[0].curves_to_load[0].0, AXIS_X);
        assert_eq!(plans[0].params.z_handle_packed, UNUSED_HANDLE);
    }

    #[test]
    fn z_move_dispatches_to_f446_only() {
        let shaped = trajectory::ShapedSegment {
            axes: [make_trivial_nurbs(0.0), make_trivial_nurbs(0.0), make_nontrivial_nurbs()],
            e_mode: geometry::segment::EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        };
        let configs = vec![
            McuAxisConfig { mcu_id: 0, axes: vec![AXIS_X, AXIS_Y], kinematics: 0 },
            McuAxisConfig { mcu_id: 1, axes: vec![AXIS_Z], kinematics: 1 },
        ];
        let plans = build_push_params(&shaped, &configs, 1000, 2000);

        assert_eq!(plans.len(), 1, "only F446 should get a push");
        assert_eq!(plans[0].mcu_id, 1);
        assert_eq!(plans[0].curves_to_load[0].0, AXIS_Z);
    }
}
```

- [ ] **Step 2: Register module**

Add to `rust/motion-bridge/src/lib.rs`:

```rust
mod dispatch;
```

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p motion-bridge -- dispatch`
Expected: 2 tests PASS.

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/dispatch.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(motion-bridge): per-MCU dispatch with axis mapping and push plan"
```

---

## Stage C — PyO3 surface: bridge methods + Python shim

### Task 8: Bridge motion methods (`bridge.rs`)

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

Add the motion-submission PyO3 methods to `PyMotionBridge`. This wires classify → planner thread → dispatch. The planner thread is spawned lazily on first `submit_move` or eagerly on an `init_planner` call.

- [ ] **Step 1: Add fields to `PyMotionBridge`**

Add to the struct:

```rust
planner: Mutex<Option<planner::PlannerHandle>>,
planner_config: Mutex<config::PlannerConfig>,
commanded_pos: Mutex<[f64; 3]>,
planner_error: Arc<Mutex<Option<planner::PlannerError>>>,
```

- [ ] **Step 2: Add PyO3 methods**

Add these `#[pymethods]` to `PyMotionBridge`:

```rust
/// Initialize the planner thread with config from printer.cfg.
#[pyo3(signature = (max_velocity, max_accel, max_z_velocity, max_z_accel,
                    square_corner_velocity,
                    shaper_type_x, shaper_freq_x,
                    shaper_type_y, shaper_freq_y,
                    window_capacity=32, beta_max_iters=10))]
fn init_planner(
    &self,
    max_velocity: f64,
    max_accel: f64,
    max_z_velocity: f64,
    max_z_accel: f64,
    square_corner_velocity: f64,
    shaper_type_x: &str,
    shaper_freq_x: f64,
    shaper_type_y: &str,
    shaper_freq_y: f64,
    window_capacity: usize,
    beta_max_iters: u8,
) -> PyResult<()> {
    // Build PlannerConfig, spawn planner thread.
    // Store in self.planner.
    todo!("implementer fills this")
}

/// Submit a travel move. Phase 2: de must be 0.
#[pyo3(signature = (dx, dy, dz, de, feedrate))]
fn submit_move(&self, dx: f64, dy: f64, dz: f64, de: f64, feedrate: f64) -> PyResult<()> {
    // Check planner error first.
    // classify_and_build from self.commanded_pos.
    // Enqueue to planner.
    // Update commanded_pos optimistically.
    todo!("implementer fills this")
}

/// Flush all pending moves and block until physical execution completes.
fn wait_moves(&self) -> PyResult<()> {
    // Send Flush to planner, block on notify.
    // Check planner error.
    todo!("implementer fills this")
}

/// Submit a dwell: flush + advance print time.
fn submit_dwell(&self, duration_s: f64) -> PyResult<()> {
    todo!("implementer fills this")
}

/// Reset commanded position.
fn set_position(&self, x: f64, y: f64, z: f64) -> PyResult<()> {
    let mut pos = self.commanded_pos.lock().unwrap();
    *pos = [x, y, z];
    Ok(())
}

/// Update velocity/accel limits at runtime (SET_VELOCITY_LIMIT).
fn update_limits(&self, max_velocity: f64, max_accel: f64) -> PyResult<()> {
    todo!("implementer fills this")
}

/// Update shaper config at runtime (SET_INPUT_SHAPER).
fn update_shaper(&self, shaper_type_x: &str, freq_x: f64,
                  shaper_type_y: &str, freq_y: f64) -> PyResult<()> {
    todo!("implementer fills this")
}

/// Estimated print time of last queued move.
fn get_last_move_time(&self) -> f64 {
    // Return from planner's tracked print_time.
    0.0
}
```

Note: Each `todo!()` has a clear contract from the spec. The implementer fills them using the planner handle, classify module, and config types from Tasks 4-6. The `submit_move` body is roughly:

```rust
{
    let err_guard = self.planner_error.lock().unwrap();
    if let Some(e) = err_guard.as_ref() {
        return Err(PyRuntimeError::new_err(e.to_string()));
    }
    drop(err_guard);

    let pos = *self.commanded_pos.lock().unwrap();
    let classified = classify::classify_and_build(pos, dx, dy, dz, de, feedrate)
        .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

    let planner_guard = self.planner.lock().unwrap();
    let planner = planner_guard.as_ref()
        .ok_or_else(|| PyRuntimeError::new_err("planner not initialized — call init_planner first"))?;
    planner.submit_move(classified)?;

    let mut pos = self.commanded_pos.lock().unwrap();
    pos[0] += dx;
    pos[1] += dy;
    pos[2] += dz;
    Ok(())
}
```

- [ ] **Step 3: Run full bridge build**

Run: `cd rust && cargo build -p motion-bridge`
Expected: Compiles. (Tests for the PyO3 methods require the Python interop tested in Task 9.)

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(motion-bridge): PyO3 motion submission methods on MotionBridge"
```

### Task 9: Python `motion_toolhead.py` un-stubbing

**Files:**
- Modify: `klippy/motion_toolhead.py`

- [ ] **Step 1: Un-stub `move()`, `manual_move()`, `dwell()`, `wait_moves()`, `set_position()`, `get_last_move_time()`**

Replace the existing stubs. Key changes:

```python
def move(self, newpos, speed):
    dx = newpos[0] - self.commanded_pos[0]
    dy = newpos[1] - self.commanded_pos[1]
    dz = newpos[2] - self.commanded_pos[2]
    de = newpos[3] - self.commanded_pos[3]
    feedrate = min(speed, self.max_velocity)
    if abs(dz) > 1e-9 and abs(dx) < 1e-9 and abs(dy) < 1e-9:
        feedrate = min(feedrate, self.max_z_velocity)
    self.bridge.submit_move(dx, dy, dz, de, feedrate)
    self.commanded_pos[:] = newpos

def manual_move(self, coord, speed):
    curpos = list(self.commanded_pos)
    for i in range(len(coord)):
        if coord[i] is not None:
            curpos[i] = coord[i]
    self.move(curpos, speed)

def dwell(self, delay):
    self.bridge.submit_dwell(delay)

def wait_moves(self):
    self.bridge.wait_moves()

def set_position(self, newpos, homing_axes=()):
    self.commanded_pos[:] = newpos
    self.bridge.set_position(newpos[0], newpos[1], newpos[2])

def get_last_move_time(self):
    return self.bridge.get_last_move_time()
```

- [ ] **Step 2: Store bridge reference in `__init__`**

In `__init__`, after the bridge is available (it's already created by `motion_mcu.py` during MCU init), store it:

```python
self.bridge = self.printer.lookup_object('motion_bridge')
```

The implementer must check the exact lookup pattern used in Phase 1's `motion_mcu.py` to find the bridge object.

- [ ] **Step 3: Add `max_z_velocity` / `max_z_accel` config parsing**

In `__init__`, after the existing velocity config:

```python
self.max_z_velocity = config.getfloat("max_z_velocity", self.max_velocity, above=0.0)
self.max_z_accel = config.getfloat("max_z_accel", self.max_accel, above=0.0)
```

- [ ] **Step 4: Wire `init_planner` call**

Add a printer event handler that calls `bridge.init_planner(...)` after all MCUs are configured (during the `klippy:connect` event). The implementer should check what Phase 1 uses for the startup sequence and add the `init_planner` call at the right point.

- [ ] **Step 5: Wire `SET_VELOCITY_LIMIT` to bridge**

In `cmd_SET_VELOCITY_LIMIT`, after updating local values:

```python
if hasattr(self, 'bridge'):
    self.bridge.update_limits(self.max_velocity, self.max_accel)
```

- [ ] **Step 6: Add `SET_INPUT_SHAPER` handler**

Register in `__init__`:

```python
gcode.register_command(
    "SET_INPUT_SHAPER",
    self.cmd_SET_INPUT_SHAPER,
    desc="Set input shaper parameters",
)
```

Implement:

```python
def cmd_SET_INPUT_SHAPER(self, gcmd):
    type_x = gcmd.get("SHAPER_TYPE_X", None)
    freq_x = gcmd.get_float("SHAPER_FREQ_X", None, above=0.0)
    type_y = gcmd.get("SHAPER_TYPE_Y", None)
    freq_y = gcmd.get_float("SHAPER_FREQ_Y", None, above=0.0)
    if type_x is not None and freq_x is not None and type_y is not None and freq_y is not None:
        self.bridge.update_shaper(type_x, freq_x, type_y, freq_y)
```

- [ ] **Step 7: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "feat(klippy): un-stub motion_toolhead move/wait/dwell for Phase 2"
```

---

## Stage D — Integration testing

### Task 10: kalico-sim integration test scaffold

**Files:**
- Create: `rust/motion-bridge/tests/sim_motion.rs`

This test boots the bridge with a simulated MCU and validates end-to-end motion.

- [ ] **Step 1: Create test file with single-axis X move test**

The implementer must study the existing sim infrastructure:
- Check `rust/kalico-host-rt/tests/` for how sim MCUs are created
- Check `rust/runtime/` for the host-sim feature
- Check `rust/motion-bridge/tests/` for existing Phase 1 test patterns

The test structure:

```rust
//! kalico-sim integration tests for Phase 2 motion.

use motion_bridge::bridge::PyMotionBridge;
// ... imports for sim MCU setup ...

/// Boot a bridge with a sim MCU, submit a G1 X10 move, verify step events.
#[test]
fn single_axis_x_move() {
    // 1. Create sim MCU (Octopus-equivalent with CoreXY config)
    // 2. Create bridge, claim MCU, init planner with test config
    // 3. bridge.submit_move(10.0, 0.0, 0.0, 0.0, 100.0)  // G1 X10 F6000
    // 4. bridge.wait_moves()
    // 5. Collect step events from sim
    // 6. Assert: steps on A and B belts (CoreXY), correct direction,
    //    step count within tolerance, monotonic timing
    todo!("implementer builds the full test using sim infrastructure")
}

#[test]
fn single_axis_z_move_different_mcu() {
    // 1. Create 2 sim MCUs (Octopus + F446)
    // 2. Init bridge with both MCUs
    // 3. bridge.submit_move(0.0, 0.0, 5.0, 0.0, 50.0)  // G1 Z5 F3000
    // 4. bridge.wait_moves()
    // 5. Assert: steps only on F446's Z, nothing on Octopus
    todo!("implementer builds this")
}

#[test]
fn extrusion_rejected() {
    // Submit a move with de != 0, verify it returns an error.
    todo!("implementer builds this")
}
```

Note: The exact sim MCU setup depends on the runtime's host-sim feature. The implementer must read the existing test infrastructure and adapt. The key assertion is: step events are captured, direction and count are correct.

- [ ] **Step 2: Run the tests**

Run: `cd rust && cargo test -p motion-bridge --test sim_motion -- --nocapture`
Expected: All tests PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/tests/sim_motion.rs
git commit -m "test(motion-bridge): kalico-sim integration tests for Phase 2 motion"
```

### Task 11: Shaper validation test

**Files:**
- Modify: `rust/motion-bridge/tests/sim_motion.rs`

- [ ] **Step 1: Add shaper validation test**

```rust
#[test]
fn shaper_attenuates_resonance_and_respects_accel_limit() {
    // 1. Boot bridge with sim MCU, configure smooth_mzv at 50Hz, max_accel=3000
    // 2. Submit G1 X50 F6000 (fast move that excites shaper)
    // 3. Capture position trajectory from sim at 40kHz sample rate
    // 4. Compute acceleration by second-differencing the position
    // 5. FFT the acceleration profile
    // 6. Assert: power at 50Hz is attenuated (compare to unshapged expectation)
    // 7. Assert: peak acceleration <= 3000 mm/s² (β-medium guarantee)
    todo!("implementer builds this with actual FFT — use rustfft or similar")
}
```

- [ ] **Step 2: Add velocity limit compliance test**

```rust
#[test]
fn velocity_limit_respected() {
    // 1. Boot bridge, set max_velocity=100
    // 2. Submit G1 X50 F6000 (requested speed >> limit)
    // 3. Capture trajectory, compute velocity
    // 4. Assert: peak velocity <= 100 mm/s
    todo!("implementer builds this")
}
```

- [ ] **Step 3: Add SET_VELOCITY_LIMIT test**

```rust
#[test]
fn set_velocity_limit_applies_to_next_move() {
    // 1. Boot bridge, max_velocity=300
    // 2. Submit G1 X10 F6000, wait
    // 3. update_limits(max_velocity=50, max_accel=500)
    // 4. Submit G1 X20 F6000, wait
    // 5. Assert: second move's peak velocity <= 50
    todo!("implementer builds this")
}
```

- [ ] **Step 4: Run all tests**

Run: `cd rust && cargo test -p motion-bridge --test sim_motion -- --nocapture`
Expected: All tests PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/tests/sim_motion.rs
git commit -m "test(motion-bridge): shaper + velocity limit validation tests"
```

### Task 12: Renode gate test

**Files:**
- Create: `scripts/renode_phase2_gate.sh` (or integrate into existing Renode test harness)

- [ ] **Step 1: Write Renode test script**

This test uses the existing Renode harness from Step 7-C-io. The implementer should:
1. Check `scripts/` and `tests/` for existing Renode test patterns
2. Boot Renode with the H723 firmware binary
3. Connect the bridge to Renode's serial port
4. Submit test moves through the bridge
5. Capture GPIO pin toggles (step pulses) via Renode's LoggingUartAnalyzer or GPIO peripheral logger
6. Verify: correct pin toggles for step pulses

- [ ] **Step 2: Run the Renode test**

Expected: Wire-level protocol verified, step pins toggle.

- [ ] **Step 3: Commit**

```bash
git add scripts/renode_phase2_gate.sh
git commit -m "test: Renode gate test for Phase 2 wire-level motion verification"
```

---

## Stage E — Verify all existing tests still pass

### Task 13: Full regression check

- [ ] **Step 1: Run all Rust tests**

Run: `cd rust && cargo test --workspace`
Expected: All existing Phase 1 tests + new Phase 2 tests PASS.

- [ ] **Step 2: Run Phase 1 Python boot smoke test**

Run whatever CI boot test exists from Phase 1. The implementer should check `scripts/` or the existing test infrastructure.

- [ ] **Step 3: Verify cargo build for the cdylib**

Run: `cd rust && cargo build -p motion-bridge --release`
Expected: `libmotion_bridge.so` (or `.dylib`) builds cleanly.

- [ ] **Step 4: Commit any fixups**

If any existing tests broke, fix them and commit.
