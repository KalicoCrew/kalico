# Phase-stepping math kernel Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a leaf module `rust/runtime/src/phase.rs` containing three pure functions that compute the coil-current waveform `(I_A, I_B) = (cos θ, sin θ)` from a motor position. The module is dead code (importable but unused) until build-order Step 10 wires it into the engine's TIM5 ISR.

**Architecture:** Pure functions in a leaf Rust module. Zero state, zero allocation, zero side effects. The module does not import any other runtime crate module, so it cannot collide with the sibling `step.rs` / `step_time.rs` paths. Host builds use `f32::cos` / `f32::sin` from std; MCU builds (`mcu-h7`, `mcu-f4`) route to `libm::cosf` / `libm::sinf`, mirroring the same pattern `rust/nurbs/src/float.rs` already uses for `fmaf` / `sqrtf` / `fabsf`. Spec: [`docs/superpowers/specs/2026-05-12-phase-stepping-math-kernel-design.md`](../specs/2026-05-12-phase-stepping-math-kernel-design.md).

**Tech Stack:** Rust 2024 edition, runtime crate (`rust/runtime/`), `libm` 0.2 as an optional dependency gated on MCU features, `proptest` 1.5 already a dev-dependency for randomized sweeps.

---

## File structure

Two files modified, one created:

- **Create**: `rust/runtime/src/phase.rs` — the leaf module with three pure functions plus their `#[cfg(test)] mod tests`. Self-contained; nothing else imports it yet.
- **Modify**: `rust/runtime/src/lib.rs` — add `pub mod phase;` to the existing module-declaration block.
- **Modify**: `rust/runtime/Cargo.toml` — add `libm = { version = "0.2", optional = true, default-features = false }` and gate it on the existing `mcu-h7` / `mcu-f4` features.

No engine, state, wire-protocol, kalico-c-api, klippy, or C-side changes.

---

## Task 1: Cargo.toml dependency + module scaffold

**Files:**
- Modify: `rust/runtime/Cargo.toml`
- Create: `rust/runtime/src/phase.rs`
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Add `libm` as an optional MCU-only dependency**

In `rust/runtime/Cargo.toml`, find the `[dependencies]` section (currently lines 11-13):

```toml
[dependencies]
nurbs = { path = "../nurbs", default-features = false }
heapless = { workspace = true }
```

Add `libm` so the section reads:

```toml
[dependencies]
nurbs = { path = "../nurbs", default-features = false }
heapless = { workspace = true }
libm = { version = "0.2", optional = true, default-features = false }
```

Then in the `[features]` section (currently around lines 28-32), the MCU feature lines currently read:

```toml
mcu-h7 = ["nurbs/mcu-h7"]
mcu-f4 = ["nurbs/mcu-f4"]
```

Change them to:

```toml
mcu-h7 = ["nurbs/mcu-h7", "dep:libm"]
mcu-f4 = ["nurbs/mcu-f4", "dep:libm"]
```

Do not touch the `host` feature, `kalico-sim`, `loom`, `test-injection`, or any other Cargo.toml field.

- [ ] **Step 2: Create the empty module**

Create `rust/runtime/src/phase.rs` with this exact content:

```rust
//! Phase-stepping math kernel. Pure functions only.
//!
//! Spec: `docs/superpowers/specs/2026-05-12-phase-stepping-math-kernel-design.md`.
//!
//! Build-order Step 10 (phase stepping) will wire these into the
//! `StepMode::Modulated` path of the TIM5 ISR; until then this module is
//! dead code (importable, unused). Imports nothing from other runtime
//! modules so the eventual integration can decide its dispatch shape
//! without renaming or refactoring this kernel.
```

- [ ] **Step 3: Wire the module into lib.rs**

In `rust/runtime/src/lib.rs`, the current module-declaration block (lines 18-38) ends with `pub mod wire;`. Add `pub mod phase;` to the block. Insert it alphabetically between `pub mod kinematics;` (line 25) and `pub mod queue;` (line 26):

```rust
pub mod kinematics;
pub mod phase;
pub mod queue;
```

- [ ] **Step 4: Verify the runtime crate still compiles in all three feature configurations**

Run each of these and confirm a clean build with no warnings about `phase`:

```bash
cargo build -p runtime --no-default-features --features host
cargo build -p runtime --no-default-features --features mcu-h7
cargo build -p runtime --no-default-features --features mcu-f4
```

Expected for each: `Compiling runtime v0.1.0 ...` then `Finished` with no warnings. The `mcu-h7` and `mcu-f4` builds confirm `libm` activates correctly under those features.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/Cargo.toml rust/runtime/src/phase.rs rust/runtime/src/lib.rs
git -c commit.gpgsign=false commit -m "feat(runtime): scaffold phase math kernel module"
```

---

## Task 2: `electrical_angle_rad` — the mechanical-to-electrical angle conversion

A two-phase hybrid stepper has 4 full mechanical steps per electrical revolution. `λ_mech = 4 / full_steps_per_mm` is the mechanical wavelength (mm per electrical revolution). The electrical angle is then `θ = 2π × motor_position_mm / λ_mech = (π/2) × full_steps_per_mm × motor_position_mm`.

**Files:**
- Modify: `rust/runtime/src/phase.rs`

- [ ] **Step 1: Write the failing test**

Append this to `rust/runtime/src/phase.rs`:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use core::f32::consts::{FRAC_PI_2, PI};

    /// Spot-check the angle at three known points: origin, one full
    /// mechanical step (π/2), one electrical revolution (2π).
    #[test]
    fn electrical_angle_rad_known_points() {
        let s = 80.0_f32;
        // At p=0, angle is 0 regardless of full_steps_per_mm.
        assert_eq!(electrical_angle_rad(0.0, s), 0.0);
        // One full mechanical step = 1/s mm → quarter electrical revolution.
        let one_full_step = 1.0 / s;
        let angle = electrical_angle_rad(one_full_step, s);
        assert!(
            (angle - FRAC_PI_2).abs() < 1e-5,
            "one full step: got {angle}, expected {FRAC_PI_2}",
        );
        // Four full mechanical steps = full electrical revolution (2π).
        let one_electrical_rev = 4.0 / s;
        let angle = electrical_angle_rad(one_electrical_rev, s);
        let expected = 2.0 * PI;
        assert!(
            (angle - expected).abs() < 1e-4,
            "one electrical rev: got {angle}, expected {expected}",
        );
    }
}
```

- [ ] **Step 2: Run the test and verify it fails**

```bash
cargo test -p runtime --lib --no-default-features --features host phase::tests::electrical_angle_rad_known_points
```

Expected: a build error of the form `cannot find function 'electrical_angle_rad' in this scope`. That's the TDD "red" — the function does not exist yet.

- [ ] **Step 3: Implement `electrical_angle_rad`**

In `rust/runtime/src/phase.rs`, insert the function above the `#[cfg(test)] mod tests` block:

```rust
/// Electrical angle of the rotor, in radians, for a two-phase hybrid
/// stepper at the given mechanical position.
///
/// `full_steps_per_mm` is the *full-step* density (full steps per mm of
/// mechanical travel) — typically `full_steps_per_rev * gearing /
/// rotation_distance`. The wire-protocol field `MotorConfig::steps_per_mm`
/// is *microsteps* per mm and must be divided by the microstep
/// multiplier before being passed here. `full_steps_per_mm` is assumed
/// positive; behavior at zero or negative inputs is unspecified.
#[inline]
pub fn electrical_angle_rad(motor_position_mm: f32, full_steps_per_mm: f32) -> f32 {
    // λ_mech = 4 / full_steps_per_mm (mm per electrical revolution)
    // θ = 2π × p / λ_mech = (π/2) × full_steps_per_mm × p
    core::f32::consts::FRAC_PI_2 * full_steps_per_mm * motor_position_mm
}
```

- [ ] **Step 4: Run the test and verify it passes**

```bash
cargo test -p runtime --lib --no-default-features --features host phase::tests::electrical_angle_rad_known_points
```

Expected: `test phase::tests::electrical_angle_rad_known_points ... ok` and `test result: ok. 1 passed`.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/phase.rs
git -c commit.gpgsign=false commit -m "feat(runtime/phase): electrical_angle_rad"
```

---

## Task 3: `phase_currents` — coil-current waveform synthesis

`phase_currents(p, s) = (cos θ, sin θ)` where `θ = electrical_angle_rad(p, s)`. The trig calls route through the same host/MCU `cfg`-split pattern that `rust/nurbs/src/float.rs:46-71` uses for `mul_add` and `sqrt`.

**Files:**
- Modify: `rust/runtime/src/phase.rs`

- [ ] **Step 1: Write the failing closed-form sweep test**

In the existing `#[cfg(test)] mod tests` block in `rust/runtime/src/phase.rs`, append:

```rust
    /// Closed-form sweep: each component of (I_A, I_B) must match an
    /// `f64::cos` / `f64::sin` reference computed against the *same f32
    /// angle* the implementation uses. (Using f64 trig of an f64 angle
    /// would fail at large positions purely from the f32-multiplication
    /// rounding in the angle itself, which is not what we're testing.)
    #[test]
    fn phase_currents_matches_f64_reference_over_sweep() {
        let positions: [f32; 41] = core::array::from_fn(|i| (i as f32 - 20.0) * 50.0);
        for &s in &[40.0_f32, 80.0, 200.0, 400.0] {
            for &p in &positions {
                let (i_a, i_b) = phase_currents(p, s);
                // Reference: f64 cos/sin of the SAME f32 angle our function
                // computed internally. Isolates trig precision from
                // angle-multiplication precision.
                let theta_f32 = electrical_angle_rad(p, s);
                let ref_a = (theta_f32 as f64).cos() as f32;
                let ref_b = (theta_f32 as f64).sin() as f32;
                assert!(
                    (i_a - ref_a).abs() <= 1e-6,
                    "i_a mismatch at p={p} s={s}: got {i_a}, ref {ref_a}",
                );
                assert!(
                    (i_b - ref_b).abs() <= 1e-6,
                    "i_b mismatch at p={p} s={s}: got {i_b}, ref {ref_b}",
                );
            }
        }
    }
```

- [ ] **Step 2: Run the test and verify it fails**

```bash
cargo test -p runtime --lib --no-default-features --features host phase::tests::phase_currents_matches_f64_reference_over_sweep
```

Expected: build error `cannot find function 'phase_currents' in this scope`.

- [ ] **Step 3: Implement `phase_currents` with the host/MCU cfg split**

In `rust/runtime/src/phase.rs`, above the `#[cfg(test)] mod tests` block (i.e., after `electrical_angle_rad`), add the two private trig helpers and the public function:

```rust
#[inline]
fn cos_f32(theta: f32) -> f32 {
    #[cfg(feature = "host")]
    {
        f32::cos(theta)
    }
    #[cfg(not(feature = "host"))]
    {
        libm::cosf(theta)
    }
}

#[inline]
fn sin_f32(theta: f32) -> f32 {
    #[cfg(feature = "host")]
    {
        f32::sin(theta)
    }
    #[cfg(not(feature = "host"))]
    {
        libm::sinf(theta)
    }
}

/// Coil currents `(I_A, I_B)`, each in `[-1.0, +1.0]`, that position the
/// rotor at `motor_position_mm`. `I_A = cos(θ)`, `I_B = sin(θ)`, where
/// `θ = electrical_angle_rad(motor_position_mm, full_steps_per_mm)`.
///
/// Outputs are normalized to full-scale coil current; the eventual MCU
/// emitter is responsible for scaling to the TMC driver's register units.
#[inline]
pub fn phase_currents(motor_position_mm: f32, full_steps_per_mm: f32) -> (f32, f32) {
    let theta = electrical_angle_rad(motor_position_mm, full_steps_per_mm);
    (cos_f32(theta), sin_f32(theta))
}
```

- [ ] **Step 4: Run the test and verify it passes**

```bash
cargo test -p runtime --lib --no-default-features --features host phase::tests::phase_currents_matches_f64_reference_over_sweep
```

Expected: `test phase::tests::phase_currents_matches_f64_reference_over_sweep ... ok`.

- [ ] **Step 5: Add three more invariant tests (each passes on first run — characterization tests guard against future LUT/polynomial regressions)**

Append to the same `#[cfg(test)] mod tests` block:

```rust
    /// Unit-circle invariant: cos²θ + sin²θ = 1. The identity holds at
    /// every angle regardless of magnitude (each component stays in
    /// [-1, 1]), so the full LCG range is safe here.
    #[test]
    fn phase_currents_unit_circle_invariant() {
        // Deterministic pseudo-random sweep using a small LCG. proptest
        // would also work, but for 1000 samples a fixed LCG is faster to
        // read and avoids the proptest harness in this leaf module.
        let mut state: u32 = 0xDEAD_BEEF;
        for _ in 0..1000 {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let p = (state as i32 as f32) * 1e-5;
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let s = 40.0_f32 + (state & 0x3FF) as f32; // ≈ [40, 1064)
            let (i_a, i_b) = phase_currents(p, s);
            let magnitude = i_a * i_a + i_b * i_b;
            assert!(
                (magnitude - 1.0).abs() <= 1e-5,
                "unit-circle invariant violated at p={p} s={s}: magnitude={magnitude}",
            );
        }
    }

    /// One electrical revolution forward returns the same currents.
    ///
    /// Tested at restricted positions (|p| ≤ 10 mm) so f32-multiplication
    /// rounding in θ stays small enough that the periodicity-residual
    /// `(p + 4/s) − p` survives the float roundoff. At |p| ≤ 10, s ≤ 296,
    /// |θ| ≤ ~4650 rad; ulp(θ) ≤ ~5.6e-4 rad, hence cos/sin can differ
    /// across a 2π shift by up to that amount even though the math is
    /// exactly periodic.
    #[test]
    fn phase_currents_periodic_over_electrical_revolution() {
        let mut state: u32 = 0x1234_5678;
        for _ in 0..100 {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            // ±10 mm: (state as i32) % 100_000 gives [-99999, 99999],
            // × 1e-4 yields [-10, 10] mm.
            let p_int = (state as i32).wrapping_rem(100_000);
            let p = p_int as f32 * 1e-4;
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let s = 40.0_f32 + (state & 0xFF) as f32; // ≈ [40, 296)
            let one_rev = 4.0 / s;
            let (a0, b0) = phase_currents(p, s);
            let (a1, b1) = phase_currents(p + one_rev, s);
            assert!(
                (a0 - a1).abs() <= 1e-3 && (b0 - b1).abs() <= 1e-3,
                "periodicity violated at p={p} s={s}: (a0,b0)=({a0},{b0}) (a1,b1)=({a1},{b1})",
            );
        }
    }

    /// Zero motion: rotor at electrical-zero → full current in coil A,
    /// none in coil B. Exact equality is appropriate here — cos(0) and
    /// sin(0) are exact in f32 and libm both yield (1.0, 0.0).
    #[test]
    fn phase_currents_at_origin() {
        for &s in &[40.0_f32, 80.0, 200.0, 400.0] {
            let (i_a, i_b) = phase_currents(0.0, s);
            assert_eq!(i_a, 1.0, "i_a at origin should be exactly 1.0; got {i_a}");
            assert_eq!(i_b, 0.0, "i_b at origin should be exactly 0.0; got {i_b}");
        }
    }
```

- [ ] **Step 6: Run the new tests and verify they all pass**

```bash
cargo test -p runtime --lib --no-default-features --features host phase::tests::
```

Expected: all four `phase::tests::*` tests pass:
```
test phase::tests::electrical_angle_rad_known_points ... ok
test phase::tests::phase_currents_at_origin ... ok
test phase::tests::phase_currents_matches_f64_reference_over_sweep ... ok
test phase::tests::phase_currents_periodic_over_electrical_revolution ... ok
test phase::tests::phase_currents_unit_circle_invariant ... ok

test result: ok. 5 passed; 0 failed
```

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/phase.rs
git -c commit.gpgsign=false commit -m "feat(runtime/phase): phase_currents + invariant tests"
```

---

## Task 4: `phase_currents_with_invert` — direction polarity

When a stepper's `dir_pin` is configured with `!PIN` polarity (klippy invert flag), the rotor's "positive direction" is reversed. In phase-stepping math, this is implemented by negating the electrical angle before computing cos/sin. Because cos is even and sin is odd, the net effect on the output is `(I_A, I_B) → (I_A, −I_B)`.

**Files:**
- Modify: `rust/runtime/src/phase.rs`

- [ ] **Step 1: Write the failing direction-inversion test**

Append to the `#[cfg(test)] mod tests` block:

```rust
    /// Equivalence: inverting direction at position `p` produces the same
    /// output as not inverting at position `-p`. Both should equal
    /// `(cos θ, sin θ)` for `θ = -electrical_angle_rad(p, s)`.
    ///
    /// Restricted to |p| ≤ 1000 mm (the spec's stated sweep range) so
    /// the f32 angles compared on each side go through identical
    /// rounding and bit-exact agreement is achievable.
    #[test]
    fn phase_currents_with_invert_negates_angle() {
        let mut state: u32 = 0xCAFE_F00D;
        for _ in 0..200 {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            // ±1000 mm: (state as i32) % 10_000_000 × 1e-4 → ±1000.
            let p_int = (state as i32).wrapping_rem(10_000_000);
            let p = p_int as f32 * 1e-4;
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let s = 40.0_f32 + (state & 0xFF) as f32;
            let inverted = phase_currents_with_invert(p, s, true);
            let reflected = phase_currents_with_invert(-p, s, false);
            assert!(
                (inverted.0 - reflected.0).abs() <= 1e-6
                    && (inverted.1 - reflected.1).abs() <= 1e-6,
                "invert/reflect mismatch at p={p} s={s}: inverted={inverted:?} reflected={reflected:?}",
            );
        }
    }

    /// `invert_dir=false` is identical to plain `phase_currents`.
    #[test]
    fn phase_currents_with_invert_false_matches_plain() {
        for &s in &[40.0_f32, 80.0, 200.0, 400.0] {
            for p_int in -10..=10 {
                let p = p_int as f32 * 5.0;
                let plain = phase_currents(p, s);
                let with_flag = phase_currents_with_invert(p, s, false);
                assert_eq!(
                    plain, with_flag,
                    "invert=false should match plain at p={p} s={s}",
                );
            }
        }
    }
```

- [ ] **Step 2: Run the tests and verify they fail**

```bash
cargo test -p runtime --lib --no-default-features --features host phase::tests::phase_currents_with_invert
```

Expected: build error `cannot find function 'phase_currents_with_invert' in this scope`.

- [ ] **Step 3: Implement `phase_currents_with_invert`**

In `rust/runtime/src/phase.rs`, after the existing `phase_currents` function and before the `#[cfg(test)] mod tests` block, add:

```rust
/// Same as `phase_currents` but honors a per-stepper direction-inversion
/// flag — the equivalent of klippy's `dir_pin: !PIN` polarity. When
/// `invert_dir` is `true`, the electrical angle is negated before
/// computing cos/sin; the net effect on the output is
/// `(I_A, I_B) → (I_A, −I_B)` (cos is even, sin is odd).
#[inline]
pub fn phase_currents_with_invert(
    motor_position_mm: f32,
    full_steps_per_mm: f32,
    invert_dir: bool,
) -> (f32, f32) {
    let theta = electrical_angle_rad(motor_position_mm, full_steps_per_mm);
    let theta = if invert_dir { -theta } else { theta };
    (cos_f32(theta), sin_f32(theta))
}
```

- [ ] **Step 4: Run the tests and verify they pass**

```bash
cargo test -p runtime --lib --no-default-features --features host phase::tests::
```

Expected: 7 tests pass. The two new ones:
```
test phase::tests::phase_currents_with_invert_false_matches_plain ... ok
test phase::tests::phase_currents_with_invert_negates_angle ... ok
```

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/phase.rs
git -c commit.gpgsign=false commit -m "feat(runtime/phase): phase_currents_with_invert"
```

---

## Task 5: Continuity-bound regression test

The kernel is Lipschitz-continuous with constant `(π/2) × full_steps_per_mm` (spec §4: `d(cos θ, sin θ)/dp = (π/2) × full_steps_per_mm`). Adjacent samples spaced by `δ` must satisfy `‖Δ(I_A, I_B)‖₂ ≤ (π/2) × full_steps_per_mm × δ × (1 + slack)`. Future LUT or polynomial trig variants are a likely source of accidental discontinuities; this test is the regression guard.

**Files:**
- Modify: `rust/runtime/src/phase.rs`

- [ ] **Step 1: Add the continuity-bound test**

Append to the `#[cfg(test)] mod tests` block:

```rust
    /// Lipschitz bound: for δ = 1 µm, the output norm-change must stay
    /// under (π/2) × full_steps_per_mm × δ × (1 + slack) + noise_floor.
    /// Passes for the libm-based implementation; serves as a regression
    /// guard for future LUT or polynomial trig variants.
    ///
    /// Restricted to |p| ≤ 100 mm so ulp(p) ≪ δ; otherwise `p + δ`
    /// rounds to `p` in f32 and the test becomes vacuous (passes
    /// trivially because both calls return the same currents). The
    /// noise floor (1e-6) absorbs libm's ≤ 1-ULP precision floor at
    /// small s, where the Lipschitz term itself is small enough to be
    /// comparable to f32 rounding noise.
    #[test]
    fn phase_currents_continuity_bound() {
        let delta = 1e-3_f32; // 1 µm in mm
        let mut state: u32 = 0xFEED_BABE;
        for _ in 0..100 {
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            // ±100 mm: (state as i32) % 1_000_000 × 1e-4 → ±100.
            let p_int = (state as i32).wrapping_rem(1_000_000);
            let p = p_int as f32 * 1e-4;
            state = state.wrapping_mul(1_103_515_245).wrapping_add(12_345);
            let s = 40.0_f32 + (state & 0xFF) as f32;
            let (a0, b0) = phase_currents(p, s);
            let (a1, b1) = phase_currents(p + delta, s);
            let da = a1 - a0;
            let db = b1 - b0;
            let norm = (da * da + db * db).sqrt();
            let lipschitz = core::f32::consts::FRAC_PI_2 * s * delta;
            let bound = lipschitz * 1.001 + 1e-6; // slack + f32 noise floor
            assert!(
                norm <= bound,
                "continuity bound violated at p={p} s={s}: norm={norm}, lipschitz={lipschitz}, bound={bound}",
            );
        }
    }
```

- [ ] **Step 2: Run the test and verify it passes**

```bash
cargo test -p runtime --lib --no-default-features --features host phase::tests::phase_currents_continuity_bound
```

Expected: `test phase::tests::phase_currents_continuity_bound ... ok`.

- [ ] **Step 3: Commit**

```bash
git add rust/runtime/src/phase.rs
git -c commit.gpgsign=false commit -m "test(runtime/phase): continuity-bound regression guard"
```

---

## Task 6: Final verification across all build configurations and clippy

The runtime crate has strict crate-level lints (`#![deny(clippy::panic, clippy::unwrap_used, ...)]` in `lib.rs:5-16`). Confirm `phase.rs` is clean under those plus the workspace lints, and that all three feature configurations still build.

**Files:**
- (No file changes expected. If clippy flags anything, fix it inline.)

- [ ] **Step 1: Run all `phase` tests with default features**

```bash
cargo test -p runtime --lib phase::tests::
```

Expected: 8 tests pass.

```
test phase::tests::electrical_angle_rad_known_points ... ok
test phase::tests::phase_currents_at_origin ... ok
test phase::tests::phase_currents_continuity_bound ... ok
test phase::tests::phase_currents_matches_f64_reference_over_sweep ... ok
test phase::tests::phase_currents_periodic_over_electrical_revolution ... ok
test phase::tests::phase_currents_unit_circle_invariant ... ok
test phase::tests::phase_currents_with_invert_false_matches_plain ... ok
test phase::tests::phase_currents_with_invert_negates_angle ... ok

test result: ok. 8 passed
```

- [ ] **Step 2: Run the full runtime unit-test suite to confirm no regression**

```bash
cargo test -p runtime --lib
```

Expected: all existing tests pass alongside the 8 new ones. There should be no test failures or new warnings.

- [ ] **Step 3: Run clippy with the workspace lints**

```bash
cargo clippy -p runtime --no-default-features --features host --lib -- -D warnings
```

Expected: no warnings, no errors. If clippy flags anything in `phase.rs`, fix inline:
- A common one: `clippy::float_cmp` on the exact `assert_eq!(i_a, 1.0, ...)` in `phase_currents_at_origin`. If so, add `#[allow(clippy::float_cmp)]` to that specific test function, with a comment noting that `cos(0)` and `sin(0)` are bit-exact in f32.

- [ ] **Step 4: Confirm MCU feature builds**

```bash
cargo build -p runtime --no-default-features --features mcu-h7 --lib
cargo build -p runtime --no-default-features --features mcu-f4 --lib
```

Expected for both: clean build, no warnings. These exercise the `libm::cosf` / `libm::sinf` branches of the cfg-splits.

- [ ] **Step 5: If any fixes were applied in steps 3 or 4, commit them**

```bash
git status
# If rust/runtime/src/phase.rs shows changes:
git add rust/runtime/src/phase.rs
git -c commit.gpgsign=false commit -m "fix(runtime/phase): clippy/MCU-build cleanups"
```

If nothing was modified, skip the commit step.

- [ ] **Step 6: Verify the final state of touched files**

```bash
git log --oneline -10 -- rust/runtime/Cargo.toml rust/runtime/src/lib.rs rust/runtime/src/phase.rs
```

Expected: the new commits from tasks 1, 2, 3, 4, 5 (and 6 if clippy fixes were needed) appear in order.

```bash
git diff main -- rust/runtime/src/phase.rs | head -10
```

Expected: a `+` diff with `phase.rs` content, starting at line 1 (the file is new in this branch).

---

## Spec coverage check

Re-checking the plan against the spec ([`docs/superpowers/specs/2026-05-12-phase-stepping-math-kernel-design.md`](../specs/2026-05-12-phase-stepping-math-kernel-design.md)):

| Spec requirement | Implemented by |
|---|---|
| New file `rust/runtime/src/phase.rs` | Task 1 step 2 |
| `pub mod phase;` in `lib.rs` | Task 1 step 3 |
| `libm` dep in Cargo.toml gated on `mcu-h7`/`mcu-f4` | Task 1 step 1 |
| `fn electrical_angle_rad(...)` | Task 2 |
| `fn phase_currents(...)` | Task 3 |
| `fn phase_currents_with_invert(...)` | Task 4 |
| libm-based, host falls back to `f32::cos`/`f32::sin` via std | Task 3 step 3 (cfg-split helpers) |
| Test: closed-form correctness sweep | Task 3 step 1 |
| Test: periodicity | Task 3 step 5 |
| Test: unit-circle invariant | Task 3 step 5 |
| Test: direction inversion equivalence | Task 4 step 1 |
| Test: continuity bound | Task 5 step 1 |
| Test: zero motion | Task 3 step 5 |
| No engine / state / wire-protocol / klippy / C touches | Honored — only Cargo.toml, lib.rs, phase.rs |
| Module exists as dead code (importable, unused) | No `use phase::*;` anywhere; nothing imports the module |
