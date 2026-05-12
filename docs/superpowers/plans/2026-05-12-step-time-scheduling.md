# Step-time scheduling for non-phase-stepped axes — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the all-axes-polled-at-40kHz architecture with per-stepper `StepMode`. Non-phase-stepped axes (the only kind we ship today) move from TIM5-ISR polling to Klipper `struct timer` event scheduling, computing each step's exact firing time via Newton iteration on the cubic position polynomial. F446 stops wedging; H7 unchanged.

**Architecture:** Per-stepper `AtomicU8` `StepMode` (`Modulated` | `StepTime`) lives in `SharedState`. `StepTime` steppers get a Klipper `struct timer` per stepper; the timer ISR fires step pulses, samples endstops, asks the engine for the next step time, reschedules. TIM5 only enables when at least one `Modulated` stepper exists (so F4 never enables it). `StepMode` is runtime-mutable to support future sensorless homing on phase-stepped axes (TMC StallGuard requires the driver's internal sequencer, which direct/phase-stepping mode bypasses).

**Tech Stack:** Rust (no_std for runtime, std for tests), C (Klipper MCU build, gcc-arm-none-eabi), Python (klippy host), Klipper sched.h `struct timer`.

**Spec:** [`docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md`](../specs/2026-05-12-step-time-scheduling-design.md)

**Test infrastructure:**
- Rust unit tests: `cargo test -p runtime --features std` (runs on host)
- Rust sim integration: `cargo test -p runtime --features std --test sim_*` (Linux build of MCU runtime)
- Klippy: `python3 -m pytest klippy/test/` (or whatever the project uses; check `Makefile` if unsure)
- Bench: `dderg@trident.local` SSH; build via `make`, flash F4 via `make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32f446xx_*`

**Conventions to honor:**
- No `Co-Authored-By: Claude` in commit messages (user instruction)
- `cargo clean` between H7 and F4 builds (memory: `cargo_clean_between_mcus.md`)
- `make clean` between H7 and F4 (memory: `always_make_clean.md`)
- Never issue G-code without explicit user approval (memory: `no_gcode_without_permission.md`) — applies to bench-test tasks

---

## File Structure

**Rust runtime crate** (`rust/runtime/src/`)
- `state.rs` — add `step_modes: [AtomicU8; MAX_STEPPER_OIDS]` to `SharedState`
- `step_time.rs` *(new)* — Newton-based `compute_next_step_time` helper + `StepMode` enum
- `engine.rs` — minor: add `arm_step_timer` method that calls into `step_time::compute_next_step_time` for a given stepper

**Rust FFI** (`rust/kalico-c-api/src/runtime_ffi.rs`)
- Add three exports: `kalico_runtime_set_step_mode`, `kalico_runtime_compute_next_step_time`, `kalico_runtime_arm_step_timer`

**Wire protocol** (`rust/kalico-protocol/`)
- Extend `configure_axes_blob` payload with N×u8 `step_mode` array (N = stepper count, already in the blob)

**MCU C** (`src/`)
- `runtime_tick.c` — per-stepper `struct timer` array; `step_time_event` ISR; allocate/cancel on segment lifecycle
- `runtime_endstop.c` (or wherever `runtime_endstop_sample_pins` lives) — add `runtime_endstop_sample_one(stepper_idx)`
- `stm32/runtime_tick_h7.c`, `stm32/runtime_tick_f4.c` — `runtime_tick_enable`/`disable` become conditional on `n_modulated_steppers > 0`

**Klippy host** (`klippy/`)
- `extras/stepper.py` (or `klippy/stepper.py` — verify with grep) — parse `phase_stepping: 0|1` from per-stepper config
- `motion_bridge.py` (or wherever `configure_axes_blob` is assembled host-side) — emit `step_mode` array; reject `phase_stepping: 1` if MCU caps lack `PHASE_STEPPING_BIT`

**Tests** (`rust/runtime/tests/`, new files)
- `step_time_newton.rs` *(new)* — unit tests for the Newton algorithm
- `step_time_capability.rs` *(new)* — capability ceiling tests
- `sim_steptime_z_jog.rs` *(new)* — sim integration: F4-config Z jog
- `sim_steptime_mode_flip.rs` *(new)* — sim integration: runtime mode flip

---

## Phase A — Engine core (Rust, TDD)

### Task A1: Add `StepMode` enum + per-stepper field

**Files:**
- Modify: `rust/runtime/src/state.rs`
- Modify: `rust/runtime/src/lib.rs`
- Test: `rust/runtime/tests/step_time_basic.rs` *(new)*

- [ ] **Step 1: Write the failing test**

Create `rust/runtime/tests/step_time_basic.rs`:

```rust
//! Basic StepMode enum + per-stepper field tests.

use runtime::state::{StepMode, SharedState, MAX_STEPPER_OIDS};
use core::sync::atomic::Ordering;

#[test]
fn default_step_mode_is_step_time() {
    let shared = SharedState::new();
    for i in 0..MAX_STEPPER_OIDS {
        let raw = shared.step_modes[i].load(Ordering::Acquire);
        assert_eq!(
            StepMode::from_u8(raw),
            Some(StepMode::StepTime),
            "stepper {} default should be StepTime",
            i,
        );
    }
}

#[test]
fn step_mode_roundtrip_via_atomic() {
    let shared = SharedState::new();
    shared.step_modes[0].store(StepMode::Modulated as u8, Ordering::Release);
    let raw = shared.step_modes[0].load(Ordering::Acquire);
    assert_eq!(StepMode::from_u8(raw), Some(StepMode::Modulated));
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd rust && cargo test -p runtime --features std --test step_time_basic`
Expected: compile error — `StepMode` not in scope, `step_modes` field not in `SharedState`.

- [ ] **Step 3: Add `StepMode` enum to `state.rs`**

Add to `rust/runtime/src/state.rs` (top of file, near other enum-like definitions):

```rust
/// Per-stepper stepping-output strategy. Stored as `AtomicU8` in
/// `SharedState::step_modes`; runtime-mutable via `runtime_set_step_mode`.
///
/// Spec: docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md §3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StepMode {
    /// Driven by TIM5 ISR at the MCU's modulation rate. Current behavior
    /// (polled curve eval + `StepAccumulator`). Future: grows to include
    /// sin/cos commutation per build-order Step 10.
    Modulated = 0,
    /// Driven by per-stepper Klipper `struct timer`. Engine computes each
    /// step's firing time via Newton iteration on the position polynomial.
    /// Default for all steppers; mandatory on MCUs that don't advertise
    /// the `PHASE_STEPPING` capability bit.
    StepTime = 1,
}

impl StepMode {
    pub fn from_u8(raw: u8) -> Option<StepMode> {
        match raw {
            0 => Some(StepMode::Modulated),
            1 => Some(StepMode::StepTime),
            _ => None,
        }
    }
}
```

- [ ] **Step 4: Add `step_modes` field to `SharedState`**

In `rust/runtime/src/state.rs`, find `pub struct SharedState {` (~line 116). Add to the end of the struct (after `stepper_counts`):

```rust
    /// Per-stepper `StepMode` (spec §5). Atomic so the host can flip a
    /// stepper between Modulated and StepTime at runtime (needed for future
    /// sensorless homing on phase-stepped axes — TMC StallGuard requires
    /// the driver's internal sequencer, which the direct/phase-stepping
    /// path bypasses). Default `StepTime` (enum value 1).
    pub step_modes: [AtomicU8; MAX_STEPPER_OIDS],
```

In `impl SharedState`, find `pub const fn new()` and update both `new()` and the trailing default array. Replace the closing of the struct literal `stepper_counts: [...]` block with:

```rust
            stepper_counts: [
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
            ],
            step_modes: [
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
            ],
```

Make sure `AtomicU8` is imported at the top of `state.rs` (check existing imports — if missing, add `use core::sync::atomic::AtomicU8;`).

- [ ] **Step 5: Re-export `StepMode` from `rust/runtime/src/lib.rs`**

Find the existing re-export block in `rust/runtime/src/lib.rs`. Add:

```rust
pub use state::StepMode;
```

- [ ] **Step 6: Run the test to verify it passes**

Run: `cd rust && cargo test -p runtime --features std --test step_time_basic`
Expected: 2 tests pass.

- [ ] **Step 7: Run full test suite to check for regressions**

Run: `cd rust && cargo test -p runtime --features std`
Expected: all existing tests still pass.

- [ ] **Step 8: Commit**

```bash
git add rust/runtime/src/state.rs rust/runtime/src/lib.rs rust/runtime/tests/step_time_basic.rs
git commit -m "feat(runtime): add StepMode enum + per-stepper AtomicU8 field

Default StepTime. Capability check and runtime-mutability come in
following tasks. Per spec §3."
```

---

### Task A2: Newton-based `compute_next_step_time` (pure function)

**Files:**
- Create: `rust/runtime/src/step_time.rs`
- Modify: `rust/runtime/src/lib.rs`
- Test: `rust/runtime/tests/step_time_newton.rs` *(new)*

- [ ] **Step 1: Write the failing tests**

Create `rust/runtime/tests/step_time_newton.rs`:

```rust
//! Newton-based step-time computation tests.
//!
//! Strategy: synthesize a known cubic position polynomial, ask
//! `compute_next_step_time` for the next step's time, verify against the
//! analytic answer (where one exists) or against high-precision iteration.

use runtime::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

/// Helper: trivial linear "curve" — position(t) = velocity * t. Verifies
/// that a constant-velocity initial guess converges in 1 iteration.
fn linear_curve(velocity: f32) -> impl Fn(f32) -> (f32, f32) {
    move |t| (velocity * t, velocity)
}

/// Helper: cubic curve with given coefficients. position(t) = a*t^3 + b*t^2 + c*t.
fn cubic_curve(a: f32, b: f32, c: f32) -> impl Fn(f32) -> (f32, f32) {
    move |t| {
        let pos = a * t * t * t + b * t * t + c * t;
        let vel = 3.0 * a * t * t + 2.0 * b * t + c;
        (pos, vel)
    }
}

#[test]
fn linear_curve_converges_in_one_iteration() {
    // velocity = 1.0 mm/s; step_distance = 0.0025 mm (typical 400 step/mm × 16x microstep)
    // Expected next step at t = 0.0025 (forward direction).
    let eval = linear_curve(1.0);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    match result {
        StepTimeResult::NextAt(t) => {
            assert!((t - 0.0025).abs() < 1e-9, "expected t≈0.0025, got {}", t);
        }
        other => panic!("expected NextAt, got {:?}", other),
    }
}

#[test]
fn linear_curve_reverse_direction() {
    // negative velocity → next step is backward (current_step - 1).
    let eval = linear_curve(-1.0);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    match result {
        StepTimeResult::NextAt(t) => {
            assert!((t - 0.0025).abs() < 1e-9);
        }
        other => panic!("expected NextAt, got {:?}", other),
    }
}

#[test]
fn cubic_curve_converges_within_three_iterations() {
    // position(t) = 0.1*t^3 + 0.5*t^2 + 1.0*t  (mm)
    // At t=0: position=0, velocity=1.0. Look for first step at 0.0025 mm.
    // The cubic adds a small correction to the linear estimate.
    let eval = cubic_curve(0.1, 0.5, 1.0);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    let t = match result {
        StepTimeResult::NextAt(t) => t,
        other => panic!("expected NextAt, got {:?}", other),
    };
    // Verify the returned time actually puts position at the step boundary.
    let (pos, _) = eval(t);
    assert!(
        (pos - 0.0025).abs() < 0.0025 * 1e-5,
        "position at returned t={} is {}, expected 0.0025",
        t,
        pos,
    );
}

#[test]
fn segment_exhaustion_returns_none() {
    // velocity 1.0 mm/s, segment ends at t=0.001 (1 ms). One step = 0.0025 mm
    // can't fit before segment end.
    let eval = linear_curve(1.0);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 0.001,
    };
    let result = compute_next_step_time(&q);
    assert!(
        matches!(result, StepTimeResult::SegmentExhausted),
        "expected SegmentExhausted, got {:?}",
        result,
    );
}

#[test]
fn velocity_near_zero_returns_segment_exhausted() {
    // Velocity essentially zero — segment can't produce steps.
    let eval = linear_curve(1e-10);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 0.0025,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    let result = compute_next_step_time(&q);
    assert!(
        matches!(result, StepTimeResult::SegmentExhausted),
        "expected SegmentExhausted at v≈0, got {:?}",
        result,
    );
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd rust && cargo test -p runtime --features std --test step_time_newton`
Expected: compile error — `step_time` module doesn't exist.

- [ ] **Step 3: Create `rust/runtime/src/step_time.rs`**

```rust
//! Step-time scheduling: compute the next step pulse time for a stepper
//! by Newton-iterating the position polynomial.
//!
//! Spec: docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md §8.

/// Stepper-side step-distance lower bound for the Newton tolerance. If
/// `position - target` is below this fraction of one step, accept.
const NEWTON_TOL_FRACTION: f32 = 1e-6;

/// Newton iteration cap. Quadratic convergence on a cubic from a
/// velocity-based initial guess hits FP precision in ≤3 iterations for
/// well-conditioned cases. Past 3, something is wrong; give up.
const MAX_NEWTON_ITERS: usize = 3;

/// Velocity below this magnitude is treated as "stopped": the segment
/// can't produce another step at a meaningful rate. Returning
/// `SegmentExhausted` defers to the next-segment arming path.
///
/// 1e-6 mm/cycle at a 180 MHz clock is 180 mm/s — well below any real Z
/// velocity. Any lower threshold risks Newton numerical instability.
const EPS_VELOCITY: f32 = 1e-9;

/// Query for `compute_next_step_time`. The `eval` closure must return
/// `(position, velocity)` at the requested time, where position is in
/// the stepper's motor frame (already through kinematics — for a
/// Cartesian Z this is just the axis position).
pub struct StepTimeQuery<'a, F: Fn(f32) -> (f32, f32)> {
    pub eval: &'a F,
    pub step_distance: f32,
    pub current_step: i32,
    /// Time at which to start the search (in segment-relative units; the
    /// engine's mapping from MCU clock to this domain is fixed per
    /// segment).
    pub t_curr: f32,
    /// End of the active segment in the same time domain as `t_curr`.
    pub t_segment_end: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepTimeResult {
    /// The next step fires at time `t` (same domain as `t_curr`).
    NextAt(f32),
    /// The active segment can't produce another step in the current
    /// direction. Engine re-arms on the next pushed segment.
    SegmentExhausted,
}

pub fn compute_next_step_time<F: Fn(f32) -> (f32, f32)>(
    q: &StepTimeQuery<F>,
) -> StepTimeResult {
    let (_pos_curr, v_curr) = (q.eval)(q.t_curr);
    if v_curr.abs() < EPS_VELOCITY {
        return StepTimeResult::SegmentExhausted;
    }
    let dir = if v_curr > 0.0 { 1.0 } else { -1.0 };
    let target = (q.current_step as f32 + dir) * q.step_distance;

    // Initial guess: constant velocity.
    let mut dt = q.step_distance / v_curr.abs();
    let tol = q.step_distance.abs() * NEWTON_TOL_FRACTION;

    for _ in 0..MAX_NEWTON_ITERS {
        let t_try = q.t_curr + dt;
        if t_try > q.t_segment_end || t_try < q.t_curr {
            return StepTimeResult::SegmentExhausted;
        }
        let (pos, vel) = (q.eval)(t_try);
        let err = pos - target;
        if err.abs() < tol {
            return StepTimeResult::NextAt(t_try);
        }
        if vel.abs() < EPS_VELOCITY {
            return StepTimeResult::SegmentExhausted;
        }
        dt -= err / vel;
    }

    // Final iteration didn't hit tolerance — return best estimate IF it's
    // still in-segment and on the correct side of the boundary. (Quadratic
    // convergence means we're typically at FP precision after 2 iters; if
    // we reach 3 without converging, the curve is degenerate. Conservative:
    // return SegmentExhausted.)
    let t_final = q.t_curr + dt;
    if t_final > q.t_segment_end || t_final < q.t_curr {
        return StepTimeResult::SegmentExhausted;
    }
    let (pos, _) = (q.eval)(t_final);
    if (pos - target).abs() < q.step_distance.abs() * 1e-3 {
        // Within 0.1% of step — acceptable, return it.
        StepTimeResult::NextAt(t_final)
    } else {
        StepTimeResult::SegmentExhausted
    }
}
```

- [ ] **Step 4: Add `pub mod step_time;` to `rust/runtime/src/lib.rs`**

Find the existing module declarations near the top of `rust/runtime/src/lib.rs` and add:

```rust
pub mod step_time;
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cd rust && cargo test -p runtime --features std --test step_time_newton`
Expected: 5 tests pass.

- [ ] **Step 6: Run full test suite**

Run: `cd rust && cargo test -p runtime --features std`
Expected: all tests pass.

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/step_time.rs rust/runtime/src/lib.rs rust/runtime/tests/step_time_newton.rs
git commit -m "feat(runtime): Newton-based compute_next_step_time

Pure function operating on a (position, velocity) closure. No engine
state. Quadratic convergence on cubic position polynomial; 3 iteration
cap. Per spec §8."
```

---

### Task A3: `runtime_set_step_mode` + capability ceiling

**Files:**
- Modify: `rust/runtime/src/state.rs`
- Test: `rust/runtime/tests/step_time_capability.rs` *(new)*

- [ ] **Step 1: Write the failing tests**

Create `rust/runtime/tests/step_time_capability.rs`:

```rust
//! Capability-ceiling tests for runtime_set_step_mode.

use runtime::state::{SharedState, StepMode, MAX_STEPPER_OIDS, set_step_mode, SetStepModeError};
use core::sync::atomic::Ordering;

#[test]
fn set_step_mode_with_capability_succeeds() {
    let shared = SharedState::new();
    let result = set_step_mode(&shared, 0, StepMode::Modulated, /* mcu_supports_phase = */ true);
    assert!(result.is_ok());
    assert_eq!(
        StepMode::from_u8(shared.step_modes[0].load(Ordering::Acquire)),
        Some(StepMode::Modulated),
    );
}

#[test]
fn set_step_mode_modulated_without_capability_rejects() {
    let shared = SharedState::new();
    let result = set_step_mode(&shared, 0, StepMode::Modulated, /* mcu_supports_phase = */ false);
    assert_eq!(result, Err(SetStepModeError::CapabilityMissing));
    // State unchanged.
    assert_eq!(
        StepMode::from_u8(shared.step_modes[0].load(Ordering::Acquire)),
        Some(StepMode::StepTime),
    );
}

#[test]
fn set_step_mode_step_time_always_succeeds() {
    let shared = SharedState::new();
    // Even without phase capability, StepTime is fine.
    let result = set_step_mode(&shared, 0, StepMode::StepTime, false);
    assert!(result.is_ok());
}

#[test]
fn set_step_mode_out_of_range_rejects() {
    let shared = SharedState::new();
    let result = set_step_mode(
        &shared,
        MAX_STEPPER_OIDS as u8,
        StepMode::StepTime,
        true,
    );
    assert_eq!(result, Err(SetStepModeError::OutOfRange));
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd rust && cargo test -p runtime --features std --test step_time_capability`
Expected: compile error — `set_step_mode` and `SetStepModeError` don't exist.

- [ ] **Step 3: Add `set_step_mode` + error enum to `state.rs`**

At the bottom of `rust/runtime/src/state.rs`:

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetStepModeError {
    /// Requested `StepMode::Modulated` on an MCU whose capability bitmap
    /// does not advertise `PHASE_STEPPING`. Spec §4.
    CapabilityMissing,
    /// `stepper_idx >= MAX_STEPPER_OIDS`.
    OutOfRange,
}

/// Atomically flip a stepper's `StepMode`. Enforces the capability
/// ceiling: `Modulated` is rejected if the MCU doesn't advertise the
/// phase-stepping bit. Spec §10.
pub fn set_step_mode(
    shared: &SharedState,
    stepper_idx: u8,
    mode: StepMode,
    mcu_supports_phase: bool,
) -> Result<(), SetStepModeError> {
    if (stepper_idx as usize) >= MAX_STEPPER_OIDS {
        return Err(SetStepModeError::OutOfRange);
    }
    if mode == StepMode::Modulated && !mcu_supports_phase {
        return Err(SetStepModeError::CapabilityMissing);
    }
    shared.step_modes[stepper_idx as usize].store(mode as u8, core::sync::atomic::Ordering::Release);
    Ok(())
}
```

- [ ] **Step 4: Re-export from lib.rs**

In `rust/runtime/src/lib.rs`, extend the existing `pub use state::...` block (or add a new line):

```rust
pub use state::{set_step_mode, SetStepModeError, StepMode};
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cd rust && cargo test -p runtime --features std --test step_time_capability`
Expected: 4 tests pass.

- [ ] **Step 6: Run full test suite**

Run: `cd rust && cargo test -p runtime --features std`

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/state.rs rust/runtime/src/lib.rs rust/runtime/tests/step_time_capability.rs
git commit -m "feat(runtime): set_step_mode with capability ceiling

Modulated requires MCU PHASE_STEPPING capability bit; StepTime always
permitted. Per spec §4 + §10."
```

---

### Task A4: Engine integration — `arm_step_timer` reads engine state

**Files:**
- Modify: `rust/runtime/src/engine.rs`
- Modify: `rust/runtime/src/lib.rs`
- Test: `rust/runtime/tests/step_time_engine.rs` *(new)*

This task wires `compute_next_step_time` to the engine's per-stepper curve view. The engine already maintains the active segment and the curve-pool slot per axis; we add a method that builds a `(position, velocity)` closure over the active curve for a given stepper and calls into `step_time::compute_next_step_time`.

- [ ] **Step 1: Read existing engine state plumbing**

Run: `grep -n 'fn scalar_eval_with_derivative\|current segment\|active_segment\|curve_for_stepper' rust/runtime/src/engine.rs | head -10`

You're looking for: (1) the per-tick eval pattern that maps `now → u → CurveView`, (2) the stepper-to-axis mapping. Note line numbers; reuse the same plumbing.

- [ ] **Step 2: Write the failing test**

Create `rust/runtime/tests/step_time_engine.rs`:

```rust
//! Engine-level arm_step_timer integration test. Uses a synthetic segment
//! pushed through the normal queue plumbing to verify that the engine
//! returns a sensible next-step time.

// NOTE: this test uses the same fixture pattern as existing
// `rust/runtime/tests/sim_*.rs` integration tests. Mirror their setup.

use runtime::sim_fixtures::{push_test_segment_linear_z, init_test_runtime};
use runtime::engine::arm_step_timer_for_stepper;

#[test]
fn arm_step_timer_returns_first_step_time_on_linear_z() {
    // Setup: Z curve, velocity 1 mm/s, segment 1 second long, step_distance 0.0025 mm.
    let mut rt = init_test_runtime();
    push_test_segment_linear_z(&mut rt, /*velocity_mm_s=*/1.0, /*duration_s=*/1.0);

    let z_stepper_idx = 2; // X=0, Y=1, Z=2 by convention
    let result = arm_step_timer_for_stepper(&rt, z_stepper_idx, /*now_cycles=*/0);
    // First step at t ≈ 0.0025 s. At 180 MHz that's 450_000 cycles.
    let next = result.expect("expected NextAt");
    assert!(
        (next as i64 - 450_000).abs() < 10,
        "expected ~450_000 cycles for first step, got {}",
        next,
    );
}
```

- [ ] **Step 3: Run the test to verify it fails**

Run: `cd rust && cargo test -p runtime --features std --test step_time_engine`
Expected: compile error or fixture missing — `push_test_segment_linear_z`, `arm_step_timer_for_stepper` not yet defined.

- [ ] **Step 4: Add the fixture helper**

In `rust/runtime/src/sim_fixtures.rs`, find the existing helpers (e.g. `push_test_segment_x_only`). Add a sibling:

```rust
/// Synthesize and enqueue a Z-only segment at constant velocity. Uses a
/// degree-3 Bezier with collinear control points so position(t) is
/// exactly `start + velocity * t` over the segment.
pub fn push_test_segment_linear_z(
    rt: &mut TestRuntime,
    velocity_mm_s: f32,
    duration_s: f32,
) {
    // Use existing degree-3 collinear-cps Bezier construction; mirror the
    // x_only helper but place the curve on the Z axis.
    // ... (mirror existing helper exactly; only the axis index changes)
    todo!("mirror push_test_segment_x_only with axis=Z");
}
```

**NOTE TO EXECUTOR:** the `todo!` is a hand-off marker — read the existing `push_test_segment_x_only` (or similarly-named helper) and adapt it. The fixture pattern is established; don't invent a new one.

- [ ] **Step 5: Add `arm_step_timer_for_stepper` to engine.rs**

Find the existing per-axis eval block in `engine.rs` (around line 752 — search for `scalar_eval_with_derivative` to find the per-tick pattern). Add a public function near it:

```rust
/// Given a runtime context and a stepper index, return the MCU clock
/// cycle at which the next step pulse should fire for that stepper.
///
/// Returns `None` if the active segment for the stepper's axis can't
/// produce another step in the current direction (engine re-arms on the
/// next pushed segment).
///
/// Per spec §5: this is the `runtime_arm_step_timer` engine entry point
/// used by the per-stepper Klipper `struct timer`.
pub fn arm_step_timer_for_stepper(
    ctx: &RuntimeContext,
    stepper_idx: u8,
    now_cycles: u64,
) -> Option<u64> {
    // 1. Identify the axis this stepper belongs to (existing mapping —
    //    locate via grep for `stepper_to_axis` or similar).
    let axis_idx = stepper_axis_for_oid(stepper_idx)?;

    // 2. Locate the active curve-pool slot for this axis. Reuse the same
    //    accessor the per-tick eval uses (see ~line 750 area of engine.rs).
    let curve_view = ctx.curve_pool.active_view_for_axis(axis_idx)?;

    // 3. Read the stepper's current step count (signed; published by ISR).
    let current_step = ctx.shared.stepper_counts[stepper_idx as usize]
        .load(core::sync::atomic::Ordering::Acquire);

    // 4. Read the segment's t_start_cycles (anchor) and duration (segment_end
    //    relative to start). Map `now_cycles` into segment-relative `t_curr`
    //    in the curve's domain (usually 0..1 for normalized NURBS u).
    let segment = curve_view.segment_meta();
    let t_start = segment.t_start_cycles();
    let t_end = segment.t_end_cycles();
    if now_cycles >= t_end {
        return None;
    }
    let t_curr_norm = (now_cycles.saturating_sub(t_start)) as f32
        / (t_end - t_start) as f32;
    let t_end_norm = 1.0_f32;

    // 5. Look up the stepper's step_distance from its config (already
    //    stored at configure_axes time — find via grep for `step_distance`
    //    or `steps_per_mm`).
    let step_distance = stepper_config_step_distance(stepper_idx);

    // 6. Build the eval closure over the curve. Reuse `scalar_eval_with_derivative`.
    let eval = |t_norm: f32| -> (f32, f32) {
        match scalar_eval_with_derivative(&curve_view, t_norm) {
            Ok(pd) => pd,
            Err(_) => (0.0, 0.0),
        }
    };

    let query = crate::step_time::StepTimeQuery {
        eval: &eval,
        step_distance,
        current_step,
        t_curr: t_curr_norm,
        t_segment_end: t_end_norm,
    };
    match crate::step_time::compute_next_step_time(&query) {
        crate::step_time::StepTimeResult::NextAt(t_norm) => {
            let dt_norm = t_norm - t_curr_norm;
            let dt_cycles = (dt_norm * (t_end - t_start) as f32) as u64;
            Some(now_cycles + dt_cycles)
        }
        crate::step_time::StepTimeResult::SegmentExhausted => None,
    }
}
```

**NOTE TO EXECUTOR:** the helper functions `stepper_axis_for_oid`, `stepper_config_step_distance`, and `curve_view.segment_meta()` may not exist with those exact names. Locate the existing equivalents via grep — every one of these is data already used by the per-tick `runtime_handle_tick` flow. The point of this step is to plumb the *same* values into the new function. If a clean accessor doesn't exist, add it (small helper, single-line wrapper around existing struct field reads).

- [ ] **Step 6: Run the test to verify it passes**

Run: `cd rust && cargo test -p runtime --features std --test step_time_engine`
Expected: pass with the computed next-step time within 10 cycles of 450_000.

- [ ] **Step 7: Run full test suite**

Run: `cd rust && cargo test -p runtime --features std`

- [ ] **Step 8: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/src/sim_fixtures.rs rust/runtime/tests/step_time_engine.rs
git commit -m "feat(runtime): arm_step_timer_for_stepper engine entry point

Wires Newton-based compute_next_step_time to the engine's active
per-axis curve. Returns the absolute MCU clock cycle for the next step
pulse, or None on segment exhaustion. Per spec §5."
```

---

## Phase B — FFI exports

### Task B1: Export three new FFI functions

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`

- [ ] **Step 1: Read the existing FFI export pattern**

Run: `grep -n 'pub unsafe extern "C" fn kalico_runtime' rust/kalico-c-api/src/runtime_ffi.rs | head -10`

Pick the simplest existing export (e.g., `kalico_runtime_get_stepper_count`) and use it as the template — signatures, KALICO_OK / KALICO_ERR_* return convention, null-pointer handling.

- [ ] **Step 2: Add `kalico_runtime_set_step_mode`**

Add to `rust/kalico-c-api/src/runtime_ffi.rs` inside the `pub mod exports` block (where other `kalico_runtime_*` exports live):

```rust
/// Flip a stepper's `StepMode` at runtime. Spec §10.
///
/// Returns `KALICO_OK` on success.
/// Returns `KALICO_ERR_INVALID_HANDLE` if `stepper_idx >= MAX_STEPPER_OIDS`.
/// Returns `KALICO_ERR_INVALID_ARG` if `mode > 1`.
/// Returns `KALICO_ERR_CAPABILITY_MISSING` if `mode == Modulated` and the
/// MCU doesn't advertise the PHASE_STEPPING capability bit.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_set_step_mode(
    handle: *mut RuntimeHandle,
    stepper_idx: u8,
    mode: u8,
    mcu_supports_phase: u8,
) -> i32 {
    if handle.is_null() {
        return KALICO_ERR_INVALID_HANDLE;
    }
    let ctx = unsafe { &*((*handle).context_ptr) };
    let mode = match runtime::state::StepMode::from_u8(mode) {
        Some(m) => m,
        None => return KALICO_ERR_INVALID_ARG,
    };
    match runtime::state::set_step_mode(&ctx.shared, stepper_idx, mode, mcu_supports_phase != 0) {
        Ok(()) => KALICO_OK,
        Err(runtime::state::SetStepModeError::CapabilityMissing) => {
            KALICO_ERR_CAPABILITY_MISSING
        }
        Err(runtime::state::SetStepModeError::OutOfRange) => KALICO_ERR_INVALID_HANDLE,
    }
}
```

- [ ] **Step 3: Add `KALICO_ERR_CAPABILITY_MISSING` constant**

Find the existing `KALICO_ERR_*` constants in the file. Add:

```rust
pub const KALICO_ERR_CAPABILITY_MISSING: i32 = -64; // pick the next free negative number; check existing values
```

(Use the next free numeric value — read the existing constants to avoid collision.)

- [ ] **Step 4: Add `kalico_runtime_arm_step_timer`**

```rust
/// Compute the MCU clock cycle for the next step pulse on `stepper_idx`.
/// Writes the result to `*out_cycles_abs` and returns `KALICO_OK`.
///
/// Returns `KALICO_ERR_NO_STEP` if the active segment can't produce
/// another step in the current direction (caller should NOT register the
/// timer; re-arming happens on the next `push_segment`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_arm_step_timer(
    handle: *mut RuntimeHandle,
    stepper_idx: u8,
    now_cycles: u64,
    out_cycles_abs: *mut u64,
) -> i32 {
    if handle.is_null() || out_cycles_abs.is_null() {
        return KALICO_ERR_INVALID_HANDLE;
    }
    let ctx = unsafe { &*((*handle).context_ptr) };
    match runtime::engine::arm_step_timer_for_stepper(ctx, stepper_idx, now_cycles) {
        Some(t) => {
            unsafe { *out_cycles_abs = t };
            KALICO_OK
        }
        None => KALICO_ERR_NO_STEP,
    }
}
```

Add `KALICO_ERR_NO_STEP: i32 = -65;` (or next free number) alongside `KALICO_ERR_CAPABILITY_MISSING`.

- [ ] **Step 5: Add `kalico_runtime_compute_next_step_time`**

Same as `arm_step_timer` but takes an explicit `now_cycles` for the ISR re-arm path (the timer ISR knows its own waketime; passes it in):

```rust
/// Compute the next step's MCU clock cycle after the given anchor time.
/// Used by the per-stepper step-time ISR to chain timer events.
///
/// Identical semantics to `arm_step_timer` — kept as a separate symbol
/// because the caller and call site have different lifecycle (segment
/// load vs ISR chain). Engine implementation may diverge later.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_compute_next_step_time(
    handle: *mut RuntimeHandle,
    stepper_idx: u8,
    now_cycles: u64,
    out_cycles_abs: *mut u64,
) -> i32 {
    // Same body as kalico_runtime_arm_step_timer.
    unsafe {
        kalico_runtime_arm_step_timer(handle, stepper_idx, now_cycles, out_cycles_abs)
    }
}
```

(For now, they share a body. Keep the symbols separate so the C side has the right names — if the engine paths diverge later, the symbol contract is already in place.)

- [ ] **Step 6: Build the FFI crate to verify**

Run: `cd rust && cargo build -p kalico-c-api`
Expected: clean build (no errors).

- [ ] **Step 7: Verify cbindgen produces the new headers**

Run: `cd rust && cargo build -p kalico-c-api --release 2>&1 | grep -i 'cbindgen\|header'`

Check that `rust/kalico-c-api/include/kalico_runtime.h` (or wherever cbindgen writes) now contains the three new function signatures. If cbindgen is configured to auto-run, this should be automatic. If not, manually run cbindgen per the existing project convention.

- [ ] **Step 8: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/
git commit -m "feat(ffi): export set_step_mode, arm_step_timer, compute_next_step_time

C-callable entry points for the per-stepper step-time scheduling path.
Per spec §5."
```

---

## Phase C — Wire protocol (configure_axes_blob)

### Task C1: Extend `configure_axes_blob` payload with per-stepper StepMode array

**Files:**
- Modify: `rust/kalico-protocol/src/...` (find the configure_axes_blob format definition)
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (the `kalico_runtime_configure_axes_blob` handler)
- Test: extend existing `configure_axes_blob` tests

- [ ] **Step 1: Locate the existing blob format**

Run: `grep -rn 'configure_axes_blob\|ConfigureAxesBlob' rust/kalico-protocol/src/ rust/runtime/src/`

Note the version byte (if any) and the existing field layout. We need to add an N-byte step_mode array somewhere — typically appended at the end, behind a length byte, for forward compatibility.

- [ ] **Step 2: Add `step_mode` array field to the blob**

Modify the blob parser to read N bytes (where N = stepper_count, already a field in the blob) after the existing payload. If the blob has a version field, bump it. If not, append at the end and treat older firmware/older host as "all StepTime" by default.

```rust
// In the blob parsing function (find via grep):
let stepper_count = read_u8(&mut cursor)?;
// ... existing fields ...
let mut step_modes = [StepMode::StepTime; MAX_STEPPER_OIDS];
for i in 0..(stepper_count as usize).min(MAX_STEPPER_OIDS) {
    let raw = read_u8(&mut cursor).unwrap_or(StepMode::StepTime as u8);
    step_modes[i] = StepMode::from_u8(raw).unwrap_or(StepMode::StepTime);
}
```

The exact field name and parsing style depend on the existing code; mirror the style at the call site.

- [ ] **Step 3: Wire the parsed `step_modes` into `SharedState`**

In `kalico_runtime_configure_axes_blob` in `runtime_ffi.rs`, after the blob is parsed, copy the step modes into `ctx.shared.step_modes`:

```rust
for i in 0..MAX_STEPPER_OIDS {
    ctx.shared.step_modes[i].store(
        parsed_blob.step_modes[i] as u8,
        core::sync::atomic::Ordering::Release,
    );
}
```

- [ ] **Step 4: Test**

Write a test that constructs a blob with an explicit `step_modes` array, runs it through `kalico_runtime_configure_axes_blob`, and verifies `shared.step_modes` was set.

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(protocol): configure_axes_blob carries per-stepper StepMode

Appended N×u8 step_mode array. Older host emitting the legacy format
falls back to StepTime everywhere (safe default). Per spec §4."
```

---

## Phase D — MCU C integration

### Task D1: Per-stepper `struct timer` allocation + `step_time_event` ISR

**Files:**
- Modify: `src/runtime_tick.c`

- [ ] **Step 1: Add per-stepper timer context array**

Add near the top of `src/runtime_tick.c` (after existing static state declarations):

```c
/// One per stepper. The Klipper scheduler's `struct timer` self-reschedules
/// from the ISR `step_time_event` callback. Allocated at configure_axes
/// time. Inactive steppers' timers stay un-registered.
///
/// MAX_STEPPER_OIDS mirrors the Rust constant (see rust/runtime/src/state.rs).
struct step_timer_ctx {
    struct timer timer;
    uint8_t stepper_idx;
    struct gpio_out step_pin;       // pre-resolved by configure_axes
    uint8_t enabled;                 // 1 = registered with scheduler
};

#define MAX_STEPPER_OIDS_C 8
static struct step_timer_ctx step_timers[MAX_STEPPER_OIDS_C];
```

- [ ] **Step 2: Write the `step_time_event` ISR**

```c
static uint_fast8_t
step_time_event(struct timer *t)
{
    struct step_timer_ctx *ctx = container_of(t, struct step_timer_ctx, timer);

    // Fire the step pulse. The minimum-pulse-width discipline (existing
    // stepper.c invariant) is handled by the GPIO toggle: rising edge here,
    // falling edge sufficiently delayed by Klipper's existing per-stepper
    // minimum-pulse logic. If the pulse needs an explicit chained
    // falling-edge event, mirror that here too.
    gpio_out_toggle_noirq(ctx->step_pin);

    // Sample endstops armed on this stepper's axis (per spec §7).
    runtime_endstop_sample_one(ctx->stepper_idx);

    // Ask the engine for the next step time.
    uint64_t next_cycles_abs = 0;
    int32_t err = kalico_runtime_compute_next_step_time(
        runtime_handle, ctx->stepper_idx, (uint64_t)t->waketime, &next_cycles_abs);

    if (err != KALICO_OK) {
        // Segment exhausted or other no-step condition. Engine will re-arm
        // this timer when the next segment arrives via push_segment.
        ctx->enabled = 0;
        return SF_DONE;
    }

    // Klipper timer is u32 cycles; truncate. The engine returns an
    // absolute u64 cycle count consistent with runtime_widened_host_clock,
    // so the low u32 is exactly the value the scheduler expects.
    t->waketime = (uint32_t)next_cycles_abs;
    return SF_RESCHEDULE;
}
```

- [ ] **Step 3: Initialize the timers at `configure_axes` time**

Find the existing `kalico_dispatch_frame` / `configure_axes_blob` handler in C. After it has applied the blob (and the Rust side has the per-stepper config including `StepMode`), populate `step_timers[]`:

```c
// Called from configure_axes handler after Rust has stored step_modes.
static void
init_step_time_timers(void)
{
    uint32_t now = timer_read_time();
    for (uint8_t i = 0; i < MAX_STEPPER_OIDS_C; i++) {
        step_timers[i].timer.func = step_time_event;
        step_timers[i].stepper_idx = i;
        // step_pin populated from existing stepper config lookup;
        // mirror the existing stepper.c lookup pattern.
        step_timers[i].step_pin = stepper_lookup_step_pin(i);
        step_timers[i].enabled = 0;
    }
}
```

- [ ] **Step 4: Arm a step-time timer on segment load**

Find the existing `runtime_handle_push_segment` callsite (where segments are accepted). After the segment is in the queue and the engine accepts it, arm StepTime steppers:

```c
static void
arm_step_time_steppers_after_push(uint8_t stepper_count)
{
    uint32_t now = timer_read_time();
    for (uint8_t i = 0; i < stepper_count && i < MAX_STEPPER_OIDS_C; i++) {
        // Read the stepper's StepMode (atomic in Rust; expose via a small
        // FFI accessor `kalico_runtime_get_step_mode(handle, idx) -> u8`).
        uint8_t mode = kalico_runtime_get_step_mode(runtime_handle, i);
        if (mode != 1 /*StepTime*/) continue;
        if (step_timers[i].enabled) continue;  // already running

        uint64_t first_cycles_abs = 0;
        int32_t err = kalico_runtime_arm_step_timer(
            runtime_handle, i, (uint64_t)now, &first_cycles_abs);
        if (err != KALICO_OK) continue;  // segment can't produce step for this stepper

        step_timers[i].timer.waketime = (uint32_t)first_cycles_abs;
        step_timers[i].enabled = 1;
        sched_add_timer(&step_timers[i].timer);
    }
}
```

(`kalico_runtime_get_step_mode` is a one-line FFI getter; add it next to the setter in `runtime_ffi.rs`.)

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c rust/kalico-c-api/src/runtime_ffi.rs
git commit -m "feat(mcu): per-stepper step_time_event ISR + arm-on-push

Each StepTime stepper gets its own Klipper struct timer. ISR fires
step pulse, samples endstops, asks engine for next step time, reschedules.
Engine re-arms on the next pushed segment. Per spec §6."
```

---

### Task D2: TIM5 conditional enable

**Files:**
- Modify: `src/stm32/runtime_tick_h7.c`, `src/stm32/runtime_tick_f4.c`

- [ ] **Step 1: Wrap the body of `runtime_tick_enable` with a Modulated-count check**

In each `runtime_tick_*.c`, change `runtime_tick_enable` to:

```c
__attribute__((used, externally_visible))
void
runtime_tick_enable(void)
{
    // If no stepper is in Modulated mode, TIM5 has no work — leave it
    // disabled. F4 (no PHASE capability) will always hit this path; H7
    // with all-StepTime config hits it too. Per spec §6.3.
    if (kalico_runtime_count_modulated_steppers(runtime_handle) == 0) {
        return;
    }
    // ... existing body (engine widen seed, TIM5->SR clear, CEN, IRQ enable)
}
```

- [ ] **Step 2: Add `kalico_runtime_count_modulated_steppers` FFI export**

In `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_count_modulated_steppers(
    handle: *mut RuntimeHandle,
) -> u8 {
    if handle.is_null() { return 0; }
    let ctx = unsafe { &*((*handle).context_ptr) };
    let mut count = 0u8;
    for i in 0..runtime::state::MAX_STEPPER_OIDS {
        if ctx.shared.step_modes[i].load(core::sync::atomic::Ordering::Acquire)
            == runtime::state::StepMode::Modulated as u8
        {
            count += 1;
        }
    }
    count
}
```

- [ ] **Step 3: Symmetric `runtime_tick_disable` on count→0 transition**

If a runtime `set_step_mode` flips the last Modulated stepper to StepTime, we should disable TIM5. Add a hook: after every `kalico_runtime_set_step_mode` call, check the count and call `runtime_tick_enable()` (idempotent on Modulated≥1) or `runtime_tick_disable()`.

```rust
// In kalico_runtime_set_step_mode, after the successful store:
let modulated_count = /* same loop as count_modulated_steppers */;
if modulated_count == 0 {
    unsafe extern "C" { fn runtime_tick_disable(); }
    unsafe { runtime_tick_disable() };
} else {
    unsafe extern "C" { fn runtime_tick_enable(); }
    unsafe { runtime_tick_enable() };
}
```

- [ ] **Step 4: Commit**

```bash
git commit -am "feat(mcu): TIM5 enable conditional on Modulated stepper count

F4 (no PHASE capability) never enables TIM5. H7 in all-StepTime
config also leaves it idle. set_step_mode atomically arms/disarms
TIM5 to match the current count. Per spec §6.3."
```

---

### Task D3: Endstop sampling migration

**Files:**
- Modify: `src/runtime_endstop.c` (or wherever `runtime_endstop_sample_pins` lives — find via grep)

- [ ] **Step 1: Add `runtime_endstop_sample_one(stepper_idx)`**

Read the existing `runtime_endstop_sample_pins()`. Refactor it so the per-stepper sampling work is in a helper:

```c
static inline void
sample_endstops_for_stepper(uint8_t stepper_idx)
{
    // existing per-stepper logic from runtime_endstop_sample_pins, extracted
}

void
runtime_endstop_sample_pins(void)
{
    for (uint8_t i = 0; i < MAX_STEPPER_OIDS_C; i++) {
        sample_endstops_for_stepper(i);
    }
}

__attribute__((used, externally_visible))
void
runtime_endstop_sample_one(uint8_t stepper_idx)
{
    if (stepper_idx >= MAX_STEPPER_OIDS_C) return;
    sample_endstops_for_stepper(stepper_idx);
}
```

This keeps the H7-Modulated path identical (TIM5 ISR still calls `runtime_endstop_sample_pins()`) while letting the step-time ISR call `runtime_endstop_sample_one(ctx->stepper_idx)` for its own axis.

- [ ] **Step 2: Commit**

```bash
git commit -am "feat(endstop): runtime_endstop_sample_one for step-time ISR use

Refactor: per-stepper logic factored out of runtime_endstop_sample_pins
into sample_endstops_for_stepper; new public runtime_endstop_sample_one
exposes it to the step-time ISR. TIM5 path unchanged. Per spec §7."
```

---

## Phase E — Klippy host: capability check + config plumbing

### Task E1: Parse `phase_stepping: 0|1` per stepper

**Files:**
- Modify: `klippy/stepper.py` (or `klippy/extras/stepper.py` — verify with `find klippy -name 'stepper.py'`)
- Test: `klippy/test/test_phase_stepping_config.py` *(new, if klippy has a test framework — otherwise inline assertion in a smoke test)*

- [ ] **Step 1: Locate the existing stepper config-parse code**

Run: `grep -rn 'getfloat.*step_distance\|getint.*microsteps' klippy/ | head -10`

Find where individual stepper config keys are read. Add a new read for `phase_stepping`:

```python
# In the relevant stepper init function:
self.phase_stepping = config.getboolean('phase_stepping', False)
```

- [ ] **Step 2: Pipe it through to motion_bridge**

In `klippy/motion_bridge.py` (or wherever the `configure_axes_blob` is assembled), include the per-stepper `phase_stepping` flag in the emitted blob payload as the appended `step_mode` array (0 = Modulated, 1 = StepTime — invert the user-facing flag).

- [ ] **Step 3: Capability check**

When `phase_stepping=True` is requested, check the MCU's `caps.capabilities & PHASE_STEPPING_BIT`. If unset, raise a clear config error:

```python
if stepper_config.phase_stepping:
    if not (mcu_caps & PHASE_STEPPING_BIT):
        raise config.error(
            f"Stepper '{stepper_name}' requests phase_stepping=1, but its "
            f"MCU does not advertise the PHASE_STEPPING capability. "
            f"Phase stepping requires a sufficiently fast MCU (STM32H7 family); "
            f"the STM32F4 family is not supported."
        )
```

- [ ] **Step 4: Commit**

```bash
git commit -am "feat(klippy): per-stepper phase_stepping config + capability check

Reads 'phase_stepping' boolean from each [stepper_*] section. Default
false. Rejected at config time if the MCU lacks the PHASE_STEPPING
capability bit. Per spec §4."
```

---

## Phase F — Sim integration tests + bench verification

### Task F1: Sim test — F4 config Z jog produces correct step pulses

**Files:**
- Create: `rust/runtime/tests/sim_steptime_z_jog.rs`

- [ ] **Step 1: Mirror an existing sim test**

Run: `ls rust/runtime/tests/sim_*.rs`

Pick the simplest existing sim test (e.g. `sim_motion.rs`) as a template. The pattern: spin up the Linux build of the MCU runtime, push synthetic segments, observe step counts and timing.

- [ ] **Step 2: Write the test**

```rust
//! F4-equivalent config: all steppers in StepTime mode, no TIM5 enable.
//! Push a Z curve, verify step pulses fire at the times the engine
//! computed (within ±1 cycle of compute_next_step_time's output).

#[test]
fn z_jog_steps_fire_at_expected_times() {
    // 1. Init runtime with all steppers in StepTime mode (no phase capability).
    let mut sim = sim_fixtures::init_with_caps(/*phase=*/false);

    // 2. Push a linear Z segment: 1 mm/s for 1 second.
    sim.push_segment_linear_z(/*velocity=*/1.0, /*duration=*/1.0);

    // 3. Run the sim scheduler for the segment's duration.
    let step_times: Vec<u64> = sim.collect_step_times_for_stepper(/*Z=*/2);

    // 4. Expected: 400 step/mm × 1 mm = 400 steps over 1 second.
    //    At 180 MHz, that's a step every 450_000 cycles.
    assert_eq!(step_times.len(), 400);
    for (i, &t) in step_times.iter().enumerate() {
        let expected = 450_000u64 * (i as u64 + 1);
        assert!(
            (t as i64 - expected as i64).abs() < 10,
            "step {} fired at {}, expected {}",
            i, t, expected,
        );
    }

    // 5. Verify TIM5 was never enabled (the sim runtime tracks this).
    assert_eq!(sim.tim5_enable_count(), 0, "TIM5 should not enable in all-StepTime config");
}
```

- [ ] **Step 3: Add sim-fixture helpers as needed**

`sim_fixtures::init_with_caps`, `push_segment_linear_z`, `collect_step_times_for_stepper`, `tim5_enable_count` — extend the existing fixtures to expose these. Most existing fixtures already have segment-push helpers; the others are simple counters.

- [ ] **Step 4: Run the test**

Run: `cd rust && cargo test -p runtime --features std --test sim_steptime_z_jog`

- [ ] **Step 5: Commit**

```bash
git commit -am "test(sim): F4-config Z jog with step-time scheduling

400 steps over 1 second, ±10 cycle tolerance against engine's
computed times. Verifies TIM5 never enables in all-StepTime config.
Per spec §11."
```

---

### Task F2: Sim test — runtime mode flip mid-segment

**Files:**
- Create: `rust/runtime/tests/sim_steptime_mode_flip.rs`

- [ ] **Step 1: Write the test**

```rust
//! Mid-segment StepMode flip. Starts in Modulated, flips to StepTime
//! halfway through a segment, verifies the rest of the steps fire via
//! the step-time ISR.

#[test]
fn mode_flip_mid_segment_continues_step_output() {
    let mut sim = sim_fixtures::init_with_caps(/*phase=*/true);
    sim.set_step_mode(/*stepper=*/2, StepMode::Modulated).unwrap();

    sim.push_segment_linear_z(/*velocity=*/1.0, /*duration=*/1.0);

    // Run for 0.5 seconds.
    sim.run_for_cycles(180_000_000 / 2);

    // Confirm steps fired so far via Modulated (TIM5) path.
    let count_before = sim.stepper_count(2);
    assert!(count_before >= 195 && count_before <= 205, "~200 steps at 0.5s");

    // Flip to StepTime.
    sim.set_step_mode(2, StepMode::StepTime).unwrap();

    // Run remaining 0.5 seconds.
    sim.run_for_cycles(180_000_000 / 2);

    let count_after = sim.stepper_count(2);
    assert_eq!(count_after, 400, "total step count should reach 400");
    // No double-counting at the seam.
    assert!(count_after - count_before >= 195 && count_after - count_before <= 205);
}
```

- [ ] **Step 2: Run + commit**

```bash
git commit -am "test(sim): mid-segment StepMode flip preserves step count

Starts Modulated, flips to StepTime at t=0.5s, verifies remaining
steps fire via step-time ISR without double-counting at the seam.
Per spec §10 + §11."
```

---

### Task F3: Bench verification on Trident F4

**Files:** none (verification only; user-driven motion commands)

- [ ] **Step 1: Build F4 firmware**

```bash
ssh dderg@trident.local "cd ~/klipper && sudo systemctl stop klipper && \
    cp .config.f446.test .config && make olddefconfig && \
    cd rust && cargo clean && cd .. && make clean && make -j4 2>&1 | tail -5"
```

Expected: clean build, no warnings about persistent_diag layout.

- [ ] **Step 2: Verify build layout**

```bash
ssh dderg@trident.local "cd ~/klipper && arm-none-eabi-objdump -t out/klipper.elf | grep -E 'RT_CELL|step_timers|_persistent_diag_end'"
```

Expected: `RT_CELL` at `0x20xxxxxx` (F4 .bss), `step_timers` array allocated.

- [ ] **Step 3: Flash F4**

```bash
ssh dderg@trident.local "cd ~/klipper && make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32f446xx_2C0036000851313133353932-if00 2>&1 | tail -3"
```

Expected: `Verification Complete`.

- [ ] **Step 4: Build + flash H7 (regression check)**

```bash
ssh dderg@trident.local "cd ~/klipper && cp .config.h7.bak .config && make olddefconfig && \
    cd rust && cargo clean && cd .. && make clean && make -j4 2>&1 | tail -3 && \
    make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32h723xx_490017000851323235363233-if00 2>&1 | tail -3"
```

- [ ] **Step 5: Restart klippy + verify ready**

```bash
ssh dderg@trident.local "sudo systemctl start klipper && sleep 10 && \
    curl -s http://localhost:7125/printer/info | python3 -c 'import sys,json; print(json.load(sys.stdin)[\"result\"][\"state\"])'"
```

Expected: `ready`

- [ ] **Step 6: Hand off to user for motion testing**

**STOP. Do not issue G-code yourself.** Tell the user the firmware is ready; the bench acceptance criteria (§11.3 of the spec) require:

1. 10 sequential Z jogs without F4 reset → user issues these
2. `out_max_gap < 50 ms` on F4 prior_diag after the 10 jogs
3. Full G28 homing cycle + 5 Z hops, no IWDG fire
4. H7 prior_diag unchanged vs pre-deploy baseline

After the user reports back, grep klippy.log for `bottom.*prior_diag_tasks` and verify the criteria.

- [ ] **Step 7: Commit (if any bench-driven fixes were needed)**

If everything passed, no commit needed. If a bench-driven fix landed during this task, commit with `fix(...)` prefix and a description of what the bench data showed.

---

## Final pass

- [ ] **Run the full Rust test suite end-to-end**

```bash
cd rust && cargo test --features std --workspace
```

Expected: all tests pass.

- [ ] **Spec coverage check**

Verify by grep that the plan touched every numbered spec section that requires code:
- §4 Configuration — Task E1
- §5 Engine API — Tasks A2, A3, A4, B1
- §6 MCU integration — Tasks D1, D2
- §7 Endstop sampling — Task D3
- §8 Step-time computation — Task A2
- §9 Segment lifecycle — Tasks D1, A4
- §10 Runtime mutability — Tasks A3, B1, F2
- §11 Testing — Tasks A1, A2, A3, A4, F1, F2, F3
- §12 Files touched — all phases

- [ ] **Final commit on completion**

```bash
git commit --allow-empty -m "feat(runtime): step-time scheduling refactor complete

Closes spec docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md.
F446 wedge fixed; H7 regression verified; per-stepper StepMode runtime-mutable
ready for future StallGuard-during-homing-on-phase-stepped-axis support."
```
