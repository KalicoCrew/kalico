# Step 7-B: Layer 4 MCU Evaluator Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the MCU runtime to evaluate per-axis scalar NURBS at 40 kHz, integrate E-follows-XY extrusion, and generate step pulses for hybrid stepping.

**Architecture:** Bottom-up: curve pool (scalar slots) → segment struct (4 handles + E-mode) → engine evaluator (per-axis scalar eval + E dispatch) → step generation (accumulator-based multi-step burst). Each layer tested in isolation before wiring.

**Tech Stack:** Rust `no_std` (runtime crate), `nurbs` crate (scalar eval), `heapless` SPSC queues, C FFI (`runtime_tick.c`, `runtime_ffi.rs`).

**Spec:** `docs/superpowers/specs/2026-04-30-step7b-layer4-mcu-evaluator-design.md`

---

## File Map

### Modified files

| File | Changes |
|------|---------|
| `rust/runtime/src/curve_pool.rs` | Replace `LoadedCurve` (3D vector) with `LoadedScalarCurve` (1D scalar). Bump constants. Drop weights. |
| `rust/runtime/src/segment.rs` | 4 handles, `EMode`, `extrusion_ratio`. Update size assert. |
| `rust/runtime/src/trace.rs` | Add `motor_z: f32`. Pad to 40 bytes. Update size assert. |
| `rust/runtime/src/engine.rs` | Per-axis scalar eval, E-mode dispatch, step generation, `McuAxisConfig`, `needs_xy_seed`. |
| `rust/runtime/src/state.rs` | Add `homed: AtomicBool` to `SharedState`. Update `TickState` to 4 motors. |
| `rust/runtime/src/reclaim.rs` | Multi-handle retirement via segment_id → handles table. |
| `rust/runtime/src/kinematics.rs` | Update `corexy_with_e` to 4-element output (add Z passthrough). |
| `rust/runtime/src/slot.rs` | Update `PaSlot`/`IsSlot` for 4-element motor arrays. |
| `rust/runtime/src/error.rs` | Add fault codes: `NotHomed`, `StepBurstExceeded`, `ZeroDurationSegment`. |
| `rust/runtime/src/sim_fixtures.rs` | Replace 3D fixtures with scalar fixtures at various degrees. |
| `rust/runtime/src/stream.rs` | Reset `needs_xy_seed` and step accumulators on arm/flush. |
| `rust/runtime/src/lib.rs` | Re-export new types (`EMode`, `McuAxisConfig`, `MotorConfig`, `StepMotorState`). |
| `rust/kalico-c-api/src/runtime_ffi.rs` | Scalar blob passthrough, extended push_segment, multi-handle retirement table. |
| `src/runtime_tick.c` | Scalar blob parsing, extended DECL_COMMAND, 40-byte trace drain. |

### New files

| File | Purpose |
|------|---------|
| `rust/runtime/src/step.rs` | `StepMotorState` + accumulator-based step generation logic. |
| `rust/runtime/src/config.rs` | `McuAxisConfig`, `MotorConfig`, `EMode` types + validation. |

### Test files (modified/new)

| File | Purpose |
|------|---------|
| `rust/runtime/tests/curve_pool_alloc.rs` | Update for scalar curves, add degree-9 load/reject tests. |
| `rust/runtime/tests/engine_tick.rs` | Rewrite for scalar eval, E-mode dispatch, step output. |
| `rust/runtime/tests/fixtures/mod.rs` | New scalar fixture helpers. |
| `rust/runtime/tests/reclaim_pipeline.rs` | Multi-handle retirement. |
| `rust/runtime/tests/step_generation.rs` | New: accumulator, burst, cap, reversal, seeding. |
| `rust/runtime/tests/e_mode_dispatch.rs` | New: CoupledToXy, Independent, Travel, transitions. |
| `rust/runtime/tests/homed_gate.rs` | New: engine refuses to run when not homed. |

---

## Task 1: Curve Pool — Scalar Refactor

**Files:**
- Modify: `rust/runtime/src/curve_pool.rs`
- Modify: `rust/runtime/tests/curve_pool_alloc.rs`

- [ ] **Step 1: Update constants and `LoadedScalarCurve` struct**

In `rust/runtime/src/curve_pool.rs`, replace the existing constants and `LoadedCurve`:

```rust
// Old:
pub const MAX_CONTROL_POINTS: usize = 8;
pub const MAX_DIM: usize = 3;
pub const MAX_KNOT_VECTOR_LEN: usize = MAX_CONTROL_POINTS + 4;
pub const MAX_DEGREE: u8 = 3;

// New:
pub const MAX_CONTROL_POINTS: usize = 80;
pub const MAX_KNOT_VECTOR_LEN: usize = 91; // MAX_CONTROL_POINTS + MAX_DEGREE as usize + 1
pub const MAX_DEGREE: u8 = 10;
pub const CURVE_POOL_N: usize = 64;
```

Keep `MAX_DIM` as a deprecated constant (`pub const MAX_DIM: usize = 1;`) until Task 8 updates the FFI — removing it before Task 8 would break `kalico-c-api` compilation. Replace `LoadedCurve` (which had `control_points: [[f32; 3]; 8]`, `weights: [f32; 8]`, `knots: [f32; 12]`) with:

```rust
#[derive(Clone)]
pub struct LoadedScalarCurve {
    pub control_points: [f32; MAX_CONTROL_POINTS],
    pub knots: [f32; MAX_KNOT_VECTOR_LEN],
    pub n_cp: u8,
    pub n_knots: u8,
    pub degree: u8,
}
```

No weights array — all live pipeline NURBS are polynomial.

- [ ] **Step 2: Update `CurveView` to return scalar slices**

Change `CurveView` from returning `&[[f32; 3]]` control points to `&[f32]`:

```rust
pub struct CurveView<'a> {
    pub control_points: &'a [f32],
    pub knots: &'a [f32],
    pub degree: u8,
}
```

Remove the `weights` field from `CurveView` — no rational NURBS.

- [ ] **Step 3: Update `try_alloc_and_load` to accept scalar data**

Keep the existing slot-indexed `try_alloc_and_load` API shape but change its payload from 3D vector data + weights to scalar data `(degree: u8, knots: &[f32], cps: &[f32])`. Remove the weights parameter. Validate `degree <= MAX_DEGREE`, `n_cp <= MAX_CONTROL_POINTS`, `knots.len() == n_cp + degree as usize + 1`. Update all existing callers (tests, FFI) that construct `LoadedCurve` to use the new scalar signature.

- [ ] **Step 4: Add `CurveHandle::UNUSED_SENTINEL`**

```rust
impl CurveHandle {
    pub const UNUSED_SENTINEL: Self = Self { slot_idx: u16::MAX - 1, generation: u16::MAX - 1 };

    pub fn is_unused_sentinel(self) -> bool {
        self == Self::UNUSED_SENTINEL
    }
}
```

- [ ] **Step 5: Update tests in `curve_pool_alloc.rs`**

Replace all 3D fixtures with scalar data. Add tests:
- Load a degree-1 linear scalar curve (2 CPs, 4 knots) → success.
- Load a degree-9 curve (64 CPs, 74 knots) → success.
- Load a degree-10 curve (80 CPs, 91 knots) → success (at limit).
- Load a degree-11 curve → rejected with `DegreeTooHigh`.
- Load 81 CPs → rejected with `InvalidLengths`.
- Resolve a loaded curve → `CurveView` has correct scalar data.
- `UNUSED_SENTINEL.is_unused_sentinel()` returns true.

- [ ] **Step 6: Run tests**

Run: `cd rust && cargo test -p runtime -- curve_pool`
Expected: all curve_pool tests pass.

- [ ] **Step 7: Commit**

```
git add rust/runtime/src/curve_pool.rs rust/runtime/tests/curve_pool_alloc.rs
git commit -m "refactor: curve pool to scalar slots (degree 10, 80 CPs, 64 slots)"
```

---

## Task 2: Segment Struct + EMode + Config Types

**Files:**
- Modify: `rust/runtime/src/segment.rs`
- Create: `rust/runtime/src/config.rs`
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Create `config.rs` with `EMode`, `McuAxisConfig`, `MotorConfig`**

```rust
//! MCU axis configuration and E-mode types.

use crate::segment::KinematicTag;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EMode {
    CoupledToXy = 0,
    Independent = 1,
    Travel = 2,
}

#[derive(Debug, Clone)]
pub struct MotorConfig {
    pub steps_per_mm: f32,
    pub is_awd: bool,
    pub invert_dir: bool,
}

#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    /// Per-motor config, indexed in motor space (post-kinematic-transform):
    /// CoreXyAndE: [A=0, B=1, Z=2, E=3]; CartesianXyzAndE: [X=0, Y=1, Z=2, E=3].
    pub motors: [Option<MotorConfig>; 4],
    pub kinematics: KinematicTag,
}

impl McuAxisConfig {
    /// Validate config constraints. Returns Err description on failure.
    pub fn validate(&self) -> Result<(), &'static str> {
        match self.kinematics {
            KinematicTag::CoreXyAndE => {
                let has_a = self.motors[0].is_some();
                let has_b = self.motors[1].is_some();
                if has_a != has_b {
                    return Err("CoreXY: must own both A and B or neither");
                }
                Ok(())
            }
            KinematicTag::CartesianXyzAndE => Ok(()),
        }
    }
}
```

- [ ] **Step 2: Update `Segment` struct with 4 handles + E-mode**

In `rust/runtime/src/segment.rs`:

```rust
use crate::config::EMode;
use crate::curve_pool::CurveHandle;

#[derive(Debug, Clone, Copy)]
#[repr(C)]
pub struct Segment {
    pub id: u32,
    pub x_handle: CurveHandle,
    pub y_handle: CurveHandle,
    pub z_handle: CurveHandle,
    pub e_handle: CurveHandle,
    pub t_start: u64,
    pub t_end: u64,
    pub kinematics: KinematicTag,
    pub e_mode: EMode,
    pub extrusion_ratio: f32,
    pub flags: u8,
    pub _pad: [u8; 2],
}
```

Update the size assert from 32 to the new size. Keep `segment_size_is_under_64_bytes`. Remove the `segment_size_locked_at_32_bytes` test and replace with a new locked-size test at the actual new size.

Update `duration()` method — unchanged.

Update all test `Segment` construction sites to use the new fields (use `CurveHandle::UNUSED_SENTINEL` for unused handles, `EMode::CoupledToXy` as default, `extrusion_ratio: 0.0`).

- [ ] **Step 3: Add `mod config;` to `lib.rs` and re-export**

```rust
pub mod config;
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p runtime -- segment`
Expected: segment tests pass with new struct.

- [ ] **Step 5: Commit**

```
git add rust/runtime/src/segment.rs rust/runtime/src/config.rs rust/runtime/src/lib.rs
git commit -m "feat: segment struct with 4 per-axis handles, EMode, and McuAxisConfig"
```

---

## Task 3: Trace Struct + Error Codes + SharedState

**Files:**
- Modify: `rust/runtime/src/trace.rs`
- Modify: `rust/runtime/src/error.rs`
- Modify: `rust/runtime/src/state.rs`

- [ ] **Step 1: Add `motor_z` to `TraceSample`, update padding**

In `rust/runtime/src/trace.rs`:

```rust
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceSample {
    pub tick: u64,
    pub motor_a: f32,
    pub motor_b: f32,
    pub motor_z: f32,
    pub motor_e: f32,
    pub segment_id: u32,
    pub curve_handle: CurveHandle,
    pub flags: u8,
    pub _pad: [u8; 7],
}
```

Update `Default` impl to include `motor_z: 0.0` and `_pad: [0; 7]`.

Update the size assert to 40 bytes.

- [ ] **Step 2: Add new fault codes**

In `rust/runtime/src/error.rs`, add:

```rust
pub const KALICO_ERR_NOT_HOMED: i32 = -20;
pub const KALICO_ERR_STEP_BURST_EXCEEDED: i32 = -21;
pub const KALICO_ERR_ZERO_DURATION_SEGMENT: i32 = -22;
```

And corresponding `RuntimeError` / `FaultCode` variants:
```rust
NotHomed,
StepBurstExceeded,
ZeroDurationSegment,
```

Update `impl From<RuntimeError> for i32` (or the equivalent match in `FaultCode::as_i32()`) with the new variants mapped to the new constants. Update any exhaustiveness tests.

- [ ] **Step 3: Add `homed` to `SharedState`**

In `rust/runtime/src/state.rs`, add to `SharedState`:

```rust
pub homed: AtomicBool,
```

Initialize to `false` in `SharedState::new()`.

- [ ] **Step 4: Update `TickState` to 4 motors**

```rust
pub struct TickState {
    pub dt: f32,
    pub positions: [f32; 4],  // [x, y, z, e] logical
    pub motors: [f32; 4],     // [a, b, z, e] or [x, y, z, e] post-kinematic
}
```

- [ ] **Step 5: Run tests**

Run: `cd rust && cargo test -p runtime`
Expected: compilation succeeds. Some tests will need `TraceSample` and `Segment` construction updates — fix all call sites.

- [ ] **Step 6: Fix all compilation errors across test files**

Every test that constructs a `Segment` or `TraceSample` must use the new field layout. This is a mechanical update: add the missing fields with default/sentinel values. Fix each file:
- `engine_tick.rs`, `engine_underrun.rs`, `hold_segment.rs`, `flush_basic.rs`, `flush_drains_queue.rs`, `flush_timeout.rs`, `force_idle_short_circuit.rs`, `max_boundary_iters.rs`, `reclaim_pipeline.rs`, `segment_id_atomics.rs`, `stream_lifecycle.rs`, `trace_overflow.rs`, `fixtures/mod.rs`.

Update `TickState` usages in `slot.rs` and `engine.rs` for 4-element arrays. Also widen `Engine::last_motors` from `[f32; 3]` to `[f32; 4]` and update all trace emit sites to write `motor_z: last_motors[2]`, `motor_e: last_motors[3]`.

- [ ] **Step 7: Run full test suite**

Run: `cd rust && cargo test -p runtime`
Expected: all tests pass.

- [ ] **Step 8: Commit**

```
git add rust/runtime/src/trace.rs rust/runtime/src/error.rs rust/runtime/src/state.rs
git add rust/runtime/tests/
git commit -m "feat: 4-motor trace (40 bytes), homed gate, new fault codes"
```

---

## Task 4: Kinematics + Slot Pipeline Update

**Files:**
- Modify: `rust/runtime/src/kinematics.rs`
- Modify: `rust/runtime/src/slot.rs`

- [ ] **Step 1: Update kinematics to 4-element arrays**

In `rust/runtime/src/kinematics.rs`:

```rust
/// CoreXY: (x, y, z, e) → (a=x+y, b=x-y, z, e)
#[inline(always)]
pub fn corexy_with_e(pos: [f32; 4]) -> [f32; 4] {
    [pos[0] + pos[1], pos[0] - pos[1], pos[2], pos[3]]
}

/// Cartesian: identity
#[inline(always)]
pub fn cartesian_xyz_with_e(pos: [f32; 4]) -> [f32; 4] {
    pos
}
```

- [ ] **Step 2: Update slot traits for 4-element TickState**

In `rust/runtime/src/slot.rs`, update `PaSlot::apply` and `IsSlot::apply` to work with the new `TickState` (4-element `positions` and `motors`). The `NoopPa`/`NoopIs` impls remain no-ops.

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p runtime`
Expected: pass.

- [ ] **Step 4: Commit**

```
git add rust/runtime/src/kinematics.rs rust/runtime/src/slot.rs
git commit -m "refactor: kinematics and slots to 4-element motor arrays"
```

---

## Task 5: Step Generation Module

**Files:**
- Create: `rust/runtime/src/step.rs`
- Create: `rust/runtime/tests/step_generation.rs`
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Write failing tests for step accumulator**

Create `rust/runtime/tests/step_generation.rs`:

```rust
use runtime::step::{StepMotorState, StepResult, MAX_STEPS_PER_TICK_DEFAULT};

#[test]
fn zero_delta_produces_no_steps() {
    let mut state = StepMotorState::new(160.0); // 160 steps/mm
    let result = state.update(0.0); // position = 0
    assert_eq!(result.unwrap().n_steps, 0);
}

#[test]
fn one_step_forward() {
    let mut state = StepMotorState::new(160.0);
    // Move 1/160 mm = exactly 1 step
    let result = state.update(1.0 / 160.0).unwrap();
    assert_eq!(result.n_steps, 1);
}

#[test]
fn four_steps_at_peak_speed() {
    let mut state = StepMotorState::new(160.0);
    // Move 4/160 mm = 4 steps (simulates one tick at 1000 mm/s)
    let result = state.update(4.0 / 160.0).unwrap();
    assert_eq!(result.n_steps, 4);
}

#[test]
fn negative_steps_on_reversal() {
    let mut state = StepMotorState::new(160.0);
    state.update(10.0 / 160.0).unwrap(); // forward 10 steps
    let result = state.update(7.0 / 160.0).unwrap(); // back 3 steps
    assert_eq!(result.n_steps, -3);
}

#[test]
fn fractional_accumulation() {
    let mut state = StepMotorState::new(160.0);
    // Move 0.5 steps — should accumulate, no step yet
    let r1 = state.update(0.5 / 160.0).unwrap();
    assert_eq!(r1.n_steps, 0);
    // Move another 0.5 steps — now should emit 1 step
    let r2 = state.update(1.0 / 160.0).unwrap();
    assert_eq!(r2.n_steps, 1);
}

#[test]
fn burst_cap_faults() {
    let mut state = StepMotorState::new(160.0);
    // Jump 100 steps in one tick — exceeds MAX_STEPS_PER_TICK_DEFAULT (16)
    let result = state.update(100.0 / 160.0);
    assert!(result.is_err());
}

#[test]
fn seed_prevents_initial_burst() {
    let mut state = StepMotorState::new(160.0);
    state.seed(50.0); // seed at 50mm motor position
    let result = state.update(50.0).unwrap(); // same position
    assert_eq!(result.n_steps, 0);
}

#[test]
fn drift_over_many_ticks() {
    let mut state = StepMotorState::new(160.0);
    let step_mm = 1.0 / 160.0;
    let mut pos = 0.0_f64;
    let mut total_steps: i64 = 0;
    for _ in 0..1_000_000 {
        pos += step_mm;
        let r = state.update(pos as f32).unwrap();
        total_steps += r.n_steps as i64;
    }
    // After 1M ticks of 1 step each, should have exactly 1M steps
    assert_eq!(total_steps, 1_000_000);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd rust && cargo test -p runtime -- step_generation`
Expected: FAIL — `runtime::step` module not found.

- [ ] **Step 3: Implement `step.rs`**

Create `rust/runtime/src/step.rs`:

```rust
//! Accumulator-based step generation.

pub const MAX_STEPS_PER_TICK_DEFAULT: i32 = 16;

#[derive(Debug)]
pub struct StepResult {
    pub n_steps: i32,
}

#[derive(Debug)]
#[derive(Clone, Copy)]
pub struct StepMotorState {
    step_accumulator: f64,
    steps_per_mm: f32,
    max_steps_per_tick: i32,
}

impl Default for StepMotorState {
    fn default() -> Self {
        Self { step_accumulator: 0.0, steps_per_mm: 0.0, max_steps_per_tick: MAX_STEPS_PER_TICK_DEFAULT }
    }
}

impl StepMotorState {
    pub fn new(steps_per_mm: f32) -> Self {
        Self {
            step_accumulator: 0.0,
            steps_per_mm,
            max_steps_per_tick: MAX_STEPS_PER_TICK_DEFAULT,
        }
    }

    pub fn seed(&mut self, motor_position_mm: f32) {
        self.step_accumulator = f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
    }

    pub fn update(&mut self, motor_position_mm: f32) -> Result<StepResult, ()> {
        let new_pos_steps = f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
        let delta = new_pos_steps - self.step_accumulator;
        let n_steps = delta as i32; // truncates toward zero
        if n_steps.abs() > self.max_steps_per_tick {
            return Err(());
        }
        self.step_accumulator += f64::from(n_steps);
        Ok(StepResult { n_steps })
    }
}
```

Add `pub mod step;` to `lib.rs`.

Note: `StepMotorState::update` returns `StepResult { n_steps }` — the count and direction of steps to emit. Actual GPIO pulse emission (BSRR writes, dir pin, AWD dual-pin, timing delays) is hardware-specific and not testable on the host. The engine's tick method calls `update()` and stores the result; the actual pulse emission is a thin hardware layer wired in 7-D when real GPIO is available. Tests verify step counts, not GPIO toggles.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd rust && cargo test -p runtime -- step_generation`
Expected: all 8 tests pass.

- [ ] **Step 5: Commit**

```
git add rust/runtime/src/step.rs rust/runtime/src/lib.rs rust/runtime/tests/step_generation.rs
git commit -m "feat: accumulator-based step generation with burst cap"
```

---

## Task 6: Engine Evaluator Rewrite

**Files:**
- Modify: `rust/runtime/src/engine.rs`
- Create: `rust/runtime/tests/e_mode_dispatch.rs`
- Modify: `rust/runtime/tests/engine_tick.rs`
- Modify: `rust/runtime/tests/fixtures/mod.rs`

This is the largest task. The engine's `tick_with_current` method is rewritten to:
1. Resolve per-axis scalar handles (not one 3D handle).
2. Eval via `ScalarNurbsRef` per axis.
3. Dispatch E-mode (CoupledToXy / Independent / Travel).
4. Apply kinematic transform.
5. Generate steps per motor.

- [ ] **Step 1: Update `Engine` struct with new fields**

Add to `Engine`:

```rust
prev_x: f32,
prev_y: f32,
e_accumulator: f64,
needs_xy_seed: bool,
step_state: [crate::step::StepMotorState; 4],
mcu_config: Option<crate::config::McuAxisConfig>,
```

Update `Engine::new()` and `Engine::default()` to initialize these fields. `mcu_config` starts as `None`; set via a new `Engine::configure(&mut self, config: McuAxisConfig)` method.

- [ ] **Step 2: Update scalar fixtures**

In `rust/runtime/tests/fixtures/mod.rs`, replace 3D vector NURBS fixtures with scalar NURBS helpers:

```rust
/// Create a degree-1 linear scalar NURBS from `start` to `end` on [0, 1].
pub fn linear_scalar(start: f32, end: f32) -> (u8, Vec<f32>, Vec<f32>) {
    let degree = 1u8;
    let knots = vec![0.0, 0.0, 1.0, 1.0];
    let cps = vec![start, end];
    (degree, knots, cps)
}

/// Load a scalar NURBS into the curve pool, returning the handle.
pub fn load_scalar(pool: &CurvePool, degree: u8, knots: &[f32], cps: &[f32]) -> CurveHandle {
    pool.try_alloc_and_load(degree, knots, cps).expect("load failed")
}
```

- [ ] **Step 3: Replace `nurbs_eval_3d` with `scalar_eval`**

In `engine.rs`, replace the `nurbs_eval_3d` function with:

```rust
fn scalar_eval(curve: &CurveView<'_>, u: f32) -> Result<f32, ()> {
    use nurbs::ScalarNurbsRef;
    let view = ScalarNurbsRef::<f32>::try_new(
        curve.degree,
        curve.knots,
        curve.control_points,
        None, // no weights — polynomial
    ).map_err(|_| ())?;
    Ok(nurbs::eval::eval(&view, u))
}
```

- [ ] **Step 4: Rewrite `tick_with_current` eval path**

Replace the single-handle resolve + 3D eval with:
1. Resolve X, Y, Z handles (skip sentinels based on `mcu_config`).
2. Eval each via `scalar_eval`.
3. E-mode dispatch:
   - `CoupledToXy`: compute `v_xy` from `(x - prev_x, y - prev_y)`, accumulate `e_accumulator`.
   - `Independent`: resolve `e_handle`, eval.
   - `Travel`: E = last value.
4. If `needs_xy_seed`, evaluate X(u=0) and Y(u=0) FIRST, seed `prev_x/prev_y` from those values, apply kinematic transform to get motor positions, seed all step accumulators, then clear `needs_xy_seed`. This must happen BEFORE the E finite-difference computation to avoid a spurious first-tick delta.
5. Kinematic transform: `[x, y, z, e]` → `[a, b, z, e]`.
6. Step generation per owned motor via `step_state[i].update(motors[i])`.

The boundary loop and hold-segment paths carry the 4-handle segment through unchanged (boundary loop already uses `current.curve_handle` only for SEGMENT_END trace — now uses `current.x_handle` as the primary diagnostic handle).

- [ ] **Step 5: Add homed gate**

After `force_idle` check AND after clock widening (so `now` is available for `latch_fault`), but before segment activation:

```rust
if !shared.homed.load(Ordering::Acquire) {
    if shared.stream_open.load(Ordering::Acquire) {
        self.latch_fault(RuntimeError::NotHomed, 0, CurveHandle::UNUSED_SENTINEL, now, trace, shared, None);
        return Err(RuntimeError::NotHomed);
    }
    return Ok(());
}
```

- [ ] **Step 6: Update engine_tick.rs tests**

Update existing tests to construct 4-handle segments with scalar curve fixtures. Verify:
- Idle → segment push → tick → trace sample has correct motor positions.
- SEGMENT_END emitted at correct time.
- Multi-segment boundary crossing works.

Use `McuAxisConfig` with `CoreXyAndE`, motors [A, B, _, E] configured.

- [ ] **Step 7: Write E-mode dispatch tests**

Create `rust/runtime/tests/e_mode_dispatch.rs`:

- `coupled_e_accumulates_arc_length`: Load X curve (0→50mm linear), Y constant at 0. E mode CoupledToXy with ratio 0.04. Tick through segment. Verify final E ≈ 0.04 × 50 = 2.0 mm.
- `independent_e_tracks_nurbs`: Load X/Y constant, E NURBS linear 10→5. E mode Independent. Verify E follows the NURBS.
- `travel_e_stays_constant`: X/Y moving, E mode Travel. Verify E unchanged.
- `e_seed_after_independent`: CoupledToXy segment, then Independent retraction, then CoupledToXy. Verify E accumulator syncs correctly across transitions.
- `xy_seed_prevents_spurious_extrusion`: First segment starts at X=100. Verify first-tick E delta is zero (not computed from prev_x=0 to x=100).

- [ ] **Step 8: Write homed gate test**

Create `rust/runtime/tests/homed_gate.rs`:

- `engine_refuses_to_run_when_not_homed`: Set `homed = false`, open stream, push segment, tick → fault `NotHomed`.
- `engine_runs_when_homed`: Set `homed = true` → runs normally.

- [ ] **Step 9: Run all tests**

Run: `cd rust && cargo test -p runtime`
Expected: all tests pass.

- [ ] **Step 10: Commit**

```
git add rust/runtime/src/engine.rs rust/runtime/tests/
git commit -m "feat: per-axis scalar evaluator with E-mode dispatch and step generation"
```

---

## Task 7: Multi-Handle Retirement

**Files:**
- Modify: `rust/runtime/src/reclaim.rs`
- Modify: `rust/runtime/tests/reclaim_pipeline.rs`

- [ ] **Step 1: Add segment_id → handles lookup to reclaim**

The foreground maintains a table mapping `segment_id` to its 4 `CurveHandle`s. On SEGMENT_END, retire all non-sentinel handles.

In `rust/runtime/src/reclaim.rs`, add:

```rust
use crate::curve_pool::CurveHandle;

pub const RETIREMENT_TABLE_N: usize = 16;

pub struct RetirementTable {
    entries: [(u32, [CurveHandle; 4]); RETIREMENT_TABLE_N],
    head: usize,
}

impl RetirementTable {
    pub const fn new() -> Self {
        Self {
            entries: [(0, [CurveHandle::UNUSED_SENTINEL; 4]); RETIREMENT_TABLE_N],
            head: 0,
        }
    }

    pub fn register(&mut self, segment_id: u32, handles: [CurveHandle; 4]) {
        self.entries[self.head] = (segment_id, handles);
        self.head = (self.head + 1) % RETIREMENT_TABLE_N;
    }

    pub fn lookup(&self, segment_id: u32) -> Option<[CurveHandle; 4]> {
        self.entries.iter()
            .find(|(id, _)| *id == segment_id)
            .map(|(_, handles)| *handles)
    }
}
```

Add `retirement_table: RetirementTable` to `FgState` in `state.rs`. Initialize it as `RetirementTable::new()` in `RuntimeContext::init`. The producer calls `fg.retirement_table.register(segment_id, handles)` when pushing each segment. Update `drain_and_reclaim` to accept `&RetirementTable` and retire all handles:

```rust
pub fn drain_and_reclaim<F>(
    pool: &CurvePool,
    table: &RetirementTable,
    mut drain_one: F,
    limit: usize,
) -> usize
where
    F: FnMut() -> Option<TraceSample>,
{
    let mut drained = 0;
    while drained < limit {
        let Some(sample) = drain_one() else { break };
        if sample.flags & TRACE_FLAG_SEGMENT_END != 0 {
            if let Some(handles) = table.lookup(sample.segment_id) {
                for h in &handles {
                    if !h.is_unused_sentinel() && *h != CurveHandle::HOLD_SEGMENT_SENTINEL {
                        pool.confirm_retired(*h);
                    }
                }
            }
        }
        drained += 1;
    }
    drained
}
```

- [ ] **Step 2: Update `reclaim_pipeline.rs` tests**

- Load 3 scalar curves (X, Y, Z) into pool.
- Push a segment with all 3 handles + UNUSED for E.
- Register in retirement table.
- Tick through → SEGMENT_END emitted.
- Drain and reclaim → all 3 slots freed.
- Verify new allocs succeed on those slots.

- [ ] **Step 3: Run tests**

Run: `cd rust && cargo test -p runtime -- reclaim`
Expected: pass.

- [ ] **Step 4: Commit**

```
git add rust/runtime/src/reclaim.rs rust/runtime/tests/reclaim_pipeline.rs
git commit -m "feat: multi-handle retirement via segment_id lookup table"
```

---

## Task 8: FFI Layer Update

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `src/runtime_tick.c`

- [ ] **Step 1: Update Rust FFI `kalico_load_curve` to scalar blob passthrough**

Replace the `n_cp * MAX_DIM` slice construction with a single aligned blob passthrough. The C handler passes the raw scalar NURBS wire blob (8-byte header + knots + CPs, as defined in `nurbs::wire`); the Rust FFI calls `ScalarNurbsRef::try_from_wire` on the full blob for validation, then copies the parsed degree/knots/CPs into the `LoadedScalarCurve` slot. Do NOT pass separate knots/cps buffers — `try_from_wire` expects a single contiguous wire buffer with header.

- [ ] **Step 2: Update Rust FFI `kalico_push_segment` for 4 handles + E-mode**

Accept 4 packed handles (`x_handle`, `y_handle`, `z_handle`, `e_handle`), `e_mode: u8`, `extrusion_ratio: u32` (f32 bits). Construct the new `Segment` struct. Register handles in the `RetirementTable`.

Validate `t_end > t_start` — return `KALICO_ERR_ZERO_DURATION_SEGMENT` on failure.

- [ ] **Step 3: Add `kalico_set_homed` command**

```rust
pub unsafe extern "C" fn kalico_set_homed(ctx: *mut RuntimeContext) {
    let shared = &(*ctx).shared;
    shared.homed.store(true, Ordering::Release);
}
```

- [ ] **Step 4: Update C-side `runtime_tick.c`**

Update `kalico_load_curve` DECL_COMMAND:
- Remove the `cps_len % 12` check and 3D scratch buffers.
- Pass raw `cps`, `knots` blob pointers and lengths to the Rust FFI.
- Remove the `weights` parameter (or accept and ignore for backward compat during transition).

Update `kalico_push_segment` DECL_COMMAND:
- Add `x_handle`, `y_handle`, `z_handle`, `e_handle`, `e_mode`, `extrusion_ratio` parameters.

Update trace drain buffer sizing from 32 to 40 bytes per sample.

Add `DECL_COMMAND(kalico_set_homed, ...)`.

Add `DECL_COMMAND(kalico_configure_axes, ...)` — accepts a blob containing the serialized `McuAxisConfig` (kinematics tag + per-motor config array). The Rust FFI parses and validates (rejects invalid CoreXY configs that own only one of A/B), stores in `Engine::configure()`.

Regenerate the cbindgen header (`kalico_runtime.h`) after updating `runtime_ffi.rs` — `src/runtime_tick.c` includes this header and will compile against stale declarations otherwise. Run: `cd rust && cargo run -p kalico-c-api --bin gen_headers`.

- [ ] **Step 5: Run FFI tests**

Run: `cd rust && cargo test -p kalico-c-api`
Expected: pass (or update tests for new signatures).

- [ ] **Step 6: Commit**

```
git add rust/kalico-c-api/src/runtime_ffi.rs src/runtime_tick.c
git commit -m "feat: FFI layer updated for scalar curves, 4-handle segments, homed gate"
```

---

## Task 9: Integration Tests + Full Suite Verification

**Files:**
- All test files

- [ ] **Step 1: Run the complete runtime test suite**

Run: `cd rust && cargo test -p runtime`
Expected: all tests pass.

- [ ] **Step 2: Run the complete kalico-c-api test suite**

Run: `cd rust && cargo test -p kalico-c-api`
Expected: all tests pass.

- [ ] **Step 3: Run the complete workspace test suite**

Run: `cd rust && cargo test`
Expected: all tests pass (no regressions in nurbs, gcode, temporal, trajectory crates).

- [ ] **Step 4: Verify Renode sim build compiles**

Run: `cd rust && cargo build -p runtime --features kalico-sim`
Expected: builds without errors. The 43 KB curve pool fits in the Renode 128 KB RAM model (with reduced `CURVE_POOL_N` if needed — add a `cfg(feature = "kalico-sim")` override to cap at 16 for sim builds).

- [ ] **Step 5: Final commit if any fixups needed**

```
git add -A
git commit -m "fix: integration test fixups for Step 7-B"
```

---

## Task 10: Documentation Update

**Files:**
- Modify: `CLAUDE.md` (build order)
- Modify: `docs/superpowers/plan-changes-log.md`

- [ ] **Step 1: Update CLAUDE.md build order**

Add checkmark to Step 7-B. Update the description to reflect completion.

- [ ] **Step 2: Update plan-changes-log**

Add entry:
```
### 2026-04-30 — Step 7-B complete
- Curve pool refactored to per-axis scalar (degree 10, 80 CPs, 64 slots).
- Segment struct carries 4 per-axis handles + EMode + extrusion_ratio.
- Engine evaluator: per-axis scalar de Boor, CoupledToXy E integration, Independent E eval, Travel hold.
- Step generation: f64 accumulator, multi-step burst with MAX_STEPS_PER_TICK cap, AWD support.
- Multi-handle retirement via foreground segment_id → handles table.
- Safety gate: homed flag in SharedState.
- FFI + C-side updated for scalar blobs, 4-handle push, 40-byte trace.
```

- [ ] **Step 3: Commit**

```
git add CLAUDE.md docs/superpowers/plan-changes-log.md
git commit -m "docs: mark Step 7-B complete in CLAUDE.md and plan-changes-log"
```
