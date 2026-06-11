# Phase Stepping ↔ Sensorless Homing Mode Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Sensorless (StallGuard) homing works on `phase_stepping: true` steppers by switching the stepper to Pulse (step/dir) mode around the trip move, with phase handover that preserves the homed origin exactly.

**Architecture:** The MCU runtime gains three OID-addressed phase-handover primitives (jog-to-phase with ramp, align-offset-no-motion, phase-state query) exposed as MCU commands. Python orchestrates the sequence from `TMCVirtualPinHelper.arm()/disarm()` via a refactored TMC5160 driver that owns enter/exit of direct mode. Spec: `docs/superpowers/specs/2026-06-11-phase-stepping-sensorless-homing-mode-switch-design.md`.

**Tech Stack:** Rust (`rust/runtime`, `rust/kalico-c-api`), C MCU commands (`src/stepper.c`), Klippy Python (`klippy/extras/tmc5160.py`, `klippy/extras/tmc.py`, `klippy/motion_toolhead.py`).

**Key encodings (do not confuse):**
- `kalico_set_axis_mode` / `AxisState.mode`: **0 = Pulse, 1 = Phase** (`rust/runtime/src/stepping_state.rs:16`).
- `configure_axes` bridge blob `step_modes`: 0 = Modulated(=Phase), 1 = StepTime(=Pulse) — a *different* vocabulary (`runtime_ffi.rs:770`). This plan only uses the first one.
- Phase index space: `(axis.last_step_count + stepper.phase_offset_microsteps) & 0x3FF`, identical to MSCNT's 0–1023 space; `_xdirect_preload` already maps `mscnt → (cos, sin)` exactly like `PHASE_LUT` (`rust/runtime/src/phase_lut.rs`).

**Existing machinery this plan builds on (verified):**
- `Engine::set_axis_mode` (`rust/runtime/src/engine.rs:408`) — refuses while motion armed (-2), resets step queue, re-seeds `last_phase_target` on entry to Phase. MCU command `kalico_set_axis_mode axis_idx=%c mode=%c` (`src/stepper.c:283`), which `shutdown()`s on nonzero rc.
- `Engine::set_stepper_offset` (`engine.rs:458`) + `ramp_phase_offset` (`dispatch_stepper.rs:254`) — ISR-ramped offset slew. MCU command `kalico_set_stepper_offset` (`src/stepper.c:297`).
- `kalico_phase_stepping_{enable,disable}_spi` MCU commands (`src/stepper.c:263-280`).
- `TMC5160._xdirect_preload` (`klippy/extras/tmc5160.py:495`) — direct-mode entry sequence, registered as post-enable callback.
- `TMCVirtualPinHelper.arm()/disarm()` (`klippy/extras/tmc.py:640/673`) — bracket the trip move via `trip_move_begin/end` (`klippy/extras/homing.py:361,402`); `toolhead.wait_moves()` guarantees standstill before `arm()` (`homing.py:358`); home current is applied before and restored after the whole trip sequence (`homing.py:321-341`), and the direct-mode current helper forces `ihold = irun` (`tmc5160.py:280,358`), so home current automatically scales both XDIRECT (IHOLD) and step-mode (IRUN) — current handling needs **no new code**, only this ordering.

---

### Task 1: Runtime phase-handover primitives (`phase_handover` module)

**Files:**
- Create: `rust/runtime/src/phase_handover.rs`
- Create: `rust/runtime/src/phase_handover/tests.rs`
- Modify: `rust/runtime/src/lib.rs` (add `pub mod phase_handover;` next to the existing `pub mod` list)

- [ ] **Step 1: Write the failing tests**

Create `rust/runtime/src/phase_handover/tests.rs`. Reuse the construction patterns from `rust/runtime/src/dispatch_stepper/tests.rs` (`make_stepper()` / `make_axis(...)` at lines 10/23 build `StepperRef` and `AxisConfig` directly; mirror how that file constructs `SharedState`). The tests below are normative for behavior; adapt constructor calls to the existing helpers:

```rust
use super::*;
use crate::stepping_state::{AxisState, StepMode, StepperRef};
use core::sync::atomic::Ordering;

fn axis_with_stepper(mode: StepMode, oid: u8) -> AxisState {
    let mut axis = AxisState::new_unconfigured();
    axis.mode.store(mode as u8, Ordering::Release);
    axis.microstep_distance = 0.000_625;
    axis.steppers.push(StepperRef::new(oid, Some(7))).unwrap();
    axis
}

#[test]
fn shortest_delta_forward() {
    assert_eq!(shortest_phase_delta(10, 44), 34);
}

#[test]
fn shortest_delta_wraps_backward() {
    // 1000 -> 10 is +34 through the wrap, not -990.
    assert_eq!(shortest_phase_delta(1000, 10), 34);
}

#[test]
fn shortest_delta_wraps_forward_negative() {
    assert_eq!(shortest_phase_delta(10, 1000), -34);
}

#[test]
fn shortest_delta_zero() {
    assert_eq!(shortest_phase_delta(512, 512), 0);
}

#[test]
fn shortest_delta_halfway_is_positive() {
    assert_eq!(shortest_phase_delta(0, 512), 512);
}

#[test]
fn find_stepper_locates_by_oid_across_axes() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    axes[0] = Some(axis_with_stepper(StepMode::Phase, 3));
    axes[2] = Some(axis_with_stepper(StepMode::Pulse, 9));
    let (axis_idx, _, stepper) = find_stepper(&axes, 9).unwrap();
    assert_eq!(axis_idx, 2);
    assert_eq!(stepper.stepper_oid, 9);
    assert!(find_stepper(&axes, 99).is_none());
}

#[test]
fn align_to_sets_both_offsets_and_matches_target_phase() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Pulse, 5);
    axis.last_step_count = 70_000; // 70000 & 0x3FF = 368
    axes[1] = Some(axis);
    assert_eq!(align_to(&axes, 5, 100), 0);
    let axis = axes[1].as_ref().unwrap();
    let stepper = &axis.steppers[0];
    let off = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    assert_eq!(off, stepper.phase_offset_target.load(Ordering::Acquire));
    assert_eq!((axis.last_step_count.wrapping_add(off)) & 0x3FF, 100);
    // Shortest path: |delta| <= 512.
    assert!(off.abs() <= 512);
}

#[test]
fn align_to_rejects_unknown_oid_and_bad_phase() {
    let axes: [Option<AxisState>; 4] = [const { None }; 4];
    assert_ne!(align_to(&axes, 5, 100), 0);
    let mut axes2: [Option<AxisState>; 4] = [const { None }; 4];
    axes2[0] = Some(axis_with_stepper(StepMode::Pulse, 5));
    assert_ne!(align_to(&axes2, 5, 1024), 0);
}

#[test]
fn jog_to_moves_offset_target_by_shortest_path_requires_phase_mode() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Phase, 5);
    axis.last_step_count = 1020; // phase 1020
    axes[0] = Some(axis);
    let shared = /* SharedState construction per dispatch_stepper/tests.rs */;
    assert_eq!(jog_to(&axes, &shared, 5, 4, 1), 0);
    let stepper = &axes[0].as_ref().unwrap().steppers[0];
    // 1020 -> 4 is +8 through the wrap.
    assert_eq!(stepper.phase_offset_target.load(Ordering::Acquire), 8);
    assert_eq!(
        shared.max_phase_offset_ramp_per_sample.load(Ordering::Acquire),
        1
    );
    // Pulse mode is refused.
    axes[0]
        .as_ref()
        .unwrap()
        .mode
        .store(StepMode::Pulse as u8, Ordering::Release);
    assert_ne!(jog_to(&axes, &shared, 5, 4, 1), 0);
}

#[test]
fn jog_to_composes_with_pending_target_not_current_offset() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let axis = axis_with_stepper(StepMode::Phase, 5);
    axes[0] = Some(axis);
    let shared = /* as above */;
    {
        let stepper = &axes[0].as_ref().unwrap().steppers[0];
        stepper.phase_offset_target.store(100, Ordering::Release);
        stepper.phase_offset_microsteps.store(40, Ordering::Release);
    }
    // last_step_count = 0, pending phase = 100; jog to 110 adds +10 on top
    // of the pending target, not on the in-flight current offset.
    assert_eq!(jog_to(&axes, &shared, 5, 110, 1), 0);
    let stepper = &axes[0].as_ref().unwrap().steppers[0];
    assert_eq!(stepper.phase_offset_target.load(Ordering::Acquire), 110);
}

#[test]
fn query_reports_phase_mode_and_settled() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Phase, 5);
    axis.last_step_count = 2048; // phase 0
    axes[3] = Some(axis);
    {
        let stepper = &axes[3].as_ref().unwrap().steppers[0];
        stepper.phase_offset_microsteps.store(5, Ordering::Release);
        stepper.phase_offset_target.store(5, Ordering::Release);
    }
    let q = query(&axes, 5).unwrap();
    assert_eq!(q.axis_idx, 3);
    assert_eq!(q.mode, StepMode::Phase as u8);
    assert_eq!(q.phase, 5);
    assert!(q.settled);
    axes[3].as_ref().unwrap().steppers[0]
        .phase_offset_target
        .store(9, Ordering::Release);
    assert!(!query(&axes, 5).unwrap().settled);
}
```

- [ ] **Step 2: Run tests to verify they fail to compile (module missing)**

Run from `rust/`: `cargo nextest run -p runtime -E 'test(phase_handover)'`
Expected: compile error, `phase_handover` not found.

- [ ] **Step 3: Implement `rust/runtime/src/phase_handover.rs`**

```rust
use core::sync::atomic::Ordering;

use crate::state::SharedState;
use crate::stepping_state::{AxisState, StepMode, StepperRef};

pub const PHASE_PERIOD: i32 = 1024;
pub const PHASE_MASK: i32 = PHASE_PERIOD - 1;

pub struct PhaseQuery {
    pub axis_idx: u8,
    pub mode: u8,
    pub phase: u16,
    pub settled: bool,
}

pub fn shortest_phase_delta(current_phase: u16, target_phase: u16) -> i32 {
    let raw =
        (i32::from(target_phase) - i32::from(current_phase)).rem_euclid(PHASE_PERIOD);
    if raw > PHASE_PERIOD / 2 {
        raw - PHASE_PERIOD
    } else {
        raw
    }
}

pub fn find_stepper(
    axes: &[Option<AxisState>],
    stepper_oid: u8,
) -> Option<(usize, &AxisState, &StepperRef)> {
    for (axis_idx, axis_opt) in axes.iter().enumerate() {
        let Some(axis) = axis_opt else { continue };
        for stepper in &axis.steppers {
            if stepper.stepper_oid == stepper_oid {
                return Some((axis_idx, axis, stepper));
            }
        }
    }
    None
}

#[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
fn phase_of(last_step_count: i32, offset: i32) -> u16 {
    (last_step_count.wrapping_add(offset) & PHASE_MASK) as u16
}

pub fn jog_to(
    axes: &[Option<AxisState>],
    shared: &SharedState,
    stepper_oid: u8,
    target_phase: u16,
    max_microsteps_per_sample: u16,
) -> i32 {
    if i32::from(target_phase) >= PHASE_PERIOD {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    }
    if max_microsteps_per_sample == 0 || max_microsteps_per_sample > 256 {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    }
    let Some((_, axis, stepper)) = find_stepper(axes, stepper_oid) else {
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        return -1;
    };
    if axis.mode.load(Ordering::Acquire) != StepMode::Phase as u8 {
        return -3;
    }
    let pending_target = stepper.phase_offset_target.load(Ordering::Acquire);
    let pending_phase = phase_of(axis.last_step_count, pending_target);
    let delta = shortest_phase_delta(pending_phase, target_phase);
    stepper
        .phase_offset_target
        .store(pending_target.wrapping_add(delta), Ordering::Release);
    shared
        .max_phase_offset_ramp_per_sample
        .store(max_microsteps_per_sample, Ordering::Release);
    0
}

pub fn align_to(axes: &[Option<AxisState>], stepper_oid: u8, target_phase: u16) -> i32 {
    if i32::from(target_phase) >= PHASE_PERIOD {
        return -1;
    }
    let motion_active = axes
        .iter()
        .any(|a| a.as_ref().map_or(false, |ax| ax.armed.is_some()));
    if motion_active {
        return -2;
    }
    let Some((_, axis, stepper)) = find_stepper(axes, stepper_oid) else {
        return -1;
    };
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let current_phase = phase_of(axis.last_step_count, current);
    let new_offset =
        current.wrapping_add(shortest_phase_delta(current_phase, target_phase));
    stepper
        .phase_offset_microsteps
        .store(new_offset, Ordering::Release);
    stepper
        .phase_offset_target
        .store(new_offset, Ordering::Release);
    0
}

pub fn query(axes: &[Option<AxisState>], stepper_oid: u8) -> Option<PhaseQuery> {
    let (axis_idx, axis, stepper) = find_stepper(axes, stepper_oid)?;
    let current = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    let target = stepper.phase_offset_target.load(Ordering::Acquire);
    #[allow(clippy::cast_possible_truncation)]
    Some(PhaseQuery {
        axis_idx: axis_idx as u8,
        mode: axis.mode.load(Ordering::Acquire),
        phase: phase_of(axis.last_step_count, current),
        settled: current == target,
    })
}

#[cfg(test)]
mod tests;
```

Add `pub mod phase_handover;` to `rust/runtime/src/lib.rs`. If `raise_jog_parameters_invalid` is private to `fault_helpers`, make it `pub` (it is already used cross-module from `engine.rs:470`, so it likely is).

- [ ] **Step 4: Run tests to verify they pass**

Run from `rust/`: `cargo nextest run -p runtime -E 'test(phase_handover)'`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/phase_handover.rs rust/runtime/src/phase_handover/tests.rs rust/runtime/src/lib.rs
git commit -m "feat(runtime): phase handover primitives - jog_to, align_to, query by stepper oid"
```

---

### Task 2: Engine wrappers + FFI + header regeneration

**Files:**
- Modify: `rust/runtime/src/engine.rs` (next to `set_stepper_offset`, ~line 458)
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (next to `kalico_runtime_set_stepper_offset`, ~line 948)
- Regenerate: `rust/kalico-c-api/include/kalico_runtime.h`, `rust/kalico-c-api/include/kalico_nurbs.h`

- [ ] **Step 1: Add thin Engine wrappers in `rust/runtime/src/engine.rs`**

```rust
    pub fn phase_jog_to(
        &self,
        shared: &SharedState,
        stepper_oid: u8,
        target_phase: u16,
        max_microsteps_per_sample: u16,
    ) -> i32 {
        crate::phase_handover::jog_to(
            &self.stepping_axes,
            shared,
            stepper_oid,
            target_phase,
            max_microsteps_per_sample,
        )
    }

    pub fn phase_align_to(&self, stepper_oid: u8, target_phase: u16) -> i32 {
        crate::phase_handover::align_to(&self.stepping_axes, stepper_oid, target_phase)
    }

    pub fn phase_state(
        &self,
        stepper_oid: u8,
    ) -> Option<crate::phase_handover::PhaseQuery> {
        crate::phase_handover::query(&self.stepping_axes, stepper_oid)
    }
```

- [ ] **Step 2: Add FFI functions in `rust/kalico-c-api/src/runtime_ffi.rs`**

Mirror the `kalico_runtime_set_stepper_offset` pattern exactly (null/init checks, `RuntimeContext` projection, SAFETY comments in the same style):

```rust
    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_phase_jog_to(
        rt: *mut KalicoRuntime,
        stepper_oid: u8,
        target_phase: u16,
        max_microsteps_per_sample: u16,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only; &SharedState borrow is independent of &mut IsrState — SharedState is atomics-only.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let shared: &SharedState = &*core::ptr::addr_of!((*ctx).shared);
            (*isr_ptr).engine.phase_jog_to(
                shared,
                stepper_oid,
                target_phase,
                max_microsteps_per_sample,
            )
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_phase_align_to(
        rt: *mut KalicoRuntime,
        stepper_oid: u8,
        target_phase: u16,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only; §11.2 raw-pointer projection.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            (*isr_ptr).engine.phase_align_to(stepper_oid, target_phase)
        }
    }

    #[unsafe(no_mangle)]
    pub unsafe extern "C" fn kalico_runtime_get_phase_state(
        rt: *mut KalicoRuntime,
        stepper_oid: u8,
        out_axis_idx: *mut u8,
        out_mode: *mut u8,
        out_phase: *mut u16,
        out_settled: *mut u8,
    ) -> i32 {
        if rt.is_null() {
            return KALICO_ERR_NULL_PTR;
        }
        if out_axis_idx.is_null()
            || out_mode.is_null()
            || out_phase.is_null()
            || out_settled.is_null()
        {
            return KALICO_ERR_NULL_PTR;
        }
        if !INIT_DONE.load(Ordering::Acquire) {
            return KALICO_ERR_NOT_INIT;
        }
        let ctx = rt.cast::<RuntimeContext>();
        // SAFETY: foreground-only; §11.2 raw-pointer projection.
        unsafe {
            let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
            let Some(q) = (*isr_ptr).engine.phase_state(stepper_oid) else {
                return KALICO_ERR_INVALID_ARG;
            };
            *out_axis_idx = q.axis_idx;
            *out_mode = q.mode;
            *out_phase = q.phase;
            *out_settled = u8::from(q.settled);
        }
        KALICO_OK
    }
```

- [ ] **Step 3: Regenerate the C headers**

The headers are cbindgen-generated by `rust/kalico-c-api/src/bin/gen_headers.rs`. Run from `rust/`:

```bash
cargo run -p kalico-c-api --bin gen_headers
git diff --stat rust/kalico-c-api/include/
```

Expected: `kalico_runtime.h` (and possibly `kalico_nurbs.h`) gain the three new declarations. If the bin needs arguments, check its `main()` for usage — do not hand-edit the headers.

- [ ] **Step 4: Build + run the runtime and c-api suites**

Run from `rust/`: `cargo nextest run -p runtime -p kalico-c-api`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/engine.rs rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/
git commit -m "feat(c-api): expose phase handover primitives over FFI"
```

---

### Task 3: MCU commands in `src/stepper.c`

**Files:**
- Modify: `src/stepper.c` (immediately after `command_kalico_set_stepper_offset`, line ~311)

- [ ] **Step 1: Add the three command handlers**

The `oid` argument is the stepper's `config_stepper` OID (same value carried in the configure_axis blob, `src/stepper.c:177`). The response field is named `oid` so klippy's `lookup_query_command(oid=...)` response routing works.

```c
void
command_kalico_phase_jog_to(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_phase_jog_to before runtime init");
    uint8_t stepper_oid = args[0];
    uint16_t target_phase = args[1];
    uint16_t max_per_sample = args[2];
    int32_t rc = kalico_runtime_phase_jog_to(
        runtime_handle, stepper_oid, target_phase, max_per_sample);
    if (rc != 0)
        shutdown("kalico_phase_jog_to rejected (bad args or not in phase mode)");
}
DECL_COMMAND(command_kalico_phase_jog_to,
             "kalico_phase_jog_to oid=%c target_phase=%hu"
             " max_microsteps_per_sample=%hu");

void
command_kalico_phase_align_to(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_phase_align_to before runtime init");
    uint8_t stepper_oid = args[0];
    uint16_t target_phase = args[1];
    int32_t rc = kalico_runtime_phase_align_to(
        runtime_handle, stepper_oid, target_phase);
    if (rc != 0)
        shutdown("kalico_phase_align_to rejected (motion in progress or bad args)");
}
DECL_COMMAND(command_kalico_phase_align_to,
             "kalico_phase_align_to oid=%c target_phase=%hu");

void
command_kalico_get_phase_state(uint32_t *args)
{
    if (!runtime_handle)
        shutdown("kalico_get_phase_state before runtime init");
    uint8_t stepper_oid = args[0];
    uint8_t axis_idx = 0, mode = 0, settled = 0;
    uint16_t phase = 0;
    int32_t rc = kalico_runtime_get_phase_state(
        runtime_handle, stepper_oid, &axis_idx, &mode, &phase, &settled);
    if (rc != 0)
        shutdown("kalico_get_phase_state unknown stepper oid");
    sendf("kalico_phase_state oid=%c axis_idx=%c mode=%c phase=%hu settled=%c",
          stepper_oid, axis_idx, mode, phase, settled);
}
DECL_COMMAND(command_kalico_get_phase_state,
             "kalico_get_phase_state oid=%c");
```

The new FFI prototypes come from the regenerated `kalico_runtime.h` (Task 2); confirm `src/stepper.c` includes it (the existing `kalico_runtime_set_stepper_offset` call proves it does).

- [ ] **Step 2: Compile-check a host-testable MCU build**

The fastest syntax gate without cross-toolchains is the Linux MCU target:

```bash
make clean
make menuconfig 2>/dev/null || true   # skip; use a stored config instead:
cp test/configs/linux.config .config 2>/dev/null || echo "use an existing CI config for linux mcu"
make olddefconfig && make -j$(sysctl -n hw.ncpu)
```

If no linux config exists in `test/configs/`, fall back to whichever config CI uses for the kalico-sim image (the kalico-sim skill documents the sim build). Expected: compiles with no warnings about implicit declarations of the three `kalico_runtime_phase_*` symbols.

- [ ] **Step 3: Commit**

```bash
git add src/stepper.c
git commit -m "feat(mcu): kalico_phase_jog_to / align_to / get_phase_state commands"
```

---

### Task 4: TMC5160 driver — re-runnable enter/exit of phase mode

**Files:**
- Modify: `klippy/extras/tmc5160.py:388-556` (class `TMC5160`)

- [ ] **Step 1: Add state and the oid setter in `TMC5160.__init__`**

After line 405 (`self._phase_cs_pin_id = None`) add:

```python
        self._phase_stepper_oid = None
        self._phase_axis_idx = None
        self._cached_mscnt = None
        self._phase_mode_active = False
        self._phase_state_query = None
```

And rename the registered post-enable callback (line 420) from `self._xdirect_preload` to `self.enter_phase_mode`.

Add methods:

```python
    def set_phase_stepper_oid(self, oid):
        self._phase_stepper_oid = oid

    def phase_stepping_active(self):
        return self._phase_mode_active
```

- [ ] **Step 2: Refactor `_xdirect_preload` into `enter_phase_mode` + command plumbing**

Replace the `_xdirect_preload` method (lines 495-556) with:

```python
    def _phase_mcu(self):
        return self.mcu_tmc.tmc_spi.spi.get_mcu()

    def _lookup_phase_commands(self):
        mcu_obj = self._phase_mcu()
        if self._phase_stepper_oid is None:
            raise self.printer.command_error(
                "phase_stepping: stepper oid not registered for %s "
                "(motion_toolhead init_planner did not run?)" % (self.name,)
            )
        enable_spi = mcu_obj.lookup_command("kalico_phase_stepping_enable_spi")
        disable_spi = mcu_obj.lookup_command("kalico_phase_stepping_disable_spi")
        set_axis_mode = mcu_obj.lookup_command(
            "kalico_set_axis_mode axis_idx=%c mode=%c"
        )
        jog = mcu_obj.lookup_command(
            "kalico_phase_jog_to oid=%c target_phase=%hu"
            " max_microsteps_per_sample=%hu"
        )
        align = mcu_obj.lookup_command(
            "kalico_phase_align_to oid=%c target_phase=%hu"
        )
        if self._phase_state_query is None:
            self._phase_state_query = mcu_obj.lookup_query_command(
                "kalico_get_phase_state oid=%c",
                "kalico_phase_state oid=%c axis_idx=%c mode=%c phase=%hu"
                " settled=%c",
                oid=self._phase_stepper_oid,
            )
        return enable_spi, disable_spi, set_axis_mode, jog, align

    def _query_phase_state(self):
        params = self._phase_state_query.send([self._phase_stepper_oid])
        return params

    def enter_phase_mode(self):
        enable_spi, disable_spi, set_axis_mode, _jog, align = (
            self._lookup_phase_commands()
        )
        # Suppress ISR XDIRECT writes during our foreground SPI traffic
        # (the disable command is idempotent; harmless if already disabled).
        disable_spi.send([])
        # Write CHOPCONF (toff>0) first, then set GCONF.direct_mode=1.
        # direct_mode is deliberately NOT in the field cache (removed from
        # _enable_direct_mode) so _init_registers doesn't write it while
        # the chip still has toff=0 from the virtual-enable disable phase.
        # The bootstrap charge pump depends on the chopper switching —
        # direct_mode with toff=0 drains the bootstrap caps and triggers
        # uv_cp after a few moves.
        chopconf_val = self.fields.registers.get("CHOPCONF")
        if chopconf_val is not None:
            self.mcu_tmc.set_register("CHOPCONF", chopconf_val)
        gconf_val = self.fields.registers.get("GCONF", 0)
        gconf_val |= 1 << 16  # direct_mode
        gconf_val &= ~(1 << 2)  # SpreadCycle (clear en_pwm_mode)
        self.mcu_tmc.set_register("GCONF", gconf_val)
        self.fields.registers["GCONF"] = gconf_val
        mscnt = self.mcu_tmc.get_register("MSCNT") & 0x3FF
        self._cached_mscnt = mscnt
        angle = mscnt * 2.0 * math.pi / 1024.0
        coil_a = int(round(248.0 * math.cos(angle)))
        coil_b = int(round(248.0 * math.sin(angle)))
        xdirect_val = ((coil_b & 0xFFFF) << 16) | (coil_a & 0xFFFF)
        self.mcu_tmc.set_register("XTARGET", xdirect_val)
        logging.info(
            "TMC5160 XDIRECT preload: mscnt=%d coil_a=%d coil_b=%d raw=0x%08x",
            mscnt,
            coil_a,
            coil_b,
            xdirect_val,
        )
        state = self._query_phase_state()
        self._phase_axis_idx = state["axis_idx"]
        align.send([self._phase_stepper_oid, mscnt])
        enable_spi.send([])
        set_axis_mode.send([self._phase_axis_idx, 1])
        # Stop the periodic DRV_STATUS/GSTAT checks while the ISR is
        # writing XDIRECT. The ISR's inline SPI manipulates the SPI
        # peripheral registers directly — foreground register reads
        # during ISR activity return corrupted data (e.g., GSTAT reads
        # as 0x010a0023 instead of a valid 3-bit value), triggering
        # false drv_err/uv_cp shutdowns. DMA-based SPI (Phase 2) will
        # fix the arbitration; until then, suppress the checks.
        self._echeck_helper.stop_checks()
        self._phase_mode_active = True
        logging.info("TMC5160 %s: phase mode entered", self.name)
```

Note `self.name` — check the attribute exists on `TMC5160` (the current helper has it via `BaseTMCCurrentHelper`); if the printer object lacks one, use `" ".join(...)` of the config name as in `__init__` and store it.

- [ ] **Step 3: Add `exit_phase_mode`**

```python
    PHASE_JOG_MAX_PER_SAMPLE = 1
    PHASE_SETTLE_TIMEOUT = 0.5

    def exit_phase_mode(self):
        if not self._phase_mode_active:
            raise self.printer.command_error(
                "exit_phase_mode called but %s is not in phase mode"
                % (self.name,)
            )
        _enable_spi, disable_spi, set_axis_mode, jog, _align = (
            self._lookup_phase_commands()
        )
        state = self._query_phase_state()
        if state["mode"] != 1:
            raise self.printer.command_error(
                "phase mode bookkeeping desync on %s: host=phase mcu=%d"
                % (self.name, state["mode"])
            )
        jog.send(
            [
                self._phase_stepper_oid,
                self._cached_mscnt,
                self.PHASE_JOG_MAX_PER_SAMPLE,
            ]
        )
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + self.PHASE_SETTLE_TIMEOUT
        while True:
            state = self._query_phase_state()
            if state["settled"] and state["phase"] == self._cached_mscnt:
                break
            if reactor.monotonic() > deadline:
                raise self.printer.command_error(
                    "phase handover jog did not settle on %s "
                    "(phase=%d target=%d)"
                    % (self.name, state["phase"], self._cached_mscnt)
                )
            reactor.pause(reactor.monotonic() + 0.005)
        disable_spi.send([])
        gconf_val = self.fields.registers.get("GCONF", 0)
        gconf_val &= ~(1 << 16)  # clear direct_mode
        self.mcu_tmc.set_register("GCONF", gconf_val)
        self.fields.registers["GCONF"] = gconf_val
        set_axis_mode.send([self._phase_axis_idx, 0])
        self._echeck_helper.start_checks()
        self._phase_mode_active = False
        logging.info("TMC5160 %s: phase mode exited (pulse stepping)", self.name)
```

Sequencing constraints encoded above (do not reorder):
- The jog must run while ISR SPI writes are still enabled — the slew happens through `dispatch_phase` XDIRECT writes.
- `disable_spi` before the foreground GCONF write (SPI bus arbitration).
- Clearing `direct_mode` lands while the rotor sits exactly on `LUT_hw[MSCNT]` (that is what the jog just guaranteed), so the chip's chopper takes over with zero current discontinuity.
- `set_axis_mode(…, 0)` uses encoding 0 = Pulse.

- [ ] **Step 4: Verify python syntax and the suite still passes**

```bash
python3 -m py_compile klippy/extras/tmc5160.py
cd rust && cargo nextest run -p runtime -p kalico-c-api && cd ..
```

Expected: no output from py_compile; tests PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/tmc5160.py
git commit -m "feat(tmc5160): re-runnable enter/exit of phase stepping with MSCNT handover"
```

---

### Task 5: Hook the switch into sensorless homing (`TMCVirtualPinHelper`)

**Files:**
- Modify: `klippy/extras/tmc.py:587-691` (class `TMCVirtualPinHelper`)
- Modify: `klippy/extras/tmc5160.py` (`TMC5160.__init__`, line 397)

- [ ] **Step 1: Give the helper a phase-mode hook**

In `TMCVirtualPinHelper.__init__` (after `self.mcu_endstop = None`, line ~601) add:

```python
        self.phase_mode_helper = None
        self._phase_exited = False
```

In `arm()` (line 640), add at the very top, before the SGTHRS block:

```python
        pmh = self.phase_mode_helper
        if pmh is not None and pmh.phase_stepping_active():
            pmh.exit_phase_mode()
            self._phase_exited = True
            if pmh.phase_stepping_active():
                raise self.printer.command_error(
                    "phase stepping still active after exit_phase_mode; "
                    "refusing to start a StallGuard homing move"
                )
```

In `disarm()` (line 673), add at the very bottom, after the THIGH restore:

```python
        if self._phase_exited:
            self._phase_exited = False
            self.phase_mode_helper.enter_phase_mode()
```

Ordering rationale (encoded, not commented): `exit_phase_mode` runs first so the rest of `arm()` mutates a GCONF cache that already has `direct_mode` cleared; `enter_phase_mode` runs last in `disarm()` so it sets `direct_mode` and clears `en_pwm_mode` after the stealthchop restore wrote its own GCONF value. `self._phase_exited` is set only after a successful exit, so an exception mid-exit propagates out of `trip_move_begin` without a matching re-enter on a half-switched driver (fail loudly; the driver is left in or near Pulse mode, the safe mode).

- [ ] **Step 2: Wire the TMC5160 into its virtual-pin helper**

In `TMC5160.__init__`, line 397, capture the helper:

```python
        self._virtual_pin_helper = tmc.TMCVirtualPinHelper(config, self.mcu_tmc)
```

And after the phase-stepping config block (after line 410, `self._phase_stepping = True`):

```python
            self._virtual_pin_helper.phase_mode_helper = self
```

- [ ] **Step 3: Syntax check**

```bash
python3 -m py_compile klippy/extras/tmc.py klippy/extras/tmc5160.py
```

Expected: silent.

- [ ] **Step 4: Commit**

```bash
git add klippy/extras/tmc.py klippy/extras/tmc5160.py
git commit -m "feat(homing): switch phase-stepped TMC5160 to pulse mode around StallGuard trip moves"
```

---

### Task 6: Push the stepper OID into the TMC driver (`motion_toolhead.py`)

**Files:**
- Modify: `klippy/motion_toolhead.py` (the `phase_configs` build loop, ~line 690-717)

- [ ] **Step 1: Add the OID push**

In the loop `for stepper_name, stepper_obj in slot:` after the `get_phase_config()` call (`bus_id, cs_pin_id = tmc.get_phase_config()`), add:

```python
                    tmc.set_phase_stepper_oid(stepper_obj.get_oid())
```

`stepper_obj.get_oid()` is already used for `bind_list` a few lines up, so the method exists on these objects. The TMC lookup above this line already hard-errors when the driver is not a TMC5160 with phase support, so no `hasattr` guard.

- [ ] **Step 2: Syntax check + commit**

```bash
python3 -m py_compile klippy/motion_toolhead.py
git add klippy/motion_toolhead.py
git commit -m "feat(motion): register stepper oid with TMC5160 for phase handover commands"
```

---

### Task 7: Full suite + format gate

- [ ] **Step 1: Run the full Rust suite**

From `rust/`: `cargo nextest run`
Expected: PASS (~11s). If anything regressed in motion-bridge tests that snapshot the command table, update those snapshots only after confirming the new commands are the sole diff.

- [ ] **Step 2: Format check (LAST step before any push, re-run after late edits)**

From `rust/`: `cargo fmt --all --check`
Expected: clean. If not, `cargo fmt --all`, re-run tests, amend.

- [ ] **Step 3: Commit any stragglers**

```bash
git status --short   # expect clean
```

---

### Task 8: Simulator validation (kalico-sim)

- [ ] **Step 1: Invoke the `kalico-sim` skill** to run this scenario end-to-end:

Scenario: a config with `phase_stepping: true` + `microsteps: 256` on stepper_x, a `[tmc5160 stepper_x]` section with `diag0_pin` and `endstop_pin: tmc5160_stepper_x:virtual_endstop`, then `G28 X` with the sim tripping the endstop.

Acceptance criteria (assert via the sim's MCU command log / query-logs skill on the sim's structured logs):
1. Before `home_axis_start`: `kalico_phase_jog_to` (target = the MSCNT cached at enable), followed by `kalico_phase_stepping_disable_spi`, a GCONF write clearing bit 16, and `kalico_set_axis_mode … mode=0`.
2. After the trip: `kalico_set_axis_mode … mode=1` preceded by `kalico_phase_align_to` with `target_phase` equal to the freshly read MSCNT.
3. The reconstructed homed position matches the trip position exactly (reuse the assert pattern from the existing cross-MCU homing sim test, commit 53b5fc56b).
4. No MCU shutdown, no `jog_parameters_invalid` fault.

If the sim's TMC5160 model cannot produce a meaningful MSCNT, assert the command *sequence* (criteria 1, 2 ordering) and criterion 3 with MSCNT=whatever the model returns — sequence and position invariance are the load-bearing checks.

- [ ] **Step 2: Commit the sim test**

```bash
git add <sim test files>
git commit -m "test(sim): phase stepping mode switch around sensorless homing"
```

---

### Task 9: Bench validation (Trident) — gated, manual

Not automatable from this plan; execute with the user in the loop:

- [ ] Flash both MCUs via the `flashing-trident-mcus` skill (commit → push → pull → build → flash; H7 + F446, `make clean` between C builds).
- [ ] With the user's explicit per-command permission (hard rule: no G-code without a per-command "yes"): sensorless `G28` on the phase-stepped axis. Verify via query-logs: DIAG trip registered, no shutdown, no hiccups (mcu-diagnostics skill), and homed-position repeatability across several homes.
- [ ] Listen/feel check at each transition: no clunk on exit (the ≤2-full-step slew is gentle at 1 microstep/sample) and none on re-entry (must be motion-free by construction).
