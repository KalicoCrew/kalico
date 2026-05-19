//! Per-axis Klipper SysTick consumer. Mainline pattern: fire one entry
//! per dispatch when its cycle_abs has arrived; never early.
//!
//! Body called from C-side `struct timer.func` via `extern "C"`.
//!
//! Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! (Task 10 of 2026-05-19-stepping-redesign-implementation.md).
//!
//! Lifecycle:
//! 1. C-side `init_per_axis_step_timers` installs four `struct timer`s, one
//!    per axis (X=0, Y=1, Z=2, E=3), each bound to a thin C trampoline that
//!    calls into this module.
//! 2. Each dispatch peeks the head of `step_queues[axis_idx]` and, if its
//!    `cycle_abs` is at-or-before `now`, pops + emits one step pulse via
//!    `runtime_emit_step_pulses(axis_idx, dir)`.
//! 3. The returned `u32` is the next waketime: the next entry's `cycle_abs`
//!    (floored by `dispatcher_floor_cycles` to prevent runaway re-entry), or
//!    `now + sample_period_cycles` if the queue is empty.
//!
//! Coexists alongside the legacy `step_time_event` / `runtime_producer_event`
//! path until Task 16 removes the older code.

#![allow(unsafe_code)]

use crate::step_queue::{peek as queue_peek, pop as queue_pop};

unsafe extern "C" {
    fn timer_read_time() -> u32;
    fn timer_is_before(a: u32, b: u32) -> u8;
    fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32);
}

unsafe extern "C" {
    fn kalico_runtime_get_dispatcher_floor_cycles() -> u32;
    fn kalico_runtime_get_sample_period_cycles() -> u32;
}

/// Rust body for the per-axis `struct timer.func` callback. Called from C
/// trampolines (one per axis 0..=3) in `src/runtime_tick.c`. Returns the
/// next waketime (u32 cycle absolute) that the C wrapper writes back to
/// `t->waketime`.
///
/// Mainline pattern: one entry per dispatch, never fire early.
#[unsafe(no_mangle)]
pub extern "C" fn kalico_per_axis_step_event(axis_idx: u8) -> u32 {
    // SAFETY: `timer_read_time` is a single u32 read of an MMIO timer
    // register on the MCU (or a software counter in host builds). Safe to
    // call from any non-reentrant context; the per-axis timer dispatch is
    // serialised with itself by Klipper's SysTick scheduler.
    let now = unsafe { timer_read_time() };
    let queue_ptr = resolve_queue_ptr(axis_idx as usize);

    // Pop one entry if its `cycle_abs` has arrived. Guard against a null
    // queue pointer (host builds and pre-Task-11 boot states) to keep this
    // entry point sound even before `init_per_axis_step_timers` would have
    // populated the C-side `step_queues` array.
    if !queue_ptr.is_null() {
        // SAFETY: `queue_ptr` is non-null, points at a live `StepQueue` for
        // the duration of the program (storage is the C-declared
        // `step_queues[N_AXIS_STEP_QUEUES]` placed in `.axi_bss`), and this
        // timer is the sole consumer for axis `axis_idx`.
        if let Some(entry) = unsafe { queue_peek(queue_ptr) } {
            // SAFETY: `timer_is_before` is pure (Klipper helper) — see
            // `src/board/misc.h`. `now`/`entry.cycle_abs` are plain u32s.
            let arrived = unsafe { timer_is_before(now, entry.cycle_abs) } == 0;
            if arrived {
                // SAFETY: same as peek above — sole consumer discipline.
                let _ = unsafe { queue_pop(queue_ptr) };
                // SAFETY: `runtime_emit_step_pulses` is the C-side step
                // emitter (`src/stepper.c`); a NOP-on-out-of-range guard
                // covers `axis_idx >= RUNTIME_MOTOR_COUNT`.
                unsafe { runtime_emit_step_pulses(axis_idx, entry.dir as i32) };
            }
        }
    }

    // Next waketime: prefer the next pending entry's `cycle_abs`, floored
    // by `dispatcher_floor_cycles` to prevent runaway re-entry; if the
    // queue is empty, sleep until the next sample boundary.
    // SAFETY: both accessors are read-only AtomicU32 loads on `SharedState`
    // (via `runtime_handle_or_null`); they return 0 if the runtime hasn't
    // initialised yet. `0` for either tunable degrades safely: a 0 floor
    // means "no extra padding," and a 0 sample period means "wake `now`,"
    // which the next dispatch will immediately reschedule.
    let floor_cycles = unsafe { kalico_runtime_get_dispatcher_floor_cycles() };
    let sample_period = unsafe { kalico_runtime_get_sample_period_cycles() };
    let floor_time = now.wrapping_add(floor_cycles);
    let next_sample = now.wrapping_add(sample_period);

    if queue_ptr.is_null() {
        return next_sample;
    }

    // SAFETY: `queue_ptr` non-null + sole-consumer as above.
    match unsafe { queue_peek(queue_ptr) } {
        Some(next) => {
            // max(next.cycle_abs, floor_time), wrap-aware via
            // `timer_is_before`. If `next.cycle_abs` is already past
            // `floor_time`, schedule for the entry's exact arrival; else
            // push the wake out to the floor to avoid spinning.
            // SAFETY: pure helper, see above.
            if unsafe { timer_is_before(next.cycle_abs, floor_time) } != 0 {
                floor_time
            } else {
                next.cycle_abs
            }
        }
        None => next_sample,
    }
}

/// MCU build: resolve the queue pointer from the C-declared
/// `step_queues[N_AXIS_STEP_QUEUES]` array. Bounds-checked by the caller
/// (axis_idx ∈ 0..=3 is implicit from the four C trampolines).
#[cfg(not(any(test, feature = "host")))]
fn resolve_queue_ptr(axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    use crate::step_queue::{step_queues, StepQueue};
    // SAFETY: `step_queues` is the C-declared array, `.add(axis_idx)` is
    // in-bounds for axis_idx ∈ 0..N_AXIS_STEP_QUEUES (caller invariant).
    unsafe { step_queues.get().cast::<StepQueue>().add(axis_idx) }
}

/// Host / test build: there is no C-declared `step_queues` to project from
/// — return null and let `kalico_per_axis_step_event` fall through its
/// null-check guards. Host-side smoke tests for the timer body would need
/// to mock through a host-only hook; deferred to Task 18 / bench bring-up.
#[cfg(any(test, feature = "host"))]
fn resolve_queue_ptr(_axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    core::ptr::null_mut()
}
