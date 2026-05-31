//! Per-axis Klipper `SysTick` consumer. Mainline pattern: fire one entry
//! per dispatch when its `cycle_abs` has arrived; never early.
//!
//! Body called from C-side `struct timer.func` via `extern "C"`.
//!
//! Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! (Task 10 of 2026-05-19-stepping-redesign-implementation.md).
//!
//! Lifecycle:
//! 1. C-side `arm_per_axis_step_timer` installs a `struct timer` for each axis
//!    this MCU drives (as that axis is configured), bound to a thin C
//!    trampoline (X=0, Y=1, Z=2, E=3) that calls into this module. Axes the
//!    MCU does not own get no timer (avoids needless sample-rate dispatch).
//! 2. Each dispatch peeks the head of `step_queues[axis_idx]` and, if its
//!    `cycle_abs` is at-or-before `now`, pops + emits one step pulse via
//!    `runtime_emit_step_pulses(axis_idx, dir)`.
//! 3. The returned `u32` is the next waketime: the next entry's `cycle_abs`
//!    (floored by `dispatcher_floor_cycles` to prevent runaway re-entry), or
//!    `now + idle_park_cycles` (a long ~100 ms / 10 Hz fallback) if the
//!    queue is empty. The empty queue is NOT polled at the sample rate; the
//!    producer (TIM5 ISR) kicks this timer forward on the idle→active
//!    transition (`kalico_kick_per_axis_timer`).
//!
//! Coexists alongside the legacy `step_time_event` / `runtime_producer_event`
//! path until Task 16 removes the older code.

#![allow(unsafe_code)]

use crate::step_queue::{peek as queue_peek, pop as queue_pop};

// MCU build links these C symbols (defined in src/generic/armcm_timer.c and
// src/stepper.c). Host builds (`feature = "host"`) and unit tests must not
// pull undefined symbols into the cdylib — `cargo build -p motion-bridge`
// produces `motion_bridge_native.so`, and `dlopen` would fail at klippy
// import time if these extern "C" decls had no matching definition. Provide
// inert host stubs under the same gate the rest of this module uses for
// `resolve_queue_ptr`.
#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn timer_read_time() -> u32;
    fn timer_is_before(a: u32, b: u32) -> u8;
    fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32);
}

#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn kalico_runtime_get_dispatcher_floor_cycles() -> u32;
    fn kalico_runtime_get_sample_period_cycles() -> u32;
    // C-side getter (src/runtime_tick.c): `timer_from_us(100000)` — the long
    // (~100 ms / 10 Hz) fallback an idle axis parks at when its queue is
    // empty. The producer (TIM5 ISR) kicks the timer forward on the
    // idle→active transition (see `kalico_kick_per_axis_timer`), so this is
    // only a safety net, never the steady-motion cadence. Lives in C because
    // the µs→cycles conversion (`timer_from_us`) is a per-MCU C primitive.
    fn kalico_runtime_get_idle_park_cycles() -> u32;
}

#[cfg(any(test, feature = "host"))]
unsafe fn timer_read_time() -> u32 {
    0
}
#[cfg(any(test, feature = "host"))]
unsafe fn timer_is_before(_a: u32, _b: u32) -> u8 {
    0
}
#[cfg(any(test, feature = "host"))]
unsafe fn runtime_emit_step_pulses(_axis_idx: u8, _n_steps: i32) {}
#[cfg(any(test, feature = "host"))]
unsafe fn kalico_runtime_get_dispatcher_floor_cycles() -> u32 {
    0
}
#[cfg(any(test, feature = "host"))]
unsafe fn kalico_runtime_get_sample_period_cycles() -> u32 {
    0
}
#[cfg(any(test, feature = "host"))]
unsafe fn kalico_runtime_get_idle_park_cycles() -> u32 {
    0
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
    // entry point sound even before `arm_per_axis_step_timer` would have
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
                unsafe { runtime_emit_step_pulses(axis_idx, i32::from(entry.dir)) };
            }
        }
    }

    // Next waketime: prefer the next pending entry's `cycle_abs`, floored
    // by `dispatcher_floor_cycles` to prevent runaway re-entry; if the
    // queue is empty, PARK at the long idle fallback (~100 ms). The empty
    // queue is no longer polled at the sample rate — the producer (TIM5
    // ISR) kicks this timer forward via `kalico_kick_per_axis_timer` the
    // moment it pushes the first step into a previously-empty queue, so the
    // park is only a safety net (10 Hz), never the cadence that drives
    // motion. This removes the idle-axis sample-rate dispatch load that
    // starved the motion tick (-311 TickIntervalExceeded).
    // SAFETY: both accessors are read-only AtomicU32 loads on `SharedState`
    // (via `runtime_handle_or_null`); `kalico_runtime_get_idle_park_cycles`
    // is a side-effect-free C `timer_from_us` conversion. They return 0 if
    // the runtime hasn't initialised yet; a 0 floor means "no extra
    // padding," and a 0 idle park means "wake `now`," which the next
    // dispatch will immediately reschedule (degrades safely).
    let floor_cycles = unsafe { kalico_runtime_get_dispatcher_floor_cycles() };
    let idle_park = unsafe { kalico_runtime_get_idle_park_cycles() };
    let floor_time = now.wrapping_add(floor_cycles);
    let idle_park_time = now.wrapping_add(idle_park);

    if queue_ptr.is_null() {
        return idle_park_time;
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
        None => idle_park_time,
    }
}

/// MCU build: resolve the queue pointer from the C-declared
/// `step_queues[N_AXIS_STEP_QUEUES]` array. Bounds-checked by the caller
/// (axis_idx ∈ 0..=3 is implicit from the four C trampolines).
#[cfg(not(any(test, feature = "host")))]
fn resolve_queue_ptr(axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    use crate::step_queue::{StepQueue, step_queues};
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
