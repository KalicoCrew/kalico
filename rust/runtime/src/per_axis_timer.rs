//! Step-output timer consumer (motion-tick priority-lift Step 1).
//!
//! A single dedicated hardware timer (TIM3 on H7, TIM2 on F4 — see
//! `src/stm32/runtime_tick_h7.c` / `runtime_tick_f4.c`) fires this body. The
//! body scans the per-axis SPSC `step_queues[]` this MCU owns, emits every
//! step whose `cycle_abs` has arrived (bounded per dispatch), and returns the
//! soonest *absolute* cycle the timer must next fire at — or
//! [`STEP_OUTPUT_DISABLE`] when no owned queue has pending work, telling the C
//! side to switch the timer off (event-driven, NOT idle-polling).
//!
//! Spec: docs/superpowers/specs/2026-05-31-motion-tick-priority-lift-design.md
//! (Step 1: dedicated step-output timer at PARITY priority — same NVIC
//! priority as the TIM5 producer and SysTick; no priority flip in this step).
//!
//! ## Replaces the per-axis SysTick consumer
//!
//! Previously there was one Klipper `struct timer` per axis on the SysTick
//! software-timer queue, each calling `kalico_per_axis_step_event(axis)`. That
//! substrate is gone: the four trampolines, `arm_per_axis_step_timer`'s
//! `sched_add_timer` body, and `kalico_kick_per_axis_timer` are all removed
//! from `src/runtime_tick.c`. Emission now lives on one hardware timer.
//!
//! ## Load-bearing invariant: same NVIC priority
//!
//! The producer (TIM5 ISR, `tick.rs`) and this consumer (the step-output timer
//! ISR) run at the *same* NVIC priority. On a single ARMv7-M core with
//! PRIGROUP = 0, same-priority interrupts cannot preempt each other, so the
//! `step_queues` u16/volatile SPSC stays non-racing without atomics, and the
//! producer's `kalico_kick_step_output` compare-register poke is non-racing
//! against this body's own re-arm. If a future change ever makes producer and
//! consumer DIFFERENT priorities, both the SPSC and the kick must move to a
//! true preemption-safe scheme (see `step_queue.rs`).

#![allow(unsafe_code)]

use crate::step_queue::{N_AXIS_STEP_QUEUES, peek as queue_peek, pop as queue_pop};

/// Sentinel returned by [`kalico_step_output_event`] when no owned queue has a
/// pending step: the C side disables the step-output timer (no idle poll). The
/// producer kick (`kalico_kick_step_output`) re-arms it on the next push into a
/// previously-empty owned queue. `u32::MAX` is safe as a sentinel because a
/// real `cycle_abs` landing exactly on `u32::MAX` is reinterpreted by the C
/// side as "disable", costing at most one delayed step (the next push re-arms).
pub const STEP_OUTPUT_DISABLE: u32 = u32::MAX;

/// A step whose `cycle_abs` is within this signed window of `now` (at-or-before,
/// plus a tiny forward slack) is emitted on this dispatch. Mirrors the
/// mainline "fire when arrived, never early" rule: the window is 0 so a step is
/// emitted only once `now >= cycle_abs` (wrap-safe signed compare). Kept as a
/// named constant so the bench can widen it if hardware compare latency ever
/// demands a small lead.
const DUE_WINDOW_CYCLES: i32 = 0;

/// Maximum steps emitted across all owned axes in a single dispatch. Bounds ISR
/// duration; if the cap is hit with work remaining, the body returns `now` so
/// the C side re-fires immediately (same semantics as the old per-axis path's
/// floor re-entry).
pub const MAX_STEPS_PER_EVENT: u32 = 32;

// MCU build links these C symbols. Host builds (`feature = "host"`) and unit
// tests must not pull undefined symbols into the cdylib, so they are stubbed.
#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn timer_read_time() -> u32;
    fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32);
    // C-side getter (src/runtime_tick.c): bitmask of axes this MCU owns
    // (drives). Bit N set ⇒ axis N participates in the soonest-across scan.
    // Self-populated by the producer kick the first time an axis is pushed.
    fn kalico_step_output_owned_mask() -> u8;
}

#[cfg(any(test, feature = "host"))]
pub use test_hooks::{owned_mask as host_owned_mask, set_now, set_owned_mask};

#[cfg(any(test, feature = "host"))]
unsafe fn timer_read_time() -> u32 {
    test_hooks::now()
}
#[cfg(any(test, feature = "host"))]
unsafe fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32) {
    test_hooks::record_emit(axis_idx, n_steps);
}
#[cfg(any(test, feature = "host"))]
unsafe fn kalico_step_output_owned_mask() -> u8 {
    test_hooks::owned_mask()
}

/// Step-output timer ISR body. Returns the next absolute cycle to fire at, or
/// [`STEP_OUTPUT_DISABLE`] to switch the timer off.
///
/// Called from the C `STEP_OUT_TIM_IRQHandler` via `extern "C"`. The C side
/// arms its compare register to the returned value (with 16-bit chaining on
/// H7's TIM3, where the far re-arm is split into ≤0xF000 chunks).
#[unsafe(no_mangle)]
pub extern "C" fn kalico_step_output_event() -> u32 {
    // SAFETY: `timer_read_time` is a single u32 MMIO read (host: a test hook).
    let now = unsafe { timer_read_time() };
    // SAFETY: side-effect-free C getter (host: a test hook).
    let owned = unsafe { kalico_step_output_owned_mask() };
    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_ENTER);

    let mut emitted: u32 = 0;
    // Emit due steps across owned axes until the per-dispatch cap is hit or no
    // owned queue has a due head left.
    'outer: loop {
        let mut emitted_this_pass = false;
        for axis_idx in 0..N_AXIS_STEP_QUEUES {
            if owned & (1u8 << axis_idx) == 0 {
                continue;
            }
            if emitted >= MAX_STEPS_PER_EVENT {
                break 'outer;
            }
            let q = resolve_queue_ptr(axis_idx);
            if q.is_null() {
                continue;
            }
            // SAFETY: `q` is non-null (checked) and points at a live StepQueue;
            // this body is the sole consumer (same-priority invariant).
            crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_POP);
            let Some(entry) = (unsafe { queue_peek(q) }) else {
                continue;
            };
            // Wrap-safe arrival test: emit once `now` has reached `cycle_abs`.
            let delta = entry.cycle_abs.wrapping_sub(now) as i32;
            if delta <= DUE_WINDOW_CYCLES {
                // SAFETY: sole-consumer discipline as above.
                let _ = unsafe { queue_pop(q) };
                // SAFETY: C step emitter guards out-of-range motor indices.
                crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_EMIT);
                unsafe { runtime_emit_step_pulses(axis_idx as u8, i32::from(entry.dir)) };
                emitted += 1;
                emitted_this_pass = true;
            }
        }
        if !emitted_this_pass {
            break;
        }
    }

    // If the cap stopped us with work still due, re-fire immediately.
    if emitted >= MAX_STEPS_PER_EVENT {
        crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_EXIT);
        return now;
    }

    // Otherwise return the soonest remaining head across owned axes, wrap-safe.
    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_STEPOUT_EXIT);
    next_wake_across_owned(now, owned).unwrap_or(STEP_OUTPUT_DISABLE)
}

/// Soonest (wrap-safe-minimum) `cycle_abs` of the head of every owned non-empty
/// queue, or `None` if all owned queues are empty. "Soonest" is the entry whose
/// signed delta from `now` is smallest, so a head just past a u32 wrap boundary
/// still compares correctly against one just before it.
fn next_wake_across_owned(now: u32, owned: u8) -> Option<u32> {
    let mut best: Option<(i32, u32)> = None;
    for axis_idx in 0..N_AXIS_STEP_QUEUES {
        if owned & (1u8 << axis_idx) == 0 {
            continue;
        }
        let q = resolve_queue_ptr(axis_idx);
        if q.is_null() {
            continue;
        }
        // SAFETY: non-null + sole-consumer as above.
        if let Some(entry) = unsafe { queue_peek(q) } {
            let delta = entry.cycle_abs.wrapping_sub(now) as i32;
            match best {
                Some((best_delta, _)) if delta >= best_delta => {}
                _ => best = Some((delta, entry.cycle_abs)),
            }
        }
    }
    best.map(|(_, cycle_abs)| cycle_abs)
}

/// MCU build: resolve the C-declared `step_queues[N_AXIS_STEP_QUEUES]` pointer.
#[cfg(not(any(test, feature = "host")))]
fn resolve_queue_ptr(axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    crate::step_queue::queue_for_axis(axis_idx)
}

/// Host / test build: queues live in a test-local array (see `test_hooks`).
#[cfg(any(test, feature = "host"))]
fn resolve_queue_ptr(axis_idx: usize) -> *mut crate::step_queue::StepQueue {
    test_hooks::queue_for_axis(axis_idx)
}

// ─── Host/test hooks ──────────────────────────────────────────────────────
//
// The MCU resolves `step_queues`, `timer_read_time`, the owned mask and the
// emitter through the C link. Host/test builds back those with thread-local
// state so `cargo test -p runtime` can drive the scheduler deterministically.
#[cfg(any(test, feature = "host"))]
pub mod test_hooks {
    use crate::step_queue::StepQueue;
    use core::cell::RefCell;
    use std::vec::Vec;

    const N_QUEUES: usize = crate::step_queue::N_AXIS_STEP_QUEUES;

    std::thread_local! {
        static NOW: RefCell<u32> = const { RefCell::new(0) };
        static OWNED: RefCell<u8> = const { RefCell::new(0) };
        static QUEUES: RefCell<[StepQueue; N_QUEUES]> =
            RefCell::new(core::array::from_fn(|_| StepQueue::new()));
        static EMITS: RefCell<Vec<(u8, i32)>> = const { RefCell::new(Vec::new()) };
    }

    pub fn set_now(v: u32) {
        NOW.with(|c| *c.borrow_mut() = v);
    }
    pub fn now() -> u32 {
        NOW.with(|c| *c.borrow())
    }
    pub fn set_owned_mask(m: u8) {
        OWNED.with(|c| *c.borrow_mut() = m);
    }
    pub fn owned_mask() -> u8 {
        OWNED.with(|c| *c.borrow())
    }
    pub fn record_emit(axis: u8, n: i32) {
        EMITS.with(|c| c.borrow_mut().push((axis, n)));
    }
    pub fn take_emits() -> Vec<(u8, i32)> {
        EMITS.with(|c| core::mem::take(&mut *c.borrow_mut()))
    }
    pub fn reset() {
        set_now(0);
        set_owned_mask(0);
        let _ = take_emits();
        QUEUES.with(|c| {
            for q in c.borrow_mut().iter_mut() {
                q.clear();
            }
        });
    }
    /// Returns a raw pointer into the thread-local queue array. The pointer is
    /// only used synchronously within a single `kalico_step_output_event` call
    /// on the same thread, so it does not outlive the borrow.
    pub fn queue_for_axis(axis_idx: usize) -> *mut StepQueue {
        if axis_idx >= N_QUEUES {
            return core::ptr::null_mut();
        }
        QUEUES.with(|c| {
            let mut arr = c.borrow_mut();
            // axis_idx < N_QUEUES checked above; matches step_queue.rs's
            // explicit-allow pattern for the deny(indexing_slicing) lint.
            #[allow(clippy::indexing_slicing)]
            let ptr: *mut StepQueue = &mut arr[axis_idx];
            ptr
        })
    }
}

#[cfg(test)]
mod tests;
