//! Step-output timer ISR body (TIM3 on H7, TIM2 on F4).
//!
//! Scans per-axis SPSC `step_queues[]` this MCU owns, emits every step whose
//! `cycle_abs` has arrived (bounded per dispatch), and returns the soonest
//! absolute cycle to fire at next — or [`STEP_OUTPUT_DISABLE`] when all owned
//! queues are empty (event-driven, not idle-polling).
//!
//! NVIC-priority invariant: producer (TIM5) and this consumer run at the same
//! priority; see `src/generic/kalico_nvic_prio.h` for the full rationale.

#![allow(unsafe_code)]

use crate::step_queue::{N_AXIS_STEP_QUEUES, peek as queue_peek, pop as queue_pop};

/// Returned when no owned queue has a pending step — tells C to disable the
/// timer. `u32::MAX` is the sentinel; a `cycle_abs` landing exactly there costs
/// at most one delayed step (the next producer kick re-arms).
pub const STEP_OUTPUT_DISABLE: u32 = u32::MAX;

/// Emit window: 0 means "fire only once `now >= cycle_abs`" (wrap-safe signed
/// compare). Named so the bench can widen it if compare latency demands a lead.
const DUE_WINDOW_CYCLES: i32 = 0;

/// Steps emitted per dispatch across all owned axes. Cap exceeded → return
/// `now` so C re-fires immediately rather than dropping the remaining work.
pub const MAX_STEPS_PER_EVENT: u32 = 32;

// MCU build links these C symbols. Host builds (`feature = "host"`) and unit
// tests must not pull undefined symbols into the cdylib, so they are stubbed.
#[cfg(not(any(test, feature = "host")))]
unsafe extern "C" {
    fn timer_read_time() -> u32;
    fn runtime_emit_step_pulses(axis_idx: u8, n_steps: i32);
    // Bitmask of axes this MCU owns; bit N ⇒ axis N is scanned.
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
/// [`STEP_OUTPUT_DISABLE`] to switch the timer off until the next producer kick.
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

/// Wrap-safe minimum `cycle_abs` across all owned non-empty queues, or `None`.
/// Uses signed delta from `now` so heads across the u32 wrap boundary compare correctly.
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
