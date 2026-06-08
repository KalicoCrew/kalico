// TIM5 ISR body — clock widening, inter-arrival gap guard, and `Engine::tick`.
// Stepper dispatch is in `dispatch_stepper` (compiled only with
// `motion-module-stepper`).

#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::fault_helpers::raise_tick_interval_exceeded;
use crate::state::SharedState;

#[cfg(feature = "motion-module-stepper")]
pub use crate::dispatch_stepper::{AXIS_A, AXIS_B, AXIS_E, AXIS_Z, DISPLACEMENT_THRESHOLD_MM};

pub use crate::stepping_state::N_AXES;

// C-side scheduler accessor for the most-recently-dispatched timer func.
// Read only on the `-311` fault path. MCU/sim link only; host/test → 0.
#[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
unsafe extern "C" {
    fn sched_last_dispatched_func() -> u32;
}

// Stacked exception-frame captures from the TIM5 naked-wrapper shim
// (src/stm32/runtime_tick_*.c). Read only on the `-311` fault path:
//   - `runtime_tim5_stacked_pc()`: instruction about to execute when TIM5 preempted.
//   - `runtime_tim5_stacked_exc()`: stacked xPSR exception number (0 = thread).
#[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
unsafe extern "C" {
    fn runtime_tim5_stacked_pc() -> u32;
    fn runtime_tim5_stacked_exc() -> u32;
}

#[inline]
fn tim5_stacked_pc() -> u32 {
    #[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
    // SAFETY: side-effect-free volatile frame read. Safe from the TIM5 ISR.
    unsafe {
        runtime_tim5_stacked_pc()
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "kalico-sim")))]
    {
        0
    }
}

#[inline]
fn tim5_stacked_exc() -> u32 {
    #[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
    // SAFETY: side-effect-free volatile frame read. Safe from the TIM5 ISR.
    unsafe {
        runtime_tim5_stacked_exc()
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "kalico-sim")))]
    {
        0
    }
}

#[inline]
fn last_dispatched_func() -> u32 {
    #[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
    // SAFETY: side-effect-free ring-buffer index read. Safe from the TIM5 ISR.
    unsafe {
        sched_last_dispatched_func()
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "kalico-sim")))]
    {
        0
    }
}

const TICK_GAP_FAULT_MULT: u64 = 2;

pub fn isr_sample_tick(
    isr: &mut crate::state::IsrState,
    shared: &SharedState,
    storage: &mut [crate::piece_ring::PieceEntry],
    raw_cyccnt: u32,
) {
    let body_start = unsafe { cyccnt_read() };
    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ISR_ENTER);

    bump_relaxed(isr.engine.tick_counter.inner_atomic());

    let now = isr.widen_state.widen(raw_cyccnt);

    // Publish unconditionally — skipping on a fault tick freezes the
    // foreground clock and pegs the scheduler.
    crate::clock::publish_widened_now(shared, now);

    let after_widen = unsafe { cyccnt_read() };
    update_max(
        &shared.isr_widen_cycles_max,
        after_widen.wrapping_sub(body_start),
    );

    let after_arm = after_widen;
    update_max(
        &shared.isr_arm_cycles_max,
        after_arm.wrapping_sub(after_widen),
    );

    // Inter-arrival guard: only fires when the previous tick was active
    // (last_tick_now == Some). Idle/boot ticks leave it None so the guard
    // never trips during config or between moves.
    let period = isr.engine.sample_period_cycles as u64;
    if let Some(last) = isr.last_tick_now {
        let gap = now.wrapping_sub(last);
        if period != 0 && gap > period * TICK_GAP_FAULT_MULT {
            // Integer division is intentional: `gap_ticks` is the integer
            // count of sample periods elapsed, used as a fault detail tag.
            #[allow(clippy::integer_division)]
            let gap_ticks = (gap / period) as u32;
            // Store before the fault code latches so the host always sees
            // populated values. Stacked PC is the primary addr2line target.
            shared
                .tick_blocker_pc
                .store(tim5_stacked_pc(), Ordering::Release);
            shared
                .tick_blocker_exc
                .store(tim5_stacked_exc(), Ordering::Release);
            shared
                .tick_blocker_func
                .store(last_dispatched_func(), Ordering::Release);
            raise_tick_interval_exceeded(shared, gap_ticks);
            isr.last_tick_now = Some(now);
            crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ISR_EXIT);
            return;
        }
    }

    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_TICK);
    let active = {
        let crate::state::IsrState { engine, .. } = isr;
        engine.tick(now, shared, storage)
    };

    let body_end = unsafe { cyccnt_read() };
    update_max(
        &shared.isr_eval_cycles_max,
        body_end.wrapping_sub(after_arm),
    );

    // Some only when this tick was active; idle ticks clear it so the gap
    // check never straddles an idle gap.
    isr.last_tick_now = if active { Some(now) } else { None };
    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ISR_EXIT);
}

/// Read the DWT cycle counter via `isr_phase::cyccnt()` — sole declaration
/// there avoids duplicate `extern "C"` symbols at link time.
#[inline]
unsafe fn cyccnt_read() -> u32 {
    crate::isr_phase::cyccnt()
}

#[inline]
pub(crate) fn update_max(slot: &portable_atomic::AtomicU32, val: u32) {
    let prev = slot.load(Ordering::Relaxed);
    if val > prev {
        slot.store(val, Ordering::Relaxed);
    }
}

#[inline]
pub(crate) fn bump_relaxed(slot: &portable_atomic::AtomicU32) {
    let prev = slot.load(Ordering::Relaxed);
    slot.store(prev.wrapping_add(1), Ordering::Relaxed);
}

#[cfg(test)]
mod tests;
