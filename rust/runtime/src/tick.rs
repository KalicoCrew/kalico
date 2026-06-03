//! TIM5 ISR body — the unified motion evaluator (clock + timing stage).
//!
//! This module owns the ISR entry point (`isr_sample_tick`), the DWT clock
//! widening, the inter-arrival gap guard, and the `Engine::tick` call.
//! Stepper dispatch (pulse / phase) has been extracted to the
//! `dispatch_stepper` module, which is compiled only when the
//! `motion-module-stepper` Cargo feature is active.
//!
//! Utility functions `bump_relaxed` and `update_max` are `pub(crate)` so
//! that `dispatch_stepper` can import them without duplicating the
//! implementations.

#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::fault_helpers::raise_tick_interval_exceeded;
use crate::state::SharedState;

// ── Re-exports for backward compatibility ────────────────────────────────────

/// Axis-index constants and `N_AXES` re-exported from dispatch_stepper when
/// the feature is active, or from stepping_state directly when it is not.
#[cfg(feature = "motion-module-stepper")]
pub use crate::dispatch_stepper::{AXIS_A, AXIS_B, AXIS_E, AXIS_Z, DISPLACEMENT_THRESHOLD_MM};

pub use crate::stepping_state::N_AXES;

// ── -311 diagnostic externs ───────────────────────────────────────────────────

// C-side scheduler accessor for the most-recently-dispatched timer func.
// Read only on the `-311` fault path. MCU/sim link only; host/test → 0.
#[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
unsafe extern "C" {
    fn sched_last_dispatched_func() -> u32;
}

// Stacked exception-frame captures from the TIM5 naked-wrapper shim
// (src/stm32/runtime_tick_*.c). Read only on the `-311` fault path:
//   - `runtime_tim5_stacked_pc()`: instruction that was about to execute when
//     TIM5 preempted — the code that held the CPU/PRIMASK across the late tick.
//   - `runtime_tim5_stacked_exc()`: stacked xPSR exception number (0 = thread).
#[cfg(any(not(any(test, feature = "host")), feature = "kalico-sim"))]
unsafe extern "C" {
    fn runtime_tim5_stacked_pc() -> u32;
    fn runtime_tim5_stacked_exc() -> u32;
}

/// Stacked PC at TIM5 entry — the instruction that held the CPU across the late
/// tick. Returns 0 on host/test builds.
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

/// Stacked xPSR exception number at TIM5 entry (0 = thread). Returns 0 on
/// host/test builds.
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

/// Most-recently-dispatched scheduler timer func address. Returns 0 on
/// host/test builds (no C scheduler linked).
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

// ─── ISR sample entry ─────────────────────────────────────────────────────

/// Fault when the gap between two consecutive active ticks exceeds this
/// multiple of the tick period.
const TICK_GAP_FAULT_MULT: u64 = 2;

/// Single-call ISR body for the piece-ring walker engine (Task 6).
///
/// Widens the 32-bit DWT clock, publishes `widened_now` unconditionally (so
/// the foreground clock never stalls), then checks the inter-arrival gap —
/// but only if the previous tick was active (had a piece playing). Idle and
/// boot ticks clear `last_tick_now` to `None`, so the guard never fires
/// during config or between moves.
///
/// `storage` is projected from `RuntimeContext::piece_storage` by the FFI
/// caller (`kalico_runtime_tick_sample`).
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

    // Publish unconditionally — skipping this on a fault tick freezes the
    // foreground clock and pegs the scheduler.
    crate::clock::publish_widened_now(shared, now);

    let after_widen = unsafe { cyccnt_read() };
    update_max(
        &shared.isr_widen_cycles_max,
        after_widen.wrapping_sub(body_start),
    );

    // No segment-arm stage — the new engine manages its own piece advancement.
    let after_arm = after_widen;
    update_max(
        &shared.isr_arm_cycles_max,
        after_arm.wrapping_sub(after_widen),
    );

    // Inter-arrival guard: only fires when the previous tick was active
    // (last_tick_now == Some).  Idle/boot ticks leave it None, so the guard
    // never trips during config or between moves.  An idle→active transition
    // re-baselines: the first active tick sets Some, the second is the first
    // one compared.
    let period = isr.engine.sample_period_cycles as u64;
    if let Some(last) = isr.last_tick_now {
        let gap = now.wrapping_sub(last);
        if period != 0 && gap > period * TICK_GAP_FAULT_MULT {
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
            return; // skip dispatch; publish already happened; foreground escalation shuts down
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

    // Update baseline: Some only when this tick was active; idle ticks clear it
    // so the gap check never straddles an idle gap.
    isr.last_tick_now = if active { Some(now) } else { None };
    crate::isr_phase::set_phase(crate::isr_phase::RT_PHASE_ISR_EXIT);
}

/// Read the DWT cycle counter. Delegates to `isr_phase::cyccnt()` — sole
/// declaration there avoids duplicate `extern "C"` symbols at link time.
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
