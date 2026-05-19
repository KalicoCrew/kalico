//! Fault-raising helpers for the unified stepping engine.
//!
//! Each helper publishes a `(fault_code, fault_detail)` pair on
//! [`SharedState`] in the canonical order: detail first, then code. The
//! ordering matters because the foreground status frame reads `last_error`
//! to decide whether to surface `fault_detail` at all; storing the detail
//! first guarantees that a foreground reader observing a non-zero
//! `last_error` always sees the populated detail.
//!
//! `fault_detail` encoding (per spec
//! `docs/superpowers/specs/2026-05-19-stepping-redesign-design.md` §9.2):
//! the top byte is reserved; bits 16..24 hold the axis (or per-axis
//! stepper) index, the lower bits are fault-specific context.
//!
//! These helpers are intentionally tiny — they exist so that the hot path
//! (TIM5 ISR) does not have to spell out the two-store sequence inline at
//! every call site, and so the encoding stays in one place when the fault
//! taxonomy evolves.

use core::sync::atomic::Ordering;

use crate::error::FaultCode;
use crate::state::SharedState;

/// Latch a `StepQueueOverflow` fault and bump the per-axis overflow counter.
///
/// `axis_idx` is the per-axis index in `0..4` (X, Y, Z, E). Indexes outside
/// the supported range are silently dropped for the per-axis counter
/// (`queue_overflow_count` has four slots); the fault code and detail are
/// still published so the host sees the event.
#[inline]
pub fn raise_step_queue_overflow(shared: &SharedState, axis_idx: usize) {
    let detail = (axis_idx as u32 & 0xFF) << 16;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::StepQueueOverflow.as_i32(), Ordering::Release);
    if axis_idx < 4 {
        #[allow(clippy::indexing_slicing)] // bound checked immediately above
        shared.queue_overflow_count[axis_idx].fetch_add(1, Ordering::Release);
    }
}

/// Latch a `PositionCountOverflow` fault.
///
/// `axis_idx` is the per-axis index in `0..4`; for steppers paired to an
/// axis we encode the axis index (each axis carries ≤ 4 steppers, so
/// per-axis granularity is sufficient for the host to localize the fault).
#[inline]
pub fn raise_position_count_overflow(shared: &SharedState, axis_idx: usize) {
    let detail = (axis_idx as u32 & 0xFF) << 16;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::PositionCountOverflow.as_i32(), Ordering::Release);
}

/// Latch a `MathNonFinite` fault. `axis_idx` is encoded into the detail
/// payload (top-byte reserved, axis index in bits 16..24) so the host can
/// localize which axis evaluation went non-finite.
#[inline]
pub fn raise_math_non_finite(shared: &SharedState, axis_idx: usize) {
    let detail = (axis_idx as u32 & 0xFF) << 16;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::MathNonFinite.as_i32(), Ordering::Release);
}

/// Latch a `PieceAdvanceUnderflow` fault.
///
/// Raised by `advance_piece_if_needed` when the per-axis advancement loop
/// fails to make progress within its bounded iteration budget (>4 iters).
/// That can only happen if the active piece is so short (or has a
/// non-finite duration) that a single TIM5 sample-window spans many
/// pieces — the loop would otherwise be unbounded, so we latch a fault
/// and break out. `axis_idx` is encoded into bits 16..24 of `fault_detail`
/// so the host can localize the offending axis.
#[inline]
pub fn raise_piece_advance_underflow(shared: &SharedState, axis_idx: usize) {
    let detail = (axis_idx as u32 & 0xFF) << 16;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::PieceAdvanceUnderflow.as_i32(), Ordering::Release);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn step_queue_overflow_publishes_code_and_bumps_counter() {
        let shared = SharedState::new();
        raise_step_queue_overflow(&shared, 2);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            FaultCode::StepQueueOverflow.as_i32()
        );
        // axis_idx 2 → 2 << 16 = 0x00020000
        assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0002_0000);
        assert_eq!(shared.queue_overflow_count[2].load(Ordering::Acquire), 1);
        // Other axes untouched.
        assert_eq!(shared.queue_overflow_count[0].load(Ordering::Acquire), 0);
    }

    #[test]
    fn step_queue_overflow_out_of_range_axis_does_not_panic() {
        let shared = SharedState::new();
        // 7 is outside the queue_overflow_count[4] range. The fault is
        // still latched but no counter is incremented.
        raise_step_queue_overflow(&shared, 7);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            FaultCode::StepQueueOverflow.as_i32()
        );
        assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0007_0000);
    }

    #[test]
    fn position_count_overflow_publishes_code_and_detail() {
        let shared = SharedState::new();
        raise_position_count_overflow(&shared, 1);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            FaultCode::PositionCountOverflow.as_i32()
        );
        assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0001_0000);
    }

    #[test]
    fn math_non_finite_publishes_code_and_detail() {
        let shared = SharedState::new();
        raise_math_non_finite(&shared, 3);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            FaultCode::MathNonFinite.as_i32()
        );
        assert_eq!(shared.fault_detail.load(Ordering::Acquire), 0x0003_0000);
    }
}
