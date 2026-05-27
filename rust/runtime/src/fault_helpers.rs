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

/// Latch a `PhaseModeNotAvailable` fault.
///
/// Raised by the `configure_axis` foreground path when the requested step
/// mode is `Phase` but the SPI dispatch path is not yet available. The
/// `axis_idx` is encoded into bits 16..24 of `fault_detail` so the host
/// can identify which axis configuration was rejected.
#[inline]
pub fn raise_phase_mode_not_available(shared: &SharedState, axis_idx: usize) {
    let detail = (axis_idx as u32 & 0xFF) << 16;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::PhaseModeNotAvailable.as_i32(), Ordering::Release);
}

/// Latch a `JogParametersInvalid` fault.
///
/// Raised by the command-dispatcher entry points for jog-style mutations
/// (`set_stepper_offset` is the first Task-12 caller) when the supplied
/// parameters are out of range (e.g. `max_microsteps_per_sample` outside
/// `1..=256`, or `stepper_idx` not bound to any configured axis). No
/// per-event detail is carried — the foreground knows which command it
/// just sent and the host-visible error code is enough to localize the
/// fault for surfacing through `kalico_status_v6`.
#[inline]
pub fn raise_jog_parameters_invalid(shared: &SharedState) {
    shared.fault_detail.store(0, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::JogParametersInvalid.as_i32(), Ordering::Release);
}

#[cfg(test)]
mod tests;
