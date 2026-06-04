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

// The fault-emit path FFI-calls the C `kalico_log_emit` (gated to the MCU/sim
// builds). The crate denies `unsafe_code` by default; opt this module out the
// same way the other FFI modules do (e.g. dispatch_stepper.rs).
#![allow(unsafe_code)]

use core::sync::atomic::{AtomicU8, Ordering};

use crate::error::FaultCode;
#[allow(unused_imports)] // used only in the gated MCU/mcu-linux extern block
use crate::log_codes::{EVENT_RUNTIME_FAULT_LATCHED, SUBSYSTEM_RUNTIME};
use crate::state::SharedState;

/// Wire log levels — must match motion-bridge's mcu_level_str (0=trace,1=debug,2=warn,3=error).
const LOG_LEVEL_ERROR: u8 = 3;

/// Minimum level emitted (gate at emit, spec decision E). Default = warn (2).
/// Stage 4 adds a runtime setter; Stage 3 leaves the default so error-level
/// faults always pass. Relaxed ordering is fine.
static MIN_LEVEL: AtomicU8 = AtomicU8::new(2);

// kalico_log_emit (src/kalico_log.c): present in MCU + mcu-linux sim firmware,
// absent in the pure-host cdylib / cargo test. Mirror dispatch_stepper.rs gating.
#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
unsafe extern "C" {
    fn kalico_log_emit(level: u8, subsystem: u8, event: u16, code: u16, arg0: u32, arg1: u32);
}

/// Emit a structured log for a latched fault, gated by `MIN_LEVEL`. Fault identity
/// rides in `code` (host resolves `code_name`); `arg0` = `fault_detail`. No-op on
/// the pure-host build.
#[inline]
fn emit_fault_log(fault: FaultCode, detail: u32) {
    if LOG_LEVEL_ERROR < MIN_LEVEL.load(Ordering::Relaxed) {
        return;
    }
    #[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
    // SAFETY: kalico_log_emit is a pure C logging sink; no aliasing or
    // ownership constraints on its arguments.
    unsafe {
        kalico_log_emit(
            LOG_LEVEL_ERROR,
            SUBSYSTEM_RUNTIME,
            EVENT_RUNTIME_FAULT_LATCHED,
            fault.as_u16(),
            detail,
            0,
        );
    }
    #[cfg(not(any(not(any(test, feature = "host")), feature = "mcu-linux")))]
    {
        let _ = (fault, detail);
    }
}

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
    emit_fault_log(FaultCode::StepQueueOverflow, detail);
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
    emit_fault_log(FaultCode::PositionCountOverflow, detail);
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
    emit_fault_log(FaultCode::MathNonFinite, detail);
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
    emit_fault_log(FaultCode::PieceAdvanceUnderflow, detail);
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
    emit_fault_log(FaultCode::PhaseModeNotAvailable, detail);
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
    let detail = 0u32;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::JogParametersInvalid.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::JogParametersInvalid, detail);
}

/// Latch a `PieceStartInPast` fault (spec §6 safety invariant).
///
/// Raised by the ISR `get_piece_for_time` logic when the candidate next piece's
/// `start_time` is more than `2 * sample_period_cycles` in the past — the MCU
/// was not fed in time.  All motion stops; the host must reset before resuming.
///
/// `fault_detail` encoding:
/// - bits 16..24: `axis_idx` (which axis detected the late piece)
/// - bits  0..16: `deficit_us` — `now - start_time` in microseconds, saturated
///   at 65535 µs (~65 ms); values ≥ 65535 read as 0xFFFF = ">=65ms".
///   A value in the low hundreds of µs indicates a boundary near-miss; values
///   in the milliseconds-plus range indicate a real host fall-behind.
#[inline]
pub fn raise_piece_start_in_past(shared: &SharedState, axis_idx: usize, deficit_us: u32) {
    let detail = ((axis_idx as u32 & 0xFF) << 16) | deficit_us.min(0xFFFF);
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::PieceStartInPast.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::PieceStartInPast, detail);
}

/// Latch a `TickIntervalExceeded` fault (ISR inter-arrival guard).
///
/// Raised when the gap between two consecutive TIM5 ticks exceeds
/// `2 * sample_period_cycles` — the ISR was starved. The measured gap in
/// units of tick periods (saturated to 16 bits) is stored in the low 16
/// bits of `fault_detail` for host diagnosis. Detail-first ordering is
/// mandatory: the foreground reads `last_error` first, then `fault_detail`;
/// storing detail before code guarantees the host always sees a valid pair.
#[inline]
pub fn raise_tick_interval_exceeded(shared: &SharedState, gap_ticks: u32) {
    let detail = gap_ticks.min(0xFFFF);
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::TickIntervalExceeded.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::TickIntervalExceeded, detail);
}

/// Latch a `StepsPerSampleExceeded` fault.
///
/// Raised by the ISR pulse-dispatch path when a single TIM5 sample window
/// would require more than `MAX_STEPS_PER_SAMPLE` microsteps. This is an
/// unrecoverable discontinuity — most commonly a missing/incorrect position
/// seed, so the motor-frame baseline disagrees with the piece stream. Mirrors
/// `raise_piece_start_in_past`: all motion stops, the host must reset.
/// `axis_idx` is encoded into bits 16..24 of `fault_detail`; the saturated
/// per-sample step count is carried in the low 16 bits for host diagnosis.
#[inline]
pub fn raise_steps_per_sample_exceeded(shared: &SharedState, axis_idx: usize, abs_steps: u32) {
    let detail = ((axis_idx as u32 & 0xFF) << 16) | abs_steps.min(0xFFFF);
    shared.fault_detail.store(detail, Ordering::Release);
    shared.last_error.store(
        FaultCode::StepsPerSampleExceeded.as_i32(),
        Ordering::Release,
    );
    emit_fault_log(FaultCode::StepsPerSampleExceeded, detail);
}

/// Latch an `UnknownStepMode` fault.
///
/// Silently dropping an unrecognized mode would hide a host/firmware version
/// mismatch; we fail loud instead. Detail encoding:
/// `((axis_idx & 0xFF) << 16) | (mode & 0xFF)`.
#[inline]
pub fn raise_unknown_step_mode(shared: &SharedState, axis_idx: usize, mode: u8) {
    let detail = ((axis_idx as u32 & 0xFF) << 16) | u32::from(mode);
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::UnknownStepMode.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::UnknownStepMode, detail);
}

#[cfg(test)]
mod tests;
