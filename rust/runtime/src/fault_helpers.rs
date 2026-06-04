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

static MIN_LEVEL: AtomicU8 = AtomicU8::new(2);

#[cfg(any(not(any(test, feature = "host")), feature = "mcu-linux"))]
unsafe extern "C" {
    fn kalico_log_emit(level: u8, subsystem: u8, event: u16, code: u16, arg0: u32, arg1: u32);
}

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

/// Latch a `StepQueueOverflow` fault.
///
/// `axis_idx` is in `0..4` (X, Y, Z, E). `fault_detail` encoding:
/// `(axis_idx & 0xFF) << 16`.
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
/// `axis_idx` encoded into bits 16..24 of `fault_detail`.
#[inline]
pub fn raise_position_count_overflow(shared: &SharedState, axis_idx: usize) {
    let detail = (axis_idx as u32 & 0xFF) << 16;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::PositionCountOverflow.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::PositionCountOverflow, detail);
}

/// Latch a `MathNonFinite` fault.
///
/// `axis_idx` encoded into bits 16..24 of `fault_detail`.
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
/// `axis_idx` encoded into bits 16..24 of `fault_detail`.
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
/// `axis_idx` encoded into bits 16..24 of `fault_detail`.
#[inline]
pub fn raise_phase_mode_not_available(shared: &SharedState, axis_idx: usize) {
    let detail = (axis_idx as u32 & 0xFF) << 16;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::PhaseModeNotAvailable.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::PhaseModeNotAvailable, detail);
}

/// Latch a `JogParametersInvalid` fault. No per-event detail.
#[inline]
pub fn raise_jog_parameters_invalid(shared: &SharedState) {
    let detail = 0u32;
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::JogParametersInvalid.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::JogParametersInvalid, detail);
}

/// Latch a `PieceStartInPast` fault.
///
/// `fault_detail` encoding:
/// - bits 16..24: `axis_idx`
/// - bits  0..16: `deficit_us` saturated at 0xFFFF (~65 ms)
#[inline]
pub fn raise_piece_start_in_past(shared: &SharedState, axis_idx: usize, deficit_us: u32) {
    let detail = ((axis_idx as u32 & 0xFF) << 16) | deficit_us.min(0xFFFF);
    shared.fault_detail.store(detail, Ordering::Release);
    shared
        .last_error
        .store(FaultCode::PieceStartInPast.as_i32(), Ordering::Release);
    emit_fault_log(FaultCode::PieceStartInPast, detail);
}

/// Latch a `TickIntervalExceeded` fault.
///
/// `fault_detail` low 16 bits: gap in tick periods (saturated).
/// Detail stored before code so the foreground always sees a populated pair.
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
/// `fault_detail` encoding:
/// - bits 16..24: `axis_idx`
/// - bits  0..16: `abs_steps` saturated at 0xFFFF
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

/// Latch an `UnknownStepMode` fault. Detail: `((axis_idx & 0xFF) << 16) | (mode & 0xFF)`.
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
