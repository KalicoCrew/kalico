//! Stream lifecycle state machine (host + MCU side). Spec §8.
//!
//! Phase 1 introduced the `FgStreamState` enum so `FgState::stream_state_machine`
//! has a type to point at; Phase 3.2 stubs the FFI handlers
//! (`open` / `arm` / `terminal` / `flush` / `clock_sync_respond`); Phase 6
//! fleshes out the transition rules and the §8.5 `force_idle` handshake.
//!
//! All Phase-3.2 stubs return `KALICO_ERR_STREAM_STATE_VIOLATION` (-140) so
//! the host sees a recognisable "not-yet-implemented" code rather than
//! silently passing.

#![allow(unsafe_code)]

use core::sync::atomic::Ordering;

use crate::error::KALICO_ERR_STREAM_STATE_VIOLATION;
use crate::state::{FgState, SharedState};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FgStreamState {
    Idle = 0,
    StreamOpening = 1,
    StreamOpenPriming = 2,
    Arming = 3,
    Armed = 4,
    Running = 5,
    Draining = 6,
    Drained = 7,
    Fault = 8,
}

/// `kalico_stream_open` handler stub (Phase 6).
///
/// Phase 3.2 returns `STREAM_STATE_VIOLATION` until Phase 6 wires the actual
/// transitions. The signature is final — the FFI shim takes the result code
/// directly and reads `shared.credit_epoch` for the out-param itself.
pub fn open(_fg: &mut FgState, _shared: &SharedState, _stream_id: u32) -> i32 {
    KALICO_ERR_STREAM_STATE_VIOLATION
}

/// `kalico_stream_arm` handler stub (Phase 6).
///
/// Returns `(result_code, armed_t_start)`. Phase 3.2 stub returns
/// `STREAM_STATE_VIOLATION` and `0` for the armed-t_start; Phase 6 computes
/// `t_start = max(t_start_t0, mcu_clock_now + arm_lead_cycles)` per §6.4.
pub fn arm(
    _fg: &mut FgState,
    _shared: &SharedState,
    _t_start_t0: u64,
    _arm_lead_cycles: u32,
) -> (i32, u64) {
    (KALICO_ERR_STREAM_STATE_VIOLATION, 0)
}

/// `kalico_stream_terminal` handler stub (Phase 6).
pub fn terminal(_fg: &mut FgState, _shared: &SharedState, _segment_id: u32) -> i32 {
    KALICO_ERR_STREAM_STATE_VIOLATION
}

/// `kalico_stream_flush` handler stub (Phase 6).
///
/// Phase 6 implements the §8.5 `force_idle` handshake (Decision A — set
/// `force_idle=true` first, ack-wait, then clear `stream_open`); the stub
/// returns `STREAM_STATE_VIOLATION` so callers see a recognisable
/// not-yet-implemented code.
pub fn flush(_fg: &mut FgState, _shared: &SharedState) -> i32 {
    KALICO_ERR_STREAM_STATE_VIOLATION
}

/// `kalico_clock_sync_request` handler stub (Phase 6).
///
/// Returns `(result_code, mcu_clock)`. Phase 6 reads the §11.4 widened-now
/// from `SharedState` and packs it into a `kalico_clock_sync_response` with
/// `request_id` echoed back; the stub returns the widened-now snapshot for
/// FFI shape validation but `STREAM_STATE_VIOLATION` for the result.
pub fn clock_sync_respond(
    _fg: &mut FgState,
    shared: &SharedState,
    _request_id: u32,
    _host_send_time_lo: u32,
    _host_send_time_hi: u32,
) -> (i32, u64) {
    let lo = shared.widened_now_lo.load(Ordering::Acquire);
    let hi = shared.widened_now_hi.load(Ordering::Acquire);
    let mcu_clock = (u64::from(hi) << 32) | u64::from(lo);
    (KALICO_ERR_STREAM_STATE_VIOLATION, mcu_clock)
}
