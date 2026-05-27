//! Stream lifecycle state machine stub — Task 5 placeholder.
//!
//! The full stream state machine (force_idle handshake, flush, arm protocol)
//! has been removed. This stub retains `FgStreamState` and the lifecycle
//! functions so `state.rs`, `engine.rs`, and `kalico-c-api` compile
//! until Task 6.

#![allow(unsafe_code)]

use crate::error::{KALICO_ERR_STREAM_STATE_VIOLATION, KALICO_OK};
use crate::state::{FgState, RuntimeContext, SharedState};

/// Foreground stream lifecycle state.
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

/// Called by `Engine::retire_if_complete` when a segment retires.
/// Stub — no-op until Task 6 wires the terminal-segment handshake.
pub fn check_terminal_on_retire(_shared: &SharedState, _segment_id: u32) {}

// ── Stream lifecycle stubs ── Task 6 replaces with real state-machine bodies.

/// Stub: stream open. Always returns `KALICO_ERR_STREAM_STATE_VIOLATION`.
pub fn open(_fg: &mut FgState, _shared: &SharedState, _stream_id: u32) -> i32 {
    KALICO_ERR_STREAM_STATE_VIOLATION
}

/// Stub: stream arm. Always returns `(KALICO_ERR_STREAM_STATE_VIOLATION, 0)`.
pub fn arm(
    _fg: &mut FgState,
    _shared: &SharedState,
    _t_start_t0: u64,
    _arm_lead_cycles: u32,
) -> (i32, u64) {
    (KALICO_ERR_STREAM_STATE_VIOLATION, 0)
}

/// Stub: stream terminal. Always returns `KALICO_ERR_STREAM_STATE_VIOLATION`.
pub fn terminal(_fg: &mut FgState, _shared: &SharedState, _segment_id: u32) -> i32 {
    KALICO_ERR_STREAM_STATE_VIOLATION
}

/// Stub: stream flush. Always returns `KALICO_OK`.
///
/// # Safety
/// `ctx` must be non-null and point to a valid `RuntimeContext`.
/// `out_credit_epoch` may be null; if non-null it must be a valid `*mut u32`.
pub unsafe fn flush(_ctx: *mut RuntimeContext, _out_credit_epoch: *mut u32) -> i32 {
    KALICO_OK
}
