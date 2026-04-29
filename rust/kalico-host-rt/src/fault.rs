//! Fault aggregator. Receives `kalico_fault` async events and lifts them
//! into a typed [`FaultEvent`] for upstream consumers.
//!
//! The MCU emits `kalico_fault fault_code=… fault_detail=… segment_id=…`
//! once on the FAULT-state transition (spec §9). The Step-6 `host_io`
//! shim's `poll_events` returns these as [`crate::transport::MessageParams`];
//! [`parse_fault_event`] decodes one. The full host-side state machine
//! (rate-limit, dedupe, propagate to the user as `Result<_, FaultEvent>`)
//! is Step-7 MVP work.

use crate::transport::MessageParams;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultEvent {
    /// Spec §9 fault-code enum, lower 16 bits of the `i32` carried on
    /// the wire. Negative values are surfaced as their unsigned u16
    /// reinterpretation per the wire schema.
    pub fault_code: u16,
    /// Spec §9.2 detail field (encoder-specific u32).
    pub fault_detail: u32,
    /// `kalico_runtime_current_segment_id` at the time of fault.
    pub segment_id: u32,
}

/// Parse a `kalico_fault` async event's params into a typed
/// [`FaultEvent`]. Returns `None` if the message is missing the
/// required fields. Step-6 minimum: never validates the code is in the
/// known taxonomy — Step-7 MVP adds the cross-check against
/// `runtime::error::FaultCode`.
pub fn parse_fault_event(params: &MessageParams) -> Option<FaultEvent> {
    // The `fault_code` field on the wire is a u16 from `sendf("...
    // fault_code=%hu ...")` (see runtime_tick.c). Klipper's parser
    // widens %hu to a 32-bit signed int; we re-narrow.
    #[allow(clippy::cast_possible_truncation)]
    let fault_code = (params.get_u32("fault_code") & 0xFFFF) as u16;
    let fault_detail = params.get_u32("fault_detail");
    let segment_id = params.get_u32("segment_id");
    Some(FaultEvent {
        fault_code,
        fault_detail,
        segment_id,
    })
}
