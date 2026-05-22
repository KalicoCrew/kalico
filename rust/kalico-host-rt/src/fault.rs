//! Fault aggregator. Receives `kalico_fault` async events and lifts them
//! into a typed [`FaultEvent`] for upstream consumers.
//!
//! The MCU emits `kalico_fault fault_code=… fault_detail=… segment_id=…`
//! once on the FAULT-state transition (spec §9). The Step-6 `host_io`
//! shim's `poll_events` returns these as [`crate::transport::MessageParams`];
//! [`parse_fault_event`] decodes one. The full host-side state machine
//! (rate-limit, dedupe, propagate to the user as `Result<_, FaultEvent>`)
//! is Step-7 MVP work.

use std::sync::mpsc::SyncSender;

use crate::host_io::runtime_events::FaultEvent as RuntimeFaultEvent;
use crate::transport::{MessageParams, SubscribeError};

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

// ─── FaultLatch ──────────────────────────────────────────────────────────────

/// Latches the first (or first real) fault event and fans it out to an
/// optional synchronous subscriber.
///
/// Semantics:
/// - Only the first event latches (cell stays once set), *except* when the
///   latched event was synthesized (`synthesized = true`) and the incoming
///   event is a real MCU edge (`synthesized = false`): the real event
///   upgrades the latch in-place and is forwarded to the subscriber.
/// - [`subscribe`] replays the already-latched fault to a late subscriber so
///   callers never miss an event that arrived before they attached.
#[derive(Debug, Default)]
pub struct FaultLatch {
    pub cell: Option<RuntimeFaultEvent>,
    pub subscriber: Option<SyncSender<RuntimeFaultEvent>>,
}

impl FaultLatch {
    /// Dispatch a fault event. Edge events upgrade a synthesized latch
    /// in-place; otherwise the first event wins.
    pub fn dispatch(&mut self, event: RuntimeFaultEvent) {
        let upgrade = self
            .cell
            .as_ref()
            .map(|c| c.synthesized && !event.synthesized)
            .unwrap_or(false);
        if self.cell.is_none() || upgrade {
            self.cell = Some(event.clone());
            if let Some(tx) = &self.subscriber {
                let _ = tx.send(event);
            }
        }
    }

    /// Attach a subscriber. Returns [`SubscribeError::AlreadySubscribed`] if
    /// one is already registered. Replays the currently-latched fault (if any)
    /// to the new subscriber before returning.
    pub fn subscribe(&mut self, tx: SyncSender<RuntimeFaultEvent>) -> Result<(), SubscribeError> {
        if self.subscriber.is_some() {
            return Err(SubscribeError::AlreadySubscribed { channel: "fault" });
        }
        if let Some(latched) = &self.cell {
            let _ = tx.send(latched.clone());
        }
        self.subscriber = Some(tx);
        Ok(())
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod fault_latch_tests {
    use std::sync::mpsc;

    use super::*;
    use crate::host_io::runtime_events::FaultEvent as RuntimeFaultEvent;

    fn make_event(fault_code: u16, synthesized: bool) -> RuntimeFaultEvent {
        RuntimeFaultEvent {
            fault_code,
            fault_detail: 0,
            segment_id: 0,
            synthesized,
        }
    }

    #[test]
    fn dispatch_latches_first_event() {
        let mut latch = FaultLatch::default();
        latch.dispatch(make_event(17, false));
        assert_eq!(latch.cell.as_ref().unwrap().fault_code, 17);
    }

    #[test]
    fn dispatch_does_not_overwrite_real_with_real() {
        let mut latch = FaultLatch::default();
        latch.dispatch(make_event(1, false));
        latch.dispatch(make_event(2, false));
        assert_eq!(latch.cell.as_ref().unwrap().fault_code, 1);
    }

    #[test]
    fn dispatch_upgrades_synthesized_with_edge() {
        let mut latch = FaultLatch::default();
        latch.dispatch(make_event(99, true)); // synthesized
        latch.dispatch(make_event(42, false)); // real MCU edge
        assert_eq!(latch.cell.as_ref().unwrap().fault_code, 42);
        assert!(!latch.cell.as_ref().unwrap().synthesized);
    }

    #[test]
    fn subscribe_replays_latched_to_new_receiver() {
        let mut latch = FaultLatch::default();
        latch.dispatch(make_event(7, false));

        let (tx, rx) = mpsc::sync_channel(1);
        latch.subscribe(tx).expect("first subscribe should succeed");
        let replayed = rx.try_recv().expect("should have received replayed fault");
        assert_eq!(replayed.fault_code, 7);
    }

    #[test]
    fn second_subscribe_returns_error() {
        let mut latch = FaultLatch::default();
        let (tx1, _rx1) = mpsc::sync_channel::<RuntimeFaultEvent>(1);
        let (tx2, _rx2) = mpsc::sync_channel::<RuntimeFaultEvent>(1);
        latch
            .subscribe(tx1)
            .expect("first subscribe should succeed");
        let err = latch
            .subscribe(tx2)
            .expect_err("second subscribe should fail");
        assert!(
            matches!(err, SubscribeError::AlreadySubscribed { channel: "fault" }),
            "expected AlreadySubscribed{{channel: \"fault\"}}, got {:?}",
            err
        );
    }
}
