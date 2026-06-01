use super::*;
use crate::host_io::runtime_events::{FaultEvent, RuntimeEvent, StatusEvent};
use arc_swap::ArcSwap;
use std::sync::{Arc, Mutex};

fn make_dispatcher() -> EventDispatcher {
    let snap = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    EventDispatcher::new(snap, 256, 64)
}

fn fault_status(engine_status: u8, last_fault: u16, segment_id: u32) -> RuntimeEvent {
    RuntimeEvent::Status(StatusEvent {
        engine_status,
        queue_depth: 0,
        current_segment_id: segment_id,
        last_fault,
        fault_detail: 0,
        retired_through_segment_id: 0,
    })
}

#[test]
fn fault_status_synthesizes_when_no_edge_observed() {
    let mut d = make_dispatcher();
    d.dispatch(fault_status(3, 17, 42));
    let cell = d
        .fault_latch
        .cell
        .as_ref()
        .expect("fault should be synthesized");
    assert_eq!(cell.fault_code, 17);
    assert!(cell.synthesized);
    assert_eq!(cell.segment_id, 42);
}

#[test]
fn synthesis_idempotent_across_repeated_status_frames() {
    let mut d = make_dispatcher();
    d.dispatch(fault_status(3, 17, 42));
    d.dispatch(fault_status(3, 17, 42));
    // Still latched once (cell still present, still synthesized).
    let cell = d.fault_latch.cell.as_ref().unwrap();
    assert!(cell.synthesized);
}

#[test]
fn edge_event_upgrades_synthesized_in_place() {
    let mut d = make_dispatcher();
    d.dispatch(fault_status(3, 17, 42));
    // Edge event with exact segment_id preferred.
    d.dispatch(RuntimeEvent::Fault(FaultEvent {
        fault_code: 17,
        fault_detail: 0,
        segment_id: 39,
        synthesized: false,
    }));
    let cell = d.fault_latch.cell.as_ref().unwrap();
    assert!(!cell.synthesized, "edge upgrade clears synthesized");
    assert_eq!(cell.segment_id, 39, "edge segment_id preferred");
}

#[test]
fn status_without_fault_does_not_synthesize() {
    let mut d = make_dispatcher();
    d.dispatch(fault_status(1, 0, 0)); // engine_status != 3
    assert!(d.fault_latch.cell.is_none());
}

// Helper for the v2 credit-flow synthesis tests. queue_depth and the
// retirement watermark are the only fields that matter for the
// synthesized CreditFreed; engine_status is RUNNING and no fault.
fn status_with_watermark(queue_depth: u8, retired_through: u32) -> RuntimeEvent {
    RuntimeEvent::Status(StatusEvent {
        engine_status: 1,
        queue_depth,
        current_segment_id: 0,
        last_fault: 0,
        fault_detail: 0,
        retired_through_segment_id: retired_through,
    })
}

/// v2: a Status frame whose watermark advances past the previous
/// observation must synthesize a `CreditFreed` dispatched through the
/// normal `CreditFreed` path — forwarded to the bridge poller via
/// `take_runtime_event`.
#[test]
fn status_watermark_advance_synthesizes_credit_freed() {
    use std::sync::mpsc::sync_channel;
    let mut d = make_dispatcher();

    // Subscribe to the runtime-event channel so we can observe the
    // synthesized CreditFreed forwarded to the bridge poller.
    let (tx, rx) = sync_channel::<RuntimeEvent>(8);
    d.runtime_event_dispatcher.subscribe(tx).unwrap();

    // First Status with watermark=5 and queue_depth=2 → free_slots=5.
    d.dispatch(status_with_watermark(2, 5));

    // Drain channel — exactly 1 Status and 1 CreditFreed must have
    // been dispatched, in that order.
    let evt1 = rx.recv().unwrap();
    let evt2 = rx.recv().unwrap();
    assert!(matches!(evt1, RuntimeEvent::Status(_)));
    match evt2 {
        RuntimeEvent::CreditFreed(c) => {
            assert_eq!(c.retired_through_segment_id, 5);
            assert_eq!(c.free_slots, 5);
        }
        other => panic!("expected synthesized CreditFreed, got {other:?}"),
    }
    assert!(rx.try_recv().is_err(), "no further events");
}

/// Repeated Status frames with the same watermark must NOT re-synthesize
/// `CreditFreed`.
#[test]
fn status_watermark_unchanged_does_not_synthesize() {
    use std::sync::mpsc::sync_channel;
    let mut d = make_dispatcher();

    let (tx, rx) = sync_channel::<RuntimeEvent>(8);
    d.runtime_event_dispatcher.subscribe(tx).unwrap();

    // Prime: watermark=5 advances from 0 → 1 CreditFreed.
    d.dispatch(status_with_watermark(0, 5));
    // Drain: 1 Status + 1 CreditFreed.
    let _ = rx.recv().unwrap();
    let _ = rx.recv().unwrap();

    // Same watermark again — no new CreditFreed.
    d.dispatch(status_with_watermark(0, 5));
    // Only 1 Status arrives.
    let _ = rx.recv().unwrap();
    assert!(rx.try_recv().is_err(), "no CreditFreed on unchanged watermark");
}

/// Stale Status frames (watermark < last seen) must not synthesize.
#[test]
fn status_watermark_regression_does_not_synthesize() {
    use std::sync::mpsc::sync_channel;
    let mut d = make_dispatcher();

    let (tx, rx) = sync_channel::<RuntimeEvent>(8);
    d.runtime_event_dispatcher.subscribe(tx).unwrap();

    d.dispatch(status_with_watermark(0, 10));
    // Drain 1 Status + 1 CreditFreed.
    let _ = rx.recv().unwrap();
    let _ = rx.recv().unwrap();

    // Regress to 8 (an out-of-order or stale frame).
    d.dispatch(status_with_watermark(0, 8));
    // Only 1 Status arrives.
    let _ = rx.recv().unwrap();
    assert!(rx.try_recv().is_err(), "regression ignored — no CreditFreed");
}

#[test]
fn heartbeat_callback_fires_with_retired_counts() {
    let status = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut d = EventDispatcher::new(status, 16, 8);

    let recorder: Arc<Mutex<Vec<Vec<u32>>>> = Arc::new(Mutex::new(Vec::new()));
    let recorder2 = Arc::clone(&recorder);
    d.heartbeat_callback = Some(Arc::new(move |counts: &[u32]| {
        recorder2.lock().unwrap().push(counts.to_vec());
    }));

    d.dispatch(RuntimeEvent::Heartbeat {
        retired_counts: vec![5, 1],
    });

    let got = recorder.lock().unwrap();
    assert_eq!(got.len(), 1, "callback must fire exactly once");
    assert_eq!(got[0], vec![5, 1]);
}

#[test]
fn heartbeat_is_not_forwarded_to_runtime_rx() {
    use std::sync::mpsc::sync_channel;

    let status = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut d = EventDispatcher::new(status, 16, 8);

    let (tx, rx) = sync_channel::<RuntimeEvent>(8);
    d.runtime_event_dispatcher.subscribe(tx).unwrap();

    d.dispatch(RuntimeEvent::Heartbeat {
        retired_counts: vec![3, 7],
    });

    assert!(
        rx.try_recv().is_err(),
        "Heartbeat must NOT reach the runtime_rx channel"
    );
}

#[test]
fn mcu_log_hook_is_called_on_mcu_log_event() {
    use std::time::Instant;
    use crate::host_io::runtime_events::McuLogEvent;

    let snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut dispatcher = EventDispatcher::new(snapshot, 16, 8);

    let received: Arc<Mutex<Vec<McuLogEvent>>> = Arc::new(Mutex::new(Vec::new()));
    let received_clone = received.clone();
    dispatcher.set_mcu_log_hook(move |e: McuLogEvent| {
        received_clone.lock().unwrap().push(e);
    });

    let event = RuntimeEvent::McuLog(McuLogEvent {
        mcu_tick: 12345,
        level: 2,
        subsystem: 1,
        event: 1,
        code: 0xFEC9,
        seq: 1,
        args: [0, 0],
        host_recv: Instant::now(),
    });
    dispatcher.dispatch(event);

    let got = received.lock().unwrap();
    assert_eq!(got.len(), 1);
    assert_eq!(got[0].mcu_tick, 12345);
}

#[test]
fn mcu_log_without_hook_does_not_panic() {
    use std::time::Instant;
    use crate::host_io::runtime_events::McuLogEvent;

    let snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut dispatcher = EventDispatcher::new(snapshot, 16, 8);
    // No hook set — must not panic.
    dispatcher.dispatch(RuntimeEvent::McuLog(McuLogEvent {
        mcu_tick: 0,
        level: 0,
        subsystem: 0,
        event: 0,
        code: 0,
        seq: 0,
        args: [0, 0],
        host_recv: Instant::now(),
    }));
}

#[test]
fn mcu_log_also_forwarded_to_runtime_rx() {
    use std::sync::mpsc::sync_channel;
    use std::time::Instant;
    use crate::host_io::runtime_events::McuLogEvent;

    let snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
    let mut dispatcher = EventDispatcher::new(snapshot, 16, 8);

    let (tx, rx) = sync_channel::<RuntimeEvent>(8);
    dispatcher.runtime_event_dispatcher.subscribe(tx).unwrap();

    dispatcher.dispatch(RuntimeEvent::McuLog(McuLogEvent {
        mcu_tick: 99,
        level: 3,
        subsystem: 0,
        event: 0,
        code: 0,
        seq: 0,
        args: [0, 0],
        host_recv: Instant::now(),
    }));

    match rx.recv().unwrap() {
        RuntimeEvent::McuLog(e) => assert_eq!(e.mcu_tick, 99),
        other => panic!("expected McuLog on runtime_rx, got {other:?}"),
    }
}
