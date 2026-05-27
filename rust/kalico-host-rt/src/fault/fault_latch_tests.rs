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
