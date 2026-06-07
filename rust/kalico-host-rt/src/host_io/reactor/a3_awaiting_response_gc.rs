use super::*;
use crate::host_io::test_harness::ReactorHarness;
use std::sync::mpsc::sync_channel;
use std::time::Duration;

fn submit_with_call_id(
    h: &mut ReactorHarness,
    call_id: u64,
    deadline_offset: Duration,
) -> std::sync::mpsc::Receiver<Result<crate::transport::MessageParams, TransportError>> {
    let (tx, rx) = sync_channel(1);
    let _ = h.reactor.dispatch_submission(
        call_id,
        vec![call_id as u8],
        "noop".into(),
        tx,
        h.clock.now() + deadline_offset,
    );
    rx
}

#[test]
fn abandon_on_drop_marks_entry_then_late_response_is_discarded() {
    let mut h = ReactorHarness::new();
    let _rx = submit_with_call_id(&mut h, 1, Duration::from_secs(60));
    h.tick();
    assert_eq!(h.awaiting_depth(), 1);

    h.submission_tx.send(ReactorCommand::Abandon(1)).unwrap();
    h.tick();

    assert_eq!(h.awaiting_depth(), 1);
    let entry = h
        .reactor
        .awaiting_response
        .iter()
        .next()
        .expect("entry still present");
    assert!(
        entry.abandoned,
        "abandon command should have flagged the entry"
    );
}

#[test]
fn per_entry_dispatcher_timeout_completes_with_dispatcher_timeout() {
    let mut h = ReactorHarness::new();
    let rx = submit_with_call_id(&mut h, 1, Duration::from_millis(10));
    h.tick();
    assert_eq!(h.awaiting_depth(), 1);

    h.advance_clock(Duration::from_millis(50));
    h.tick();

    let result = rx
        .recv_timeout(Duration::from_millis(100))
        .expect("completion delivered");
    assert!(
        matches!(result, Err(TransportError::DispatcherTimeout)),
        "expected DispatcherTimeout, got {result:?}"
    );
    assert_eq!(h.awaiting_depth(), 0);
}

#[test]
fn disconnect_clears_all_pending_with_closed() {
    let mut h = ReactorHarness::new();
    let rx1 = submit_with_call_id(&mut h, 1, Duration::from_secs(60));
    let rx2 = submit_with_call_id(&mut h, 2, Duration::from_secs(60));
    h.tick();
    assert_eq!(h.awaiting_depth(), 2);

    h.submission_tx.send(ReactorCommand::Shutdown).unwrap();
    let outcome = h.tick();
    assert_eq!(outcome, TickOutcome::Closed);

    for rx in [rx1, rx2] {
        let result = rx
            .recv_timeout(Duration::from_millis(100))
            .expect("completion delivered");
        assert!(
            matches!(result, Err(TransportError::Closed)),
            "expected Closed, got {result:?}"
        );
    }
    assert_eq!(h.awaiting_depth(), 0);
    assert_eq!(h.unacked_depth(), 0);
}
