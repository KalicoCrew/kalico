use super::*;
use crate::host_io::test_harness::ReactorHarness;
use crate::host_io::wire::build_frame;
use std::sync::mpsc::sync_channel;
use std::time::Duration;

fn submit_one(h: &mut ReactorHarness, payload: u8) {
    let (tx, _rx) = sync_channel(1);
    let _ = h.reactor.dispatch_submission(
        payload as u64,
        vec![payload],
        "noop".into(),
        tx,
        h.clock.now() + Duration::from_secs(60),
    );
}

fn ack(wire_seq: u8) -> Vec<u8> {
    build_frame(&[], wire_seq)
}

#[test]
fn duplicate_ack_triggers_retransmit() {
    let mut h = ReactorHarness::new();
    submit_one(&mut h, 1);
    submit_one(&mut h, 2);
    h.tick();
    h.feed_rx(&ack(2));
    h.tick();
    let len_before = h.tx_log().len();
    h.feed_rx(&ack(2));
    h.tick();
    assert!(
        h.tx_log().len() > len_before,
        "duplicate ack should trigger retransmit"
    );
}

#[test]
fn ignore_nak_seq_suppresses_paired_second_nak() {
    let mut h = ReactorHarness::new();
    submit_one(&mut h, 1);
    submit_one(&mut h, 2);
    h.tick();
    h.feed_rx(&ack(2));
    h.tick();
    let len_before = h.tx_log().len();
    h.feed_rx(&ack(2));
    h.feed_rx(&ack(2));
    h.tick();
    let delta = h.tx_log().len() - len_before;
    assert_eq!(
        delta,
        1 + 6,
        "second NAK must be suppressed by ignore_nak_seq"
    );
}

#[test]
fn rto_fires_at_srtt_plus_4_rttvar() {
    let mut h = ReactorHarness::new();
    submit_one(&mut h, 1);
    h.tick();
    h.advance_clock(Duration::from_millis(200));
    h.feed_rx(&ack(2));
    h.tick();
    assert_eq!(h.reactor.rtt.current_rto(), Duration::from_millis(600));

    submit_one(&mut h, 2);
    h.tick();
    let len_before = h.tx_log().len();
    h.advance_clock(Duration::from_millis(599));
    h.tick();
    assert_eq!(h.tx_log().len(), len_before, "RTO not yet expired");
    h.advance_clock(Duration::from_millis(2));
    h.tick();
    assert!(h.tx_log().len() > len_before, "RTO should have fired");
}

#[test]
fn rto_clamped_to_floor_25ms() {
    use crate::host_io::rtt::MIN_RTO;
    let mut h = ReactorHarness::new();
    assert_eq!(h.reactor.rtt.current_rto(), MIN_RTO);
    submit_one(&mut h, 1);
    h.tick();
    h.advance_clock(Duration::from_micros(100));
    h.feed_rx(&ack(2));
    h.tick();
    assert!(h.reactor.rtt.current_rto() >= MIN_RTO);
    assert_eq!(h.reactor.rtt.current_rto(), MIN_RTO);
}

#[test]
fn rto_clamped_to_ceiling_5s() {
    use crate::host_io::rtt::MAX_RTO;
    let mut h = ReactorHarness::new();
    submit_one(&mut h, 1);
    h.tick();
    h.advance_clock(Duration::from_secs(10));
    h.feed_rx(&ack(2));
    h.tick();
    assert_eq!(h.reactor.rtt.current_rto(), MAX_RTO);
}

#[test]
fn max_retry_count_closes_with_fault_and_completes_pending() {
    let mut h = ReactorHarness::new();
    let (tx, rx) = sync_channel(1);
    let _ = h.reactor.dispatch_submission(
        1,
        vec![0xAA],
        "noop".into(),
        tx,
        h.clock.now() + Duration::from_secs(600),
    );
    h.tick();
    for _ in 0..8 {
        h.advance_clock(Duration::from_secs(16));
        h.tick();
    }
    assert_eq!(h.reactor.state, ReactorState::Closed);
    let outcome = h.tick();
    assert_eq!(outcome, TickOutcome::Closed);
    let result = rx
        .recv_timeout(Duration::from_millis(100))
        .expect("completion delivered");
    assert!(
        matches!(result, Err(TransportError::Closed)),
        "expected Closed, got {result:?}"
    );
    let latched = h.reactor.event_dispatcher.fault_latch.cell.as_ref();
    let fc = latched.expect("fault latched").fault_code;
    assert_eq!(fc, FaultCode::HostRetransmitExhausted.as_u16());
}
