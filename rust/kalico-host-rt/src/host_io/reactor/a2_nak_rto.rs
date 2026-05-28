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
    // Forward-progress ack: rseq=2 pops seq=1; seq=2 remains.
    h.feed_rx(&ack(2));
    h.tick();
    let len_before = h.tx_log().len();
    // Duplicate ack on rseq=2 → NAK retransmit (window non-empty).
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
    // Two duplicate acks on rseq=2 in the same poll cycle.
    h.feed_rx(&ack(2));
    h.feed_rx(&ack(2));
    h.tick();
    let delta = h.tx_log().len() - len_before;
    // One retransmit = 1 SYNC + (frame_bytes for seq=2). Frame for [2] = 6 bytes.
    assert_eq!(
        delta,
        1 + 6,
        "second NAK must be suppressed by ignore_nak_seq"
    );
}

#[test]
fn rto_fires_at_srtt_plus_4_rttvar() {
    let mut h = ReactorHarness::new();
    // Submit at clock T0.
    submit_one(&mut h, 1);
    h.tick();
    // Advance 200ms; ack: RTT sample = 200ms (chosen so the resulting
    // RTO is above the 500ms MIN_RTO floor and the clamp doesn't mask
    // the SRTT + 4×RTTVAR formula under test).
    h.advance_clock(Duration::from_millis(200));
    h.feed_rx(&ack(2));
    h.tick();
    // After one sample of 200ms: SRTT=200, RTTVAR=100; RTO = 200 + max(G, 4*100) = 600ms.
    assert_eq!(h.reactor.rtt.current_rto(), Duration::from_millis(600));

    // Submit frame 2 at current clock; sent_at is "now".
    submit_one(&mut h, 2);
    h.tick();
    let len_before = h.tx_log().len();
    // Advance 599ms — just shy of RTO.
    h.advance_clock(Duration::from_millis(599));
    h.tick();
    assert_eq!(h.tx_log().len(), len_before, "RTO not yet expired");
    // Advance 2ms more → past RTO.
    h.advance_clock(Duration::from_millis(2));
    h.tick();
    assert!(h.tx_log().len() > len_before, "RTO should have fired");
}

#[test]
fn rto_clamped_to_floor_25ms() {
    use crate::host_io::rtt::MIN_RTO;
    let mut h = ReactorHarness::new();
    // Default starts at MIN_RTO.
    assert_eq!(h.reactor.rtt.current_rto(), MIN_RTO);
    // Drive a tiny RTT sample (100µs). SRTT=100µs, RTTVAR=50µs;
    // raw RTO = 100µs + max(1ms, 200µs) = ~1.1ms. Clamped to MIN_RTO=25ms.
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
    // Huge sample: 10s. SRTT=10s, RTTVAR=5s; raw RTO = 10 + max(G, 20) = 30s.
    // Clamped to MAX_RTO=5s.
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
    // Force 8 successive TimeoutDriven retransmits via clock advance.
    // Each tick advances clock past current RTO; write_retransmit increments
    // retry_count for every unacked entry. On the 8th call,
    // retry_count >= MAX_RETRY_COUNT AND silence >= MCU_SILENCE_FOR_CLOSE
    // (currently 120 s) → state→Closed, Err returned.
    // We advance 20s per iteration (8 × 20s = 160s > MCU_SILENCE_FOR_CLOSE),
    // which is well past the MAX_RTO ceiling (5s) so the RTO guard fires
    // on every tick, and well past MCU_SILENCE_FOR_CLOSE so the silence
    // gate is satisfied before retry_count reaches MAX_RETRY_COUNT.
    for _ in 0..8 {
        // 20s >> MAX_RTO (5s) ensures RTO fires; 8 × 20s = 160s >
        // MCU_SILENCE_FOR_CLOSE (120s) satisfies the silence gate.
        h.advance_clock(Duration::from_secs(20));
        h.tick();
    }
    // Reactor should now be Closed.
    assert_eq!(h.reactor.state, ReactorState::Closed);
    // The next tick processes Closed → TickOutcome::Closed + flush_all_completions.
    let outcome = h.tick();
    assert_eq!(outcome, TickOutcome::Closed);
    // Pending submission must have completed with TransportError::Closed.
    let result = rx
        .recv_timeout(Duration::from_millis(100))
        .expect("completion delivered");
    assert!(
        matches!(result, Err(TransportError::Closed)),
        "expected Closed, got {result:?}"
    );
    // Fault was staged with HostRetransmitExhausted code.
    let latched = h.reactor.event_dispatcher.fault_latch.cell.as_ref();
    let fc = latched.expect("fault latched").fault_code;
    assert_eq!(fc, FaultCode::HostRetransmitExhausted.as_u16());
}
