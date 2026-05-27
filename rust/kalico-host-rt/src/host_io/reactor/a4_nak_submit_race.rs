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

#[test]
fn submit_then_nak_in_same_tick_keeps_state_consistent() {
    let mut h = ReactorHarness::new();
    // Stage: submit two frames (seq=1, seq=2). Ack rseq=2 → pops seq=1.
    // Window now has just seq=2.
    submit_one(&mut h, 1);
    submit_one(&mut h, 2);
    h.tick();
    h.feed_rx(&build_frame(&[], 2)); // forward-progress ack rseq=2
    h.tick();
    let len_before_race = h.tx_log().len();
    let depth_before_race = h.unacked_depth();
    assert_eq!(depth_before_race, 1, "seq=2 outstanding");

    // Same-tick race: queue a fresh submission AND a duplicate NAK on rseq=2.
    // Reactor::run() loop body order: command drain (step 1) before serial
    // poll (step 2), so the new frame writes first; NAK retransmit follows.
    let (tx_new, _rx_new) = sync_channel(1);
    // Use SubmitTyped to bypass parser.encode (the harness's empty parser
    // doesn't know any commands). The reactor command-drain path treats
    // SubmitTyped identically aside from encoding.
    h.submission_tx
        .send(ReactorCommand::SubmitTyped {
            call_id: 3,
            payload: vec![3u8],
            expected_response_name: "noop".into(),
            completion: tx_new,
            deadline: h.clock.now() + Duration::from_secs(60),
        })
        .unwrap();
    h.feed_rx(&build_frame(&[], 2)); // duplicate ack on rseq=2 → NAK

    h.tick();

    // Both events processed:
    // - Submission of frame 3 wrote to tx_log first (step 1: command drain).
    // - NAK retransmit followed (step 2: serial poll → handle_ack_nak).
    //   At NAK time the window contains {seq=2, seq=3}, so the retransmit
    //   buffer = 1 SYNC byte + frame_for_seq2 + frame_for_seq3.
    // Window post-tick: still {seq=2, seq=3} (NAK retransmit doesn't pop).
    assert_eq!(h.unacked_depth(), 2);
    assert_eq!(h.reactor.last_ack_seq, 2);

    // Compute the exact expected byte delta. Each frame is 5 (header+CRC+SYNC)
    // + 1 byte payload = 6 bytes. We expect:
    //   - new frame (seq=3): 6 bytes
    //   - retransmit buffer: 1 SYNC + 6 (seq=2 frame) + 6 (seq=3 frame) = 13 bytes
    // Total delta = 19 bytes. If the NAK was suppressed or retransmit didn't
    // fire, delta would be only 6 (new frame alone). The exact-equality
    // assertion proves the retransmit ran with both frames.
    let frame_size = 5 + 1; // empty MIN + 1-byte payload
    let expected_delta = frame_size + (1 + 2 * frame_size);
    let actual_delta = h.tx_log().len() - len_before_race;
    assert_eq!(
        actual_delta,
        expected_delta,
        "expected new frame ({frame_size} B) + retransmit buffer (1 SYNC + 2 frames = {}) \
         = {expected_delta} B; got {actual_delta} B",
        1 + 2 * frame_size
    );
}
