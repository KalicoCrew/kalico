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
    submit_one(&mut h, 1);
    submit_one(&mut h, 2);
    h.tick();
    h.feed_rx(&build_frame(&[], 2));
    h.tick();
    let len_before_race = h.tx_log().len();
    let depth_before_race = h.unacked_depth();
    assert_eq!(depth_before_race, 1, "seq=2 outstanding");

    let (tx_new, _rx_new) = sync_channel(1);
    h.submission_tx
        .send(ReactorCommand::SubmitTyped {
            call_id: 3,
            payload: vec![3u8],
            expected_response_name: "noop".into(),
            completion: tx_new,
            deadline: h.clock.now() + Duration::from_secs(60),
        })
        .unwrap();
    h.feed_rx(&build_frame(&[], 2));

    h.tick();

    assert_eq!(h.unacked_depth(), 2);
    assert_eq!(h.reactor.last_ack_seq, 2);

    let frame_size = 5 + 1;
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
