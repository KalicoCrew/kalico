use super::*;
use crate::host_io::test_harness::ReactorHarness;
use crate::host_io::wire::build_frame;
use std::sync::mpsc::sync_channel;
use std::time::Duration;

fn ack_frame(wire_seq_nibble: u8) -> Vec<u8> {
    build_frame(&[], wire_seq_nibble)
}

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
fn empty_window_snap_advances_both_counters() {
    let mut h = ReactorHarness::new();
    assert_eq!(h.reactor.send_seq, 1);
    assert_eq!(h.reactor.receive_seq, 1);
    assert!(h.reactor.unacked_window.is_empty());

    h.feed_rx(&ack_frame(5));
    h.tick();

    assert_eq!(h.reactor.send_seq, 5);
    assert_eq!(h.reactor.receive_seq, 5);
}

#[test]
fn mid_range_mod16_wrap_pops_correct_entries() {
    let mut h = ReactorHarness::new();
    for p in 1u8..=12 {
        submit_one(&mut h, p);
    }
    h.tick();
    assert_eq!(h.unacked_depth(), 12);
    assert_eq!(h.reactor.send_seq, 13);
    assert_eq!(h.reactor.receive_seq, 1);

    h.feed_rx(&ack_frame(12));
    h.tick();
    assert_eq!(h.reactor.last_ack_seq, 12);
    assert_eq!(h.reactor.receive_seq, 12);
    assert_eq!(h.unacked_depth(), 1);

    for p in 13u8..=20 {
        submit_one(&mut h, p);
    }
    h.tick();
    assert_eq!(h.unacked_depth(), 9);
    assert_eq!(h.reactor.send_seq, 21);

    h.feed_rx(&ack_frame(2));
    h.tick();
    assert_eq!(h.reactor.last_ack_seq, 18);
    assert_eq!(h.reactor.receive_seq, 18);
    assert_eq!(h.unacked_depth(), 3);
}

#[test]
fn near_u64_max_decode_does_not_panic() {
    let mut h = ReactorHarness::new();
    h.reactor.receive_seq = u64::MAX - 5;
    h.reactor.send_seq = u64::MAX - 5;
    h.reactor.last_ack_seq = u64::MAX - 6;

    submit_one(&mut h, 0);
    h.tick();
    assert_eq!(h.unacked_depth(), 1);

    let target_rseq: u64 = u64::MAX - 4;
    let nibble = (target_rseq & 0x0F) as u8;
    h.feed_rx(&ack_frame(nibble));
    h.tick();
    assert_eq!(h.reactor.last_ack_seq, target_rseq);
    assert_eq!(h.reactor.receive_seq, target_rseq);

    let nibble_behind = ((h.reactor.receive_seq - 8) & 0x0F) as u8;
    h.feed_rx(&ack_frame(nibble_behind));
    h.tick();
}
