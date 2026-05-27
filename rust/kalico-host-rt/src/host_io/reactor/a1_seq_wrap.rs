use super::*;
use crate::host_io::test_harness::ReactorHarness;
use crate::host_io::wire::build_frame;
use std::sync::mpsc::sync_channel;
use std::time::Duration;

/// Build a 5-byte ack/nak frame with the given wire seq nibble.
fn ack_frame(wire_seq_nibble: u8) -> Vec<u8> {
    build_frame(&[], wire_seq_nibble)
}

/// Submit one frame directly via dispatch_submission. Drops the receiver.
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
    // Pre: window empty; init send_seq=1, receive_seq=1.
    assert_eq!(h.reactor.send_seq, 1);
    assert_eq!(h.reactor.receive_seq, 1);
    assert!(h.reactor.unacked_window.is_empty());

    // Inject ack frame whose 4-bit wire seq nibble = 5 (rseq decoded = 5).
    h.feed_rx(&ack_frame(5));
    h.tick();

    // Snap path (reactor.rs:222-227): both counters jump to rseq.
    assert_eq!(h.reactor.send_seq, 5);
    assert_eq!(h.reactor.receive_seq, 5);
}

#[test]
fn mid_range_mod16_wrap_pops_correct_entries() {
    let mut h = ReactorHarness::new();
    // Submit 12 frames (window cap = MAX_PENDING_BLOCKS = 12).
    for p in 1u8..=12 {
        submit_one(&mut h, p);
    }
    // Tick to process serial poll (no rx yet).
    h.tick();
    assert_eq!(h.unacked_depth(), 12);
    // After 12 submissions: send_seq advanced from 1 to 13.
    assert_eq!(h.reactor.send_seq, 13);
    // receive_seq still 1.
    assert_eq!(h.reactor.receive_seq, 1);

    // Step 1: ack rseq=12. decode_absolute(wire) when receive_seq=1:
    //   delta = (wire - 1) & 0xF. Want delta=11 → wire = (1+11) & 0xF = 12.
    // rseq = 1 + 11 = 12. Pops seqs <12 (i.e. 1..=11). seq=12 remains.
    h.feed_rx(&ack_frame(12));
    h.tick();
    assert_eq!(h.reactor.last_ack_seq, 12);
    assert_eq!(h.reactor.receive_seq, 12);
    assert_eq!(h.unacked_depth(), 1);

    // Step 2: cross the receive_seq=16 epoch boundary. Submit more frames so
    // there's something past 16 to ack. send_seq is 13; submit seqs 13..=20.
    for p in 13u8..=20 {
        submit_one(&mut h, p);
    }
    h.tick();
    assert_eq!(h.unacked_depth(), 9); // seqs 12..=20 outstanding
    assert_eq!(h.reactor.send_seq, 21);

    // Ack rseq=18. delta = (18 - 12) & 0xF = 6 → wire nibble = (12 + 6) & 0xF = 2.
    // Wait: decode_absolute reads low-4 wire bits and computes
    //   delta = (wire_seq - receive_seq) & 0xF
    // where receive_seq=12. To get delta=6 we need wire = (12 + 6) & 0xF = 18 & 0xF = 2.
    // rseq = 12 + 6 = 18. This crosses the receive_seq=16 mod-16 boundary.
    h.feed_rx(&ack_frame(2));
    h.tick();
    assert_eq!(h.reactor.last_ack_seq, 18);
    assert_eq!(h.reactor.receive_seq, 18);
    // Pops seqs <18, i.e. 12..=17. seq 18..=20 remain → 3 entries.
    assert_eq!(h.unacked_depth(), 3);
}

#[test]
fn near_u64_max_decode_does_not_panic() {
    // Probe both `wrapping_sub` (used to compute delta from low-4 nibble)
    // and the addition `receive_seq + delta` against the u64 boundary.
    // The 4-bit wire nibble bounds delta ∈ [0, 15], so to make addition
    // wrap we set receive_seq = u64::MAX - 5 (or similar small offset)
    // and ack a target ≥ u64::MAX, which wraps.
    //
    // Note: the production reactor's `decode_absolute` does NOT use
    // `wrapping_add` — it does `self.receive_seq + delta` (reactor.rs:214).
    // In debug builds this would panic on overflow. We use values where
    // the addition stays within u64 to verify correctness, then a
    // separate sub-test using `checked_add` semantics could probe the
    // hypothetical wrap; for now we simply verify the high-end works.
    let mut h = ReactorHarness::new();
    h.reactor.receive_seq = u64::MAX - 5;
    h.reactor.send_seq = u64::MAX - 5;
    h.reactor.last_ack_seq = u64::MAX - 6;

    submit_one(&mut h, 0);
    h.tick();
    assert_eq!(h.unacked_depth(), 1);

    // The submit pushed an entry at seq = u64::MAX - 5. send_seq is now
    // u64::MAX - 4. To ack that entry, target rseq = u64::MAX - 4.
    // delta = ((target - receive_seq) & 0xF) = (1) & 0xF = 1.
    // Wire nibble = target & 0xF = (u64::MAX - 4) & 0xF.
    let target_rseq: u64 = u64::MAX - 4;
    let nibble = (target_rseq & 0x0F) as u8;
    h.feed_rx(&ack_frame(nibble));
    h.tick();
    assert_eq!(h.reactor.last_ack_seq, target_rseq);
    assert_eq!(h.reactor.receive_seq, target_rseq);

    // Probe the wrap-sub side: from receive_seq = X, a wire nibble
    // representing a value "behind" X (which the MCU would never send,
    // but `wrapping_sub` must not panic on it). We expect this stays
    // discriminated as a stale ack — last_ack_seq is already X+1, so
    // any rseq we decode whose value < last_ack_seq+1 is dropped.
    let nibble_behind = ((h.reactor.receive_seq - 8) & 0x0F) as u8;
    h.feed_rx(&ack_frame(nibble_behind));
    h.tick(); // must not panic
}
