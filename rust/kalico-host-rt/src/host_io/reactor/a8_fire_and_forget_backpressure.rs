use super::*;
use crate::host_io::test_harness::ReactorHarness;
use crate::host_io::window::MAX_PENDING_BLOCKS;
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

/// Fill the unacked window to capacity (12 frames).
fn fill_window(h: &mut ReactorHarness) {
    for i in 0..MAX_PENDING_BLOCKS {
        submit_one(h, i as u8);
    }
    assert_eq!(h.unacked_depth(), MAX_PENDING_BLOCKS);
    assert!(h.reactor.unacked_window.is_full());
}

#[test]
fn a8_fire_and_forget_enqueues_under_window_full() {
    let mut h = ReactorHarness::new();
    fill_window(&mut h);

    let tx_len_before = h.tx_log().len();

    // Dispatch a fire-and-forget payload while the window is full.
    let payload = vec![0xAB, 0xCD, 0xEF];
    h.reactor
        .dispatch_fire_and_forget(payload.clone(), false)
        .expect("enqueue should not error under ceiling");

    // The payload must NOT have been written to the wire — it should be
    // sitting in the pending_fire_and_forget queue.
    assert_eq!(
        h.tx_log().len(),
        tx_len_before,
        "no bytes should hit the wire while window is full"
    );
    assert_eq!(h.reactor.pending_fire_and_forget.len(), 1);

    // Free a slot by acking one outstanding frame. With send_seq starting
    // at 1, the 12 outstanding frames carry seqs 1..=12. Acking rseq=2
    // (wire nibble = 2) pops everything with seq < 2 → pops seq=1.
    h.feed_rx(&build_frame(&[], 2));
    h.tick();

    // After the tick, drain_pending_submissions should have flushed the
    // fire-and-forget payload to the wire.
    assert_eq!(
        h.reactor.pending_fire_and_forget.len(),
        0,
        "fire-and-forget queue should be drained after window opens",
    );
    assert!(
        h.tx_log().len() > tx_len_before,
        "payload should now be on the wire (got tx_log delta = {})",
        h.tx_log().len() - tx_len_before,
    );
}

#[test]
fn a8_pending_fire_and_submission_drain_in_fifo_order() {
    let mut h = ReactorHarness::new();
    fill_window(&mut h);

    h.reactor
        .dispatch_fire_and_forget(vec![0xF1], false)
        .expect("first fire-and-forget enqueues");
    let (tx, _rx) = sync_channel(1);
    h.reactor
        .dispatch_submission(
            99,
            vec![0xA5],
            "noop".into(),
            tx,
            h.clock.now() + Duration::from_secs(60),
        )
        .expect("submission enqueues");
    h.reactor
        .dispatch_fire_and_forget(vec![0xF2], false)
        .expect("second fire-and-forget enqueues");

    assert_eq!(
        h.reactor
            .pending_outbound_order
            .iter()
            .copied()
            .collect::<Vec<_>>(),
        vec![
            PendingOutboundKind::FireAndForget,
            PendingOutboundKind::Submission,
            PendingOutboundKind::FireAndForget,
        ],
    );

    h.feed_ack_all();
    h.tick();

    let payloads = h
        .reactor
        .unacked_window
        .iter()
        .map(|e| {
            let crc_off = e.frame_bytes.len() - crate::host_io::wire::MESSAGE_TRAILER_SIZE;
            e.frame_bytes[crate::host_io::wire::MESSAGE_HEADER_SIZE..crc_off].to_vec()
        })
        .collect::<Vec<_>>();
    assert_eq!(
        payloads,
        vec![vec![0xF1], vec![0xA5], vec![0xF2]],
        "queued fire-and-forget and response-bearing submissions must preserve FIFO wire order",
    );
    assert!(h.reactor.pending_outbound_order.is_empty());
}

#[test]
fn a8_overflow_returns_backpressure_error() {
    let mut h = ReactorHarness::new();
    fill_window(&mut h);

    // Fill the pending_fire_and_forget queue to the ceiling.
    for _ in 0..PENDING_FIRE_AND_FORGET_CEILING {
        h.reactor
            .dispatch_fire_and_forget(vec![0x01], false)
            .expect("enqueue should succeed up to ceiling");
    }
    assert_eq!(
        h.reactor.pending_fire_and_forget.len(),
        PENDING_FIRE_AND_FORGET_CEILING,
    );

    // The next payload must error with Backpressure (not silent-drop).
    let result = h.reactor.dispatch_fire_and_forget(vec![0x02], false);
    assert!(
        matches!(result, Err(TransportError::Backpressure)),
        "overflow must return Backpressure, got {result:?}",
    );
    // Queue length is unchanged — the payload was rejected, not enqueued.
    assert_eq!(
        h.reactor.pending_fire_and_forget.len(),
        PENDING_FIRE_AND_FORGET_CEILING,
    );
}
