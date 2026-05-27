use super::*;
use crate::host_io::test_harness::ReactorHarness;
use crate::host_io::wire;
use crate::passthrough_queue::{NotifyId, PassthroughEntry, PassthroughRouter};
use std::sync::Arc;
use std::sync::mpsc::sync_channel;
use std::time::Duration;

/// Build a harness with a passthrough router pre-installed for one MCU.
/// Returns the harness, the MCU handle, and a command queue ID.
fn harness_with_router() -> (
    ReactorHarness,
    crate::passthrough_queue::McuHandle,
    crate::passthrough_queue::CommandQueueId,
) {
    let mut h = ReactorHarness::new();
    let mut router = PassthroughRouter::with_clock(
        Arc::clone(&h.clock) as Arc<dyn crate::clock::Clock + Send + Sync>
    );
    let mcu = router.claim_mcu("test_mcu");
    let qid = router.alloc_command_queue(mcu).unwrap();
    h.reactor.set_passthrough_router(router, mcu);
    (h, mcu, qid)
}

fn entry(payload: &[u8], min_clock: u64, req_clock: u64) -> PassthroughEntry {
    PassthroughEntry::new(payload.to_vec(), min_clock, req_clock, NotifyId::none())
}

fn entry_with_notify(payload: &[u8], notify_id: NotifyId) -> PassthroughEntry {
    PassthroughEntry::new(payload.to_vec(), 0, 0, notify_id)
}

// -----------------------------------------------------------------------
// Test 1: Passthrough entries appear on the mock wire after tick.
// -----------------------------------------------------------------------

#[test]
fn passthrough_entry_appears_on_wire() {
    let (mut h, mcu, qid) = harness_with_router();

    // Push one entry directly into the router.
    h.reactor
        .passthrough_router
        .as_mut()
        .unwrap()
        .push(mcu, qid, entry(&[0xAA, 0xBB], 0, 0))
        .unwrap();

    let tx_before = h.tx_log().len();
    h.tick();
    let tx_after = h.tx_log().len();

    // A frame was written: 5 (wire overhead) + 2 (payload) = 7 bytes.
    assert_eq!(
        tx_after - tx_before,
        7,
        "passthrough entry should produce a 7-byte wire frame"
    );
    assert_eq!(h.unacked_depth(), 1, "entry should be in unacked window");
}

// -----------------------------------------------------------------------
// Test 2: Multiple passthrough entries emit in req_clock order.
// -----------------------------------------------------------------------

#[test]
fn passthrough_entries_emit_in_req_clock_order() {
    let (mut h, mcu, qid) = harness_with_router();

    // Push entries with different req_clock values (out of order).
    let router = h.reactor.passthrough_router.as_mut().unwrap();
    router.push(mcu, qid, entry(&[0x03], 0, 300)).unwrap();
    router.push(mcu, qid, entry(&[0x01], 0, 100)).unwrap();
    router.push(mcu, qid, entry(&[0x02], 0, 200)).unwrap();

    h.tick();
    assert_eq!(h.unacked_depth(), 3);

    // Check wire order: extract frames from tx_log.
    let tx = h.tx_log();
    let mut frames = Vec::new();
    let mut buf = tx.clone();
    while let Some(pkt) = wire::extract_packet(&mut buf) {
        // Payload is bytes [2..msglen-3].
        let msglen = pkt[0] as usize;
        let payload = pkt[2..msglen - 3].to_vec();
        frames.push(payload);
    }
    assert_eq!(frames.len(), 3, "should have 3 frames");
    assert_eq!(frames[0], vec![0x01], "first frame should be req_clock=100");
    assert_eq!(
        frames[1],
        vec![0x02],
        "second frame should be req_clock=200"
    );
    assert_eq!(frames[2], vec![0x03], "third frame should be req_clock=300");
}

// -----------------------------------------------------------------------
// Test 3: Passthrough and typed commands interleave on the same wire.
// -----------------------------------------------------------------------

#[test]
fn passthrough_interleaves_with_typed_commands() {
    let (mut h, mcu, qid) = harness_with_router();

    // Submit a typed command (goes through dispatch_submission → wire).
    let (tx, _rx) = sync_channel(1);
    let _ = h.reactor.dispatch_submission(
        1,
        vec![0xCC],
        "noop".into(),
        tx,
        h.clock.now() + Duration::from_secs(60),
    );

    // Push a passthrough entry.
    h.reactor
        .passthrough_router
        .as_mut()
        .unwrap()
        .push(mcu, qid, entry(&[0xDD], 0, 0))
        .unwrap();

    h.tick();

    // Both should be in the unacked window. The typed command was
    // dispatched at submit time (before tick); the passthrough entry
    // is drained during tick's step 3b.
    assert_eq!(
        h.unacked_depth(),
        2,
        "both typed and passthrough should be in-flight"
    );

    // Verify both payloads are on the wire.
    let tx_log = h.tx_log();
    let mut buf = tx_log;
    let mut payloads = Vec::new();
    while let Some(pkt) = wire::extract_packet(&mut buf) {
        let msglen = pkt[0] as usize;
        if msglen > wire::MESSAGE_MIN {
            payloads.push(pkt[2..msglen - 3].to_vec());
        }
    }
    assert_eq!(payloads.len(), 2);
    assert!(
        payloads.contains(&vec![0xCC]),
        "typed command payload on wire"
    );
    assert!(
        payloads.contains(&vec![0xDD]),
        "passthrough payload on wire"
    );
}

// -----------------------------------------------------------------------
// Test 4: Window backpressure stops passthrough emission.
// -----------------------------------------------------------------------

#[test]
fn window_backpressure_stops_passthrough_emission() {
    let (mut h, mcu, qid) = harness_with_router();

    // Push more entries than the unacked window allows (MAX_PENDING_BLOCKS=12).
    let router = h.reactor.passthrough_router.as_mut().unwrap();
    for i in 0..20u8 {
        router.push(mcu, qid, entry(&[i], 0, i as u64)).unwrap();
    }

    h.tick();

    // The reactor's unacked window should be full but not overflow.
    assert!(
        h.unacked_depth() <= crate::host_io::window::MAX_PENDING_BLOCKS,
        "unacked window must not exceed MAX_PENDING_BLOCKS, got {}",
        h.unacked_depth()
    );
    assert!(
        h.unacked_depth() > 0,
        "some entries should have been emitted"
    );
}

// -----------------------------------------------------------------------
// Test 5: InstallPassthroughRouter command installs via mpsc.
// -----------------------------------------------------------------------

#[test]
fn install_passthrough_router_via_command() {
    let mut h = ReactorHarness::new();

    // Router is not installed yet.
    assert!(h.reactor.passthrough_router.is_none());
    assert!(h.reactor.passthrough_mcu.is_none());

    // Create and send the router via the command channel.
    let mut router = PassthroughRouter::with_clock(
        Arc::clone(&h.clock) as Arc<dyn crate::clock::Clock + Send + Sync>
    );
    let mcu = router.claim_mcu("test_mcu");
    let _qid = router.alloc_command_queue(mcu).unwrap();

    h.submission_tx
        .send(ReactorCommand::InstallPassthroughRouter(router))
        .unwrap();
    h.tick();

    assert!(
        h.reactor.passthrough_router.is_some(),
        "router should be installed"
    );
    assert_eq!(
        h.reactor.passthrough_mcu,
        Some(mcu),
        "MCU handle should be set"
    );
}

// -----------------------------------------------------------------------
// Test 6: PassthroughSend command pushes entries via mpsc.
// -----------------------------------------------------------------------

#[test]
fn passthrough_send_via_command() {
    let (mut h, mcu, qid) = harness_with_router();

    // Send entry via the command channel.
    h.submission_tx
        .send(ReactorCommand::PassthroughSend {
            mcu,
            queue_id: qid,
            entry: entry(&[0xEE], 0, 0),
        })
        .unwrap();

    // First tick: drain command (pushes into router).
    // Same tick's step 3b: drain_passthrough emits it.
    h.tick();

    assert_eq!(h.unacked_depth(), 1, "entry should be emitted");
    let tx_log = h.tx_log();
    let mut buf = tx_log;
    let pkt = wire::extract_packet(&mut buf).expect("frame on wire");
    let msglen = pkt[0] as usize;
    let payload = &pkt[2..msglen - 3];
    assert_eq!(payload, &[0xEE]);
}

// -----------------------------------------------------------------------
// Test 7: Shared sequence numbers between typed and passthrough.
// -----------------------------------------------------------------------

#[test]
fn shared_sequence_numbers() {
    let (mut h, mcu, qid) = harness_with_router();

    // Submit a typed command first (gets seq=1).
    let (tx, _rx) = sync_channel(1);
    let _ = h.reactor.dispatch_submission(
        1,
        vec![0xAA],
        "noop".into(),
        tx,
        h.clock.now() + Duration::from_secs(60),
    );

    // Push a passthrough entry (should get seq=2).
    h.reactor
        .passthrough_router
        .as_mut()
        .unwrap()
        .push(mcu, qid, entry(&[0xBB], 0, 0))
        .unwrap();

    h.tick();

    // Extract wire-seq nibbles from the two frames.
    let tx_log = h.tx_log();
    let mut buf = tx_log;
    let mut wire_seqs = Vec::new();
    while let Some(pkt) = wire::extract_packet(&mut buf) {
        let wire_seq = pkt[1] & wire::MESSAGE_SEQ_MASK;
        wire_seqs.push(wire_seq);
    }
    assert_eq!(wire_seqs.len(), 2, "two frames on wire");
    // Seq numbers should be consecutive (1, 2 — mod 16).
    assert_eq!(wire_seqs[0], 1, "typed command gets wire seq 1");
    assert_eq!(wire_seqs[1], 2, "passthrough entry gets wire seq 2");
    assert_eq!(h.reactor.send_seq, 3, "send_seq advanced to 3");
}

// -----------------------------------------------------------------------
// Test 8: ACK frees receive window for passthrough router.
// -----------------------------------------------------------------------

#[test]
fn ack_frees_passthrough_receive_window() {
    let (mut h, mcu, qid) = harness_with_router();

    // Push enough entries to fill the router's receive window.
    let router = h.reactor.passthrough_router.as_mut().unwrap();
    for i in 0..20u8 {
        router.push(mcu, qid, entry(&[i], 0, i as u64)).unwrap();
    }

    h.tick();
    let emitted_first = h.unacked_depth();
    assert!(emitted_first > 0, "some entries should have been emitted");

    // If the router's window blocked emission, there should be entries
    // left. Acknowledge all outstanding frames to free the window.
    let rseq = h.reactor.send_seq;
    let wire_nibble = (rseq & 0x0F) as u8;
    h.feed_rx(&wire::build_frame(&[], wire_nibble));
    h.tick();

    // After ack, the window should have freed and more entries emitted.
    let emitted_total = h.unacked_depth();
    // We might have emitted more, or the window was the bottleneck rather
    // than the reactor's unacked window. Either way, a second batch of
    // emission should have occurred.
    assert!(
        emitted_total > 0,
        "after ack, more entries should be in flight or window was not the bottleneck"
    );
}

// -----------------------------------------------------------------------
// Test 9: No passthrough router installed — tick is a no-op for passthrough.
// -----------------------------------------------------------------------

#[test]
fn no_router_installed_tick_is_noop() {
    let mut h = ReactorHarness::new();
    // No router installed; tick should not crash.
    let outcome = h.tick();
    assert_eq!(outcome, TickOutcome::Continue);
    assert_eq!(h.unacked_depth(), 0);
    assert!(h.tx_log().is_empty());
}

// -----------------------------------------------------------------------
// Test 10: Passthrough notify map is populated for notify-bearing entries.
// -----------------------------------------------------------------------

#[test]
fn notify_bearing_entry_tracked_in_map() {
    let (mut h, mcu, qid) = harness_with_router();

    let nid = NotifyId::new(42);
    h.reactor
        .passthrough_router
        .as_mut()
        .unwrap()
        .push(mcu, qid, entry_with_notify(&[0xFF], nid))
        .unwrap();

    h.tick();

    // The notify map should have an entry keyed by the seq that was used.
    assert_eq!(h.reactor.passthrough_notify_map.len(), 1);
    let (&seq, &(mapped_mcu, mapped_nid)) =
        h.reactor.passthrough_notify_map.iter().next().unwrap();
    assert_eq!(seq, 1, "first emission gets seq=1");
    assert_eq!(mapped_mcu, mcu);
    assert_eq!(mapped_nid, nid);
}

// -----------------------------------------------------------------------
// Test 11: Fire-and-forget (no notify) entries do not populate the map.
// -----------------------------------------------------------------------

#[test]
fn no_notify_entry_not_in_map() {
    let (mut h, mcu, qid) = harness_with_router();

    h.reactor
        .passthrough_router
        .as_mut()
        .unwrap()
        .push(mcu, qid, entry(&[0x01], 0, 0))
        .unwrap();

    h.tick();

    assert!(
        h.reactor.passthrough_notify_map.is_empty(),
        "fire-and-forget entries should not populate notify map"
    );
}
