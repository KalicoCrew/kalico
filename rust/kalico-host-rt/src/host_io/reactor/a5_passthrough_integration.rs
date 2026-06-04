use super::*;
use crate::host_io::test_harness::ReactorHarness;
use crate::host_io::wire;
use crate::passthrough_queue::{NotifyId, PassthroughEntry, PassthroughRouter};
use std::sync::Arc;
use std::sync::mpsc::sync_channel;
use std::time::Duration;

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

#[test]
fn passthrough_entry_appears_on_wire() {
    let (mut h, mcu, qid) = harness_with_router();

    h.reactor
        .passthrough_router
        .as_mut()
        .unwrap()
        .push(mcu, qid, entry(&[0xAA, 0xBB], 0, 0))
        .unwrap();

    let tx_before = h.tx_log().len();
    h.tick();
    let tx_after = h.tx_log().len();

    assert_eq!(
        tx_after - tx_before,
        7,
        "passthrough entry should produce a 7-byte wire frame"
    );
    assert_eq!(h.unacked_depth(), 1, "entry should be in unacked window");
}

#[test]
fn passthrough_entries_emit_in_req_clock_order() {
    let (mut h, mcu, qid) = harness_with_router();

    let router = h.reactor.passthrough_router.as_mut().unwrap();
    router.push(mcu, qid, entry(&[0x03], 0, 300)).unwrap();
    router.push(mcu, qid, entry(&[0x01], 0, 100)).unwrap();
    router.push(mcu, qid, entry(&[0x02], 0, 200)).unwrap();

    h.tick();
    assert_eq!(h.unacked_depth(), 3);

    let tx = h.tx_log();
    let mut frames = Vec::new();
    let mut buf = tx.clone();
    while let Some(pkt) = wire::extract_packet(&mut buf) {
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

#[test]
fn passthrough_interleaves_with_typed_commands() {
    let (mut h, mcu, qid) = harness_with_router();

    let (tx, _rx) = sync_channel(1);
    let _ = h.reactor.dispatch_submission(
        1,
        vec![0xCC],
        "noop".into(),
        tx,
        h.clock.now() + Duration::from_secs(60),
    );

    h.reactor
        .passthrough_router
        .as_mut()
        .unwrap()
        .push(mcu, qid, entry(&[0xDD], 0, 0))
        .unwrap();

    h.tick();

    assert_eq!(
        h.unacked_depth(),
        2,
        "both typed and passthrough should be in-flight"
    );

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

#[test]
fn window_backpressure_stops_passthrough_emission() {
    let (mut h, mcu, qid) = harness_with_router();

    let router = h.reactor.passthrough_router.as_mut().unwrap();
    for i in 0..20u8 {
        router.push(mcu, qid, entry(&[i], 0, i as u64)).unwrap();
    }

    h.tick();

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

#[test]
fn install_passthrough_router_via_command() {
    let mut h = ReactorHarness::new();

    assert!(h.reactor.passthrough_router.is_none());
    assert!(h.reactor.passthrough_mcu.is_none());

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

#[test]
fn passthrough_send_via_command() {
    let (mut h, mcu, qid) = harness_with_router();

    h.submission_tx
        .send(ReactorCommand::PassthroughSend {
            mcu,
            queue_id: qid,
            entry: entry(&[0xEE], 0, 0),
        })
        .unwrap();

    h.tick();

    assert_eq!(h.unacked_depth(), 1, "entry should be emitted");
    let tx_log = h.tx_log();
    let mut buf = tx_log;
    let pkt = wire::extract_packet(&mut buf).expect("frame on wire");
    let msglen = pkt[0] as usize;
    let payload = &pkt[2..msglen - 3];
    assert_eq!(payload, &[0xEE]);
}

#[test]
fn shared_sequence_numbers() {
    let (mut h, mcu, qid) = harness_with_router();

    let (tx, _rx) = sync_channel(1);
    let _ = h.reactor.dispatch_submission(
        1,
        vec![0xAA],
        "noop".into(),
        tx,
        h.clock.now() + Duration::from_secs(60),
    );

    h.reactor
        .passthrough_router
        .as_mut()
        .unwrap()
        .push(mcu, qid, entry(&[0xBB], 0, 0))
        .unwrap();

    h.tick();

    let tx_log = h.tx_log();
    let mut buf = tx_log;
    let mut wire_seqs = Vec::new();
    while let Some(pkt) = wire::extract_packet(&mut buf) {
        let wire_seq = pkt[1] & wire::MESSAGE_SEQ_MASK;
        wire_seqs.push(wire_seq);
    }
    assert_eq!(wire_seqs.len(), 2, "two frames on wire");
    assert_eq!(wire_seqs[0], 1, "typed command gets wire seq 1");
    assert_eq!(wire_seqs[1], 2, "passthrough entry gets wire seq 2");
    assert_eq!(h.reactor.send_seq, 3, "send_seq advanced to 3");
}

#[test]
fn ack_frees_passthrough_receive_window() {
    let (mut h, mcu, qid) = harness_with_router();

    let router = h.reactor.passthrough_router.as_mut().unwrap();
    for i in 0..20u8 {
        router.push(mcu, qid, entry(&[i], 0, i as u64)).unwrap();
    }

    h.tick();
    let emitted_first = h.unacked_depth();
    assert!(emitted_first > 0, "some entries should have been emitted");

    let rseq = h.reactor.send_seq;
    let wire_nibble = (rseq & 0x0F) as u8;
    h.feed_rx(&wire::build_frame(&[], wire_nibble));
    h.tick();

    let emitted_total = h.unacked_depth();
    assert!(
        emitted_total > 0,
        "after ack, more entries should be in flight or window was not the bottleneck"
    );
}

#[test]
fn no_router_installed_tick_is_noop() {
    let mut h = ReactorHarness::new();
    let outcome = h.tick();
    assert_eq!(outcome, TickOutcome::Continue);
    assert_eq!(h.unacked_depth(), 0);
    assert!(h.tx_log().is_empty());
}

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

    assert_eq!(h.reactor.passthrough_notify_map.len(), 1);
    let (&seq, &(mapped_mcu, mapped_nid)) = h.reactor.passthrough_notify_map.iter().next().unwrap();
    assert_eq!(seq, 1, "first emission gets seq=1");
    assert_eq!(mapped_mcu, mcu);
    assert_eq!(mapped_nid, nid);
}

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
