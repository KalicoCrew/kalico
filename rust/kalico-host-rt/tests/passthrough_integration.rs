//! End-to-end integration tests for the passthrough_queue module wired into
//! the reactor via `ReactorHarness` (Tasks 24-28).
//!
//! These tests use the `test-harness` feature which exposes `ReactorHarness`
//! and helpers for passthrough operations.

use std::sync::{Arc, Mutex};
use std::time::Duration;

use kalico_host_rt::clock::Clock;
use kalico_host_rt::host_io::test_harness::ReactorHarness;
use kalico_host_rt::host_io::wire;
use kalico_host_rt::passthrough_queue::{
    ConfigStagePhase, NotifyId, NotifyResponse, PassthroughEntry, PassthroughRouter,
};

// ---------------------------------------------------------------------------
// Helper: build a harness with one MCU + one command queue pre-installed.
// Returns (harness, mcu_handle, queue_id).
// ---------------------------------------------------------------------------

fn harness_with_router() -> (
    ReactorHarness,
    kalico_host_rt::passthrough_queue::McuHandle,
    kalico_host_rt::passthrough_queue::CommandQueueId,
) {
    let mut h = ReactorHarness::new();
    let mut router = PassthroughRouter::with_clock(
        Arc::clone(&h.clock) as Arc<dyn kalico_host_rt::clock::Clock + Send + Sync>
    );
    let mcu = router.claim_mcu("test_mcu");
    let qid = router.alloc_command_queue(mcu).unwrap();
    h.install_passthrough_router(router, mcu);
    (h, mcu, qid)
}

/// Construct a `PassthroughEntry` with given payload and clock values.
fn entry(payload: &[u8], min_clock: u64, req_clock: u64) -> PassthroughEntry {
    PassthroughEntry::new(payload.to_vec(), min_clock, req_clock, NotifyId::none())
}

/// Construct a `PassthroughEntry` with a notify ID.
fn entry_with_notify(payload: &[u8], notify_id: NotifyId) -> PassthroughEntry {
    PassthroughEntry::new(payload.to_vec(), 0, 0, notify_id)
}

/// Parse all frames from raw wire bytes; returns a `Vec<Vec<u8>>` of payloads.
fn extract_payloads(tx_bytes: Vec<u8>) -> Vec<Vec<u8>> {
    let mut buf = tx_bytes;
    let mut payloads = Vec::new();
    while let Some(pkt) = wire::extract_packet(&mut buf) {
        let msglen = pkt[0] as usize;
        if msglen > wire::MESSAGE_MIN {
            payloads.push(pkt[2..msglen - 3].to_vec());
        }
    }
    payloads
}

// ---------------------------------------------------------------------------
// Task 24: Single-MCU emission ordering
// Push three entries with different req_clocks; verify wire order is
// lowest req_clock first.
// ---------------------------------------------------------------------------

#[test]
fn task24_single_mcu_emission_ordering() {
    let (mut h, mcu, qid) = harness_with_router();

    // Push in descending req_clock order so the test validates sorting.
    h.passthrough_push(mcu, qid, entry(&[0x03], 0, 300))
        .unwrap();
    h.passthrough_push(mcu, qid, entry(&[0x01], 0, 100))
        .unwrap();
    h.passthrough_push(mcu, qid, entry(&[0x02], 0, 200))
        .unwrap();

    h.tick();

    let payloads = extract_payloads(h.tx_log());
    assert_eq!(payloads.len(), 3, "all three entries should be on the wire");
    assert_eq!(payloads[0], vec![0x01], "first emission: req_clock=100");
    assert_eq!(payloads[1], vec![0x02], "second emission: req_clock=200");
    assert_eq!(payloads[2], vec![0x03], "third emission: req_clock=300");
}

// ---------------------------------------------------------------------------
// Task 25: Multi-MCU isolation
// Two MCU handles on a single router; entries pushed to MCU-A must not
// appear when iterating over MCU-B's emissions, and vice versa.
//
// Since each reactor owns one MCU, we verify isolation by querying the
// router directly: push to both, then drain each MCU's queue through
// pop_next_for_emission and confirm payloads belong to the correct MCU.
// ---------------------------------------------------------------------------

#[test]
fn task25_multi_mcu_isolation() {
    // Build a router with two MCUs and separate command queues.
    let clock = kalico_host_rt::clock::MockClock::new();
    let clock_arc = Arc::clone(&clock) as Arc<dyn kalico_host_rt::clock::Clock + Send + Sync>;
    let mut router = PassthroughRouter::with_clock(clock_arc);

    let mcu_a = router.claim_mcu("mcu_a");
    let mcu_b = router.claim_mcu("mcu_b");
    let qa = router.alloc_command_queue(mcu_a).unwrap();
    let qb = router.alloc_command_queue(mcu_b).unwrap();

    // Push distinct payloads to each MCU.
    router
        .push(
            mcu_a,
            qa,
            PassthroughEntry::new(vec![0xAA], 0, 1, NotifyId::none()),
        )
        .unwrap();
    router
        .push(
            mcu_a,
            qa,
            PassthroughEntry::new(vec![0xAB], 0, 2, NotifyId::none()),
        )
        .unwrap();
    router
        .push(
            mcu_b,
            qb,
            PassthroughEntry::new(vec![0xBB], 0, 1, NotifyId::none()),
        )
        .unwrap();

    // Drain MCU-A.
    let mut a_payloads = Vec::new();
    while let Some(e) = router.pop_next_for_emission(mcu_a).unwrap() {
        a_payloads.push(e.bytes().to_vec());
    }

    // Drain MCU-B.
    let mut b_payloads = Vec::new();
    while let Some(e) = router.pop_next_for_emission(mcu_b).unwrap() {
        b_payloads.push(e.bytes().to_vec());
    }

    // MCU-A should have its two entries; MCU-B should have its one entry.
    assert_eq!(a_payloads.len(), 2, "MCU-A should have exactly 2 entries");
    assert_eq!(b_payloads.len(), 1, "MCU-B should have exactly 1 entry");

    // No cross-talk: MCU-A payloads must not contain 0xBB.
    assert!(
        !a_payloads.contains(&vec![0xBB]),
        "MCU-A must not contain MCU-B's payload"
    );
    // MCU-B payloads must not contain MCU-A payloads.
    assert!(
        !b_payloads.contains(&vec![0xAA]),
        "MCU-B must not contain MCU-A's first payload"
    );
    assert!(
        !b_payloads.contains(&vec![0xAB]),
        "MCU-B must not contain MCU-A's second payload"
    );
}

// ---------------------------------------------------------------------------
// Task 26: Notify round-trip
// Push a query with a notify_id. Simulate a response. Verify the callback
// fires with correct sent_time <= receive_time and correct response bytes.
// ---------------------------------------------------------------------------

#[test]
fn task26_notify_round_trip() {
    let (mut h, mcu, qid) = harness_with_router();

    // Register a notify callback.
    let captured: Arc<Mutex<Option<NotifyResponse>>> = Arc::new(Mutex::new(None));
    let captured2 = Arc::clone(&captured);

    let notify_id = h
        .passthrough_register_notify(
            mcu,
            Box::new(move |resp| {
                *captured2.lock().unwrap() = Some(resp);
            }),
        )
        .unwrap();

    // Push a notify-bearing entry.
    h.passthrough_push(mcu, qid, entry_with_notify(&[0xDE, 0xAD], notify_id))
        .unwrap();

    // Tick so the reactor emits the entry (records sent_time).
    h.tick();

    // Verify entry is on the wire.
    let payloads = extract_payloads(h.tx_log());
    assert_eq!(payloads.len(), 1, "notify-bearing entry should be on wire");

    // Advance clock to ensure receive_time > sent_time.
    h.advance_clock(Duration::from_millis(20));

    // Dispatch the simulated response through the harness helper.
    h.passthrough_dispatch_response(mcu, notify_id, vec![0xBE, 0xEF])
        .unwrap();

    // The callback should have fired.
    let resp = captured
        .lock()
        .unwrap()
        .take()
        .expect("callback must have fired");

    assert_eq!(resp.bytes, vec![0xBE, 0xEF], "response bytes must match");
    assert!(
        resp.receive_time >= resp.sent_time,
        "receive_time ({}) must be >= sent_time ({})",
        resp.receive_time,
        resp.sent_time,
    );
}

// ---------------------------------------------------------------------------
// Task 27: Window backpressure
// Fill the receive window; verify emission stops. Ack in-flight bytes;
// verify emission resumes.
// ---------------------------------------------------------------------------

#[test]
fn task27_window_backpressure() {
    let (mut h, mcu, qid) = harness_with_router();

    // Push 20 entries — more than the default window (12 pending blocks).
    for i in 0u8..20 {
        h.passthrough_push(mcu, qid, entry(&[i], 0, u64::from(i)))
            .unwrap();
    }

    // First tick: the router's receive window will block after a few entries.
    h.tick();

    let emitted_first_wave = extract_payloads(h.tx_log()).len();
    assert!(
        emitted_first_wave > 0,
        "at least some entries should be emitted"
    );
    assert!(
        emitted_first_wave < 20,
        "window backpressure should block before all 20 entries emit; emitted={}",
        emitted_first_wave
    );

    // Ack all outstanding frames via the reactor's wire protocol. This frees
    // the reactor's unacked window AND (via the reactor's ack path) also
    // calls `router.record_ack` for the passthrough router's receive window.
    h.feed_ack_all();

    // Second tick: processes the ACK, freeing both windows.
    h.tick();

    let total_payloads = extract_payloads(h.tx_log()).len();
    assert!(
        total_payloads > emitted_first_wave,
        "after ack, more entries should have been emitted; total={} first_wave={}",
        total_payloads,
        emitted_first_wave
    );
}

// ---------------------------------------------------------------------------
// Task 28: Config-stage emission ordering
// Register config_cmds and init_cmds. Begin config phase. Verify config
// commands drain before init commands.
// ---------------------------------------------------------------------------

#[test]
fn task28_config_stage_ordering() {
    let (mut h, mcu, _qid) = harness_with_router();

    // Register two config commands and two init commands.
    h.passthrough_add_config_cmd(mcu, vec![0xC1]).unwrap();
    h.passthrough_add_config_cmd(mcu, vec![0xC2]).unwrap();
    h.passthrough_add_init_cmd(mcu, vec![0xD1]).unwrap();
    h.passthrough_add_init_cmd(mcu, vec![0xD2]).unwrap();

    // Phase should still be Collecting.
    assert_eq!(
        h.passthrough_config_phase(mcu).unwrap(),
        ConfigStagePhase::Collecting,
        "should still be in Collecting before begin_config_phase"
    );

    // Transition to SendingConfig.
    h.passthrough_begin_config_phase(mcu).unwrap();
    assert_eq!(
        h.passthrough_config_phase(mcu).unwrap(),
        ConfigStagePhase::SendingConfig,
        "should be SendingConfig after begin_config_phase"
    );

    // Drain all config/init entries.
    let entries = h.passthrough_drain_config_entries(mcu).unwrap();

    // All four entries should have been yielded.
    assert_eq!(entries.len(), 4, "config + init = 4 entries total");

    // Config commands come first.
    assert_eq!(entries[0], vec![0xC1], "first config cmd");
    assert_eq!(entries[1], vec![0xC2], "second config cmd");

    // Init commands follow.
    assert_eq!(
        entries[2],
        vec![0xD1],
        "first init cmd must follow config cmds"
    );
    assert_eq!(
        entries[3],
        vec![0xD2],
        "second init cmd must follow config cmds"
    );

    // After all entries consumed, phase should be Runtime.
    assert_eq!(
        h.passthrough_config_phase(mcu).unwrap(),
        ConfigStagePhase::Runtime,
        "phase should advance to Runtime after all entries drained"
    );
}

// ---------------------------------------------------------------------------
// Task 28b: Config commands complete before runtime traffic flows
// After the config stage drains, runtime push entries should be emittable.
// ---------------------------------------------------------------------------

#[test]
fn task28b_runtime_traffic_after_config_completes() {
    let (mut h, mcu, qid) = harness_with_router();

    // Register one config command.
    h.passthrough_add_config_cmd(mcu, vec![0xCF]).unwrap();
    h.passthrough_begin_config_phase(mcu).unwrap();

    // Drain the config entry manually (simulating the bridge draining it).
    let config_entries = h.passthrough_drain_config_entries(mcu).unwrap();
    assert_eq!(config_entries.len(), 1);
    assert_eq!(config_entries[0], vec![0xCF]);

    // Now we're in Runtime phase.
    assert_eq!(
        h.passthrough_config_phase(mcu).unwrap(),
        ConfigStagePhase::Runtime,
    );

    // Push a runtime entry.
    h.passthrough_push(mcu, qid, entry(&[0xEE], 0, 0)).unwrap();
    h.tick();

    // Runtime entry should appear on the wire.
    let payloads = extract_payloads(h.tx_log());
    assert_eq!(
        payloads.len(),
        1,
        "runtime entry should be on wire after config stage completes"
    );
    assert_eq!(payloads[0], vec![0xEE]);
}
