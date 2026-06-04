use std::sync::{Arc, Mutex};
use std::time::Duration;

use kalico_host_rt::clock::Clock;
use kalico_host_rt::host_io::test_harness::ReactorHarness;
use kalico_host_rt::host_io::wire;
use kalico_host_rt::passthrough_queue::{
    ConfigStagePhase, NotifyId, NotifyResponse, PassthroughEntry, PassthroughRouter,
};

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

fn entry(payload: &[u8], min_clock: u64, req_clock: u64) -> PassthroughEntry {
    PassthroughEntry::new(payload.to_vec(), min_clock, req_clock, NotifyId::none())
}

fn entry_with_notify(payload: &[u8], notify_id: NotifyId) -> PassthroughEntry {
    PassthroughEntry::new(payload.to_vec(), 0, 0, notify_id)
}

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

#[test]
fn task24_single_mcu_emission_ordering() {
    let (mut h, mcu, qid) = harness_with_router();

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

#[test]
fn task25_multi_mcu_isolation() {
    let clock = kalico_host_rt::clock::MockClock::new();
    let clock_arc = Arc::clone(&clock) as Arc<dyn kalico_host_rt::clock::Clock + Send + Sync>;
    let mut router = PassthroughRouter::with_clock(clock_arc);

    let mcu_a = router.claim_mcu("mcu_a");
    let mcu_b = router.claim_mcu("mcu_b");
    let qa = router.alloc_command_queue(mcu_a).unwrap();
    let qb = router.alloc_command_queue(mcu_b).unwrap();

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

    let mut a_payloads = Vec::new();
    while let Some(e) = router.pop_next_for_emission(mcu_a).unwrap() {
        a_payloads.push(e.bytes().to_vec());
    }

    let mut b_payloads = Vec::new();
    while let Some(e) = router.pop_next_for_emission(mcu_b).unwrap() {
        b_payloads.push(e.bytes().to_vec());
    }

    assert_eq!(a_payloads.len(), 2, "MCU-A should have exactly 2 entries");
    assert_eq!(b_payloads.len(), 1, "MCU-B should have exactly 1 entry");

    assert!(
        !a_payloads.contains(&vec![0xBB]),
        "MCU-A must not contain MCU-B's payload"
    );
    assert!(
        !b_payloads.contains(&vec![0xAA]),
        "MCU-B must not contain MCU-A's first payload"
    );
    assert!(
        !b_payloads.contains(&vec![0xAB]),
        "MCU-B must not contain MCU-A's second payload"
    );
}

#[test]
fn task26_notify_round_trip() {
    let (mut h, mcu, qid) = harness_with_router();

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

    h.passthrough_push(mcu, qid, entry_with_notify(&[0xDE, 0xAD], notify_id))
        .unwrap();

    h.tick();

    let payloads = extract_payloads(h.tx_log());
    assert_eq!(payloads.len(), 1, "notify-bearing entry should be on wire");

    h.advance_clock(Duration::from_millis(20));

    h.passthrough_dispatch_response(mcu, notify_id, vec![0xBE, 0xEF])
        .unwrap();

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

#[test]
fn task27_window_backpressure() {
    let (mut h, mcu, qid) = harness_with_router();

    for i in 0u8..20 {
        h.passthrough_push(mcu, qid, entry(&[i], 0, u64::from(i)))
            .unwrap();
    }

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

    h.feed_ack_all();

    h.tick();

    let total_payloads = extract_payloads(h.tx_log()).len();
    assert!(
        total_payloads > emitted_first_wave,
        "after ack, more entries should have been emitted; total={} first_wave={}",
        total_payloads,
        emitted_first_wave
    );
}

#[test]
fn task28_config_stage_ordering() {
    let (mut h, mcu, _qid) = harness_with_router();

    h.passthrough_add_config_cmd(mcu, vec![0xC1]).unwrap();
    h.passthrough_add_config_cmd(mcu, vec![0xC2]).unwrap();
    h.passthrough_add_init_cmd(mcu, vec![0xD1]).unwrap();
    h.passthrough_add_init_cmd(mcu, vec![0xD2]).unwrap();

    assert_eq!(
        h.passthrough_config_phase(mcu).unwrap(),
        ConfigStagePhase::Collecting,
        "should still be in Collecting before begin_config_phase"
    );

    h.passthrough_begin_config_phase(mcu).unwrap();
    assert_eq!(
        h.passthrough_config_phase(mcu).unwrap(),
        ConfigStagePhase::SendingConfig,
        "should be SendingConfig after begin_config_phase"
    );

    let entries = h.passthrough_drain_config_entries(mcu).unwrap();

    assert_eq!(entries.len(), 4, "config + init = 4 entries total");

    assert_eq!(entries[0], vec![0xC1], "first config cmd");
    assert_eq!(entries[1], vec![0xC2], "second config cmd");

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

    assert_eq!(
        h.passthrough_config_phase(mcu).unwrap(),
        ConfigStagePhase::Runtime,
        "phase should advance to Runtime after all entries drained"
    );
}

#[test]
fn task28b_runtime_traffic_after_config_completes() {
    let (mut h, mcu, qid) = harness_with_router();

    h.passthrough_add_config_cmd(mcu, vec![0xCF]).unwrap();
    h.passthrough_begin_config_phase(mcu).unwrap();

    let config_entries = h.passthrough_drain_config_entries(mcu).unwrap();
    assert_eq!(config_entries.len(), 1);
    assert_eq!(config_entries[0], vec![0xCF]);

    assert_eq!(
        h.passthrough_config_phase(mcu).unwrap(),
        ConfigStagePhase::Runtime,
    );

    h.passthrough_push(mcu, qid, entry(&[0xEE], 0, 0)).unwrap();
    h.tick();

    let payloads = extract_payloads(h.tx_log());
    assert_eq!(
        payloads.len(),
        1,
        "runtime entry should be on wire after config stage completes"
    );
    assert_eq!(payloads[0], vec![0xEE]);
}
