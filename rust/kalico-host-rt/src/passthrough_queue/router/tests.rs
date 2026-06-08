use super::*;
use crate::clock::MockClock;
use crate::passthrough_queue::entry::NotifyId;
use std::sync::Mutex;
use std::time::Duration;

fn make_router() -> (PassthroughRouter, Arc<MockClock>) {
    let clock = MockClock::new();
    let router = PassthroughRouter::with_clock(Arc::clone(&clock) as Arc<dyn Clock + Send + Sync>);
    (router, clock)
}

fn entry(min_clock: u64, req_clock: u64) -> PassthroughEntry {
    PassthroughEntry::new(vec![0x01], min_clock, req_clock, NotifyId::none())
}

fn entry_with_notify(min_clock: u64, req_clock: u64, nid: NotifyId) -> PassthroughEntry {
    PassthroughEntry::new(vec![0x01], min_clock, req_clock, nid)
}

#[test]
fn two_mcus_claim_release_independently() {
    let (mut router, _) = make_router();
    let a = router.claim_mcu("mcu_a");
    let b = router.claim_mcu("mcu_b");
    assert_ne!(a, b);

    router.release_mcu(a);
    let q = router.alloc_command_queue(b);
    assert!(q.is_ok());
    assert!(router.alloc_command_queue(a).is_err());
}

#[test]
fn alloc_command_queue_per_mcu() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q1 = router.alloc_command_queue(mcu).unwrap();
    let q2 = router.alloc_command_queue(mcu).unwrap();
    assert_ne!(q1, q2);
}

#[test]
fn push_routes_correctly_through_mcu_state() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    router.push(mcu, q, entry(0, 100)).unwrap();
    router.push(mcu, q, entry(0, 50)).unwrap();

    let e = router.pop_next_for_emission(mcu).unwrap().unwrap();
    assert_eq!(e.req_clock(), 50);
}

#[test]
fn register_notify_and_dispatch_response_round_trip() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    let captured = Arc::new(Mutex::new(None));
    let captured2 = Arc::clone(&captured);
    let nid = router
        .register_notify(
            mcu,
            Box::new(move |resp| {
                *captured2.lock().unwrap() = Some(resp);
            }),
        )
        .unwrap();

    router.push(mcu, q, entry_with_notify(0, 10, nid)).unwrap();

    let _ = router.pop_next_for_emission(mcu).unwrap();

    clock.advance(Duration::from_millis(50));

    router
        .dispatch_response(mcu, nid, vec![0xBE, 0xEF])
        .unwrap();

    let resp = captured.lock().unwrap().take().unwrap();
    assert_eq!(resp.bytes, vec![0xBE, 0xEF]);
    assert!(resp.receive_time >= resp.sent_time);
}

#[test]
fn pop_next_for_emission_respects_window_gate() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    for i in 0..20 {
        router.push(mcu, q, entry(0, i)).unwrap();
    }

    let mut emitted = 0u32;
    while router.pop_next_for_emission(mcu).unwrap().is_some() {
        emitted += 1;
        if emitted > 100 {
            panic!("window gate did not kick in");
        }
    }
    assert!(emitted > 0);
    assert!(emitted < 20);
}

#[test]
fn record_ack_frees_window_capacity() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    for i in 0..20 {
        router.push(mcu, q, entry(0, i)).unwrap();
    }

    while router.pop_next_for_emission(mcu).unwrap().is_some() {}

    router.record_ack(mcu, 50).unwrap();

    let got = router.pop_next_for_emission(mcu).unwrap();
    assert!(got.is_some());
}

#[test]
fn set_clock_est_stores_values() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");

    assert_eq!(router.compute_ack_clock(mcu).unwrap(), 0);

    router.set_clock_est(mcu, 48_000_000.0, 0.0, 1000).unwrap();

    let ack = router.compute_ack_clock(mcu).unwrap();
    assert!(ack >= 1000, "ack_clock should be at least last_clock");
}

#[test]
fn compute_ack_clock_projects_from_host_time() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let base_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 1_000_000.0, base_host, 0)
        .unwrap();

    let ack0 = router.compute_ack_clock(mcu).unwrap();
    assert_eq!(ack0, 0);

    clock.advance(Duration::from_secs(1));
    let ack1 = router.compute_ack_clock(mcu).unwrap();
    let diff = (ack1 as i64 - 1_000_000_i64).unsigned_abs();
    assert!(diff <= 1, "expected ~1_000_000, got {ack1}");
}

#[test]
fn set_clock_est_rebased_advances_with_mock_clock() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    // `offset_raw` must be in the same RAW-domain epoch as `bridge_now_raw`
    // inside `set_clock_est_rebased`.  Capture it here so the conversion
    // `raw_at_anchor = bridge_now_raw - bridge_now_instant` yields a small,
    // positive number and `clock_offset` lands near the mock clock's current
    // Instant value.
    let offset_raw = crate::clock::monotonic_raw_secs();

    router
        .set_clock_est_rebased(mcu, 1_000_000.0, offset_raw, 10_000_000, 0.0)
        .unwrap();

    let ack0 = router.compute_ack_clock(mcu).unwrap();

    clock.advance(Duration::from_secs(1));
    let ack1 = router.compute_ack_clock(mcu).unwrap();
    let diff = (ack1 as i64 - ack0 as i64 - 1_000_000_i64).unsigned_abs();
    assert!(
        diff <= 1,
        "ack_clock must advance by ~1_000_000 ticks per second; \
         ack0={ack0} ack1={ack1} diff={diff}"
    );
}

#[test]
fn set_clock_est_rebased_epsilon_independent() {
    let freq = 1_000_000.0_f64;
    let offset_raw = 990.0_f64;
    let last_clock = 10_000_000_u64;

    let (mut router_a, _clock_a) = make_router();
    let mcu_a = router_a.claim_mcu("mcu_a");

    let (mut router_b, _clock_b) = make_router();
    let mcu_b = router_b.claim_mcu("mcu_b");

    router_a
        .set_clock_est_rebased(mcu_a, freq, offset_raw, last_clock, 1000.0)
        .unwrap();

    router_b
        .set_clock_est_rebased(mcu_b, freq, offset_raw, last_clock, 1000.0 - 0.050)
        .unwrap();

    let ack_a = router_a.compute_ack_clock(mcu_a).unwrap();
    let ack_b = router_b.compute_ack_clock(mcu_b).unwrap();

    let diff = (ack_a as i64 - ack_b as i64).unsigned_abs();
    assert!(
        diff <= 2,
        "compute_ack_clock must not vary with host_now_raw (ε-independence); \
         host_now_raw differed by 50 ms but ack_clock differed by {diff} ticks \
         (ack_a={ack_a} ack_b={ack_b})"
    );
}

#[test]
fn compute_ack_clock_unknown_mcu_errors() {
    let (router, _) = make_router();
    let bogus = McuHandle(999);
    assert!(router.compute_ack_clock(bogus).is_err());
}

#[test]
fn flush_callback_fires_on_non_empty_to_empty_transition() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    let count = Arc::new(Mutex::new(0u32));
    let count2 = Arc::clone(&count);
    router
        .register_flush_callback(
            mcu,
            Box::new(move || {
                *count2.lock().unwrap() += 1;
            }),
        )
        .unwrap();

    router.push(mcu, q, entry(0, 10)).unwrap();
    let _ = router.pop_next_for_emission(mcu).unwrap();

    router.check_flush(mcu).unwrap();
    assert_eq!(*count.lock().unwrap(), 1);
}

#[test]
fn flush_callback_does_not_fire_if_never_non_empty() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let _q = router.alloc_command_queue(mcu).unwrap();

    let count = Arc::new(Mutex::new(0u32));
    let count2 = Arc::clone(&count);
    router
        .register_flush_callback(
            mcu,
            Box::new(move || {
                *count2.lock().unwrap() += 1;
            }),
        )
        .unwrap();

    router.check_flush(mcu).unwrap();
    assert_eq!(*count.lock().unwrap(), 0);
}

#[test]
fn flush_multiple_callbacks_all_fire() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    let c1 = Arc::new(Mutex::new(0u32));
    let c2 = Arc::new(Mutex::new(0u32));
    let c1b = Arc::clone(&c1);
    let c2b = Arc::clone(&c2);

    router
        .register_flush_callback(
            mcu,
            Box::new(move || {
                *c1b.lock().unwrap() += 1;
            }),
        )
        .unwrap();
    router
        .register_flush_callback(
            mcu,
            Box::new(move || {
                *c2b.lock().unwrap() += 1;
            }),
        )
        .unwrap();

    router.push(mcu, q, entry(0, 10)).unwrap();
    let _ = router.pop_next_for_emission(mcu).unwrap();
    router.check_flush(mcu).unwrap();

    assert_eq!(*c1.lock().unwrap(), 1);
    assert_eq!(*c2.lock().unwrap(), 1);
}

#[test]
fn stats_increment_on_emit() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    let s0 = router.get_stats(mcu).unwrap();
    assert_eq!(s0.bytes_write, 0);
    assert_eq!(s0.send_seq, 0);

    router.push(mcu, q, entry(0, 10)).unwrap();
    let _ = router.pop_next_for_emission(mcu).unwrap();

    let s1 = router.get_stats(mcu).unwrap();
    assert_eq!(s1.bytes_write, 1);
    assert_eq!(s1.send_seq, 1);
}

#[test]
fn stats_increment_on_response_receive() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    let nid = router.register_notify(mcu, Box::new(|_| {})).unwrap();
    router.push(mcu, q, entry_with_notify(0, 10, nid)).unwrap();
    let _ = router.pop_next_for_emission(mcu).unwrap();

    router
        .dispatch_response(mcu, nid, vec![0xAA, 0xBB])
        .unwrap();

    let s = router.get_stats(mcu).unwrap();
    assert_eq!(s.bytes_read, 2);
    assert_eq!(s.receive_seq, 1);
}

#[test]
fn stats_are_per_mcu() {
    let (mut router, _) = make_router();
    let mcu_a = router.claim_mcu("a");
    let mcu_b = router.claim_mcu("b");
    let qa = router.alloc_command_queue(mcu_a).unwrap();
    let qb = router.alloc_command_queue(mcu_b).unwrap();

    router.push(mcu_a, qa, entry(0, 10)).unwrap();
    let _ = router.pop_next_for_emission(mcu_a).unwrap();

    router.push(mcu_b, qb, entry(0, 20)).unwrap();
    router.push(mcu_b, qb, entry(0, 30)).unwrap();
    let _ = router.pop_next_for_emission(mcu_b).unwrap();
    let _ = router.pop_next_for_emission(mcu_b).unwrap();

    let sa = router.get_stats(mcu_a).unwrap();
    let sb = router.get_stats(mcu_b).unwrap();

    assert_eq!(sa.send_seq, 1);
    assert_eq!(sb.send_seq, 2);
}

#[test]
fn stats_ready_bytes_reflects_live_queue() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    router.push(mcu, q, entry(0, 10)).unwrap();
    router.push(mcu, q, entry(0, 20)).unwrap();

    let s = router.get_stats(mcu).unwrap();
    assert_eq!(s.ready_bytes, 2);
}

#[test]
fn extract_old_captures_sent_and_received() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    let nid = router.register_notify(mcu, Box::new(|_| {})).unwrap();
    router.push(mcu, q, entry_with_notify(0, 10, nid)).unwrap();
    let _ = router.pop_next_for_emission(mcu).unwrap();
    router
        .dispatch_response(mcu, nid, vec![0xDE, 0xAD])
        .unwrap();

    let (sent, received) = router.extract_old(mcu).unwrap();
    assert_eq!(sent.len(), 1);
    assert_eq!(received.len(), 1);
    assert_eq!(received[0].bytes, vec![0xDE, 0xAD]);
}

#[test]
fn extract_old_capped_at_100() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    for i in 0..120 {
        router.push(mcu, q, entry(0, i)).unwrap();
    }
    let mut emitted = 0;
    while router.pop_next_for_emission(mcu).unwrap().is_some() {
        emitted += 1;
        router.record_ack(mcu, 1).unwrap();
    }
    assert!(emitted > 100, "need >100 emits, got {emitted}");

    let (sent, _) = router.extract_old(mcu).unwrap();
    assert_eq!(sent.len(), 100);
}

#[test]
fn flush_does_not_fire_twice_without_new_entries() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    let q = router.alloc_command_queue(mcu).unwrap();

    let count = Arc::new(Mutex::new(0u32));
    let count2 = Arc::clone(&count);
    router
        .register_flush_callback(
            mcu,
            Box::new(move || {
                *count2.lock().unwrap() += 1;
            }),
        )
        .unwrap();

    router.push(mcu, q, entry(0, 10)).unwrap();
    let _ = router.pop_next_for_emission(mcu).unwrap();
    router.check_flush(mcu).unwrap();
    assert_eq!(*count.lock().unwrap(), 1);

    router.check_flush(mcu).unwrap();
    assert_eq!(*count.lock().unwrap(), 1);
}

#[test]
fn wall_time_at_mcu_known_record_returns_wall_time() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let anchor_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 100_000_000.0, anchor_host, 100_000_000)
        .unwrap();

    let (dt, estimated) = router
        .wall_time_at_mcu(mcu, 100_000_000)
        .expect("must return Some when clock record is set");

    let now_unix = time::OffsetDateTime::now_utc();
    let diff = (dt - now_unix).abs();
    assert!(
        diff <= time::Duration::seconds(1),
        "wall time {dt} must be within 1 s of system clock {now_unix}"
    );
    assert!(
        !estimated,
        "estimated must be false when delta is exactly 0 ticks"
    );
}

#[test]
fn wall_time_at_mcu_far_from_anchor_returns_estimated_true() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let anchor_host = crate::clock::instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 100_000_000.0, anchor_host, 100_000_000)
        .unwrap();

    let (_, estimated) = router
        .wall_time_at_mcu(mcu, 300_000_000)
        .expect("must return Some when clock record is set");

    assert!(
        estimated,
        "estimated must be true when tick is 2 MCU-seconds from anchor"
    );
}

/// `clock_to_host_secs` round-trips with `host_time_to_mcu_clock`.
///
/// Seed a 1 MHz clock anchored at last_clock=10_000_000.  Project a host time
/// forward by 1.5 s → mcu_clock = 11_500_000.  The inverse must return the
/// original host time within floating-point precision.
#[test]
fn clock_to_host_secs_round_trips() {
    let (mut router, clock) = make_router();
    let mcu = router.claim_mcu("mcu");

    let base_host = instant_to_f64(clock.now());
    router
        .set_clock_est(mcu, 1_000_000.0, base_host, 10_000_000)
        .unwrap();

    let trip_host = base_host + 1.5;
    let mcu_clock = router.host_time_to_mcu_clock(mcu, trip_host).unwrap();
    assert_eq!(mcu_clock, 11_500_000, "forward projection must yield 11_500_000");

    let recovered = router.clock_to_host_secs(mcu, mcu_clock).unwrap();
    let diff = (recovered - trip_host).abs();
    assert!(
        diff < 1e-9,
        "round-trip error too large: recovered={recovered:.12} expected={trip_host:.12} diff={diff:e}"
    );
}

/// `clock_to_host_secs` returns `None` when no clock record has been set.
#[test]
fn clock_to_host_secs_no_record_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");
    assert!(
        router.clock_to_host_secs(mcu, 1_000_000).is_none(),
        "must return None when clock_freq == 0"
    );
}

/// `clock_to_host_secs` returns `None` for an unknown MCU handle.
#[test]
fn clock_to_host_secs_unknown_mcu_returns_none() {
    let (router, _) = make_router();
    assert!(router.clock_to_host_secs(McuHandle::from_raw(999), 0).is_none());
}

/// `wall_time_at_mcu` returns `None` before any clock record has been set.
#[test]
fn wall_time_at_mcu_no_record_returns_none() {
    let (mut router, _) = make_router();
    let mcu = router.claim_mcu("mcu");

    assert!(
        router.wall_time_at_mcu(mcu, 1_000_000_000).is_none(),
        "must return None when no clock record has been set"
    );
}
