use super::*;
use std::cell::RefCell;
use std::sync::atomic::Ordering;

use kalico_host_rt::transport::{MessageParams, MessageValue};

/// Build a `MessageParams` with two u32 fields.
fn params_u32_2(k1: &str, v1: u32, k2: &str, v2: u32) -> MessageParams {
    let mut p = MessageParams::new();
    p.insert(k1, MessageValue::U32(v1));
    p.insert(k2, MessageValue::U32(v2));
    p
}

/// Build a `MessageParams` with a single u32 field — enough to exercise
/// the closure filters without any real transport.
fn params_u32(key: &str, val: u32) -> MessageParams {
    let mut p = MessageParams::new();
    p.insert(key, MessageValue::U32(val));
    p
}

// ---------------------------------------------------------------------------
// Closure-filter tests
// ---------------------------------------------------------------------------

/// `BridgeGpio` closure: a frame whose `arm_id` does NOT match the
/// registered arm must be silently ignored; a matching frame must fan out.
#[test]
fn bridge_gpio_closure_filters_by_arm_id() {
    let triggered = Arc::new(AtomicBool::new(false));
    let fan = Arc::new(FanOut::new(vec![SinkSpec { mcu: 1, trsync_oid: 10 }]));
    let sent = Arc::new(std::sync::Mutex::new(Vec::<(u32, String)>::new()));

    let want_arm_id: Option<u32> = Some(42_u32);
    let fan_clone = Arc::clone(&fan);
    let triggered_clone = Arc::clone(&triggered);
    let sent_clone = Arc::clone(&sent);
    // Simulate what `prepare` builds for a BridgeGpio source.
    let closure = move |params: &MessageParams| {
        if let Some(want) = want_arm_id {
            if params.get_u32("arm_id") != want {
                return;
            }
        }
        fan_clone.on_trip(|mcu, cmd| {
            sent_clone.lock().unwrap().push((mcu, cmd.to_string()));
        });
        triggered_clone.store(true, Ordering::Release);
    };

    // Wrong arm_id — must be ignored.
    closure(&params_u32("arm_id", 99));
    assert!(!triggered.load(Ordering::Acquire), "wrong arm_id must not trigger");
    assert!(sent.lock().unwrap().is_empty(), "no send on arm_id mismatch");

    // Correct arm_id — must fan out.
    closure(&params_u32("arm_id", 42));
    assert!(triggered.load(Ordering::Acquire), "matching arm_id must trigger");
    assert_eq!(sent.lock().unwrap().len(), 1, "one send per sink on first trip");
}

/// `Trsync` closure: `can_trigger != 0` (still armed) must be ignored;
/// `can_trigger == 0` (probe hit / soft-trip) must fan out.
#[test]
fn trsync_closure_ignores_nonzero_can_trigger() {
    let triggered = Arc::new(AtomicBool::new(false));
    let fan = Arc::new(FanOut::new(vec![SinkSpec { mcu: 2, trsync_oid: 11 }]));
    let sent = Arc::new(std::sync::Mutex::new(Vec::<(u32, String)>::new()));

    let want_arm_id: Option<u32> = None; // Trsync path
    let fan_clone = Arc::clone(&fan);
    let triggered_clone = Arc::clone(&triggered);
    let sent_clone = Arc::clone(&sent);
    let closure = move |params: &MessageParams| {
        if let Some(want) = want_arm_id {
            if params.get_u32("arm_id") != want {
                return;
            }
        } else {
            if params.get_u32("can_trigger") != 0 {
                return;
            }
        }
        fan_clone.on_trip(|mcu, cmd| {
            sent_clone.lock().unwrap().push((mcu, cmd.to_string()));
        });
        triggered_clone.store(true, Ordering::Release);
    };

    // Still armed (can_trigger = 1) — must be ignored.
    closure(&params_u32("can_trigger", 1));
    assert!(!triggered.load(Ordering::Acquire), "can_trigger=1 must not trigger");
    assert!(sent.lock().unwrap().is_empty(), "no send while still armed");

    // Probe hit (can_trigger = 0) — must fan out.
    closure(&params_u32("can_trigger", 0));
    assert!(triggered.load(Ordering::Acquire), "can_trigger=0 must trigger");
    assert_eq!(sent.lock().unwrap().len(), 1, "one send per sink on first trip");
}

// ---------------------------------------------------------------------------

#[test]
fn first_trip_fans_trigger_to_all_sinks_once() {
    let sinks = vec![
        SinkSpec { mcu: 1, trsync_oid: 10 },
        SinkSpec { mcu: 2, trsync_oid: 11 },
        SinkSpec { mcu: 3, trsync_oid: 12 },
    ];
    let sent = RefCell::new(Vec::<(u32, String)>::new());
    let dispatch = FanOut::new(sinks);

    dispatch.on_trip(|mcu, cmd| sent.borrow_mut().push((mcu, cmd.to_string())));
    // second trip is a no-op (one-shot)
    dispatch.on_trip(|mcu, cmd| sent.borrow_mut().push((mcu, cmd.to_string())));

    let sent = sent.into_inner();
    assert_eq!(sent.len(), 3, "exactly one trigger per sink, one-shot");
    assert_eq!(sent[0], (1, "trsync_trigger oid=10 reason=1".to_string()));
    assert_eq!(sent[1], (2, "trsync_trigger oid=11 reason=1".to_string()));
    assert_eq!(sent[2], (3, "trsync_trigger oid=12 reason=1".to_string()));
}

#[test]
fn build_trigger_cmd_formats_reason_endstop_hit() {
    assert_eq!(build_trigger_cmd(42), "trsync_trigger oid=42 reason=1");
}

// ---------------------------------------------------------------------------
// Extension liveness tests (closure-level, no reactor)
// ---------------------------------------------------------------------------

/// A `trsync_state can_trigger=1` report from participant 0 must produce
/// exactly one `trsync_set_timeout` send to participant 1's io and none to
/// participant 0's io. Mirrors what `prepare` registers, exercised directly.
#[test]
fn extension_report_sends_timeout_to_other_participant_only() {
    use std::sync::Mutex;
    use kalico_host_rt::clock::instant_to_f64;
    use std::time::Instant;

    const MCU_P0: u32 = 10;
    const MCU_P1: u32 = 11;
    const OID_P0: u8 = 20;
    const OID_P1: u8 = 21;
    const FREQ: f64 = 520_000_000.0;
    const EXPIRE_S: f64 = 0.25;

    let sent_p0 = Arc::new(Mutex::new(Vec::<String>::new()));
    let sent_p1 = Arc::new(Mutex::new(Vec::<String>::new()));

    // Capture all `trsync_set_timeout` commands sent to each participant's io.
    let sent_p0_cap = Arc::clone(&sent_p0);
    let sent_p1_cap = Arc::clone(&sent_p1);

    let min_extend_s = 0.8 * 0.3 * EXPIRE_S;
    // Pre-populate expire_time so the initial report only advances participant 1.
    // With expire_time=EXPIRE_S, participant 0's anchor is P1's silence (0.0),
    // giving expire=EXPIRE_S which equals the existing expire_time — delta=0
    // fails the `expire > p.expire_time` guard and P0 is not re-sent.
    let engine = Arc::new(Mutex::new(extension::ExtensionEngine::new(
        vec![
            extension::Participant { last_status_time: 0.0, expire_time: EXPIRE_S },
            extension::Participant { last_status_time: 0.0, expire_time: 0.0 },
        ],
        EXPIRE_S,
        min_extend_s,
    )));

    // Fake send tables for both participants: (mcu, oid, Arc<Mutex<Vec<String>>>)
    let participant_ios: Vec<(u32, u8, Arc<Mutex<Vec<String>>>)> = vec![
        (MCU_P0, OID_P0, Arc::clone(&sent_p0)),
        (MCU_P1, OID_P1, Arc::clone(&sent_p1)),
    ];

    // Build the closure that `prepare` would register for participant 0.
    let engine_c = Arc::clone(&engine);
    let participant_ios_c = participant_ios.clone();
    let idx: usize = 0;
    let mcu = MCU_P0;

    // Simulate a fixed known now_ticks so the conversion is deterministic.
    let now_ticks: u64 = 52_000_000_000;
    let host_now = instant_to_f64(Instant::now());

    let clock_of = {
        let now_ticks = now_ticks;
        move |_: u32| -> Option<(u64, f64)> { Some((now_ticks, FREQ)) }
    };

    // The closure mirrors what `prepare` registers for each participant.
    let closure = {
        let engine = Arc::clone(&engine_c);
        let participant_ios = participant_ios_c.clone();
        let clock_of = clock_of.clone();
        move |params: &MessageParams| {
            if params.get_u32("can_trigger") == 0 {
                return;
            }
            let clock32 = params.get_u32("clock");
            let (now_t, freq) = match clock_of(mcu) {
                Some(v) => v,
                None => return,
            };
            let report_ticks = extension::clock32_to_64(now_t, clock32);
            let status_time =
                extension::ticks_to_host_time(report_ticks, now_t, host_now, freq);

            let sends = engine
                .lock()
                .unwrap_or_else(|e| e.into_inner())
                .on_report(idx, status_time);

            for (target_idx, expire_t) in sends {
                let (target_mcu, target_oid, ref cap) = participant_ios[target_idx];
                let (target_now_t, target_freq) = match clock_of(target_mcu) {
                    Some(v) => v,
                    None => continue,
                };
                let expire_ticks = extension::host_time_to_ticks(
                    expire_t,
                    target_now_t,
                    host_now,
                    target_freq,
                );
                let cmd = format!(
                    "trsync_set_timeout oid={} clock={}",
                    target_oid,
                    expire_ticks & 0xFFFF_FFFF
                );
                cap.lock().unwrap().push(cmd);
            }
        }
    };

    // Feed a report with can_trigger=1 and clock=now_ticks (low 32 bits) to
    // produce a large enough status_time to advance past min_extend_s.
    let clock32 = now_ticks as u32;
    closure(&params_u32_2("can_trigger", 1, "clock", clock32));

    let p0_sends = sent_p0_cap.lock().unwrap().clone();
    let p1_sends = sent_p1_cap.lock().unwrap().clone();

    assert!(p0_sends.is_empty(), "participant 0 must not extend itself; got: {p0_sends:?}");
    assert_eq!(p1_sends.len(), 1, "exactly one trsync_set_timeout for participant 1");
    assert!(
        p1_sends[0].starts_with("trsync_set_timeout oid=21 clock="),
        "send must target oid=21, got: {}",
        p1_sends[0]
    );
}
