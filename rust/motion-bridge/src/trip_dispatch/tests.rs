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
// Extension liveness tests — through the real `prepare()` path
// ---------------------------------------------------------------------------

use std::sync::Arc;
use kalico_host_rt::host_io::test_harness::ReactorHarness;

use crate::test_support::{
    build_extension_parser, build_trigger_relay_parser, build_trsync_state_frame, extract_payloads,
};

/// Poll `f()` up to `max_attempts` times with `interval` between attempts.
/// Returns `true` if `f()` returned `true` before the limit.
fn poll_until(f: impl Fn() -> bool, interval: std::time::Duration, max_attempts: u32) -> bool {
    for _ in 0..max_attempts {
        if f() {
            return true;
        }
        std::thread::sleep(interval);
    }
    false
}

/// A `trsync_state can_trigger=1` from P0 through the REAL `prepare()` path:
/// exactly one `trsync_set_timeout` must land on P1's wire with a clock value
/// that is in the FUTURE of the stub's `now_ticks`.
///
/// This is the regression test for the zero-initialization bug: the old code
/// initialised participants at `last_status_time=0.0 / expire_time=0.0`, which
/// caused the first report to compute an anchor at process-start → expire in the
/// past → a past-clock `trsync_set_timeout` → REASON_COMMS_TIMEOUT at arm.
#[test]
fn extension_first_report_sends_future_clock_to_other_participant() {
    const MCU_P0: u32 = 10;
    const MCU_P1: u32 = 11;
    const OID_P0: u8 = 20;
    const OID_P1: u8 = 21;
    const FREQ: f64 = 520_000_000.0;
    const EXPIRE_S: f64 = 0.25;
    // now_ticks is large enough to be representative of real uptime.
    const NOW_TICKS: u64 = 52_000_000_000;

    let parser = build_extension_parser();

    let p0_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let p1_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));

    let (p0_io, p0_port) = p0_harness.into_background_io();
    let (p1_io, p1_port) = p1_harness.into_background_io();

    let handle = prepare(
        vec![],
        vec![],
        vec![],
        vec![
            (ParticipantSpec { mcu: MCU_P0, trsync_oid: OID_P0 }, Arc::clone(&p0_io)),
            (ParticipantSpec { mcu: MCU_P1, trsync_oid: OID_P1 }, Arc::clone(&p1_io)),
        ],
        EXPIRE_S,
        move |_| Some((NOW_TICKS, FREQ)),
    )
    .expect("prepare must succeed");

    // Feed P0 a report with a clock value 0.1 s ahead of NOW_TICKS.
    // This pushes status_time = host_now + 0.1 s, which advances the anchor
    // for P1 by 0.1 s > min_extend_s (0.06 s), guaranteeing a send.
    let ahead_ticks = (0.1 * FREQ) as u64;
    let clock32 = (NOW_TICKS + ahead_ticks) as u32;
    let frame = build_trsync_state_frame(OID_P0, 1, clock32, 1);
    p0_port.rx.lock().unwrap().extend(frame);

    let p1_got_send = poll_until(
        || !p1_port.tx.lock().unwrap().is_empty(),
        std::time::Duration::from_millis(5),
        200,
    );
    assert!(p1_got_send, "P1 must receive a trsync_set_timeout within 1 s");

    // P0 must NOT have sent anything to itself.
    assert!(
        p0_port.tx.lock().unwrap().is_empty(),
        "P0 must not extend itself"
    );

    // Decode what P1 received and verify the clock is in the future.
    let p1_tx = p1_port.tx.lock().unwrap().clone();
    let payloads = extract_payloads(p1_tx);
    assert_eq!(payloads.len(), 1, "exactly one trsync_set_timeout for P1");

    let (name, params) = parser
        .decode_body(&payloads[0])
        .expect("P1 payload must decode");
    assert_eq!(name, "trsync_set_timeout");
    assert_eq!(params.get_u32("oid"), u32::from(OID_P1));

    // Regression assertion: the sent clock must be strictly in the future.
    // host_time_to_ticks(expire_t, now_ticks, host_now, freq) =
    //   now_ticks + (expire_t - host_now) * freq
    // With expire_t = host_now_at_prepare + EXPIRE_S ≈ host_now + EXPIRE_S,
    // delta ≈ +EXPIRE_S * FREQ = +130_000_000 ticks > 0.
    let clock32_sent = params.get_u32("clock");
    let expire_ticks = extension::clock32_to_64(NOW_TICKS, clock32_sent);
    assert!(
        expire_ticks > NOW_TICKS,
        "sent clock must be in the future of now_ticks \
         (old code sent a past-clock ≈ process-start, got expire_ticks={expire_ticks} \
         now_ticks={NOW_TICKS})"
    );

    cleanup(handle);
}

/// Immediately after `prepare`, a single report from P0 must not produce any
/// send with a tick value below `now_ticks` on either participant's wire.
/// This is the direct regression guard for the zero-initialization bug.
#[test]
fn no_past_clock_sent_on_first_report_after_prepare() {
    const MCU_P0: u32 = 10;
    const MCU_P1: u32 = 11;
    const OID_P0: u8 = 20;
    const OID_P1: u8 = 21;
    const FREQ: f64 = 520_000_000.0;
    const EXPIRE_S: f64 = 0.25;
    const NOW_TICKS: u64 = 52_000_000_000;

    let parser = build_extension_parser();
    let p0_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let p1_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));

    let (p0_io, p0_port) = p0_harness.into_background_io();
    let (p1_io, p1_port) = p1_harness.into_background_io();

    let handle = prepare(
        vec![],
        vec![],
        vec![],
        vec![
            (ParticipantSpec { mcu: MCU_P0, trsync_oid: OID_P0 }, Arc::clone(&p0_io)),
            (ParticipantSpec { mcu: MCU_P1, trsync_oid: OID_P1 }, Arc::clone(&p1_io)),
        ],
        EXPIRE_S,
        move |_| Some((NOW_TICKS, FREQ)),
    )
    .expect("prepare must succeed");

    let clock32 = NOW_TICKS as u32;
    let frame = build_trsync_state_frame(OID_P0, 1, clock32, 1);
    p0_port.rx.lock().unwrap().extend(frame);

    // Wait up to 1 s for P1 to receive anything or for the deadline to pass.
    let _ = poll_until(
        || !p1_port.tx.lock().unwrap().is_empty(),
        std::time::Duration::from_millis(5),
        200,
    );

    // Scan every trsync_set_timeout sent to either participant and assert
    // no clock value is in the past.
    for (label, port) in [("P0", &p0_port), ("P1", &p1_port)] {
        let tx = port.tx.lock().unwrap().clone();
        for payload in extract_payloads(tx) {
            if let Ok((name, params)) = parser.decode_body(&payload) {
                if name == "trsync_set_timeout" {
                    let clock32_sent = params.get_u32("clock");
                    let expire_ticks = extension::clock32_to_64(NOW_TICKS, clock32_sent);
                    assert!(
                        expire_ticks > NOW_TICKS,
                        "{label}: trsync_set_timeout carried a past clock \
                         (expire_ticks={expire_ticks} <= now_ticks={NOW_TICKS})"
                    );
                }
            }
        }
    }

    cleanup(handle);
}

// ---------------------------------------------------------------------------
// Participant can_trigger=0 relay tests — mainline trdispatch parity
// ---------------------------------------------------------------------------

/// A `trsync_state can_trigger=0` arriving on a PARTICIPANT's io must fan
/// `trsync_trigger` to all sinks and set the handle's triggered flag,
/// mirroring mainline trdispatch's `if (!can_trigger)` broadcast branch.
#[test]
fn participant_timeout_fans_trigger_to_sink() {
    const MCU_PARTICIPANT: u32 = 10;
    const MCU_SINK: u32 = 20;
    const OID_PARTICIPANT: u8 = 5;
    const OID_SINK: u8 = 7;
    const FREQ: f64 = 520_000_000.0;
    const EXPIRE_S: f64 = 0.25;
    const NOW_TICKS: u64 = 52_000_000_000;

    let parser = build_trigger_relay_parser();

    let participant_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let sink_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));

    let (participant_io, participant_port) = participant_harness.into_background_io();
    let (sink_io, sink_port) = sink_harness.into_background_io();

    let handle = prepare(
        vec![],
        vec![SinkSpec { mcu: MCU_SINK, trsync_oid: OID_SINK }],
        vec![(MCU_SINK, Arc::clone(&sink_io))],
        vec![
            (
                ParticipantSpec { mcu: MCU_PARTICIPANT, trsync_oid: OID_PARTICIPANT },
                Arc::clone(&participant_io),
            ),
        ],
        EXPIRE_S,
        move |_| Some((NOW_TICKS, FREQ)),
    )
    .expect("prepare must succeed");

    assert!(!handle.was_triggered(), "triggered must be false before any frame");

    // Feed participant a trsync_state with can_trigger=0 (expire timer fired).
    let frame = build_trsync_state_frame(OID_PARTICIPANT, 0, NOW_TICKS as u32, 1);
    participant_port.rx.lock().unwrap().extend(frame);

    let sink_got_trigger = poll_until(
        || !sink_port.tx.lock().unwrap().is_empty(),
        std::time::Duration::from_millis(5),
        200,
    );
    assert!(sink_got_trigger, "sink must receive trsync_trigger after participant can_trigger=0");

    assert!(handle.was_triggered(), "handle triggered flag must be set after participant timeout");

    let sink_tx = sink_port.tx.lock().unwrap().clone();
    let payloads = extract_payloads(sink_tx);
    assert_eq!(payloads.len(), 1, "exactly one trsync_trigger for sink");

    let (name, params) = parser
        .decode_body(&payloads[0])
        .expect("sink payload must decode");
    assert_eq!(name, "trsync_trigger");
    assert_eq!(params.get_u32("oid"), u32::from(OID_SINK));
    assert_eq!(params.get_u32("reason"), u32::from(REASON_ENDSTOP_HIT));

    cleanup(handle);
}

/// A second `can_trigger=0` from the same participant must NOT produce a
/// second `trsync_trigger` — FanOut is one-shot.
#[test]
fn participant_timeout_fan_is_one_shot() {
    const MCU_PARTICIPANT: u32 = 10;
    const MCU_SINK: u32 = 20;
    const OID_PARTICIPANT: u8 = 5;
    const OID_SINK: u8 = 7;
    const FREQ: f64 = 520_000_000.0;
    const EXPIRE_S: f64 = 0.25;
    const NOW_TICKS: u64 = 52_000_000_000;

    let parser = build_trigger_relay_parser();

    let participant_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let sink_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));

    let (participant_io, participant_port) = participant_harness.into_background_io();
    let (sink_io, sink_port) = sink_harness.into_background_io();

    let handle = prepare(
        vec![],
        vec![SinkSpec { mcu: MCU_SINK, trsync_oid: OID_SINK }],
        vec![(MCU_SINK, Arc::clone(&sink_io))],
        vec![
            (
                ParticipantSpec { mcu: MCU_PARTICIPANT, trsync_oid: OID_PARTICIPANT },
                Arc::clone(&participant_io),
            ),
        ],
        EXPIRE_S,
        move |_| Some((NOW_TICKS, FREQ)),
    )
    .expect("prepare must succeed");

    // First can_trigger=0: should fan out.
    let frame1 = build_trsync_state_frame(OID_PARTICIPANT, 0, NOW_TICKS as u32, 1);
    participant_port.rx.lock().unwrap().extend(frame1);
    let _ = poll_until(
        || !sink_port.tx.lock().unwrap().is_empty(),
        std::time::Duration::from_millis(5),
        200,
    );

    // Drain sink's tx so we can detect any second send.
    let _ = sink_port.tx.lock().unwrap().drain(..).collect::<Vec<_>>();

    // Second can_trigger=0: FanOut already fired — must be a no-op.
    let frame2 = build_trsync_state_frame(OID_PARTICIPANT, 0, NOW_TICKS as u32, 2);
    participant_port.rx.lock().unwrap().extend(frame2);

    // Wait long enough that a spurious send would have arrived.
    std::thread::sleep(std::time::Duration::from_millis(50));
    assert!(
        sink_port.tx.lock().unwrap().is_empty(),
        "second participant can_trigger=0 must not produce a second trsync_trigger"
    );

    cleanup(handle);
}

/// Classic source (SourceSpec::Trsync) + participant registered on the same IO
/// for the same trsync_oid — the Beacon case. Two interceptors coexist on one IO:
///
/// * `can_trigger=0`: source interceptor fans `trsync_trigger` to the sink;
///   participant interceptor also fires the fan-out (no-op — FanOut one-shot)
///   and sets the `triggered` flag. Net result: one trigger to the sink, flag set.
/// * `can_trigger=1`: source ignores it; participant feeds the ExtensionEngine
///   and `trsync_set_timeout` lands on the second participant's wire.
#[test]
fn trsync_source_and_participant_on_same_io_trip_and_extend() {
    const MCU_BEACON: u32 = 5;
    const MCU_SINK: u32 = 20;
    const MCU_EXTEND: u32 = 30;
    const OID_BEACON: u8 = 7;
    const OID_SINK: u8 = 8;
    const OID_EXTEND: u8 = 9;
    const FREQ: f64 = 520_000_000.0;
    const EXPIRE_S: f64 = 0.25;
    const NOW_TICKS: u64 = 52_000_000_000;

    // All three MCUs share the same parser so frames decode on every harness.
    let parser = {
        let dict_json = serde_json::json!({
            "commands": {
                "trsync_set_timeout oid=%c clock=%u": 31,
                "trsync_trigger oid=%c reason=%c": 32
            },
            "responses": {
                "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u": 30
            },
            "output": {},
            "enumerations": {},
            "config": {},
            "version": "test",
            "app": "test"
        });
        let dict: kalico_host_rt::host_io::parser::DataDictionary =
            serde_json::from_value(dict_json).expect("parser dict");
        Arc::new(kalico_host_rt::host_io::parser::MsgProtoParser::from_dictionary(dict)
            .expect("parser build"))
    };

    let beacon_harness = kalico_host_rt::host_io::test_harness::ReactorHarness::new_with_parser(Arc::clone(&parser));
    let sink_harness   = kalico_host_rt::host_io::test_harness::ReactorHarness::new_with_parser(Arc::clone(&parser));
    let extend_harness = kalico_host_rt::host_io::test_harness::ReactorHarness::new_with_parser(Arc::clone(&parser));

    let (beacon_io, beacon_port) = beacon_harness.into_background_io();
    let (sink_io,   sink_port)   = sink_harness.into_background_io();
    let (extend_io, extend_port) = extend_harness.into_background_io();

    let handle = prepare(
        // Beacon as a Trsync source.
        vec![(SourceSpec::Trsync { mcu: MCU_BEACON, trsync_oid: OID_BEACON }, Arc::clone(&beacon_io))],
        // Sink: the stepper trsync that receives trsync_trigger.
        vec![SinkSpec { mcu: MCU_SINK, trsync_oid: OID_SINK }],
        vec![(MCU_SINK, Arc::clone(&sink_io))],
        // Participants: Beacon (for liveness) + a second stepper participant.
        vec![
            (ParticipantSpec { mcu: MCU_BEACON, trsync_oid: OID_BEACON }, Arc::clone(&beacon_io)),
            (ParticipantSpec { mcu: MCU_EXTEND, trsync_oid: OID_EXTEND }, Arc::clone(&extend_io)),
        ],
        EXPIRE_S,
        move |_| Some((NOW_TICKS, FREQ)),
    )
    .expect("prepare must succeed");

    assert!(!handle.was_triggered(), "not triggered before any frame");

    // --- can_trigger=0: Beacon trips.
    // Expect exactly one trsync_trigger to the sink; triggered flag set.
    let frame_trip = build_trsync_state_frame(OID_BEACON, 0, NOW_TICKS as u32, 1);
    beacon_port.rx.lock().unwrap().extend(frame_trip);

    let sink_got_trigger = poll_until(
        || !sink_port.tx.lock().unwrap().is_empty(),
        std::time::Duration::from_millis(5),
        200,
    );
    assert!(sink_got_trigger, "sink must receive trsync_trigger after Beacon can_trigger=0");
    assert!(handle.was_triggered(), "triggered flag must be set after Beacon can_trigger=0");

    let sink_tx = sink_port.tx.lock().unwrap().clone();
    let payloads = extract_payloads(sink_tx);
    assert_eq!(payloads.len(), 1, "exactly one trsync_trigger for sink (FanOut one-shot)");
    let (name, params) = parser.decode_body(&payloads[0]).expect("decode sink payload");
    assert_eq!(name, "trsync_trigger");
    assert_eq!(params.get_u32("oid"), u32::from(OID_SINK));
    assert_eq!(params.get_u32("reason"), u32::from(REASON_ENDSTOP_HIT));

    // No extension sends must have happened yet (can_trigger=0 returns early
    // from the participant interceptor before feeding ExtensionEngine).
    assert!(
        extend_port.tx.lock().unwrap().is_empty(),
        "extend participant must not receive trsync_set_timeout on can_trigger=0"
    );

    cleanup(handle);

    // --- can_trigger=1: Beacon reports liveness.
    // A fresh handle to avoid the already-fired FanOut.
    let handle2 = prepare(
        vec![(SourceSpec::Trsync { mcu: MCU_BEACON, trsync_oid: OID_BEACON }, Arc::clone(&beacon_io))],
        vec![SinkSpec { mcu: MCU_SINK, trsync_oid: OID_SINK }],
        vec![(MCU_SINK, Arc::clone(&sink_io))],
        vec![
            (ParticipantSpec { mcu: MCU_BEACON, trsync_oid: OID_BEACON }, Arc::clone(&beacon_io)),
            (ParticipantSpec { mcu: MCU_EXTEND, trsync_oid: OID_EXTEND }, Arc::clone(&extend_io)),
        ],
        EXPIRE_S,
        move |_| Some((NOW_TICKS, FREQ)),
    )
    .expect("prepare handle2 must succeed");

    // Drain stale tx from the first run.
    sink_port.tx.lock().unwrap().clear();
    extend_port.tx.lock().unwrap().clear();

    // Feed a can_trigger=1 frame 0.1s ahead of NOW_TICKS to guarantee extension.
    let ahead_ticks = (0.1 * FREQ) as u64;
    let clock32 = (NOW_TICKS + ahead_ticks) as u32;
    let frame_alive = build_trsync_state_frame(OID_BEACON, 1, clock32, 2);
    beacon_port.rx.lock().unwrap().extend(frame_alive);

    let extend_got_timeout = poll_until(
        || !extend_port.tx.lock().unwrap().is_empty(),
        std::time::Duration::from_millis(5),
        200,
    );
    assert!(
        extend_got_timeout,
        "extend participant must receive trsync_set_timeout after Beacon can_trigger=1"
    );
    assert!(
        sink_port.tx.lock().unwrap().is_empty(),
        "sink must not receive trsync_trigger for can_trigger=1"
    );
    assert!(!handle2.was_triggered(), "no trip for can_trigger=1");

    cleanup(handle2);
}

/// A `can_trigger=0` on a participant's io must NOT cause the ExtensionEngine
/// to send a `trsync_set_timeout` — the participant path returns early before
/// feeding the engine.
#[test]
fn participant_timeout_does_not_feed_extension_engine() {
    const MCU_P0: u32 = 10;
    const MCU_P1: u32 = 11;
    const OID_P0: u8 = 5;
    const OID_P1: u8 = 6;
    const FREQ: f64 = 520_000_000.0;
    const EXPIRE_S: f64 = 0.25;
    const NOW_TICKS: u64 = 52_000_000_000;

    let parser = build_extension_parser();

    let p0_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let p1_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));

    let (p0_io, p0_port) = p0_harness.into_background_io();
    let (p1_io, p1_port) = p1_harness.into_background_io();

    let handle = prepare(
        vec![],
        vec![],
        vec![],
        vec![
            (ParticipantSpec { mcu: MCU_P0, trsync_oid: OID_P0 }, Arc::clone(&p0_io)),
            (ParticipantSpec { mcu: MCU_P1, trsync_oid: OID_P1 }, Arc::clone(&p1_io)),
        ],
        EXPIRE_S,
        move |_| Some((NOW_TICKS, FREQ)),
    )
    .expect("prepare must succeed");

    // Feed P0 a trsync_state with can_trigger=0.
    let frame = build_trsync_state_frame(OID_P0, 0, NOW_TICKS as u32, 1);
    p0_port.rx.lock().unwrap().extend(frame);

    // Wait long enough that an extension send would have arrived.
    std::thread::sleep(std::time::Duration::from_millis(50));

    // Neither participant must receive a trsync_set_timeout — the engine was
    // not fed.
    assert!(
        p0_port.tx.lock().unwrap().is_empty(),
        "P0 must not receive trsync_set_timeout after participant can_trigger=0"
    );
    assert!(
        p1_port.tx.lock().unwrap().is_empty(),
        "P1 must not receive trsync_set_timeout after participant can_trigger=0"
    );

    cleanup(handle);
}
