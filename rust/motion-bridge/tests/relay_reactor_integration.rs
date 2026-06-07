//! Integration test: `TripDispatch` relay interceptor fires on a live reactor
//! and produces an outbound `trsync_trigger` frame on the sink MCU's wire.
//!
//! # What this test proves
//!
//! The unit tests in `trip_dispatch/tests.rs` call relay closures directly and
//! never touch the reactor.  This test closes the seam using the PRODUCTION
//! frame classification: `kalico_endstop_tripped` is registered in the `output`
//! section of the data dictionary (matching the MCU firmware's `output()` call),
//! which routes through `DecodedFrame::Output` in the reactor.  The bug this
//! test exercises was that interceptors were only dispatched for `Response`
//! frames, silently skipping all `Output` frames including
//! `kalico_endstop_tripped`.
//!
//! ```text
//! inbound wire bytes
//!   → SerialFrameIo → KlipperFrame
//!   → Reactor::handle_inbound_frame
//!   → parser.decode  (real MsgProtoParser, not empty)
//!   → DecodedFrame::Output branch  ← production path for kalico_endstop_tripped
//!   → InterceptorTable::dispatch   ← interceptors now run for Output frames
//!   → relay closure  (trip_dispatch::FanOut + arm_id filter, real code)
//!   → submission_tx FireAndForget → sink reactor tick
//!   → dispatch_fire_and_forget → sink wire bytes
//! ```
//!
//! Two `ReactorHarness` instances (source MCU, sink MCU) are driven
//! synchronously — no background threads.
//!
//! # What this test does NOT prove
//!
//! The firmware half of the trip: `trsync_do_trigger → runtime_stop_on_trigger
//! → software_trip`.  That seam requires Renode or real hardware (Task 9).

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::Sender;

use kalico_host_rt::host_io::ReactorCommand;
use kalico_host_rt::host_io::parser::{DataDictionary, MsgProtoParser};
use kalico_host_rt::host_io::test_harness::ReactorHarness;
use kalico_host_rt::host_io::wire;
use motion_bridge_native::test_support::{
    build_extension_parser, build_trsync_state_frame, extract_payloads,
};
use motion_bridge_native::trip_dispatch::{
    FanOut, ParticipantSpec, REASON_ENDSTOP_HIT, SinkSpec, SourceSpec,
    cleanup as trip_dispatch_cleanup, prepare as trip_dispatch_prepare,
};

// ---------------------------------------------------------------------------
// Parser helpers
// ---------------------------------------------------------------------------

/// Build a minimal `MsgProtoParser` that understands:
///
/// - `kalico_endstop_tripped arm_id=%u trip_clock_lo=%u trip_clock_hi=%u
///   trip_source_idx=%u fmt_version=%u stepper_count=%u stepper_data=%*s`
///   (**output** msgid=10 — matches production MCU firmware which uses
///   `output("kalico_endstop_tripped ...")`, not a solicited response)
/// - `trsync_trigger oid=%c reason=%c`
///   (command msgid=20, sent to sink MCU)
///
/// Placing `kalico_endstop_tripped` in the `output` section is intentional:
/// the pre-fix code only dispatched interceptors for `Response` frames, so
/// tests using `responses` gave false confidence.  The `output` section is the
/// production path this test must exercise.
fn build_test_parser() -> Arc<MsgProtoParser> {
    let dict_json = serde_json::json!({
        "commands": {
            "trsync_trigger oid=%c reason=%c": 20
        },
        "responses": {},
        "output": {
            "kalico_endstop_tripped arm_id=%u trip_clock_lo=%u trip_clock_hi=%u trip_source_idx=%u fmt_version=%u stepper_count=%u stepper_data=%*s": 10
        },
        "enumerations": {},
        "config": {},
        "version": "test",
        "app": "test"
    });
    let dict: DataDictionary = serde_json::from_value(dict_json).expect("bad test dict");
    Arc::new(MsgProtoParser::from_dictionary(dict).expect("parser build failed"))
}

/// Encode a `kalico_endstop_tripped` frame (msgid=10) with the given `arm_id`
/// and zero-length stepper blob.
fn build_endstop_tripped_frame(arm_id: u32, seq: u8) -> Vec<u8> {
    use kalico_host_rt::host_io::parser::encode_vlq;

    let mut payload = Vec::new();
    encode_vlq(&mut payload, 10).unwrap();
    encode_vlq(&mut payload, arm_id as i64).unwrap();
    encode_vlq(&mut payload, 0).unwrap();
    encode_vlq(&mut payload, 0).unwrap();
    encode_vlq(&mut payload, 0).unwrap();
    encode_vlq(&mut payload, 1).unwrap();
    encode_vlq(&mut payload, 0).unwrap();
    payload.push(0u8);

    wire::build_frame(&payload, seq)
}

// ---------------------------------------------------------------------------
// Main positive test
// ---------------------------------------------------------------------------

/// An inbound `kalico_endstop_tripped` frame for the registered arm_id fires
/// the relay interceptor and produces exactly one outbound `trsync_trigger`
/// frame (oid = sink oid, reason = 1) on the sink MCU's wire.
#[test]
fn endstop_tripped_frame_relays_trsync_trigger_through_live_reactor() {
    const SRC_ARM_ID: u32 = 42;
    const SINK_OID: u8 = 7;

    let parser = build_test_parser();

    let mut src = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let mut sink = ReactorHarness::new_with_parser(Arc::clone(&parser));

    let sink_submission_tx: Sender<ReactorCommand> = sink.submission_tx.clone();

    let sinks = vec![SinkSpec { mcu: 0, trsync_oid: SINK_OID }];
    let fan = Arc::new(FanOut::new(sinks));
    let triggered = Arc::new(AtomicBool::new(false));
    let fan2 = Arc::clone(&fan);
    let triggered2 = Arc::clone(&triggered);
    let want_arm_id: Option<u32> = Some(SRC_ARM_ID);

    let _interceptor_id = src.register_interceptor(
        "kalico_endstop_tripped",
        None,
        Box::new(move |params| {
            if let Some(want) = want_arm_id {
                if params.get_u32("arm_id") != want {
                    return;
                }
            }
            fan2.on_trip(|_mcu, cmd| {
                let _ = sink_submission_tx.send(ReactorCommand::FireAndForget {
                    cmd: cmd.to_owned(),
                });
            });
            triggered2.store(true, Ordering::Release);
        }),
    );

    let frame = build_endstop_tripped_frame(SRC_ARM_ID, 1);
    src.feed_rx(&frame);
    src.tick();

    assert!(triggered.load(Ordering::Acquire), "relay closure must have set triggered");

    sink.tick();

    let sink_tx = sink.tx_log();
    assert!(!sink_tx.is_empty(), "sink must have written bytes after relay");

    let payloads = extract_payloads(sink_tx);
    assert_eq!(payloads.len(), 1, "exactly one outbound frame on the sink");

    let payload = &payloads[0];
    let decoded_result = parser.decode_body(payload);
    assert!(decoded_result.is_ok(), "payload must decode: {:?}", decoded_result);
    let (name, params) = decoded_result.unwrap();

    assert_eq!(name, "trsync_trigger", "outbound frame must be trsync_trigger");
    assert_eq!(
        params.get_u32("oid"),
        u32::from(SINK_OID),
        "oid must match sink oid"
    );
    assert_eq!(
        params.get_u32("reason"),
        u32::from(REASON_ENDSTOP_HIT),
        "reason must be REASON_ENDSTOP_HIT (1)"
    );
}

// ---------------------------------------------------------------------------
// Negative test: wrong arm_id is filtered — no relay
// ---------------------------------------------------------------------------

/// An inbound `kalico_endstop_tripped` for a DIFFERENT arm_id must not
/// produce any outbound `trsync_trigger`.
#[test]
fn endstop_tripped_wrong_arm_id_produces_no_relay() {
    const SRC_ARM_ID: u32 = 42;
    const WRONG_ARM_ID: u32 = 99;
    const SINK_OID: u8 = 7;

    let parser = build_test_parser();
    let mut src = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let mut sink = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let sink_submission_tx: Sender<ReactorCommand> = sink.submission_tx.clone();

    let sinks = vec![SinkSpec { mcu: 0, trsync_oid: SINK_OID }];
    let fan = Arc::new(FanOut::new(sinks));
    let triggered = Arc::new(AtomicBool::new(false));
    let fan2 = Arc::clone(&fan);
    let triggered2 = Arc::clone(&triggered);
    let want_arm_id: Option<u32> = Some(SRC_ARM_ID);

    let _interceptor_id = src.register_interceptor(
        "kalico_endstop_tripped",
        None,
        Box::new(move |params| {
            if let Some(want) = want_arm_id {
                if params.get_u32("arm_id") != want {
                    return;
                }
            }
            fan2.on_trip(|_mcu, cmd| {
                let _ = sink_submission_tx.send(ReactorCommand::FireAndForget {
                    cmd: cmd.to_owned(),
                });
            });
            triggered2.store(true, Ordering::Release);
        }),
    );

    let frame = build_endstop_tripped_frame(WRONG_ARM_ID, 1);
    src.feed_rx(&frame);
    src.tick();
    sink.tick();

    assert!(
        !triggered.load(Ordering::Acquire),
        "wrong arm_id must not set triggered"
    );
    assert!(
        sink.tx_log().is_empty(),
        "wrong arm_id must produce no outbound bytes on sink"
    );
}

// ---------------------------------------------------------------------------
// One-shot test: second trip must NOT produce a second trsync_trigger
// ---------------------------------------------------------------------------

/// `FanOut` is one-shot: the second inbound `kalico_endstop_tripped` for the
/// same arm_id must not produce a second outbound `trsync_trigger`.
#[test]
fn second_endstop_tripped_does_not_relay_again() {
    const SRC_ARM_ID: u32 = 42;
    const SINK_OID: u8 = 7;

    let parser = build_test_parser();
    let mut src = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let mut sink = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let sink_submission_tx: Sender<ReactorCommand> = sink.submission_tx.clone();

    let sinks = vec![SinkSpec { mcu: 0, trsync_oid: SINK_OID }];
    let fan = Arc::new(FanOut::new(sinks));
    let fan2 = Arc::clone(&fan);
    let want_arm_id: Option<u32> = Some(SRC_ARM_ID);

    let _interceptor_id = src.register_interceptor(
        "kalico_endstop_tripped",
        None,
        Box::new(move |params| {
            if let Some(want) = want_arm_id {
                if params.get_u32("arm_id") != want {
                    return;
                }
            }
            fan2.on_trip(|_mcu, cmd| {
                let _ = sink_submission_tx.send(ReactorCommand::FireAndForget {
                    cmd: cmd.to_owned(),
                });
            });
        }),
    );

    let frame1 = build_endstop_tripped_frame(SRC_ARM_ID, 1);
    src.feed_rx(&frame1);
    src.tick();
    sink.tick();

    let payloads_after_first = extract_payloads(sink.tx_log());
    assert_eq!(
        payloads_after_first.len(),
        1,
        "first trip must produce exactly one trsync_trigger"
    );

    let frame2 = build_endstop_tripped_frame(SRC_ARM_ID, 2);
    src.feed_rx(&frame2);
    src.tick();
    sink.tick();

    let payloads_after_second = extract_payloads(sink.tx_log());
    assert_eq!(
        payloads_after_second.len(),
        1,
        "second trip must NOT produce another trsync_trigger (FanOut one-shot)"
    );
}

// ---------------------------------------------------------------------------
// Extension liveness: trsync_state can_trigger=1 → trsync_set_timeout
// ---------------------------------------------------------------------------

/// An inbound `trsync_state can_trigger=1` for participant 0, delivered
/// through the REAL `prepare()` path (via `into_background_io`), must produce
/// exactly one outbound `trsync_set_timeout` on participant 1's wire with a
/// clock value in the future of `now_ticks`, and nothing on participant 0's wire.
///
/// This test uses the same `into_background_io` + `poll_until` pattern as the
/// unit tests in `trip_dispatch/tests.rs`, making the seam coverage identical
/// without duplicating or shadowing any `prepare` internals.
#[test]
fn trsync_state_report_extends_other_participant_timeout_through_live_reactor() {
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

    let handle = motion_bridge_native::trip_dispatch::prepare(
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

    assert!(
        p0_port.tx.lock().unwrap().is_empty(),
        "P0 must not extend itself"
    );

    let p1_tx = p1_port.tx.lock().unwrap().clone();
    let payloads = extract_payloads(p1_tx);
    assert_eq!(payloads.len(), 1, "exactly one trsync_set_timeout for P1");

    let (name, params) = parser
        .decode_body(&payloads[0])
        .expect("P1 payload must decode");
    assert_eq!(name, "trsync_set_timeout");
    assert_eq!(params.get_u32("oid"), u32::from(OID_P1));

    let clock32_sent = params.get_u32("clock");
    let expire_ticks =
        motion_bridge_native::trip_dispatch::extension::clock32_to_64(NOW_TICKS, clock32_sent);
    assert!(
        expire_ticks > NOW_TICKS,
        "sent clock must be in the future of now_ticks \
         (expire_ticks={expire_ticks} now_ticks={NOW_TICKS})"
    );

    motion_bridge_native::trip_dispatch::cleanup(handle);
}

// ---------------------------------------------------------------------------
// Self-relay: source MCU == sink MCU (sensorless homing on bridge MCU)
// ---------------------------------------------------------------------------

/// When source and sink are the SAME `KalicoHostIo` (as in sensorless homing
/// where the H7 is both the endstop MCU and the trsync sink), the relay
/// interceptor fires from the H7's reactor thread and sends `trsync_trigger`
/// back via the same MCU's `submission_tx`.  The reactor processes it on the
/// next tick and the bytes appear on the same mock wire.
///
/// This test would have caught the "relay fires but bridge MCU never receives
/// command" class of bug: if the relay used the wrong send path (e.g. a stale
/// `KalicoHostIo`, or a path that doesn't reach the reactor's `write_frame`),
/// the trigger would not land on the wire and the assertion would fail.
#[test]
fn endstop_tripped_relays_trsync_trigger_when_source_and_sink_are_same_mcu() {
    const MCU_BRIDGE: u32 = 1;
    const SRC_ARM_ID: u32 = 7;
    const SINK_OID: u8 = 8;

    let parser = build_test_parser();

    let bridge_harness = ReactorHarness::new_with_parser(Arc::clone(&parser));
    let (bridge_io, bridge_port) = bridge_harness.into_background_io();

    let handle = trip_dispatch_prepare(
        vec![(
            SourceSpec::BridgeGpio { mcu: MCU_BRIDGE, arm_id: SRC_ARM_ID },
            Arc::clone(&bridge_io),
        )],
        vec![SinkSpec { mcu: MCU_BRIDGE, trsync_oid: SINK_OID }],
        vec![(MCU_BRIDGE, Arc::clone(&bridge_io))],
        vec![],
        0.0,
        |_| None,
    )
    .expect("prepare must succeed for self-relay");

    assert!(!handle.was_triggered(), "not triggered before any frame");

    let frame = build_endstop_tripped_frame(SRC_ARM_ID, 1);
    bridge_port.rx.lock().unwrap().extend(frame);

    let got_trigger = poll_until(
        || !bridge_port.tx.lock().unwrap().is_empty(),
        std::time::Duration::from_millis(5),
        200,
    );
    assert!(
        got_trigger,
        "bridge MCU must receive trsync_trigger on its own wire after \
         endstop trip on same MCU"
    );

    assert!(handle.was_triggered(), "handle triggered flag must be set");

    let tx_bytes = bridge_port.tx.lock().unwrap().clone();
    let payloads = extract_payloads(tx_bytes);
    assert_eq!(payloads.len(), 1, "exactly one trsync_trigger for self-relay sink");

    let (name, params) = parser
        .decode_body(&payloads[0])
        .expect("self-relay payload must decode");
    assert_eq!(name, "trsync_trigger", "frame must be trsync_trigger");
    assert_eq!(
        params.get_u32("oid"),
        u32::from(SINK_OID),
        "oid must match sink oid"
    );
    assert_eq!(
        params.get_u32("reason"),
        u32::from(REASON_ENDSTOP_HIT),
        "reason must be REASON_ENDSTOP_HIT"
    );

    trip_dispatch_cleanup(handle);
}

fn poll_until(f: impl Fn() -> bool, interval: std::time::Duration, max_attempts: u32) -> bool {
    for _ in 0..max_attempts {
        if f() {
            return true;
        }
        std::thread::sleep(interval);
    }
    false
}
