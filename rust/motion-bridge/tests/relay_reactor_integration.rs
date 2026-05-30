//! Integration test: `TripDispatch` relay interceptor fires on a live reactor
//! and produces an outbound `trsync_trigger` frame on the sink MCU's wire.
//!
//! # What this test proves
//!
//! The unit tests in `trip_dispatch/tests.rs` call relay closures directly and
//! never touch the reactor.  This test closes the seam:
//!
//! ```text
//! inbound wire bytes
//!   → SerialFrameIo → KlipperFrame
//!   → Reactor::handle_inbound_frame
//!   → parser.decode  (real MsgProtoParser, not empty)
//!   → InterceptorTable::dispatch
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
use motion_bridge_native::trip_dispatch::{FanOut, REASON_ENDSTOP_HIT, SinkSpec};

// ---------------------------------------------------------------------------
// Parser helpers
// ---------------------------------------------------------------------------

/// Build a minimal `MsgProtoParser` that understands:
///
/// - `kalico_endstop_tripped arm_id=%u trip_clock_lo=%u trip_clock_hi=%u
///   trip_source_idx=%u fmt_version=%u stepper_count=%u stepper_data=%*s`
///   (response msgid=10, unsolicited from source MCU)
/// - `trsync_trigger oid=%c reason=%c`
///   (command msgid=20, sent to sink MCU)
///
/// Msgids are arbitrary — they just need to be consistent within each test's
/// harness pair.
fn build_test_parser() -> Arc<MsgProtoParser> {
    let dict_json = serde_json::json!({
        "commands": {
            "trsync_trigger oid=%c reason=%c": 20
        },
        "responses": {
            "kalico_endstop_tripped arm_id=%u trip_clock_lo=%u trip_clock_hi=%u trip_source_idx=%u fmt_version=%u stepper_count=%u stepper_data=%*s": 10
        },
        "output": {},
        "enumerations": {},
        "config": {},
        "version": "test",
        "app": "test"
    });
    let dict: DataDictionary = serde_json::from_value(dict_json).expect("bad test dict");
    Arc::new(MsgProtoParser::from_dictionary(dict).expect("parser build failed"))
}

/// Encode a `kalico_endstop_tripped` frame (msgid=10) with the given `arm_id`
/// and zero-length stepper blob.  The resulting bytes are ready to be fed to
/// `ReactorHarness::feed_rx`.
fn build_endstop_tripped_frame(arm_id: u32, seq: u8) -> Vec<u8> {
    // Build the msgproto payload by encoding the command string.
    // `kalico_endstop_tripped` is registered as a *response* (MCU→host,
    // unsolicited), so the reactor decodes it from the wire.  We encode the
    // raw payload bytes manually via the same VLQ path the firmware uses.
    use kalico_host_rt::host_io::parser::encode_vlq;

    // VLQ-encode: msgid=10, arm_id, trip_clock_lo=0, trip_clock_hi=0,
    // trip_source_idx=0, fmt_version=1, stepper_count=0, stepper_data=""
    let mut payload = Vec::new();
    encode_vlq(&mut payload, 10).unwrap();          // msgid
    encode_vlq(&mut payload, arm_id as i64).unwrap(); // arm_id (%u)
    encode_vlq(&mut payload, 0).unwrap();            // trip_clock_lo
    encode_vlq(&mut payload, 0).unwrap();            // trip_clock_hi
    encode_vlq(&mut payload, 0).unwrap();            // trip_source_idx
    encode_vlq(&mut payload, 1).unwrap();            // fmt_version = 1
    encode_vlq(&mut payload, 0).unwrap();            // stepper_count = 0
    payload.push(0u8);                               // stepper_data length-prefix = 0

    wire::build_frame(&payload, seq)
}

// ---------------------------------------------------------------------------
// Helper: extract payloads from raw tx bytes (same as passthrough_integration)
// ---------------------------------------------------------------------------

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

    // Source MCU harness — must decode `kalico_endstop_tripped`.
    let mut src = ReactorHarness::new_with_parser(Arc::clone(&parser));
    // Sink MCU harness — must encode `trsync_trigger` when FireAndForget
    // arrives.  Shares the same parser since both messages live in it.
    let mut sink = ReactorHarness::new_with_parser(Arc::clone(&parser));

    // We need the sink's submission channel to send FireAndForget from the
    // interceptor closure.  Clone the Sender before the closure captures it.
    let sink_submission_tx: Sender<ReactorCommand> = sink.submission_tx.clone();

    // Build the relay closure the same way `trip_dispatch::prepare` does for
    // `SourceSpec::BridgeGpio`.  Uses real `FanOut` + arm_id filter.
    let sinks = vec![SinkSpec { mcu: 0, trsync_oid: SINK_OID }];
    let fan = Arc::new(FanOut::new(sinks));
    let triggered = Arc::new(AtomicBool::new(false));
    let fan2 = Arc::clone(&fan);
    let triggered2 = Arc::clone(&triggered);
    let want_arm_id: Option<u32> = Some(SRC_ARM_ID);

    let _interceptor_id = src.register_interceptor(
        "kalico_endstop_tripped",
        None, // BridgeGpio: no oid filter
        Box::new(move |params| {
            // Arm_id filter — same logic as trip_dispatch::prepare closure.
            if let Some(want) = want_arm_id {
                if params.get_u32("arm_id") != want {
                    return;
                }
            }
            fan2.on_trip(|_mcu, cmd| {
                // Send the command string to the sink reactor via FireAndForget.
                // This is what KalicoHostIo::send_fire_and_forget does over
                // its submission_tx, minus the Arc<KalicoHostIo> wrapper.
                let _ = sink_submission_tx.send(ReactorCommand::FireAndForget {
                    cmd: cmd.to_owned(),
                });
            });
            triggered2.store(true, Ordering::Release);
        }),
    );

    // Feed an inbound `kalico_endstop_tripped` frame for arm_id=42.
    let frame = build_endstop_tripped_frame(SRC_ARM_ID, 1);
    src.feed_rx(&frame);
    // Tick the source reactor: drain commands → poll_serial → interceptor fires.
    src.tick();

    // Verify the triggered flag was set by the relay closure.
    assert!(triggered.load(Ordering::Acquire), "relay closure must have set triggered");

    // Tick the sink reactor: drain commands (including the FireAndForget we just
    // sent) → encode + write trsync_trigger onto the sink wire.
    sink.tick();

    // Decode the sink's outbound wire bytes.
    let sink_tx = sink.tx_log();
    assert!(!sink_tx.is_empty(), "sink must have written bytes after relay");

    let payloads = extract_payloads(sink_tx);
    assert_eq!(payloads.len(), 1, "exactly one outbound frame on the sink");

    // Decode the payload to verify oid and reason.
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

    // Feed a frame with the WRONG arm_id.
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

    // First trip — sequence nibble 1.
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

    // Second trip with a different sequence nibble so the reactor doesn't
    // treat it as a retransmit.
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
