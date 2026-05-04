//! Phase 10 Task 10.5 unit tests: `producer::push_segment` against
//! `MockTransport`.

mod mock_transport;

use std::sync::Arc;
use std::time::Duration;

use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::producer::{ProducerError, SegmentPushParams, push_segment};
use kalico_host_rt::transport::{MessageValue, TransportError};

use mock_transport::{MockTransport, mp_with};

fn default_params() -> SegmentPushParams {
    SegmentPushParams {
        id: 0,
        x_handle_packed: 0,
        y_handle_packed: 0,
        z_handle_packed: 0,
        e_handle_packed: 0,
        t_start: 0,
        t_end: 0,
        kinematics: 0,
        e_mode: 0,
        extrusion_ratio: 0.0,
    }
}

/// Spawn a thread that waits for a call with `name`, then completes it.
fn spawn_completer(mock: Arc<MockTransport>, name: &'static str, params: kalico_host_rt::transport::MessageParams) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let _ = mock.wait_for_call(name);
        mock.complete_call(name, params);
    })
}

#[test]
fn happy_path_pushes_and_returns_accepted_id_and_epoch() {
    let mock = Arc::new(MockTransport::new());
    let credit = CreditCounter::new(4);

    let _completer = spawn_completer(
        mock.clone(),
        "kalico_push_response",
        mp_with(&[
            ("result", MessageValue::I32(0)),
            ("accepted_segment_id", MessageValue::U32(42)),
            ("credit_epoch", MessageValue::U32(7)),
        ]),
    );

    let params = SegmentPushParams {
        id: 42,
        x_handle_packed: 0xCAFE_BABE,
        t_start: 1000,
        t_end: 2000,
        ..default_params()
    };
    let info = push_segment(&*mock, &credit, &params).expect("happy push");
    assert_eq!(info.accepted_segment_id, 42);
    assert_eq!(info.credit_epoch, 7);
    assert_eq!(credit.available(), 3, "credit decremented exactly once");
    let last = mock.last_sent().unwrap();
    assert!(last.contains("id=42"));
    assert!(last.contains("x_handle=3405691582"));
    assert!(last.contains("t_start_lo=1000"));
    assert!(last.contains("t_end_lo=2000"));
    assert!(last.contains("kinematics=0"));
    assert!(last.contains("e_mode=0"));
    assert!(last.contains("extrusion_ratio="));
    // Field ordering — `t_start_hi` must come before `t_start_lo`.
    let hi_pos = last.find("t_start_hi=").expect("t_start_hi missing");
    let lo_pos = last.find("t_start_lo=").expect("t_start_lo missing");
    assert!(
        hi_pos < lo_pos,
        "t_start_hi must precede t_start_lo to match firmware DECL_COMMAND"
    );
    let end_hi_pos = last.find("t_end_hi=").expect("t_end_hi missing");
    let end_lo_pos = last.find("t_end_lo=").expect("t_end_lo missing");
    assert!(
        end_hi_pos < end_lo_pos,
        "t_end_hi must precede t_end_lo to match firmware DECL_COMMAND"
    );
}

#[test]
fn no_credit_returns_nocredit_without_sending() {
    let mock = Arc::new(MockTransport::new());
    let credit = CreditCounter::new(1);
    credit.try_acquire().unwrap(); // exhaust
    let err = push_segment(&*mock, &credit, &SegmentPushParams {
        t_end: 100,
        ..default_params()
    }).unwrap_err();
    assert!(
        matches!(err, ProducerError::NoCredit),
        "expected NoCredit, got {err:?}"
    );
    assert_eq!(mock.sent_count(), 0, "must not send when out of credit");
}

#[test]
fn mcu_rejection_releases_credit() {
    let mock = Arc::new(MockTransport::new());
    let credit = CreditCounter::new(2);

    let _completer = spawn_completer(
        mock.clone(),
        "kalico_push_response",
        mp_with(&[
            ("result", MessageValue::I32(-103)),
            ("accepted_segment_id", MessageValue::U32(0)),
            ("credit_epoch", MessageValue::U32(0)),
        ]),
    );

    let err = push_segment(&*mock, &credit, &default_params()).unwrap_err();
    match err {
        ProducerError::McuRejected(r) => assert_eq!(r, -103),
        other => panic!("expected McuRejected, got {other:?}"),
    }
    assert_eq!(credit.available(), 2, "credit must be restored on reject");
}

#[test]
fn transport_timeout_releases_credit() {
    let mock = Arc::new(MockTransport::new());
    let credit = CreditCounter::new(1);
    // No response queued and short timeout → transport returns Timeout.
    let err = kalico_host_rt::producer::push_segment_with_timeout(
        &*mock,
        &credit,
        &default_params(),
        Duration::from_millis(20),
    ).unwrap_err();
    assert!(
        matches!(err, ProducerError::Transport(_)),
        "expected Transport(_), got {err:?}"
    );
    assert_eq!(credit.available(), 1, "credit must be restored on timeout");
    // Clean up the leftover pending call so mock can be dropped cleanly.
    mock.drop_pending("kalico_push_response");
}

#[test]
fn high_64bit_t_start_t_end_split_lo_hi() {
    let mock = Arc::new(MockTransport::new());
    let credit = CreditCounter::new(1);

    let _completer = spawn_completer(
        mock.clone(),
        "kalico_push_response",
        mp_with(&[
            ("result", MessageValue::I32(0)),
            ("accepted_segment_id", MessageValue::U32(0)),
            ("credit_epoch", MessageValue::U32(0)),
        ]),
    );

    let params = SegmentPushParams {
        t_start: 0x1_0000_0005,
        t_end: 0x1_0000_0006,
        ..default_params()
    };
    let _ = push_segment(&*mock, &credit, &params).unwrap();
    let last = mock.last_sent().unwrap();
    assert!(
        last.contains("t_start_lo=5") && last.contains("t_start_hi=1"),
        "unexpected wire encoding: {last}"
    );
    assert!(
        last.contains("t_end_lo=6") && last.contains("t_end_hi=1"),
        "unexpected wire encoding: {last}"
    );
}

#[test]
fn missing_result_field_surfaces_parse_error_and_releases_credit() {
    let mock = Arc::new(MockTransport::new());
    let credit = CreditCounter::new(2);

    let _completer = spawn_completer(
        mock.clone(),
        "kalico_push_response",
        mp_with(&[
            ("accepted_segment_id", MessageValue::U32(1)),
            ("credit_epoch", MessageValue::U32(1)),
        ]),
    );

    let err = push_segment(&*mock, &credit, &default_params()).unwrap_err();
    match err {
        ProducerError::Transport(TransportError::Parse(msg)) => {
            assert!(
                msg.contains("missing 'result' field"),
                "expected diagnostic to mention missing 'result', got {msg}"
            );
        }
        other => panic!("expected Transport(Parse(_)), got {other:?}"),
    }
    assert_eq!(
        credit.available(),
        2,
        "credit must be restored when malformed response trips parse error"
    );
}

// ---------------------------------------------------------------------------
// load_curve — incremental upload protocol (spec §4)
// ---------------------------------------------------------------------------

use kalico_host_rt::producer::{
    CHUNK_BYTES, CurveLoadParams, DEFAULT_LOAD_CURVE_TIMEOUT, load_curve,
};
use mock_transport::OwnedFieldValue;

fn make_curve(degree: u8, n_cps: usize, n_knots: usize) -> CurveLoadParams {
    CurveLoadParams {
        degree,
        cps_f32:   (0..n_cps).map(|i| i as f32 + 0.5).collect(),
        knots_f32: (0..n_knots).map(|i| i as f32 * 0.1).collect(),
    }
}

fn finalize_response(handle: u32, result: i32) -> kalico_host_rt::transport::MessageParams {
    mp_with(&[
        ("result", MessageValue::I32(result)),
        ("curve_handle_packed", MessageValue::U32(handle)),
    ])
}

#[test]
fn load_curve_drives_begin_chunks_finalize_in_order() {
    let mock = Arc::new(MockTransport::new());

    // 12 cps × 4 = 48 B → ceil(48/40) = 2 chunks (40 B + 8 B).
    // 14 knots × 4 = 56 B → ceil(56/40) = 2 chunks (40 B + 16 B).
    let params = make_curve(/*degree=*/ 5, /*n_cps=*/ 12, /*n_knots=*/ 14);
    let cps_buf = params.cps_bytes();
    let knots_buf = params.knots_bytes();
    let expected_chunks =
        cps_buf.len().div_ceil(CHUNK_BYTES) + knots_buf.len().div_ceil(CHUNK_BYTES);

    let _completer = spawn_completer(
        mock.clone(),
        "kalico_load_curve_finalize_response",
        finalize_response(0xCAFE_0007, 0),
    );

    let handle = load_curve(&*mock, /*slot=*/ 7, &params, DEFAULT_LOAD_CURVE_TIMEOUT)
        .expect("load_curve happy path");
    assert_eq!(handle, 0xCAFE_0007);

    // 1 begin (send_typed) + N chunks (send_typed).
    let typed = mock.sent_typed();
    let begins:  Vec<_> = typed.iter().filter(|r| r.name == "kalico_load_curve_begin").collect();
    let chunks:  Vec<_> = typed.iter().filter(|r| r.name == "kalico_load_curve_chunk").collect();
    assert_eq!(begins.len(), 1, "exactly one begin frame");
    assert_eq!(chunks.len(), expected_chunks, "chunk count = ceil(cps/40)+ceil(knots/40)");

    // Begin args: version=1, slot=7, degree=5, total_cps=12, total_knots=14.
    let begin_args: std::collections::HashMap<&str, &OwnedFieldValue> =
        begins[0].args.iter().map(|(k, v)| (k.as_str(), v)).collect();
    assert!(matches!(begin_args.get("version"), Some(OwnedFieldValue::Byte(1))));
    assert!(matches!(begin_args.get("slot"),    Some(OwnedFieldValue::U16(7))));
    assert!(matches!(begin_args.get("degree"),  Some(OwnedFieldValue::Byte(5))));
    assert!(matches!(begin_args.get("total_cps"),   Some(OwnedFieldValue::U16(12))));
    assert!(matches!(begin_args.get("total_knots"), Some(OwnedFieldValue::U16(14))));

    // Reassemble cps + knots from the chunk frames; verify byte-for-byte
    // identity with the producer's source buffers, plus correct kind/offset.
    let mut reconstructed_cps   = vec![0u8; cps_buf.len()];
    let mut reconstructed_knots = vec![0u8; knots_buf.len()];
    let mut seen_kinds = (0usize, 0usize); // (cps chunk count, knots chunk count)
    for chunk in &chunks {
        let argmap: std::collections::HashMap<&str, &OwnedFieldValue> =
            chunk.args.iter().map(|(k, v)| (k.as_str(), v)).collect();
        assert!(matches!(argmap.get("slot"), Some(OwnedFieldValue::U16(7))));
        let kind = match argmap.get("kind") {
            Some(OwnedFieldValue::Byte(b)) => *b,
            other => panic!("missing/wrong kind: {other:?}"),
        };
        let offset = match argmap.get("offset") {
            Some(OwnedFieldValue::U16(o)) => *o as usize,
            other => panic!("missing/wrong offset: {other:?}"),
        };
        let data = match argmap.get("data") {
            Some(OwnedFieldValue::Buffer(b)) => b.clone(),
            other => panic!("missing/wrong data: {other:?}"),
        };
        assert!(data.len() <= CHUNK_BYTES,
                "chunk data exceeds CHUNK_BYTES: {}", data.len());
        // offset must be a multiple of CHUNK_BYTES (sequential chunking).
        assert_eq!(offset % CHUNK_BYTES, 0, "non-aligned chunk offset {offset}");
        let dst = if kind == 0 {
            seen_kinds.0 += 1;
            &mut reconstructed_cps
        } else if kind == 1 {
            seen_kinds.1 += 1;
            &mut reconstructed_knots
        } else {
            panic!("unexpected kind {kind}");
        };
        dst[offset..offset + data.len()].copy_from_slice(&data);
    }
    assert_eq!(seen_kinds.0, cps_buf.len().div_ceil(CHUNK_BYTES));
    assert_eq!(seen_kinds.1, knots_buf.len().div_ceil(CHUNK_BYTES));
    assert_eq!(reconstructed_cps, cps_buf,   "cps reassembly mismatch");
    assert_eq!(reconstructed_knots, knots_buf, "knots reassembly mismatch");

    // Finalize: exactly one synchronous call_typed → kalico_load_curve_finalize.
    let finalize_calls = mock.sent_starting_with("kalico_load_curve_finalize");
    assert_eq!(finalize_calls.len(), 1, "one finalize call");
}

#[test]
fn load_curve_returns_mcu_rejected_on_nonzero_finalize_result() {
    let mock = Arc::new(MockTransport::new());
    let _completer = spawn_completer(
        mock.clone(),
        "kalico_load_curve_finalize_response",
        finalize_response(0, -2),
    );

    let params = make_curve(3, 4, 8);
    let err = load_curve(&*mock, 0, &params, DEFAULT_LOAD_CURVE_TIMEOUT).unwrap_err();
    match err {
        ProducerError::McuRejected(r) => assert_eq!(r, -2),
        other => panic!("expected McuRejected(-2), got {other:?}"),
    }
}

#[test]
fn load_curve_handles_non_aligned_buffer_lengths() {
    // Construct a curve whose cps_bytes is not a multiple of CHUNK_BYTES.
    // 12 cps × 4 = 48 B → 40 B + 8 B (one full chunk + one partial).
    // 11 knots × 4 = 44 B → 40 B + 4 B.
    let mock = Arc::new(MockTransport::new());
    let _completer = spawn_completer(
        mock.clone(),
        "kalico_load_curve_finalize_response",
        finalize_response(42, 0),
    );

    let params = make_curve(3, 12, 11);
    let cps_buf = params.cps_bytes();
    let knots_buf = params.knots_bytes();
    assert_eq!(cps_buf.len(), 48);
    assert_eq!(knots_buf.len(), 44);

    let h = load_curve(&*mock, 0, &params, DEFAULT_LOAD_CURVE_TIMEOUT).unwrap();
    assert_eq!(h, 42);

    let chunks = mock.sent_typed_named("kalico_load_curve_chunk");
    assert_eq!(chunks.len(), 4, "2 cps + 2 knot chunks");

    // First cps chunk: kind=0, offset=0, data.len()=40.
    let first_cps = &chunks[0];
    let m: std::collections::HashMap<&str, &OwnedFieldValue> =
        first_cps.args.iter().map(|(k, v)| (k.as_str(), v)).collect();
    assert!(matches!(m.get("kind"),   Some(OwnedFieldValue::Byte(0))));
    assert!(matches!(m.get("offset"), Some(OwnedFieldValue::U16(0))));
    let d0 = match m.get("data") { Some(OwnedFieldValue::Buffer(b)) => b, _ => panic!() };
    assert_eq!(d0.len(), CHUNK_BYTES);

    // Second cps chunk: kind=0, offset=40, data.len()=8.
    let second_cps = &chunks[1];
    let m: std::collections::HashMap<&str, &OwnedFieldValue> =
        second_cps.args.iter().map(|(k, v)| (k.as_str(), v)).collect();
    assert!(matches!(m.get("kind"),   Some(OwnedFieldValue::Byte(0))));
    assert!(matches!(m.get("offset"), Some(OwnedFieldValue::U16(40))));
    let d1 = match m.get("data") { Some(OwnedFieldValue::Buffer(b)) => b, _ => panic!() };
    assert_eq!(d1.len(), 8, "tail chunk = cps_bytes mod CHUNK_BYTES");

    // Third chunk: first knots, kind=1, offset=0, data.len()=40.
    let first_kn = &chunks[2];
    let m: std::collections::HashMap<&str, &OwnedFieldValue> =
        first_kn.args.iter().map(|(k, v)| (k.as_str(), v)).collect();
    assert!(matches!(m.get("kind"),   Some(OwnedFieldValue::Byte(1))));
    assert!(matches!(m.get("offset"), Some(OwnedFieldValue::U16(0))));

    // Fourth chunk: knots tail, kind=1, offset=40, data.len()=4.
    let second_kn = &chunks[3];
    let m: std::collections::HashMap<&str, &OwnedFieldValue> =
        second_kn.args.iter().map(|(k, v)| (k.as_str(), v)).collect();
    assert!(matches!(m.get("kind"),   Some(OwnedFieldValue::Byte(1))));
    assert!(matches!(m.get("offset"), Some(OwnedFieldValue::U16(40))));
    let d3 = match m.get("data") { Some(OwnedFieldValue::Buffer(b)) => b, _ => panic!() };
    assert_eq!(d3.len(), 4);
}

// Suppress the dead-code warning that fires when this test binary is
// compiled without referring to every helper from `mock_transport.rs`.
const _: Duration = Duration::from_millis(0);
