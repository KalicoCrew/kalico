use super::*;

fn roundtrip<T>(v: &T) -> T
where
    T: Encode + Decode + PartialEq + std::fmt::Debug,
{
    let bytes = v.encoded_to_vec();
    let decoded = T::decode(&bytes).expect("decode ok");
    decoded
}

#[test]
fn message_kind_round_trips_via_u16() {
    for &k in &[
        MessageKind::Identify,
        MessageKind::IdentifyResponse,
        MessageKind::LoadCurveCubic,
        MessageKind::LoadCurveResponse,
        MessageKind::PushSegment,
        MessageKind::PushSegmentResponse,
        MessageKind::ConfigureAxes,
        MessageKind::ConfigureAxesResponse,
        MessageKind::QueryRuntimeCaps,
        MessageKind::RuntimeCapsResponse,
        MessageKind::ResetCurvePool,
        MessageKind::ResetCurvePoolResponse,
        MessageKind::StatusEvent,
        MessageKind::CreditFreed,
        MessageKind::FaultEvent,
    ] {
        assert_eq!(MessageKind::from_u16(k.as_u16()), Some(k));
    }
    assert_eq!(MessageKind::from_u16(0xFFFF), None);
}

#[test]
fn load_curve_cubic_roundtrip_realistic() {
    // 3 cubic pieces (each 5 × u32 = 20 bytes).
    let piece_count = 3u8;
    let pieces_len = piece_count as usize * 20;
    // Deterministic pseudo-random fill via a simple LCG.
    let mut state: u32 = 0xC0FFEEEE;
    let next = |s: &mut u32| -> u8 {
        *s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        (*s >> 24) as u8
    };
    let pieces_bytes: Vec<u8> = (0..pieces_len).map(|_| next(&mut state)).collect();
    let msg = LoadCurveCubic {
        slot_idx: 7,
        axis_idx: 1,
        piece_count,
        pieces_bytes: pieces_bytes.clone(),
    };
    let got = roundtrip(&msg);
    assert_eq!(got.slot_idx, 7);
    assert_eq!(got.axis_idx, 1);
    assert_eq!(got.piece_count, piece_count);
    assert_eq!(got.pieces_bytes, pieces_bytes);

    // Encoded body size: 2 (slot_idx) + 1 (axis_idx) + 1 (piece_count) + 3*20 = 64.
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 4 + pieces_len);
}

#[test]
fn load_curve_response_roundtrip() {
    let v = LoadCurveResponse {
        result: -3,
        curve_handle_packed: 0xDEAD_BEEF,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 8);
}

#[test]
fn push_segment_roundtrip() {
    let v = PushSegment {
        id: 0x0102_0304,
        handle_x: 0x1111_2222,
        handle_y: 0x3333_4444,
        handle_z: 0x5555_6666,
        handle_e: 0x7777_8888,
        t_start: 0x0011_2233_4455_6677,
        t_end: 0x8899_AABB_CCDD_EEFF,
        kinematics: 0x05,
        e_mode: 0x01,
        extrusion_ratio: 0.0234567,
    };
    assert_eq!(roundtrip(&v), v);
    // 4*5 (ids+handles) + 8*2 (timestamps) + 1 + 1 + 4 = 42.
    assert_eq!(v.encoded_to_vec().len(), 42);
}

#[test]
fn push_segment_response_roundtrip() {
    let v = PushSegmentResponse {
        result: 0,
        accepted_segment_id: 12345,
        credit_epoch: 67890,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 12);
}

#[test]
fn configure_axes_roundtrip() {
    let v = ConfigureAxes {
        kinematics: 0,
        present_mask: 0b1111,
        awd_mask: 0b0011,
        invert_mask: 0b0010,
        steps_per_mm: [80.0, 80.0, 400.0, 415.0],
    };
    assert_eq!(roundtrip(&v), v);
    // 4 (header bytes) + 16 (4×f32) = 20.
    assert_eq!(v.encoded_to_vec().len(), 20);
    let r = ConfigureAxesResponse { result: -7 };
    assert_eq!(roundtrip(&r), r);
    assert_eq!(r.encoded_to_vec().len(), 4);
}

#[test]
fn reset_curve_pool_roundtrip() {
    let req = ResetCurvePool;
    assert_eq!(roundtrip(&req), req);
    assert_eq!(req.encoded_to_vec().len(), 0);

    let resp = ResetCurvePoolResponse { result: 0 };
    assert_eq!(roundtrip(&resp), resp);
    assert_eq!(resp.encoded_to_vec().len(), 4);

    let resp_err = ResetCurvePoolResponse { result: -7 };
    assert_eq!(roundtrip(&resp_err), resp_err);
}

#[test]
fn status_event_roundtrip() {
    let v = StatusEvent {
        engine_status: 2,
        queue_depth: 7,
        current_segment_id: 999,
        last_fault: -42,
        fault_detail: 0xCAFE,
        reset_epoch: 0x1234_5678,
        retired_through_segment_id: 997,
    };
    assert_eq!(roundtrip(&v), v);
    // 1+1+4+4+4+4+4 = 22.
    assert_eq!(v.encoded_to_vec().len(), 22);
}

#[test]
fn credit_freed_roundtrip() {
    let v = CreditFreed {
        retired_through_segment_id: 4242,
        free_slots: 14,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 5);
}

#[test]
fn fault_event_roundtrip() {
    let v = FaultEvent {
        fault_code: 0x0007,
        fault_detail: 0xBAAD_F00D,
        segment_id: 11,
    };
    assert_eq!(roundtrip(&v), v);
    assert_eq!(v.encoded_to_vec().len(), 10);
}

#[test]
fn runtime_caps_roundtrip() {
    let original = RuntimeCapsResponse {
        curve_pool_n: 4,
        max_pieces_per_curve: 16,
    };
    let mut buf = Vec::new();
    original.encode(&mut buf);
    assert_eq!(buf.len(), RUNTIME_CAPS_RESPONSE_BODY_LEN);
    let mut c = Cursor::new(&buf);
    let decoded = RuntimeCapsResponse::decode_from(&mut c).unwrap();
    assert_eq!(decoded, original);
}

#[test]
fn decode_rejects_truncated_load_curve_cubic() {
    // 3-byte truncation of the 4-byte header: triggers UnexpectedEof.
    let bytes = &[0x01u8, 0x00, 0x00]; // slot_idx LE + axis_idx, no piece_count
    assert!(matches!(
        LoadCurveCubic::decode(bytes),
        Err(DecodeError::UnexpectedEof | DecodeError::ArrayLengthExceedsBuffer { .. })
    ));
}

#[test]
fn decode_rejects_oversized_piece_count() {
    // slot_idx u16 = 0, axis_idx u8 = 0, piece_count u8 = 0xFF (255 pieces = 5100
    // bytes), but no piece data follows. The decoder must reject without allocating.
    let bytes = &[0x00u8, 0x00, 0x00, 0xFF]; // header only, no pieces
    match LoadCurveCubic::decode(bytes) {
        Err(DecodeError::ArrayLengthExceedsBuffer { claimed, .. }) => {
            assert_eq!(claimed, 0xFF);
        }
        other => panic!("expected ArrayLengthExceedsBuffer, got {other:?}"),
    }
}

#[test]
fn decode_rejects_trailing_bytes() {
    let v = FaultEvent {
        fault_code: 1,
        fault_detail: 2,
        segment_id: 3,
    };
    let mut bytes = v.encoded_to_vec();
    bytes.push(0xAA);
    match FaultEvent::decode(&bytes) {
        Err(DecodeError::TrailingBytes { remaining: 1 }) => {}
        other => panic!("expected TrailingBytes(1), got {other:?}"),
    }
}
