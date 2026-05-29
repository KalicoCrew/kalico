use super::*;

fn roundtrip<T>(v: &T) -> T
where
    T: Encode + Decode + PartialEq + std::fmt::Debug,
{
    let bytes = v.encoded_to_vec();
    T::decode(&bytes).expect("decode ok")
}

#[test]
fn message_kind_round_trips_via_u16() {
    for &k in &[
        MessageKind::Identify,
        MessageKind::IdentifyResponse,
        MessageKind::ConfigureAxes,
        MessageKind::ConfigureAxesResponse,
        MessageKind::QueryRuntimeCaps,
        MessageKind::RuntimeCapsResponse,
        MessageKind::PushPieces,
        MessageKind::PushPiecesResponse,
        MessageKind::FaultEvent,
        MessageKind::StatusHeartbeat,
    ] {
        assert_eq!(MessageKind::from_u16(k.as_u16()), Some(k));
    }
    // Old IDs (freed by removing dead messages) must no longer decode.
    assert_eq!(MessageKind::from_u16(0x0010), None); // LoadCurveCubic
    assert_eq!(MessageKind::from_u16(0x0011), None); // LoadCurveResponse
    assert_eq!(MessageKind::from_u16(0x0020), None); // PushSegment
    assert_eq!(MessageKind::from_u16(0x0021), None); // PushSegmentResponse
    assert_eq!(MessageKind::from_u16(0x0050), None); // ResetCurvePool
    assert_eq!(MessageKind::from_u16(0x0051), None); // ResetCurvePoolResponse
    assert_eq!(MessageKind::from_u16(0x0080), None); // StatusEvent (old)
    assert_eq!(MessageKind::from_u16(0x0081), None); // CreditFreed
    assert_eq!(MessageKind::from_u16(0xFFFF), None);
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
fn runtime_caps_response_new_format() {
    let msg = RuntimeCapsResponse {
        total_piece_memory: 63488,
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 4);
    let mut cursor = Cursor::new(&buf);
    let decoded = RuntimeCapsResponse::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.total_piece_memory, 63488);
}

#[test]
fn status_heartbeat_roundtrip_empty() {
    let msg = StatusHeartbeat {
        engine_state: 0,
        fault_code: 0,
        consumed_counts: vec![],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 3); // 1+1+1 (num_axes=0)
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.consumed_counts.len(), 0);
}

#[test]
fn status_heartbeat_roundtrip_with_axes() {
    let msg = StatusHeartbeat {
        engine_state: 1,
        fault_code: 0,
        consumed_counts: vec![42, 42, 10, 5],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    // 1 + 1 + 1 + 4*4 = 19 bytes
    assert_eq!(buf.len(), 19);
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.engine_state, 1);
    assert_eq!(decoded.fault_code, 0);
    assert_eq!(decoded.consumed_counts, vec![42, 42, 10, 5]);
}

#[test]
fn push_pieces_roundtrip_single() {
    let msg = PushPieces {
        axis_idx: 2,
        piece_count: 1,
        start_slot: 0,
        new_head: 0,
        pieces_bytes: vec![0xAB; 32],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    // axis_idx(1) + piece_count(1) + start_slot(2) + new_head(4) + 32 bytes pieces = 40 bytes.
    assert_eq!(buf.len(), 40);
    let mut cursor = Cursor::new(&buf);
    let decoded = PushPieces::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.axis_idx, 2);
    assert_eq!(decoded.piece_count, 1);
    assert_eq!(decoded.start_slot, 0);
    assert_eq!(decoded.new_head, 0);
    assert_eq!(decoded.pieces_bytes.len(), 32);
    assert_eq!(decoded.pieces_bytes[0], 0xAB);
}

#[test]
fn push_pieces_v2_roundtrip_carries_slot_and_head() {
    let msg = PushPieces {
        axis_idx: 2,
        piece_count: 1,
        start_slot: 41,
        new_head: 5000,
        pieces_bytes: vec![0xAB; 32],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    // axis_idx(1) + piece_count(1) + start_slot(2) + new_head(4) + 32 = 40 bytes.
    assert_eq!(buf.len(), 40);
    let mut cursor = Cursor::new(&buf);
    let decoded = PushPieces::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.axis_idx, 2);
    assert_eq!(decoded.piece_count, 1);
    assert_eq!(decoded.start_slot, 41);
    assert_eq!(decoded.new_head, 5000);
    assert_eq!(decoded.pieces_bytes, vec![0xAB; 32]);
}

#[test]
fn push_pieces_roundtrip_multiple() {
    let msg = PushPieces {
        axis_idx: 0,
        piece_count: 3,
        start_slot: 0,
        new_head: 0,
        pieces_bytes: vec![0x42; 96], // 3 * 32 = 96
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    // axis_idx(1) + piece_count(1) + start_slot(2) + new_head(4) + 3*32 = 104 bytes.
    assert_eq!(buf.len(), 8 + 3 * 32);
    let mut cursor = Cursor::new(&buf);
    let decoded = PushPieces::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.piece_count, 3);
    assert_eq!(decoded.start_slot, 0);
    assert_eq!(decoded.new_head, 0);
    assert_eq!(decoded.pieces_bytes.len(), 96);
}

#[test]
fn push_pieces_response_roundtrip() {
    let msg = PushPiecesResponse { result: -2 };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    let mut cursor = Cursor::new(&buf);
    let decoded = PushPiecesResponse::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.result, -2);
}

#[test]
fn push_pieces_kind_in_message_kind_table() {
    assert_eq!(MessageKind::from_u16(0x0060), Some(MessageKind::PushPieces));
    assert_eq!(
        MessageKind::from_u16(0x0061),
        Some(MessageKind::PushPiecesResponse)
    );
    assert_eq!(MessageKind::PushPieces.as_u16(), 0x0060);
    assert_eq!(MessageKind::PushPiecesResponse.as_u16(), 0x0061);
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
