use super::roundtrip;
use super::*;

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
        MessageKind::McuLog,
        MessageKind::ClaimHandshakeReply,
        MessageKind::ClaimHandshake,
        MessageKind::SdoRead,
        MessageKind::SdoReadResponse,
        MessageKind::SdoWrite,
        MessageKind::SdoWriteResponse,
    ] {
        assert_eq!(MessageKind::from_u16(k.as_u16()), Some(k));
    }
    assert_eq!(MessageKind::from_u16(0x0010), None); // LoadCurveCubic
    assert_eq!(MessageKind::from_u16(0x0011), None); // LoadCurveResponse
    assert_eq!(MessageKind::from_u16(0x0020), None); // PushSegment
    assert_eq!(MessageKind::from_u16(0x0021), None); // PushSegmentResponse
    assert_eq!(MessageKind::from_u16(0x0050), None); // ResetCurvePool
    assert_eq!(MessageKind::from_u16(0x0051), None); // ResetCurvePoolResponse
    assert_eq!(MessageKind::from_u16(0x0080), None); // StatusEvent (old)
    assert_eq!(MessageKind::from_u16(0x0081), None); // CreditFreed
    assert_eq!(MessageKind::from_u16(0x0090), None); // ClaimHandshakeReply (old, relocated to 0x0043)
    assert_eq!(MessageKind::from_u16(0x0091), None); // ClaimHandshake (old, relocated to 0x0042)
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
        retired_counts: vec![],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 3);
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.retired_counts.len(), 0);
}

#[test]
fn status_heartbeat_roundtrip_with_axes() {
    let msg = StatusHeartbeat {
        engine_state: 1,
        fault_code: 0,
        retired_counts: vec![42, 42, 10, 5],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
    assert_eq!(buf.len(), 19);
    let mut cursor = Cursor::new(&buf);
    let decoded = StatusHeartbeat::decode_from(&mut cursor).unwrap();
    assert_eq!(decoded.engine_state, 1);
    assert_eq!(decoded.fault_code, 0);
    assert_eq!(decoded.retired_counts, vec![42, 42, 10, 5]);
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
        pieces_bytes: vec![0x42; 96],
    };
    let mut buf = Vec::new();
    msg.encode(&mut buf);
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
    // Wire layout: result(i32 LE, 4 bytes) | arrival_clock(u64 LE, 8 bytes) |
    //              front_start_time(u64 LE, 8 bytes) = 20 bytes total.
    let msg = PushPiecesResponse {
        result: -2,
        arrival_clock: 0x0102_0304_0506_0708_u64,
        front_start_time: 0xDEAD_BEEF_CAFE_1234_u64,
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(
        buf.len(),
        20,
        "PushPiecesResponse body must be exactly 20 bytes"
    );
    assert_eq!(&buf[0..4], &0xFFFF_FFFE_u32.to_le_bytes());
    assert_eq!(&buf[4..12], &0x0102_0304_0506_0708_u64.to_le_bytes());
    assert_eq!(&buf[12..20], &0xDEAD_BEEF_CAFE_1234_u64.to_le_bytes());
    let decoded = PushPiecesResponse::decode(&buf).expect("decode ok");
    assert_eq!(decoded.result, -2);
    assert_eq!(decoded.arrival_clock, 0x0102_0304_0506_0708_u64);
    assert_eq!(decoded.front_start_time, 0xDEAD_BEEF_CAFE_1234_u64);
}

#[test]
fn push_pieces_response_error_path_zeros() {
    let msg = PushPiecesResponse {
        result: -7,
        arrival_clock: 0,
        front_start_time: 0,
    };
    let buf = msg.encoded_to_vec();
    assert_eq!(buf.len(), 20);
    let decoded = PushPiecesResponse::decode(&buf).expect("decode ok");
    assert_eq!(decoded.result, -7);
    assert_eq!(decoded.arrival_clock, 0);
    assert_eq!(decoded.front_start_time, 0);
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
fn set_torque_round_trips() {
    let msg = SetTorque {
        value: 1,
        execute_at_ns: 0xDEAD_BEEF_CAFE_F00D,
    };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 9, "u8 + u64 = 9 bytes");
    let back = SetTorque::decode(&bytes).expect("decode");
    assert_eq!(back, msg);
}

#[test]
fn set_torque_response_round_trips() {
    let msg = SetTorqueResponse { result: -311 };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 4, "i32 = 4 bytes");
    let back = SetTorqueResponse::decode(&bytes).expect("decode");
    assert_eq!(back, msg);
}

#[test]
fn set_torque_kinds_have_stable_tags() {
    assert_eq!(MessageKind::SetTorque.as_u16(), 0x0070);
    assert_eq!(MessageKind::SetTorqueResponse.as_u16(), 0x0071);
    assert_eq!(MessageKind::from_u16(0x0070), Some(MessageKind::SetTorque));
    assert_eq!(
        MessageKind::from_u16(0x0071),
        Some(MessageKind::SetTorqueResponse)
    );
    assert!(!MessageKind::SetTorque.is_event());
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

#[test]
fn stop_round_trips_empty_body() {
    let bytes = Stop.encoded_to_vec();
    assert!(bytes.is_empty(), "Stop body is empty");
    let back = Stop::decode(&bytes).expect("decode");
    assert_eq!(back, Stop);
}

#[test]
fn stop_response_round_trips() {
    let msg = StopResponse {
        result: 0,
        discard_clock: 0x0123_4567_89AB_CDEF,
    };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 12, "i32 + u64 = 12 bytes");
    assert_eq!(StopResponse::decode(&bytes).expect("decode"), msg);
}

#[test]
fn stop_kinds_have_stable_tags() {
    assert_eq!(MessageKind::Stop.as_u16(), 0x0072);
    assert_eq!(MessageKind::StopResponse.as_u16(), 0x0073);
    assert_eq!(MessageKind::from_u16(0x0072), Some(MessageKind::Stop));
    assert_eq!(
        MessageKind::from_u16(0x0073),
        Some(MessageKind::StopResponse)
    );
    assert!(!MessageKind::Stop.is_event());
}

#[test]
fn endstop_trip_round_trips_and_is_event() {
    let msg = EndstopTrip {
        endstop_id: 3,
        trip_clock: 0x0123_4567_89AB_CDEF,
    };
    let bytes = msg.encoded_to_vec();
    assert_eq!(bytes.len(), 9, "u8 + u64 = 9 bytes");
    assert_eq!(bytes[0], 3);
    assert_eq!(&bytes[1..9], &0x0123_4567_89AB_CDEF_u64.to_le_bytes());
    assert_eq!(EndstopTrip::decode(&bytes).expect("decode"), msg);
    assert!(MessageKind::EndstopTrip.is_event());
    assert_eq!(MessageKind::EndstopTrip.as_u16(), 0x0085);
}
