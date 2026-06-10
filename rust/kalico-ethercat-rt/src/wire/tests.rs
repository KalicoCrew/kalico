use super::*;
use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::frame::decode_frame;
use kalico_protocol::messages::{
    RestoreDriveLimitsResponse, SetDriveLimits, SetDriveLimitsResponse, SlaveState, SlaveStatus,
    StopResponse,
};

#[test]
fn decodes_identify_on_control_channel() {
    let payload = frame_payload(MessageKind::Identify, 1, &[3u8]);
    match decode_command(0, &payload).unwrap() {
        Command::Identify {
            correlation_id: 1,
            proto_version: 3,
        } => {}
        other => panic!("wrong variant: {other:?}"),
    }
}

#[test]
fn decodes_push_pieces_on_pieces_channel() {
    let msg = PushPieces {
        axis_idx: 0,
        piece_count: 0,
        start_slot: 0,
        new_head: 1,
        pieces_bytes: vec![],
    };
    let payload = frame_payload(MessageKind::PushPieces, 7, &msg.encoded_to_vec());
    match decode_command(KALICO_CHANNEL_PIECES, &payload).unwrap() {
        Command::PushPieces {
            correlation_id,
            msg: m,
        } => {
            assert_eq!(correlation_id, 7);
            assert_eq!(m.axis_idx, 0);
            assert_eq!(m.piece_count, 0);
            assert_eq!(m.new_head, 1);
        }
        other => panic!("wrong variant: {other:?}"),
    }
}

#[test]
fn push_pieces_response_decodes_back() {
    let frame = push_pieces_response_frame(42, 0, 0, 1_000_000_000);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 42);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::PushPiecesResponse)
    );
    let r = PushPiecesResponse::decode(body).unwrap();
    assert_eq!(r.result, 0);
    assert_eq!(r.front_start_time, 1_000_000_000);
}

#[test]
fn claim_handshake_reply_frame_decodes() {
    let reply = ClaimHandshakeReply {
        slave_statuses: vec![SlaveStatus {
            slave_idx: 1,
            state: SlaveState::Ok,
            fault_code: 0,
        }],
    };
    let frame = claim_handshake_reply_frame(7, &reply);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 7);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::ClaimHandshakeReply)
    );
    let decoded = ClaimHandshakeReply::decode(body).unwrap();
    assert_eq!(decoded, reply);
}

#[test]
fn decode_command_yields_claim_handshake_variant() {
    let payload = frame_payload(MessageKind::ClaimHandshake, 99, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::ClaimHandshake { correlation_id: 99 } => {}
        other => panic!("expected ClaimHandshake, got {other:?}"),
    }
}

#[test]
fn status_heartbeat_frame_on_events_channel() {
    let frame = status_heartbeat_frame(1, 0, &[42u32, 0u32]);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_EVENTS);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::StatusHeartbeat)
    );
    assert_eq!(hdr.correlation_id, 0);
    let hb = StatusHeartbeat::decode(body).unwrap();
    assert_eq!(hb.engine_state, 1);
    assert_eq!(hb.retired_counts, vec![42u32, 0u32]);
}

#[test]
fn decodes_set_torque_command() {
    let msg = SetTorque {
        value: 1,
        execute_at_ns: 123_456_789,
    };
    let payload = frame_payload(MessageKind::SetTorque, 7, &msg.encoded_to_vec());
    let cmd = decode_command(0, &payload[..]).expect("decode");
    match cmd {
        Command::SetTorque {
            correlation_id,
            msg: m,
        } => {
            assert_eq!(correlation_id, 7);
            assert_eq!(m.value, 1);
            assert_eq!(m.execute_at_ns, 123_456_789);
        }
        other => panic!("expected SetTorque, got {other:?}"),
    }
}

#[test]
fn set_torque_response_frame_round_trips() {
    let frame = set_torque_response_frame(9, -312);
    let mut demux = Demuxer::new();
    let (frames, errs) = demux.feed_slice(&frame);
    assert!(errs.is_empty());
    let Frame::Kalico { payload, .. } = &frames[0] else {
        panic!("expected kalico frame");
    };
    let (hdr, body) = decode_message_header(payload).expect("header");
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SetTorqueResponse)
    );
    assert_eq!(hdr.correlation_id, 9);
    let resp = SetTorqueResponse::decode(body).expect("body");
    assert_eq!(resp.result, -312);
}

#[test]
fn decodes_stop_command() {
    let payload = frame_payload(MessageKind::Stop, 11, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::Stop { correlation_id: 11 } => {}
        other => panic!("expected Stop, got {other:?}"),
    }
}

#[test]
fn decodes_set_drive_limits_command() {
    let msg = SetDriveLimits {
        following_error_counts: 8192,
        max_torque_tenth_pct: 500,
    };
    let payload = frame_payload(MessageKind::SetDriveLimits, 3, &msg.encoded_to_vec());
    match decode_command(0, &payload).unwrap() {
        Command::SetDriveLimits {
            correlation_id: 3,
            msg: m,
        } => {
            assert_eq!(m.following_error_counts, 8192);
            assert_eq!(m.max_torque_tenth_pct, 500);
        }
        other => panic!("expected SetDriveLimits, got {other:?}"),
    }
}

#[test]
fn decodes_restore_drive_limits_command() {
    let payload = frame_payload(MessageKind::RestoreDriveLimits, 4, &[]);
    match decode_command(0, &payload).unwrap() {
        Command::RestoreDriveLimits { correlation_id: 4 } => {}
        other => panic!("expected RestoreDriveLimits, got {other:?}"),
    }
}

#[test]
fn drive_limits_response_frames_round_trip() {
    let frame = set_drive_limits_response_frame(6, -315);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 6);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::SetDriveLimitsResponse)
    );
    assert_eq!(SetDriveLimitsResponse::decode(body).unwrap().result, -315);

    let frame = restore_drive_limits_response_frame(7, 0);
    let (_, payload) = decode_frame(&frame).unwrap();
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::RestoreDriveLimitsResponse)
    );
    assert_eq!(RestoreDriveLimitsResponse::decode(body).unwrap().result, 0);
    assert_eq!(hdr.correlation_id, 7);
}

#[test]
fn status_heartbeat_frame_carries_fault_code() {
    let frame = status_heartbeat_frame(1, 0x8611, &[5u32]);
    let (_, payload) = decode_frame(&frame).unwrap();
    let (_, body) = decode_message_header(payload).unwrap();
    let hb = StatusHeartbeat::decode(body).unwrap();
    assert_eq!(hb.fault_code, 0x8611);
    assert_eq!(hb.engine_state, 1);
}

#[test]
fn stop_response_frame_round_trips() {
    let frame = stop_response_frame(5, -311, 123_456_789);
    let (chan, payload) = decode_frame(&frame).unwrap();
    assert_eq!(chan, CHANNEL_CONTROL);
    let (hdr, body) = decode_message_header(payload).unwrap();
    assert_eq!(hdr.correlation_id, 5);
    assert_eq!(
        MessageKind::from_u16(hdr.kind_raw),
        Some(MessageKind::StopResponse)
    );
    let r = StopResponse::decode(body).unwrap();
    assert_eq!(r.result, -311);
    assert_eq!(r.discard_clock, 123_456_789);
}
