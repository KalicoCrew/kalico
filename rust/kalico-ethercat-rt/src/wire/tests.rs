use super::*;
use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::frame::decode_frame;
use kalico_protocol::messages::{SlaveState, SlaveStatus};

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
    let frame = status_heartbeat_frame(1, &[42u32, 0u32], 0);
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
