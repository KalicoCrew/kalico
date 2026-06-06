//! Wire helpers for the EtherCAT RT endpoint: `PushPieces` decode, response
//! builders, and `StatusHeartbeat` event emission.

use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL, CHANNEL_EVENTS};
use kalico_native_transport::wire_helpers::{
    decode_message_header, encode_message_header, MESSAGE_VERSION_DEFAULT,
};
use kalico_protocol::bootstrap::{IdentifyResponse, IDENTIFY_RESPONSE_BODY_LEN};
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{
    ClaimHandshakeReply, MessageKind, PushPieces, PushPiecesResponse, RuntimeCapsResponse,
    SetTorque, SetTorqueResponse, StatusHeartbeat,
};
use kalico_protocol::KALICO_CHANNEL_PIECES;

/// A decoded control-channel command plus the correlation id to answer with.
#[derive(Debug)]
pub enum Command {
    Identify {
        correlation_id: u32,
        proto_version: u8,
    },
    PushPieces {
        correlation_id: u32,
        msg: PushPieces,
    },
    QueryRuntimeCaps {
        correlation_id: u32,
    },
    ClaimHandshake {
        correlation_id: u32,
    },
    SetTorque {
        correlation_id: u32,
        msg: SetTorque,
    },
    Unknown {
        correlation_id: u32,
        kind_raw: u16,
    },
}

#[derive(Debug, PartialEq, Eq)]
pub enum DecodeCmdError {
    BadHeader,
    BadBody,
}

pub fn decode_command(channel: u8, payload: &[u8]) -> Result<Command, DecodeCmdError> {
    let (hdr, body) = decode_message_header(payload).ok_or(DecodeCmdError::BadHeader)?;
    let cid = hdr.correlation_id;
    if channel == KALICO_CHANNEL_PIECES
        || MessageKind::from_u16(hdr.kind_raw) == Some(MessageKind::PushPieces)
    {
        let msg = PushPieces::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
        return Ok(Command::PushPieces {
            correlation_id: cid,
            msg,
        });
    }
    match MessageKind::from_u16(hdr.kind_raw) {
        Some(MessageKind::Identify) => {
            let proto_version = body.first().copied().unwrap_or(0);
            Ok(Command::Identify {
                correlation_id: cid,
                proto_version,
            })
        }
        Some(MessageKind::QueryRuntimeCaps) => Ok(Command::QueryRuntimeCaps {
            correlation_id: cid,
        }),
        Some(MessageKind::ClaimHandshake) => Ok(Command::ClaimHandshake {
            correlation_id: cid,
        }),
        Some(MessageKind::SetTorque) => {
            let msg = SetTorque::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SetTorque {
                correlation_id: cid,
                msg,
            })
        }
        _ => Ok(Command::Unknown {
            correlation_id: cid,
            kind_raw: hdr.kind_raw,
        }),
    }
}

pub fn frame_payload(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(7 + body.len());
    out.extend_from_slice(&encode_message_header(
        kind,
        MESSAGE_VERSION_DEFAULT,
        correlation_id,
    ));
    out.extend_from_slice(body);
    out
}

pub fn control_frame(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    encode_frame(CHANNEL_CONTROL, &frame_payload(kind, correlation_id, body))
}

pub fn set_torque_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = SetTorqueResponse { result }.encoded_to_vec();
    control_frame(MessageKind::SetTorqueResponse, cid, &body)
}

pub fn push_pieces_response_frame(
    cid: u32,
    result: i32,
    arrival_clock: u64,
    front_start_time: u64,
) -> Vec<u8> {
    let body = PushPiecesResponse {
        result,
        arrival_clock,
        front_start_time,
    }
    .encoded_to_vec();
    control_frame(MessageKind::PushPiecesResponse, cid, &body)
}

pub fn status_heartbeat_frame(engine_state: u8, retired_counts: &[u32]) -> Vec<u8> {
    let hb = StatusHeartbeat {
        engine_state,
        fault_code: 0,
        retired_counts: retired_counts.to_vec(),
    };
    let body = hb.encoded_to_vec();
    let payload = {
        let mut p = encode_message_header(MessageKind::StatusHeartbeat, MESSAGE_VERSION_DEFAULT, 0)
            .to_vec();
        p.extend_from_slice(&body);
        p
    };
    encode_frame(CHANNEL_EVENTS, &payload)
}

pub fn runtime_caps_response_frame(cid: u32, total_piece_memory: u32) -> Vec<u8> {
    let body = RuntimeCapsResponse { total_piece_memory }.encoded_to_vec();
    control_frame(MessageKind::RuntimeCapsResponse, cid, &body)
}

pub fn claim_handshake_reply_frame(cid: u32, reply: &ClaimHandshakeReply) -> Vec<u8> {
    control_frame(
        MessageKind::ClaimHandshakeReply,
        cid,
        &reply.encoded_to_vec(),
    )
}

pub fn identify_response_frame(cid: u32, proto_version: u8) -> Vec<u8> {
    let resp = IdentifyResponse {
        proto_version,
        firmware_ver: 1,
        build_hash: [0u8; 20],
        schema_hash: [0u8; 32],
        reset_epoch: 0,
        capabilities: 0,
        mcu_serial: *b"ETHERCAT-RT\0",
    };
    let body = resp.encode_body_to_array();
    debug_assert_eq!(body.len(), IDENTIFY_RESPONSE_BODY_LEN);
    control_frame(MessageKind::IdentifyResponse, cid, &body)
}

#[cfg(test)]
mod tests {
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
        let frame = status_heartbeat_frame(1, &[42u32, 0u32]);
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
        // control channel (not the pieces channel)
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
}
