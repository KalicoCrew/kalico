use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL, CHANNEL_EVENTS};
use kalico_native_transport::wire_helpers::{
    decode_message_header, encode_message_header, MESSAGE_VERSION_DEFAULT,
};
use kalico_protocol::bootstrap::{IdentifyResponse, IDENTIFY_RESPONSE_BODY_LEN};
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{
    ClaimHandshakeReply, MessageKind, PushPieces, PushPiecesResponse, RestoreDriveLimitsResponse,
    RuntimeCapsResponse, SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse, SetDriveLimits,
    SetDriveLimitsResponse, SetTorque, SetTorqueResponse, StatusHeartbeat, StopResponse,
};
use kalico_protocol::KALICO_CHANNEL_PIECES;

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
    Stop {
        correlation_id: u32,
    },
    SetDriveLimits {
        correlation_id: u32,
        msg: SetDriveLimits,
    },
    RestoreDriveLimits {
        correlation_id: u32,
    },
    SdoRead {
        correlation_id: u32,
        msg: SdoRead,
    },
    SdoWrite {
        correlation_id: u32,
        msg: SdoWrite,
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
        Some(MessageKind::Stop) => Ok(Command::Stop {
            correlation_id: cid,
        }),
        Some(MessageKind::SetDriveLimits) => {
            let msg = SetDriveLimits::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SetDriveLimits {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::RestoreDriveLimits) => Ok(Command::RestoreDriveLimits {
            correlation_id: cid,
        }),
        Some(MessageKind::SdoRead) => {
            let msg = SdoRead::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SdoRead {
                correlation_id: cid,
                msg,
            })
        }
        Some(MessageKind::SdoWrite) => {
            let msg = SdoWrite::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
            Ok(Command::SdoWrite {
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

pub fn stop_response_frame(cid: u32, result: i32, discard_clock: u64) -> Vec<u8> {
    let body = StopResponse {
        result,
        discard_clock,
    }
    .encoded_to_vec();
    control_frame(MessageKind::StopResponse, cid, &body)
}

pub fn set_torque_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = SetTorqueResponse { result }.encoded_to_vec();
    control_frame(MessageKind::SetTorqueResponse, cid, &body)
}

pub fn sdo_read_response_frame(cid: u32, resp: &SdoReadResponse) -> Vec<u8> {
    control_frame(MessageKind::SdoReadResponse, cid, &resp.encoded_to_vec())
}

pub fn sdo_write_response_frame(cid: u32, resp: &SdoWriteResponse) -> Vec<u8> {
    control_frame(MessageKind::SdoWriteResponse, cid, &resp.encoded_to_vec())
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

pub fn set_drive_limits_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = SetDriveLimitsResponse { result }.encoded_to_vec();
    control_frame(MessageKind::SetDriveLimitsResponse, cid, &body)
}

pub fn restore_drive_limits_response_frame(cid: u32, result: i32) -> Vec<u8> {
    let body = RestoreDriveLimitsResponse { result }.encoded_to_vec();
    control_frame(MessageKind::RestoreDriveLimitsResponse, cid, &body)
}

pub fn status_heartbeat_frame(
    engine_state: u8,
    fault_code: u16,
    retired_counts: &[u32],
) -> Vec<u8> {
    let hb = StatusHeartbeat {
        engine_state,
        fault_code,
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
mod tests;
