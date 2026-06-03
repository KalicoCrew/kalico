//! Wire helpers for the EtherCAT RT endpoint: `PushPieces` decode, response
//! builders, and `StatusHeartbeat` event emission.
//!
//! The old `LoadCurveCubic` / `PushSegment` / `ResetCurvePool` / `CreditFreed`
//! protocol has been replaced by the push-pieces model (§7.3 / §7.4):
//!
//! - Host → endpoint: `PushPieces` (0x0060) on `KALICO_CHANNEL_PIECES` (0x02)
//! - Endpoint → host: `PushPiecesResponse` (0x0061) on the control channel
//! - Endpoint → host: `StatusHeartbeat` (0x0083) on the events channel

use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL, CHANNEL_EVENTS};
use kalico_native_transport::wire_helpers::{
    decode_message_header, encode_message_header, MESSAGE_VERSION_DEFAULT,
};
use kalico_protocol::bootstrap::{IdentifyResponse, IDENTIFY_RESPONSE_BODY_LEN};
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{
    MessageKind, PushPieces, PushPiecesResponse, RuntimeCapsResponse, StatusHeartbeat,
};
use kalico_protocol::KALICO_CHANNEL_PIECES;

/// A decoded control-channel command plus the correlation id to answer with.
#[derive(Debug)]
pub enum Command {
    Identify { correlation_id: u32, proto_version: u8 },
    PushPieces { correlation_id: u32, msg: PushPieces },
    /// Host issued `QueryRuntimeCaps` (0x0040) on the control channel.
    /// The endpoint must respond with `RuntimeCapsResponse` carrying
    /// `total_piece_memory = AXIS_RING_CAPACITY * NUM_AXES * size_of::<PieceEntry>()`.
    QueryRuntimeCaps { correlation_id: u32 },
    Unknown { correlation_id: u32, kind_raw: u16 },
}

#[derive(Debug, PartialEq, Eq)]
pub enum DecodeCmdError {
    BadHeader,
    BadBody,
}

/// Decode a command from a `Frame::Kalico` payload on any channel.
///
/// The `channel` parameter disambiguates `PushPieces` (sent on
/// `KALICO_CHANNEL_PIECES = 0x02`) from control-channel messages.
pub fn decode_command(channel: u8, payload: &[u8]) -> Result<Command, DecodeCmdError> {
    let (hdr, body) = decode_message_header(payload).ok_or(DecodeCmdError::BadHeader)?;
    let cid = hdr.correlation_id;
    // PushPieces arrives on KALICO_CHANNEL_PIECES; decode it regardless of the
    // `kind_raw` guard so the endpoint handles it correctly even if the host
    // sends it with MessageKind::PushPieces (0x0060).
    if channel == KALICO_CHANNEL_PIECES
        || MessageKind::from_u16(hdr.kind_raw) == Some(MessageKind::PushPieces)
    {
        let msg = PushPieces::decode(body).map_err(|_| DecodeCmdError::BadBody)?;
        return Ok(Command::PushPieces { correlation_id: cid, msg });
    }
    match MessageKind::from_u16(hdr.kind_raw) {
        Some(MessageKind::Identify) => {
            let proto_version = body.first().copied().unwrap_or(0);
            Ok(Command::Identify { correlation_id: cid, proto_version })
        }
        Some(MessageKind::QueryRuntimeCaps) => {
            // Body is empty (the request carries no parameters).
            Ok(Command::QueryRuntimeCaps { correlation_id: cid })
        }
        _ => Ok(Command::Unknown { correlation_id: cid, kind_raw: hdr.kind_raw }),
    }
}

/// Build a control-channel command payload (header + body).
pub fn frame_payload(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(7 + body.len());
    out.extend_from_slice(&encode_message_header(kind, MESSAGE_VERSION_DEFAULT, correlation_id));
    out.extend_from_slice(body);
    out
}

/// Wrap a header+body payload into a full Layer-1 frame on the control channel.
pub fn control_frame(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    encode_frame(CHANNEL_CONTROL, &frame_payload(kind, correlation_id, body))
}

/// Build a `PushPiecesResponse` (0x0061) control frame.
///
/// - `result`: 0 = OK, negative = error code.
/// - `arrival_clock`: MCU-clock ticks at piece_sink_commit (0 for EtherCAT — not a
///   hardware MCU clock; `clock_freq = 1e9` Hz would require converting ns to ticks
///   which is identity; kept at 0 for the endpoint stub).
/// - `front_start_time`: `start_time` of the first piece (ns) so the host can
///   compute arrival lead.
pub fn push_pieces_response_frame(
    cid: u32,
    result: i32,
    arrival_clock: u64,
    front_start_time: u64,
) -> Vec<u8> {
    let body = PushPiecesResponse { result, arrival_clock, front_start_time }.encoded_to_vec();
    control_frame(MessageKind::PushPiecesResponse, cid, &body)
}

/// Build a `StatusHeartbeat` (0x0083) event frame.
///
/// `retired_counts` is one entry per configured axis (one for EtherCAT endpoints
/// that track a single servo axis). `engine_state` is 1 (running) unless the
/// ring is empty (0 = idle).
pub fn status_heartbeat_frame(engine_state: u8, retired_counts: &[u32]) -> Vec<u8> {
    let hb = StatusHeartbeat {
        engine_state,
        fault_code: 0,
        retired_counts: retired_counts.to_vec(),
    };
    let body = hb.encoded_to_vec();
    let payload = {
        let mut p =
            encode_message_header(MessageKind::StatusHeartbeat, MESSAGE_VERSION_DEFAULT, 0)
                .to_vec();
        p.extend_from_slice(&body);
        p
    };
    encode_frame(CHANNEL_EVENTS, &payload)
}

/// Build a `RuntimeCapsResponse` (0x0041) control frame.
///
/// `total_piece_memory` is the total bytes of piece storage this endpoint
/// provides: `AXIS_RING_CAPACITY * NUM_AXES * size_of::<PieceEntry>()`.
/// The host divides by `size_of::<PieceEntry>()` (= 32) and by the axis
/// count to recover the per-axis ring depth = `AXIS_RING_CAPACITY`.
pub fn runtime_caps_response_frame(cid: u32, total_piece_memory: u32) -> Vec<u8> {
    let body = RuntimeCapsResponse { total_piece_memory }.encoded_to_vec();
    control_frame(MessageKind::RuntimeCapsResponse, cid, &body)
}

/// Canned identify response advertising one motion channel, no special caps.
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
    use kalico_native_transport::frame::decode_frame;

    #[test]
    fn decodes_identify_on_control_channel() {
        let payload = frame_payload(MessageKind::Identify, 1, &[3u8]);
        match decode_command(0, &payload).unwrap() {
            Command::Identify { correlation_id: 1, proto_version: 3 } => {}
            other => panic!("wrong variant: {other:?}"),
        }
    }

    #[test]
    fn decodes_push_pieces_on_pieces_channel() {
        // Minimal valid PushPieces: 0 pieces (piece_count=0, no bytes).
        let msg = PushPieces {
            axis_idx: 0,
            piece_count: 0,
            start_slot: 0,
            new_head: 1,
            pieces_bytes: vec![],
        };
        let payload = frame_payload(MessageKind::PushPieces, 7, &msg.encoded_to_vec());
        match decode_command(KALICO_CHANNEL_PIECES, &payload).unwrap() {
            Command::PushPieces { correlation_id, msg: m } => {
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
    fn status_heartbeat_frame_on_events_channel() {
        let frame = status_heartbeat_frame(1, &[42u32, 0u32]);
        let (chan, payload) = decode_frame(&frame).unwrap();
        assert_eq!(chan, CHANNEL_EVENTS);
        let (hdr, body) = decode_message_header(payload).unwrap();
        assert_eq!(
            MessageKind::from_u16(hdr.kind_raw),
            Some(MessageKind::StatusHeartbeat)
        );
        // Unsolicited — correlation_id is 0.
        assert_eq!(hdr.correlation_id, 0);
        let hb = StatusHeartbeat::decode(body).unwrap();
        assert_eq!(hb.engine_state, 1);
        assert_eq!(hb.retired_counts, vec![42u32, 0u32]);
    }
}
