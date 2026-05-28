//! Wire-level helpers for the kalico-native transport: per-message header
//! encode/decode and field-level decoders the framer / transport layer need
//! during demux.
//!
//! Schema-level types (`MessageKind`, `PROTO_VERSION`, `SCHEMA_HASH`,
//! per-message structs) are owned by the `kalico-protocol` crate. Wire-level
//! plumbing is owned here. The split keeps `kalico-protocol` foundational
//! (zero deps on transport) and lets the transport own framing / dispatch
//! concerns.

use kalico_protocol::{MessageKind, PER_MESSAGE_HEADER_LEN};

/// Default per-message schema version. All MVP messages start at `0x01`.
pub const MESSAGE_VERSION_DEFAULT: u8 = 0x01;

/// Encode the per-message header (type | version | `correlation_id`) prefix
/// of a control-channel payload (§7.2). Body bytes go after.
#[must_use]
pub fn encode_message_header(kind: MessageKind, version: u8, correlation_id: u32) -> [u8; 7] {
    let tag = (kind as u16).to_le_bytes();
    let cid = correlation_id.to_le_bytes();
    [tag[0], tag[1], version, cid[0], cid[1], cid[2], cid[3]]
}

#[derive(Debug)]
pub struct MessageHeader {
    pub kind_raw: u16,
    pub version: u8,
    pub correlation_id: u32,
}

#[must_use]
pub fn decode_message_header(buf: &[u8]) -> Option<(MessageHeader, &[u8])> {
    if buf.len() < PER_MESSAGE_HEADER_LEN {
        return None;
    }
    let kind_raw = u16::from_le_bytes([buf[0], buf[1]]);
    let version = buf[2];
    let correlation_id = u32::from_le_bytes([buf[3], buf[4], buf[5], buf[6]]);
    Some((
        MessageHeader {
            kind_raw,
            version,
            correlation_id,
        },
        &buf[PER_MESSAGE_HEADER_LEN..],
    ))
}

/// Decode just the `reset_epoch` field out of a `StatusEvent` body (§7.4).
/// Layout: `engine_status` u8 | `queue_depth` u8 | `current_segment_id` u32 |
/// `last_fault` i32 | `fault_detail` u32 | `reset_epoch` u32. `reset_epoch` is at
/// offset 14.
#[must_use]
pub fn status_event_reset_epoch(body: &[u8]) -> Option<u32> {
    if body.len() < 18 {
        return None;
    }
    Some(u32::from_le_bytes([body[14], body[15], body[16], body[17]]))
}
