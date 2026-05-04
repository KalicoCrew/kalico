//! Bootstrap-ABI Identify / `IdentifyResponse` codecs (§5).
//!
//! Both messages have a frozen byte layout outside the schema and are
//! decoded by hand. They still ride the framing layer (sync, len, channel,
//! crc) and use type tag `0x0001` / `0x0002` on the control channel with
//! the standard 7-byte per-message header (§7.2).
//!

use crate::wire_helpers::{encode_message_header, MESSAGE_VERSION_DEFAULT};
use kalico_protocol::{MessageKind, PER_MESSAGE_HEADER_LEN};

/// Bootstrap Identify body length: `proto_version` (u8). 1 byte.
pub const BOOTSTRAP_IDENTIFY_BODY_LEN: usize = 1;
/// Bootstrap `IdentifyResponse` body length per §5: 81 bytes.
pub const BOOTSTRAP_IDENTIFY_RESPONSE_BODY_LEN: usize = 81;

/// Total kalico-frame payload length (per-message header + body).
pub const BOOTSTRAP_IDENTIFY_LEN: usize = PER_MESSAGE_HEADER_LEN + BOOTSTRAP_IDENTIFY_BODY_LEN;
pub const BOOTSTRAP_IDENTIFY_RESPONSE_LEN: usize =
    PER_MESSAGE_HEADER_LEN + BOOTSTRAP_IDENTIFY_RESPONSE_BODY_LEN;

#[derive(Debug, Clone)]
pub struct IdentifyResponse {
    pub proto_version: u8,
    pub firmware_ver: u32,
    pub build_hash: [u8; 20],
    pub schema_hash: [u8; 32],
    pub reset_epoch: u32,
    pub capabilities: u64,
    pub mcu_serial: [u8; 12],
}

/// Encode the full payload (per-message header + bootstrap body) for an
/// `Identify` command. The framing layer wraps this further.
pub fn encode_identify(correlation_id: u32, proto_version: u8) -> Vec<u8> {
    let mut out = Vec::with_capacity(BOOTSTRAP_IDENTIFY_LEN);
    out.extend_from_slice(&encode_message_header(
        MessageKind::Identify,
        MESSAGE_VERSION_DEFAULT,
        correlation_id,
    ));
    out.push(proto_version);
    out
}

/// Decode an `IdentifyResponse` payload (per-message header + body). The
/// caller has already validated framing and CRC. Returns the decoded struct
/// and the `correlation_id` from the per-message header.
pub fn decode_identify_response(payload: &[u8]) -> Option<(u32, IdentifyResponse)> {
    if payload.len() != BOOTSTRAP_IDENTIFY_RESPONSE_LEN {
        return None;
    }
    let (header, body) = crate::wire_helpers::decode_message_header(payload)?;
    if header.kind_raw != MessageKind::IdentifyResponse as u16 {
        return None;
    }
    let proto_version = body[0];
    let firmware_ver = u32::from_le_bytes([body[1], body[2], body[3], body[4]]);
    let mut build_hash = [0u8; 20];
    build_hash.copy_from_slice(&body[5..25]);
    let mut schema_hash = [0u8; 32];
    schema_hash.copy_from_slice(&body[25..57]);
    let reset_epoch = u32::from_le_bytes([body[57], body[58], body[59], body[60]]);
    let capabilities = u64::from_le_bytes([
        body[61], body[62], body[63], body[64], body[65], body[66], body[67], body[68],
    ]);
    let mut mcu_serial = [0u8; 12];
    mcu_serial.copy_from_slice(&body[69..81]);
    Some((
        header.correlation_id,
        IdentifyResponse {
            proto_version,
            firmware_ver,
            build_hash,
            schema_hash,
            reset_epoch,
            capabilities,
            mcu_serial,
        },
    ))
}

/// Test helper: encode a complete `IdentifyResponse` payload (header + body).
pub fn encode_identify_response(correlation_id: u32, resp: &IdentifyResponse) -> Vec<u8> {
    let mut out = Vec::with_capacity(BOOTSTRAP_IDENTIFY_RESPONSE_LEN);
    out.extend_from_slice(&encode_message_header(
        MessageKind::IdentifyResponse,
        MESSAGE_VERSION_DEFAULT,
        correlation_id,
    ));
    out.push(resp.proto_version);
    out.extend_from_slice(&resp.firmware_ver.to_le_bytes());
    out.extend_from_slice(&resp.build_hash);
    out.extend_from_slice(&resp.schema_hash);
    out.extend_from_slice(&resp.reset_epoch.to_le_bytes());
    out.extend_from_slice(&resp.capabilities.to_le_bytes());
    out.extend_from_slice(&resp.mcu_serial);
    debug_assert_eq!(out.len(), BOOTSTRAP_IDENTIFY_RESPONSE_LEN);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identify_roundtrip_lengths() {
        let buf = encode_identify(0x1234_5678, 0x01);
        assert_eq!(buf.len(), BOOTSTRAP_IDENTIFY_LEN);
    }

    #[test]
    fn identify_response_roundtrip() {
        let resp = IdentifyResponse {
            proto_version: 0x01,
            firmware_ver: 0xDEAD_BEEF,
            build_hash: [0x42; 20],
            schema_hash: [0xAB; 32],
            reset_epoch: 0xCAFE_BABE,
            capabilities: 0x0000_0000_0000_0001,
            mcu_serial: *b"abcdef012345",
        };
        let buf = encode_identify_response(7, &resp);
        let (cid, decoded) = decode_identify_response(&buf).unwrap();
        assert_eq!(cid, 7);
        assert_eq!(decoded.proto_version, 0x01);
        assert_eq!(decoded.firmware_ver, 0xDEAD_BEEF);
        assert_eq!(decoded.schema_hash, [0xAB; 32]);
        assert_eq!(decoded.reset_epoch, 0xCAFE_BABE);
        assert_eq!(decoded.capabilities, 1);
        assert_eq!(&decoded.mcu_serial, b"abcdef012345");
    }
}
