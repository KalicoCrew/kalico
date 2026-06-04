use kalico_protocol::{MessageKind, PER_MESSAGE_HEADER_LEN};

pub const MESSAGE_VERSION_DEFAULT: u8 = 0x01;

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
