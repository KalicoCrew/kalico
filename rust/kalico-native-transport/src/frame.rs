use thiserror::Error;

pub const FRAME_SYNC: u8 = 0x55;
pub const CHANNEL_CONTROL: u8 = 0;
pub const CHANNEL_EVENTS: u8 = 1;

pub const FRAME_MIN_LEN_FIELD: usize = 2 + 1 + 2;
pub const FRAME_MAX_TOTAL: usize = 1 + u16::MAX as usize;

#[derive(Debug, Error)]
pub enum FrameError {
    #[error("bad sync byte: expected 0x55, got 0x{0:02x}")]
    BadSync(u8),
    #[error("frame too short: need at least {need} bytes, have {have}")]
    TooShort { need: usize, have: usize },
    #[error("len field {0} below minimum {1}")]
    LenTooSmall(usize, usize),
    #[error("crc mismatch: header says 0x{expected:04x}, computed 0x{actual:04x}")]
    CrcMismatch { expected: u16, actual: u16 },
}

// CRC-16/CCITT (poly 0x1021, init 0xFFFF, no reflection, no final xor).
// Must stay bit-identical to kalico-host-rt/src/host_io/wire.rs:crc16_ccitt.
pub fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in buf {
        let data = u16::from(byte) ^ (crc & 0x00FF);
        let data = (data ^ ((data << 4) & 0x00FF)) & 0xFF;
        crc = (crc >> 8) ^ (data << 8) ^ (data << 3) ^ (data >> 4);
    }
    crc
}

pub fn encode_frame(channel: u8, payload: &[u8]) -> Vec<u8> {
    let len_field = FRAME_MIN_LEN_FIELD + payload.len();
    assert!(
        u16::try_from(len_field).is_ok(),
        "kalico frame payload exceeds u16 length cap ({} > {})",
        len_field,
        u16::MAX
    );
    let total = 1 + len_field;
    let mut out = Vec::with_capacity(total);
    out.push(FRAME_SYNC);
    out.extend_from_slice(&(len_field as u16).to_le_bytes());
    out.push(channel);
    out.extend_from_slice(payload);
    let crc = crc16_ccitt(&out[1..out.len()]);
    out.extend_from_slice(&crc.to_le_bytes());
    debug_assert_eq!(out.len(), total);
    out
}

pub fn decode_frame(buf: &[u8]) -> Result<(u8, &[u8]), FrameError> {
    if buf.len() < 1 + FRAME_MIN_LEN_FIELD {
        return Err(FrameError::TooShort {
            need: 1 + FRAME_MIN_LEN_FIELD,
            have: buf.len(),
        });
    }
    if buf[0] != FRAME_SYNC {
        return Err(FrameError::BadSync(buf[0]));
    }
    let len_field = u16::from_le_bytes([buf[1], buf[2]]) as usize;
    if len_field < FRAME_MIN_LEN_FIELD {
        return Err(FrameError::LenTooSmall(len_field, FRAME_MIN_LEN_FIELD));
    }
    let total = 1 + len_field;
    if buf.len() < total {
        return Err(FrameError::TooShort {
            need: total,
            have: buf.len(),
        });
    }
    let channel = buf[3];
    let payload_end = total - 2;
    let payload = &buf[4..payload_end];
    let crc_expected = u16::from_le_bytes([buf[payload_end], buf[payload_end + 1]]);
    let crc_actual = crc16_ccitt(&buf[1..payload_end]);
    if crc_expected != crc_actual {
        return Err(FrameError::CrcMismatch {
            expected: crc_expected,
            actual: crc_actual,
        });
    }
    Ok((channel, payload))
}

#[cfg(test)]
mod tests;
