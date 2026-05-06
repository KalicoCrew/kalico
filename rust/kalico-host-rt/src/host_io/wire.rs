pub const MESSAGE_MIN: usize = 5;
pub const MESSAGE_HEADER_SIZE: usize = 2;
pub const MESSAGE_TRAILER_SIZE: usize = 3;
pub const MESSAGE_SEQ_MASK: u8 = 0x0F;
pub const MESSAGE_DEST: u8 = 0x10;
pub const MESSAGE_SYNC: u8 = 0x7E;
pub const MESSAGE_MAX: usize = 64;

pub use kalico_native_transport::frame::crc16_ccitt;

pub fn build_frame(payload: &[u8], seq: u8) -> Vec<u8> {
    let msglen = MESSAGE_MIN + payload.len();
    let seq_byte = (seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
    let mut frame = Vec::with_capacity(msglen);
    frame.push(msglen as u8);
    frame.push(seq_byte);
    frame.extend_from_slice(payload);
    let crc = crc16_ccitt(&frame);
    frame.push((crc >> 8) as u8);
    frame.push((crc & 0xFF) as u8);
    frame.push(MESSAGE_SYNC);
    frame
}

/// Offline klipper-frame parser. Retained for `tests/captures_replay.rs`,
/// `tests/partial_frame_assembly.rs`, `tests/passthrough_integration.rs`,
/// and reactor's own test module to decode bytes the reactor wrote through
/// SerialFrameIo::write_all (raw passthrough). The live reactor path no
/// longer calls this — see SerialFrameIo + Demuxer for live decoding.
#[doc(hidden)]
pub fn extract_packet(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    while !buf.is_empty() {
        let msglen = buf[0] as usize;
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&msglen) {
            buf.remove(0);
            continue;
        }
        if buf.len() < msglen {
            return None;
        }
        let seq_byte = buf[1];
        if (seq_byte & !MESSAGE_SEQ_MASK) != MESSAGE_DEST || buf[msglen - 1] != MESSAGE_SYNC {
            buf.remove(0);
            continue;
        }
        let crc_off = msglen - MESSAGE_TRAILER_SIZE;
        let crc_expected = (u16::from(buf[crc_off]) << 8) | u16::from(buf[crc_off + 1]);
        let crc_actual = crc16_ccitt(&buf[..crc_off]);
        if crc_expected != crc_actual {
            buf.remove(0);
            continue;
        }
        let pkt = buf[..msglen].to_vec();
        buf.drain(..msglen);
        return Some(pkt);
    }
    None
}

/// Build a retransmit buffer: leading MESSAGE_SYNC byte followed by every
/// frame in the unacked window concatenated. Per spec §3.8 step 1+2.
pub fn build_retransmit_buffer<'a>(frames: impl IntoIterator<Item = &'a [u8]>) -> Vec<u8> {
    let mut buf = Vec::new();
    buf.push(MESSAGE_SYNC);
    for frame in frames {
        buf.extend_from_slice(frame);
    }
    buf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc16_matches_klipper_test_vector() {
        // Python: msgproto.crc16_ccitt(bytearray([5, 0x10])) returns 0x9E81.
        assert_eq!(crc16_ccitt(&[0x05, 0x10]), 0x9E81);
    }

    #[test]
    fn build_frame_roundtrips() {
        let frame = build_frame(&[0x01, 0x02], 0);
        assert_eq!(frame[0], 5 + 2);    // len
        assert_eq!(frame[1] & MESSAGE_SEQ_MASK, 0);
        assert_eq!(frame[1] & !MESSAGE_SEQ_MASK, MESSAGE_DEST);
        assert_eq!(*frame.last().unwrap(), MESSAGE_SYNC);
    }

    #[test]
    fn retransmit_buffer_starts_with_sync() {
        let f1 = build_frame(&[1], 0);
        let f2 = build_frame(&[2], 1);
        let buf = build_retransmit_buffer([f1.as_slice(), f2.as_slice()]);
        assert_eq!(buf[0], MESSAGE_SYNC);
        assert_eq!(buf.len(), 1 + f1.len() + f2.len());
    }
}
