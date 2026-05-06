//! Stream-level demuxer (§6 of the spec).
//!
//! Routes a single incoming USB-CDC byte stream into two parallel logical
//! streams:
//!
//! * Klipper frames — emitted as `DemuxOutput::KlipperFrame(Vec<u8>)`. The
//!   forwarded bytes are the *full* Klipper frame including the leading
//!   length byte. Caller hands these to the existing Klipper parser
//!   (`kalico-host-rt`'s `extract_packet`).
//! * Kalico frames — emitted as `DemuxOutput::KalicoFrame { channel, payload }`
//!   already CRC-validated. Caller hands payload to schema dispatch.
//!
//! The state machine is byte-oriented and interruptible at any boundary;
//! `feed_slice` simply iterates byte-by-byte.

use crate::frame::{crc16_ccitt, FRAME_MIN_LEN_FIELD, FRAME_SYNC};

const KLIPPER_LEN_MIN: u8 = 5;
const KLIPPER_LEN_MAX: u8 = 64;
const KLIPPER_INTERFRAME_SYNC: u8 = 0x7E;

#[derive(Debug)]
enum State {
    WaitingForFrame,
    InsideKlipper {
        buf: Vec<u8>,
        remaining: usize,
    },
    InsideKalico {
        buf: Vec<u8>,
        // Once header is parsed: total frame length (including leading sync).
        // 0 means header not yet known.
        total_len: usize,
    },
}

/// Validated Klipper frame: length, CRC16-CCITT, and trailing 0x7E all checked
/// inside the demuxer per spec §3.4.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KlipperFrame {
    bytes: Vec<u8>, // private — invariant: passed full validation
}

impl KlipperFrame {
    /// Construct from already-validated bytes. Pub-crate to keep the
    /// validation invariant unforgeable from outside this crate.
    pub(crate) fn from_validated(bytes: Vec<u8>) -> Self {
        Self { bytes }
    }
    /// The seq+DEST byte at index 1.
    pub fn seq_byte(&self) -> u8 { self.bytes[1] }
    /// Body slice: bytes[2 .. len-3] (excludes length byte, seq byte, CRC, trailer).
    pub fn body(&self) -> &[u8] {
        let len = self.bytes.len();
        &self.bytes[2..len - 3]
    }
    /// Full validated frame bytes.
    pub fn bytes(&self) -> &[u8] { &self.bytes }
    /// Consume into the owned Vec (for retransmit/await-response stash).
    pub fn into_bytes(self) -> Vec<u8> { self.bytes }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StreamError {
    KlipperCrcMismatch    { seq: u8, expected: u16, actual: u16 },
    KlipperBadTrailer     { got: u8 },
    KlipperLenOutOfRange  { len: u8 },
    KalicoCrcMismatch     { channel: u8, expected: u16, actual: u16 },
    KalicoLenBelowMin     { len: u16 },
    KalicoFrameTooShort   { got: usize },
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::KlipperCrcMismatch { seq, expected, actual } =>
                write!(f, "klipper crc mismatch seq=0x{seq:02x} expected=0x{expected:04x} actual=0x{actual:04x}"),
            Self::KlipperBadTrailer { got } =>
                write!(f, "klipper bad trailer 0x{got:02x}"),
            Self::KlipperLenOutOfRange { len } =>
                write!(f, "klipper len out of range: {len}"),
            Self::KalicoCrcMismatch { channel, expected, actual } =>
                write!(f, "kalico crc mismatch ch={channel} expected=0x{expected:04x} actual=0x{actual:04x}"),
            Self::KalicoLenBelowMin { len } =>
                write!(f, "kalico len below min: {len}"),
            Self::KalicoFrameTooShort { got } =>
                write!(f, "kalico frame too short: {got} bytes"),
        }
    }
}

#[derive(Debug)]
pub enum Frame {
    Klipper(KlipperFrame),
    Kalico { channel: u8, payload: Vec<u8> },
}

#[derive(Debug)]
pub enum PollOutcome {
    Frames { frames: Vec<Frame>, errors: Vec<StreamError> },
    Timeout,
    PhantomZero,
}

#[derive(Debug)]
pub enum DemuxOutput {
    /// Complete Klipper frame, starting with the length byte.
    KlipperFrame(Vec<u8>),
    /// CRC-validated kalico frame.
    KalicoFrame { channel: u8, payload: Vec<u8> },
    /// Stream-level error; the demuxer has resynced to `WaitingForFrame`.
    StreamError(String),
}

#[derive(Debug)]
pub struct Demuxer {
    state: State,
}

impl Default for Demuxer {
    fn default() -> Self {
        Self::new()
    }
}

impl Demuxer {
    pub fn new() -> Self {
        Self { state: State::WaitingForFrame }
    }

    /// Feed a single byte; returns at most one output (a complete frame, or
    /// a stream error). Multiple calls may be needed before a frame emerges.
    pub fn feed(&mut self, byte: u8) -> Option<DemuxOutput> {
        match &mut self.state {
            State::WaitingForFrame => {
                match byte {
                    KLIPPER_LEN_MIN..=KLIPPER_LEN_MAX => {
                        // Begin Klipper frame: byte is the length, including itself.
                        let total = byte as usize;
                        let mut buf = Vec::with_capacity(total);
                        buf.push(byte);
                        self.state = State::InsideKlipper { buf, remaining: total - 1 };
                        None
                    }
                    FRAME_SYNC => {
                        let mut buf = Vec::with_capacity(64);
                        buf.push(byte);
                        self.state = State::InsideKalico { buf, total_len: 0 };
                        None
                    }
                    KLIPPER_INTERFRAME_SYNC => {
                        // Stray inter-frame sync byte; tolerated.
                        None
                    }
                    other => {
                        log::trace!("demuxer: dropping out-of-frame byte 0x{other:02x}");
                        None
                    }
                }
            }
            State::InsideKlipper { buf, remaining } => {
                buf.push(byte);
                *remaining -= 1;
                if *remaining == 0 {
                    let frame = std::mem::take(buf);
                    self.state = State::WaitingForFrame;
                    return Some(parse_klipper_frame(frame));
                }
                None
            }
            State::InsideKalico { buf, total_len } => {
                buf.push(byte);
                if *total_len == 0 && buf.len() >= 3 {
                    // Header (sync + len_lo + len_hi) is now in the buffer.
                    let len_field = u16::from_le_bytes([buf[1], buf[2]]) as usize;
                    if len_field < FRAME_MIN_LEN_FIELD {
                        let err = format!(
                            "kalico frame len field {len_field} below minimum {FRAME_MIN_LEN_FIELD}"
                        );
                        self.state = State::WaitingForFrame;
                        return Some(DemuxOutput::StreamError(err));
                    }
                    *total_len = 1 + len_field;
                }
                if *total_len > 0 && buf.len() == *total_len {
                    let frame = std::mem::take(buf);
                    self.state = State::WaitingForFrame;
                    Some(parse_kalico_frame(&frame))
                } else {
                    None
                }
            }
        }
    }

    pub fn feed_slice(&mut self, bytes: &[u8]) -> Vec<DemuxOutput> {
        let mut out = Vec::new();
        for &b in bytes {
            if let Some(o) = self.feed(b) {
                out.push(o);
            }
        }
        out
    }
}

fn parse_klipper_frame(frame: Vec<u8>) -> DemuxOutput {
    use crate::frame::crc16_ccitt;
    const MESSAGE_DEST: u8 = 0x10;
    const MESSAGE_SEQ_MASK: u8 = 0x0F;
    const MESSAGE_SYNC: u8 = 0x7E;
    const MESSAGE_TRAILER_SIZE: usize = 3;

    let len = frame.len();
    // Trailer check.
    if frame[len - 1] != MESSAGE_SYNC {
        return DemuxOutput::StreamError(format!(
            "klipper bad trailer 0x{:02x}", frame[len - 1]
        ));
    }
    // Seq-byte DEST flag (per extract_packet at wire.rs:44).
    let seq_byte = frame[1];
    if (seq_byte & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
        return DemuxOutput::StreamError(format!(
            "klipper bad seq/DEST byte 0x{:02x}", seq_byte
        ));
    }
    // CRC over bytes[0 .. len-3] (length byte + seq + payload), big-endian.
    let crc_off = len - MESSAGE_TRAILER_SIZE;
    let crc_expected = (u16::from(frame[crc_off]) << 8) | u16::from(frame[crc_off + 1]);
    let crc_actual = crc16_ccitt(&frame[..crc_off]);
    if crc_expected != crc_actual {
        return DemuxOutput::StreamError(format!(
            "klipper crc mismatch: expected 0x{crc_expected:04x}, got 0x{crc_actual:04x}"
        ));
    }
    DemuxOutput::KlipperFrame(frame)
}

fn parse_kalico_frame(frame: &[u8]) -> DemuxOutput {
    // We've consumed exactly `total_len` bytes; revalidate CRC + extract.
    if frame.len() < 1 + FRAME_MIN_LEN_FIELD {
        return DemuxOutput::StreamError("kalico frame shorter than minimum".to_string());
    }
    let payload_end = frame.len() - 2;
    let crc_expected = u16::from_le_bytes([frame[payload_end], frame[payload_end + 1]]);
    let crc_actual = crc16_ccitt(&frame[1..payload_end]);
    if crc_expected != crc_actual {
        return DemuxOutput::StreamError(format!(
            "kalico crc mismatch: expected 0x{crc_expected:04x}, got 0x{crc_actual:04x}"
        ));
    }
    let channel = frame[3];
    let payload = frame[4..payload_end].to_vec();
    DemuxOutput::KalicoFrame { channel, payload }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{encode_frame, CHANNEL_CONTROL};

    fn good_klipper_frame(payload: &[u8], seq: u8) -> Vec<u8> {
        // Build a valid Klipper frame: [len][seq|DEST][payload][crc_hi][crc_lo][0x7E]
        use crate::frame::crc16_ccitt;
        const MESSAGE_DEST: u8 = 0x10;
        const MESSAGE_SEQ_MASK: u8 = 0x0F;
        const MESSAGE_SYNC: u8 = 0x7E;
        let len = 5 + payload.len();
        assert!(len <= 64);
        let mut buf = Vec::with_capacity(len);
        buf.push(len as u8);
        buf.push((seq & MESSAGE_SEQ_MASK) | MESSAGE_DEST);
        buf.extend_from_slice(payload);
        let crc = crc16_ccitt(&buf);
        buf.push((crc >> 8) as u8);
        buf.push((crc & 0xFF) as u8);
        buf.push(MESSAGE_SYNC);
        buf
    }

    fn fake_klipper_frame(payload: &[u8]) -> Vec<u8> {
        good_klipper_frame(payload, 0)
    }

    #[test]
    fn klipper_validates_good_crc_and_trailer() {
        let frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
        let mut d = Demuxer::new();
        let outs = d.feed_slice(&frame);
        assert_eq!(outs.len(), 1, "expected one DemuxOutput");
        assert!(matches!(&outs[0], DemuxOutput::KlipperFrame(f) if f == &frame));
    }

    #[test]
    fn klipper_bad_crc_emits_stream_error() {
        let mut frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
        let len = frame.len();
        frame[len - 3] ^= 0xFF; // corrupt CRC hi
        let mut d = Demuxer::new();
        let outs = d.feed_slice(&frame);
        assert!(outs.iter().any(|o| matches!(o, DemuxOutput::StreamError(_))),
            "expected a StreamError, got {outs:?}");
    }

    #[test]
    fn klipper_bad_trailer_emits_stream_error() {
        let mut frame = good_klipper_frame(&[0x01, 0x02, 0x03], 0);
        let last = frame.len() - 1;
        frame[last] = 0x00; // not 0x7E
        let mut d = Demuxer::new();
        let outs = d.feed_slice(&frame);
        assert!(outs.iter().any(|o| matches!(o, DemuxOutput::StreamError(_))),
            "expected a StreamError, got {outs:?}");
    }

    #[test]
    fn klipper_then_kalico_then_klipper() {
        let k1 = fake_klipper_frame(&[1, 2, 3]);
        let kal = encode_frame(CHANNEL_CONTROL, b"hello there kalico");
        let k2 = fake_klipper_frame(&[9, 9]);

        let mut d = Demuxer::new();
        let mut stream = Vec::new();
        stream.extend_from_slice(&k1);
        stream.extend_from_slice(&kal);
        stream.extend_from_slice(&k2);
        let outs = d.feed_slice(&stream);

        assert_eq!(outs.len(), 3);
        assert!(matches!(&outs[0], DemuxOutput::KlipperFrame(f) if f == &k1));
        match &outs[1] {
            DemuxOutput::KalicoFrame { channel, payload } => {
                assert_eq!(*channel, CHANNEL_CONTROL);
                assert_eq!(payload.as_slice(), b"hello there kalico");
            }
            other => panic!("expected kalico frame, got {other:?}"),
        }
        assert!(matches!(&outs[2], DemuxOutput::KlipperFrame(f) if f == &k2));
    }

    #[test]
    fn kalico_payload_with_7e_does_not_resync() {
        // Payload contains the Klipper inter-frame sync byte; demuxer must
        // not break out of kalico state mid-frame.
        let payload = vec![0x7E; 200];
        let kal = encode_frame(CHANNEL_CONTROL, &payload);
        let mut d = Demuxer::new();
        let outs = d.feed_slice(&kal);
        assert_eq!(outs.len(), 1);
        match &outs[0] {
            DemuxOutput::KalicoFrame { channel, payload: p } => {
                assert_eq!(*channel, CHANNEL_CONTROL);
                assert_eq!(p, &payload);
            }
            other => panic!("expected kalico frame, got {other:?}"),
        }
    }

    #[test]
    fn kalico_payload_with_55_does_not_confuse() {
        let payload = vec![FRAME_SYNC; 200];
        let kal = encode_frame(CHANNEL_CONTROL, &payload);
        let mut d = Demuxer::new();
        let outs = d.feed_slice(&kal);
        assert_eq!(outs.len(), 1);
        assert!(matches!(&outs[0], DemuxOutput::KalicoFrame { .. }));
    }

    #[test]
    fn partial_frames_split_across_feeds() {
        let kal = encode_frame(CHANNEL_CONTROL, &(0u8..200).collect::<Vec<u8>>());
        let mut d = Demuxer::new();
        // Feed in 17-byte chunks.
        let mut total = 0;
        for chunk in kal.chunks(17) {
            let outs = d.feed_slice(chunk);
            for o in outs {
                if matches!(o, DemuxOutput::KalicoFrame { .. }) {
                    total += 1;
                }
            }
        }
        assert_eq!(total, 1);
    }

    #[test]
    fn malformed_kalico_len_recovers() {
        // sync + len=2 (below min). Demuxer flags error and resyncs.
        let mut d = Demuxer::new();
        let mut bytes = vec![FRAME_SYNC];
        bytes.extend_from_slice(&2u16.to_le_bytes());
        let outs = d.feed_slice(&bytes);
        assert_eq!(outs.len(), 1);
        assert!(matches!(&outs[0], DemuxOutput::StreamError(_)));
        // Now feed a valid Klipper frame; should still parse.
        let k = fake_klipper_frame(&[1]);
        let outs = d.feed_slice(&k);
        assert_eq!(outs.len(), 1);
        assert!(matches!(&outs[0], DemuxOutput::KlipperFrame(_)));
    }

    #[test]
    fn stray_7e_between_frames_tolerated() {
        let mut d = Demuxer::new();
        let kal = encode_frame(CHANNEL_CONTROL, b"abc");
        let outs = d.feed_slice(&[0x7E, 0x7E, 0x7E]);
        assert!(outs.is_empty());
        let outs = d.feed_slice(&kal);
        assert_eq!(outs.len(), 1);
        assert!(matches!(&outs[0], DemuxOutput::KalicoFrame { .. }));
    }

    #[test]
    fn out_of_frame_garbage_dropped() {
        let mut d = Demuxer::new();
        // 0x80 is not Klipper-len-range (5..=64), not 0x55, not 0x7E.
        let outs = d.feed_slice(&[0x80, 0x81, 0x82]);
        assert!(outs.is_empty());
        let kal = encode_frame(CHANNEL_CONTROL, b"x");
        let outs = d.feed_slice(&kal);
        assert_eq!(outs.len(), 1);
    }
}
