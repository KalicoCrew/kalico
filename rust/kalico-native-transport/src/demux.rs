//! Stream-level demuxer (§6 of the spec).
//!
//! Routes a single incoming USB-CDC byte stream into two parallel logical
//! streams:
//!
//! * Klipper frames — emitted as `Frame::Klipper(KlipperFrame)`. The
//!   forwarded bytes are the *full* Klipper frame including the leading
//!   length byte. Caller hands these to the existing Klipper parser
//!   (`kalico-host-rt`'s `extract_packet`).
//! * Kalico frames — emitted as `Frame::Kalico { channel, payload }`
//!   already CRC-validated. Caller hands payload to schema dispatch.
//!
//! The state machine is byte-oriented and interruptible at any boundary;
//! `feed_slice` simply iterates byte-by-byte.

use crate::frame::{FRAME_MIN_LEN_FIELD, FRAME_SYNC, crc16_ccitt};

const KLIPPER_LEN_MIN: u8 = 5;
const KLIPPER_LEN_MAX: u8 = 64;
const KLIPPER_INTERFRAME_SYNC: u8 = 0x7E;
// The next four shadow authoritative definitions in
// `kalico-host-rt::host_io::wire`. We can't import from that crate
// (it's downstream); keep these in sync if `wire.rs` ever changes them.
const MESSAGE_DEST: u8 = 0x10;
const MESSAGE_SEQ_MASK: u8 = 0x0F;
const MESSAGE_SYNC: u8 = 0x7E;
const MESSAGE_TRAILER_SIZE: usize = 3;

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
    pub fn seq_byte(&self) -> u8 {
        self.bytes[1]
    }
    /// Body slice: bytes[2 .. len-3] (excludes length byte, seq byte, CRC, trailer).
    pub fn body(&self) -> &[u8] {
        let len = self.bytes.len();
        &self.bytes[2..len - 3]
    }
    /// Full validated frame bytes.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }
    /// Consume into the owned Vec (for retransmit/await-response stash).
    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum StreamError {
    KlipperCrcMismatch {
        seq: u8,
        expected: u16,
        actual: u16,
    },
    KlipperBadTrailer {
        got: u8,
    },
    KlipperBadSeqDest {
        got: u8,
    },
    KlipperLenOutOfRange {
        len: u8,
    },
    KalicoCrcMismatch {
        channel: u8,
        expected: u16,
        actual: u16,
    },
    KalicoLenBelowMin {
        len: u16,
    },
    KalicoFrameTooShort {
        got: usize,
    },
}

impl std::fmt::Display for StreamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::KlipperCrcMismatch {
                seq,
                expected,
                actual,
            } => write!(
                f,
                "klipper crc mismatch seq=0x{seq:02x} expected=0x{expected:04x} actual=0x{actual:04x}"
            ),
            Self::KlipperBadTrailer { got } => write!(f, "klipper bad trailer 0x{got:02x}"),
            Self::KlipperBadSeqDest { got } => write!(f, "klipper bad seq/DEST byte 0x{got:02x}"),
            Self::KlipperLenOutOfRange { len } => write!(f, "klipper len out of range: {len}"),
            Self::KalicoCrcMismatch {
                channel,
                expected,
                actual,
            } => write!(
                f,
                "kalico crc mismatch ch={channel} expected=0x{expected:04x} actual=0x{actual:04x}"
            ),
            Self::KalicoLenBelowMin { len } => write!(f, "kalico len below min: {len}"),
            Self::KalicoFrameTooShort { got } => write!(f, "kalico frame too short: {got} bytes"),
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
    Frames {
        frames: Vec<Frame>,
        errors: Vec<StreamError>,
    },
    Timeout,
    PhantomZero,
}

#[derive(Debug)]
pub struct Demuxer {
    state: State,
    replay: std::collections::VecDeque<u8>,
}

impl Default for Demuxer {
    fn default() -> Self {
        Self::new()
    }
}

impl Demuxer {
    pub fn new() -> Self {
        Self {
            state: State::WaitingForFrame,
            replay: std::collections::VecDeque::new(),
        }
    }

    pub fn feed_slice(&mut self, bytes: &[u8]) -> (Vec<Frame>, Vec<StreamError>) {
        let mut frames = Vec::new();
        let mut errors = Vec::new();
        // Drain any pre-existing replay before consuming new bytes.
        while let Some(rb) = self.replay.pop_front() {
            match self.feed_inner(rb) {
                Ok(Some(f)) => frames.push(f),
                Ok(None) => {}
                Err(e) => errors.push(e),
            }
        }
        for &b in bytes {
            match self.feed_inner(b) {
                Ok(Some(f)) => frames.push(f),
                Ok(None) => {}
                Err(e) => errors.push(e),
            }
            // Each new byte may trigger validation that pushes bytes into
            // replay; drain them all before the next live byte.
            while let Some(rb) = self.replay.pop_front() {
                match self.feed_inner(rb) {
                    Ok(Some(f)) => frames.push(f),
                    Ok(None) => {}
                    Err(e) => errors.push(e),
                }
            }
        }
        (frames, errors)
    }

    /// Feed a single byte through the state machine.
    ///
    /// Returns:
    /// - `Ok(Some(frame))` — a complete, validated frame was produced.
    /// - `Ok(None)` — still accumulating bytes.
    /// - `Err(e)` — validation failure; the demuxer has resynced and (for
    ///   Klipper frames) pushed `frame[1..]` into `self.replay` for
    ///   1-byte-shift resync.
    fn feed_inner(&mut self, byte: u8) -> Result<Option<Frame>, StreamError> {
        match &mut self.state {
            State::WaitingForFrame => {
                match byte {
                    KLIPPER_LEN_MIN..=KLIPPER_LEN_MAX => {
                        // Begin Klipper frame: byte is the length, including itself.
                        let total = byte as usize;
                        let mut buf = Vec::with_capacity(total);
                        buf.push(byte);
                        self.state = State::InsideKlipper {
                            buf,
                            remaining: total - 1,
                        };
                        Ok(None)
                    }
                    FRAME_SYNC => {
                        let mut buf = Vec::with_capacity(64);
                        buf.push(byte);
                        self.state = State::InsideKalico { buf, total_len: 0 };
                        Ok(None)
                    }
                    KLIPPER_INTERFRAME_SYNC => {
                        // Stray inter-frame sync byte; tolerated.
                        Ok(None)
                    }
                    other => {
                        log::trace!("demuxer: dropping out-of-frame byte 0x{other:02x}");
                        Ok(None)
                    }
                }
            }
            State::InsideKlipper { buf, remaining } => {
                buf.push(byte);
                *remaining -= 1;
                if *remaining == 0 {
                    let frame = std::mem::take(buf);
                    self.state = State::WaitingForFrame;
                    match parse_klipper_frame(&frame) {
                        Ok(f) => Ok(Some(f)),
                        Err(e) => {
                            // 1-byte-shift resync: re-feed frame[1..] through the
                            // demuxer (preserving the demux.rs:13 "byte-oriented
                            // and interruptible" invariant). Drop only the false-latch
                            // length byte (frame[0]).
                            self.replay.extend(frame.iter().copied().skip(1));
                            Err(e)
                        }
                    }
                } else {
                    Ok(None)
                }
            }
            State::InsideKalico { buf, total_len } => {
                buf.push(byte);
                if *total_len == 0 && buf.len() >= 3 {
                    // Header (sync + len_lo + len_hi) is now in the buffer.
                    let len_field = u16::from_le_bytes([buf[1], buf[2]]) as usize;
                    if len_field < FRAME_MIN_LEN_FIELD {
                        self.state = State::WaitingForFrame;
                        return Err(StreamError::KalicoLenBelowMin {
                            len: len_field as u16,
                        });
                    }
                    *total_len = 1 + len_field;
                }
                if *total_len > 0 && buf.len() == *total_len {
                    let frame = std::mem::take(buf);
                    self.state = State::WaitingForFrame;
                    parse_kalico_frame(&frame).map(Some)
                } else {
                    Ok(None)
                }
            }
        }
    }
}

fn parse_klipper_frame(frame: &[u8]) -> Result<Frame, StreamError> {
    let len = frame.len();
    // Trailer check.
    if frame[len - 1] != MESSAGE_SYNC {
        return Err(StreamError::KlipperBadTrailer {
            got: frame[len - 1],
        });
    }
    // Seq-byte DEST flag (per extract_packet at wire.rs:44).
    let seq_byte = frame[1];
    if (seq_byte & !MESSAGE_SEQ_MASK) != MESSAGE_DEST {
        return Err(StreamError::KlipperBadSeqDest { got: seq_byte });
    }
    // CRC over bytes[0 .. len-3] (length byte + seq + payload), big-endian.
    let crc_off = len - MESSAGE_TRAILER_SIZE;
    let crc_expected = (u16::from(frame[crc_off]) << 8) | u16::from(frame[crc_off + 1]);
    let crc_actual = crc16_ccitt(&frame[..crc_off]);
    if crc_expected != crc_actual {
        return Err(StreamError::KlipperCrcMismatch {
            seq: seq_byte & MESSAGE_SEQ_MASK,
            expected: crc_expected,
            actual: crc_actual,
        });
    }
    Ok(Frame::Klipper(KlipperFrame::from_validated(frame.to_vec())))
}

fn parse_kalico_frame(frame: &[u8]) -> Result<Frame, StreamError> {
    // We've consumed exactly `total_len` bytes; revalidate CRC + extract.
    if frame.len() < 1 + FRAME_MIN_LEN_FIELD {
        return Err(StreamError::KalicoFrameTooShort { got: frame.len() });
    }
    let payload_end = frame.len() - 2;
    let crc_expected = u16::from_le_bytes([frame[payload_end], frame[payload_end + 1]]);
    let crc_actual = crc16_ccitt(&frame[1..payload_end]);
    if crc_expected != crc_actual {
        return Err(StreamError::KalicoCrcMismatch {
            channel: frame[3],
            expected: crc_expected,
            actual: crc_actual,
        });
    }
    let channel = frame[3];
    let payload = frame[4..payload_end].to_vec();
    Ok(Frame::Kalico { channel, payload })
}

#[cfg(test)]
mod tests;
