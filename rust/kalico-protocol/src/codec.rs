//! Encode/Decode traits and primitive helpers used by message bodies.
//!
//! Per spec §7.2, the trait covers **message body only** — the per-message
//! header (`type` + `version` + `correlation_id`) is the framing layer's
//! responsibility. The type tag is supplied externally via [`MessageKind`].
//!
//! Body encoding rules (per spec §7):
//! - All multi-byte integers little-endian.
//! - Floats f32/f64 little-endian IEEE-754.
//! - Arrays length-prefixed with `u32_le` followed by elements packed
//!   contiguously.
//! - Structs packed in declaration order with no padding.
//!
//! No 255-byte cap (that was Klipper's `Buffer` field; kalico's `u32` length
//! prefix is the only cap).

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecodeError {
    /// Buffer ended before the message body was fully consumed.
    UnexpectedEof,
    /// Length-prefixed array claimed more elements than the buffer holds
    /// (could panic-cause a huge allocation; we reject up-front).
    ArrayLengthExceedsBuffer { claimed: u32, available: usize },
    /// Trailing bytes after a successful decode of a fixed-size message.
    /// Variable-size messages don't use this; they consume what they need.
    TrailingBytes { remaining: usize },
}

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::UnexpectedEof => f.write_str("unexpected EOF while decoding message body"),
            Self::ArrayLengthExceedsBuffer { claimed, available } => write!(
                f,
                "array length {claimed} exceeds remaining buffer ({available} bytes)"
            ),
            Self::TrailingBytes { remaining } => {
                write!(f, "{remaining} trailing byte(s) after decode")
            }
        }
    }
}

impl std::error::Error for DecodeError {}

/// Encode a message body to a `Vec<u8>`. The body excludes the per-message
/// header (`type` + `version` + `correlation_id`); the framing layer prepends
/// that.
pub trait Encode {
    fn encode(&self, out: &mut Vec<u8>);

    fn encoded_to_vec(&self) -> Vec<u8> {
        let mut v = Vec::new();
        self.encode(&mut v);
        v
    }
}

/// Decode a message body from a byte slice. Variable-size messages
/// (`LoadCurve`) consume exactly what they need; fixed-size messages consume
/// their full extent. Callers are expected to have already sliced the buffer
/// to the message body extent — extra trailing bytes are reported via
/// [`DecodeError::TrailingBytes`] from the convenience [`Decode::decode`]
/// method.
pub trait Decode: Sized {
    fn decode_from(cursor: &mut Cursor<'_>) -> Result<Self, DecodeError>;

    fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        let mut cur = Cursor::new(buf);
        let v = Self::decode_from(&mut cur)?;
        if !cur.is_empty() {
            return Err(DecodeError::TrailingBytes {
                remaining: cur.remaining(),
            });
        }
        Ok(v)
    }
}

#[derive(Debug)]
pub struct Cursor<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Cursor<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }
    pub fn remaining(&self) -> usize {
        self.buf.len() - self.pos
    }
    pub fn is_empty(&self) -> bool {
        self.pos >= self.buf.len()
    }
    fn take(&mut self, n: usize) -> Result<&'a [u8], DecodeError> {
        if self.remaining() < n {
            return Err(DecodeError::UnexpectedEof);
        }
        let s = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        Ok(s)
    }
}

// ---- primitive helpers ----

pub fn put_u8(out: &mut Vec<u8>, v: u8) {
    out.push(v);
}
pub fn put_u16(out: &mut Vec<u8>, v: u16) {
    out.extend_from_slice(&v.to_le_bytes());
}
pub fn put_u32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_le_bytes());
}
pub fn put_u64(out: &mut Vec<u8>, v: u64) {
    out.extend_from_slice(&v.to_le_bytes());
}
pub fn put_i32(out: &mut Vec<u8>, v: i32) {
    out.extend_from_slice(&v.to_le_bytes());
}
pub fn put_f32(out: &mut Vec<u8>, v: f32) {
    out.extend_from_slice(&v.to_le_bytes());
}
pub fn put_f32_array(out: &mut Vec<u8>, vs: &[f32]) {
    put_u32(
        out,
        u32::try_from(vs.len()).expect("array length exceeds u32"),
    );
    for v in vs {
        put_f32(out, *v);
    }
}

pub fn get_u8(c: &mut Cursor<'_>) -> Result<u8, DecodeError> {
    Ok(c.take(1)?[0])
}
pub fn get_u16(c: &mut Cursor<'_>) -> Result<u16, DecodeError> {
    let s = c.take(2)?;
    Ok(u16::from_le_bytes([s[0], s[1]]))
}
pub fn get_u32(c: &mut Cursor<'_>) -> Result<u32, DecodeError> {
    let s = c.take(4)?;
    Ok(u32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}
pub fn get_u64(c: &mut Cursor<'_>) -> Result<u64, DecodeError> {
    let s = c.take(8)?;
    Ok(u64::from_le_bytes([
        s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7],
    ]))
}
pub fn get_i32(c: &mut Cursor<'_>) -> Result<i32, DecodeError> {
    let s = c.take(4)?;
    Ok(i32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}
pub fn get_f32(c: &mut Cursor<'_>) -> Result<f32, DecodeError> {
    let s = c.take(4)?;
    Ok(f32::from_le_bytes([s[0], s[1], s[2], s[3]]))
}

/// Read a `u32_le`-length-prefixed `f32` array. Validates that the claimed
/// length fits the remaining buffer before allocating, so a hostile peer
/// can't induce a multi-GB allocation.
pub fn get_f32_array(c: &mut Cursor<'_>) -> Result<Vec<f32>, DecodeError> {
    let n = get_u32(c)?;
    let n_usize = n as usize;
    let needed = n_usize
        .checked_mul(4)
        .ok_or(DecodeError::ArrayLengthExceedsBuffer {
            claimed: n,
            available: c.remaining(),
        })?;
    if needed > c.remaining() {
        return Err(DecodeError::ArrayLengthExceedsBuffer {
            claimed: n,
            available: c.remaining(),
        });
    }
    let mut v = Vec::with_capacity(n_usize);
    for _ in 0..n_usize {
        v.push(get_f32(c)?);
    }
    Ok(v)
}
