//! Abstract transport layer. Hides whether the underlying wire I/O is the
//! Step-6 [`crate::host_io::KalicoHostIo`] (real serial port) or a test
//! harness (`MockTransport`, lives in `tests/`).
//!
//! Step-6 Phase-10 modules consume `&mut dyn Transport` (or `T: Transport`
//! generics) so they can be unit-tested against `MockTransport` and run in
//! production against the minimal shim. Production-grade hardening
//! (Step-7 MVP) replaces the shim's body but keeps the trait shape.

use std::collections::HashMap;
use std::io;
use std::time::Duration;

/// Errors surfaced by every [`Transport`] method.
#[derive(Debug)]
pub enum TransportError {
    /// Underlying I/O failure (port closed, permission denied, ...).
    Io(io::Error),
    /// `wait_for_response` exceeded the caller's timeout.
    Timeout,
    /// The transport has been closed by the peer or by `disconnect`.
    Closed,
    /// Wire-format / parser error (malformed frame, unknown msg-id, ...).
    Parse(String),
}

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransportError::Io(e) => write!(f, "transport I/O error: {e}"),
            TransportError::Timeout => write!(f, "transport timed out"),
            TransportError::Closed => write!(f, "transport closed"),
            TransportError::Parse(s) => write!(f, "transport parse error: {s}"),
        }
    }
}

impl std::error::Error for TransportError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            TransportError::Io(e) => Some(e),
            _ => None,
        }
    }
}

impl From<io::Error> for TransportError {
    fn from(e: io::Error) -> Self {
        TransportError::Io(e)
    }
}

/// Wire-protocol transport. Send a Klipper-msgproto-style command line,
/// then block (up to `timeout`) on a named response.
///
/// The trait is `Send` so producers can be hand-rolled across threads,
/// even though Step-6 only uses it from a single foreground thread.
pub trait Transport: Send {
    /// Send a command line in Klipper msgproto format
    /// (e.g. `"kalico_stream_arm t_start_t0_lo=0 ..."`).
    fn send(&mut self, cmd: &str) -> Result<(), TransportError>;

    /// Block until an inbound message named `name` arrives or `timeout`
    /// elapses. Returns the parsed key=value pairs.
    fn wait_for_response(
        &mut self,
        name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError>;

    /// Pull any inbound async events of `name` (non-blocking; returns the
    /// drained vector). The Step-6 shim's implementation is a synchronous
    /// drain of buffered packets; Step-7 MVP replaces this with an async
    /// dispatcher.
    fn poll_events(&mut self, name: &str) -> Vec<MessageParams>;
}

/// Parsed key=value pairs from a wire response. Field accessors return a
/// type-defaulted value (zero) when the key is absent or carries the
/// wrong scalar type — host-rt callers use this with known schemas, so
/// missing fields are programmer errors rather than runtime conditions.
#[derive(Debug, Default, Clone)]
pub struct MessageParams {
    pub fields: HashMap<String, MessageValue>,
}

impl MessageParams {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert<K: Into<String>>(&mut self, key: K, value: MessageValue) {
        self.fields.insert(key.into(), value);
    }

    pub fn get_i32(&self, k: &str) -> i32 {
        match self.fields.get(k) {
            Some(MessageValue::I32(v)) => *v,
            // Wire schema sometimes carries result codes as unsigned;
            // we accept either tag and reinterpret the bit pattern.
            #[allow(clippy::cast_possible_wrap)]
            Some(MessageValue::U32(v)) => *v as i32,
            _ => 0,
        }
    }

    pub fn get_u32(&self, k: &str) -> u32 {
        match self.fields.get(k) {
            Some(MessageValue::U32(v)) => *v,
            // See get_i32: tolerate either signed/unsigned tag.
            #[allow(clippy::cast_sign_loss)]
            Some(MessageValue::I32(v)) => *v as u32,
            _ => 0,
        }
    }

    pub fn get_u64(&self, k: &str) -> u64 {
        match self.fields.get(k) {
            Some(MessageValue::U64(v)) => *v,
            Some(MessageValue::U32(v)) => u64::from(*v),
            _ => 0,
        }
    }

    // --- Fallible accessors ------------------------------------------------
    //
    // I1 fix: load-bearing fields (`result`, etc.) cannot fall back to
    // `0` on a malformed response — `result == 0` means "success", so a
    // missing field would be silently treated as a successful push. The
    // `try_get_*` family returns `None` if the field is absent or
    // carries a wrong scalar type, letting the caller surface a
    // `Parse` transport error instead.
    pub fn try_get_i32(&self, k: &str) -> Option<i32> {
        match self.fields.get(k)? {
            MessageValue::I32(v) => Some(*v),
            #[allow(clippy::cast_possible_wrap)]
            MessageValue::U32(v) => Some(*v as i32),
            _ => None,
        }
    }

    pub fn try_get_u32(&self, k: &str) -> Option<u32> {
        match self.fields.get(k)? {
            MessageValue::U32(v) => Some(*v),
            #[allow(clippy::cast_sign_loss)]
            MessageValue::I32(v) => Some(*v as u32),
            _ => None,
        }
    }

    pub fn try_get_u64(&self, k: &str) -> Option<u64> {
        match self.fields.get(k)? {
            MessageValue::U64(v) => Some(*v),
            MessageValue::U32(v) => Some(u64::from(*v)),
            _ => None,
        }
    }

    pub fn get_bytes(&self, k: &str) -> Option<&[u8]> {
        match self.fields.get(k) {
            Some(MessageValue::Bytes(b)) => Some(b.as_slice()),
            _ => None,
        }
    }
}

/// Scalar variants the wire schema can carry. The Klipper VLQ encoding
/// distinguishes signed/unsigned but msgproto's parser already maps
/// fields to Python ints; we keep the tagged variant so Rust callers
/// don't have to `reinterpret_cast`.
#[derive(Debug, Clone)]
pub enum MessageValue {
    I32(i32),
    U32(u32),
    U64(u64),
    Bytes(Vec<u8>),
}
