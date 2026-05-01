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
    /// Dispatcher entry passed its deadline before being serviced.
    DispatcherTimeout,
    /// Submission queue full; command rejected without being sent.
    Backpressure,
}

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransportError::Io(e) => write!(f, "transport I/O error: {e}"),
            TransportError::Timeout => write!(f, "transport timed out"),
            TransportError::Closed => write!(f, "transport closed"),
            TransportError::Parse(s) => write!(f, "transport parse error: {s}"),
            TransportError::DispatcherTimeout => write!(f, "dispatcher timeout (entry past deadline)"),
            TransportError::Backpressure => write!(f, "transport backpressure: submission queue full"),
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

#[derive(Debug)]
pub enum SubscribeError {
    AlreadySubscribed { channel: &'static str },
    Closed,
}

impl std::fmt::Display for SubscribeError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SubscribeError::AlreadySubscribed { channel } =>
                write!(f, "channel '{channel}' already has a subscriber"),
            SubscribeError::Closed => write!(f, "transport closed"),
        }
    }
}

impl std::error::Error for SubscribeError {}

/// Wire-protocol transport. Issues a Klipper-msgproto-style command,
/// waits for the named response, and returns the parsed fields.
///
/// The trait is `Send + Sync` so it can be shared across threads (e.g.
/// an `Arc<T>` used from multiple producer threads). Implementations
/// use internal synchronization (`Mutex` / channels) to satisfy `&self`.
pub trait Transport: Send + Sync {
    /// Send `cmd` (Klipper msgproto format) and block until a message
    /// named `expected_response_name` arrives or `timeout` elapses.
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError>;

    /// Typed variant: encodes `name` + `args` via the loaded data
    /// dictionary and waits for `expected_response_name`.
    fn call_typed(
        &self,
        name: &str,
        args: &[(&str, crate::host_io::parser::FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError>;
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

    pub fn try_get_str(&self, k: &str) -> Option<&str> {
        match self.fields.get(k)? {
            MessageValue::String(s) => Some(s.as_str()),
            MessageValue::Bytes(b)  => std::str::from_utf8(b).ok(),
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
    /// Carries %s text fields AND resolved enum names
    /// (format!("?{i}") for unknown enum ints). Per spec §4.11.
    String(String),
}

#[cfg(test)]
mod try_get_str_tests {
    use super::*;

    #[test]
    fn returns_string_directly() {
        let mut p = MessageParams::new();
        p.insert("name", MessageValue::String("PA0".into()));
        assert_eq!(p.try_get_str("name"), Some("PA0"));
    }

    #[test]
    fn falls_back_to_utf8_bytes() {
        let mut p = MessageParams::new();
        p.insert("data", MessageValue::Bytes(b"hello".to_vec()));
        assert_eq!(p.try_get_str("data"), Some("hello"));
    }

    #[test]
    fn returns_none_for_int_field() {
        let mut p = MessageParams::new();
        p.insert("count", MessageValue::U32(42));
        assert_eq!(p.try_get_str("count"), None);
    }
}
