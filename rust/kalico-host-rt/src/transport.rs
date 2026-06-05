use std::collections::HashMap;
use std::io;
use std::time::Duration;

#[derive(Debug)]
pub enum TransportError {
    Io(io::Error),
    Timeout,
    Closed,
    Parse(String),
    DispatcherTimeout,
    Backpressure,
}

impl std::fmt::Display for TransportError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransportError::Io(e) => write!(f, "transport I/O error: {e}"),
            TransportError::Timeout => write!(f, "transport timed out"),
            TransportError::Closed => write!(f, "transport closed"),
            TransportError::Parse(s) => write!(f, "transport parse error: {s}"),
            TransportError::DispatcherTimeout => {
                write!(f, "dispatcher timeout (entry past deadline)")
            }
            TransportError::Backpressure => {
                write!(f, "transport backpressure (pending submission queue full)")
            }
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
            SubscribeError::AlreadySubscribed { channel } => {
                write!(f, "channel '{channel}' already has a subscriber")
            }
            SubscribeError::Closed => write!(f, "transport closed"),
        }
    }
}

impl std::error::Error for SubscribeError {}

pub trait Transport: Send + Sync {
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError>;

    fn call_typed(
        &self,
        name: &str,
        args: &[(&str, crate::host_io::parser::FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError>;

    fn send_typed(
        &self,
        name: &str,
        args: &[(&str, crate::host_io::parser::FieldValue<'_>)],
    ) -> Result<(), TransportError>;
}

#[derive(Debug, Default, Clone)]
pub struct MessageParams {
    pub fields: HashMap<String, MessageValue>,
    /// CLOCK_MONOTONIC_RAW seconds at the instant the request frame was written to
    /// the wire. Zero when not measured (e.g. non-bridge path or before first sample).
    pub sent_time_raw: f64,
    /// CLOCK_MONOTONIC_RAW seconds at the instant the matching response frame was
    /// received. Zero when not measured.
    pub recv_time_raw: f64,
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
            #[allow(clippy::cast_possible_wrap)]
            Some(MessageValue::U32(v)) => *v as i32,
            _ => 0,
        }
    }

    pub fn get_u32(&self, k: &str) -> u32 {
        match self.fields.get(k) {
            Some(MessageValue::U32(v)) => *v,
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
            MessageValue::Bytes(b) => std::str::from_utf8(b).ok(),
            _ => None,
        }
    }
}

#[derive(Debug, Clone)]
pub enum MessageValue {
    I32(i32),
    U32(u32),
    U64(u64),
    Bytes(Vec<u8>),
    String(String),
}

#[cfg(test)]
mod try_get_str_tests;
