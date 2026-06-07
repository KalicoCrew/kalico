use std::collections::HashMap;
use std::io;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, bounded, unbounded};
use thiserror::Error;

use crate::bootstrap::{
    BOOTSTRAP_IDENTIFY_RESPONSE_LEN, decode_identify_response, encode_identify,
};
use crate::connection::Connection;
use crate::demux::{Demuxer, Frame, StreamError};
use crate::frame::{CHANNEL_CONTROL, CHANNEL_EVENTS, encode_frame};
use crate::wire_helpers::{MESSAGE_VERSION_DEFAULT, decode_message_header, encode_message_header};
use kalico_protocol::{MessageKind, PROTO_VERSION};

#[derive(Debug, Error)]
pub enum TransportError {
    #[error("transport not in Identified state ({0:?})")]
    NotIdentified(ConnectionStateTag),
    #[error("MCU reset_epoch changed mid-call; in-flight calls invalidated")]
    Reset,
    #[error("schema_hash mismatch — host build {host_hex}, MCU build {mcu_hex}")]
    SchemaHashMismatch { host_hex: String, mcu_hex: String },
    #[error("proto_version mismatch — host 0x{host:02x}, MCU 0x{mcu:02x}")]
    ProtoVersionMismatch { host: u8, mcu: u8 },
    #[error("call timed out after {0:?}")]
    Timeout(Duration),
    #[error("bootstrap response malformed")]
    MalformedBootstrap,
    #[error("io error: {0}")]
    Io(#[from] io::Error),
    #[error("transport faulted: {0}")]
    Faulted(String),
}

#[derive(Debug, Clone)]
pub enum ConnectionState {
    Disconnected,
    Unidentified,
    Identified { reset_epoch: u32 },
    Faulted(String),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnectionStateTag {
    Disconnected,
    Unidentified,
    Identified,
    Faulted,
}

impl ConnectionState {
    pub fn tag(&self) -> ConnectionStateTag {
        match self {
            Self::Disconnected => ConnectionStateTag::Disconnected,
            Self::Unidentified => ConnectionStateTag::Unidentified,
            Self::Identified { .. } => ConnectionStateTag::Identified,
            Self::Faulted(_) => ConnectionStateTag::Faulted,
        }
    }
}

#[derive(Debug, Clone)]
pub struct EventMessage {
    pub kind: MessageKind,
    pub body: Vec<u8>,
}

#[derive(Debug, Clone)]
pub enum EpochChange {
    Established { reset_epoch: u32 },
    /// MCU rebooted; old epoch -> new epoch. Bridge must invalidate slot pool.
    Changed { old: u32, new: u32 },
    Faulted(String),
}

pub trait Transport: Send + Sync {
    fn call(
        &self,
        msg_type: MessageKind,
        body: &[u8],
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError>;

    fn send_event(&self, msg_type: MessageKind, body: &[u8]) -> Result<(), TransportError>;

    fn subscribe_events(&self) -> Receiver<EventMessage>;
}

struct PendingCall {
    notify: Sender<CallOutcome>,
}

#[derive(Debug)]
enum CallOutcome {
    Response { kind: MessageKind, body: Vec<u8> },
    Reset,
}

struct Inner<C: Connection> {
    conn: Mutex<C>,
    state: Mutex<ConnectionState>,
    pending: Mutex<HashMap<u32, PendingCall>>,
    next_correlation_id: AtomicU32,
    events_tx: Sender<EventMessage>,
    events_rx: Receiver<EventMessage>,
    epoch_tx: Sender<EpochChange>,
    epoch_rx: Receiver<EpochChange>,
    expected_schema_hash: [u8; 32],
    expected_proto_version: u8,
    demuxer: Mutex<Demuxer>,
}

pub struct KalicoNativeTransport<C: Connection> {
    inner: Arc<Inner<C>>,
}

impl<C: Connection> std::fmt::Debug for KalicoNativeTransport<C> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KalicoNativeTransport")
            .field("state", &*self.inner.state.lock().unwrap())
            .finish()
    }
}

impl<C: Connection + 'static> KalicoNativeTransport<C> {
    pub fn new(conn: C) -> Self {
        Self::with_schema_hash(conn, kalico_protocol::SCHEMA_HASH, PROTO_VERSION)
    }

    pub fn with_schema_hash(conn: C, schema_hash: [u8; 32], proto_version: u8) -> Self {
        let (events_tx, events_rx) = unbounded();
        let (epoch_tx, epoch_rx) = unbounded();
        Self {
            inner: Arc::new(Inner {
                conn: Mutex::new(conn),
                state: Mutex::new(ConnectionState::Disconnected),
                pending: Mutex::new(HashMap::new()),
                next_correlation_id: AtomicU32::new(1),
                events_tx,
                events_rx,
                epoch_tx,
                epoch_rx,
                expected_schema_hash: schema_hash,
                expected_proto_version: proto_version,
                demuxer: Mutex::new(Demuxer::new()),
            }),
        }
    }

    pub fn state(&self) -> ConnectionState {
        self.inner.state.lock().unwrap().clone()
    }

    pub fn epoch_change_subscribe(&self) -> Receiver<EpochChange> {
        self.inner.epoch_rx.clone()
    }

    pub fn identify(&self, timeout: Duration) -> Result<u32, TransportError> {
        *self.inner.state.lock().unwrap() = ConnectionState::Unidentified;

        let cid = self
            .inner
            .next_correlation_id
            .fetch_add(1, Ordering::SeqCst);
        let payload = encode_identify(cid, self.inner.expected_proto_version);
        let frame = encode_frame(CHANNEL_CONTROL, &payload);
        self.inner.conn.lock().unwrap().write_all(&frame)?;

        let deadline = Instant::now() + timeout;
        loop {
            self.pump_rx_once()?;
            match &*self.inner.state.lock().unwrap() {
                ConnectionState::Identified { reset_epoch } => return Ok(*reset_epoch),
                ConnectionState::Faulted(s) => {
                    return Err(TransportError::Faulted(s.clone()));
                }
                _ => {}
            }
            if Instant::now() >= deadline {
                return Err(TransportError::Timeout(timeout));
            }
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    pub fn pump_rx_once(&self) -> Result<(), TransportError> {
        let mut buf = [0u8; 4096];
        let n = self.inner.conn.lock().unwrap().read(&mut buf)?;
        if n == 0 {
            return Ok(());
        }
        let (frames, errors) = self.inner.demuxer.lock().unwrap().feed_slice(&buf[..n]);
        for e in errors {
            self.dispatch_error(e);
        }
        for f in frames {
            self.dispatch_frame(f);
        }
        Ok(())
    }

    fn dispatch_error(&self, e: StreamError) {
        log::warn!("kalico stream error: {e}");
    }

    fn dispatch_frame(&self, f: Frame) {
        match f {
            Frame::Klipper(_) => {}
            Frame::Kalico { channel, payload } => {
                self.dispatch_kalico(channel, &payload);
            }
        }
    }

    fn dispatch_kalico(&self, channel: u8, payload: &[u8]) {
        let Some((header, body)) = decode_message_header(payload) else {
            log::warn!("kalico frame too short for header");
            return;
        };
        let kind_raw = header.kind_raw;
        if kind_raw == MessageKind::IdentifyResponse as u16 {
            self.handle_identify_response(payload);
            return;
        }
        if !matches!(
            *self.inner.state.lock().unwrap(),
            ConnectionState::Identified { .. }
        ) {
            log::trace!("dropping kalico frame in non-Identified state, kind 0x{kind_raw:04x}");
            return;
        }
        let Some(kind) = MessageKind::from_u16(kind_raw) else {
            log::warn!("unknown kalico message kind 0x{kind_raw:04x}");
            return;
        };

        if channel == CHANNEL_EVENTS || kind.is_event() {
            let _ = self.inner.events_tx.send(EventMessage {
                kind,
                body: body.to_vec(),
            });
            return;
        }

        if header.correlation_id == 0 {
            log::warn!("control-channel response with correlation_id=0 (kind 0x{kind_raw:04x})");
            return;
        }
        let mut pending = self.inner.pending.lock().unwrap();
        if let Some(p) = pending.remove(&header.correlation_id) {
            let _ = p.notify.send(CallOutcome::Response {
                kind,
                body: body.to_vec(),
            });
        } else {
            log::warn!(
                "no pending call for correlation_id {} (kind 0x{:04x})",
                header.correlation_id,
                kind_raw
            );
        }
    }

    fn handle_identify_response(&self, payload: &[u8]) {
        if payload.len() != BOOTSTRAP_IDENTIFY_RESPONSE_LEN {
            self.fault(format!(
                "IdentifyResponse wrong length: got {}, expected {}",
                payload.len(),
                BOOTSTRAP_IDENTIFY_RESPONSE_LEN
            ));
            return;
        }
        let Some((_cid, resp)) = decode_identify_response(payload) else {
            self.fault("IdentifyResponse failed to decode".to_string());
            return;
        };
        if resp.proto_version != self.inner.expected_proto_version {
            self.fault(format!(
                "proto_version mismatch — host 0x{:02x}, MCU 0x{:02x}",
                self.inner.expected_proto_version, resp.proto_version
            ));
            return;
        }
        if resp.schema_hash != self.inner.expected_schema_hash {
            let host_hex = hex(&self.inner.expected_schema_hash);
            let mcu_hex = hex(&resp.schema_hash);
            self.fault(format!(
                "schema_hash mismatch — host build {host_hex}, MCU build {mcu_hex}"
            ));
            return;
        }
        let new_epoch = resp.reset_epoch;
        let prior = std::mem::replace(
            &mut *self.inner.state.lock().unwrap(),
            ConnectionState::Identified {
                reset_epoch: new_epoch,
            },
        );
        let evt = match prior {
            ConnectionState::Identified { reset_epoch: old } if old != new_epoch => {
                EpochChange::Changed {
                    old,
                    new: new_epoch,
                }
            }
            _ => EpochChange::Established {
                reset_epoch: new_epoch,
            },
        };
        let _ = self.inner.epoch_tx.send(evt);
    }

    fn fault(&self, msg: String) {
        log::error!("kalico transport faulted: {msg}");
        *self.inner.state.lock().unwrap() = ConnectionState::Faulted(msg.clone());
        let drained: Vec<PendingCall> = {
            let mut p = self.inner.pending.lock().unwrap();
            p.drain().map(|(_, v)| v).collect()
        };
        for p in drained {
            let _ = p.notify.send(CallOutcome::Reset);
        }
        let _ = self.inner.epoch_tx.send(EpochChange::Faulted(msg));
    }
}

fn hex(bytes: &[u8]) -> String {
    let mut s = String::with_capacity(bytes.len() * 2);
    for b in bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

impl<C: Connection + 'static> Transport for KalicoNativeTransport<C> {
    fn call(
        &self,
        msg_type: MessageKind,
        body: &[u8],
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError> {
        let tag = self.inner.state.lock().unwrap().tag();
        if tag != ConnectionStateTag::Identified {
            return Err(TransportError::NotIdentified(tag));
        }
        let cid = self
            .inner
            .next_correlation_id
            .fetch_add(1, Ordering::SeqCst);
        let (tx, rx) = bounded::<CallOutcome>(1);
        self.inner
            .pending
            .lock()
            .unwrap()
            .insert(cid, PendingCall { notify: tx });

        let mut payload = Vec::with_capacity(7 + body.len());
        payload.extend_from_slice(&encode_message_header(
            msg_type,
            MESSAGE_VERSION_DEFAULT,
            cid,
        ));
        payload.extend_from_slice(body);
        let frame = encode_frame(CHANNEL_CONTROL, &payload);
        self.inner.conn.lock().unwrap().write_all(&frame)?;

        let deadline = Instant::now() + timeout;
        loop {
            self.pump_rx_once()?;
            match rx.recv_timeout(Duration::from_millis(1)) {
                Ok(CallOutcome::Response { kind, body }) => return Ok((kind, body)),
                Ok(CallOutcome::Reset) => return Err(TransportError::Reset),
                Err(crossbeam_channel::RecvTimeoutError::Timeout) => {
                    if Instant::now() >= deadline {
                        self.inner.pending.lock().unwrap().remove(&cid);
                        return Err(TransportError::Timeout(timeout));
                    }
                }
                Err(crossbeam_channel::RecvTimeoutError::Disconnected) => {
                    return Err(TransportError::Faulted(
                        "pending-call channel disconnected".to_string(),
                    ));
                }
            }
            if !matches!(
                *self.inner.state.lock().unwrap(),
                ConnectionState::Identified { .. }
            ) {
                self.inner.pending.lock().unwrap().remove(&cid);
                return Err(TransportError::Reset);
            }
        }
    }

    fn send_event(&self, msg_type: MessageKind, body: &[u8]) -> Result<(), TransportError> {
        let tag = self.inner.state.lock().unwrap().tag();
        if tag != ConnectionStateTag::Identified {
            return Err(TransportError::NotIdentified(tag));
        }
        let mut payload = Vec::with_capacity(7 + body.len());
        payload.extend_from_slice(&encode_message_header(msg_type, MESSAGE_VERSION_DEFAULT, 0));
        payload.extend_from_slice(body);
        let frame = encode_frame(CHANNEL_EVENTS, &payload);
        self.inner.conn.lock().unwrap().write_all(&frame)?;
        Ok(())
    }

    fn subscribe_events(&self) -> Receiver<EventMessage> {
        self.inner.events_rx.clone()
    }
}
