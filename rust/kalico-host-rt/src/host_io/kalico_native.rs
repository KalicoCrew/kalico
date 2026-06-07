use std::collections::HashMap;
use std::sync::mpsc::SyncSender;
use std::time::Instant;

use kalico_native_transport::wire_helpers::{
    MESSAGE_VERSION_DEFAULT, decode_message_header, encode_message_header,
};
use kalico_native_transport::{
    BOOTSTRAP_IDENTIFY_RESPONSE_LEN, CHANNEL_CONTROL, decode_identify_response, encode_frame,
    encode_identify,
};
use kalico_protocol::{
    Decode, FaultEvent as KFaultEvent, McuLog as KMcuLog, MessageKind, PROTO_VERSION, SCHEMA_HASH,
    StatusHeartbeat as KStatusHeartbeat,
};

use crate::host_io::runtime_events::{FaultEvent, McuLogEvent, RuntimeEvent};
use crate::transport::TransportError;

#[derive(Debug)]
pub enum KalicoCallOutcome {
    Response { kind: MessageKind, body: Vec<u8> },
    Reset,
}

#[derive(Debug)]
pub struct IdentifyOutcome {
    pub reset_epoch: u32,
    pub capabilities: u64,
}

pub struct PendingKalicoCall {
    pub completion: SyncSender<Result<KalicoCallOutcome, TransportError>>,
    pub deadline: Instant,
}

impl std::fmt::Debug for PendingKalicoCall {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("PendingKalicoCall")
            .field("completion", &"<SyncSender>")
            .field("deadline", &self.deadline)
            .finish()
    }
}

#[derive(Debug)]
pub struct KalicoNativeState {
    pub pending: HashMap<u32, PendingKalicoCall>,
    pub next_correlation_id: u32,
    pub identify_pending: Option<SyncSender<Result<IdentifyOutcome, TransportError>>>,
    pub reset_epoch: Option<u32>,
    pub identified: bool,
}

impl Default for KalicoNativeState {
    fn default() -> Self {
        Self {
            pending: HashMap::new(),
            next_correlation_id: 1,
            identify_pending: None,
            reset_epoch: None,
            identified: false,
        }
    }
}

impl KalicoNativeState {
    pub fn allocate_correlation_id(&mut self) -> u32 {
        let cid = self.next_correlation_id;
        let next = cid.wrapping_add(1);
        self.next_correlation_id = if next == 0 { 1 } else { next };
        cid
    }
}

pub fn build_kalico_frame(
    channel: u8,
    kind: MessageKind,
    correlation_id: u32,
    body: &[u8],
) -> Vec<u8> {
    let mut payload = Vec::with_capacity(7 + body.len());
    payload.extend_from_slice(&encode_message_header(
        kind,
        MESSAGE_VERSION_DEFAULT,
        correlation_id,
    ));
    payload.extend_from_slice(body);
    encode_frame(channel, &payload)
}

pub fn build_kalico_control_frame(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    build_kalico_frame(CHANNEL_CONTROL, kind, correlation_id, body)
}

pub fn build_kalico_identify_frame(correlation_id: u32) -> Vec<u8> {
    let payload = encode_identify(correlation_id, PROTO_VERSION);
    encode_frame(CHANNEL_CONTROL, &payload)
}

#[derive(Debug)]
pub enum KalicoDispatchResult {
    Handled,
    Event(RuntimeEvent),
    Ignored,
}

pub fn dispatch_kalico_frame(
    state: &mut KalicoNativeState,
    channel: u8,
    payload: &[u8],
) -> KalicoDispatchResult {
    let Some((header, body)) = decode_message_header(payload) else {
        log::warn!("kalico frame too short for per-message header");
        return KalicoDispatchResult::Ignored;
    };

    if header.kind_raw == MessageKind::IdentifyResponse as u16 {
        return handle_identify_response(state, payload);
    }

    let Some(kind) = MessageKind::from_u16(header.kind_raw) else {
        log::warn!("unknown kalico message kind 0x{:04x}", header.kind_raw);
        return KalicoDispatchResult::Ignored;
    };

    if channel == kalico_native_transport::CHANNEL_EVENTS || kind.is_event() {
        return lift_event_to_runtime_event(state, kind, body);
    }

    if header.correlation_id == 0 {
        log::warn!(
            "kalico control-channel frame with correlation_id=0 (kind 0x{:04x})",
            header.kind_raw
        );
        return KalicoDispatchResult::Ignored;
    }
    if let Some(p) = state.pending.remove(&header.correlation_id) {
        let _ = p.completion.send(Ok(KalicoCallOutcome::Response {
            kind,
            body: body.to_vec(),
        }));
        KalicoDispatchResult::Handled
    } else {
        log::warn!(
            "no pending kalico call for correlation_id {} (kind 0x{:04x})",
            header.correlation_id,
            header.kind_raw
        );
        KalicoDispatchResult::Ignored
    }
}

fn handle_identify_response(state: &mut KalicoNativeState, payload: &[u8]) -> KalicoDispatchResult {
    if payload.len() != BOOTSTRAP_IDENTIFY_RESPONSE_LEN {
        log::error!(
            "kalico IdentifyResponse wrong length: got {}, expected {}",
            payload.len(),
            BOOTSTRAP_IDENTIFY_RESPONSE_LEN
        );
        if let Some(c) = state.identify_pending.take() {
            let _ = c.send(Err(TransportError::Parse(format!(
                "IdentifyResponse wrong length: got {}, expected {}",
                payload.len(),
                BOOTSTRAP_IDENTIFY_RESPONSE_LEN
            ))));
        }
        return KalicoDispatchResult::Ignored;
    }
    let Some((_cid, resp)) = decode_identify_response(payload) else {
        log::error!("kalico IdentifyResponse failed to decode");
        if let Some(c) = state.identify_pending.take() {
            let _ = c.send(Err(TransportError::Parse(
                "IdentifyResponse failed to decode".into(),
            )));
        }
        return KalicoDispatchResult::Ignored;
    };

    if resp.proto_version != PROTO_VERSION {
        let msg = format!(
            "kalico proto_version mismatch — host 0x{:02x}, MCU 0x{:02x}",
            PROTO_VERSION, resp.proto_version
        );
        log::error!("{msg}");
        if let Some(c) = state.identify_pending.take() {
            let _ = c.send(Err(TransportError::Parse(msg)));
        }
        return KalicoDispatchResult::Ignored;
    }
    if resp.schema_hash != SCHEMA_HASH {
        let host_hex = hex32(&SCHEMA_HASH);
        let mcu_hex = hex32(&resp.schema_hash);
        let msg = format!("kalico schema_hash mismatch — host {host_hex}, MCU {mcu_hex}");
        log::error!("{msg}");
        if let Some(c) = state.identify_pending.take() {
            let _ = c.send(Err(TransportError::Parse(msg)));
        }
        return KalicoDispatchResult::Ignored;
    }

    state.reset_epoch = Some(resp.reset_epoch);
    state.identified = true;
    if let Some(c) = state.identify_pending.take() {
        let _ = c.send(Ok(IdentifyOutcome {
            reset_epoch: resp.reset_epoch,
            capabilities: resp.capabilities,
        }));
    }
    tracing::info!(
        subsystem = "bridge",
        event = "identify_complete",
        reset_epoch = resp.reset_epoch,
        capabilities = resp.capabilities,
        state_epoch = ?state.reset_epoch,
        "kalico identify complete"
    );
    log::info!(
        "kalico identified: reset_epoch=0x{:08x}, caps=0x{:016x}, schema_hash matches",
        resp.reset_epoch,
        resp.capabilities,
    );
    KalicoDispatchResult::Handled
}

fn lift_event_to_runtime_event(
    state: &mut KalicoNativeState,
    kind: MessageKind,
    body: &[u8],
) -> KalicoDispatchResult {
    let _ = state;
    match kind {
        MessageKind::FaultEvent => match KFaultEvent::decode(body) {
            Ok(f) => KalicoDispatchResult::Event(RuntimeEvent::Fault(FaultEvent {
                fault_code: f.fault_code,
                fault_detail: f.fault_detail,
                segment_id: f.segment_id,
                synthesized: false,
            })),
            Err(e) => {
                log::warn!("kalico FaultEvent decode failed: {e:?}");
                KalicoDispatchResult::Ignored
            }
        },
        MessageKind::StatusHeartbeat => match KStatusHeartbeat::decode(body) {
            Ok(hb) => KalicoDispatchResult::Event(RuntimeEvent::Heartbeat {
                retired_counts: hb.retired_counts,
            }),
            Err(e) => {
                log::warn!("kalico StatusHeartbeat decode failed: {e:?}");
                KalicoDispatchResult::Ignored
            }
        },
        MessageKind::McuLog => match KMcuLog::decode(body) {
            Ok(msg) => KalicoDispatchResult::Event(RuntimeEvent::McuLog(McuLogEvent {
                mcu_tick: msg.mcu_tick,
                level: msg.level,
                subsystem: msg.subsystem,
                event: msg.event,
                code: msg.code,
                seq: msg.seq,
                args: msg.args,
                host_recv: Instant::now(),
            })),
            Err(e) => {
                log::warn!("kalico McuLog decode failed: {e:?}");
                KalicoDispatchResult::Ignored
            }
        },
        _ => {
            log::warn!("unexpected event kind on events channel: {kind:?}");
            KalicoDispatchResult::Ignored
        }
    }
}

fn hex32(bytes: &[u8; 32]) -> String {
    let mut s = String::with_capacity(64);
    for b in bytes {
        s.push_str(&format!("{b:02x}"));
    }
    s
}

#[cfg(test)]
mod tests;
