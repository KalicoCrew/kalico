//! Kalico-native transport state for [`Reactor`] (option (b) of the
//! Phase C-B integration choice — see
//! `docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md` §17 Q1).
//!
//! `KalicoHostIo`'s reactor owns the OS handle and runs a single demuxer
//! over each incoming byte. Klipper-shaped frames (length byte 5..=64)
//! continue down the legacy parser; kalico-shaped frames (sync byte 0x55)
//! land here, where this module:
//!
//! * routes control-channel responses to pending kalico calls keyed by
//!   `correlation_id`;
//! * surfaces `IdentifyResponse` (bootstrap ABI per spec §5) to the
//!   identify caller, validating `proto_version` and `schema_hash`;
//! * lifts `FaultEvent` / `StatusHeartbeat` to [`RuntimeEvent`] variants
//!   so motion-bridge's existing event plumbing keeps working unchanged;
//!
//! The Phase A `KalicoNativeTransport<C: Connection>` is retained for
//! the `sim_handshake` example and unit tests; production traffic flows
//! through this inline integration.

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

/// Outcome surfaced to the bridge for a kalico control-channel call.
#[derive(Debug)]
pub enum KalicoCallOutcome {
    Response { kind: MessageKind, body: Vec<u8> },
    Reset,
}

/// Outcome surfaced to the bridge for an identify handshake.
#[derive(Debug)]
pub struct IdentifyOutcome {
    pub reset_epoch: u32,
    /// Raw `capabilities` bitmap from the `IdentifyResponse` (spec §5,
    /// bytes 61..69). Bit 0 = `PHASE_STEPPING_CAPABLE`. 0 when the
    /// firmware predates the capability field (shouldn't happen with our
    /// schema-hash check, but kept defensive).
    pub capabilities: u64,
}

/// Per-call wait state held by the reactor. The bridge's `kalico_call`
/// allocates the bounded `sync_channel`, hands the sender here, and blocks
/// on the receiver.
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

/// Reactor-side state machine for kalico-native traffic.
#[derive(Debug)]
pub struct KalicoNativeState {
    /// In-flight calls keyed by `correlation_id`.
    pub pending: HashMap<u32, PendingKalicoCall>,
    /// Monotonic `correlation_id` allocator. 0 is reserved (events).
    pub next_correlation_id: u32,
    /// Pending identify handshake completion.
    pub identify_pending: Option<SyncSender<Result<IdentifyOutcome, TransportError>>>,
    /// First-observed `reset_epoch` from `IdentifyResponse` / `StatusEvent`.
    pub reset_epoch: Option<u32>,
    /// True after a successful identify handshake — motion dispatch is
    /// gated on this on the bridge side.
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
        // Wrap-around: cid 0 is reserved for events (§7.2). On overflow skip
        // 0 and resume at 1.
        let next = cid.wrapping_add(1);
        self.next_correlation_id = if next == 0 { 1 } else { next };
        cid
    }
}

/// Build a kalico frame on an explicit channel: per-message header + body,
/// wrapped in the Layer-1 frame envelope. For control-channel calls use
/// `CHANNEL_CONTROL` (0x00); for pieces use
/// [`kalico_protocol::KALICO_CHANNEL_PIECES`] (0x02).
/// Responses always arrive on the control channel keyed by `correlation_id`.
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

/// Build a control-channel frame: per-message header + body, wrapped in
/// the Layer-1 frame envelope.
pub fn build_kalico_control_frame(kind: MessageKind, correlation_id: u32, body: &[u8]) -> Vec<u8> {
    build_kalico_frame(CHANNEL_CONTROL, kind, correlation_id, body)
}

/// Build an Identify (bootstrap-ABI) frame.
pub fn build_kalico_identify_frame(correlation_id: u32) -> Vec<u8> {
    let payload = encode_identify(correlation_id, PROTO_VERSION);
    encode_frame(CHANNEL_CONTROL, &payload)
}

/// Result of dispatching a single complete kalico frame at the reactor's
/// byte-stream level.
#[derive(Debug)]
pub enum KalicoDispatchResult {
    /// The frame was routed to a pending call, an identify, or as an event
    /// already lifted by the dispatcher.
    Handled,
    /// The frame surfaced a `RuntimeEvent` that the reactor must forward
    /// to its event dispatcher (the dispatcher needs `&mut self`, so we
    /// return the event here rather than borrow it inline).
    Event(RuntimeEvent),
    /// Ignored / decode-error / stale state.
    Ignored,
}

/// Dispatch a complete kalico frame's payload (post Layer-1 strip) to the
/// reactor-side state. `channel` is the Layer-1 channel byte (0=control,
/// 1=events).
pub fn dispatch_kalico_frame(
    state: &mut KalicoNativeState,
    channel: u8,
    payload: &[u8],
) -> KalicoDispatchResult {
    let Some((header, body)) = decode_message_header(payload) else {
        log::warn!("kalico frame too short for per-message header");
        return KalicoDispatchResult::Ignored;
    };

    // Bootstrap: IdentifyResponse handled out-of-schema.
    if header.kind_raw == MessageKind::IdentifyResponse as u16 {
        return handle_identify_response(state, payload);
    }

    let Some(kind) = MessageKind::from_u16(header.kind_raw) else {
        log::warn!("unknown kalico message kind 0x{:04x}", header.kind_raw);
        return KalicoDispatchResult::Ignored;
    };

    // Events on the events channel (or by tag) — lift to RuntimeEvent.
    if channel == kalico_native_transport::CHANNEL_EVENTS || kind.is_event() {
        return lift_event_to_runtime_event(state, kind, body);
    }

    // Control-channel response — route by correlation_id.
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
    let _ = state; // no per-event state mutations required for surviving events
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
            Ok(hb) => {
                // engine_state / fault_code intentionally dropped — the pump needs only retired_counts.
                KalicoDispatchResult::Event(RuntimeEvent::Heartbeat {
                    retired_counts: hb.retired_counts,
                })
            }
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
mod tests {
    use super::*;
    use kalico_protocol::{Encode, StatusHeartbeat};

    fn make_state() -> KalicoNativeState {
        KalicoNativeState::default()
    }

    #[test]
    fn status_heartbeat_lifts_to_runtime_event() {
        let hb = StatusHeartbeat {
            engine_state: 1,
            fault_code: 0,
            retired_counts: vec![7, 0, 3],
        };
        let mut body = Vec::new();
        hb.encode(&mut body);
        let mut st = make_state();
        match lift_event_to_runtime_event(&mut st, MessageKind::StatusHeartbeat, &body) {
            KalicoDispatchResult::Event(RuntimeEvent::Heartbeat { retired_counts }) => {
                assert_eq!(retired_counts, vec![7, 0, 3]);
            }
            other => panic!("expected Heartbeat event, got {other:?}"),
        }
    }
}
