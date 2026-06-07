//! Single-thread poll-reactor.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::mpsc::Receiver;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;

use crate::clock::{Clock, RealClock};
use crate::host_io::ReactorCommand;
use crate::host_io::events::EventDispatcher;
use crate::host_io::identify::IdentifySeqState;
use crate::host_io::kalico_native::{
    KalicoDispatchResult, KalicoNativeState, PendingKalicoCall, build_kalico_frame,
    build_kalico_identify_frame, dispatch_kalico_frame,
};
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::rtt::RttEstimator;
use crate::host_io::runtime_events::{FaultEvent, StatusEvent};
use crate::host_io::serial_frame_io::SerialFrameIo;
use crate::host_io::window::{AwaitingResponse, UnackedWindow};
use crate::passthrough_queue::{McuHandle, NotifyId, PassthroughRouter};
use crate::transport::TransportError;
use kalico_native_transport::demux::{Frame, KlipperFrame, PollOutcome};
use runtime::error::FaultCode;

pub struct Reactor {
    pub(crate) io: SerialFrameIo,
    pub(crate) parser: Arc<MsgProtoParser>,
    pub(crate) submission_rx: Receiver<ReactorCommand>,
    pub(crate) unacked_window: UnackedWindow,
    pub(crate) awaiting_response: AwaitingResponse,
    pub(crate) rtt: RttEstimator,
    pub(crate) status_snapshot: Arc<ArcSwap<StatusEvent>>,
    pub(crate) event_dispatcher: EventDispatcher,

    pub(crate) send_seq: u64,
    pub(crate) receive_seq: u64,
    pub(crate) last_ack_seq: u64,
    pub(crate) ignore_nak_seq: u64,
    pub(crate) retransmit_seq: u64,
    pub(crate) rtt_sample_seq: u64,
    pub(crate) rtt_sample_armed: bool,

    pub(crate) state: ReactorState,

    pub(crate) closed_via_shutdown: bool,

    pub(crate) pending_host_fault: Option<FaultEvent>,

    pub(crate) pending_submissions: VecDeque<PendingSubmission>,

    /// When `get_clock_async` is in flight: the CLOCK_MONOTONIC_RAW sent-time
    /// captured before the frame was written to wire.  The next unsolicited
    /// "clock" response matching this will be delivered as a PassthroughResponse
    /// with RAW RTT stamps rather than going through the generic path.
    pub(crate) pending_clock_sent_raw: Option<f64>,

    /// Queued fire-and-forget payloads; the bool marks a `get_clock` frame
    /// whose RAW send stamp is captured at the actual wire write.
    pub(crate) pending_fire_and_forget: VecDeque<(Vec<u8>, bool)>,
    pub(crate) pending_outbound_order: VecDeque<PendingOutboundKind>,
    pub(crate) zero_byte_first_seen: Option<Instant>,
    pub(crate) last_recv_time: Instant,
    pub(crate) last_write_time: Instant,
    pub(crate) zero_byte_consec: u32,
    pub(crate) clock: Arc<dyn Clock>,
    pub(crate) passthrough_router: Option<PassthroughRouter>,
    pub(crate) passthrough_notify_map: std::collections::HashMap<u64, (McuHandle, NotifyId)>,
    pub(crate) passthrough_mcu: Option<McuHandle>,
    pub(crate) kalico_state: KalicoNativeState,
    pub(crate) interceptors: crate::host_io::interceptor::InterceptorTable,
}

pub(crate) struct PendingSubmission {
    pub call_id: u64,
    pub payload: Vec<u8>,
    pub expected_response_name: String,
    pub completion:
        std::sync::mpsc::SyncSender<Result<crate::transport::MessageParams, TransportError>>,
    pub deadline: Instant,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum PendingOutboundKind {
    Submission,
    FireAndForget,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ReactorState {
    Active,
    Closed,
}

impl Reactor {
    pub fn new(
        io: SerialFrameIo,
        parser: Arc<MsgProtoParser>,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        seq: IdentifySeqState,
        config: crate::host_io::KalicoHostIoConfig,
    ) -> Self {
        Self::new_with_clock(
            io,
            parser,
            submission_rx,
            status_snapshot,
            seq,
            config,
            Arc::new(RealClock),
        )
    }

    pub fn new_with_clock(
        io: SerialFrameIo,
        parser: Arc<MsgProtoParser>,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        seq: IdentifySeqState,
        config: crate::host_io::KalicoHostIoConfig,
        clock: Arc<dyn Clock>,
    ) -> Self {
        let event_dispatcher = EventDispatcher::new(
            Arc::clone(&status_snapshot),
            config.trace_capacity,
            config.host_event_capacity,
        );
        Self {
            io,
            parser,
            submission_rx,
            unacked_window: UnackedWindow::default(),
            awaiting_response: AwaitingResponse::default(),
            rtt: RttEstimator::default(),
            status_snapshot,
            event_dispatcher,
            send_seq: seq.next_send_seq_abs,
            receive_seq: seq.mcu_receive_seq_abs,
            last_ack_seq: seq.mcu_receive_seq_abs.saturating_sub(1),
            ignore_nak_seq: 0,
            retransmit_seq: 0,
            rtt_sample_seq: 0,
            rtt_sample_armed: false,
            state: ReactorState::Active,
            closed_via_shutdown: false,
            pending_host_fault: None,
            pending_clock_sent_raw: None,
            pending_submissions: VecDeque::new(),
            pending_fire_and_forget: VecDeque::new(),
            pending_outbound_order: VecDeque::new(),
            zero_byte_first_seen: None,
            last_recv_time: clock.now(),
            last_write_time: clock.now(),
            zero_byte_consec: 0,
            clock,
            passthrough_router: None,
            passthrough_notify_map: std::collections::HashMap::new(),
            passthrough_mcu: None,
            kalico_state: KalicoNativeState::default(),
            interceptors: crate::host_io::interceptor::InterceptorTable::new(),
        }
    }

    #[cfg(any(test, feature = "test-harness"))]
    pub fn new_for_tests(
        port: Box<dyn serialport::SerialPort>,
        parser: Arc<MsgProtoParser>,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        config: crate::host_io::KalicoHostIoConfig,
        clock: Arc<dyn Clock>,
    ) -> Self {
        Self::new_with_clock(
            SerialFrameIo::new(port),
            parser,
            submission_rx,
            status_snapshot,
            IdentifySeqState {
                next_send_seq_abs: 1,
                mcu_receive_seq_abs: 1,
            },
            config,
            clock,
        )
    }

    pub(crate) fn write_frame(&mut self, frame: &[u8]) -> Result<(), TransportError> {
        use std::sync::atomic::{AtomicU64, Ordering};
        static WRITE_SEQ: AtomicU64 = AtomicU64::new(0);
        let seq = WRITE_SEQ.fetch_add(1, Ordering::Relaxed);
        let t0 = std::time::Instant::now();
        let proto = if !frame.is_empty() && frame[0] == 0x55 {
            "kalico"
        } else {
            "klipper"
        };
        let bytes = frame.len();
        let result: Result<(), TransportError> = (|| {
            self.io.write_all(frame)?;
            self.io.flush()?;
            Ok(())
        })();
        if result.is_ok() {
            self.last_write_time = std::time::Instant::now();
        }
        let dt = t0.elapsed();
        tracing::trace!(
            subsystem = "mcu-comms",
            event = "frame_write",
            tid = ?std::thread::current().id(),
            seq,
            proto,
            bytes,
            dt_ms = dt.as_secs_f64() * 1000.0,
            result = ?result.as_ref().map(|_| "OK"),
            first8 = ?&frame[..frame.len().min(8)],
            "frame write"
        );
        result
    }
}

impl Reactor {
    pub fn set_passthrough_router(&mut self, router: PassthroughRouter, mcu: McuHandle) {
        self.passthrough_router = Some(router);
        self.passthrough_mcu = Some(mcu);
    }
}

#[derive(Debug, Clone, Copy)]
pub enum RetransmitTrigger {
    NakDriven,
    TimeoutDriven,
}

const PENDING_SUBMISSION_CEILING: usize = 256;
pub const PENDING_FIRE_AND_FORGET_CEILING: usize = 256;
const MAX_RETRY_COUNT: u32 = 8;

// Retry exhaustion alone is not sufficient to declare Closed: under Renode
// (1 µs quantum) a long-running MCU command can stall status emission for
// several seconds wall while the wire remains healthy. Only close when
// retry exhaustion coincides with genuine MCU silence.
const MCU_SILENCE_FOR_CLOSE: Duration = Duration::from_secs(120);

const MAX_SUBMITS_PER_ITER: usize = 4;
const READ_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(1);
const ZERO_BYTE_DEBOUNCE: std::time::Duration = std::time::Duration::from_millis(100);

impl Reactor {
    pub(crate) fn dispatch_submission(
        &mut self,
        call_id: u64,
        payload: Vec<u8>,
        expected_response_name: String,
        completion: std::sync::mpsc::SyncSender<
            Result<crate::transport::MessageParams, TransportError>,
        >,
        deadline: Instant,
    ) -> Result<(), TransportError> {
        if self.unacked_window.is_full() {
            if self.pending_submissions.len() >= PENDING_SUBMISSION_CEILING {
                let _ = completion.send(Err(TransportError::Backpressure));
                return Ok(());
            }
            self.pending_submissions.push_back(PendingSubmission {
                call_id,
                payload,
                expected_response_name,
                completion,
                deadline,
            });
            self.pending_outbound_order
                .push_back(PendingOutboundKind::Submission);
            return Ok(());
        }

        let seq = self.send_seq;
        self.send_seq += 1;
        let wire_seq = (seq & 0x0F) as u8;
        let frame = crate::host_io::wire::build_frame(&payload, wire_seq);

        let sent_time_raw = crate::clock::monotonic_raw_secs();
        self.write_frame(&frame)?;

        let now = self.clock.now();
        self.unacked_window
            .push(crate::host_io::window::UnackedEntry {
                seq,
                frame_bytes: frame,
                sent_at: now,
                retry_count: 0,
            });
        let _trace_name = expected_response_name.clone();
        self.awaiting_response
            .push(crate::host_io::window::AwaitEntry {
                call_id,
                seq,
                expected_response_name,
                completion,
                submitted_at: now,
                deadline,
                abandoned: false,
                sent_time_raw,
            })?;
        tracing::trace!(
            subsystem = "mcu-comms",
            event = "await_response",
            tid = ?std::thread::current().id(),
            call_id,
            seq,
            name = %_trace_name,
            await_len = self.awaiting_response.len(),
            "push await entry"
        );

        if !self.rtt_sample_armed {
            self.rtt_sample_seq = seq;
            self.rtt_sample_armed = true;
        }
        Ok(())
    }

    pub(crate) fn dispatch_fire_and_forget(
        &mut self,
        payload: Vec<u8>,
        is_get_clock: bool,
    ) -> Result<(), TransportError> {
        if self.unacked_window.is_full() {
            if self.pending_fire_and_forget.len() >= PENDING_FIRE_AND_FORGET_CEILING {
                log::error!(
                    "dispatch_fire_and_forget: pending_fire_and_forget at ceiling ({}); refusing payload",
                    PENDING_FIRE_AND_FORGET_CEILING,
                );
                return Err(TransportError::Backpressure);
            }
            self.pending_fire_and_forget
                .push_back((payload, is_get_clock));
            self.pending_outbound_order
                .push_back(PendingOutboundKind::FireAndForget);
            return Ok(());
        }
        let seq = self.send_seq;
        self.send_seq += 1;
        let wire_seq = (seq & 0x0F) as u8;
        let frame = crate::host_io::wire::build_frame(&payload, wire_seq);
        // get_clock send stamps MUST be captured at the wire write, not at
        // command processing: on a busy link (beacon's status stream) the
        // frame can queue for multiple ms, and an early stamp pairs the
        // response clock with a fictitious send time — observed as a
        // constant +5.6ms outlier on every beacon clocksync sample.
        if is_get_clock {
            self.pending_clock_sent_raw = Some(crate::clock::monotonic_raw_secs());
        }
        if let Err(e) = self.write_frame(&frame) {
            if is_get_clock {
                self.pending_clock_sent_raw = None;
            }
            return Err(e);
        }
        let now = self.clock.now();
        self.unacked_window
            .push(crate::host_io::window::UnackedEntry {
                seq,
                frame_bytes: frame,
                sent_at: now,
                retry_count: 0,
            });
        Ok(())
    }

    pub(crate) fn drain_pending_submissions(&mut self) {
        while !self.unacked_window.is_full() {
            let Some(kind) = self.pending_outbound_order.pop_front() else {
                break;
            };
            match kind {
                PendingOutboundKind::Submission => {
                    let Some(p) = self.pending_submissions.pop_front() else {
                        log::error!("pending outbound order referenced missing submission");
                        continue;
                    };
                    let completion = p.completion.clone();
                    if let Err(e) = self.dispatch_submission(
                        p.call_id,
                        p.payload,
                        p.expected_response_name,
                        completion,
                        p.deadline,
                    ) {
                        // Propagate the transport error itself, not a
                        // misleading DispatcherTimeout.
                        let is_io = matches!(e, TransportError::Io(_));
                        let _ = p.completion.send(Err(e));
                        if is_io {
                            if self.pending_host_fault.is_none() {
                                self.pending_host_fault =
                                    Some(crate::host_io::runtime_events::FaultEvent {
                                        fault_code: FaultCode::HostDisconnect.as_u16(),
                                        fault_detail: 0,
                                        segment_id: 0,
                                        synthesized: false,
                                    });
                            }
                            self.state = ReactorState::Closed;
                            return;
                        }
                    }
                }
                PendingOutboundKind::FireAndForget => {
                    let Some((payload, is_get_clock)) = self.pending_fire_and_forget.pop_front()
                    else {
                        log::error!("pending outbound order referenced missing fire-and-forget");
                        continue;
                    };
                    if let Err(e) = self.dispatch_fire_and_forget(payload, is_get_clock) {
                        if matches!(e, TransportError::Io(_)) {
                            if self.pending_host_fault.is_none() {
                                self.pending_host_fault =
                                    Some(crate::host_io::runtime_events::FaultEvent {
                                        fault_code: FaultCode::HostDisconnect.as_u16(),
                                        fault_detail: 0,
                                        segment_id: 0,
                                        synthesized: false,
                                    });
                            }
                            self.state = ReactorState::Closed;
                            return;
                        }
                        log::warn!(
                            "drain_pending_submissions: fire-and-forget redispatch error: {e}"
                        );
                    }
                }
            }
        }
    }

    pub(crate) fn drain_passthrough(&mut self) {
        let mcu = match self.passthrough_mcu {
            Some(m) => m,
            None => return,
        };

        let mut router = match self.passthrough_router.take() {
            Some(r) => r,
            None => return,
        };

        let _ = router.promote_all(mcu, 0);

        loop {
            if self.unacked_window.is_full() {
                break;
            }
            let entry = match router.pop_next_for_emission(mcu) {
                Ok(Some(e)) => e,
                _ => break,
            };

            let seq = self.send_seq;
            self.send_seq += 1;
            let wire_seq = (seq & 0x0F) as u8;
            let frame = crate::host_io::wire::build_frame(entry.bytes(), wire_seq);

            if let Err(_e) = self.write_frame(&frame) {
                if self.pending_host_fault.is_none() {
                    self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                        fault_code: FaultCode::HostDisconnect.as_u16(),
                        fault_detail: 0,
                        segment_id: 0,
                        synthesized: false,
                    });
                }
                self.state = ReactorState::Closed;
                self.passthrough_router = Some(router);
                return;
            }

            let now = self.clock.now();
            self.unacked_window
                .push(crate::host_io::window::UnackedEntry {
                    seq,
                    frame_bytes: frame,
                    sent_at: now,
                    retry_count: 0,
                });

            if !entry.notify_id().is_none() {
                self.passthrough_notify_map
                    .insert(seq, (mcu, entry.notify_id()));
            }

            if !self.rtt_sample_armed {
                self.rtt_sample_seq = seq;
                self.rtt_sample_armed = true;
            }
        }

        self.passthrough_router = Some(router);
    }

    fn update_receive_seq(&mut self, rseq: u64) -> Result<(), TransportError> {
        if self.unacked_window.is_empty() {
            self.send_seq = rseq;
            self.receive_seq = rseq;
            return Ok(());
        }
        let popped = self.unacked_window.pop_acked(rseq);
        for entry in &popped {
            if self.rtt_sample_armed && entry.seq >= self.rtt_sample_seq {
                let rtt = self.clock.now() - entry.sent_at;
                self.rtt.update(rtt);
                self.rtt_sample_armed = false;
                break;
            }
        }
        if let (Some(router), Some(mcu)) = (self.passthrough_router.as_mut(), self.passthrough_mcu)
        {
            for entry in &popped {
                let payload_len = entry
                    .frame_bytes
                    .len()
                    .saturating_sub(crate::host_io::wire::MESSAGE_MIN);
                let _ = router.record_ack(mcu, payload_len as u64);
            }
        }
        self.receive_seq = rseq;
        Ok(())
    }

    pub(crate) fn handle_ack_nak(&mut self, wire_seq_nibble: u8) -> Result<(), TransportError> {
        let rseq = crate::host_io::wire::decode_absolute(self.receive_seq, wire_seq_nibble);

        if rseq > self.receive_seq {
            self.update_receive_seq(rseq)?;
        }

        if self.last_ack_seq < rseq {
            self.last_ack_seq = rseq;
        } else if rseq > self.ignore_nak_seq && !self.unacked_window.is_empty() {
            self.write_retransmit(RetransmitTrigger::NakDriven)?;
        }
        Ok(())
    }

    pub(crate) fn write_retransmit(
        &mut self,
        trigger: RetransmitTrigger,
    ) -> Result<(), TransportError> {
        let buf = {
            let frames: Vec<&[u8]> = self
                .unacked_window
                .iter()
                .map(|e| e.frame_bytes.as_slice())
                .collect();
            crate::host_io::wire::build_retransmit_buffer(frames)
        };
        self.write_frame(&buf)?;

        match trigger {
            RetransmitTrigger::NakDriven => {
                if self.receive_seq < self.retransmit_seq {
                    self.ignore_nak_seq = self.retransmit_seq;
                } else {
                    self.ignore_nak_seq = self.receive_seq;
                }
            }
            RetransmitTrigger::TimeoutDriven => {
                self.ignore_nak_seq = self.send_seq;
            }
        }
        self.retransmit_seq = self.send_seq;
        self.rtt_sample_armed = false;

        let now = self.clock.now();
        let silence = now.duration_since(self.last_recv_time);
        for entry in self.unacked_window.iter_mut() {
            entry.retry_count += 1;
            if entry.retry_count >= MAX_RETRY_COUNT && silence >= MCU_SILENCE_FOR_CLOSE {
                self.state = ReactorState::Closed;
                self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                    fault_code: FaultCode::HostRetransmitExhausted.as_u16(),
                    fault_detail: entry.retry_count,
                    segment_id: 0,
                    synthesized: false,
                });
                return Err(TransportError::Closed);
            }
        }

        if matches!(trigger, RetransmitTrigger::TimeoutDriven) {
            self.rtt.backoff();
        }
        Ok(())
    }
}

impl Reactor {
    pub(crate) fn handle_inbound_frame(
        &mut self,
        frame: KlipperFrame,
    ) -> Result<(), TransportError> {
        let bytes = frame.bytes();
        if bytes.len() < crate::host_io::wire::MESSAGE_MIN {
            return Ok(());
        }
        let wire_seq_nibble = bytes[1] & 0x0F;
        if bytes.len() == crate::host_io::wire::MESSAGE_MIN {
            self.handle_ack_nak(wire_seq_nibble)?;
            return Ok(());
        }
        let rseq = crate::host_io::wire::decode_absolute(self.receive_seq, wire_seq_nibble);
        let rseq_jump = rseq.saturating_sub(self.receive_seq);
        if rseq_jump > 1 {
            tracing::warn!(
                subsystem = "mcu-comms",
                event = "rx_seq_jump",
                receive_seq_prev = self.receive_seq,
                receive_seq_new = rseq,
                jump = rseq_jump,
                "receive_seq jumped >1: MCU dropped a response or we missed a frame"
            );
        }
        if rseq != self.receive_seq {
            self.update_receive_seq(rseq)?;
        }
        let decoded = match self.parser.decode(bytes) {
            Ok(d) => d,
            Err(e) => {
                tracing::warn!(
                    subsystem = "mcu-comms",
                    event = "decode_error",
                    error = ?e,
                    bytes_len = bytes.len(),
                    first16 = ?&bytes[..bytes.len().min(16)],
                    "frame decode error"
                );
                return Ok(());
            }
        };
        let raw_payload = {
            let msglen = bytes[0] as usize;
            let trailer = crate::host_io::wire::MESSAGE_TRAILER_SIZE;
            let header = crate::host_io::wire::MESSAGE_HEADER_SIZE;
            if msglen > header + trailer {
                bytes[header..msglen - trailer].to_vec()
            } else {
                Vec::new()
            }
        };

        match decoded {
            crate::host_io::parser::DecodedFrame::Response { name, params } => {
                let await_len_before = self.awaiting_response.len();
                if let Some(idx) = self.awaiting_response.find_match(&name) {
                    let entry = self.awaiting_response.remove(idx);
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "await_response",
                        tid = ?std::thread::current().id(),
                        %name,
                        idx,
                        await_len = await_len_before,
                        matched_call_id = entry.call_id,
                        matched_seq = entry.seq,
                        "solicited response matched"
                    );
                    let mut params = params;
                    params.sent_time_raw = entry.sent_time_raw;
                    params.recv_time_raw = crate::clock::monotonic_raw_secs();
                    let _ = entry.completion.send(Ok(params));
                } else {
                    let oid = params.fields.get("oid").and_then(|v| match v {
                        crate::transport::MessageValue::U32(n) => Some(*n),
                        crate::transport::MessageValue::I32(n) => Some(*n as u32),
                        _ => None,
                    });
                    if name.contains("software_trip") || name.contains("trsync_state") {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "unsolicited_frame",
                            %name,
                            ?oid,
                            interceptor_count = self.interceptors.entry_count(),
                            params = ?params,
                            "unsolicited frame (software_trip/trsync_state)"
                        );
                    } else {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "unsolicited_frame",
                            %name,
                            ?oid,
                            interceptor_count = self.interceptors.entry_count(),
                            "unsolicited frame"
                        );
                    }
                    // If this is the "clock" response for a pending get_clock_async
                    // request, inject RAW timestamps so Python clocksync sees an
                    // honest RTT rather than the usual half_rtt=0 artefact.
                    if name == "clock" {
                        if let Some(sent_raw) = self.pending_clock_sent_raw.take() {
                            let recv_raw = crate::clock::monotonic_raw_secs();
                            let mut stamped = params.clone();
                            stamped.sent_time_raw = sent_raw;
                            stamped.recv_time_raw = recv_raw;
                            let event =
                                crate::host_io::runtime_events::RuntimeEvent::PassthroughResponse {
                                    name,
                                    params: stamped,
                                };
                            self.dispatch_runtime_event(event);
                            return Ok(());
                        }
                    }

                    self.interceptors.dispatch(&name, oid, &params);

                    if !self.try_dispatch_passthrough_response(&raw_payload) {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "unsolicited_no_interceptor",
                            tid = ?std::thread::current().id(),
                            %name,
                            await_len = await_len_before,
                            "unsolicited frame with no interceptor match"
                        );
                        let event =
                            crate::host_io::runtime_events::RuntimeEvent::PassthroughResponse {
                                name,
                                params,
                            };
                        self.dispatch_runtime_event(event);
                    }
                }
            }
            crate::host_io::parser::DecodedFrame::Output { name, params } => {
                // Output frames (MCU `output()` calls) bypass the solicited-
                // response machinery but MUST still run through interceptors so
                // that relay handlers registered by `trip_dispatch::prepare`
                // (e.g. for `kalico_endstop_tripped`) can fire.  The frame is
                // still lifted to a `RuntimeEvent` and forwarded to Python via
                // `dispatch_runtime_event` after interceptors run.
                let oid = params.fields.get("oid").and_then(|v| match v {
                    crate::transport::MessageValue::U32(n) => Some(*n),
                    crate::transport::MessageValue::I32(n) => Some(*n as u32),
                    _ => None,
                });
                let interceptor_count = self.interceptors.entry_count();
                if interceptor_count > 0 {
                    tracing::debug!(
                        subsystem = "trip-relay",
                        event = "output_frame_intercepted",
                        %name,
                        ?oid,
                        interceptor_count,
                        "output frame dispatched through interceptors"
                    );
                }
                self.interceptors.dispatch(&name, oid, &params);
                let event = crate::host_io::runtime_events::RuntimeEvent::lift(&name, params);
                self.dispatch_runtime_event(event);
            }
        }
        Ok(())
    }

    fn try_dispatch_passthrough_response(&mut self, raw_payload: &[u8]) -> bool {
        if self.passthrough_notify_map.is_empty() {
            return false;
        }
        let oldest_seq = match self.passthrough_notify_map.keys().copied().min() {
            Some(s) => s,
            None => return false,
        };
        let (mcu, notify_id) = match self.passthrough_notify_map.remove(&oldest_seq) {
            Some(pair) => pair,
            None => return false,
        };
        if let Some(router) = self.passthrough_router.as_mut() {
            let _ = router.dispatch_response(mcu, notify_id, raw_payload.to_vec());
        }
        true
    }

    fn dispatch_runtime_event(&mut self, event: crate::host_io::runtime_events::RuntimeEvent) {
        self.event_dispatcher.dispatch(event);
    }

    pub(crate) fn handle_kalico_frame(&mut self, channel: u8, payload: &[u8]) {
        match dispatch_kalico_frame(&mut self.kalico_state, channel, payload) {
            KalicoDispatchResult::Handled | KalicoDispatchResult::Ignored => {}
            KalicoDispatchResult::Event(ev) => {
                self.dispatch_runtime_event(ev);
            }
        }
    }
}

impl Reactor {
    fn poll_serial(&mut self) {
        let t0 = std::time::Instant::now();
        let deadline = self.clock.now() + READ_TIMEOUT;
        let outcome = self.io.poll_frames_until(deadline);
        let dt = t0.elapsed();
        if dt > std::time::Duration::from_millis(5) {
            let label: &'static str = match &outcome {
                Ok(PollOutcome::Frames { .. }) => "Frames",
                Ok(PollOutcome::Timeout) => "Timeout",
                Ok(PollOutcome::PhantomZero) => "PhantomZero",
                Err(_) => "Err",
            };
            tracing::debug!(
                subsystem = "mcu-comms",
                event = "slow_poll",
                dt_ms = dt.as_secs_f64() * 1000.0,
                outcome = label,
                "poll_serial exceeded 5ms"
            );
        }
        match outcome {
            Ok(PollOutcome::Frames { frames, errors }) => {
                self.zero_byte_first_seen = None;
                self.zero_byte_consec = 0;
                if !frames.is_empty() || !errors.is_empty() {
                    self.last_recv_time = self.clock.now();
                }
                for e in errors {
                    log::warn!("kalico stream error: {e}");
                }
                for f in frames {
                    match f {
                        Frame::Klipper(kf) => {
                            if self.handle_inbound_frame(kf).is_err() {
                                return;
                            }
                        }
                        Frame::Kalico { channel, payload } => {
                            self.handle_kalico_frame(channel, &payload);
                        }
                    }
                }
            }
            Ok(PollOutcome::Timeout) => {
                self.zero_byte_first_seen = None;
                self.zero_byte_consec = 0;
            }
            Ok(PollOutcome::PhantomZero) => {
                self.zero_byte_consec = self.zero_byte_consec.saturating_add(1);
                let now = self.clock.now();
                let first = *self.zero_byte_first_seen.get_or_insert(now);
                if now.duration_since(first) >= ZERO_BYTE_DEBOUNCE {
                    let silence_ms = now.duration_since(self.last_recv_time).as_millis();
                    let since_write_ms = now.duration_since(self.last_write_time).as_millis();
                    log::warn!(
                        "[usb-drop] silence_ms={} since_write_ms={} consec_zero={} err=PhantomZero(Ok(0) for >={ZERO_BYTE_DEBOUNCE:?})",
                        silence_ms,
                        since_write_ms,
                        self.zero_byte_consec,
                    );
                    self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                        fault_code: FaultCode::HostDisconnect.as_u16(),
                        fault_detail: 0,
                        segment_id: 0,
                        synthesized: false,
                    });
                    self.state = ReactorState::Closed;
                }
            }
            Err(e) => {
                let now = self.clock.now();
                let silence_ms = now.duration_since(self.last_recv_time).as_millis();
                let since_write_ms = now.duration_since(self.last_write_time).as_millis();
                log::warn!(
                    "[usb-drop] silence_ms={} since_write_ms={} consec_zero={} err={:?}",
                    silence_ms,
                    since_write_ms,
                    self.zero_byte_consec,
                    e,
                );
                self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                    fault_code: FaultCode::HostDisconnect.as_u16(),
                    fault_detail: 0,
                    segment_id: 0,
                    synthesized: false,
                });
                self.state = ReactorState::Closed;
            }
        }
    }
}

impl Reactor {
    pub(crate) fn transition_closed_on_io_fault(&mut self) {
        if self.pending_host_fault.is_none() {
            self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                fault_code: FaultCode::HostDisconnect.as_u16(),
                fault_detail: 0,
                segment_id: 0,
                synthesized: false,
            });
        }
        self.state = ReactorState::Closed;
    }

    fn handle_command(&mut self, cmd: crate::host_io::ReactorCommand) {
        use crate::host_io::ReactorCommand;
        match cmd {
            ReactorCommand::Submit {
                call_id,
                cmd,
                expected_response_name,
                completion,
                deadline,
            } => match self.parser.encode(&cmd) {
                Ok(payload) => {
                    if let Err(e) = self.dispatch_submission(
                        call_id,
                        payload,
                        expected_response_name,
                        completion.clone(),
                        deadline,
                    ) {
                        let is_io = matches!(e, TransportError::Io(_));
                        let _ = completion.send(Err(e));
                        if is_io {
                            self.transition_closed_on_io_fault();
                        }
                    }
                }
                Err(e) => {
                    let _ = completion.send(Err(TransportError::Parse(format!("{e:?}"))));
                }
            },
            ReactorCommand::SubmitTyped {
                call_id,
                payload,
                expected_response_name,
                completion,
                deadline,
            } => {
                tracing::debug!(
                    subsystem = "mcu-comms",
                    event = "submit_typed",
                    call_id,
                    resp = %expected_response_name,
                    payload_len = payload.len(),
                    unacked = self.unacked_window.len(),
                    pending_sub = self.pending_submissions.len(),
                    state = ?self.state,
                    "SubmitTyped"
                );
                if let Err(e) = self.dispatch_submission(
                    call_id,
                    payload,
                    expected_response_name,
                    completion.clone(),
                    deadline,
                ) {
                    let is_io = matches!(e, TransportError::Io(_));
                    let _ = completion.send(Err(e));
                    if is_io {
                        self.transition_closed_on_io_fault();
                    }
                }
            }
            ReactorCommand::Abandon(call_id) => {
                self.awaiting_response.mark_abandoned(call_id);
            }
            ReactorCommand::Shutdown => {
                self.state = ReactorState::Closed;
                self.closed_via_shutdown = true;
            }
            ReactorCommand::MarkExpectedDisconnect => {
                tracing::info!(
                    subsystem = "mcu-comms",
                    event = "expected_disconnect",
                    kalico_pending = self.kalico_state.pending.len(),
                    await_n = self.awaiting_response.len(),
                    unacked_n = self.unacked_window.len(),
                    "MarkExpectedDisconnect received"
                );
                self.closed_via_shutdown = true;
            }
            ReactorCommand::AttachHeartbeatCallback(wrapper) => {
                self.event_dispatcher.heartbeat_callback = Some(wrapper.0);
            }
            ReactorCommand::SetMcuLogHook(wrapper) => {
                self.event_dispatcher
                    .set_mcu_log_hook(move |e| (wrapper.0)(e));
            }
            ReactorCommand::SubscribeFault { sender, reply } => {
                let result = self.event_dispatcher.fault_latch.subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::SubscribeTrace { sender, reply } => {
                let result = self.event_dispatcher.trace_ring.subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::SubscribeRuntimeEvents { sender, reply } => {
                let result = self
                    .event_dispatcher
                    .runtime_event_dispatcher
                    .subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::SubscribeHostEvents { sender, reply } => {
                let result = self
                    .event_dispatcher
                    .host_event_dispatcher
                    .subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::InstallPassthroughRouter(router) => {
                let mcu = router.mcu_handles().next().copied();
                self.passthrough_router = Some(router);
                self.passthrough_mcu = mcu;
            }
            ReactorCommand::PassthroughSend {
                mcu,
                queue_id,
                entry,
            } => {
                if let Some(ref mut router) = self.passthrough_router {
                    let _ = router.push(mcu, queue_id, entry);
                }
            }
            ReactorCommand::FireAndForget { cmd } => match self.parser.encode(&cmd) {
                Ok(payload) => {
                    let cmd_disp = if cmd.len() > 120 {
                        &cmd[..120]
                    } else {
                        cmd.as_str()
                    };
                    let head: Vec<String> = payload
                        .iter()
                        .take(16)
                        .map(|b| format!("{b:02x}"))
                        .collect();
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "fire_and_forget_sent",
                        cmd = %cmd_disp,
                        payload_len = payload.len(),
                        head = %head.join(","),
                        "FireAndForget encoded OK"
                    );
                    if let Err(e) = self.dispatch_fire_and_forget(payload, false) {
                        let is_io = matches!(e, TransportError::Io(_));
                        tracing::error!(
                            subsystem = "mcu-comms",
                            event = "fire_and_forget_send_error",
                            cmd = %cmd_disp,
                            error = %e,
                            "FireAndForget dispatch failed"
                        );
                        if is_io {
                            self.transition_closed_on_io_fault();
                        }
                    }
                }
                Err(e) => {
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "fire_and_forget_encode_error",
                        cmd = ?cmd,
                        error = ?e,
                        "FireAndForget encode failed"
                    );
                }
            },
            ReactorCommand::FireAndForgetTyped { payload } => {
                if let Err(e) = self.dispatch_fire_and_forget(payload, false) {
                    let is_io = matches!(e, TransportError::Io(_));
                    log::warn!("FireAndForgetTyped: send error: {e}");
                    if is_io {
                        self.transition_closed_on_io_fault();
                    }
                }
            }
            ReactorCommand::KalicoIdentify {
                completion,
                deadline: _,
            } => {
                let cid = self.kalico_state.allocate_correlation_id();
                let frame = build_kalico_identify_frame(cid);
                if self.kalico_state.identify_pending.is_some() {
                    let _ = completion.send(Err(TransportError::Backpressure));
                    return;
                }
                self.kalico_state.identify_pending = Some(completion);
                if let Err(e) = self.write_frame(&frame) {
                    let is_io = matches!(e, TransportError::Io(_));
                    if let Some(c) = self.kalico_state.identify_pending.take() {
                        let _ = c.send(Err(e));
                    }
                    if is_io {
                        self.transition_closed_on_io_fault();
                    }
                }
            }
            ReactorCommand::KalicoCall {
                channel,
                kind,
                body,
                completion,
                deadline,
            } => {
                if !self.kalico_state.identified {
                    let _ = completion.send(Err(TransportError::Parse(
                        "kalico transport not yet identified".into(),
                    )));
                    return;
                }
                let cid = self.kalico_state.allocate_correlation_id();
                let frame = build_kalico_frame(channel, kind, cid, &body);
                self.kalico_state.pending.insert(
                    cid,
                    PendingKalicoCall {
                        completion: completion.clone(),
                        deadline,
                    },
                );
                if let Err(e) = self.write_frame(&frame) {
                    let is_io = matches!(e, TransportError::Io(_));
                    if let Some(p) = self.kalico_state.pending.remove(&cid) {
                        let _ = p.completion.send(Err(e));
                    }
                    if is_io {
                        self.transition_closed_on_io_fault();
                    }
                }
            }
            ReactorCommand::GetClockAndDeliver => match self.parser.encode("get_clock") {
                Ok(payload) => {
                    // The RAW send stamp is captured inside
                    // dispatch_fire_and_forget at the actual wire write —
                    // never here, where the frame may still queue behind a
                    // busy link for milliseconds.
                    if let Err(e) = self.dispatch_fire_and_forget(payload, true) {
                        let is_io = matches!(e, TransportError::Io(_));
                        tracing::error!(
                            subsystem = "mcu-comms",
                            event = "get_clock_async_send_error",
                            error = %e,
                            "GetClockAndDeliver dispatch failed"
                        );
                        if is_io {
                            self.transition_closed_on_io_fault();
                        }
                    }
                }
                Err(e) => {
                    tracing::error!(
                        subsystem = "mcu-comms",
                        event = "get_clock_async_encode_error",
                        error = ?e,
                        "GetClockAndDeliver: encode 'get_clock' failed"
                    );
                }
            },
            ReactorCommand::Noop => {}
            ReactorCommand::RegisterInterceptor {
                msg_name,
                oid,
                callback,
                reply,
            } => {
                let id = self.interceptors.register(msg_name, oid, callback);
                let _ = reply.send(id);
            }
            ReactorCommand::UnregisterInterceptor { id } => {
                self.interceptors.unregister(id);
            }
        }
    }
}

impl Reactor {
    fn flush_all_completions(&mut self) {
        self.pending_clock_sent_raw = None;
        for entry in self.awaiting_response.drain_all() {
            let _ = entry.completion.send(Err(TransportError::Closed));
        }
        self.unacked_window.clear();
        for p in self.pending_submissions.drain(..) {
            let _ = p.completion.send(Err(TransportError::Closed));
        }
        self.pending_fire_and_forget.clear();
        self.pending_outbound_order.clear();
        self.passthrough_notify_map.clear();

        let drained: Vec<PendingKalicoCall> =
            self.kalico_state.pending.drain().map(|(_, v)| v).collect();
        for p in drained {
            let _ = p.completion.send(Err(TransportError::Closed));
        }
        if let Some(c) = self.kalico_state.identify_pending.take() {
            let _ = c.send(Err(TransportError::Closed));
        }
    }

    pub(crate) fn gc_kalico_pending(&mut self) {
        let now = self.clock.now();
        let expired: Vec<u32> = self
            .kalico_state
            .pending
            .iter()
            .filter_map(|(cid, p)| if p.deadline <= now { Some(*cid) } else { None })
            .collect();
        for cid in expired {
            if let Some(p) = self.kalico_state.pending.remove(&cid) {
                let _ = p.completion.send(Err(TransportError::Timeout));
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum TickOutcome {
    Continue,
    Closed,
}

impl Reactor {
    pub fn run(&mut self) {
        loop {
            if matches!(self.tick_once(), TickOutcome::Closed) {
                break;
            }
        }
    }

    pub fn exited_gracefully(&self) -> bool {
        self.closed_via_shutdown
    }

    pub fn tick_once(&mut self) -> TickOutcome {
        let t_tick = std::time::Instant::now();

        let s1 = std::time::Instant::now();
        for _ in 0..MAX_SUBMITS_PER_ITER {
            match self.submission_rx.try_recv() {
                Ok(cmd) => self.handle_command(cmd),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    self.state = ReactorState::Closed;
                    break;
                }
            }
        }

        let t_step1 = s1.elapsed();

        let s2 = std::time::Instant::now();
        self.poll_serial();
        let t_step2 = s2.elapsed();

        let s3 = std::time::Instant::now();
        self.drain_pending_submissions();
        let t_step3 = s3.elapsed();

        let s3b = std::time::Instant::now();
        self.drain_passthrough();
        let t_step3b = s3b.elapsed();

        let s4 = std::time::Instant::now();
        if let Some(front) = self.unacked_window.front() {
            let now = self.clock.now();
            if now >= front.sent_at + self.rtt.current_rto() {
                let unacked_n = self.unacked_window.len();
                let front_seq = front.seq;
                if let Err(e) = self.write_retransmit(RetransmitTrigger::TimeoutDriven) {
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "retransmit",
                        front_seq = front_seq,
                        unacked_n = unacked_n,
                        error = ?e,
                        "retransmit error"
                    );
                    if matches!(e, TransportError::Io(_)) {
                        log::warn!("retransmit Io error: {e:?}; transitioning Closed");
                        self.transition_closed_on_io_fault();
                    }
                }
            }
        }
        let t_step4 = s4.elapsed();

        if let Some(fault) = self.pending_host_fault.take() {
            self.event_dispatcher.fault_latch.dispatch(fault);
        }

        self.event_dispatcher.host_event_dispatcher.drain_pending();

        let now = self.clock.now();
        let evicted = self.awaiting_response.evict_expired(now);
        for entry in evicted {
            let _ = entry
                .completion
                .send(Err(TransportError::DispatcherTimeout));
        }

        self.gc_kalico_pending();

        if self.state == ReactorState::Closed {
            self.flush_all_completions();
            return TickOutcome::Closed;
        }

        let dt_tick = t_tick.elapsed();
        if dt_tick > std::time::Duration::from_millis(5) {
            tracing::debug!(
                subsystem = "mcu-comms",
                event = "slow_tick",
                dt_ms = dt_tick.as_secs_f64() * 1000.0,
                step1_ms = t_step1.as_secs_f64() * 1000.0,
                step2_ms = t_step2.as_secs_f64() * 1000.0,
                step3_ms = t_step3.as_secs_f64() * 1000.0,
                step3b_ms = t_step3b.as_secs_f64() * 1000.0,
                step4_ms = t_step4.as_secs_f64() * 1000.0,
                "tick_once exceeded 5ms"
            );
        }
        TickOutcome::Continue
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod a1_seq_wrap;

#[cfg(test)]
mod a2_nak_rto;

#[cfg(test)]
mod a4_nak_submit_race;

#[cfg(test)]
mod a3_awaiting_response_gc;

#[cfg(test)]
mod a5_passthrough_integration;

#[cfg(test)]
mod a8_fire_and_forget_backpressure;

#[cfg(test)]
mod fire_and_forget_typed_routing;

#[cfg(test)]
mod io_fault_propagation;
