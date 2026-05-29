//! Single-thread poll-reactor. Spec §3.7.

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

    // 64-bit absolute sequence counters. Per spec §3.1 / serialqueue.c:660-666.
    pub(crate) send_seq: u64,
    pub(crate) receive_seq: u64,
    pub(crate) last_ack_seq: u64,
    pub(crate) ignore_nak_seq: u64,
    pub(crate) retransmit_seq: u64,
    pub(crate) rtt_sample_seq: u64,
    pub(crate) rtt_sample_armed: bool,

    pub(crate) state: ReactorState,

    /// 2026-05-17 wedge-detection: distinguishes the "graceful shutdown"
    /// Closed transition (driven by `ReactorCommand::Shutdown` from
    /// `KalicoHostIo::drop` on process exit) from the "unexpected IO
    /// fault" Closed transition (transport_closed_on_io_fault). Set
    /// `true` only in the Shutdown handler. The thread-exit hook in
    /// `KalicoHostIo::open_with_port` reads this AFTER `reactor.run()`
    /// returns; if `false` (we exited due to a fault), it aborts the
    /// process so klippy crashes cleanly instead of silently
    /// pretending-to-be-up with a dead MCU FD.
    pub(crate) closed_via_shutdown: bool,

    pub(crate) pending_host_fault: Option<FaultEvent>,

    pub(crate) pending_submissions: VecDeque<PendingSubmission>,

    /// Backpressure-respecting fire-and-forget queue. When the unacked window
    /// is full, fire-and-forget payloads are enqueued here instead of dropped,
    /// then drained alongside `pending_submissions` once the window opens.
    /// See spec §6.0 in `2026-05-04-incremental-curve-upload-design.md`.
    pub(crate) pending_fire_and_forget: VecDeque<Vec<u8>>,

    /// FIFO order for pending submissions and fire-and-forget frames. Klipper
    /// config relies on strict wire order: a response-bearing barrier such as
    /// `get_config` must not overtake earlier fire-and-forget config frames.
    pub(crate) pending_outbound_order: VecDeque<PendingOutboundKind>,

    /// First-observed instant of a phantom `Ok(0)` from `port.read`.
    /// Per spec §3.11, treat as Closed only if it persists past
    /// `ZERO_BYTE_DEBOUNCE`. Cleared on any non-zero read.
    pub(crate) zero_byte_first_seen: Option<Instant>,

    /// Most recent moment `poll_serial` saw bytes-from-the-wire (Frames
    /// outcome, with or without complete frames). Used to gate the
    /// `MAX_RETRY_COUNT`-driven Closed escalation in `write_retransmit`
    /// — if the MCU is still actively emitting frames (e.g. periodic
    /// kalico_status at 10 Hz, or responses to other in-flight commands),
    /// we should not give up on a specific unacked entry just because its
    /// ACK got dropped by the firmware's 320-byte transmit_buf overflow.
    /// Closed only fires when retry exhaustion coincides with genuine
    /// MCU silence past `MCU_SILENCE_FOR_CLOSE`.
    pub(crate) last_recv_time: Instant,

    /// Injected clock seam (spec §2.3). Routes `Instant::now()` so tests
    /// can deterministically advance time via `MockClock`.
    pub(crate) clock: Arc<dyn Clock>,

    /// Optional passthrough router for klippy bridge integration. When
    /// `Some`, passthrough entries are emitted alongside typed commands
    /// using the same wire framing and sequence numbers.
    pub(crate) passthrough_router: Option<PassthroughRouter>,

    /// Maps wire sequence numbers to `(McuHandle, NotifyId)` so inbound
    /// responses can be routed back through the passthrough router's
    /// `dispatch_response`. Entries are inserted when a notify-bearing
    /// passthrough entry is emitted and removed when the response arrives
    /// or the entry is acked.
    pub(crate) passthrough_notify_map: std::collections::HashMap<u64, (McuHandle, NotifyId)>,

    /// The MCU handle that this reactor serves. Set when the passthrough
    /// router is installed. Phase 1 has one reactor per MCU.
    pub(crate) passthrough_mcu: Option<McuHandle>,

    // ── Phase C-B: kalico-native transport state ───────────────────────
    /// Pending kalico calls / identify state. Stream demuxing now lives
    /// inside `io: SerialFrameIo`.
    pub(crate) kalico_state: KalicoNativeState,

    /// Frame interceptor table. Callbacks registered here fire on the
    /// reactor thread before an unsolicited frame is forwarded to the
    /// `RuntimeEvent` dispatcher. Keyed by `(msg_name, oid)`.
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
            pending_submissions: VecDeque::new(),
            pending_fire_and_forget: VecDeque::new(),
            pending_outbound_order: VecDeque::new(),
            zero_byte_first_seen: None,
            last_recv_time: clock.now(),
            clock,
            passthrough_router: None,
            passthrough_notify_map: std::collections::HashMap::new(),
            passthrough_mcu: None,
            kalico_state: KalicoNativeState::default(),
            interceptors: crate::host_io::interceptor::InterceptorTable::new(),
        }
    }

    /// Test-only constructor that wraps a raw `Box<dyn SerialPort>` in a
    /// `SerialFrameIo` internally. Lets the existing test fixtures and
    /// harnesses keep using bespoke `SerialPort` implementations without
    /// each callsite having to know about `SerialFrameIo`.
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

    /// Single chokepoint for all wire writes. Per spec §3.7.
    pub(crate) fn write_frame(&mut self, frame: &[u8]) -> Result<(), TransportError> {
        // Diag: trace write durations and errors. Every write is logged with
        // a monotonic sequence number so we can correlate against the MCU
        // diag's rxflvl_n. If write_n grows during the wedge but rxflvl_n
        // stays frozen, the bytes left the host but never reached the MCU.
        // If write_n also freezes, the reactor itself is starving.
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
        let dt = t0.elapsed();
        // Wedge-isolation: log EVERY write unconditionally. Volume is
        // bounded and we need full visibility around the bridge_call hang.
        eprintln!(
            "[trace-write] tid={:?} seq={seq} proto={proto} bytes={bytes} dt_ms={:.3} result={:?} first8={:02x?}",
            std::thread::current().id(),
            dt.as_secs_f64() * 1000.0,
            result.as_ref().map(|_| "OK"),
            &frame[..frame.len().min(8)]
        );
        result
    }
}

impl Reactor {
    /// Install a passthrough router for bridge integration. The `mcu` handle
    /// identifies which MCU record in the router this reactor serves.
    pub fn set_passthrough_router(&mut self, router: PassthroughRouter, mcu: McuHandle) {
        self.passthrough_router = Some(router);
        self.passthrough_mcu = Some(mcu);
    }
}

/// Why a retransmit was triggered. C20 uses this to select the retransmit arm.
#[derive(Debug, Clone, Copy)]
pub enum RetransmitTrigger {
    NakDriven,
    TimeoutDriven,
}

const PENDING_SUBMISSION_CEILING: usize = 256;
pub const PENDING_FIRE_AND_FORGET_CEILING: usize = 256;
const MAX_RETRY_COUNT: u32 = 8;

/// Minimum wire silence (no Frames/errors batch observed from `poll_serial`)
/// required, on top of `retry_count >= MAX_RETRY_COUNT`, before declaring
/// the transport Closed via the retransmit-exhaustion path.
///
/// In production, MCU emits `kalico_status` at ~10 Hz, but under Renode (1 µs
/// quantum, ~0.2× wall) a long-running command like LoadCurve can block
/// `command_task` from yielding to status emits for 3-5 seconds wall — the
/// MCU is alive and will eventually respond, it just can't talk while
/// crunching. 10 seconds is well past the worst sim stall observed
/// (3.2 s) yet still surfaces a genuinely hung MCU within a reasonable
/// window. Port-level disconnects (USB unplug, TCP close) bypass this
/// guard via the `PhantomZero` / `Err(_)` arms of `poll_serial` →
/// `HostDisconnect` fault.
const MCU_SILENCE_FOR_CLOSE: Duration = Duration::from_secs(120);

const MAX_SUBMITS_PER_ITER: usize = 4;
// The reactor has no FD/eventfd wakeup for submissions sent over
// `submission_rx`; planner dispatch can therefore be waiting on a
// `kalico_call` while this thread is blocked in the serial read. Keep the
// read poll bounded to 1 ms so producer LoadCurve/PushSegment calls are not
// blocked by a coarse read timeout.
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
            })?;
        eprintln!(
            "[trace-await] tid={:?} push call_id={call_id} seq={seq} name={_trace_name} await_len={}",
            std::thread::current().id(),
            self.awaiting_response.len()
        );

        if !self.rtt_sample_armed {
            self.rtt_sample_seq = seq;
            self.rtt_sample_armed = true;
        }
        Ok(())
    }

    /// Send a command with no expected application-level response.
    /// The frame is still tracked in the unacked window for wire-level
    /// retransmit on NAK.
    pub(crate) fn dispatch_fire_and_forget(
        &mut self,
        payload: Vec<u8>,
    ) -> Result<(), TransportError> {
        if self.unacked_window.is_full() {
            // Spec §6.0: enqueue instead of dropping. Drained by
            // `drain_pending_submissions` once the window opens. Overflow of
            // the queue itself is a host-side bug — surface as Backpressure.
            if self.pending_fire_and_forget.len() >= PENDING_FIRE_AND_FORGET_CEILING {
                log::error!(
                    "dispatch_fire_and_forget: pending_fire_and_forget at ceiling ({}); refusing payload",
                    PENDING_FIRE_AND_FORGET_CEILING,
                );
                return Err(TransportError::Backpressure);
            }
            self.pending_fire_and_forget.push_back(payload);
            self.pending_outbound_order
                .push_back(PendingOutboundKind::FireAndForget);
            return Ok(());
        }
        let seq = self.send_seq;
        self.send_seq += 1;
        let wire_seq = (seq & 0x0F) as u8;
        let frame = crate::host_io::wire::build_frame(&payload, wire_seq);
        self.write_frame(&frame)?;
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
                        // The queued submission is already popped — propagate
                        // the underlying transport error to the caller so it
                        // doesn't surface as a `DispatcherTimeout`. On I/O
                        // failure also stage a HostDisconnect fault and stop
                        // draining; the run loop will observe the Closed state
                        // on the next iteration.
                        let is_io = matches!(e, TransportError::Io(_));
                        let _ = p.completion.send(Err(e));
                        if is_io {
                            eprintln!(
                                "[trace-close] drain_pending_submissions Io error kalico_pending={} await_n={} unacked_n={}",
                                self.kalico_state.pending.len(),
                                self.awaiting_response.len(),
                                self.unacked_window.len(),
                            );
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
                    let Some(payload) = self.pending_fire_and_forget.pop_front() else {
                        log::error!("pending outbound order referenced missing fire-and-forget");
                        continue;
                    };
                    if let Err(e) = self.dispatch_fire_and_forget(payload) {
                        if matches!(e, TransportError::Io(_)) {
                            eprintln!(
                                "[trace-close] drain pending fire-and-forget Io error: {e:?}"
                            );
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
                        // Non-I/O errors (e.g. window-full / Backpressure on re-enqueue):
                        // drop the payload silently and continue. Backpressure here
                        // means the queue is at the ceiling, which the dispatch path
                        // already logged.
                        log::warn!(
                            "drain_pending_submissions: fire-and-forget redispatch error: {e}"
                        );
                    }
                }
            }
        }
    }

    /// Drain passthrough entries from the router onto the wire. Called after
    /// `drain_pending_submissions` in the tick loop so both typed commands
    /// and passthrough entries share the same wire, sequence numbers, and
    /// unacked window.
    pub(crate) fn drain_passthrough(&mut self) {
        let mcu = match self.passthrough_mcu {
            Some(m) => m,
            None => return,
        };

        // Take the router out temporarily to avoid double-borrow of `self`.
        let mut router = match self.passthrough_router.take() {
            Some(r) => r,
            None => return,
        };

        // Promote entries whose min_clock has been reached. Placeholder
        // ack_clock=0 until Task 20 wires real clock_sync.
        let _ = router.promote_all(mcu, 0);

        // Emit entries while window has room and router has entries.
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
                eprintln!("[trace-close] drain_passthrough write_frame Io error: {_e:?}");
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

            // Track notify association so inbound responses can be
            // routed back through the router's dispatch_response.
            if !entry.notify_id().is_none() {
                self.passthrough_notify_map
                    .insert(seq, (mcu, entry.notify_id()));
            }

            if !self.rtt_sample_armed {
                self.rtt_sample_seq = seq;
                self.rtt_sample_armed = true;
            }
        }

        // Put the router back.
        self.passthrough_router = Some(router);
    }

    // -------------------------------------------------------------------------
    // Wire-protocol ack/nak handling — spec §3.5 (Codex finding #1 corrected).
    // -------------------------------------------------------------------------

    /// Advance `receive_seq` and pop newly-acked entries from the window.
    ///
    /// Special case: if the unacked window is empty this is the very first
    /// response from the MCU (first-connection sentinel) — snap both counters.
    fn update_receive_seq(&mut self, rseq: u64) -> Result<(), TransportError> {
        if self.unacked_window.is_empty() {
            // First-connection sentinel: snap both seqs.
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
        // Inform the passthrough router's receive window about acked bytes.
        if let (Some(router), Some(mcu)) = (self.passthrough_router.as_mut(), self.passthrough_mcu)
        {
            for entry in &popped {
                let payload_len = entry
                    .frame_bytes
                    .len()
                    .saturating_sub(crate::host_io::wire::MESSAGE_MIN);
                let _ = router.record_ack(mcu, payload_len as u64);
            }
            // Notify map entries are NOT removed on ACK — an ACK only proves
            // the MCU received the command, not that the response has arrived.
            // Entries are removed when the response is dispatched
            // (try_dispatch_passthrough_response) or on disconnect
            // (flush_all_completions).
        }
        self.receive_seq = rseq;
        Ok(())
    }

    /// Process one ack/nak nibble from the MCU.
    ///
    /// Algorithm (Codex finding #1 corrected order):
    ///   Step 1 — advance receive_seq if rseq is new (forward progress).
    ///   Step 2 — ack/nak discrimination:
    ///     • last_ack_seq < rseq  → forward-progress ack; update last_ack_seq.
    ///     • rseq > ignore_nak_seq AND window non-empty → duplicate-ack NAK.
    ///     • else → stale, drop.
    pub(crate) fn handle_ack_nak(&mut self, wire_seq_nibble: u8) -> Result<(), TransportError> {
        let rseq = crate::host_io::wire::decode_absolute(self.receive_seq, wire_seq_nibble);

        // Step 1: advance receive_seq if rseq is new.
        if rseq > self.receive_seq {
            self.update_receive_seq(rseq)?;
        }

        // Step 2: ack/nak discrimination.
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
        // Build retransmit buffer: leading SYNC + all unacked frames.
        let buf = {
            let frames: Vec<&[u8]> = self
                .unacked_window
                .iter()
                .map(|e| e.frame_bytes.as_slice())
                .collect();
            crate::host_io::wire::build_retransmit_buffer(frames)
        };
        self.write_frame(&buf)?;

        // Two-arm ignore_nak_seq (Codex finding #7).
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

        // Retry cap: increment all; fault only on exhaustion AND silence.
        //
        // The retry counter alone is a poor proxy for "MCU is dead": a
        // single long-running MCU command (LoadCurve takes several
        // seconds wall under Renode's 1µs quantum) can stall the
        // command_task long enough for the host's RFC-6298 RTO ladder
        // (25→50→100→…→3200 ms, sum 6.4 s) to fire 8 times — meanwhile
        // the MCU is still emitting kalico_status at 10 Hz and the wire
        // is healthy. The earlier behavior tore the reactor down with
        // HostRetransmitExhausted in exactly that scenario, blocking
        // every motion test at LoadCurve #2.
        //
        // Real "MCU dead" signature: no frames OR stream errors arrive
        // for at least `MCU_SILENCE_FOR_CLOSE`. Gate the escalation on
        // both retry exhaustion AND silence so we still trip when the
        // wire is truly down (HostDisconnect already handles port-level
        // EOF / errors via the `PhantomZero` / `Err(_)` arms of
        // `poll_serial`).
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

        // RTO backoff ONLY on TimeoutDriven.
        if matches!(trigger, RetransmitTrigger::TimeoutDriven) {
            self.rtt.backoff();
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Inbound frame routing — spec §3.5 / §3.6.
// ---------------------------------------------------------------------------

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
            // 5-byte ack/nak frame.
            self.handle_ack_nak(wire_seq_nibble)?;
            return Ok(());
        }
        // Real msg-id frame — advance receive_seq if needed.
        let rseq = crate::host_io::wire::decode_absolute(self.receive_seq, wire_seq_nibble);
        let rseq_jump = rseq.saturating_sub(self.receive_seq);
        if rseq_jump > 1 {
            eprintln!(
                "[trace-rx-jump] receive_seq prev={} new={} jump={} (>1 means MCU dropped a response or we missed a frame)",
                self.receive_seq, rseq, rseq_jump
            );
        }
        if rseq != self.receive_seq {
            self.update_receive_seq(rseq)?;
        }
        // Parse + dispatch. Decode errors are warn-logged and the frame is dropped
        // (not propagated as Closed) — dictionary version skew is recoverable.
        let decoded = match self.parser.decode(bytes) {
            Ok(d) => d,
            Err(e) => {
                eprintln!(
                    "[trace-decode-err] decode error: {e:?}; bytes_len={} first16={:02x?}",
                    bytes.len(),
                    &bytes[..bytes.len().min(16)]
                );
                return Ok(());
            }
        };
        // Extract the raw payload (between header and trailer) for
        // passthrough notify dispatch. The payload is bytes [2..msglen-3].
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
                    eprintln!(
                        "[trace-resp] tid={:?} match name={name} idx={idx} await_len={await_len_before} matched_call_id={} matched_seq={}",
                        std::thread::current().id(),
                        entry.call_id,
                        entry.seq
                    );
                    let _ = entry.completion.send(Ok(params));
                } else {
                    let oid = params.fields.get("oid").and_then(|v| match v {
                        crate::transport::MessageValue::U32(n) => Some(*n),
                        crate::transport::MessageValue::I32(n) => Some(*n as u32),
                        _ => None,
                    });
                    // DIAG: trace every unsolicited frame + interceptor state
                    {
                        use std::io::Write;
                        if let Ok(mut f) = std::fs::OpenOptions::new()
                            .create(true).append(true)
                            .open("/tmp/interceptor_trace.log")
                        {
                            if name.contains("software_trip") || name.contains("trsync_state") {
                                let _ = writeln!(f,
                                    "[{:?}] unsolicited name={} oid={:?} interceptor_count={} params={:?}",
                                    std::time::SystemTime::now(),
                                    name, oid,
                                    self.interceptors.entry_count(),
                                    params,
                                );
                            } else {
                                let _ = writeln!(f,
                                    "[{:?}] unsolicited name={} oid={:?} interceptor_count={}",
                                    std::time::SystemTime::now(),
                                    name, oid,
                                    self.interceptors.entry_count(),
                                );
                            }
                        }
                    }
                    self.interceptors.dispatch(&name, oid, &params);

                    if !self.try_dispatch_passthrough_response(&raw_payload) {
                        eprintln!(
                            "[trace-resp] tid={:?} unsolicited name={name} await_len={await_len_before}",
                            std::thread::current().id()
                        );
                        let event = crate::host_io::runtime_events::RuntimeEvent::PassthroughResponse {
                            name,
                            params,
                        };
                        self.dispatch_runtime_event(event);
                    }
                }
            }
            crate::host_io::parser::DecodedFrame::Output { name, params } => {
                let event = crate::host_io::runtime_events::RuntimeEvent::lift(&name, params);
                self.dispatch_runtime_event(event);
            }
        }
        Ok(())
    }

    /// Try to dispatch a raw response payload through the passthrough router's
    /// notify table. Returns `true` if a pending passthrough notify consumed
    /// the response, `false` otherwise (caller should fall through to the
    /// runtime event dispatcher).
    fn try_dispatch_passthrough_response(&mut self, raw_payload: &[u8]) -> bool {
        if self.passthrough_notify_map.is_empty() {
            return false;
        }
        // FIFO: find the lowest seq with a pending notify.
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

    /// Handle a complete kalico-native frame surfaced by the demuxer.
    /// Routes responses to pending calls, identify-response to the
    /// identify caller, and events into [`event_dispatcher`].
    pub(crate) fn handle_kalico_frame(&mut self, channel: u8, payload: &[u8]) {
        match dispatch_kalico_frame(&mut self.kalico_state, channel, payload) {
            KalicoDispatchResult::Handled | KalicoDispatchResult::Ignored => {}
            KalicoDispatchResult::Event(ev) => {
                self.dispatch_runtime_event(ev);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Serial polling — spec §3.7.
// ---------------------------------------------------------------------------

impl Reactor {
    fn poll_serial(&mut self) {
        let t0 = std::time::Instant::now();
        let deadline = self.clock.now() + READ_TIMEOUT;
        let outcome = self.io.poll_frames_until(deadline);
        let dt = t0.elapsed();
        if dt > std::time::Duration::from_millis(5) {
            // Long polls indicate either a slow underlying read (host-side
            // kernel issue) or that the read itself blocks past its
            // intended deadline. The READ_TIMEOUT is ~1 ms; anything
            // beyond 5 ms is anomalous.
            let label: &'static str = match &outcome {
                Ok(PollOutcome::Frames { .. }) => "Frames",
                Ok(PollOutcome::Timeout) => "Timeout",
                Ok(PollOutcome::PhantomZero) => "PhantomZero",
                Err(_) => "Err",
            };
            eprintln!(
                "[trace-poll] dt_ms={:.2} outcome={label}",
                dt.as_secs_f64() * 1000.0
            );
        }
        match outcome {
            Ok(PollOutcome::Frames { frames, errors }) => {
                self.zero_byte_first_seen = None;
                // Any non-empty read counts as MCU activity. Even an
                // errors-only batch (CRC failures, malformed Klipper
                // frames) means the wire delivered bytes — the MCU is
                // alive, just talking imperfectly. Gates the
                // retry-exhaustion logic in `write_retransmit`.
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
            }
            Ok(PollOutcome::PhantomZero) => {
                let now = self.clock.now();
                let first = *self.zero_byte_first_seen.get_or_insert(now);
                if now.duration_since(first) >= ZERO_BYTE_DEBOUNCE {
                    eprintln!(
                        "[trace-close] poll_serial PhantomZero exceeded debounce kalico_pending={} await_n={} unacked_n={}",
                        self.kalico_state.pending.len(),
                        self.awaiting_response.len(),
                        self.unacked_window.len(),
                    );
                    log::warn!(
                        "port read returned Ok(0) for >= {ZERO_BYTE_DEBOUNCE:?}; transitioning to Closed"
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
                eprintln!(
                    "[trace-close] poll_serial Io error: {e:?} kalico_pending={} await_n={} unacked_n={}",
                    self.kalico_state.pending.len(),
                    self.awaiting_response.len(),
                    self.unacked_window.len(),
                );
                log::warn!("port read error: {e:?}; transitioning to Closed");
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

// ---------------------------------------------------------------------------
// Command dispatch — spec §3.7.
// ---------------------------------------------------------------------------

impl Reactor {
    /// Stage HostDisconnect + transition to Closed on a transport-level
    /// Io fault. Idempotent — won't overwrite an existing pending_host_fault.
    /// Used by the immediate-dispatch paths (Submit, FireAndForget,
    /// KalicoCall) and the RTO retransmit path to mirror the established
    /// drain-path / poll_serial behavior.
    pub(crate) fn transition_closed_on_io_fault(&mut self) {
        eprintln!(
            "[trace-close] transition_closed_on_io_fault (write-path Io error) state_was={:?} kalico_pending={} await_n={} unacked_n={}",
            self.state,
            self.kalico_state.pending.len(),
            self.awaiting_response.len(),
            self.unacked_window.len(),
        );
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
                {
                    use std::io::Write as _;
                    if let Ok(mut f) = std::fs::OpenOptions::new()
                        .create(true).append(true)
                        .open("/tmp/kalico-firewire.log")
                    {
                        let _ = writeln!(f,
                            "[diag-submit] SubmitTyped call_id={call_id} resp={expected_response_name} \
                             payload_len={} unacked={} pending_sub={} state={:?}",
                            payload.len(),
                            self.unacked_window.len(),
                            self.pending_submissions.len(),
                            self.state,
                        );
                    }
                }
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
                eprintln!(
                    "[trace-close] ReactorCommand::Shutdown received kalico_pending={} await_n={} unacked_n={}",
                    self.kalico_state.pending.len(),
                    self.awaiting_response.len(),
                    self.unacked_window.len(),
                );
                self.state = ReactorState::Closed;
                self.closed_via_shutdown = true;
            }
            ReactorCommand::MarkExpectedDisconnect => {
                // 2026-05-18: klippy's bridge-mode firmware_restart path sent
                // a `reset` command which is about to drop the MCU's USB-CDC.
                // Mark the eventual close as graceful so the spawn-time
                // EXIT_ON_FAULT guard sees `exited_gracefully() == true`
                // when the BrokenPipe arrives, and klippy can continue its
                // in-process restart without systemd having to interpose.
                eprintln!(
                    "[trace-close] ReactorCommand::MarkExpectedDisconnect \
                     received kalico_pending={} await_n={} unacked_n={}",
                    self.kalico_state.pending.len(),
                    self.awaiting_response.len(),
                    self.unacked_window.len(),
                );
                self.closed_via_shutdown = true;
            }
            ReactorCommand::AttachHeartbeatCallback(wrapper) => {
                self.event_dispatcher.heartbeat_callback = Some(wrapper.0);
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
                // The MCU handle is expected to already be claimed inside
                // the router by the bridge before sending this command.
                // For Phase 1 (one reactor per MCU), we peek at the first
                // MCU handle in the router.
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
            ReactorCommand::FireAndForget { cmd } => {
                // Diag 2026-05-19: write every FireAndForget event to a
                // dedicated trace file at /tmp/kalico-firewire.log because
                // log::info/eprintln stderr output is getting swallowed by
                // klippy's systemd wrapper. Append-only, line-per-event.
                use std::io::Write as _;
                let mut trace = std::fs::OpenOptions::new()
                    .create(true)
                    .append(true)
                    .open("/tmp/kalico-firewire.log")
                    .ok();
                match self.parser.encode(&cmd) {
                    Ok(payload) => {
                        let cmd_disp = if cmd.len() > 120 {
                            &cmd[..120]
                        } else {
                            cmd.as_str()
                        };
                        let head: Vec<String> = payload
                            .iter()
                            .take(16)
                            .map(|b| format!("{:02x}", b))
                            .collect();
                        if let Some(ref mut f) = trace {
                            let _ = writeln!(
                                f,
                                "[firewire] OK cmd=\"{}\" payload_len={} head=[{}]",
                                cmd_disp,
                                payload.len(),
                                head.join(",")
                            );
                        }
                        if let Err(e) = self.dispatch_fire_and_forget(payload) {
                            let is_io = matches!(e, TransportError::Io(_));
                            if let Some(ref mut f) = trace {
                                let _ = writeln!(
                                    f,
                                    "[firewire] dispatch_err cmd=\"{}\" err={}",
                                    cmd_disp, e
                                );
                            }
                            eprintln!("[bridge-error] FireAndForget send: {e}");
                            if is_io {
                                self.transition_closed_on_io_fault();
                            }
                        }
                    }
                    Err(e) => {
                        if let Some(ref mut f) = trace {
                            let _ =
                                writeln!(f, "[firewire] ENCODE_FAILED cmd=\"{}\" err={:?}", cmd, e);
                        }
                        eprintln!(
                            "[bridge-error] FireAndForget encode failed for cmd={cmd:?}: {e:?}"
                        );
                    }
                }
            }
            ReactorCommand::FireAndForgetTyped { payload } => {
                if let Err(e) = self.dispatch_fire_and_forget(payload) {
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
                // Bootstrap-ABI Identify: hand-encoded frame, no schema.
                let cid = self.kalico_state.allocate_correlation_id();
                let frame = build_kalico_identify_frame(cid);
                // Park the completion before writing to avoid losing a fast
                // response.
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
                eprintln!(
                    "[trace-kcall] entry channel={channel} kind={kind:?} body_len={} state={:?} identified={} pending_n={}",
                    body.len(),
                    self.state,
                    self.kalico_state.identified,
                    self.kalico_state.pending.len(),
                );
                if matches!(self.state, ReactorState::Closed) {
                    eprintln!(
                        "[trace-kcall] FAIL: reactor already Closed before write — completing with TransportError::Closed"
                    );
                    let _ = completion.send(Err(TransportError::Closed));
                    return;
                }
                if !self.kalico_state.identified {
                    eprintln!("[trace-kcall] FAIL: not identified");
                    let _ = completion.send(Err(TransportError::Parse(
                        "kalico transport not yet identified".into(),
                    )));
                    return;
                }
                let cid = self.kalico_state.allocate_correlation_id();
                let frame = build_kalico_frame(channel, kind, cid, &body);
                eprintln!(
                    "[trace-kcall] write_frame channel={channel} kind={kind:?} cid={cid} frame_len={}",
                    frame.len()
                );
                self.kalico_state.pending.insert(
                    cid,
                    PendingKalicoCall {
                        completion: completion.clone(),
                        deadline,
                    },
                );
                if let Err(e) = self.write_frame(&frame) {
                    eprintln!("[trace-kcall] write_frame ERROR cid={cid} kind={kind:?} err={e:?}");
                    let is_io = matches!(e, TransportError::Io(_));
                    if let Some(p) = self.kalico_state.pending.remove(&cid) {
                        let _ = p.completion.send(Err(e));
                    }
                    if is_io {
                        self.transition_closed_on_io_fault();
                    }
                } else {
                    eprintln!("[trace-kcall] write_frame OK cid={cid} kind={kind:?}");
                }
            }
            ReactorCommand::Noop => {
                // Liveness probe from `KalicoHostIo::is_alive`. Nothing to do.
            }
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

// ---------------------------------------------------------------------------
// Disconnect GC — spec §3.7.
// ---------------------------------------------------------------------------

impl Reactor {
    fn flush_all_completions(&mut self) {
        for entry in self.awaiting_response.drain_all() {
            let _ = entry.completion.send(Err(TransportError::Closed));
        }
        // Spec §3.11: clear UnackedWindow on transition to Closed. Pending
        // submissions also evicted with Closed so callers learn the channel
        // is dead rather than hanging on the rendezvous channel.
        self.unacked_window.clear();
        for p in self.pending_submissions.drain(..) {
            let _ = p.completion.send(Err(TransportError::Closed));
        }
        // Spec §6.0: pending fire-and-forget payloads have no caller to
        // notify; drop them on disconnect.
        self.pending_fire_and_forget.clear();
        self.pending_outbound_order.clear();
        self.passthrough_notify_map.clear();

        // Phase C-B: drop in-flight kalico calls + identify caller.
        let drained: Vec<PendingKalicoCall> =
            self.kalico_state.pending.drain().map(|(_, v)| v).collect();
        for p in drained {
            let _ = p.completion.send(Err(TransportError::Closed));
        }
        if let Some(c) = self.kalico_state.identify_pending.take() {
            let _ = c.send(Err(TransportError::Closed));
        }
    }

    /// GC kalico calls whose deadline has passed. The caller side already
    /// imposes its own `recv_timeout`, so this is belt-and-braces — keeps
    /// the `pending` map from growing if a caller stops waiting before the
    /// reactor times out.
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

// ---------------------------------------------------------------------------
// Main poll loop — spec §3.7.
// ---------------------------------------------------------------------------

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

    /// True iff the reactor reached `Closed` via the graceful
    /// `ReactorCommand::Shutdown` path (which `KalicoHostIo::drop` sends on
    /// process exit). False iff the transition was forced by an unexpected
    /// IO fault — in that case the caller (the thread spawn site in
    /// `KalicoHostIo::open_with_port`) aborts the process so klippy fails
    /// cleanly instead of pretending-to-be-up with a stale FD.
    pub fn exited_gracefully(&self) -> bool {
        self.closed_via_shutdown
    }

    /// One iteration of the reactor's main loop. Extracted from `run()` so
    /// tests can drive the reactor deterministically via the test harness
    /// (spec §2.4). Closed-state cleanup runs inside; on `TickOutcome::Closed`
    /// the next call must not be made (the loop in `run()` exits).
    pub fn tick_once(&mut self) -> TickOutcome {
        // Diag: tick_once duration. Long ticks (>5 ms) point at reactor
        // thread starvation independent of write_frame / poll_serial. Per-
        // step breakdown (drain_pending, poll_serial, drain_passthrough,
        // RTO step) helps isolate where the time went.
        let t_tick = std::time::Instant::now();

        // 1. Drain reactor commands (bounded per iteration).
        let s1 = std::time::Instant::now();
        for _ in 0..MAX_SUBMITS_PER_ITER {
            match self.submission_rx.try_recv() {
                Ok(cmd) => self.handle_command(cmd),
                Err(std::sync::mpsc::TryRecvError::Empty) => break,
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    eprintln!(
                        "[trace-close] submission_rx Disconnected — all senders dropped kalico_pending={} await_n={} unacked_n={}",
                        self.kalico_state.pending.len(),
                        self.awaiting_response.len(),
                        self.unacked_window.len(),
                    );
                    self.state = ReactorState::Closed;
                    break;
                }
            }
        }

        let t_step1 = s1.elapsed();

        // 2. Poll serial port.
        let s2 = std::time::Instant::now();
        self.poll_serial();
        let t_step2 = s2.elapsed();

        // 3. Drain pending submissions (ack in step 2 may have freed window slots).
        let s3 = std::time::Instant::now();
        self.drain_pending_submissions();
        let t_step3 = s3.elapsed();

        // 3b. Drain passthrough entries from the router onto the wire.
        let s3b = std::time::Instant::now();
        self.drain_passthrough();
        let t_step3b = s3b.elapsed();

        // 4. RTO timer step. On Io error from write_retransmit, escalate
        // to Closed (mirrors drain-path / poll_serial / dispatch_submission
        // Io-error handling). See Finding 6 in
        // docs/superpowers/specs/2026-05-09-bridge-call-stall-investigation.md.
        let s4 = std::time::Instant::now();
        if let Some(front) = self.unacked_window.front() {
            let now = self.clock.now();
            if now >= front.sent_at + self.rtt.current_rto() {
                let unacked_n = self.unacked_window.len();
                let front_seq = front.seq;
                if let Err(e) = self.write_retransmit(RetransmitTrigger::TimeoutDriven) {
                    eprintln!(
                        "[trace-rto] retransmit error front_seq={front_seq} unacked_n={unacked_n} err={e:?}"
                    );
                    if matches!(e, TransportError::Io(_)) {
                        log::warn!("retransmit Io error: {e:?}; transitioning Closed");
                        self.transition_closed_on_io_fault();
                    }
                }
            }
        }
        let t_step4 = s4.elapsed();

        // 4b. Drain staged host fault into the FaultLatch.
        if let Some(fault) = self.pending_host_fault.take() {
            self.event_dispatcher.fault_latch.dispatch(fault);
        }

        // 4c. Forward any TraceRing host-event diagnostics queued in the
        //     shared inbox to the host-event subscriber.
        self.event_dispatcher.host_event_dispatcher.drain_pending();

        // 5. AwaitingResponse GC (layer 2 — per-entry deadline).
        let now = self.clock.now();
        let evicted = self.awaiting_response.evict_expired(now);
        for entry in evicted {
            let _ = entry
                .completion
                .send(Err(TransportError::DispatcherTimeout));
        }

        // 5b. Phase C-B: GC expired kalico calls.
        self.gc_kalico_pending();

        // 6. Closed-state exit.
        if self.state == ReactorState::Closed {
            self.flush_all_completions();
            return TickOutcome::Closed;
        }

        let dt_tick = t_tick.elapsed();
        if dt_tick > std::time::Duration::from_millis(5) {
            eprintln!(
                "[trace-tick] dt_ms={:.2} step1={:.2} step2={:.2} step3={:.2} step3b={:.2} step4={:.2}",
                dt_tick.as_secs_f64() * 1000.0,
                t_step1.as_secs_f64() * 1000.0,
                t_step2.as_secs_f64() * 1000.0,
                t_step3.as_secs_f64() * 1000.0,
                t_step3b.as_secs_f64() * 1000.0,
                t_step4.as_secs_f64() * 1000.0
            );
        }
        TickOutcome::Continue
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests;

// ---------------------------------------------------------------------------
// A1 — seq-wrap boundaries. Spec §3.1.
// Three boundaries: empty-window snap, mid-range mod-16, near u64::MAX.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a1_seq_wrap;

// ---------------------------------------------------------------------------
// A2 — NAK/RTO branches. Spec §3.2.
// Six sub-tests, one per branch.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a2_nak_rto;

// ---------------------------------------------------------------------------
// A4 — NAK + submit same-tick race consistency. Spec §3.4.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a4_nak_submit_race;

// ---------------------------------------------------------------------------
// A3 — AwaitingResponse three-layer GC. Spec §3.3.
// Three sub-tests, one per GC layer.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a3_awaiting_response_gc;

// ---------------------------------------------------------------------------
// A5 — Passthrough queue reactor integration. Task 17.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a5_passthrough_integration;

// ---------------------------------------------------------------------------
// A8 — Backpressure-respecting fire-and-forget. Spec §6.0 of
// `2026-05-04-incremental-curve-upload-design.md`.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a8_fire_and_forget_backpressure;

// ---------------------------------------------------------------------------
// FireAndForgetTyped routing — Step 2 of incremental-curve-upload spec.
// Validates ReactorCommand::FireAndForgetTyped is routed through
// dispatch_fire_and_forget and lands on the wire.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod fire_and_forget_typed_routing;

// ---------------------------------------------------------------------------
// Io-fault propagation tests. Every code path that calls write_frame must
// transition to Closed + stage HostDisconnect on Io error, mirroring the
// behavior of drain_pending_submissions / drain_passthrough / poll_serial.
//
// Pre-fix (Finding 1 in 2026-05-09 bridge-call stall investigation), the
// IMMEDIATE-dispatch paths in handle_command (Submit / SubmitTyped /
// FireAndForget / FireAndForgetTyped / KalicoCall / KalicoIdentify) and
// the RTO retransmit path silently swallowed Io errors — completion got
// the error but reactor stayed Active. This module asserts the FIXED
// behavior: every Io-faulting path transitions to Closed.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod io_fault_propagation;
