//! Single-thread poll-reactor. Spec §3.7.

use std::collections::VecDeque;
use std::io::Read;
use std::sync::mpsc::Receiver;
use std::sync::Arc;
use std::time::Instant;

use arc_swap::ArcSwap;

use crate::clock::{Clock, RealClock};
use crate::host_io::ReactorCommand;
use crate::host_io::events::EventDispatcher;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::rtt::RttEstimator;
use crate::host_io::runtime_events::{FaultEvent, StatusEvent};
use crate::host_io::window::{UnackedWindow, AwaitingResponse};
use crate::transport::TransportError;
use runtime::error::FaultCode;

pub struct Reactor {
    pub(crate) port:               Box<dyn serialport::SerialPort>,
    pub(crate) parser:             Arc<MsgProtoParser>,
    pub(crate) submission_rx:      Receiver<ReactorCommand>,
    pub(crate) unacked_window:     UnackedWindow,
    pub(crate) awaiting_response:  AwaitingResponse,
    pub(crate) rtt:                RttEstimator,
    pub(crate) rx_buf:             Vec<u8>,
    pub(crate) status_snapshot:    Arc<ArcSwap<StatusEvent>>,
    pub(crate) event_dispatcher:   EventDispatcher,

    // 64-bit absolute sequence counters. Per spec §3.1 / serialqueue.c:660-666.
    pub(crate) send_seq:           u64,
    pub(crate) receive_seq:        u64,
    pub(crate) last_ack_seq:       u64,
    pub(crate) ignore_nak_seq:     u64,
    pub(crate) retransmit_seq:     u64,
    pub(crate) rtt_sample_seq:     u64,
    pub(crate) rtt_sample_armed:   bool,

    pub(crate) state: ReactorState,

    pub(crate) pending_host_fault: Option<FaultEvent>,

    pub(crate) pending_submissions: VecDeque<PendingSubmission>,

    /// First-observed instant of a phantom `Ok(0)` from `port.read`.
    /// Per spec §3.11, treat as Closed only if it persists past
    /// `ZERO_BYTE_DEBOUNCE`. Cleared on any non-zero read.
    pub(crate) zero_byte_first_seen: Option<Instant>,

    /// Injected clock seam (spec §2.3). Routes `Instant::now()` so tests
    /// can deterministically advance time via `MockClock`.
    pub(crate) clock: Arc<dyn Clock>,
}

pub(crate) struct PendingSubmission {
    pub call_id:                u64,
    pub payload:                Vec<u8>,
    pub expected_response_name: String,
    pub completion:             std::sync::mpsc::SyncSender<Result<crate::transport::MessageParams, TransportError>>,
    pub deadline:               Instant,
}

#[derive(Debug, PartialEq, Eq)]
pub enum ReactorState {
    Active,
    Closed,
}

impl Reactor {
    pub fn new(
        port: Box<dyn serialport::SerialPort>,
        parser: Arc<MsgProtoParser>,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        rx_buf_initial: Vec<u8>,
        config: crate::host_io::KalicoHostIoConfig,
    ) -> Self {
        Self::new_with_clock(
            port, parser, submission_rx, status_snapshot,
            rx_buf_initial, config, Arc::new(RealClock),
        )
    }

    pub fn new_with_clock(
        port: Box<dyn serialport::SerialPort>,
        parser: Arc<MsgProtoParser>,
        submission_rx: Receiver<ReactorCommand>,
        status_snapshot: Arc<ArcSwap<StatusEvent>>,
        rx_buf_initial: Vec<u8>,
        config: crate::host_io::KalicoHostIoConfig,
        clock: Arc<dyn Clock>,
    ) -> Self {
        let event_dispatcher = EventDispatcher::new(
            Arc::clone(&status_snapshot),
            config.trace_capacity,
            config.host_event_capacity,
        );
        Self {
            port,
            parser,
            submission_rx,
            unacked_window: UnackedWindow::default(),
            awaiting_response: AwaitingResponse::default(),
            rtt: RttEstimator::default(),
            rx_buf: rx_buf_initial,
            status_snapshot,
            event_dispatcher,
            send_seq: 1,
            receive_seq: 1,
            last_ack_seq: 0,
            ignore_nak_seq: 0,
            retransmit_seq: 0,
            rtt_sample_seq: 0,
            rtt_sample_armed: false,
            state: ReactorState::Active,
            pending_host_fault: None,
            pending_submissions: VecDeque::new(),
            zero_byte_first_seen: None,
            clock,
        }
    }

    /// Single chokepoint for all wire writes. Per spec §3.7.
    pub(crate) fn write_frame(&mut self, frame: &[u8]) -> Result<(), TransportError> {
        self.port.write_all(frame).map_err(TransportError::Io)?;
        self.port.flush().map_err(TransportError::Io)?;
        Ok(())
    }
}

/// Why a retransmit was triggered. C20 uses this to select the retransmit arm.
#[derive(Debug, Clone, Copy)]
pub enum RetransmitTrigger {
    NakDriven,
    TimeoutDriven,
}

const PENDING_SUBMISSION_CEILING: usize = 256;
const MAX_RETRY_COUNT: u32 = 8;

const MAX_SUBMITS_PER_ITER: usize = 4;
const READ_TIMEOUT: std::time::Duration = std::time::Duration::from_millis(100);
const ZERO_BYTE_DEBOUNCE: std::time::Duration = std::time::Duration::from_millis(100);

impl Reactor {
    pub(crate) fn dispatch_submission(
        &mut self,
        call_id: u64,
        payload: Vec<u8>,
        expected_response_name: String,
        completion: std::sync::mpsc::SyncSender<Result<crate::transport::MessageParams, TransportError>>,
        deadline: Instant,
    ) -> Result<(), TransportError> {
        if self.unacked_window.is_full() {
            if self.pending_submissions.len() >= PENDING_SUBMISSION_CEILING {
                let _ = completion.send(Err(TransportError::Backpressure));
                return Ok(());
            }
            self.pending_submissions.push_back(PendingSubmission {
                call_id, payload, expected_response_name, completion, deadline,
            });
            return Ok(());
        }

        let seq = self.send_seq;
        self.send_seq += 1;
        let wire_seq = (seq & 0x0F) as u8;
        let frame = crate::host_io::wire::build_frame(&payload, wire_seq);

        self.write_frame(&frame)?;

        let now = self.clock.now();
        self.unacked_window.push(crate::host_io::window::UnackedEntry {
            seq, frame_bytes: frame, sent_at: now, retry_count: 0,
        });
        self.awaiting_response.push(crate::host_io::window::AwaitEntry {
            call_id, seq,
            expected_response_name,
            completion,
            submitted_at: now,
            deadline,
            abandoned: false,
        })?;

        if !self.rtt_sample_armed {
            self.rtt_sample_seq = seq;
            self.rtt_sample_armed = true;
        }
        Ok(())
    }

    pub(crate) fn drain_pending_submissions(&mut self) {
        while !self.unacked_window.is_full() {
            let Some(p) = self.pending_submissions.pop_front() else { break; };
            let completion = p.completion.clone();
            if let Err(e) = self.dispatch_submission(
                p.call_id, p.payload, p.expected_response_name, completion, p.deadline,
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
                    if self.pending_host_fault.is_none() {
                        self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                            fault_code:   FaultCode::HostDisconnect.as_u16(),
                            fault_detail: 0,
                            segment_id:   0,
                            synthesized:  false,
                        });
                    }
                    self.state = ReactorState::Closed;
                    return;
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // Wire-protocol ack/nak handling — spec §3.5 (Codex finding #1 corrected).
    // -------------------------------------------------------------------------

    /// Reconstruct an absolute 64-bit seq from the 4-bit wire nibble.
    ///
    /// The wire nibble is the low 4 bits of the MCU's receive_seq.
    /// Outstanding window ≤ 12 << 16, so one nibble-mod-16 delta suffices.
    fn decode_absolute(&self, wire_seq: u8) -> u64 {
        let delta = (u64::from(wire_seq).wrapping_sub(self.receive_seq)) & 0x0F;
        self.receive_seq + delta
    }

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
        let rseq = self.decode_absolute(wire_seq_nibble);

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

    pub(crate) fn write_retransmit(&mut self, trigger: RetransmitTrigger) -> Result<(), TransportError> {
        // Build retransmit buffer: leading SYNC + all unacked frames.
        let buf = {
            let frames: Vec<&[u8]> = self.unacked_window.iter()
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

        // Retry cap: increment all; fault on exhaustion.
        for entry in self.unacked_window.iter_mut() {
            entry.retry_count += 1;
            if entry.retry_count >= MAX_RETRY_COUNT {
                self.state = ReactorState::Closed;
                self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                    fault_code:   FaultCode::HostRetransmitExhausted.as_u16(),
                    fault_detail: entry.retry_count,
                    segment_id:   0,
                    synthesized:  false,
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
    pub(crate) fn handle_inbound_frame(&mut self, packet: Vec<u8>) -> Result<(), TransportError> {
        if packet.len() < crate::host_io::wire::MESSAGE_MIN {
            return Ok(());
        }
        let wire_seq_nibble = packet[1] & 0x0F;
        if packet.len() == crate::host_io::wire::MESSAGE_MIN {
            // 5-byte ack/nak frame.
            self.handle_ack_nak(wire_seq_nibble)?;
            return Ok(());
        }
        // Real msg-id frame — advance receive_seq if needed.
        let rseq = self.decode_absolute(wire_seq_nibble);
        if rseq != self.receive_seq {
            self.update_receive_seq(rseq)?;
        }
        // Parse + dispatch. Decode errors are warn-logged and the frame is dropped
        // (not propagated as Closed) — dictionary version skew is recoverable.
        let decoded = match self.parser.decode(&packet) {
            Ok(d) => d,
            Err(e) => {
                log::warn!("decode error on inbound frame: {e:?}; dropping");
                return Ok(());
            }
        };
        match decoded {
            crate::host_io::parser::DecodedFrame::Response { name, params } => {
                if let Some(idx) = self.awaiting_response.find_match(&name) {
                    let entry = self.awaiting_response.remove(idx);
                    let _ = entry.completion.send(Ok(params));
                } else {
                    let event = crate::host_io::runtime_events::RuntimeEvent::lift(&name, params);
                    self.dispatch_runtime_event(event);
                }
            }
            crate::host_io::parser::DecodedFrame::Output { name, params } => {
                let event = crate::host_io::runtime_events::RuntimeEvent::lift(&name, params);
                self.dispatch_runtime_event(event);
            }
        }
        Ok(())
    }

    fn dispatch_runtime_event(&mut self, event: crate::host_io::runtime_events::RuntimeEvent) {
        self.event_dispatcher.dispatch(event);
    }
}

// ---------------------------------------------------------------------------
// Serial polling — spec §3.7.
// ---------------------------------------------------------------------------

impl Reactor {
    fn poll_serial(&mut self) {
        let mut scratch = [0u8; 256];
        if self.port.set_timeout(READ_TIMEOUT).is_err() {
            return;
        }
        match self.port.read(&mut scratch) {
            Ok(n) if n > 0 => {
                self.zero_byte_first_seen = None;
                self.rx_buf.extend_from_slice(&scratch[..n]);
                while let Some(packet) = crate::host_io::wire::extract_packet(&mut self.rx_buf) {
                    if self.handle_inbound_frame(packet).is_err() {
                        return; // Closed — run loop will see state and exit.
                    }
                }
            }
            Ok(_) => {
                // Spec §3.11: phantom Ok(0) is rare on USB-CDC; if it persists
                // past ZERO_BYTE_DEBOUNCE we treat the port as disconnected.
                let now = self.clock.now();
                let first = *self.zero_byte_first_seen.get_or_insert(now);
                if now.duration_since(first) >= ZERO_BYTE_DEBOUNCE {
                    log::warn!("port read returned Ok(0) for >= {ZERO_BYTE_DEBOUNCE:?}; transitioning to Closed");
                    self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                        fault_code:   FaultCode::HostDisconnect.as_u16(),
                        fault_detail: 0,
                        segment_id:   0,
                        synthesized:  false,
                    });
                    self.state = ReactorState::Closed;
                }
            }
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) if e.kind() == std::io::ErrorKind::Interrupted => {}
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(e) => {
                log::warn!("port read error: {e:?}; transitioning to Closed");
                self.pending_host_fault = Some(crate::host_io::runtime_events::FaultEvent {
                    fault_code:   FaultCode::HostDisconnect.as_u16(),
                    fault_detail: 0,
                    segment_id:   0,
                    synthesized:  false,
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
    fn handle_command(&mut self, cmd: crate::host_io::ReactorCommand) {
        use crate::host_io::ReactorCommand;
        match cmd {
            ReactorCommand::Submit { call_id, cmd, expected_response_name, completion, deadline } => {
                match self.parser.encode(&cmd) {
                    Ok(payload) => {
                        if let Err(e) = self.dispatch_submission(
                            call_id, payload, expected_response_name, completion.clone(), deadline,
                        ) {
                            let _ = completion.send(Err(e));
                        }
                    }
                    Err(e) => {
                        let _ = completion.send(Err(TransportError::Parse(format!("{e:?}"))));
                    }
                }
            }
            ReactorCommand::SubmitTyped { call_id, payload, expected_response_name, completion, deadline } => {
                if let Err(e) = self.dispatch_submission(
                    call_id, payload, expected_response_name, completion.clone(), deadline,
                ) {
                    let _ = completion.send(Err(e));
                }
            }
            ReactorCommand::Abandon(call_id) => {
                self.awaiting_response.mark_abandoned(call_id);
            }
            ReactorCommand::Shutdown => {
                self.state = ReactorState::Closed;
            }
            ReactorCommand::AttachCreditCounter(counter) => {
                self.event_dispatcher.credit_counter = Some(counter);
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
                let result = self.event_dispatcher.runtime_event_dispatcher.subscribe(sender);
                let _ = reply.send(result);
            }
            ReactorCommand::SubscribeHostEvents { sender, reply } => {
                let result = self.event_dispatcher.host_event_dispatcher.subscribe(sender);
                let _ = reply.send(result);
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
    }
}

// ---------------------------------------------------------------------------
// Main poll loop — spec §3.7.
// ---------------------------------------------------------------------------

#[derive(Debug, PartialEq, Eq)]
pub(crate) enum TickOutcome {
    Continue,
    Closed,
}

impl Reactor {
    pub fn run(&mut self) {
        loop {
            if matches!(self.tick_once(), TickOutcome::Closed) { break; }
        }
    }

    /// One iteration of the reactor's main loop. Extracted from `run()` so
    /// tests can drive the reactor deterministically via the test harness
    /// (spec §2.4). Closed-state cleanup runs inside; on `TickOutcome::Closed`
    /// the next call must not be made (the loop in `run()` exits).
    pub(crate) fn tick_once(&mut self) -> TickOutcome {
        // 1. Drain reactor commands (bounded per iteration).
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

        // 2. Poll serial port.
        self.poll_serial();

        // 3. Drain pending submissions (ack in step 2 may have freed window slots).
        self.drain_pending_submissions();

        // 4. RTO timer step.
        if let Some(front) = self.unacked_window.front() {
            let now = self.clock.now();
            if now >= front.sent_at + self.rtt.current_rto() {
                let _ = self.write_retransmit(RetransmitTrigger::TimeoutDriven);
            }
        }

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
            let _ = entry.completion.send(Err(TransportError::DispatcherTimeout));
        }

        // 6. Closed-state exit.
        if self.state == ReactorState::Closed {
            self.flush_all_completions();
            return TickOutcome::Closed;
        }
        TickOutcome::Continue
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};

    // -----------------------------------------------------------------------
    // MockPort: a SerialPort that reads TimedOut and captures writes.
    // -----------------------------------------------------------------------

    struct MockPort {
        written: Arc<Mutex<Vec<u8>>>,
    }

    impl std::io::Read for MockPort {
        fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
            Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "mock"))
        }
    }

    impl std::io::Write for MockPort {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
            self.written.lock().unwrap().extend_from_slice(buf);
            Ok(buf.len())
        }
        fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
    }

    impl serialport::SerialPort for MockPort {
        fn name(&self) -> Option<String> { Some("mock".into()) }
        fn baud_rate(&self) -> serialport::Result<u32> { Ok(115_200) }
        fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
            Ok(serialport::DataBits::Eight)
        }
        fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
            Ok(serialport::FlowControl::None)
        }
        fn parity(&self) -> serialport::Result<serialport::Parity> {
            Ok(serialport::Parity::None)
        }
        fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
            Ok(serialport::StopBits::One)
        }
        fn timeout(&self) -> std::time::Duration { std::time::Duration::from_millis(1) }
        fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> { Ok(()) }
        fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> { Ok(()) }
        fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> { Ok(()) }
        fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> { Ok(()) }
        fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> { Ok(()) }
        fn set_timeout(&mut self, _: std::time::Duration) -> serialport::Result<()> { Ok(()) }
        fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> { Ok(()) }
        fn read_data_set_ready(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn bytes_to_read(&self) -> serialport::Result<u32> { Ok(0) }
        fn bytes_to_write(&self) -> serialport::Result<u32> { Ok(0) }
        fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> { Ok(()) }
        fn try_clone(&self) -> serialport::Result<Box<dyn serialport::SerialPort>> {
            Err(serialport::Error::new(serialport::ErrorKind::Unknown, "mock: try_clone unsupported"))
        }
        fn set_break(&self) -> serialport::Result<()> { Ok(()) }
        fn clear_break(&self) -> serialport::Result<()> { Ok(()) }
        fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> { Ok(()) }
        fn read_clear_to_send(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn read_ring_indicator(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn read_carrier_detect(&mut self) -> serialport::Result<bool> { Ok(false) }
    }

    // -----------------------------------------------------------------------
    // Helper: build a Reactor with the given seqs pre-populated in the window.
    // -----------------------------------------------------------------------

    fn test_reactor_with_inflight(seqs: &[u64]) -> (Reactor, Arc<Mutex<Vec<u8>>>) {
        let written = Arc::new(Mutex::new(Vec::<u8>::new()));
        let port = MockPort { written: Arc::clone(&written) };

        // Build a minimal MsgProtoParser (empty data dict is fine for these tests).
        let parser = Arc::new(crate::host_io::parser::MsgProtoParser::new_empty());

        let (_, rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(arc_swap::ArcSwap::from_pointee(
            crate::host_io::runtime_events::StatusEvent::default(),
        ));

        let mut reactor = Reactor::new(
            Box::new(port), parser, rx, status_snapshot, Vec::new(),
            crate::host_io::KalicoHostIoConfig::default(),
        );

        // Pre-populate the unacked window.
        let max_seq = seqs.iter().copied().max().unwrap_or(0);
        for &seq in seqs {
            reactor.unacked_window.push(crate::host_io::window::UnackedEntry {
                seq,
                frame_bytes: vec![],
                sent_at: std::time::Instant::now(),
                retry_count: 0,
            });
        }
        if max_seq > 0 {
            reactor.send_seq = max_seq + 1;
        }
        // receive_seq=1, last_ack_seq=0 are the Reactor::new defaults.

        (reactor, written)
    }

    // -----------------------------------------------------------------------
    // Test 1 — decode_absolute wraps correctly.
    // -----------------------------------------------------------------------

    #[test]
    fn decode_absolute_wraps_correctly() {
        let (reactor, _) = test_reactor_with_inflight(&[]);
        // receive_seq = 1 (default). Wire nibble 0x02 → delta = (2 - 1) & 0x0F = 1 → abs = 2.
        assert_eq!(reactor.decode_absolute(0x02), 2);

        // Simulate receive_seq = 14.
        let mut r2 = test_reactor_with_inflight(&[]).0;
        r2.receive_seq = 14;
        // Wire nibble 0x01 → delta = (1 - 14) & 0x0F = (-13 mod 16) = 3 → abs = 14 + 3 = 17.
        assert_eq!(r2.decode_absolute(0x01), 17);
    }

    // -----------------------------------------------------------------------
    // Test 2 — forward-progress ack updates last_ack_seq.
    // -----------------------------------------------------------------------

    #[test]
    fn forward_progress_ack_updates_last_ack_seq() {
        // One in-flight entry with seq=2; receive_seq=1, last_ack_seq=0.
        let (mut reactor, _written) = test_reactor_with_inflight(&[2]);

        reactor.handle_ack_nak(0x02).expect("handle_ack_nak");
        assert_eq!(reactor.last_ack_seq, 2);
    }

    // -----------------------------------------------------------------------
    // Test 3 — duplicate ack triggers retransmit.
    // -----------------------------------------------------------------------

    #[test]
    fn duplicate_ack_triggers_retransmit() {
        // Window: seqs=[1, 2].  receive_seq=1, last_ack_seq=0.
        let (mut reactor, written) = test_reactor_with_inflight(&[1, 2]);

        // First call: rseq=2 → forward progress, last_ack_seq=2, pops seq=1.
        reactor.handle_ack_nak(0x02).expect("first handle_ack_nak");
        assert_eq!(reactor.last_ack_seq, 2);

        let bytes_before = written.lock().unwrap().len();

        // Second call: rseq=2 again → duplicate ack → retransmit should fire.
        reactor.handle_ack_nak(0x02).expect("second handle_ack_nak");

        let bytes_after = written.lock().unwrap().len();
        assert!(
            bytes_after > bytes_before,
            "duplicate ack must trigger retransmit (write buffer grew: {bytes_before} → {bytes_after})"
        );
    }

    // -----------------------------------------------------------------------
    // Test 4 — stale ack damped by ignore_nak_seq.
    // -----------------------------------------------------------------------

    #[test]
    fn stale_ack_damped_by_ignore_nak_seq() {
        // Window: seqs=[1, 2]; ignore_nak_seq=10 (high sentinel — damps retransmit).
        let (mut reactor, written) = test_reactor_with_inflight(&[1, 2]);
        reactor.ignore_nak_seq = 10;

        // First call: rseq=2 → forward progress, last_ack_seq=2.
        reactor.handle_ack_nak(0x02).expect("first handle_ack_nak");

        let bytes_before = written.lock().unwrap().len();

        // Second call: rseq=2 again.  rseq(2) > ignore_nak_seq(10) is FALSE → no retransmit.
        reactor.handle_ack_nak(0x02).expect("second handle_ack_nak");

        let bytes_after = written.lock().unwrap().len();
        assert_eq!(
            bytes_before, bytes_after,
            "ignore_nak_seq damps retransmit: write buffer must not grow"
        );
    }

    // -----------------------------------------------------------------------
    // Tests 5–9 — write_retransmit two-arm logic.
    // -----------------------------------------------------------------------

    #[test]
    fn nak_driven_sets_ignore_nak_to_receive_seq() {
        let (mut reactor, _port) = test_reactor_with_inflight(&[1, 2, 3]);
        reactor.receive_seq = 5;
        reactor.retransmit_seq = 0; // receive_seq >= retransmit_seq → arm 1
        reactor.write_retransmit(RetransmitTrigger::NakDriven).unwrap();
        assert_eq!(reactor.ignore_nak_seq, 5); // = receive_seq
    }

    #[test]
    fn second_nak_uses_retransmit_seq() {
        let (mut reactor, _port) = test_reactor_with_inflight(&[1, 2, 3]);
        reactor.receive_seq = 3;
        reactor.retransmit_seq = 7; // receive_seq < retransmit_seq → arm 2
        reactor.write_retransmit(RetransmitTrigger::NakDriven).unwrap();
        assert_eq!(reactor.ignore_nak_seq, 7); // = retransmit_seq
    }

    #[test]
    fn timeout_driven_sets_ignore_nak_to_send_seq() {
        let (mut reactor, _port) = test_reactor_with_inflight(&[1, 2, 3]);
        reactor.send_seq = 10;
        reactor.write_retransmit(RetransmitTrigger::TimeoutDriven).unwrap();
        assert_eq!(reactor.ignore_nak_seq, 10); // = send_seq
    }

    #[test]
    fn nak_driven_does_not_back_off_rto() {
        let (mut reactor, _port) = test_reactor_with_inflight(&[1]);
        let rto_before = reactor.rtt.current_rto();
        reactor.write_retransmit(RetransmitTrigger::NakDriven).unwrap();
        assert_eq!(reactor.rtt.current_rto(), rto_before);
    }

    #[test]
    fn timeout_driven_doubles_rto() {
        let (mut reactor, _port) = test_reactor_with_inflight(&[1]);
        let rto_before = reactor.rtt.current_rto();
        reactor.write_retransmit(RetransmitTrigger::TimeoutDriven).unwrap();
        assert!(reactor.rtt.current_rto() >= rto_before * 2);
    }

    // -----------------------------------------------------------------------
    // BrokenPipePort: a SerialPort that returns BrokenPipe from read.
    // -----------------------------------------------------------------------

    struct BrokenPipePort;

    impl std::io::Read for BrokenPipePort {
        fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
            Err(std::io::Error::new(std::io::ErrorKind::BrokenPipe, "mock disconnect"))
        }
    }

    impl std::io::Write for BrokenPipePort {
        fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> { Ok(buf.len()) }
        fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
    }

    impl serialport::SerialPort for BrokenPipePort {
        fn name(&self) -> Option<String> { Some("broken-pipe-mock".into()) }
        fn baud_rate(&self) -> serialport::Result<u32> { Ok(115_200) }
        fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
            Ok(serialport::DataBits::Eight)
        }
        fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
            Ok(serialport::FlowControl::None)
        }
        fn parity(&self) -> serialport::Result<serialport::Parity> {
            Ok(serialport::Parity::None)
        }
        fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
            Ok(serialport::StopBits::One)
        }
        fn timeout(&self) -> std::time::Duration { std::time::Duration::from_millis(1) }
        fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> { Ok(()) }
        fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> { Ok(()) }
        fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> { Ok(()) }
        fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> { Ok(()) }
        fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> { Ok(()) }
        fn set_timeout(&mut self, _: std::time::Duration) -> serialport::Result<()> { Ok(()) }
        fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> { Ok(()) }
        fn read_data_set_ready(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn bytes_to_read(&self) -> serialport::Result<u32> { Ok(0) }
        fn bytes_to_write(&self) -> serialport::Result<u32> { Ok(0) }
        fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> { Ok(()) }
        fn try_clone(&self) -> serialport::Result<Box<dyn serialport::SerialPort>> {
            Err(serialport::Error::new(serialport::ErrorKind::Unknown, "mock: try_clone unsupported"))
        }
        fn set_break(&self) -> serialport::Result<()> { Ok(()) }
        fn clear_break(&self) -> serialport::Result<()> { Ok(()) }
        fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> { Ok(()) }
        fn read_clear_to_send(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn read_ring_indicator(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn read_carrier_detect(&mut self) -> serialport::Result<bool> { Ok(false) }
    }

    // -----------------------------------------------------------------------
    // BrokenWritePort: a SerialPort whose writes fail with BrokenPipe.
    // -----------------------------------------------------------------------

    struct BrokenWritePort;

    impl std::io::Read for BrokenWritePort {
        fn read(&mut self, _buf: &mut [u8]) -> std::io::Result<usize> {
            Err(std::io::Error::new(std::io::ErrorKind::TimedOut, "mock"))
        }
    }
    impl std::io::Write for BrokenWritePort {
        fn write(&mut self, _buf: &[u8]) -> std::io::Result<usize> {
            Err(std::io::Error::new(std::io::ErrorKind::BrokenPipe, "mock write fail"))
        }
        fn flush(&mut self) -> std::io::Result<()> { Ok(()) }
    }
    impl serialport::SerialPort for BrokenWritePort {
        fn name(&self) -> Option<String> { Some("broken-write-mock".into()) }
        fn baud_rate(&self) -> serialport::Result<u32> { Ok(115_200) }
        fn data_bits(&self) -> serialport::Result<serialport::DataBits> { Ok(serialport::DataBits::Eight) }
        fn flow_control(&self) -> serialport::Result<serialport::FlowControl> { Ok(serialport::FlowControl::None) }
        fn parity(&self) -> serialport::Result<serialport::Parity> { Ok(serialport::Parity::None) }
        fn stop_bits(&self) -> serialport::Result<serialport::StopBits> { Ok(serialport::StopBits::One) }
        fn timeout(&self) -> std::time::Duration { std::time::Duration::from_millis(1) }
        fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> { Ok(()) }
        fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> { Ok(()) }
        fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> { Ok(()) }
        fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> { Ok(()) }
        fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> { Ok(()) }
        fn set_timeout(&mut self, _: std::time::Duration) -> serialport::Result<()> { Ok(()) }
        fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> { Ok(()) }
        fn read_data_set_ready(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn bytes_to_read(&self) -> serialport::Result<u32> { Ok(0) }
        fn bytes_to_write(&self) -> serialport::Result<u32> { Ok(0) }
        fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> { Ok(()) }
        fn try_clone(&self) -> serialport::Result<Box<dyn serialport::SerialPort>> {
            Err(serialport::Error::new(serialport::ErrorKind::Unknown, "mock: try_clone unsupported"))
        }
        fn set_break(&self) -> serialport::Result<()> { Ok(()) }
        fn clear_break(&self) -> serialport::Result<()> { Ok(()) }
        fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> { Ok(()) }
        fn read_clear_to_send(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn read_ring_indicator(&mut self) -> serialport::Result<bool> { Ok(false) }
        fn read_carrier_detect(&mut self) -> serialport::Result<bool> { Ok(false) }
    }

    // -----------------------------------------------------------------------
    // Test: drain_pending_submissions surfaces write errors to the caller
    // (Codex finding) — completion sees TransportError::Io, host fault is
    // staged, and the reactor transitions to Closed.
    // -----------------------------------------------------------------------

    #[test]
    fn drain_pending_surfaces_write_failure() {
        let (_, rx) = std::sync::mpsc::channel::<crate::host_io::ReactorCommand>();
        let status_snapshot = Arc::new(arc_swap::ArcSwap::from_pointee(
            crate::host_io::runtime_events::StatusEvent::default(),
        ));
        let parser = Arc::new(crate::host_io::parser::MsgProtoParser::new_empty());
        let mut reactor = Reactor::new(
            Box::new(BrokenWritePort), parser, rx, status_snapshot, Vec::new(),
            crate::host_io::KalicoHostIoConfig::default(),
        );

        // Queue one pending submission. unacked_window is empty so the
        // drain loop will pop it and try to dispatch immediately.
        let (tx, completion_rx) =
            std::sync::mpsc::sync_channel::<Result<crate::transport::MessageParams, TransportError>>(1);
        reactor.pending_submissions.push_back(PendingSubmission {
            call_id: 7,
            payload: vec![0xAA, 0xBB],
            expected_response_name: "noop".into(),
            completion: tx,
            deadline: Instant::now() + std::time::Duration::from_secs(1),
        });

        reactor.drain_pending_submissions();

        let received = completion_rx.try_recv().expect("completion must be signaled");
        match received {
            Err(TransportError::Io(e)) => assert_eq!(e.kind(), std::io::ErrorKind::BrokenPipe),
            other => panic!("expected Io(BrokenPipe), got {other:?}"),
        }
        assert_eq!(reactor.state, ReactorState::Closed, "state must transition to Closed");
        let fault = reactor.pending_host_fault.as_ref().expect("host fault must be staged");
        assert_eq!(fault.fault_code, FaultCode::HostDisconnect.as_u16());
        assert!(reactor.pending_submissions.is_empty(), "draining must stop after I/O failure");
    }

    // -----------------------------------------------------------------------
    // Test: BrokenPipe on poll_serial sets pending_host_fault and closes.
    // After run(), event_dispatcher.fault_latch.cell is populated.
    // -----------------------------------------------------------------------

    #[test]
    fn broken_pipe_latches_host_disconnect_fault() {
        let (_, rx) = std::sync::mpsc::channel::<crate::host_io::ReactorCommand>();
        let status_snapshot = Arc::new(arc_swap::ArcSwap::from_pointee(
            crate::host_io::runtime_events::StatusEvent::default(),
        ));
        let parser = Arc::new(crate::host_io::parser::MsgProtoParser::new_empty());
        let mut reactor = Reactor::new(
            Box::new(BrokenPipePort), parser, rx, status_snapshot, Vec::new(),
            crate::host_io::KalicoHostIoConfig::default(),
        );

        reactor.run(); // runs until Closed

        // The fault should be latched in the FaultLatch cell.
        assert!(
            reactor.event_dispatcher.fault_latch.cell.is_some(),
            "FaultLatch should have a cell after BrokenPipe"
        );
        let cell = reactor.event_dispatcher.fault_latch.cell.as_ref().unwrap();
        assert_eq!(
            cell.fault_code,
            FaultCode::HostDisconnect.as_u16(),
            "fault_code must be KALICO_ERR_HOST_DISCONNECT"
        );
        assert!(!cell.synthesized, "host disconnect fault is not synthesized");
    }
}
