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
        // Modular arithmetic on the 64-bit absolute seq counter. Practical
        // wraparound is unreachable (would take >500 years at 1 GHz frame
        // rate) but `wrapping_add` makes the boundary explicit and lets
        // tests probe the high end without debug panics.
        self.receive_seq.wrapping_add(delta)
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

// ---------------------------------------------------------------------------
// A1 — seq-wrap boundaries. Spec §3.1.
// Three boundaries: empty-window snap, mid-range mod-16, near u64::MAX.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a1_seq_wrap {
    use super::*;
    use crate::host_io::test_harness::ReactorHarness;
    use crate::host_io::wire::build_frame;
    use std::sync::mpsc::sync_channel;
    use std::time::Duration;

    /// Build a 5-byte ack/nak frame with the given wire seq nibble.
    fn ack_frame(wire_seq_nibble: u8) -> Vec<u8> {
        build_frame(&[], wire_seq_nibble)
    }

    /// Submit one frame directly via dispatch_submission. Drops the receiver.
    fn submit_one(h: &mut ReactorHarness, payload: u8) {
        let (tx, _rx) = sync_channel(1);
        let _ = h.reactor.dispatch_submission(
            payload as u64, vec![payload], "noop".into(),
            tx, h.clock.now() + Duration::from_secs(60),
        );
    }

    #[test]
    fn empty_window_snap_advances_both_counters() {
        let mut h = ReactorHarness::new();
        // Pre: window empty; init send_seq=1, receive_seq=1.
        assert_eq!(h.reactor.send_seq, 1);
        assert_eq!(h.reactor.receive_seq, 1);
        assert!(h.reactor.unacked_window.is_empty());

        // Inject ack frame whose 4-bit wire seq nibble = 5 (rseq decoded = 5).
        h.feed_rx(&ack_frame(5));
        h.tick();

        // Snap path (reactor.rs:222-227): both counters jump to rseq.
        assert_eq!(h.reactor.send_seq, 5);
        assert_eq!(h.reactor.receive_seq, 5);
    }

    #[test]
    fn mid_range_mod16_wrap_pops_correct_entries() {
        let mut h = ReactorHarness::new();
        // Submit 12 frames (window cap = MAX_PENDING_BLOCKS = 12).
        for p in 1u8..=12 {
            submit_one(&mut h, p);
        }
        // Tick to process serial poll (no rx yet).
        h.tick();
        assert_eq!(h.unacked_depth(), 12);
        // After 12 submissions: send_seq advanced from 1 to 13.
        assert_eq!(h.reactor.send_seq, 13);
        // receive_seq still 1.
        assert_eq!(h.reactor.receive_seq, 1);

        // Step 1: ack rseq=12. decode_absolute(wire) when receive_seq=1:
        //   delta = (wire - 1) & 0xF. Want delta=11 → wire = (1+11) & 0xF = 12.
        // rseq = 1 + 11 = 12. Pops seqs <12 (i.e. 1..=11). seq=12 remains.
        h.feed_rx(&ack_frame(12));
        h.tick();
        assert_eq!(h.reactor.last_ack_seq, 12);
        assert_eq!(h.reactor.receive_seq, 12);
        assert_eq!(h.unacked_depth(), 1);

        // Step 2: cross the receive_seq=16 epoch boundary. Submit more frames so
        // there's something past 16 to ack. send_seq is 13; submit seqs 13..=20.
        for p in 13u8..=20 {
            submit_one(&mut h, p);
        }
        h.tick();
        assert_eq!(h.unacked_depth(), 9); // seqs 12..=20 outstanding
        assert_eq!(h.reactor.send_seq, 21);

        // Ack rseq=18. delta = (18 - 12) & 0xF = 6 → wire nibble = (12 + 6) & 0xF = 2.
        // Wait: decode_absolute reads low-4 wire bits and computes
        //   delta = (wire_seq - receive_seq) & 0xF
        // where receive_seq=12. To get delta=6 we need wire = (12 + 6) & 0xF = 18 & 0xF = 2.
        // rseq = 12 + 6 = 18. This crosses the receive_seq=16 mod-16 boundary.
        h.feed_rx(&ack_frame(2));
        h.tick();
        assert_eq!(h.reactor.last_ack_seq, 18);
        assert_eq!(h.reactor.receive_seq, 18);
        // Pops seqs <18, i.e. 12..=17. seq 18..=20 remain → 3 entries.
        assert_eq!(h.unacked_depth(), 3);
    }

    #[test]
    fn near_u64_max_decode_does_not_panic() {
        // Probe both `wrapping_sub` (used to compute delta from low-4 nibble)
        // and the addition `receive_seq + delta` against the u64 boundary.
        // The 4-bit wire nibble bounds delta ∈ [0, 15], so to make addition
        // wrap we set receive_seq = u64::MAX - 5 (or similar small offset)
        // and ack a target ≥ u64::MAX, which wraps.
        //
        // Note: the production reactor's `decode_absolute` does NOT use
        // `wrapping_add` — it does `self.receive_seq + delta` (reactor.rs:214).
        // In debug builds this would panic on overflow. We use values where
        // the addition stays within u64 to verify correctness, then a
        // separate sub-test using `checked_add` semantics could probe the
        // hypothetical wrap; for now we simply verify the high-end works.
        let mut h = ReactorHarness::new();
        h.reactor.receive_seq = u64::MAX - 5;
        h.reactor.send_seq    = u64::MAX - 5;
        h.reactor.last_ack_seq = u64::MAX - 6;

        submit_one(&mut h, 0);
        h.tick();
        assert_eq!(h.unacked_depth(), 1);

        // The submit pushed an entry at seq = u64::MAX - 5. send_seq is now
        // u64::MAX - 4. To ack that entry, target rseq = u64::MAX - 4.
        // delta = ((target - receive_seq) & 0xF) = (1) & 0xF = 1.
        // Wire nibble = target & 0xF = (u64::MAX - 4) & 0xF.
        let target_rseq: u64 = u64::MAX - 4;
        let nibble = (target_rseq & 0x0F) as u8;
        h.feed_rx(&ack_frame(nibble));
        h.tick();
        assert_eq!(h.reactor.last_ack_seq, target_rseq);
        assert_eq!(h.reactor.receive_seq, target_rseq);

        // Probe the wrap-sub side: from receive_seq = X, a wire nibble
        // representing a value "behind" X (which the MCU would never send,
        // but `wrapping_sub` must not panic on it). We expect this stays
        // discriminated as a stale ack — last_ack_seq is already X+1, so
        // any rseq we decode whose value < last_ack_seq+1 is dropped.
        let nibble_behind = ((h.reactor.receive_seq - 8) & 0x0F) as u8;
        h.feed_rx(&ack_frame(nibble_behind));
        h.tick(); // must not panic
    }
}

// ---------------------------------------------------------------------------
// A2 — NAK/RTO branches. Spec §3.2.
// Six sub-tests, one per branch.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a2_nak_rto {
    use super::*;
    use crate::host_io::test_harness::ReactorHarness;
    use crate::host_io::wire::build_frame;
    use std::sync::mpsc::sync_channel;
    use std::time::Duration;

    fn submit_one(h: &mut ReactorHarness, payload: u8) {
        let (tx, _rx) = sync_channel(1);
        let _ = h.reactor.dispatch_submission(
            payload as u64, vec![payload], "noop".into(),
            tx, h.clock.now() + Duration::from_secs(60),
        );
    }

    fn ack(wire_seq: u8) -> Vec<u8> { build_frame(&[], wire_seq) }

    #[test]
    fn duplicate_ack_triggers_retransmit() {
        let mut h = ReactorHarness::new();
        submit_one(&mut h, 1);
        submit_one(&mut h, 2);
        h.tick();
        // Forward-progress ack: rseq=2 pops seq=1; seq=2 remains.
        h.feed_rx(&ack(2));
        h.tick();
        let len_before = h.tx_log().len();
        // Duplicate ack on rseq=2 → NAK retransmit (window non-empty).
        h.feed_rx(&ack(2));
        h.tick();
        assert!(h.tx_log().len() > len_before, "duplicate ack should trigger retransmit");
    }

    #[test]
    fn ignore_nak_seq_suppresses_paired_second_nak() {
        let mut h = ReactorHarness::new();
        submit_one(&mut h, 1);
        submit_one(&mut h, 2);
        h.tick();
        h.feed_rx(&ack(2));
        h.tick();
        let len_before = h.tx_log().len();
        // Two duplicate acks on rseq=2 in the same poll cycle.
        h.feed_rx(&ack(2));
        h.feed_rx(&ack(2));
        h.tick();
        let delta = h.tx_log().len() - len_before;
        // One retransmit = 1 SYNC + (frame_bytes for seq=2). Frame for [2] = 6 bytes.
        assert_eq!(delta, 1 + 6, "second NAK must be suppressed by ignore_nak_seq");
    }

    #[test]
    fn rto_fires_at_srtt_plus_4_rttvar() {
        let mut h = ReactorHarness::new();
        // Submit at clock T0.
        submit_one(&mut h, 1);
        h.tick();
        // Advance 50ms; ack: RTT sample = 50ms.
        h.advance_clock(Duration::from_millis(50));
        h.feed_rx(&ack(2));
        h.tick();
        // After one sample of 50ms: SRTT=50, RTTVAR=25; RTO = 50 + max(G, 4*25) = 150ms.
        assert_eq!(h.reactor.rtt.current_rto(), Duration::from_millis(150));

        // Submit frame 2 at current clock; sent_at is "now".
        submit_one(&mut h, 2);
        h.tick();
        let len_before = h.tx_log().len();
        // Advance 149ms — just shy of RTO.
        h.advance_clock(Duration::from_millis(149));
        h.tick();
        assert_eq!(h.tx_log().len(), len_before, "RTO not yet expired");
        // Advance 2ms more → past RTO.
        h.advance_clock(Duration::from_millis(2));
        h.tick();
        assert!(h.tx_log().len() > len_before, "RTO should have fired");
    }

    #[test]
    fn rto_clamped_to_floor_25ms() {
        use crate::host_io::rtt::MIN_RTO;
        let mut h = ReactorHarness::new();
        // Default starts at MIN_RTO.
        assert_eq!(h.reactor.rtt.current_rto(), MIN_RTO);
        // Drive a tiny RTT sample (100µs). SRTT=100µs, RTTVAR=50µs;
        // raw RTO = 100µs + max(1ms, 200µs) = ~1.1ms. Clamped to MIN_RTO=25ms.
        submit_one(&mut h, 1);
        h.tick();
        h.advance_clock(Duration::from_micros(100));
        h.feed_rx(&ack(2));
        h.tick();
        assert!(h.reactor.rtt.current_rto() >= MIN_RTO);
        assert_eq!(h.reactor.rtt.current_rto(), MIN_RTO);
    }

    #[test]
    fn rto_clamped_to_ceiling_5s() {
        use crate::host_io::rtt::MAX_RTO;
        let mut h = ReactorHarness::new();
        submit_one(&mut h, 1);
        h.tick();
        // Huge sample: 10s. SRTT=10s, RTTVAR=5s; raw RTO = 10 + max(G, 20) = 30s.
        // Clamped to MAX_RTO=5s.
        h.advance_clock(Duration::from_secs(10));
        h.feed_rx(&ack(2));
        h.tick();
        assert_eq!(h.reactor.rtt.current_rto(), MAX_RTO);
    }

    #[test]
    fn max_retry_count_closes_with_fault_and_completes_pending() {
        let mut h = ReactorHarness::new();
        let (tx, rx) = sync_channel(1);
        let _ = h.reactor.dispatch_submission(
            1, vec![0xAA], "noop".into(),
            tx, h.clock.now() + Duration::from_secs(600),
        );
        h.tick();
        // Force 8 successive TimeoutDriven retransmits via clock advance.
        // Each tick advances clock past current RTO; write_retransmit increments
        // retry_count for every unacked entry (reactor.rs:293-305). On the 8th
        // call, retry_count >= MAX_RETRY_COUNT → state→Closed, Err returned.
        for _ in 0..8 {
            // Advance well past any RTO ceiling (5s).
            h.advance_clock(Duration::from_secs(10));
            h.tick();
        }
        // Reactor should now be Closed.
        assert_eq!(h.reactor.state, ReactorState::Closed);
        // The next tick processes Closed → TickOutcome::Closed + flush_all_completions.
        let outcome = h.tick();
        assert_eq!(outcome, TickOutcome::Closed);
        // Pending submission must have completed with TransportError::Closed.
        let result = rx.recv_timeout(Duration::from_millis(100))
            .expect("completion delivered");
        assert!(matches!(result, Err(TransportError::Closed)),
            "expected Closed, got {result:?}");
        // Fault was staged with HostRetransmitExhausted code.
        let latched = h.reactor.event_dispatcher.fault_latch.cell.as_ref();
        let fc = latched.expect("fault latched").fault_code;
        assert_eq!(fc, FaultCode::HostRetransmitExhausted.as_u16());
    }
}

// ---------------------------------------------------------------------------
// A4 — NAK + submit same-tick race consistency. Spec §3.4.
// ---------------------------------------------------------------------------

#[cfg(test)]
mod a4_nak_submit_race {
    use super::*;
    use crate::host_io::test_harness::ReactorHarness;
    use crate::host_io::wire::build_frame;
    use std::sync::mpsc::sync_channel;
    use std::time::Duration;

    fn submit_one(h: &mut ReactorHarness, payload: u8) {
        let (tx, _rx) = sync_channel(1);
        let _ = h.reactor.dispatch_submission(
            payload as u64, vec![payload], "noop".into(),
            tx, h.clock.now() + Duration::from_secs(60),
        );
    }

    #[test]
    fn submit_then_nak_in_same_tick_keeps_state_consistent() {
        let mut h = ReactorHarness::new();
        // Stage: submit two frames (seq=1, seq=2). Ack rseq=2 → pops seq=1.
        // Window now has just seq=2.
        submit_one(&mut h, 1);
        submit_one(&mut h, 2);
        h.tick();
        h.feed_rx(&build_frame(&[], 2)); // forward-progress ack rseq=2
        h.tick();
        let len_before_race = h.tx_log().len();
        let depth_before_race = h.unacked_depth();
        assert_eq!(depth_before_race, 1, "seq=2 outstanding");

        // Same-tick race: queue a fresh submission AND a duplicate NAK on rseq=2.
        // Reactor::run() loop body order: command drain (step 1) before serial
        // poll (step 2), so the new frame writes first; NAK retransmit follows.
        let (tx_new, _rx_new) = sync_channel(1);
        // Use SubmitTyped to bypass parser.encode (the harness's empty parser
        // doesn't know any commands). The reactor command-drain path treats
        // SubmitTyped identically aside from encoding.
        h.submission_tx.send(ReactorCommand::SubmitTyped {
            call_id: 3,
            payload: vec![3u8],
            expected_response_name: "noop".into(),
            completion: tx_new,
            deadline: h.clock.now() + Duration::from_secs(60),
        }).unwrap();
        h.feed_rx(&build_frame(&[], 2)); // duplicate ack on rseq=2 → NAK

        h.tick();

        // Both events processed:
        // - Submission of frame 3 wrote to tx_log first (step 1: command drain).
        // - NAK retransmit followed (step 2: serial poll → handle_ack_nak).
        //   At NAK time the window contains {seq=2, seq=3}, so the retransmit
        //   buffer = 1 SYNC byte + frame_for_seq2 + frame_for_seq3.
        // Window post-tick: still {seq=2, seq=3} (NAK retransmit doesn't pop).
        assert_eq!(h.unacked_depth(), 2);
        assert_eq!(h.reactor.last_ack_seq, 2);

        // Compute the exact expected byte delta. Each frame is 5 (header+CRC+SYNC)
        // + 1 byte payload = 6 bytes. We expect:
        //   - new frame (seq=3): 6 bytes
        //   - retransmit buffer: 1 SYNC + 6 (seq=2 frame) + 6 (seq=3 frame) = 13 bytes
        // Total delta = 19 bytes. If the NAK was suppressed or retransmit didn't
        // fire, delta would be only 6 (new frame alone). The exact-equality
        // assertion proves the retransmit ran with both frames.
        let frame_size = 5 + 1; // empty MIN + 1-byte payload
        let expected_delta = frame_size + (1 + 2 * frame_size);
        let actual_delta = h.tx_log().len() - len_before_race;
        assert_eq!(actual_delta, expected_delta,
            "expected new frame ({frame_size} B) + retransmit buffer (1 SYNC + 2 frames = {}) \
             = {expected_delta} B; got {actual_delta} B",
            1 + 2 * frame_size);
    }
}
