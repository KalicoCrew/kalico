//! Test-only reactor harness. See spec §2.5.
//!
//! Provides `ReactorHarness` for `#[cfg(test)] mod` blocks inside `reactor.rs`
//! that need direct access to `pub(crate)` `Reactor` fields. Constructs a
//! Reactor outside the production `KalicoHostIo::open` path with a
//! `FakeSerialPort` and a hand-driven `MockClock`.

#![cfg(any(test, feature = "test-harness"))]

use std::collections::VecDeque;
use std::io::{self, Read, Write};
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::mpsc::{Sender, sync_channel};
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use serialport::SerialPort;

use crate::clock::{Clock, MockClock};
use crate::host_io::KalicoHostIoConfig;
use crate::host_io::ReactorCommand;
use crate::host_io::identify::IdentifySeqState;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::reactor::{Reactor, TickOutcome};
use crate::host_io::runtime_events::StatusEvent;
use crate::host_io::serial_frame_io::SerialFrameIo;
use crate::transport::{MessageParams, TransportError};

// ---------------------------------------------------------------------------
// FakeSerialPort
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub struct FakePortHandles {
    pub rx: Arc<Mutex<VecDeque<u8>>>,
    pub tx: Arc<Mutex<Vec<u8>>>,
}

pub struct FakeSerialPort {
    handles: FakePortHandles,
}

impl FakeSerialPort {
    pub fn new() -> (Box<Self>, FakePortHandles) {
        let h = FakePortHandles {
            rx: Arc::new(Mutex::new(VecDeque::new())),
            tx: Arc::new(Mutex::new(Vec::new())),
        };
        (Box::new(Self { handles: h.clone() }), h)
    }
}

impl Read for FakeSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let mut g = self.handles.rx.lock().unwrap();
        let n = std::cmp::min(g.len(), buf.len());
        for slot in buf.iter_mut().take(n) {
            *slot = g.pop_front().unwrap();
        }
        if n == 0 {
            // Mirror non-blocking-read-no-data semantics.
            Err(io::Error::new(io::ErrorKind::TimedOut, "no data"))
        } else {
            Ok(n)
        }
    }
}

impl Write for FakeSerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.handles.tx.lock().unwrap().extend_from_slice(buf);
        Ok(buf.len())
    }
    fn flush(&mut self) -> io::Result<()> { Ok(()) }
}

// Stub the rest of the SerialPort trait. The reactor only calls write_all,
// flush, set_timeout, and read; everything else returns sensible defaults
// or Unsupported errors.
impl SerialPort for FakeSerialPort {
    fn name(&self) -> Option<String> { Some("fake".into()) }
    fn baud_rate(&self) -> serialport::Result<u32> { Ok(0) }
    fn data_bits(&self) -> serialport::Result<serialport::DataBits> {
        Err(serialport::Error::new(serialport::ErrorKind::Unknown, "unsupported"))
    }
    fn flow_control(&self) -> serialport::Result<serialport::FlowControl> {
        Err(serialport::Error::new(serialport::ErrorKind::Unknown, "unsupported"))
    }
    fn parity(&self) -> serialport::Result<serialport::Parity> {
        Err(serialport::Error::new(serialport::ErrorKind::Unknown, "unsupported"))
    }
    fn stop_bits(&self) -> serialport::Result<serialport::StopBits> {
        Err(serialport::Error::new(serialport::ErrorKind::Unknown, "unsupported"))
    }
    fn timeout(&self) -> Duration { Duration::from_millis(0) }
    fn set_baud_rate(&mut self, _: u32) -> serialport::Result<()> { Ok(()) }
    fn set_flow_control(&mut self, _: serialport::FlowControl) -> serialport::Result<()> { Ok(()) }
    fn set_parity(&mut self, _: serialport::Parity) -> serialport::Result<()> { Ok(()) }
    fn set_data_bits(&mut self, _: serialport::DataBits) -> serialport::Result<()> { Ok(()) }
    fn set_stop_bits(&mut self, _: serialport::StopBits) -> serialport::Result<()> { Ok(()) }
    fn set_timeout(&mut self, _: Duration) -> serialport::Result<()> { Ok(()) }
    fn write_request_to_send(&mut self, _: bool) -> serialport::Result<()> { Ok(()) }
    fn write_data_terminal_ready(&mut self, _: bool) -> serialport::Result<()> { Ok(()) }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> { Ok(false) }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> { Ok(false) }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> { Ok(false) }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> { Ok(false) }
    fn bytes_to_read(&self) -> serialport::Result<u32> {
        Ok(self.handles.rx.lock().unwrap().len() as u32)
    }
    fn bytes_to_write(&self) -> serialport::Result<u32> { Ok(0) }
    fn clear(&self, _: serialport::ClearBuffer) -> serialport::Result<()> { Ok(()) }
    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        Err(serialport::Error::new(serialport::ErrorKind::Unknown, "unsupported"))
    }
    fn set_break(&self) -> serialport::Result<()> { Ok(()) }
    fn clear_break(&self) -> serialport::Result<()> { Ok(()) }
}

// ---------------------------------------------------------------------------
// ReactorHarness
// ---------------------------------------------------------------------------

pub struct ReactorHarness {
    pub reactor: Reactor,
    pub clock: Arc<MockClock>,
    pub port_handles: FakePortHandles,
    pub submission_tx: Sender<ReactorCommand>,
}

impl ReactorHarness {
    pub fn new() -> Self {
        let (port, port_handles) = FakeSerialPort::new();
        let clock = MockClock::new();
        let parser = Arc::new(MsgProtoParser::new_empty());
        let (submission_tx, submission_rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
        let config = KalicoHostIoConfig::default();
        let reactor = Reactor::new_for_tests(
            port, parser, submission_rx, status_snapshot,
            config, clock.clone(),
        );
        Self { reactor, clock, port_handles, submission_tx }
    }

    /// Construct a harness with an explicit `IdentifySeqState`, simulating
    /// a reactor coming up after identify burned a non-zero number of
    /// sequences. Used by the H7 regression test (spec §3.3, §5.2).
    pub fn new_with_seq_state(seq: IdentifySeqState) -> Self {
        let (port, port_handles) = FakeSerialPort::new();
        let clock = MockClock::new();
        let parser = Arc::new(MsgProtoParser::new_empty());
        let (submission_tx, submission_rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
        let config = KalicoHostIoConfig::default();
        let clock_dyn: Arc<dyn Clock> = clock.clone();
        let reactor = Reactor::new_with_clock(
            SerialFrameIo::new(port),
            parser,
            submission_rx,
            status_snapshot,
            seq,
            config,
            clock_dyn,
        );
        Self { reactor, clock, port_handles, submission_tx }
    }

    pub fn feed_rx(&self, bytes: &[u8]) {
        self.port_handles.rx.lock().unwrap().extend(bytes);
    }

    pub fn advance_clock(&self, by: Duration) {
        self.clock.advance(by);
    }

    pub fn tick(&mut self) -> TickOutcome {
        self.reactor.tick_once()
    }

    pub fn tx_log(&self) -> Vec<u8> {
        self.port_handles.tx.lock().unwrap().clone()
    }

    pub fn unacked_depth(&self) -> usize { self.reactor.unacked_window.len() }
    pub fn awaiting_depth(&self) -> usize { self.reactor.awaiting_response.len() }
    pub fn send_seq(&self) -> u64 { self.reactor.send_seq }

    /// Feed an ACK frame that acknowledges all frames up to (but not
    /// including) the current `send_seq`. This clears the reactor's unacked
    /// window so tests can verify resumed emission after backpressure.
    pub fn feed_ack_all(&self) {
        let seq_nibble = (self.reactor.send_seq & 0x0F) as u8;
        let frame = crate::host_io::wire::build_frame(&[], seq_nibble);
        self.feed_rx(&frame);
    }

    /// Submit directly through `Reactor::dispatch_submission`, bypassing the
    /// mpsc channel. Returns the completion `Receiver` so the test can poll.
    pub fn submit_via_dispatch(
        &mut self,
        call_id: u64,
        payload: Vec<u8>,
        expected_response_name: &str,
        deadline: Instant,
    ) -> std::sync::mpsc::Receiver<Result<MessageParams, TransportError>> {
        let (tx, rx) = sync_channel(1);
        let _ = self.reactor.dispatch_submission(
            call_id, payload, expected_response_name.to_string(), tx, deadline,
        );
        rx
    }

    // ── Passthrough-router helpers (used by external integration tests) ──

    /// Install a passthrough router for the given MCU handle.
    pub fn install_passthrough_router(
        &mut self,
        router: crate::passthrough_queue::PassthroughRouter,
        mcu: crate::passthrough_queue::McuHandle,
    ) {
        self.reactor.set_passthrough_router(router, mcu);
    }

    /// Push an entry directly into the passthrough router.
    pub fn passthrough_push(
        &mut self,
        mcu: crate::passthrough_queue::McuHandle,
        queue_id: crate::passthrough_queue::CommandQueueId,
        entry: crate::passthrough_queue::PassthroughEntry,
    ) -> Result<(), crate::passthrough_queue::RouterError> {
        self.reactor
            .passthrough_router
            .as_mut()
            .expect("no passthrough router installed")
            .push(mcu, queue_id, entry)
    }

    /// Dispatch a response through the passthrough router's notify table.
    pub fn passthrough_dispatch_response(
        &mut self,
        mcu: crate::passthrough_queue::McuHandle,
        notify_id: crate::passthrough_queue::NotifyId,
        response_bytes: Vec<u8>,
    ) -> Result<(), crate::passthrough_queue::RouterError> {
        self.reactor
            .passthrough_router
            .as_mut()
            .expect("no passthrough router installed")
            .dispatch_response(mcu, notify_id, response_bytes)
    }

    /// Register a notify callback on the passthrough router.
    pub fn passthrough_register_notify(
        &mut self,
        mcu: crate::passthrough_queue::McuHandle,
        cb: crate::passthrough_queue::NotifyCallback,
    ) -> Result<crate::passthrough_queue::NotifyId, crate::passthrough_queue::RouterError> {
        self.reactor
            .passthrough_router
            .as_mut()
            .expect("no passthrough router installed")
            .register_notify(mcu, cb)
    }

    /// Acknowledge bytes through the passthrough router's receive window.
    pub fn passthrough_record_ack(
        &mut self,
        mcu: crate::passthrough_queue::McuHandle,
        acked_bytes: u64,
    ) -> Result<(), crate::passthrough_queue::RouterError> {
        self.reactor
            .passthrough_router
            .as_mut()
            .expect("no passthrough router installed")
            .record_ack(mcu, acked_bytes)
    }

    /// Add a config command to the MCU's config stage.
    pub fn passthrough_add_config_cmd(
        &mut self,
        mcu: crate::passthrough_queue::McuHandle,
        bytes: Vec<u8>,
    ) -> Result<bool, crate::passthrough_queue::RouterError> {
        self.reactor
            .passthrough_router
            .as_mut()
            .expect("no passthrough router installed")
            .add_config_cmd(mcu, bytes)
    }

    /// Add an init command to the MCU's config stage.
    pub fn passthrough_add_init_cmd(
        &mut self,
        mcu: crate::passthrough_queue::McuHandle,
        bytes: Vec<u8>,
    ) -> Result<bool, crate::passthrough_queue::RouterError> {
        self.reactor
            .passthrough_router
            .as_mut()
            .expect("no passthrough router installed")
            .add_init_cmd(mcu, bytes)
    }

    /// Begin the config phase for the MCU.
    pub fn passthrough_begin_config_phase(
        &mut self,
        mcu: crate::passthrough_queue::McuHandle,
    ) -> Result<(), crate::passthrough_queue::RouterError> {
        self.reactor
            .passthrough_router
            .as_mut()
            .expect("no passthrough router installed")
            .begin_config_phase(mcu)
    }

    /// Drain all available config/init entries from the router.
    pub fn passthrough_drain_config_entries(
        &mut self,
        mcu: crate::passthrough_queue::McuHandle,
    ) -> Result<Vec<Vec<u8>>, crate::passthrough_queue::RouterError> {
        let router = self.reactor
            .passthrough_router
            .as_mut()
            .expect("no passthrough router installed");
        let mut entries = Vec::new();
        while let Some(e) = router.next_config_entry(mcu)? {
            entries.push(e);
        }
        Ok(entries)
    }

    /// Returns the current config-stage phase for the MCU.
    pub fn passthrough_config_phase(
        &self,
        mcu: crate::passthrough_queue::McuHandle,
    ) -> Result<crate::passthrough_queue::ConfigStagePhase, crate::passthrough_queue::RouterError> {
        self.reactor
            .passthrough_router
            .as_ref()
            .expect("no passthrough router installed")
            .config_phase(mcu)
    }

    /// Number of entries in the passthrough notify map (notify-bearing entries
    /// that have been emitted and are awaiting a response).
    pub fn passthrough_notify_map_len(&self) -> usize {
        self.reactor.passthrough_notify_map.len()
    }
}

#[cfg(test)]
mod smoke {
    use super::*;

    #[test]
    fn empty_tick_changes_nothing() {
        let mut h = ReactorHarness::new();
        let outcome = h.tick();
        assert_eq!(outcome, TickOutcome::Continue);
        assert_eq!(h.unacked_depth(), 0);
        assert_eq!(h.awaiting_depth(), 0);
        assert!(h.tx_log().is_empty());
    }

    #[test]
    fn reactor_first_bridge_call_after_identify_succeeds_with_nonzero_initial_seq() {
        // Spec §3.3, §5.2 — H7 regression. Pre-refactor the reactor hardcoded
        // send_seq:1 / receive_seq:1, so any post-identify state where the
        // host had already burned ≥1 sequences would put a stale seq=1 on the
        // wire. Firmware that already advanced past seq=1 ignores it; first
        // bridge_call hangs until host-side timeout.
        //
        // With IdentifySeqState plumbing, the reactor adopts the post-identify
        // counters and the next outbound frame carries the correct seq nibble.
        let mut h = ReactorHarness::new_with_seq_state(IdentifySeqState {
            next_send_seq_abs: 5,
            mcu_receive_seq_abs: 5,
        });

        // Sanity: the public send_seq accessor reflects the adopted state
        // *before* any frame goes out.
        assert_eq!(h.send_seq(), 5, "reactor must adopt next_send_seq_abs from identify");

        let deadline = Instant::now() + Duration::from_secs(1);
        let _completion = h.submit_via_dispatch(
            42, vec![0x01], "noop", deadline,
        );

        let written = h.tx_log();
        assert!(!written.is_empty(), "reactor should have written a frame");
        // Frame layout (see wire::build_frame): [len][seq|DEST][payload..][crc_hi][crc_lo][SYNC]
        let seq_byte = written[1];
        let wire_seq = seq_byte & 0x0F;
        assert_eq!(
            wire_seq, 5,
            "first frame after identify must carry seq=5 (= next_send_seq_abs mod 16), not seq=1",
        );
        // And send_seq must have advanced past it.
        assert_eq!(h.send_seq(), 6, "send_seq must increment after dispatch");
    }

    #[test]
    fn clock_advance_is_visible_to_reactor() {
        let h = ReactorHarness::new();
        let t0 = h.reactor.clock.now();
        h.advance_clock(Duration::from_secs(1));
        let t1 = h.reactor.clock.now();
        assert_eq!(t1 - t0, Duration::from_secs(1));
    }
}
