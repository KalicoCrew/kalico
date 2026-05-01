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

use crate::clock::MockClock;
use crate::host_io::KalicoHostIoConfig;
use crate::host_io::ReactorCommand;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::reactor::{Reactor, TickOutcome};
use crate::host_io::runtime_events::StatusEvent;
use crate::transport::{MessageParams, TransportError};

// ---------------------------------------------------------------------------
// FakeSerialPort
// ---------------------------------------------------------------------------

#[derive(Clone)]
pub(crate) struct FakePortHandles {
    pub rx: Arc<Mutex<VecDeque<u8>>>,
    pub tx: Arc<Mutex<Vec<u8>>>,
}

pub(crate) struct FakeSerialPort {
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

pub(crate) struct ReactorHarness {
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
        let reactor = Reactor::new_with_clock(
            port, parser, submission_rx, status_snapshot,
            Vec::new(), config, clock.clone(),
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
    fn clock_advance_is_visible_to_reactor() {
        let h = ReactorHarness::new();
        let t0 = h.reactor.clock.now();
        h.advance_clock(Duration::from_secs(1));
        let t1 = h.reactor.clock.now();
        assert_eq!(t1 - t0, Duration::from_secs(1));
    }
}
