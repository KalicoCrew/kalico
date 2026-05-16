//! `TcpSerialPort` — a `serialport::SerialPort` adapter backed by a `TcpStream`.
//!
//! Renode exposes the simulated USART2 as a TCP listener (see
//! `tools/sim/h723_sim.resc`'s `CreateServerSocketTerminal 3334`). This adapter
//! lets `KalicoHostIo` connect to it without modifying the SerialFrameIo /
//! reactor stack — the port reads/writes look like a normal serial connection.
//!
//! Only `read`, `write`, `flush`, `set_timeout`, and `timeout` are exercised
//! by the production reactor. Everything else (RTS, parity, baud, break) is
//! a no-op or returns `NotSupported`, matching Renode's UART model which
//! ignores those settings.
//!
//! ## Test-only
//!
//! This adapter is shipped as part of the host runtime so tests in other
//! crates (`motion-bridge`, etc.) can wire `KalicoHostIo` against Renode
//! without depending on platform-specific PTY plumbing. Production code
//! uses real `/dev/ttyACM*` paths via `open_with_config`.

use std::io::{self, Read, Write};
use std::net::TcpStream;
use std::time::Duration;

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits};

use crate::transport::TransportError;

pub struct TcpSerialPort {
    stream: TcpStream,
    name: String,
    timeout: Duration,
}

impl TcpSerialPort {
    /// Connect to `addr` (host:port or `127.0.0.1:3334`-style). Disables
    /// Nagle so command writes go on the wire immediately (the Klipper
    /// wire protocol is latency-sensitive; coalescing small frames adds
    /// 40 ms+ on macOS).
    pub fn connect(addr: &str) -> Result<Self, TransportError> {
        let stream = TcpStream::connect(addr)
            .map_err(|e| TransportError::Io(io::Error::other(
                format!("TcpSerialPort::connect({addr}): {e}"),
            )))?;
        // Note: NOT setting TCP_NODELAY. pyserial's `socket://` URL handler
        // doesn't either, and the Renode TCP bridge appears sensitive to
        // small-batch writes when Nagle is off — sub-millisecond identify-
        // NAK retries flood the firmware's USART2 RX FIFO faster than the
        // 1µs simulation quantum can drain it (Renode's USART model
        // overruns on >1 byte/µs sustained, dropping bytes silently).
        // Default read timeout: 100 ms, matching `SerialFrameIo::new` callers.
        let default_timeout = Duration::from_millis(100);
        stream
            .set_read_timeout(Some(default_timeout))
            .map_err(|e| TransportError::Io(io::Error::other(
                format!("TcpSerialPort: set_read_timeout: {e}"),
            )))?;
        // Long write timeout — Renode's TCP server can stall briefly under
        // heavy traffic but never refuses inbound. A 100 ms write timeout
        // (the read default) caused identify-NAK loops to abort mid-frame:
        // partial-frame remnant on the firmware's USART RX buffer wedged
        // the demuxer until the next chunk completed an unrelated frame
        // boundary.
        stream
            .set_write_timeout(Some(Duration::from_secs(5)))
            .map_err(|e| TransportError::Io(io::Error::other(
                format!("TcpSerialPort: set_write_timeout: {e}"),
            )))?;
        Ok(Self {
            stream,
            name: format!("tcp://{addr}"),
            timeout: default_timeout,
        })
    }
}

impl Read for TcpSerialPort {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        self.stream.read(buf)
    }
}

impl Write for TcpSerialPort {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        self.stream.write(buf)
    }
    fn flush(&mut self) -> io::Result<()> {
        self.stream.flush()
    }
}

fn no_support(method: &'static str) -> serialport::Error {
    serialport::Error::new(
        serialport::ErrorKind::Io(io::ErrorKind::Unsupported),
        format!("TcpSerialPort: {method} is not supported"),
    )
}

impl SerialPort for TcpSerialPort {
    fn name(&self) -> Option<String> { Some(self.name.clone()) }

    // Renode UART ignores these — return plausible defaults so callers
    // logging the values get something sensible.
    fn baud_rate(&self) -> serialport::Result<u32> { Ok(250_000) }
    fn data_bits(&self) -> serialport::Result<DataBits> { Ok(DataBits::Eight) }
    fn flow_control(&self) -> serialport::Result<FlowControl> { Ok(FlowControl::None) }
    fn parity(&self) -> serialport::Result<Parity> { Ok(Parity::None) }
    fn stop_bits(&self) -> serialport::Result<StopBits> { Ok(StopBits::One) }

    fn timeout(&self) -> Duration { self.timeout }

    fn set_baud_rate(&mut self, _baud_rate: u32) -> serialport::Result<()> { Ok(()) }
    fn set_data_bits(&mut self, _data_bits: DataBits) -> serialport::Result<()> { Ok(()) }
    fn set_flow_control(&mut self, _flow_control: FlowControl) -> serialport::Result<()> { Ok(()) }
    fn set_parity(&mut self, _parity: Parity) -> serialport::Result<()> { Ok(()) }
    fn set_stop_bits(&mut self, _stop_bits: StopBits) -> serialport::Result<()> { Ok(()) }

    fn set_timeout(&mut self, timeout: Duration) -> serialport::Result<()> {
        // TcpStream uses `None` to mean "block forever". Treat zero / very
        // tiny durations as a 5 ms floor — macOS SO_RCVTIMEO with sub-ms
        // durations sometimes returns immediately with WouldBlock before any
        // bytes can land in the socket buffer (kernel scheduling vs. timeout
        // resolution race). 5 ms is far below the reactor's polling cadence
        // but generous enough that arriving frames are actually read.
        let effective = timeout.max(Duration::from_millis(5));
        // 2026-05-16: avoid calling setsockopt(SO_RCVTIMEO) on every poll
        // iteration. macOS intermittently rejects same-value re-sets with
        // EINVAL when the syscall rate climbs (each LoadCurve read-wait
        // re-calls this thousands of times). Skip the syscall when the
        // requested timeout is unchanged from what we already set.
        if effective == self.timeout {
            return Ok(());
        }
        self.stream
            .set_read_timeout(Some(effective))
            .map_err(|e| serialport::Error::new(
                serialport::ErrorKind::Io(e.kind()),
                format!("TcpSerialPort: set_read_timeout: {e}"),
            ))?;
        // Write timeout intentionally left at the construction default (100 ms).
        // The reactor's `SerialFrameIo::poll_frames_until` shrinks the read
        // timeout to whatever budget remains — sometimes a single millisecond
        // — but our write path (frame send + write_all) shouldn't be subject
        // to the same per-poll shrinkage. A 100 ms write timeout is plenty
        // for Renode's TCP bridge, which never throttles inbound bytes.
        self.timeout = effective;
        Ok(())
    }

    fn write_request_to_send(&mut self, _level: bool) -> serialport::Result<()> {
        Err(no_support("write_request_to_send"))
    }
    fn write_data_terminal_ready(&mut self, _level: bool) -> serialport::Result<()> {
        Err(no_support("write_data_terminal_ready"))
    }
    fn read_clear_to_send(&mut self) -> serialport::Result<bool> {
        Err(no_support("read_clear_to_send"))
    }
    fn read_data_set_ready(&mut self) -> serialport::Result<bool> {
        Err(no_support("read_data_set_ready"))
    }
    fn read_ring_indicator(&mut self) -> serialport::Result<bool> {
        Err(no_support("read_ring_indicator"))
    }
    fn read_carrier_detect(&mut self) -> serialport::Result<bool> {
        Err(no_support("read_carrier_detect"))
    }

    fn bytes_to_read(&self) -> serialport::Result<u32> { Ok(0) }
    fn bytes_to_write(&self) -> serialport::Result<u32> { Ok(0) }

    fn clear(&self, _buffer_to_clear: ClearBuffer) -> serialport::Result<()> { Ok(()) }

    fn try_clone(&self) -> serialport::Result<Box<dyn SerialPort>> {
        let cloned = self.stream.try_clone().map_err(|e| serialport::Error::new(
            serialport::ErrorKind::Io(e.kind()),
            format!("TcpSerialPort: try_clone: {e}"),
        ))?;
        Ok(Box::new(TcpSerialPort {
            stream: cloned,
            name: self.name.clone(),
            timeout: self.timeout,
        }))
    }

    fn set_break(&self) -> serialport::Result<()> { Err(no_support("set_break")) }
    fn clear_break(&self) -> serialport::Result<()> { Err(no_support("clear_break")) }
}
