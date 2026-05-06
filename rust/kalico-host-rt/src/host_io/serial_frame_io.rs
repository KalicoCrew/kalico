//! Production frame-source: owns the SerialPort, the Demuxer, and the
//! scratch buffer. Single owner of the wire across identify→reactor handoff.
//! See spec §3.1, §3.5.

use std::io::{self, Read};
use std::time::Instant;

use serialport::SerialPort;

use kalico_native_transport::demux::{Demuxer, PollOutcome};

use crate::transport::TransportError;

pub struct SerialFrameIo {
    port: Box<dyn SerialPort>,
    demuxer: Demuxer,
    scratch: [u8; 1024],
}

impl SerialFrameIo {
    pub fn new(port: Box<dyn SerialPort>) -> Self {
        Self { port, demuxer: Demuxer::new(), scratch: [0u8; 1024] }
    }

    /// Read one batch of bytes from the port and demux. The deadline bounds
    /// how long the underlying port read may block; identify uses long
    /// deadlines, the reactor's poll_serial uses `now + READ_TIMEOUT`.
    pub fn poll_frames_until(&mut self, deadline: Instant)
        -> Result<PollOutcome, TransportError>
    {
        let now = Instant::now();
        let remaining = deadline.saturating_duration_since(now);
        if let Err(e) = self.port.set_timeout(remaining) {
            return Err(TransportError::Io(io::Error::new(io::ErrorKind::Other, e.to_string())));
        }
        match self.port.read(&mut self.scratch) {
            Ok(0) => Ok(PollOutcome::PhantomZero),
            Ok(n) => {
                let (frames, errors) = self.demuxer.feed_slice(&self.scratch[..n]);
                Ok(PollOutcome::Frames { frames, errors })
            }
            Err(e) if matches!(e.kind(), io::ErrorKind::TimedOut | io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock) =>
                Ok(PollOutcome::Timeout),
            Err(e) => Err(TransportError::Io(e)),
        }
    }

    /// Raw byte passthrough. Does NOT validate, frame, or re-shape outbound
    /// bytes. Both Klipper-shaped frames (build_frame) and Kalico-native
    /// frames (KalicoIdentify::build_*) are pre-built by their encoders and
    /// written verbatim. See spec §3.1.
    pub fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.port.write_all(bytes).map_err(TransportError::Io)
    }

    pub fn flush(&mut self) -> Result<(), TransportError> {
        self.port.flush().map_err(TransportError::Io)
    }

    /// Test-only access to the underlying port for fixtures that need to
    /// observe what was written. Gated behind a feature so it doesn't leak
    /// into production callers.
    #[cfg(any(test, feature = "test-harness"))]
    pub fn port_mut(&mut self) -> &mut Box<dyn SerialPort> {
        &mut self.port
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    // FakeSerialPort lives in test_harness — the tests here will be added
    // in Task 9 once that wiring is in place. Skeleton tests intentionally
    // omitted at file creation time.
}
