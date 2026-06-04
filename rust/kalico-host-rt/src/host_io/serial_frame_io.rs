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
        Self {
            port,
            demuxer: Demuxer::new(),
            scratch: [0u8; 1024],
        }
    }

    pub fn poll_frames_until(&mut self, deadline: Instant) -> Result<PollOutcome, TransportError> {
        let now = Instant::now();
        let remaining = deadline.saturating_duration_since(now);
        if let Err(e) = self.port.set_timeout(remaining) {
            return Err(TransportError::Io(io::Error::new(
                io::ErrorKind::Other,
                e.to_string(),
            )));
        }
        match self.port.read(&mut self.scratch) {
            // USB-CDC Ok(0) is an idle timeout, not a half-close; treat as Timeout so the reactor
            // stays alive across idle windows. Real disconnects arrive as Err(ENODEV).
            Ok(0) => Ok(PollOutcome::Timeout),
            Ok(n) => {
                let (frames, errors) = self.demuxer.feed_slice(&self.scratch[..n]);
                Ok(PollOutcome::Frames { frames, errors })
            }
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                        | io::ErrorKind::WouldBlock
                ) =>
            {
                Ok(PollOutcome::Timeout)
            }
            Err(e) => Err(TransportError::Io(e)),
        }
    }

    pub fn write_all(&mut self, bytes: &[u8]) -> Result<(), TransportError> {
        self.port.write_all(bytes).map_err(TransportError::Io)
    }

    pub fn flush(&mut self) -> Result<(), TransportError> {
        self.port.flush().map_err(TransportError::Io)
    }

    #[cfg(any(test, feature = "test-harness"))]
    pub fn port_mut(&mut self) -> &mut Box<dyn SerialPort> {
        &mut self.port
    }
}

#[cfg(test)]
mod tests;
