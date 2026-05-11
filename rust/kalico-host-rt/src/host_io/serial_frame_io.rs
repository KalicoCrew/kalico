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
            // USB-CDC TTYs (the production transport for kalico MCUs) return
            // `Ok(0)` during idle gaps as a normal "no bytes available within
            // timeout" signal — NOT a disconnect. The reactor's PhantomZero
            // debounce was designed for TCP-style streams where `Ok(0)` is a
            // genuine half-close. Treating Ok(0) as Timeout here keeps the
            // reactor alive across idle windows; real USB-CDC disconnects
            // still surface via the `Err(Io(...))` arm below (the kernel
            // returns ENODEV when the device is unplugged).
            //
            // 2026-05-12 bench: F446 reactor was closing itself between
            // QueryRuntimeCaps and ConfigureAxes because the idle window
            // exceeded the 100 ms ZERO_BYTE_DEBOUNCE. configure_axes for
            // F446 then returned `TransportError::Closed` without writing
            // any bytes to the wire. H7 dodged the bug only because its
            // configure_axes ran first without an idle gap.
            Ok(0) => Ok(PollOutcome::Timeout),
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
    use std::time::{Duration, Instant};

    use kalico_native_transport::demux::{Frame, PollOutcome};
    use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL};

    use crate::host_io::test_harness::FakeSerialPort;
    use crate::host_io::wire::build_frame;

    // Helper: drain all bytes written to the fake port's TX buffer.
    fn drain_tx(handles: &crate::host_io::test_harness::FakePortHandles) -> Vec<u8> {
        let mut g = handles.tx.lock().unwrap();
        let v = g.clone();
        g.clear();
        v
    }

    // Helper: feed bytes into the fake port's RX buffer.
    fn feed_rx(handles: &crate::host_io::test_harness::FakePortHandles, bytes: &[u8]) {
        handles.rx.lock().unwrap().extend(bytes);
    }

    #[test]
    fn write_all_passes_klipper_bytes_through_unmodified() {
        let (port, handles) = FakeSerialPort::new();
        let mut io = SerialFrameIo::new(port);
        let frame = build_frame(&[0x01, 0x02], 0);
        io.write_all(&frame).unwrap();
        io.flush().unwrap();
        let written = drain_tx(&handles);
        assert_eq!(written, frame, "write_all must not modify outbound Klipper bytes");
    }

    #[test]
    fn write_all_passes_kalico_bytes_through_unmodified() {
        let (port, handles) = FakeSerialPort::new();
        let mut io = SerialFrameIo::new(port);
        let frame = encode_frame(CHANNEL_CONTROL, b"hello");
        io.write_all(&frame).unwrap();
        io.flush().unwrap();
        let written = drain_tx(&handles);
        assert_eq!(written, frame, "write_all must not modify outbound kalico-native bytes");
    }

    /// Regression: partial Klipper frame bytes consumed during "identify" must
    /// survive in the Demuxer so that the subsequent "reactor" read completes
    /// the frame. This test would have caught both df07d5a03 and 9c5dedc33.
    #[test]
    fn partial_klipper_frame_survives_identify_to_reactor_handoff() {
        let (port, handles) = FakeSerialPort::new();
        let mut io = SerialFrameIo::new(port);
        let complete = build_frame(&[0xAA], 0);
        let next = build_frame(&[0xBB], 1);

        // Phase 1 — "identify" reads the complete frame plus the FIRST half of `next`.
        let split = next.len() / 2;
        feed_rx(&handles, &complete);
        feed_rx(&handles, &next[..split]);

        let outcome = io.poll_frames_until(Instant::now() + Duration::from_millis(50)).unwrap();
        let phase1_frames = match outcome {
            PollOutcome::Frames { frames, .. } => frames,
            other => panic!("phase 1 expected Frames, got {other:?}"),
        };
        assert_eq!(phase1_frames.len(), 1, "phase 1 should yield only the complete frame");
        assert!(
            matches!(&phase1_frames[0], Frame::Klipper(kf) if kf.bytes() == complete.as_slice()),
            "phase 1 frame must match `complete`",
        );

        // Phase 2 — "reactor side" feeds the remaining bytes. The Demuxer
        // state inside SerialFrameIo must have kept the partial bytes so that
        // `next` is completed here without losing the already-consumed prefix.
        feed_rx(&handles, &next[split..]);

        let outcome = io.poll_frames_until(Instant::now() + Duration::from_millis(50)).unwrap();
        let phase2_frames = match outcome {
            PollOutcome::Frames { frames, .. } => frames,
            other => panic!("phase 2 expected Frames, got {other:?}"),
        };
        assert_eq!(phase2_frames.len(), 1, "phase 2 should complete the second frame");
        assert!(
            matches!(&phase2_frames[0], Frame::Klipper(kf) if kf.bytes() == next.as_slice()),
            "phase 2 frame must match `next`",
        );
    }
}
