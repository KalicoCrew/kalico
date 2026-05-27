use super::*;
use std::time::{Duration, Instant};

use kalico_native_transport::demux::{Frame, PollOutcome};
use kalico_native_transport::frame::{CHANNEL_CONTROL, encode_frame};

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
    assert_eq!(
        written, frame,
        "write_all must not modify outbound Klipper bytes"
    );
}

#[test]
fn write_all_passes_kalico_bytes_through_unmodified() {
    let (port, handles) = FakeSerialPort::new();
    let mut io = SerialFrameIo::new(port);
    let frame = encode_frame(CHANNEL_CONTROL, b"hello");
    io.write_all(&frame).unwrap();
    io.flush().unwrap();
    let written = drain_tx(&handles);
    assert_eq!(
        written, frame,
        "write_all must not modify outbound kalico-native bytes"
    );
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

    let outcome = io
        .poll_frames_until(Instant::now() + Duration::from_millis(50))
        .unwrap();
    let phase1_frames = match outcome {
        PollOutcome::Frames { frames, .. } => frames,
        other => panic!("phase 1 expected Frames, got {other:?}"),
    };
    assert_eq!(
        phase1_frames.len(),
        1,
        "phase 1 should yield only the complete frame"
    );
    assert!(
        matches!(&phase1_frames[0], Frame::Klipper(kf) if kf.bytes() == complete.as_slice()),
        "phase 1 frame must match `complete`",
    );

    // Phase 2 — "reactor side" feeds the remaining bytes. The Demuxer
    // state inside SerialFrameIo must have kept the partial bytes so that
    // `next` is completed here without losing the already-consumed prefix.
    feed_rx(&handles, &next[split..]);

    let outcome = io
        .poll_frames_until(Instant::now() + Duration::from_millis(50))
        .unwrap();
    let phase2_frames = match outcome {
        PollOutcome::Frames { frames, .. } => frames,
        other => panic!("phase 2 expected Frames, got {other:?}"),
    };
    assert_eq!(
        phase2_frames.len(),
        1,
        "phase 2 should complete the second frame"
    );
    assert!(
        matches!(&phase2_frames[0], Frame::Klipper(kf) if kf.bytes() == next.as_slice()),
        "phase 2 frame must match `next`",
    );
}
