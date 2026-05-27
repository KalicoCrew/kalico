use super::*;
use std::io::Cursor;
use crate::demux::Frame;
use crate::frame::{encode_frame, CHANNEL_CONTROL};

#[test]
fn poll_frames_until_returns_phantom_zero_on_eof() {
    let cursor = Cursor::new(Vec::<u8>::new());
    let mut fs = FrameSource::from_read_no_timeout(cursor);
    let outcome = fs
        .poll_frames_until(Instant::now() + Duration::from_millis(100))
        .unwrap();
    assert!(matches!(outcome, PollOutcome::PhantomZero));
}

#[test]
fn poll_frames_until_returns_frames_in_arrival_order() {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(&encode_frame(CHANNEL_CONTROL, b"first"));
    bytes.extend_from_slice(&encode_frame(CHANNEL_CONTROL, b"second"));
    let cursor = Cursor::new(bytes);
    let mut fs = FrameSource::from_read_no_timeout(cursor);
    let outcome = fs
        .poll_frames_until(Instant::now() + Duration::from_millis(100))
        .unwrap();
    match outcome {
        PollOutcome::Frames { frames, errors } => {
            assert!(errors.is_empty());
            assert_eq!(frames.len(), 2);
            let payloads: Vec<_> = frames
                .iter()
                .map(|f| match f {
                    Frame::Kalico { payload, .. } => payload.clone(),
                    _ => panic!("expected kalico"),
                })
                .collect();
            assert_eq!(payloads[0], b"first");
            assert_eq!(payloads[1], b"second");
        }
        other => panic!("expected Frames, got {other:?}"),
    }
}

#[test]
fn poll_frames_until_propagates_set_timeout_error() {
    let cursor = Cursor::new(Vec::<u8>::new());
    let mut fs = FrameSource::new(
        cursor,
        Box::new(|_, _| Err(io::Error::new(io::ErrorKind::Other, "broken"))),
    );
    let result =
        fs.poll_frames_until(Instant::now() + Duration::from_millis(100));
    assert!(matches!(result, Err(FrameSourceError::SetTimeout(_))));
}
