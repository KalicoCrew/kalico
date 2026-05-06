//! Generic frame-source over any `R: Read`. Test-only / corpus-replay
//! companion to `kalico-host-rt::SerialFrameIo`. See spec §3.1.

use std::io::{self, Read};
use std::time::{Duration, Instant};

use crate::demux::{Demuxer, PollOutcome};

#[derive(Debug, thiserror::Error)]
pub enum FrameSourceError {
    #[error("set_timeout failed: {0}")]
    SetTimeout(io::Error),
    #[error("io error: {0}")]
    Io(io::Error),
}

pub struct FrameSource<R: Read> {
    reader: R,
    set_timeout: Box<dyn FnMut(&mut R, Duration) -> io::Result<()>>,
    demuxer: Demuxer,
    scratch: [u8; 1024],
}

impl<R: Read + std::fmt::Debug> std::fmt::Debug for FrameSource<R> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FrameSource")
            .field("reader", &self.reader)
            .field("set_timeout", &"<fn>")
            .field("demuxer", &self.demuxer)
            .finish()
    }
}

impl<R: Read> FrameSource<R> {
    pub fn new(
        reader: R,
        set_timeout: Box<dyn FnMut(&mut R, Duration) -> io::Result<()>>,
    ) -> Self {
        Self { reader, set_timeout, demuxer: Demuxer::new(), scratch: [0u8; 1024] }
    }

    pub fn from_read_no_timeout(reader: R) -> Self {
        Self::new(reader, Box::new(|_, _| Ok(())))
    }

    pub fn into_inner(self) -> R {
        self.reader
    }

    pub fn poll_frames_until(
        &mut self,
        deadline: Instant,
    ) -> Result<PollOutcome, FrameSourceError> {
        let now = Instant::now();
        let remaining = deadline.saturating_duration_since(now);
        (self.set_timeout)(&mut self.reader, remaining)
            .map_err(FrameSourceError::SetTimeout)?;
        match self.reader.read(&mut self.scratch) {
            Ok(0) => Ok(PollOutcome::PhantomZero),
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
            Err(e) => Err(FrameSourceError::Io(e)),
        }
    }
}

#[cfg(test)]
mod tests {
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
}
