//! `UnixNativeConn`: a blocking same-host Unix-socket client speaking pure
//! kalico-native frames. Implements [`NativeCall`] so producers can drive an
//! EtherCAT RT endpoint exactly as they drive a serial `KalicoHostIo`.
//! Same-host => no clock-sync round-trips; the caller stamps segment times on
//! the shared `CLOCK_MONOTONIC` (see the EtherCAT RT endpoint).
//!
//! ## Retirement / heartbeat flow
//!
//! The endpoint emits unsolicited `StatusHeartbeat` (0x0083) frames on the
//! events channel. `UnixNativeConn::poll_events` drains those frames and
//! invokes the installed heartbeat callback with the `retired_counts` slice so
//! the bridge can update per-ring flow-control accounting between calls.

use std::io::{ErrorKind, Read, Write};
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicU32, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::frame::CHANNEL_EVENTS;
use kalico_native_transport::wire_helpers::decode_message_header;
use kalico_protocol::codec::Decode;
use kalico_protocol::messages::{MessageKind, StatusHeartbeat};

use crate::host_io::kalico_native::{build_kalico_control_frame, build_kalico_frame};
use crate::native_call::NativeCall;
use crate::transport::TransportError;

/// Mutable I/O state guarded together so `kalico_call(&self, ...)` is `Sync`.
struct ConnState {
    stream: UnixStream,
    demux: Demuxer,
    buf: [u8; 4096],
}

pub struct UnixNativeConn {
    state: Mutex<ConnState>,
    next_cid: AtomicU32,
    /// Optional callback invoked with `retired_counts` whenever a
    /// `StatusHeartbeat` event frame arrives from the endpoint. Routes
    /// retirement into the pump's heartbeat channel, symmetric with the serial
    /// path's `attach_heartbeat_callback`.
    heartbeat_callback: Mutex<Option<Arc<dyn Fn(&[u32]) + Send + Sync>>>,
}

impl core::fmt::Debug for UnixNativeConn {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("UnixNativeConn")
            .field("next_cid", &self.next_cid.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl UnixNativeConn {
    /// Connect to a listening kalico-native endpoint at `path`.
    pub fn connect(path: &str) -> std::io::Result<Self> {
        let stream = UnixStream::connect(path)?;
        Ok(Self::from_stream(stream))
    }

    /// Wrap an already-connected stream (used by tests via `UnixStream::pair`).
    pub fn from_stream(stream: UnixStream) -> Self {
        Self {
            state: Mutex::new(ConnState {
                stream,
                demux: Demuxer::new(),
                buf: [0u8; 4096],
            }),
            // Start at 1 so a zero correlation id never collides with a
            // freshly-zeroed field on the wire.
            next_cid: AtomicU32::new(1),
            heartbeat_callback: Mutex::new(None),
        }
    }

    /// Install a callback that is invoked with `retired_counts` whenever a
    /// `StatusHeartbeat` (0x0083) event frame is received from the endpoint.
    /// May be called at any time; replaces any previously installed callback.
    /// The callback is invoked on whichever thread drains the socket.
    pub fn attach_heartbeat_callback(&self, cb: Arc<dyn Fn(&[u32]) + Send + Sync>) {
        let mut guard = self
            .heartbeat_callback
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        *guard = Some(cb);
    }

    /// Non-blocking drain of any inbound frames that arrived since the last
    /// `kalico_call` or `poll_events` call. Dispatches `StatusHeartbeat` events
    /// to the installed heartbeat callback. Call this periodically from a
    /// background thread to replenish pump flow-control accounting even when no
    /// new commands are being dispatched.
    ///
    /// Returns the number of `StatusHeartbeat` frames processed.
    pub fn poll_events(&self) -> usize {
        let cb = {
            let g = self
                .heartbeat_callback
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            g.clone()
        };
        let mut st = self.state.lock().unwrap_or_else(|p| p.into_inner());
        // Set a short timeout for this opportunistic drain — we do not want to
        // stall the caller if the socket is idle.
        let _ = st.stream.set_read_timeout(Some(Duration::from_millis(1)));
        let mut count = 0usize;
        loop {
            let ConnState { stream, demux, buf } = &mut *st;
            let n = match stream.read(buf) {
                Ok(0) => break, // EOF
                Ok(n) => n,
                Err(ref e)
                    if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut =>
                {
                    break; // nothing more right now
                }
                Err(_) => break,
            };
            let (frames, _errs) = demux.feed_slice(&buf[..n]);
            for f in frames {
                count += dispatch_frame(f, cb.as_deref());
            }
        }
        count
    }
}

/// Dispatch a single demuxed frame. Returns 1 if a `StatusHeartbeat` was processed.
fn dispatch_frame(frame: Frame, cb: Option<&(dyn Fn(&[u32]) + Send + Sync)>) -> usize {
    let Frame::Kalico { channel, payload } = frame else {
        return 0;
    };
    if channel != CHANNEL_EVENTS {
        return 0;
    }
    let Some((hdr, body)) = decode_message_header(&payload) else {
        return 0;
    };
    if MessageKind::from_u16(hdr.kind_raw) != Some(MessageKind::StatusHeartbeat) {
        return 0;
    }
    let Ok(hb) = StatusHeartbeat::decode(body) else {
        return 0;
    };
    if let Some(cb) = cb {
        cb(&hb.retired_counts);
    }
    1
}

impl UnixNativeConn {
    /// Issue a kalico-native call on an explicit outbound `channel`. The
    /// response always arrives on the control channel matched by
    /// `correlation_id`. Mirrors `KalicoHostIo::kalico_call_on_channel`.
    ///
    /// Used by the pump's `WireSink` to send `PushPieces` on
    /// `KALICO_CHANNEL_PIECES` (0x02) while receiving `PushPiecesResponse`
    /// on the control channel.
    pub fn kalico_call_on_channel(
        &self,
        channel: u8,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError> {
        let cid = self.next_cid.fetch_add(1, Ordering::Relaxed);
        let frame = build_kalico_frame(channel, kind, cid, &body);

        let cb = {
            let g = self
                .heartbeat_callback
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            g.clone()
        };

        let mut st = self.state.lock().unwrap_or_else(|p| p.into_inner());

        st.stream.write_all(&frame).map_err(TransportError::Io)?;

        st.stream
            .set_read_timeout(Some(Duration::from_millis(50)))
            .map_err(TransportError::Io)?;

        let deadline = Instant::now() + timeout;
        loop {
            if Instant::now() >= deadline {
                return Err(TransportError::Timeout);
            }
            let ConnState { stream, demux, buf } = &mut *st;
            let n = match stream.read(buf) {
                Ok(0) => return Err(TransportError::Closed),
                Ok(n) => n,
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    continue;
                }
                Err(e) => return Err(TransportError::Io(e)),
            };
            let (frames, _errs) = demux.feed_slice(&buf[..n]);
            for f in frames {
                // Route event-channel frames (StatusHeartbeat) to the heartbeat
                // callback even while waiting for the control-channel response.
                if let Frame::Kalico { channel: ch, .. } = &f {
                    if *ch == CHANNEL_EVENTS {
                        dispatch_frame(f, cb.as_deref());
                        continue;
                    }
                }
                if let Frame::Kalico { payload, .. } = f {
                    if let Some((hdr, resp_body)) = decode_message_header(&payload) {
                        if hdr.correlation_id == cid {
                            let resp_kind =
                                MessageKind::from_u16(hdr.kind_raw).ok_or_else(|| {
                                    TransportError::Parse(format!(
                                        "unknown response kind 0x{:04x}",
                                        hdr.kind_raw
                                    ))
                                })?;
                            return Ok((resp_kind, resp_body.to_vec()));
                        }
                    }
                }
            }
        }
    }
}

impl NativeCall for UnixNativeConn {
    fn kalico_call(
        &self,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(MessageKind, Vec<u8>), TransportError> {
        let cid = self.next_cid.fetch_add(1, Ordering::Relaxed);
        let frame = build_kalico_control_frame(kind, cid, &body);

        let cb = {
            let g = self
                .heartbeat_callback
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            g.clone()
        };

        let mut st = self.state.lock().unwrap_or_else(|p| p.into_inner());

        st.stream.write_all(&frame).map_err(TransportError::Io)?;

        // Bound each blocking read so the deadline is honoured even if the
        // peer goes silent.
        st.stream
            .set_read_timeout(Some(Duration::from_millis(50)))
            .map_err(TransportError::Io)?;

        let deadline = Instant::now() + timeout;
        loop {
            if Instant::now() >= deadline {
                return Err(TransportError::Timeout);
            }
            let ConnState { stream, demux, buf } = &mut *st;
            let n = match stream.read(buf) {
                Ok(0) => return Err(TransportError::Closed),
                Ok(n) => n,
                Err(ref e)
                    if e.kind() == std::io::ErrorKind::WouldBlock
                        || e.kind() == std::io::ErrorKind::TimedOut =>
                {
                    continue;
                }
                Err(e) => return Err(TransportError::Io(e)),
            };
            let (frames, _errs) = demux.feed_slice(&buf[..n]);
            for f in frames {
                if let Frame::Kalico { channel, .. } = &f {
                    if *channel == CHANNEL_EVENTS {
                        dispatch_frame(f, cb.as_deref());
                        continue;
                    }
                }
                if let Frame::Kalico { payload, .. } = f {
                    if let Some((hdr, resp_body)) = decode_message_header(&payload) {
                        if hdr.correlation_id == cid {
                            let resp_kind =
                                MessageKind::from_u16(hdr.kind_raw).ok_or_else(|| {
                                    TransportError::Parse(format!(
                                        "unknown response kind 0x{:04x}",
                                        hdr.kind_raw
                                    ))
                                })?;
                            return Ok((resp_kind, resp_body.to_vec()));
                        }
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use kalico_native_transport::frame::{CHANNEL_CONTROL, CHANNEL_EVENTS, encode_frame};
    use kalico_native_transport::wire_helpers::{MESSAGE_VERSION_DEFAULT, encode_message_header};
    use kalico_protocol::codec::Encode;
    use std::thread;

    /// Stub endpoint: read one framed request, reply with `reply_kind` echoing
    /// the request's correlation id and a fixed body.
    fn spawn_stub(mut peer: UnixStream, reply_kind: MessageKind, reply_body: Vec<u8>) {
        thread::spawn(move || {
            let mut demux = Demuxer::new();
            let mut buf = [0u8; 4096];
            loop {
                let n = match peer.read(&mut buf) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => n,
                };
                let (frames, _e) = demux.feed_slice(&buf[..n]);
                for f in frames {
                    if let Frame::Kalico { payload, .. } = f {
                        let (hdr, _b) = decode_message_header(&payload).unwrap();
                        let mut out = encode_message_header(
                            reply_kind,
                            MESSAGE_VERSION_DEFAULT,
                            hdr.correlation_id,
                        )
                        .to_vec();
                        out.extend_from_slice(&reply_body);
                        let frame = encode_frame(CHANNEL_CONTROL, &out);
                        peer.write_all(&frame).unwrap();
                        return;
                    }
                }
            }
        });
    }

    #[test]
    fn round_trips_a_call_by_correlation_id() {
        let (client, server) = UnixStream::pair().unwrap();
        spawn_stub(
            server,
            MessageKind::PushPiecesResponse,
            vec![0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        );
        let conn = UnixNativeConn::from_stream(client);
        let (kind, _body) = conn
            .kalico_call(MessageKind::PushPieces, vec![0; 8], Duration::from_secs(2))
            .expect("call ok");
        assert_eq!(kind, MessageKind::PushPiecesResponse);
    }

    #[test]
    fn times_out_when_peer_silent() {
        let (client, _server) = UnixStream::pair().unwrap();
        // _server never replies.
        let conn = UnixNativeConn::from_stream(client);
        let r = conn.kalico_call(MessageKind::PushPieces, vec![], Duration::from_millis(150));
        assert!(matches!(r, Err(TransportError::Timeout)));
    }

    /// Helper: build a `StatusHeartbeat` event frame as the endpoint would.
    fn make_heartbeat_frame(retired_counts: &[u32]) -> Vec<u8> {
        let hb = StatusHeartbeat {
            engine_state: 1,
            fault_code: 0,
            retired_counts: retired_counts.to_vec(),
        };
        let body = hb.encoded_to_vec();
        let mut payload =
            encode_message_header(MessageKind::StatusHeartbeat, MESSAGE_VERSION_DEFAULT, 0)
                .to_vec();
        payload.extend_from_slice(&body);
        encode_frame(CHANNEL_EVENTS, &payload)
    }

    /// Stub endpoint: send a `StatusHeartbeat` event frame *before* replying to
    /// the control-channel call. Verifies that `kalico_call` drains the event
    /// frame and invokes the heartbeat callback without mistaking it for the
    /// correlated response.
    fn spawn_stub_with_event(
        mut peer: UnixStream,
        reply_kind: MessageKind,
        reply_body: Vec<u8>,
        event_before_reply: Vec<u8>,
    ) {
        thread::spawn(move || {
            let mut demux = Demuxer::new();
            let mut buf = [0u8; 4096];
            loop {
                let n = match peer.read(&mut buf) {
                    Ok(0) | Err(_) => return,
                    Ok(n) => n,
                };
                let (frames, _e) = demux.feed_slice(&buf[..n]);
                for f in frames {
                    if let Frame::Kalico { payload, .. } = f {
                        let (hdr, _b) = decode_message_header(&payload).unwrap();
                        // Send the event first.
                        peer.write_all(&event_before_reply).unwrap();
                        // Then send the correlated response.
                        let mut out = encode_message_header(
                            reply_kind,
                            MESSAGE_VERSION_DEFAULT,
                            hdr.correlation_id,
                        )
                        .to_vec();
                        out.extend_from_slice(&reply_body);
                        let frame = encode_frame(CHANNEL_CONTROL, &out);
                        peer.write_all(&frame).unwrap();
                        return;
                    }
                }
            }
        });
    }

    #[test]
    fn heartbeat_event_during_call_invokes_callback() {
        use std::sync::atomic::{AtomicU32, Ordering};

        let (client, server) = UnixStream::pair().unwrap();
        let hb_frame = make_heartbeat_frame(&[42u32]);
        let resp_body = vec![0u8; 20]; // PushPiecesResponse: i32 + u64 + u64
        spawn_stub_with_event(server, MessageKind::PushPiecesResponse, resp_body, hb_frame);

        let conn = UnixNativeConn::from_stream(client);
        let last_retired = Arc::new(AtomicU32::new(0));
        let lr = Arc::clone(&last_retired);
        conn.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
            if let Some(&v) = retired.first() {
                lr.store(v, Ordering::SeqCst);
            }
        }));

        let (kind, _body) = conn
            .kalico_call(MessageKind::PushPieces, vec![0; 8], Duration::from_secs(2))
            .expect("call ok");
        assert_eq!(kind, MessageKind::PushPiecesResponse);
        // The heartbeat callback must have been called with retired_counts=[42].
        assert_eq!(last_retired.load(Ordering::SeqCst), 42);
    }

    #[test]
    fn poll_events_drains_heartbeat_frames() {
        use std::sync::atomic::{AtomicU32, Ordering};

        let (client, server) = UnixStream::pair().unwrap();
        // Write two heartbeat frames from the "server" side before polling.
        {
            let mut s = server;
            s.write_all(&make_heartbeat_frame(&[3u32])).unwrap();
            s.write_all(&make_heartbeat_frame(&[7u32])).unwrap();
        }

        let conn = UnixNativeConn::from_stream(client);
        let last_retired = Arc::new(AtomicU32::new(0));
        let lr = Arc::clone(&last_retired);
        conn.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
            if let Some(&v) = retired.first() {
                lr.store(v, Ordering::SeqCst);
            }
        }));

        // Poll should process both frames.
        let count = conn.poll_events();
        assert_eq!(count, 2, "expected 2 StatusHeartbeat frames");
        // Last callback invocation carries retired_counts=[7].
        assert_eq!(last_retired.load(Ordering::SeqCst), 7);
    }
}
