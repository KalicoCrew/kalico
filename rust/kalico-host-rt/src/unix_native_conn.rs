use std::collections::HashMap;
use std::io::{self, ErrorKind, Read, Write};
use std::net::Shutdown;
use std::os::unix::net::UnixStream;
use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::mpsc::{RecvTimeoutError, SyncSender, sync_channel};
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::Duration;

use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::frame::CHANNEL_EVENTS;
use kalico_native_transport::wire_helpers::decode_message_header;
use kalico_protocol::codec::Decode;
use kalico_protocol::messages::{MessageKind, StatusHeartbeat};

use crate::host_io::kalico_native::{build_kalico_control_frame, build_kalico_frame};
use crate::native_call::NativeCall;
use crate::transport::TransportError;

type CallResult = Result<(MessageKind, Vec<u8>), TransportError>;
type HeartbeatCallback = Arc<dyn Fn(&StatusHeartbeat) + Send + Sync>;

struct Pending {
    waiters: HashMap<u32, SyncSender<CallResult>>,
    closed: Option<TransportError>,
}

struct Shared {
    pending: Mutex<Pending>,
    heartbeat_callback: Mutex<Option<HeartbeatCallback>>,
    peer_closed: AtomicBool,
    write_stream: Mutex<UnixStream>,
}

impl Shared {
    fn latch_closed(&self, err: TransportError) {
        let mut pending = self.pending.lock().unwrap_or_else(|p| p.into_inner());
        if pending.closed.is_none() {
            pending.closed = Some(clone_transport_error(&err));
        }
        // Set peer_closed BEFORE draining waiters: a woken waiter that returns
        // to its caller must never observe peer_closed()==false afterward. The
        // store is Release; the waiter's Acquire load in peer_closed() pairs with
        // it, so once the error is sent the flag is already visible.
        self.peer_closed.store(true, Ordering::Release);
        for (_cid, tx) in pending.waiters.drain() {
            let _ = tx.send(Err(clone_transport_error(&err)));
        }
    }
}

/// `TransportError` is not `Clone` (it holds an `io::Error`).  Reconstruct the
/// variant by discriminant; the pump only inspects the variant for Fatal vs
/// Transient classification (`Closed | Io(_)`), never the inner message.
fn clone_transport_error(err: &TransportError) -> TransportError {
    match err {
        TransportError::Io(e) => TransportError::Io(io::Error::from(e.kind())),
        TransportError::Timeout => TransportError::Timeout,
        TransportError::Closed => TransportError::Closed,
        TransportError::Parse(s) => TransportError::Parse(s.clone()),
        TransportError::DispatcherTimeout => TransportError::DispatcherTimeout,
        TransportError::Backpressure => TransportError::Backpressure,
    }
}

pub struct UnixNativeConn {
    shared: Arc<Shared>,
    next_cid: AtomicU32,
    reader_handle: Option<JoinHandle<()>>,
    read_stream_for_shutdown: UnixStream,
}

impl core::fmt::Debug for UnixNativeConn {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("UnixNativeConn")
            .field("next_cid", &self.next_cid.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl UnixNativeConn {
    pub fn connect(path: &str) -> io::Result<Self> {
        let stream = UnixStream::connect(path)?;
        Self::from_stream(stream)
    }

    pub fn from_stream(stream: UnixStream) -> io::Result<Self> {
        // Blocking reads with no SO_RCVTIMEO: the reader thread is the only
        // reader; a fixed read timeout here is the ~4ms CONFIG_HZ stall that
        // caused -308.  The reader is woken from a blocked read only by inbound
        // bytes, peer EOF, or our own shutdown(Both) in Drop.
        stream.set_read_timeout(None)?;

        let read_clone = stream.try_clone()?;
        let shutdown_clone = stream.try_clone()?;

        let shared = Arc::new(Shared {
            pending: Mutex::new(Pending {
                waiters: HashMap::new(),
                closed: None,
            }),
            heartbeat_callback: Mutex::new(None),
            peer_closed: AtomicBool::new(false),
            write_stream: Mutex::new(stream),
        });

        let reader_shared = Arc::clone(&shared);
        let reader_handle = std::thread::Builder::new()
            .name("ec-conn-reader".into())
            .spawn(move || run_reader(reader_shared, read_clone))
            .expect("spawn ec-conn-reader thread");

        Ok(Self {
            shared,
            next_cid: AtomicU32::new(1),
            reader_handle: Some(reader_handle),
            read_stream_for_shutdown: shutdown_clone,
        })
    }

    /// Returns `true` once the peer has closed the socket or a fatal I/O error
    /// has been observed.  Set by the reader thread on EOF/IO and by the write
    /// path on write failure; it never resets.
    pub fn peer_closed(&self) -> bool {
        self.shared.peer_closed.load(Ordering::Acquire)
    }

    pub fn attach_heartbeat_callback(&self, cb: HeartbeatCallback) {
        let mut guard = self
            .shared
            .heartbeat_callback
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        *guard = Some(cb);
    }

    fn call(&self, frame: &[u8], cid: u32, timeout: Duration) -> CallResult {
        let (tx, rx) = sync_channel::<CallResult>(1);

        {
            let mut pending = self
                .shared
                .pending
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if let Some(err) = &pending.closed {
                return Err(clone_transport_error(err));
            }
            // Insert into `waiters` before write_all — a response can arrive on
            // the reader thread before our write returns; inserting after would
            // drop it.
            pending.waiters.insert(cid, tx);
        }

        {
            let mut ws = self
                .shared
                .write_stream
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if let Err(e) = ws.write_all(frame) {
                drop(ws);
                let mut pending = self
                    .shared
                    .pending
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                pending.waiters.remove(&cid);
                if pending.closed.is_none() {
                    pending.closed = Some(TransportError::Io(io::Error::from(e.kind())));
                }
                // Same ordering invariant as latch_closed: flip peer_closed
                // before waking the other waiters.
                self.shared.peer_closed.store(true, Ordering::Release);
                for (_cid, tx) in pending.waiters.drain() {
                    let _ = tx.send(Err(TransportError::Io(io::Error::from(e.kind()))));
                }
                return Err(TransportError::Io(e));
            }
        }

        match rx.recv_timeout(timeout) {
            Ok(result) => result,
            Err(RecvTimeoutError::Timeout) => {
                let mut pending = self
                    .shared
                    .pending
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                pending.waiters.remove(&cid);
                Err(TransportError::Timeout)
            }
            Err(RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }

    pub fn kalico_call_on_channel(
        &self,
        channel: u8,
        kind: MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> CallResult {
        let cid = self.next_cid.fetch_add(1, Ordering::Relaxed);
        let frame = build_kalico_frame(channel, kind, cid, &body);
        self.call(&frame, cid, timeout)
    }
}

impl NativeCall for UnixNativeConn {
    fn kalico_call(&self, kind: MessageKind, body: Vec<u8>, timeout: Duration) -> CallResult {
        let cid = self.next_cid.fetch_add(1, Ordering::Relaxed);
        let frame = build_kalico_control_frame(kind, cid, &body);
        self.call(&frame, cid, timeout)
    }
}

impl Drop for UnixNativeConn {
    fn drop(&mut self) {
        // shutdown(Both) before join: the reader is parked in a blocking read
        // with no timeout; closing the socket is the only thing that wakes it.
        // Without this, join() would hang until the peer happens to write/close.
        let _ = self.read_stream_for_shutdown.shutdown(Shutdown::Both);
        if let Some(h) = self.reader_handle.take() {
            let _ = h.join();
        }
    }
}

fn run_reader(shared: Arc<Shared>, mut stream: UnixStream) {
    let mut demux = Demuxer::new();
    let mut buf = [0u8; 4096];
    loop {
        match stream.read(&mut buf) {
            Ok(0) => {
                shared.latch_closed(TransportError::Closed);
                return;
            }
            Ok(n) => {
                // Snapshot the callback AFTER read() returns, not before: a clone
                // taken before parking in read() would latch the value at
                // park-time, dropping any heartbeat that arrives in the window
                // between attach_heartbeat_callback and the next read wakeup.
                let cb = {
                    let g = shared
                        .heartbeat_callback
                        .lock()
                        .unwrap_or_else(|p| p.into_inner());
                    g.clone()
                };
                let (frames, _errs) = demux.feed_slice(&buf[..n]);
                for f in frames {
                    route_frame(&shared, f, cb.as_deref());
                }
            }
            Err(ref e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(e) => {
                shared.latch_closed(TransportError::Io(e));
                return;
            }
        }
    }
}

fn route_frame(shared: &Shared, frame: Frame, cb: Option<&(dyn Fn(&StatusHeartbeat) + Send + Sync)>) {
    let Frame::Kalico { channel, payload } = &frame else {
        return;
    };
    if *channel == CHANNEL_EVENTS {
        dispatch_frame(frame, cb);
        return;
    }
    let Some((hdr, resp_body)) = decode_message_header(payload) else {
        return;
    };
    let cid = hdr.correlation_id;
    let tx = {
        let mut pending = shared.pending.lock().unwrap_or_else(|p| p.into_inner());
        pending.waiters.remove(&cid)
    };
    let Some(tx) = tx else {
        log::warn!("UnixNativeConn: response for unknown correlation_id {cid} dropped");
        return;
    };
    let result = match MessageKind::from_u16(hdr.kind_raw) {
        Some(resp_kind) => Ok((resp_kind, resp_body.to_vec())),
        None => Err(TransportError::Parse(format!(
            "unknown response kind 0x{:04x}",
            hdr.kind_raw
        ))),
    };
    let _ = tx.send(result);
}

fn dispatch_frame(frame: Frame, cb: Option<&(dyn Fn(&StatusHeartbeat) + Send + Sync)>) -> usize {
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
        cb(&hb);
    }
    1
}

#[cfg(test)]
mod tests;
