use super::*;
use kalico_native_transport::frame::{encode_frame, CHANNEL_CONTROL, CHANNEL_EVENTS};
use kalico_native_transport::wire_helpers::{encode_message_header, MESSAGE_VERSION_DEFAULT};
use kalico_protocol::codec::Encode;
use std::sync::atomic::AtomicU32;
use std::thread;
use std::time::Instant;

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
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let (kind, _body) = conn
        .kalico_call(MessageKind::PushPieces, vec![0; 8], Duration::from_secs(2))
        .expect("call ok");
    assert_eq!(kind, MessageKind::PushPiecesResponse);
}

#[test]
fn timeout_still_timeout_when_peer_alive_silent() {
    // Keep the server end alive but silent: it stays connected and never
    // replies, so the call must elapse its deadline -> Timeout (not Closed,
    // which would fire on EOF).
    let (client, server) = UnixStream::pair().unwrap();
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let r = conn.kalico_call(MessageKind::PushPieces, vec![], Duration::from_millis(150));
    assert!(matches!(r, Err(TransportError::Timeout)), "got {r:?}");
    drop(server);
}

#[test]
fn reader_death_wakes_waiter_with_closed() {
    // Stub reads the request then drops without replying -> reader sees EOF
    // -> latch_closed(Closed) -> the waiting call returns Err(Closed) well
    // before its own (long) deadline.
    let (client, mut server) = UnixStream::pair().unwrap();
    thread::spawn(move || {
        let mut buf = [0u8; 4096];
        let _ = server.read(&mut buf);
        server.shutdown(Shutdown::Both).ok();
        drop(server);
    });
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let start = Instant::now();
    let r = conn.kalico_call(MessageKind::PushPieces, vec![0; 8], Duration::from_secs(10));
    assert!(matches!(r, Err(TransportError::Closed)), "got {r:?}");
    assert!(
        start.elapsed() < Duration::from_secs(2),
        "waiter was woken by death, not by deadline: {:?}",
        start.elapsed()
    );
    assert!(conn.peer_closed(), "peer_closed must be set after reader death");
}

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
                    peer.write_all(&event_before_reply).unwrap();
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
    let (client, server) = UnixStream::pair().unwrap();
    let hb_frame = make_heartbeat_frame(&[42u32]);
    let resp_body = vec![0u8; 20]; // PushPiecesResponse: i32 + u64 + u64
    spawn_stub_with_event(server, MessageKind::PushPiecesResponse, resp_body, hb_frame);

    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
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

    // Callback and response now both land on the reader thread; the callback
    // may fire just after the call returns, so spin briefly.
    let deadline = Instant::now() + Duration::from_secs(2);
    while last_retired.load(Ordering::SeqCst) != 42 {
        assert!(Instant::now() < deadline, "heartbeat callback never fired");
        thread::sleep(Duration::from_millis(1));
    }
}

/// Streams a fixed heartbeat continuously (no reply path). Returns a stop
/// flag the caller sets to terminate the writer + drop the socket.
fn spawn_heartbeat_stream(peer: UnixStream, retired: &[u32]) -> Arc<AtomicBool> {
    let hb = make_heartbeat_frame(retired);
    let stop = Arc::new(AtomicBool::new(false));
    let stop_w = Arc::clone(&stop);
    thread::spawn(move || {
        let mut peer = peer;
        while !stop_w.load(Ordering::Acquire) {
            if peer.write_all(&hb).is_err() {
                break;
            }
            thread::sleep(Duration::from_millis(1));
        }
    });
    stop
}

#[test]
fn heartbeat_callback_fires_with_no_call_in_flight() {
    // Stream heartbeats continuously rather than pre-buffering one-shots:
    // pre-buffered frames can be read+dispatched before the callback is
    // attached (cb==None) and silently dropped. A continuous stream
    // guarantees a heartbeat arrives AFTER attach.
    let (client, server) = UnixStream::pair().unwrap();
    let stop = spawn_heartbeat_stream(server, &[7u32]);

    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let last_retired = Arc::new(AtomicU32::new(0));
    let lr = Arc::clone(&last_retired);
    conn.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
        if let Some(&v) = retired.first() {
            lr.store(v, Ordering::SeqCst);
        }
    }));

    let deadline = Instant::now() + Duration::from_secs(2);
    while last_retired.load(Ordering::SeqCst) != 7 {
        assert!(
            Instant::now() < deadline,
            "heartbeats did not reach the callback (last={})",
            last_retired.load(Ordering::SeqCst)
        );
        thread::sleep(Duration::from_millis(1));
    }
    stop.store(true, Ordering::Release);
}

fn spawn_streaming_stub(mut peer: UnixStream, hb_period: Duration) {
    // Streams heartbeats continuously AND replies promptly to each call.
    thread::spawn(move || {
        let mut writer = peer.try_clone().unwrap();
        let hb = make_heartbeat_frame(&[0u32]);
        let stop = Arc::new(AtomicBool::new(false));
        let stop_w = Arc::clone(&stop);
        let hb_thread = thread::spawn(move || {
            while !stop_w.load(Ordering::Acquire) {
                if writer.write_all(&hb).is_err() {
                    break;
                }
                thread::sleep(hb_period);
            }
        });

        let mut demux = Demuxer::new();
        let mut buf = [0u8; 4096];
        loop {
            let n = match peer.read(&mut buf) {
                Ok(0) | Err(_) => break,
                Ok(n) => n,
            };
            let (frames, _e) = demux.feed_slice(&buf[..n]);
            for f in frames {
                if let Frame::Kalico { payload, .. } = f {
                    let (hdr, _b) = decode_message_header(&payload).unwrap();
                    let mut out = encode_message_header(
                        MessageKind::PushPiecesResponse,
                        MESSAGE_VERSION_DEFAULT,
                        hdr.correlation_id,
                    )
                    .to_vec();
                    out.extend_from_slice(&[0u8; 20]);
                    let frame = encode_frame(CHANNEL_CONTROL, &out);
                    if peer.write_all(&frame).is_err() {
                        stop.store(true, Ordering::Release);
                        let _ = hb_thread.join();
                        return;
                    }
                }
            }
        }
        stop.store(true, Ordering::Release);
        let _ = hb_thread.join();
    });
}

#[test]
fn concurrent_call_does_not_inflate_rtt_while_heartbeats_flow() {
    const CALLS: usize = 50;
    // Inflate the heartbeat period far past any plausible scheduler-wakeup
    // cost so the regression boundary is unambiguous under CPU contention
    // (the Pi runs builds at -j$(nproc), so contention is the normal case).
    // Old design: each call serialized behind one ~HB_PERIOD heartbeat read
    // -> CALLS * HB_PERIOD ~= 1000ms. New design: calls are decoupled from
    // the heartbeat cadence, so total wall time is dominated by 50 socket
    // round-trips (sub-ms each, low-ms even with heavy slack). The 4x-below
    // -serialized budget sits ~5x above realistic worst case, so neither a
    // false-fail (contention) nor a false-pass (regression) is possible.
    const HB_PERIOD: Duration = Duration::from_millis(20);
    let serialized_floor = HB_PERIOD * CALLS as u32;
    let budget = serialized_floor / 4;

    let (client, server) = UnixStream::pair().unwrap();
    spawn_streaming_stub(server, HB_PERIOD);
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");

    let start = Instant::now();
    for _ in 0..CALLS {
        let (kind, _b) = conn
            .kalico_call(MessageKind::PushPieces, vec![0; 8], Duration::from_secs(2))
            .expect("call ok");
        assert_eq!(kind, MessageKind::PushPiecesResponse);
    }
    let elapsed = start.elapsed();
    assert!(
        elapsed < budget,
        "calls serialized behind heartbeats: {CALLS} calls took {elapsed:?} \
         (budget {budget:?}, serialized floor {serialized_floor:?})"
    );
}
