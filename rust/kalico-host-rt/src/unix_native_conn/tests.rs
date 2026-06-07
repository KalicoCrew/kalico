use super::*;
use kalico_native_transport::frame::{CHANNEL_CONTROL, CHANNEL_EVENTS, encode_frame};
use kalico_native_transport::wire_helpers::{MESSAGE_VERSION_DEFAULT, encode_message_header};
use kalico_protocol::codec::Encode;
use std::thread;

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
    let conn = UnixNativeConn::from_stream(client);
    let r = conn.kalico_call(MessageKind::PushPieces, vec![], Duration::from_millis(150));
    assert!(matches!(r, Err(TransportError::Timeout)));
}

fn make_heartbeat_frame(retired_counts: &[u32]) -> Vec<u8> {
    let hb = StatusHeartbeat {
        engine_state: 1,
        fault_code: 0,
        retired_counts: retired_counts.to_vec(),
    };
    let body = hb.encoded_to_vec();
    let mut payload =
        encode_message_header(MessageKind::StatusHeartbeat, MESSAGE_VERSION_DEFAULT, 0).to_vec();
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
    assert_eq!(last_retired.load(Ordering::SeqCst), 42);
}

#[test]
fn poll_events_drains_heartbeat_frames() {
    use std::sync::atomic::{AtomicU32, Ordering};

    let (client, server) = UnixStream::pair().unwrap();
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

    let count = conn.poll_events();
    assert_eq!(count, 2, "expected 2 StatusHeartbeat frames");
    assert_eq!(last_retired.load(Ordering::SeqCst), 7);
}
