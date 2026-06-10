use super::*;
use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::frame::{CHANNEL_CONTROL, encode_frame};
use kalico_native_transport::wire_helpers::{
    MESSAGE_VERSION_DEFAULT, decode_message_header, encode_message_header,
};
use std::io::{Read, Write};
use std::os::unix::net::UnixStream;

fn spawn_start_endpoint(peer: UnixStream, result: i32) -> std::sync::mpsc::Receiver<StartCapture> {
    spawn_start_endpoint_with_kind(peer, MessageKind::StartCaptureResponse, {
        let resp = StartCaptureResponse { result };
        resp.encoded_to_vec()
    })
}

fn spawn_start_endpoint_with_kind(
    mut peer: UnixStream,
    kind: MessageKind,
    body: Vec<u8>,
) -> std::sync::mpsc::Receiver<StartCapture> {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
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
                    let (hdr, _body) =
                        decode_message_header(&payload).expect("valid message header");
                    let msg = StartCapture::decode(_body).expect("valid StartCapture body");
                    let _ = tx.send(msg);
                    let mut out =
                        encode_message_header(kind, MESSAGE_VERSION_DEFAULT, hdr.correlation_id)
                            .to_vec();
                    out.extend_from_slice(&body);
                    let frame = encode_frame(CHANNEL_CONTROL, &out);
                    peer.write_all(&frame).unwrap();
                    return;
                }
            }
        }
    });
    rx
}

fn spawn_stop_endpoint(
    peer: UnixStream,
    resp: StopCaptureResponse,
) -> std::sync::mpsc::Receiver<()> {
    spawn_stop_endpoint_with_kind(peer, MessageKind::StopCaptureResponse, resp.encoded_to_vec())
}

fn spawn_stop_endpoint_with_kind(
    mut peer: UnixStream,
    kind: MessageKind,
    body: Vec<u8>,
) -> std::sync::mpsc::Receiver<()> {
    let (tx, rx) = std::sync::mpsc::channel();
    std::thread::spawn(move || {
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
                    let (hdr, _body) =
                        decode_message_header(&payload).expect("valid message header");
                    let _ = tx.send(());
                    let mut out =
                        encode_message_header(kind, MESSAGE_VERSION_DEFAULT, hdr.correlation_id)
                            .to_vec();
                    out.extend_from_slice(&body);
                    let frame = encode_frame(CHANNEL_CONTROL, &out);
                    peer.write_all(&frame).unwrap();
                    return;
                }
            }
        }
    });
    rx
}

#[test]
fn start_capture_round_trips_fields_and_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let rx = spawn_start_endpoint(server, 0);
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let result =
        send_start_capture(&conn, "/tmp/cap.scap", "2026-06-10T00:00:00Z", "axis_x")
            .expect("call");
    assert_eq!(result, 0);
    let seen = rx.recv().expect("endpoint saw the command");
    assert_eq!(seen.path, "/tmp/cap.scap");
    assert_eq!(seen.started_utc, "2026-06-10T00:00:00Z");
    assert_eq!(seen.drive_name, "axis_x");
}

#[test]
fn start_capture_surfaces_nonzero_result() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_start_endpoint(server, -324);
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    assert_eq!(
        send_start_capture(&conn, "/tmp/cap.scap", "2026-06-10T00:00:00Z", "axis_x")
            .expect("call"),
        -324
    );
}

#[test]
fn start_capture_transport_error_is_err() {
    let (client, server) = UnixStream::pair().unwrap();
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    drop(server);
    assert!(
        send_start_capture(&conn, "/tmp/cap.scap", "2026-06-10T00:00:00Z", "axis_x").is_err()
    );
}

#[test]
fn start_capture_wrong_kind_response_is_rejected() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_start_endpoint_with_kind(
        server,
        MessageKind::PushPiecesResponse,
        vec![0u8; 20],
    );
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let err =
        send_start_capture(&conn, "/tmp/cap.scap", "2026-06-10T00:00:00Z", "axis_x")
            .expect_err("should error on wrong kind");
    assert!(err.contains("unexpected response kind"));
}

#[test]
fn stop_capture_round_trips_response() {
    let (client, server) = UnixStream::pair().unwrap();
    let expected = StopCaptureResponse {
        result: 0,
        samples: 1024,
        overflow_cycle: StopCaptureResponse::NO_OVERFLOW,
    };
    let _rx = spawn_stop_endpoint(server, expected);
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let resp = send_stop_capture(&conn).expect("call");
    assert_eq!(resp.result, 0);
    assert_eq!(resp.samples, 1024);
    assert_eq!(resp.overflow_cycle, StopCaptureResponse::NO_OVERFLOW);
}

#[test]
fn stop_capture_surfaces_overflow_cycle() {
    let (client, server) = UnixStream::pair().unwrap();
    let expected = StopCaptureResponse {
        result: -323,
        samples: 512,
        overflow_cycle: 42,
    };
    let _rx = spawn_stop_endpoint(server, expected);
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let resp = send_stop_capture(&conn).expect("call");
    assert_eq!(resp.result, -323);
    assert_eq!(resp.samples, 512);
    assert_eq!(resp.overflow_cycle, 42);
}

#[test]
fn stop_capture_transport_error_is_err() {
    let (client, server) = UnixStream::pair().unwrap();
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    drop(server);
    assert!(send_stop_capture(&conn).is_err());
}

#[test]
fn stop_capture_wrong_kind_response_is_rejected() {
    let (client, server) = UnixStream::pair().unwrap();
    let _rx = spawn_stop_endpoint_with_kind(server, MessageKind::PushPiecesResponse, vec![0u8; 20]);
    let conn = UnixNativeConn::from_stream(client).expect("from_stream");
    let err = send_stop_capture(&conn).expect_err("should error on wrong kind");
    assert!(err.contains("unexpected response kind"));
}
