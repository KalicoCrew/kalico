use std::fs;
use std::io::Read as IoRead;
use std::process::{Child, Command};
use std::thread;
use std::time::{Duration, Instant};

use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Cursor, Decode, Encode};
use kalico_protocol::messages::{
    ClaimHandshakeReply, MessageKind, StartCapture, StartCaptureResponse, StopCapture,
    StopCaptureResponse,
};

use kalico_ethercat_rt::capture::{ERR_CAPTURE_ACTIVE, ERR_CAPTURE_FILE, ERR_CAPTURE_NOT_ACTIVE, RECORD_SIZE};

const STUB_BIN: &str = env!("CARGO_BIN_EXE_kalico-ethercat-rt-stub");

struct ChildGuard {
    child: Option<Child>,
}

impl ChildGuard {
    fn new(child: Child) -> Self {
        Self { child: Some(child) }
    }

    fn defuse(&mut self) -> Child {
        self.child.take().expect("already defused")
    }
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if let Some(mut c) = self.child.take() {
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

fn socket_path(tag: &str) -> String {
    format!("/tmp/kalico-cap-{}-{}.sock", tag, std::process::id())
}

fn capture_file(tag: &str) -> String {
    format!(
        "/tmp/kalico-capture-it-{}-{}.scap",
        tag,
        std::process::id()
    )
}

fn wait_for_socket(path: &str, deadline: Instant) {
    loop {
        if std::path::Path::new(path).exists() {
            return;
        }
        assert!(
            Instant::now() < deadline,
            "stub socket {path:?} did not appear within deadline"
        );
        thread::sleep(Duration::from_millis(10));
    }
}

fn do_handshake(conn: &UnixNativeConn) -> ClaimHandshakeReply {
    let (kind, body) = conn
        .kalico_call(
            MessageKind::ClaimHandshake,
            Vec::new(),
            Duration::from_secs(5),
        )
        .expect("ClaimHandshake kalico_call must succeed");

    assert_eq!(
        kind,
        MessageKind::ClaimHandshakeReply,
        "expected ClaimHandshakeReply (0x{:04x}), got kind 0x{:04x}",
        MessageKind::ClaimHandshakeReply.as_u16(),
        kind.as_u16(),
    );

    ClaimHandshakeReply::decode_from(&mut Cursor::new(&body))
        .expect("ClaimHandshakeReply must decode from response body")
}

fn spawn_and_claim(tag: &str) -> (ChildGuard, UnixNativeConn, String) {
    let path = socket_path(tag);
    let _ = fs::remove_file(&path);

    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .spawn()
        .expect("stub binary must spawn");
    let guard = ChildGuard::new(child);

    wait_for_socket(&path, Instant::now() + Duration::from_secs(5));

    let conn = UnixNativeConn::connect(&path).expect("UnixNativeConn::connect must succeed");
    let _reply = do_handshake(&conn);

    (guard, conn, path)
}

fn start_capture(conn: &UnixNativeConn, path: &str) -> i32 {
    let body = StartCapture {
        path: path.to_owned(),
        started_utc: "2026-06-10T12:00:00Z".to_owned(),
        drive_name: "x".to_owned(),
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::StartCapture, body, Duration::from_secs(5))
        .expect("StartCapture call must succeed");
    assert_eq!(
        kind,
        MessageKind::StartCaptureResponse,
        "expected StartCaptureResponse (0x{:04x}), got 0x{:04x}",
        MessageKind::StartCaptureResponse.as_u16(),
        kind.as_u16()
    );
    StartCaptureResponse::decode_from(&mut Cursor::new(&resp))
        .expect("StartCaptureResponse must decode")
        .result
}

fn stop_capture(conn: &UnixNativeConn) -> StopCaptureResponse {
    let body = StopCapture.encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::StopCapture, body, Duration::from_secs(5))
        .expect("StopCapture call must succeed");
    assert_eq!(
        kind,
        MessageKind::StopCaptureResponse,
        "expected StopCaptureResponse (0x{:04x}), got 0x{:04x}",
        MessageKind::StopCaptureResponse.as_u16(),
        kind.as_u16()
    );
    StopCaptureResponse::decode_from(&mut Cursor::new(&resp))
        .expect("StopCaptureResponse must decode")
}

#[test]
fn capture_start_records_stop_produces_consistent_file() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-basic");
    let path = capture_file("basic");
    let _ = fs::remove_file(&path);

    let rc = start_capture(&conn, &path);
    assert_eq!(rc, 0, "StartCapture must return 0, got {rc}");

    thread::sleep(Duration::from_millis(500));

    let resp = stop_capture(&conn);
    assert_eq!(resp.result, 0, "StopCapture result must be 0, got {}", resp.result);
    assert!(
        resp.samples > 100,
        "expected >100 samples after 500 ms, got {}",
        resp.samples
    );
    assert_eq!(
        resp.overflow_cycle,
        StopCaptureResponse::NO_OVERFLOW,
        "expected no overflow, got overflow_cycle={}",
        resp.overflow_cycle
    );

    let mut file = fs::File::open(&path).expect("capture file must exist after stop");
    let mut contents = Vec::new();
    file.read_to_end(&mut contents)
        .expect("capture file must be readable");

    let newline_pos = contents
        .iter()
        .position(|&b| b == b'\n')
        .expect("capture file must contain a header newline");
    let header = std::str::from_utf8(&contents[..newline_pos])
        .expect("header must be valid UTF-8");
    assert!(
        header.contains("\"version\":1"),
        "header must contain \"version\":1; header={header:?}"
    );
    assert!(
        header.contains(&format!("\"record_size\":{RECORD_SIZE}")),
        "header must contain \"record_size\":{RECORD_SIZE}; header={header:?}"
    );

    let body = &contents[newline_pos + 1..];
    assert_eq!(
        body.len() % RECORD_SIZE,
        0,
        "body length {} is not a multiple of RECORD_SIZE {}",
        body.len(),
        RECORD_SIZE
    );
    let file_records = body.len() / RECORD_SIZE;
    assert_eq!(
        file_records,
        resp.samples as usize,
        "file record count {file_records} must equal samples {} from StopCaptureResponse",
        resp.samples
    );

    let _ = fs::remove_file(&path);
    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}

#[test]
fn double_start_rejected_without_killing_first_capture() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-dbl");
    let path1 = capture_file("dbl-1");
    let path2 = capture_file("dbl-2");
    let _ = fs::remove_file(&path1);
    let _ = fs::remove_file(&path2);

    let rc1 = start_capture(&conn, &path1);
    assert_eq!(rc1, 0, "first StartCapture must return 0, got {rc1}");

    let rc2 = start_capture(&conn, &path2);
    assert_eq!(
        rc2,
        ERR_CAPTURE_ACTIVE,
        "second StartCapture must return ERR_CAPTURE_ACTIVE ({ERR_CAPTURE_ACTIVE}), got {rc2}"
    );

    let resp = stop_capture(&conn);
    assert_eq!(
        resp.result, 0,
        "StopCapture after double-start must return 0, got {}",
        resp.result
    );

    assert!(
        !std::path::Path::new(&path2).exists(),
        "second capture file {path2:?} must NOT exist after rejected start"
    );

    let _ = fs::remove_file(&path1);
    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}

#[test]
fn stop_without_start_rejected() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-no-start");

    let resp = stop_capture(&conn);
    assert_eq!(
        resp.result,
        ERR_CAPTURE_NOT_ACTIVE,
        "StopCapture without active capture must return ERR_CAPTURE_NOT_ACTIVE ({ERR_CAPTURE_NOT_ACTIVE}), got {}",
        resp.result
    );
    assert_eq!(
        resp.samples, 0,
        "samples must be 0 when no capture was active, got {}",
        resp.samples
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}

#[test]
fn unwritable_path_reports_file_error() {
    let (mut guard, conn, sock) = spawn_and_claim("cap-bad-path");

    let rc = start_capture(&conn, "/dev/null/nope/x.scap");
    assert_eq!(
        rc,
        ERR_CAPTURE_FILE,
        "StartCapture with unwritable path must return ERR_CAPTURE_FILE ({ERR_CAPTURE_FILE}), got {rc}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = fs::remove_file(&sock);
}
