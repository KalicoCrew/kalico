use std::process::{Child, Command};
use std::thread;
use std::time::{Duration, Instant};

use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{
    MessageKind, SdoRead, SdoReadResponse, SdoWrite, SdoWriteResponse, ERR_SDO_VALUE_RANGE,
    ERR_SDO_VERIFY_MISMATCH, SDO_SIZE_PROBE,
};

const STUB_BIN: &str = env!("CARGO_BIN_EXE_kalico-ethercat-rt-stub");
const COE_ABORT_READ_ONLY: i32 = 0x0601_0002;
const COE_ABORT_NOT_FOUND: i32 = 0x0602_0000;

struct ChildGuard {
    child: Option<Child>,
}

impl Drop for ChildGuard {
    fn drop(&mut self) {
        if let Some(mut c) = self.child.take() {
            let _ = c.kill();
            let _ = c.wait();
        }
    }
}

fn spawn_and_claim(tag: &str) -> (ChildGuard, UnixNativeConn) {
    let path = format!("/tmp/kalico-sdo-{}-{}.sock", tag, std::process::id());
    let _ = std::fs::remove_file(&path);
    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .spawn()
        .expect("stub binary must spawn");
    let guard = ChildGuard { child: Some(child) };

    let deadline = Instant::now() + Duration::from_secs(5);
    while !std::path::Path::new(&path).exists() {
        assert!(Instant::now() < deadline, "stub socket did not appear");
        thread::sleep(Duration::from_millis(10));
    }
    let conn = UnixNativeConn::connect(&path).expect("connect must succeed");
    let (kind, _) = conn
        .kalico_call(
            MessageKind::ClaimHandshake,
            Vec::new(),
            Duration::from_secs(5),
        )
        .expect("ClaimHandshake must succeed");
    assert_eq!(kind, MessageKind::ClaimHandshakeReply);
    (guard, conn)
}

fn sdo_read(conn: &UnixNativeConn, index: u16, subindex: u8) -> SdoReadResponse {
    let body = SdoRead { index, subindex }.encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SdoRead, body, Duration::from_secs(5))
        .expect("SdoRead call must succeed");
    assert_eq!(kind, MessageKind::SdoReadResponse);
    SdoReadResponse::decode(&resp).expect("SdoReadResponse must decode")
}

fn sdo_write(
    conn: &UnixNativeConn,
    index: u16,
    subindex: u8,
    size: u8,
    value: i64,
) -> SdoWriteResponse {
    let body = SdoWrite {
        index,
        subindex,
        size,
        value,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SdoWrite, body, Duration::from_secs(5))
        .expect("SdoWrite call must succeed");
    assert_eq!(kind, MessageKind::SdoWriteResponse);
    SdoWriteResponse::decode(&resp).expect("SdoWriteResponse must decode")
}

fn probe_count(conn: &UnixNativeConn) -> u32 {
    let r = sdo_read(conn, 0x5FFF, 0);
    assert_eq!(r.result, 0);
    u32::from_le_bytes(r.data)
}

#[test]
fn read_returns_preloaded_value() {
    let (_guard, conn) = spawn_and_claim("read");
    let r = sdo_read(&conn, 0x2002, 0);
    assert_eq!((r.result, r.size, r.data), (0, 2, [100, 0, 0, 0]));
}

#[test]
fn typed_write_skips_probe_untyped_probes() {
    let (_guard, conn) = spawn_and_claim("probe");
    let before = probe_count(&conn);
    let r = sdo_write(&conn, 0x2002, 0, 2, 250);
    assert_eq!(r.result, 0);
    let after_typed = probe_count(&conn);
    assert_eq!(after_typed - before, 1, "typed write: verify read only");
    let r = sdo_write(&conn, 0x2002, 0, SDO_SIZE_PROBE, 300);
    assert_eq!(r.result, 0);
    assert_eq!(r.readback_data, [44, 1, 0, 0]);
    let after_untyped = probe_count(&conn);
    assert_eq!(
        after_untyped - after_typed,
        2,
        "untyped write: probe + verify"
    );
}

#[test]
fn clamping_object_fails_verify_with_settled_value() {
    let (_guard, conn) = spawn_and_claim("clamp");
    let r = sdo_write(&conn, 0x2003, 0, 2, 600);
    assert_eq!(r.result, ERR_SDO_VERIFY_MISMATCH);
    assert_eq!((r.readback_size, r.readback_data), (2, [0xF4, 0x01, 0, 0]));
}

#[test]
fn read_only_and_unknown_objects_surface_abort_codes() {
    let (_guard, conn) = spawn_and_claim("abort");
    assert_eq!(
        sdo_write(&conn, 0x6041, 0, 2, 1).result,
        COE_ABORT_READ_ONLY
    );
    assert_eq!(sdo_read(&conn, 0x7777, 0).result, COE_ABORT_NOT_FOUND);
    assert_eq!(
        sdo_write(&conn, 0x2002, 0, SDO_SIZE_PROBE, 70_000).result,
        ERR_SDO_VALUE_RANGE
    );
}
