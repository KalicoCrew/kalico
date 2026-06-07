use std::process::{Child, Command};
use std::thread;
use std::time::{Duration, Instant};

use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Cursor, Decode};
use kalico_protocol::messages::{ClaimHandshakeReply, MessageKind, SlaveState};

const STUB_BIN: &str = env!("CARGO_BIN_EXE_kalico-ethercat-rt-stub");

struct ChildGuard {
    child: Option<Child>,
}

impl ChildGuard {
    fn new(child: Child) -> Self {
        Self { child: Some(child) }
    }

    /// Consume without killing — caller takes responsibility for the child.
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
    format!("/tmp/kalico-stub-{}-{}.sock", tag, std::process::id())
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

fn wait_for_exit(child: &mut Child, deadline: Instant) -> std::process::ExitStatus {
    loop {
        match child.try_wait().expect("try_wait must not fail") {
            Some(status) => return status,
            None => {
                assert!(
                    Instant::now() < deadline,
                    "stub process did not exit within deadline — orphan process"
                );
                thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

#[test]
fn stub_claim_succeeds_and_disconnect_terminates_process() {
    let path = socket_path("claim");
    let _ = std::fs::remove_file(&path);

    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .spawn()
        .expect("stub binary must spawn");
    let mut guard = ChildGuard::new(child);

    wait_for_socket(&path, Instant::now() + Duration::from_secs(5));

    let conn = UnixNativeConn::connect(&path).expect("UnixNativeConn::connect must succeed");
    let reply = do_handshake(&conn);

    assert_eq!(
        reply.slave_statuses.len(),
        1,
        "handshake reply must contain exactly 1 slave status, got {}",
        reply.slave_statuses.len()
    );
    assert_eq!(
        reply.slave_statuses[0].state,
        SlaveState::Ok,
        "slave 0 state must be Ok, got {:?}",
        reply.slave_statuses[0].state
    );

    drop(conn);

    let mut child = guard.defuse();
    let _status = wait_for_exit(&mut child, Instant::now() + Duration::from_secs(3));

    let _ = std::fs::remove_file(&path);
}

#[test]
fn stub_fail_bringup_propagates_offline_error() {
    let path = socket_path("fail-bringup");
    let _ = std::fs::remove_file(&path);

    let child = Command::new(STUB_BIN)
        .args(["--socket", &path, "--fail-bringup", "slave=1"])
        .spawn()
        .expect("stub binary must spawn");
    let mut guard = ChildGuard::new(child);

    wait_for_socket(&path, Instant::now() + Duration::from_secs(5));

    let conn = UnixNativeConn::connect(&path).expect("UnixNativeConn::connect must succeed");
    let reply = do_handshake(&conn);

    assert_eq!(
        reply.slave_statuses.len(),
        1,
        "handshake reply must contain exactly 1 slave status, got {}",
        reply.slave_statuses.len()
    );
    assert_eq!(
        reply.slave_statuses[0].state,
        SlaveState::Offline,
        "slave status state must be Offline for --fail-bringup slave=1, got {:?}",
        reply.slave_statuses[0].state
    );
    assert_eq!(
        reply.slave_statuses[0].slave_idx, 1,
        "slave_idx must be 1, got {}",
        reply.slave_statuses[0].slave_idx
    );

    drop(conn);

    let mut child = guard.defuse();
    let status = wait_for_exit(&mut child, Instant::now() + Duration::from_secs(3));

    assert!(
        !status.success(),
        "stub must exit with non-zero status after --fail-bringup, got {status:?}"
    );

    let _ = std::fs::remove_file(&path);
}
