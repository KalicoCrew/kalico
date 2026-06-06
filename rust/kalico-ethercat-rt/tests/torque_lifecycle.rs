use std::process::{Child, Command};
use std::thread;
use std::time::{Duration, Instant};

use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Cursor, Decode, Encode};
use kalico_protocol::messages::{
    ClaimHandshakeReply, MessageKind, PushPieces, SetTorque, SetTorqueResponse,
};
use runtime::piece_ring::PieceEntry;

const STUB_BIN: &str = env!("CARGO_BIN_EXE_kalico-ethercat-rt-stub");

// ---------------------------------------------------------------------------
// RAII helpers (copied from stub_lifecycle.rs — no shared test module exists)
// ---------------------------------------------------------------------------

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
    format!("/tmp/kalico-tq-{}-{}.sock", tag, std::process::id())
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

// ---------------------------------------------------------------------------
// Torque-specific helpers
// ---------------------------------------------------------------------------

fn set_torque(conn: &UnixNativeConn, value: bool, execute_at_ns: u64) -> i32 {
    let body = SetTorque {
        value: u8::from(value),
        execute_at_ns,
    }
    .encoded_to_vec();
    let (kind, resp) = conn
        .kalico_call(MessageKind::SetTorque, body, Duration::from_secs(5))
        .expect("SetTorque call must succeed");
    assert_eq!(
        kind,
        MessageKind::SetTorqueResponse,
        "expected SetTorqueResponse, got 0x{:04x}",
        kind.as_u16()
    );
    SetTorqueResponse::decode(&resp)
        .expect("SetTorqueResponse must decode")
        .result
}

fn now_ns() -> u64 {
    kalico_ethercat_rt::clock::monotonic_ns()
}

/// Build a one-entry PushPieces body and send it; returns whatever the stub
/// replies. The assertion is process exit, not response result.
fn push_one_piece(conn: &UnixNativeConn, start_time: u64) {
    let entry = PieceEntry {
        start_time,
        coeffs: [0.0_f32; 4],
        duration: 0.001,
        _reserved: 0,
    };
    let mut pieces_bytes = Vec::with_capacity(32);
    pieces_bytes.extend_from_slice(&entry.to_le_bytes());
    let msg = PushPieces {
        axis_idx: 0,
        piece_count: 1,
        start_slot: 0,
        new_head: 1,
        pieces_bytes,
    };
    let body = msg.encoded_to_vec();
    // We ignore the response — the test assertion is process exit.
    let _ = conn.kalico_call(MessageKind::PushPieces, body, Duration::from_secs(5));
}

// ---------------------------------------------------------------------------
// Helper: spawn stub + wait for socket + do handshake
// ---------------------------------------------------------------------------

fn spawn_and_claim(tag: &str, extra_args: &[&str]) -> (ChildGuard, UnixNativeConn, String) {
    let path = socket_path(tag);
    let _ = std::fs::remove_file(&path);

    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .args(extra_args)
        .spawn()
        .expect("stub binary must spawn");
    let guard = ChildGuard::new(child);

    wait_for_socket(&path, Instant::now() + Duration::from_secs(5));

    let conn = UnixNativeConn::connect(&path).expect("UnixNativeConn::connect must succeed");
    let _reply = do_handshake(&conn);

    // The guard is returned so the caller decides when / how to defuse it.
    // We defuse inside each test after the behavioral assertions are done.
    (guard, conn, path)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Test 1: enable → disable(scheduled) → sleep past deadline → re-enable
//
// The re-enable succeeding with result 0 is the behavioral proof that the
// scheduled disable executed and parked the gate.  A gate still in Enabled
// with no pending disable would reject with -312, not return 0.
#[test]
fn enable_acks_disable_schedules_and_parks() {
    let (mut guard, conn, path) = spawn_and_claim("tq-parks", &[]);

    let result = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(result, 0, "enable must return 0, got {result}");

    let disable_at = now_ns() + 200_000_000;
    let result = set_torque(&conn, false, disable_at);
    assert_eq!(
        result, 0,
        "scheduled disable must return 0 immediately, got {result}"
    );

    // Wait well past the 200 ms deadline so the 1 ms tick loop has time to
    // execute the disable and park the gate.
    thread::sleep(Duration::from_millis(400));

    // Re-enable: result 0 proves the gate is Parked (not Enabled).
    let result = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        result, 0,
        "re-enable after scheduled disable executed must return 0 (gate Parked), got {result}"
    );

    drop(conn);
    let _ = guard.defuse().wait();
    let _ = std::fs::remove_file(&path);
}

// Test 2: double enable is rejected and the stub exits.
#[test]
fn double_enable_rejects_and_exits() {
    let (mut guard, conn, path) = spawn_and_claim("tq-dbl-en", &[]);

    let r1 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r1, 0, "first enable must return 0, got {r1}");

    let r2 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r2, -312, "double enable must return -312, got {r2}");

    // Reject causes stub to exit — defuse and wait.
    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(4));
    let _ = std::fs::remove_file(&path);
}

// Test 3: disable with execute_at in the past is rejected and the stub exits.
#[test]
fn disable_in_past_rejects_and_exits() {
    let (mut guard, conn, path) = spawn_and_claim("tq-past", &[]);

    let r1 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r1, 0, "enable must return 0, got {r1}");

    // execute_at=1 is always in the past.
    let r2 = set_torque(&conn, false, 1);
    assert_eq!(r2, -311, "disable-in-past must return -311, got {r2}");

    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(4));
    let _ = std::fs::remove_file(&path);
}

// Test 4: disable while parked (no enable first) is rejected and the stub exits.
#[test]
fn disable_while_parked_rejects_and_exits() {
    let (mut guard, conn, path) = spawn_and_claim("tq-dis-park", &[]);

    let result = set_torque(&conn, false, now_ns() + 200_000_000);
    assert_eq!(
        result, -312,
        "disable while parked must return -312, got {result}"
    );

    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(4));
    let _ = std::fs::remove_file(&path);
}

// Test 5: re-enable while a disable is pending cancels the pending disable.
//
// Proof sequence:
//   enable → 0
//   disable(now+500ms) → 0          (pending disable scheduled)
//   enable immediately → 0          (cancel pending disable; gate stays Enabled)
//   sleep 700ms (past cancelled deadline)
//   enable → -312                   (gate still Enabled, no pending disable → reject)
//   stub exits after that reject.
#[test]
fn reenable_with_pending_disable_cancels_it() {
    let (mut guard, conn, path) = spawn_and_claim("tq-cancel", &[]);

    let r1 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(r1, 0, "initial enable must return 0, got {r1}");

    let cancel_at = now_ns() + 500_000_000;
    let r2 = set_torque(&conn, false, cancel_at);
    assert_eq!(r2, 0, "scheduling disable must return 0, got {r2}");

    // Immediately re-enable — cancels the pending disable.
    let r3 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r3, 0,
        "re-enable with pending disable must return 0 (cancel), got {r3}"
    );

    // Sleep well past the cancelled deadline.
    thread::sleep(Duration::from_millis(700));

    // The gate is still Enabled and has no pending disable → duplicate enable → -312.
    // This proves the cancelled disable never fired.
    let r4 = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(
        r4, -312,
        "enable while still Enabled must return -312 (cancelled disable did not fire), got {r4}"
    );

    // Reject caused exit.
    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(4));
    let _ = std::fs::remove_file(&path);
}

// Test 6: pieces pushed while parked → fault heartbeat → stub exits.
//
// The gate is never enabled. Pushing a piece causes the next 1 ms tick to
// detect pieces-while-parked and call exit(1). We assert process exit only;
// we do not assert on the PushPieces response result (the ring can accept).
#[test]
fn pieces_while_parked_fault_exits() {
    let (mut guard, conn, path) = spawn_and_claim("tq-pcs-park", &[]);

    push_one_piece(&conn, now_ns());

    // The tick fires within 1 ms; give a generous window for the process to exit.
    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(5));
    let _ = std::fs::remove_file(&path);
}

// Test 7: --fail-enable flag makes the stub return -310 and exit.
#[test]
fn fail_enable_flag_returns_310_and_exits() {
    let (mut guard, conn, path) = spawn_and_claim("tq-fail-en", &["--fail-enable"]);

    let result = set_torque(&conn, true, now_ns() + 50_000_000);
    assert_eq!(result, -310, "--fail-enable must return -310, got {result}");

    let mut child = guard.defuse();
    wait_for_exit(&mut child, Instant::now() + Duration::from_secs(4));
    let _ = std::fs::remove_file(&path);
}
