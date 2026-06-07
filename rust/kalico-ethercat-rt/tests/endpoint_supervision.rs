use std::process::{Child, Command};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use kalico_host_rt::native_call::NativeCall;
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::messages::MessageKind;

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
    format!(
        "/tmp/kalico-supervision-{}-{}.sock",
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

fn do_handshake(conn: &UnixNativeConn) {
    conn.kalico_call(
        MessageKind::ClaimHandshake,
        Vec::new(),
        Duration::from_secs(5),
    )
    .expect("ClaimHandshake must succeed");
}

fn spawn_and_connect(tag: &str) -> (ChildGuard, UnixNativeConn, String) {
    let path = socket_path(tag);
    let _ = std::fs::remove_file(&path);

    let child = Command::new(STUB_BIN)
        .args(["--socket", &path])
        .spawn()
        .expect("stub must spawn");
    let guard = ChildGuard::new(child);

    wait_for_socket(&path, Instant::now() + Duration::from_secs(5));

    let conn = UnixNativeConn::connect(&path).expect("connect must succeed");
    do_handshake(&conn);

    (guard, conn, path)
}

fn wait_for_child_exit(child: &mut Child, deadline: Instant) {
    loop {
        match child.try_wait().expect("try_wait must not fail") {
            Some(_) => return,
            None => {
                assert!(
                    Instant::now() < deadline,
                    "stub did not exit within deadline"
                );
                thread::sleep(Duration::from_millis(10));
            }
        }
    }
}

fn spawn_supervision_thread(
    conn: Arc<UnixNativeConn>,
    mut child: Child,
    on_death: impl Fn(&str) + Send + 'static,
) -> thread::JoinHandle<()> {
    thread::Builder::new()
        .name("test-ec-supervision".into())
        .spawn(move || loop {
            if conn.peer_closed() {
                on_death("conn EOF");
                return;
            }

            match child.try_wait() {
                Ok(Some(status)) => {
                    on_death(&format!("child exited: {status}"));
                    return;
                }
                Ok(None) => {}
                Err(e) => {
                    on_death(&format!("try_wait error: {e}"));
                    return;
                }
            }

            thread::sleep(Duration::from_millis(1));
        })
        .expect("spawn supervision thread")
}

fn assert_detected_within(detected: &AtomicBool, deadline: Instant) {
    loop {
        assert!(
            Instant::now() < deadline,
            "supervision did not detect endpoint death within deadline"
        );
        if detected.load(Ordering::Acquire) {
            return;
        }
        thread::sleep(Duration::from_millis(10));
    }
}

#[test]
fn peer_closed_set_when_server_side_closes() {
    let (mut guard, conn, path) = spawn_and_connect("peer-closed");

    {
        let mut child = guard.defuse();
        child.kill().expect("SIGKILL must succeed");
        wait_for_child_exit(&mut child, Instant::now() + Duration::from_secs(3));
    }

    let deadline = Instant::now() + Duration::from_secs(5);
    loop {
        if conn.peer_closed() {
            break;
        }
        assert!(
            Instant::now() < deadline,
            "peer_closed() did not become true after stub SIGKILL"
        );
        thread::sleep(Duration::from_millis(5));
    }

    let _ = std::fs::remove_file(&path);
}

#[test]
fn detects_child_exit_on_sigkill() {
    let detected = Arc::new(AtomicBool::new(false));
    let detected_reason: Arc<std::sync::Mutex<Option<String>>> =
        Arc::new(std::sync::Mutex::new(None));

    let (mut guard, conn, path) = spawn_and_connect("sigkill");
    let conn_arc = Arc::new(conn);
    let mut child = guard.defuse();

    child.kill().expect("SIGKILL must succeed");
    wait_for_child_exit(&mut child, Instant::now() + Duration::from_secs(3));

    let detected_clone = Arc::clone(&detected);
    let reason_clone = Arc::clone(&detected_reason);
    let handle = spawn_supervision_thread(Arc::clone(&conn_arc), child, move |reason| {
        detected_clone.store(true, Ordering::Release);
        *reason_clone.lock().unwrap() = Some(reason.to_owned());
    });

    assert_detected_within(&detected, Instant::now() + Duration::from_secs(5));
    handle.join().expect("supervision thread must exit cleanly");

    let reason = detected_reason.lock().unwrap().clone().unwrap_or_default();
    assert!(
        reason.contains("child exited") || reason.contains("conn EOF"),
        "unexpected detection reason: {reason}"
    );

    drop(conn_arc);
    let _ = std::fs::remove_file(&path);
}
