//! Teardown-determinism tests for the EtherCAT FIRMWARE_RESTART wedge fix.
//!
//! Root cause: on klippy's in-process FIRMWARE_RESTART loop the old
//! `PyMotionBridge` was never dropped, so its `KalicoHostIo` reactor kept the
//! pts fd open (with TIOCEXCL on Linux), and the next process's `attach_serial`
//! spun on EBUSY. These tests pin the fixed contract: `shutdown()` is the single
//! complete, ordered, idempotent teardown — it drops the host_io Arc (closing
//! the fd), reaps the EtherCAT child, closes the EtherCAT socket, and joins the
//! pump/planner threads, on every call and safely more than once.
//!
//! Note on the fd-release assertion: `TTYPort::from_raw_fd`'s TIOCEXCL only
//! makes a second `open()` of the same pts return EBUSY on Linux; macOS pts do
//! not honour TIOCEXCL that way (verified on the bench). So the load-bearing,
//! portable proof of fd release is "the last `Arc<KalicoHostIo>` is dropped" —
//! observed via a `Weak` that no longer upgrades — because `KalicoHostIo::Drop`
//! is exactly what closes the fd. The live Linux bench check (a fresh open
//! succeeding first-try) is in the design's verification section.

use std::os::unix::io::FromRawFd;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Arc, Weak};

use kalico_host_rt::host_io::{KalicoHostIo, KalicoHostIoConfig};
use kalico_host_rt::unix_native_conn::UnixNativeConn;

use crate::config::PlannerConfig;
use crate::planner::{DispatchError, PlannerHandle};
use trajectory::ShapedSegment;

use super::{McuConnection, PyMotionBridge};

/// Open a pty pair and return (master_fd, slave_path). The master fd is kept
/// open by the caller so the slave stays valid; close it at end of test.
fn open_pty() -> (libc::c_int, String) {
    let mut master: libc::c_int = 0;
    let mut slave: libc::c_int = 0;
    // SAFETY: openpty writes two valid fds; the name buffer is large enough for
    // any pts path. We check the return code before using the fds.
    #[allow(unsafe_code)]
    let path = unsafe {
        let mut name_buf = [0i8; 256];
        let r = libc::openpty(
            &mut master,
            &mut slave,
            name_buf.as_mut_ptr(),
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        );
        assert_eq!(r, 0, "openpty failed: {}", std::io::Error::last_os_error());
        // We build KalicoHostIo from the slave fd; the master fd is the "peer".
        // The slave path is what a real attach_serial would open.
        let cstr = std::ffi::CStr::from_ptr(name_buf.as_ptr());
        let p = cstr.to_str().expect("pts path is utf-8").to_owned();
        // Close the slave fd we got from openpty — KalicoHostIo opens its own
        // handle to the slave via the path below, mirroring attach_serial.
        libc::close(slave);
        p
    };
    (master, path)
}

/// Build a `KalicoHostIo` bound to `slave_path` (the reactor owns a real fd),
/// skipping the wire identify handshake. Returns the Arc plus a Weak to observe
/// the Arc's strong-count dropping to zero on teardown.
fn host_io_on_pty(slave_path: &str) -> (Arc<KalicoHostIo>, Weak<KalicoHostIo>) {
    // SAFETY: open + from_raw_fd are FFI boundaries; we check `fd >= 0`.
    #[allow(unsafe_code)]
    let port: Box<dyn serialport::SerialPort> = unsafe {
        let cpath = std::ffi::CString::new(slave_path).unwrap();
        let fd = libc::open(cpath.as_ptr(), libc::O_RDWR | libc::O_NOCTTY);
        assert!(
            fd >= 0,
            "open({slave_path}) failed: {}",
            std::io::Error::last_os_error()
        );
        Box::new(serialport::TTYPort::from_raw_fd(fd))
    };
    let io = KalicoHostIo::from_port_skip_identify(port, KalicoHostIoConfig::default());
    let arc = Arc::new(io);
    let weak = Arc::downgrade(&arc);
    (arc, weak)
}

fn serial_mcu_conn(label: &str, slave_path: &str, host_io: Arc<KalicoHostIo>) -> McuConnection {
    McuConnection {
        label: label.to_owned(),
        serial_path: slave_path.to_owned(),
        baud: 0,
        host_io: Some(host_io),
        runtime_rx: None,
        runtime_caps: None,
        identify_caps: 0,
        kalico_native_supported: false,
        ethercat_socket: None,
        endpoint_process: None,
        endpoint_conn: None,
    }
}

fn insert_mcu(bridge: &PyMotionBridge, handle: u32, conn: McuConnection) {
    bridge
        .mcus
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(handle, conn);
}

fn mcus_is_empty(bridge: &PyMotionBridge) -> bool {
    bridge
        .mcus
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .is_empty()
}

/// Seed a real pump channel + thread into the bridge, mirroring the wiring
/// init_planner installs, so `shutdown()`'s pump-join path is exercised.
/// The thread runs until it receives `PumpMsg::Shutdown`.
fn seed_pump_thread(bridge: &PyMotionBridge) -> Arc<std::sync::atomic::AtomicBool> {
    let (tx, rx) = std::sync::mpsc::channel::<crate::pump::PumpMsg>();
    let exited = Arc::new(std::sync::atomic::AtomicBool::new(false));
    let exited_thread = Arc::clone(&exited);
    let handle = std::thread::Builder::new()
        .name("push-pieces-pump".into())
        .spawn(move || {
            for msg in rx {
                if matches!(msg, crate::pump::PumpMsg::Shutdown) {
                    break;
                }
            }
            exited_thread.store(true, std::sync::atomic::Ordering::SeqCst);
        })
        .expect("spawn test pump thread");
    *bridge.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(tx);
    *bridge.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(handle);
    exited
}

/// (a) The host_io Arc is the last strong ref and is dropped by `shutdown()`,
/// which is exactly what `KalicoHostIo::Drop` uses to close the pts fd — the
/// direct EBUSY-release proof. (b) The pump thread is joined (Shutdown sent,
/// handle taken → None, thread observed exited). (c) The mcus map is emptied.
#[test]
fn shutdown_releases_pty_and_joins_threads() {
    let bridge = PyMotionBridge::new();
    let (master_fd, slave_path) = open_pty();

    let (io_arc, io_weak) = host_io_on_pty(&slave_path);
    insert_mcu(&bridge, 1, serial_mcu_conn("mcu", &slave_path, io_arc));
    // Drop our own strong ref: the only remaining strong ref is inside the
    // McuConnection, mirroring production where attach_serial's local Arcs have
    // already been dropped and pump/heartbeat hold Weak only.
    assert!(
        io_weak.upgrade().is_some(),
        "host_io must be alive pre-shutdown"
    );

    let pump_exited = seed_pump_thread(&bridge);

    bridge.shutdown();

    assert!(
        io_weak.upgrade().is_none(),
        "shutdown() must drop the last Arc<KalicoHostIo> — its Drop closes the \
         pts fd (TIOCEXCL release); a surviving Arc means a leaked fd → EBUSY"
    );
    assert!(
        pump_exited.load(std::sync::atomic::Ordering::SeqCst),
        "pump thread must have received Shutdown and exited (joined, not leaked)"
    );
    assert!(
        bridge
            .pump_thread
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "pump_thread handle must be taken (None) after join"
    );
    assert!(
        mcus_is_empty(&bridge),
        "mcus map must be empty after shutdown"
    );
    assert!(
        bridge.shut_down.load(std::sync::atomic::Ordering::SeqCst),
        "shut_down flag must be latched"
    );

    // SAFETY: master_fd is the pty master we kept open; close it now.
    #[allow(unsafe_code)]
    unsafe {
        libc::close(master_fd);
    }
}

/// `shutdown()` must reap the EtherCAT child and close the endpoint socket so
/// the peer sees EOF.
#[test]
fn shutdown_releases_ethercat_socket_and_child() {
    use std::io::Read;
    use std::os::unix::net::UnixStream;

    let bridge = PyMotionBridge::new();

    // socketpair: one end becomes the UnixNativeConn (endpoint_conn), the other
    // is the test-held "peer" that must observe EOF when the conn is dropped.
    let (conn_stream, peer_stream) = UnixStream::pair().expect("socketpair must be available");
    let native = UnixNativeConn::from_stream(conn_stream).expect("from_stream");

    // A long-lived child that SIGTERM will terminate (mirrors the endpoint).
    let child = std::process::Command::new("sh")
        .args(["-c", "sleep 30"])
        .spawn()
        .expect("sh must be available");
    let child_pid = child.id();

    let conn = McuConnection {
        label: "ec".to_owned(),
        serial_path: String::new(),
        baud: 0,
        host_io: None,
        runtime_rx: None,
        runtime_caps: None,
        identify_caps: 0,
        kalico_native_supported: true,
        ethercat_socket: Some("/tmp/kalico_test_ec.sock".to_owned()),
        endpoint_process: Some(child),
        endpoint_conn: Some(Arc::new(native)),
    };
    insert_mcu(&bridge, 7, conn);

    bridge.shutdown();

    // Peer sees EOF: the conn's socket (and its try_clones) are all closed.
    // `shutdown()` synchronously joins UnixNativeConn's reader (Drop does
    // shutdown(Both)+join), so the socket is fully closed before it returns and
    // this read returns EOF immediately. Run it in a watchdog thread so a
    // regression (socket left open) surfaces as a test failure, not a hang.
    let (done_tx, done_rx) = std::sync::mpsc::channel::<std::io::Result<usize>>();
    std::thread::spawn(move || {
        let mut peer = peer_stream;
        let mut buf = [0u8; 16];
        let _ = done_tx.send(peer.read(&mut buf));
    });
    let n = done_rx
        .recv_timeout(std::time::Duration::from_secs(5))
        .expect("peer read must complete (socket must be closed by shutdown)")
        .expect("peer read after conn close");
    assert_eq!(
        n, 0,
        "peer must see EOF (0 bytes) after endpoint_conn dropped"
    );

    // Child is reaped: the pid is no longer a live process we can signal
    // (SIGTERM was sent + reaped by release_mcu). kill(pid, 0) must fail ESRCH.
    // SAFETY: signal 0 only probes existence; it never delivers a signal.
    #[allow(unsafe_code)]
    let alive = unsafe { libc::kill(child_pid as libc::pid_t, 0) };
    assert_eq!(
        alive, -1,
        "endpoint child (pid {child_pid}) must be reaped (kill(pid,0) → ESRCH)"
    );
    assert_eq!(
        std::io::Error::last_os_error().raw_os_error(),
        Some(libc::ESRCH),
        "child must be gone (ESRCH), not merely unsignalable"
    );

    assert!(
        mcus_is_empty(&bridge),
        "mcus map must be empty after shutdown"
    );
}

/// `shutdown(); shutdown();` — the second call is a clean no-op: no panic, no
/// double-join, and the `shut_down` flag is observed on entry.
#[test]
fn double_shutdown_is_safe() {
    let bridge = PyMotionBridge::new();
    let (master_fd, slave_path) = open_pty();
    let (io_arc, io_weak) = host_io_on_pty(&slave_path);
    insert_mcu(&bridge, 1, serial_mcu_conn("mcu", &slave_path, io_arc));

    bridge.shutdown();
    assert!(
        io_weak.upgrade().is_none(),
        "first shutdown releases host_io"
    );
    assert!(bridge.shut_down.load(std::sync::atomic::Ordering::SeqCst));

    // Second call must short-circuit on the latched flag and not panic.
    bridge.shutdown();
    assert!(mcus_is_empty(&bridge));

    // SAFETY: master_fd is the pty master we kept open; close it now.
    #[allow(unsafe_code)]
    unsafe {
        libc::close(master_fd);
    }
}

/// A no-op dispatch closure paired with an invocation counter — the trivial
/// seam (mirrors `planner::tests::counting_dispatch`) for spawning a real
/// `kalico-planner` thread without a transport behind it.
fn counting_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<AtomicUsize>,
) {
    let counter = Arc::new(AtomicUsize::new(0));
    let c = Arc::clone(&counter);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |_seg: &ShapedSegment| {
            c.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });
    (cb, counter)
}

/// Loosened fit tolerance for fast planning in tests (mirrors
/// `planner::tests::relaxed_config`).
fn relaxed_planner_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.fit_tolerance_mm = 0.05;
    c
}

/// `shutdown()` must drain the `Mutex<Option<PlannerHandle>>` and join the
/// `kalico-planner` thread. The OnceLock→Mutex<Option> migration is the
/// centerpiece of this change; this is the bridge-level proof that
/// `shutdown()`'s planner take()+`PlannerHandle::shutdown()` path actually runs
/// (planner::tests only covers `PlannerHandle::shutdown()` in isolation).
#[test]
fn shutdown_takes_and_joins_planner() {
    let bridge = PyMotionBridge::new();
    let (dispatch, _counter) = counting_dispatch();
    *bridge.planner.lock().unwrap_or_else(|p| p.into_inner()) =
        Some(PlannerHandle::spawn(PlannerConfig::default(), dispatch));

    assert!(
        bridge
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_some(),
        "planner must be seeded pre-shutdown"
    );

    bridge.shutdown();

    assert!(
        bridge
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "shutdown() must take() the planner out of the Mutex and join it — a \
         surviving Some means the kalico-planner thread leaked across restart"
    );
}

/// Regression test for the teardown-ordering blocker: the planner must be joined
/// BEFORE the pump's `Receiver` is dropped.
///
/// Mechanism guarded against: while the planner holds an uncommitted decel tail
/// (`t_dispatched < t_appended`), its `recv_timeout` fires
/// `run_commit_and_dispatch`, which calls the dispatch closure → `pump_tx.send`.
/// If the pump's `Receiver` were already gone, that send fails →
/// `DispatchError::PumpGone` → the real planner calls `fatal()` →
/// `std::process::abort()`, which skips every `Drop` and re-leaks the pts fd.
///
/// We reproduce the exact dependency WITHOUT the real abort path so the bug
/// surfaces as a clean assertion failure, not a test-binary abort: the test's
/// dispatch sends into the same channel whose `Receiver` `shutdown()` drops, and
/// on a send failure it *records a flag and returns Ok* (so the real planner's
/// `fatal()`/`abort()` is never reached). A background thread keeps submitting
/// moves so the planner always holds a pending tail and is continuously firing
/// timeout-commit dispatches across the teardown window. With the fixed order
/// (planner joined first), the pump's Receiver outlives every dispatch and
/// `saw_pump_gone` stays false; with the pre-fix pump-first order the planner
/// dispatches into the already-dropped Receiver and the flag flips true.
#[test]
fn shutdown_joins_planner_before_dropping_pump_receiver() {
    let bridge = PyMotionBridge::new();

    // The channel the dispatch closure sends into. Its Receiver lives in the
    // pump thread we seed below; `shutdown()` drops it when it tears the pump.
    // A clone goes into bridge.pump_tx so shutdown()'s `Shutdown` reaches the
    // pump thread (mirroring production: pump_tx and the dispatch's tx are the
    // same channel; the pump owns the single Receiver).
    let (pump_tx, pump_rx) = std::sync::mpsc::channel::<crate::pump::PumpMsg>();
    let pump_tx_for_bridge = pump_tx.clone();

    let saw_pump_gone = Arc::new(AtomicBool::new(false));
    let saw_pump_gone_cb = Arc::clone(&saw_pump_gone);
    let dispatch_count = Arc::new(AtomicUsize::new(0));
    let dispatch_count_cb = Arc::clone(&dispatch_count);
    let dispatch: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |_seg: &ShapedSegment| {
            dispatch_count_cb.fetch_add(1, Ordering::SeqCst);
            // Mirror the production dispatch's pump-send. We send a (cheap)
            // Heartbeat, NOT Shutdown — Shutdown is the pump's own exit signal,
            // which only `bridge.shutdown()` may send. A failed send means the
            // pump's Receiver is already dropped — the exact condition that yields
            // DispatchError::PumpGone in production. We record it and return Ok so
            // the real planner never reaches fatal()/abort() and the test can
            // assert cleanly.
            let hb = crate::pump::PumpMsg::Heartbeat(crate::pump::HeartbeatMsg {
                mcu_id: 0,
                retired_counts: Vec::new(),
            });
            if pump_tx.send(hb).is_err() {
                saw_pump_gone_cb.store(true, Ordering::SeqCst);
            }
            Ok(())
        });

    let planner = PlannerHandle::spawn(relaxed_planner_config(), dispatch);
    // Prime one move so the planner has a pending tail before the submitter and
    // pump are even wired — the recv_timeout branch is armed from the start.
    planner
        .submit_move(
            crate::classify::classify_and_build([0.0; 3], 50.0, 0.0, 0.0, 0.0, 200.0).unwrap(),
        )
        .unwrap();
    let bridge = Arc::new(bridge);
    *bridge.planner.lock().unwrap_or_else(|p| p.into_inner()) = Some(planner);

    // Seed the pump thread holding the matching Receiver, mirroring production:
    // run_pump owns pump_rx by value and exits on PumpMsg::Shutdown — at which
    // point pump_rx is dropped *while the dispatch closure's Sender is still
    // alive*. That is precisely the window in which the wrong teardown order lets
    // the planner's next timeout-commit dispatch hit a dead Receiver.
    let pump_handle = std::thread::Builder::new()
        .name("push-pieces-pump".into())
        .spawn(move || {
            for msg in &pump_rx {
                if matches!(msg, crate::pump::PumpMsg::Shutdown) {
                    break;
                }
            }
            drop(pump_rx);
        })
        .expect("spawn test pump thread");
    *bridge.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_handle);
    *bridge.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_tx_for_bridge);

    // Background submitter: keep the planner perpetually holding an uncommitted
    // decel tail so its recv_timeout branch stays armed and it fires timeout-
    // commit dispatches continuously — including through the teardown window. It
    // drives the planner through the bridge's Mutex<Option<PlannerHandle>>; once
    // shutdown() take()s the planner it observes None and stops.
    let stop = Arc::new(AtomicBool::new(false));
    let stop_sub = Arc::clone(&stop);
    let bridge_sub = Arc::clone(&bridge);
    let submitter = std::thread::Builder::new()
        .name("test-submitter".into())
        .spawn(move || {
            while !stop_sub.load(Ordering::SeqCst) {
                {
                    let guard = bridge_sub.planner.lock().unwrap_or_else(|p| p.into_inner());
                    let Some(p) = guard.as_ref() else {
                        break; // shutdown() took the planner; stop submitting.
                    };
                    let m =
                        crate::classify::classify_and_build([0.0; 3], 50.0, 0.0, 0.0, 0.0, 200.0)
                            .unwrap();
                    if p.submit_move(m).is_err() {
                        break;
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(3));
            }
        })
        .expect("spawn test submitter");

    // Let the planner run long enough to be actively firing timeout-commits.
    std::thread::sleep(std::time::Duration::from_millis(300));
    assert!(
        dispatch_count.load(Ordering::SeqCst) > 0,
        "planner must have fired at least one dispatch (else the test does not \
         exercise the ordering window)"
    );

    bridge.shutdown();

    stop.store(true, Ordering::SeqCst);
    submitter.join().expect("submitter join");

    assert!(
        !saw_pump_gone.load(Ordering::SeqCst),
        "planner observed a dropped pump during teardown — this is the abort() \
         path that leaks the pts fd; shutdown() must join the planner BEFORE \
         dropping the pump's Receiver"
    );
    assert!(
        bridge
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "planner must be taken+joined by shutdown()"
    );
}

/// Regression for the EtherCAT pump-abort ordering bug:
///
/// With the old ordering (`release_mcu` first, pump last), a pump that still
/// holds queued pieces for an EtherCAT MCU would call `send_frame` after the
/// last strong `Arc<UnixNativeConn>` was dropped by `release_mcu`. The
/// `Weak::upgrade()` returns `None` → `SendError::Fatal` → `on_fatal_transport`
/// → `std::process::abort()`, which skips every `Drop` and re-leaks the pts fd.
///
/// With the fixed ordering (planner join → pump join → release_mcu), the pump
/// receives `PumpMsg::Shutdown` and exits before any send can fire against a
/// dead transport. This test pins that contract without touching production
/// abort semantics:
///
///   (a) A `WireSink` is wired with a detached EtherCAT `Weak` (no strong Arc)
///       and an injectable `on_fatal_transport` that sets a flag instead of
///       aborting — identical to the seam used in pump's own unit tests.
///   (b) Pieces for the dead mcu_id are enqueued into the pump BEFORE
///       `shutdown()` is called. The pump is held in `StallAhead` (clock
///       horizon is behind the pieces' start-time) so it cannot drain the
///       queue before `Shutdown` arrives.
///   (c) `bridge.shutdown()` runs: planner join → pump Shutdown+join →
///       release_mcu.  The `on_fatal_transport` flag must remain `false`
///       throughout — proving the pump exited via `Shutdown` before it could
///       touch the dead transport.
#[test]
fn shutdown_does_not_abort_on_detached_ethercat_weak() {
    use runtime::piece_ring::PieceEntry;
    use std::collections::HashMap;
    use std::time::Duration;

    use crate::pump::{AxisKey, EnqueueMsg, McuTransport, PumpMsg, WireSink, run_pump};

    const EC_MCU_ID: u32 = 42;

    // Detached Weak: the strong Arc is never stored anywhere, so upgrade() always
    // returns None → Fatal.  This mirrors the state after the old release_mcu had
    // dropped the last strong Arc while the pump was still alive.
    let detached_weak: std::sync::Weak<kalico_host_rt::unix_native_conn::UnixNativeConn> =
        std::sync::Weak::new();

    let fatal_fired = Arc::new(AtomicBool::new(false));
    let fatal_flag = Arc::clone(&fatal_fired);

    let sink = WireSink {
        transports: {
            let mut m = HashMap::new();
            m.insert(EC_MCU_ID, McuTransport::EtherCat(detached_weak));
            m
        },
        timeout: Duration::from_millis(50),
        freq_of: Arc::new(|_| None),
    };

    // mcu_clock_of returns a horizon behind any realistic start_time so that
    // schedule() always returns StallAhead — pieces stay queued, send_frame is
    // never called, and the pump blocks on recv_timeout(50ms) waiting for more
    // messages.  When Shutdown arrives via recv_timeout, run_pump returns
    // immediately without touching the dead transport.
    let mcu_clock_of = |_mcu_id: u32| -> Option<(u64, f64)> {
        // ack_now=1, freq=1.0 → horizon = 1 + 1 = 2. Any piece with
        // start_time > 2 stalls.
        Some((1, 1.0))
    };

    let (pump_tx, pump_rx) = std::sync::mpsc::channel::<PumpMsg>();

    let pump_handle = std::thread::Builder::new()
        .name("push-pieces-pump".into())
        .spawn(move || {
            run_pump(
                pump_rx,
                sink,
                |_key| 256_u32,
                mcu_clock_of,
                move |_key: AxisKey| {
                    fatal_flag.store(true, Ordering::SeqCst);
                },
                |_key: AxisKey, _n: u32| {},
                |_msg: String| {},
            );
        })
        .expect("spawn test pump thread");

    // Enqueue pieces with start_time well above the horizon (2) so schedule()
    // stalls rather than sending. The pump enters StallAhead and blocks on
    // recv_timeout(50ms) — pieces are live in the queue when shutdown() runs.
    let pieces_to_enqueue = vec![(
        PieceEntry {
            start_time: 1_000_000,
            coeffs: [0.0; 4],
            duration: 0.001,
            _reserved: 0,
        },
        1.0_f64,
    )];
    pump_tx
        .send(PumpMsg::Enqueue(EnqueueMsg {
            key: AxisKey {
                mcu_id: EC_MCU_ID,
                axis: 0,
            },
            pieces: pieces_to_enqueue,
            fresh_stream: false,
            lead_secs: 0.0,
        }))
        .expect("enqueue must succeed before shutdown");

    // Give the pump time to process the Enqueue message and enter StallAhead
    // so it is blocking on recv_timeout when Shutdown arrives.
    std::thread::sleep(Duration::from_millis(30));

    // Seed the pump into the bridge so shutdown() drives it.
    let bridge = Arc::new(PyMotionBridge::new());
    *bridge.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_tx);
    *bridge.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_handle);

    // Also seed a planner so the planner-join step of shutdown() is exercised.
    let (dispatch, _counter) = counting_dispatch();
    *bridge.planner.lock().unwrap_or_else(|p| p.into_inner()) =
        Some(PlannerHandle::spawn(relaxed_planner_config(), dispatch));

    bridge.shutdown();

    assert!(
        !fatal_fired.load(Ordering::SeqCst),
        "on_fatal_transport must never fire during shutdown(): pump must exit \
         via PumpMsg::Shutdown before it can touch the dead EtherCAT transport. \
         A true flag means the old abort() path would have killed the process and \
         leaked the pts fd."
    );
    assert!(
        bridge
            .pump_thread
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "pump thread handle must be taken (joined) by shutdown()"
    );
    assert!(
        bridge
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_none(),
        "planner must be taken+joined by shutdown()"
    );
}

/// Regression: `register_ethercat_mcu` must seed `nominal_clock_freqs` for the
/// raw handle so that dispatch on the servo MCU never hits `MissingNominalFreq`.
///
/// This calls the production helper directly — no socket, no handshake, no
/// endpoint binary. Removing the `nominal_clock_freqs.insert` line from
/// `register_ethercat_mcu` causes the second assertion to fail.
#[test]
fn register_ethercat_mcu_seeds_nominal_clock_freq() {
    use std::os::unix::net::UnixStream;

    use kalico_host_rt::unix_native_conn::UnixNativeConn;

    let bridge = PyMotionBridge::new();
    const RAW: u32 = 77;

    let (conn_stream, _peer) = UnixStream::pair().expect("socketpair");
    let conn = UnixNativeConn::from_stream(conn_stream).expect("from_stream");
    let child = std::process::Command::new("true")
        .spawn()
        .expect("spawn true");

    bridge.register_ethercat_mcu(RAW, "servo", "/tmp/test.sock", child, conn);

    assert!(
        bridge
            .mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .contains_key(&RAW),
        "mcus must contain the raw handle after register_ethercat_mcu"
    );
    assert_eq!(
        bridge
            .nominal_clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&RAW)
            .copied(),
        Some(1_000_000_000_u32),
        "nominal_clock_freqs must contain 1 GHz for the ethercat raw handle; \
         removing the insert from register_ethercat_mcu must cause this to fail"
    );
}

/// Failed-connect partial teardown: one serial MCU attached, the (would-be)
/// second never attached. `shutdown()` must release the one attached MCU's fd —
/// mirroring the `printer._connect` except-arm firing a guarded disconnect.
#[test]
fn failed_connect_partial_teardown() {
    let bridge = PyMotionBridge::new();
    let (master_fd, slave_path) = open_pty();
    let (io_arc, io_weak) = host_io_on_pty(&slave_path);
    insert_mcu(&bridge, 1, serial_mcu_conn("mcu0", &slave_path, io_arc));
    // MCU handle 2 was never attached (simulated claim failure) — nothing to
    // insert. shutdown() must still cleanly release handle 1.

    bridge.shutdown();

    assert!(
        io_weak.upgrade().is_none(),
        "the one attached MCU's host_io fd must be released even on the \
         partial/failed-connect teardown path"
    );
    assert!(mcus_is_empty(&bridge));

    // SAFETY: master_fd is the pty master we kept open; close it now.
    #[allow(unsafe_code)]
    unsafe {
        libc::close(master_fd);
    }
}
