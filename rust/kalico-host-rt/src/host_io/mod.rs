//! `host_io` — production host I/O implementing [`Transport`].
//!
//! Phase C: `KalicoHostIo` spawns a background reactor thread on `open`.
//! `Transport::call` / `call_typed` submit commands via an mpsc channel
//! and block on a rendezvous channel for the response. The Phase-B
//! mutex shim has been removed.

pub mod call_handle;
pub mod events;
pub mod identify;
pub(crate) mod interceptor;
pub mod kalico_native;
pub mod parser;
pub mod reactor;
pub mod rtt;
pub mod runtime_events;
pub mod serial_frame_io;
pub mod tcp_serial_port;
pub use identify::IdentifySeqState;
pub use interceptor::InterceptorId;
pub use serial_frame_io::SerialFrameIo;
pub use tcp_serial_port::TcpSerialPort;
#[cfg(any(test, feature = "test-harness"))]
pub mod test_harness;
pub mod window;
pub mod wire;

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;
use std::time::Duration;

use arc_swap::ArcSwap;

use crate::host_io::events::HostEvent;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::runtime_events::{
    FaultEvent, McuLogEvent, RuntimeEvent, StatusEvent, TraceEvent,
};
use crate::passthrough_queue::{CommandQueueId, McuHandle, PassthroughEntry, PassthroughRouter};
use crate::transport::{MessageParams, SubscribeError, Transport, TransportError};
use std::sync::mpsc::SyncSender;

pub(super) fn sp_err(e: &serialport::Error) -> TransportError {
    TransportError::Io(std::io::Error::other(format!("serialport: {e}")))
}

const DEFAULT_BAUD: u32 = 250_000;

#[derive(Debug, Clone)]
pub struct KalicoHostIoConfig {
    /// Capacity of the bounded ring delivering `TraceEvent` to the trace
    /// subscriber. Overruns set the sticky `OVERFLOW` flag on the next event.
    pub trace_capacity: usize,
    /// Capacity of the bounded host-event inbox shared by the reactor (TraceRing
    /// overflow/disconnect/reattach diagnostics). Drained once per loop iter.
    pub host_event_capacity: usize,
    /// Capacity of the bounded `RuntimeEvent` catch-all subscriber channel.
    pub runtime_event_capacity: usize,
    pub default_call_timeout: Duration,
    pub identify_timeout: Duration,
    pub default_dispatcher_timeout: Duration,
}

impl Default for KalicoHostIoConfig {
    fn default() -> Self {
        Self {
            trace_capacity: 256,
            host_event_capacity: 64,
            runtime_event_capacity: 64,
            default_call_timeout: Duration::from_millis(100),
            identify_timeout: Duration::from_millis(15_000),
            default_dispatcher_timeout: Duration::from_secs(30),
        }
    }
}

/// Newtype wrapper for a heartbeat callback so `ReactorCommand` can remain
/// `#[derive(Debug)]`. Fired on every `StatusHeartbeat` with the per-axis
/// consumed-piece counts.
pub struct HeartbeatCallback(pub Arc<dyn Fn(&[u32]) + Send + Sync>);

impl std::fmt::Debug for HeartbeatCallback {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("HeartbeatCallback(<fn>)")
    }
}

/// Boxed hook fired on every decoded `McuLog (0x0084)` event. Runs on the
/// reactor thread — must be non-blocking. Mirrors `HeartbeatCallback`.
pub struct McuLogHook(pub Box<dyn Fn(McuLogEvent) + Send + Sync>);

impl std::fmt::Debug for McuLogHook {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("McuLogHook(<fn>)")
    }
}

#[derive(Debug)]
pub enum ReactorCommand {
    Submit {
        call_id: u64,
        cmd: String,
        expected_response_name: String,
        completion: SyncSender<Result<MessageParams, TransportError>>,
        deadline: std::time::Instant,
    },
    SubmitTyped {
        call_id: u64,
        payload: Vec<u8>,
        expected_response_name: String,
        completion: SyncSender<Result<MessageParams, TransportError>>,
        deadline: std::time::Instant,
    },
    Abandon(u64),
    AttachHeartbeatCallback(HeartbeatCallback),
    /// Install the MCU-log hook (`KalicoHostIo::set_mcu_log_hook`). Replaces
    /// any previously installed hook.
    SetMcuLogHook(McuLogHook),
    SubscribeFault {
        sender: SyncSender<FaultEvent>,
        reply: SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeTrace {
        sender: SyncSender<TraceEvent>,
        reply: SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeRuntimeEvents {
        sender: SyncSender<RuntimeEvent>,
        reply: SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeHostEvents {
        sender: SyncSender<HostEvent>,
        reply: SyncSender<Result<(), SubscribeError>>,
    },
    /// Install the passthrough router (bridge startup). Replaces any
    /// previously installed router.
    InstallPassthroughRouter(PassthroughRouter),
    /// Push a raw passthrough entry into the router for a specific MCU.
    PassthroughSend {
        mcu: McuHandle,
        queue_id: CommandQueueId,
        entry: PassthroughEntry,
    },
    /// Send a command with no expected response (fire-and-forget).
    /// The frame is still tracked in the unacked window for wire-level
    /// retransmit on NAK, but no application-level response is awaited.
    FireAndForget {
        cmd: String,
    },
    /// Send a pre-encoded payload with no expected response (fire-and-forget).
    /// Sibling of `FireAndForget` for the typed-args path; the payload has
    /// already been encoded via `parser.encode_typed`. Routed through the
    /// reactor's `dispatch_fire_and_forget`, which respects the
    /// `pending_fire_and_forget` backpressure queue (spec §6.0).
    FireAndForgetTyped {
        payload: Vec<u8>,
    },
    /// Phase C-B: kalico-native bootstrap-ABI Identify handshake.
    /// The reactor allocates a correlation_id, builds the bootstrap frame,
    /// writes it to the wire, and parks `completion` until the
    /// `IdentifyResponse` arrives (or the deadline fires).
    KalicoIdentify {
        completion:
            SyncSender<Result<crate::host_io::kalico_native::IdentifyOutcome, TransportError>>,
        deadline: std::time::Instant,
    },
    /// Phase C-B: kalico-native call on `channel`. The reactor allocates a
    /// correlation_id, builds the frame from `kind` + `body` on the specified
    /// channel, writes it, and parks `completion` keyed by correlation_id.
    /// Spec §7.2. Outbound channel is explicit; the response always arrives on
    /// the control channel matched by correlation_id.
    KalicoCall {
        /// Layer-1 channel byte for the outbound frame. Control-channel calls
        /// use `CHANNEL_CONTROL` (0x00); PushPieces uses
        /// [`kalico_protocol::KALICO_CHANNEL_PIECES`] (0x02).
        channel: u8,
        kind: kalico_protocol::MessageKind,
        body: Vec<u8>,
        completion:
            SyncSender<Result<crate::host_io::kalico_native::KalicoCallOutcome, TransportError>>,
        deadline: std::time::Instant,
    },
    Shutdown,
    /// No-op sent by `KalicoHostIo::is_alive` to probe whether the reactor
    /// thread is still running. The reactor discards it immediately. If
    /// `send()` returns `Err`, the receiver (reactor) has dropped and the
    /// connection is dead.
    Noop,
    /// Register a frame interceptor. The reactor calls `callback` on the
    /// reactor thread for every unsolicited frame matching `(msg_name, oid)`
    /// before forwarding to the `RuntimeEvent` dispatcher. Replies with the
    /// allocated `InterceptorId` so the caller can later unregister.
    RegisterInterceptor {
        msg_name: String,
        oid: Option<u32>,
        callback: crate::host_io::interceptor::InterceptorCallback,
        reply: SyncSender<crate::host_io::InterceptorId>,
    },
    /// Unregister a previously registered frame interceptor.
    UnregisterInterceptor {
        id: crate::host_io::InterceptorId,
    },
    /// 2026-05-18: marks the reactor's pending close as graceful so a
    /// subsequent transport drop does NOT trigger the EXIT_ON_FAULT abort
    /// in the spawn-time guard. Used by klippy's bridge-MCU
    /// `_restart_via_command` path right before sending the firmware `reset`
    /// command — the reset triggers `NVIC_SystemReset` on the MCU which
    /// drops USB-CDC, and without this signal the reactor would interpret
    /// the kernel BrokenPipe as a wedge and abort the whole klippy
    /// process. The reactor handles this by setting
    /// `closed_via_shutdown = true` without transitioning to `Closed` —
    /// it keeps running until either an actual `Shutdown` command arrives
    /// or the transport drops.
    MarkExpectedDisconnect,
}

pub struct KalicoHostIo {
    submission_tx: Sender<ReactorCommand>,
    next_call_id: AtomicU64,
    reactor_handle: Option<JoinHandle<()>>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    parser: Arc<MsgProtoParser>,
    config: KalicoHostIoConfig,
    clock: Arc<dyn crate::clock::Clock>,
    /// Raw identify bytes (zlib-compressed blob as received from firmware).
    /// Suitable for passing directly to klippy's `process_identify`.
    raw_identify_bytes: Vec<u8>,
    /// EXIT_ON_FAULT gate. `true` (default) = non-graceful transport drop
    /// aborts the process (motion MCUs). `false` = reactor exits cleanly,
    /// `is_alive()` goes false, klippy reconnect machinery takes over.
    /// Settable after `open` via `set_critical` because criticality is only
    /// known after the kalico identify handshake.
    is_critical: Arc<AtomicBool>,
}

impl std::fmt::Debug for KalicoHostIo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("KalicoHostIo")
            .field("next_call_id", &self.next_call_id.load(Ordering::Relaxed))
            .finish_non_exhaustive()
    }
}

impl Drop for KalicoHostIo {
    fn drop(&mut self) {
        let _ = self.submission_tx.send(ReactorCommand::Shutdown);
        if let Some(h) = self.reactor_handle.take() {
            let _ = h.join();
        }
    }
}

impl KalicoHostIo {
    pub fn open(path: &str, baud: u32) -> Result<Self, TransportError> {
        Self::open_with_config(path, baud, KalicoHostIoConfig::default())
    }

    pub fn open_default(path: &str) -> Result<Self, TransportError> {
        Self::open(path, DEFAULT_BAUD)
    }

    /// Open a Linux PTY or pipe path using `O_RDWR | O_NOCTTY`, bypassing the
    /// `serialport` baud-rate configuration that can interfere with pseudo-
    /// terminals. Use this for paths like `/tmp/klipper_sim_socket` (a symlink
    /// to `/dev/pts/N`) or `/tmp/klipper_host_*` that klipper's Linux MCU
    /// creates.
    #[cfg(target_family = "unix")]
    pub fn open_pipe(path: &str) -> Result<Self, TransportError> {
        Self::open_pipe_with_config(path, KalicoHostIoConfig::default())
    }

    #[cfg(target_family = "unix")]
    pub fn open_pipe_with_config(
        path: &str,
        config: KalicoHostIoConfig,
    ) -> Result<Self, TransportError> {
        use std::os::unix::io::FromRawFd;

        // SAFETY: `libc::open` and `TTYPort::from_raw_fd` are both unsafe FFI
        // boundaries. We check the return value of `open` before using the fd.
        #[allow(unsafe_code)]
        let port_box: Box<dyn serialport::SerialPort> = {
            let cpath = std::ffi::CString::new(path)
                .map_err(|e| TransportError::Io(std::io::Error::other(e)))?;
            let fd = unsafe { libc::open(cpath.as_ptr(), libc::O_RDWR | libc::O_NOCTTY) };
            if fd < 0 {
                return Err(TransportError::Io(std::io::Error::last_os_error()));
            }
            // Apply cfmakeraw — required for real CDC ACM TTYs (/dev/ttyACM*).
            // libc::open leaves the TTY in cooked mode (ECHO=on, ICANON=on,
            // ICRNL=on). On firmware that emits continuous unsolicited frames
            // (kalico_status @ 10 Hz on H7) the kernel echoes every device→host
            // byte back to the device's bulk-OUT endpoint, drowning identify
            // and corrupting the on-firmware demux state. PTYs are usually OK
            // either way (no flood), but real CDC ACM is not — so apply raw
            // mode unconditionally on this path. tcgetattr returns ENOTTY for
            // non-TTY fds (regular files / pipes), in which case we silently
            // skip raw setup so non-TTY callers (e.g. fixtures using fifos)
            // still work.
            #[allow(unsafe_code)]
            unsafe {
                let mut tio: libc::termios = std::mem::zeroed();
                if libc::tcgetattr(fd, &mut tio) == 0 {
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "termios_setup",
                        phase = "pre-cfmakeraw",
                        path,
                        iflag = tio.c_iflag,
                        oflag = tio.c_oflag,
                        cflag = tio.c_cflag,
                        lflag = tio.c_lflag,
                        "termios pre-cfmakeraw"
                    );
                    libc::cfmakeraw(&mut tio);
                    if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
                        let err = std::io::Error::last_os_error();
                        libc::close(fd);
                        return Err(TransportError::Io(std::io::Error::other(format!(
                            "tcsetattr({path}): {err}"
                        ))));
                    }
                    // Read back to verify cfmakeraw stuck.
                    let mut tio2: libc::termios = std::mem::zeroed();
                    if libc::tcgetattr(fd, &mut tio2) == 0 {
                        tracing::debug!(
                            subsystem = "mcu-comms",
                            event = "termios_setup",
                            phase = "post-cfmakeraw",
                            path,
                            iflag = tio2.c_iflag,
                            oflag = tio2.c_oflag,
                            cflag = tio2.c_cflag,
                            lflag = tio2.c_lflag,
                            vmin = tio2.c_cc[libc::VMIN],
                            vtime = tio2.c_cc[libc::VTIME],
                            "termios post-cfmakeraw"
                        );
                    }
                }
            }
            let port = unsafe { Box::new(serialport::TTYPort::from_raw_fd(fd)) };
            // Verify serialport::TTYPort didn't mutate termios behind our back.
            #[allow(unsafe_code)]
            unsafe {
                let mut tio3: libc::termios = std::mem::zeroed();
                if libc::tcgetattr(fd, &mut tio3) == 0 {
                    tracing::debug!(
                        subsystem = "mcu-comms",
                        event = "termios_setup",
                        phase = "post-serialport",
                        path,
                        iflag = tio3.c_iflag,
                        oflag = tio3.c_oflag,
                        cflag = tio3.c_cflag,
                        lflag = tio3.c_lflag,
                        vmin = tio3.c_cc[libc::VMIN],
                        vtime = tio3.c_cc[libc::VTIME],
                        "termios post-serialport"
                    );
                }
            }
            port
        };
        Self::open_with_port(port_box, config)
    }

    pub fn open_with_config(
        path: &str,
        baud: u32,
        config: KalicoHostIoConfig,
    ) -> Result<Self, TransportError> {
        let port_box: Box<dyn serialport::SerialPort> = serialport::new(path, baud)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| {
                TransportError::Io(std::io::Error::other(format!(
                    "serialport::open({path}@{baud}): {e}"
                )))
            })?;
        Self::open_with_port(port_box, config)
    }

    /// Open a TCP connection wrapped in a `SerialPort` adapter. Used by sim
    /// integration tests where Renode exposes the firmware's USART2 over a
    /// TCP socket (see `tools/sim/h723_sim.resc`'s `CreateServerSocketTerminal`).
    /// The adapter shims `read`/`write`/`set_timeout` onto a real `TcpStream`;
    /// other `SerialPort` methods (RTS, parity, baud) return no-op / NotSupported
    /// because Renode's UART model ignores them.
    pub fn open_tcp(addr: &str, config: KalicoHostIoConfig) -> Result<Self, TransportError> {
        let port_box: Box<dyn serialport::SerialPort> =
            Box::new(tcp_serial_port::TcpSerialPort::connect(addr)?);
        Self::open_with_port(port_box, config)
    }

    /// Construct a `KalicoHostIo` from a caller-supplied `SerialPort`. Used
    /// by integration tests (e.g. the Renode TCP socket adapter) and by
    /// `open_with_config` / `open_pipe_with_config` / `open_tcp`.
    pub fn open_with_port(
        mut port_box: Box<dyn serialport::SerialPort>,
        config: KalicoHostIoConfig,
    ) -> Result<Self, TransportError> {
        // Ensure read timeout is set (pipe_open path skips .timeout() builder).
        let _ = port_box.set_timeout(Duration::from_millis(100));
        let mut io = crate::host_io::serial_frame_io::SerialFrameIo::new(port_box);

        let (parser_owned, raw_identify_bytes, identify_seq) =
            identify::identify_handshake(&mut io, config.identify_timeout)?;

        let parser = Arc::new(parser_owned);
        let (submission_tx, submission_rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));

        let clock: Arc<dyn crate::clock::Clock> = Arc::new(crate::clock::RealClock);
        let reactor_parser = Arc::clone(&parser);
        let reactor_status = Arc::clone(&status_snapshot);
        let reactor_config = config.clone();
        let reactor_clock = Arc::clone(&clock);
        // Default critical until the bridge downgrades after identify.
        let is_critical = Arc::new(AtomicBool::new(true));
        let reactor_is_critical = Arc::clone(&is_critical);
        let reactor_handle = std::thread::spawn(move || {
            tracing::info!(
                subsystem = "mcu-comms",
                event = "reactor_spawn",
                thread_id = ?std::thread::current().id(),
                "port-bound reactor starting"
            );
            let mut reactor = crate::host_io::reactor::Reactor::new_with_clock(
                io,
                reactor_parser,
                submission_rx,
                reactor_status,
                identify_seq,
                reactor_config,
                reactor_clock,
            );
            reactor.run();
            // 2026-05-17 wedge-detection: if the reactor exited because the
            // MCU's transport died (USB disconnect, kernel ENODEV, etc.)
            // instead of because `KalicoHostIo::drop` sent a graceful
            // Shutdown, abort the process. Without this, klippy keeps
            // "running" with a dead bridge FD — the operator sees
            // /proc/<pid>/fd missing the ttyACMx entry and klippy.log
            // silent. Aborting forces systemd to restart klipper, which is
            // the recovery action a human would take anyway.
            if !reactor.exited_gracefully() {
                let critical = reactor_is_critical.load(Ordering::Acquire);
                if !critical {
                    tracing::warn!(
                        subsystem = "mcu-comms",
                        event = "reactor_exit_non_critical",
                        thread_id = ?std::thread::current().id(),
                        "transport closed via IO error on NON-CRITICAL MCU; reactor exiting without abort — klippy reconnect path will handle recovery"
                    );
                    let _ = std::io::Write::flush(&mut std::io::stderr());
                    return;
                }
                tracing::error!(
                    subsystem = "mcu-comms",
                    event = "reactor_exit_on_fault",
                    thread_id = ?std::thread::current().id(),
                    "EXIT_ON_FAULT — transport closed via IO error on CRITICAL MCU; aborting klippy so systemd restarts it"
                );
                // Flush stderr so the message reaches journalctl before we
                // tear the process down.
                let _ = std::io::Write::flush(&mut std::io::stderr());
                // Test harnesses (e.g. tools/sim/run_sim_motion_jogs.sh)
                // need to observe the wedge and continue with assertions,
                // not get SIGABRT'd. Setting KALICO_NO_EXIT_ON_FAULT=1
                // suppresses the abort while still emitting the warning.
                // Production never sets this env var.
                if std::env::var_os("KALICO_NO_EXIT_ON_FAULT").is_none() {
                    std::process::abort();
                }
            }
        });

        Ok(Self {
            submission_tx,
            next_call_id: AtomicU64::new(1),
            reactor_handle: Some(reactor_handle),
            status_snapshot,
            parser,
            config,
            clock,
            raw_identify_bytes,
            is_critical,
        })
    }
}

impl Transport for KalicoHostIo {
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let call_id = self.next_call_id.fetch_add(1, Ordering::Relaxed);
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;

        self.submission_tx
            .send(ReactorCommand::Submit {
                call_id,
                cmd: cmd.to_string(),
                expected_response_name: expected_response_name.to_string(),
                completion: tx,
                deadline,
            })
            .map_err(|_| TransportError::Closed)?;

        let handle = crate::host_io::call_handle::CallHandle {
            call_id,
            submission_tx: self.submission_tx.clone(),
        };

        // Spec §5.5: defuse only on a real completion (Ok or reactor-side Err),
        // i.e. when the reactor already cleaned up the AwaitEntry. On caller-side
        // Timeout / Disconnected the reactor still owns the entry — let Drop fire
        // ReactorCommand::Abandon so Layer-1 GC removes it promptly instead of
        // waiting for the Layer-2 dispatcher deadline.
        match rx.recv_timeout(timeout) {
            Ok(r) => {
                handle.defuse();
                r
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }

    fn call_typed(
        &self,
        name: &str,
        args: &[(&str, crate::host_io::parser::FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let payload = self
            .parser
            .encode_typed(name, args)
            .map_err(|e| TransportError::Parse(format!("{e:?}")))?;

        let call_id = self.next_call_id.fetch_add(1, Ordering::Relaxed);
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;

        self.submission_tx
            .send(ReactorCommand::SubmitTyped {
                call_id,
                payload,
                expected_response_name: expected_response_name.to_string(),
                completion: tx,
                deadline,
            })
            .map_err(|_| TransportError::Closed)?;

        let handle = crate::host_io::call_handle::CallHandle {
            call_id,
            submission_tx: self.submission_tx.clone(),
        };

        // See `call` above for the defuse semantics rationale.
        match rx.recv_timeout(timeout) {
            Ok(r) => {
                handle.defuse();
                r
            }
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }

    fn send_typed(
        &self,
        name: &str,
        args: &[(&str, crate::host_io::parser::FieldValue<'_>)],
    ) -> Result<(), TransportError> {
        // Forward to the inherent method (same encoding + reactor dispatch).
        KalicoHostIo::send_typed(self, name, args)
    }
}

impl KalicoHostIo {
    /// Check whether the reactor thread is still running.
    ///
    /// Probes by sending a [`ReactorCommand::Noop`] on the submission channel.
    /// Returns `true` if the send succeeds (receiver alive), `false` if the
    /// reactor has exited and dropped its receiver end.
    ///
    /// Non-blocking: `std::sync::mpsc::Sender::send` returns `Err` immediately
    /// when the receiver is disconnected.
    pub fn is_alive(&self) -> bool {
        self.submission_tx.send(ReactorCommand::Noop).is_ok()
    }

    /// Gate the EXIT_ON_FAULT abort for this MCU. `true` (default) aborts on
    /// non-graceful transport drop (motion MCUs). `false` exits cleanly so
    /// klippy's non-critical-disconnect machinery handles reconnect.
    pub fn set_critical(&self, critical: bool) {
        self.is_critical.store(critical, Ordering::Release);
    }

    /// Read the current criticality flag (see [`set_critical`]).
    pub fn is_critical(&self) -> bool {
        self.is_critical.load(Ordering::Acquire)
    }

    /// Register a callback fired on every `StatusHeartbeat` with the per-axis
    /// consumed-piece counts. Runs on the reactor/event thread — must be
    /// non-blocking (it only sends on a channel; see motion-bridge pump).
    pub fn attach_heartbeat_callback(&self, cb: Arc<dyn Fn(&[u32]) + Send + Sync>) {
        let _ = self
            .submission_tx
            .send(ReactorCommand::AttachHeartbeatCallback(HeartbeatCallback(
                cb,
            )));
    }

    /// Attach a hook fired on every decoded `McuLog (0x0084)` event. The hook
    /// receives an owned [`McuLogEvent`] and runs on the reactor thread — must
    /// be non-blocking. Replaces any previously installed hook.
    pub fn set_mcu_log_hook(&self, hook: Box<dyn Fn(McuLogEvent) + Send + Sync>) {
        let _ = self
            .submission_tx
            .send(ReactorCommand::SetMcuLogHook(McuLogHook(hook)));
    }

    pub fn subscribe_fault(&self) -> Result<std::sync::mpsc::Receiver<FaultEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(1);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx
            .send(ReactorCommand::SubscribeFault {
                sender,
                reply: reply_tx,
            })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok(receiver)
    }

    pub fn take_trace_subscription(
        &self,
    ) -> Result<std::sync::mpsc::Receiver<TraceEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(self.config.trace_capacity);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx
            .send(ReactorCommand::SubscribeTrace {
                sender,
                reply: reply_tx,
            })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok(receiver)
    }

    pub fn take_runtime_event_subscription(
        &self,
    ) -> Result<std::sync::mpsc::Receiver<RuntimeEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(self.config.runtime_event_capacity);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx
            .send(ReactorCommand::SubscribeRuntimeEvents {
                sender,
                reply: reply_tx,
            })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok(receiver)
    }

    pub fn take_host_event_subscription(
        &self,
    ) -> Result<std::sync::mpsc::Receiver<HostEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(self.config.host_event_capacity);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx
            .send(ReactorCommand::SubscribeHostEvents {
                sender,
                reply: reply_tx,
            })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok(receiver)
    }

    pub fn status(&self) -> std::sync::Arc<crate::host_io::runtime_events::StatusEvent> {
        self.status_snapshot.load_full()
    }

    /// Return the raw identify bytes (zlib-compressed blob from firmware).
    /// Pass directly to klippy's `msgproto.MessageParser.process_identify`.
    pub fn raw_identify_bytes(&self) -> &[u8] {
        &self.raw_identify_bytes
    }

    /// Send a human-readable command string (e.g. `"get_uptime"`) and wait
    /// for a response with the given name. Returns a `HashMap<String, i64>`
    /// for integer fields; callers cast as needed.
    ///
    /// This is the Rust equivalent of klippy's `serial.send_with_response`.
    pub fn send_with_response(
        &self,
        cmd: &str,
        response: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        self.call(cmd, response, timeout)
    }

    /// Register a callback that fires on the reactor thread for every
    /// unsolicited frame whose `msg_name` (and optional `oid`) matches,
    /// before the frame is forwarded to the `RuntimeEvent` dispatcher.
    /// Returns an [`InterceptorId`] that can be passed to
    /// [`unregister_frame_interceptor`] to remove the callback.
    pub fn register_frame_interceptor(
        &self,
        msg_name: &str,
        oid: Option<u32>,
        callback: Box<dyn Fn(&crate::transport::MessageParams) + Send + Sync>,
    ) -> Result<InterceptorId, TransportError> {
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx
            .send(ReactorCommand::RegisterInterceptor {
                msg_name: msg_name.to_owned(),
                oid,
                callback: crate::host_io::interceptor::InterceptorCallback(callback),
                reply: reply_tx,
            })
            .map_err(|_| TransportError::Closed)?;
        reply_rx.recv().map_err(|_| TransportError::Closed)
    }

    /// Remove a previously registered frame interceptor. The callback will
    /// not fire for any frames processed after this call returns.
    pub fn unregister_frame_interceptor(&self, id: InterceptorId) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::UnregisterInterceptor { id })
            .map_err(|_| TransportError::Closed)
    }

    /// Send a command to the MCU without waiting for any response.
    /// The frame is wire-level ACKed by the MCU's next outbound frame but no
    /// application-level reply is expected.
    pub fn send_fire_and_forget(&self, cmd: &str) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::FireAndForget {
                cmd: cmd.to_owned(),
            })
            .map_err(|_| TransportError::Closed)
    }

    /// 2026-05-18: tell the reactor that a transport drop is imminent and
    /// must NOT trigger the EXIT_ON_FAULT abort. Used by klippy's
    /// bridge-mode `_restart_via_command` path right before sending the
    /// firmware `reset` command — `NVIC_SystemReset` on the MCU drops
    /// USB-CDC at the kernel and the reactor's BrokenPipe handler would
    /// otherwise interpret that as a wedge and abort the whole klippy
    /// process. The reactor handles `MarkExpectedDisconnect` by setting
    /// its internal `closed_via_shutdown` flag so the spawn-time
    /// `!reactor.exited_gracefully()` check sees the close as graceful.
    pub fn mark_expected_disconnect(&self) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::MarkExpectedDisconnect)
            .map_err(|_| TransportError::Closed)
    }

    /// Typed-args fire-and-forget. Encodes via `parser.encode_typed` (the same
    /// path as `call_typed`) and dispatches through the reactor's
    /// backpressure-respecting fire-and-forget queue (spec §6.0 / §6.1).
    /// No response is awaited; caller surfaces errors via the encoded payload
    /// being late or via length mismatch at higher protocol layers.
    pub fn send_typed(
        &self,
        name: &str,
        args: &[(&str, crate::host_io::parser::FieldValue<'_>)],
    ) -> Result<(), TransportError> {
        let payload = self
            .parser
            .encode_typed(name, args)
            .map_err(|e| TransportError::Parse(format!("{e:?}")))?;
        self.submission_tx
            .send(ReactorCommand::FireAndForgetTyped { payload })
            .map_err(|_| TransportError::Closed)
    }

    // ── Phase C-B: kalico-native transport surface ─────────────────────
    //
    // The reactor owns the wire and runs both protocols' demux state
    // machines. These methods submit kalico-native commands via the same
    // submission channel that drives the Klipper-protocol surface; the
    // reactor allocates correlation_ids, encodes frames, parks the caller
    // on a `SyncSender<...>`, and unblocks them when the response arrives.

    /// Run the bootstrap-ABI Identify handshake (spec §5). Validates
    /// `proto_version` and `schema_hash` against the host's compiled
    /// constants; on mismatch returns an error and motion dispatch must
    /// refuse to start. Returns the MCU's `reset_epoch` (spec §9).
    pub fn kalico_identify(
        &self,
        timeout: Duration,
    ) -> Result<crate::host_io::kalico_native::IdentifyOutcome, TransportError> {
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;
        self.submission_tx
            .send(ReactorCommand::KalicoIdentify {
                completion: tx,
                deadline,
            })
            .map_err(|_| TransportError::Closed)?;
        match rx.recv_timeout(timeout) {
            Ok(r) => r,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }

    /// Issue a kalico-native control-channel call (spec §7.2): one frame
    /// out on `CHANNEL_CONTROL` (kind + body), one frame in (matching
    /// correlation_id). Used by `producer::load_curve` / `push_segment` etc.
    /// See also [`kalico_call_on_channel`] for the pieces channel.
    pub fn kalico_call(
        &self,
        kind: kalico_protocol::MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(kalico_protocol::MessageKind, Vec<u8>), TransportError> {
        self.kalico_call_on_channel(
            kalico_native_transport::CHANNEL_CONTROL,
            kind,
            body,
            timeout,
        )
    }

    /// Issue a kalico-native call on an explicit outbound `channel`. The
    /// response always arrives on the control channel matched by
    /// correlation_id. Used by the pump to send `PushPieces` on
    /// `CHANNEL_PIECES` (0x02) while receiving `PushPiecesResponse` on the
    /// control channel.
    pub fn kalico_call_on_channel(
        &self,
        channel: u8,
        kind: kalico_protocol::MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(kalico_protocol::MessageKind, Vec<u8>), TransportError> {
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;
        self.submission_tx
            .send(ReactorCommand::KalicoCall {
                channel,
                kind,
                body,
                completion: tx,
                deadline,
            })
            .map_err(|_| TransportError::Closed)?;
        match rx.recv_timeout(timeout) {
            Ok(Ok(crate::host_io::kalico_native::KalicoCallOutcome::Response { kind, body })) => {
                Ok((kind, body))
            }
            Ok(Ok(crate::host_io::kalico_native::KalicoCallOutcome::Reset)) => {
                Err(TransportError::Closed)
            }
            Ok(Err(e)) => Err(e),
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }
}

#[cfg(test)]
mod test_internals;
