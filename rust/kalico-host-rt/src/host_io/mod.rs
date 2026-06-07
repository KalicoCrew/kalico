//! Production host I/O implementing [`Transport`].

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
    pub trace_capacity: usize,
    pub host_event_capacity: usize,
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

pub struct HeartbeatCallback(pub Arc<dyn Fn(&[u32]) + Send + Sync>);

impl std::fmt::Debug for HeartbeatCallback {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("HeartbeatCallback(<fn>)")
    }
}

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
    InstallPassthroughRouter(PassthroughRouter),
    PassthroughSend {
        mcu: McuHandle,
        queue_id: CommandQueueId,
        entry: PassthroughEntry,
    },
    FireAndForget {
        cmd: String,
    },
    FireAndForgetTyped {
        payload: Vec<u8>,
    },
    KalicoIdentify {
        completion:
            SyncSender<Result<crate::host_io::kalico_native::IdentifyOutcome, TransportError>>,
        deadline: std::time::Instant,
    },
    KalicoCall {
        channel: u8,
        kind: kalico_protocol::MessageKind,
        body: Vec<u8>,
        completion:
            SyncSender<Result<crate::host_io::kalico_native::KalicoCallOutcome, TransportError>>,
        deadline: std::time::Instant,
    },
    /// Encode and send `get_clock` (with the reactor's own per-MCU parser —
    /// the bridge-level parser carries whichever MCU's dictionary was set
    /// last and must never encode for a specific MCU). The send timestamp is
    /// captured in the reactor immediately before the wire write; when the
    /// "clock" response arrives as an unsolicited frame the reactor injects
    /// honest RAW timestamps and delivers it as a PassthroughResponse
    /// runtime event.
    GetClockAndDeliver,
    Shutdown,
    Noop,
    RegisterInterceptor {
        msg_name: String,
        oid: Option<u32>,
        callback: crate::host_io::interceptor::InterceptorCallback,
        reply: SyncSender<crate::host_io::InterceptorId>,
    },
    UnregisterInterceptor {
        id: crate::host_io::InterceptorId,
    },
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
    raw_identify_bytes: Vec<u8>,
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
            // O_CLOEXEC: the host transport fd must never be inherited by an
            // EtherCAT endpoint child (spawned via Command, which only sets
            // CLOEXEC on its own pipes). An inherited copy of this pts fd would
            // keep TIOCEXCL held even after the parent closes its copy, so the
            // next klippy's attach_serial would still get EBUSY.
            let fd = unsafe {
                libc::open(
                    cpath.as_ptr(),
                    libc::O_RDWR | libc::O_NOCTTY | libc::O_CLOEXEC,
                )
            };
            if fd < 0 {
                return Err(TransportError::Io(std::io::Error::last_os_error()));
            }
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

    pub fn open_tcp(addr: &str, config: KalicoHostIoConfig) -> Result<Self, TransportError> {
        let port_box: Box<dyn serialport::SerialPort> =
            Box::new(tcp_serial_port::TcpSerialPort::connect(addr)?);
        Self::open_with_port(port_box, config)
    }

    pub fn open_with_port(
        mut port_box: Box<dyn serialport::SerialPort>,
        config: KalicoHostIoConfig,
    ) -> Result<Self, TransportError> {
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
                let _ = std::io::Write::flush(&mut std::io::stderr());
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

    /// Build a `KalicoHostIo` from an already-open port, skipping the blocking
    /// identify handshake. The reactor still runs and owns the fd, so this is a
    /// faithful seam for teardown tests (Drop must send Shutdown, join the
    /// reactor, and close the fd) without needing a wire-protocol responder.
    #[cfg(any(test, feature = "test-harness"))]
    pub fn from_port_skip_identify(
        port_box: Box<dyn serialport::SerialPort>,
        config: KalicoHostIoConfig,
    ) -> Self {
        use crate::host_io::identify::IdentifySeqState;

        let io = crate::host_io::serial_frame_io::SerialFrameIo::new(port_box);
        let parser = Arc::new(MsgProtoParser::new_empty());
        let (submission_tx, submission_rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));
        let clock: Arc<dyn crate::clock::Clock> = Arc::new(crate::clock::RealClock);
        let identify_seq = IdentifySeqState {
            next_send_seq_abs: 1,
            mcu_receive_seq_abs: 0,
        };

        let reactor_parser = Arc::clone(&parser);
        let reactor_status = Arc::clone(&status_snapshot);
        let reactor_config = config.clone();
        let reactor_clock = Arc::clone(&clock);
        let is_critical = Arc::new(AtomicBool::new(false));
        let reactor_handle = std::thread::spawn(move || {
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
        });

        Self {
            submission_tx,
            next_call_id: AtomicU64::new(1),
            reactor_handle: Some(reactor_handle),
            status_snapshot,
            parser,
            config,
            clock,
            raw_identify_bytes: Vec::new(),
            is_critical,
        }
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
        KalicoHostIo::send_typed(self, name, args)
    }
}

impl KalicoHostIo {
    pub fn is_alive(&self) -> bool {
        self.submission_tx.send(ReactorCommand::Noop).is_ok()
    }

    pub fn set_critical(&self, critical: bool) {
        self.is_critical.store(critical, Ordering::Release);
    }

    pub fn is_critical(&self) -> bool {
        self.is_critical.load(Ordering::Acquire)
    }

    pub fn attach_heartbeat_callback(&self, cb: Arc<dyn Fn(&[u32]) + Send + Sync>) {
        let _ = self
            .submission_tx
            .send(ReactorCommand::AttachHeartbeatCallback(HeartbeatCallback(
                cb,
            )));
    }

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

    pub fn raw_identify_bytes(&self) -> &[u8] {
        &self.raw_identify_bytes
    }

    pub fn send_with_response(
        &self,
        cmd: &str,
        response: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        self.call(cmd, response, timeout)
    }

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

    pub fn unregister_frame_interceptor(&self, id: InterceptorId) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::UnregisterInterceptor { id })
            .map_err(|_| TransportError::Closed)
    }

    pub fn send_fire_and_forget(&self, cmd: &str) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::FireAndForget {
                cmd: cmd.to_owned(),
            })
            .map_err(|_| TransportError::Closed)
    }

    /// Ask the reactor to encode+send `get_clock` and stamp the matching
    /// "clock" response with CLOCK_MONOTONIC_RAW timestamps (send stamp
    /// captured in the reactor immediately before the wire write). The
    /// response is delivered as a `PassthroughResponse { name: "clock", .. }`
    /// runtime event with `MessageParams::sent_time_raw` / `recv_time_raw`
    /// filled in, so the Python `_bridge_event_poller` can inject them into
    /// `#sent_time` / `#receive_time` before dispatching to `_handle_clock`.
    pub fn get_clock_async(&self) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::GetClockAndDeliver)
            .map_err(|_| TransportError::Closed)
    }

    pub fn mark_expected_disconnect(&self) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::MarkExpectedDisconnect)
            .map_err(|_| TransportError::Closed)
    }

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
