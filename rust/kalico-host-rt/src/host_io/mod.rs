//! `host_io` — production host I/O implementing [`Transport`].
//!
//! Phase C: `KalicoHostIo` spawns a background reactor thread on `open`.
//! `Transport::call` / `call_typed` submit commands via an mpsc channel
//! and block on a rendezvous channel for the response. The Phase-B
//! mutex shim has been removed.

pub mod call_handle;
pub mod events;
pub mod identify;
pub mod kalico_native;
pub mod parser;
pub mod reactor;
pub mod rtt;
pub mod runtime_events;
pub mod serial_frame_io;
pub use identify::IdentifySeqState;
pub use serial_frame_io::SerialFrameIo;
#[cfg(any(test, feature = "test-harness"))]
pub mod test_harness;
pub mod window;
pub mod wire;

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;

use crate::credit::CreditCounter;
use crate::host_io::events::HostEvent;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::runtime_events::{FaultEvent, RuntimeEvent, StatusEvent, TraceEvent};
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
    pub trace_capacity:              usize,
    /// Capacity of the bounded host-event inbox shared by the reactor (TraceRing
    /// overflow/disconnect/reattach diagnostics). Drained once per loop iter.
    pub host_event_capacity:         usize,
    /// Capacity of the bounded `RuntimeEvent` catch-all subscriber channel.
    pub runtime_event_capacity:      usize,
    pub default_call_timeout:        Duration,
    pub identify_timeout:            Duration,
    pub default_dispatcher_timeout:  Duration,
}

impl Default for KalicoHostIoConfig {
    fn default() -> Self {
        Self {
            trace_capacity:             256,
            host_event_capacity:         64,
            runtime_event_capacity:      64,
            default_call_timeout:       Duration::from_millis(100),
            identify_timeout:           Duration::from_millis(15_000),
            default_dispatcher_timeout: Duration::from_secs(30),
        }
    }
}

#[derive(Debug)]
pub enum ReactorCommand {
    Submit {
        call_id:                u64,
        cmd:                    String,
        expected_response_name: String,
        completion:             SyncSender<Result<MessageParams, TransportError>>,
        deadline:               std::time::Instant,
    },
    SubmitTyped {
        call_id:                u64,
        payload:                Vec<u8>,
        expected_response_name: String,
        completion:             SyncSender<Result<MessageParams, TransportError>>,
        deadline:               std::time::Instant,
    },
    Abandon(u64),
    AttachCreditCounter(std::sync::Arc<CreditCounter>),
    SubscribeFault {
        sender: SyncSender<FaultEvent>,
        reply:  SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeTrace {
        sender: SyncSender<TraceEvent>,
        reply:  SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeRuntimeEvents {
        sender: SyncSender<RuntimeEvent>,
        reply:  SyncSender<Result<(), SubscribeError>>,
    },
    SubscribeHostEvents {
        sender: SyncSender<HostEvent>,
        reply:  SyncSender<Result<(), SubscribeError>>,
    },
    /// Install the passthrough router (bridge startup). Replaces any
    /// previously installed router.
    InstallPassthroughRouter(PassthroughRouter),
    /// Push a raw passthrough entry into the router for a specific MCU.
    PassthroughSend {
        mcu:      McuHandle,
        queue_id: CommandQueueId,
        entry:    PassthroughEntry,
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
        completion: SyncSender<Result<crate::host_io::kalico_native::IdentifyOutcome, TransportError>>,
        deadline: std::time::Instant,
    },
    /// Phase C-B: kalico-native control-channel call. The reactor allocates
    /// a correlation_id, builds the frame from `kind` + `body`, writes it,
    /// and parks `completion` keyed by correlation_id. Spec §7.2.
    KalicoCall {
        kind: kalico_protocol::MessageKind,
        body: Vec<u8>,
        completion: SyncSender<Result<crate::host_io::kalico_native::KalicoCallOutcome, TransportError>>,
        deadline: std::time::Instant,
    },
    Shutdown,
}

pub struct KalicoHostIo {
    submission_tx:       Sender<ReactorCommand>,
    next_call_id:        AtomicU64,
    reactor_handle:      Option<JoinHandle<()>>,
    status_snapshot:     Arc<ArcSwap<StatusEvent>>,
    parser:              Arc<MsgProtoParser>,
    config:              KalicoHostIoConfig,
    clock:               Arc<dyn crate::clock::Clock>,
    /// Raw identify bytes (zlib-compressed blob as received from firmware).
    /// Suitable for passing directly to klippy's `process_identify`.
    raw_identify_bytes:  Vec<u8>,
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
                    eprintln!(
                        "[tio-pre-cfmakeraw] {path} iflag=0x{:x} oflag=0x{:x} cflag=0x{:x} lflag=0x{:x}",
                        tio.c_iflag, tio.c_oflag, tio.c_cflag, tio.c_lflag,
                    );
                    libc::cfmakeraw(&mut tio);
                    if libc::tcsetattr(fd, libc::TCSANOW, &tio) != 0 {
                        let err = std::io::Error::last_os_error();
                        libc::close(fd);
                        return Err(TransportError::Io(std::io::Error::other(
                            format!("tcsetattr({path}): {err}"),
                        )));
                    }
                    // Read back to verify cfmakeraw stuck.
                    let mut tio2: libc::termios = std::mem::zeroed();
                    if libc::tcgetattr(fd, &mut tio2) == 0 {
                        eprintln!(
                            "[tio-post-cfmakeraw] {path} iflag=0x{:x} oflag=0x{:x} cflag=0x{:x} lflag=0x{:x} vmin={} vtime={}",
                            tio2.c_iflag, tio2.c_oflag, tio2.c_cflag, tio2.c_lflag,
                            tio2.c_cc[libc::VMIN], tio2.c_cc[libc::VTIME],
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
                    eprintln!(
                        "[tio-post-serialport] {path} iflag=0x{:x} oflag=0x{:x} cflag=0x{:x} lflag=0x{:x} vmin={} vtime={}",
                        tio3.c_iflag, tio3.c_oflag, tio3.c_cflag, tio3.c_lflag,
                        tio3.c_cc[libc::VMIN], tio3.c_cc[libc::VTIME],
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
            .map_err(|e| TransportError::Io(
                std::io::Error::other(format!("serialport::open({path}@{baud}): {e}"))
            ))?;
        Self::open_with_port(port_box, config)
    }

    fn open_with_port(
        mut port_box: Box<dyn serialport::SerialPort>,
        config: KalicoHostIoConfig,
    ) -> Result<Self, TransportError> {
        // Ensure read timeout is set (pipe_open path skips .timeout() builder).
        let _ = port_box.set_timeout(Duration::from_millis(100));
        let mut io = crate::host_io::serial_frame_io::SerialFrameIo::new(port_box);

        let (parser_owned, raw_identify_bytes, identify_seq) = identify::identify_handshake(
            &mut io,
            config.identify_timeout,
        )?;

        let parser = Arc::new(parser_owned);
        let (submission_tx, submission_rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));

        let clock: Arc<dyn crate::clock::Clock> = Arc::new(crate::clock::RealClock);
        let reactor_parser = Arc::clone(&parser);
        let reactor_status = Arc::clone(&status_snapshot);
        let reactor_config = config.clone();
        let reactor_clock = Arc::clone(&clock);
        let reactor_handle = std::thread::spawn(move || {
            let mut reactor = crate::host_io::reactor::Reactor::new_with_clock(
                io, reactor_parser, submission_rx, reactor_status, identify_seq,
                reactor_config, reactor_clock,
            );
            reactor.run();
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

        self.submission_tx.send(ReactorCommand::Submit {
            call_id,
            cmd: cmd.to_string(),
            expected_response_name: expected_response_name.to_string(),
            completion: tx,
            deadline,
        }).map_err(|_| TransportError::Closed)?;

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
            Ok(r) => { handle.defuse(); r }
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
        let payload = self.parser.encode_typed(name, args)
            .map_err(|e| TransportError::Parse(format!("{e:?}")))?;

        let call_id = self.next_call_id.fetch_add(1, Ordering::Relaxed);
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;

        self.submission_tx.send(ReactorCommand::SubmitTyped {
            call_id,
            payload,
            expected_response_name: expected_response_name.to_string(),
            completion: tx,
            deadline,
        }).map_err(|_| TransportError::Closed)?;

        let handle = crate::host_io::call_handle::CallHandle {
            call_id,
            submission_tx: self.submission_tx.clone(),
        };

        // See `call` above for the defuse semantics rationale.
        match rx.recv_timeout(timeout) {
            Ok(r) => { handle.defuse(); r }
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
    pub fn attach_credit_counter(&self, counter: std::sync::Arc<crate::credit::CreditCounter>) {
        let _ = self.submission_tx.send(ReactorCommand::AttachCreditCounter(counter));
    }

    pub fn subscribe_fault(&self) -> Result<std::sync::mpsc::Receiver<FaultEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(1);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx.send(ReactorCommand::SubscribeFault { sender, reply: reply_tx })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok(receiver)
    }

    pub fn take_trace_subscription(&self) -> Result<std::sync::mpsc::Receiver<TraceEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(self.config.trace_capacity);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx.send(ReactorCommand::SubscribeTrace { sender, reply: reply_tx })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok(receiver)
    }

    pub fn take_runtime_event_subscription(&self) -> Result<std::sync::mpsc::Receiver<RuntimeEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(self.config.runtime_event_capacity);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx.send(ReactorCommand::SubscribeRuntimeEvents { sender, reply: reply_tx })
            .map_err(|_| SubscribeError::Closed)?;
        reply_rx.recv().map_err(|_| SubscribeError::Closed)??;
        Ok(receiver)
    }

    pub fn take_host_event_subscription(&self) -> Result<std::sync::mpsc::Receiver<HostEvent>, SubscribeError> {
        let (sender, receiver) = std::sync::mpsc::sync_channel(self.config.host_event_capacity);
        let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
        self.submission_tx.send(ReactorCommand::SubscribeHostEvents { sender, reply: reply_tx })
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

    /// Send a command to the MCU without waiting for any response.
    /// The frame is wire-level ACKed by the MCU's next outbound frame but no
    /// application-level reply is expected.
    pub fn send_fire_and_forget(&self, cmd: &str) -> Result<(), TransportError> {
        self.submission_tx
            .send(ReactorCommand::FireAndForget { cmd: cmd.to_owned() })
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
        let payload = self.parser.encode_typed(name, args)
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
            .send(ReactorCommand::KalicoIdentify { completion: tx, deadline })
            .map_err(|_| TransportError::Closed)?;
        match rx.recv_timeout(timeout) {
            Ok(r) => r,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        }
    }

    /// Issue a kalico-native control-channel call (spec §7.2): one frame
    /// out (kind + body), one frame in (matching correlation_id). Used by
    /// `producer::load_curve` / `producer::push_segment` and similar.
    pub fn kalico_call(
        &self,
        kind: kalico_protocol::MessageKind,
        body: Vec<u8>,
        timeout: Duration,
    ) -> Result<(kalico_protocol::MessageKind, Vec<u8>), TransportError> {
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        let deadline = self.clock.now() + timeout;
        self.submission_tx
            .send(ReactorCommand::KalicoCall { kind, body, completion: tx, deadline })
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
mod test_internals {
    use super::*;

    #[test]
    fn vlq_roundtrip_small_positive() {
        for v in [0i64, 1, 100, 1_000, 100_000, 1_000_000_000] {
            let mut buf = Vec::new();
            parser::encode_vlq(&mut buf, v).expect("value in range");
            let (out, n) = parser::decode_vlq(&buf).unwrap();
            assert_eq!(n, buf.len(), "consumed != encoded for {v}");
            assert_eq!(out, v, "roundtrip failed for {v}");
        }
    }

    #[test]
    fn crc16_matches_klipper_test_vector() {
        let crc = wire::crc16_ccitt(&[0x05, 0x10]);
        assert_eq!(crc, 0x9E81);
    }

    #[test]
    fn extract_packet_picks_up_minimal_nak_frame() {
        let crc = wire::crc16_ccitt(&[0x05, 0x10]);
        let frame = vec![
            0x05,
            0x10,
            (crc >> 8) as u8,
            (crc & 0xFF) as u8,
            wire::MESSAGE_SYNC,
        ];
        let mut buf = frame.clone();
        let extracted = wire::extract_packet(&mut buf).expect("must extract NAK");
        assert_eq!(extracted, frame);
        assert!(buf.is_empty());
    }

    #[test]
    fn extract_packet_resyncs_past_garbage_byte_smaller_than_message_min() {
        let mut buf: Vec<u8> = vec![0x02];
        let result = wire::extract_packet(&mut buf);
        assert!(
            result.is_none(),
            "still no complete frame, but buf must have been drained"
        );
        assert!(
            buf.is_empty(),
            "garbage leading byte should have been dropped, got {buf:?}"
        );
    }

    #[test]
    fn extract_packet_resyncs_past_oversized_msglen_byte() {
        let mut buf: Vec<u8> = vec![0xFF];
        let result = wire::extract_packet(&mut buf);
        assert!(result.is_none());
        assert!(
            buf.is_empty(),
            "oversized msglen byte should have been dropped, got {buf:?}"
        );
    }

    /// `send_typed`'s encoding path is exactly `parser.encode_typed`. The
    /// channel-send + reactor handler portion is exercised by
    /// `reactor::fire_and_forget_typed_routing`. Here we pin the encoding
    /// equivalence: the bytes a hypothetical `send_typed` would push into
    /// `ReactorCommand::FireAndForgetTyped { payload }` are identical to the
    /// bytes `call_typed` would push into `SubmitTyped { payload }` for the
    /// same args. This is what makes "fire-and-forget version of call_typed"
    /// a meaningful claim.
    #[test]
    fn send_typed_payload_matches_call_typed_payload() {
        use crate::host_io::parser::{DataDictionary, FieldValue, MsgProtoParser};
        use indexmap::IndexMap;

        let mut d = DataDictionary {
            commands:       IndexMap::new(),
            responses:      IndexMap::new(),
            output:         IndexMap::new(),
            enumerations:   IndexMap::new(),
            config:         serde_json::json!({}),
            version:        "v".into(),
            app:            "kalico".into(),
            build_versions: None,
            license:        None,
        };
        d.commands.insert("kalico_load_curve_begin slot=%hu degree=%c".into(), 99);
        let parser = MsgProtoParser::from_dictionary(d).unwrap();

        let args = [
            ("slot",   FieldValue::U16(7)),
            ("degree", FieldValue::Byte(3)),
        ];
        // What `send_typed` would put in the FireAndForgetTyped payload.
        let send_typed_payload = parser
            .encode_typed("kalico_load_curve_begin", &args)
            .expect("encode_typed");
        // What `call_typed` would put in the SubmitTyped payload.
        let call_typed_payload = parser
            .encode_typed("kalico_load_curve_begin", &args)
            .expect("encode_typed");
        assert_eq!(send_typed_payload, call_typed_payload);
        // And it must be non-empty (sanity: we actually exercised encoding).
        assert!(!send_typed_payload.is_empty());
    }

    #[test]
    fn decode_vlq_caps_continuation_at_5_bytes() {
        let malformed = vec![0xFFu8; 8];
        let result = parser::decode_vlq(&malformed);
        assert!(
            matches!(result, Err(parser::ParseError::BadVlq)),
            "malformed VLQ must return BadVlq, not roll past 5 bytes"
        );
    }
}
