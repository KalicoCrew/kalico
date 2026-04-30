//! `host_io` — production host I/O implementing [`Transport`].
//!
//! Phase C: `KalicoHostIo` spawns a background reactor thread on `open`.
//! `Transport::call` / `call_typed` submit commands via an mpsc channel
//! and block on a rendezvous channel for the response. The Phase-B
//! mutex shim has been removed.

pub mod call_handle;
pub mod events;
pub mod identify;
pub mod parser;
pub mod reactor;
pub mod rtt;
pub mod runtime_events;
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
use crate::transport::{MessageParams, SubscribeError, Transport, TransportError};
use std::sync::mpsc::SyncSender;

pub(super) fn sp_err(e: &serialport::Error) -> TransportError {
    TransportError::Io(std::io::Error::other(format!("serialport: {e}")))
}

const DEFAULT_BAUD: u32 = 250_000;

pub struct KalicoHostIoConfig {
    pub trace_capacity:              usize,
    pub default_call_timeout:        Duration,
    pub identify_timeout:            Duration,
    pub default_dispatcher_timeout:  Duration,
}

impl Default for KalicoHostIoConfig {
    fn default() -> Self {
        Self {
            trace_capacity:             256,
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
    Shutdown,
}

pub struct KalicoHostIo {
    submission_tx:   Sender<ReactorCommand>,
    next_call_id:    AtomicU64,
    reactor_handle:  Option<JoinHandle<()>>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    parser:          Arc<MsgProtoParser>,
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

    pub fn open_with_config(
        path: &str,
        baud: u32,
        config: KalicoHostIoConfig,
    ) -> Result<Self, TransportError> {
        let mut port_box: Box<dyn serialport::SerialPort> = serialport::new(path, baud)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| TransportError::Io(
                std::io::Error::other(format!("serialport::open({path}@{baud}): {e}"))
            ))?;

        let (parser_owned, _seq, rx_buf) = identify::identify_handshake(
            &mut port_box,
            config.identify_timeout,
        )?;

        let parser = Arc::new(parser_owned);
        let (submission_tx, submission_rx) = std::sync::mpsc::channel();
        let status_snapshot = Arc::new(ArcSwap::from_pointee(StatusEvent::default()));

        let reactor_parser = Arc::clone(&parser);
        let reactor_status = Arc::clone(&status_snapshot);
        let reactor_handle = std::thread::spawn(move || {
            let mut reactor = crate::host_io::reactor::Reactor::new(
                port_box, reactor_parser, submission_rx, reactor_status, rx_buf,
            );
            reactor.run();
        });

        Ok(Self {
            submission_tx,
            next_call_id: AtomicU64::new(1),
            reactor_handle: Some(reactor_handle),
            status_snapshot,
            parser,
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
        let deadline = Instant::now() + timeout;

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

        let result = match rx.recv_timeout(timeout) {
            Ok(r) => r,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        };
        handle.defuse();
        result
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
        let deadline = Instant::now() + timeout;

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

        let result = match rx.recv_timeout(timeout) {
            Ok(r) => r,
            Err(std::sync::mpsc::RecvTimeoutError::Timeout) => Err(TransportError::Timeout),
            Err(std::sync::mpsc::RecvTimeoutError::Disconnected) => Err(TransportError::Closed),
        };
        handle.defuse();
        result
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
