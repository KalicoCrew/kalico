//! `host_io` — production host I/O implementing [`Transport`].
//!
//! Phase B: the `KalicoHostIo` struct holds a `Mutex`-wrapped shim that
//! performs synchronous serial I/O. The `Transport::call` impl locks the
//! shim, encodes the command, writes it, then polls until the named
//! response arrives or the timeout fires.
//!
//! Phase C will lift the blocking lock to a background reactor thread;
//! for now the shim-lock approach is sufficient for single-stream
//! MVP use.

pub mod call_handle;
pub mod events;
pub mod identify;
pub mod parser;
pub mod rtt;
pub mod runtime_events;
pub mod window;
pub mod wire;

use std::collections::VecDeque;
use std::io::{Read, Write};
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::Sender;
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use serialport::SerialPort;

use crate::credit::CreditCounter;
use crate::host_io::events::HostEvent;
use crate::host_io::parser::MsgProtoParser;
use crate::host_io::runtime_events::{FaultEvent, RuntimeEvent, StatusEvent, TraceEvent};
use crate::transport::{MessageParams, SubscribeError, Transport, TransportError};
use std::sync::mpsc::SyncSender;

fn sp_err(e: &serialport::Error) -> TransportError {
    TransportError::Io(std::io::Error::other(format!("serialport: {e}")))
}

const DEFAULT_BAUD: u32 = 250_000;
const DEFAULT_IDENTIFY_TIMEOUT_MS: u64 = 15_000;

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

struct KalicoHostIoShimInner {
    port:          Box<dyn SerialPort>,
    host_send_seq: u8,
    rx_buf:        Vec<u8>,
    pending:       VecDeque<(String, MessageParams)>,
    parser:        MsgProtoParser,
}

pub struct KalicoHostIo {
    submission_tx:   Sender<ReactorCommand>,
    next_call_id:    AtomicU64,
    reactor_handle:  Option<JoinHandle<()>>,
    status_snapshot: Arc<ArcSwap<StatusEvent>>,
    shim:            Arc<Mutex<KalicoHostIoShimInner>>,
}

impl std::fmt::Debug for KalicoHostIo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let info = match self.shim.try_lock() {
            Ok(s) => format!(
                "host_send_seq={} pending={} rx_buf_len={}",
                s.host_send_seq,
                s.pending.len(),
                s.rx_buf.len()
            ),
            Err(_) => "(locked)".to_string(),
        };
        f.debug_struct("KalicoHostIo")
            .field("next_call_id", &self.next_call_id.load(Ordering::Relaxed))
            .field("shim", &info)
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
        Self::open_with_timeout(path, baud, Duration::from_millis(DEFAULT_IDENTIFY_TIMEOUT_MS))
    }

    pub fn open_default(path: &str) -> Result<Self, TransportError> {
        Self::open(path, DEFAULT_BAUD)
    }

    pub fn open_with_timeout(
        path: &str,
        baud: u32,
        identify_timeout: Duration,
    ) -> Result<Self, TransportError> {
        let mut port: Box<dyn SerialPort> = serialport::new(path, baud)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| {
                TransportError::Io(std::io::Error::other(format!(
                    "serialport::open({path}@{baud}): {e}"
                )))
            })?;

        let (parser, host_send_seq, rx_buf) = identify::identify_handshake(&mut port, identify_timeout)?;
        let (submission_tx, _rx) = std::sync::mpsc::channel();

        Ok(Self {
            submission_tx,
            next_call_id: AtomicU64::new(1),
            reactor_handle: None,
            status_snapshot: Arc::new(ArcSwap::from_pointee(StatusEvent::default())),
            shim: Arc::new(Mutex::new(KalicoHostIoShimInner {
                port,
                host_send_seq,
                rx_buf,
                pending: VecDeque::new(),
                parser,
            })),
        })
    }
}

fn pump_rx(shim: &mut KalicoHostIoShimInner, timeout: Duration) -> Result<(), TransportError> {
    let mut scratch = [0u8; 256];
    let read_to = timeout.min(Duration::from_millis(100));
    shim.port.set_timeout(read_to).map_err(|e| sp_err(&e))?;
    match shim.port.read(&mut scratch) {
        Ok(n) if n > 0 => {
            shim.rx_buf.extend_from_slice(&scratch[..n]);
            while let Some(packet) = wire::extract_packet(&mut shim.rx_buf) {
                match shim.parser.decode(&packet) {
                    Ok(crate::host_io::parser::DecodedFrame::Response { name, params }) => {
                        shim.pending.push_back((name, params));
                    }
                    Ok(crate::host_io::parser::DecodedFrame::Output { name, params }) => {
                        shim.pending.push_back((name, params));
                    }
                    Err(_) => {}
                }
            }
            Ok(())
        }
        Ok(_) => Ok(()),
        Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(()),
        Err(e) => Err(TransportError::Io(e)),
    }
}

impl Transport for KalicoHostIo {
    fn call(
        &self,
        cmd: &str,
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let mut shim = self.shim.lock().expect("shim mutex poisoned");
        let payload = shim.parser.encode(cmd)
            .map_err(|e| TransportError::Parse(format!("{e:?}")))?;
        let old_seq = shim.host_send_seq;
        let frame = wire::build_frame(&payload, old_seq);
        shim.port.write_all(&frame).map_err(TransportError::Io)?;
        shim.port.flush().map_err(TransportError::Io)?;
        shim.host_send_seq = old_seq.wrapping_add(1) & wire::MESSAGE_SEQ_MASK;

        let deadline = Instant::now() + timeout;
        loop {
            if let Some(idx) = shim.pending.iter().position(|(n, _)| n == expected_response_name) {
                let (_, params) = shim.pending.remove(idx).expect("position guarantees Some");
                return Ok(params);
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(TransportError::Timeout);
            }
            pump_rx(&mut shim, deadline - now)?;
        }
    }

    fn call_typed(
        &self,
        name: &str,
        args: &[(&str, crate::host_io::parser::FieldValue<'_>)],
        expected_response_name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let mut shim = self.shim.lock().expect("shim mutex poisoned");
        let payload = shim.parser.encode_typed(name, args)
            .map_err(|e| TransportError::Parse(format!("{e:?}")))?;
        let old_seq = shim.host_send_seq;
        let frame = wire::build_frame(&payload, old_seq);
        shim.port.write_all(&frame).map_err(TransportError::Io)?;
        shim.port.flush().map_err(TransportError::Io)?;
        shim.host_send_seq = old_seq.wrapping_add(1) & wire::MESSAGE_SEQ_MASK;

        let deadline = Instant::now() + timeout;
        loop {
            if let Some(idx) = shim.pending.iter().position(|(n, _)| n == expected_response_name) {
                let (_, params) = shim.pending.remove(idx).expect("position guarantees Some");
                return Ok(params);
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(TransportError::Timeout);
            }
            pump_rx(&mut shim, deadline - now)?;
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
