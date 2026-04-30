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

use crate::host_io::parser::MsgProtoParser;
use crate::host_io::runtime_events::StatusEvent;
use crate::transport::{MessageParams, MessageValue, Transport, TransportError};

fn sp_err(e: &serialport::Error) -> TransportError {
    TransportError::Io(std::io::Error::other(format!("serialport: {e}")))
}

const IDENTIFY_CHUNK: u32 = 40;
const DEFAULT_BAUD: u32 = 250_000;
const DEFAULT_IDENTIFY_TIMEOUT_MS: u64 = 15_000;

#[derive(Debug)]
pub enum ReactorCommand {
    Abandon(u64),
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

        let (parser, host_send_seq, rx_buf) = identify_handshake(&mut port, identify_timeout)?;
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

/// Synchronous identify handshake.
///
/// Drains stale RX bytes, then issues `identify offset=N count=40` until
/// the response data terminates with an empty chunk. Returns `(parser,
/// seq, rx_buf)` on success.
pub(crate) fn identify_handshake(
    port: &mut Box<dyn SerialPort>,
    timeout: Duration,
) -> Result<(MsgProtoParser, u8, Vec<u8>), TransportError> {
    let deadline = Instant::now() + timeout;

    let drain_until = Instant::now() + Duration::from_millis(300);
    let mut scratch = [0u8; 4096];
    while Instant::now() < drain_until {
        port.set_timeout(Duration::from_millis(50)).map_err(|e| sp_err(&e))?;
        match port.read(&mut scratch) {
            Ok(0) => break,
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => break,
            Err(e) => return Err(TransportError::Io(e)),
        }
    }

    let mut rx_buf: Vec<u8> = Vec::new();
    let mut seq: u8 = 0;
    let mut identify_data: Vec<u8> = Vec::new();

    loop {
        // Encode `identify offset=N count=40` by hand (no dict yet).
        let mut payload = Vec::with_capacity(16);
        payload.push(0u8); // msgid=0 → identify
        parser::encode_vlq(&mut payload, identify_data.len() as i64)
            .expect("identify offset is u32-range");
        parser::encode_vlq(&mut payload, i64::from(IDENTIFY_CHUNK))
            .expect("identify count is u32-range");

        let frame = wire::build_frame(&payload, seq);
        port.write_all(&frame).map_err(TransportError::Io)?;
        port.flush().map_err(TransportError::Io)?;
        seq = seq.wrapping_add(1) & wire::MESSAGE_SEQ_MASK;

        let attempt_deadline = deadline.min(Instant::now() + Duration::from_millis(150));
        let resp = wait_for_identify_response(port, &mut rx_buf, attempt_deadline)?
            .ok_or_else(|| {
                TransportError::Parse(
                    "identify timed out (Phase-B shim, no NAK resync)".into(),
                )
            })?;

        let offset = resp.get_u32("offset") as usize;
        if offset != identify_data.len() {
            continue;
        }
        let chunk = resp.get_bytes("data").map(<[u8]>::to_vec).unwrap_or_default();
        if chunk.is_empty() {
            break;
        }
        identify_data.extend_from_slice(&chunk);
        if Instant::now() >= deadline {
            return Err(TransportError::Parse("identify exceeded timeout".into()));
        }
    }

    let parser = build_parser_from_identify(&identify_data);
    Ok((parser, seq, rx_buf))
}

fn wait_for_identify_response(
    port: &mut Box<dyn SerialPort>,
    rx_buf: &mut Vec<u8>,
    deadline: Instant,
) -> Result<Option<MessageParams>, TransportError> {
    let mut scratch = [0u8; 256];
    loop {
        let now = Instant::now();
        if now >= deadline {
            return Ok(None);
        }
        let remaining = deadline - now;
        let read_to = remaining.min(Duration::from_millis(100));
        port.set_timeout(read_to).map_err(|e| sp_err(&e))?;
        match port.read(&mut scratch) {
            Ok(n) if n > 0 => {
                rx_buf.extend_from_slice(&scratch[..n]);
                while let Some(packet) = wire::extract_packet(rx_buf) {
                    if let Some(params) = decode_identify_response(&packet) {
                        return Ok(Some(params));
                    }
                }
            }
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(TransportError::Io(e)),
        }
    }
}

fn build_parser_from_identify(identify_data: &[u8]) -> MsgProtoParser {
    use crate::host_io::parser::DataDictionary;

    // Try zlib inflate first (Klipper firmware compresses the dict).
    let json_bytes = if identify_data.first() == Some(&0x78) {
        let mut decoder = flate2::read::ZlibDecoder::new(identify_data);
        let mut out = Vec::new();
        if std::io::Read::read_to_end(&mut decoder, &mut out).is_ok() {
            out
        } else {
            identify_data.to_vec()
        }
    } else {
        identify_data.to_vec()
    };

    let Ok(json_str) = std::str::from_utf8(&json_bytes) else {
        log::warn!("kalico-host-rt: identify blob is not valid UTF-8 after inflate");
        return empty_parser();
    };

    let dict: DataDictionary = match serde_json::from_str(json_str) {
        Ok(d) => d,
        Err(e) => {
            log::warn!("kalico-host-rt: identify JSON parse failed: {e}");
            return empty_parser();
        }
    };

    match MsgProtoParser::from_dictionary(dict) {
        Ok(p) => p,
        Err(e) => {
            log::warn!("kalico-host-rt: MsgProtoParser::from_dictionary failed: {e:?}");
            empty_parser()
        }
    }
}

fn empty_parser() -> MsgProtoParser {
    use crate::host_io::parser::DataDictionary;
    use indexmap::IndexMap;
    let dict = DataDictionary {
        commands: IndexMap::new(),
        responses: IndexMap::new(),
        output: IndexMap::new(),
        enumerations: IndexMap::new(),
        config: serde_json::json!({}),
        version: String::new(),
        app: String::new(),
        build_versions: None,
        license: None,
    };
    MsgProtoParser::from_dictionary(dict).expect("empty dict cannot fail")
}

fn decode_identify_response(packet: &[u8]) -> Option<MessageParams> {
    if packet.len() < wire::MESSAGE_MIN + 1 {
        return None;
    }
    let body = &packet[wire::MESSAGE_HEADER_SIZE..packet.len() - wire::MESSAGE_TRAILER_SIZE];
    if body.is_empty() {
        return None;
    }
    let mut pos = 0usize;
    let msgid = body[pos];
    pos += 1;
    if msgid != 0 {
        return None;
    }
    let (offset, n) = parser::decode_vlq(&body[pos..]).ok()?;
    pos += n;
    let (data, _n) = decode_bytes(&body[pos..])?;
    let mut params = MessageParams::new();
    #[allow(clippy::cast_sign_loss)]
    {
        params.insert("offset", MessageValue::U32(offset as u32));
    }
    params.insert("data", MessageValue::Bytes(data));
    Some(params)
}

fn decode_bytes(buf: &[u8]) -> Option<(Vec<u8>, usize)> {
    if buf.is_empty() {
        return None;
    }
    let len = buf[0] as usize;
    if buf.len() < 1 + len {
        return None;
    }
    Some((buf[1..=len].to_vec(), 1 + len))
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
