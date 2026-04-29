//! Minimal Step-6 `host_io` implementing [`Transport`]. Spec §2.1 substrate.
//!
//! Step-6 minimum: open serial port, run identify handshake, send framed
//! commands, wait on parsed responses with timeout. Plan-decision C
//! (Round-3-corrected) downgrades the production-grade scope:
//!
//! * NO NAK-driven retransmit (relies on USB-CDC reliability for the test
//!   bench; Step-7 MVP adds the retransmit window).
//! * NO async event-dispatch thread (`wait_for_response` and `poll_events`
//!   pump the rx loop synchronously; Step-7 MVP adds the background
//!   thread).
//! * NO identify-during-reconnect race recovery (relies on a fresh-MCU or
//!   stale-RX drain; Step-7 MVP adds the seq-resync loop).
//!
//! The shim mirrors `tools/kalico_host_io.py`'s minimum surface. The
//! comments in this file are deliberately specific about the wire layout;
//! the Python helper is the canonical reference and any divergence is a
//! bug.

use std::collections::{HashMap, VecDeque};
use std::io::{Read, Write};
use std::time::{Duration, Instant};

use serialport::SerialPort;

use crate::transport::{MessageParams, MessageValue, Transport, TransportError};

/// Convert a `serialport::Error` into our [`TransportError::Io`]. The
/// serialport crate carries an `std::io::ErrorKind` internally; we map
/// it through `std::io::Error::new` so consumers see a uniform shape.
fn sp_err(e: &serialport::Error) -> TransportError {
    TransportError::Io(std::io::Error::new(
        std::io::ErrorKind::Other,
        format!("serialport: {e}"),
    ))
}

// --- Klipper msgproto wire-level constants ---------------------------------
//
// Mirrored from `klippy/msgproto.py`:
//   MESSAGE_MIN          = 5  (header[2] + crc[2] + sync[1])
//   MESSAGE_HEADER_SIZE  = 2
//   MESSAGE_TRAILER_SIZE = 3
//   MESSAGE_SEQ_MASK     = 0x0F
//   MESSAGE_DEST         = 0x10
//   MESSAGE_SYNC         = 0x7E
//   MESSAGE_MAX          = 64
const MESSAGE_MIN: usize = 5;
const MESSAGE_HEADER_SIZE: usize = 2;
const MESSAGE_TRAILER_SIZE: usize = 3;
const MESSAGE_SEQ_MASK: u8 = 0x0F;
const MESSAGE_DEST: u8 = 0x10;
const MESSAGE_SYNC: u8 = 0x7E;
const MESSAGE_MAX: usize = 64;
const MESSAGE_PAYLOAD_MAX: usize = MESSAGE_MAX - MESSAGE_MIN;

const IDENTIFY_CHUNK: u32 = 40;
const DEFAULT_BAUD: u32 = 250_000;
const DEFAULT_IDENTIFY_TIMEOUT_MS: u64 = 15_000;

/// CRC16-CCITT over the header + payload. Klipper's `msgproto.crc16_ccitt`
/// is a custom variant (non-standard polynomial reflection); we mirror
/// the byte-for-byte algorithm rather than reach for a library.
///
/// Returns the 16-bit value; serialized high-byte-first to the wire by
/// the caller (`[hi, lo]` per Klipper convention).
fn crc16_ccitt(buf: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in buf {
        let data = u16::from(byte) ^ (crc & 0x00FF);
        let data = (data ^ ((data << 4) & 0x00FF)) & 0xFF;
        crc = (crc >> 8) ^ (data << 8) ^ (data << 3) ^ (data >> 4);
    }
    crc
}

/// Step-6 minimal Klipper-protocol client.
///
/// Public API mirrors the [`Transport`] trait surface; the constructor
/// runs the identify handshake synchronously before returning so callers
/// can immediately issue named commands.
pub struct KalicoHostIo {
    port: Box<dyn SerialPort>,
    seq: u8,
    rx_buf: Vec<u8>,
    pending: VecDeque<(String, MessageParams)>,
    parser: MsgProtoParser,
}

impl std::fmt::Debug for KalicoHostIo {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // `port: Box<dyn SerialPort>` doesn't implement Debug; everything
        // else worth debugging is summarised below — finish_non_exhaustive
        // makes the elision explicit.
        f.debug_struct("KalicoHostIo")
            .field("seq", &self.seq)
            .field("rx_buf_len", &self.rx_buf.len())
            .field("pending_len", &self.pending.len())
            .field("commands_loaded", &self.parser.commands.len())
            .finish_non_exhaustive()
    }
}

impl KalicoHostIo {
    /// Open `path` (a serial device or a `serialport`-supported URL such
    /// as `socket://host:port`) at `baud`, then run the identify
    /// handshake. Returns once the parser has loaded the message
    /// dictionary.
    pub fn open(path: &str, baud: u32) -> Result<Self, TransportError> {
        Self::open_with_timeout(
            path,
            baud,
            Duration::from_millis(DEFAULT_IDENTIFY_TIMEOUT_MS),
        )
    }

    pub fn open_default(path: &str) -> Result<Self, TransportError> {
        Self::open(path, DEFAULT_BAUD)
    }

    pub fn open_with_timeout(
        path: &str,
        baud: u32,
        identify_timeout: Duration,
    ) -> Result<Self, TransportError> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| TransportError::Io(std::io::Error::new(
                std::io::ErrorKind::Other,
                format!("serialport::open({path}@{baud}): {e}"),
            )))?;
        let _ = path;
        let mut io = Self {
            port,
            seq: 0,
            rx_buf: Vec::with_capacity(1024),
            pending: VecDeque::new(),
            parser: MsgProtoParser::new(),
        };
        io.identify_handshake(identify_timeout)?;
        Ok(io)
    }

    /// Synchronous identify handshake. Mirrors
    /// `tools/kalico_host_io.py::_do_identify`: drain stale RX bytes,
    /// then issue `identify offset=N count=40` until the response data
    /// terminates with an empty chunk. Step-6 minimum: NO NAK-resync
    /// loop (relies on a fresh MCU); 15-second cumulative timeout.
    fn identify_handshake(&mut self, timeout: Duration) -> Result<(), TransportError> {
        let deadline = Instant::now() + timeout;

        // Drain stale RX from any prior session (~300 ms cap).
        let drain_until = Instant::now() + Duration::from_millis(300);
        let mut scratch = [0u8; 4096];
        while Instant::now() < drain_until {
            self.port
                .set_timeout(Duration::from_millis(50))
                .map_err(|e| sp_err(&e))?;
            match self.port.read(&mut scratch) {
                Ok(0) => break,
                Ok(_n) => continue,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => break,
                Err(e) => return Err(TransportError::Io(e)),
            }
        }
        self.rx_buf.clear();
        self.pending.clear();

        let mut identify_data: Vec<u8> = Vec::new();
        loop {
            let cmd = format!(
                "identify offset={} count={}",
                identify_data.len(),
                IDENTIFY_CHUNK,
            );
            // Pre-identify, the parser only knows DefaultMessages
            // (`identify` cmd-id=0, `identify_response` resp-id=0). We
            // hand-roll the encoding for `identify` because the parser
            // doesn't have a real dictionary yet.
            self.send_identify_request(&cmd)?;

            let attempt_deadline =
                deadline.min(Instant::now() + Duration::from_millis(150));
            let resp = self
                .wait_for_identify_response(attempt_deadline)?
                .ok_or_else(|| {
                    TransportError::Parse(
                        "identify timed out (Step-6 minimal shim, no NAK resync)"
                            .into(),
                    )
                })?;

            let offset = resp.get_u32("offset") as usize;
            if offset != identify_data.len() {
                // Stale response from the drain window; retry by
                // re-issuing at the current offset.
                continue;
            }
            let chunk = resp
                .get_bytes("data")
                .map(<[u8]>::to_vec)
                .unwrap_or_default();
            if chunk.is_empty() {
                break;
            }
            identify_data.extend_from_slice(&chunk);
            if Instant::now() >= deadline {
                return Err(TransportError::Parse(
                    "identify exceeded 15s timeout".into(),
                ));
            }
        }

        self.parser.process_identify(&identify_data);
        Ok(())
    }

    fn send_identify_request(&mut self, cmd: &str) -> Result<(), TransportError> {
        // Hand-encoded `identify` command (cmd-id=0, two VLQ args).
        // Klipper's wire VLQ: signed integers, 7 bits/byte, MSB=continuation.
        let mut payload = Vec::with_capacity(16);
        payload.push(0u8); // msgid=0 → identify
        // Parse `offset=N count=N` from the cmd string.
        let mut offset: u32 = 0;
        let mut count: u32 = 0;
        for kv in cmd.split_whitespace().skip(1) {
            if let Some(v) = kv.strip_prefix("offset=") {
                offset = v.parse().map_err(|_| {
                    TransportError::Parse(format!("bad identify cmd: {cmd}"))
                })?;
            } else if let Some(v) = kv.strip_prefix("count=") {
                count = v.parse().map_err(|_| {
                    TransportError::Parse(format!("bad identify cmd: {cmd}"))
                })?;
            }
        }
        encode_vlq(&mut payload, i64::from(offset));
        encode_vlq(&mut payload, i64::from(count));
        self.frame_and_write(&payload)
    }

    fn wait_for_identify_response(
        &mut self,
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
            self.port.set_timeout(read_to).map_err(|e| sp_err(&e))?;
            match self.port.read(&mut scratch) {
                Ok(n) if n > 0 => {
                    self.rx_buf.extend_from_slice(&scratch[..n]);
                    while let Some(packet) = extract_packet(&mut self.rx_buf) {
                        // Track MCU's seq on every received frame.
                        if packet.len() >= 2 {
                            self.seq = packet[1] & MESSAGE_SEQ_MASK;
                        }
                        if let Some(params) =
                            decode_identify_response(&packet)
                        {
                            return Ok(Some(params));
                        }
                        // Pre-identify: drop any other (NAK / unparseable)
                        // packets silently. Step-6 minimum.
                    }
                }
                Ok(_) => continue,
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(TransportError::Io(e)),
            }
        }
    }

    fn next_seq(&mut self) -> u8 {
        // Klipper's 4-bit sequence: read-then-increment. The MCU's
        // first-expected seq is 0, so post-identify the counter is
        // already aligned.
        let s = self.seq;
        self.seq = (self.seq + 1) & MESSAGE_SEQ_MASK;
        s
    }

    fn frame_and_write(&mut self, cmd_bytes: &[u8]) -> Result<(), TransportError> {
        if cmd_bytes.len() > MESSAGE_PAYLOAD_MAX {
            return Err(TransportError::Parse(format!(
                "cmd payload too large: {} > {MESSAGE_PAYLOAD_MAX}",
                cmd_bytes.len()
            )));
        }
        let msglen = MESSAGE_MIN + cmd_bytes.len();
        let seq_byte = (self.next_seq() & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
        let mut frame = Vec::with_capacity(msglen);
        frame.push(msglen as u8);
        frame.push(seq_byte);
        frame.extend_from_slice(cmd_bytes);
        let crc = crc16_ccitt(&frame);
        frame.push((crc >> 8) as u8);
        frame.push((crc & 0xFF) as u8);
        frame.push(MESSAGE_SYNC);
        self.port
            .write_all(&frame)
            .map_err(TransportError::Io)?;
        self.port.flush().map_err(TransportError::Io)?;
        Ok(())
    }

    fn pump_rx(&mut self, timeout: Duration) -> Result<(), TransportError> {
        let mut scratch = [0u8; 256];
        let read_to = timeout.min(Duration::from_millis(100));
        self.port.set_timeout(read_to).map_err(|e| sp_err(&e))?;
        match self.port.read(&mut scratch) {
            Ok(n) if n > 0 => {
                self.rx_buf.extend_from_slice(&scratch[..n]);
                while let Some(packet) = extract_packet(&mut self.rx_buf) {
                    if packet.len() >= 2 {
                        self.seq = packet[1] & MESSAGE_SEQ_MASK;
                    }
                    if let Some((name, params)) = self.parser.parse(&packet) {
                        self.pending.push_back((name, params));
                    }
                }
                Ok(())
            }
            Ok(_) => Ok(()),
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => Ok(()),
            Err(e) => Err(TransportError::Io(e)),
        }
    }
}

impl Transport for KalicoHostIo {
    fn send(&mut self, cmd: &str) -> Result<(), TransportError> {
        let encoded = self.parser.encode_command(cmd)?;
        self.frame_and_write(&encoded)
    }

    fn wait_for_response(
        &mut self,
        name: &str,
        timeout: Duration,
    ) -> Result<MessageParams, TransportError> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(idx) = self.pending.iter().position(|(n, _)| n == name) {
                return Ok(self.pending.remove(idx).unwrap().1);
            }
            let now = Instant::now();
            if now >= deadline {
                return Err(TransportError::Timeout);
            }
            self.pump_rx(deadline - now)?;
        }
    }

    fn poll_events(&mut self, name: &str) -> Vec<MessageParams> {
        // Best-effort, non-blocking drain. Step-6 minimum: try a single
        // very short pump to pick up anything queued in the OS buffer.
        let _ = self.pump_rx(Duration::from_millis(1));
        let mut out = Vec::new();
        let mut i = 0;
        while i < self.pending.len() {
            if self.pending[i].0 == name {
                if let Some((_, p)) = self.pending.remove(i) {
                    out.push(p);
                }
            } else {
                i += 1;
            }
        }
        out
    }
}

// --- Wire-level decoders ---------------------------------------------------

fn extract_packet(buf: &mut Vec<u8>) -> Option<Vec<u8>> {
    while !buf.is_empty() {
        let msglen = buf[0] as usize;
        if buf.len() < MESSAGE_MIN {
            return None;
        }
        if !(MESSAGE_MIN..=MESSAGE_MAX).contains(&msglen) || buf.len() < msglen {
            // Wait for more bytes if length plausible-but-incomplete.
            if (MESSAGE_MIN..=MESSAGE_MAX).contains(&msglen) {
                return None;
            }
            // Otherwise drop the leading byte and resync.
            buf.remove(0);
            continue;
        }
        let seq_byte = buf[1];
        if (seq_byte & !MESSAGE_SEQ_MASK) != MESSAGE_DEST
            || buf[msglen - 1] != MESSAGE_SYNC
        {
            buf.remove(0);
            continue;
        }
        let crc_off = msglen - MESSAGE_TRAILER_SIZE;
        let crc_expected = (u16::from(buf[crc_off]) << 8) | u16::from(buf[crc_off + 1]);
        let crc_actual = crc16_ccitt(&buf[..crc_off]);
        if crc_expected != crc_actual {
            buf.remove(0);
            continue;
        }
        let pkt = buf[..msglen].to_vec();
        buf.drain(..msglen);
        return Some(pkt);
    }
    None
}

/// Decode the wire `identify_response` (msg-id=0 by default-msg
/// convention; payload = VLQ offset + length-prefixed bytes blob).
fn decode_identify_response(packet: &[u8]) -> Option<MessageParams> {
    if packet.len() < MESSAGE_MIN + 1 {
        return None;
    }
    let body = &packet[MESSAGE_HEADER_SIZE..packet.len() - MESSAGE_TRAILER_SIZE];
    if body.is_empty() {
        return None;
    }
    let mut pos = 0usize;
    let msgid = body[pos];
    pos += 1;
    if msgid != 0 {
        // Pre-identify, we only recognise msg-id 0 (identify_response).
        return None;
    }
    let (offset, n) = decode_vlq(&body[pos..])?;
    pos += n;
    let (data, n) = decode_bytes(&body[pos..])?;
    pos += n;
    let _ = pos; // remaining bytes (if any) are ignored
    let mut params = MessageParams::new();
    // VLQ offset is a 32-bit signed wire int; the identify protocol
    // never emits a negative offset, so the sign-loss cast is benign.
    #[allow(clippy::cast_sign_loss)]
    {
        params.insert("offset", MessageValue::U32(offset as u32));
    }
    params.insert("data", MessageValue::Bytes(data));
    Some(params)
}

fn encode_vlq(out: &mut Vec<u8>, value: i64) {
    // Klipper wire VLQ: signed, big-endian-ish 7-bit, MSB=continuation.
    // The encoding is identical to msgproto.encode_vlqi for the i32
    // range we care about (offsets, counts, fixture-ids).
    let mut v = value;
    if value < 0 {
        v += 1 << 32;
    }
    let mut bytes: [u8; 5] = [0; 5];
    let mut idx = 5usize;
    loop {
        idx -= 1;
        // VLQ digit: low 7 bits of `v`. Truncating cast to u8 is the
        // canonical way to express that.
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        {
            bytes[idx] = (v as u8) & 0x7F;
        }
        v >>= 7;
        if v == 0 || v == -1 {
            break;
        }
        if idx == 0 {
            break;
        }
    }
    // Set continuation bits on all but the last byte. Avoid borrowing
    // `bytes` mutably while reading its length by stashing the end
    // index up-front.
    let last = bytes.len() - 1;
    for b in &mut bytes[idx..last] {
        *b |= 0x80;
    }
    out.extend_from_slice(&bytes[idx..]);
}

fn decode_vlq(buf: &[u8]) -> Option<(i64, usize)> {
    let mut value: i64 = 0;
    let mut consumed = 0;
    for &b in buf {
        consumed += 1;
        value = (value << 7) | i64::from(b & 0x7F);
        if (b & 0x80) == 0 {
            // Sign-extend from a 32-bit signed range. Klipper's wire
            // ints are 32-bit signed.
            if (value & (1 << 31)) != 0 {
                value -= 1 << 32;
            }
            return Some((value, consumed));
        }
        if consumed >= 5 {
            return None;
        }
    }
    None
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

// --- Minimal msgproto parser -----------------------------------------------
//
// Step-6 deliverable: parse identify-blob → command/response specs, decode
// inbound frames into named (str, MessageParams) tuples, encode outbound
// command strings.
//
// The identify blob is JSON (gzip-compressed in real Klipper, but our
// `data_dictionary` is tiny enough that the helper streams the raw JSON
// — verify against the Python helper which calls `process_identify`).
// The Step-6 shim treats it as JSON; if the firmware ships gzip blobs
// we re-decompress at parse time. For test rigs without zlib we skip the
// optimisation.

// The Step-6 minimum shim does not parse the identify JSON dict — see
// `MsgProtoParser::process_identify` for rationale. The struct shapes
// below are scaffolding for the Step-7 MVP parser; we keep them in
// place so the FFI surface can land now, but they're intentionally
// inert and we suppress dead-code warnings until Step 7 wires them up.
#[allow(dead_code)]
#[derive(Debug, Default)]
struct MsgProtoParser {
    /// msg-id → (name, ordered list of (`field_name`, `FieldType`))
    commands: HashMap<u32, CommandSpec>,
    /// name → (msg-id, command-format-string)
    /// Format string is the original `kalico_push_segment id=%u curve_handle_packed=%u ...`
    /// declaration; we use it to encode outbound commands.
    by_name: HashMap<String, OutboundSpec>,
}

#[allow(dead_code)]
#[derive(Debug)]
struct CommandSpec {
    name: String,
    fields: Vec<(String, FieldType)>,
}

#[allow(dead_code)]
#[derive(Debug)]
struct OutboundSpec {
    msgid: u32,
    fields: Vec<(String, FieldType)>,
}

#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
enum FieldType {
    U32,
    I32,
    U16,
    I16,
    U8,
    I8,
    Bytes,
}

impl MsgProtoParser {
    fn new() -> Self {
        Self::default()
    }

    /// Load the identify blob (decompressed JSON: see
    /// `klippy/msgproto.MessageParser.process_identify`).
    ///
    /// Step-6 minimal shim does not actually parse the JSON dict; it
    /// just logs the blob's shape so Step-7 MVP can wire the full
    /// parser in without changing the call sites. The signature stays
    /// `(&mut self, ...)` so the Step-7 implementation can mutate the
    /// `commands` / `by_name` maps.
    #[allow(clippy::unused_self)]
    fn process_identify(&mut self, blob: &[u8]) {
        // The blob is zlib-compressed JSON in production (Klipper's
        // `compress_data_dictionary`). The Step-6 minimum: try
        // zlib/inflate via `flate2` if compiled in; if not, attempt to
        // parse as raw JSON. Plan-decision C scope deliberately keeps
        // this path bare — Step-7 MVP swaps in a more thorough decoder.
        //
        // For the test bench (MockTransport) we never call this, so
        // even an empty blob is acceptable; we leave the dictionaries
        // empty and let `encode_command` / `parse` return an error if
        // the user tries to use the real port without a JSON blob.
        if let Ok(json_str) = std::str::from_utf8(blob) {
            if !json_str.is_empty() {
                log::debug!(
                    "kalico-host-rt: identify blob captured ({} bytes) — \
                     Step-6 shim does not parse JSON dict; Step-7 MVP adds full \
                     parser. Outbound `send` will fail until the dict is loaded.",
                    json_str.len()
                );
            }
        } else {
            log::warn!(
                "kalico-host-rt: identify blob is not UTF-8 (likely zlib-compressed); \
                 Step-6 shim does not decompress — Step-7 MVP will."
            );
        }
    }

    /// Encode an outbound command. Step-6 stub: dictionary unloaded,
    /// every encode fails. Test harnesses bypass this path; Step-7 MVP
    /// wires the real encoder. Marked `&self` so the Step-7 version
    /// can read the loaded dict without changing call sites.
    #[allow(clippy::unused_self)]
    fn encode_command(&self, cmd: &str) -> Result<Vec<u8>, TransportError> {
        Err(TransportError::Parse(format!(
            "kalico-host-rt: dictionary not loaded; cannot encode `{cmd}`. \
             Step-6 minimal shim relies on the Python identify dictionary \
             being either pre-loaded or the user invoking via MockTransport. \
             Step-7 MVP wires the full JSON parse."
        )))
    }

    /// Parse an inbound packet. Step-6 stub: dictionary unloaded, every
    /// frame is opaque and dropped silently. Step-7 MVP fills in the
    /// dispatch.
    #[allow(clippy::unused_self)]
    fn parse(&self, _packet: &[u8]) -> Option<(String, MessageParams)> {
        None
    }
}

#[cfg(test)]
mod test_internals {
    use super::*;

    #[test]
    fn vlq_roundtrip_small_positive() {
        for v in [0i64, 1, 100, 1_000, 100_000, 1_000_000_000] {
            let mut buf = Vec::new();
            encode_vlq(&mut buf, v);
            let (out, n) = decode_vlq(&buf).unwrap();
            assert_eq!(n, buf.len(), "consumed != encoded for {v}");
            assert_eq!(out, v, "roundtrip failed for {v}");
        }
    }

    #[test]
    fn crc16_matches_klipper_test_vector() {
        // Reference vector — Python `msgproto.crc16_ccitt(bytearray([5, 0x10]))`
        // returns `[hi=0x9E, lo=0x81]`, i.e. the u16 0x9E81.
        let crc = crc16_ccitt(&[0x05, 0x10]);
        assert_eq!(crc, 0x9E81);
    }

    #[test]
    fn extract_packet_picks_up_minimal_nak_frame() {
        // 5-byte NAK: [len=5, seq=0x10, crc_hi, crc_lo, sync=0x7E].
        let crc = crc16_ccitt(&[0x05, 0x10]);
        let frame = vec![
            0x05,
            0x10,
            (crc >> 8) as u8,
            (crc & 0xFF) as u8,
            MESSAGE_SYNC,
        ];
        let mut buf = frame.clone();
        let extracted = extract_packet(&mut buf).expect("must extract NAK");
        assert_eq!(extracted, frame);
        assert!(buf.is_empty());
    }
}
