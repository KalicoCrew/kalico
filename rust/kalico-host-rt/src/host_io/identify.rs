//! Synchronous identify handshake — extracts the firmware data-dictionary
//! so we can build a [`MsgProtoParser`] before the reactor starts.

use std::io::{Read, Write};
use std::time::{Duration, Instant};

use serialport::SerialPort;

use crate::host_io::parser::{DataDictionary, MsgProtoParser, decode_vlq, encode_vlq};
use crate::host_io::wire::{
    MESSAGE_HEADER_SIZE, MESSAGE_MIN, MESSAGE_SEQ_MASK, MESSAGE_TRAILER_SIZE,
    build_frame,
};
use crate::transport::{MessageParams, MessageValue, TransportError};
use kalico_native_transport::demux::{Demuxer, DemuxOutput};

const IDENTIFY_CHUNK: u32 = 40;

/// Synchronous identify handshake.
///
/// Drains stale RX bytes, then issues `identify offset=N count=40` until
/// the response data terminates with an empty chunk. Returns `(parser,
/// raw_identify_bytes, seq, rx_buf)` on success. `raw_identify_bytes` is
/// the raw (zlib-compressed or plain) blob as received from the firmware —
/// suitable for passing to klippy's `msgproto.MessageParser.process_identify`.
pub fn identify_handshake(
    port: &mut Box<dyn SerialPort>,
    timeout: Duration,
) -> Result<(MsgProtoParser, Vec<u8>, u8, Vec<u8>), TransportError> {
    let deadline = Instant::now() + timeout;

    let drain_until = Instant::now() + Duration::from_millis(300);
    let mut scratch = [0u8; 4096];
    while Instant::now() < drain_until {
        port.set_timeout(Duration::from_millis(50)).map_err(|e| super::sp_err(&e))?;
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
    // The firmware emits unsolicited `kalico_status` frames on channel 1 from
    // boot, interleaved with our klipper-protocol identify response. The
    // demuxer separates those out so the slow O(n²) byte-by-byte resync
    // inside `extract_packet` never sees them.
    let mut demuxer = Demuxer::new();

    loop {
        // Encode `identify offset=N count=40` by hand (no dict yet).
        // Hardcoded msgids per klippy/msgproto.py:11-12 and the firmware's
        // baked-in command table:
        //   identify offset=%u count=%c           → msgid 1 (host→fw)
        //   identify_response offset=%u data=%.*s → msgid 0 (fw→host)
        // (Easy to flip — and a previous version of this code did, leading
        // to silent identify timeouts because the firmware never sees a
        // valid command id.)
        let mut payload = Vec::with_capacity(16);
        payload.push(1u8); // msgid=1 → identify request
        encode_vlq(&mut payload, identify_data.len() as i64)
            .expect("identify offset is u32-range");
        encode_vlq(&mut payload, i64::from(IDENTIFY_CHUNK))
            .expect("identify count is u32-range");

        let frame = build_frame(&payload, seq);
        port.write_all(&frame).map_err(TransportError::Io)?;
        port.flush().map_err(TransportError::Io)?;
        seq = seq.wrapping_add(1) & MESSAGE_SEQ_MASK;

        let attempt_deadline = deadline.min(Instant::now() + Duration::from_millis(150));
        let resp = wait_for_identify_response(
            port,
            &mut rx_buf,
            &mut demuxer,
            attempt_deadline,
        )?
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

    let raw_identify_bytes = identify_data.clone();
    let parser = build_parser_from_identify(&identify_data)?;
    Ok((parser, raw_identify_bytes, seq, rx_buf))
}

fn wait_for_identify_response(
    port: &mut Box<dyn SerialPort>,
    rx_buf: &mut Vec<u8>,
    demuxer: &mut Demuxer,
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
        port.set_timeout(read_to).map_err(|e| super::sp_err(&e))?;
        match port.read(&mut scratch) {
            Ok(n) if n > 0 => {
                // Keep raw bytes for the reactor to consume after identify
                // completes — its own demuxer re-processes them from scratch.
                rx_buf.extend_from_slice(&scratch[..n]);
                // Run our own local demuxer so kalico-protocol frames don't
                // confuse the legacy klipper packet parser. KalicoFrame and
                // StreamError outputs are dropped here; only KlipperFrame
                // outputs are checked for the identify_response.
                for out in demuxer.feed_slice(&scratch[..n]) {
                    if let DemuxOutput::KlipperFrame(packet) = out {
                        if let Some(params) = decode_identify_response(&packet) {
                            return Ok(Some(params));
                        }
                    }
                }
            }
            Ok(_) => {}
            Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
            Err(e) => return Err(TransportError::Io(e)),
        }
    }
}

fn build_parser_from_identify(identify_data: &[u8]) -> Result<MsgProtoParser, TransportError> {
    // Klipper firmware compresses the data dictionary with zlib. Spec §4.1
    // mandates a hard error on any parse failure — silently degrading to an
    // empty parser would cascade-fail every subsequent decode with
    // UnknownMsgid, hiding the root cause.
    let json_bytes = if identify_data.first() == Some(&0x78) {
        let mut decoder = flate2::read::ZlibDecoder::new(identify_data);
        let mut out = Vec::new();
        std::io::Read::read_to_end(&mut decoder, &mut out)
            .map_err(|e| TransportError::Parse(format!("identify blob zlib inflate failed: {e}")))?;
        out
    } else {
        identify_data.to_vec()
    };

    let json_str = std::str::from_utf8(&json_bytes)
        .map_err(|e| TransportError::Parse(format!("identify blob is not valid UTF-8: {e}")))?;

    let dict: DataDictionary = serde_json::from_str(json_str)
        .map_err(|e| TransportError::Parse(format!("identify JSON parse failed: {e}")))?;

    MsgProtoParser::from_dictionary(dict)
        .map_err(|e| TransportError::Parse(format!("MsgProtoParser::from_dictionary failed: {e:?}")))
}

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
        return None;
    }
    let (offset, n) = decode_vlq(&body[pos..]).ok()?;
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
