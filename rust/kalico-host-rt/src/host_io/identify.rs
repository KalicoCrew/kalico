//! Synchronous identify handshake — extracts the firmware data-dictionary
//! so we can build a [`MsgProtoParser`] before the reactor starts.

use std::time::{Duration, Instant};

use crate::host_io::parser::{DataDictionary, MsgProtoParser, decode_vlq, encode_vlq};
use crate::host_io::serial_frame_io::SerialFrameIo;
use crate::host_io::wire;
use crate::host_io::wire::{
    MESSAGE_HEADER_SIZE, MESSAGE_MIN, MESSAGE_SEQ_MASK, MESSAGE_SYNC, MESSAGE_TRAILER_SIZE,
    build_frame,
};
use crate::transport::{MessageParams, MessageValue, TransportError};
use kalico_native_transport::demux::{Frame, PollOutcome};

/// Sequence-state snapshot returned by identify, adopted by the reactor.
/// See spec §3.1, §3.3.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IdentifySeqState {
    /// Next absolute send-seq the reactor should use for its first
    /// outbound frame after identify completes.
    pub next_send_seq_abs: u64,
    /// Absolute receive-seq adopted from the seq nibble of the last
    /// validated Klipper frame seen during identify (walked across
    /// all responses via wire::decode_absolute).
    pub mcu_receive_seq_abs: u64,
}

const IDENTIFY_CHUNK: u32 = 40;

/// Synchronous identify handshake.
///
/// Drains stale RX bytes, then issues `identify offset=N count=40` until
/// the response data terminates with an empty chunk. Returns `(parser,
/// raw_identify_bytes, seq)` on success. `raw_identify_bytes` is the raw
/// (zlib-compressed or plain) blob as received from the firmware — suitable
/// for passing to klippy's `msgproto.MessageParser.process_identify`. `seq`
/// is the next sequence number to use on the wire (wired into the reactor as
/// `IdentifySeqState` in commit 2).
pub fn identify_handshake(
    io: &mut SerialFrameIo,
    timeout: Duration,
) -> Result<(MsgProtoParser, Vec<u8>, IdentifySeqState), TransportError> {
    let deadline = Instant::now() + timeout;

    // Demuxer-flush phase. Between USB enumeration and the moment our caller
    // applied raw-mode termios (cfmakeraw inside `serialport::open`), the
    // Linux TTY layer ran with default cooked-mode settings — including
    // `ECHO=on`. That echoes every byte the firmware emits (our periodic
    // kalico-native StatusEvent frames) right back to the device as bulk-OUT
    // data. The firmware's demuxer is byte-stateful: a leading `0x55` puts
    // it into `DEMUX_S_KALICO`, where it sits accumulating bytes until a
    // length-determined frame size is reached and CRC-validated. If raw mode
    // takes effect mid-StatusEvent-echo, the demuxer is left holding a
    // partial frame and waits for more bytes. The first identify request we
    // send then gets consumed as the *tail* of that corrupt kalico frame
    // instead of being recognized as a fresh Klipper frame — identify times
    // out without ever reaching command_find_and_dispatch.
    //
    // Fix: write 70 bytes of `0x7E` (Klipper's interframe sync byte) before
    // the drain. After at most 63 bytes any partial frame in either state
    // overflows its known length and falls through to validation (which
    // fails on the all-`0x7E` payload), and the demuxer resets to WAITING.
    // Subsequent `0x7E` bytes are tolerated in WAITING. The flush is
    // idempotent on a clean demuxer, so it costs us nothing in the common
    // case and rescues us in the racy hot-plug case.
    io.write_all(&[MESSAGE_SYNC; 70])?;
    io.flush()?;

    // Drain phase: poll for ~300ms and discard everything (frames + errors).
    // The firmware emits unsolicited `kalico_status` frames on channel 1 from
    // boot, plus any pre-existing klipper output and any frames our flush
    // bytes triggered (e.g. a stale partial-frame's CRC-fail dispatch).
    let drain_until = Instant::now() + Duration::from_millis(300);
    while Instant::now() < drain_until {
        match io.poll_frames_until(drain_until)? {
            PollOutcome::Frames { .. } => {}
            PollOutcome::Timeout | PollOutcome::PhantomZero => break,
        }
    }

    // Absolute seq counters per spec §4.2. `next_send_seq_abs` starts at 1
    // to match the reactor's pre-refactor default; `mcu_recv_abs` starts at
    // 0 and walks via wire::decode_absolute on every validated Klipper frame.
    let mut next_send_seq_abs: u64 = 1;
    let mut mcu_recv_abs: u64 = 0;
    let mut identify_data: Vec<u8> = Vec::new();

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

        let wire_seq = (next_send_seq_abs as u8) & MESSAGE_SEQ_MASK;
        let frame = build_frame(&payload, wire_seq);
        io.write_all(&frame)?;
        io.flush()?;
        next_send_seq_abs += 1;

        let attempt_deadline = deadline.min(Instant::now() + Duration::from_millis(150));
        let resp = wait_for_identify_response(io, attempt_deadline, &mut mcu_recv_abs)?
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
    Ok((
        parser,
        raw_identify_bytes,
        IdentifySeqState {
            next_send_seq_abs,
            mcu_receive_seq_abs: mcu_recv_abs,
        },
    ))
}

fn wait_for_identify_response(
    io: &mut SerialFrameIo,
    deadline: Instant,
    mcu_recv_abs: &mut u64,
) -> Result<Option<MessageParams>, TransportError> {
    loop {
        let now = Instant::now();
        if now >= deadline {
            return Ok(None);
        }
        match io.poll_frames_until(deadline)? {
            PollOutcome::Frames { frames, errors } => {
                for e in errors {
                    log::warn!("identify stream error: {e}");
                }
                for f in frames {
                    if let Frame::Klipper(kf) = f {
                        // Walk mcu_recv_abs for *every* validated Klipper
                        // frame, before attempting to decode it as an
                        // identify_response. Stray frames during identify
                        // (e.g. residual responses to drained-but-still-
                        // in-flight requests) still advance the absolute
                        // seq, pinning consistency with the wire (spec §4.2).
                        *mcu_recv_abs = wire::decode_absolute(
                            *mcu_recv_abs,
                            kf.seq_byte() & MESSAGE_SEQ_MASK,
                        );
                        if let Some(params) = decode_identify_response(kf.bytes()) {
                            return Ok(Some(params));
                        }
                    }
                    // Kalico-native frames that arrive during identify are
                    // discarded — the reactor's `kalico_state` is not yet
                    // initialized to receive them.
                }
            }
            PollOutcome::Timeout | PollOutcome::PhantomZero => {
                // Loop back, re-checking the deadline. PhantomZero during
                // identify is treated as a benign idle tick; SerialFrameIo's
                // debounce semantics belong to the reactor.
            }
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
