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
///
/// # NAK / ACK / stale-ACK classification
///
/// Mirrors `tools/kalico_host_io.py::_handle_identify_sync_packet`'s
/// three-way classification of empty-body (MESSAGE_MIN-length) Klipper
/// frames received during identify. Every MCU→host frame carries the
/// firmware's current `next_sequence` in its seq nibble (command.c:298 for
/// NAK, command.c after dispatch for ACK). Treat them differently:
///
/// 1. **Bare ACK** — `wire_seq == (sent_seq + 1) & 0xF`. Firmware accepted
///    our send and advanced. Walk `next_send_seq_abs` forward (subsequent
///    sends should use the new value). Do NOT flag as NAK — the response
///    we want is still en route.
/// 2. **NAK** — `wire_seq != sent_seq` AND `wire_seq != (sent_seq + 1) & 0xF`.
///    Firmware rejected (seq mismatch on the host message) and is telling
///    us its current expectation. Walk `next_send_seq_abs` and flag the
///    attempt as having seen a NAK so the caller knows it must retry.
/// 3. **Stale ACK** — `wire_seq == sent_seq`. Leftover frame from a
///    previous host send (typical reconnect scenario: ACK for sent_seq-1
///    that the kernel buffered before we opened the port). **Ignore
///    entirely** — accepting it rewinds our counter at the 15→0 wrap and
///    pins us behind firmware's actual next_sequence forever.
///
/// The previous implementation walked `mcu_recv_abs` on every validated
/// Klipper frame regardless of seq classification, which under Renode's
/// 1µs-quantum pacing left 10+ in-flight identify frames advancing
/// firmware's `next_sequence` faster than the resync loop could catch up
/// — every retransmit chased a moving target until the per-chunk attempt
/// cap was hit.
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

    // Absolute seq counters per spec §4.2. The firmware's `next_sequence`
    // (command.c:16) starts at `MESSAGE_DEST | 0x0` on boot but advances
    // every time it dispatches a valid host message. If we open the device
    // long after boot — or after an earlier failed bridge attempt that
    // managed to land a few valid frames — firmware's `next_sequence` is
    // ahead of zero. The firmware's NAK path (command.c:298) does NOT
    // advance `next_sequence`, but it DOES emit an empty-body response
    // frame whose own seq byte echoes its current `next_sequence`. We use
    // that to resync.
    //
    // We start from 0 and let the resync logic take over on the first
    // received frame. mcu_recv_abs is the absolute counter walked via
    // wire::decode_absolute on NAK frames + valid identify_response frames.
    let mut next_send_seq_abs: u64 = 0;
    let mut mcu_recv_abs: u64 = 0;
    let mut identify_data: Vec<u8> = Vec::new();

    let chunk_deadline_per_attempt = Duration::from_millis(200);
    // Total NAK-resync attempts across the entire identify handshake.
    // Matches Python's `resync_attempts < 20` cap.
    const MAX_TOTAL_RESYNC_ATTEMPTS: usize = 20;
    let mut resync_attempts: usize = 0;

    'outer: loop {
        let offset = identify_data.len();
        // `sent_seq` is the wire nibble of the most-recent identify request
        // we put on the bus for THIS chunk offset. `None` means we haven't
        // sent for this chunk yet (or we've been told to reset and pick a
        // fresh seq from `next_send_seq_abs`). When `Some`, retransmits
        // use the SAME wire seq — firmware's NAK path matches against this.
        let mut sent_seq: Option<u8> = None;
        let mut retransmitted_same_seq = false;

        // Per-chunk attempt loop. Mirrors Python's `while ... < deadline`
        // inner loop. We exit on success (got the right response), or on
        // a give-up condition (offset>0 + already retransmitted, or
        // exhausted the global resync cap).
        loop {
            if Instant::now() >= deadline {
                return Err(TransportError::Parse(
                    "identify timed out (no firmware response)".into(),
                ));
            }

            // Issue (or re-issue) the request.
            //
            // First send of a chunk picks the wire seq from
            // `next_send_seq_abs` and records it; subsequent retransmits
            // re-use the recorded `sent_seq`.
            let wire_seq = match sent_seq {
                Some(s) => s,
                None => {
                    let s = (next_send_seq_abs as u8) & MESSAGE_SEQ_MASK;
                    sent_seq = Some(s);
                    s
                }
            };
            let mut payload = Vec::with_capacity(16);
            payload.push(1u8); // msgid=1 → identify request
            encode_vlq(&mut payload, offset as i64)
                .expect("identify offset is u32-range");
            encode_vlq(&mut payload, i64::from(IDENTIFY_CHUNK))
                .expect("identify count is u32-range");
            let frame = build_frame(&payload, wire_seq);
            io.write_all(&frame)?;
            io.flush()?;

            let attempt_deadline = deadline.min(Instant::now() + chunk_deadline_per_attempt);
            let mut last_wait_saw_nak = false;
            let outcome = wait_for_klipper_frame(
                io,
                attempt_deadline,
                wire_seq,
                &mut next_send_seq_abs,
                &mut mcu_recv_abs,
                &mut last_wait_saw_nak,
            )?;

            match outcome {
                WaitOutcome::Response(params) => {
                    let resp_offset = params.get_u32("offset") as usize;
                    if resp_offset != offset {
                        // Stale response from a prior in-flight request —
                        // ignore the data, but our seq has already been
                        // walked via the response's seq byte. Retry this
                        // chunk on the next iteration of the outer loop.
                        continue 'outer;
                    }
                    let chunk = params
                        .get_bytes("data")
                        .map(<[u8]>::to_vec)
                        .unwrap_or_default();
                    if chunk.is_empty() {
                        break 'outer;
                    }
                    identify_data.extend_from_slice(&chunk);
                    if Instant::now() >= deadline {
                        return Err(TransportError::Parse(
                            "identify exceeded timeout".into(),
                        ));
                    }
                    continue 'outer;
                }
                WaitOutcome::Timeout => {
                    // Inner-attempt deadline elapsed with no `identify_response`
                    // matching this chunk. Branch on what we observed.
                    if offset == 0
                        && last_wait_saw_nak
                        && resync_attempts < MAX_TOTAL_RESYNC_ATTEMPTS
                    {
                        // Pre-progress NAK resync (Python: offset==0 path).
                        // Drop our recorded sent_seq so the next request
                        // picks a fresh wire seq from the (newly-walked)
                        // next_send_seq_abs.
                        sent_seq = None;
                        retransmitted_same_seq = false;
                        resync_attempts += 1;
                        continue;
                    }
                    if !retransmitted_same_seq {
                        // Allow exactly ONE retransmit at the same wire seq.
                        // Matches Python's `not retransmitted_same_seq`
                        // branch for offset>0 (and for offset==0 once the
                        // resync cap is hit).
                        retransmitted_same_seq = true;
                        continue;
                    }
                    // Exhausted single-retransmit budget for this chunk.
                    // Python raises HostIoError("Timed out") here; preserve
                    // the previous-impl error message so existing callers /
                    // tests that match on it keep working.
                    return Err(TransportError::Parse(
                        "identify exceeded NAK-resync attempts for one chunk".into(),
                    ));
                }
            }
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

/// What the identify wait loop saw on the wire.
enum WaitOutcome {
    /// A Klipper frame whose body decoded to a real `identify_response`.
    Response(MessageParams),
    /// No `identify_response` arrived within the deadline. Empty-body
    /// (NAK / bare ACK) frames may have been observed mid-wait — the
    /// `last_wait_saw_nak` out-parameter reports whether any of them
    /// classified as a true NAK.
    Timeout,
}

/// Wait for an `identify_response` or for the per-attempt deadline.
///
/// Empty-body Klipper frames are classified against `sent_seq` per the
/// scheme in [`identify_handshake`]'s docstring. The classification
/// updates `next_send_seq_abs` and/or `mcu_recv_abs` as appropriate, and
/// sets `last_wait_saw_nak = true` only on true NAKs (so the caller can
/// decide whether to take the aggressive offset==0 resync branch).
///
/// We continue draining frames after each classification — we want the
/// *real* `identify_response` if it shows up, not the first empty frame.
fn wait_for_klipper_frame(
    io: &mut SerialFrameIo,
    deadline: Instant,
    sent_seq: u8,
    next_send_seq_abs: &mut u64,
    mcu_recv_abs: &mut u64,
    last_wait_saw_nak: &mut bool,
) -> Result<WaitOutcome, TransportError> {
    let sent_nibble = sent_seq & MESSAGE_SEQ_MASK;
    let ack_after_sent = (sent_nibble.wrapping_add(1)) & MESSAGE_SEQ_MASK;
    loop {
        let now = Instant::now();
        if now >= deadline {
            return Ok(WaitOutcome::Timeout);
        }
        match io.poll_frames_until(deadline)? {
            PollOutcome::Frames { frames, errors } => {
                for e in errors {
                    log::warn!("identify stream error: {e}");
                }
                for f in frames {
                    if let Frame::Klipper(kf) = f {
                        let wire_seq = kf.seq_byte() & MESSAGE_SEQ_MASK;
                        let is_empty_body = kf.bytes().len() <= MESSAGE_MIN;
                        if is_empty_body {
                            // Empty-body classification: bare ACK / NAK /
                            // stale ACK. Walk counters per the docstring.
                            if wire_seq == ack_after_sent {
                                // Bare ACK for our send. Advance the send
                                // counter; firmware now expects sent+1.
                                *next_send_seq_abs = wire::decode_absolute(
                                    *next_send_seq_abs,
                                    wire_seq,
                                );
                                *mcu_recv_abs = wire::decode_absolute(
                                    *mcu_recv_abs,
                                    wire_seq,
                                );
                            } else if wire_seq != sent_nibble {
                                // True NAK. Adopt firmware's expectation
                                // and surface the flag for the caller.
                                *next_send_seq_abs = wire::decode_absolute(
                                    *next_send_seq_abs,
                                    wire_seq,
                                );
                                *mcu_recv_abs = wire::decode_absolute(
                                    *mcu_recv_abs,
                                    wire_seq,
                                );
                                *last_wait_saw_nak = true;
                            }
                            // else: stale ACK — wire_seq == sent_nibble.
                            // Ignore entirely (don't walk anything).
                            // Keep waiting for the real response.
                        } else if let Some(params) =
                            decode_identify_response(kf.bytes())
                        {
                            // Valid identify_response. Its seq nibble is
                            // firmware's post-dispatch next_sequence, i.e.
                            // an in-band ACK rolled into the response.
                            *next_send_seq_abs = wire::decode_absolute(
                                *next_send_seq_abs,
                                wire_seq,
                            );
                            *mcu_recv_abs = wire::decode_absolute(
                                *mcu_recv_abs,
                                wire_seq,
                            );
                            return Ok(WaitOutcome::Response(params));
                        } else {
                            // Non-empty Klipper frame that isn't an
                            // identify_response — e.g. an `output(...)`
                            // line from boot. Pre-identify we have no
                            // parser, so we can't do better than walk the
                            // counter (the frame did pass length+CRC) and
                            // drop the body. Matches Python's
                            // `_set_seq(pkt[1] & MASK)` then drop branch.
                            *next_send_seq_abs = wire::decode_absolute(
                                *next_send_seq_abs,
                                wire_seq,
                            );
                            *mcu_recv_abs = wire::decode_absolute(
                                *mcu_recv_abs,
                                wire_seq,
                            );
                        }
                    }
                    // Kalico-native frames during identify: discarded.
                    // The reactor's `kalico_state` isn't initialized yet.
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
