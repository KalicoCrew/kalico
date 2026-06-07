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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IdentifySeqState {
    pub next_send_seq_abs: u64,
    pub mcu_receive_seq_abs: u64,
}

const IDENTIFY_CHUNK: u32 = 40;

pub fn identify_handshake(
    io: &mut SerialFrameIo,
    timeout: Duration,
) -> Result<(MsgProtoParser, Vec<u8>, IdentifySeqState), TransportError> {
    let deadline = Instant::now() + timeout;

    // Write 70 × 0x7E before draining: flushes any partial frame in the MCU
    // demuxer caused by TTY ECHO being on during USB enumeration (cooked mode
    // echoes our own unsolicited frames back to the device, corrupting the
    // demux state machine). 63 bytes is enough to overflow any partial frame;
    // 70 is conservative and idempotent on a clean demuxer.
    io.write_all(&[MESSAGE_SYNC; 70])?;
    io.flush()?;

    let drain_until = Instant::now() + Duration::from_millis(300);
    while Instant::now() < drain_until {
        match io.poll_frames_until(drain_until)? {
            PollOutcome::Frames { .. } => {}
            PollOutcome::Timeout | PollOutcome::PhantomZero => break,
        }
    }

    let mut next_send_seq_abs: u64 = 0;
    let mut mcu_recv_abs: u64 = 0;
    let mut identify_data: Vec<u8> = Vec::new();

    let chunk_deadline_per_attempt = Duration::from_millis(200);
    let max_resync_attempts = 64usize;

    'outer: loop {
        for _attempt in 0..max_resync_attempts {
            let mut payload = Vec::with_capacity(16);
            payload.push(1u8); // msgid=1 = identify request (hardcoded; no dict yet)
            encode_vlq(&mut payload, identify_data.len() as i64)
                .expect("identify offset is u32-range");
            encode_vlq(&mut payload, i64::from(IDENTIFY_CHUNK))
                .expect("identify count is u32-range");

            let wire_seq = (next_send_seq_abs as u8) & MESSAGE_SEQ_MASK;
            let frame = build_frame(&payload, wire_seq);
            io.write_all(&frame)?;
            io.flush()?;

            let attempt_deadline = deadline.min(Instant::now() + chunk_deadline_per_attempt);
            let outcome =
                wait_for_klipper_frame(io, attempt_deadline, &mut mcu_recv_abs, Some(wire_seq))?;

            match outcome {
                IdentifyOutcome::Response(params) => {
                    next_send_seq_abs = mcu_recv_abs;

                    let offset = params.get_u32("offset") as usize;
                    if offset != identify_data.len() {
                        continue;
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
                        return Err(TransportError::Parse("identify exceeded timeout".into()));
                    }
                    continue 'outer;
                }
                IdentifyOutcome::Nak => {
                    next_send_seq_abs = mcu_recv_abs;
                    continue;
                }
                IdentifyOutcome::Timeout => {
                    if Instant::now() >= deadline {
                        return Err(TransportError::Parse(
                            "identify timed out (no firmware response)".into(),
                        ));
                    }
                    continue;
                }
            }
        }
        return Err(TransportError::Parse(
            "identify exceeded NAK-resync attempts for one chunk".into(),
        ));
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

enum IdentifyOutcome {
    Response(MessageParams),
    Nak,
    Timeout,
}

fn wait_for_klipper_frame(
    io: &mut SerialFrameIo,
    deadline: Instant,
    mcu_recv_abs: &mut u64,
    sent_seq_nibble: Option<u8>,
) -> Result<IdentifyOutcome, TransportError> {
    loop {
        let now = Instant::now();
        if now >= deadline {
            return Ok(IdentifyOutcome::Timeout);
        }
        match io.poll_frames_until(deadline)? {
            PollOutcome::Frames { frames, errors } => {
                for e in errors {
                    log::warn!("identify stream error: {e}");
                }
                for f in frames {
                    if let Frame::Klipper(kf) = f {
                        let frame_seq_nibble = kf.seq_byte() & MESSAGE_SEQ_MASK;
                        *mcu_recv_abs = wire::decode_absolute(*mcu_recv_abs, frame_seq_nibble);
                        if let Some(params) = decode_identify_response(kf.bytes()) {
                            return Ok(IdentifyOutcome::Response(params));
                        }
                        // Suppress stale ACKs (seq nibble matches what we just sent):
                        // the real identify_response is right behind in the pipe.
                        // A different nibble is a real NAK — caller adopts mcu_recv_abs.
                        if sent_seq_nibble == Some(frame_seq_nibble) {
                            continue;
                        }
                        return Ok(IdentifyOutcome::Nak);
                    }
                }
            }
            PollOutcome::Timeout | PollOutcome::PhantomZero => {}
        }
    }
}

fn build_parser_from_identify(identify_data: &[u8]) -> Result<MsgProtoParser, TransportError> {
    let json_bytes = if identify_data.first() == Some(&0x78) {
        let mut decoder = flate2::read::ZlibDecoder::new(identify_data);
        let mut out = Vec::new();
        std::io::Read::read_to_end(&mut decoder, &mut out).map_err(|e| {
            TransportError::Parse(format!("identify blob zlib inflate failed: {e}"))
        })?;
        out
    } else {
        identify_data.to_vec()
    };

    let json_str = std::str::from_utf8(&json_bytes)
        .map_err(|e| TransportError::Parse(format!("identify blob is not valid UTF-8: {e}")))?;

    let dict: DataDictionary = serde_json::from_str(json_str)
        .map_err(|e| TransportError::Parse(format!("identify JSON parse failed: {e}")))?;

    MsgProtoParser::from_dictionary(dict).map_err(|e| {
        TransportError::Parse(format!("MsgProtoParser::from_dictionary failed: {e:?}"))
    })
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
