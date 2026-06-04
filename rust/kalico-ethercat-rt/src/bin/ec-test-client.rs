//! Sends one gentle there-and-back move to a running kalico-ethercat-rt endpoint.
//!
//! Usage: ec-test-client [--socket PATH] [--mm F] [--secs F]
//!
//! The client builds two Bernstein ease pieces (0→mm over secs, mm→0 over secs)
//! and sends them as a single `PushPieces` message on `KALICO_CHANNEL_PIECES`.
//! Time domain: both client and endpoint read `CLOCK_MONOTONIC` (nanoseconds)
//! via `kalico_ethercat_rt::clock::monotonic_ns`. A 150 ms lead ensures the
//! pieces arrive, arm, and pre-roll before play begins.

use std::io::{Read, Write};
use std::os::unix::net::UnixStream;
use std::time::{Duration, Instant};

use kalico_ethercat_rt::clock::monotonic_ns;
use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::frame::CHANNEL_CONTROL;
use kalico_native_transport::wire_helpers::decode_message_header;
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{MessageKind, PushPieces, PushPiecesResponse};
use kalico_protocol::KALICO_CHANNEL_PIECES;

fn arg_val(args: &[String], key: &str) -> Option<String> {
    args.iter()
        .position(|a| a == key)
        .and_then(|i| args.get(i + 1).cloned())
}

/// Append a 32-byte `PieceEntry` in wire format to `out`.
///
/// `bp` = Bernstein control points [b0, b1, b2, b3], `dur` = duration in seconds,
/// `start_time_ns` = CLOCK_MONOTONIC nanoseconds for piece start.
fn append_piece(bp: [f32; 4], dur: f32, start_time_ns: u64, out: &mut Vec<u8>) {
    out.extend_from_slice(&start_time_ns.to_le_bytes());
    for x in bp {
        out.extend_from_slice(&x.to_bits().to_le_bytes());
    }
    out.extend_from_slice(&dur.to_bits().to_le_bytes());
    out.extend_from_slice(&0u32.to_le_bytes()); // _reserved
}

/// Build a `PushPieces` frame on `KALICO_CHANNEL_PIECES`.
fn push_pieces_frame(cid: u32, pieces_bytes: Vec<u8>, piece_count: u8, new_head: u32) -> Vec<u8> {
    use kalico_native_transport::frame::encode_frame;
    use kalico_native_transport::wire_helpers::{encode_message_header, MESSAGE_VERSION_DEFAULT};

    let msg = PushPieces {
        axis_idx: 0,
        piece_count,
        start_slot: 0,
        new_head,
        pieces_bytes,
    };
    let mut body = Vec::new();
    msg.encode(&mut body);
    let mut payload =
        encode_message_header(MessageKind::PushPieces, MESSAGE_VERSION_DEFAULT, cid).to_vec();
    payload.extend_from_slice(&body);
    encode_frame(KALICO_CHANNEL_PIECES, &payload)
}

/// Read bytes from `stream` into a `Demuxer` until we get a `PushPiecesResponse`
/// or the deadline passes.
fn read_push_pieces_response(
    stream: &mut UnixStream,
    deadline: Instant,
) -> Option<PushPiecesResponse> {
    let mut demux = Demuxer::new();
    let mut buf = [0u8; 1024];
    loop {
        if Instant::now() >= deadline {
            eprintln!("client: timed out waiting for PushPiecesResponse");
            return None;
        }
        match stream.read(&mut buf) {
            Ok(0) => {
                eprintln!("client: server closed connection");
                return None;
            }
            Ok(n) => {
                let (frames, errs) = demux.feed_slice(&buf[..n]);
                for e in &errs {
                    eprintln!("client: demux error: {e:?}");
                }
                for f in frames {
                    if let Frame::Kalico { channel, payload } = f {
                        if channel != CHANNEL_CONTROL {
                            continue;
                        }
                        let Some((hdr, body)) = decode_message_header(&payload) else {
                            continue;
                        };
                        if MessageKind::from_u16(hdr.kind_raw)
                            == Some(MessageKind::PushPiecesResponse)
                        {
                            match PushPiecesResponse::decode(body) {
                                Ok(resp) => {
                                    eprintln!(
                                        "client: PushPiecesResponse result={} \
                                         arrival_clock={} front_start_time={}",
                                        resp.result, resp.arrival_clock, resp.front_start_time
                                    );
                                    return Some(resp);
                                }
                                Err(e) => {
                                    eprintln!("client: failed to decode PushPiecesResponse: {e}");
                                    return None;
                                }
                            }
                        }
                    }
                }
            }
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                // No bytes yet — loop back and check deadline.
            }
            Err(e) => {
                eprintln!("client: read error: {e}");
                return None;
            }
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let socket = arg_val(&args, "--socket").unwrap_or_else(|| "/tmp/kalico-ethercat.sock".into());
    let mm: f32 = arg_val(&args, "--mm")
        .and_then(|s| s.parse().ok())
        .unwrap_or(20.0);
    let secs: f32 = arg_val(&args, "--secs")
        .and_then(|s| s.parse().ok())
        .unwrap_or(2.0);

    let mut stream = UnixStream::connect(&socket).expect("connect");

    // Stamp pieces on the shared CLOCK_MONOTONIC timeline.
    // 150 ms lead so they arrive, arm, and pre-roll before play begins.
    const LEAD_NS: u64 = 150_000_000;
    let t0_ns: u64 = monotonic_ns() + LEAD_NS;
    let piece_dur_ns: u64 = (secs * 1_000_000_000.0) as u64;

    // Piece 0: ease 0→mm over secs. Bernstein [0,0,mm,mm].
    // Piece 1: ease mm→0 over secs. Bernstein [mm,mm,0,0].
    let mut pieces_bytes = Vec::with_capacity(2 * 32);
    append_piece([0.0, 0.0, mm, mm], secs, t0_ns, &mut pieces_bytes);
    append_piece(
        [mm, mm, 0.0, 0.0],
        secs,
        t0_ns + piece_dur_ns,
        &mut pieces_bytes,
    );

    // Send PushPieces on KALICO_CHANNEL_PIECES (channel 0x02).
    let frame = push_pieces_frame(1, pieces_bytes, 2, 2);
    stream.write_all(&frame).expect("write PushPieces");
    eprintln!("client: sent PushPieces (axis=0, pieces=2, mm={mm}, secs={secs})");

    // Wait for the PushPiecesResponse.
    stream
        .set_read_timeout(Some(Duration::from_millis(500)))
        .expect("set_read_timeout");
    let resp_deadline = Instant::now() + Duration::from_millis(500);
    let _resp = read_push_pieces_response(&mut stream, resp_deadline);

    // Drain any remaining bytes (e.g. StatusHeartbeat events) for ~500 ms.
    let mut buf = [0u8; 1024];
    let drain_deadline = Instant::now() + Duration::from_millis(500);
    while Instant::now() < drain_deadline {
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => eprintln!("client: {n} trailing bytes"),
            Err(_) => break,
        }
    }
    eprintln!("client: done");
}
