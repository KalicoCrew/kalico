use std::io::{Read, Write};
use std::os::unix::net::UnixStream;
use std::thread;
use std::time::Duration;

use kalico_ethercat_rt::curves::{AxisRing, AXIS_RING_CAPACITY, NUM_AXES};
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::wire::{
    identify_response_frame, push_pieces_response_frame, runtime_caps_response_frame,
    status_heartbeat_frame, Command,
};
use kalico_host_rt::native_call::NativeCall;
use kalico_native_transport::demux::{Demuxer, Frame};
use kalico_native_transport::frame::{CHANNEL_CONTROL, CHANNEL_EVENTS};
use kalico_native_transport::wire_helpers::{
    decode_message_header, encode_message_header, MESSAGE_VERSION_DEFAULT,
};
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{
    MessageKind, PushPieces, PushPiecesResponse, RuntimeCapsResponse, StatusHeartbeat,
};
use kalico_protocol::KALICO_CHANNEL_PIECES;
use runtime::piece_ring::PieceEntry;

fn encode_frame(channel: u8, payload: &[u8]) -> Vec<u8> {
    kalico_native_transport::frame::encode_frame(channel, payload)
}

fn push_pieces_wire_frame(cid: u32, axis: u8, pieces: &[PieceEntry], new_head: u32) -> Vec<u8> {
    let mut pieces_bytes = Vec::with_capacity(pieces.len() * 32);
    for p in pieces {
        pieces_bytes.extend_from_slice(&p.to_le_bytes());
    }
    let msg = PushPieces {
        axis_idx: axis,
        piece_count: pieces.len() as u8,
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

fn drain_frames(
    stream: &mut UnixStream,
    demux: &mut Demuxer,
) -> (Vec<PushPiecesResponse>, Vec<StatusHeartbeat>) {
    let mut ppr = Vec::new();
    let mut hbs = Vec::new();
    let mut buf = [0u8; 4096];
    loop {
        match stream.read(&mut buf) {
            Ok(0) => break,
            Ok(n) => {
                let (frames, _) = demux.feed_slice(&buf[..n]);
                for f in frames {
                    if let Frame::Kalico { channel, payload } = f {
                        let Some((hdr, body)) = decode_message_header(&payload) else {
                            continue;
                        };
                        match MessageKind::from_u16(hdr.kind_raw) {
                            Some(MessageKind::PushPiecesResponse) if channel == CHANNEL_CONTROL => {
                                if let Ok(r) = PushPiecesResponse::decode(body) {
                                    ppr.push(r);
                                }
                            }
                            Some(MessageKind::StatusHeartbeat) if channel == CHANNEL_EVENTS => {
                                if let Ok(hb) = StatusHeartbeat::decode(body) {
                                    hbs.push(hb);
                                }
                            }
                            _ => {}
                        }
                    }
                }
            }
            Err(ref e)
                if e.kind() == std::io::ErrorKind::WouldBlock
                    || e.kind() == std::io::ErrorKind::TimedOut =>
            {
                break;
            }
            Err(_) => break,
        }
    }
    (ppr, hbs)
}

#[test]
fn push_pieces_and_heartbeat_closes_the_loop() {
    let socket_path = format!("/tmp/kalico-ethercat-test-{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket_path);

    const N: usize = 3;
    const PIECE_DUR_NS: u64 = 1_000_000;
    const BASE_NS: u64 = 10_000_000_000;

    let pieces: Vec<PieceEntry> = (0..N)
        .map(|i| PieceEntry {
            start_time: BASE_NS + i as u64 * PIECE_DUR_NS,
            coeffs: [0.0_f32; 4],
            duration: PIECE_DUR_NS as f32 / 1_000_000_000.0,
            _reserved: 0,
        })
        .collect();

    let (tx, rx) = std::sync::mpsc::channel::<u32>();

    let socket_path_sv = socket_path.clone();
    let server_thread = thread::spawn(move || {
        let mut server = FrameServer::bind(&socket_path_sv).expect("bind server socket");
        let mut ring = AxisRing::new();
        let mut last_sent_retired: u32 = 0;
        let mut heartbeat_sent = false;

        let cmd_deadline = std::time::Instant::now() + Duration::from_secs(2);
        let mut push_received = false;
        while !push_received {
            assert!(
                std::time::Instant::now() < cmd_deadline,
                "command phase timed out waiting for PushPieces"
            );
            for cmd in server.poll_commands() {
                match cmd {
                    Command::Identify {
                        correlation_id,
                        proto_version,
                    } => {
                        server.respond(&identify_response_frame(correlation_id, proto_version));
                    }
                    Command::PushPieces {
                        correlation_id,
                        msg,
                    } => {
                        let front_start_time = if msg.piece_count > 0 && msg.pieces_bytes.len() >= 8
                        {
                            u64::from_le_bytes(msg.pieces_bytes[0..8].try_into().unwrap_or([0; 8]))
                        } else {
                            0
                        };
                        let pushed = ring.push_from_bytes(msg.piece_count, &msg.pieces_bytes);
                        let result = if pushed == msg.piece_count {
                            0i32
                        } else {
                            -309
                        };
                        server.respond(&push_pieces_response_frame(
                            correlation_id,
                            result,
                            BASE_NS,
                            front_start_time,
                        ));
                        push_received = true;
                    }
                    Command::QueryRuntimeCaps { correlation_id } => {
                        let total = (AXIS_RING_CAPACITY * NUM_AXES * 32) as u32;
                        server.respond(&runtime_caps_response_frame(correlation_id, total));
                    }
                    Command::ClaimHandshake { .. } => {}
                    Command::SetTorque { .. } => {}
                    Command::StartCapture { .. } => {}
                    Command::StopCapture { .. } => {}
                    Command::Stop { .. } => {}
                    Command::ResumeStream { .. } => {}
                    Command::SetDriveLimits { .. } | Command::RestoreDriveLimits { .. } => {}
                    Command::SdoRead { .. } | Command::SdoWrite { .. } => {
                        todo!("wired in the endpoint task")
                    }
                    Command::Unknown { .. } => {}
                }
            }
            if !push_received {
                thread::sleep(Duration::from_millis(1));
            }
        }

        let mut now: u64 = BASE_NS;
        let total_cycles = (N as u64) + 4;
        for _ in 0..total_cycles {
            for cmd in server.poll_commands() {
                match cmd {
                    Command::Unknown { .. }
                    | Command::Identify { .. }
                    | Command::QueryRuntimeCaps { .. }
                    | Command::ClaimHandshake { .. }
                    | Command::SetTorque { .. }
                    | Command::StartCapture { .. }
                    | Command::StopCapture { .. }
                    | Command::Stop { .. }
                    | Command::ResumeStream { .. }
                    | Command::SetDriveLimits { .. }
                    | Command::RestoreDriveLimits { .. }
                    | Command::SdoRead { .. }
                    | Command::SdoWrite { .. }
                    | Command::PushPieces { .. } => {}
                }
            }

            let _ = ring.sample(now);

            let current_retired = ring.retired_count();
            let should_emit = !heartbeat_sent || current_retired != last_sent_retired;
            if should_emit {
                let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
                server.respond(&status_heartbeat_frame(
                    engine_state,
                    0,
                    &[current_retired],
                    0,
                ));
                last_sent_retired = current_retired;
                heartbeat_sent = true;
            }

            now = now.saturating_add(PIECE_DUR_NS);

            thread::sleep(Duration::from_millis(1));
        }

        let _ = tx.send(ring.retired_count());
    });

    let wait_deadline = std::time::Instant::now() + Duration::from_millis(500);
    loop {
        if std::path::Path::new(&socket_path).exists() {
            break;
        }
        assert!(
            std::time::Instant::now() < wait_deadline,
            "stub socket did not appear within 500 ms"
        );
        thread::sleep(Duration::from_millis(5));
    }

    let mut client = UnixStream::connect(&socket_path).expect("connect client");
    client.set_nonblocking(true).expect("set_nonblocking");

    let frame = push_pieces_wire_frame(1, 0, &pieces, N as u32);
    client.write_all(&frame).expect("write PushPieces");

    let mut demux = Demuxer::new();
    let mut got_response = false;
    let mut final_retired = 0u32;
    let deadline = std::time::Instant::now() + Duration::from_secs(3);

    loop {
        if std::time::Instant::now() >= deadline {
            panic!(
                "loop closed? got_response={got_response} \
                 final_retired={final_retired} (expected {N}) — \
                 did not converge within deadline"
            );
        }

        let (pprs, hbs) = drain_frames(&mut client, &mut demux);

        for r in &pprs {
            assert_eq!(
                r.result, 0,
                "stub must accept the PushPieces frame (result=0), got {}",
                r.result
            );
            got_response = true;
        }

        for hb in &hbs {
            if let Some(&r) = hb.retired_counts.first() {
                final_retired = final_retired.max(r);
            }
        }

        if got_response && final_retired == N as u32 {
            break;
        }

        thread::sleep(Duration::from_millis(2));
    }

    assert!(got_response, "PushPiecesResponse must have been received");
    assert_eq!(
        final_retired, N as u32,
        "StatusHeartbeat.retired_counts[0] must equal pieces pushed ({})",
        N
    );

    drop(client);
    let final_count = rx
        .recv_timeout(Duration::from_secs(2))
        .expect("server thread did not report retired count");
    assert_eq!(
        final_count, N as u32,
        "server-side retired count must equal N"
    );
    let _ = server_thread.join();
    let _ = std::fs::remove_file(&socket_path);
}

#[test]
fn piece_start_in_past_faults_and_returns_none() {
    let mut ring = AxisRing::new();

    let now_ns: u64 = 10_000_000_000;
    let stale_start = now_ns - 1_000_000_000;

    ring.push_entry(PieceEntry {
        start_time: stale_start,
        coeffs: [0.0_f32; 4],
        duration: 0.001,
        _reserved: 0,
    })
    .unwrap();

    let result = ring.sample(now_ns);
    assert!(
        result.is_none(),
        "expected None (PieceStartInPast fault) for a piece starting 1 s before now, \
         got {result:?}"
    );
}

#[test]
fn ethercat_endpoint_query_runtime_caps_round_trip() {
    use kalico_host_rt::unix_native_conn::UnixNativeConn;

    let socket_path = format!("/tmp/kalico-caps-rt-{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket_path);

    const PIECE_ENTRY_SIZE: usize = 32;
    let expected_total: u32 = (AXIS_RING_CAPACITY * NUM_AXES * PIECE_ENTRY_SIZE) as u32;

    {
        let sp = socket_path.clone();
        thread::spawn(move || {
            let mut server = FrameServer::bind(&sp).expect("endpoint: bind");
            let deadline = std::time::Instant::now() + Duration::from_secs(5);
            loop {
                assert!(
                    std::time::Instant::now() < deadline,
                    "endpoint: timed out waiting for QueryRuntimeCaps"
                );
                for cmd in server.poll_commands() {
                    if let Command::QueryRuntimeCaps { correlation_id } = cmd {
                        let total = (AXIS_RING_CAPACITY * NUM_AXES * PIECE_ENTRY_SIZE) as u32;
                        server.respond(&runtime_caps_response_frame(correlation_id, total));
                        return;
                    }
                    if let Command::Identify {
                        correlation_id,
                        proto_version,
                    } = cmd
                    {
                        server.respond(&identify_response_frame(correlation_id, proto_version));
                    }
                }
                thread::sleep(Duration::from_millis(1));
            }
        });
    }

    {
        let wait_deadline = std::time::Instant::now() + Duration::from_millis(500);
        loop {
            if std::path::Path::new(&socket_path).exists() {
                break;
            }
            assert!(
                std::time::Instant::now() < wait_deadline,
                "endpoint socket did not appear within 500 ms"
            );
            thread::sleep(Duration::from_millis(5));
        }
    }

    let conn = UnixNativeConn::connect(&socket_path).expect("UnixNativeConn::connect must succeed");

    let (resp_kind, resp_body) = conn
        .kalico_call(
            MessageKind::QueryRuntimeCaps,
            vec![],
            Duration::from_secs(5),
        )
        .expect("QueryRuntimeCaps kalico_call must succeed");

    assert_eq!(
        resp_kind,
        MessageKind::RuntimeCapsResponse,
        "response kind must be RuntimeCapsResponse"
    );

    let caps = RuntimeCapsResponse::decode(&resp_body)
        .expect("RuntimeCapsResponse must decode from response body");

    assert_eq!(
        caps.total_piece_memory, expected_total,
        "total_piece_memory must be AXIS_RING_CAPACITY({AXIS_RING_CAPACITY}) \
         * NUM_AXES({NUM_AXES}) * 32 = {expected_total}, \
         got {}",
        caps.total_piece_memory,
    );

    let total_pieces = caps.total_piece_memory / PIECE_ENTRY_SIZE as u32;
    let ring_depth = if NUM_AXES as u32 == 0 {
        total_pieces
    } else {
        (total_pieces / NUM_AXES as u32).max(1)
    };

    assert_eq!(
        ring_depth, AXIS_RING_CAPACITY as u32,
        "derived ring_depth must equal AXIS_RING_CAPACITY ({AXIS_RING_CAPACITY}), \
         got {ring_depth}"
    );

    let _ = std::fs::remove_file(&socket_path);
}
