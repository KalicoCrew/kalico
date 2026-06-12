use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use kalico_ethercat_rt::curves::AXIS_RING_CAPACITY;
use kalico_ethercat_rt::curves::NUM_AXES;
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::wire::{
    identify_response_frame, push_pieces_response_frame, runtime_caps_response_frame,
    status_heartbeat_frame, Command,
};
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{MessageKind, PushPieces, PushPiecesResponse};
use kalico_protocol::KALICO_CHANNEL_PIECES;
use runtime::piece_ring::PieceEntry;

const N: usize = AXIS_RING_CAPACITY + 20;
const BATCH: usize = 8;
const PIECE_DUR_NS: u64 = 1_000_000;
const BASE_NS: u64 = 10_000_000_000;
const CALL_TIMEOUT: Duration = Duration::from_secs(10);

fn run_endpoint(socket_path: String, faulted: Arc<AtomicBool>) {
    use kalico_ethercat_rt::curves::AxisRing;

    let mut server = FrameServer::bind(&socket_path).expect("endpoint: bind");
    let mut ring = AxisRing::new();
    let mut total_pushed: usize = 0;
    let mut last_sent_retired: u32 = 0;
    let mut now: u64 = BASE_NS;

    let deadline = std::time::Instant::now() + Duration::from_secs(60);

    loop {
        assert!(
            std::time::Instant::now() < deadline,
            "endpoint: timed out (pushed={total_pushed}/{N}, \
             retired={})",
            ring.retired_count()
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
                    let front_start_time = if msg.piece_count > 0 && msg.pieces_bytes.len() >= 8 {
                        u64::from_le_bytes(msg.pieces_bytes[0..8].try_into().unwrap_or([0u8; 8]))
                    } else {
                        0
                    };
                    let pushed = ring.push_from_bytes(msg.piece_count, &msg.pieces_bytes);
                    let result = if pushed == msg.piece_count {
                        0i32
                    } else {
                        -309i32
                    };
                    server.respond(&push_pieces_response_frame(
                        correlation_id,
                        result,
                        BASE_NS,
                        front_start_time,
                    ));
                    total_pushed += pushed as usize;
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

        if !ring.is_empty() {
            let _pos = ring.sample(now);

            if ring.take_fault().is_some() {
                faulted.store(true, Ordering::SeqCst);
            }

            let current_retired = ring.retired_count();
            if current_retired != last_sent_retired {
                let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
                server.respond(&status_heartbeat_frame(
                    engine_state,
                    0,
                    &[current_retired],
                    0,
                ));
                last_sent_retired = current_retired;
            }

            now = now.saturating_add(PIECE_DUR_NS);
        }

        if total_pushed >= N && ring.retired_count() as usize >= N {
            let engine_state: u8 = 0;
            server.respond(&status_heartbeat_frame(
                engine_state,
                0,
                &[ring.retired_count()],
                0,
            ));
            break;
        }

        thread::sleep(Duration::from_millis(1));
    }
}

fn push_pieces_body(pieces: &[PieceEntry], start: usize, end: usize) -> Vec<u8> {
    let batch = &pieces[start..end];
    let mut bytes = Vec::with_capacity(batch.len() * 32);
    for p in batch {
        bytes.extend_from_slice(&p.to_le_bytes());
    }
    let msg = PushPieces {
        axis_idx: 0,
        piece_count: batch.len() as u8,
        start_slot: 0,
        new_head: end as u32,
        pieces_bytes: bytes,
    };
    let mut body = Vec::new();
    msg.encode(&mut body);
    body
}

#[test]
fn unix_native_conn_and_frame_server_sustain_streaming_past_ring_depth() {
    let socket_path = format!("/tmp/kalico-e2e-stream-{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket_path);

    let pieces: Vec<PieceEntry> = (0..N)
        .map(|i| PieceEntry {
            start_time: BASE_NS + i as u64 * PIECE_DUR_NS,
            coeffs: [0.0_f32; 4],
            duration: PIECE_DUR_NS as f32 / 1_000_000_000.0,
            _reserved: 0,
        })
        .collect();

    let last_retired = Arc::new(AtomicU32::new(0));
    let faulted = Arc::new(AtomicBool::new(false));

    {
        let sp = socket_path.clone();
        let fault_flag = Arc::clone(&faulted);
        thread::spawn(move || run_endpoint(sp, fault_flag));
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

    {
        let lr = Arc::clone(&last_retired);
        conn.attach_heartbeat_callback(Arc::new(
            move |hb: &kalico_protocol::messages::StatusHeartbeat| {
                if let Some(&v) = hb.retired_counts.first() {
                    let mut prev = lr.load(Ordering::Acquire);
                    loop {
                        if v <= prev {
                            break;
                        }
                        match lr.compare_exchange_weak(prev, v, Ordering::AcqRel, Ordering::Acquire)
                        {
                            Ok(_) => break,
                            Err(cur) => prev = cur,
                        }
                    }
                }
            },
        ));
    }

    let mut total_sent: usize = 0;

    let pump_deadline = std::time::Instant::now() + Duration::from_secs(60);
    while total_sent < N {
        assert!(
            std::time::Instant::now() < pump_deadline,
            "client: pump timed out at {total_sent}/{N} pieces \
             (last_retired={})",
            last_retired.load(Ordering::Acquire)
        );

        let retired = last_retired.load(Ordering::Acquire) as usize;
        let occupancy = total_sent.saturating_sub(retired);
        let room = AXIS_RING_CAPACITY.saturating_sub(occupancy);

        if room == 0 {
            thread::sleep(Duration::from_millis(2));
            continue;
        }

        let batch_size = room.min(BATCH).min(N - total_sent);
        let batch_end = total_sent + batch_size;

        let body = push_pieces_body(&pieces, total_sent, batch_end);
        let (resp_kind, resp_body) = conn
            .kalico_call_on_channel(
                KALICO_CHANNEL_PIECES,
                MessageKind::PushPieces,
                body,
                CALL_TIMEOUT,
            )
            .unwrap_or_else(|e| {
                panic!(
                    "client: kalico_call_on_channel failed at \
                     pieces {total_sent}..{batch_end}: {e:?}"
                )
            });

        assert_eq!(
            resp_kind,
            MessageKind::PushPiecesResponse,
            "response kind must be PushPiecesResponse for batch {total_sent}..{batch_end}"
        );
        let resp = PushPiecesResponse::decode(&resp_body).unwrap_or_else(|e| {
            panic!(
                "client: PushPiecesResponse decode failed for \
                 batch {total_sent}..{batch_end}: {e:?}"
            )
        });
        assert_eq!(
            resp.result, 0,
            "PushPiecesResponse.result must be 0 (OK) for batch \
             {total_sent}..{batch_end}"
        );

        total_sent = batch_end;
    }

    let retire_deadline = std::time::Instant::now() + Duration::from_secs(30);
    loop {
        assert!(
            std::time::Instant::now() < retire_deadline,
            "client: retirement stalled — expected {N}, last_retired={}",
            last_retired.load(Ordering::Acquire)
        );

        let retired = last_retired.load(Ordering::Acquire) as usize;
        if retired >= N {
            break;
        }

        thread::sleep(Duration::from_millis(2));
    }

    let final_retired = last_retired.load(Ordering::Acquire) as usize;
    assert_eq!(
        final_retired, N,
        "heartbeat callback must report retired_counts[0] == N ({N}), \
         got {final_retired}"
    );

    assert!(
        !faulted.load(Ordering::SeqCst),
        "endpoint detected a PieceStartInPast fault — contiguous \
         in-synthetic-time delivery must not fault the ring"
    );

    let _ = std::fs::remove_file(&socket_path);
}
