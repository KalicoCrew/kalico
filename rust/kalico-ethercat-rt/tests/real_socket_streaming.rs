//! End-to-end integration test: real `UnixNativeConn` ↔ real `FrameServer`
//! over a Unix socket, streaming PAST the ring depth with synthetic-clock
//! retirement flowing back via the heartbeat callback.
//!
//! ## What this proves
//!
//! `stub_loop.rs` covers the `FrameServer` side using a raw `UnixStream`
//! client that hand-builds wire frames.  THIS test replaces the raw client
//! with the real `UnixNativeConn` — the transport the production motion-bridge
//! uses.  Together they catch wire-framing / channel / correlation-id /
//! heartbeat-encoding mismatches between the two real components before any
//! hardware is involved.
//!
//! ## Streaming past ring depth
//!
//! `N = AXIS_RING_CAPACITY + 20 = 276` pieces.  The ring holds at most
//! `AXIS_RING_CAPACITY = 256` pieces at once.  To push all 276 without
//! overflowing the ring the endpoint must retire older pieces while the
//! client continues sending newer ones.
//!
//! The endpoint therefore runs both command-polling and DC-loop advancement
//! in a single interleaved loop: it polls for `PushPieces` frames and
//! advances the synthetic clock in lock-step.  The DC clock starts at
//! `BASE_NS` and steps forward only when the ring is non-empty, which
//! guarantees no piece is sampled more than `2 × EC_DC_PERIOD_NS = 2 ms`
//! after its start time (since the client always delivers pieces in order and
//! the clock never runs ahead of the available pieces).
//!
//! ## Synthetic clock (deterministic retirement)
//!
//! Piece `i` starts at `BASE_NS + i × PIECE_DUR_NS`.  The endpoint clock
//! also starts at `BASE_NS` and advances by `PIECE_DUR_NS` per tick.  Clock
//! and piece timelines are aligned so:
//!
//! - Tick 0 (clock = BASE_NS): piece 0 is current → sampled, not yet retired.
//! - Tick 1 (clock = BASE_NS + 1 ms): piece 0 is elapsed → retired; piece 1
//!   is current (if present).
//! - … and so on for all N pieces.
//!
//! The client pump sends one batch per `kalico_call_on_channel` call.  After
//! each batch it polls `conn.poll_events()` so the heartbeat callback can
//! update the retired counter and the client can gate further sends on the
//! ring occupancy.
//!
//! No wall-clock timing drives correctness.  Wall-clock `sleep` is used only
//! for OS-level socket scheduling between loop iterations.
//!
//! ## Assertions
//!
//! - Every `PushPiecesResponse.result == 0` (ring accepted every piece).
//! - `heartbeat_callback` eventually reports `retired_counts[0] == N`.
//! - The endpoint never sets the fault flag (no `PieceStartInPast` on the
//!   contiguous synthetic timeline).
//! - No deadlock: all loops are bounded with generous deadlines and emit a
//!   clear panic message if they stall.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use kalico_ethercat_rt::curves::AXIS_RING_CAPACITY;
use kalico_ethercat_rt::curves::NUM_AXES;
use kalico_ethercat_rt::server::FrameServer;
use kalico_ethercat_rt::wire::{
    Command, identify_response_frame, push_pieces_response_frame, runtime_caps_response_frame,
    status_heartbeat_frame,
};
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use kalico_protocol::codec::{Decode, Encode};
use kalico_protocol::messages::{MessageKind, PushPieces, PushPiecesResponse};
use kalico_protocol::KALICO_CHANNEL_PIECES;
use runtime::piece_ring::PieceEntry;

// ── Constants ─────────────────────────────────────────────────────────────────

/// Total pieces streamed — intentionally past one full ring depth.
const N: usize = AXIS_RING_CAPACITY + 20; // 276 at AXIS_RING_CAPACITY=256

/// Maximum pieces in a single `PushPieces` batch.  Kept well below
/// `AXIS_RING_CAPACITY` so the ring never overflows within a single batch.
const BATCH: usize = 8;

/// Piece duration — one DC cycle (1 ms = 1_000_000 ns).
const PIECE_DUR_NS: u64 = 1_000_000;

/// Synthetic epoch: large enough that `saturating_sub` never underflows and
/// the epoch is not confused with the zero-clock default state.
const BASE_NS: u64 = 10_000_000_000;

/// Per-call timeout passed to `kalico_call_on_channel`.
const CALL_TIMEOUT: Duration = Duration::from_secs(10);

// ── Endpoint thread ───────────────────────────────────────────────────────────

/// Run the `FrameServer` endpoint with an interleaved command-poll + DC-clock
/// loop.
///
/// The endpoint accepts `PushPieces` frames while simultaneously advancing the
/// synthetic clock in 1 ms steps.  The clock starts at `BASE_NS` and steps
/// forward on every iteration that finds the ring non-empty, so pieces are
/// always retired promptly and the ring never fills up as long as the client
/// keeps sending.
///
/// Runs until both conditions hold:
/// - All N pieces have been received (via `total_pushed == N`).
/// - All N pieces have been retired by the DC loop (via `ring.retired_count() == N`).
fn run_endpoint(socket_path: String, faulted: Arc<AtomicBool>) {
    use kalico_ethercat_rt::curves::AxisRing;

    let mut server = FrameServer::bind(&socket_path).expect("endpoint: bind");
    let mut ring = AxisRing::new();
    let mut total_pushed: usize = 0;
    let mut last_sent_retired: u32 = 0;

    // Synthetic clock.  Starts at BASE_NS and advances one PIECE_DUR_NS per
    // iteration when the ring is non-empty.
    let mut now: u64 = BASE_NS;

    let deadline = std::time::Instant::now() + Duration::from_secs(60);

    loop {
        assert!(
            std::time::Instant::now() < deadline,
            "endpoint: timed out (pushed={total_pushed}/{N}, \
             retired={})",
            ring.retired_count()
        );

        // ── 1. Command phase per iteration: drain available commands ────────
        for cmd in server.poll_commands() {
            match cmd {
                Command::Identify { correlation_id, proto_version } => {
                    server.respond(&identify_response_frame(correlation_id, proto_version));
                }
                Command::PushPieces { correlation_id, msg } => {
                    let front_start_time = if msg.piece_count > 0
                        && msg.pieces_bytes.len() >= 8
                    {
                        u64::from_le_bytes(
                            msg.pieces_bytes[0..8].try_into().unwrap_or([0u8; 8]),
                        )
                    } else {
                        0
                    };
                    let pushed =
                        ring.push_from_bytes(msg.piece_count, &msg.pieces_bytes);
                    let result =
                        if pushed == msg.piece_count { 0i32 } else { -309i32 };
                    server.respond(&push_pieces_response_frame(
                        correlation_id,
                        result,
                        BASE_NS,
                        front_start_time,
                    ));
                    total_pushed += pushed as usize;
                }
                Command::QueryRuntimeCaps { correlation_id } => {
                    // Endpoint responds with its actual ring capacity.
                    let total = (AXIS_RING_CAPACITY * NUM_AXES * 32) as u32;
                    server.respond(&runtime_caps_response_frame(correlation_id, total));
                }
                Command::Unknown { .. } => {}
            }
        }

        // ── 2. DC loop tick: advance clock and sample if ring non-empty ─────
        if !ring.is_empty() {
            let _pos = ring.sample(now);

            // Check for faults: PieceStartInPast means the clock advanced
            // faster than pieces were delivered — a test-design bug.
            if ring.take_fault().is_some() {
                faulted.store(true, Ordering::SeqCst);
            }

            let current_retired = ring.retired_count();
            if current_retired != last_sent_retired {
                let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
                server.respond(&status_heartbeat_frame(
                    engine_state,
                    &[current_retired],
                ));
                last_sent_retired = current_retired;
            }

            // Advance synthetic clock by one piece duration.
            now = now.saturating_add(PIECE_DUR_NS);
        }

        // ── 3. Exit when all pieces received AND retired ─────────────────
        if total_pushed >= N && ring.retired_count() as usize >= N {
            // Final heartbeat to ensure the client sees the terminal count.
            let engine_state: u8 = 0; // ring is empty
            server.respond(&status_heartbeat_frame(engine_state, &[ring.retired_count()]));
            break;
        }

        // Brief yield so the client thread gets CPU time to issue the next send.
        thread::sleep(Duration::from_millis(1));
    }
}

// ── Helpers ───────────────────────────────────────────────────────────────────

/// Build the encoded `PushPieces` body for `pieces[start..end]`.
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

// ── Test ───────────────────────────────────────────────────────────────────────

/// Prove that `UnixNativeConn` and `FrameServer` interoperate over a real Unix
/// socket for sustained streaming past one ring depth (`N = 52 > RING_CAP = 32`),
/// with retirement flowing back via the heartbeat callback.
///
/// Chain exercised:
/// ```text
/// client                            endpoint (synthetic clock)
/// ──────                            ──────────────────────────
/// UnixNativeConn                    FrameServer
///   ::kalico_call_on_channel   ──►    ::poll_commands
///     (PushPieces, PIECES ch)         decode_command → ring.push_from_bytes
///                              ◄──    push_pieces_response_frame
///   decode PushPiecesResponse         ring.sample(now) → retired_count++
///   ::poll_events              ◄──    status_heartbeat_frame(retired_counts)
///   heartbeat_callback(retired)
///   (flow-control gate: send next batch when ring has room)
/// ```
#[test]
fn unix_native_conn_and_frame_server_sustain_streaming_past_ring_depth() {
    let socket_path = format!(
        "/tmp/kalico-e2e-stream-{}.sock",
        std::process::id()
    );
    let _ = std::fs::remove_file(&socket_path);

    // Build all N contiguous 1 ms pieces up front.
    // Piece i starts at BASE_NS + i * PIECE_DUR_NS.
    let pieces: Vec<PieceEntry> = (0..N)
        .map(|i| PieceEntry {
            start_time: BASE_NS + i as u64 * PIECE_DUR_NS,
            coeffs: [0.0_f32; 4],
            duration: PIECE_DUR_NS as f32 / 1_000_000_000.0,
            _reserved: 0,
        })
        .collect();

    // Shared state for the heartbeat callback.
    let last_retired = Arc::new(AtomicU32::new(0));
    let faulted = Arc::new(AtomicBool::new(false));

    // ── Spawn endpoint thread ───────────────────────────────────────────────
    {
        let sp = socket_path.clone();
        let fault_flag = Arc::clone(&faulted);
        thread::spawn(move || run_endpoint(sp, fault_flag));
    }

    // Wait for the socket to appear (up to 500 ms).
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

    // ── Connect real UnixNativeConn ─────────────────────────────────────────
    let conn = UnixNativeConn::connect(&socket_path)
        .expect("UnixNativeConn::connect must succeed");

    // Install heartbeat callback — monotonically records max(retired_counts[0]).
    {
        let lr = Arc::clone(&last_retired);
        conn.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
            if let Some(&v) = retired.first() {
                // Advance the atomic only when v is larger (monotone).
                let mut prev = lr.load(Ordering::Acquire);
                loop {
                    if v <= prev {
                        break;
                    }
                    match lr.compare_exchange_weak(
                        prev,
                        v,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    ) {
                        Ok(_) => break,
                        Err(cur) => prev = cur,
                    }
                }
            }
        }));
    }

    // ── Pump all N pieces in flow-controlled batches ─────────────────────────
    //
    // Ring occupancy (from the client's perspective):
    //   occupancy = total_sent − retired
    //
    // We gate each batch on: occupancy + batch_size ≤ AXIS_RING_CAPACITY.
    // Between retries we poll `conn.poll_events()` so the heartbeat callback
    // can advance `last_retired` and make room for the next batch.

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
            // Ring full — poll for heartbeats and yield.
            conn.poll_events();
            thread::sleep(Duration::from_millis(2));
            continue;
        }

        // Clamp batch to available room and remaining pieces.
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
            resp.result,
            0,
            "PushPiecesResponse.result must be 0 (OK) for batch \
             {total_sent}..{batch_end}"
        );

        total_sent = batch_end;

        // Opportunistically drain any events that arrived with the response.
        conn.poll_events();
    }

    // All N pieces delivered.  Now wait for the heartbeat to report retired == N.
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

        conn.poll_events();
        thread::sleep(Duration::from_millis(2));
    }

    // ── Final assertions ────────────────────────────────────────────────────
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
