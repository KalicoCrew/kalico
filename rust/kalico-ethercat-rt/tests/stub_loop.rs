//! Integration test: the stub's DC loop logic — `FrameServer::poll_commands` +
//! `AxisRing` + `StatusHeartbeat` emission — exercised against a live
//! `UnixStream::pair`. Verifies the full closed loop:
//!
//!   host → PushPieces → stub → AxisRing.push → PushPiecesResponse
//!   stub DC-cycle → AxisRing.sample past expiry → StatusHeartbeat(retired_count)
//!   host ← StatusHeartbeat.retired_counts[0] = N
//!
//! This is the key correctness property: N pieces pushed, N pieces eventually
//! retired, heartbeat carries N — so the pump's `AxisQueue.retired` reaches
//! `pushed` and room is replenished.
//!
//! No hardware, no subprocess. The stub's `FrameServer` is constructed from
//! a `UnixListener` at a temp socket path; the client side uses a `UnixStream`
//! connect. The DC loop runs inline for a bounded number of iterations.
//!
//! ## Synthetic clock (deterministic retirement)
//!
//! The hardened walker (`motion_core::get_position_and_velocity`) faults and
//! returns `None` when it adopts a piece whose start is more than
//! `drift_budget + EC_DC_PERIOD_NS` (1.2 ms) in the past — so the old strategy
//! of pushing pieces 10 s in the past no longer causes retirement; it causes a
//! fault instead.
//!
//! Both tests therefore use a **synthetic clock**: pieces start at a known
//! `base` ns, and the test drives the stub's sample loop with `now` values
//! that advance in controlled 1 ms steps from `base`. Each piece is sampled
//! while current (no fault), then retired once `now` crosses its end.
//! Wall-clock sleeps are not used for retirement logic — only for socket
//! synchronisation where unavoidable.

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

// ── Wire helpers (mirrors push_pieces_frame from ec-test-client) ─────────

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

/// Drain all available frames from a non-blocking `stream` and decode them.
/// Returns `(push_pieces_responses, heartbeats)`.
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
                break; // nothing more available right now
            }
            Err(_) => break,
        }
    }
    (ppr, hbs)
}

/// Verify the full ring-walk + heartbeat emission loop using a **synthetic clock**.
///
/// Strategy: N contiguous 1 ms pieces. The server thread has two phases:
///
/// 1. **Command phase** (no sampling): poll the socket until PushPieces arrives;
///    push the pieces and respond. This ensures the ring is populated before the
///    synthetic clock starts advancing, eliminating the race between the advancing
///    `now` and late PushPieces delivery.
/// 2. **DC loop phase**: advance a synthetic `now` from `BASE_NS` in 1 ms steps.
///    Each piece is adopted while current (no PieceStartInPast fault) and retired
///    once `now` crosses its end.
///
/// Only wall-clock `sleep` is used for socket synchronisation (yielding between
/// command-phase polls and DC cycles), never for retirement timing.
///
/// Asserts:
/// - `PushPiecesResponse.result == 0` (all pieces accepted).
/// - A `StatusHeartbeat` eventually carries `retired_counts[0] == N`.
#[test]
fn push_pieces_and_heartbeat_closes_the_loop() {
    // Use a temp socket file.
    let socket_path = format!("/tmp/kalico-ethercat-test-{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket_path);

    // N contiguous 1 ms pieces. The synthetic clock advances in 1 ms steps,
    // retiring one piece per step once the DC phase begins.
    const N: usize = 3;
    const PIECE_DUR_NS: u64 = 1_000_000; // 1 ms in ns — one DC cycle

    // Synthetic epoch. Well clear of 0 to avoid underflow in saturating_sub.
    const BASE_NS: u64 = 10_000_000_000; // 10 s in ns (arbitrary non-zero epoch)

    // Build pieces before spawning: piece i starts at BASE_NS + i*1ms.
    let pieces: Vec<PieceEntry> = (0..N)
        .map(|i| PieceEntry {
            start_time: BASE_NS + i as u64 * PIECE_DUR_NS,
            coeffs: [0.0_f32; 4],
            duration: PIECE_DUR_NS as f32 / 1_000_000_000.0, // 1 ms in seconds
            _reserved: 0,
        })
        .collect();

    // Channel through which the server thread reports the final retired count.
    let (tx, rx) = std::sync::mpsc::channel::<u32>();

    let socket_path_sv = socket_path.clone();
    let server_thread = thread::spawn(move || {
        let mut server = FrameServer::bind(&socket_path_sv).expect("bind server socket");
        let mut ring = AxisRing::new();
        let mut last_sent_retired: u32 = 0;
        let mut heartbeat_sent = false;

        // ── Phase 1: command phase ────────────────────────────────────────
        // Poll until PushPieces arrives and the ring is populated. The synthetic
        // clock does NOT advance in this phase, so pieces (which start at BASE_NS)
        // are always fresh when we begin sampling.
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
                            BASE_NS, // arrival_clock: synthetic ns
                            front_start_time,
                        ));
                        push_received = true;
                    }
                    Command::QueryRuntimeCaps { correlation_id } => {
                        let total = (AXIS_RING_CAPACITY * NUM_AXES * 32) as u32;
                        server.respond(&runtime_caps_response_frame(correlation_id, total));
                    }
                    Command::Unknown { .. } => {}
                }
            }
            if !push_received {
                thread::sleep(Duration::from_millis(1));
            }
        }

        // ── Phase 2: DC loop with synthetic clock ─────────────────────────
        // Pieces start at BASE_NS + k*1ms. Advance `now` from BASE_NS in 1 ms
        // steps. Each piece is adopted while current (fault check passes) and
        // retired once `now` crosses piece_end. N+4 steps is sufficient for all
        // N pieces to retire plus a few extra heartbeat flush cycles.
        let mut now: u64 = BASE_NS;
        let total_cycles = (N as u64) + 4;
        for _ in 0..total_cycles {
            // No new commands expected in DC phase; drain anyway to flush any
            // stray socket data (e.g. the client dropping the connection).
            for cmd in server.poll_commands() {
                match cmd {
                    Command::Unknown { .. }
                    | Command::Identify { .. }
                    | Command::QueryRuntimeCaps { .. }
                    | Command::PushPieces { .. } => {}
                }
            }

            // Sample with the synthetic clock: no wall-clock sleep for retirement.
            let _ = ring.sample(now);

            let current_retired = ring.retired_count();
            let should_emit = !heartbeat_sent || current_retired != last_sent_retired;
            if should_emit {
                let engine_state: u8 = if ring.is_empty() { 0 } else { 1 };
                server.respond(&status_heartbeat_frame(engine_state, &[current_retired]));
                last_sent_retired = current_retired;
                heartbeat_sent = true;
            }

            // Advance synthetic clock by one 1 ms step.
            now = now.saturating_add(PIECE_DUR_NS);

            // Brief yield so the client can drain the socket.
            thread::sleep(Duration::from_millis(1));
        }

        // Report the final retired count back to the test.
        let _ = tx.send(ring.retired_count());
    });

    // Wait for the socket to appear (up to 500 ms).
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

    // Connect the client in non-blocking mode so reads don't stall.
    let mut client = UnixStream::connect(&socket_path).expect("connect client");
    client.set_nonblocking(true).expect("set_nonblocking");

    // Send PushPieces with all N pieces.
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

    // Drop client to let the server thread finish its loop.
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

/// Verify the `PieceStartInPast` fault path of the hardened walker.
///
/// The walker faults when a freshly adopted piece's start is more than
/// `fault_tolerance = drift_budget + EC_DC_PERIOD_NS` in the past, where:
///   `drift_budget = (200e-6 * 1_000_000_000) = 200_000 ns`
///   `fault_tolerance = 200_000 + 1_000_000 = 1_200_000 ns = 1.2 ms`
///
/// `get_position_and_velocity` calls `EtherCatFaultSink::piece_start_in_past`
/// and returns `None`. This test asserts that `sample()` returns `None` for
/// such a stale piece, locking in the "host pump fell behind" semantics.
///
/// The piece starts 1 s before `now` — far exceeding the 1.2 ms fault tolerance.
#[test]
fn piece_start_in_past_faults_and_returns_none() {
    let mut ring = AxisRing::new();

    // `now` is a concrete synthetic time (10 s into the epoch).
    let now_ns: u64 = 10_000_000_000;

    // Piece starts 1 s before now — 1_000_000_000 ns >> 1.2 ms fault tolerance.
    let stale_start = now_ns - 1_000_000_000;

    ring.push_entry(PieceEntry {
        start_time: stale_start,
        coeffs: [0.0_f32; 4],
        duration: 0.001, // 1 ms — nominally still within its window, but stale at adoption
        _reserved: 0,
    })
    .unwrap();

    // The walker adopts the piece, detects start is > 8.2 ms in the past, faults,
    // and returns None. We are 1_000_000_000 ns (1 s) late — solidly in the fault window.
    let result = ring.sample(now_ns);
    assert!(
        result.is_none(),
        "expected None (PieceStartInPast fault) for a piece starting 1 s before now, \
         got {result:?}"
    );
}

/// Verify that the EtherCAT endpoint answers `QueryRuntimeCaps` with a
/// `RuntimeCapsResponse` whose `total_piece_memory` encodes the real ring
/// capacity, and that the host's `axis_ring_depth` derivation recovers
/// `AXIS_RING_CAPACITY` exactly.
///
/// Protocol path exercised (same as `init_planner` in `motion-bridge`):
///   host → QueryRuntimeCaps (control channel 0)
///   endpoint → RuntimeCapsResponse { total_piece_memory }
///   host: ring_depth = (total_piece_memory / 32) / NUM_AXES
///         → must equal AXIS_RING_CAPACITY (= 256)
///
/// This pins the single-source-of-truth invariant: the host never uses a
/// hard-coded constant; it always reads the depth from the endpoint's report.
#[test]
fn ethercat_endpoint_query_runtime_caps_round_trip() {
    use kalico_host_rt::unix_native_conn::UnixNativeConn;

    // Bind a temp socket for the in-process endpoint.
    let socket_path = format!("/tmp/kalico-caps-rt-{}.sock", std::process::id());
    let _ = std::fs::remove_file(&socket_path);

    // Expected total_piece_memory the endpoint should report.
    // size_of::<PieceEntry>() == 32 is verified by a compile-time assert in
    // runtime::piece_ring.
    const PIECE_ENTRY_SIZE: usize = 32;
    let expected_total: u32 = (AXIS_RING_CAPACITY * NUM_AXES * PIECE_ENTRY_SIZE) as u32;

    // ── Spawn endpoint thread ──────────────────────────────────────────────
    {
        let sp = socket_path.clone();
        thread::spawn(move || {
            let mut server = FrameServer::bind(&sp).expect("endpoint: bind");
            // Service exactly one QueryRuntimeCaps call, then exit.
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
                        return; // one call is enough for this test
                    }
                    // Respond to stray Identify or Unknown without failing the test.
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

    // ── Host side: connect UnixNativeConn and issue QueryRuntimeCaps ───────
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

    // Derive ring_depth the same way motion-bridge does:
    //   axis_ring_depth(total_pieces, num_axes) = max(total_pieces / num_axes, 1)
    //   where total_pieces = total_piece_memory / 32
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
