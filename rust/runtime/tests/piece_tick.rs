//! Integration tests for the per-axis piece-ring ISR tick path.
//!
//! Exercises the full pipeline: configure_axis → push_pieces → tick → verify
//! observable state.  Tests cover:
//!   - Arming a piece when start_time is reached
//!   - Holding at t=0 before start_time (future piece adopted into cache)
//!   - Idle behaviour with an empty ring
//!   - Hard fault when a piece's start_time exceeds the drift-budget tolerance
//!   - Within-tolerance arming (1 tick late — must NOT fault)
//!   - Advancing through consecutive pieces
//!   - push_pieces rejection when the ring is full

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::error::FaultCode;
use runtime::piece_ring::PieceEntry;
use runtime::state::{SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

// 520 MHz clock, 40 kHz ISR → 13_000 cycles per tick.
const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u64 = (CLOCK_FREQ / SAMPLE_RATE) as u64; // 13_000

// A trivial constant piece: all Bernstein control points equal → position
// does not change over the piece duration (useful to avoid driving any steps
// and keep the queue from interfering with assertions). The constant value is
// 0 so it matches the engine's unseeded baseline (last_step_count == 0): the
// first sample computes signed_steps == 0 and drives no steps. A non-zero
// constant here would arm with an instantaneous |Δsteps| far above
// MAX_STEPS_PER_SAMPLE and (correctly) trip the StepsPerSampleExceeded fault.
fn const_piece(start_time: u64, duration: f32) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [0.0; 4],
        duration,
        _reserved: 0,
    }
}

fn make_engine() -> Engine {
    Engine::new(CLOCK_FREQ, SAMPLE_RATE)
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

/// Configure axis 0 as Pulse with the given ring depth.
fn configure_axis0(engine: &mut Engine, ring_depth: usize) {
    let rc = engine.configure_axis(
        0,
        StepMode::Pulse,
        0.0125,
        ring_depth,
        &[pulse_binding()],
        TOTAL_RING_PIECES,
    );
    assert_eq!(rc, 0, "configure_axis failed");
}

// ── Test 1: arming when start_time is reached ────────────────────────────────

/// Push a single piece with `start_time = TICK_CYCLES`.  Tick at `now =
/// TICK_CYCLES`.  The ISR should arm the piece (popping it from the ring) and
/// not latch any fault.  After the tick the ring is empty (consumed == 1) and
/// `last_error` is 0.
#[test]
fn tick_arms_piece_when_start_time_reached() {
    let mut engine = make_engine();
    configure_axis0(&mut engine, 64);

    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];

    let piece = const_piece(TICK_CYCLES, 0.001);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    // Install a real step queue so dispatch_axis has somewhere to write.
    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();
    engine.tick(TICK_CYCLES, &shared, &mut storage);

    // Piece armed but its window has NOT yet ended (end = TICK_CYCLES +
    // 0.001 * 520e6 = 533_000 >> TICK_CYCLES=13_000). Under retire-time
    // semantics the cursor does not advance until the window elapses.
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece armed but still playing -> retired must be 0 at arm time"
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault should be latched"
    );

    // Ring depth is 64 and len == 1 (the playing piece still occupies its
    // slot). Depth 64 >> 1 so there is still room; pushing a second piece
    // must succeed.
    let piece2 = const_piece(TICK_CYCLES + 520_000, 0.001);
    let rc2 = engine.push_pieces(0, &[piece2], &mut storage);
    assert_eq!(
        rc2, 0,
        "should be able to push while piece is playing (depth >> 1)"
    );
}

// ── Test 2: hold at t=0 before start_time ────────────────────────────────────

/// Push a piece that hasn't started yet.  With the gap branch removed (spec
/// §4.4 cursor walk), the ISR adopts the future piece into its cache and holds
/// it at `t = 0` via `eval_horner`'s saturating elapsed — it does NOT idle in
/// the ring.  Observable contract before `start_time`:
///   - no fault is latched (a future piece passes the 2-tick check trivially);
///   - the piece is armed (`armed.is_some()`) but NOT retired — under
///     retire-time semantics `retired` stays 0 until the piece's window ends;
///   - no motion is produced (the constant piece dispatches no steps).
#[test]
fn tick_holds_at_t0_before_start_time() {
    let mut engine = make_engine();
    configure_axis0(&mut engine, 64);

    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];

    // Piece starts far in the future.
    let piece = const_piece(100_000, 0.001);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0);

    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();
    // Tick well before the piece starts → adopt into cache, hold at t=0.
    engine.tick(TICK_CYCLES, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault for a future piece held at t=0"
    );
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "future piece is armed but its window has not ended -> retired must be 0"
    );

    // A second pre-start tick must keep holding the SAME piece — the window
    // has not ended (piece_end > now) so branch 1 holds and retired stays 0.
    engine.tick(TICK_CYCLES * 2, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "still no fault on a later pre-start tick"
    );
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece still playing (future start, held at t=0) -> retired must remain 0"
    );
}

// ── Test 3: idle when ring is empty ──────────────────────────────────────────

/// Configure axis 0 with no pieces pushed.  Tick must complete without
/// faulting or reporting consumption.
#[test]
fn tick_idle_when_ring_empty() {
    let mut engine = make_engine();
    configure_axis0(&mut engine, 64);

    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];

    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();
    engine.tick(TICK_CYCLES, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault for empty ring"
    );
    assert_eq!(engine.retired_counts()[0], 0);
}

// ── Test 4: fault on piece start in past ─────────────────────────────────────

/// Push a piece with `start_time = 1_000`.  Tick well past the drift-budget
/// fault tolerance and confirm `PieceStartInPast` is latched.
///
/// Tolerance at 520 MHz / 40 kHz (TICK_CYCLES = 13_000):
///   drift_budget = 200 µs × 520_000_000 Hz = 104_000 cycles
///   fault_tolerance = 104_000 + 13_000 = 117_000 cycles
///
/// We tick at start + 10 × TICK_CYCLES = start + 130_000, which exceeds
/// 117_000 and must trigger the fault.
#[test]
fn tick_faults_on_piece_start_in_past() {
    let mut engine = make_engine();
    configure_axis0(&mut engine, 64);

    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];

    let start = 1_000_u64;
    let piece = const_piece(start, 0.001);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0);

    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();
    // 10 ticks past start (130_000 cycles) — exceeds drift-budget tolerance (117_000).
    let now = start + TICK_CYCLES * 10;
    engine.tick(now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceStartInPast.as_i32(),
        "PieceStartInPast fault must be latched when piece exceeds drift-budget tolerance"
    );
}

// ── Test 5: within-fault-tolerance arms ok ───────────────────────────────────

/// Push a piece with `start_time = 1_000`.  Tick at exactly 1 tick past start
/// (`now = 1_000 + TICK_CYCLES`).  This is well within the drift-budget tolerance
/// (1 tick = 13_000 cycles << 117_000 cycles) so the piece must ARM (no fault).
#[test]
fn tick_within_fault_tolerance_arms_ok() {
    let mut engine = make_engine();
    configure_axis0(&mut engine, 64);

    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];

    let start = 1_000_u64;
    let piece = const_piece(start, 0.001);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0);

    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();
    // Exactly 1 tick late — within tolerance.
    let now = start + TICK_CYCLES;
    engine.tick(now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault expected within drift-budget tolerance"
    );
    // Piece was armed; window ends at start + 0.001 * 520e6 = 521_000 cycles,
    // well after now = 14_000. Under retire-time semantics retired stays 0.
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece armed but still playing -> retired must be 0"
    );
}

// ── Test 6: advance through consecutive pieces ───────────────────────────────

/// Push two consecutive pieces.  Piece A spans [TICK_CYCLES, TICK_CYCLES +
/// 0.001 × 520e6).  Piece B starts where A ends.
///
/// Under retire-time semantics:
/// Tick at `now = TICK_CYCLES`  → A is armed, still playing → retired == 0.
/// Tick at `now = A_end`        → A's window ends; A is retired, B is armed
///                                → retired == 1.
/// Assert no fault throughout.
#[test]
fn tick_advances_through_consecutive_pieces() {
    let mut engine = make_engine();
    configure_axis0(&mut engine, 64);

    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];

    // Piece A: starts at TICK_CYCLES, 1 ms duration.
    let a_start = TICK_CYCLES;
    let a_duration = 0.001_f32;
    // A_end in clock cycles: start + floor(duration * clock_freq)
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let a_end = a_start + (a_duration * CLOCK_FREQ as f32) as u64; // 533_000

    // Piece B: starts exactly where A ends.
    let b_start = a_end;
    let b_duration = 0.001_f32;

    let piece_a = const_piece(a_start, a_duration);
    let piece_b = const_piece(b_start, b_duration);

    let rc = engine.push_pieces(0, &[piece_a, piece_b], &mut storage);
    assert_eq!(rc, 0, "push of both pieces must succeed");

    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();

    // First tick: arms piece A. Window ends at a_end >> a_start → still
    // playing. Under retire-time semantics retired == 0 at arm time.
    engine.tick(a_start, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault after arming piece A"
    );
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece A armed but still playing -> retired must be 0"
    );

    // Second tick at a_end (== b_start): piece A's window ends → retire A
    // (retired → 1), then arm piece B. Piece B is still playing (b_end >> a_end).
    engine.tick(a_end, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault after retiring A and arming piece B"
    );
    assert_eq!(
        engine.retired_counts()[0],
        1,
        "piece A retired; piece B still playing -> retired must be 1"
    );
}

// ── Test 7: push_pieces rejects when ring is full ────────────────────────────

// ── Test 8: retire cursor bumps at window end, not at arm ────────────────────

/// One axis, two back-to-back pieces each of duration D (10 ms at 520 MHz).
/// The tick sequence is designed so each piece is armed within the 2-tick
/// fault tolerance (ticking at or just after the piece's start_time):
///   1. `now = p0_start`        — arm piece 0; still playing → retired == 0.
///   2. `now = p0_start + D/2`  — mid piece 0; still playing → retired == 0.
///   3. `now = p1_start`        — p0 window ends; arm p1 → retired == 1.
///   4. `now = p1_start + D/2`  — mid piece 1; still playing → retired == 1.
///   5. `now = p1_start + D + 1`— p1 window ends; ring drained → retired == 2.
#[test]
fn retired_count_bumps_at_window_end_not_arm() {
    let mut engine = make_engine();
    configure_axis0(&mut engine, 64);

    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0,
        };
        TOTAL_RING_PIECES
    ];

    // D = 10 ms worth of cycles at 520 MHz (5_200_000).
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let d: u64 = (0.010_f32 * CLOCK_FREQ as f32) as u64;

    // Piece 0 starts at TICK_CYCLES so first tick arms it within 2-tick
    // fault tolerance (now == start_time → lateness == 0).
    let p0_start: u64 = TICK_CYCLES;
    let p1_start: u64 = p0_start + d;

    let piece0 = PieceEntry {
        start_time: p0_start,
        coeffs: [0.0; 4],
        duration: 0.010,
        _reserved: 0,
    };
    let piece1 = PieceEntry {
        start_time: p1_start,
        coeffs: [0.0; 4],
        duration: 0.010,
        _reserved: 0,
    };

    let rc = engine.push_pieces(0, &[piece0, piece1], &mut storage);
    assert_eq!(rc, 0, "push_pieces must succeed");

    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();

    // Tick 1: at p0_start → arm piece 0. Window extends to p1_start, so
    // piece 0 is still playing → retired == 0.
    engine.tick(p0_start, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault arming piece 0"
    );
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece 0 just armed and still playing -> retired must be 0"
    );

    // Tick 2: mid-way through piece 0. Still playing → retired == 0.
    engine.tick(p0_start + d / 2, &shared, &mut storage);
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece 0 still playing -> retired must be 0"
    );

    // Tick 3: exactly at p1_start (== p0 end). Engine sees piece 0's window
    // has ended, retires it (retired → 1), then arms piece 1 (lateness = 0).
    engine.tick(p1_start, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault retiring p0 / arming p1"
    );
    assert_eq!(
        engine.retired_counts()[0],
        1,
        "piece 0 window ended -> retired must be 1"
    );

    // Tick 4: mid-way through piece 1. Still playing → retired == 1.
    engine.tick(p1_start + d / 2, &shared, &mut storage);
    assert_eq!(
        engine.retired_counts()[0],
        1,
        "piece 1 still playing -> retired must be 1"
    );

    // Tick 5: one cycle past piece 1's end. Engine retires piece 1 → retired == 2.
    engine.tick(p1_start + d + 1, &shared, &mut storage);
    assert_eq!(
        engine.retired_counts()[0],
        2,
        "both pieces finished -> retired must equal sent (2)"
    );
}

/// Configure axis 0 with `ring_depth = 4`.  Pushing 4 pieces must succeed;
/// pushing a 5th must return a negative result (RING_FULL).
#[test]
fn push_pieces_rejects_when_ring_full() {
    let mut engine = make_engine();

    let rc = engine.configure_axis(
        0,
        StepMode::Pulse,
        0.0125,
        4, // ring depth of 4
        &[pulse_binding()],
        TOTAL_RING_PIECES,
    );
    assert_eq!(rc, 0, "configure_axis failed");

    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0
        };
        TOTAL_RING_PIECES
    ];

    // Fill the ring with 4 pieces (each starting well in the future so the
    // ISR won't arm/consume them).
    for i in 0..4_u64 {
        let piece = const_piece(100_000_000 + i * 520_000, 0.001);
        let rc = engine.push_pieces(0, &[piece], &mut storage);
        assert_eq!(rc, 0, "push {i} must succeed (ring not yet full)");
    }

    // 5th push must be rejected.
    let overflow_piece = const_piece(200_000_000, 0.001);
    let rc = engine.push_pieces(0, &[overflow_piece], &mut storage);
    assert!(
        rc < 0,
        "5th push must return a negative error (RING_FULL), got {rc}"
    );
}
