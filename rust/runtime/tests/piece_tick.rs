//! Integration tests for the per-axis piece-ring ISR tick path.
//!
//! Exercises the full pipeline: configure_axis → push_pieces → tick → verify
//! observable state.  Tests cover:
//!   - Arming a piece when start_time is reached
//!   - Idle behaviour before start_time
//!   - Idle behaviour with an empty ring
//!   - Hard fault when a piece's start_time is more than 2 ticks in the past
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
// and keep the queue from interfering with assertions).
fn const_piece(start_time: u64, duration: f32) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [10.0; 4],
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

    // Piece armed → popped → consumed count becomes 1.
    assert_eq!(
        engine.consumed_counts()[0],
        1,
        "piece should have been popped (consumed) after tick at start_time"
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault should be latched"
    );

    // The ring slot was consumed; pushing a second piece must succeed.
    let piece2 = const_piece(TICK_CYCLES + 520_000, 0.001);
    let rc2 = engine.push_pieces(0, &[piece2], &mut storage);
    assert_eq!(rc2, 0, "should be able to push after consumption");
}

// ── Test 2: idle before start_time ───────────────────────────────────────────

/// Push a piece that hasn't started yet.  The ISR must idle (no fault, no
/// consumption) and leave the ring untouched.
#[test]
fn tick_idle_before_start_time() {
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
    // Tick at a time well before the piece starts.
    engine.tick(TICK_CYCLES, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault for idle before start_time"
    );
    assert_eq!(
        engine.consumed_counts()[0],
        0,
        "piece must not be consumed before its start_time"
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
    assert_eq!(engine.consumed_counts()[0], 0);
}

// ── Test 4: fault on piece start in past ─────────────────────────────────────

/// Push a piece with `start_time = 1_000`.  Tick at 3 ticks past that start
/// time (`now = 1_000 + 3*TICK_CYCLES`).  The fault tolerance is 2 ticks, so
/// this must latch `PieceStartInPast`.
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
    // 3 ticks past start — exceeds 2-tick tolerance.
    let now = start + TICK_CYCLES * 3;
    engine.tick(now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceStartInPast.as_i32(),
        "PieceStartInPast fault must be latched when piece is >2 ticks in the past"
    );
}

// ── Test 5: within-fault-tolerance arms ok ───────────────────────────────────

/// Push a piece with `start_time = 1_000`.  Tick at exactly 1 tick past start
/// (`now = 1_000 + TICK_CYCLES`).  This is within the 2-tick tolerance so the
/// piece must ARM (no fault).
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
        "no fault expected within 2-tick tolerance"
    );
    // The piece was armed/popped.
    assert_eq!(engine.consumed_counts()[0], 1, "piece should be consumed");
}

// ── Test 6: advance through consecutive pieces ───────────────────────────────

/// Push two consecutive pieces.  Piece A spans [TICK_CYCLES, TICK_CYCLES +
/// 0.001 × 520e6).  Piece B starts where A ends.
///
/// Tick at `now = TICK_CYCLES`  → A is armed (consumed = 1).
/// Tick at `now = A_end`        → A is expired, B is armed (consumed = 2).
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

    // First tick: arms piece A.
    engine.tick(a_start, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault after arming piece A"
    );
    assert_eq!(
        engine.consumed_counts()[0],
        1,
        "piece A must be consumed after first tick"
    );

    // Second tick: A has expired (now == a_end == b_start), arms piece B.
    engine.tick(a_end, &shared, &mut storage);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault after arming piece B"
    );
    assert_eq!(
        engine.consumed_counts()[0],
        2,
        "both pieces must be consumed after second tick"
    );
}

// ── Test 7: push_pieces rejects when ring is full ────────────────────────────

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
