use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::error::FaultCode;
use runtime::piece_ring::PieceEntry;
use runtime::state::{SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u64 = (CLOCK_FREQ / SAMPLE_RATE) as u64;

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

    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();
    engine.tick(TICK_CYCLES, &shared, &mut storage);

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

    let piece2 = const_piece(TICK_CYCLES + 520_000, 0.001);
    let rc2 = engine.push_pieces(0, &[piece2], &mut storage);
    assert_eq!(
        rc2, 0,
        "should be able to push while piece is playing (depth >> 1)"
    );
}

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

    let piece = const_piece(100_000, 0.001);
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0);

    let mut q0 = StepQueue::new();
    let mut qs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    qs[0] = &mut q0;
    engine.test_install_step_queues(qs);

    let shared = SharedState::new();
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
    let now = start + TICK_CYCLES * 10;
    engine.tick(now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::PieceStartInPast.as_i32(),
        "PieceStartInPast fault must be latched when piece exceeds drift-budget tolerance"
    );
}

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
    let now = start + TICK_CYCLES;
    engine.tick(now, &shared, &mut storage);

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault expected within drift-budget tolerance"
    );
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece armed but still playing -> retired must be 0"
    );
}

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

    let a_start = TICK_CYCLES;
    let a_duration = 0.001_f32;
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let a_end = a_start + (a_duration * CLOCK_FREQ as f32) as u64;

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

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let d: u64 = (0.010_f32 * CLOCK_FREQ as f32) as u64;

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

    engine.tick(p0_start + d / 2, &shared, &mut storage);
    assert_eq!(
        engine.retired_counts()[0],
        0,
        "piece 0 still playing -> retired must be 0"
    );

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

    engine.tick(p1_start + d / 2, &shared, &mut storage);
    assert_eq!(
        engine.retired_counts()[0],
        1,
        "piece 1 still playing -> retired must be 1"
    );

    engine.tick(p1_start + d + 1, &shared, &mut storage);
    assert_eq!(
        engine.retired_counts()[0],
        2,
        "both pieces finished -> retired must equal sent (2)"
    );
}

#[test]
fn push_pieces_rejects_when_ring_full() {
    let mut engine = make_engine();

    let rc = engine.configure_axis(
        0,
        StepMode::Pulse,
        0.0125,
        4,
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

    for i in 0..4_u64 {
        let piece = const_piece(100_000_000 + i * 520_000, 0.001);
        let rc = engine.push_pieces(0, &[piece], &mut storage);
        assert_eq!(rc, 0, "push {i} must succeed (ring not yet full)");
    }

    let overflow_piece = const_piece(200_000_000, 0.001);
    let rc = engine.push_pieces(0, &[overflow_piece], &mut storage);
    assert!(
        rc < 0,
        "5th push must return a negative error (RING_FULL), got {rc}"
    );
}
