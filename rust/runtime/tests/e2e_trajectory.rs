//! Analytic ground truth for the magic numbers used in assertions below.
//!
//! ## Linear ramp [0, 1/3, 2/3, 1] mm over 100 ms
//!
//! Bernstein [0, 1/3, 2/3, 1] → unit-interval monomial:
//!   c0 = 0
//!   c1 = 3·(1/3 − 0) = 1
//!   c2 = 3·(2/3 − 2/3 + 0) = 0
//!   c3 = 1 − 2 + 1 − 0 = 0
//!
//! Duration-rescaled (d = 0.1 s):
//!   c1' = 1 / 0.1 = 10 mm/s (constant velocity)
//!
//! So P(t) = 10·t mm, V(t) = 10 mm/s exactly.
//!
//!   P(0.1) = 1.0 mm  → 1/0.0125 = 80 microsteps
//!   P(0.05) = 0.5 mm (mid-point) → 40 microsteps
//!
//! ## Ease ramp [0, 0, 1, 1] mm over 100 ms
//!
//! Bernstein [0, 0, 1, 1]:
//!   c0 = 0, c1_unit = 0, c2_unit = 3, c3_unit = -2
//! Duration-rescaled (d = 0.1):
//!   c2' = 3/0.01 = 300, c3' = -2/0.001 = -2000
//! P(t) = 300t² - 2000t³
//! P(0.05) = 300·0.0025 - 2000·0.000125 = 0.75 - 0.25 = 0.50 mm → 40 steps
//! P(0.1)  = 300·0.01 - 2000·0.001 = 3 - 2 = 1 mm ✓

use core::sync::atomic::Ordering;

use runtime::engine::Engine;
use runtime::piece_ring::PieceEntry;
use runtime::state::{SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

const CLOCK_FREQ: u32 = 520_000_000;
const SAMPLE_RATE: u32 = 40_000;
const TICK_CYCLES: u64 = (CLOCK_FREQ / SAMPLE_RATE) as u64;
const MICROSTEP_DISTANCE: f32 = 0.0125;
const TARGET_MM: f32 = 1.0;
const DURATION_S: f32 = 0.1;
const EXPECTED_STEPS: i32 = 80;

fn make_engine() -> Engine {
    Engine::new(CLOCK_FREQ, SAMPLE_RATE)
}

fn make_storage() -> Vec<PieceEntry> {
    vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0,
        };
        TOTAL_RING_PIECES
    ]
}

fn pulse_binding() -> StepperBindingRust {
    StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    }
}

fn configure_axis0(engine: &mut Engine) {
    let rc = engine.configure_axis(
        0,
        StepMode::Pulse,
        MICROSTEP_DISTANCE,
        64,
        &[pulse_binding()],
        TOTAL_RING_PIECES,
    );
    assert_eq!(rc, 0, "configure_axis failed");
}

/// The queues are heap-allocated so they stay at a fixed address for Engine's
/// raw pointer table; moving them after `test_install_step_queues` would be UB.
fn setup_queues(engine: &mut Engine) -> (Box<[StepQueue; MAX_AXES]>, SharedState) {
    let mut qs: Box<[StepQueue; MAX_AXES]> = Box::new(core::array::from_fn(|_| StepQueue::new()));
    let mut ptrs: [*mut StepQueue; MAX_AXES] = [core::ptr::null_mut(); MAX_AXES];
    for (i, q) in qs.iter_mut().enumerate() {
        ptrs[i] = q as *mut StepQueue;
    }
    engine.test_install_step_queues(ptrs);
    (qs, SharedState::new())
}

/// Simulates the C ISR consuming step entries: advances `head` to match `tail`.
/// Without this, a long trajectory overflows `StepQueue` before the piece ends.
fn drain_queue(qs: &mut Box<[StepQueue; MAX_AXES]>) {
    if let Some(q) = qs.first_mut() {
        q.head = q.tail;
    }
}

#[test]
fn e2e_linear_ramp_full_window() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let mut storage = make_storage();
    let (mut qs, shared) = setup_queues(&mut engine);

    let piece = PieceEntry {
        start_time: TICK_CYCLES,
        coeffs: [0.0, TARGET_MM / 3.0, 2.0 * TARGET_MM / 3.0, TARGET_MM],
        duration: DURATION_S,
        _reserved: 0,
    };
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let duration_cycles: u64 = (DURATION_S * CLOCK_FREQ as f32) as u64;
    let piece_end_cycles = TICK_CYCLES + duration_cycles;
    let mid_now = TICK_CYCLES + duration_cycles / 2;
    let mut saw_mid = false;
    let mut mid_position_count: i32 = 0;
    let mut saw_nonzero_steps = false;

    let mut now = TICK_CYCLES;
    while now <= piece_end_cycles + TICK_CYCLES {
        engine.tick(now, &shared, &mut storage);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            0,
            "unexpected fault at now={now} (error {})",
            shared.last_error.load(Ordering::Acquire)
        );

        let current_pos = engine
            .stepping_axes
            .first()
            .and_then(|s| s.as_ref())
            .and_then(|a| a.steppers.first())
            .map(|s| s.position_count.load(Ordering::Acquire))
            .unwrap_or(0);

        if current_pos > 0 {
            saw_nonzero_steps = true;
        }

        if !saw_mid && now >= mid_now {
            saw_mid = true;
            mid_position_count = current_pos;
        }

        drain_queue(&mut qs);
        now += TICK_CYCLES;
    }

    let final_pos = engine
        .stepping_axes
        .first()
        .and_then(|s| s.as_ref())
        .and_then(|a| a.steppers.first())
        .map(|s| s.position_count.load(Ordering::Acquire))
        .expect("axis 0 stepper must exist");

    assert!(
        (final_pos - EXPECTED_STEPS).abs() <= 1,
        "final position_count={final_pos} is more than ±1 away from expected {EXPECTED_STEPS}. \
         P(0.1 s) for Bernstein [0,1/3,2/3,1] must equal 1 mm exactly. \
         This indicates a monomial rescaling, t-domain, or p_prev carry-over bug."
    );

    let expected_mid_steps: i32 = (TARGET_MM / 2.0 / MICROSTEP_DISTANCE).round() as i32;
    assert!(
        (mid_position_count - expected_mid_steps).abs() <= 2,
        "mid-point position_count={mid_position_count} steps, expected ~{expected_mid_steps}. \
         Analytic: P(0.05) = 0.5 mm for Bernstein [0,1/3,2/3,1]. ±2 step discretisation tolerance."
    );

    assert!(
        saw_nonzero_steps,
        "position_count was 0 throughout the entire trajectory. \
         The Horner-eval → step-count path was not exercised. \
         This test is designed to distinguish MOVING from zero-motion pieces."
    );

    assert_eq!(
        engine.retired_counts()[0],
        1,
        "retired_counts()[0] must be 1 after the piece window closes. \
         Got {} — the retire cursor did not advance.",
        engine.retired_counts()[0]
    );
}

#[test]
fn e2e_ease_ramp_full_window() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let mut storage = make_storage();
    let (mut qs, shared) = setup_queues(&mut engine);

    let piece = PieceEntry {
        start_time: TICK_CYCLES,
        coeffs: [0.0, 0.0, TARGET_MM, TARGET_MM],
        duration: DURATION_S,
        _reserved: 0,
    };
    let rc = engine.push_pieces(0, &[piece], &mut storage);
    assert_eq!(rc, 0);

    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let duration_cycles: u64 = (DURATION_S * CLOCK_FREQ as f32) as u64;
    let piece_end_cycles = TICK_CYCLES + duration_cycles;
    let mid_now = TICK_CYCLES + duration_cycles / 2;
    let mut saw_mid = false;
    let mut mid_position_count: i32 = 0;

    let mut now = TICK_CYCLES;
    while now <= piece_end_cycles + TICK_CYCLES {
        engine.tick(now, &shared, &mut storage);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            0,
            "fault during ease ramp at now={now}"
        );
        let current_pos = engine
            .stepping_axes
            .first()
            .and_then(|s| s.as_ref())
            .and_then(|a| a.steppers.first())
            .map(|s| s.position_count.load(Ordering::Acquire))
            .unwrap_or(0);
        if !saw_mid && now >= mid_now {
            saw_mid = true;
            mid_position_count = current_pos;
        }
        drain_queue(&mut qs);
        now += TICK_CYCLES;
    }

    let final_pos = engine
        .stepping_axes
        .first()
        .and_then(|s| s.as_ref())
        .and_then(|a| a.steppers.first())
        .map(|s| s.position_count.load(Ordering::Acquire))
        .expect("axis 0 stepper must exist");

    assert!(
        (final_pos - EXPECTED_STEPS).abs() <= 1,
        "ease ramp: final position_count={final_pos}, expected {EXPECTED_STEPS} ±1. \
         P(d) for Bernstein [0,0,1,1] with d=0.1 s must equal 1 mm."
    );

    let expected_mid_steps: i32 = (TARGET_MM / 2.0 / MICROSTEP_DISTANCE).round() as i32;
    assert!(
        (mid_position_count - expected_mid_steps).abs() <= 2,
        "ease ramp: mid-point position_count={mid_position_count}, expected ~{expected_mid_steps}. \
         Analytic P(0.05) = 0.50 mm for Bernstein [0,0,1,1]."
    );

    assert_eq!(
        engine.retired_counts()[0],
        1,
        "ease ramp: retired_counts()[0] must be 1 after window closes"
    );
}

#[test]
fn e2e_two_consecutive_moving_pieces() {
    let mut engine = make_engine();
    configure_axis0(&mut engine);
    let mut storage = make_storage();
    let (mut qs, shared) = setup_queues(&mut engine);

    let half_mm = TARGET_MM / 2.0;
    let half_dur = DURATION_S / 2.0;
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let half_dur_cycles: u64 = (half_dur * CLOCK_FREQ as f32) as u64;

    let a_start = TICK_CYCLES;
    let b_start = a_start + half_dur_cycles;

    let piece_a = PieceEntry {
        start_time: a_start,
        coeffs: [0.0, half_mm / 3.0, 2.0 * half_mm / 3.0, half_mm],
        duration: half_dur,
        _reserved: 0,
    };
    let piece_b = PieceEntry {
        start_time: b_start,
        coeffs: [
            half_mm,
            half_mm + (half_mm / 3.0),
            half_mm + 2.0 * (half_mm / 3.0),
            TARGET_MM,
        ],
        duration: half_dur,
        _reserved: 0,
    };

    let rc = engine.push_pieces(0, &[piece_a, piece_b], &mut storage);
    assert_eq!(rc, 0, "push_pieces failed");

    let b_end = b_start + half_dur_cycles;
    let mut now = a_start;
    while now <= b_end + TICK_CYCLES {
        engine.tick(now, &shared, &mut storage);
        assert_eq!(
            shared.last_error.load(Ordering::Acquire),
            0,
            "fault at now={now}"
        );
        drain_queue(&mut qs);
        now += TICK_CYCLES;
    }

    let final_pos = engine
        .stepping_axes
        .first()
        .and_then(|s| s.as_ref())
        .and_then(|a| a.steppers.first())
        .map(|s| s.position_count.load(Ordering::Acquire))
        .expect("axis 0 stepper must exist");

    assert!(
        (final_pos - EXPECTED_STEPS).abs() <= 2,
        "two-piece total: position_count={final_pos}, expected {EXPECTED_STEPS} ±2. \
         Each linear piece contributes 40 steps; combined = 80."
    );

    assert_eq!(
        engine.retired_counts()[0],
        2,
        "two pieces must both retire; retired_counts={}",
        engine.retired_counts()[0]
    );
}
