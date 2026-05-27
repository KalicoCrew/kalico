//! Integration test for the full per-sample evaluator (Task 8).
//!
//! Validates the observable behaviours of [`runtime_tick_sample`]
//! independently of the lower-level dispatch / monomial / fault helpers:
//!
//! 1. A constant-velocity pulse-mode axis enqueues the expected number
//!    of steps and bumps each yoked stepper's `position_count` by the
//!    same amount.
//! 2. All four axes (A, B, Z, E) are evaluated identically — no
//!    E-follows-XY arc-length integration, no PA, no XY-derived quantities.
//!    The `xy_arc_length_accumulates_in_segment` and
//!    `extruder_follows_xy_arc_length` tests were removed when those fields
//!    were deleted from `TickContext` and `TickCaches`.
//!
//! ### Plan deviation — polynomial coefficient scaling
//!
//! The spec uses `t_local` in seconds as the cubic Bezier polynomial
//! argument (see `docs/superpowers/specs/2026-05-19-stepping-redesign-design.md`
//! §"Pseudocode identifiers"). That means the slicer is responsible for
//! baking the time-domain scale into the control points themselves — the
//! evaluator does *not* normalize by `piece.duration`.
//!
//! Consequence: a 1 mm move over a 25 µs sample with linear velocity
//! ~40 m/s (40000 mm/s) needs Bernstein control points
//! `[0, 1/3, 2/3, 1] · 40000`, not the unit-interval `[0, 1/3, 2/3, 1]`
//! that the plan's verbatim test fixture used.

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};

use heapless::Vec;

use runtime::curve_pool::CurvePool;
use runtime::monomial::{BezierPieceMonomial, bernstein_to_monomial};
use runtime::state::SharedState;
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{
    AxisConfig, MAX_STEPPERS_PER_AXIS, StepMode, StepperRef, TickCaches,
};
use runtime::tick::{N_AXES, TickContext, runtime_tick_sample};

const SAMPLE_PERIOD_SEC: f32 = 25e-6;
const SAMPLE_PERIOD_CYCLES: u32 = 13_000;
const CYCLES_PER_SECOND: f32 = 520e6;

fn make_stepper() -> StepperRef {
    StepperRef {
        stepper_oid: 0,
        position_count: AtomicI32::new(0),
        tmc_cs_oid: None,
        last_coil_A: AtomicI16::new(0),
        last_coil_B: AtomicI16::new(0),
        phase_offset_microsteps: AtomicI32::new(0),
        phase_offset_target: AtomicI32::new(0),
        last_phase_target: AtomicI32::new(0),
    }
}

fn idle_axis() -> AxisConfig {
    AxisConfig {
        mode: AtomicU8::new(StepMode::Pulse as u8),
        steppers: Vec::new(),
        curve_handle: None,
        piece_cursor: 0,
        piece: None,
        piece_start_time_cycles: 0,
        last_step_count: 0,
        microstep_distance: 0.25,
    }
}

/// Linear motion piece: `P(t) = scale · t` (so velocity is `scale`).
fn linear_piece(scale: f32, duration_sec: f32) -> BezierPieceMonomial {
    let mut piece = bernstein_to_monomial([0.0, scale / 3.0, 2.0 * scale / 3.0, scale]);
    piece.duration = duration_sec;
    piece
}

#[test]
fn constant_velocity_produces_expected_step_count() {
    // 1 mm over a 25 µs sample = 40000 mm/s linear velocity.
    // With 0.25 mm/microstep, expect 4 steps.
    let velocity = 1.0 / SAMPLE_PERIOD_SEC; // 40 m/s
    let piece = linear_piece(velocity, SAMPLE_PERIOD_SEC);

    let mut steppers_a: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers_a.push(make_stepper());

    let mut axes = [
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: steppers_a,
            curve_handle: None,
            piece_cursor: 0,
            piece: Some(piece),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
        },
        idle_axis(),
        idle_axis(),
        idle_axis(),
    ];

    let mut queues = [
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
    ];
    let queue_ptrs = [
        &mut queues[0] as *mut StepQueue,
        &mut queues[1] as *mut StepQueue,
        &mut queues[2] as *mut StepQueue,
        &mut queues[3] as *mut StepQueue,
    ];

    let shared = SharedState::new();
    let mut caches = TickCaches::new();
    let pool = CurvePool::new();

    let mut ctx = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        curve_pool: &pool,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_period_cycles: SAMPLE_PERIOD_CYCLES,
        cycles_per_second: CYCLES_PER_SECOND,
        now_cycles: SAMPLE_PERIOD_CYCLES,
        now_cycles_u64: SAMPLE_PERIOD_CYCLES as u64,
        t_sample_end_global: SAMPLE_PERIOD_SEC,
    };

    runtime_tick_sample(&mut ctx);

    assert_eq!(
        axes[0].last_step_count, 4,
        "expected 4 microsteps over the 1 mm sample"
    );
    assert_eq!(
        axes[0].steppers[0].position_count.load(Ordering::Acquire),
        4,
        "stepper position_count should reflect the dispatched steps"
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault should latch on a clean constant-velocity sample"
    );
}

/// All four axes are evaluated uniformly. Verify that an E-axis Bezier piece
/// is dispatched exactly like any other axis — no XY-coupling, no PA.
#[test]
fn e_axis_evaluated_uniformly_like_other_axes() {
    let velocity = 1.0 / SAMPLE_PERIOD_SEC;
    let piece_e = linear_piece(velocity, SAMPLE_PERIOD_SEC);

    let mut steppers_e: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers_e.push(make_stepper());

    let mut axes = [
        idle_axis(),
        idle_axis(),
        idle_axis(),
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: steppers_e,
            curve_handle: None,
            piece_cursor: 0,
            piece: Some(piece_e),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
        },
    ];

    let mut queues = [
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
    ];
    let queue_ptrs = [
        &mut queues[0] as *mut StepQueue,
        &mut queues[1] as *mut StepQueue,
        &mut queues[2] as *mut StepQueue,
        &mut queues[3] as *mut StepQueue,
    ];

    let shared = SharedState::new();
    let mut caches = TickCaches::new();
    let pool = CurvePool::new();

    let mut ctx = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        curve_pool: &pool,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_period_cycles: SAMPLE_PERIOD_CYCLES,
        cycles_per_second: CYCLES_PER_SECOND,
        now_cycles: SAMPLE_PERIOD_CYCLES,
        now_cycles_u64: SAMPLE_PERIOD_CYCLES as u64,
        t_sample_end_global: SAMPLE_PERIOD_SEC,
    };

    runtime_tick_sample(&mut ctx);

    // Check p_prev[E] before dropping ctx (it holds the mutable borrow on axes).
    let p_prev_e = ctx.caches.p_prev[3];
    // Drop ctx to release the mutable borrow on axes before reading axes[3].
    drop(ctx);

    assert_eq!(
        axes[3].last_step_count, 4,
        "E axis (idx 3) should produce 4 microsteps like any other axis"
    );
    assert_eq!(
        axes[3].steppers[0].position_count.load(Ordering::Acquire),
        4,
        "E stepper position_count should reflect dispatched steps"
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault should latch on uniform E evaluation"
    );
    // p_prev for E should be updated after the tick.
    assert!(
        p_prev_e > 0.0,
        "p_prev[E] should be advanced by the dispatched motion"
    );
    assert_eq!(N_AXES, 4, "spec invariant: four axes");
}
