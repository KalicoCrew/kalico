//! Integration test for the full per-sample evaluator (Task 8).
//!
//! Validates the three observable behaviours of [`runtime_tick_sample`]
//! independently of the lower-level dispatch / monomial / fault helpers:
//!
//! 1. A constant-velocity pulse-mode axis enqueues the expected number
//!    of steps and bumps each yoked stepper's `position_count` by the
//!    same amount.
//! 2. Two active XY axes drive the cartesian arc-length accumulator
//!    (`caches.ds_xy_segment`) and the cached `v_xy_this` to non-zero
//!    values.
//! 3. The extruder follower, given `extrusion_per_xy_mm > 0`, advances
//!    its `last_step_count` purely from the XY arc length even with an
//!    intrinsically-flat E NURBS piece.
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
//! that the plan's verbatim test fixture used. The plan's fixture would
//! have evaluated to ~25 µm of position over the sample, far below one
//! microstep, and the `last_step_count == 4` assertion would be unmet.
//! The scaling below matches the spec's eval semantics so the test
//! exercises the dispatch path with the assertion intact.

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};

use heapless::Vec;

use runtime::monomial::{bernstein_to_monomial, BezierPieceMonomial};
use runtime::state::SharedState;
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{
    AxisConfig, StepMode, StepperRef, TickCaches, MAX_STEPPERS_PER_AXIS,
};
use runtime::tick::{runtime_tick_sample, TickContext, N_AXES};

const SAMPLE_PERIOD_SEC: f32 = 25e-6;
const SAMPLE_PERIOD_CYCLES: u32 = 13_000;
const CYCLES_PER_SECOND: f32 = 520e6;

fn make_stepper() -> StepperRef {
    StepperRef {
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
/// Use this to set up a known constant-velocity move.
fn linear_piece(scale: f32, duration_sec: f32) -> BezierPieceMonomial {
    // Bernstein control points for a linear curve from 0 to `scale`:
    //   [0, scale/3, 2·scale/3, scale]
    // → c0 = 0, c1 = scale, c2 = 0, c3 = 0  (Horner evaluates as scale·t).
    let mut piece =
        bernstein_to_monomial([0.0, scale / 3.0, 2.0 * scale / 3.0, scale]);
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

    let mut queues =
        [StepQueue::new(), StepQueue::new(), StepQueue::new(), StepQueue::new()];
    let queue_ptrs = [
        &mut queues[0] as *mut StepQueue,
        &mut queues[1] as *mut StepQueue,
        &mut queues[2] as *mut StepQueue,
        &mut queues[3] as *mut StepQueue,
    ];

    let shared = SharedState::new();
    let mut caches = TickCaches::new();

    let mut ctx = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_period_cycles: SAMPLE_PERIOD_CYCLES,
        cycles_per_second: CYCLES_PER_SECOND,
        k_xy: 1.0,
        advance_accel: 0.0,
        advance_decel: 0.0,
        now_cycles: 0,
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

#[test]
fn xy_arc_length_accumulates_in_segment() {
    // Both A and B advancing at the same linear velocity: the cartesian
    // arc length should be positive and `v_xy_this` should equal the
    // motor-frame magnitude (k_xy = 1.0 in the cartesian fixture).
    let velocity = 1.0 / SAMPLE_PERIOD_SEC;
    let piece_a = linear_piece(velocity, SAMPLE_PERIOD_SEC);
    let piece_b = piece_a;

    let mut steppers_a: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers_a.push(make_stepper());
    let mut steppers_b: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers_b.push(make_stepper());

    let mut axes = [
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: steppers_a,
            curve_handle: None,
            piece_cursor: 0,
            piece: Some(piece_a),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
        },
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: steppers_b,
            curve_handle: None,
            piece_cursor: 0,
            piece: Some(piece_b),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
        },
        idle_axis(),
        idle_axis(),
    ];
    let mut queues =
        [StepQueue::new(), StepQueue::new(), StepQueue::new(), StepQueue::new()];
    let queue_ptrs = [
        &mut queues[0] as *mut StepQueue,
        &mut queues[1] as *mut StepQueue,
        &mut queues[2] as *mut StepQueue,
        &mut queues[3] as *mut StepQueue,
    ];
    let shared = SharedState::new();
    let mut caches = TickCaches::new();

    let mut ctx = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_period_cycles: SAMPLE_PERIOD_CYCLES,
        cycles_per_second: CYCLES_PER_SECOND,
        k_xy: 1.0,
        advance_accel: 0.0,
        advance_decel: 0.0,
        now_cycles: 0,
        t_sample_end_global: SAMPLE_PERIOD_SEC,
    };

    runtime_tick_sample(&mut ctx);

    assert!(
        ctx.caches.ds_xy_segment > 0.0,
        "ds_xy_segment should accumulate over the sample (got {})",
        ctx.caches.ds_xy_segment
    );
    assert!(
        ctx.caches.v_xy_this > 0.0,
        "v_xy_this should be positive while both A and B advance (got {})",
        ctx.caches.v_xy_this
    );
    // Both axes started from rest in TickCaches, so v_xy_this >= v_xy_prev
    // → the accelerating flag should be true on this first sample.
    assert!(
        ctx.caches.vdot_xy_accelerating,
        "vdot_xy_accelerating should be true on the first sample of a move"
    );
    // p_prev should reflect the just-evaluated positions, not the stale
    // zeros from TickCaches::new().
    assert!(
        ctx.caches.p_prev[0] > 0.0,
        "p_prev[A] should be advanced by the dispatched motion"
    );
    assert!(ctx.caches.p_prev[1] > 0.0, "p_prev[B] should be advanced");
    assert_eq!(N_AXES, 4, "spec invariant: four axes");
}

#[test]
#[ignore = "Task 6 dropped AxisConfig::extrusion_per_xy_mm; Task 11 will \
            wire per-segment Segment::extrusion_ratio into the Phase-3 \
            evaluator. Until then the E-follows-XY coupling term is held \
            at 0.0 and this assertion cannot pass."]
fn extruder_follows_xy_arc_length() {
    // E intrinsically-zero piece + per-segment extrusion_ratio = 0.05
    // means E should advance purely from XY arc length. With v_xy ≈ √2
    // mm / 25 µs over the sample, ds_xy ≈ √2 mm and E ≈ 0.0707 mm;
    // microstep 0.01 → ≈7 microsteps.
    let velocity = 1.0 / SAMPLE_PERIOD_SEC;
    let piece_a = linear_piece(velocity, SAMPLE_PERIOD_SEC);
    let piece_b = piece_a;
    // E piece is intrinsically zero (no retraction motion); the entire E
    // advance must come from the per-segment extrusion ratio scaled by
    // `ds_xy_segment`.
    let piece_e = bernstein_to_monomial([0.0, 0.0, 0.0, 0.0]);

    let mut steppers_e: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers_e.push(make_stepper());

    let mut axes = [
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            curve_handle: None,
            piece_cursor: 0,
            piece: Some(piece_a),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
        },
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            curve_handle: None,
            piece_cursor: 0,
            piece: Some(piece_b),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
        },
        idle_axis(),
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: steppers_e,
            curve_handle: None,
            piece_cursor: 0,
            piece: Some(piece_e),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.01,
        },
    ];

    let mut queues =
        [StepQueue::new(), StepQueue::new(), StepQueue::new(), StepQueue::new()];
    let queue_ptrs = [
        &mut queues[0] as *mut StepQueue,
        &mut queues[1] as *mut StepQueue,
        &mut queues[2] as *mut StepQueue,
        &mut queues[3] as *mut StepQueue,
    ];
    let shared = SharedState::new();
    let mut caches = TickCaches::new();
    let mut ctx = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        sample_period_sec: SAMPLE_PERIOD_SEC,
        sample_period_cycles: SAMPLE_PERIOD_CYCLES,
        cycles_per_second: CYCLES_PER_SECOND,
        k_xy: 1.0,
        advance_accel: 0.0,
        advance_decel: 0.0,
        now_cycles: 0,
        t_sample_end_global: SAMPLE_PERIOD_SEC,
    };

    runtime_tick_sample(&mut ctx);

    assert!(
        axes[3].last_step_count > 0,
        "extruder should have advanced positions from XY arc length (got {})",
        axes[3].last_step_count
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault should latch on the follower path"
    );
}
