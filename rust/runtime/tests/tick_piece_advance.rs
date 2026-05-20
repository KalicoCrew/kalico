//! Tests for piece advancement + segment retirement (Task 9).
//!
//! Two-test suite covering the two observable effects of the redesign:
//!
//! 1. When sample time advances past `piece.duration`, the axis's
//!    `piece` field is cleared to `None` (host refills via Task 11).
//! 2. When all four axes are idle AND the cached `ds_xy_segment` is
//!    non-zero (signalling the tail of an active segment), Phase 5
//!    increments `retired_through_segment_id` and resets the
//!    arc-length accumulator.
//!
//! Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! "Piece advancement" + "Segment retirement" sections.

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

use runtime::monomial::bernstein_to_monomial;
use runtime::state::SharedState;
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{
    AxisConfig, StepMode, StepperRef, TickCaches, MAX_STEPPERS_PER_AXIS,
};
use runtime::tick::{runtime_tick_sample, TickContext};

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

#[test]
fn piece_advances_when_sample_passes_duration() {
    // Single piece, duration 10 µs. Tick at t = 20 µs — well past the
    // piece's end. The advancement helper should clear `axis.piece` to
    // None on the first iteration of its inner loop.
    let scale = 1.0 / 10e-6;
    let piece = {
        let mut p =
            bernstein_to_monomial([0.0, scale / 3.0, 2.0 * scale / 3.0, scale]);
        p.duration = 10e-6;
        p
    };
    let mut steppers = Vec::<StepperRef, MAX_STEPPERS_PER_AXIS>::new();
    let _ = steppers.push(make_stepper());
    let mut axes = [
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers,
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
        &mut queues[0] as *mut _,
        &mut queues[1] as *mut _,
        &mut queues[2] as *mut _,
        &mut queues[3] as *mut _,
    ];
    let shared = SharedState::new();
    let mut caches = TickCaches::new();
    let mut ctx = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        sample_period_sec: 25e-6,
        sample_period_cycles: 13_000,
        cycles_per_second: 520e6,
        k_xy: 1.0,
        advance_accel: 0.0,
        advance_decel: 0.0,
        now_cycles: 0,
        t_sample_end_global: 20e-6, // past piece duration
    };
    runtime_tick_sample(&mut ctx);
    assert!(
        axes[0].piece.is_none(),
        "piece should have been advanced (set to None)"
    );
}

#[test]
fn segment_retirement_increments_counter_and_resets_arc_length() {
    // Two-phase test:
    //
    // - Sample 1 (t = 25 µs): active pieces on AXIS_A and AXIS_B. The
    //   advancement check runs (t_local == duration, so no advance),
    //   the cubic gets evaluated, and `ds_xy_segment` accumulates a
    //   positive value via Phase 2.
    // - Sample 2 (t = 50 µs): both pieces' start times haven't moved,
    //   so `t_local = 50 µs > 25 µs duration` → the advancement helper
    //   clears both `piece` fields to None. Phase 2 sees `xy_active ==
    //   false` and skips its update. Phase 5 then observes
    //   `any_active == false` AND `ds_xy_segment > 0` → it bumps
    //   `retired_through_segment_id` and zeroes the accumulator.
    let scale = 1.0 / 25e-6;
    let piece = {
        let mut p =
            bernstein_to_monomial([0.0, scale / 3.0, 2.0 * scale / 3.0, scale]);
        p.duration = 25e-6;
        p
    };
    let mut steppers_a = Vec::<StepperRef, MAX_STEPPERS_PER_AXIS>::new();
    let _ = steppers_a.push(make_stepper());
    let mut steppers_b = Vec::<StepperRef, MAX_STEPPERS_PER_AXIS>::new();
    let _ = steppers_b.push(make_stepper());
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
        AxisConfig {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: steppers_b,
            curve_handle: None,
            piece_cursor: 0,
            piece: Some(piece),
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.25,
        },
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
        &mut queues[0] as *mut _,
        &mut queues[1] as *mut _,
        &mut queues[2] as *mut _,
        &mut queues[3] as *mut _,
    ];
    let shared = SharedState::new();
    let mut caches = TickCaches::new();

    // First tick: at the end of the piece's duration → cubic evaluation
    // accumulates a positive `ds_xy_segment` (the cubic's terminal
    // velocity is `scale > 0`).
    let mut ctx1 = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        sample_period_sec: 25e-6,
        sample_period_cycles: 13_000,
        cycles_per_second: 520e6,
        k_xy: 1.0,
        advance_accel: 0.0,
        advance_decel: 0.0,
        now_cycles: 0,
        t_sample_end_global: 25e-6,
    };
    runtime_tick_sample(&mut ctx1);
    assert!(
        caches.ds_xy_segment > 0.0,
        "first tick should accumulate ds_xy_segment, got {}",
        caches.ds_xy_segment
    );
    let id_before = shared.retired_through_segment_id.load(Ordering::Acquire);

    // Second tick: t = 50 µs > 25 µs duration → both axes get cleared
    // by `advance_piece_if_needed`. Phase 5 retires the segment.
    let mut ctx2 = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        sample_period_sec: 25e-6,
        sample_period_cycles: 13_000,
        cycles_per_second: 520e6,
        k_xy: 1.0,
        advance_accel: 0.0,
        advance_decel: 0.0,
        now_cycles: 0,
        t_sample_end_global: 50e-6,
    };
    runtime_tick_sample(&mut ctx2);

    assert_eq!(
        caches.ds_xy_segment, 0.0,
        "Phase 5 should reset ds_xy_segment after retirement"
    );
    let id_after = shared.retired_through_segment_id.load(Ordering::Acquire);
    assert_eq!(
        id_after,
        id_before + 1,
        "retirement counter should increment by 1"
    );
}
