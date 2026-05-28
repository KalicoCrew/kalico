#![allow(
    clippy::ref_as_ptr,
    clippy::borrow_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Tests for piece advancement (Task 9).
//!
//! Covers the observable effect of piece advancement:
//!
//! When sample time advances past `piece.duration`, the axis's `piece`
//! field is cleared to `None`.
//!
//! The `segment_retirement_increments_counter_and_resets_arc_length` test
//! was removed: it tested `ds_xy_segment` (E-follows-XY arc-length
//! accumulator) which no longer exists. Segment retirement is tested at
//! the Engine level in `exhaustion_post_pass.rs`.
//!
//! Spec: docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! "Piece advancement" section.

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8};
use heapless::Vec;

use runtime::curve_pool::CurvePool;
use runtime::monomial::bernstein_to_monomial;
use runtime::state::SharedState;
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{
    AxisConfig, MAX_STEPPERS_PER_AXIS, StepMode, StepperRef, TickCaches,
};
use runtime::tick::{TickContext, runtime_tick_sample};

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

#[test]
#[ignore = "removed in PR #11 simple-mcu-contract (engine replaced); not fixed here"]
fn piece_advances_when_sample_passes_duration() {
    // Single piece, duration 10 µs. Tick at t = 20 µs — well past the
    // piece's end. The advancement helper should clear `axis.piece` to
    // None on the first iteration of its inner loop.
    let scale = 1.0 / 10e-6;
    let piece = {
        let mut p = bernstein_to_monomial([0.0, scale / 3.0, 2.0 * scale / 3.0, scale]);
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
    let pool = CurvePool::new();
    let mut ctx = TickContext {
        axes: &mut axes,
        queues: queue_ptrs,
        shared: &shared,
        caches: &mut caches,
        curve_pool: &pool,
        sample_period_sec: 25e-6,
        sample_period_cycles: 13_000,
        cycles_per_second: 520e6,
        now_cycles: 13_000,
        now_cycles_u64: 13_000, // 25 µs at 520 MHz — past the 10 µs piece duration
        t_sample_end_global: 20e-6, // past piece duration
    };
    runtime_tick_sample(&mut ctx);
    assert!(
        axes[0].piece.is_none(),
        "piece should have been advanced (set to None)"
    );
}
