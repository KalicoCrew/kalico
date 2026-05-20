//! Integration smoke tests for `runtime::tick::dispatch_axis`.
//!
//! Lives in `tests/` so it can be exercised even when the broader
//! library test build is broken by unrelated engine.rs type drift. The
//! finer-grained `#[cfg(test)] mod tests` blocks inside `src/tick.rs`
//! re-validate the same invariants once the lib-test path compiles.

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

use runtime::error::FaultCode;
use runtime::monomial::BezierPieceMonomial;
use runtime::state::SharedState;
use runtime::step_queue::{StepQueue, STEP_QUEUE_DEPTH};
use runtime::stepping_state::{AxisConfig, StepMode, StepperRef, MAX_STEPPERS_PER_AXIS};
use runtime::tick::dispatch_axis;

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

fn make_axis(mode: StepMode, microstep_distance: f32) -> AxisConfig {
    let mut steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers.push(make_stepper());
    AxisConfig {
        mode: AtomicU8::new(mode as u8),
        steppers,
        piece: None::<BezierPieceMonomial>,
        piece_start_time_cycles: 0,
        last_step_count: 0,
        microstep_distance,
        extrusion_per_xy_mm: 0.0,
    }
}

#[test]
fn pulse_zero_motion_no_steps_scheduled() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0, &mut axis, q_ptr, &shared,
        /* p_end */ 0.0,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    assert_eq!(q.tail, q.head);
    assert_eq!(axis.last_step_count, 0);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn pulse_positive_motion_enqueues_n_steps() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0, &mut axis, q_ptr, &shared,
        /* p_end */ 0.05,
        /* v_end */ 2000.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 1_000,
        /* cycles_per_second */ 520_000_000.0,
    );

    assert_eq!(q.tail.wrapping_sub(q.head), 4);
    assert_eq!(axis.last_step_count, 4);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 4);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

/// Regression: when only some of the requested pushes fit in the queue
/// (partial overflow), `position_count` and `last_step_count` MUST
/// reflect the steps that landed in the queue — those WILL drive
/// physical GPIO toggles regardless of fault state. Previously the bump
/// happened only after the loop, so a partial-overflow desynced host
/// position from physical reality.
#[test]
fn pulse_partial_push_commits_position_count_for_pushed_steps() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    // Leave exactly one slot free: depth 32, fill 31. The first push in
    // dispatch_pulse succeeds, the second hits StepQueueFull.
    q.tail = (STEP_QUEUE_DEPTH as u16) - 1;
    q.head = 0;
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0, &mut axis, q_ptr, &shared,
        /* p_end */ 0.05, // 4 microsteps requested
        /* v_end */ 2000.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 1_000,
        /* cycles_per_second */ 520_000_000.0,
    );

    // Exactly one push landed — the rest overflowed.
    assert_eq!(
        q.tail.wrapping_sub(q.head),
        STEP_QUEUE_DEPTH as u16,
        "queue should be exactly full (31 prefill + 1 push)"
    );
    // last_step_count must reflect the partial commit, not the full
    // requested target (which would have been 4).
    assert_eq!(
        axis.last_step_count, 1,
        "last_step_count must reflect pushes that landed, not requested target"
    );
    // position_count must bump by exactly the number of successful
    // pushes — not 0 (the pre-fix bug) and not 4 (the requested count).
    assert_eq!(
        axis.steppers[0].position_count.load(Ordering::Acquire),
        1,
        "position_count must commit for pushed steps before fault"
    );
    // And the fault is latched.
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepQueueOverflow.as_i32()
    );
}

#[test]
fn pulse_queue_overflow_latches_fault() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    // Pre-fill the SPSC ring head/tail so any push fails immediately.
    // STEP_QUEUE_DEPTH = 32; setting tail = 32 and head = 0 marks "full".
    q.tail = 32;
    q.head = 0;
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        2, &mut axis, q_ptr, &shared,
        /* p_end */ 0.0125, // 1 step
        /* v_end */ 1000.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepQueueOverflow.as_i32()
    );
    assert_eq!(shared.queue_overflow_count[2].load(Ordering::Acquire), 1);
}

#[test]
fn phase_mode_updates_coil_state_no_queue_writes() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);

    // 256 microsteps → PHASE_LUT[256] = (0, 248).
    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0, &mut axis, q_ptr, &shared,
        /* p_end */ 256.0 * 0.0125,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    assert_eq!(q.tail, q.head, "phase mode must not enqueue step pulses");
    assert_eq!(axis.last_step_count, 256);
    assert_eq!(axis.steppers[0].last_coil_A.load(Ordering::Acquire), 0);
    assert_eq!(axis.steppers[0].last_coil_B.load(Ordering::Acquire), 248);
    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        256
    );
    assert_eq!(
        axis.steppers[0].position_count.load(Ordering::Acquire),
        256
    );
}

#[test]
fn phase_mode_honors_phase_offset() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);
    axis.steppers[0]
        .phase_offset_microsteps
        .store(7, Ordering::Release);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0, &mut axis, q_ptr, &shared,
        /* p_end */ 256.0 * 0.0125,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        263
    );
    assert_eq!(
        axis.steppers[0].position_count.load(Ordering::Acquire),
        263
    );
}
