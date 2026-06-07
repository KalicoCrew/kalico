#![allow(clippy::indexing_slicing)]

use super::{DISPLACEMENT_THRESHOLD_MM, dispatch_axis};
use crate::state::SharedState;
use crate::step_queue::StepQueue;
use crate::stepping_state::{AxisConfig, StepMode, StepperRef};
use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

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

fn make_axis(mode: StepMode, microstep_distance: f32) -> AxisConfig {
    let mut steppers: Vec<StepperRef, 4> = Vec::new();
    let _ = steppers.push(make_stepper());
    AxisConfig {
        mode: AtomicU8::new(mode as u8),
        steppers,
        microstep_distance,
        ..AxisConfig::new_unconfigured()
    }
}

#[test]
fn pulse_zero_motion_no_steps_scheduled() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        q_ptr,
        &shared,
        /* p_end */ 0.0,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    assert_eq!(q.tail, q.head, "no steps should be enqueued");
    assert_eq!(axis.last_step_count, 0);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        0,
        "no fault should latch"
    );
}

#[test]
fn pulse_positive_motion_enqueues_n_steps() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        q_ptr,
        &shared,
        /* p_end */ 0.05,
        /* v_end */ 2000.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 1_000,
        /* cycles_per_second */ 520_000_000.0,
    );

    let enq = q.tail.wrapping_sub(q.head);
    assert_eq!(enq, 4, "expected 4 step entries, got {enq}");
    assert_eq!(axis.last_step_count, 4);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 4);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn pulse_below_displacement_threshold_uses_uniform_fallback() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    axis.last_step_count = -2;
    let tiny = DISPLACEMENT_THRESHOLD_MM / 10.0;

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        q_ptr,
        &shared,
        /* p_end */ tiny,
        /* v_end */ 0.0,
        /* p_sample_start */ -tiny,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    let enq = q.tail.wrapping_sub(q.head);
    assert_eq!(enq, 2);
    assert_eq!(axis.last_step_count, 0);
}

#[test]
fn phase_mode_updates_coil_state_no_queue_writes() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        q_ptr,
        &shared,
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
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 256);
}

#[test]
fn phase_mode_ramps_offset_toward_target_at_max_per_sample() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);
    axis.steppers[0]
        .phase_offset_target
        .store(10, Ordering::Release);
    shared
        .max_phase_offset_ramp_per_sample
        .store(4, Ordering::Release);

    let q_ptr: *mut StepQueue = &mut q;
    for expected in [4_i32, 8, 10] {
        dispatch_axis(
            0,
            &mut axis,
            q_ptr,
            &shared,
            /* p_end */ 256.0 * 0.0125,
            /* v_end */ 0.0,
            /* p_sample_start */ 0.0,
            /* sample_period_sec */ 25e-6,
            /* sample_start_cycles */ 0,
            /* cycles_per_second */ 520_000_000.0,
        );
        assert_eq!(
            axis.steppers[0]
                .phase_offset_microsteps
                .load(Ordering::Acquire),
            expected,
            "ramp should advance to {expected}",
        );
    }
}

#[test]
fn phase_mode_ramp_disabled_when_max_per_sample_is_zero() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Phase, 0.0125);
    axis.steppers[0]
        .phase_offset_microsteps
        .store(3, Ordering::Release);
    axis.steppers[0]
        .phase_offset_target
        .store(99, Ordering::Release);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        q_ptr,
        &shared,
        /* p_end */ 256.0 * 0.0125,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    assert_eq!(
        axis.steppers[0]
            .phase_offset_microsteps
            .load(Ordering::Acquire),
        3,
        "ramp should be a no-op when max_per_sample == 0",
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
        0,
        &mut axis,
        q_ptr,
        &shared,
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
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 263);
}

#[test]
fn unknown_step_mode_raises_fault() {
    use crate::error::FaultCode;

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    let raw_mode: u8 = 0x42;
    let mut steppers: heapless::Vec<StepperRef, 4> = heapless::Vec::new();
    let _ = steppers.push(make_stepper());
    let mut axis = AxisConfig {
        mode: AtomicU8::new(raw_mode),
        steppers,
        microstep_distance: 0.0125,
        ..AxisConfig::new_unconfigured()
    };

    let q_ptr: *mut StepQueue = &mut q;
    let axis_idx: usize = 2;
    dispatch_axis(
        axis_idx,
        &mut axis,
        q_ptr,
        &shared,
        /* p_end */ 1.0,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    assert_eq!(
        q.tail, q.head,
        "no steps should be enqueued for unknown mode"
    );

    let last_err = shared.last_error.load(Ordering::Acquire);
    assert_eq!(
        last_err,
        FaultCode::UnknownStepMode.as_i32(),
        "expected UnknownStepMode fault code, got {last_err}"
    );

    let detail = shared.fault_detail.load(Ordering::Acquire);
    let expected_detail = ((axis_idx as u32 & 0xFF) << 16) | u32::from(raw_mode);
    assert_eq!(
        detail, expected_detail,
        "fault_detail should encode (axis_idx << 16) | mode"
    );
}
