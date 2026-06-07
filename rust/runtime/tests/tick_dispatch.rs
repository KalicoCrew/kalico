#![cfg(feature = "motion-module-stepper")]

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

use runtime::dispatch_stepper::dispatch_axis;
use runtime::error::FaultCode;
use runtime::state::SharedState;
use runtime::step_queue::{STEP_QUEUE_DEPTH, StepQueue};
use runtime::stepping_state::{AxisConfig, MAX_STEPPERS_PER_AXIS, StepMode, StepperRef};

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
    let mut steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
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
        0.0,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
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
        0,
        &mut axis,
        q_ptr,
        &shared,
        0.05,
        2000.0,
        0.0,
        25e-6,
        1_000,
        520_000_000.0,
    );

    assert_eq!(q.tail.wrapping_sub(q.head), 4);
    assert_eq!(axis.last_step_count, 4);
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 4);
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
}

#[test]
fn pulse_partial_push_commits_position_count_for_pushed_steps() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    q.tail = (STEP_QUEUE_DEPTH as u16) - 1;
    q.head = 0;
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        0,
        &mut axis,
        q_ptr,
        &shared,
        0.05,
        2000.0,
        0.0,
        25e-6,
        1_000,
        520_000_000.0,
    );

    assert_eq!(
        q.tail.wrapping_sub(q.head),
        STEP_QUEUE_DEPTH as u16,
        "queue should be exactly full (31 prefill + 1 push)"
    );
    assert_eq!(
        axis.last_step_count, 1,
        "last_step_count must reflect pushes that landed, not requested target"
    );
    assert_eq!(
        axis.steppers[0].position_count.load(Ordering::Acquire),
        1,
        "position_count must commit for pushed steps before fault"
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepQueueOverflow.as_i32()
    );
}

#[test]
fn pulse_queue_overflow_latches_fault() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    q.tail = STEP_QUEUE_DEPTH as u16;
    q.head = 0;
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        2,
        &mut axis,
        q_ptr,
        &shared,
        0.0125,
        1000.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
    );

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepQueueOverflow.as_i32()
    );
    assert_eq!(shared.queue_overflow_count[2].load(Ordering::Acquire), 1);
}

#[test]
fn pulse_steps_per_sample_exceeded_hard_faults() {
    let shared = SharedState::new();
    let mut q = StepQueue::new();
    let mut axis = make_axis(StepMode::Pulse, 0.0125);

    let q_ptr: *mut StepQueue = &mut q;
    dispatch_axis(
        1,
        &mut axis,
        q_ptr,
        &shared,
        0.5,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
    );

    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::StepsPerSampleExceeded.as_i32(),
        "over-threshold delta must latch StepsPerSampleExceeded"
    );
    assert_eq!(q.tail, q.head, "no steps may be enqueued on overrun");
    assert_eq!(
        axis.last_step_count, 0,
        "baseline must not advance on fault"
    );
    assert_eq!(
        shared.fault_detail.load(Ordering::Acquire),
        (1u32 << 16) | 40,
        "fault_detail encodes axis index and saturated step count"
    );
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
        256.0 * 0.0125,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
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
        256.0 * 0.0125,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
    );

    assert_eq!(
        axis.steppers[0].last_phase_target.load(Ordering::Acquire),
        263
    );
    assert_eq!(axis.steppers[0].position_count.load(Ordering::Acquire), 263);
}
