#![cfg(feature = "motion-module-stepper")]
#![allow(clippy::unwrap_used)]

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

use runtime::dispatch_stepper::dispatch_axis;
use runtime::phase_lut::PHASE_LUT;
use runtime::state::{MAX_STEPPER_OIDS, SharedState};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{AxisConfig, MAX_STEPPERS_PER_AXIS, StepMode, StepperRef};
use runtime::test_xdirect_capture;

fn make_phase_stepper(stepper_oid: u8, tmc_cs_oid: u8) -> StepperRef {
    StepperRef {
        stepper_oid,
        position_count: AtomicI32::new(0),
        tmc_cs_oid: Some(tmc_cs_oid),
        last_coil_A: AtomicI16::new(0),
        last_coil_B: AtomicI16::new(0),
        phase_offset_microsteps: AtomicI32::new(0),
        phase_offset_target: AtomicI32::new(0),
        last_phase_target: AtomicI32::new(0),
    }
}

fn make_phase_axis(microstep_distance: f32, stepper: StepperRef) -> AxisConfig {
    let mut steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers.push(stepper);
    AxisConfig {
        mode: AtomicU8::new(StepMode::Phase as u8),
        steppers,
        microstep_distance,
        ..AxisConfig::new_unconfigured()
    }
}

fn configure_phase_slot(shared: &SharedState, motor_idx: usize, axis_idx: usize) {
    assert!(motor_idx < MAX_STEPPER_OIDS);
    shared.phase_slot_idx[motor_idx].store(axis_idx as u8, Ordering::Release);
    let prev_count = shared.phase_motor_count.load(Ordering::Acquire);
    if (motor_idx as u8) >= prev_count {
        shared
            .phase_motor_count
            .store(motor_idx as u8 + 1, Ordering::Release);
    }
}

#[test]
fn phase_dispatch_records_correct_coils_for_motor_0() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    let axis_idx: usize = 0;
    let motor_idx: usize = 0;
    configure_phase_slot(&shared, motor_idx, axis_idx);

    let stepper = make_phase_stepper(0, 2);
    let mut axis = make_phase_axis(0.0125, stepper);
    let q_ptr: *mut StepQueue = &mut q;

    let p_end = 256.0_f32 * 0.0125;
    dispatch_axis(
        axis_idx,
        &mut axis,
        q_ptr,
        &shared,
        p_end,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
    );

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1, "expected exactly one SPI capture");
    let rec = &records[0];
    assert_eq!(rec.motor_idx, motor_idx as u8, "motor_idx mismatch");

    let (expected_a, expected_b) = PHASE_LUT[256];
    assert_eq!(rec.coil_a, expected_a, "coil_a must match PHASE_LUT[256]");
    assert_eq!(rec.coil_b, expected_b, "coil_b must match PHASE_LUT[256]");

    assert_eq!(q.tail, q.head, "Phase mode must not write to step queue");
}

#[test]
fn phase_dispatch_resolves_motor_idx_from_slot_table() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    let axis_idx: usize = 1;
    let motor_idx: usize = 2;
    shared.phase_slot_idx[0].store(0u8, Ordering::Release);
    shared.phase_slot_idx[1].store(0u8, Ordering::Release);
    configure_phase_slot(&shared, motor_idx, axis_idx);

    let stepper = make_phase_stepper(1, 5);
    let mut axis = make_phase_axis(0.0125, stepper);
    let q_ptr: *mut StepQueue = &mut q;

    let p_end = 512.0_f32 * 0.0125;
    dispatch_axis(
        axis_idx,
        &mut axis,
        q_ptr,
        &shared,
        p_end,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
    );

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1, "expected exactly one SPI capture");
    assert_eq!(
        records[0].motor_idx, motor_idx as u8,
        "motor_idx must resolve to 2, not 0 or 1"
    );

    let (expected_a, expected_b) = PHASE_LUT[512];
    assert_eq!(records[0].coil_a, expected_a);
    assert_eq!(records[0].coil_b, expected_b);
}

#[test]
fn phase_dispatch_no_capture_for_pulse_only_stepper() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    let stepper = StepperRef {
        stepper_oid: 0,
        position_count: AtomicI32::new(0),
        tmc_cs_oid: None,
        last_coil_A: AtomicI16::new(0),
        last_coil_B: AtomicI16::new(0),
        phase_offset_microsteps: AtomicI32::new(0),
        phase_offset_target: AtomicI32::new(0),
        last_phase_target: AtomicI32::new(0),
    };

    configure_phase_slot(&shared, 0, 0);
    let mut axis = make_phase_axis(0.0125, stepper);
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

    let records = test_xdirect_capture::drain();
    assert!(
        records.is_empty(),
        "Pulse-only stepper must not produce a capture"
    );
    let (expected_a, expected_b) = PHASE_LUT[256];
    assert_eq!(
        axis.steppers[0].last_coil_A.load(Ordering::Acquire),
        expected_a
    );
    assert_eq!(
        axis.steppers[0].last_coil_B.load(Ordering::Acquire),
        expected_b
    );
}

#[test]
fn phase_dispatch_two_steppers_two_captures() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    configure_phase_slot(&shared, 0, 0);
    configure_phase_slot(&shared, 1, 0);

    let s0 = make_phase_stepper(0, 3);
    let s1 = make_phase_stepper(1, 4);
    let mut steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS> = Vec::new();
    let _ = steppers.push(s0);
    let _ = steppers.push(s1);

    let mut axis = AxisConfig {
        mode: AtomicU8::new(StepMode::Phase as u8),
        steppers,
        microstep_distance: 0.0125,
        ..AxisConfig::new_unconfigured()
    };

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

    let records = test_xdirect_capture::drain();
    assert_eq!(
        records.len(),
        2,
        "expected two SPI captures for two steppers"
    );
    assert_eq!(records[0].motor_idx, 0, "first stepper → motor_idx 0");
    assert_eq!(records[1].motor_idx, 1, "second stepper → motor_idx 1");

    let (expected_a, expected_b) = PHASE_LUT[256];
    assert_eq!(records[0].coil_a, expected_a);
    assert_eq!(records[0].coil_b, expected_b);
    assert_eq!(records[1].coil_a, expected_a);
    assert_eq!(records[1].coil_b, expected_b);

    assert_eq!(q.tail, q.head, "Phase mode must not write to step queue");
}

#[test]
fn phase_dispatch_at_phase_zero() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    configure_phase_slot(&shared, 0, 0);
    let stepper = make_phase_stepper(0, 1);
    let mut axis = make_phase_axis(0.0125, stepper);

    dispatch_axis(
        0,
        &mut axis,
        q_ptr_from(&mut q),
        &shared,
        0.0,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
    );

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1);
    let (expected_a, expected_b) = PHASE_LUT[0];
    assert_eq!(records[0].coil_a, expected_a, "PHASE_LUT[0] coil_a");
    assert_eq!(records[0].coil_b, expected_b, "PHASE_LUT[0] coil_b");
}

#[test]
fn phase_dispatch_empty_slot_table_uses_sentinel_motor_idx() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    assert_eq!(shared.phase_motor_count.load(Ordering::Acquire), 0);

    let mut q = StepQueue::new();
    let stepper = make_phase_stepper(0, 7);
    let mut axis = make_phase_axis(0.0125, stepper);

    dispatch_axis(
        0,
        &mut axis,
        q_ptr_from(&mut q),
        &shared,
        256.0 * 0.0125,
        0.0,
        0.0,
        25e-6,
        0,
        520_000_000.0,
    );

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1, "capture must still happen");
    assert_eq!(
        records[0].motor_idx, 0xFF,
        "sentinel motor_idx when slot table empty"
    );
}

fn q_ptr_from(q: &mut StepQueue) -> *mut StepQueue {
    q as *mut StepQueue
}
