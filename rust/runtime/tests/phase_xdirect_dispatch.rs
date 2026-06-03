#![cfg(feature = "motion-module-stepper")]
//! End-to-end test for the Phase-mode dispatch path.
//!
//! Verifies that, given:
//!   - `axis.mode == StepMode::Phase`
//!   - a stepper with `tmc_cs_oid == Some(cs)`
//!   - `shared.phase_slot_idx[motor_idx] == axis_idx`
//!   - `shared.phase_motor_count` set accordingly
//!
//! `dispatch_axis` routes through `dispatch_phase`, computes the correct
//! coil currents via the LUT, and routes them to `test_xdirect_capture`
//! rather than the null SPI queue.
//!
//! Complementary tests in `tick_dispatch.rs` cover phase-mode bookkeeping
//! (coil state, position_count, last_phase_target, no step-queue writes);
//! this test focuses exclusively on the SPI capture path introduced by the
//! `fix-spi-drain` change.

#![allow(clippy::unwrap_used)]

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8, Ordering};
use heapless::Vec;

use runtime::phase_lut::PHASE_LUT;
use runtime::state::{MAX_STEPPER_OIDS, SharedState};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{AxisConfig, MAX_STEPPERS_PER_AXIS, StepMode, StepperRef};
use runtime::test_xdirect_capture;
use runtime::dispatch_stepper::dispatch_axis;

/// Build a minimal `StepperRef` with a real TMC CS OID so `dispatch_phase`
/// enters the SPI-capture branch.
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

/// Build an `AxisConfig` in Phase mode.
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

/// Populate `shared.phase_slot_idx` and `phase_motor_count` so
/// `dispatch_phase`'s motor-idx resolver can map `axis_idx` → `motor_idx`.
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

// ─── Test 1: Basic capture — motor_idx, coil_a, coil_b round-trip ───────────

/// A single Phase-mode dispatch with `p_end = 256 * microstep` must call
/// `test_xdirect_capture::record(motor_idx=0, coil_a=?, coil_b=?)` with the
/// LUT values for phase 256 (0x100 & 0x3FF = 256).
///
/// PHASE_LUT[256] == (0, 248) by the identity sinusoid construction:
///   sin(256/1024 * 2π) ≈ sin(π/2) ≈ 1.0  → +248 (A)  ... wait, actually
///   the LUT uses: i_a = sin(phase/1024*2π)*248, i_b = cos(phase/1024*2π)*248.
///   At phase=256 (= quarter cycle): sin≈1, cos≈0 → (248, 0).
/// Check against PHASE_LUT to be exact rather than hard-code a value that
/// may drift if the LUT precision changes.
#[test]
fn phase_dispatch_records_correct_coils_for_motor_0() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    // axis_idx = 0 (X), motor_idx = 0 (first TMC on the bus).
    let axis_idx: usize = 0;
    let motor_idx: usize = 0;
    configure_phase_slot(&shared, motor_idx, axis_idx);

    let stepper = make_phase_stepper(0, /* tmc_cs_oid */ 2);
    let mut axis = make_phase_axis(0.0125, stepper);
    let q_ptr: *mut StepQueue = &mut q;

    // 256 microsteps target → phase = 256 & 0x3FF = 256.
    let p_end = 256.0_f32 * 0.0125;
    dispatch_axis(
        axis_idx,
        &mut axis,
        q_ptr,
        &shared,
        p_end,
        /* v_end */ 0.0,
        /* p_sample_start */ 0.0,
        /* sample_period_sec */ 25e-6,
        /* sample_start_cycles */ 0,
        /* cycles_per_second */ 520_000_000.0,
    );

    let records = test_xdirect_capture::drain();
    assert_eq!(records.len(), 1, "expected exactly one SPI capture");
    let rec = &records[0];
    assert_eq!(rec.motor_idx, motor_idx as u8, "motor_idx mismatch");

    // Verify coils match the LUT for phase 256.
    let (expected_a, expected_b) = PHASE_LUT[256];
    assert_eq!(rec.coil_a, expected_a, "coil_a must match PHASE_LUT[256]");
    assert_eq!(rec.coil_b, expected_b, "coil_b must match PHASE_LUT[256]");

    // Step queue must remain empty — Phase mode does not emit step pulses.
    assert_eq!(q.tail, q.head, "Phase mode must not write to step queue");
}

// ─── Test 2: motor_idx resolved from phase_slot_idx ─────────────────────────

/// When `motor_idx != axis_idx`, the resolver must still find the correct
/// entry by scanning `phase_slot_idx`. Motor 2 maps to axis 1 (Y).
#[test]
fn phase_dispatch_resolves_motor_idx_from_slot_table() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    // Motor 2 → axis 1 (Y in a CoreXY layout).
    let axis_idx: usize = 1;
    let motor_idx: usize = 2;
    // Also add a dummy motor 0 for a different axis so the scan isn't trivial.
    shared.phase_slot_idx[0].store(0u8, Ordering::Release); // motor 0 → axis 0 (X)
    shared.phase_slot_idx[1].store(0u8, Ordering::Release); // motor 1 → axis 0 (X, AWD pair)
    configure_phase_slot(&shared, motor_idx, axis_idx);

    let stepper = make_phase_stepper(1, /* tmc_cs_oid */ 5);
    let mut axis = make_phase_axis(0.0125, stepper);
    let q_ptr: *mut StepQueue = &mut q;

    // Target phase 512 (half cycle).
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

// ─── Test 3: Stepper without tmc_cs_oid does not capture ────────────────────

/// A Pulse-only stepper (tmc_cs_oid = None) on a Phase-mode axis must NOT
/// produce a capture — it has no TMC driver to write to.
#[test]
fn phase_dispatch_no_capture_for_pulse_only_stepper() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    // Pulse-only stepper: tmc_cs_oid = None.
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
    // Coil state is still updated in dispatch_phase regardless.
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

// ─── Test 4: Two steppers on same Phase axis → two captures ─────────────────

/// An AWD axis has two steppers, both with tmc_cs_oid. Both must produce a
/// capture record, ordered by stepper-in-axis order, with motor_idx values
/// from consecutive phase_slot_idx entries for the same axis.
#[test]
fn phase_dispatch_two_steppers_two_captures() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    // Both motors map to axis 0. Motor 0 is the first stepper, motor 1 the second.
    configure_phase_slot(&shared, 0, 0); // motor_idx=0 → axis=0, stepper j=0
    configure_phase_slot(&shared, 1, 0); // motor_idx=1 → axis=0, stepper j=1

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

    // Both get the same coil values because they're on the same axis at the same phase.
    let (expected_a, expected_b) = PHASE_LUT[256];
    assert_eq!(records[0].coil_a, expected_a);
    assert_eq!(records[0].coil_b, expected_b);
    assert_eq!(records[1].coil_a, expected_a);
    assert_eq!(records[1].coil_b, expected_b);

    assert_eq!(q.tail, q.head, "Phase mode must not write to step queue");
}

// ─── Test 5: Phase 0 anchor matches LUT ─────────────────────────────────────

/// Phase 0 corresponds to the start of the electrical cycle.
/// PHASE_LUT[0] = (0, 248) by the sin/cos construction (sin(0)=0, cos(0)=1).
#[test]
fn phase_dispatch_at_phase_zero() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    let mut q = StepQueue::new();

    configure_phase_slot(&shared, 0, 0);
    let stepper = make_phase_stepper(0, 1);
    let mut axis = make_phase_axis(0.0125, stepper);

    // p_end = 0.0 → microstep target = 0 → phase = 0 & 0x3FF = 0.
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

// ─── Test 6: motor_idx 0xFF when slot table is empty ────────────────────────

/// If `phase_motor_count == 0` (unconfigured), the resolver finds no match
/// and falls back to motor_idx = 0xFF. The capture still happens (the stepper
/// has tmc_cs_oid) but with motor_idx = 0xFF, so the C consumer can detect
/// the misconfiguration.
#[test]
fn phase_dispatch_empty_slot_table_uses_sentinel_motor_idx() {
    let _guard = test_xdirect_capture::lock_for_test();
    test_xdirect_capture::clear();

    let shared = SharedState::new();
    // phase_motor_count stays 0 (SharedState::new() default).
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

// Helper: turn a `&mut StepQueue` into a raw pointer for dispatch_axis.
fn q_ptr_from(q: &mut StepQueue) -> *mut StepQueue {
    q as *mut StepQueue
}
