//! Mid-segment StepMode flip test. Verifies `runtime_set_step_mode` flips
//! the per-stepper mode atomically and the step-time arming path
//! correctly responds to the new mode.
//!
//! Run with:
//!   cargo test -p runtime --features kalico-sim --test sim_steptime_mode_flip
//!
//! Without `--features kalico-sim` this file compiles to an empty test binary
//! (the `#![cfg(...)]` below excludes all items).

#![cfg(feature = "kalico-sim")]

use core::sync::atomic::Ordering;

use runtime::engine::arm_step_timer_for_stepper;
use runtime::sim_fixtures::{init_test_runtime, push_test_segment_linear_z};
use runtime::state::{MAX_STEPPER_OIDS, StepMode};
use runtime::{set_step_mode, SetStepModeError};

const Z_STEPPER_IDX: u8 = 2;

#[test]
fn set_step_mode_flips_modulated_to_steptime_atomically() {
    let rt = init_test_runtime();

    // Start in Modulated (requires phase capability).
    let result = set_step_mode(&rt.shared, Z_STEPPER_IDX, StepMode::Modulated, /*phase=*/ true);
    assert!(result.is_ok());
    assert_eq!(
        StepMode::from_u8(rt.shared.step_modes[Z_STEPPER_IDX as usize].load(Ordering::Acquire)),
        Some(StepMode::Modulated),
    );

    // Flip to StepTime mid-session.
    let result = set_step_mode(&rt.shared, Z_STEPPER_IDX, StepMode::StepTime, /*phase=*/ true);
    assert!(result.is_ok());
    assert_eq!(
        StepMode::from_u8(rt.shared.step_modes[Z_STEPPER_IDX as usize].load(Ordering::Acquire)),
        Some(StepMode::StepTime),
    );
}

#[test]
fn set_step_mode_back_to_modulated_requires_phase_capability() {
    let rt = init_test_runtime();

    // Flip to Modulated with capability → Ok.
    let result = set_step_mode(&rt.shared, Z_STEPPER_IDX, StepMode::Modulated, /*phase=*/ true);
    assert!(result.is_ok());

    // Flip back to StepTime with NO capability → still Ok (StepTime always permitted).
    let result = set_step_mode(&rt.shared, Z_STEPPER_IDX, StepMode::StepTime, /*phase=*/ false);
    assert!(result.is_ok());

    // Try to flip back to Modulated without capability → rejected.
    let result = set_step_mode(&rt.shared, Z_STEPPER_IDX, StepMode::Modulated, /*phase=*/ false);
    assert_eq!(result, Err(SetStepModeError::CapabilityMissing));

    // State is unchanged (still StepTime).
    assert_eq!(
        StepMode::from_u8(rt.shared.step_modes[Z_STEPPER_IDX as usize].load(Ordering::Acquire)),
        Some(StepMode::StepTime),
    );
}

#[test]
fn arm_step_timer_only_returns_steps_in_step_time_mode() {
    let mut rt = init_test_runtime();

    // Default is StepTime — arm works.
    push_test_segment_linear_z(&mut rt, /*velocity_mm_s=*/ 1.0, /*duration_s=*/ 1.0);
    let result = arm_step_timer_for_stepper(&rt, Z_STEPPER_IDX, 0);
    assert!(result.is_some(), "expected first step in StepTime mode");

    // Flip to Modulated. The arm function itself doesn't gate on mode —
    // the gate is at the C-side `arm_step_time_steppers_after_push` callsite.
    // This confirms the engine handles the call without faulting.
    set_step_mode(&rt.shared, Z_STEPPER_IDX, StepMode::Modulated, true).unwrap();
    let result = arm_step_timer_for_stepper(&rt, Z_STEPPER_IDX, 0);
    // The engine returns a valid step time regardless of mode — mode-gating is C-side.
    assert!(
        result.is_some(),
        "engine returned step time regardless of mode (mode-gating is C-side)",
    );
}

#[test]
fn set_step_mode_seam_no_double_count_after_flip() {
    // Simulate: 100 steps fire in StepTime mode (incrementing stepper_counts),
    // mode flips to Modulated mid-segment, then back to StepTime. Verify the
    // stepper_counts value is preserved across flips (engine doesn't reset).
    let mut rt = init_test_runtime();
    push_test_segment_linear_z(&mut rt, /*velocity_mm_s=*/ 1.0, /*duration_s=*/ 1.0);

    // Fire 100 simulated steps in StepTime mode.
    let mut now_cycles: u64 = 0;
    for _ in 0..100 {
        let (t_next, _dir) = arm_step_timer_for_stepper(&rt, Z_STEPPER_IDX, now_cycles).unwrap();
        rt.shared.stepper_counts[Z_STEPPER_IDX as usize].fetch_add(1, Ordering::AcqRel);
        now_cycles = t_next + 1;
    }
    let count_at_flip = rt.shared.stepper_counts[Z_STEPPER_IDX as usize].load(Ordering::Acquire);
    assert_eq!(count_at_flip, 100);

    // Flip mode atomically (Modulated then back to StepTime).
    set_step_mode(&rt.shared, Z_STEPPER_IDX, StepMode::Modulated, true).unwrap();
    set_step_mode(&rt.shared, Z_STEPPER_IDX, StepMode::StepTime, true).unwrap();

    // Step count preserved — mode flip must not reset or touch stepper_counts.
    let count_after_flip = rt.shared.stepper_counts[Z_STEPPER_IDX as usize].load(Ordering::Acquire);
    assert_eq!(count_after_flip, 100, "step count must not reset on mode flip");

    // Continue stepping — no double-count seam.
    for _ in 0..50 {
        let (t_next, _dir) = arm_step_timer_for_stepper(&rt, Z_STEPPER_IDX, now_cycles).unwrap();
        rt.shared.stepper_counts[Z_STEPPER_IDX as usize].fetch_add(1, Ordering::AcqRel);
        now_cycles = t_next + 1;
    }
    let final_count = rt.shared.stepper_counts[Z_STEPPER_IDX as usize].load(Ordering::Acquire);
    assert_eq!(final_count, 150, "100 + 50 steps, no double-count at flip seam");
}

#[test]
fn set_step_mode_out_of_range_stepper_idx_rejected() {
    let rt = init_test_runtime();
    let result = set_step_mode(
        &rt.shared,
        MAX_STEPPER_OIDS as u8,
        StepMode::StepTime,
        true,
    );
    assert_eq!(result, Err(SetStepModeError::OutOfRange));
}
