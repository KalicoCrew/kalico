//! Capability-ceiling tests for runtime_set_step_mode.

use runtime::state::{SharedState, StepMode, MAX_STEPPER_OIDS, set_step_mode, SetStepModeError};
use core::sync::atomic::Ordering;

#[test]
fn set_step_mode_with_capability_succeeds() {
    let shared = SharedState::new();
    let result = set_step_mode(&shared, 0, StepMode::Modulated, /* mcu_supports_phase = */ true);
    assert!(result.is_ok());
    assert_eq!(
        StepMode::from_u8(shared.step_modes[0].load(Ordering::Acquire)),
        Some(StepMode::Modulated),
    );
}

#[test]
fn set_step_mode_modulated_without_capability_rejects() {
    let shared = SharedState::new();
    let result = set_step_mode(&shared, 0, StepMode::Modulated, /* mcu_supports_phase = */ false);
    assert_eq!(result, Err(SetStepModeError::CapabilityMissing));
    // State unchanged.
    assert_eq!(
        StepMode::from_u8(shared.step_modes[0].load(Ordering::Acquire)),
        Some(StepMode::StepTime),
    );
}

#[test]
fn set_step_mode_step_time_always_succeeds() {
    let shared = SharedState::new();
    // Even without phase capability, StepTime is fine.
    let result = set_step_mode(&shared, 0, StepMode::StepTime, false);
    assert!(result.is_ok());
}

#[test]
fn set_step_mode_out_of_range_rejects() {
    let shared = SharedState::new();
    let result = set_step_mode(
        &shared,
        MAX_STEPPER_OIDS as u8,
        StepMode::StepTime,
        true,
    );
    assert_eq!(result, Err(SetStepModeError::OutOfRange));
}
