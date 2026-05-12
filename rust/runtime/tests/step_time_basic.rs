//! Basic StepMode enum + per-stepper field tests.

use runtime::state::{StepMode, SharedState, MAX_STEPPER_OIDS};
use core::sync::atomic::Ordering;

#[test]
fn default_step_mode_is_step_time() {
    let shared = SharedState::new();
    for i in 0..MAX_STEPPER_OIDS {
        let raw = shared.step_modes[i].load(Ordering::Acquire);
        assert_eq!(
            StepMode::from_u8(raw),
            Some(StepMode::StepTime),
            "stepper {} default should be StepTime",
            i,
        );
    }
}

#[test]
fn step_mode_roundtrip_via_atomic() {
    let shared = SharedState::new();
    shared.step_modes[0].store(StepMode::Modulated as u8, Ordering::Release);
    let raw = shared.step_modes[0].load(Ordering::Acquire);
    assert_eq!(StepMode::from_u8(raw), Some(StepMode::Modulated));
}
