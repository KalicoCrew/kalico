//! MCU axis-configuration stubs — Task 5 placeholder.
//!
//! The full `McuAxisConfig` (with kinematics-aware motor tables) has been
//! removed. This stub retains the minimum needed for `Engine::configure` and
//! `sim_fixtures` to compile until Task 6.

use crate::segment::KinematicTag;

/// Per-motor configuration.
#[derive(Debug, Clone, Copy)]
pub struct MotorConfig {
    pub steps_per_mm: f32,
    pub is_awd: bool,
    pub invert_dir: bool,
}

/// Extruder evaluation mode (stub — no logic, just the enum for FFI compat).
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EMode {
    CoupledToXy = 0,
    Independent = 1,
    Travel = 2,
}

/// Per-MCU axis configuration (stub).
#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    pub motors: [Option<MotorConfig>; 4],
    pub kinematics: KinematicTag,
}

impl McuAxisConfig {
    /// Stub validation — always returns `Ok(())`.
    /// Task 6 will add real kinematics/motor-count checks.
    pub fn validate(&self) -> Result<(), ()> {
        Ok(())
    }
}
