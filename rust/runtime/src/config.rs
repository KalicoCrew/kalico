//! MCU axis configuration types — `EMode`, `MotorConfig`, `McuAxisConfig`.
//!
//! Step 7-B Task 2: configuration types for per-axis motor mapping and
//! extruder mode selection.

use crate::segment::KinematicTag;

/// Extruder mode for a segment. Determines how the E axis is evaluated.
///
/// - `CoupledToXy`: `E_actual(t)` = `extrusion_ratio` * integral |`v_xy(t)`| dt.
///   The MCU derives E from the shaped XY trajectory per-sample.
/// - `Independent`: E has its own NURBS curve (retraction / prime / filament
///   change — E motion with no XY).
/// - `Travel`: No extrusion; E handle is unused.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EMode {
    CoupledToXy = 0,
    Independent = 1,
    Travel = 2,
}

/// Per-motor configuration, one entry per physical stepper.
#[derive(Debug, Clone)]
pub struct MotorConfig {
    /// Steps per millimetre for this motor's axis.
    pub steps_per_mm: f32,
    /// All-wheel-drive flag (both A and B steppers on `CoreXY`, for example).
    pub is_awd: bool,
    /// Invert step direction pin.
    pub invert_dir: bool,
}

/// Per-MCU axis configuration. Maps logical axes to physical motors and
/// selects the kinematic transform.
#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    /// Per-motor config, indexed in motor space (post-kinematic-transform):
    /// `CoreXyAndE`: [A=0, B=1, Z=2, E=3]; `CartesianXyzAndE`: [X=0, Y=1, Z=2, E=3].
    pub motors: [Option<MotorConfig>; 4],
    /// Kinematic transform tag for this MCU.
    pub kinematics: KinematicTag,
}

impl McuAxisConfig {
    /// Validate motor configuration against kinematic constraints.
    ///
    /// `CoreXY` requires both A and B motors to be present or both absent —
    /// having only one of the pair is a configuration error.
    pub fn validate(&self) -> Result<(), &'static str> {
        match self.kinematics {
            KinematicTag::CoreXyAndE => {
                let has_a = self.motors[0].is_some();
                let has_b = self.motors[1].is_some();
                if has_a != has_b {
                    return Err("CoreXY: must own both A and B or neither");
                }
                Ok(())
            }
            KinematicTag::CartesianXyzAndE => Ok(()),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::indexing_slicing)]
mod tests {
    use super::*;

    #[test]
    fn emode_discriminant_values() {
        assert_eq!(EMode::CoupledToXy as u8, 0);
        assert_eq!(EMode::Independent as u8, 1);
        assert_eq!(EMode::Travel as u8, 2);
    }

    #[test]
    fn corexy_validate_both_present_ok() {
        let cfg = McuAxisConfig {
            motors: [
                Some(MotorConfig {
                    steps_per_mm: 80.0,
                    is_awd: false,
                    invert_dir: false,
                }),
                Some(MotorConfig {
                    steps_per_mm: 80.0,
                    is_awd: false,
                    invert_dir: false,
                }),
                None,
                None,
            ],
            kinematics: KinematicTag::CoreXyAndE,
        };
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn corexy_validate_both_absent_ok() {
        let cfg = McuAxisConfig {
            motors: [None, None, None, None],
            kinematics: KinematicTag::CoreXyAndE,
        };
        assert!(cfg.validate().is_ok());
    }

    #[test]
    fn corexy_validate_only_a_fails() {
        let cfg = McuAxisConfig {
            motors: [
                Some(MotorConfig {
                    steps_per_mm: 80.0,
                    is_awd: false,
                    invert_dir: false,
                }),
                None,
                None,
                None,
            ],
            kinematics: KinematicTag::CoreXyAndE,
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn corexy_validate_only_b_fails() {
        let cfg = McuAxisConfig {
            motors: [
                None,
                Some(MotorConfig {
                    steps_per_mm: 80.0,
                    is_awd: false,
                    invert_dir: false,
                }),
                None,
                None,
            ],
            kinematics: KinematicTag::CoreXyAndE,
        };
        assert!(cfg.validate().is_err());
    }

    #[test]
    fn cartesian_validate_always_ok() {
        let cfg = McuAxisConfig {
            motors: [
                Some(MotorConfig {
                    steps_per_mm: 80.0,
                    is_awd: false,
                    invert_dir: false,
                }),
                None,
                None,
                None,
            ],
            kinematics: KinematicTag::CartesianXyzAndE,
        };
        assert!(cfg.validate().is_ok());
    }
}
