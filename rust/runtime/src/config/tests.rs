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
