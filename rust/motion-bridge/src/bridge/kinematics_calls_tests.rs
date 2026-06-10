use crate::dispatch::{AXIS_X, AXIS_Y, KINEMATICS_COREXY};

#[test]
fn corexy_inverse_maps_motor_to_toolhead() {
    let xyz = crate::kinematics::inverse(KINEMATICS_COREXY, [4.0, 2.0, 0.0, 0.0]);
    assert!(
        (xyz[0] - 3.0).abs() < 1e-9,
        "expected x=3.0, got {}",
        xyz[0]
    );
    assert!(
        (xyz[1] - 1.0).abs() < 1e-9,
        "expected y=1.0, got {}",
        xyz[1]
    );
}

#[test]
fn corexy_forward_delta_only_slot0_moves_when_dx_eq_dy() {
    let m = crate::kinematics::forward(KINEMATICS_COREXY, [1.0, 1.0, 0.0]);
    let slots: Vec<(u8, f64)> = (0u8..4)
        .filter(|&s| m[s as usize].abs() > 1e-9)
        .map(|s| (s, m[s as usize]))
        .collect();
    assert_eq!(
        slots.len(),
        1,
        "expected only 1 moving slot, got: {slots:?}"
    );
    assert_eq!(
        slots[0].0, AXIS_X as u8,
        "moving slot should be AXIS_X (A motor)"
    );
    assert!(
        (slots[0].1 - 2.0).abs() < 1e-9,
        "A delta should be 2.0, got {}",
        slots[0].1
    );
}

#[test]
fn corexy_forward_motor_positions_xyz() {
    let m = crate::kinematics::forward(KINEMATICS_COREXY, [3.0, 1.0, 5.0]);
    assert!(
        (m[AXIS_X] - 4.0).abs() < 1e-9,
        "slot AXIS_X (A) = 4, got {}",
        m[AXIS_X]
    );
    assert!(
        (m[AXIS_Y] - 2.0).abs() < 1e-9,
        "slot AXIS_Y (B) = 2, got {}",
        m[AXIS_Y]
    );
}
