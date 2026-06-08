use super::*;
use crate::dispatch::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps, KINEMATICS_COREXY};

fn corexy_bridge() -> PyMotionBridge {
    let b = PyMotionBridge::new_for_test();
    b.mcu_axis_configs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .push(McuAxisConfig {
            mcu_id: 5,
            axes: vec![AXIS_X, AXIS_Y, AXIS_Z],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps::default(),
        });
    b
}

#[test]
fn kin_tag_resolves_configured_mcu() {
    let b = corexy_bridge();
    assert_eq!(b.kin_tag_for(5), Some(KINEMATICS_COREXY));
}

#[test]
fn kin_tag_returns_none_for_unconfigured_mcu() {
    let b = corexy_bridge();
    assert_eq!(b.kin_tag_for(99), None);
}

#[test]
fn corexy_inverse_maps_motor_to_toolhead() {
    // CoreXY: x = 0.5*(A+B), y = 0.5*(A-B)
    // A=4, B=2 -> x = 0.5*(4+2) = 3, y = 0.5*(4-2) = 1
    let xyz = crate::kinematics::inverse(KINEMATICS_COREXY, [4.0, 2.0, 0.0, 0.0]);
    assert!((xyz[0] - 3.0).abs() < 1e-9, "expected x=3.0, got {}", xyz[0]);
    assert!((xyz[1] - 1.0).abs() < 1e-9, "expected y=1.0, got {}", xyz[1]);
}

#[test]
fn corexy_forward_delta_only_slot0_moves_when_dx_eq_dy() {
    // CoreXY: A = x+y, B = x-y
    // dx=1, dy=1 -> A=2, B=0
    // Only slot 0 (A) has nonzero delta; slot 1 (B) = 0 is filtered out.
    let b = corexy_bridge();
    let tag = b.kin_tag_for(5).unwrap();
    let m = crate::kinematics::forward(tag, [1.0, 1.0, 0.0]);
    let slots: Vec<(u8, f64)> = (0u8..4)
        .filter(|&s| m[s as usize].abs() > 1e-9)
        .map(|s| (s, m[s as usize]))
        .collect();
    assert_eq!(slots.len(), 1, "expected only 1 moving slot, got: {slots:?}");
    assert_eq!(slots[0].0, 0, "moving slot should be 0 (A motor)");
    assert!((slots[0].1 - 2.0).abs() < 1e-9, "A delta should be 2.0, got {}", slots[0].1);
}

#[test]
fn forward_motor_positions_returns_present_slots() {
    // Asking for the absolute motor positions of toolhead (3, 1, 5) on CoreXY mcu 5.
    // CoreXY: A = x+y = 3+1 = 4, B = x-y = 3-1 = 2, Z = 5
    // Present slots: [0, 1, 2]
    let b = corexy_bridge();
    let tag = b.kin_tag_for(5).unwrap();
    let m = crate::kinematics::forward(tag, [3.0, 1.0, 5.0]);
    let cfgs = b.mcu_axis_configs.lock().unwrap_or_else(|p| p.into_inner());
    let present: Vec<usize> = cfgs
        .iter()
        .find(|c| c.mcu_id == 5)
        .map(|c| c.axes.clone())
        .unwrap_or_default();
    drop(cfgs);
    let result: Vec<(u8, f64)> = present
        .into_iter()
        .filter(|&s| s < 4)
        .map(|s| (s as u8, m[s]))
        .collect();
    assert_eq!(result.len(), 3);
    assert!((result[0].1 - 4.0).abs() < 1e-9, "slot 0 (A) = 4, got {}", result[0].1);
    assert!((result[1].1 - 2.0).abs() < 1e-9, "slot 1 (B) = 2, got {}", result[1].1);
    assert!((result[2].1 - 5.0).abs() < 1e-9, "slot 2 (Z) = 5, got {}", result[2].1);
}
