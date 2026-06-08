use super::*;
use crate::dispatch::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps};

fn make_corexy_bridge() -> PyMotionBridge {
    let b = PyMotionBridge::new_for_test();
    // MCU 1: slots [X=0, Y=1, Z=2], kinematics=CoreXY (tag=0)
    {
        let mut cfgs = b.mcu_axis_configs.lock().unwrap_or_else(|p| p.into_inner());
        cfgs.push(McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X, AXIS_Y, AXIS_Z],
            kinematics: crate::dispatch::KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        });
    }
    // Register stepper OIDs: oid=10->slot0(A), oid=11->slot1(B), oid=12->slot2(Z)
    b.insert_stepper_slot(1, 10, 0);
    b.insert_stepper_slot(1, 11, 1);
    b.insert_stepper_slot(1, 12, 2);
    b
}

/// CoreXY forward: slot0(A) = x+y, slot1(B) = x-y, slot2(Z) = z.
/// set_position(x=10, y=4, z=3):
///   A = 10+4 = 14, B = 10-4 = 6, Z = 3.
#[test]
fn ground_constants_installs_correct_corexy_motor_positions() {
    let b = make_corexy_bridge();
    b.ground_constants_inner(10.0, 4.0, 3.0, 0.0);

    let a = b.eval_motor_mm_now(1, 10).expect("slot A (oid=10) should have a constant curve");
    let bm = b.eval_motor_mm_now(1, 11).expect("slot B (oid=11) should have a constant curve");
    let z = b.eval_motor_mm_now(1, 12).expect("slot Z (oid=12) should have a constant curve");

    assert!(
        (a - 14.0).abs() < 1e-9,
        "slot A: expected 14.0, got {a}"
    );
    assert!(
        (bm - 6.0).abs() < 1e-9,
        "slot B: expected 6.0, got {bm}"
    );
    assert!(
        (z - 3.0).abs() < 1e-9,
        "slot Z: expected 3.0, got {z}"
    );
}

/// After ground_constants_inner, a second call updates the curves.
#[test]
fn ground_constants_replaces_on_second_call() {
    let b = make_corexy_bridge();
    b.ground_constants_inner(10.0, 4.0, 3.0, 0.0);
    b.ground_constants_inner(5.0, 1.0, 7.0, 0.0);

    let a = b.eval_motor_mm_now(1, 10).unwrap();
    let bm = b.eval_motor_mm_now(1, 11).unwrap();
    let z = b.eval_motor_mm_now(1, 12).unwrap();

    // A = 5+1=6, B = 5-1=4, Z=7
    assert!((a - 6.0).abs() < 1e-9, "slot A after re-ground: expected 6.0, got {a}");
    assert!((bm - 4.0).abs() < 1e-9, "slot B after re-ground: expected 4.0, got {bm}");
    assert!((z - 7.0).abs() < 1e-9, "slot Z after re-ground: expected 7.0, got {z}");
}

/// The constant curve clamps: eval at any t returns value_mm.
#[test]
fn grounded_curve_evals_to_constant_at_any_time() {
    let b = make_corexy_bridge();
    b.ground_constants_inner(10.0, 4.0, 3.0, 1000.0);

    let mcu = 1u32;
    let slot = 0u8;

    // eval via the internal helper (which resolves oid->slot)
    for t in [0.0, 500.0, 1000.0, 9999.0_f64] {
        let v = b
            .retained_motor_curve
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .eval(mcu, slot, t)
            .expect("eval should return Some for grounded slot");
        assert!(
            (v - 14.0).abs() < 1e-9,
            "slot A at t={t}: expected 14.0, got {v}"
        );
    }
}

/// Cartesian bridge: forward is identity, so motor mm == toolhead mm.
#[test]
fn ground_constants_cartesian_identity() {
    let b = PyMotionBridge::new_for_test();
    {
        let mut cfgs = b.mcu_axis_configs.lock().unwrap_or_else(|p| p.into_inner());
        cfgs.push(McuAxisConfig {
            mcu_id: 2,
            axes: vec![AXIS_X, AXIS_Y, AXIS_Z],
            kinematics: 1, // KINEMATICS_CARTESIAN
            caps: McuCaps { total_piece_memory: 62 * 1024 },
        });
    }
    b.insert_stepper_slot(2, 20, 0);
    b.insert_stepper_slot(2, 21, 1);
    b.insert_stepper_slot(2, 22, 2);

    b.ground_constants_inner(7.0, 3.0, 1.5, 0.0);

    assert!((b.eval_motor_mm_now(2, 20).unwrap() - 7.0).abs() < 1e-9);
    assert!((b.eval_motor_mm_now(2, 21).unwrap() - 3.0).abs() < 1e-9);
    assert!((b.eval_motor_mm_now(2, 22).unwrap() - 1.5).abs() < 1e-9);
}
