use super::*;
use crate::dispatch::{AXIS_E, AXIS_X, AXIS_Y, McuAxisConfig, McuCaps, KINEMATICS_COREXY};

/// Real-bench topology: CoreXY MCU with slots [0=X, 1=Y, 3=E] present
/// (present_mask = 0b0000_1011 = 0xb). Z is on a separate MCU and is absent
/// here to match the bench fixture described in the bug report.
fn make_bench_corexy() -> PyMotionBridge {
    let b = PyMotionBridge::new_for_test();
    {
        let mut cfgs = b.mcu_axis_configs.lock().unwrap_or_else(|p| p.into_inner());
        cfgs.push(McuAxisConfig {
            mcu_id: 7,
            axes: vec![AXIS_X, AXIS_Y, AXIS_E],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps { total_piece_memory: 62 * 1024 },
        });
    }
    // oid→slot registrations that _configure_axes_per_mcu populates
    // before calling ground_origin.
    b.insert_stepper_slot(7, 0, 0); // stepper_x  → slot 0 (A motor)
    b.insert_stepper_slot(7, 1, 1); // stepper_y  → slot 1 (B motor)
    b.insert_stepper_slot(7, 3, 3); // extruder   → slot 3 (E motor)
    b
}

/// After ground_origin (origin = 0,0,0), every present-slot oid evaluates to
/// 0.0 via both eval_motor_mm_now and eval_motor_mm_at_t_abs, covering the
/// filament_motion_sensor → get_past_mcu_position path that fired the crash.
#[test]
fn at_rest_eval_returns_zero_not_error() {
    let b = make_bench_corexy();
    b.ground_constants_inner(0.0, 0.0, 0.0, 0.0);

    let mcu = 7u32;
    for oid in [0u8, 1, 3] {
        let now = b
            .eval_motor_mm_now(mcu, oid)
            .unwrap_or_else(|| panic!("eval_motor_mm_now returned None for oid={oid}"));
        assert!(
            now.abs() < 1e-9,
            "eval_motor_mm_now oid={oid}: expected 0.0, got {now}"
        );

        for t_abs in [0.0_f64, 0.5, 100.0, 9999.0] {
            let at_t = b
                .eval_motor_mm_at_t_abs(mcu, oid, t_abs)
                .unwrap_or_else(|| {
                    panic!("eval_motor_mm_at_t_abs returned None for oid={oid} t_abs={t_abs}")
                });
            assert!(
                at_t.abs() < 1e-9,
                "eval_motor_mm_at_t_abs oid={oid} t={t_abs}: expected 0.0, got {at_t}"
            );
        }
    }
}

/// eval stays fail-loud (returns None) for an oid that was never mapped.
/// ground_origin must NOT silently zero-fill unmapped oids — that would mask
/// genuine mid-operation gaps.
#[test]
fn unmapped_oid_still_returns_none_after_ground() {
    let b = make_bench_corexy();
    b.ground_constants_inner(0.0, 0.0, 0.0, 0.0);

    let unmapped_oid = 99u8;
    assert!(
        b.eval_motor_mm_now(7, unmapped_oid).is_none(),
        "eval must return None for an oid with no oid->slot mapping"
    );
    assert!(
        b.eval_motor_mm_at_t_abs(7, unmapped_oid, 0.0).is_none(),
        "eval_at_t must return None for an oid with no oid->slot mapping"
    );
}
