use super::*;
use crate::dispatch::{KINEMATICS_COREXY, McuCaps};
use geometry::segment::EMode;

fn linear_axis(p0: f64, p1: f64) -> ScalarNurbs<f64> {
    let d = p1 - p0;
    let bern = [p0, p0 + d / 3.0, p0 + 2.0 * d / 3.0, p1];
    let piece = nurbs::bezier::BezierPiece::from_bernstein(&bern, 0.0_f64, 1.0_f64);
    nurbs::bezier::bezier_pieces_to_nurbs(&[piece])
}

fn seg_x_move() -> ShapedSegment {
    ShapedSegment {
        axes: [
            linear_axis(0.0, 10.0),
            linear_axis(0.0, 0.0),
            linear_axis(0.0, 0.0),
        ],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 1.0,
    }
}

#[test]
fn cartesian_x_axis_yields_pieces_with_projected_start_time() {
    let cfg = vec![McuAxisConfig {
        mcu_id: 7,
        axes: vec![AXIS_X, AXIS_Y, 2],
        kinematics: 1,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
    }];

    let msgs = enqueue_segment(&seg_x_move(), &cfg, 100.0, true, 0.0, crate::pump::MAX_LEAD_SECS, |_mcu, hs| {
        (hs * 1_000.0) as u64
    });

    let x = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 7, axis: 0 })
        .expect("X axis EnqueueMsg must be present");

    assert!(!x.pieces.is_empty(), "X must have at least one piece");
    assert_eq!(
        x.pieces[0].0.start_time, 100_000,
        "start_time = (t0=100) * 1000 = 100_000"
    );
    assert!(
        x.pieces.iter().all(|(p, _)| p.duration > 0.0),
        "all piece durations must be positive"
    );

    assert!(
        msgs.iter().any(|m| m.key == AxisKey { mcu_id: 7, axis: 1 }),
        "Y axis must be emitted"
    );
    assert!(
        msgs.iter().any(|m| m.key == AxisKey { mcu_id: 7, axis: 2 }),
        "Z axis must be emitted"
    );
}

#[test]
fn corexy_x_slot_is_x_plus_y() {
    let cfg = vec![McuAxisConfig {
        mcu_id: 1,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: KINEMATICS_COREXY,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
    }];

    let seg = ShapedSegment {
        axes: [
            linear_axis(0.0, 10.0),
            linear_axis(0.0, 4.0),
            linear_axis(0.0, 0.0),
        ],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 1.0,
    };

    let msgs = enqueue_segment(&seg, &cfg, 0.0, true, 0.0, crate::pump::MAX_LEAD_SECS, |_mcu, hs| (hs * 1_000.0) as u64);

    let a = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 1, axis: 0 })
        .expect("motor-A (AXIS_X slot) must be present");

    let last_coeff = a.pieces.last().unwrap().0.coeffs[3];
    assert!(
        (last_coeff - 14.0_f32).abs() < 1e-3,
        "motor-A endpoint coefficient expected ≈14, got {last_coeff}"
    );

    let b = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 1, axis: 1 })
        .expect("motor-B (AXIS_Y slot) must be present");

    let b_last = b.pieces.last().unwrap().0.coeffs[3];
    assert!(
        (b_last - 6.0_f32).abs() < 1e-3,
        "motor-B endpoint coefficient expected ≈6, got {b_last}"
    );
}
