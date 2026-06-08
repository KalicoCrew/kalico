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

    let (msgs, _motor_curves) = enqueue_segment(
        &seg_x_move(),
        &cfg,
        100.0,
        true,
        0.0,
        crate::pump::MAX_LEAD_SECS,
        |_mcu, hs| (hs * 1_000.0) as u64,
        None,
    );

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

    let (msgs, _motor_curves) = enqueue_segment(
        &seg,
        &cfg,
        0.0,
        true,
        0.0,
        crate::pump::MAX_LEAD_SECS,
        |_mcu, hs| (hs * 1_000.0) as u64,
        None,
    );

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

#[test]
fn subdivide_preserves_curve_and_continuity() {
    let coeffs = [1.0_f64, 4.0, 2.0, 8.0];
    let pieces = subdivide_bernstein(coeffs, 0.2, 0.025);
    assert_eq!(pieces.len(), 8);
    let total: f64 = pieces.iter().map(|(_, d)| *d).sum();
    assert!((total - 0.2).abs() < 1e-12);
    for w in pieces.windows(2) {
        assert!((w[0].0[3] - w[1].0[0]).abs() < 1e-12);
    }
    let eval = |c: &[f64; 4], u: f64| {
        let v = 1.0 - u;
        c[0] * v * v * v + 3.0 * c[1] * v * v * u + 3.0 * c[2] * v * u * u + c[3] * u * u * u
    };
    for s in [0.0, 0.07, 0.13, 0.2] {
        let direct = eval(&coeffs, s / 0.2);
        let mut acc = 0.0;
        let mut found = None;
        for (c, d) in &pieces {
            if s <= acc + d + 1e-12 {
                found = Some(eval(c, ((s - acc) / d).clamp(0.0, 1.0)));
                break;
            }
            acc += d;
        }
        assert!((found.unwrap() - direct).abs() < 1e-9);
    }
}

#[test]
fn short_pieces_pass_through_unsplit() {
    let pieces = subdivide_bernstein([0.0, 1.0, 2.0, 3.0], 0.02, 0.025);
    assert_eq!(pieces.len(), 1);
}

#[test]
fn flatten_axis_max_piece_secs_splits_long_piece() {
    let cfg = vec![McuAxisConfig {
        mcu_id: 7,
        axes: vec![AXIS_X],
        kinematics: 1,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
    }];

    fn linear_axis_scaled(p0: f64, p1: f64, duration: f64) -> ScalarNurbs<f64> {
        let d = p1 - p0;
        let bern = [p0, p0 + d / 3.0, p0 + 2.0 * d / 3.0, p1];
        let piece = nurbs::bezier::BezierPiece::from_bernstein(&bern, 0.0_f64, duration);
        nurbs::bezier::bezier_pieces_to_nurbs(&[piece])
    }

    let seg = ShapedSegment {
        axes: [
            linear_axis_scaled(0.0, 10.0, 0.2),
            linear_axis_scaled(0.0, 0.0, 0.2),
            linear_axis_scaled(0.0, 0.0, 0.2),
        ],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 0.2,
    };

    let (msgs, _motor_curves) = enqueue_segment(
        &seg,
        &cfg,
        100.0,
        true,
        0.0,
        crate::pump::MAX_LEAD_SECS,
        |_mcu, hs| (hs * 1_000.0) as u64,
        Some(0.025),
    );

    let x = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 7, axis: 0 })
        .expect("X axis EnqueueMsg must be present");

    assert_eq!(x.pieces.len(), 8, "0.2s / 0.025s = 8 sub-pieces");

    for (p, _) in &x.pieces {
        assert!(
            p.duration <= 0.025 + 1e-6,
            "each piece duration must be ≤ 0.025+ε, got {}",
            p.duration
        );
        assert!(p.duration > 0.0, "piece duration must be positive");
    }

    let times: Vec<u64> = x.pieces.iter().map(|(p, _)| p.start_time).collect();
    for w in times.windows(2) {
        assert!(w[1] > w[0], "start_times must be strictly increasing");
    }

    let host_sidecars: Vec<f64> = x.pieces.iter().map(|(_, hs)| *hs).collect();
    for w in host_sidecars.windows(2) {
        assert!(
            (w[1] - w[0] - 0.025).abs() < 1e-9,
            "host-time sidecar must advance by sub_dur=0.025, got delta {}",
            w[1] - w[0]
        );
    }
}
