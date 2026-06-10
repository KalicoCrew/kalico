use super::*;
use crate::dispatch::{KINEMATICS_COREXY, McuCaps};
use geometry::segment::EMode;

fn constant_axis(value: f64, n_pieces: usize, piece_dur: f64) -> ScalarNurbs<f64> {
    let bern = [value; 4];
    let mut pieces = Vec::with_capacity(n_pieces);
    let mut u = 0.0_f64;
    for _ in 0..n_pieces {
        let u_end = u + piece_dur;
        pieces.push(nurbs::bezier::BezierPiece::from_bernstein(&bern, u, u_end));
        u = u_end;
    }
    nurbs::bezier::bezier_pieces_to_nurbs(&pieces)
}

fn multi_piece_axis(pieces_bern: &[([f64; 4], f64)]) -> ScalarNurbs<f64> {
    let mut pieces = Vec::with_capacity(pieces_bern.len());
    let mut u = 0.0_f64;
    for (bern, dur) in pieces_bern {
        let u_end = u + dur;
        pieces.push(nurbs::bezier::BezierPiece::from_bernstein(bern, u, u_end));
        u = u_end;
    }
    nurbs::bezier::bezier_pieces_to_nurbs(&pieces)
}

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

    let msgs = enqueue_segment(
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

    let msgs = enqueue_segment(
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

    let msgs = enqueue_segment(
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

fn axis_cfg_single(axis: usize) -> Vec<McuAxisConfig> {
    vec![McuAxisConfig {
        mcu_id: 1,
        axes: vec![axis],
        kinematics: 1,
        caps: McuCaps {
            total_piece_memory: 62 * 1024,
        },
    }]
}

#[test]
fn constant_follower_axis_merges_all_knots_to_one_piece() {
    let n_knots = 50;
    let piece_dur = 0.43e-3_f64;
    let total = n_knots as f64 * piece_dur;
    let curve = constant_axis(5.0, n_knots, piece_dur);

    let seg = ShapedSegment {
        axes: [curve, linear_axis(0.0, 0.0), linear_axis(0.0, 0.0)],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: total,
    };

    let msgs = enqueue_segment(
        &seg,
        &axis_cfg_single(0),
        0.0,
        true,
        0.0,
        crate::pump::MAX_LEAD_SECS,
        |_, hs| (hs * 1e9) as u64,
        Some(0.025),
    );

    let axis = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 1, axis: 0 })
        .expect("axis 0 must be present");

    assert_eq!(
        axis.pieces.len(),
        1,
        "{n_knots} constant knot pieces must merge to exactly 1 piece, got {}",
        axis.pieces.len()
    );

    let (piece, host_secs) = &axis.pieces[0];
    assert_eq!(*host_secs, 0.0, "merged piece host_secs must be t0=0");
    assert!(
        (piece.duration as f64 - total).abs() < 1e-9,
        "merged duration must equal sum of knot durations {total}, got {}",
        piece.duration
    );
    assert!(
        piece.coeffs.iter().all(|&c| (c - 5.0_f32).abs() < 1e-5),
        "all coefficients must equal the constant value 5.0"
    );
}

#[test]
fn motion_constant_motion_merges_only_the_constant_run() {
    let motion_up: [f64; 4] = [0.0, 1.0, 2.0, 3.0];
    let const_at_3: [f64; 4] = [3.0, 3.0, 3.0, 3.0];
    let motion_up2: [f64; 4] = [3.0, 4.0, 5.0, 6.0];
    let dur = 0.01_f64;

    let curve = multi_piece_axis(&[
        (motion_up, dur),
        (const_at_3, dur),
        (const_at_3, dur),
        (const_at_3, dur),
        (motion_up2, dur),
    ]);

    let seg = ShapedSegment {
        axes: [curve, linear_axis(0.0, 0.0), linear_axis(0.0, 0.0)],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 5.0 * dur,
    };

    let msgs = enqueue_segment(
        &seg,
        &axis_cfg_single(0),
        0.0,
        true,
        0.0,
        crate::pump::MAX_LEAD_SECS,
        |_, hs| (hs * 1e9) as u64,
        None,
    );

    let axis = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 1, axis: 0 })
        .expect("axis 0 must be present");

    assert_eq!(
        axis.pieces.len(),
        3,
        "motion + merged-constant + motion = 3 pieces, got {}",
        axis.pieces.len()
    );

    let (_, host0) = axis.pieces[0];
    let (merged, host1) = axis.pieces[1];
    let (_, host2) = axis.pieces[2];

    assert!(
        (host0 - 0.0).abs() < 1e-12,
        "first piece host_secs must be 0.0"
    );
    assert!(
        (host1 - dur).abs() < 1e-12,
        "constant run starts at t=dur, got {host1}"
    );
    assert!(
        (merged.duration as f64 - 3.0 * dur).abs() < 1e-6,
        "merged constant run duration must be 3*dur, got {}",
        merged.duration
    );
    assert!(
        (host2 - 4.0 * dur).abs() < 1e-12,
        "second motion piece host_secs must be 4*dur, got {host2}"
    );
}

#[test]
fn constant_runs_at_different_values_do_not_merge_across_motion_boundary() {
    let const_a: [f64; 4] = [2.0, 2.0, 2.0, 2.0];
    let transition: [f64; 4] = [2.0, 3.0, 4.0, 5.0];
    let const_b: [f64; 4] = [5.0, 5.0, 5.0, 5.0];
    let dur = 0.01_f64;

    let curve = multi_piece_axis(&[
        (const_a, dur),
        (const_a, dur),
        (transition, dur),
        (const_b, dur),
        (const_b, dur),
    ]);

    let seg = ShapedSegment {
        axes: [curve, linear_axis(0.0, 0.0), linear_axis(0.0, 0.0)],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 4.0 * dur,
    };

    let msgs = enqueue_segment(
        &seg,
        &axis_cfg_single(0),
        0.0,
        true,
        0.0,
        crate::pump::MAX_LEAD_SECS,
        |_, hs| (hs * 1e9) as u64,
        None,
    );

    let axis = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 1, axis: 0 })
        .expect("axis 0 must be present");

    assert_eq!(
        axis.pieces.len(),
        3,
        "const@2×2 + motion + const@5×2 → 3 pieces (two merged constant runs + one motion), got {}",
        axis.pieces.len()
    );

    let (pa, _) = axis.pieces[0];
    let (pb, _) = axis.pieces[1];
    let (pc, _) = axis.pieces[2];

    assert!(
        (pa.coeffs[0] - 2.0_f32).abs() < 1e-5,
        "first merged run must hold value 2.0, got {}",
        pa.coeffs[0]
    );
    assert!(
        (pa.duration as f64 - 2.0 * dur).abs() < 1e-9,
        "first merged duration must be 2*dur, got {}",
        pa.duration
    );

    assert!(
        pb.coeffs.windows(2).all(|w| (w[0] - w[1]).abs() > 1e-5),
        "middle piece (transition) must not be constant"
    );
    assert!(
        (pb.duration as f64 - dur).abs() < 1e-9,
        "transition piece duration must be 1*dur, got {}",
        pb.duration
    );

    assert!(
        (pc.coeffs[0] - 5.0_f32).abs() < 1e-5,
        "second merged run must hold value 5.0, got {}",
        pc.coeffs[0]
    );
    assert!(
        (pc.duration as f64 - 2.0 * dur).abs() < 1e-9,
        "second merged duration must be 2*dur, got {}",
        pc.duration
    );
}

#[test]
fn constant_run_subdivides_under_max_piece_secs_after_merging() {
    let n_knots = 20;
    let piece_dur = 0.005_f64;
    let total = n_knots as f64 * piece_dur;
    let curve = constant_axis(3.0, n_knots, piece_dur);

    let seg = ShapedSegment {
        axes: [curve, linear_axis(0.0, 0.0), linear_axis(0.0, 0.0)],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: total,
    };

    let max_piece = 0.025_f64;
    let msgs = enqueue_segment(
        &seg,
        &axis_cfg_single(0),
        0.0,
        true,
        0.0,
        crate::pump::DRIP_WINDOW_SECS,
        |_, hs| (hs * 1e9) as u64,
        Some(max_piece),
    );

    let axis = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 1, axis: 0 })
        .expect("axis 0 must be present");

    let min_expected = (total / max_piece).floor() as usize;
    assert!(
        axis.pieces.len() >= min_expected,
        "a homing follower's constant run must drip in <= max_piece_secs \
         pieces (a whole-move piece never retires, pinning the cohort \
         watchdog and escaping the dead-man leash); got {} pieces",
        axis.pieces.len()
    );
    let sum: f64 = axis.pieces.iter().map(|(p, _)| p.duration as f64).sum();
    assert!(
        (sum - total).abs() < 1e-5,
        "durations must sum to {total}, got {sum}"
    );
    for (p, _) in &axis.pieces {
        assert!(p.duration as f64 <= max_piece + 1e-6);
        assert!(p.coeffs.iter().all(|&c| (f64::from(c) - 3.0).abs() < 1e-6));
    }
}

#[test]
fn constant_at_or_under_max_piece_secs_stays_whole() {
    let subs = subdivide_bernstein([2.0; 4], 0.020, 0.025);
    assert_eq!(subs.len(), 1);
    assert_eq!(subs[0], ([2.0; 4], 0.020));
}

fn shifted_axis(pieces_bern: &[([f64; 4], f64)], u_base: f64) -> ScalarNurbs<f64> {
    let mut pieces = Vec::with_capacity(pieces_bern.len());
    let mut u = u_base;
    for (bern, dur) in pieces_bern {
        let u_end = u + dur;
        pieces.push(nurbs::bezier::BezierPiece::from_bernstein(bern, u, u_end));
        u = u_end;
    }
    nurbs::bezier::bezier_pieces_to_nurbs(&pieces)
}

#[test]
fn nonzero_curve_base_preserves_host_times() {
    const U_BASE: f64 = 10.0;
    let curve = shifted_axis(
        &[
            ([5.0; 4], 0.4),
            ([5.0; 4], 0.4),
            ([1.0, 2.0, 3.0, 4.0], 0.2),
        ],
        U_BASE,
    );
    let total = 1.0;

    let seg = ShapedSegment {
        axes: [curve, linear_axis(0.0, 0.0), linear_axis(0.0, 0.0)],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: U_BASE,
        t_end: U_BASE + total,
    };

    let t0 = 100.0;
    let msgs = enqueue_segment(
        &seg,
        &axis_cfg_single(0),
        t0,
        true,
        0.0,
        crate::pump::MAX_LEAD_SECS,
        |_, hs| (hs * 1e9) as u64,
        None,
    );
    let axis = msgs
        .iter()
        .find(|m| m.key == AxisKey { mcu_id: 1, axis: 0 })
        .expect("axis 0 must be present");

    assert_eq!(
        axis.pieces.len(),
        2,
        "two constant knots merge, motion stays"
    );
    let host0 = axis.pieces[0].1;
    let host1 = axis.pieces[1].1;
    assert!(
        (host0 - (t0 + U_BASE)).abs() < 1e-9,
        "merged constant must start at t0 + u_base = {}, got {host0}",
        t0 + U_BASE
    );
    assert!(
        (host1 - (t0 + U_BASE + 0.8)).abs() < 1e-9,
        "motion piece must start at t0 + u_base + 0.8 = {}, got {host1}",
        t0 + U_BASE + 0.8
    );
}
