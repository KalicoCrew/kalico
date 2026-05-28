use super::*;
use geometry::segment::EMode;
use nurbs::ScalarNurbs;

fn make_curve(n_pieces: usize, piece_dur: f32, slope: f32) -> CurveLoadParams {
    let mut bp = Vec::with_capacity(n_pieces);
    let mut dur = Vec::with_capacity(n_pieces);
    for i in 0..n_pieces {
        let v0 = slope * i as f32;
        let v1 = slope * (i as f32 + 1.0 / 3.0);
        let v2 = slope * (i as f32 + 2.0 / 3.0);
        let v3 = slope * (i as f32 + 1.0);
        bp.push([v0, v1, v2, v3]);
        dur.push(piece_dur);
    }
    CurveLoadParams {
        bp_per_piece: bp,
        duration_per_piece: dur,
    }
}

#[test]
fn de_casteljau_split_at_midpoint() {
    let bp: [f32; 4] = [0.0, 1.0, 2.0, 3.0];
    let (left, right) = super::de_casteljau_split(bp, 0.5);
    assert!((left[0] - 0.0).abs() < 1e-6);
    assert!((left[3] - 1.5).abs() < 1e-6);
    assert!((right[0] - 1.5).abs() < 1e-6);
    assert!((right[3] - 3.0).abs() < 1e-6);
    assert!((left[3] - right[0]).abs() < 1e-6);
}

#[test]
fn de_casteljau_split_at_quarter() {
    let bp: [f32; 4] = [0.0, 0.0, 0.0, 12.0];
    let (left, right) = super::de_casteljau_split(bp, 0.25);
    assert!((left[0] - 0.0).abs() < 1e-5, "left start");
    assert!(
        (left[3] - 0.1875).abs() < 1e-4,
        "left end = eval(0.25) got {}",
        left[3]
    );
    assert!((right[0] - 0.1875).abs() < 1e-4, "right start");
    assert!((right[3] - 12.0).abs() < 1e-5, "right end");
}

#[test]
fn extract_time_window_full_range_is_identity() {
    let curve = make_curve(5, 0.1, 1.0);
    let result = super::extract_time_window(&curve, 0.0, 0.5);
    assert_eq!(result.piece_count(), 5);
    assert_eq!(result.bp_per_piece, curve.bp_per_piece);
}

#[test]
fn extract_time_window_first_half() {
    let curve = make_curve(10, 0.1, 1.0);
    let result = super::extract_time_window(&curve, 0.0, 0.5);
    assert_eq!(result.piece_count(), 5);
    for i in 0..5 {
        assert_eq!(result.bp_per_piece[i], curve.bp_per_piece[i]);
    }
}

#[test]
fn extract_time_window_mid_piece_boundary_uses_de_casteljau() {
    let curve = make_curve(4, 1.0, 1.0);
    let result = super::extract_time_window(&curve, 0.0, 2.5);
    assert_eq!(result.piece_count(), 3, "2 whole + 1 subdivided");
    assert_eq!(result.bp_per_piece[0], curve.bp_per_piece[0]);
    assert_eq!(result.bp_per_piece[1], curve.bp_per_piece[1]);
    assert!((result.duration_per_piece[2] - 0.5).abs() < 1e-5);
    assert!((result.bp_per_piece[2][0] - curve.bp_per_piece[2][0]).abs() < 1e-5);
}

#[test]
fn extract_time_window_second_half_starts_mid_piece() {
    let curve = make_curve(4, 1.0, 1.0);
    let result = super::extract_time_window(&curve, 2.5, 4.0);
    assert_eq!(result.piece_count(), 2, "1 subdivided + 1 whole");
    assert!((result.duration_per_piece[0] - 0.5).abs() < 1e-5);
    assert_eq!(result.bp_per_piece[1], curve.bp_per_piece[3]);
}

#[test]
fn split_plan_no_split_needed() {
    let curve = make_curve(5, 0.1, 1.0);
    let plan = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, curve)],
        params: SegmentPushParams {
            id: 0,
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: 1000,
            t_end: 2000,
            kinematics: 0,
            e_mode: 2,
            extrusion_ratio: 0.0,
        },
    };
    let result = super::split_plan_if_needed(plan, 10, 1_000_000.0).unwrap();
    assert_eq!(result.len(), 1, "no split needed");
    assert_eq!(result[0].curves_to_load[0].1.piece_count(), 5);
}

#[test]
fn split_plan_equal_axes_splits_correctly() {
    // 10 pieces per axis, max_pieces=4 → stride=2
    let curve_x = make_curve(10, 0.1, 1.0);
    let curve_y = make_curve(10, 0.1, 2.0);
    let plan = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, curve_x), (AXIS_Y, curve_y)],
        params: SegmentPushParams {
            id: 0,
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: 0,
            t_end: 1_000_000,
            kinematics: 0,
            e_mode: 2,
            extrusion_ratio: 0.0,
        },
    };
    let result = super::split_plan_if_needed(plan, 4, 1_000_000.0).unwrap();
    assert!(
        result.len() >= 2,
        "should produce multiple chunks, got {}",
        result.len()
    );
    for chunk in &result {
        for (_, curve) in &chunk.curves_to_load {
            assert!(
                curve.piece_count() <= 4,
                "chunk has {} pieces",
                curve.piece_count()
            );
        }
    }
    // Timing continuity
    assert_eq!(result[0].params.t_start, 0);
    assert_eq!(result.last().unwrap().params.t_end, 1_000_000);
    for i in 1..result.len() {
        assert_eq!(
            result[i].params.t_start,
            result[i - 1].params.t_end,
            "timing gap between chunks {} and {}",
            i - 1,
            i
        );
    }
}

#[test]
fn split_plan_unequal_axes_uses_de_casteljau() {
    // X has 10 pieces (each 0.1s), Z has 3 pieces (each ~0.333s).
    // max_pieces=4 → stride=2. Z pieces straddle X boundaries → de Casteljau.
    let curve_x = make_curve(10, 0.1, 1.0);
    let dur_z = 1.0_f32 / 3.0;
    let curve_z = make_curve(3, dur_z, 5.0);
    let plan = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, curve_x), (AXIS_Z, curve_z)],
        params: SegmentPushParams {
            id: 0,
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: 0,
            t_end: 1_000_000,
            kinematics: 2,
            e_mode: 2,
            extrusion_ratio: 0.0,
        },
    };
    let result = super::split_plan_if_needed(plan, 4, 1_000_000.0).unwrap();
    assert!(result.len() >= 3, "should produce multiple chunks");
    for chunk in &result {
        for (_, curve) in &chunk.curves_to_load {
            assert!(
                curve.piece_count() <= 4,
                "axis piece count {} exceeds max 4",
                curve.piece_count()
            );
        }
        assert_eq!(chunk.curves_to_load.len(), 2, "both axes in every chunk");
    }
}

#[test]
fn split_plan_preserves_e_mode_and_extrusion_ratio() {
    let curve = make_curve(10, 0.1, 1.0);
    let plan = McuPushPlan {
        mcu_id: 7,
        curves_to_load: vec![(AXIS_X, curve)],
        params: SegmentPushParams {
            id: 0,
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: 0,
            t_end: 1_000_000,
            kinematics: 0,
            e_mode: 1,
            extrusion_ratio: 0.042,
        },
    };
    let result = super::split_plan_if_needed(plan, 4, 1_000_000.0).unwrap();
    for chunk in &result {
        assert_eq!(chunk.params.e_mode, 1);
        assert!((chunk.params.extrusion_ratio - 0.042).abs() < 1e-6);
        assert_eq!(chunk.params.kinematics, 0);
        assert_eq!(chunk.mcu_id, 7);
    }
}

#[test]
fn split_plan_cap_below_3_errors_only_when_splitting_needed() {
    // 2 pieces, cap=2 → no split → ok
    let small = make_curve(2, 0.1, 1.0);
    let plan_ok = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, small)],
        params: SegmentPushParams {
            id: 0,
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: 0,
            t_end: 1000,
            kinematics: 0,
            e_mode: 2,
            extrusion_ratio: 0.0,
        },
    };
    assert!(super::split_plan_if_needed(plan_ok, 2, 1e6).is_ok());

    // 5 pieces, cap=2 → split needed, cap too low → error
    let big = make_curve(5, 0.1, 1.0);
    let plan_err = McuPushPlan {
        mcu_id: 0,
        curves_to_load: vec![(AXIS_X, big)],
        params: SegmentPushParams {
            id: 0,
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: 0,
            t_end: 1000,
            kinematics: 0,
            e_mode: 2,
            extrusion_ratio: 0.0,
        },
    };
    assert!(super::split_plan_if_needed(plan_err, 2, 1e6).is_err());
}

fn linear_curve(a: f64, b: f64) -> ScalarNurbs<f64> {
    // degree-3 Bézier with collinear cps a, lerp(1/3), lerp(2/3), b
    let cps = vec![a, a + (b - a) / 3.0, a + 2.0 * (b - a) / 3.0, b];
    ScalarNurbs::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], cps, None).unwrap()
}

fn constant_curve(v: f64) -> ScalarNurbs<f64> {
    ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![v, v, v, v],
        None,
    )
    .unwrap()
}

fn shaped(axes: [ScalarNurbs<f64>; 3]) -> ShapedSegment {
    ShapedSegment {
        axes,
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 1.0,
    }
}

fn cfgs() -> Vec<McuAxisConfig> {
    vec![
        McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KINEMATICS_COREXY, // 0
            caps: McuCaps::default(),
        },
        McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_Z],
            kinematics: 2,
            caps: McuCaps::default(),
        },
    ]
}

/// **Post-2026-05-11.** Every kinematic axis listed in the MCU's
/// `cfg.axes` gets a curve, even when the curve is trivially
/// constant. This matches the new "axis is absolute coordinate"
/// semantic on the MCU side — `UNUSED` handles mean "hold at
/// `prev_value`", and the bridge must therefore anchor each axis to
/// klippy's commanded position on every segment.
#[test]
fn x_move_sends_curves_for_every_kinematic_axis_on_each_mcu() {
    // X varies, Y and Z constant. cfgs[0] (Octopus) drives X+Y,
    // cfgs[1] (F446) drives Z. Both MCUs must receive curves.
    let seg = shaped([
        linear_curve(0.0, 10.0),
        constant_curve(100.0),
        constant_curve(5.0),
    ]);
    let plans = build_push_params(&seg, &cfgs(), 1_000, 2_000);

    assert_eq!(plans.len(), 2, "both MCUs should get a plan");

    // Octopus: X + Y (both kinematic axes for this MCU).
    let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus plan");
    assert_eq!(octopus.curves_to_load.len(), 2);
    assert_eq!(octopus.curves_to_load[0].0, AXIS_X);
    assert_eq!(octopus.curves_to_load[1].0, AXIS_Y);
    assert_eq!(octopus.params.kinematics, KINEMATICS_COREXY);

    // F446: Z (only kinematic axis for this MCU).
    let f446 = plans.iter().find(|p| p.mcu_id == 1).expect("f446 plan");
    assert_eq!(f446.curves_to_load.len(), 1);
    assert_eq!(f446.curves_to_load[0].0, AXIS_Z);
    assert_eq!(f446.params.kinematics, 2);

    // All handle packed fields still default to UNUSED — the caller
    // fills them after `load_curve` returns.
    assert_eq!(octopus.params.x_handle_packed, UNUSED_HANDLE);
    assert_eq!(octopus.params.y_handle_packed, UNUSED_HANDLE);
    assert_eq!(octopus.params.t_start, 1_000);
    assert_eq!(octopus.params.t_end, 2_000);
    assert_eq!(octopus.params.e_mode, 2);
}

#[test]
fn z_move_sends_curves_for_every_kinematic_axis_on_each_mcu() {
    // Z varies, X+Y constant. Both MCUs still get plans because
    // both have kinematic axes that need to be anchored.
    let seg = shaped([
        constant_curve(50.0),
        constant_curve(100.0),
        linear_curve(0.0, 5.0),
    ]);
    let plans = build_push_params(&seg, &cfgs(), 1_000, 2_000);

    assert_eq!(plans.len(), 2);

    let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus");
    assert_eq!(octopus.curves_to_load.len(), 2);
    let f446 = plans.iter().find(|p| p.mcu_id == 1).expect("f446");
    assert_eq!(f446.curves_to_load.len(), 1);
}

#[test]
fn set_handle_fills_correct_field() {
    let seg = shaped([
        linear_curve(0.0, 10.0),
        constant_curve(100.0),
        constant_curve(5.0),
    ]);
    let mut plans = build_push_params(&seg, &cfgs(), 0, 100);
    // Find the Octopus plan; it's no longer guaranteed to be plans[0]
    // since the iteration order over a Vec preserves cfg order, but
    // be explicit anyway.
    let octopus_idx = plans.iter().position(|p| p.mcu_id == 0).expect("octopus");
    plans[octopus_idx].set_handle(AXIS_X, 0xCAFE);
    assert_eq!(plans[octopus_idx].params.x_handle_packed, 0xCAFE);
    assert_eq!(plans[octopus_idx].params.y_handle_packed, UNUSED_HANDLE);
}

/// **Regression for bench session 2026-05-11.** Verify the bridge
/// sends Y curves for pure-X jogs. The old `is_trivially_constant`
/// skip would have omitted Y here, leaving the MCU's Y handle at
/// UNUSED_SENTINEL — which the engine then evaluated as `y = 0.0`
/// instead of holding `prev_y`. The first segment whose Y curve
/// non-trivially-noise crossed the 1e-12 threshold would suddenly
/// produce `y = 100` instead of `y = 0`, generating a 100 mm delta
/// on motor A (CoreXY: `motor_a = x + y`) and tripping
/// STEP_BURST_EXCEEDED.
#[test]
fn constant_y_axis_for_pure_x_move_still_sends_a_curve() {
    let seg = shaped([
        linear_curve(0.0, 25.0),
        constant_curve(100.0),
        constant_curve(10.0),
    ]);
    let plans = build_push_params(&seg, &cfgs(), 0, 1_000);

    let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus");
    let has_x = octopus
        .curves_to_load
        .iter()
        .any(|(axis, _)| *axis == AXIS_X);
    let has_y = octopus
        .curves_to_load
        .iter()
        .any(|(axis, _)| *axis == AXIS_Y);
    assert!(has_x, "X curve must be sent on a pure-X move");
    assert!(
        has_y,
        "Y curve must be sent even when constant — the engine's UNUSED-handle hold semantic needs prev_y anchored to klippy's commanded Y for cross-segment continuity"
    );
}

/// **Regression for bench CoreXY diagonal bug (2026-05-21).** A pure-X
/// jog with `KINEMATICS_COREXY` must produce motor-frame curves in the
/// X/Y handle slots rather than logical X/Y curves.
///
/// Input: X linear 0 → 10, Y constant 100, Z constant 5.
/// Expected motor-frame output:
///   - AXIS_X slot carries motor-A curve (X+Y): CPs [100, 103.33, 106.67, 110]
///   - AXIS_Y slot carries motor-B curve (X-Y): CPs [-100, -96.67, -93.33, -90]
///
/// The test uses a Cartesian MCU for Z (mcu_id=1) to verify Z passes
/// through unchanged, and checks that the slot *indices* are still
/// AXIS_X / AXIS_Y (the MCU reads motor-A from x_handle, motor-B from
/// y_handle by convention).
#[test]
fn corexy_pure_x_jog_combines_into_motor_frame_curves() {
    // Build a two-MCU config: Octopus (CoreXY, mcu_id=0) + F446 (Z, mcu_id=1).
    let corexy_cfgs = vec![
        McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KINEMATICS_COREXY, // 0
            caps: McuCaps::default(),
        },
        McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_Z],
            kinematics: 2, // CartesianXyzAndE — Z passthrough
            caps: McuCaps::default(),
        },
    ];

    // X: linear 0 → 10 (CPs: 0, 3.333, 6.667, 10)
    // Y: constant 100 (CPs: 100, 100, 100, 100)
    // Z: constant 5
    let seg = shaped([
        linear_curve(0.0, 10.0),
        constant_curve(100.0),
        constant_curve(5.0),
    ]);
    let plans = build_push_params(&seg, &corexy_cfgs, 0, 1_000);

    let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus plan");
    assert_eq!(octopus.curves_to_load.len(), 2);

    // Slot ordering: AXIS_X then AXIS_Y (matches cfg.axes iteration order).
    let (motor_a_slot, motor_a_params) = &octopus.curves_to_load[0];
    let (motor_b_slot, motor_b_params) = &octopus.curves_to_load[1];
    assert_eq!(*motor_a_slot, AXIS_X, "motor-A must be in AXIS_X slot");
    assert_eq!(*motor_b_slot, AXIS_Y, "motor-B must be in AXIS_Y slot");

    // Motor-A = X + Y. Single-piece Bézier with CPs [0+100, 3.33+100, 6.67+100, 10+100].
    assert_eq!(
        motor_a_params.bp_per_piece.len(),
        1,
        "single-piece input → single-piece output"
    );
    let a_cps = motor_a_params.bp_per_piece[0];
    let a_expected = [100.0_f32, 103.333_333, 106.666_666, 110.0];
    for (k, (&got, &exp)) in a_cps.iter().zip(a_expected.iter()).enumerate() {
        assert!(
            (got - exp).abs() < 1e-3,
            "motor-A CP[{k}]: got {got}, expected ~{exp}"
        );
    }

    // Motor-B = X - Y. CPs [0-100, 3.33-100, 6.67-100, 10-100].
    assert_eq!(
        motor_b_params.bp_per_piece.len(),
        1,
        "single-piece input → single-piece output"
    );
    let b_cps = motor_b_params.bp_per_piece[0];
    let b_expected = [-100.0_f32, -96.666_666, -93.333_333, -90.0];
    for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
        assert!(
            (got - exp).abs() < 1e-3,
            "motor-B CP[{k}]: got {got}, expected ~{exp}"
        );
    }

    // Z passes through unchanged (F446 mcu_id=1 is Cartesian, not CoreXY).
    let f446 = plans.iter().find(|p| p.mcu_id == 1).expect("f446 plan");
    assert_eq!(f446.curves_to_load.len(), 1);
    assert_eq!(f446.curves_to_load[0].0, AXIS_Z);
}

/// **Regression: multi-piece knot-vector mismatch.** Exercises the knot-union
/// path in `add_with_knot_union`. X has two Bézier pieces (a two-segment cubic
/// spline on [0,1]) while Y is a single-piece constant. Without the knot-union
/// pass the old `nurbs::algebra::add` would return `KnotMismatch` and the
/// `.expect(...)` would have panicked in release.
///
/// X: two-piece cubic: piece 1 = linear 0→5 on [0, 0.5], piece 2 = linear 5→10 on [0.5, 1].
/// Y: constant 20.
/// Expected motor-A: X+Y = two-piece, values 20→25→30.
/// Expected motor-B: X-Y = two-piece, values -20→-15→-10.
#[test]
fn corexy_multi_piece_x_knot_union_combines_correctly() {
    use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};

    // Build a two-piece X curve by constructing two Bézier pieces and recomposing.
    let piece1 = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 0.5,
        // linear 0→5: Pascal-shifted at 0.0; coeffs = [0, 10] (c0 + c1*(u-0))
        // At u=0.5: 0 + 10*0.5 = 5. Correct.
        coeffs: vec![0.0, 10.0, 0.0, 0.0],
    };
    let piece2 = BezierPiece::<f64> {
        u_start: 0.5,
        u_end: 1.0,
        // linear 5→10: Pascal-shifted at 0.5; c0=5, c1=10.
        coeffs: vec![5.0, 10.0, 0.0, 0.0],
    };
    let x_two_piece = bezier_pieces_to_nurbs(&[piece1, piece2]);

    // Y: single-piece constant 20.
    let y_const = constant_curve(20.0);

    let corexy_cfg = vec![McuAxisConfig {
        mcu_id: 0,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: KINEMATICS_COREXY,
        caps: McuCaps::default(),
    }];
    let seg = ShapedSegment {
        axes: [x_two_piece, y_const, constant_curve(0.0)],
        e_mode: geometry::segment::EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 1.0,
    };
    let plans = build_push_params(&seg, &corexy_cfg, 0, 1_000);

    let plan = plans.iter().find(|p| p.mcu_id == 0).expect("plan");
    // Both motor curves must be present and be two-piece (X had 2 pieces, Y had 1;
    // union produces 2 pieces).
    assert_eq!(plan.curves_to_load.len(), 2);
    let (_, motor_a) = &plan.curves_to_load[0];
    let (_, motor_b) = &plan.curves_to_load[1];

    // Motor-A = X+Y: two pieces — values at u=0, 0.5, 1.0 should be 20, 25, 30.
    assert_eq!(motor_a.bp_per_piece.len(), 2, "motor-A must be 2 pieces");
    // First CP of piece 0 ≈ X(0)+Y(0) = 0+20 = 20.
    let a0 = motor_a.bp_per_piece[0];
    assert!(
        (a0[0] as f64 - 20.0).abs() < 0.01,
        "motor-A piece0 CP0: got {}",
        a0[0]
    );
    // Last CP of piece 1 ≈ X(1)+Y(1) = 10+20 = 30.
    let a1 = motor_a.bp_per_piece[1];
    assert!(
        (a1[3] as f64 - 30.0).abs() < 0.01,
        "motor-A piece1 CP3: got {}",
        a1[3]
    );

    // Motor-B = X-Y: two pieces — values at u=0, 0.5, 1.0 should be -20, -15, -10.
    assert_eq!(motor_b.bp_per_piece.len(), 2, "motor-B must be 2 pieces");
    let b0 = motor_b.bp_per_piece[0];
    assert!(
        (b0[0] as f64 - (-20.0)).abs() < 0.01,
        "motor-B piece0 CP0: got {}",
        b0[0]
    );
    let b1 = motor_b.bp_per_piece[1];
    assert!(
        (b1[3] as f64 - (-10.0)).abs() < 0.01,
        "motor-B piece1 CP3: got {}",
        b1[3]
    );
}

/// **Pure-Y jog — sign of motor-B path.** X is constant, Y is linear.
/// Motor-A = X+Y (positive), Motor-B = X-Y (negative Y contribution).
///
/// Input: X constant 50, Y linear 0→10.
/// Motor-A = 50+Y: CPs [50, 53.33, 56.67, 60].
/// Motor-B = 50-Y: CPs [50, 46.67, 43.33, 40].
#[test]
fn corexy_pure_y_jog_motor_b_has_negative_y_contribution() {
    let corexy_cfg = vec![McuAxisConfig {
        mcu_id: 0,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: KINEMATICS_COREXY,
        caps: McuCaps::default(),
    }];
    // X constant 50, Y linear 0→10.
    let seg = shaped([
        constant_curve(50.0),
        linear_curve(0.0, 10.0),
        constant_curve(0.0),
    ]);
    let plans = build_push_params(&seg, &corexy_cfg, 0, 1_000);

    let plan = plans.iter().find(|p| p.mcu_id == 0).expect("plan");
    assert_eq!(plan.curves_to_load.len(), 2);
    let (a_slot, motor_a) = &plan.curves_to_load[0];
    let (b_slot, motor_b) = &plan.curves_to_load[1];
    assert_eq!(*a_slot, AXIS_X);
    assert_eq!(*b_slot, AXIS_Y);

    assert_eq!(motor_a.bp_per_piece.len(), 1);
    let a_cps = motor_a.bp_per_piece[0];
    let a_expected = [50.0_f32, 53.333_333, 56.666_666, 60.0];
    for (k, (&got, &exp)) in a_cps.iter().zip(a_expected.iter()).enumerate() {
        assert!(
            (got - exp).abs() < 1e-3,
            "motor-A CP[{k}]: got {got}, expected ~{exp}"
        );
    }

    assert_eq!(motor_b.bp_per_piece.len(), 1);
    let b_cps = motor_b.bp_per_piece[0];
    // B = X - Y = 50 - [0, 3.33, 6.67, 10] = [50, 46.67, 43.33, 40]
    let b_expected = [50.0_f32, 46.666_666, 43.333_333, 40.0];
    for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
        assert!(
            (got - exp).abs() < 1e-3,
            "motor-B CP[{k}]: got {got}, expected ~{exp}"
        );
    }
}

/// **Diagonal jog — both motors step at distinct rates.** X and Y are both
/// linear but with different slopes. Motor-A and Motor-B must both be
/// non-constant and differ from each other.
///
/// Input: X linear 0→6, Y linear 0→4.
/// Motor-A = X+Y: CPs [0, 3.33, 6.67, 10].
/// Motor-B = X-Y: CPs [0, 0.67, 1.33, 2].
#[test]
fn corexy_diagonal_jog_both_motors_step_at_distinct_rates() {
    let corexy_cfg = vec![McuAxisConfig {
        mcu_id: 0,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: KINEMATICS_COREXY,
        caps: McuCaps::default(),
    }];
    // X linear 0→6, Y linear 0→4.
    let seg = shaped([
        linear_curve(0.0, 6.0),
        linear_curve(0.0, 4.0),
        constant_curve(0.0),
    ]);
    let plans = build_push_params(&seg, &corexy_cfg, 0, 1_000);

    let plan = plans.iter().find(|p| p.mcu_id == 0).expect("plan");
    assert_eq!(plan.curves_to_load.len(), 2);
    let (_, motor_a) = &plan.curves_to_load[0];
    let (_, motor_b) = &plan.curves_to_load[1];

    assert_eq!(motor_a.bp_per_piece.len(), 1);
    assert_eq!(motor_b.bp_per_piece.len(), 1);

    let a_cps = motor_a.bp_per_piece[0];
    let b_cps = motor_b.bp_per_piece[0];

    // Motor-A = X+Y: linear 0→10. CPs [0, 3.33, 6.67, 10].
    let a_expected = [0.0_f32, 3.333_333, 6.666_666, 10.0];
    for (k, (&got, &exp)) in a_cps.iter().zip(a_expected.iter()).enumerate() {
        assert!(
            (got - exp).abs() < 1e-3,
            "motor-A CP[{k}]: got {got}, expected ~{exp}"
        );
    }

    // Motor-B = X-Y: linear 0→2. CPs [0, 0.667, 1.333, 2].
    let b_expected = [0.0_f32, 0.666_666, 1.333_333, 2.0];
    for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
        assert!(
            (got - exp).abs() < 1e-3,
            "motor-B CP[{k}]: got {got}, expected ~{exp}"
        );
    }

    // Sanity: motor-A and motor-B differ (they cannot be equal on a diagonal).
    let a_last = a_cps[3];
    let b_last = b_cps[3];
    assert!(
        (a_last - b_last).abs() > 0.1,
        "motor-A and motor-B must differ on a diagonal jog"
    );
}
