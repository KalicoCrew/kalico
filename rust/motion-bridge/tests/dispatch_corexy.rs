//! Integration tests for the CoreXY host-side motor-frame combine in
//! `dispatch::build_push_params`.
//!
//! These tests live here (not inside `dispatch.rs`'s `#[cfg(test)]`) because
//! the motion-bridge lib binary links pyo3's `extension-module` feature, which
//! requires a live Python interpreter at process start. The lib test binary
//! therefore crashes on the test runner (signal 6, `_PyExc_BaseException` not
//! found) before any test code runs. Integration tests link against the rlib,
//! avoiding the extension-module bootstrap.
//!
//! Covered invariants:
//! - Motor-A slot carries X+Y, motor-B slot carries X−Y (sign correctness).
//! - Multi-piece X with single-piece Y: knot-union path exercises
//!   `add_with_knot_union` and must not panic.
//! - Pure-Y jog: motor-B is X−Y, so the Y contribution is subtracted.
//! - Diagonal jog: both motors step at distinct rates.
//! - `cfgs()` helper with `KINEMATICS_COREXY` tag propagates to `params.kinematics`.

use geometry::segment::EMode;
use kalico_host_rt::producer::CurveLoadParams;
use motion_bridge_native::dispatch::{
    AXIS_X, AXIS_Y, AXIS_Z, KINEMATICS_COREXY, McuAxisConfig, McuCaps, UNUSED_HANDLE,
    build_push_params,
};
use nurbs::ScalarNurbs;
use trajectory::ShapedSegment;

/// Evaluate a `CurveLoadParams` at a global parameter `u ∈ [0, total_duration]`.
///
/// Finds which piece `u` falls in by accumulating `duration_per_piece`, then
/// evaluates the cubic Bernstein polynomial at the local parameter
/// `t = (u - piece_start) / piece_dur` using de Casteljau's algorithm.
///
/// Used by the multi-piece interior-parameter test to assert values at
/// `u = 0.25, 0.5, 0.75` without requiring the full NURBS eval machinery.
fn eval_curve_load_at(params: &CurveLoadParams, u: f64) -> f64 {
    let mut piece_start = 0.0_f64;
    for (i, bp) in params.bp_per_piece.iter().enumerate() {
        let dur = params.duration_per_piece[i] as f64;
        let piece_end = piece_start + dur;
        // Include the right endpoint of the last piece, otherwise check half-open.
        let in_piece = if i + 1 == params.bp_per_piece.len() {
            u <= piece_end + 1e-14
        } else {
            u < piece_end - 1e-14
        };
        if in_piece || i + 1 == params.bp_per_piece.len() {
            // Local parameter in [0, 1].
            let t = ((u - piece_start) / dur).clamp(0.0, 1.0);
            // de Casteljau on the 4 Bernstein control points.
            let b: [f64; 4] = [bp[0] as f64, bp[1] as f64, bp[2] as f64, bp[3] as f64];
            // Level 1.
            let c0 = b[0] + t * (b[1] - b[0]);
            let c1 = b[1] + t * (b[2] - b[1]);
            let c2 = b[2] + t * (b[3] - b[2]);
            // Level 2.
            let d0 = c0 + t * (c1 - c0);
            let d1 = c1 + t * (c2 - c1);
            // Level 3.
            return d0 + t * (d1 - d0);
        }
        piece_start = piece_end;
    }
    unreachable!("u={u} is outside the curve domain");
}

fn linear_curve(a: f64, b: f64) -> ScalarNurbs<f64> {
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

fn corexy_only_cfg() -> Vec<McuAxisConfig> {
    vec![McuAxisConfig {
        mcu_id: 0,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: KINEMATICS_COREXY,
        caps: McuCaps::default(),
    }]
}

fn two_mcu_cfgs() -> Vec<McuAxisConfig> {
    vec![
        McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KINEMATICS_COREXY,
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

/// Pure-X jog: motor-A = X+Y, motor-B = X−Y (sign correctness, fast path).
///
/// Input: X linear 0→10, Y constant 100.
/// Expected: motor-A CPs [100, 103.33, 106.67, 110], motor-B CPs [-100, -96.67, -93.33, -90].
#[test]
fn corexy_pure_x_jog_combines_into_motor_frame_curves() {
    let seg = shaped([linear_curve(0.0, 10.0), constant_curve(100.0), constant_curve(5.0)]);
    let plans = build_push_params(&seg, &two_mcu_cfgs(), 0, 1_000);

    let octopus = plans.iter().find(|p| p.mcu_id == 0).expect("octopus plan");
    assert_eq!(octopus.curves_to_load.len(), 2);
    assert_eq!(octopus.params.kinematics, KINEMATICS_COREXY);

    let (motor_a_slot, motor_a) = &octopus.curves_to_load[0];
    let (motor_b_slot, motor_b) = &octopus.curves_to_load[1];
    assert_eq!(*motor_a_slot, AXIS_X, "motor-A must be in AXIS_X slot");
    assert_eq!(*motor_b_slot, AXIS_Y, "motor-B must be in AXIS_Y slot");

    assert_eq!(motor_a.bp_per_piece.len(), 1);
    let a_cps = motor_a.bp_per_piece[0];
    let a_expected = [100.0_f32, 103.333_333, 106.666_666, 110.0];
    for (k, (&got, &exp)) in a_cps.iter().zip(a_expected.iter()).enumerate() {
        assert!((got - exp).abs() < 1e-3, "motor-A CP[{k}]: got {got}, expected ~{exp}");
    }

    assert_eq!(motor_b.bp_per_piece.len(), 1);
    let b_cps = motor_b.bp_per_piece[0];
    let b_expected = [-100.0_f32, -96.666_666, -93.333_333, -90.0];
    for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
        assert!((got - exp).abs() < 1e-3, "motor-B CP[{k}]: got {got}, expected ~{exp}");
    }

    // Z passes through unchanged on the Cartesian MCU.
    let f446 = plans.iter().find(|p| p.mcu_id == 1).expect("f446 plan");
    assert_eq!(f446.curves_to_load.len(), 1);
    assert_eq!(f446.curves_to_load[0].0, AXIS_Z);
    assert_eq!(f446.params.x_handle_packed, UNUSED_HANDLE);
}

/// Multi-piece knot-union regression. X has two Bézier pieces, Y has one.
/// Without `add_with_knot_union` the old `nurbs::algebra::add` would return
/// `KnotMismatch` and the `.expect(...)` would panic.
///
/// X: linear 0→5 on [0,0.5] then 5→10 on [0.5,1]. Y: constant 20.
/// Expected motor-A: two pieces, values 20→25→30. Motor-B: -20→-15→-10.
#[test]
fn corexy_multi_piece_x_knot_union_combines_correctly() {
    use nurbs::bezier::{BezierPiece, bezier_pieces_to_nurbs};

    let piece1 = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 0.5,
        // linear 0→5 in Pascal-shifted basis at 0: c0=0, c1=10, so at u=0.5: 0+10*0.5=5.
        coeffs: vec![0.0, 10.0, 0.0, 0.0],
    };
    let piece2 = BezierPiece::<f64> {
        u_start: 0.5,
        u_end: 1.0,
        // linear 5→10 in Pascal-shifted basis at 0.5: c0=5, c1=10.
        coeffs: vec![5.0, 10.0, 0.0, 0.0],
    };
    let x_two_piece = bezier_pieces_to_nurbs(&[piece1, piece2]);
    let y_const = constant_curve(20.0);

    let seg = ShapedSegment {
        axes: [x_two_piece, y_const, constant_curve(0.0)],
        e_mode: EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: 1.0,
    };
    let plans = build_push_params(&seg, &corexy_only_cfg(), 0, 1_000);

    let plan = plans.iter().find(|p| p.mcu_id == 0).expect("plan");
    assert_eq!(plan.curves_to_load.len(), 2);
    let (_, motor_a) = &plan.curves_to_load[0];
    let (_, motor_b) = &plan.curves_to_load[1];

    // Both motor curves must be two pieces (X had 2 pieces, Y had 1; union → 2).
    assert_eq!(motor_a.bp_per_piece.len(), 2, "motor-A must be 2 pieces after knot union");
    assert_eq!(motor_b.bp_per_piece.len(), 2, "motor-B must be 2 pieces after knot union");

    // Motor-A = X+Y: at u=0 → 0+20=20; at u=1 → 10+20=30.
    let a0 = motor_a.bp_per_piece[0];
    assert!((a0[0] as f64 - 20.0).abs() < 0.01, "motor-A piece0 CP0: got {}", a0[0]);
    let a1 = motor_a.bp_per_piece[1];
    assert!((a1[3] as f64 - 30.0).abs() < 0.01, "motor-A piece1 CP3: got {}", a1[3]);

    // Motor-B = X-Y: at u=0 → 0-20=-20; at u=1 → 10-20=-10.
    let b0 = motor_b.bp_per_piece[0];
    assert!((b0[0] as f64 - (-20.0)).abs() < 0.01, "motor-B piece0 CP0: got {}", b0[0]);
    let b1 = motor_b.bp_per_piece[1];
    assert!((b1[3] as f64 - (-10.0)).abs() < 0.01, "motor-B piece1 CP3: got {}", b1[3]);

    // Interior-parameter evaluation via de Casteljau on the Bernstein CPs.
    //
    // X(u) = 10u (linear), Y = 20 (constant).
    // Motor-A(u) = 10u + 20; motor-B(u) = 10u - 20.
    //
    // These assertions exercise the four interior control points that the
    // endpoint-only check above leaves uncovered. A bug in `split_piece_at`
    // at the interior breakpoint u=0.5, or a recompose error at the shared
    // break, would produce wrong values at u=0.25, 0.5, or 0.75 while
    // keeping the endpoint CPs intact, so this detects those failure modes.
    const INNER_TOL: f64 = 1e-9;
    let inner_cases_a: [(f64, f64); 3] = [(0.25, 22.5), (0.5, 25.0), (0.75, 27.5)];
    for (u, expected) in inner_cases_a {
        let got = eval_curve_load_at(motor_a, u);
        assert!(
            (got - expected).abs() < INNER_TOL,
            "motor-A interior u={u}: expected {expected}, got {got} (diff={diff:.2e})",
            diff = (got - expected).abs(),
        );
    }
    let inner_cases_b: [(f64, f64); 3] = [(0.25, -17.5), (0.5, -15.0), (0.75, -12.5)];
    for (u, expected) in inner_cases_b {
        let got = eval_curve_load_at(motor_b, u);
        assert!(
            (got - expected).abs() < INNER_TOL,
            "motor-B interior u={u}: expected {expected}, got {got} (diff={diff:.2e})",
            diff = (got - expected).abs(),
        );
    }
}

/// Pure-Y jog: motor-B = X−Y, so Y is subtracted from X.
///
/// Input: X constant 50, Y linear 0→10.
/// Motor-A = 50+Y: [50, 53.33, 56.67, 60]. Motor-B = 50-Y: [50, 46.67, 43.33, 40].
#[test]
fn corexy_pure_y_jog_motor_b_has_negative_y_contribution() {
    let seg = shaped([constant_curve(50.0), linear_curve(0.0, 10.0), constant_curve(0.0)]);
    let plans = build_push_params(&seg, &corexy_only_cfg(), 0, 1_000);

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
        assert!((got - exp).abs() < 1e-3, "motor-A CP[{k}]: got {got}, expected ~{exp}");
    }

    assert_eq!(motor_b.bp_per_piece.len(), 1);
    let b_cps = motor_b.bp_per_piece[0];
    // B = X - Y = 50 - [0, 3.33, 6.67, 10] = [50, 46.67, 43.33, 40].
    let b_expected = [50.0_f32, 46.666_666, 43.333_333, 40.0];
    for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
        assert!((got - exp).abs() < 1e-3, "motor-B CP[{k}]: got {got}, expected ~{exp}");
    }
}

/// Diagonal jog: X linear 0→6, Y linear 0→4.
/// Motor-A = X+Y: linear 0→10. Motor-B = X-Y: linear 0→2.
/// Both motors step at distinct rates — neither is zero.
#[test]
fn corexy_diagonal_jog_both_motors_step_at_distinct_rates() {
    let seg = shaped([linear_curve(0.0, 6.0), linear_curve(0.0, 4.0), constant_curve(0.0)]);
    let plans = build_push_params(&seg, &corexy_only_cfg(), 0, 1_000);

    let plan = plans.iter().find(|p| p.mcu_id == 0).expect("plan");
    assert_eq!(plan.curves_to_load.len(), 2);
    let (_, motor_a) = &plan.curves_to_load[0];
    let (_, motor_b) = &plan.curves_to_load[1];

    assert_eq!(motor_a.bp_per_piece.len(), 1);
    assert_eq!(motor_b.bp_per_piece.len(), 1);

    let a_cps = motor_a.bp_per_piece[0];
    let b_cps = motor_b.bp_per_piece[0];

    // Motor-A = X+Y: 0→10.
    let a_expected = [0.0_f32, 3.333_333, 6.666_666, 10.0];
    for (k, (&got, &exp)) in a_cps.iter().zip(a_expected.iter()).enumerate() {
        assert!((got - exp).abs() < 1e-3, "motor-A CP[{k}]: got {got}, expected ~{exp}");
    }

    // Motor-B = X-Y: 0→2.
    let b_expected = [0.0_f32, 0.666_666, 1.333_333, 2.0];
    for (k, (&got, &exp)) in b_cps.iter().zip(b_expected.iter()).enumerate() {
        assert!((got - exp).abs() < 1e-3, "motor-B CP[{k}]: got {got}, expected ~{exp}");
    }

    // Sanity: A and B differ.
    assert!(
        (a_cps[3] - b_cps[3]).abs() > 0.1,
        "motor-A and motor-B must differ on a diagonal jog"
    );
}
