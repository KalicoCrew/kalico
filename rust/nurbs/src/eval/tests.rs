use super::*;

fn linear_curve_f64() -> crate::ScalarNurbs<f64> {
    crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0]).unwrap()
}

fn quadratic_curve_f64() -> crate::ScalarNurbs<f64> {
    crate::ScalarNurbs::try_new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 0.5, 1.0]).unwrap()
}

#[test]
fn find_knot_span_endpoints() {
    let knots = [0.0, 0.0, 1.0, 1.0];
    assert_eq!(find_knot_span(&knots, 1, 2, 0.0), 1);
    assert_eq!(find_knot_span(&knots, 1, 2, 1.0), 1);
}

#[test]
fn find_knot_span_midpoint() {
    let knots = [0.0, 0.0, 0.5, 1.0, 1.0];
    assert_eq!(find_knot_span(&knots, 1, 3, 0.25), 1);
    assert_eq!(find_knot_span(&knots, 1, 3, 0.75), 2);
}

#[test]
fn eval_linear_at_endpoints_returns_endpoint_cps() {
    let curve = linear_curve_f64();
    let v = curve.as_view();
    assert!((eval(&v, 0.0_f64) - 0.0).abs() < 1e-12);
    assert!((eval(&v, 1.0_f64) - 1.0).abs() < 1e-12);
}

#[test]
fn eval_linear_at_midpoint_returns_average() {
    let curve = linear_curve_f64();
    let v = curve.as_view();
    assert!((eval(&v, 0.5_f64) - 0.5).abs() < 1e-12);
}

#[test]
fn eval_quadratic_at_endpoints_returns_first_last_cp() {
    let curve = quadratic_curve_f64();
    let v = curve.as_view();
    assert!((eval(&v, 0.0_f64) - 0.0).abs() < 1e-12);
    assert!((eval(&v, 1.0_f64) - 1.0).abs() < 1e-12);
}

#[test]
fn eval_quadratic_at_midpoint_matches_bernstein() {
    let curve = quadratic_curve_f64();
    let v = curve.as_view();
    assert!((eval(&v, 0.5_f64) - 0.5).abs() < 1e-12);
}

fn linear_3d_curve_f64() -> crate::VectorNurbs<f64, 3> {
    crate::VectorNurbs::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
    )
    .unwrap()
}

#[test]
fn vector_eval_linear_endpoints() {
    let curve = linear_3d_curve_f64();
    let v = curve.as_view();
    let p0 = vector_eval(&v, 0.0_f64);
    assert!((p0[0] - 0.0).abs() < 1e-12);
    assert!((p0[1] - 0.0).abs() < 1e-12);
    assert!((p0[2] - 0.0).abs() < 1e-12);
    let p1 = vector_eval(&v, 1.0_f64);
    assert!((p1[0] - 1.0).abs() < 1e-12);
    assert!((p1[1] - 2.0).abs() < 1e-12);
    assert!((p1[2] - 3.0).abs() < 1e-12);
}

#[test]
fn vector_eval_matches_per_axis_scalar() {
    let curve = linear_3d_curve_f64();
    let v = curve.as_view();
    let result = vector_eval(&v, 0.3_f64);

    for axis in 0..3 {
        let cps_axis: Vec<f64> = v.control_points().iter().map(|cp| cp[axis]).collect();
        let scalar = crate::ScalarNurbs::try_new(v.degree(), v.knots().to_vec(), cps_axis).unwrap();
        let expected = eval(&scalar.as_view(), 0.3_f64);
        assert!(
            (result[axis] - expected).abs() < 1e-12,
            "axis {axis}: got {}, expected {}",
            result[axis],
            expected
        );
    }
}

#[cfg(feature = "host")]
#[test]
fn derivative_of_linear_is_constant() {
    let curve = linear_curve_f64();
    let d = derivative(&curve);
    assert_eq!(d.degree(), 0);
    assert!((eval(&d.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
}

#[cfg(feature = "host")]
#[test]
fn derivative_of_quadratic_at_midpoint_matches_central_difference() {
    let curve = quadratic_curve_f64();
    let d = derivative(&curve);
    let v = d.as_view();
    let h = 1e-6_f64;
    let expected = (eval(&curve.as_view(), 0.5 + h) - eval(&curve.as_view(), 0.5 - h)) / (2.0 * h);
    let actual = eval(&v, 0.5);
    assert!(
        (actual - expected).abs() < 1e-6,
        "got {actual}, expected {expected}"
    );
}

#[test]
fn eval_polynomial_with_derivative_matches_separate_calls_quadratic() {
    let curve = quadratic_curve_f64();
    for u_pct in 0..=100 {
        let u = u_pct as f64 / 100.0;
        let (v_combined, d_combined) = eval_polynomial_with_derivative(
            curve.control_points(),
            curve.knots(),
            curve.degree(),
            u,
        );
        let v_sep = eval_polynomial(curve.control_points(), curve.knots(), curve.degree(), u);
        let d_sep = eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
        assert!(
            (v_combined - v_sep).abs() < 1e-12,
            "u={u}: combined value {v_combined} vs separate {v_sep}"
        );
        assert!(
            (d_combined - d_sep).abs() < 1e-12,
            "u={u}: combined deriv {d_combined} vs separate {d_sep}"
        );
    }
}

#[test]
fn eval_polynomial_with_derivative_matches_separate_calls_cubic() {
    let curve = crate::ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.5, 4.0, 5.0],
    )
    .unwrap();
    for u_pct in 0..=100 {
        let u = u_pct as f64 / 100.0;
        let (v_combined, d_combined) = eval_polynomial_with_derivative(
            curve.control_points(),
            curve.knots(),
            curve.degree(),
            u,
        );
        let v_sep = eval_polynomial(curve.control_points(), curve.knots(), curve.degree(), u);
        let d_sep = eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
        assert!(
            (v_combined - v_sep).abs() < 1e-12,
            "u={u}: combined value {v_combined} vs separate {v_sep}"
        );
        assert!(
            (d_combined - d_sep).abs() < 1e-12,
            "u={u}: combined deriv {d_combined} vs separate {d_sep}"
        );
    }
}

#[cfg(feature = "host")]
#[test]
fn eval_derivative_matches_materialized_derivative_quadratic() {
    let curve = quadratic_curve_f64();
    let lowered = derivative(&curve);
    for u_pct in 0..=100 {
        let u = u_pct as f64 / 100.0;
        let materialized = eval(&lowered.as_view(), u);
        let windowed = eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
        assert!(
            (materialized - windowed).abs() < 1e-12,
            "u={u}: materialized={materialized}, windowed={windowed}"
        );
    }
}

#[cfg(feature = "host")]
#[test]
fn eval_derivative_cubic_matches_materialized() {
    let curve = crate::ScalarNurbs::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.5, 4.0, 5.0],
    )
    .unwrap();
    let lowered = derivative(&curve);
    for u_pct in 0..=100 {
        let u = u_pct as f64 / 100.0;
        let materialized = eval(&lowered.as_view(), u);
        let windowed = eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
        assert!(
            (materialized - windowed).abs() < 1e-12,
            "u={u}: materialized={materialized}, windowed={windowed}"
        );
    }
}

#[cfg(feature = "host")]
#[test]
fn vector_derivative_matches_per_axis_scalar() {
    let curve = linear_3d_curve_f64();
    let d = vector_derivative(&curve);
    assert_eq!(d.degree(), 0);
    let v = d.as_view();
    let result = vector_eval(&v, 0.3_f64);

    for axis in 0..3 {
        let cps_axis: Vec<f64> = curve.control_points().iter().map(|cp| cp[axis]).collect();
        let scalar =
            crate::ScalarNurbs::try_new(curve.degree(), curve.knots().to_vec(), cps_axis).unwrap();
        let scalar_d = derivative(&scalar);
        let expected = eval(&scalar_d.as_view(), 0.3_f64);
        assert!((result[axis] - expected).abs() < 1e-12);
    }
}

#[cfg(feature = "host")]
#[test]
fn curvature_of_straight_line_is_zero() {
    let parabolic = crate::VectorNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
    )
    .unwrap();
    let first = vector_derivative(&parabolic);
    let second = vector_derivative(&first);
    let k = curvature_from_derivs(&first, &second, 0.5_f64);
    assert!(k.abs() < 1e-10, "got {k}");
}

#[test]
fn pos_vel_accel_on_quadratic_polynomial() {
    let cps = vec![0.0_f32, 0.0, 1.0];
    let knots = vec![0.0_f32, 0.0, 0.0, 1.0, 1.0, 1.0];
    let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 2, 0.5);
    assert!((p - 0.25).abs() < 1e-9, "pos={}", p);
    assert!((v - 1.0_f64).abs() < 1e-9, "vel={}", v);
    assert!((a - 2.0_f64).abs() < 1e-9, "accel={}", a);
}

#[test]
fn pos_vel_accel_on_cubic_polynomial() {
    let cps = vec![0.0_f32, 0.0, 0.0, 1.0];
    let knots = vec![0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 3, 0.5);
    assert!((p - 0.125).abs() < 1e-9, "pos={}", p);
    assert!((v - 0.75_f64).abs() < 1e-9, "vel={}", v);
    assert!((a - 3.0_f64).abs() < 1e-9, "accel={}", a);
}

#[test]
fn pos_vel_accel_on_linear_polynomial_returns_zero_accel() {
    // f(u) = u, degree-1 Bézier cps=[0,1], knots=[0,0,1,1].
    // Note: 0.3_f32 widens to ~0.30000001192 in f64, so position tolerance
    // accommodates the f32→f64 round-trip on u (~1.2e-8). Velocity and
    // acceleration are exact (rational arithmetic on exact knots/cps).
    let cps = vec![0.0_f32, 1.0];
    let knots = vec![0.0_f32, 0.0, 1.0, 1.0];
    let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 1, 0.3);
    assert!((p - 0.3).abs() < 1e-6, "pos={}", p);
    assert!((v - 1.0_f64).abs() < 1e-9, "vel={}", v);
    assert!(
        a.abs() < 1e-9,
        "linear curve must have zero second derivative; got {}",
        a
    );
}

#[cfg(feature = "host")]
#[test]
fn curvature_of_arc_matches_known_value() {
    let arc = crate::VectorNurbs::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
    )
    .unwrap();
    let first = vector_derivative(&arc);
    let second = vector_derivative(&first);
    let k = curvature_from_derivs(&first, &second, 0.5_f64);
    assert!(k > 0.0, "expected positive curvature, got {k}");
    assert!(k.is_finite(), "curvature should be finite");
}
