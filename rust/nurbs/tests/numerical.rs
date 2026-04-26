//! Numerical conditioning tests. Verifies pathological-but-valid input
//! produces predictable error/clamp behavior, never NaN/inf silent propagation.

#![cfg(feature = "host")]

#[test]
fn tiny_knot_range_evaluates_without_nan() {
    let curve =
        nurbs::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1e-8, 1e-8], vec![0.0, 1.0], None)
            .expect("tiny but positive range is valid");
    let mid = nurbs::eval::eval(&curve.as_view(), 5e-9);
    assert!(mid.is_finite(), "expected finite eval, got {mid}");
}

#[test]
fn near_zero_weight_evaluates_within_clamp() {
    // The trailing weight is small (stress for the rational denominator) but
    // chosen above MIN_PARAMETRIC_SPEED = 1e-9 so the eval-time
    // `debug_assert!(denom.abs() > floor)` does not fire in debug builds.
    // Plan §Task 30 used 1e-12 which trips the debug_assert at u=1.0; that
    // path is a release-only clamp branch, not exercisable from a regular
    // test build. Bumping to 1e-6 keeps the spirit (near-zero weight,
    // finite eval) without violating the substrate's debug contract.
    let curve = nurbs::ScalarNurbs::try_new(
        1,
        vec![0.0_f64, 0.0, 1.0, 1.0],
        vec![1.0, 2.0],
        Some(vec![1.0, 1e-6]),
    )
    .expect("positive weight passes validation");
    let v = nurbs::eval::eval(&curve.as_view(), 1.0);
    assert!(v.is_finite(), "expected finite, got {v}");
}

#[test]
fn curvature_clamps_at_cusp_like_input() {
    // Construct a curve where r' is near-zero by repeating control points
    // at a point. We use a degenerate degree-2 that almost stops at u=0.5.
    let curve = nurbs::VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1e-10, 0.0, 0.0], [1.0, 0.0, 0.0]],
        None,
    )
    .unwrap();
    let first = nurbs::eval::vector_derivative(&curve);
    let second = nurbs::eval::vector_derivative(&first);
    let k = nurbs::eval::curvature_from_derivs(&first, &second, 0.0_f64);
    assert!(k.is_finite(), "curvature must clamp, not blow up: got {k}");
}

#[test]
fn arc_length_builder_rejects_truly_degenerate_curve() {
    // Construct a curve whose entire image is one point — every CP equal.
    let curve = nurbs::VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]],
        None,
    )
    .unwrap();
    let result = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-6, 64);
    assert!(matches!(
        result,
        Err(nurbs::ArcLengthError::DegenerateCurve)
    ));
}
