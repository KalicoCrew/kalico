//! Numerical conditioning tests. Verifies pathological-but-valid input
//! produces predictable error/clamp behavior, never NaN/inf silent propagation.

#![cfg(feature = "host")]

#[test]
fn tiny_knot_range_evaluates_without_nan() {
    let curve = nurbs::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1e-8, 1e-8], vec![0.0, 1.0])
        .expect("tiny but positive range is valid");
    let mid = nurbs::eval::eval(&curve.as_view(), 5e-9);
    assert!(mid.is_finite(), "expected finite eval, got {mid}");
}

#[test]
fn curvature_clamps_at_cusp_like_input() {
    // Construct a curve where r' is near-zero by repeating control points
    // at a point. We use a degenerate degree-2 that almost stops at u=0.5.
    let curve = nurbs::VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        vec![[0.0, 0.0, 0.0], [1e-10, 0.0, 0.0], [1.0, 0.0, 0.0]],
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
    )
    .unwrap();
    let result = nurbs::arc_length::build_arc_length_table_vector(&curve, 1e-6, 64);
    assert!(matches!(
        result,
        Err(nurbs::ArcLengthError::DegenerateCurve)
    ));
}
