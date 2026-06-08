#![allow(clippy::cast_lossless)]

use nurbs::AlgebraError;
use nurbs::algebra::compose_vector_piece;
use nurbs::bezier::BezierPiece;

#[test]
fn identity_composition_returns_outer() {
    let outer_x = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0, 3.0, 4.0], // p(s) = 1 + 2s + 3s² + 4s³ on [0,1]
    };
    let outer_y = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0, 0.0, 0.0], // p(s) = s
    };
    let outer_z = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![5.0, 0.0, 0.0, 0.0], // p(s) = 5
    };
    // identity(t) = t in Pascal-shifted basis on [0, 1] is [0, 1].
    let inner = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0],
    };

    let composed = compose_vector_piece::<3>(&[&outer_x, &outer_y, &outer_z], &inner).unwrap();

    for i in 0..=100 {
        let t = i as f64 / 100.0;
        let composed_x = composed[0].evaluate(t);
        let composed_y = composed[1].evaluate(t);
        let composed_z = composed[2].evaluate(t);
        let expected_x = outer_x.evaluate(inner.evaluate(t));
        let expected_y = outer_y.evaluate(inner.evaluate(t));
        let expected_z = outer_z.evaluate(inner.evaluate(t));
        assert!(
            (composed_x - expected_x).abs() < 1e-10,
            "x mismatch at t={t}"
        );
        assert!(
            (composed_y - expected_y).abs() < 1e-10,
            "y mismatch at t={t}"
        );
        assert!(
            (composed_z - expected_z).abs() < 1e-10,
            "z mismatch at t={t}"
        );
    }
}

#[test]
fn linear_inner_is_parameter_rescaling() {
    let outer = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0, 0.0, 1.0], // p(s) = s + s³
    };
    // inner(t) = 0.5 * t = t/2: maps [0, 1] → [0, 0.5].
    let inner = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.5],
    };
    let outer_subdomain = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 0.5,
        coeffs: outer.coeffs.clone(),
    };

    let composed = compose_vector_piece::<1>(&[&outer_subdomain], &inner).unwrap();

    for i in 0..=50 {
        let t = i as f64 / 100.0;
        let composed_val = composed[0].evaluate(t);
        let expected = outer_subdomain.evaluate(inner.evaluate(t));
        assert!(
            (composed_val - expected).abs() < 1e-10,
            "mismatch at t={t}: composed={composed_val} expected={expected}"
        );
    }
}

#[test]
fn cubic_outer_quadratic_inner_yields_degree_6() {
    // outer(s) = 1 + s + s² + s³ on s ∈ [0, 1].
    let outer = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 1.0, 1.0, 1.0],
    };
    // inner(t) = t² on t ∈ [0, 1] (Pascal-shifted around 0): coeffs = [0, 0, 1].
    let inner = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.0, 1.0],
    };

    let composed = compose_vector_piece::<1>(&[&outer], &inner).unwrap();

    assert_eq!(
        composed[0].degree(),
        6,
        "expected degree 6, got {}",
        composed[0].degree()
    );

    // Sample values must match outer(inner(t)) = 1 + t² + t⁴ + t⁶.
    for i in 0..=100 {
        let t = i as f64 / 100.0;
        let composed_val = composed[0].evaluate(t);
        let expected = 1.0 + t * t + t * t * t * t + t * t * t * t * t * t;
        assert!(
            (composed_val - expected).abs() < 1e-10,
            "mismatch at t={t}: got {composed_val} expected {expected}"
        );
    }
}

#[test]
fn rejects_mismatched_inner_outer_endpoints() {
    let outer = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0, 0.0, 1.0],
    };
    let inner = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.5],
    };

    let result = compose_vector_piece::<1>(&[&outer], &inner);
    assert!(matches!(result, Err(AlgebraError::SupportMismatch)));
}
