use super::*;

#[test]
fn evaluate_constant_polynomial_is_constant() {
    let p = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![3.5],
    };
    assert_eq!(p.evaluate(0.0), 3.5);
    assert_eq!(p.evaluate(0.5), 3.5);
    assert_eq!(p.evaluate(1.0), 3.5);
}

#[test]
fn evaluate_linear_polynomial() {
    let p = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0],
    };
    assert_eq!(p.evaluate(0.0), 1.0);
    assert_eq!(p.evaluate(0.5), 2.0);
    assert_eq!(p.evaluate(1.0), 3.0);
}

#[test]
fn evaluate_uses_shifted_basis() {
    let p = BezierPiece::<f64> {
        u_start: 5.0,
        u_end: 7.0,
        coeffs: vec![1.0, 2.0],
    };
    assert_eq!(p.evaluate(5.0), 1.0);
    assert_eq!(p.evaluate(6.0), 3.0);
    assert_eq!(p.evaluate(7.0), 5.0);
}

#[test]
fn zero_creates_zero_polynomial_of_given_degree() {
    let p = BezierPiece::<f64>::zero(0.0, 1.0, 3);
    assert_eq!(p.coeffs, vec![0.0, 0.0, 0.0, 0.0]);
    assert_eq!(p.degree(), 3);
    assert_eq!(p.evaluate(0.5), 0.0);
}

#[test]
fn bernstein_round_trip_preserves_polynomial() {
    let monom = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0, 3.0],
    };
    let bernstein = monom.to_bernstein();
    let back = BezierPiece::from_bernstein(&bernstein, 0.0, 1.0);

    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let exp = monom.evaluate(u);
        let got = back.evaluate(u);
        assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn cubic_bernstein_round_trip_on_shifted_support() {
    let p = BezierPiece::<f64> {
        u_start: 1.0,
        u_end: 3.0,
        coeffs: vec![1.0, -2.0, 3.0, -4.0],
    };
    let bern = p.to_bernstein();
    let back = BezierPiece::from_bernstein(&bern, 1.0, 3.0);
    for u in [1.0, 1.5, 2.0, 2.5, 3.0] {
        let exp = p.evaluate(u);
        let got = back.evaluate(u);
        assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn from_bernstein_to_monomial_for_known_case() {
    let p = BezierPiece::from_bernstein(&[0.0_f64, 1.0], 0.0, 1.0);
    assert!((p.coeffs[0] - 0.0).abs() < 1e-12);
    assert!((p.coeffs[1] - 1.0).abs() < 1e-12);
}

#[test]
fn add_two_pieces_same_support() {
    let a = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0],
    };
    let b = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![3.0, 4.0],
    };
    let sum = (&a + &b).unwrap();
    assert_eq!(sum.coeffs, vec![4.0, 6.0]);
    assert_eq!(sum.u_start, 0.0);
    assert_eq!(sum.u_end, 1.0);
}

#[test]
fn add_two_pieces_mismatched_degrees_pads_with_zero() {
    let a = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0, 3.0],
    };
    let b = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0],
    };
    let sum = (&a + &b).unwrap();
    assert_eq!(sum.coeffs, vec![2.0, 2.0, 3.0]);
}

#[test]
fn add_two_pieces_mismatched_support_errors() {
    let a = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0],
    };
    let b = BezierPiece::<f64> {
        u_start: 0.5,
        u_end: 1.0,
        coeffs: vec![1.0],
    };
    assert!(matches!(&a + &b, Err(AlgebraError::SupportMismatch)));
}

use crate::ScalarNurbs;

#[test]
fn extract_single_bezier_piece_from_clamped_curve() {
    let curve =
        ScalarNurbs::<f64>::try_new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 1.0, 4.0])
            .unwrap();

    let pieces = extract_bezier_pieces(&curve);
    assert_eq!(pieces.len(), 1);
    let p = &pieces[0];
    assert_eq!(p.u_start, 0.0);
    assert_eq!(p.u_end, 1.0);
    assert_eq!(p.degree(), 2);
    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let exp = crate::eval::eval(&curve.as_view(), u);
        let got = p.evaluate(u);
        assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
    }
}

#[test]
fn extract_two_bezier_pieces_from_curve_with_interior_knot() {
    let curve = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

    let pieces = extract_bezier_pieces(&curve);
    assert_eq!(pieces.len(), 2);
    assert_eq!(pieces[0].u_start, 0.0);
    assert_eq!(pieces[0].u_end, 0.5);
    assert_eq!(pieces[1].u_start, 0.5);
    assert_eq!(pieces[1].u_end, 1.0);
    let mid_left = pieces[0].evaluate(0.5);
    let mid_right = pieces[1].evaluate(0.5);
    assert!((mid_left - mid_right).abs() < 1e-12);
    for u in [0.0, 0.25, 0.5] {
        let exp = crate::eval::eval(&curve.as_view(), u);
        let got = pieces[0].evaluate(u);
        assert!((exp - got).abs() < 1e-12);
    }
    for u in [0.5, 0.75, 1.0] {
        let exp = crate::eval::eval(&curve.as_view(), u);
        let got = pieces[1].evaluate(u);
        assert!((exp - got).abs() < 1e-12);
    }
}

#[test]
fn split_piece_at_preserves_evaluation_on_each_side() {
    let original = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, 2.0],
    };
    let (left, right) = split_piece_at(&original, 0.4);

    assert_eq!(left.u_start, 0.0);
    assert_eq!(left.u_end, 0.4);
    assert_eq!(right.u_start, 0.4);
    assert_eq!(right.u_end, 1.0);

    for u in [0.0, 0.2, 0.4] {
        let exp = original.evaluate(u);
        let got = left.evaluate(u);
        assert!((exp - got).abs() < 1e-12);
    }
    for u in [0.4, 0.7, 1.0] {
        let exp = original.evaluate(u);
        let got = right.evaluate(u);
        assert!((exp - got).abs() < 1e-12);
    }
}

#[test]
fn bezier_pieces_to_nurbs_round_trips_extraction() {
    let original = ScalarNurbs::<f64>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
        vec![0.0, 1.0, 2.0, 3.0],
    )
    .unwrap();

    let pieces = extract_bezier_pieces(&original);
    let recomposed = bezier_pieces_to_nurbs(&pieces);

    for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
        let exp = crate::eval::eval(&original.as_view(), u);
        let got = crate::eval::eval(&recomposed.as_view(), u);
        assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
    }
}
