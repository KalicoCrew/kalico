#![allow(clippy::float_cmp)]
use nurbs::bezier::BezierPiece;

#[test]
fn differentiate_quadratic() {
    let p = BezierPiece::<f64> {
        u_start: 1.0,
        u_end: 3.0,
        coeffs: vec![3.0, 2.0, 5.0],
    };
    let dp = p.differentiate();
    assert_eq!(dp.degree(), 1);
    assert!((dp.coeffs[0] - 2.0).abs() < 1e-12);
    assert!((dp.coeffs[1] - 10.0).abs() < 1e-12);
    assert_eq!(dp.u_start, 1.0);
    assert_eq!(dp.u_end, 3.0);
}

#[test]
fn differentiate_constant_is_zero() {
    let p = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![7.0],
    };
    let dp = p.differentiate();
    assert_eq!(dp.degree(), 0);
    assert!((dp.coeffs[0]).abs() < 1e-12);
}

#[test]
fn differentiate_cubic_matches_finite_diff() {
    let p = BezierPiece::<f64> {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![1.0, -3.0, 2.0, 4.0],
    };
    let dp = p.differentiate();
    let h = 1e-7;
    for &u in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let fd = (p.evaluate(u + h) - p.evaluate(u - h)) / (2.0 * h);
        assert!(
            (dp.evaluate(u) - fd).abs() < 1e-5,
            "at u={u}: dp={}, fd={fd}",
            dp.evaluate(u)
        );
    }
}
