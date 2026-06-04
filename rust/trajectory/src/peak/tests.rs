use super::*;
use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};

#[test]
fn peak_accel_of_constant_is_zero() {
    let piece = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![5.0],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);
    assert!(peak_accel(&curve).abs() < 1e-6);
}

#[test]
fn peak_accel_of_linear_is_zero() {
    let piece = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![3.0, 2.0],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);
    assert!(
        peak_accel(&curve).abs() < 1.0,
        "expected ~0, got {}",
        peak_accel(&curve)
    );
}

#[test]
fn peak_accel_of_quadratic() {
    let piece = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 0.0, 5.0],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);
    let peak = peak_accel(&curve);
    assert!((peak - 10.0).abs() < 0.01, "expected ~10.0, got {peak}",);
}

#[test]
fn peak_accel_of_cubic() {
    let piece = BezierPiece {
        u_start: 0.0,
        u_end: 2.0,
        coeffs: vec![0.0, 0.0, 0.0, 1.0],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);
    let peak = peak_accel(&curve);
    assert!((peak - 12.0).abs() < 0.15, "expected ~12.0, got {peak}",);
}

#[test]
fn peak_accel_multi_piece() {
    let p1 = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0, 1.0],
    };
    let p2 = BezierPiece {
        u_start: 1.0,
        u_end: 2.0,
        coeffs: vec![2.0, 1.0, 50.0],
    };
    let curve = bezier_pieces_to_nurbs(&[p1, p2]);
    let peak = peak_accel(&curve);
    assert!((peak - 100.0).abs() < 1.0, "expected ~100.0, got {peak}",);
}

#[test]
fn peak_accel_interior_extremum() {
    let piece = BezierPiece {
        u_start: 0.0,
        u_end: 1.0,
        coeffs: vec![0.0, 1.0, 0.0, -1.0],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);
    let peak = peak_accel(&curve);
    assert!((peak - 6.0).abs() < 0.1, "expected ~6.0, got {peak}",);
}

#[test]
fn peak_accel_production_frequency_kernel() {
    use nurbs::algebra::{convolve, PiecewisePolynomialKernel};

    let piece = BezierPiece {
        u_start: 0.0,
        u_end: 0.1,
        coeffs: vec![0.0, 0.0, 0.0, 1666.67],
    };
    let curve = bezier_pieces_to_nurbs(&[piece]);

    let t_sm: f64 = 0.8025 / 150.0;
    let h = t_sm / 2.0;
    let c = 15.0 / (16.0 * h.powi(5));
    let coeffs = vec![c * h.powi(4), 0.0, -2.0 * c * h * h, 0.0, c];
    let kernel = PiecewisePolynomialKernel::single_poly_from_absolute(coeffs, (-h, h));

    let convolved = convolve(&curve, &kernel).unwrap();

    let peak = peak_accel(&convolved);
    assert!(peak.is_finite(), "peak is not finite: {peak}");
    assert!(peak > 100.0, "peak too low: {peak}");
    assert!(
        peak < 1_000_000.0,
        "peak too high (numerical blowup?): {peak}"
    );
}
