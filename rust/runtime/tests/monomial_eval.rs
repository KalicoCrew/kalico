//! Tests for the cubic Bezier monomial (Horner) evaluator.
//!
//! Validates:
//! 1. Constant curves (b0 == b1 == b2 == b3) evaluate to the constant.
//! 2. Linear curves (b_i = b0 + i·step/3) evaluate to the line at any t.
//! 3. Monomial-form evaluation agrees with a de Casteljau reference on a
//!    "random-ish" cubic Bezier across 101 sample points to 1e-4.

use runtime::monomial::{
    bernstein_to_monomial, eval_position, eval_position_velocity, eval_velocity,
};

/// Numerically-stable de Casteljau reference for cubic Bezier position.
/// Three rounds of linear interpolation.
fn de_casteljau_position(bp: [f32; 4], t: f32) -> f32 {
    let s = 1.0 - t;
    // Round 1
    let b01 = s * bp[0] + t * bp[1];
    let b11 = s * bp[1] + t * bp[2];
    let b21 = s * bp[2] + t * bp[3];
    // Round 2
    let b02 = s * b01 + t * b11;
    let b12 = s * b11 + t * b21;
    // Round 3
    s * b02 + t * b12
}

#[test]
fn bernstein_to_monomial_constant_curve() {
    let bp = [3.5_f32, 3.5, 3.5, 3.5];
    let m = bernstein_to_monomial(bp);

    for i in 0..=100 {
        let t = i as f32 / 100.0;
        let p = eval_position(&m, t);
        assert!(
            (p - 3.5).abs() < 1e-5,
            "constant-curve position at t={t} was {p}, expected 3.5"
        );
        let v = eval_velocity(&m, t);
        assert!(
            v.abs() < 1e-5,
            "constant-curve velocity at t={t} was {v}, expected 0"
        );
    }
}

#[test]
fn bernstein_to_monomial_linear_curve() {
    // Linear from 0 -> 9: control points spaced evenly on the line.
    //   b0=0, b1=3, b2=6, b3=9   (b_i = i * 3)
    // P(t) = 9·t,  V(t) = 9.
    let bp = [0.0_f32, 3.0, 6.0, 9.0];
    let m = bernstein_to_monomial(bp);

    for i in 0..=100 {
        let t = i as f32 / 100.0;
        let p = eval_position(&m, t);
        let expected_p = 9.0 * t;
        assert!(
            (p - expected_p).abs() < 1e-5,
            "linear-curve position at t={t} was {p}, expected {expected_p}"
        );

        let v = eval_velocity(&m, t);
        assert!(
            (v - 9.0).abs() < 1e-5,
            "linear-curve velocity at t={t} was {v}, expected 9.0"
        );
    }
}

#[test]
fn bernstein_to_monomial_roundtrip_against_de_casteljau() {
    // A "random-ish" cubic Bezier — picked deterministically to exercise
    // all four control-point contributions with non-trivial geometry.
    //
    //   b0 = -1.25,  b1 = 4.10,  b2 = -2.75,  b3 = 6.40
    //
    // The values are mixed-sign and non-monotone so c2, c3 are non-zero.
    let bp = [-1.25_f32, 4.10, -2.75, 6.40];
    let m = bernstein_to_monomial(bp);

    let tol = 1e-4_f32;

    for i in 0..=100 {
        let t = i as f32 / 100.0;
        let p_mono = eval_position(&m, t);
        let p_ref = de_casteljau_position(bp, t);
        assert!(
            (p_mono - p_ref).abs() < tol,
            "position mismatch at t={t}: mono={p_mono}, ref={p_ref}, \
             diff={diff}",
            diff = (p_mono - p_ref).abs()
        );

        // Combined evaluator agrees with the separate ones.
        let (p_combined, v_combined) = eval_position_velocity(&m, t);
        assert!(
            (p_combined - p_mono).abs() < 1e-6,
            "eval_position_velocity position disagreed with eval_position at t={t}"
        );
        let v_solo = eval_velocity(&m, t);
        assert!(
            (v_combined - v_solo).abs() < 1e-6,
            "eval_position_velocity velocity disagreed with eval_velocity at t={t}"
        );
    }
}

#[test]
fn bernstein_to_monomial_with_duration_rescales_coefficients() {
    use runtime::monomial::bernstein_to_monomial_with_duration;
    // Linear ramp from 0 to 10mm over 25 µs:
    // Unit-interval Bernstein for P(τ) = 10·τ is [0, 10/3, 20/3, 10].
    // Seconds-domain monomial: c0=0, c1=10/25e-6=4e5, c2=0, c3=0.
    let piece = bernstein_to_monomial_with_duration([0.0, 10.0/3.0, 20.0/3.0, 10.0], 25e-6);
    // Evaluate at t=25e-6 should give 10.0 mm.
    let p = piece.coeffs[0]
          + piece.coeffs[1] * 25e-6
          + piece.coeffs[2] * (25e-6 * 25e-6)
          + piece.coeffs[3] * (25e-6 * 25e-6 * 25e-6);
    assert!((p - 10.0).abs() < 1e-3, "P(25µs) = {} (expected 10.0)", p);
    assert!((piece.duration - 25e-6).abs() < 1e-12);
    // vel_coeffs pre-baked: vc0 = c1, vc1 = 2*c2, vc2 = 3*c3
    assert!((piece.vel_coeffs[0] - 4e5).abs() < 1e-3);
}

#[test]
fn bernstein_to_monomial_with_duration_quadratic() {
    use runtime::monomial::bernstein_to_monomial_with_duration;
    // Pure quadratic P(τ) = τ² in unit interval.
    // Bernstein CPs for τ² are [0, 0, 1/3, 1] (degree-3 elevation of τ²).
    let piece = bernstein_to_monomial_with_duration([0.0, 0.0, 1.0/3.0, 1.0], 1.0);
    // At t=1.0, P should = 1.0
    let p = piece.coeffs[0] + piece.coeffs[1] + piece.coeffs[2] + piece.coeffs[3];
    assert!((p - 1.0).abs() < 1e-5);
    // At t=0.5, P should = 0.25
    let p = piece.coeffs[0]
          + piece.coeffs[1] * 0.5
          + piece.coeffs[2] * 0.25
          + piece.coeffs[3] * 0.125;
    assert!((p - 0.25).abs() < 1e-5);
}
