#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

use runtime::monomial::{
    bernstein_to_monomial, eval_position, eval_position_velocity, eval_velocity,
};

fn de_casteljau_position(bp: [f32; 4], t: f32) -> f32 {
    let s = 1.0 - t;
    let b01 = s * bp[0] + t * bp[1];
    let b11 = s * bp[1] + t * bp[2];
    let b21 = s * bp[2] + t * bp[3];
    let b02 = s * b01 + t * b11;
    let b12 = s * b11 + t * b21;
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
    let piece = bernstein_to_monomial_with_duration([0.0, 10.0 / 3.0, 20.0 / 3.0, 10.0], 25e-6);
    let p = piece.coeffs[0]
        + piece.coeffs[1] * 25e-6
        + piece.coeffs[2] * (25e-6 * 25e-6)
        + piece.coeffs[3] * (25e-6 * 25e-6 * 25e-6);
    assert!((p - 10.0).abs() < 1e-3, "P(25µs) = {} (expected 10.0)", p);
    assert!((piece.duration - 25e-6).abs() < 1e-12);
    assert!((piece.vel_coeffs[0] - 4e5).abs() < 1e-3);
}

#[test]
fn bernstein_to_monomial_with_duration_quadratic() {
    use runtime::monomial::bernstein_to_monomial_with_duration;
    let piece = bernstein_to_monomial_with_duration([0.0, 0.0, 1.0 / 3.0, 1.0], 1.0);
    let p = piece.coeffs[0] + piece.coeffs[1] + piece.coeffs[2] + piece.coeffs[3];
    assert!((p - 1.0).abs() < 1e-5);
    let p =
        piece.coeffs[0] + piece.coeffs[1] * 0.5 + piece.coeffs[2] * 0.25 + piece.coeffs[3] * 0.125;
    assert!((p - 0.25).abs() < 1e-5);
}
