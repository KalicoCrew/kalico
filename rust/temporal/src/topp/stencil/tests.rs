use super::*;

fn build_b_from_polynomial<F: Fn(f64) -> f64>(n: usize, h: f64, f: F) -> Vec<f64> {
    (0..n).map(|i| f(i as f64 * h)).collect()
}

/// Quadratic b(s) = α·s² + β·s + γ. b''(s) = 2α everywhere; b''''(s) = 0.
/// Width-1 stencil's truncation coefficient (h²·b''''/12) vanishes, so
/// the estimate is exact up to floating-point round-off.
#[test]
fn s_dddot_at_quadratic_interior_within_machine_epsilon() {
    let alpha = 0.7;
    let beta = 1.3;
    let gamma = 5.0;
    let h = 0.5;
    let n = 10;
    let b = build_b_from_polynomial(n, h, |s| alpha * s * s + beta * s + gamma);

    // Expected: s‴_i = √b_i · α at all interior indices.
    for i in 1..n - 1 {
        let s = i as f64 * h;
        let b_i = alpha * s * s + beta * s + gamma;
        let expected = b_i.sqrt() * alpha;
        let got = s_dddot_at(&b, i, h);
        assert!(
            (got - expected).abs() < 1e-12,
            "i={i}: got {got}, expected {expected} (diff {})",
            got - expected
        );
    }
}

/// Cubic b(s) = α·s³ + β·s² + γ·s + δ. b''(s) = 6α·s + 2β; b''''(s) = 0.
/// Width-1 stencil exact up to round-off.
#[test]
fn s_dddot_at_cubic_interior_within_machine_epsilon() {
    let alpha = 0.4;
    let beta = -0.2;
    let gamma = 1.0;
    let delta = 3.0;
    let h = 0.5;
    let n = 10;
    let b = build_b_from_polynomial(n, h, |s| {
        alpha * s * s * s + beta * s * s + gamma * s + delta
    });

    for i in 1..n - 1 {
        let s = i as f64 * h;
        let b_i = alpha * s * s * s + beta * s * s + gamma * s + delta;
        let b_pp = 6.0 * alpha * s + 2.0 * beta;
        let expected = b_i.sqrt() * b_pp / 2.0;
        let got = s_dddot_at(&b, i, h);
        assert!(
            (got - expected).abs() < 1e-10,
            "i={i}: got {got}, expected {expected} (diff {})",
            got - expected
        );
    }
}

/// Quartic b(s) = α·s⁴ + …. b''''(s) = 24α (constant non-zero), so the
/// width-1 stencil has a leading h²·b''''/12 = 2αh² truncation on b''.
/// s‴ truncation is √b · h² · α. Pin within that tolerance.
#[test]
fn s_dddot_at_quartic_interior_within_truncation_bound() {
    let alpha = 0.1;
    let h = 0.25;
    let n = 12;
    let b = build_b_from_polynomial(n, h, |s| alpha * s * s * s * s + 100.0);

    for i in 1..n - 1 {
        let s = i as f64 * h;
        let b_i = alpha * s * s * s * s + 100.0;
        let b_pp = 12.0 * alpha * s * s;
        let expected = b_i.sqrt() * b_pp / 2.0;
        let got = s_dddot_at(&b, i, h);
        // Truncation tolerance: |error| ≤ √b · h² · α (with safety factor 2).
        let tol = 2.0 * b_i.sqrt() * h * h * alpha;
        assert!(
            (got - expected).abs() < tol,
            "i={i}: got {got}, expected {expected} (diff {}, tol {tol})",
            got - expected
        );
    }
}

/// Constant b(s) = c. b''(s) = 0 everywhere, so s‴ = 0 at every grid
/// index including boundaries. (Forward / backward second-differences of
/// a constant are also zero.)
#[test]
fn s_dddot_at_constant_returns_zero_everywhere() {
    let h = 1.0;
    let n = 8;
    let b = vec![100.0; n];

    for i in 0..n {
        let got = s_dddot_at(&b, i, h);
        assert!(got.abs() < 1e-12, "i={i}: got {got}, expected 0");
    }
}

/// b[i] = 0 should produce s_dddot = 0 (the .max(0.0).sqrt() guard
/// makes s_dot = 0). No NaN/Inf even with non-zero b-FD numerator.
#[test]
fn s_dddot_at_handles_zero_b_without_nan() {
    let h = 1.0;
    let mut b = vec![10.0; 5];
    b[1] = 0.0;
    let got = s_dddot_at(&b, 1, h);
    assert_eq!(got, 0.0, "expected exactly 0.0, got {got}");
    assert!(got.is_finite());
}

/// b[i] slightly negative (Clarabel residual rounding) should also
/// produce 0, not NaN.
#[test]
fn s_dddot_at_handles_slightly_negative_b_without_nan() {
    let h = 1.0;
    let mut b = vec![10.0; 5];
    b[1] = -1e-15;
    let got = s_dddot_at(&b, 1, h);
    assert_eq!(got, 0.0);
    assert!(got.is_finite());
}

/// Boundary stencil at i=0 with b(s) = α·s² + γ (β=0, b''=2α, b''''=0).
/// Forward FD has O(h)·b''' leading error, but b''' = 0 here too, so
/// forward FD is also exact for quadratics.
#[test]
fn s_dddot_at_boundary_quadratic_exact() {
    let alpha = 0.5;
    let gamma = 4.0;
    let h = 0.3;
    let n = 6;
    let b = build_b_from_polynomial(n, h, |s| alpha * s * s + gamma);

    // i=0
    let b_0 = gamma;
    let expected_0 = b_0.sqrt() * alpha;
    let got_0 = s_dddot_at(&b, 0, h);
    assert!(
        (got_0 - expected_0).abs() < 1e-12,
        "i=0: got {got_0}, expected {expected_0}"
    );

    // i=n-1
    let s_last = (n - 1) as f64 * h;
    let b_last = alpha * s_last * s_last + gamma;
    let expected_last = b_last.sqrt() * alpha;
    let got_last = s_dddot_at(&b, n - 1, h);
    assert!(
        (got_last - expected_last).abs() < 1e-12,
        "i=n-1: got {got_last}, expected {expected_last}"
    );
}

#[test]
fn b_dd_weights_exact_on_quadratic_nonuniform() {
    // b(s) = 3s² − 2s + 1 → b″ = 6 everywhere, any spacing.
    let b = |s: f64| 3.0 * s * s - 2.0 * s + 1.0;
    let (hl, hr) = (0.3, 0.7);
    let s_i = 1.0;
    let w = b_dd_weights(hl, hr);
    let approx = w[0] * b(s_i - hl) + w[1] * b(s_i) + w[2] * b(s_i + hr);
    assert!((approx - 6.0).abs() < 1e-10, "got {approx}");
}

#[test]
fn b_d_weights_exact_on_quadratic_nonuniform() {
    let b = |s: f64| 3.0 * s * s - 2.0 * s + 1.0; // b′(1) = 4
    let (hl, hr) = (0.3, 0.7);
    let w = b_d_weights(hl, hr);
    let approx = w[0] * b(1.0 - hl) + w[1] * b(1.0) + w[2] * b(1.0 + hr);
    assert!((approx - 4.0).abs() < 1e-10, "got {approx}");
}

#[test]
fn weights_reduce_to_uniform() {
    let h = 0.5;
    let wd = b_d_weights(h, h);
    assert!((wd[0] - (-1.0 / (2.0 * h))).abs() < 1e-12);
    assert!(wd[1].abs() < 1e-12);
    assert!((wd[2] - 1.0 / (2.0 * h)).abs() < 1e-12);
    let wdd = b_dd_weights(h, h);
    assert!((wdd[0] - 1.0 / (h * h)).abs() < 1e-12);
    assert!((wdd[1] - (-2.0 / (h * h))).abs() < 1e-12);
    assert!((wdd[2] - 1.0 / (h * h)).abs() < 1e-12);
}

#[test]
fn s_dddot_weights_matches_legacy_uniform() {
    let b = vec![100.0, 144.0, 196.0, 256.0, 324.0];
    let h = 0.25;
    let h_intervals = vec![h; 4];
    for i in 0..5 {
        let legacy = s_dddot_at(&b, i, h);
        let general = s_dddot_at_weights(&b, i, &h_intervals);
        assert!(
            (legacy - general).abs() < 1e-9,
            "i={i}: {legacy} vs {general}"
        );
    }
}

/// `stencil_for` dispatch.
#[test]
fn stencil_for_dispatches_correctly() {
    assert_eq!(stencil_for(10, 0), SDddotStencil::StartBoundary);
    assert_eq!(stencil_for(10, 1), SDddotStencil::Interior);
    assert_eq!(stencil_for(10, 5), SDddotStencil::Interior);
    assert_eq!(stencil_for(10, 8), SDddotStencil::Interior);
    assert_eq!(stencil_for(10, 9), SDddotStencil::EndBoundary);
    assert_eq!(stencil_for(3, 0), SDddotStencil::StartBoundary);
    assert_eq!(stencil_for(3, 1), SDddotStencil::Interior);
    assert_eq!(stencil_for(3, 2), SDddotStencil::EndBoundary);
}
