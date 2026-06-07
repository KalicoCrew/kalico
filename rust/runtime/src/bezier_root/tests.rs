#![allow(clippy::unwrap_used)]

use super::*;

#[test]
fn eval_at_t0_returns_p0_exactly() {
    let result = eval_cubic_bernstein(100.0, 200.0, 300.0, 400.0, 0.0);
    assert_eq!(result, 100.0);
}

#[test]
fn eval_at_t1_returns_p3_exactly() {
    let result = eval_cubic_bernstein(100.0, 200.0, 300.0, 400.0, 1.0);
    assert_eq!(result, 400.0);
}

#[test]
fn eval_collinear_linear_curve_matches_t() {
    let cps = (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0);
    for &t in &[0.1, 0.25, 0.5, 0.75, 0.9] {
        let result = eval_cubic_bernstein(cps.0, cps.1, cps.2, cps.3, t);
        assert!(
            (result - t).abs() < 1e-6,
            "eval({t}) = {result}, expected {t}"
        );
    }
}

#[test]
fn eval_s_curve_at_midpoint_is_half() {
    let result = eval_cubic_bernstein(0.0, 0.0, 1.0, 1.0, 0.5);
    assert!((result - 0.5).abs() < 1e-6);
}

#[test]
fn deriv_at_t0_equals_three_times_first_diff() {
    let result = eval_cubic_derivative_bernstein(10.0, 25.0, 40.0, 60.0, 0.0);
    assert_eq!(result, 3.0 * (25.0 - 10.0));
}

#[test]
fn deriv_at_t1_equals_three_times_last_diff() {
    let result = eval_cubic_derivative_bernstein(10.0, 25.0, 40.0, 60.0, 1.0);
    assert_eq!(result, 3.0 * (60.0 - 40.0));
}

#[test]
fn deriv_of_collinear_linear_curve_is_unity() {
    for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
        let result = eval_cubic_derivative_bernstein(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, t);
        assert!(
            (result - 1.0).abs() < 1e-5,
            "deriv({t}) = {result}, expected 1.0"
        );
    }
}

#[test]
fn solve_linear_curve_at_origin_finds_root() {
    let r = solve_monotone_cubic_root(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 0.5, 0.0, 1.0);
    assert!(r.is_some());
    assert!((r.unwrap() - 0.5).abs() < 1e-4);
}

/// **Bench-failure-mode regression.** Linear curve from X=100 to
/// X=101 (10-piece scenario's piece 0 boundary case). Target 100.5.
/// Pre-fix: Cardano's monomial leading-coefficient cancellation at
/// these CP magnitudes drove the trig branch into spurious roots.
/// Post-fix: de Casteljau / Newton solves cleanly in `f32` as well
/// (eval relative error ~6 ulp = 7.2e-5 mm at 100 mm scale).
#[test]
fn solve_linear_curve_at_offset_finds_root() {
    let r = solve_monotone_cubic_root(
        100.0,
        100.0 + 1.0 / 3.0,
        100.0 + 2.0 / 3.0,
        101.0,
        100.5,
        0.0,
        1.0,
    );
    assert!(r.is_some(), "must find root for offset-100mm linear curve");
    // `f32` Newton convergence at 100mm scale: target precision is
    // EPS_CONVERGENCE = 1e-4 mm in P-space; in t-space at slope 1
    // mm/Δu that's 1e-4 in t. Loosen to 1e-3 for the assertion.
    assert!((r.unwrap() - 0.5).abs() < 1e-3);
}

#[test]
fn solve_accel_from_rest_finds_correct_root() {
    let r = solve_monotone_cubic_root(0.0, 0.0, 0.5, 1.0, 0.5, 0.0, 1.0);
    assert!(r.is_some(), "monotone curve with v(0)=0 must still solve");
    let t = r.unwrap();
    // True root for B(t) = 1.5·t² − 0.5·t³ = 0.5 is t ≈ 0.6527036447
    // (the only real root of t³ − 3·t² + 1 in [0, 1]). The curve is
    // NOT symmetric: P0=P1=0 but P2≠P3, so B(0.5) = 0.3125 ≠ 0.5.
    assert!((t - 0.6527036).abs() < 5e-3, "expected t ≈ 0.6527, got {t}");
}

#[test]
fn solve_target_above_range_returns_none() {
    let r = solve_monotone_cubic_root(0.0, 0.1, 0.2, 0.3, 0.5, 0.0, 1.0);
    assert!(r.is_none());
}

#[test]
fn solve_target_below_range_returns_none() {
    let r = solve_monotone_cubic_root(0.0, 0.1, 0.2, 0.3, -0.1, 0.0, 1.0);
    assert!(r.is_none());
}

#[test]
fn solve_target_at_t_high_is_inclusive() {
    let r = solve_monotone_cubic_root(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0, 0.0, 1.0);
    assert!(r.is_some());
    assert!((r.unwrap() - 1.0).abs() < 1e-6);
}

#[test]
fn solve_target_at_t_low_is_exclusive() {
    let r = solve_monotone_cubic_root(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 0.0, 0.0, 1.0);
    assert!(r.is_none());
}

#[test]
fn solve_monotone_decreasing_curve() {
    let r = solve_monotone_cubic_root(1.0, 2.0 / 3.0, 1.0 / 3.0, 0.0, 0.5, 0.0, 1.0);
    assert!(r.is_some());
    assert!((r.unwrap() - 0.5).abs() < 1e-4);
}

#[test]
fn solve_nm_scale_curve_does_not_panic() {
    let r = solve_monotone_cubic_root(0.0, 1e-9, 2e-9, 3e-9, 1.5e-9, 0.0, 1.0);
    assert!(r.is_some(), "nm-scale curve must not panic");
    let t = r.unwrap();
    assert!(t.is_finite() && (0.0..=1.0).contains(&t));
}

#[test]
fn solve_large_offset_curve_finds_plausible_root() {
    let r = solve_monotone_cubic_root(
        1000.0,
        1000.0 + 1.0 / 3.0,
        1000.0 + 2.0 / 3.0,
        1001.0,
        1000.5,
        0.0,
        1.0,
    );
    assert!(r.is_some());
    // `f32` ulp at 1000 mm scale is ~1.2e-4 mm, so Newton at
    // EPS_CONVERGENCE=1e-4 has roughly one digit of headroom — root
    // accuracy in t-space ~1e-3.
    assert!((r.unwrap() - 0.5).abs() < 5e-3);
}

#[test]
fn solve_walk_monotonic_t_across_targets() {
    let cps = (100.0_f32, 100.0 + 1.0 / 3.0, 100.0 + 2.0 / 3.0, 101.0);
    let mut last_t = 0.0;
    for i in 1..=10 {
        let target = 100.0 + i as f32 * 0.1;
        let r = solve_monotone_cubic_root(cps.0, cps.1, cps.2, cps.3, target, 0.0, 1.0);
        assert!(r.is_some(), "step {i} (target={target}) must solve");
        let t = r.unwrap();
        assert!(
            t > last_t,
            "step {i}: t={t} not greater than previous t={last_t}"
        );
        last_t = t;
    }
}

#[test]
fn solve_noisy_input_does_not_break_solver() {
    let perturbation = 1e-5_f32;
    let r = solve_monotone_cubic_root(
        100.0 + perturbation,
        100.0 + 1.0 / 3.0 - perturbation,
        100.0 + 2.0 / 3.0 + perturbation,
        101.0 - perturbation,
        100.5,
        0.0,
        1.0,
    );
    assert!(r.is_some());
    assert!(
        (r.unwrap() - 0.5).abs() < 5e-3,
        "perturbed root should be within 5e-3 of nominal (f32 precision band)"
    );
}

#[test]
fn solve_non_finite_returns_none() {
    let r = solve_monotone_cubic_root(f32::NAN, 1.0, 2.0, 3.0, 1.5, 0.0, 1.0);
    assert!(r.is_none());
}

#[test]
fn solve_degenerate_interval_returns_none() {
    let r = solve_monotone_cubic_root(0.0, 1.0, 2.0, 3.0, 1.5, 0.5, 0.5);
    assert!(r.is_none());
}
