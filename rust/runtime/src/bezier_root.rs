//! Monotone cubic Bézier root finder in Bernstein basis.
//!
//! Spec: docs/superpowers/specs/2026-05-14-bernstein-step-root-design.md
//!
//! Replaces the prior Cardano-on-monomial solver. The Bernstein basis
//! (Farouki–Goodman 1996) is provably optimal-stable for polynomial root
//! finding; staying in this basis avoids the cancellation in
//! `a = -P0 + 3·P1 - 3·P2 + P3` that wedged the bench at toolhead
//! coordinates around 100 mm.
//!
//! Algorithm: WebKit `UnitBezier.h::solveCurveX` outer structure
//! (Newton with slope guard + bisection fallback), inner evaluators
//! per Mainar & Peña 2004 (de Casteljau on Bernstein CPs).

#![cfg_attr(not(feature = "host"), no_std)]

/// Maximum Newton iterations before falling through to bisection.
/// WebKit/Chromium use 4 with an 11-sample spline seed; Gecko uses 8
/// with a naive `t=x` seed. Our `t = (target - P0) / (P3 - P0)` seed
/// is informationally between the two. Six iterations is a comfortable
/// middle that consistently converges in host fuzzing.
const MAX_NEWTON_ITER: u32 = 6;

/// Maximum bisection iterations. f64 mantissa is 52 bits; 54 halvings
/// is sufficient to refine any interval in `[0, 1]` to subnormal
/// precision. Bisection is rare in practice (Newton dominates) but the
/// fallback must terminate cleanly.
const MAX_BISECTION_ITER: u32 = 54;

/// Newton convergence tolerance in motor-frame mm. Sized to live above
/// the f32-source noise floor: CPs are stored as f32 on the MCU, cast
/// to f64 for compute, giving absolute noise ~`|max(P_i)| × ε_f32 ≈
/// 6 nm` for typical 100 mm-scale toolhead coordinates. Step resolution
/// at 800 spm is 1250 nm; 10 nm convergence resolves roots to 1/125 of
/// a step — far below physical detection.
const EPS_CONVERGENCE: f64 = 1e-5;

/// Slope-stall threshold for Newton: when `|P'(t)| < EPS_SLOPE_STALL`,
/// the Newton update direction is unreliable; abort to bisection.
const EPS_SLOPE_STALL: f64 = 1e-7;

/// Bisection-interval-collapse threshold. Below this the bracket cannot
/// usefully shrink further.
const EPS_INTERVAL: f64 = 1e-12;

/// Span `P3 - P0` below this implies the linear-interp seed
/// `(target-P0)/(P3-P0)` would explode; fall back to midpoint.
const EPS_DEGENERATE_SPAN: f64 = 1e-9;

/// Tolerance for "target outside [min(v_lo, v_hi), max(v_lo, v_hi)]".
/// Tracks `EPS_CONVERGENCE`.
const EPS_OUT_OF_RANGE: f64 = 1e-5;

/// Find `t ∈ (t_low, t_high]` such that the cubic Bézier curve with
/// control points `(p0, p1, p2, p3)` evaluates to `target`.
///
/// The curve is required to be monotone on `[t_low, t_high]`. The caller
/// (`Engine::producer_step` via the piecewise walker) upholds this via
/// the planner's piecewise-cubic refit contract: each piece is C¹ and
/// the planner emits monotone-within-piece motion for each axis.
///
/// Returns `None` if `target` lies outside the curve's value range on
/// `[t_low, t_high]`, if Newton fails to converge AND bisection fails to
/// converge (extreme degeneracy), or on non-finite inputs.
#[must_use]
pub fn solve_monotone_cubic_root(
    p0: f64,
    p1: f64,
    p2: f64,
    p3: f64,
    target: f64,
    t_low: f64,
    t_high: f64,
) -> Option<f64> {
    // Defensive: reject non-finite and degenerate inputs.
    if !p0.is_finite()
        || !p1.is_finite()
        || !p2.is_finite()
        || !p3.is_finite()
        || !target.is_finite()
        || !t_low.is_finite()
        || !t_high.is_finite()
    {
        return None;
    }
    if t_high <= t_low {
        return None;
    }

    // Endpoint values: degenerate de Casteljau collapses to a single CP.
    let v_lo = eval_cubic_bernstein(p0, p1, p2, p3, t_low);
    let v_hi = eval_cubic_bernstein(p0, p1, p2, p3, t_high);

    // Monotonicity invariant: planner guarantees the curve crosses
    // through every value in [min(v_lo, v_hi), max(v_lo, v_hi)] exactly
    // once. If target is outside this range, no root exists.
    let (v_min, v_max) = if v_lo <= v_hi {
        (v_lo, v_hi)
    } else {
        (v_hi, v_lo)
    };
    if target < v_min - EPS_OUT_OF_RANGE || target > v_max + EPS_OUT_OF_RANGE {
        return None;
    }

    let direction_is_increasing = v_hi > v_lo;
    let span = v_hi - v_lo;

    // Initial guess: linear interpolation between endpoints. For a near-
    // linear curve this lands within one Newton step of the root. For
    // a curved (real cubic) shape, ~4 Newton iterations.
    let mut t = if libm::fabs(span) < EPS_DEGENERATE_SPAN {
        0.5 * (t_low + t_high)
    } else {
        let raw = (target - v_lo) / span * (t_high - t_low) + t_low;
        raw.clamp(t_low, t_high)
    };

    // Newton phase.
    let mut f;
    for _ in 0..MAX_NEWTON_ITER {
        f = eval_cubic_bernstein(p0, p1, p2, p3, t) - target;
        if libm::fabs(f) < EPS_CONVERGENCE {
            // Spec §3.3: enforce `t ∈ (t_low, t_high]` exclusivity on
            // t_low. A target-equals-v_lo seed converges at t == t_low
            // but the contract excludes that boundary; reject and let
            // bisection (also exclusivity-guarded) decide.
            return if t > t_low { Some(t.min(t_high)) } else { None };
        }
        let df = eval_cubic_derivative_bernstein(p0, p1, p2, p3, t);
        if libm::fabs(df) < EPS_SLOPE_STALL {
            break; // fall through to bisection
        }
        let t_next = t - f / df;
        if t_next < t_low || t_next > t_high {
            break; // escaped bracket — fall through to bisection
        }
        t = t_next;
    }

    // Bisection fallback. The monotonicity invariant guarantees
    // [t_low, t_high] is a valid bracket whose endpoints straddle target.
    let mut lo = t_low;
    let mut hi = t_high;
    for _ in 0..MAX_BISECTION_ITER {
        let mid = 0.5 * (lo + hi);
        // Spec §3.3: midpoint at the boundary collapses the bracket to
        // either endpoint. t_low is exclusive ⇒ None; t_high is
        // inclusive ⇒ Some(t_high).
        if mid <= t_low || mid >= t_high {
            return if mid > t_low { Some(mid.min(t_high)) } else { None };
        }
        let v_mid = eval_cubic_bernstein(p0, p1, p2, p3, mid);
        let f_mid = v_mid - target;
        if libm::fabs(f_mid) < EPS_CONVERGENCE {
            return Some(mid);
        }
        if (f_mid < 0.0) == direction_is_increasing {
            lo = mid;
        } else {
            hi = mid;
        }
        if hi - lo < EPS_INTERVAL {
            let collapsed = 0.5 * (lo + hi);
            return if collapsed > t_low {
                Some(collapsed.min(t_high))
            } else {
                None
            };
        }
    }
    None
}

/// Evaluate `P(t)` for a cubic Bézier curve with Bernstein control
/// points `(p0, p1, p2, p3)` at parameter `t`. Standard de Casteljau
/// triangular scheme — three rounds of convex-combination lerps. No
/// monomial conversion, no cancellation.
///
/// Mainar & Peña 2004, "Evaluation of the derivative of a polynomial
/// in Bernstein form," App. Math. and Computation 158(1):195-204.
#[inline]
pub(crate) fn eval_cubic_bernstein(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
    let one_minus_t = 1.0 - t;
    // Round 1: collapse 4 CPs to 3
    let b00 = one_minus_t * p0 + t * p1;
    let b01 = one_minus_t * p1 + t * p2;
    let b02 = one_minus_t * p2 + t * p3;
    // Round 2: collapse 3 to 2
    let b10 = one_minus_t * b00 + t * b01;
    let b11 = one_minus_t * b01 + t * b02;
    // Round 3: collapse 2 to 1
    one_minus_t * b10 + t * b11
}

/// Evaluate `P'(t)` for the same cubic Bézier curve. The derivative
/// is itself a degree-2 Bernstein polynomial on the three difference
/// control points `d_i = 3·(P_{i+1} - P_i)`; evaluate by de Casteljau
/// on those.
#[inline]
pub(crate) fn eval_cubic_derivative_bernstein(
    p0: f64,
    p1: f64,
    p2: f64,
    p3: f64,
    t: f64,
) -> f64 {
    let one_minus_t = 1.0 - t;
    // Difference control points of the degree-2 derivative curve.
    let d0 = 3.0 * (p1 - p0);
    let d1 = 3.0 * (p2 - p1);
    let d2 = 3.0 * (p3 - p2);
    // Round 1: collapse 3 to 2
    let e0 = one_minus_t * d0 + t * d1;
    let e1 = one_minus_t * d1 + t * d2;
    // Round 2: collapse 2 to 1
    one_minus_t * e0 + t * e1
}

#[cfg(test)]
mod tests {
    use super::*;

    /// De Casteljau at t=0 must collapse to P0 exactly (degenerate path:
    /// every lerp uses weight 0 on the right-hand operand).
    #[test]
    fn eval_at_t0_returns_p0_exactly() {
        let result = eval_cubic_bernstein(100.0, 200.0, 300.0, 400.0, 0.0);
        assert_eq!(result, 100.0);
    }

    /// De Casteljau at t=1 must collapse to P3 exactly.
    #[test]
    fn eval_at_t1_returns_p3_exactly() {
        let result = eval_cubic_bernstein(100.0, 200.0, 300.0, 400.0, 1.0);
        assert_eq!(result, 400.0);
    }

    /// For collinear CPs that interpolate linearly from 0 to 1, eval(t) = t.
    #[test]
    fn eval_collinear_linear_curve_matches_t() {
        let cps = (0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0);
        for &t in &[0.1, 0.25, 0.5, 0.75, 0.9] {
            let result = eval_cubic_bernstein(cps.0, cps.1, cps.2, cps.3, t);
            assert!(
                (result - t).abs() < 1e-12,
                "eval({t}) = {result}, expected {t}"
            );
        }
    }

    /// For the curve `0 -> 0 -> 1 -> 1` (S-curve), B(0.5) = 0.5 exactly
    /// by Bernstein symmetry.
    #[test]
    fn eval_s_curve_at_midpoint_is_half() {
        let result = eval_cubic_bernstein(0.0, 0.0, 1.0, 1.0, 0.5);
        assert!((result - 0.5).abs() < 1e-12);
    }

    /// Derivative at t=0 must equal 3·(P1 - P0) by the Bernstein derivative
    /// identity (the first difference control point evaluated at t=0).
    #[test]
    fn deriv_at_t0_equals_three_times_first_diff() {
        let result = eval_cubic_derivative_bernstein(10.0, 25.0, 40.0, 60.0, 0.0);
        assert_eq!(result, 3.0 * (25.0 - 10.0));
    }

    /// Derivative at t=1 must equal 3·(P3 - P2).
    #[test]
    fn deriv_at_t1_equals_three_times_last_diff() {
        let result = eval_cubic_derivative_bernstein(10.0, 25.0, 40.0, 60.0, 1.0);
        assert_eq!(result, 3.0 * (60.0 - 40.0));
    }

    /// For a collinear linear curve `0 -> 1/3 -> 2/3 -> 1` (representing
    /// the line y=t), the derivative is identically 1.
    #[test]
    fn deriv_of_collinear_linear_curve_is_unity() {
        for &t in &[0.0, 0.25, 0.5, 0.75, 1.0] {
            let result =
                eval_cubic_derivative_bernstein(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, t);
            assert!(
                (result - 1.0).abs() < 1e-12,
                "deriv({t}) = {result}, expected 1.0"
            );
        }
    }

    /// Linear-at-origin curve `y = t`. Target 0.5 → root at 0.5.
    #[test]
    fn solve_linear_curve_at_origin_finds_root() {
        let r = solve_monotone_cubic_root(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 0.5, 0.0, 1.0);
        assert!(r.is_some());
        assert!((r.unwrap() - 0.5).abs() < 1e-5);
    }

    /// **Bench-failure-mode regression.** Linear curve from X=100 to
    /// X=101 (10-piece scenario's piece 0 boundary case). Target 100.5.
    /// Pre-fix: Cardano's monomial leading-coefficient cancellation at
    /// these CP magnitudes drove the trig branch into spurious roots.
    /// Post-fix: de Casteljau / Newton solves cleanly.
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
        assert!((r.unwrap() - 0.5).abs() < 1e-5);
    }

    /// Accel-from-rest curve: P0=P1 makes P'(0)=0, so naive Newton seeded
    /// at t=0 would stall. Our seed is the linear-interp `(target-v_lo)/span`,
    /// which lands at t=0.5 here — past the slope-zero region — so Newton
    /// converges in a few iterations without invoking the bisection
    /// fallback. The behavioral guarantee being pinned: the algorithm
    /// finds the correct root for an asymmetric P'(0)=0 curve.
    #[test]
    fn solve_accel_from_rest_finds_correct_root() {
        // Curve 0 → 0 → 0.5 → 1 (P'(0)=0). Target 0.5.
        let r = solve_monotone_cubic_root(0.0, 0.0, 0.5, 1.0, 0.5, 0.0, 1.0);
        assert!(r.is_some(), "monotone curve with v(0)=0 must still solve");
        let t = r.unwrap();
        // True root for B(t) = 1.5·t² − 0.5·t³ = 0.5 is t ≈ 0.6527036447
        // (the only real root of t³ − 3·t² + 1 in [0, 1]). The curve is
        // NOT symmetric: P0=P1=0 but P2≠P3, so B(0.5) = 0.3125 ≠ 0.5.
        assert!(
            (t - 0.6527036446661392).abs() < 1e-3,
            "expected t ≈ 0.6527, got {t}"
        );
    }

    /// Target above the curve's max → None.
    #[test]
    fn solve_target_above_range_returns_none() {
        let r = solve_monotone_cubic_root(0.0, 0.1, 0.2, 0.3, 0.5, 0.0, 1.0);
        assert!(r.is_none());
    }

    /// Target below the curve's min → None.
    #[test]
    fn solve_target_below_range_returns_none() {
        let r = solve_monotone_cubic_root(0.0, 0.1, 0.2, 0.3, -0.1, 0.0, 1.0);
        assert!(r.is_none());
    }

    /// Target exactly at t_high → returns t_high (inclusive).
    #[test]
    fn solve_target_at_t_high_is_inclusive() {
        let r = solve_monotone_cubic_root(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0, 0.0, 1.0);
        assert!(r.is_some());
        assert!((r.unwrap() - 1.0).abs() < 1e-9);
    }

    /// Target exactly at t_low → t_low is exclusive, returns None.
    #[test]
    fn solve_target_at_t_low_is_exclusive() {
        let r = solve_monotone_cubic_root(0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 0.0, 0.0, 1.0);
        assert!(r.is_none());
    }

    /// Monotone-decreasing curve. Target between endpoints.
    #[test]
    fn solve_monotone_decreasing_curve() {
        let r = solve_monotone_cubic_root(1.0, 2.0 / 3.0, 1.0 / 3.0, 0.0, 0.5, 0.0, 1.0);
        assert!(r.is_some());
        assert!((r.unwrap() - 0.5).abs() < 1e-5);
    }

    /// nm-scale curve. Precision at extreme scale is not an issue for
    /// Bernstein de Casteljau — the algorithm is scale-invariant up to
    /// f64 ULP.
    #[test]
    fn solve_nm_scale_curve_precision() {
        let r = solve_monotone_cubic_root(
            0.0, 1e-9, 2e-9, 3e-9,
            1.5e-9,
            0.0, 1.0,
        );
        assert!(r.is_some(), "nm-scale curve must still solve");
        assert!((r.unwrap() - 0.5).abs() < 1e-5);
    }

    /// Large-offset curve at km scale. Same.
    #[test]
    fn solve_large_offset_curve_precision() {
        let r = solve_monotone_cubic_root(
            1000.0,
            1000.0 + 1.0 / 3.0,
            1000.0 + 2.0 / 3.0,
            1001.0,
            1000.5,
            0.0, 1.0,
        );
        assert!(r.is_some());
        assert!((r.unwrap() - 0.5).abs() < 1e-5);
    }

    /// Multi-step walk: successive targets produce monotonically
    /// increasing t. This is the regression for "producer pushed step
    /// at wrong u_global" from the bench wedge.
    #[test]
    fn solve_walk_monotonic_t_across_targets() {
        let cps = (100.0, 100.0 + 1.0 / 3.0, 100.0 + 2.0 / 3.0, 101.0);
        let mut last_t = 0.0;
        for i in 1..=10 {
            let target = 100.0 + i as f64 * 0.1;
            let r =
                solve_monotone_cubic_root(cps.0, cps.1, cps.2, cps.3, target, 0.0, 1.0);
            assert!(r.is_some(), "step {i} (target={target}) must solve");
            let t = r.unwrap();
            assert!(
                t > last_t,
                "step {i}: t={t} not greater than previous t={last_t}"
            );
            last_t = t;
        }
    }

    /// Noisy CPs: simulate worst-case f32 round-trip by perturbing each
    /// CP. de Casteljau is well-conditioned; the root should still be
    /// within EPS_CONVERGENCE of the true value.
    #[test]
    fn solve_noisy_input_does_not_break_solver() {
        let perturbation = 1e-5_f64; // ~10× f32 ULP at unit magnitude
        let r = solve_monotone_cubic_root(
            100.0 + perturbation,
            100.0 + 1.0 / 3.0 - perturbation,
            100.0 + 2.0 / 3.0 + perturbation,
            101.0 - perturbation,
            100.5,
            0.0, 1.0,
        );
        assert!(r.is_some());
        assert!(
            (r.unwrap() - 0.5).abs() < 1e-3,
            "perturbed root should be within 1e-3 of nominal"
        );
    }

    /// Non-finite input → None, no panic.
    #[test]
    fn solve_non_finite_returns_none() {
        let r = solve_monotone_cubic_root(
            f64::NAN, 1.0, 2.0, 3.0,
            1.5, 0.0, 1.0,
        );
        assert!(r.is_none());
    }

    /// Degenerate interval (t_high <= t_low) → None.
    #[test]
    fn solve_degenerate_interval_returns_none() {
        let r = solve_monotone_cubic_root(0.0, 1.0, 2.0, 3.0, 1.5, 0.5, 0.5);
        assert!(r.is_none());
    }
}
