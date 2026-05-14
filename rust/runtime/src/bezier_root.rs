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
    _p0: f64,
    _p1: f64,
    _p2: f64,
    _p3: f64,
    _target: f64,
    _t_low: f64,
    _t_high: f64,
) -> Option<f64> {
    todo!("Task 2: implement de Casteljau + Newton + bisection")
}

/// Evaluate `P(t)` for a cubic Bézier curve with Bernstein control
/// points `(p0, p1, p2, p3)` at parameter `t`. Standard de Casteljau
/// triangular scheme — three rounds of convex-combination lerps. No
/// monomial conversion, no cancellation.
///
/// Mainar & Peña 2004, "Evaluation of the derivative of a polynomial
/// in Bernstein form," App. Math. and Computation 158(1):195-204.
#[inline]
fn eval_cubic_bernstein(p0: f64, p1: f64, p2: f64, p3: f64, t: f64) -> f64 {
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
fn eval_cubic_derivative_bernstein(
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
}
