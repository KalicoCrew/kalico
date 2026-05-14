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
