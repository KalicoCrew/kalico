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
//!
//! **Precision**: `f32` throughout. The MCU's storage type is `f32`
//! (curve pool CPs and knots are `&[f32]`), the Cortex-M4F on the F446
//! has hardware FPU for single-precision only (`__aeabi_dmul` software-
//! emulates `f64` at ~250,000 cycles per producer fire — exceeds
//! Klipper's 1 ms timer-dispatch tolerance at `armcm_timer.c:152` →
//! "Rescheduled timer in the past" shutdown). De Casteljau accumulates
//! ~6 ulp relative error after three rounds; at 300 mm-scale coordinates
//! that's ~215 nm absolute, well below the 1.25 µm step resolution at
//! 800 spm. The EPS values below are sized for `f32` ulp at the maximum
//! 300 mm bed scale; see each constant for the derivation.

/// Maximum Newton iterations before falling through to bisection.
/// WebKit/Chromium use 4 with an 11-sample spline seed; Gecko uses 8
/// with a naive `t=x` seed. Our `t = (target - P0) / (P3 - P0)` seed
/// is informationally between the two. Six iterations is a comfortable
/// middle that consistently converges in host fuzzing.
const MAX_NEWTON_ITER: u32 = 6;

/// Maximum bisection iterations. `f32` mantissa is 23 bits; 25 halvings
/// is sufficient to refine any interval in `[0, 1]` to subnormal
/// precision. Bisection is rare in practice (Newton dominates) but the
/// fallback must terminate cleanly.
const MAX_BISECTION_ITER: u32 = 25;

/// Newton convergence tolerance in motor-frame mm. Sized for `f32` ulp
/// at the maximum 300 mm bed coordinate: ulp_f32(300) ≈ 3.6e-5 mm.
/// `1e-4 mm` is ~3 ulp at 300 mm scale (loose enough for reliable
/// convergence) and 100 nm absolute — `1/12` of a step at 800 spm
/// (1.25 µm/step). Well below physical detection at any print scale.
const EPS_CONVERGENCE: f32 = 1e-4;

/// Slope-stall threshold for Newton: when `|P'(t)| < EPS_SLOPE_STALL`,
/// the Newton update direction is unreliable; abort to bisection.
/// In `f32` at 100-300 mm scale, derivatives below 1e-5 mm/Δu are
/// rounding noise.
const EPS_SLOPE_STALL: f32 = 1e-5;

/// Bisection-interval-collapse threshold. Below this the bracket cannot
/// usefully shrink further. `f32` ulp at t=1 is ~1.2e-7; 1e-6 is ~10
/// ulp, the bracket can't meaningfully tighten further.
const EPS_INTERVAL: f32 = 1e-6;

/// Span `P3 - P0` below this implies the linear-interp seed
/// `(target-P0)/(P3-P0)` would explode; fall back to midpoint. In `f32`
/// this is well above the ulp at 100-300 mm scale.
const EPS_DEGENERATE_SPAN: f32 = 1e-5;

/// Tolerance for "target outside [min(v_lo, v_hi), max(v_lo, v_hi)]".
/// Tracks `EPS_CONVERGENCE`.
const EPS_OUT_OF_RANGE: f32 = 1e-4;

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
    p0: f32,
    p1: f32,
    p2: f32,
    p3: f32,
    target: f32,
    t_low: f32,
    t_high: f32,
) -> Option<f32> {
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
    let mut t = if libm::fabsf(span) < EPS_DEGENERATE_SPAN {
        0.5 * (t_low + t_high)
    } else {
        let raw = (target - v_lo) / span * (t_high - t_low) + t_low;
        raw.clamp(t_low, t_high)
    };

    // Newton phase.
    let mut f;
    for _ in 0..MAX_NEWTON_ITER {
        f = eval_cubic_bernstein(p0, p1, p2, p3, t) - target;
        if libm::fabsf(f) < EPS_CONVERGENCE {
            // Spec §3.3: enforce `t ∈ (t_low, t_high]` exclusivity on
            // t_low. A target-equals-v_lo seed converges at t == t_low
            // but the contract excludes that boundary; reject and let
            // bisection (also exclusivity-guarded) decide.
            return if t > t_low { Some(t.min(t_high)) } else { None };
        }
        let df = eval_cubic_derivative_bernstein(p0, p1, p2, p3, t);
        if libm::fabsf(df) < EPS_SLOPE_STALL {
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
            return if mid > t_low {
                Some(mid.min(t_high))
            } else {
                None
            };
        }
        let v_mid = eval_cubic_bernstein(p0, p1, p2, p3, mid);
        let f_mid = v_mid - target;
        if libm::fabsf(f_mid) < EPS_CONVERGENCE {
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
pub(crate) fn eval_cubic_bernstein(p0: f32, p1: f32, p2: f32, p3: f32, t: f32) -> f32 {
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
pub(crate) fn eval_cubic_derivative_bernstein(p0: f32, p1: f32, p2: f32, p3: f32, t: f32) -> f32 {
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
mod tests;
