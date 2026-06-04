// Monotone cubic Bézier root finder in Bernstein basis.
//
// Algorithm: WebKit `UnitBezier.h::solveCurveX` outer structure
// (Newton with slope guard + bisection fallback), inner evaluators
// per de Casteljau on Bernstein CPs (Mainar & Peña 2004).
//
// Stays in Bernstein basis (Farouki–Goodman 1996) to avoid the
// cancellation in `a = -P0 + 3·P1 - 3·P2 + P3` at toolhead coordinates
// around 100 mm that broke the prior Cardano-on-monomial solver.
//
// f32 throughout: Cortex-M4F (F446) has hardware FPU for single-precision
// only; f64 via `__aeabi_dmul` costs ~250,000 cycles per producer fire —
// exceeds Klipper's 1 ms timer-dispatch tolerance. De Casteljau accumulates
// ~6 ulp relative error after three rounds; at 300 mm scale that is ~215 nm
// absolute, well below the 1.25 µm step resolution at 800 spm.

/// Maximum Newton iterations before falling through to bisection.
/// Six iterations consistently converges in host fuzzing with the
/// `t = (target - P0) / (P3 - P0)` seed.
const MAX_NEWTON_ITER: u32 = 6;

/// Maximum bisection iterations. `f32` mantissa is 23 bits; 25 halvings
/// refines any interval in `[0, 1]` to subnormal precision.
const MAX_BISECTION_ITER: u32 = 25;

/// Newton convergence tolerance in motor-frame mm.
/// 1e-4 mm is ~3 ulp at 300 mm scale; 100 nm absolute — 1/12 of a step
/// at 800 spm.
const EPS_CONVERGENCE: f32 = 1e-4;

/// Slope-stall threshold for Newton. Below 1e-5 mm/Δu the Newton update
/// direction is rounding noise at 100–300 mm scale; fall back to bisection.
const EPS_SLOPE_STALL: f32 = 1e-5;

/// Bisection-interval-collapse threshold. f32 ulp at t=1 is ~1.2e-7;
/// 1e-6 is ~10 ulp — the bracket can't meaningfully tighten further.
const EPS_INTERVAL: f32 = 1e-6;

/// Span `P3 - P0` below this would cause the linear-interp seed
/// `(target-P0)/(P3-P0)` to explode; fall back to midpoint.
const EPS_DEGENERATE_SPAN: f32 = 1e-5;

/// Tolerance for "target outside [min(v_lo, v_hi), max(v_lo, v_hi)]".
/// Tracks `EPS_CONVERGENCE`.
const EPS_OUT_OF_RANGE: f32 = 1e-4;

/// Find `t ∈ (t_low, t_high]` such that the cubic Bézier curve with
/// control points `(p0, p1, p2, p3)` evaluates to `target`.
///
/// The curve is required to be monotone on `[t_low, t_high]`.
///
/// Returns `None` if `target` lies outside the curve's value range,
/// if Newton and bisection both fail to converge, or on non-finite inputs.
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

    let v_lo = eval_cubic_bernstein(p0, p1, p2, p3, t_low);
    let v_hi = eval_cubic_bernstein(p0, p1, p2, p3, t_high);

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

    let mut t = if libm::fabsf(span) < EPS_DEGENERATE_SPAN {
        0.5 * (t_low + t_high)
    } else {
        let raw = (target - v_lo) / span * (t_high - t_low) + t_low;
        raw.clamp(t_low, t_high)
    };

    let mut f;
    for _ in 0..MAX_NEWTON_ITER {
        f = eval_cubic_bernstein(p0, p1, p2, p3, t) - target;
        if libm::fabsf(f) < EPS_CONVERGENCE {
            // t_low is exclusive; a converged t == t_low is rejected.
            return if t > t_low { Some(t.min(t_high)) } else { None };
        }
        let df = eval_cubic_derivative_bernstein(p0, p1, p2, p3, t);
        if libm::fabsf(df) < EPS_SLOPE_STALL {
            break;
        }
        let t_next = t - f / df;
        if t_next < t_low || t_next > t_high {
            break;
        }
        t = t_next;
    }

    let mut lo = t_low;
    let mut hi = t_high;
    for _ in 0..MAX_BISECTION_ITER {
        let mid = 0.5 * (lo + hi);
        // t_low is exclusive; t_high is inclusive.
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

/// Evaluate `P(t)` for a cubic Bézier curve via de Casteljau (three rounds
/// of convex-combination lerps). No monomial conversion, no cancellation.
#[inline]
pub(crate) fn eval_cubic_bernstein(p0: f32, p1: f32, p2: f32, p3: f32, t: f32) -> f32 {
    let one_minus_t = 1.0 - t;
    let b00 = one_minus_t * p0 + t * p1;
    let b01 = one_minus_t * p1 + t * p2;
    let b02 = one_minus_t * p2 + t * p3;
    let b10 = one_minus_t * b00 + t * b01;
    let b11 = one_minus_t * b01 + t * b02;
    one_minus_t * b10 + t * b11
}

/// Evaluate `P'(t)` for the same cubic Bézier curve via de Casteljau on the
/// three difference control points `d_i = 3·(P_{i+1} - P_i)`.
#[inline]
pub(crate) fn eval_cubic_derivative_bernstein(p0: f32, p1: f32, p2: f32, p3: f32, t: f32) -> f32 {
    let one_minus_t = 1.0 - t;
    let d0 = 3.0 * (p1 - p0);
    let d1 = 3.0 * (p2 - p1);
    let d2 = 3.0 * (p3 - p2);
    let e0 = one_minus_t * d0 + t * d1;
    let e1 = one_minus_t * d1 + t * d2;
    one_minus_t * e0 + t * e1
}

#[cfg(test)]
mod tests;
