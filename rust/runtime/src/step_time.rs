//! Step-time scheduling: compute the next step pulse time for a stepper
//! by inverting the position-vs-time cubic Bézier.
//!
//! Replaces the prior Cardano-on-monomial path. Now uses
//! `bezier_root::solve_monotone_cubic_root` (Newton + bisection on
//! Bernstein control points) — see `bezier_root` module docs for why.
//!
//! Spec: docs/superpowers/specs/2026-05-14-bernstein-step-root-design.md

use crate::bezier_root::{
    eval_cubic_bernstein, eval_cubic_derivative_bernstein, solve_monotone_cubic_root,
};

/// Velocity threshold for the direction probe. Below this, fall back to
/// midpoint-position-delta.
const EPS_VELOCITY: f64 = 1e-12;

/// Position-delta threshold for the midpoint-probe fallback. Below this
/// the segment is genuinely motionless and we report SegmentExhausted.
const EPS_MOTION: f64 = 1e-12;

/// Query for `compute_next_step_time`.
#[derive(Debug, Clone, Copy)]
pub struct StepTimeQuery {
    /// Four Bézier control points of the cubic piece, in motor-frame mm.
    pub cps: [f64; 4],
    pub step_distance: f64,
    pub current_step: i32,
    /// Lower bound (exclusive) of the search interval, in normalized u-domain.
    pub t_curr: f64,
    /// Upper bound (inclusive). Always `1.0` in the production path.
    pub t_segment_end: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepTimeResult {
    /// The next step fires at time `t` in `(t_curr, t_segment_end]`. Same
    /// time domain as the caller's `t_curr`.
    NextAt { t: f64, dir: i8 },
    /// No step exists in the search interval in the determined direction.
    /// Engine retires the motor's contribution to the segment.
    SegmentExhausted,
}

/// Compute the time at which the next step pulse should fire.
///
/// 1. Determine motion direction from `P'(t_curr)` (with midpoint-position
///    probe fallback when `|v(t_curr)| < EPS_VELOCITY`).
/// 2. Compute the step target `(current_step + dir) * step_distance`.
/// 3. Solve `B(t) = target` for the smallest `t ∈ (t_curr, t_segment_end]`
///    via `bezier_root::solve_monotone_cubic_root`.
/// 4. Return `NextAt { t, dir }` or `SegmentExhausted` if no root.
#[must_use]
pub fn compute_next_step_time(q: &StepTimeQuery) -> StepTimeResult {
    let [p0, p1, p2, p3] = q.cps;

    let v0 = eval_cubic_derivative_bernstein(p0, p1, p2, p3, q.t_curr);
    let dir_i8: i8 = if libm::fabs(v0) > EPS_VELOCITY {
        if v0 > 0.0 { 1 } else { -1 }
    } else {
        let span = q.t_segment_end - q.t_curr;
        if span <= 0.0 {
            return StepTimeResult::SegmentExhausted;
        }
        let probe = q.t_curr + 0.5 * span;
        let delta = eval_cubic_bernstein(p0, p1, p2, p3, probe)
            - eval_cubic_bernstein(p0, p1, p2, p3, q.t_curr);
        if libm::fabs(delta) < EPS_MOTION {
            return StepTimeResult::SegmentExhausted;
        }
        if delta > 0.0 { 1 } else { -1 }
    };

    let target =
        (f64::from(q.current_step) + f64::from(dir_i8)) * q.step_distance;
    match solve_monotone_cubic_root(p0, p1, p2, p3, target, q.t_curr, q.t_segment_end) {
        Some(t) => StepTimeResult::NextAt { t, dir: dir_i8 },
        None => StepTimeResult::SegmentExhausted,
    }
}
