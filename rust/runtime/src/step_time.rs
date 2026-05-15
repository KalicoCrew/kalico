//! Step-time scheduling: compute the next step pulse time for a stepper
//! by inverting the position-vs-time cubic Bézier.
//!
//! Replaces the prior Cardano-on-monomial path. Now uses
//! `bezier_root::solve_monotone_cubic_root` (Newton + bisection on
//! Bernstein control points) — see `bezier_root` module docs for why.
//!
//! Spec: docs/superpowers/specs/2026-05-14-bernstein-step-root-design.md
//!
//! **Precision**: `f32` throughout, matching `bezier_root`. The MCU's
//! curve pool stores CPs as `f32`; staying in `f32` cuts producer_step
//! CPU on the Cortex-M4F by ~10× (no software f64 emulation). See
//! `bezier_root` module docs for the precision-budget derivation.

use crate::bezier_root::{
    eval_cubic_bernstein, eval_cubic_derivative_bernstein, solve_monotone_cubic_root,
};

/// Velocity threshold for the direction probe. Below this, fall back to
/// midpoint-position-delta. In `f32` at 100-300 mm scale, derivatives
/// below 1e-5 mm/Δu are rounding noise — matches `bezier_root::EPS_SLOPE_STALL`.
const EPS_VELOCITY: f32 = 1e-5;

/// Position-delta threshold for the midpoint-probe fallback. Below this
/// the segment is genuinely motionless and we report SegmentExhausted.
/// Sized for `f32` ulp at 300 mm bed scale (~3.6e-5 mm). At 1e-4 mm
/// (100 nm) we resolve well below physical step resolution (1.25 µm at
/// 800 spm) while staying safely above ulp noise.
const EPS_MOTION: f32 = 1e-4;

/// Query for `compute_next_step_time`.
#[derive(Debug, Clone, Copy)]
pub struct StepTimeQuery {
    /// Four Bézier control points of the cubic piece, in motor-frame mm.
    pub cps: [f32; 4],
    pub step_distance: f32,
    pub current_step: i32,
    /// Lower bound (exclusive) of the search interval, in normalized u-domain.
    pub t_curr: f32,
    /// Upper bound (inclusive). Always `1.0` in the production path.
    pub t_segment_end: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepTimeResult {
    /// The next step fires at time `t` in `(t_curr, t_segment_end]`. Same
    /// time domain as the caller's `t_curr`.
    NextAt { t: f32, dir: i8 },
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
    let dir_i8: i8 = if libm::fabsf(v0) > EPS_VELOCITY {
        if v0 > 0.0 { 1 } else { -1 }
    } else {
        let span = q.t_segment_end - q.t_curr;
        if span <= 0.0 {
            return StepTimeResult::SegmentExhausted;
        }
        let probe = q.t_curr + 0.5 * span;
        let delta = eval_cubic_bernstein(p0, p1, p2, p3, probe)
            - eval_cubic_bernstein(p0, p1, p2, p3, q.t_curr);
        if libm::fabsf(delta) < EPS_MOTION {
            return StepTimeResult::SegmentExhausted;
        }
        if delta > 0.0 { 1 } else { -1 }
    };

    let target = (q.current_step as f32 + dir_i8 as f32) * q.step_distance;
    match solve_monotone_cubic_root(p0, p1, p2, p3, target, q.t_curr, q.t_segment_end) {
        Some(t) => StepTimeResult::NextAt { t, dir: dir_i8 },
        None => StepTimeResult::SegmentExhausted,
    }
}
