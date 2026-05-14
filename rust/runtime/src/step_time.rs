//! Step-time scheduling: compute the next step pulse time for a stepper
//! by closed-form Cardano solution on the cubic Bézier position polynomial.
//!
//! Replaces the prior Newton-based iteration. Cardano is deterministic,
//! has no convergence/seed issues, and handles `v(0) = 0` (accel-from-rest)
//! analytically. See `cardano.rs` for the math.
//!
//! Plan: docs/superpowers/plans/2026-05-14-cardano-cubic-solver.md

use crate::cardano::{solve_smallest_root_in, CubicCoeffs};

/// Query for `compute_next_step_time`. The caller (engine) constructs the
/// `coeffs` from the segment's curve control points + kinematic transform.
pub struct StepTimeQuery<'a> {
    pub coeffs: &'a CubicCoeffs,
    pub step_distance: f64,
    pub current_step: i32,
    /// Lower bound (exclusive) of the search interval, in normalized u-domain.
    pub t_curr: f64,
    /// Upper bound (inclusive). Always `1.0` in the production path.
    pub t_segment_end: f64,
}

impl core::fmt::Debug for StepTimeQuery<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("StepTimeQuery")
            .field("coeffs", &self.coeffs)
            .field("step_distance", &self.step_distance)
            .field("current_step", &self.current_step)
            .field("t_curr", &self.t_curr)
            .field("t_segment_end", &self.t_segment_end)
            .finish()
    }
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
/// Steps:
/// 1. Determine motion direction from `eval_d1(t_curr)` (with midpoint
///    probe fallback when v(0)=0).
/// 2. Compute the step target `(current_step + dir) * step_distance`.
/// 3. Solve `coeffs(u) = target` for the smallest `u ∈ (t_curr, t_segment_end]`
///    via Cardano (cardano::solve_smallest_root_in).
/// 4. Return `NextAt { t, dir }` or `SegmentExhausted` if no root.
#[must_use]
pub fn compute_next_step_time(q: &StepTimeQuery<'_>) -> StepTimeResult {
    // Direction from instantaneous velocity at t_curr. If zero (e.g.,
    // accel-from-rest), probe the midpoint of the search interval for
    // position change sign. If still zero, the segment is genuinely
    // motionless.
    let v0 = q.coeffs.eval_d1(q.t_curr);
    let dir_i8: i8 = if libm::fabs(v0) > 1e-12 {
        if v0 > 0.0 { 1 } else { -1 }
    } else {
        let span = q.t_segment_end - q.t_curr;
        if span <= 0.0 {
            return StepTimeResult::SegmentExhausted;
        }
        let probe = q.t_curr + 0.5 * span;
        let delta = q.coeffs.eval(probe) - q.coeffs.eval(q.t_curr);
        if libm::fabs(delta) < 1e-12 {
            return StepTimeResult::SegmentExhausted;
        }
        if delta > 0.0 { 1 } else { -1 }
    };

    let target =
        (f64::from(q.current_step) + f64::from(dir_i8)) * q.step_distance;
    match solve_smallest_root_in(q.coeffs, target, q.t_curr, q.t_segment_end) {
        Some(t) => StepTimeResult::NextAt { t, dir: dir_i8 },
        None => StepTimeResult::SegmentExhausted,
    }
}
