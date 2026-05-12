//! Step-time scheduling: compute the next step pulse time for a stepper
//! by Newton-iterating the position polynomial.
//!
//! Spec: docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md §8.

/// Stepper-side step-distance lower bound for the Newton tolerance. If
/// `position - target` is below this fraction of one step, accept.
const NEWTON_TOL_FRACTION: f32 = 1e-6;

/// Newton iteration cap. Quadratic convergence on a cubic from a
/// velocity-based initial guess hits FP precision in ≤3 iterations for
/// well-conditioned cases. Past 3, something is wrong; give up.
const MAX_NEWTON_ITERS: usize = 3;

/// Velocity below this magnitude is treated as "stopped": the segment
/// can't produce another step at a meaningful rate, so we return
/// `SegmentExhausted` and defer to the next-segment arming path.
///
/// The numerical threshold depends on the time domain the `eval` closure
/// uses. For MCU clock cycles (the production path), 1e-9 mm/cycle at a
/// 180 MHz clock corresponds to 0.18 mm/s — well below any real Z
/// velocity. For normalized segment time (0..1 per segment), it's the
/// same dimensionless value but interpreted as mm per normalized unit;
/// callers should ensure their evaluator returns velocities at the
/// scale this threshold is meaningful for.
const EPS_VELOCITY: f32 = 1e-9;

/// Query for `compute_next_step_time`. The `eval` closure must return
/// `(position, velocity)` at the requested time, where position is in
/// the stepper's motor frame (already through kinematics — for a
/// Cartesian Z this is just the axis position).
pub struct StepTimeQuery<'a, F: Fn(f32) -> (f32, f32)> {
    pub eval: &'a F,
    pub step_distance: f32,
    pub current_step: i32,
    /// Time at which to start the search. The unit is whatever domain the
    /// `eval` closure uses; the production path passes MCU clock cycles
    /// directly, but unit tests pass normalized segment time. The
    /// `EPS_VELOCITY` threshold's numerical sense depends on this choice
    /// — see its doc.
    pub t_curr: f32,
    /// End of the active segment in the same time domain as `t_curr`.
    pub t_segment_end: f32,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepTimeResult {
    /// The next step fires at time `t` (same domain as `t_curr`).
    NextAt(f32),
    /// The active segment can't produce another step in the current
    /// direction. Engine re-arms on the next pushed segment.
    SegmentExhausted,
}

pub fn compute_next_step_time<F: Fn(f32) -> (f32, f32)>(
    q: &StepTimeQuery<F>,
) -> StepTimeResult {
    let (_pos_curr, v_curr) = (q.eval)(q.t_curr);
    if v_curr.abs() < EPS_VELOCITY {
        return StepTimeResult::SegmentExhausted;
    }
    let dir = if v_curr > 0.0 { 1.0_f32 } else { -1.0_f32 };
    let target = (q.current_step as f32 + dir) * q.step_distance;

    // Initial guess: constant velocity.
    let mut dt = q.step_distance / v_curr.abs();
    let tol = q.step_distance.abs() * NEWTON_TOL_FRACTION;

    for _ in 0..MAX_NEWTON_ITERS {
        let t_try = q.t_curr + dt;
        if t_try > q.t_segment_end || t_try < q.t_curr {
            return StepTimeResult::SegmentExhausted;
        }
        let (pos, vel) = (q.eval)(t_try);
        let err = pos - target;
        if err.abs() < tol {
            return StepTimeResult::NextAt(t_try);
        }
        if vel.abs() < EPS_VELOCITY {
            return StepTimeResult::SegmentExhausted;
        }
        dt -= err / vel;
    }

    // After MAX_NEWTON_ITERS loop iterations, `dt` holds the Newton correction
    // computed by the last loop body — effectively a 4th candidate that the
    // `for` loop didn't get to validate. Evaluate it once more and accept if
    // within 0.1% of the step distance (an order of magnitude looser than the
    // in-loop tolerance, since by this point quadratic convergence should have
    // already nailed it; failing the tight tolerance suggests a degenerate
    // curve where any further refinement is unreliable).
    let t_final = q.t_curr + dt;
    if t_final > q.t_segment_end || t_final < q.t_curr {
        return StepTimeResult::SegmentExhausted;
    }
    let (pos, _) = (q.eval)(t_final);
    if (pos - target).abs() < q.step_distance.abs() * 1e-3 {
        StepTimeResult::NextAt(t_final)
    } else {
        StepTimeResult::SegmentExhausted
    }
}
