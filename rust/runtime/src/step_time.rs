//! Step-time scheduling: compute the next step pulse time for a stepper
//! by Newton-iterating the position polynomial.
//!
//! ## Why (pos, vel, accel)
//!
//! The `eval` closure returns position, first derivative, and second
//! derivative of the motor-frame position polynomial. Newton's initial
//! `dt` guess is taken from the highest-magnitude non-degenerate
//! derivative:
//!   - `|v| ≥ EPS_VELOCITY` → linear: `dt = step_distance / |v|`
//!   - else `|a| ≥ EPS_ACCEL` → quadratic: `dt = sqrt(2·step_distance/|a|)`
//!   - else: forward-scan probe (rare; only on triple-degenerate curves
//!     which the planner does not emit in practice).
//!
//! Returning `SegmentExhausted` happens only when `t_try` exits
//! `[t_curr, t_segment_end]` for `MAX_NEWTON_ITERS` consecutive
//! iterations, OR when all three derivatives are below their thresholds
//! AND the forward scan finds no motion within the segment. Mid-segment
//! velocity collapse (decel-to-rest) no longer bails — the accel-based
//! seed handles it.
//!
//! Spec: docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md §3.6

const NEWTON_TOL_FRACTION: f64 = 1e-6;
const MAX_NEWTON_ITERS: usize = 3;
const EPS_VELOCITY: f64 = 1e-12;
const EPS_ACCEL: f64 = 1e-9;
const FORWARD_SCAN_FRACTION: f64 = 1e-3; // 0.1% of segment

/// Query for `compute_next_step_time`. The `eval` closure must return
/// `(position, velocity, acceleration)` at the requested time in the
/// stepper's motor frame (already through kinematics — for a Cartesian
/// Z this is just the axis position).
///
/// All scalar time/position values are `f64` for host-side precision; the
/// closure receives an `f32` `u` (the de Boor evaluator's native input
/// type) and returns `f64` derivatives.
pub struct StepTimeQuery<'a, F: Fn(f32) -> (f64, f64, f64)> {
    pub eval: &'a F,
    pub step_distance: f64,
    pub current_step: i32,
    /// Time at which to start the search. The unit is whatever domain the
    /// `eval` closure uses; the production path passes MCU clock cycles
    /// directly, but unit tests pass normalized segment time.
    pub t_curr: f64,
    /// End of the active segment in the same time domain as `t_curr`.
    pub t_segment_end: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepTimeResult {
    /// The next step fires at time `t` (same domain as `t_curr`), moving in
    /// `dir` (+1 = forward / positive, -1 = reverse / negative).
    NextAt { t: f64, dir: i8 },
    /// The active segment can't produce another step in the current
    /// direction. Engine re-arms on the next pushed segment.
    SegmentExhausted,
}

pub fn compute_next_step_time<F: Fn(f32) -> (f64, f64, f64)>(
    q: &StepTimeQuery<F>,
) -> StepTimeResult {
    let (_p0, v0, a0) = (q.eval)(q.t_curr as f32);

    // Direction + initial-dt: pick the cheapest analytic seed available.
    // When BOTH v0 and a0 are degenerate, fall back to spec §3.6 option (a):
    // estimate jerk by finite-differencing accel against a forward probe and
    // seed `dt = (6·step_distance / |j|)^(1/3)`. The probe also doubles as the
    // direction source for the triple-degenerate path.
    let (dir_i8, mut dt) = if v0.abs() >= EPS_VELOCITY {
        let dir = if v0 > 0.0 { 1_i8 } else { -1_i8 };
        (dir, q.step_distance / v0.abs())
    } else if a0.abs() >= EPS_ACCEL {
        let dir = if a0 > 0.0 { 1_i8 } else { -1_i8 };
        (dir, (2.0 * q.step_distance / a0.abs()).sqrt())
    } else {
        let span = q.t_segment_end - q.t_curr;
        if span <= 0.0 {
            return StepTimeResult::SegmentExhausted;
        }
        // Probe halfway through the remaining segment to look for any
        // non-degenerate motion.
        let probe_dt = span * 0.5;
        let (_, v_probe, a_probe) = (q.eval)((q.t_curr + probe_dt) as f32);
        if v_probe.abs() >= EPS_VELOCITY {
            // Cubic-ramp-from-rest: jerk ≈ a_probe / probe_dt (since a0 ≈ 0),
            // or fall back to a linear seed from v_probe scaled to the
            // step_distance / v_probe ratio if accel info is unreliable.
            let dir = if v_probe > 0.0 { 1_i8 } else { -1_i8 };
            let seed = if a_probe.abs() >= EPS_ACCEL {
                // Cubic ramp x(u) ≈ (j/6)·u³ where j = (a_probe - a0)/probe_dt
                // and a0 ≈ 0. Solve (j/6)·u³ = step_distance for u.
                let j = a_probe.abs() / probe_dt;
                (6.0 * q.step_distance.abs() / j).cbrt()
            } else {
                q.step_distance.abs() / v_probe.abs()
            };
            (dir, seed)
        } else if a_probe.abs() >= EPS_ACCEL {
            let dir = if a_probe > 0.0 { 1_i8 } else { -1_i8 };
            (dir, (2.0 * q.step_distance / a_probe.abs()).sqrt())
        } else {
            // Triple-degenerate everywhere we looked — no usable motion.
            return StepTimeResult::SegmentExhausted;
        }
    };
    let dir = f64::from(dir_i8);
    let target = (f64::from(q.current_step) + dir) * q.step_distance;

    // Guard against pathological seeds that would step outside the segment.
    let max_dt = q.t_segment_end - q.t_curr;
    if max_dt <= 0.0 {
        return StepTimeResult::SegmentExhausted;
    }
    if dt <= 0.0 {
        dt = max_dt * FORWARD_SCAN_FRACTION;
    }
    let tol = q.step_distance.abs() * NEWTON_TOL_FRACTION;

    for _ in 0..MAX_NEWTON_ITERS {
        let t_try = q.t_curr + dt;
        if t_try > q.t_segment_end || t_try < q.t_curr {
            return StepTimeResult::SegmentExhausted;
        }
        let (pos, vel, _acc) = (q.eval)(t_try as f32);
        let err = pos - target;
        if err.abs() < tol {
            return StepTimeResult::NextAt { t: t_try, dir: dir_i8 };
        }
        // If velocity at the candidate is degenerate, fall back to a
        // forward step. Rare; happens on degenerate accel crossings.
        if vel.abs() < EPS_VELOCITY {
            dt += (q.t_segment_end - q.t_curr) * FORWARD_SCAN_FRACTION;
            continue;
        }
        dt -= err / vel;
    }

    // After MAX iters: accept last candidate within 0.1% step_distance tolerance.
    let t_final = q.t_curr + dt;
    if t_final > q.t_segment_end || t_final < q.t_curr {
        return StepTimeResult::SegmentExhausted;
    }
    let (pos, _vel, _acc) = (q.eval)(t_final as f32);
    if (pos - target).abs() < q.step_distance.abs() * 1e-3 {
        StepTimeResult::NextAt { t: t_final, dir: dir_i8 }
    } else {
        StepTimeResult::SegmentExhausted
    }
}
