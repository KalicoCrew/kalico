//! Independent E-axis trajectory scheduling.
//!
//! Provides trapezoidal velocity profiling for E-only segments (retraction,
//! prime, filament-change) where the extruder moves independently of XY.
//!
//! Two entry points:
//! - `schedule_e_duration` — computes the time for a trapezoidal velocity
//!   profile given the E NURBS path length and dynamic limits. Used by Stage 0
//!   partition to pre-schedule E-gap durations.
//! - `schedule_e_full` — builds a time-parameterized E NURBS for the final
//!   output assembly.

use crate::ELimits;
use nurbs::eval::eval as nurbs_eval;
use nurbs::ScalarNurbs;

/// Compute the duration of a trapezoidal velocity profile for an independent E
/// segment.
///
/// The E path length is derived from the scalar NURBS endpoints:
/// `|e(u_end) - e(u_start)|`. A trapezoidal (or triangular, if the path is too
/// short to reach cruise) profile is applied with the given feedrate and limits.
///
/// Returns the total duration in seconds. Returns 0.0 for zero-length moves.
pub fn schedule_e_duration(e_nurbs: &ScalarNurbs<f64>, feedrate: f64, limits: &ELimits) -> f64 {
    let total_length = e_path_length(e_nurbs);
    if total_length <= 0.0 {
        return 0.0;
    }
    trapezoidal_duration(total_length, feedrate, limits)
}

/// Build a time-parameterized E NURBS for the final shaped-segment output.
///
/// Constructs a piecewise-polynomial s(t) trapezoidal profile and composes it
/// with the E NURBS. For MVP, since independent E moves are typically simple
/// linear retractions, we build the s(t) profile directly as a scalar NURBS
/// that maps `[t_start, t_start + duration]` to `[e_start, e_end]`.
///
/// The output NURBS has degree 2 (at most) — degree-1 cruise phases are
/// degree-elevated to 2 for uniform representation.
///
/// # Errors
///
/// Returns `ShapeError::Algebra` if NURBS construction fails (should not happen
/// for well-formed input).
pub fn schedule_e_full(
    e_nurbs: &ScalarNurbs<f64>,
    feedrate: f64,
    limits: &ELimits,
    t_start: f64,
) -> Result<ScalarNurbs<f64>, crate::ShapeError> {
    let knots = e_nurbs.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];
    let e_start = nurbs_eval(&e_nurbs.as_view(), u_start);
    let e_end = nurbs_eval(&e_nurbs.as_view(), u_end);
    let total_length = (e_end - e_start).abs();

    if total_length <= 0.0 {
        // Zero-length E move: constant NURBS at e_start.
        return ScalarNurbs::try_new(
            1,
            vec![t_start, t_start, t_start + 1e-6, t_start + 1e-6],
            vec![e_start, e_start],
        )
        .map_err(construct_to_shape_error);
    }

    let sign = if e_end >= e_start { 1.0 } else { -1.0 };
    let profile = trapezoidal_profile(total_length, feedrate, limits);

    // Build piecewise degree-2 NURBS in the time domain.
    // Three phases: accel ramp, cruise, decel ramp.
    // Each phase is a quadratic Bezier piece.
    //
    // For a trapezoidal profile:
    //   Phase 1 (accel): s(t) = 0.5 * a * (t - t0)^2, t in [t0, t0+t_ramp]
    //   Phase 2 (cruise): s(t) = s_ramp + v_cruise * (t - t1), t in [t1, t1+t_cruise]
    //   Phase 3 (decel): s(t) = s_total - 0.5 * a * (t_end - t)^2, t in [t2, t_end]
    //
    // We express e(t) = e_start + sign * s(t) for each phase.

    let t0 = t_start;
    let s_ramp = profile.s_ramp;
    let v_cruise = profile.v_cruise;

    // Build as a single degree-2 NURBS with multiple knot spans.
    // For a trapezoidal profile with 3 phases, we need a degree-2 B-spline
    // with breakpoints at t0, t1, t2, t_end.
    //
    // Simpler approach: build the position as a piecewise quadratic with
    // control points computed from the trapezoid phases.

    if profile.is_triangular {
        // Triangular: only accel + decel, no cruise.
        // Two quadratic pieces: [t0, t_peak] accel, [t_peak, t_end_tri] decel.
        let t_peak = t0 + profile.t_ramp;
        let t_end_tri = t_peak + profile.t_ramp;
        let s_peak = profile.s_ramp; // = total_length / 2

        let e_at_peak = e_start + sign * s_peak;

        // Knot vector for degree-2, 2 pieces: [t0,t0,t0, t_peak, t_end,t_end,t_end]
        // -> 4 control points

        // Control points for the two quadratic pieces:
        // Piece 1 (accel): e(t0)=e_start, e(t_peak)=e_at_peak, e'(t0)=0
        //   cp0 = e_start
        //   cp1 = e_start (tangent at t0 is zero -> cp1 = cp0)
        //     Actually for a Bezier on [t0, t_peak]: e'(t0) = 2*(cp1-cp0)/(t_peak-t0)
        //     We want e'(t0) = 0, so cp1 = cp0 = e_start
        //   cp2 = e_at_peak? No, we need C0 at the join. Let me use B-spline CPs.
        //
        // For a degree-2 B-spline with the given knot vector, the control points
        // satisfy: curve(t0) = cp0, curve(t_end) = cp3, and at t_peak the curve
        // is C1-continuous.
        //
        // Using the de Boor property:
        //   curve(t0) = cp0 (clamped start)
        //   curve(t_end) = cp3 (clamped end)
        //   At t_peak: left value = (1-alpha)*cp1 + alpha*cp2 from left piece
        //              right value = (1-beta)*cp1 + beta*cp2 from right piece
        //              where the Greville abscissae give the relationship.
        //
        // Simpler: use explicit Bezier control points for each piece.
        // Piece 1 on [t0, t_peak]: quadratic Bezier
        //   B(0) = e_start, B(1) = e_at_peak, B'(0) = 0
        //   -> cp0 = e_start, cp1 = e_start, cp2 = e_at_peak
        //   But B'(0) = 2*(cp1-cp0)/(t_peak-t0) ... with unit-parameter Bezier
        //   In unit parameter: cp0=e_start, cp1=e_start, cp2=e_at_peak -> B'(0)=0, B'(1)=2*(e_at_peak-e_start)/(t_peak-t0)
        //   Actual velocity at t_peak = 2*(s_peak)/(t_peak-t0) = 2*s_peak/t_ramp
        //   = 2*(0.5*a*t_ramp^2)/t_ramp = a*t_ramp = v_cruise. Correct!
        //
        // Piece 2 on [t_peak, t_end]: quadratic Bezier
        //   B(0) = e_at_peak, B(1) = e_end, B'(0) = sign*v_cruise (matching velocity)
        //   -> cp0 = e_at_peak, cp2 = e_end
        //   B'(0) = 2*(cp1-cp0)/(t_end-t_peak) = sign*v_cruise
        //   cp1 = e_at_peak + sign*v_cruise*(t_end-t_peak)/2
        //       = e_at_peak + sign*v_cruise*t_ramp/2
        //       = e_at_peak + sign*(a*t_ramp)*t_ramp/2
        //       = e_at_peak + sign*s_peak
        //   But e_end = e_start + sign*total_length = e_start + sign*2*s_peak
        //   So cp1 = e_start + sign*s_peak + sign*s_peak = e_start + sign*2*s_peak = e_end? No...
        //   cp1 = e_at_peak + sign*s_peak = (e_start + sign*s_peak) + sign*s_peak = e_start + sign*2*s_peak = e_end
        //   That means cp1 = e_end, cp2 = e_end. Then B'(1) = 0. Correct for decel!
        //
        // So for the B-spline with knots [t0,t0,t0,t_peak,t_end,t_end,t_end]:
        //   cp0 = e_start  (clamped start)
        //   cp1 = e_start  (zero initial velocity — Greville point at (t0+t_peak)/2)
        //   cp2 = e_end    (matching velocity at t_peak, zero final velocity)
        //   cp3 = e_end    (clamped end)
        //
        // Wait, this is a degree-2 B-spline, not two separate Beziers. Let me
        // verify via Greville abscissae. For degree 2, knots [t0,t0,t0,t_peak,t_end,t_end,t_end]:
        //   Greville[0] = (t0+t0)/2 = t0
        //   Greville[1] = (t0+t_peak)/2
        //   Greville[2] = (t_peak+t_end)/2
        //   Greville[3] = (t_end+t_end)/2 = t_end
        //
        // Evaluation at t0 (clamped): cp0 = e_start. Correct.
        // Evaluation at t_end (clamped): cp3 = e_end. Correct.
        // Derivative at t0: 2*(cp1-cp0)/(t_peak-t0). We want 0, so cp1=cp0=e_start. Correct.
        // Derivative at t_end: 2*(cp3-cp2)/(t_end-t_peak). We want 0, so cp2=cp3=e_end. Correct.
        //
        // At t_peak (interior knot, multiplicity 1):
        //   Left limit value: quadratic Bezier [cp0, cp1, cp2] at u=1 on [t0,t_peak] = cp2 = e_end
        //   Right limit value: quadratic Bezier [cp1, cp2, cp3] at u=0 on [t_peak,t_end] = cp1 = e_start
        //   These should match for C0! But e_start != e_end. This means the simple
        //   4-CP B-spline with single interior knot doesn't give C0 at the join
        //   with these CP choices.
        //
        // The issue is that for a degree-2 B-spline with simple interior knot, we
        // get C1 continuity automatically. The left and right evaluation at t_peak
        // both give a weighted combination of cp1 and cp2.
        //
        // Let me just directly compute: at t_peak, the B-spline evaluates to:
        //   The active CPs for span [t0,t_peak,t_end] are cp0,cp1,cp2 and cp1,cp2,cp3.
        //   Actually for the knot vector [t0,t0,t0,t_peak,t_end,t_end,t_end], degree=2:
        //     n = 4 CPs, m = 7 knots. Spans: [t0,t_peak) uses cps 0,1,2. [t_peak,t_end) uses cps 1,2,3.
        //
        // At u=t_peak (from the left, in span [t0,t_peak)):
        //   The de Boor gives: since u=t_peak is the right endpoint of span,
        //   the value is determined by the full triangle. For u=t_peak:
        //     alpha for knot interval [t0,t_peak]: (t_peak-t0)/(t_peak-t0)=1
        //     -> walks toward cp2 side.
        //   Result = cp2. So at left limit, value = cp2.
        //
        //   Similarly at u=t_peak from the right (span [t_peak,t_end)):
        //     alpha for knot interval [t_peak,t_end]: (t_peak-t_peak)/(t_end-t_peak)=0
        //     -> walks toward cp1 side.
        //   Result = cp1. So at right limit, value = cp1.
        //
        //   For C0 at t_peak we need cp1 = cp2. That constrains us to have cp1=cp2=e_at_peak.
        //   Then derivative at t0: 2*(cp1-cp0)/(t_peak-t0) = 2*(e_at_peak-e_start)/t_ramp
        //                        = 2*sign*s_peak/t_ramp = sign*a*t_ramp = sign*v_cruise
        //   But we want zero initial velocity! The constraint system is over-determined
        //   for only 4 CPs. We need more knots.
        //
        // OK, this approach of trying to fit a degree-2 B-spline to a triangular
        // profile with 4 CPs is getting complicated. Let me use a simpler approach:
        // build two separate degree-2 Bezier pieces and concatenate them into a
        // multi-piece NURBS with a double interior knot (C0 join).

        // For a double interior knot at t_peak, degree 2:
        // Knots: [t0,t0,t0, t_peak,t_peak, t_end,t_end,t_end] -> 5 CPs

        // Piece 1 (accel) [t0, t_peak]: Bezier CPs in unit param:
        //   cp0 = e_start, cp1 = e_start (zero initial vel), cp2 = e_at_peak
        // Piece 2 (decel) [t_peak, t_end]: Bezier CPs:
        //   cp2 = e_at_peak (shared with piece 1 end via C0), cp3 = e_end (= e_end), cp4 = e_end (zero final vel)
        //
        // Wait, with double knot at t_peak the pieces share only cp2 for C0.
        // Actually no: for degree 2 with knots [t0,t0,t0,t_peak,t_peak,t_end,t_end,t_end]:
        //   n_cps = 8 - 2 - 1 = 5. Spans: [t0,t_peak) uses cps 0,1,2. [t_peak,t_end) uses cps 2,3,4.
        //   At t_peak from left: cp2 = e_at_peak. At t_peak from right: cp2 = e_at_peak. C0 by construction.
        //   But C1 is NOT enforced (double knot drops continuity by 1, from C1 to C0).
        //
        // Control points:
        //   cp0 = e_start
        //   cp1 = e_start   (zero derivative at t0: d/dt at t0 = 2*(cp1-cp0)/(t_peak-t0) = 0)
        //   cp2 = e_at_peak (C0 at t_peak)
        //   cp3 = e_end     (makes derivative at t_peak from right = 2*(cp3-cp2)/(t_end-t_peak)
        //                    = 2*sign*s_peak/t_ramp = sign*v_cruise — matches left derivative
        //                    which is 2*(cp2-cp1)/(t_peak-t0) = 2*sign*s_peak/t_ramp = sign*v_cruise)
        //   cp4 = e_end     (zero derivative at t_end: 2*(cp4-cp3)/(t_end-t_peak) = 0)

        // Hmm, left deriv at t_peak = 2*(cp2-cp1)/(t_peak-t0) = 2*(e_at_peak - e_start)/t_ramp = 2*sign*s_peak/t_ramp
        // s_peak = total_length/2
        // v_cruise = sqrt(a*total_length)  (triangular case)
        // 2*s_peak/t_ramp = 2*(0.5*a*t_ramp^2)/t_ramp = a*t_ramp = v_cruise. Correct.
        //
        // Right deriv at t_peak = 2*(cp3-cp2)/(t_end-t_peak) = 2*(e_end - e_at_peak)/t_ramp
        // = 2*sign*(total_length - s_peak)/t_ramp = 2*sign*s_peak/t_ramp = sign*v_cruise. Correct.
        //
        // So derivative is continuous at t_peak. Actually C1, not just C0. Good.

        let cps = vec![e_start, e_start, e_at_peak, e_end, e_end];

        // Guard against degenerate knot span (t_ramp == 0 shouldn't happen but
        // clamp to a minimum epsilon).
        let dt = (t_end_tri - t0).max(1e-12);
        let t_peak_safe = t0 + dt / 2.0;
        let t_end_safe = t0 + dt;

        return ScalarNurbs::try_new(
            2,
            vec![
                t0,
                t0,
                t0,
                t_peak_safe,
                t_peak_safe,
                t_end_safe,
                t_end_safe,
                t_end_safe,
            ],
            cps,
        )
        .map_err(construct_to_shape_error);
    }

    // Full trapezoidal: 3 phases — accel, cruise, decel.
    // Degree-2 B-spline with double interior knots at t1 and t2.
    // Knots: [t0,t0,t0, t1,t1, t2,t2, t_end,t_end,t_end] -> 7 CPs
    //
    // Spans: [t0,t1) uses cps 0,1,2. [t1,t2) uses cps 2,3,4. [t2,t_end) uses cps 4,5,6.
    //
    // Phase 1 (accel on [t0, t1]):
    //   cp0 = e_start (clamped)
    //   cp1 = e_start (zero initial velocity)
    //   cp2 = e_start + sign * s_ramp (C0 at t1)
    //   Left deriv at t1: 2*(cp2-cp1)/(t1-t0) = 2*sign*s_ramp/t_ramp = sign*v_cruise
    //
    // Phase 2 (cruise on [t1, t2]):
    //   cp2 shared (C0)
    //   cp3: Right deriv at t1 = 2*(cp3-cp2)/(t2-t1). Want = sign*v_cruise.
    //        cp3 = cp2 + sign*v_cruise*(t2-t1)/2 = e_start + sign*s_ramp + sign*v_cruise*t_cruise/2
    //   cp4 = e_start + sign*(s_ramp + v_cruise*t_cruise) = e_start + sign*(total_length - s_ramp)
    //        = e_end - sign*s_ramp
    //   Left deriv at t2: 2*(cp4-cp3)/(t2-t1).
    //     cp4 - cp3 = (e_start + sign*(total_length - s_ramp)) - (e_start + sign*s_ramp + sign*v_cruise*t_cruise/2)
    //               = sign*(total_length - 2*s_ramp - v_cruise*t_cruise/2)
    //     total_length - 2*s_ramp = v_cruise * t_cruise
    //     so cp4 - cp3 = sign*(v_cruise*t_cruise - v_cruise*t_cruise/2) = sign*v_cruise*t_cruise/2
    //     2*(cp4-cp3)/(t2-t1) = 2*sign*v_cruise*t_cruise/(2*t_cruise) = sign*v_cruise. Correct!
    //
    // Phase 3 (decel on [t2, t_end]):
    //   cp4 shared (C0)
    //   cp5: Right deriv at t2 = 2*(cp5-cp4)/(t_end-t2) = sign*v_cruise
    //        cp5 = cp4 + sign*v_cruise*t_ramp/2 = (e_end - sign*s_ramp) + sign*v_cruise*t_ramp/2
    //            = e_end - sign*s_ramp + sign*(a*t_ramp)*t_ramp/2 = e_end - sign*s_ramp + sign*s_ramp = e_end
    //   cp6 = e_end (clamped, zero final velocity: 2*(cp6-cp5)/(t_end-t2)=0 since cp5=cp6=e_end)

    let e_at_t1 = e_start + sign * s_ramp;
    let e_at_t2 = e_end - sign * s_ramp;
    let t_cruise = profile.t_cruise;

    let cp3 = e_at_t1 + sign * v_cruise * t_cruise / 2.0;

    let cps = vec![
        e_start, // cp0
        e_start, // cp1
        e_at_t1, // cp2
        cp3,     // cp3
        e_at_t2, // cp4
        e_end,   // cp5
        e_end,   // cp6
    ];

    // Guard: ensure all knot spans are non-degenerate.
    let t1_safe = t0 + profile.t_ramp.max(1e-12);
    let t2_safe = t1_safe + t_cruise.max(1e-12);
    let t_end_safe = t2_safe + profile.t_ramp.max(1e-12);

    ScalarNurbs::try_new(
        2,
        vec![
            t0, t0, t0, t1_safe, t1_safe, t2_safe, t2_safe, t_end_safe, t_end_safe, t_end_safe,
        ],
        cps,
    )
    .map_err(construct_to_shape_error)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Map a `ConstructError` from NURBS construction into a `ShapeError`.
#[allow(clippy::needless_pass_by_value)] // Used as `.map_err(construct_to_shape_error)`.
fn construct_to_shape_error(e: nurbs::ConstructError) -> crate::ShapeError {
    crate::ShapeError::Algebra {
        index: 0,
        detail: nurbs::AlgebraError::NotImplemented(
            // ConstructError in e_independent NURBS construction is a logic bug.
            // Use the `NotImplemented` variant as a catch-all carrier; the Display
            // on ConstructError is lost here but the debug repr in ShapeError
            // carries the AlgebraError variant.
            match e {
                nurbs::ConstructError::DegreeExceeded { .. } => "e_independent: degree exceeded",
                nurbs::ConstructError::KnotCountMismatch { .. } => {
                    "e_independent: knot count mismatch"
                }
                nurbs::ConstructError::KnotsNotClamped => "e_independent: knots not clamped",
                nurbs::ConstructError::KnotsNotMonotone => "e_independent: knots not monotone",
                nurbs::ConstructError::DegenerateKnotRange => {
                    "e_independent: degenerate knot range"
                }
            },
        ),
    }
}

/// Extract the E path length from a scalar NURBS: `|e(u_end) - e(u_start)|`.
fn e_path_length(e_nurbs: &ScalarNurbs<f64>) -> f64 {
    let knots = e_nurbs.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];
    let e_start = nurbs_eval(&e_nurbs.as_view(), u_start);
    let e_end = nurbs_eval(&e_nurbs.as_view(), u_end);
    (e_end - e_start).abs()
}

/// Result of trapezoidal profile computation.
struct TrapProfile {
    v_cruise: f64,
    t_ramp: f64,
    t_cruise: f64,
    s_ramp: f64,
    is_triangular: bool,
}

/// Compute trapezoidal velocity profile parameters.
fn trapezoidal_profile(total_length: f64, feedrate: f64, limits: &ELimits) -> TrapProfile {
    let v_cruise = feedrate.min(limits.v_max);
    let a_max = limits.a_max;

    let t_ramp = v_cruise / a_max;
    let s_ramp = 0.5 * a_max * t_ramp * t_ramp;

    if 2.0 * s_ramp > total_length {
        // Triangular profile: can't reach cruise speed.
        let t_ramp_tri = (total_length / a_max).sqrt();
        let v_peak = a_max * t_ramp_tri;
        let s_ramp_tri = total_length / 2.0;
        TrapProfile {
            v_cruise: v_peak,
            t_ramp: t_ramp_tri,
            t_cruise: 0.0,
            s_ramp: s_ramp_tri,
            is_triangular: true,
        }
    } else {
        // Full trapezoidal profile.
        let s_cruise = total_length - 2.0 * s_ramp;
        let t_cruise = s_cruise / v_cruise;
        TrapProfile {
            v_cruise,
            t_ramp,
            t_cruise,
            s_ramp,
            is_triangular: false,
        }
    }
}

/// Compute just the duration from a trapezoidal profile.
fn trapezoidal_duration(total_length: f64, feedrate: f64, limits: &ELimits) -> f64 {
    let p = trapezoidal_profile(total_length, feedrate, limits);
    2.0 * p.t_ramp + p.t_cruise
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests;
