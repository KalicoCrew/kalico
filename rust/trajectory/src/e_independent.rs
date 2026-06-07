use crate::ELimits;
use nurbs::eval::eval as nurbs_eval;
use nurbs::ScalarNurbs;

pub fn schedule_e_duration(e_nurbs: &ScalarNurbs<f64>, feedrate: f64, limits: &ELimits) -> f64 {
    let total_length = e_path_length(e_nurbs);
    if total_length <= 0.0 {
        return 0.0;
    }
    trapezoidal_duration(total_length, feedrate, limits)
}

/// # Errors
///
/// Returns `ShapeError::Algebra` if NURBS construction fails.
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
        return ScalarNurbs::try_new(
            1,
            vec![t_start, t_start, t_start + 1e-6, t_start + 1e-6],
            vec![e_start, e_start],
        )
        .map_err(construct_to_shape_error);
    }

    let sign = if e_end >= e_start { 1.0 } else { -1.0 };
    let profile = trapezoidal_profile(total_length, feedrate, limits);

    let t0 = t_start;
    let s_ramp = profile.s_ramp;
    let v_cruise = profile.v_cruise;

    if profile.is_triangular {
        let t_peak = t0 + profile.t_ramp;
        let t_end_tri = t_peak + profile.t_ramp;
        let s_peak = profile.s_ramp;

        let e_at_peak = e_start + sign * s_peak;

        // Degree-2 B-spline with double interior knot at t_peak (C0, C1 join).
        // Knots: [t0,t0,t0, t_peak,t_peak, t_end,t_end,t_end] → 5 CPs.
        // cp0=e_start, cp1=e_start (zero v at t0), cp2=e_at_peak (C0),
        // cp3=e_end (matching v at t_peak), cp4=e_end (zero v at t_end).
        let cps = vec![e_start, e_start, e_at_peak, e_end, e_end];

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

    // Full trapezoidal: degree-2 B-spline with double interior knots at t1 and t2.
    // Knots: [t0,t0,t0, t1,t1, t2,t2, t_end,t_end,t_end] → 7 CPs.
    // Phase 1 (accel): cp0=e_start, cp1=e_start (zero v), cp2=e_start+sign*s_ramp (C0 at t1).
    // Phase 2 (cruise): cp2 shared, cp3=cp2+sign*v_cruise*t_cruise/2, cp4=e_end-sign*s_ramp.
    // Phase 3 (decel): cp4 shared, cp5=e_end, cp6=e_end (zero v).
    let e_at_t1 = e_start + sign * s_ramp;
    let e_at_t2 = e_end - sign * s_ramp;
    let t_cruise = profile.t_cruise;

    let cp3 = e_at_t1 + sign * v_cruise * t_cruise / 2.0;

    let cps = vec![e_start, e_start, e_at_t1, cp3, e_at_t2, e_end, e_end];

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

#[allow(clippy::needless_pass_by_value)]
fn construct_to_shape_error(e: nurbs::ConstructError) -> crate::ShapeError {
    crate::ShapeError::Algebra {
        index: 0,
        detail: nurbs::AlgebraError::NotImplemented(match e {
            nurbs::ConstructError::DegreeExceeded { .. } => "e_independent: degree exceeded",
            nurbs::ConstructError::KnotCountMismatch { .. } => "e_independent: knot count mismatch",
            nurbs::ConstructError::KnotsNotClamped => "e_independent: knots not clamped",
            nurbs::ConstructError::KnotsNotMonotone => "e_independent: knots not monotone",
            nurbs::ConstructError::DegenerateKnotRange => "e_independent: degenerate knot range",
        }),
    }
}

fn e_path_length(e_nurbs: &ScalarNurbs<f64>) -> f64 {
    let knots = e_nurbs.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];
    let e_start = nurbs_eval(&e_nurbs.as_view(), u_start);
    let e_end = nurbs_eval(&e_nurbs.as_view(), u_end);
    (e_end - e_start).abs()
}

struct TrapProfile {
    v_cruise: f64,
    t_ramp: f64,
    t_cruise: f64,
    s_ramp: f64,
    is_triangular: bool,
}

fn trapezoidal_profile(total_length: f64, feedrate: f64, limits: &ELimits) -> TrapProfile {
    let v_cruise = feedrate.min(limits.v_max);
    let a_max = limits.a_max;

    let t_ramp = v_cruise / a_max;
    let s_ramp = 0.5 * a_max * t_ramp * t_ramp;

    if 2.0 * s_ramp > total_length {
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

fn trapezoidal_duration(total_length: f64, feedrate: f64, limits: &ELimits) -> f64 {
    let p = trapezoidal_profile(total_length, feedrate, limits);
    2.0 * p.t_ramp + p.t_cruise
}

#[cfg(test)]
mod tests;
