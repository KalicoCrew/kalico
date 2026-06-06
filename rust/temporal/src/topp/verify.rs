use crate::topp::path::ArclengthGrid;
use crate::topp::solver::SolverResult;
use crate::{Axis, BindingConstraint, Limits};

/// 0.2% feasibility margin for velocity, acceleration, and centripetal
/// constraints. These use width-0 quantities (b_i, a_i) with no stencil noise.
pub(crate) const EPS_FEAS: f64 = 2e-3;

/// 5% feasibility margin for per-axis jerk constraints. The jerk ratio uses the
/// width-1 b-FD `s_dddot_at` stencil; on a time-optimal profile riding the jerk
/// limit the stencil's discretization noise is 1–4% (same rationale as
/// `SLP_EPS_FEAS` in solver.rs). A 0.2% band turns FD noise into phantom
/// infeasibility on micro-segments.
pub(crate) const EPS_FEAS_JERK: f64 = 5e-2;

/// Threshold below which a normalised ratio is treated as fully slack.
const SLACK_THRESHOLD: f64 = 1e-6;

/// `b_i` below which endpoints are tagged `BindingConstraint::Boundary`.
const BOUNDARY_B_TOL: f64 = 1e-9;

#[derive(Debug, Clone)]
pub(crate) struct VerifyReport {
    pub binding_per_grid: Vec<BindingConstraint>,
    /// `max_i(worst_ratio_at_i) − 1.0`. Positive means infeasible.
    #[allow(dead_code)]
    pub worst_violation: f64,
    pub worst_violation_grid: usize,
    pub feasible: bool,
}

struct PointInputs<'a> {
    cp: [f64; 3],
    cpp: [f64; 3],
    cppp: [f64; 3],
    kappa: f64,
    b_i: f64,
    s_dot: f64,
    s_ddot: f64,
    s_dddot: f64,
    limits: &'a Limits,
}

/// Output of [`ratios_at`]: per-point worst entry (for the binding-tag report)
/// plus per-class maxima needed for the two-band feasibility check.
struct PointRatios {
    /// Overall per-point worst ratio (tie-breaking: Velocity > AxisAccel >
    /// AxisJerk > Centripetal; X > Y > Z).
    worst_ratio: f64,
    worst_tag: BindingConstraint,
    /// Max ratio across the three `AxisJerk` entries at this point.
    max_jerk: f64,
    /// Max ratio across the seven non-jerk entries (velocity, accel,
    /// centripetal) at this point.
    max_non_jerk: f64,
}

/// Compute all 10 normalised constraint ratios at a single grid point and
/// return both the per-point winner (for `binding_per_grid` / `worst_violation`)
/// and the per-class maxima (for the two-band feasibility check).
///
/// Computing the class maxima over all entries — not just the per-point winner
/// — ensures that a co-located non-jerk violation is never hidden by a
/// dominant-but-in-band jerk ratio.
fn ratios_at(p: &PointInputs<'_>) -> PointRatios {
    let s_dot2 = p.s_dot * p.s_dot;
    let s_dot3 = s_dot2 * p.s_dot;

    let vel = [p.cp[0] * p.s_dot, p.cp[1] * p.s_dot, p.cp[2] * p.s_dot];
    let accel = [
        p.cpp[0] * s_dot2 + p.cp[0] * p.s_ddot,
        p.cpp[1] * s_dot2 + p.cp[1] * p.s_ddot,
        p.cpp[2] * s_dot2 + p.cp[2] * p.s_ddot,
    ];
    let jerk = [
        p.cppp[0] * s_dot3 + 3.0 * p.cpp[0] * p.s_dot * p.s_ddot + p.cp[0] * p.s_dddot,
        p.cppp[1] * s_dot3 + 3.0 * p.cpp[1] * p.s_dot * p.s_ddot + p.cp[1] * p.s_dddot,
        p.cppp[2] * s_dot3 + 3.0 * p.cpp[2] * p.s_dot * p.s_ddot + p.cp[2] * p.s_dddot,
    ];

    let lim = p.limits;
    let entries: [(f64, BindingConstraint); 10] = [
        (
            vel[0].abs() / lim.v_max[0],
            BindingConstraint::Velocity { axis: Axis::X },
        ),
        (
            vel[1].abs() / lim.v_max[1],
            BindingConstraint::Velocity { axis: Axis::Y },
        ),
        (
            vel[2].abs() / lim.v_max[2],
            BindingConstraint::Velocity { axis: Axis::Z },
        ),
        (
            accel[0].abs() / lim.a_max[0],
            BindingConstraint::AxisAccel { axis: Axis::X },
        ),
        (
            accel[1].abs() / lim.a_max[1],
            BindingConstraint::AxisAccel { axis: Axis::Y },
        ),
        (
            accel[2].abs() / lim.a_max[2],
            BindingConstraint::AxisAccel { axis: Axis::Z },
        ),
        (
            jerk[0].abs() / lim.j_max[0],
            BindingConstraint::AxisJerk { axis: Axis::X },
        ),
        (
            jerk[1].abs() / lim.j_max[1],
            BindingConstraint::AxisJerk { axis: Axis::Y },
        ),
        (
            jerk[2].abs() / lim.j_max[2],
            BindingConstraint::AxisJerk { axis: Axis::Z },
        ),
        (
            p.b_i.max(0.0) * p.kappa / lim.a_centripetal_max,
            BindingConstraint::Centripetal,
        ),
    ];

    let mut worst_ratio = 0.0_f64;
    let mut worst_tag = BindingConstraint::None;
    let mut max_jerk = 0.0_f64;
    let mut max_non_jerk = 0.0_f64;

    for (ratio, tag) in entries {
        if ratio > worst_ratio {
            worst_ratio = ratio;
            worst_tag = tag;
        }
        if matches!(tag, BindingConstraint::AxisJerk { .. }) {
            if ratio > max_jerk {
                max_jerk = ratio;
            }
        } else if ratio > max_non_jerk {
            max_non_jerk = ratio;
        }
    }

    let worst_tag = if worst_ratio < SLACK_THRESHOLD {
        BindingConstraint::None
    } else {
        worst_tag
    };

    PointRatios {
        worst_ratio,
        worst_tag,
        max_jerk,
        max_non_jerk,
    }
}

pub(crate) fn check(
    grid: &ArclengthGrid,
    result: &SolverResult,
    limits: &Limits,
    h: f64,
) -> VerifyReport {
    let n = grid.s.len();
    debug_assert_eq!(result.b.len(), n);
    debug_assert_eq!(result.a.len(), n);

    let mut binding_per_grid: Vec<BindingConstraint> = Vec::with_capacity(n);
    let mut global_worst_ratio: f64 = f64::NEG_INFINITY;
    let mut global_worst_idx: usize = 0;
    // Track worst jerk and non-jerk ratios independently so each is tested
    // against its own band (stencil-noise-tolerant for jerk, tight for rest).
    let mut worst_jerk_ratio: f64 = 0.0;
    let mut worst_non_jerk_ratio: f64 = 0.0;

    for i in 0..n {
        let b_i = result.b[i];
        let a_i = result.a[i];

        let s_dot = b_i.max(0.0).sqrt();
        let s_ddot = a_i;
        let s_dddot = crate::topp::stencil::s_dddot_at(&result.b, i, h);

        let pr = ratios_at(&PointInputs {
            cp: grid.c_prime[i],
            cpp: grid.c_double_prime[i],
            cppp: grid.c_triple_prime[i],
            kappa: grid.kappa[i],
            b_i,
            s_dot,
            s_ddot,
            s_dddot,
            limits,
        });

        let final_tag = if (i == 0 || i == n - 1) && b_i.abs() < BOUNDARY_B_TOL {
            BindingConstraint::Boundary
        } else {
            pr.worst_tag
        };

        if pr.worst_ratio > global_worst_ratio {
            global_worst_ratio = pr.worst_ratio;
            global_worst_idx = i;
        }

        // Accumulate per-class maxima across ALL entries at this point (not
        // just the per-point winner) so a co-located non-jerk violation is
        // never hidden by a dominant-but-in-band jerk ratio.
        if pr.max_jerk > worst_jerk_ratio {
            worst_jerk_ratio = pr.max_jerk;
        }
        if pr.max_non_jerk > worst_non_jerk_ratio {
            worst_non_jerk_ratio = pr.max_non_jerk;
        }

        binding_per_grid.push(final_tag);
    }

    if n == 0 {
        return VerifyReport {
            binding_per_grid,
            worst_violation: f64::NEG_INFINITY,
            worst_violation_grid: 0,
            feasible: true,
        };
    }

    let worst_violation = global_worst_ratio - 1.0;
    // Feasible when every constraint class is within its own band.
    let feasible =
        worst_jerk_ratio <= 1.0 + EPS_FEAS_JERK && worst_non_jerk_ratio <= 1.0 + EPS_FEAS;
    VerifyReport {
        binding_per_grid,
        worst_violation,
        worst_violation_grid: global_worst_idx,
        feasible,
    }
}

#[cfg(test)]
mod tests;
