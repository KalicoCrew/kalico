use crate::topp::path::ArclengthGrid;
use crate::topp::solver::SolverResult;
use crate::{Axis, BindingConstraint, Limits};

/// 0.2% feasibility margin. Uniform width-1 b-FD across SOCP/SLP/verifier.
pub(crate) const EPS_FEAS: f64 = 2e-3;

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

/// Worst normalised ratio and its tag at a single grid point.
/// Tie-breaking: Velocity > AxisAccel > AxisJerk > Centripetal; X > Y > Z.
fn worst_ratio_at(p: &PointInputs<'_>) -> (f64, BindingConstraint) {
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
    for (ratio, tag) in entries {
        if ratio > worst_ratio {
            worst_ratio = ratio;
            worst_tag = tag;
        }
    }

    if worst_ratio < SLACK_THRESHOLD {
        (worst_ratio, BindingConstraint::None)
    } else {
        (worst_ratio, worst_tag)
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

    for i in 0..n {
        let b_i = result.b[i];
        let a_i = result.a[i];

        let s_dot = b_i.max(0.0).sqrt();
        let s_ddot = a_i;
        let s_dddot = crate::topp::stencil::s_dddot_at(&result.b, i, h);

        let (worst_ratio, tag) = worst_ratio_at(&PointInputs {
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
            tag
        };

        if worst_ratio > global_worst_ratio {
            global_worst_ratio = worst_ratio;
            global_worst_idx = i;
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
    VerifyReport {
        binding_per_grid,
        worst_violation,
        worst_violation_grid: global_worst_idx,
        feasible: worst_violation <= EPS_FEAS,
    }
}

#[cfg(test)]
mod tests;
