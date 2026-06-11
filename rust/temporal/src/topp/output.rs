use crate::topp::solver::{SlpOutcome, SolverResult, SolverStatus};
use crate::topp::verify::{self, VerifyReport};
use crate::{GridConfig, GridSample, InfeasibleReason, SolveStatus, TopProfile};

const SLP_STALL_JERK_ACCEPT: f64 = 1.15;

pub(crate) fn assemble(
    s: &[f64],
    result: &SolverResult,
    verify: &VerifyReport,
    grid_config: GridConfig,
    slp_outcome: SlpOutcome,
) -> TopProfile {
    let n = s.len();
    debug_assert_eq!(result.b.len(), n);
    debug_assert_eq!(result.a.len(), n);
    debug_assert_eq!(verify.binding_per_grid.len(), n);

    let mut samples = Vec::with_capacity(n);
    for i in 0..n {
        samples.push(GridSample {
            s: s[i],
            v: result.b[i].max(0.0).sqrt(),
            a: result.a[i],
            b: result.b[i],
            binding: verify.binding_per_grid[i],
        });
    }

    let mut total_time = 0.0;
    for i in 0..n - 1 {
        let ds = s[i + 1] - s[i];
        let v_sum = samples[i].v + samples[i + 1].v;
        if v_sum > 1e-12 {
            total_time += ds * 2.0 / v_sum;
        } else {
            total_time += ds / 1e-9_f64.max(samples[i].v.max(samples[i + 1].v));
        }
    }

    let status = map_status(result.status, verify, slp_outcome);
    let total_time = if matches!(status, SolveStatus::Infeasible { .. }) {
        f64::INFINITY
    } else {
        total_time
    };

    TopProfile {
        samples,
        status,
        grid_scheme: grid_config.scheme,
        total_time,
    }
}

pub(crate) fn map_status(
    solver_status: SolverStatus,
    verify: &VerifyReport,
    slp_outcome: SlpOutcome,
) -> SolveStatus {
    let base = match solver_status {
        SolverStatus::Solved if verify.feasible => SolveStatus::Solved,
        SolverStatus::SolvedInexact { residual } if verify.feasible => {
            SolveStatus::SolvedInexact { residual }
        }
        SolverStatus::Solved | SolverStatus::SolvedInexact { .. } => SolveStatus::Infeasible {
            at_grid: verify.worst_violation_grid,
            reason: InfeasibleReason::SolverInfeasible,
        },
        SolverStatus::Infeasible => SolveStatus::Infeasible {
            at_grid: 0,
            reason: InfeasibleReason::SolverInfeasible,
        },
        SolverStatus::MaxIter { residual } => {
            if residual < verify::EPS_FEAS && verify.feasible {
                SolveStatus::SolvedInexact { residual }
            } else {
                SolveStatus::MaxIter {
                    last_residual: residual,
                }
            }
        }
    };

    let slp_stall_accepted = |last_max_ratio: f64| -> bool {
        last_max_ratio <= SLP_STALL_JERK_ACCEPT
            && verify.worst_non_jerk_ratio <= 1.0 + verify::EPS_FEAS
    };

    match (slp_outcome, &base) {
        (
            SlpOutcome::Converged { outer_iters },
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. },
        ) if outer_iters > 0 => SolveStatus::SolvedSlp { outer_iters },
        (
            SlpOutcome::Diverged {
                last_max_ratio,
                outer_iters: _,
            },
            _,
        ) if verify.feasible => {
            let _ = last_max_ratio;
            SolveStatus::SolvedInexact {
                residual: verify.worst_violation,
            }
        }
        (
            SlpOutcome::Diverged {
                last_max_ratio,
                outer_iters,
            },
            _,
        ) if slp_stall_accepted(last_max_ratio) => {
            let _ = outer_iters;
            SolveStatus::SolvedInexact {
                residual: last_max_ratio - 1.0,
            }
        }
        (
            SlpOutcome::Diverged {
                last_max_ratio,
                outer_iters,
            },
            _,
        ) => SolveStatus::DivergedSlp {
            last_max_ratio,
            outer_iters,
        },
        (SlpOutcome::MaxIters { last_max_ratio }, _) => {
            if verify.feasible || slp_stall_accepted(last_max_ratio) {
                SolveStatus::SolvedInexact {
                    residual: verify.worst_violation.max(last_max_ratio - 1.0),
                }
            } else {
                SolveStatus::MaxIterSlp { last_max_ratio }
            }
        }
        (SlpOutcome::Converged { .. } | SlpOutcome::InnerSolverFailure, _) => base,
    }
}

#[cfg(test)]
mod tests;
