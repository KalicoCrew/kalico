use crate::topp::path::ArclengthGrid;
use crate::topp::solver::{SlpOutcome, SolverResult, SolverStatus};
use crate::topp::verify::{self, VerifyReport};
use crate::{GridConfig, GridSample, InfeasibleReason, SolveStatus, TopProfile};

pub(crate) fn assemble(
    grid: &ArclengthGrid,
    result: &SolverResult,
    verify: &VerifyReport,
    grid_config: GridConfig,
    slp_outcome: SlpOutcome,
) -> TopProfile {
    let n = grid.s.len();
    debug_assert_eq!(result.b.len(), n);
    debug_assert_eq!(result.a.len(), n);
    debug_assert_eq!(verify.binding_per_grid.len(), n);

    let mut samples = Vec::with_capacity(n);
    for i in 0..n {
        samples.push(GridSample {
            s: grid.s[i],
            v: result.b[i].max(0.0).sqrt(),
            a: result.a[i],
            b: result.b[i],
            binding: verify.binding_per_grid[i],
        });
    }

    let mut total_time = 0.0;
    for i in 0..n - 1 {
        let ds = grid.s[i + 1] - grid.s[i];
        let v_sum = samples[i].v + samples[i + 1].v;
        if v_sum > 1e-12 {
            total_time += ds * 2.0 / v_sum;
        } else {
            total_time += ds / 1e-9_f64.max(samples[i].v.max(samples[i + 1].v));
        }
    }

    TopProfile {
        samples,
        status: map_status(result.status, verify, slp_outcome),
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
        ) => SolveStatus::DivergedSlp {
            last_max_ratio,
            outer_iters,
        },
        (SlpOutcome::MaxIters { last_max_ratio }, _) => {
            if verify.feasible {
                SolveStatus::SolvedInexact {
                    residual: verify.worst_violation,
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
