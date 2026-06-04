use crate::{GridConfig, Limits, TopProfile};
use constraints::{BoundaryInfeasibility, BuildOutcome, EndpointVelocities, build};
use nurbs::VectorNurbs;

pub mod constraints;
pub(crate) mod output;
pub mod path;
pub(crate) mod solver;
pub mod stencil;
pub(crate) mod verify;

#[non_exhaustive]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum ToleranceMode {
    Tight,
    Fast,
    #[default]
    Auto,
}

#[derive(Debug, thiserror::Error)]
pub enum ScheduleError {
    #[error("invalid endpoint velocity: {0}")]
    InvalidEndpointVelocity(&'static str),
    #[error("path parameterization failed: {0}")]
    PathParam(String),
    #[error("solver setup failed: {0}")]
    SolverSetup(String),
}

/// Equivalent to `schedule_segment_with_tolerance(..., ToleranceMode::Tight)`.
///
/// Solver-runtime infeasibility / max-iter surface as `SolveStatus` on the
/// returned profile, not as `ScheduleError`. `ScheduleError` is for
/// setup-time programming errors only.
pub fn schedule_segment(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
) -> Result<TopProfile, ScheduleError> {
    schedule_segment_with_tolerance(curve, limits, grid, v_start, v_end, ToleranceMode::Tight)
}

/// Solver-runtime infeasibility / max-iter surface as `SolveStatus` on the
/// returned profile, not as `ScheduleError`. `ScheduleError` is for
/// setup-time programming errors only.
pub fn schedule_segment_with_tolerance(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
    tolerance: ToleranceMode,
) -> Result<TopProfile, ScheduleError> {
    if !v_start.is_finite() || v_start < 0.0 {
        return Err(ScheduleError::InvalidEndpointVelocity(
            "v_start must be finite, ≥ 0",
        ));
    }
    if !v_end.is_finite() || v_end < 0.0 {
        return Err(ScheduleError::InvalidEndpointVelocity(
            "v_end must be finite, ≥ 0",
        ));
    }
    if !matches!(grid.scheme, crate::GridScheme::UniformArclength) {
        return Err(ScheduleError::SolverSetup(
            "only GridScheme::UniformArclength is implemented in Step 4".into(),
        ));
    }

    let arc_grid = path::sample_arclength_grid(curve, grid.n)
        .map_err(|e| ScheduleError::PathParam(format!("{e}")))?;

    let bundle = match build(&arc_grid, limits, EndpointVelocities { v_start, v_end }) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc { mvc_b }) => {
            return Ok(boundary_infeasible_profile(
                &arc_grid,
                *grid,
                crate::BoundarySide::Start,
                mvc_b,
                0,
            ));
        }
        BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMvc { mvc_b }) => {
            let last = arc_grid.s.len() - 1;
            return Ok(boundary_infeasible_profile(
                &arc_grid,
                *grid,
                crate::BoundarySide::End,
                mvc_b,
                last,
            ));
        }
    };

    let (solver_result, slp_outcome) = match tolerance {
        ToleranceMode::Tight => solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-8)
            .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?,
        ToleranceMode::Fast => solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-5)
            .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?,
        ToleranceMode::Auto => {
            let (fast_result, fast_outcome) =
                solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-5)
                    .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?;
            if solver_outcome_is_success(&fast_result, &fast_outcome) {
                (fast_result, fast_outcome)
            } else {
                solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-8)
                    .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?
            }
        }
    };

    let verify_report = verify::check(&arc_grid, &solver_result, limits, bundle.h);

    Ok(output::assemble(
        &arc_grid,
        &solver_result,
        &verify_report,
        *grid,
        slp_outcome,
    ))
}

fn solver_outcome_is_success(result: &solver::SolverResult, outcome: &solver::SlpOutcome) -> bool {
    let status_ok = matches!(
        result.status,
        solver::SolverStatus::Solved | solver::SolverStatus::SolvedInexact { .. }
    );
    let outcome_ok = matches!(outcome, solver::SlpOutcome::Converged { .. });
    status_ok && outcome_ok
}

fn boundary_infeasible_profile(
    grid: &path::ArclengthGrid,
    cfg: GridConfig,
    side: crate::BoundarySide,
    mvc_b: f64,
    at_grid: usize,
) -> TopProfile {
    use crate::{BindingConstraint, GridSample, InfeasibleReason, SolveStatus};
    let samples = grid
        .s
        .iter()
        .map(|&s| GridSample {
            s,
            v: 0.0,
            a: 0.0,
            b: 0.0,
            binding: BindingConstraint::Boundary,
        })
        .collect();
    TopProfile {
        samples,
        status: SolveStatus::Infeasible {
            at_grid,
            reason: InfeasibleReason::BoundaryAboveMVC { side, mvc_b },
        },
        grid_scheme: cfg.scheme,
        total_time: f64::INFINITY,
    }
}

#[cfg(test)]
mod tests;
