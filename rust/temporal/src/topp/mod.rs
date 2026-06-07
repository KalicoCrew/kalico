use crate::{GridConfig, Limits, TopProfile};
use constraints::{BoundaryInfeasibility, BuildOutcome, EndpointVelocities, build};
use nurbs::VectorNurbs;
use scaling::SolverScale;

pub mod chain;
pub mod constraints;
pub(crate) mod output;
pub mod path;
pub mod scaling;
pub(crate) mod solver;
pub mod stencil;
pub(crate) mod verify;

pub use solver::{AxisJerkGradient, axis_jerk_gradient_for_test};

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

    // Nondimensionalize before building the SOCP: Clarabel's KKT conditioning
    // degrades with v_max² in raw mm units (b = v² ≈ 1e6 at 1000 mm/s stalls
    // InsufficientProgress); the same problem in solver units solves cleanly.
    // The solve runs entirely in scaled units; verify and output stay physical.
    let scale = SolverScale::for_limits(limits);
    let scaled_grid = scale.scale_grid(&arc_grid);
    let scaled_limits = scale.scale_limits(limits);
    let scaled_endpoints = EndpointVelocities {
        v_start: scale.scale_velocity(v_start),
        v_end: scale.scale_velocity(v_end),
    };

    let bundle = match build(&scaled_grid, &scaled_limits, scaled_endpoints, &scale) {
        BuildOutcome::Ok(b) => b,
        BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc { mvc_b }) => {
            return Ok(boundary_infeasible_profile(
                &arc_grid,
                *grid,
                crate::BoundarySide::Start,
                scale.unscale_b(mvc_b),
                0,
            ));
        }
        BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMvc { mvc_b }) => {
            let last = arc_grid.s.len() - 1;
            return Ok(boundary_infeasible_profile(
                &arc_grid,
                *grid,
                crate::BoundarySide::End,
                scale.unscale_b(mvc_b),
                last,
            ));
        }
    };

    let call_slp = |tol| {
        solver::slp_solve_with_axis_jerk(&bundle, &scaled_grid, &scaled_limits, tol, &scale)
            .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))
    };

    let (mut solver_result, slp_outcome) = match tolerance {
        ToleranceMode::Tight => call_slp(1e-8)?,
        ToleranceMode::Fast => call_slp(1e-5)?,
        ToleranceMode::Auto => {
            let (fast_result, fast_outcome) = call_slp(1e-5)?;
            if solver_outcome_is_success(&fast_result, &fast_outcome) {
                (fast_result, fast_outcome)
            } else {
                call_slp(1e-8)?
            }
        }
    };

    scale.unscale_result(&mut solver_result);
    let h_phys = arc_grid.s[1] - arc_grid.s[0];
    let verify_report = verify::check(&arc_grid, &solver_result, limits, h_phys);

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
