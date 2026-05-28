//! TOPP pipeline: path → constraints → solver → verify → output.
//!
//! Spec §4.3.

use crate::{GridConfig, Limits, TopProfile};
use constraints::{BoundaryInfeasibility, BuildOutcome, EndpointVelocities, build};
use nurbs::VectorNurbs;

pub mod constraints;
pub(crate) mod output;
pub mod path;
pub(crate) mod solver;
pub mod stencil;
pub(crate) mod verify;

/// Solver tolerance strategy. Per Pi 5 throughput investigation Finding 2 +
/// Step-4.5 spec §2.1 (Codex review-1 corrected version).
#[non_exhaustive]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum ToleranceMode {
    /// Default Clarabel tolerances (1e-8). Always safe, slowest.
    Tight,
    /// Loosened tolerances (1e-5). ~11× faster on convergent geometry.
    /// May trigger `DivergedSlp` on fragile geometry (G5 cubic with endpoint κ).
    Fast,
    /// Try `Fast` first; on any non-success status, fall back to `Tight`.
    /// **Default for `schedule_segment_with_tolerance`.**
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

/// Backward-compatible single-segment entry point. Equivalent to
/// `schedule_segment_with_tolerance(..., ToleranceMode::Tight)`.
///
/// Spec §4.3, §4.4. Solver-runtime infeasibility / max-iter surface as
/// `SolveStatus` on the returned profile, *not* as `ScheduleError`.
/// `ScheduleError` is for setup-time programming errors only.
pub fn schedule_segment(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
) -> Result<TopProfile, ScheduleError> {
    schedule_segment_with_tolerance(curve, limits, grid, v_start, v_end, ToleranceMode::Tight)
}

/// New entry point with adaptive-tolerance support. Per Step-4.5 spec §2.1.
///
/// Solver-runtime infeasibility / max-iter surface as `SolveStatus` on the
/// returned profile, *not* as `ScheduleError`. `ScheduleError` is for
/// setup-time programming errors only.
pub fn schedule_segment_with_tolerance(
    curve: &VectorNurbs<f64, 3>,
    limits: &Limits,
    grid: &GridConfig,
    v_start: f64,
    v_end: f64,
    tolerance: ToleranceMode,
) -> Result<TopProfile, ScheduleError> {
    // Setup-time validation. NaN/negative endpoint velocities are caller bugs.
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

    // Stage 1: arclength grid.
    let arc_grid = path::sample_arclength_grid(curve, grid.n)
        .map_err(|e| ScheduleError::PathParam(format!("{e}")))?;

    // Stage 2: constraint bundle (also catches boundary-above-MVC).
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

    // Stage 3: solver. Two-stage SLP (Lee 2024 + Step 9 verifier-stencil
    // per-axis Cartesian jerk SLP). Stage 1 (path-jerk) tightens the SOCP
    // relaxation against the scalar-tangential jerk constraint
    // `|s⃛| ≤ J_path`. Stage 2 layers per-axis Cartesian jerk cuts on top
    // (active-set + L∞ trust region + accept-only-if-decrease) to drive
    // verifier-form `|j_axis| ≤ j_max[axis]` below ε_feas. For the common
    // straight-line / centripetal-cruise cases both loops converge at iter
    // 0 with no cuts, leaving runtime identical to the original SOCP. See
    // spec §11; Step 9 design notes in /tmp/pf_diagnosis.json.
    //
    // Stage 3 (modified): adaptive-tolerance routing through
    // `slp_solve_with_axis_jerk`. See `ToleranceMode` for semantics.
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
                // Fallback: re-solve at tight tolerance. Codex review-1: trigger
                // on ANY non-success status, not just `DivergedSlp`.
                solver::slp_solve_with_axis_jerk(&bundle, &arc_grid, limits, 1e-8)
                    .map_err(|e| ScheduleError::SolverSetup(format!("{e}")))?
            }
        }
    };

    // Stage 4: verify.
    let verify_report = verify::check(&arc_grid, &solver_result, limits, bundle.h);

    // Stage 5: assemble.
    Ok(output::assemble(
        &arc_grid,
        &solver_result,
        &verify_report,
        *grid,
        slp_outcome,
    ))
}

/// Per Codex review-1: `is_success` check uses solver-internal `SolverStatus`
/// (and `SlpOutcome`), not the public `SolveStatus`. The two differ — public
/// `SolveStatus` is set later by `output::assemble`.
fn solver_outcome_is_success(result: &solver::SolverResult, outcome: &solver::SlpOutcome) -> bool {
    let status_ok = matches!(
        result.status,
        solver::SolverStatus::Solved | solver::SolverStatus::SolvedInexact { .. }
    );
    // Per Codex review-1: any non-success outcome triggers fallback. Update
    // this match if `SlpOutcome` adds new success variants.
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
mod tests {
    use super::*;

    #[test]
    fn schedule_segment_straight_line_returns_profile() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
            None,
        )
        .unwrap();
        let limits = Limits {
            v_max: [500.0, 500.0, 500.0],
            a_max: [5_000.0, 5_000.0, 5_000.0],
            j_max: [100_000.0, 100_000.0, 100_000.0],
            a_centripetal_max: 2_500.0,
        };
        let cfg = GridConfig {
            scheme: crate::GridScheme::UniformArclength,
            n: 50,
        };
        let profile = schedule_segment(&curve, &limits, &cfg, 0.0, 0.0)
            .expect("schedule_segment should succeed");
        assert_eq!(profile.samples.len(), 50);
        assert!(matches!(
            profile.status,
            crate::SolveStatus::Solved | crate::SolveStatus::SolvedInexact { .. }
        ));
        // Endpoints zero-velocity, midpoint nontrivial.
        assert!(profile.samples[0].v < 1e-3);
        assert!(profile.samples[49].v < 1e-3);
        assert!(profile.samples[25].v > 100.0); // ≥ 100 mm/s
        // Total time should be finite and positive.
        assert!(profile.total_time.is_finite() && profile.total_time > 0.0);
    }
}
