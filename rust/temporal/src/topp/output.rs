//! Profile assembly: solver output + verifier report → public `TopProfile`.
//!
//! Spec §4.3 stage 5, §4.4.

use crate::topp::path::ArclengthGrid;
use crate::topp::solver::{SlpOutcome, SolverResult, SolverStatus};
use crate::topp::verify::{self, VerifyReport};
use crate::{
    GridConfig, GridSample, InfeasibleReason, SolveStatus, TopProfile,
};

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

    // Trapezoidal time integral: T = Σ Δs_i · 2 / (v_i + v_{i+1}).
    let mut total_time = 0.0;
    for i in 0..n - 1 {
        let ds = grid.s[i + 1] - grid.s[i];
        let v_sum = samples[i].v + samples[i + 1].v;
        if v_sum > 1e-12 {
            total_time += ds * 2.0 / v_sum;
        } else {
            // Defensive: shouldn't happen mid-profile for feasible solutions.
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

/// Convert internal solver status into public `SolveStatus`. Carries `verify`
/// so we can override Clarabel-success with feasibility-failure (relaxation
/// tightness gap, per spec §7.1) and `slp_outcome` so SLP-converged /
/// diverged / max-iter cases get distinct public statuses (spec §11).
pub(crate) fn map_status(
    solver_status: SolverStatus,
    verify: &VerifyReport,
    slp_outcome: SlpOutcome,
) -> SolveStatus {
    // First decide based on the inner solver. SLP-driven statuses can only
    // override a feasible-looking inner outcome.
    let base = match solver_status {
        SolverStatus::Solved if verify.feasible => SolveStatus::Solved,
        SolverStatus::SolvedInexact { residual } if verify.feasible => {
            SolveStatus::SolvedInexact { residual }
        }
        SolverStatus::Solved | SolverStatus::SolvedInexact { .. } => {
            // Inner solver succeeded but verifier disagrees: relaxation-gap.
            SolveStatus::Infeasible {
                at_grid: verify.worst_violation_grid,
                reason: InfeasibleReason::SolverInfeasible,
            }
        }
        SolverStatus::Infeasible => SolveStatus::Infeasible {
            at_grid: 0,
            reason: InfeasibleReason::SolverInfeasible,
        },
        SolverStatus::MaxIter { residual } => {
            // Per spec §6.2, verify::check accepts at EPS_FEAS=1e-3. If
            // Clarabel terminates with residual below verifier tolerance, the
            // iterate IS feasible by our standard — promote MaxIter→SolvedInexact
            // rather than fail. Discovered during fixture_7 N=200
            // InsufficientProgress investigation; see CLAUDE.md plan-changes-log.
            if residual < verify::EPS_FEAS && verify.feasible {
                SolveStatus::SolvedInexact { residual }
            } else {
                SolveStatus::MaxIter { last_residual: residual }
            }
        }
    };

    // SLP outcome refines the public status when it carries useful additional
    // information that isn't already captured by the inner solver / verifier.
    match (slp_outcome, &base) {
        // SLP cuts were required and converged. Promote a feasible inner
        // outcome to `SolvedSlp{outer_iters}`; verifier-rejected inner
        // outcomes pass through as Infeasible (the SLP loop only adds
        // path-jerk cuts, so an axis-accel verifier veto is still a real
        // failure even when SLP terminates cleanly).
        (
            SlpOutcome::Converged { outer_iters },
            SolveStatus::Solved | SolveStatus::SolvedInexact { .. },
        ) if outer_iters > 0 => SolveStatus::SolvedSlp { outer_iters },
        (SlpOutcome::Diverged { last_max_ratio, outer_iters }, _) => {
            SolveStatus::DivergedSlp {
                last_max_ratio,
                outer_iters,
            }
        }
        (SlpOutcome::MaxIters { last_max_ratio }, _) => {
            // Symmetric with the Clarabel-MaxIter promotion at the inner-solver
            // match above: the SLP outer loop's `last_max_ratio` measures the
            // path-jerk RELAXATION gap (Lee 2024 conservative cut on
            // `|b''|·√b ≤ 2J`), not a direct feasibility residual. The
            // authoritative bar is `verify::check`, which evaluates per-axis
            // Cartesian jerk on the assembled time-domain trajectory. When the
            // SLP loop times out at a band-edge N (e.g. fixture_6 seg-9 at
            // N=80, where last_max_ratio plateaus at 1.13 while
            // verify.worst_violation sits at machine epsilon), the iterate IS
            // feasible at our standard — promote rather than fail. Same logic,
            // different inner solver. Carries the verifier's measured
            // violation as the residual to keep semantics consistent.
            if verify.feasible {
                SolveStatus::SolvedInexact { residual: verify.worst_violation }
            } else {
                SolveStatus::MaxIterSlp { last_max_ratio }
            }
        }
        // Iteration-0 convergence (no cuts), verifier-rejected SLP-converged
        // outcomes, and inner-solver failures all pass `base` through unchanged.
        (
            SlpOutcome::Converged { .. } | SlpOutcome::InnerSolverFailure,
            _,
        ) => base,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topp::solver::{SolverResult, SolverStatus};
    use crate::topp::verify::VerifyReport;
    use crate::{BindingConstraint, GridConfig, GridScheme};

    fn dummy_grid(n: usize, length: f64) -> ArclengthGrid {
        let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
        let u = s.clone();
        let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
        let c_prime = vec![[1.0, 0.0, 0.0]; n];
        let c_double_prime = vec![[0.0, 0.0, 0.0]; n];
        let c_triple_prime = vec![[0.0, 0.0, 0.0]; n];
        let kappa = vec![0.0; n];
        ArclengthGrid { s, u, c, c_prime, c_double_prime, c_triple_prime, kappa, total_length: length }
    }

    #[test]
    fn assembles_samples_and_total_time() {
        let grid = dummy_grid(3, 10.0);
        let result = SolverResult {
            b: vec![0.0, 100.0, 0.0],
            a: vec![10.0, 0.0, -10.0],
            status: SolverStatus::Solved,
        };
        let verify = VerifyReport {
            binding_per_grid: vec![
                BindingConstraint::Boundary,
                BindingConstraint::None,
                BindingConstraint::Boundary,
            ],
            worst_violation: 0.0,
            worst_violation_grid: 0,
            feasible: true,
        };
        let cfg = GridConfig { scheme: GridScheme::UniformArclength, n: 3 };
        let p = assemble(
            &grid,
            &result,
            &verify,
            cfg,
            SlpOutcome::Converged { outer_iters: 0 },
        );
        assert_eq!(p.samples.len(), 3);
        assert!((p.samples[1].v - 10.0).abs() < 1e-9);
        assert!(matches!(p.status, SolveStatus::Solved));
        // Trapezoidal time over the two intervals: 2·5/(0+10) + 2·5/(10+0) = 2.0 s.
        assert!((p.total_time - 2.0).abs() < 1e-9);
    }
}
