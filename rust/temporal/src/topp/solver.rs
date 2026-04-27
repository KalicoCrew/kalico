//! Clarabel SOCP construction + solve. INTERNAL — Clarabel types do not
//! escape this module per spec §1.1 / §2.3.
//!
//! # Sign-convention note
//!
//! Clarabel solves: `Ax + s = b`, `s ∈ K`
//! The "point in K" is the slack `s = b - Ax`.
//!
//! The kalico `ConstraintBundle` uses the convention: `A_k * x + b_rhs ∈ K`
//! where the "point in K" is `A_k * x + b_rhs`.
//!
//! These are equal when `A_clarabel = -A_k` and `b_clarabel = b_rhs`:
//!   `s = b_rhs - (-A_k)*x = b_rhs + A_k*x = A_k*x + b_rhs` ∈ K  ✓
//!
//! This negation is applied uniformly to every row of A regardless of cone type.
//!
//! # Cone mapping (kalico → Clarabel 0.11)
//!
//! | kalico `Cone`            | Clarabel `SupportedConeT`  |
//! |--------------------------|---------------------------|
//! | `Zero`                   | `ZeroConeT(dim)`           |
//! | `Nonneg`                 | `NonnegativeConeT(dim)`    |
//! | `SecondOrder`            | `SecondOrderConeT(dim)`    |
//! | `RotatedSecondOrder`     | (not emitted by build())   |
//!
//! Note: `RotatedSecondOrderConeT` does not exist in Clarabel 0.11. The
//! `constraints::build()` function never emits `Cone::RotatedSecondOrder`;
//! it encodes jerk constraints using the norm-form identity
//! `z² ≤ u·v ↔ ||(2z, u-v)|| ≤ u+v` (standard SOC). The enum variant exists
//! for spec completeness but `solve()` will return `SolverSetupError` if
//! a bundle somehow contains it.
//!
//! # Clarabel `SolverStatus` → kalico `SolverStatus`
//!
//! | Clarabel                         | kalico                             |
//! |----------------------------------|------------------------------------|
//! | `Solved`                         | `SolverStatus::Solved`             |
//! | `AlmostSolved`                   | `SolverStatus::SolvedInexact{..}`  |
//! | `PrimalInfeasible`               | `SolverStatus::Infeasible`         |
//! | `DualInfeasible`                 | `SolverStatus::Infeasible`         |
//! | `AlmostPrimalInfeasible`         | `SolverStatus::Infeasible`         |
//! | `AlmostDualInfeasible`           | `SolverStatus::Infeasible`         |
//! | `MaxIterations`                  | `SolverStatus::MaxIter{..}`        |
//! | `MaxTime`                        | `SolverStatus::MaxIter{..}`        |
//! | `NumericalError`                 | `SolverStatus::Infeasible`         |
//! | `InsufficientProgress`           | `SolverStatus::Infeasible`         |
//! | `CallbackTerminated` / `Unsolved`| `SolverStatus::Infeasible`         |
//!
//! Spec §4.2.

use clarabel::algebra::CscMatrix;
use clarabel::solver::{
    DefaultSettings, DefaultSolver, IPSolver, SolverStatus as ClarabelStatus,
    SupportedConeT::{NonnegativeConeT, SecondOrderConeT, ZeroConeT},
};

use crate::topp::constraints::{Cone, ConstraintBundle};

// ---------------------------------------------------------------------------
// Public(crate) types
// ---------------------------------------------------------------------------

/// Result of a successful SOCP solve.
#[derive(Debug, Clone)]
pub(crate) struct SolverResult {
    /// Solved primal `b_i = ṡ²` per grid point.
    pub b: Vec<f64>,
    /// Solved auxiliary `a_i` per grid point (path acceleration s̈_i).
    pub a: Vec<f64>,
    /// Solver status, mapped to a kalico-defined enum (no Clarabel types).
    pub status: SolverStatus,
}

/// Kalico-internal solver status. No Clarabel types.
#[derive(Debug, Clone, Copy)]
pub(crate) enum SolverStatus {
    Solved,
    SolvedInexact { residual: f64 },
    Infeasible,
    MaxIter { last_residual: f64 },
}

/// Error from solver setup (invalid bundle). Not a solver runtime infeasibility.
#[derive(Debug, thiserror::Error)]
pub(crate) enum SolverSetupError {
    #[error("invalid constraint bundle: {0}")]
    InvalidBundle(String),
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Construct the Clarabel SOCP from `bundle` and solve it.
///
/// Returns `SolverResult` for both feasible and infeasible outcomes (the
/// solver runtime infeasibility surfaces in `SolverResult::status`).
/// `SolverSetupError` is only for programming errors caught before `solve()`.
pub(crate) fn solve(bundle: &ConstraintBundle) -> Result<SolverResult, SolverSetupError> {
    let n_vars = bundle.n_vars;
    let n_grid = bundle.n_grid;

    // -----------------------------------------------------------------------
    // Step 1: Convert kalico cones → Clarabel SupportedConeT.
    //
    // Clarabel 0.11 does not have RotatedSecondOrderConeT; constraints::build()
    // never emits it (jerk blocks use standard SOC via norm-form identity).
    // We return SolverSetupError if a bundle somehow contains it.
    // -----------------------------------------------------------------------
    let mut cones_clarabel = Vec::with_capacity(bundle.cones.len());
    for &(ref cone, dim) in &bundle.cones {
        let c = match cone {
            Cone::Zero => ZeroConeT(dim),
            Cone::Nonneg => NonnegativeConeT(dim),
            Cone::SecondOrder => SecondOrderConeT(dim),
            Cone::RotatedSecondOrder => {
                return Err(SolverSetupError::InvalidBundle(
                    "RotatedSecondOrderConeT is not supported in Clarabel 0.11; \
                     constraints::build() should never emit it"
                        .to_owned(),
                ));
            }
        };
        cones_clarabel.push(c);
    }

    // -----------------------------------------------------------------------
    // Step 2: Convert dense A (row-major Vec<Vec<f64>>) → Clarabel CscMatrix.
    //
    // Sign convention: Clarabel wants A_c = -A_k (see module-level note).
    // We negate every entry when building column-major CSC.
    //
    // Bundle stores rows; Clarabel wants CSC (column-major).
    // We iterate columns c = 0..n_vars and accumulate non-zeros from each row.
    // -----------------------------------------------------------------------
    let n_rows = bundle.a_rows.len();

    // Build CSC: colptr, rowval, nzval — column by column.
    let mut colptr: Vec<usize> = Vec::with_capacity(n_vars + 1);
    let mut rowval: Vec<usize> = Vec::new();
    let mut nzval: Vec<f64> = Vec::new();

    colptr.push(0);
    for col in 0..n_vars {
        for row in 0..n_rows {
            let v = bundle.a_rows[row][col];
            if v != 0.0 {
                rowval.push(row);
                // spec §sign-convention: negate A rows
                nzval.push(-v);
            }
        }
        colptr.push(nzval.len());
    }

    let a_csc = CscMatrix {
        m: n_rows,
        n: n_vars,
        colptr,
        rowval,
        nzval,
    };

    // -----------------------------------------------------------------------
    // Step 3: RHS vector b_clarabel = bundle.b_rhs (sign-convention: unchanged).
    // -----------------------------------------------------------------------
    let b_rhs: &[f64] = &bundle.b_rhs;

    // -----------------------------------------------------------------------
    // Step 4: Zero P (pure linear objective, no quadratic term).
    //
    // Clarabel's P is n_vars × n_vars upper triangle. For a linear problem
    // P is the zero matrix, which CSC encodes as colptr = [0, 0, ..., 0]
    // (all n_vars+1 entries zero), rowval = [], nzval = [].
    // -----------------------------------------------------------------------
    let p_zero = CscMatrix::<f64> {
        m: n_vars,
        n: n_vars,
        colptr: vec![0usize; n_vars + 1],
        rowval: vec![],
        nzval: vec![],
    };

    // -----------------------------------------------------------------------
    // Step 5: Objective vector q = bundle.objective.
    // -----------------------------------------------------------------------
    let q: &[f64] = &bundle.objective;

    // -----------------------------------------------------------------------
    // Step 6: Settings.
    //
    // spec §4.2: Clarabel defaults except verbose = false (suppress solver
    // output in test and production runs; diagnostics go through kalico
    // telemetry, not stdout).
    // -----------------------------------------------------------------------
    let mut settings = DefaultSettings::<f64>::default();
    // spec §4.2: suppress Clarabel's banner/iteration output
    settings.verbose = false;

    // -----------------------------------------------------------------------
    // Step 7: Construct and run.
    // -----------------------------------------------------------------------
    let mut solver =
        DefaultSolver::new(&p_zero, q, &a_csc, b_rhs, &cones_clarabel, settings)
            .map_err(|e| SolverSetupError::InvalidBundle(e.to_string()))?;

    solver.solve();

    // -----------------------------------------------------------------------
    // Step 8: Map Clarabel SolverStatus → kalico SolverStatus.
    //
    // Residual for inexact / max-iter: use max(r_prim, r_dual) from solution.
    // -----------------------------------------------------------------------
    let soln = &solver.solution;
    let residual = soln.r_prim.max(soln.r_dual);

    let status = match soln.status {
        ClarabelStatus::Solved => SolverStatus::Solved,
        ClarabelStatus::AlmostSolved => SolverStatus::SolvedInexact { residual },
        ClarabelStatus::MaxIterations | ClarabelStatus::MaxTime => {
            SolverStatus::MaxIter { last_residual: residual }
        }
        // All infeasibility / error / progress codes map to Infeasible.
        // Callers check SolverStatus::Infeasible and surface as SolveStatus::Infeasible.
        _ => SolverStatus::Infeasible,
    };

    // -----------------------------------------------------------------------
    // Step 9: Extract b_i and a_i from solution.x per the variable layout
    // pinned in constraints.rs:
    //   indices [0, n_grid)      : b_i
    //   indices [n_grid, 2*n_grid): a_i
    // -----------------------------------------------------------------------
    let x = &soln.x;
    let b: Vec<f64> = x[..n_grid].to_vec();
    let a: Vec<f64> = x[n_grid..2 * n_grid].to_vec();

    Ok(SolverResult { b, a, status })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topp::constraints::{build, BuildOutcome, EndpointVelocities};
    use crate::topp::path::ArclengthGrid;
    use crate::Limits;

    fn dummy_straight_grid(n: usize, length: f64) -> ArclengthGrid {
        let s: Vec<f64> = (0..n).map(|i| length * i as f64 / (n - 1) as f64).collect();
        let u = s.clone();
        let c = s.iter().map(|si| [*si, 0.0, 0.0]).collect();
        let c_prime = vec![[1.0, 0.0, 0.0]; n];
        let c_double_prime = vec![[0.0, 0.0, 0.0]; n];
        let c_triple_prime = vec![[0.0, 0.0, 0.0]; n];
        let kappa = vec![0.0; n];
        ArclengthGrid {
            s,
            u,
            c,
            c_prime,
            c_double_prime,
            c_triple_prime,
            kappa,
            total_length: length,
        }
    }

    #[test]
    fn straight_line_solves_to_nontrivial_profile() {
        let grid = dummy_straight_grid(50, 100.0);
        let limits = Limits {
            v_max: [500.0, 500.0, 500.0],
            a_max: [5_000.0, 5_000.0, 5_000.0],
            j_max: [100_000.0, 100_000.0, 100_000.0],
            a_centripetal_max: 2_500.0,
        };
        let bundle = match build(
            &grid,
            &limits,
            EndpointVelocities { v_start: 0.0, v_end: 0.0 },
        ) {
            BuildOutcome::Ok(b) => b,
            other => panic!("expected Ok, got {other:?}"),
        };
        let result = solve(&bundle).expect("solver setup");
        assert!(
            matches!(
                result.status,
                SolverStatus::Solved | SolverStatus::SolvedInexact { .. }
            ),
            "expected Solved or SolvedInexact, got {:?}",
            result.status
        );
        assert_eq!(result.b.len(), 50);
        // Endpoints clamped to 0; interior must be > 0.
        assert!(
            result.b[0].abs() < 1e-6,
            "b[0] should be ~0, got {}",
            result.b[0]
        );
        assert!(
            result.b[49].abs() < 1e-6,
            "b[49] should be ~0, got {}",
            result.b[49]
        );
        assert!(
            result.b[25] > 1.0,
            "b[25] should be > 1.0 (nontrivial speed), got {}",
            result.b[25]
        );
    }
}
