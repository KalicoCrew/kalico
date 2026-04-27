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
//! | `RotatedSecondOrder`     | (not emitted by `build()`) |
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
//! | `InsufficientProgress`           | `SolverStatus::MaxIter{..}`        |
//! | `CallbackTerminated`             | `SolverStatus::Infeasible`         |
//! | `Unsolved`                       | `SolverStatus::Infeasible`         |
//!
//! `InsufficientProgress` maps to `MaxIter` (closer to "gave up" than
//! "structurally infeasible"). Spec §4.2.

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
// removed in Task 8 when schedule_segment is wired
#[allow(dead_code)]
#[derive(Debug, Clone)]
pub(crate) struct SolverResult {
    /// Solved primal `b_i = ṡ²` per grid point.
    pub b: Vec<f64>,
    /// Solved auxiliary `a_i` per grid point (path acceleration `s̈_i`).
    pub a: Vec<f64>,
    /// Solver status, mapped to a kalico-defined enum (no Clarabel types).
    pub status: SolverStatus,
}

/// Kalico-internal solver status. No Clarabel types.
// removed in Task 8 when schedule_segment is wired
#[allow(dead_code)]
#[derive(Debug, Clone, Copy)]
pub(crate) enum SolverStatus {
    Solved,
    SolvedInexact { residual: f64 },
    Infeasible,
    /// Renamed from `last_residual` to `residual` for consistency with
    /// `SolvedInexact`; both encode `max(r_prim, r_dual)`.
    MaxIter { residual: f64 },
}

/// Error from solver setup (invalid bundle). Not a solver runtime infeasibility.
// removed in Task 8 when schedule_segment is wired
#[allow(dead_code)]
#[derive(Debug, thiserror::Error)]
pub(crate) enum SolverSetupError {
    #[error("invalid constraint bundle: {0}")]
    InvalidBundle(String),
}

// ---------------------------------------------------------------------------
// Helper: build CSC matrix A for Clarabel
// ---------------------------------------------------------------------------

/// Convert the dense row-major `ConstraintBundle::a_rows` into a
/// Clarabel-format `CscMatrix`.
///
/// Sign convention: Clarabel wants `A_clarabel = -A_k` (see module-level
/// note). Every non-zero entry is negated here.
fn build_a_csc(bundle: &ConstraintBundle) -> CscMatrix<f64> {
    let n_vars = bundle.n_vars;
    let n_rows = bundle.a_rows.len();

    let mut colptr: Vec<usize> = Vec::with_capacity(n_vars + 1);
    let mut rowval: Vec<usize> = Vec::new();
    let mut nzval: Vec<f64> = Vec::new();

    colptr.push(0);
    for col in 0..n_vars {
        for row in 0..n_rows {
            let v = bundle.a_rows[row][col];
            if v != 0.0 {
                rowval.push(row);
                // Sign-convention negation: A_clarabel = -A_k (see module-level note).
                nzval.push(-v);
            }
        }
        colptr.push(nzval.len());
    }

    CscMatrix { m: n_rows, n: n_vars, colptr, rowval, nzval }
}

// ---------------------------------------------------------------------------
// Helper: zero P matrix (pure linear objective)
// ---------------------------------------------------------------------------

/// Build the zero `n_vars × n_vars` upper-triangle CSC matrix for
/// Clarabel's quadratic objective term.
///
/// For a pure linear objective there is no quadratic term. CSC encoding of
/// the zero matrix: `colptr = [0; n_vars+1]`, `rowval = []`, `nzval = []`.
fn build_p_zero(n_vars: usize) -> CscMatrix<f64> {
    CscMatrix::<f64> {
        m: n_vars,
        n: n_vars,
        colptr: vec![0usize; n_vars + 1],
        rowval: vec![],
        nzval: vec![],
    }
}

// ---------------------------------------------------------------------------
// Helper: map kalico cones → Clarabel SupportedConeT
// ---------------------------------------------------------------------------

/// Convert each kalico `Cone` to the matching Clarabel cone.
///
/// Returns `SolverSetupError` if a `RotatedSecondOrder` cone is encountered
/// (not supported in Clarabel 0.11; `build()` should never emit it).
fn map_clarabel_cones(
    bundle: &ConstraintBundle,
) -> Result<Vec<clarabel::solver::SupportedConeT<f64>>, SolverSetupError> {
    let mut out = Vec::with_capacity(bundle.cones.len());
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
        out.push(c);
    }
    Ok(out)
}

// ---------------------------------------------------------------------------
// Helper: map Clarabel SolverStatus → kalico SolverStatus
// ---------------------------------------------------------------------------

/// Map a Clarabel `SolverStatus` to the kalico-internal `SolverStatus`.
///
/// `residual` is `max(r_prim, r_dual)` from the solution, passed through for
/// inexact / max-iter outcomes.
///
/// The match is exhaustive against Clarabel 0.11.1's enum so that a future
/// Clarabel bump that adds a new variant will fail to compile — intentional.
fn map_status(status: ClarabelStatus, residual: f64) -> SolverStatus {
    match status {
        // Clean solve.
        ClarabelStatus::Solved => SolverStatus::Solved,

        // Near-optimal: feasible but residuals above tolerance.
        ClarabelStatus::AlmostSolved => SolverStatus::SolvedInexact { residual },

        // Iteration / time budget exhausted — no certificate either way.
        // InsufficientProgress is also "gave up" rather than "infeasible",
        // so it maps here rather than to Infeasible.
        ClarabelStatus::MaxIterations
        | ClarabelStatus::MaxTime
        | ClarabelStatus::InsufficientProgress => SolverStatus::MaxIter { residual },

        // Structural infeasibility certificates, solver errors, user-aborted,
        // and never-ran all map to Infeasible — no usable primal solution.
        ClarabelStatus::PrimalInfeasible
        | ClarabelStatus::DualInfeasible
        | ClarabelStatus::AlmostPrimalInfeasible
        | ClarabelStatus::AlmostDualInfeasible
        | ClarabelStatus::NumericalError
        | ClarabelStatus::CallbackTerminated
        | ClarabelStatus::Unsolved => SolverStatus::Infeasible,
    }
}

// ---------------------------------------------------------------------------
// Helper: extract b and a from solution vector
// ---------------------------------------------------------------------------

/// Slice the Clarabel primal solution `x` into per-grid-point `b` and `a`
/// vectors.
///
/// Variable layout (pinned in `constraints.rs`):
///   - `x[0..n_grid]`          → `b_i = ṡ²`
///   - `x[n_grid..2*n_grid]`   → `a_i = s̈_i`
fn extract_solution(x: &[f64], n_grid: usize, status: SolverStatus) -> SolverResult {
    let b: Vec<f64> = x[..n_grid].to_vec();
    let a: Vec<f64> = x[n_grid..2 * n_grid].to_vec();
    SolverResult { b, a, status }
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

/// Construct the Clarabel SOCP from `bundle` and solve it.
///
/// Returns `SolverResult` for both feasible and infeasible outcomes (the
/// solver runtime infeasibility surfaces in `SolverResult::status`).
/// `SolverSetupError` is only for programming errors caught before `solve()`.
// removed in Task 8 when schedule_segment is wired
#[allow(dead_code)]
pub(crate) fn solve(bundle: &ConstraintBundle) -> Result<SolverResult, SolverSetupError> {
    let n_vars = bundle.n_vars;
    let n_grid = bundle.n_grid;

    // Step 1: map cones.
    let cones_clarabel = map_clarabel_cones(bundle)?;

    // Step 2: build sparse A (with sign-convention negation applied inside).
    let a_csc = build_a_csc(bundle);

    // Step 3: RHS vector (sign-convention: unchanged from bundle).
    let b_rhs: &[f64] = &bundle.b_rhs;

    // Step 4: zero P for pure linear objective.
    let p_zero = build_p_zero(n_vars);

    // Step 5: objective vector.
    let q: &[f64] = &bundle.objective;

    // Step 6: settings — spec §4.2: Clarabel defaults except verbose = false
    // (suppress solver output; diagnostics go through kalico telemetry).
    let settings = DefaultSettings::<f64> { verbose: false, ..Default::default() };

    // Step 7: construct and run.
    let mut solver =
        DefaultSolver::new(&p_zero, q, &a_csc, b_rhs, &cones_clarabel, settings)
            .map_err(|e| SolverSetupError::InvalidBundle(e.to_string()))?;

    solver.solve();

    // Step 8: map status; residual = max(r_prim, r_dual).
    let soln = &solver.solution;
    let residual = soln.r_prim.max(soln.r_dual);
    let status = map_status(soln.status, residual);

    // Step 9: extract b_i and a_i.
    Ok(extract_solution(&soln.x, n_grid, status))
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
            BuildOutcome::Boundary(b) => panic!("expected Ok, got Boundary({b:?})"),
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

        // Endpoints clamped to zero (v_start = v_end = 0).
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

        // For length=100mm, zero endpoints, v_max=500 mm/s, a_max=5000 mm/s²:
        //   - If accel-bound throughout: b_max ≈ 2·a·s_peak where s_peak = 50mm,
        //     so b_max ≈ 2·5000·50 = 500_000 (mm/s)².
        //   - If velocity-bound: b_max = v_max² = 250_000 (mm/s)².
        //   - Actual answer is min of the two regimes.
        // Bracket the midpoint: must be substantially > 0 (not just barely-feasible)
        // and below v_max² (the global cap).
        let b_mid = result.b[25];
        assert!(
            b_mid > 1e4,
            "b[25] = {b_mid}, expected > 1e4 (substantially accelerating)"
        );
        assert!(
            b_mid <= 250_000.0 * 1.01,
            "b[25] = {b_mid}, expected ≤ v_max² + tolerance"
        );

        // Sign check: from rest, the path must be ACCELERATING in the first half
        // and DECELERATING in the second half. A sign-flip in the constraint
        // matrix could produce a profile where b is monotonically increasing or
        // decreasing, which we'd miss without these checks.
        assert!(
            result.b[10] > result.b[1],
            "must accelerate from rest: b[1]={}, b[10]={}",
            result.b[1],
            result.b[10]
        );
        assert!(
            result.b[40] < result.b[25],
            "must decelerate toward end: b[25]={}, b[40]={}",
            result.b[25],
            result.b[40]
        );

        // Path acceleration sign: a > 0 in first half, a < 0 in second.
        assert!(
            result.a[5] > 0.0,
            "a[5] = {} should be positive (accelerating)",
            result.a[5]
        );
        assert!(
            result.a[44] < 0.0,
            "a[44] = {} should be negative (decelerating)",
            result.a[44]
        );
    }
}
