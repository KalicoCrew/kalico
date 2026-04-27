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

/// One linearized Taylor cut on `1/√b` produced by the SLP outer loop.
///
/// Encodes the constraint
///
/// ```text
/// |Δ²b_i| ≤ 2J·h² · (1/√b̄_i − (b_i − b̄_i)/(2·b̄_i^{3/2}))
/// ```
///
/// at iterate `b̄_i = b_bar`. Two `Nonneg` rows (positive and negative side)
/// are appended per cut. See `slp_solve` for the row-coefficient derivation.
#[derive(Debug, Clone, Copy)]
pub(crate) struct SlpCut {
    /// Interior grid index `i` (must satisfy `1 ≤ i ≤ N − 2`).
    pub i: usize,
    /// Iterate value `b̄_i` at which the linearization is taken.
    pub b_bar: f64,
}

// ---------------------------------------------------------------------------
// Public(crate) types
// ---------------------------------------------------------------------------

/// Result of a successful SOCP solve.
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
#[derive(Debug, thiserror::Error)]
pub(crate) enum SolverSetupError {
    #[error("invalid constraint bundle: {0}")]
    InvalidBundle(String),
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
/// Equivalent to `solve_with_cuts(bundle, &[])` — kept as the public(crate)
/// entry point for the unit-test surface (no cuts → original Consolini-Locatelli
/// SOCP).
#[allow(dead_code)]
pub(crate) fn solve(bundle: &ConstraintBundle) -> Result<SolverResult, SolverSetupError> {
    solve_with_cuts(bundle, &[])
}

/// Append one SLP cut as two `Nonneg` rows (positive- and negative-side) to
/// the Clarabel-format `A` and `b_rhs` accumulators. The current row count
/// `n_rows` is also updated.
///
/// # Cut algebra (sign-checked by hand)
///
/// At iterate `b̄`, the first-order Taylor expansion of `f(b) = 1/√b` is
///
/// ```text
/// f(b) ≈ 1/√b̄ − (b − b̄) / (2·b̄^{3/2}).
/// ```
///
/// `f` is convex on `b > 0` (`f''(b) = 3/(4·b^{5/2}) > 0`), so the tangent
/// lies BELOW the curve everywhere. The linearized cut
///
/// ```text
/// |Δ²b_i| ≤ 2J·h² · [1/√b̄_i − (b_i − b̄_i) / (2·b̄_i^{3/2})]
/// ```
///
/// is therefore TIGHTER than the original `|b''|·√b ≤ 2J` constraint:
/// satisfying the cut implies satisfying the original. This is the
/// convex-tangent-below-the-curve property Lee 2024 §III leverages.
///
/// Expanding the constant term: `2J·h² · [1/√b̄ + b̄/(2·b̄^{3/2})] = 3J·h²/√b̄`.
/// Let `α := J·h²/b̄^{3/2}`. The two rows in `A·x + b_rhs ≥ 0` form are:
///
/// ```text
/// (+):  3J·h²/√b̄  − α·b_i − (b_{i-1} − 2·b_i + b_{i+1})  ≥ 0
/// (−):  3J·h²/√b̄  − α·b_i + (b_{i-1} − 2·b_i + b_{i+1})  ≥ 0
/// ```
///
/// `b̄` MUST be positive; the SLP loop guarantees this by clamping with a
/// small floor before constructing each cut.
#[allow(clippy::too_many_arguments)] // CSC builder has many concurrent state vectors.
fn append_slp_cut_to_clarabel(
    cut: &SlpCut,
    j_path: f64,
    h: f64,
    n_rows: &mut usize,
    rowval: &mut [Vec<usize>],
    nzval: &mut [Vec<f64>],
    b_rhs: &mut Vec<f64>,
    n_grid: usize,
) {
    let b_bar = cut.b_bar;
    let i = cut.i;
    let sqrt_b = b_bar.sqrt();
    let alpha = j_path * h * h / (b_bar * sqrt_b); // = J·h² / b̄^{3/2}
    let rhs = 3.0 * j_path * h * h / sqrt_b;

    // Sign-convention negation: A_clarabel = -A_k. We build both signs of A_k
    // here and negate when pushing into nzval.
    //
    // Variable layout from constraints.rs: b_i at index i (off_b = 0).
    let bm1 = i - 1;
    let bi = i;
    let bp1 = i + 1;
    debug_assert!(bp1 < n_grid, "SLP cut interior index out of range");

    // Positive side: A_k row = [b_{i-1}: -1, b_i: -α + 2, b_{i+1}: -1], rhs = +rhs.
    let pos_row = *n_rows;
    push_nz(rowval, nzval, bm1, pos_row, -(-1.0)); // = +1
    push_nz(rowval, nzval, bi, pos_row, -(2.0 - alpha));
    push_nz(rowval, nzval, bp1, pos_row, -(-1.0)); // = +1
    b_rhs.push(rhs);
    *n_rows += 1;

    // Negative side: A_k row = [b_{i-1}: +1, b_i: -α - 2, b_{i+1}: +1], rhs = +rhs.
    let neg_row = *n_rows;
    push_nz(rowval, nzval, bm1, neg_row, -(1.0));
    push_nz(rowval, nzval, bi, neg_row, -(-alpha - 2.0));
    push_nz(rowval, nzval, bp1, neg_row, -(1.0));
    b_rhs.push(rhs);
    *n_rows += 1;
}

/// Helper: append a single non-zero entry into a column-bucketed CSC builder.
#[inline]
fn push_nz(rowval: &mut [Vec<usize>], nzval: &mut [Vec<f64>], col: usize, row: usize, v: f64) {
    if v != 0.0 {
        rowval[col].push(row);
        nzval[col].push(v);
    }
}

/// Construct the Clarabel SOCP from `bundle` and solve it, with optional SLP
/// cut rows appended to the constraint matrix.
///
/// `cuts`, when non-empty, becomes additional `Nonneg` rows of the SOCP — one
/// fresh `NonnegativeConeT` block of dim `2·cuts.len()` is added. The cuts
/// reference only `b_i` variables that already exist in the bundle; no new
/// variables are introduced (`n_vars` unchanged).
fn solve_with_cuts(
    bundle: &ConstraintBundle,
    cuts: &[SlpCut],
) -> Result<SolverResult, SolverSetupError> {
    let n_vars = bundle.n_vars;
    let n_grid = bundle.n_grid;

    // Step 1: cones — append a Nonneg block for SLP cuts (2 rows per cut).
    let mut cones_clarabel = map_clarabel_cones(bundle)?;
    if !cuts.is_empty() {
        cones_clarabel.push(NonnegativeConeT(2 * cuts.len()));
    }

    // Step 2: build sparse A column-bucketed (with sign-convention negation
    // applied inside). We construct rowval/nzval per column so we can append
    // SLP-cut rows incrementally without reshuffling.
    let mut rowval_per_col: Vec<Vec<usize>> = vec![Vec::new(); n_vars];
    let mut nzval_per_col: Vec<Vec<f64>> = vec![Vec::new(); n_vars];
    let mut n_rows = 0_usize;

    for row in &bundle.a_rows {
        for (col, &v) in row.iter().enumerate() {
            if v != 0.0 {
                rowval_per_col[col].push(n_rows);
                // Sign-convention negation: A_clarabel = -A_k.
                nzval_per_col[col].push(-v);
            }
        }
        n_rows += 1;
    }

    // Step 3: RHS vector (sign-convention: unchanged from bundle).
    let mut b_rhs: Vec<f64> = bundle.b_rhs.clone();

    // Step 3b: append SLP-cut rows (negation already baked into the helper).
    let j_path = bundle.j_path;
    let h = bundle.h;
    debug_assert!(j_path > 0.0 && h > 0.0, "bundle must carry positive j_path/h");
    for cut in cuts {
        let b_bar_floored = cut.b_bar.max(SLP_B_FLOOR);
        let cut = SlpCut { i: cut.i, b_bar: b_bar_floored };
        append_slp_cut_to_clarabel(
            &cut,
            j_path,
            h,
            &mut n_rows,
            &mut rowval_per_col,
            &mut nzval_per_col,
            &mut b_rhs,
            n_grid,
        );
    }

    // Step 4: assemble final CSC.
    let mut colptr: Vec<usize> = Vec::with_capacity(n_vars + 1);
    let mut rowval: Vec<usize> = Vec::new();
    let mut nzval: Vec<f64> = Vec::new();
    colptr.push(0);
    for col in 0..n_vars {
        rowval.extend_from_slice(&rowval_per_col[col]);
        nzval.extend_from_slice(&nzval_per_col[col]);
        colptr.push(nzval.len());
    }
    let a_csc = CscMatrix { m: n_rows, n: n_vars, colptr, rowval, nzval };

    // Step 5: zero P for pure linear objective.
    let p_zero = build_p_zero(n_vars);

    // Step 6: objective vector.
    let q: &[f64] = &bundle.objective;

    // Step 7: settings — spec §4.2: Clarabel defaults except verbose = false
    // (suppress solver output; diagnostics go through kalico telemetry).
    // SLP-cut SOCPs need more interior-point iterations than the base SOCP:
    // adding linearized cuts to the base relaxation tightens conditioning,
    // and the default 200-iter budget produces `InsufficientProgress` on
    // the empirical CL-2024 counterexample fixture. 1000 iters is enough
    // headroom; runtime is still ≤ 1 s per outer iteration on N=200 grids.
    let settings = DefaultSettings::<f64> {
        verbose: false,
        max_iter: 1000,
        ..Default::default()
    };

    // Step 8: construct and run.
    let mut solver =
        DefaultSolver::new(&p_zero, q, &a_csc, &b_rhs, &cones_clarabel, settings)
            .map_err(|e| SolverSetupError::InvalidBundle(e.to_string()))?;

    solver.solve();

    // Step 9: map status; residual = max(r_prim, r_dual).
    let soln = &solver.solution;
    let residual = soln.r_prim.max(soln.r_dual);
    let status = map_status(soln.status, residual);

    // Step 10: extract b_i and a_i.
    Ok(extract_solution(&soln.x, n_grid, status))
}

// ---------------------------------------------------------------------------
// SLP outer iteration (Lee 2024 §III–§IV) — fallback for the empirical
// CL-2024 Conjecture-4.1 counterexample on curved high-jerk-load segments.
//
// The Consolini-Locatelli SOCP relaxation is faithful but its tightness is
// conjectural and demonstrably loose on curved high-jerk-load segments
// (e.g. a R=20 mm rational-quadratic 90° arc at N=200, J=1e5 — see
// docs/research/jerk-constrained-socp-relaxation-tightness.md). The single
// non-convex constraint `|b''|·√b ≤ 2J` cannot be tightened inside one SOCP;
// the convex sublevel set `b·w² ≤ 1` is non-convex on the positive orthant
// (Hessian determinant -4w² < 0). Lee 2024's mitigation: solve the current
// SOCP, append a first-order Taylor cut on `1/√b` at the current iterate,
// re-solve. Each inner SOCP stays convex; the linearized cut lies below
// the original convex-down `1/√b` curve so it tightens (rather than
// loosens) the relaxation.
//
// # Cut placement: full-grid linearization vs. active-set
//
// Two natural cut-placement strategies were tried during implementation:
//
// 1. *Active-set* — cut only at currently violating grid points, drop
//    older cuts. Standard SLP. **Oscillated** on the empirical fixture:
//    cuts at one set of indices push the iterate to a configuration that
//    violates a different set, ad infinitum.
// 2. *Active-set with cumulative cuts* — accumulate cuts at every violator
//    seen so far. Eventually wrecks Clarabel's interior-point conditioning
//    after ~50 rows on N=200 grids; produces `InsufficientProgress`.
//
// What works: **full-grid linearization** — every iteration, drop the
// prior cut set and rebuild fresh cuts at *every interior grid point*
// (with `b̄ ≥ SLP_B_CUT_FLOOR`) using the latest iterate. Each cut is a
// valid global underestimator of `1/√b` by convexity, so the joint cut
// set is a tight envelope around the current iterate. Empirically
// converges in 1–3 outer iterations on the kalico fixtures; row count
// stays bounded at `N − 2` (cuts replace each iteration, not accumulate).
// ---------------------------------------------------------------------------

/// Maximum SLP outer iterations before declaring `MaxIterSlp`. Hard cap to
/// guard against pathological no-convergence inputs; Lee 2024 reports ~5–30
/// iterations in practice.
const SLP_MAX_OUTER_ITERS: u32 = 50;

/// Feasibility tolerance for the path-jerk violation predicate. Looser
/// than `verify::EPS_FEAS` (spec §6.2) because the SLP predicate uses a
/// finite-difference estimate of `b''(s)` directly (`Δ²b/h²`), which is
/// noisy around constraint-switch grid points on straight-line fixtures
/// (1–2% spurious "violations" at the fade-in/fade-out kinks). The CL-2024
/// Conjecture-4.1 counterexample fixture violates by ~143% at the worst
/// grid point (ratio 2.43); 5% is tight enough to catch real gaps and
/// loose enough to skip discretization noise. The post-solve verifier
/// uses the time-domain per-axis Cartesian jerk and remains the
/// authoritative feasibility check at `EPS_FEAS = 1e-3`.
const SLP_EPS_FEAS: f64 = 5e-2;

/// Floor on `b̄` used when constructing a cut at a grid point with very
/// small primal `b`. Avoids `1/√0` in the linearization. The floor is
/// physically irrelevant (cut is then trivially satisfied at the iterate
/// since the violation predicate requires `b > 0` to be non-trivial).
const SLP_B_FLOOR: f64 = 1.0;

/// Threshold on `b̄` below which a violator does NOT receive a cut: the
/// linearization coefficient `α = J·h²/b̄^{3/2}` grows like `b̄⁻³ᐟ²`, so
/// cuts at near-boundary grid points (where Clarabel's primal tends to
/// have small numerical artifacts) inject very steep rows that wreck the
/// next inner SOCP's conditioning. The path-jerk constraint at small `b`
/// is dominated by the boundary equality / centripetal cap anyway, so the
/// missing cut is safely picked up by the main relaxation. Tuned against
/// the R=20 mm 90° rational-quadratic arc fixture; `B_CUT_FLOOR ≈ (10
/// mm/s)²`.
const SLP_B_CUT_FLOOR: f64 = 100.0;

/// Warm-up window (in iterations) before the divergence rule fires. Allows
/// the loop to add cuts from a cold start; SLP iterates routinely bounce
/// around the true optimum for several iterations before settling (Lee 2024
/// reports 5–30 iterations to converge in practice).
const SLP_WARMUP_ITERS: u32 = 8;

/// Required relative improvement in best-so-far max-violator ratio over the
/// trailing `SLP_NO_IMPROVEMENT_WINDOW` iterations. If best-so-far hasn't
/// dropped by ≥ this fraction across the window, the loop is declared
/// diverged. Empirically: SLP can have non-monotone iterate-by-iterate
/// behavior (cuts at one violator unmask different violators on the next
/// iterate), so the only reliable signal is best-so-far progress.
const SLP_MIN_IMPROVEMENT: f64 = 0.01;

/// Sliding-window length (in iterations) for the no-improvement divergence
/// rule. After warm-up, the loop tracks `best_ratio[k] − best_ratio[k − W]`
/// and declares divergence if it falls below `SLP_MIN_IMPROVEMENT`.
const SLP_NO_IMPROVEMENT_WINDOW: usize = 10;

/// Outcome of the SLP outer iteration. Carried back to `schedule_segment`
/// where it is mapped onto the public `SolveStatus` enum.
#[derive(Debug, Clone, Copy)]
pub(crate) enum SlpOutcome {
    /// Inner SOCP converged with no violators (within `SLP_EPS_FEAS`).
    /// `outer_iters = 0` corresponds to the original SOCP — no cuts were
    /// needed; this is the common straight-line case.
    Converged { outer_iters: u32 },
    /// Loop hit `SLP_MAX_OUTER_ITERS` without driving the violator ratio
    /// below `1 + ε_feas`.
    MaxIters { last_max_ratio: f64 },
    /// `last_max_ratio` failed to drop monotonically across the warm-up
    /// window — declared diverged.
    Diverged {
        last_max_ratio: f64,
        outer_iters: u32,
    },
    /// Inner SOCP returned a non-feasible status (Infeasible or `MaxIter`
    /// from Clarabel itself). Surface that without further SLP iteration.
    InnerSolverFailure,
}

/// Run the SLP outer-iteration loop. Returns the final `SolverResult` plus
/// an `SlpOutcome` describing how the loop terminated.
///
/// On entry, no cuts have been added; iteration 0 solves the original
/// Consolini-Locatelli SOCP. If the iteration-0 primal is path-jerk-feasible
/// within `SLP_EPS_FEAS`, the loop returns immediately with
/// `SlpOutcome::Converged { outer_iters: 0 }` — i.e. straight-line and other
/// non-counterexample inputs see no SLP overhead. Otherwise each subsequent
/// iteration replaces the cut set with fresh full-grid linearizations of
/// `1/√b` at the latest iterate (see module-level comment for the
/// full-grid-vs-active-set rationale) and re-solves.
pub(crate) fn slp_solve(
    bundle: &ConstraintBundle,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let h = bundle.h;
    let j_path = bundle.j_path;
    debug_assert!(h > 0.0 && j_path > 0.0);

    let mut cuts: Vec<SlpCut> = Vec::new();
    let mut last_result = solve_with_cuts(bundle, &cuts)?;

    // Iteration-0: structurally-infeasible or solver-MaxIter at iter 0 means
    // there's no usable primal to scan; surface that without iterating.
    if matches!(
        last_result.status,
        SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
    ) {
        return Ok((last_result, SlpOutcome::InnerSolverFailure));
    }

    // Iteration-0 violator scan.
    let mut violators = find_jerk_violators(&last_result.b, h, j_path);
    if violators.is_empty() {
        return Ok((last_result, SlpOutcome::Converged { outer_iters: 0 }));
    }

    // Track the best iterate seen so far (lowest max-violator ratio). When
    // the SLP loop fails to converge, this is the iterate we surface so the
    // caller has the most-feasible primal even if the loop terminated badly.
    let mut best_result = last_result.clone();
    let mut best_ratio_so_far = max_ratio(&violators);

    // Track the per-iteration max-violator ratio AND the running best-so-far.
    // Best-so-far is the divergence signal: SLP cuts at one grid point can
    // unmask new violators elsewhere on the next iterate, so iterate-by-iterate
    // monotone descent is too strict. Best-so-far across a sliding window is
    // the conservative-but-meaningful progress metric.
    let mut max_ratio_history: Vec<f64> = Vec::new();
    let mut best_ratio_history: Vec<f64> = Vec::new();
    let initial_max = max_ratio(&violators);
    max_ratio_history.push(initial_max);
    best_ratio_history.push(initial_max);
    for outer in 1..=SLP_MAX_OUTER_ITERS {
        // Hybrid cut strategy: keep cuts at ALL interior grid points (not
        // just current violators), re-linearizing at the current iterate
        // each pass. Block (h)'s SOC chain is convex but loose (the CL-2024
        // Conjecture-4.1 gap); the cuts give a tighter local approximation
        // of `|b''|·√b ≤ 2J` valid at this iterate, valid bound everywhere
        // by convexity. Re-linearizing each pass avoids the iterate-by-
        // iterate oscillation that pure-active-set SLP exhibits on this
        // fixture; coverage across the whole grid prevents the relaxation
        // from finding new violators outside the prior active set.
        cuts.clear();
        let mut added = 0_usize;
        let n = last_result.b.len();
        for i in 1..n - 1 {
            let b_bar = last_result.b[i];
            if b_bar < SLP_B_CUT_FLOOR {
                continue;
            }
            cuts.push(SlpCut { i, b_bar });
            added += 1;
        }
        if added == 0 {
            // All remaining violators are below the cut floor; they're
            // dominated by other constraints in the relaxation. Surface the
            // best-so-far iterate with a `MaxIters` outcome so the public
            // API carries the residual.
            return Ok((
                best_result,
                SlpOutcome::MaxIters {
                    last_max_ratio: best_ratio_so_far,
                },
            ));
        }

        let new_result = solve_with_cuts(bundle, &cuts)?;
        // Structural infeasibility / Clarabel MaxIter on the inner solve:
        // the new primal is not trustworthy as an iterate. Stop iterating
        // and surface the best-so-far iterate with a `MaxIters` outcome so
        // downstream consumers see the most-feasible primal we found.
        if matches!(
            new_result.status,
            SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
        ) {
            return Ok((
                best_result,
                SlpOutcome::MaxIters {
                    last_max_ratio: best_ratio_so_far,
                },
            ));
        }
        last_result = new_result;

        violators = find_jerk_violators(&last_result.b, h, j_path);
        if violators.is_empty() {
            return Ok((last_result, SlpOutcome::Converged { outer_iters: outer }));
        }

        let cur_max = max_ratio(&violators);
        max_ratio_history.push(cur_max);
        let prev_best = *best_ratio_history.last().unwrap_or(&f64::INFINITY);
        let cur_best = prev_best.min(cur_max);
        best_ratio_history.push(cur_best);
        if cur_max < best_ratio_so_far {
            best_ratio_so_far = cur_max;
            best_result = last_result.clone();
        }
        let _ = cur_best; // tracking only; surfaced via SlpOutcome.

        // Divergence detection (verifier-recommended "no-improvement" rule).
        // After the warm-up window, require best-so-far ratio to have dropped
        // by ≥ SLP_MIN_IMPROVEMENT relative to its value SLP_NO_IMPROVEMENT_WINDOW
        // iterations ago. SLP iterates routinely bounce around the optimum
        // (cuts at one grid point can unmask new violators elsewhere on the
        // next iterate), so iterate-by-iterate monotone descent is too strict;
        // best-so-far across a sliding window is the robust progress metric.
        if outer > SLP_WARMUP_ITERS && best_ratio_history.len() > SLP_NO_IMPROVEMENT_WINDOW {
            let len = best_ratio_history.len();
            let baseline = best_ratio_history[len - 1 - SLP_NO_IMPROVEMENT_WINDOW];
            let current = best_ratio_history[len - 1];
            let improvement = (baseline - current) / baseline.max(1.0);
            if improvement < SLP_MIN_IMPROVEMENT {
                return Ok((
                    best_result,
                    SlpOutcome::Diverged {
                        last_max_ratio: best_ratio_so_far,
                        outer_iters: outer,
                    },
                ));
            }
        }
    }

    Ok((
        best_result,
        SlpOutcome::MaxIters {
            last_max_ratio: best_ratio_so_far,
        },
    ))
}

/// One violator of the path-jerk constraint at iteration `k`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct JerkViolator {
    /// Interior grid index of the violation (`1 ≤ i ≤ N − 2`). Currently
    /// unused under the full-grid-linearization cut strategy (cuts are
    /// placed at every interior grid point regardless), but retained for
    /// future telemetry / per-violator-cut variants.
    #[allow(dead_code)]
    pub i: usize,
    /// `|Δ²b_i|·√b_i / (2J·h²)` at the current iterate. `> 1 + ε` for a
    /// violator.
    pub ratio: f64,
}

/// Scan the interior grid points and return all violators of
/// `|Δ²b_i|·√b_i ≤ 2J·h²·(1 + ε_feas)`.
fn find_jerk_violators(b: &[f64], h: f64, j_path: f64) -> Vec<JerkViolator> {
    let n = b.len();
    if n < 3 {
        return Vec::new();
    }
    let two_jh2 = 2.0 * j_path * h * h;
    let mut out = Vec::new();
    for i in 1..n - 1 {
        let bi = b[i];
        if bi <= 0.0 {
            continue; // nothing to bind against (and √b would be undefined)
        }
        let d2b = b[i - 1] - 2.0 * bi + b[i + 1];
        let ratio = d2b.abs() * bi.sqrt() / two_jh2;
        if ratio > 1.0 + SLP_EPS_FEAS {
            out.push(JerkViolator { i, ratio });
        }
    }
    out
}

#[inline]
fn max_ratio(vs: &[JerkViolator]) -> f64 {
    vs.iter().map(|v| v.ratio).fold(0.0_f64, f64::max)
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
