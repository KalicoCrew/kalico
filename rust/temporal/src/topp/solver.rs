//! Clarabel SOCP construction + solve. INTERNAL — Clarabel types do not
//! escape this module per spec §1.1 / §2.3.
//!
//! # Sign-convention note
//!
//! This module's docs reference math symbols (`b̄`, `ā`, `√b`, `s⃛`, …) and
//! enum/variant names (`AxisJerk`, `StartBoundary`, …) inline; clippy's
//! `doc_markdown` lint flags every non-backtick CamelCase / unicode-math
//! identifier and is too noisy here. Disable at module scope.

#![allow(clippy::doc_markdown)]
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

/// One linearized Taylor cut produced by the SLP outer loop.
///
/// Two flavors are emitted:
///
/// - `PathJerk { i, b_bar }`: scalar-tangential path-jerk envelope cut. Two
///   `Nonneg` rows are appended per cut, encoding the convex first-order
///   Taylor expansion of `1/√b` at iterate `b̄_i = b_bar`. Convex-down
///   tangent ⇒ global underestimator ⇒ tightens the relaxation. See
///   `append_path_jerk_cut_to_clarabel` for the row-coefficient derivation.
///
/// - `AxisJerk { ... }`: per-axis Cartesian jerk cut at the verifier-stencil
///   linearization (Step 9). Two `Nonneg` rows per cut (positive and
///   negative side, |j_axis| ≤ j_max bound). Couples `b_i` and the
///   neighborhood of `a` (3 nonzeros interior, 2 nonzeros at boundaries).
///   See `append_axis_jerk_cut_to_clarabel`.
///
///   The AxisJerk cut linearizes `j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛`
///   at the iterate, where `s⃛ = (da/ds)·√b` is reconstructed from the
///   verifier's central / one-sided FD on `a`. The Taylor expansion is exact
///   at the iterate (verified numerically by
///   `tests/step9_cut_identity.rs`); convexity is *not* global because the
///   cross-term `a·√b` is bilinear-times-sqrt (indefinite Hessian on
///   `(a,b)`), so the cut is a LOCAL approximation. The L∞ trust region
///   on `(b, a)` plus accept-only-if-decrease backtracking in
///   `slp_solve_with_axis_jerk` is what makes the SLP converge despite the
///   non-convex linearization.
#[derive(Debug, Clone, Copy)]
pub(crate) enum SlpCut {
    /// Scalar-tangential path-jerk cut (Lee 2024 §III–§IV; spec §11).
    /// Interior grid index `i` (1 ≤ i ≤ N − 2); iterate value `b̄_i = b_bar`.
    PathJerk { i: usize, b_bar: f64 },
    /// Per-axis Cartesian jerk cut at the verifier stencil (Step 9; spec §11).
    AxisJerk(AxisJerkCut),
}

/// Per-axis Cartesian jerk cut details. Spec §5; Step 9 with width-1 b-FD
/// stencil unification per
/// `docs/superpowers/specs/2026-05-05-stencil-unification-design.md`.
#[derive(Debug, Clone, Copy)]
pub(crate) struct AxisJerkCut {
    /// Grid index at which the cut is anchored (`0 ≤ i ≤ n−1`).
    pub i: usize,
    /// Axis index (0 = X, 1 = Y, 2 = Z). Held for diagnostic / future
    /// telemetry; the row coefficients only depend on `(cp, cpp, cppp)` for
    /// that axis, which the caller pre-extracts.
    #[allow(dead_code)]
    pub axis: usize,
    /// Stencil kind — controls FD shape and which `b`-variables the row touches.
    pub stencil: AxisJerkStencil,
    /// Iterate values for the three `b̄` indices the stencil reads.
    /// Interior at i:    `[b̄_{i-1}, b̄_i, b̄_{i+1}]`.
    /// StartBoundary:    `[b̄_0,    b̄_1, b̄_2]`.
    /// EndBoundary:      `[b̄_{n-3}, b̄_{n-2}, b̄_{n-1}]`.
    pub b_bars: [f64; 3],
    /// Iterate value `ā_i` at the anchor index. Single index — under
    /// width-1 b-FD the cut row only touches `a_i`, never neighbours.
    pub a_bar_i: f64,
    /// Path derivatives at `s_i` along `axis`: `(c', c'', c''')`.
    pub cp: f64,
    pub cpp: f64,
    pub cppp: f64,
    /// Per-axis jerk bound `j_max[axis] · target_ratio`, inflated by the
    /// SLP target-ratio schedule. Used directly as the cut RHS magnitude.
    pub j_lim_inflated: f64,
}

/// Discrete shape of the stencil under width-1 b-FD. Mirrors
/// `topp::stencil::SDddotStencil`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AxisJerkStencil {
    /// Central FD: row touches `b_{i-1}, b_i, b_{i+1}, a_i`.
    Interior,
    /// Forward FD at i=0: row touches `b_0, b_1, b_2, a_0`.
    StartBoundary,
    /// Backward FD at i=n-1: row touches `b_{n-3}, b_{n-2}, b_{n-1}, a_{n-1}`.
    EndBoundary,
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
    SolvedInexact {
        residual: f64,
    },
    Infeasible,
    /// Renamed from `last_residual` to `residual` for consistency with
    /// `SolvedInexact`; both encode `max(r_prim, r_dual)`.
    MaxIter {
        residual: f64,
    },
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
/// Equivalent to `solve_with_cuts(bundle, &[], tol)` — kept as the public(crate)
/// entry point for the unit-test surface (no cuts → original Consolini-Locatelli
/// SOCP).
#[allow(dead_code)]
pub(crate) fn solve(bundle: &ConstraintBundle) -> Result<SolverResult, SolverSetupError> {
    solve_with_cuts(bundle, &[], 1e-8)
}

/// Append one path-jerk SLP cut as two `Nonneg` rows (positive- and
/// negative-side) to the Clarabel-format `A` and `b_rhs` accumulators. The
/// current row count `n_rows` is also updated.
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
fn append_path_jerk_cut_to_clarabel(
    i: usize,
    b_bar: f64,
    j_path: f64,
    h: f64,
    n_rows: &mut usize,
    rowval: &mut [Vec<usize>],
    nzval: &mut [Vec<f64>],
    b_rhs: &mut Vec<f64>,
    n_grid: usize,
) {
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

/// Append one per-axis Cartesian jerk SLP cut as two `Nonneg` rows
/// (positive- and negative-side ⇒ |j_axis| ≤ j_max·(1+ε)). Spec §5; Step 9
/// with width-1 b-FD stencil unification.
///
/// # Cut algebra
///
/// The per-axis Cartesian jerk at iterate `(b̄, ā)` under width-1 b-FD:
///
/// ```text
///   Interior:       j_axis = c'''·b̄_i^(3/2) + 3·c''·ā_i·√b̄_i
///                          + c'·(b̄_{i-1} − 2·b̄_i + b̄_{i+1})·√b̄_i / (2h²)
///   StartBoundary:  same form with D₂ = b̄_0 − 2·b̄_1 + b̄_2
///   EndBoundary:    same form with D₂ = b̄_{n-3} − 2·b̄_{n-2} + b̄_{n-1}
/// ```
///
/// where the b-FD second-difference replaces the prior central-FD-on-`a`.
/// First-order Taylor linearization at the iterate gives the row
/// coefficients. Let `S = √b̄_i` (floored at √SLP_B_FLOOR), `S3 = b̄_i^(3/2)`.
///
/// **Interior** (touches `b_{i-1}, b_i, b_{i+1}, a_i`):
/// ```text
///   α_b_im1  = c'·S / (2h²)
///   α_b_ip1  = c'·S / (2h²)
///   α_b_i    = (3/2)·c'''·S
///            + 3·c''·ā_i / (2·S)
///            − c'·S / h²
///            + c'·D₂_int / (4h² · S)
///   α_a_i    = 3·c''·S
///   K        = −(1/2)·c'''·S3
///            − (3/2)·c''·ā_i·S
///            − c'·D₂_int·S / (4h²)
/// ```
///
/// **StartBoundary i=0** (touches `b_0, b_1, b_2, a_0`):
/// ```text
///   α_b_0    = (3/2)·c'''·S + 3·c''·ā_0 / (2·S)
///            + c'·S / (2h²) + c'·D₂_fwd / (4h² · S)
///   α_b_1    = −c'·S / h²
///   α_b_2    = c'·S / (2h²)
///   α_a_0    = 3·c''·S
///   K        = −(1/2)·c'''·S3 − (3/2)·c''·ā_0·S − c'·D₂_fwd·S / (4h²)
/// ```
///
/// **EndBoundary i=n-1** (touches `b_{n-3}, b_{n-2}, b_{n-1}, a_{n-1}`):
/// ```text
///   α_b_nm3  = c'·S / (2h²)
///   α_b_nm2  = −c'·S / h²
///   α_b_nm1  = (3/2)·c'''·S + 3·c''·ā_{n-1} / (2·S)
///            + c'·S / (2h²) + c'·D₂_bwd / (4h² · S)
///   α_a_nm1  = 3·c''·S
///   K        = −(1/2)·c'''·S3 − (3/2)·c''·ā_{n-1}·S − c'·D₂_bwd·S / (4h²)
/// ```
///
/// All three cases share the same closed-form `K`: substitute the stencil-
/// specific S, ā, and D₂.
///
/// The two `Nonneg` rows in `A·x + b_rhs ≥ 0` form:
///
/// ```text
///   (+):  J_lim_inflated − (Σ α·x + K)  ≥ 0   ⇒   row = [−α₁, …, −αₖ],  rhs = J_lim − K
///   (−):  J_lim_inflated + (Σ α·x + K)  ≥ 0   ⇒   row = [+α₁, …, +αₖ],  rhs = J_lim + K
/// ```
///
/// `b̄` MUST be ≥ a positive floor; the helper floors via `SLP_B_FLOOR` to
/// avoid `1/√0` blowing up the row-coefficient magnitudes.
///
/// Identity check (numerical pin): `rust/temporal/tests/step9_cut_identity.rs`.
#[allow(clippy::too_many_arguments)]
fn append_axis_jerk_cut_to_clarabel(
    cut: &AxisJerkCut,
    h: f64,
    n_rows: &mut usize,
    rowval: &mut [Vec<usize>],
    nzval: &mut [Vec<f64>],
    b_rhs: &mut Vec<f64>,
    n_grid: usize,
) {
    let i = cut.i;
    let cp = cut.cp;
    let cpp = cut.cpp;
    let cppp = cut.cppp;
    let j = cut.j_lim_inflated;

    // SOCP variable layout: b at 0..n_grid, a at n_grid..2*n_grid.
    let off_b = 0usize;
    let off_a = n_grid;

    // Compute (α at anchor b-var, three other (var_idx, α) entries, K) per stencil.
    // The `b_anchor` is floored at SLP_B_FLOOR to keep 1/√b̄ bounded.
    let (alpha_b_anchor, entries_extra, k_const): (f64, [(usize, f64); 3], f64) = match cut
        .stencil
    {
        AxisJerkStencil::Interior => {
            debug_assert!(i >= 1 && i + 1 < n_grid, "interior index out of range");
            let b_anchor = cut.b_bars[1].max(SLP_B_FLOOR);
            let s = b_anchor.sqrt();
            let s3 = b_anchor * s;
            let b_im1 = cut.b_bars[0];
            let b_ip1 = cut.b_bars[2];
            let a_i = cut.a_bar_i;
            let d2 = b_im1 - 2.0 * b_anchor + b_ip1;
            let alpha_b_im1 = cp * s / (2.0 * h * h);
            let alpha_b_ip1 = cp * s / (2.0 * h * h);
            let alpha_a_i = 3.0 * cpp * s;
            let alpha_b_i = 1.5 * cppp * s
                + 3.0 * cpp * a_i / (2.0 * s)
                - cp * s / (h * h)
                + cp * d2 / (4.0 * h * h * s);
            let k = -0.5 * cppp * s3
                - 1.5 * cpp * a_i * s
                - cp * d2 * s / (4.0 * h * h);
            (
                alpha_b_i,
                [
                    (off_b + i - 1, alpha_b_im1),
                    (off_b + i + 1, alpha_b_ip1),
                    (off_a + i, alpha_a_i),
                ],
                k,
            )
        }
        AxisJerkStencil::StartBoundary => {
            debug_assert_eq!(i, 0, "StartBoundary stencil expects i = 0");
            debug_assert!(n_grid >= 3);
            let b_anchor = cut.b_bars[0].max(SLP_B_FLOOR);
            let s = b_anchor.sqrt();
            let s3 = b_anchor * s;
            let b_1 = cut.b_bars[1];
            let b_2 = cut.b_bars[2];
            let a_0 = cut.a_bar_i;
            let d2 = b_anchor - 2.0 * b_1 + b_2;
            let alpha_b_0 = 1.5 * cppp * s
                + 3.0 * cpp * a_0 / (2.0 * s)
                + cp * s / (2.0 * h * h)
                + cp * d2 / (4.0 * h * h * s);
            let alpha_b_1 = -cp * s / (h * h);
            let alpha_b_2 = cp * s / (2.0 * h * h);
            let alpha_a_0 = 3.0 * cpp * s;
            let k = -0.5 * cppp * s3
                - 1.5 * cpp * a_0 * s
                - cp * d2 * s / (4.0 * h * h);
            (
                alpha_b_0,
                [
                    (off_b + 1, alpha_b_1),
                    (off_b + 2, alpha_b_2),
                    (off_a, alpha_a_0),
                ],
                k,
            )
        }
        AxisJerkStencil::EndBoundary => {
            debug_assert_eq!(i, n_grid - 1, "EndBoundary stencil expects i = N-1");
            debug_assert!(n_grid >= 3);
            let b_anchor = cut.b_bars[2].max(SLP_B_FLOOR);
            let s = b_anchor.sqrt();
            let s3 = b_anchor * s;
            let b_nm3 = cut.b_bars[0];
            let b_nm2 = cut.b_bars[1];
            let a_nm1 = cut.a_bar_i;
            let d2 = b_nm3 - 2.0 * b_nm2 + b_anchor;
            let alpha_b_nm3 = cp * s / (2.0 * h * h);
            let alpha_b_nm2 = -cp * s / (h * h);
            let alpha_b_nm1 = 1.5 * cppp * s
                + 3.0 * cpp * a_nm1 / (2.0 * s)
                + cp * s / (2.0 * h * h)
                + cp * d2 / (4.0 * h * h * s);
            let alpha_a_nm1 = 3.0 * cpp * s;
            let k = -0.5 * cppp * s3
                - 1.5 * cpp * a_nm1 * s
                - cp * d2 * s / (4.0 * h * h);
            (
                alpha_b_nm1,
                [
                    (off_b + n_grid - 3, alpha_b_nm3),
                    (off_b + n_grid - 2, alpha_b_nm2),
                    (off_a + n_grid - 1, alpha_a_nm1),
                ],
                k,
            )
        }
    };

    // The "anchor" b-variable column is at the absolute grid index `off_b + i`.
    let anchor_b_col = off_b + i;

    // Sign-convention negation: A_clarabel = -A_k.
    // Positive side: A_k row = [-α], rhs = J - K. Clarabel pushes +α.
    let pos_row = *n_rows;
    push_nz(rowval, nzval, anchor_b_col, pos_row, alpha_b_anchor);
    for &(col, alpha) in &entries_extra {
        if alpha != 0.0 {
            push_nz(rowval, nzval, col, pos_row, alpha);
        }
    }
    b_rhs.push(j - k_const);
    *n_rows += 1;

    // Negative side: A_k row = [+α], rhs = J + K. Clarabel pushes -α.
    let neg_row = *n_rows;
    push_nz(rowval, nzval, anchor_b_col, neg_row, -alpha_b_anchor);
    for &(col, alpha) in &entries_extra {
        if alpha != 0.0 {
            push_nz(rowval, nzval, col, neg_row, -alpha);
        }
    }
    b_rhs.push(j + k_const);
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

/// L∞ trust region on `(b, a)` around iterate `(b̄, ā)`.
///
/// Used by the per-axis-jerk SLP outer loop (Step 9) to keep the inner SOCP
/// from leaving the local-validity neighborhood of the non-convex cut
/// linearization. Box rows enforce
/// `b̄_i·(1−ρ_b) ≤ b_i ≤ b̄_i·max(1+ρ_b, B_FLOOR_REL)` and
/// `ā_i − ρ_a·max(|ā_i|, A_TR_FLOOR) ≤ a_i ≤ ā_i + ρ_a·max(|ā_i|, A_TR_FLOOR)`,
/// for all grid points.
///
/// Boundary `b` rows are skipped — the boundary equality in block (a) of
/// the bundle pins `b_0` and `b_{N-1}` exactly; layering trust-region inequalities
/// on top is redundant and risks infeasibility if the iterate's b̄_0 already
/// equals v_start² but not exactly (numerical drift).
#[derive(Debug, Clone, Copy)]
pub(crate) struct TrustRegion {
    pub rho_b: f64,
    pub rho_a: f64,
}

/// Floor on |ā_i| used when sizing the a-trust-region radius. Pure-zero `ā`
/// would give zero-width trust region and pin `a_i` to its previous iterate;
/// a positive floor on the order of `a_max` lets the inner SOCP swing `a`
/// across the per-axis acceleration range when the iterate's `ā` is small
/// or has the wrong sign at a grid point. Verifier non-blocker: tuned
/// against fixture 4; revisit if observed to over-relax.
const A_TR_FLOOR: f64 = 5_000.0; // mm/s² (≈ a_max)

/// Floor on `b̄_i` used when sizing the b-trust-region radius. Without this,
/// a small iterate value (cruise drop, near-boundary ramp-up) gives a
/// near-zero TR that the SOCP can't satisfy against block (e) centripetal
/// caps or other lower bounds. Floor = (50 mm/s)² = 2500.
const B_TR_FLOOR: f64 = 2_500.0;

/// Construct the Clarabel SOCP from `bundle` and solve it, with optional SLP
/// cut rows and trust-region rows appended.
///
/// `cuts`, when non-empty, becomes additional `Nonneg` rows of the SOCP. The
/// cuts reference only `b` and `a` variables that already exist in the bundle;
/// no new variables are introduced (`n_vars` unchanged).
///
/// `trust_region`, when `Some`, additionally appends 2-sided box rows on every
/// `b_i` (interior) and every `a_i`, anchored at `(b_bar, a_bar)`.
fn solve_with_cuts(
    bundle: &ConstraintBundle,
    cuts: &[SlpCut],
    tol: f64,
) -> Result<SolverResult, SolverSetupError> {
    solve_with_cuts_and_trust_region(bundle, cuts, None, &[], &[], tol)
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn solve_with_cuts_and_trust_region(
    bundle: &ConstraintBundle,
    cuts: &[SlpCut],
    trust_region: Option<TrustRegion>,
    b_bar: &[f64],
    a_bar: &[f64],
    tol: f64,
) -> Result<SolverResult, SolverSetupError> {
    let n_vars = bundle.n_vars;
    let n_grid = bundle.n_grid;

    // Step 1: cones — append a Nonneg block for SLP cuts and trust region.
    let mut cones_clarabel = map_clarabel_cones(bundle)?;
    let cut_rows = 2 * cuts.len();
    if cut_rows > 0 {
        cones_clarabel.push(NonnegativeConeT(cut_rows));
    }
    // Trust-region rows: for each interior `b_i` two rows (lower + upper),
    // for every `a_i` two rows. Boundary b rows skipped (pinned by block (a)).
    let tr_rows = if trust_region.is_some() {
        if n_grid >= 2 {
            2 * (n_grid - 2) + 2 * n_grid
        } else {
            0
        }
    } else {
        0
    };
    if tr_rows > 0 {
        cones_clarabel.push(NonnegativeConeT(tr_rows));
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
    debug_assert!(
        j_path > 0.0 && h > 0.0,
        "bundle must carry positive j_path/h"
    );
    for cut in cuts {
        match cut {
            SlpCut::PathJerk { i, b_bar } => {
                let b_bar_floored = b_bar.max(SLP_B_FLOOR);
                append_path_jerk_cut_to_clarabel(
                    *i,
                    b_bar_floored,
                    j_path,
                    h,
                    &mut n_rows,
                    &mut rowval_per_col,
                    &mut nzval_per_col,
                    &mut b_rhs,
                    n_grid,
                );
            }
            SlpCut::AxisJerk(axis_cut) => {
                append_axis_jerk_cut_to_clarabel(
                    axis_cut,
                    h,
                    &mut n_rows,
                    &mut rowval_per_col,
                    &mut nzval_per_col,
                    &mut b_rhs,
                    n_grid,
                );
            }
        }
    }

    // Step 3c: append trust-region rows.
    if let Some(tr) = trust_region {
        debug_assert_eq!(b_bar.len(), n_grid);
        debug_assert_eq!(a_bar.len(), n_grid);
        // b_i interior: |b_i − b̄_i| ≤ ρ_b · max(b̄_i, B_TR_FLOOR)
        // (boundary b_0, b_{N-1} pinned by block (a); skip).
        let off_b = 0;
        for i in 1..n_grid.saturating_sub(1) {
            let bb = b_bar[i].max(0.0);
            let radius = tr.rho_b * bb.max(B_TR_FLOOR);
            let lo = bb - radius;
            let hi = bb + radius;
            // Lower: b_i − lo ≥ 0  →  A_k row [+1 on b_i], rhs = −lo
            // Upper: hi − b_i ≥ 0  →  A_k row [−1 on b_i], rhs = +hi
            let row_lo = n_rows;
            push_nz(
                &mut rowval_per_col,
                &mut nzval_per_col,
                off_b + i,
                row_lo,
                -1.0,
            );
            b_rhs.push(-lo);
            n_rows += 1;
            let row_hi = n_rows;
            push_nz(
                &mut rowval_per_col,
                &mut nzval_per_col,
                off_b + i,
                row_hi,
                1.0,
            );
            b_rhs.push(hi);
            n_rows += 1;
        }
        // a_i: ā_i − ρ_a·max(|ā|, FLOOR) ≤ a_i ≤ ā_i + ρ_a·max(|ā|, FLOOR).
        let off_a = n_grid;
        for i in 0..n_grid {
            let ab = a_bar[i];
            let radius = tr.rho_a * ab.abs().max(A_TR_FLOOR);
            let lo = ab - radius;
            let hi = ab + radius;
            let row_lo = n_rows;
            push_nz(
                &mut rowval_per_col,
                &mut nzval_per_col,
                off_a + i,
                row_lo,
                -1.0,
            );
            b_rhs.push(-lo);
            n_rows += 1;
            let row_hi = n_rows;
            push_nz(
                &mut rowval_per_col,
                &mut nzval_per_col,
                off_a + i,
                row_hi,
                1.0,
            );
            b_rhs.push(hi);
            n_rows += 1;
        }
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
    let a_csc = CscMatrix {
        m: n_rows,
        n: n_vars,
        colptr,
        rowval,
        nzval,
    };

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
    //
    // Reduced tolerances match verify::check's EPS_FEAS=1e-3 (spec §6.2).
    // Allows AlmostSolved states at this tolerance to map to SolvedInexact
    // instead of MaxIter; SLP outer loop can then continue with cuts.
    // Solved gating remains at default eps_abs.
    #[allow(clippy::similar_names)]
    let settings = DefaultSettings::<f64> {
        verbose: false,
        max_iter: 1000,
        // Primary tolerances (per-call adaptive — see `ToleranceMode`).
        tol_gap_abs: tol,
        tol_gap_rel: tol,
        tol_feas: tol,
        // Reduced (AlmostSolved) tolerances — preserve existing 1e-3 overrides.
        // These let Clarabel report `SolvedInexact` (mapped to public `SolveStatus`
        // via `output::assemble`) when the primary tolerance can't be reached;
        // SLP and downstream `verify::check` (ε_feas = 1e-3) rely on this gating.
        // Per Codex review-4: dropping these would silently restore Clarabel
        // defaults (5e-5 / 5e-5 / 1e-4), changing `AlmostSolved` semantics.
        reduced_tol_gap_abs: 1e-3,
        reduced_tol_gap_rel: 1e-3,
        reduced_tol_feas: 1e-3,
        // Determinism pin (per Codex review-4 + kalico-verifier round-3
        // recommendation): explicitly use single-threaded QDLDL backend so the
        // joining-loop early-bail's determinism premise stays valid against
        // future Clarabel versions / feature-flag changes.
        direct_solve_method: "qdldl".to_string(),
        max_threads: 1,
        ..Default::default()
    };

    // Step 8: construct and run.
    let mut solver = DefaultSolver::new(&p_zero, q, &a_csc, &b_rhs, &cones_clarabel, settings)
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
    tol: f64,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let h = bundle.h;
    let j_path = bundle.j_path;
    debug_assert!(h > 0.0 && j_path > 0.0);

    let mut cuts: Vec<SlpCut> = Vec::new();
    let mut last_result = solve_with_cuts(bundle, &cuts, tol)?;

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
            cuts.push(SlpCut::PathJerk { i, b_bar });
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

        let new_result = solve_with_cuts(bundle, &cuts, tol)?;
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
// Per-axis Cartesian jerk SLP outer loop (Step 9; spec §11)
// ---------------------------------------------------------------------------
//
// Layered on top of `slp_solve` (path-jerk SLP). The path-jerk loop tightens
// the SOCP relaxation against `|s⃛| ≤ J_path` (scalar tangential jerk). That
// is necessary but insufficient: per-axis Cartesian jerk
//
//     j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛
//
// is what the verifier (and physical hardware) checks. On curved paths with
// non-zero endpoint velocity (fixture 4), the path-jerk-feasible iterate
// can violate per-axis jerk by tens of percent at the boundary.
//
// Mitigation: a SECOND SLP loop using verifier-stencil cuts on j_axis. The
// cuts are first-order Taylor expansions of `j_axis(b, a)` at the iterate;
// because `j_axis` has bilinear-times-sqrt cross-terms (term 2: 3·c''·a·√b,
// term 3 via `s⃛`), the linearization is non-convex (Hessian indefinite on
// `(a, b)`). Three coupled remedies make convergence reliable:
//
//   1. **Active-set placement.** Cut only at (i, axis) pairs where the
//      verifier-form ratio > 1 + ε at the current iterate. Avoids over-
//      tightening at points where the relaxation is already feasible.
//
//   2. **L∞ trust region** on `(b, a)` around the current iterate. Initial
//      ρ_b = 0.05, ρ_a = 0.10 (tightened from the diagnosis's recommended
//      0.20/0.30 — the non-convex Term 2/Term 3 `a·√b` cross-terms
//      invalidate the linearization at larger radii in fixture-4 testing).
//      Adapts: expand by 1.5× on accept, contract by 0.5× on reject.
//      Caps: ρ_b ∈ [0.005, 0.20], ρ_a ∈ [0.01, 0.40].
//
//   3. **Accept-only-if-decrease.** After each inner SOCP, evaluate the
//      verifier-form max-ratio at the new iterate. If it does not decrease
//      below the previous best, the trust-region radii ρ_b, ρ_a are halved
//      (ρ ← ρ × 0.5^backtrack_count) and the inner SOCP is re-solved with
//      the tighter feasibility region, up to MAX_BACKTRACKS=3 retries per
//      outer iter.
//
// Verifier-non-blocker note: future widening to ±2 neighborhood for the
// active set may help if non-adjacent oscillation appears. Not implemented;
// add if observed.
//
// Source: Nocedal & Wright Numerical Optimization 2e, §18.5 (trust-region
// SQP). Step 9 design: /tmp/pf_diagnosis.json, spec §11.

/// Maximum per-axis-jerk SLP outer iterations. Verifier non-blocker note
/// requested 30 (bumped from 15) with a soft warning at iter 15.
const SLP9_MAX_OUTER_ITERS: u32 = 30;

/// Soft-warning threshold for slow per-axis SLP convergence.
const SLP9_WARN_AT_ITER: u32 = 15;

/// Per-axis-jerk feasibility tolerance. Mirrors `verify::EPS_FEAS = 1e-3`
/// (spec §6.2): the SOCP cut and the verifier check use the SAME stencil,
/// so the cut tolerance can match the verifier tolerance directly.
const SLP9_EPS_FEAS: f64 = 1e-3;

/// Initial L∞ trust-region radii. Diagnosis recommendation was 0.20/0.30,
/// but empirically those values let the inner SOCP move far enough that the
/// non-convex `a·√b` cross-terms invalidate the linearization (cuts at
/// the iterate are satisfied at the new point, but the verifier-form ratio
/// at the new point is significantly worse than the cut model predicted).
/// Tightening to 0.05/0.10 keeps the iterate in the local-validity
/// neighborhood. The tighter step costs more outer iterations but converges
/// reliably on fixture 4 (v_end=0).
const SLP9_RHO_B_INIT: f64 = 0.05;
const SLP9_RHO_A_INIT: f64 = 0.10;
const SLP9_RHO_B_MIN: f64 = 0.005;
const SLP9_RHO_B_MAX: f64 = 0.20;
const SLP9_RHO_A_MIN: f64 = 0.01;
const SLP9_RHO_A_MAX: f64 = 0.40;

/// Maximum backtracks per outer iter before giving up the step and treating
/// the outer iter as a no-op (will retry from a contracted trust region next
/// outer iter).
const SLP9_MAX_BACKTRACKS: u32 = 3;

/// Continuation factor on the cut RHS. When the iterate's verifier-form
/// per-axis ratio is `R > 1`, an iter-1 cut at the strict feasibility
/// tolerance `j_max·(1 + ε)` is infeasible inside any reasonable trust
/// region (the iterate lies far outside the cut). We instead inflate the
/// cut RHS to `j_max · target_ratio_k`, where
///
/// ```text
///     target_ratio_k = max(1 + ε, R_k · SLP9_TARGET_DECAY)
/// ```
///
/// and `R_k` is the best ratio seen at outer iter `k`. This is a
/// continuation / homotopy schedule: the cut starts loose enough to admit
/// the current iterate, then tightens by `SLP9_TARGET_DECAY` per outer iter
/// until it matches the verifier's `1 + ε` feasibility tolerance.
///
/// `SLP9_TARGET_DECAY = 0.85` — gentle geometric tightening per outer iter.
/// More-aggressive decay (0.5) is feasible inside the trust region only when
/// the iterate is already close to feasibility; for fixture 4 (initial
/// ratio R ≈ 1.24 at v_end=0), even target_ratio = 1.001 with R · 0.5 = 0.62
/// (clamped) leaves the inner SOCP infeasible because the cut at the worst
/// violator demands a linear improvement that's too large for the
/// trust-region neighborhood. A gentle 0.85 decay allows the SOCP to make
/// incremental progress; ~30 iters from R=1.24 to R=1.001.
const SLP9_TARGET_DECAY: f64 = 0.85;

/// Run the per-axis Cartesian jerk SLP outer loop on top of the path-jerk
/// SLP. Spec §11; Step 9.
///
/// 1. Calls `slp_solve` to get a path-jerk-feasible iterate (b̄_0, ā_0).
/// 2. Checks per-axis jerk violators using the verifier stencil.
/// 3. If any violator exceeds (1 + SLP9_EPS_FEAS), enters the per-axis SLP
///    outer loop with active-set cuts + L∞ trust region + accept-only-if-
///    decrease backtracking.
///
/// Returns the converged iterate plus the worst-case `SlpOutcome` across the
/// two stages (path-jerk and per-axis). Path-jerk failures (Diverged,
/// InnerSolverFailure) short-circuit per-axis processing.
#[allow(clippy::too_many_lines)]
pub(crate) fn slp_solve_with_axis_jerk(
    bundle: &ConstraintBundle,
    grid: &crate::topp::path::ArclengthGrid,
    limits: &crate::Limits,
    tol: f64,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    // Stage 1: path-jerk SLP. Existing logic, untouched.
    let (path_result, path_outcome) = slp_solve(bundle, tol)?;

    // Path-jerk failure modes: surface immediately. The per-axis loop has
    // nothing useful to add when the inner SOCP is already not producing a
    // trustworthy iterate.
    if matches!(
        path_outcome,
        SlpOutcome::InnerSolverFailure | SlpOutcome::Diverged { .. } | SlpOutcome::MaxIters { .. }
    ) {
        return Ok((path_result, path_outcome));
    }

    debug_assert_eq!(grid.s.len(), path_result.b.len());

    // Stage 2: per-axis-jerk active-set + trust-region SLP.
    let mut last_result = path_result.clone();
    // path_outer_iters: how many path-jerk outer iters were used (carried into
    // the final SlpOutcome::Converged so the caller sees total work).
    let path_outer_iters = match path_outcome {
        SlpOutcome::Converged { outer_iters } => outer_iters,
        _ => 0,
    };

    // Initial violator scan.
    let initial_max = max_axis_ratio(&last_result, grid, limits, bundle.h);
    if initial_max <= 1.0 + SLP9_EPS_FEAS {
        // Path-jerk-feasible iterate is also per-axis-feasible. Return it.
        return Ok((
            last_result,
            SlpOutcome::Converged {
                outer_iters: path_outer_iters,
            },
        ));
    }

    let mut best_result = last_result.clone();
    let mut best_ratio = initial_max;
    let mut rho_b = SLP9_RHO_B_INIT;
    let mut rho_a = SLP9_RHO_A_INIT;

    for outer in 1..=SLP9_MAX_OUTER_ITERS {
        if outer == SLP9_WARN_AT_ITER {
            // Soft warning to log; do not fail.
            eprintln!(
                "slp9 warning: per-axis SLP not converged at iter {outer} \
                 (best ratio = {best_ratio:.4})",
            );
        }

        // Continuation schedule on the cut RHS: tighten geometrically toward
        // (1 + ε) from the current best ratio. The first cuts are loose
        // enough that the current iterate isn't trivially infeasible inside
        // the trust region.
        let target_ratio = (best_ratio * SLP9_TARGET_DECAY).max(1.0 + SLP9_EPS_FEAS);

        // Build active-set cuts at violators (and immediate neighbors to
        // avoid one-sided over-tightening; ±0 for now per task brief, the
        // ±1 widening is reserved for the diagnosis non-blocker note).
        let cuts = build_axis_jerk_cuts(&last_result, grid, limits, target_ratio, bundle.h);
        if cuts.is_empty() {
            // No violators above threshold: converged.
            return Ok((
                last_result,
                SlpOutcome::Converged {
                    outer_iters: path_outer_iters + outer,
                },
            ));
        }

        // ---- Inner solve sequence: first try with trust region (the
        // standard SLP iteration); if that's infeasible, retry without
        // the trust region (allowing a larger jump). The second case is
        // common on the first iteration when the path-jerk-converged
        // iterate is far outside per-axis feasibility (e.g. fixture 4
        // boundary, ratio = 185×).
        let mut accepted: Option<SolverResult> = None;
        for backtrack in 0..=SLP9_MAX_BACKTRACKS {
            let bt_i32 = i32::try_from(backtrack).unwrap_or(i32::MAX);
            let tr = TrustRegion {
                rho_b: rho_b * 0.5_f64.powi(bt_i32),
                rho_a: rho_a * 0.5_f64.powi(bt_i32),
            };
            let candidate = solve_with_cuts_and_trust_region(
                bundle,
                &cuts,
                Some(tr),
                &last_result.b,
                &last_result.a,
                tol,
            )?;
            if matches!(
                candidate.status,
                SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
            ) {
                // Inner SOCP failed at this trust-region size; halve and retry.
                continue;
            }
            // Evaluate verifier-form max-ratio at the candidate.
            let cand_ratio = max_axis_ratio(&candidate, grid, limits, bundle.h);
            if cand_ratio < best_ratio {
                accepted = Some(candidate);
                best_ratio = cand_ratio;
                break;
            }
            // Strict decrease failed; backtrack.
        }
        if accepted.is_none() {
            // Fallback: solve with cuts but NO trust region. The trust
            // region is a stability device; on iterations where the
            // iterate is far outside the cut-feasible region (typical
            // first per-axis SLP iter on fixture 4 with high-curvature
            // boundary), no point inside the TR satisfies the cut.
            // Without the TR, Clarabel can jump to a cut-feasible iterate
            // (potentially far). We still gate acceptance on max-ratio
            // decrease — non-decrease is a signal the cut linearization
            // was poor and the iterate shouldn't be trusted.
            let candidate = solve_with_cuts(bundle, &cuts, tol)?;
            if !matches!(
                candidate.status,
                SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
            ) {
                let cand_ratio = max_axis_ratio(&candidate, grid, limits, bundle.h);
                if cand_ratio < best_ratio {
                    accepted = Some(candidate);
                    best_ratio = cand_ratio;
                }
            }
        }

        if let Some(new_result) = accepted {
            last_result = new_result.clone();
            best_result = new_result;
            // Expand trust region for the next outer iter (rewards
            // good steps with bigger ones).
            rho_b = (rho_b * 1.5).min(SLP9_RHO_B_MAX);
            rho_a = (rho_a * 1.5).min(SLP9_RHO_A_MAX);

            if best_ratio <= 1.0 + SLP9_EPS_FEAS {
                return Ok((
                    last_result,
                    SlpOutcome::Converged {
                        outer_iters: path_outer_iters + outer,
                    },
                ));
            }
        } else {
            // No accepted step in any of the backtracks. Contract the
            // trust region for next iter and try again.
            rho_b = (rho_b * 0.5).max(SLP9_RHO_B_MIN);
            rho_a = (rho_a * 0.5).max(SLP9_RHO_A_MIN);
            // If we're already at the minimum and still no step, declare
            // divergence so the caller sees the best-so-far iterate.
            if rho_b <= SLP9_RHO_B_MIN * 1.0001 && rho_a <= SLP9_RHO_A_MIN * 1.0001 {
                return Ok((
                    best_result,
                    SlpOutcome::Diverged {
                        last_max_ratio: best_ratio,
                        outer_iters: path_outer_iters + outer,
                    },
                ));
            }
        }
    }

    Ok((
        best_result,
        SlpOutcome::MaxIters {
            last_max_ratio: best_ratio,
        },
    ))
}

/// Verifier-form per-axis Cartesian jerk ratio at every (i, axis), max over
/// the whole grid. Mirrors `verify::check`'s formula and uses the shared
/// width-1 b-FD stencil from `topp::stencil::s_dddot_at`.
fn max_axis_ratio(
    result: &SolverResult,
    grid: &crate::topp::path::ArclengthGrid,
    limits: &crate::Limits,
    h: f64,
) -> f64 {
    let n = result.b.len();
    debug_assert_eq!(grid.s.len(), n);
    let mut worst: f64 = 0.0;
    for i in 0..n {
        let s_dot = result.b[i].max(0.0).sqrt();
        let s_dot3 = s_dot * s_dot * s_dot;
        let s_ddot = result.a[i];
        let s_dddot = crate::topp::stencil::s_dddot_at(&result.b, i, h);
        for ax in 0..3 {
            let cp = grid.c_prime[i][ax];
            let cpp = grid.c_double_prime[i][ax];
            let cppp = grid.c_triple_prime[i][ax];
            let j = cppp * s_dot3 + 3.0 * cpp * s_dot * s_ddot + cp * s_dddot;
            let lim = limits.j_max[ax];
            let ratio = j.abs() / lim;
            if ratio > worst {
                worst = ratio;
            }
        }
    }
    worst
}

/// Build the active set of per-axis jerk cuts: one cut per (i, axis) pair
/// whose verifier-form ratio exceeds 1 + SLP9_EPS_FEAS at the current iterate.
///
/// `target_ratio` controls the cut RHS via continuation: cuts demand
/// `|j_axis| ≤ j_max · target_ratio` rather than `|j_axis| ≤ j_max·(1+ε)`
/// directly. The caller schedules `target_ratio` to decay from
/// `R_current · SLP9_TARGET_DECAY` toward `1 + ε` over outer iters; iter-1
/// thus solves a relaxation that admits the (still-inflated) iterate, with
/// successive iters tightening. Without this, the iter-1 cut at strict
/// `(1+ε)` is infeasible inside the trust region whenever `R_current ≫ 1`.
///
/// Cuts at i=0 use forward FD; cuts at i=N-1 use backward FD; interior cuts
/// use central FD (matching `verify::da_ds_at`). See
/// `append_axis_jerk_cut_to_clarabel` for the row-coefficient algebra.
fn build_axis_jerk_cuts(
    result: &SolverResult,
    grid: &crate::topp::path::ArclengthGrid,
    limits: &crate::Limits,
    target_ratio: f64,
    h: f64,
) -> Vec<SlpCut> {
    let n = result.b.len();
    let mut cuts: Vec<SlpCut> = Vec::new();
    for i in 0..n {
        let s_dddot = crate::topp::stencil::s_dddot_at(&result.b, i, h);
        let s_dot = result.b[i].max(0.0).sqrt();
        let s_dot3 = s_dot * s_dot * s_dot;
        let s_ddot = result.a[i];
        for ax in 0..3 {
            let cp = grid.c_prime[i][ax];
            let cpp = grid.c_double_prime[i][ax];
            let cppp = grid.c_triple_prime[i][ax];
            let j = cppp * s_dot3 + 3.0 * cpp * s_dot * s_ddot + cp * s_dddot;
            let lim = limits.j_max[ax];
            let ratio = j.abs() / lim;
            // Active-set: only cut at indices that violate strict feasibility.
            // The cut's RHS uses the (looser) target_ratio so the iterate is
            // not trivially-infeasible against the cut at iter 1.
            if ratio <= 1.0 + SLP9_EPS_FEAS {
                continue;
            }

            let stencil = if i == 0 {
                AxisJerkStencil::StartBoundary
            } else if i == n - 1 {
                AxisJerkStencil::EndBoundary
            } else {
                AxisJerkStencil::Interior
            };
            let b_bars: [f64; 3] = match stencil {
                AxisJerkStencil::Interior => [result.b[i - 1], result.b[i], result.b[i + 1]],
                AxisJerkStencil::StartBoundary => [result.b[0], result.b[1], result.b[2]],
                AxisJerkStencil::EndBoundary => {
                    [result.b[n - 3], result.b[n - 2], result.b[n - 1]]
                }
            };
            cuts.push(SlpCut::AxisJerk(AxisJerkCut {
                i,
                axis: ax,
                stencil,
                b_bars,
                a_bar_i: result.a[i],
                cp,
                cpp,
                cppp,
                j_lim_inflated: lim * target_ratio,
            }));
        }
    }
    cuts
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Limits;
    use crate::topp::constraints::{BuildOutcome, EndpointVelocities, build};
    use crate::topp::path::ArclengthGrid;

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
            EndpointVelocities {
                v_start: 0.0,
                v_end: 0.0,
            },
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
