//! Clarabel SOCP construction + solve. INTERNAL — Clarabel types do not
//! escape this module.
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
//!
//! The kalico `ConstraintBundle` uses: `A_k * x + b_rhs ∈ K`.
//! These are equal when `A_clarabel = -A_k` and `b_clarabel = b_rhs`:
//!   `s = b_rhs - (-A_k)*x = A_k*x + b_rhs` ∈ K  ✓
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
//! `RotatedSecondOrderConeT` does not exist in Clarabel 0.11. `constraints::build()`
//! never emits `Cone::RotatedSecondOrder`; jerk constraints use the norm-form
//! identity `z² ≤ u·v ↔ ||(2z, u-v)|| ≤ u+v` (standard SOC). The variant exists
//! for exhaustiveness but `solve()` returns `SolverSetupError` if a bundle contains it.
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
//! `InsufficientProgress` → `MaxIter` (closer to "gave up" than "structurally
//! infeasible"). Spec §4.2.

use clarabel::algebra::CscMatrix;
use clarabel::solver::{
    DefaultSettings, DefaultSolver, IPSolver, SolverStatus as ClarabelStatus,
    SupportedConeT::{NonnegativeConeT, SecondOrderConeT, ZeroConeT},
};

use crate::topp::constraints::{Cone, ConstraintBundle};

/// One linearized Taylor cut produced by the SLP outer loop.
///
/// - `PathJerk { i, b_bar }`: scalar-tangential path-jerk envelope cut. Two
///   `Nonneg` rows encode the first-order Taylor expansion of `1/√b` at
///   iterate `b̄_i`. Convex-down tangent ⇒ global underestimator ⇒ tightens
///   the relaxation. See `append_path_jerk_cut_to_clarabel` for coefficients.
///
/// - `AxisJerk`: per-axis Cartesian jerk cut linearizing
///   `j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛` at the iterate. The
///   cross-term `a·√b` is bilinear-times-sqrt (indefinite Hessian on `(a,b)`),
///   so the cut is a LOCAL approximation only. The L∞ trust region + accept-
///   only-if-decrease backtracking in `slp_solve_with_axis_jerk` is what makes
///   the SLP converge despite the non-convex linearization.
///   Numerical identity check: `tests/step9_cut_identity.rs`.
#[derive(Debug, Clone, Copy)]
pub(crate) enum SlpCut {
    /// Scalar-tangential path-jerk cut (Lee 2024 §III–§IV; spec §11).
    PathJerk { i: usize, b_bar: f64 },
    /// Per-axis Cartesian jerk cut at the verifier stencil (spec §11).
    AxisJerk(AxisJerkCut),
}

/// Per-axis Cartesian jerk cut details. Spec §5; width-1 b-FD stencil.
#[derive(Debug, Clone, Copy)]
pub(crate) struct AxisJerkCut {
    pub i: usize,
    /// Retained for diagnostic / future telemetry; row coefficients depend only
    /// on `(cp, cpp, cppp)`, which the caller pre-extracts.
    #[allow(dead_code)]
    pub axis: usize,
    /// Controls FD shape and which `b`-variables the row touches.
    pub stencil: AxisJerkStencil,
    /// Iterate values for the three `b̄` indices the stencil reads.
    /// Interior: `[b̄_{i-1}, b̄_i, b̄_{i+1}]`.
    /// StartBoundary: `[b̄_0, b̄_1, b̄_2]`.
    /// EndBoundary: `[b̄_{n-3}, b̄_{n-2}, b̄_{n-1}]`.
    pub b_bars: [f64; 3],
    /// Under width-1 b-FD the cut row touches only `a_i`, never neighbours.
    pub a_bar_i: f64,
    /// Path derivatives at `s_i` along `axis`: `(c', c'', c''')`.
    pub cp: f64,
    pub cpp: f64,
    pub cppp: f64,
    /// `j_max[axis] · target_ratio`, inflated by the SLP target-ratio schedule.
    pub j_lim_inflated: f64,
}

/// Discrete shape of the stencil under width-1 b-FD.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AxisJerkStencil {
    /// Central FD: touches `b_{i-1}, b_i, b_{i+1}, a_i`.
    Interior,
    /// Forward FD at i=0: touches `b_0, b_1, b_2, a_0`.
    StartBoundary,
    /// Backward FD at i=n-1: touches `b_{n-3}, b_{n-2}, b_{n-1}, a_{n-1}`.
    EndBoundary,
}

/// Result of a successful SOCP solve.
#[derive(Debug, Clone)]
pub(crate) struct SolverResult {
    /// `b_i = ṡ²` per grid point.
    pub b: Vec<f64>,
    /// `a_i = s̈_i` per grid point.
    pub a: Vec<f64>,
    pub status: SolverStatus,
}

/// Kalico-internal solver status (no Clarabel types escape this module).
#[derive(Debug, Clone, Copy)]
pub(crate) enum SolverStatus {
    Solved,
    SolvedInexact {
        residual: f64,
    },
    Infeasible,
    /// `residual = max(r_prim, r_dual)`.
    MaxIter {
        residual: f64,
    },
}

/// Error from solver setup (invalid bundle), not a runtime infeasibility.
#[derive(Debug, thiserror::Error)]
pub(crate) enum SolverSetupError {
    #[error("invalid constraint bundle: {0}")]
    InvalidBundle(String),
}

/// Zero upper-triangle CSC for Clarabel's quadratic term (pure linear objective).
fn build_p_zero(n_vars: usize) -> CscMatrix<f64> {
    CscMatrix::<f64> {
        m: n_vars,
        n: n_vars,
        colptr: vec![0usize; n_vars + 1],
        rowval: vec![],
        nzval: vec![],
    }
}

/// Convert each kalico `Cone` to the matching Clarabel cone.
/// Returns `SolverSetupError` for `RotatedSecondOrder` (not in Clarabel 0.11;
/// `build()` should never emit it).
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

/// Map Clarabel status → kalico status. `residual = max(r_prim, r_dual)`.
///
/// Exhaustive match against Clarabel 0.11.1 — a future version adding a new
/// variant will fail to compile, which is intentional.
fn map_status(status: ClarabelStatus, residual: f64) -> SolverStatus {
    match status {
        ClarabelStatus::Solved => SolverStatus::Solved,
        ClarabelStatus::AlmostSolved => SolverStatus::SolvedInexact { residual },
        // InsufficientProgress is "gave up", not "structurally infeasible".
        ClarabelStatus::MaxIterations
        | ClarabelStatus::MaxTime
        | ClarabelStatus::InsufficientProgress => SolverStatus::MaxIter { residual },
        // All infeasibility certificates, solver errors, and never-ran states:
        // no usable primal solution.
        ClarabelStatus::PrimalInfeasible
        | ClarabelStatus::DualInfeasible
        | ClarabelStatus::AlmostPrimalInfeasible
        | ClarabelStatus::AlmostDualInfeasible
        | ClarabelStatus::NumericalError
        | ClarabelStatus::CallbackTerminated
        | ClarabelStatus::Unsolved => SolverStatus::Infeasible,
    }
}

/// Slice the primal solution into `b` and `a`. Variable layout (pinned in
/// `constraints.rs`): `x[0..n_grid]` → `b_i`, `x[n_grid..2*n_grid]` → `a_i`.
fn extract_solution(x: &[f64], n_grid: usize, status: SolverStatus) -> SolverResult {
    let b: Vec<f64> = x[..n_grid].to_vec();
    let a: Vec<f64> = x[n_grid..2 * n_grid].to_vec();
    SolverResult { b, a, status }
}

/// Solve the original Consolini-Locatelli SOCP with no SLP cuts.
/// Kept as the unit-test entry point.
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
#[allow(clippy::too_many_arguments)]
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
    let alpha = j_path * h * h / (b_bar * sqrt_b); // J·h² / b̄^{3/2}
    let rhs = 3.0 * j_path * h * h / sqrt_b;

    // Sign-convention: A_clarabel = -A_k; negation baked into push_nz values.
    let bm1 = i - 1;
    let bi = i;
    let bp1 = i + 1;
    debug_assert!(bp1 < n_grid, "SLP cut interior index out of range");

    // Positive side: A_k row = [b_{i-1}: -1, b_i: -α + 2, b_{i+1}: -1], rhs = +rhs.
    let pos_row = *n_rows;
    push_nz(rowval, nzval, bm1, pos_row, -(-1.0));
    push_nz(rowval, nzval, bi, pos_row, -(2.0 - alpha));
    push_nz(rowval, nzval, bp1, pos_row, -(-1.0));
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
/// (positive- and negative-side ⇒ |j_axis| ≤ j_max·(1+ε)). Spec §5;
/// width-1 b-FD stencil.
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
#[allow(clippy::too_many_lines)]
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

    // Variable layout (pinned in constraints.rs): b at 0..n_grid, a at n_grid..2*n_grid.
    let off_b = 0usize;
    let off_a = n_grid;

    // b_anchor floored at SLP_B_FLOOR to keep 1/√b̄ bounded.
    let (alpha_b_anchor, entries_extra, k_const): (f64, [(usize, f64); 3], f64) = match cut.stencil
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
            let alpha_b_i = 1.5 * cppp * s + 3.0 * cpp * a_i / (2.0 * s) - cp * s / (h * h)
                + cp * d2 / (4.0 * h * h * s);
            let k = -0.5 * cppp * s3 - 1.5 * cpp * a_i * s - cp * d2 * s / (4.0 * h * h);
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
            let k = -0.5 * cppp * s3 - 1.5 * cpp * a_0 * s - cp * d2 * s / (4.0 * h * h);
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
            let k = -0.5 * cppp * s3 - 1.5 * cpp * a_nm1 * s - cp * d2 * s / (4.0 * h * h);
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

    let anchor_b_col = off_b + i;

    // Sign-convention: A_clarabel = -A_k.
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

/// Floor on |ā_i| for the a-trust-region radius. Without a positive floor,
/// near-zero iterates give a zero-width TR that pins `a_i` in place; the
/// floor lets the SOCP swing `a` across the acceleration range. Tuned against
/// fixture 4; revisit if observed to over-relax.
const A_TR_FLOOR: f64 = 5_000.0; // mm/s² (≈ a_max)

/// Floor on `b̄_i` for the b-trust-region radius. Prevents near-zero iterate
/// values (boundary ramp-up, cruise drop) from producing a near-zero TR that
/// Clarabel can't satisfy against centripetal caps. (50 mm/s)² = 2500.
const B_TR_FLOOR: f64 = 2_500.0;

/// Solve the SOCP with optional SLP cut rows; no trust region.
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

    let mut cones_clarabel = map_clarabel_cones(bundle)?;
    let cut_rows = 2 * cuts.len();
    if cut_rows > 0 {
        cones_clarabel.push(NonnegativeConeT(cut_rows));
    }
    // Trust-region rows: 2 per interior b_i (boundary b pinned by block (a)),
    // 2 per a_i.
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

    // Build A column-bucketed so SLP-cut rows can be appended without reshuffling.
    let mut rowval_per_col: Vec<Vec<usize>> = vec![Vec::new(); n_vars];
    let mut nzval_per_col: Vec<Vec<f64>> = vec![Vec::new(); n_vars];
    let mut n_rows = 0_usize;

    for row in &bundle.a_rows {
        for (col, &v) in row.iter().enumerate() {
            if v != 0.0 {
                rowval_per_col[col].push(n_rows);
                nzval_per_col[col].push(-v); // sign-convention: A_clarabel = -A_k
            }
        }
        n_rows += 1;
    }

    let mut b_rhs: Vec<f64> = bundle.b_rhs.clone();
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

    if let Some(tr) = trust_region {
        debug_assert_eq!(b_bar.len(), n_grid);
        debug_assert_eq!(a_bar.len(), n_grid);
        let off_b = 0;
        for i in 1..n_grid.saturating_sub(1) {
            let bb = b_bar[i].max(0.0);
            let radius = tr.rho_b * bb.max(B_TR_FLOOR);
            let lo = bb - radius;
            let hi = bb + radius;
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

    let p_zero = build_p_zero(n_vars);
    let q: &[f64] = &bundle.objective;

    // Spec §4.2: verbose=false (diagnostics via kalico telemetry).
    // max_iter=1000: SLP-cut SOCPs condition more tightly than the base SOCP;
    //   the default 200-iter budget produces `InsufficientProgress` on the
    //   CL-2024 counterexample fixture. 1000 is sufficient; runtime stays
    //   ≤ 1 s per outer iter at N=200.
    // reduced_tol_*=1e-3: lets Clarabel report `AlmostSolved` (→ SolvedInexact)
    //   when the primary tol can't be met; dropping these restores Clarabel
    //   defaults (5e-5/5e-5/1e-4) and silently changes `AlmostSolved` semantics.
    // direct_solve_method="qdldl", max_threads=1: determinism pin — single-
    //   threaded QDLDL backend keeps the joining-loop early-bail deterministic
    //   across future Clarabel versions / feature-flag changes.
    #[allow(clippy::similar_names)]
    let settings = DefaultSettings::<f64> {
        verbose: false,
        max_iter: 1000,
        tol_gap_abs: tol,
        tol_gap_rel: tol,
        tol_feas: tol,
        reduced_tol_gap_abs: 1e-3,
        reduced_tol_gap_rel: 1e-3,
        reduced_tol_feas: 1e-3,
        direct_solve_method: "qdldl".to_string(),
        max_threads: 1,
        ..Default::default()
    };

    let mut solver = DefaultSolver::new(&p_zero, q, &a_csc, &b_rhs, &cones_clarabel, settings)
        .map_err(|e| SolverSetupError::InvalidBundle(e.to_string()))?;
    solver.solve();

    let soln = &solver.solution;
    let residual = soln.r_prim.max(soln.r_dual);
    let status = map_status(soln.status, residual);
    Ok(extract_solution(&soln.x, n_grid, status))
}

// SLP outer iteration (Lee 2024 §III–§IV).
//
// The CL-2024 SOCP relaxation is demonstrably loose on curved high-jerk-load
// segments (e.g. R=20 mm 90° arc, N=200, J=1e5). The non-convex constraint
// `|b''|·√b ≤ 2J` cannot be tightened inside one SOCP. Lee 2024's mitigation:
// append a first-order Taylor cut on `1/√b` at the current iterate and re-solve.
// Each inner SOCP stays convex; the cut lies below the convex-down `1/√b` curve
// and thus tightens the relaxation.
//
// Cut placement: active-set-only oscillates (cuts at one index unmask new
// violators elsewhere). Cumulative cuts wreck Clarabel conditioning after
// ~50 rows at N=200. **Full-grid linearization** — rebuild fresh cuts at
// every interior grid point each iteration — converges in 1–3 iters;
// row count is bounded at N−2 (replaced, not accumulated).

/// Hard cap; Lee 2024 reports ~5–30 iterations in practice.
const SLP_MAX_OUTER_ITERS: u32 = 50;

/// Looser than `verify::EPS_FEAS` (spec §6.2): the SLP predicate uses a
/// raw FD estimate of `b''(s)` (`Δ²b/h²`), which is noisy near constraint-
/// switch kinks (1–2% spurious violations). Real violations on the CL-2024
/// counterexample are ~143% (ratio 2.43); 5% separates signal from noise.
/// The post-solve verifier at `EPS_FEAS = 1e-3` remains the authority.
const SLP_EPS_FEAS: f64 = 5e-2;

/// Avoids `1/√0` in the path-jerk linearization. Physically irrelevant at
/// these speeds (cut trivially satisfied; violation predicate requires `b > 0`).
const SLP_B_FLOOR: f64 = 1.0;

/// Below this `b̄` a violator does not receive a cut. `α = J·h²/b̄^{3/2}`
/// diverges as b̄ → 0, producing very steep rows that wreck the next inner
/// SOCP's conditioning. Near-boundary small-b points are dominated by the
/// boundary equality / centripetal cap in the base relaxation. ≈ (10 mm/s)².
const SLP_B_CUT_FLOOR: f64 = 100.0;

/// Warm-up before divergence rule fires; iterates routinely bounce for
/// several iterations before settling (Lee 2024: 5–30 iterations typical).
const SLP_WARMUP_ITERS: u32 = 8;

/// Required best-so-far improvement across the trailing window. Per-iterate
/// monotone descent is too strict (one cut can unmask new violators); the
/// best-so-far sliding-window signal is more robust.
const SLP_MIN_IMPROVEMENT: f64 = 0.01;

/// Sliding window length for the no-improvement divergence rule.
const SLP_NO_IMPROVEMENT_WINDOW: usize = 10;

/// Outcome of the SLP outer iteration.
#[derive(Debug, Clone, Copy)]
pub(crate) enum SlpOutcome {
    /// No violators within `SLP_EPS_FEAS`. `outer_iters = 0` means the
    /// base SOCP was already feasible (common straight-line case).
    Converged { outer_iters: u32 },
    /// Hit `SLP_MAX_OUTER_ITERS` without feasibility.
    MaxIters { last_max_ratio: f64 },
    /// Best-so-far ratio failed to improve over the sliding window.
    Diverged {
        last_max_ratio: f64,
        outer_iters: u32,
    },
    /// Inner SOCP returned Infeasible or MaxIter; no usable primal to iterate.
    InnerSolverFailure,
}

/// Run the path-jerk SLP outer loop (Lee 2024 §III–§IV). Returns the best
/// `SolverResult` and an `SlpOutcome`. Iteration 0 is the base CL-2024 SOCP;
/// subsequent iterations rebuild full-grid `1/√b` cuts at the latest iterate.
/// Straight-line inputs see no SLP overhead (`outer_iters = 0`).
pub(crate) fn slp_solve(
    bundle: &ConstraintBundle,
    tol: f64,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let h = bundle.h;
    let j_path = bundle.j_path;
    debug_assert!(h > 0.0 && j_path > 0.0);

    let mut cuts: Vec<SlpCut> = Vec::new();
    let mut last_result = solve_with_cuts(bundle, &cuts, tol)?;

    // No usable primal at iter 0; surface immediately.
    if matches!(
        last_result.status,
        SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
    ) {
        return Ok((last_result, SlpOutcome::InnerSolverFailure));
    }

    let mut violators = find_jerk_violators(&last_result.b, h, j_path);
    if violators.is_empty() {
        return Ok((last_result, SlpOutcome::Converged { outer_iters: 0 }));
    }

    // Track best-so-far (lowest max-violator ratio) so we surface the most-
    // feasible primal even when the loop terminates without convergence.
    let mut best_result = last_result.clone();
    let mut best_ratio_so_far = max_ratio(&violators);
    let mut max_ratio_history: Vec<f64> = Vec::new();
    let mut best_ratio_history: Vec<f64> = Vec::new();
    let initial_max = max_ratio(&violators);
    max_ratio_history.push(initial_max);
    best_ratio_history.push(initial_max);
    for outer in 1..=SLP_MAX_OUTER_ITERS {
        // Full-grid linearization: rebuild cuts at every interior point each
        // pass to avoid active-set oscillation (see module-level comment).
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
            // All remaining violators below the cut floor; dominated by other
            // constraints in the relaxation.
            return Ok((
                best_result,
                SlpOutcome::MaxIters {
                    last_max_ratio: best_ratio_so_far,
                },
            ));
        }

        let new_result = solve_with_cuts(bundle, &cuts, tol)?;
        // Inner solve failed: new primal not trustworthy; return best-so-far.
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

/// One violator of the path-jerk constraint.
#[derive(Debug, Clone, Copy)]
pub(crate) struct JerkViolator {
    /// Unused under full-grid-linearization; retained for future telemetry.
    #[allow(dead_code)]
    pub i: usize,
    /// `|Δ²b_i|·√b_i / (2J·h²)`; `> 1 + ε` for a violator.
    pub ratio: f64,
}

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
            continue; // √b undefined
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

// Per-axis Cartesian jerk SLP outer loop (spec §11).
//
// Path-jerk feasibility (`|s⃛| ≤ J_path`) is necessary but not sufficient:
// hardware checks per-axis `j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛`.
// On curved paths the path-jerk-feasible iterate can violate per-axis jerk
// by tens of percent. Mitigation: a second SLP loop using verifier-stencil
// cuts on j_axis.
//
// Because `j_axis` has bilinear-times-sqrt cross-terms (`a·√b`), the Hessian
// is indefinite — the linearization is only locally valid. Three mechanisms:
//
// 1. Active-set placement: cut only at (i, axis) pairs with ratio > 1 + ε.
// 2. L∞ trust region: ρ_b = 0.05, ρ_a = 0.10 (tighter than 0.20/0.30 from
//    initial diagnosis; larger radii let the cross-terms invalidate the cut).
//    Expand 1.5× on accept, contract 0.5× on reject.
//    Caps: ρ_b ∈ [0.005, 0.20], ρ_a ∈ [0.01, 0.40].
// 3. Accept-only-if-decrease: up to MAX_BACKTRACKS=3 trust-region halvings
//    per outer iter before falling back to solving without a TR.
//
// Source: Nocedal & Wright §18.5 (trust-region SQP).

const SLP9_MAX_OUTER_ITERS: u32 = 30;
const SLP9_WARN_AT_ITER: u32 = 15;

/// Matches `verify::EPS_FEAS = 1e-3` (spec §6.2): cut and verifier use the
/// same stencil, so tolerances can align directly.
const SLP9_EPS_FEAS: f64 = 1e-3;

/// Tightened from initial diagnosis's 0.20/0.30: larger radii let the inner
/// SOCP move far enough that the `a·√b` cross-terms invalidate the cut.
/// 0.05/0.10 keeps the iterate in the local-validity neighborhood.
const SLP9_RHO_B_INIT: f64 = 0.05;
const SLP9_RHO_A_INIT: f64 = 0.10;
const SLP9_RHO_B_MIN: f64 = 0.005;
const SLP9_RHO_B_MAX: f64 = 0.20;
const SLP9_RHO_A_MIN: f64 = 0.01;
const SLP9_RHO_A_MAX: f64 = 0.40;

/// Max trust-region halvings per outer iter before treating the step as a no-op.
const SLP9_MAX_BACKTRACKS: u32 = 3;

/// Homotopy schedule: cut RHS = `j_max · max(1+ε, R_k · decay)`. Cuts start
/// loose enough to admit the current iterate and tighten each outer iter.
/// 0.85 (gentle) vs 0.5 (aggressive): at R≈1.24 (fixture 4), decay=0.5 clamps
/// target below the iterate, making the cut infeasible inside the TR. ~30 iters
/// at 0.85 vs divergence at 0.5.
const SLP9_TARGET_DECAY: f64 = 0.85;

/// Path-jerk SLP (stage 1) then per-axis-jerk SLP (stage 2; spec §11).
/// Path-jerk failures short-circuit stage 2. Returns the best iterate and the
/// worst-case `SlpOutcome` across both stages.
#[allow(clippy::too_many_lines)]
pub(crate) fn slp_solve_with_axis_jerk(
    bundle: &ConstraintBundle,
    grid: &crate::topp::path::ArclengthGrid,
    limits: &crate::Limits,
    tol: f64,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let (path_result, path_outcome) = slp_solve(bundle, tol)?;

    if matches!(
        path_outcome,
        SlpOutcome::InnerSolverFailure | SlpOutcome::Diverged { .. } | SlpOutcome::MaxIters { .. }
    ) {
        return Ok((path_result, path_outcome));
    }

    debug_assert_eq!(grid.s.len(), path_result.b.len());

    let mut last_result = path_result.clone();
    // Carried into the final SlpOutcome::Converged so the caller sees total work.
    let path_outer_iters = match path_outcome {
        SlpOutcome::Converged { outer_iters } => outer_iters,
        _ => 0,
    };

    let initial_max = max_axis_ratio(&last_result, grid, limits, bundle.h);
    if initial_max <= 1.0 + SLP9_EPS_FEAS {
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
            eprintln!(
                "slp9 warning: per-axis SLP not converged at iter {outer} \
                 (best ratio = {best_ratio:.4})",
            );
        }

        let target_ratio = (best_ratio * SLP9_TARGET_DECAY).max(1.0 + SLP9_EPS_FEAS);
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
                continue; // inner SOCP failed at this TR size; halve and retry
            }
            let cand_ratio = max_axis_ratio(&candidate, grid, limits, bundle.h);
            if cand_ratio < best_ratio {
                accepted = Some(candidate);
                best_ratio = cand_ratio;
                break;
            }
        }
        if accepted.is_none() {
            // Fallback: solve without TR. Common on the first per-axis iter
            // when the path-jerk iterate is far outside per-axis feasibility
            // (fixture 4 boundary, ratio = 185×): no point inside the TR
            // satisfies the cut. Still gate acceptance on max-ratio decrease.
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
            rho_b = (rho_b * 0.5).max(SLP9_RHO_B_MIN);
            rho_a = (rho_a * 0.5).max(SLP9_RHO_A_MIN);
            // At minimum TR and still no step: declare divergence.
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

/// Max per-axis jerk ratio over the whole grid, using the same stencil as
/// `verify::check` (`topp::stencil::s_dddot_at`).
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

/// Build active-set per-axis jerk cuts: one `SlpCut::AxisJerk` per (i, axis)
/// pair with ratio > 1 + SLP9_EPS_FEAS. `target_ratio` inflates the cut RHS
/// so the current iterate is not immediately infeasible (see `SLP9_TARGET_DECAY`).
/// Same stencil as `verify::check`; see `append_axis_jerk_cut_to_clarabel`
/// for row-coefficient algebra.
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
                AxisJerkStencil::EndBoundary => [result.b[n - 3], result.b[n - 2], result.b[n - 1]],
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

#[cfg(test)]
mod tests;
