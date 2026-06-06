//! Clarabel SOCP construction + solve. INTERNAL — Clarabel types do not
//! escape this module.
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

// clippy::doc_markdown fires on unicode-math and CamelCase names in docs here.
#![allow(clippy::doc_markdown)]

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
///   the relaxation.
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
    PathJerk { i: usize, b_bar: f64 },
    AxisJerk(AxisJerkCut),
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct AxisJerkCut {
    pub i: usize,
    #[allow(dead_code)]
    pub axis: usize,
    pub stencil: AxisJerkStencil,
    /// Iterate `b̄` values for the three stencil indices.
    /// Interior: `[b̄_{i-1}, b̄_i, b̄_{i+1}]`.
    /// StartBoundary: `[b̄_0, b̄_1, b̄_2]`.
    /// EndBoundary: `[b̄_{n-3}, b̄_{n-2}, b̄_{n-1}]`.
    pub b_bars: [f64; 3],
    pub a_bar_i: f64,
    pub cp: f64,
    pub cpp: f64,
    pub cppp: f64,
    /// `j_max[axis] · target_ratio`.
    pub j_lim_inflated: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum AxisJerkStencil {
    Interior,
    StartBoundary,
    EndBoundary,
}

#[derive(Debug, Clone)]
pub(crate) struct SolverResult {
    pub b: Vec<f64>,
    pub a: Vec<f64>,
    pub status: SolverStatus,
}

#[derive(Debug, Clone, Copy)]
pub(crate) enum SolverStatus {
    Solved,
    SolvedInexact { residual: f64 },
    Infeasible,
    MaxIter { residual: f64 },
}

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

/// Exhaustive match against Clarabel 0.11.1; a new variant will fail to compile.
fn map_status(status: ClarabelStatus, residual: f64) -> SolverStatus {
    match status {
        ClarabelStatus::Solved => SolverStatus::Solved,
        ClarabelStatus::AlmostSolved => SolverStatus::SolvedInexact { residual },
        ClarabelStatus::MaxIterations
        | ClarabelStatus::MaxTime
        | ClarabelStatus::InsufficientProgress => SolverStatus::MaxIter { residual },
        ClarabelStatus::PrimalInfeasible
        | ClarabelStatus::DualInfeasible
        | ClarabelStatus::AlmostPrimalInfeasible
        | ClarabelStatus::AlmostDualInfeasible
        | ClarabelStatus::NumericalError
        | ClarabelStatus::CallbackTerminated
        | ClarabelStatus::Unsolved => SolverStatus::Infeasible,
    }
}

/// Variable layout (pinned in `constraints.rs`): `x[0..n_grid]` → `b_i`,
/// `x[n_grid..2*n_grid]` → `a_i`.
fn extract_solution(x: &[f64], n_grid: usize, status: SolverStatus) -> SolverResult {
    let b: Vec<f64> = x[..n_grid].to_vec();
    let a: Vec<f64> = x[n_grid..2 * n_grid].to_vec();
    SolverResult { b, a, status }
}

#[allow(dead_code)]
pub(crate) fn solve(bundle: &ConstraintBundle) -> Result<SolverResult, SolverSetupError> {
    solve_with_cuts(bundle, &[], 1e-8)
}

/// Append one path-jerk SLP cut as two `Nonneg` rows.
///
/// First-order Taylor of `f(b) = 1/√b` at iterate `b̄`:
/// ```text
/// f(b) ≈ 1/√b̄ − (b − b̄) / (2·b̄^{3/2}).
/// ```
/// `f` is convex (`f'' > 0`), so the tangent lies below the curve everywhere.
/// Constant term: `3J·h²/√b̄`.  Let `α := J·h²/b̄^{3/2}`.
///
/// Rows in `A·x + b_rhs ≥ 0` form:
/// ```text
/// (+):  3J·h²/√b̄  − α·b_i − (b_{i-1} − 2·b_i + b_{i+1})  ≥ 0
/// (−):  3J·h²/√b̄  − α·b_i + (b_{i-1} − 2·b_i + b_{i+1})  ≥ 0
/// ```
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
    let alpha = j_path * h * h / (b_bar * sqrt_b);
    let rhs = 3.0 * j_path * h * h / sqrt_b;

    // Sign-convention: A_clarabel = -A_k; negation baked into push_nz values.
    let bm1 = i - 1;
    let bi = i;
    let bp1 = i + 1;
    debug_assert!(bp1 < n_grid, "SLP cut interior index out of range");

    let pos_row = *n_rows;
    push_nz(rowval, nzval, bm1, pos_row, -(-1.0));
    push_nz(rowval, nzval, bi, pos_row, -(2.0 - alpha));
    push_nz(rowval, nzval, bp1, pos_row, -(-1.0));
    b_rhs.push(rhs);
    *n_rows += 1;

    let neg_row = *n_rows;
    push_nz(rowval, nzval, bm1, neg_row, -(1.0));
    push_nz(rowval, nzval, bi, neg_row, -(-alpha - 2.0));
    push_nz(rowval, nzval, bp1, neg_row, -(1.0));
    b_rhs.push(rhs);
    *n_rows += 1;
}

/// Append one per-axis Cartesian jerk SLP cut as two `Nonneg` rows.
///
/// First-order Taylor linearization of `j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛`
/// at iterate `(b̄, ā)` under width-1 b-FD. Row coefficients per stencil case:
///
/// **Interior** (touches `b_{i-1}, b_i, b_{i+1}, a_i`):
/// ```text
///   α_b_im1  = c'·S / (2h²)
///   α_b_ip1  = c'·S / (2h²)
///   α_b_i    = (3/2)·c'''·S + 3·c''·ā_i/(2·S) − c'·S/h² + c'·D₂/(4h²·S)
///   α_a_i    = 3·c''·S
///   K        = −(1/2)·c'''·S3 − (3/2)·c''·ā_i·S − c'·D₂·S/(4h²)
/// ```
///
/// **StartBoundary** and **EndBoundary** use the same closed-form K with
/// the stencil-specific S, ā, and D₂. Identity verified by
/// `tests/step9_cut_identity.rs`.
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
    let pos_row = *n_rows;
    push_nz(rowval, nzval, anchor_b_col, pos_row, alpha_b_anchor);
    for &(col, alpha) in &entries_extra {
        if alpha != 0.0 {
            push_nz(rowval, nzval, col, pos_row, alpha);
        }
    }
    b_rhs.push(j - k_const);
    *n_rows += 1;

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
/// Box rows enforce
/// `b̄_i·(1−ρ_b) ≤ b_i ≤ b̄_i·(1+ρ_b)` and
/// `ā_i ± ρ_a·max(|ā_i|, A_TR_FLOOR)` for all interior grid points.
/// Boundary `b` rows are skipped — block (a) pins them exactly.
#[derive(Debug, Clone, Copy)]
pub(crate) struct TrustRegion {
    pub rho_b: f64,
    pub rho_a: f64,
}

/// Floor on |ā_i| for the a-trust-region radius: prevents near-zero iterates
/// from producing a zero-width TR that pins `a` in place. ≈ a_max.
const A_TR_FLOOR: f64 = 5_000.0;

/// Floor on `b̄_i` for the b-trust-region radius. Prevents near-zero iterates
/// from producing a near-zero TR Clarabel can't satisfy against centripetal
/// caps. (50 mm/s)² = 2500.
const B_TR_FLOOR: f64 = 2_500.0;

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

    // verbose=false: diagnostics via kalico telemetry.
    // max_iter=1000: SLP-cut SOCPs condition more tightly than the base SOCP;
    //   200 iters produces InsufficientProgress on the CL-2024 counterexample.
    // reduced_tol_*=1e-3: lets Clarabel report AlmostSolved; dropping these
    //   restores Clarabel defaults and silently changes AlmostSolved semantics.
    // direct_solve_method="qdldl", max_threads=1: determinism pin — single-
    //   threaded QDLDL keeps the joining-loop early-bail deterministic.
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
// segments. Lee 2024's mitigation: append a first-order Taylor cut on `1/√b`
// at the current iterate and re-solve. Each inner SOCP stays convex; the cut
// lies below the convex-down `1/√b` curve and thus tightens the relaxation.
//
// Cut placement: full-grid linearization — rebuild fresh cuts at every interior
// grid point each iteration — converges in 1–3 iters; row count is bounded at
// N−2 (replaced, not accumulated).

/// Hard cap; Lee 2024 reports ~5–30 iterations in practice.
const SLP_MAX_OUTER_ITERS: u32 = 50;

/// Looser than `verify::EPS_FEAS`: the SLP predicate uses a raw FD estimate
/// of `b''(s)`, which is noisy near constraint-switch kinks (~1–2% spurious
/// violations). Real violations are ~143% on the CL-2024 counterexample.
const SLP_EPS_FEAS: f64 = 5e-2;

/// Avoids `1/√0` in the path-jerk linearization.
const SLP_B_FLOOR: f64 = 1.0;

/// Below this `b̄` a violator does not receive a cut. `α = J·h²/b̄^{3/2}`
/// diverges as b̄ → 0, producing steep rows that wreck the inner SOCP's
/// conditioning. ≈ (10 mm/s)².
const SLP_B_CUT_FLOOR: f64 = 100.0;

/// Warm-up before divergence rule fires; iterates routinely bounce for several
/// iterations before settling (Lee 2024: 5–30 typical).
const SLP_WARMUP_ITERS: u32 = 8;

/// Required best-so-far improvement across the trailing window.
const SLP_MIN_IMPROVEMENT: f64 = 0.01;

/// Sliding window length for the no-improvement divergence rule.
const SLP_NO_IMPROVEMENT_WINDOW: usize = 10;

#[derive(Debug, Clone, Copy)]
pub(crate) enum SlpOutcome {
    /// No violators within `SLP_EPS_FEAS`. `outer_iters = 0` means the
    /// base SOCP was already feasible.
    Converged {
        outer_iters: u32,
    },
    MaxIters {
        last_max_ratio: f64,
    },
    Diverged {
        last_max_ratio: f64,
        outer_iters: u32,
    },
    InnerSolverFailure,
}

/// Run the path-jerk SLP outer loop. Returns the best `SolverResult` and an
/// `SlpOutcome`. Iteration 0 is the base CL-2024 SOCP; subsequent iterations
/// rebuild full-grid `1/√b` cuts at the latest iterate.
pub(crate) fn slp_solve(
    bundle: &ConstraintBundle,
    tol: f64,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let h = bundle.h;
    let j_path = bundle.j_path;
    debug_assert!(h > 0.0 && j_path > 0.0);

    let mut cuts: Vec<SlpCut> = Vec::new();
    let mut last_result = solve_with_cuts(bundle, &cuts, tol)?;

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

    let mut best_result = last_result.clone();
    let mut best_ratio_so_far = max_ratio(&violators);
    let mut max_ratio_history: Vec<f64> = Vec::new();
    let mut best_ratio_history: Vec<f64> = Vec::new();
    let initial_max = max_ratio(&violators);
    max_ratio_history.push(initial_max);
    best_ratio_history.push(initial_max);
    for outer in 1..=SLP_MAX_OUTER_ITERS {
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
            return Ok((
                best_result,
                SlpOutcome::MaxIters {
                    last_max_ratio: best_ratio_so_far,
                },
            ));
        }

        let new_result = solve_with_cuts(bundle, &cuts, tol)?;
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
        let _ = cur_best;

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

#[derive(Debug, Clone, Copy)]
pub(crate) struct JerkViolator {
    #[allow(dead_code)]
    pub i: usize,
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
            continue;
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

// Per-axis Cartesian jerk SLP outer loop.
//
// `j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛` has bilinear-times-sqrt
// cross-terms (indefinite Hessian). Three mechanisms keep the SLP converging:
//
// 1. Active-set placement: cut only at (i, axis) pairs with ratio > 1 + ε.
// 2. L∞ trust region: ρ_b = 0.05, ρ_a = 0.10. Expand 1.5× on accept,
//    contract 0.5× on reject. Caps: ρ_b ∈ [0.005, 0.20], ρ_a ∈ [0.01, 0.40].
// 3. Accept-only-if-decrease: up to MAX_BACKTRACKS=3 trust-region halvings
//    per outer iter before falling back to solving without a TR.

const SLP9_MAX_OUTER_ITERS: u32 = 30;
const SLP9_WARN_AT_ITER: u32 = 15;

/// Matches `verify::EPS_FEAS_JERK` and `SLP_EPS_FEAS`: the per-axis jerk ratio
/// uses the width-1 b-FD `s_dddot_at` stencil whose discretization noise on a
/// time-optimal profile riding the jerk limit is 1–4% on micro-segments
/// (n=20, h=32µm). 0.1% was inside the stencil noise floor and caused phantom
/// `Diverged` on the stub-segment pathological instance.
const SLP9_EPS_FEAS: f64 = 5e-2;

/// 0.05/0.10 keeps the iterate in the local-validity neighborhood of the
/// `a·√b` cross-term linearization.
const SLP9_RHO_B_INIT: f64 = 0.05;
const SLP9_RHO_A_INIT: f64 = 0.10;
const SLP9_RHO_B_MIN: f64 = 0.005;
const SLP9_RHO_B_MAX: f64 = 0.20;
const SLP9_RHO_A_MIN: f64 = 0.01;
const SLP9_RHO_A_MAX: f64 = 0.40;

const SLP9_MAX_BACKTRACKS: u32 = 3;

/// Homotopy schedule: cut RHS = `j_max · max(1+ε, R_k · decay)`. 0.85 (gentle)
/// vs 0.5 (aggressive): at R≈1.24, decay=0.5 clamps target below the iterate.
const SLP9_TARGET_DECAY: f64 = 0.85;

/// Path-jerk SLP (stage 1) then per-axis-jerk SLP (stage 2). Path-jerk
/// failures short-circuit stage 2.
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
                continue;
            }
            let cand_ratio = max_axis_ratio(&candidate, grid, limits, bundle.h);
            if cand_ratio < best_ratio {
                accepted = Some(candidate);
                best_ratio = cand_ratio;
                break;
            }
        }
        if accepted.is_none() {
            // Fallback: solve without TR. Common on the first per-axis iter when
            // the path-jerk iterate is far outside per-axis feasibility — no
            // point inside the TR satisfies the cut.
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
