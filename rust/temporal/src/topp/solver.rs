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
//! `RotatedSecondOrderConeT` does not exist in Clarabel 0.11. `constraints::build_chain()`
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
use crate::topp::scaling::SolverScale;

/// One linearized Taylor cut produced by the SLP outer loop.
///
/// - `PathJerkWeights`: weight-based path-jerk cut. `b″ = Σ_k w_k·b_{idx[k]}`
///   (from `b_dd_weights`); rows scaled by `h̄²` so they are O(1) regardless of
///   grid refinement. For uniform spacing the row reduces bit-exactly to the
///   legacy 3-point second-difference form.
///
/// - `AxisJerk`: per-axis Cartesian jerk cut linearizing
///   `j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛` at the iterate. The
///   cross-term `a·√b` is bilinear-times-sqrt (indefinite Hessian on `(a,b)`),
///   so the cut is a LOCAL approximation only. The L∞ trust region + accept-
///   only-if-decrease backtracking in `slp_solve_with_axis_jerk_chain` is what
///   makes the SLP converge despite the non-convex linearization.
///   Numerical identity check: `tests/step9_cut_identity.rs`.
#[derive(Debug, Clone, Copy)]
pub(crate) enum SlpCut {
    /// Weight-based path-jerk cut. Rows scaled by `h̄²` to stay O(1).
    PathJerkWeights {
        i: usize,
        b_bar: f64,
        j_path: f64,
        idx: [usize; 3],
        w: [f64; 3],
        h_bar: f64,
    },
    AxisJerk(AxisJerkCut),
}

#[derive(Debug, Clone, Copy)]
pub(crate) struct AxisJerkCut {
    /// Grid index this cut is anchored at. The anchor is `idx[anchor_pos]`.
    pub i: usize,
    #[allow(dead_code)]
    pub axis: usize,
    /// Column indices for the three stencil b-variables (from `stencil::stencil_at`).
    pub idx: [usize; 3],
    /// b″ weights matching `idx` order (from `stencil::b_dd_weights(hl, hr)`).
    pub w: [f64; 3],
    /// Iterate `b̄` values in stencil order `[b̄_{idx[0]}, b̄_{idx[1]}, b̄_{idx[2]}]`.
    pub b_bars: [f64; 3],
    pub a_bar_i: f64,
    pub cp: f64,
    pub cpp: f64,
    pub cppp: f64,
    /// `j_max[axis] · target_ratio`.
    pub j_lim_inflated: f64,
}

/// Gradient of `j_axis` at iterate `(b̄, ā)` w.r.t. stencil b-values and `a_i`.
/// Used by the cut appender and exposed for numerical identity tests.
pub struct AxisJerkGradient {
    /// Gradient w.r.t. `b̄` in stencil order `[∂/∂b̄_{idx[0]}, ..., ∂/∂b̄_{idx[2]}]`.
    pub b: [f64; 3],
    /// Gradient w.r.t. `ā_i`.
    pub a: f64,
}

/// Test-support export: computes the linearization coefficients (= gradient of
/// `j_axis`) for an interior (anchor_pos=1) stencil with the given non-uniform
/// spacings and `b_floor = 0`.
pub fn axis_jerk_gradient_for_test(
    b_bars: &[f64; 3],
    a_bar: f64,
    cp: f64,
    cpp: f64,
    cppp: f64,
    h_intervals: &[f64; 2],
) -> AxisJerkGradient {
    let w = crate::topp::stencil::b_dd_weights(h_intervals[0], h_intervals[1]);
    let b_anchor = b_bars[1].max(0.0);
    let s = b_anchor.sqrt();
    let b_dd = w[0] * b_bars[0] + w[1] * b_bars[1] + w[2] * b_bars[2];
    let anchor_coeff = 1.5 * cppp * s
        + 3.0 * cpp * a_bar / (2.0 * s.max(f64::MIN_POSITIVE))
        + cp * (w[1] * s / 2.0 + b_dd / (4.0 * s.max(f64::MIN_POSITIVE)));
    AxisJerkGradient {
        b: [
            cp * s * w[0] / 2.0,
            anchor_coeff,
            cp * s * w[2] / 2.0,
        ],
        a: 3.0 * cpp * s,
    }
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
/// `build_chain()` should never emit it).
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
                     constraints::build_chain() should never emit it"
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
    solve_with_cuts(bundle, &[], 1e-8, &SolverScale::identity())
}

/// Append one per-axis Cartesian jerk SLP cut as two `Nonneg` rows.
///
/// Unified first-order Taylor linearization of `j_axis = c'''·b^(3/2) + 3·c''·a·√b + c'·s⃛`
/// at iterate `(b̄, ā)`. Uses weight-based formula (3b): `b̄″ = w·b̄` (dot over stencil triple),
/// `S = √(max(b̄_anchor, b_floor))`, `anchor_pos = idx.iter().position(|&x| x == i)`.
///
/// ```text
///   coeff on b at idx[k], k ≠ anchor_pos:   c'·S·w[k]/2
///   coeff on b at anchor:  (3/2)·c'''·S + 3·c''·ā/(2S) + c'·(w[anchor]·S/2 + b̄″/(4S))
///   coeff on a_i:          3·c''·S
///   K:  −(1/2)·c'''·S3 − (3/2)·c''·ā·S − c'·b̄″·S/4
/// ```
///
/// Identity verified (uniform w reproduces legacy 3-case formulas exactly) by
/// `tests/step9_cut_identity.rs`.
#[allow(clippy::too_many_arguments)]
fn append_axis_jerk_cut_to_clarabel(
    cut: &AxisJerkCut,
    b_floor: f64,
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

    let anchor_pos = cut
        .idx
        .iter()
        .position(|&x| x == i)
        .expect("cut.i must appear in cut.idx");

    let b_anchor = cut.b_bars[anchor_pos].max(b_floor);
    let s = b_anchor.sqrt();
    let s3 = b_anchor * s;
    let a_i = cut.a_bar_i;
    let b_dd = cut.w[0] * cut.b_bars[0] + cut.w[1] * cut.b_bars[1] + cut.w[2] * cut.b_bars[2];

    let s_safe = if s > 0.0 { s } else { f64::MIN_POSITIVE };
    let alpha_b_anchor = 1.5 * cppp * s
        + 3.0 * cpp * a_i / (2.0 * s_safe)
        + cp * (cut.w[anchor_pos] * s / 2.0 + b_dd / (4.0 * s_safe));
    let alpha_a_i = 3.0 * cpp * s;
    let k_const = -0.5 * cppp * s3 - 1.5 * cpp * a_i * s - cp * b_dd * s / 4.0;

    let entries_extra: [(usize, f64); 3] = {
        let mut entries = [(0usize, 0.0f64); 3];
        let mut extra_slot = 0;
        for k in 0..3 {
            if k == anchor_pos {
                continue;
            }
            let col = off_b + cut.idx[k];
            let coeff = cp * s * cut.w[k] / 2.0;
            entries[extra_slot] = (col, coeff);
            extra_slot += 1;
        }
        entries[2] = (off_a + i, alpha_a_i);
        entries
    };

    let anchor_b_col = off_b + i;

    // Row-∞-norm scaling: cp·√b/h² grows as O(N²) with grid refinement,
    // reaching 1.9e6 on fixture_4 (146-cut case) vs a-column coefficients ~10.
    // A 40 000:1 in-row spread causes QDLDL to return infeasible/maxiter on
    // every trust-region subproblem and stalls the SLP. Dividing every
    // coefficient and both RHS values by row_scale is a feasible-set-exact
    // transformation for Nonneg rows (positive scalar on a ≥ 0 constraint).
    let row_scale = entries_extra
        .iter()
        .map(|&(_, a)| a.abs())
        .fold(alpha_b_anchor.abs(), f64::max);

    if row_scale == 0.0 {
        // All coefficients are zero: vacuous constraint, skip both rows.
        return;
    }

    let alpha_b_anchor_s = alpha_b_anchor / row_scale;
    let entries_extra_s: [(usize, f64); 3] =
        entries_extra.map(|(col, a)| (col, a / row_scale));
    let rhs_pos = (j - k_const) / row_scale;
    let rhs_neg = (j + k_const) / row_scale;

    // Sign-convention: A_clarabel = -A_k.
    let pos_row = *n_rows;
    push_nz(rowval, nzval, anchor_b_col, pos_row, alpha_b_anchor_s);
    for &(col, alpha) in &entries_extra_s {
        if alpha != 0.0 {
            push_nz(rowval, nzval, col, pos_row, alpha);
        }
    }
    b_rhs.push(rhs_pos);
    *n_rows += 1;

    let neg_row = *n_rows;
    push_nz(rowval, nzval, anchor_b_col, neg_row, -alpha_b_anchor_s);
    for &(col, alpha) in &entries_extra_s {
        if alpha != 0.0 {
            push_nz(rowval, nzval, col, neg_row, -alpha);
        }
    }
    b_rhs.push(rhs_neg);
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
    scale: &SolverScale,
) -> Result<SolverResult, SolverSetupError> {
    solve_with_cuts_and_trust_region(bundle, cuts, None, &[], &[], tol, scale)
}

#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
fn solve_with_cuts_and_trust_region(
    bundle: &ConstraintBundle,
    cuts: &[SlpCut],
    trust_region: Option<TrustRegion>,
    b_bar: &[f64],
    a_bar: &[f64],
    tol: f64,
    scale: &SolverScale,
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
    debug_assert!(j_path > 0.0, "bundle must carry positive j_path");
    let b_floor = scale.to_scaled_b(SLP_B_FLOOR);
    for cut in cuts {
        match cut {
            SlpCut::PathJerkWeights {
                i,
                b_bar,
                j_path: cut_j_path,
                idx,
                w,
                h_bar: cut_h_bar,
            } => {
                let b_bar_floored = b_bar.max(b_floor);
                append_path_jerk_cut_weights(
                    *i,
                    b_bar_floored,
                    *cut_j_path,
                    *cut_h_bar,
                    *idx,
                    *w,
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
                    b_floor,
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
        let b_tr_floor = scale.to_scaled_b(B_TR_FLOOR);
        let a_tr_floor = scale.to_scaled_accel(A_TR_FLOOR);
        let off_b = 0;
        for i in 1..n_grid.saturating_sub(1) {
            let bb = b_bar[i].max(0.0);
            let radius = tr.rho_b * bb.max(b_tr_floor);
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
            let radius = tr.rho_a * ab.abs().max(a_tr_floor);
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
// The chain path emits weight-based cuts carrying h̄² so rows stay O(1).

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

#[derive(Debug, Clone, Copy)]
pub(crate) struct JerkViolator {
    #[allow(dead_code)]
    pub i: usize,
    pub ratio: f64,
}

#[inline]
fn max_ratio(vs: &[JerkViolator]) -> f64 {
    vs.iter().map(|v| v.ratio).fold(0.0_f64, f64::max)
}

/// Append one path-jerk SLP cut using weight-based b″ for non-uniform grids.
///
/// Row (sign-paired) scaled by `h̄²` so row magnitudes match the legacy uniform
/// row and remain O(1) regardless of grid refinement:
///   `3J·h̄²/√b̄ − α·h̄²·b_i − h̄²·Σ_k w_k·b_{idx[k]} ≥ 0`
/// where `α = J/b̄^{3/2}`.
///
/// For uniform spacing h̄=h the weights `w_k = b_dd_weights(h,h)` give
/// `h²·w_k = [1, -2, 1]`, reproducing the legacy `append_path_jerk_cut_to_clarabel`
/// row exactly. For non-uniform junctions this is a feasible-set-identical
/// positive scaling (`h̄² > 0`).
#[allow(clippy::too_many_arguments)]
fn append_path_jerk_cut_weights(
    i: usize,
    b_bar: f64,
    j_path: f64,
    h_bar: f64,
    idx: [usize; 3],
    w: [f64; 3],
    n_rows: &mut usize,
    rowval: &mut [Vec<usize>],
    nzval: &mut [Vec<f64>],
    b_rhs: &mut Vec<f64>,
    n_grid: usize,
) {
    debug_assert!(i < n_grid);
    debug_assert!(idx[0] < n_grid && idx[1] < n_grid && idx[2] < n_grid);
    debug_assert!(h_bar > 0.0);
    let h2 = h_bar * h_bar;
    let sqrt_b = b_bar.sqrt();
    let alpha = j_path * h2 / (b_bar * sqrt_b);
    let rhs = 3.0 * j_path * h2 / sqrt_b;

    // anchor_pos: which element of idx is i (for adding alpha to the right column).
    let anchor_pos = idx
        .iter()
        .position(|&x| x == i)
        .expect("i must appear in idx");

    // Sign-convention: A_clarabel = -A_k.
    // The two Nonneg rows are:
    //   (+): rhs - alpha·b_i - h²·b_dd ≥ 0
    //   (−): rhs - alpha·b_i + h²·b_dd ≥ 0
    // In Clarabel form A_clarabel·x + rhs ≥ 0:
    //   non-anchor k: A_clarabel[idx[k]] = ∓h²·w[k]  (- for Row+, + for Row-)
    //   anchor:       A_clarabel[idx[anchor]] = ∓h²·w[anchor] − alpha
    for &neg_b_dd in &[true, false] {
        let row = *n_rows;
        let sign_b_dd: f64 = if neg_b_dd { -1.0 } else { 1.0 };
        for k in 0..3 {
            let coeff = if k == anchor_pos {
                sign_b_dd * h2 * w[k] - alpha
            } else {
                sign_b_dd * h2 * w[k]
            };
            push_nz(rowval, nzval, idx[k], row, coeff);
        }
        b_rhs.push(rhs);
        *n_rows += 1;
    }
    let _ = n_grid;
}

/// Path-jerk violators using per-point `b_dd_weights` for non-uniform spacing.
pub(crate) fn find_jerk_violators_chain(
    b: &[f64],
    h_intervals: &[f64],
    j_path: f64,
) -> Vec<JerkViolator> {
    let n = b.len();
    if n < 3 {
        return Vec::new();
    }
    let mut out = Vec::new();
    for i in 1..n - 1 {
        let bi = b[i];
        if bi <= 0.0 {
            continue;
        }
        let (idx, hl, hr) = crate::topp::stencil::stencil_at(i, n, h_intervals);
        let w = crate::topp::stencil::b_dd_weights(hl, hr);
        let b_dd = w[0] * b[idx[0]] + w[1] * b[idx[1]] + w[2] * b[idx[2]];
        let ratio = b_dd.abs() * bi.sqrt() / (2.0 * j_path);
        if ratio > 1.0 + SLP_EPS_FEAS {
            out.push(JerkViolator { i, ratio });
        }
    }
    out
}

/// Per-axis ratio scan for a chain grid. Includes a second pass over junction
/// duals so both geometries at shared junction points are evaluated.
pub(crate) fn max_axis_ratio_chain(
    result: &SolverResult,
    chain: &crate::topp::chain::ChainGrid,
) -> f64 {
    let n = result.b.len();
    debug_assert_eq!(chain.s.len(), n);
    let mut worst: f64 = 0.0;
    for i in 0..n {
        let s_dddot = crate::topp::stencil::s_dddot_at_weights(&result.b, i, &chain.h_intervals);
        let s_dot = result.b[i].max(0.0).sqrt();
        let s_dot3 = s_dot * s_dot * s_dot;
        let s_ddot = result.a[i];
        let geom = &chain.geom[i];
        let lim = chain.limits_at(i);
        for ax in 0..3 {
            let cp = geom.c_prime[ax];
            let cpp = geom.c_double_prime[ax];
            let cppp = geom.c_triple_prime[ax];
            let j = cppp * s_dot3 + 3.0 * cpp * s_dot * s_ddot + cp * s_dddot;
            let ratio = j.abs() / lim.j_max[ax];
            if ratio > worst {
                worst = ratio;
            }
        }
    }
    for jct in &chain.junctions {
        let i = jct.idx;
        let s_dddot = crate::topp::stencil::s_dddot_at_weights(&result.b, i, &chain.h_intervals);
        let s_dot = result.b[i].max(0.0).sqrt();
        let s_dot3 = s_dot * s_dot * s_dot;
        let s_ddot = result.a[i];
        let geom = &jct.geom;
        let lim = &chain.limits[jct.limits_idx];
        for ax in 0..3 {
            let cp = geom.c_prime[ax];
            let cpp = geom.c_double_prime[ax];
            let cppp = geom.c_triple_prime[ax];
            let j = cppp * s_dot3 + 3.0 * cpp * s_dot * s_ddot + cp * s_dddot;
            let ratio = j.abs() / lim.j_max[ax];
            if ratio > worst {
                worst = ratio;
            }
        }
    }
    worst
}

const SLP9_MAX_OUTER_ITERS: u32 = 30;
const SLP9_WARN_AT_ITER: u32 = 15;
const SLP9_EPS_FEAS: f64 = 5e-2;
const SLP9_RHO_B_INIT: f64 = 0.05;
const SLP9_RHO_A_INIT: f64 = 0.10;
const SLP9_RHO_B_MIN: f64 = 0.005;
const SLP9_RHO_B_MAX: f64 = 0.20;
const SLP9_RHO_A_MIN: f64 = 0.01;
const SLP9_RHO_A_MAX: f64 = 0.40;
const SLP9_MAX_BACKTRACKS: u32 = 3;
const SLP9_TARGET_DECAY: f64 = 0.85;
const SLP9_TARGET_MARGIN: f64 = 1e-3;
const SLP9_CUT_PLACEMENT_FRACTION: f64 = 0.9;

/// Cut builder for a chain grid. Uses per-point stencil weights from
/// `chain.h_intervals`. Junction dual points receive extra cuts (dual geometry
/// and limits evaluated at the same stencil triple).
pub(crate) fn build_axis_jerk_cuts_chain(
    result: &SolverResult,
    chain: &crate::topp::chain::ChainGrid,
    target_ratio: f64,
) -> Vec<SlpCut> {
    let n = result.b.len();
    let mut cuts: Vec<SlpCut> = Vec::new();

    for i in 0..n {
        let (idx, hl, hr) = crate::topp::stencil::stencil_at(i, n, &chain.h_intervals);
        let w = crate::topp::stencil::b_dd_weights(hl, hr);
        let s_dddot = crate::topp::stencil::s_dddot_at_weights(&result.b, i, &chain.h_intervals);
        let s_dot = result.b[i].max(0.0).sqrt();
        let s_dot3 = s_dot * s_dot * s_dot;
        let s_ddot = result.a[i];
        let b_bars: [f64; 3] = [result.b[idx[0]], result.b[idx[1]], result.b[idx[2]]];
        let geom = &chain.geom[i];
        let lim = chain.limits_at(i);
        for ax in 0..3 {
            let cp = geom.c_prime[ax];
            let cpp = geom.c_double_prime[ax];
            let cppp = geom.c_triple_prime[ax];
            let j = cppp * s_dot3 + 3.0 * cpp * s_dot * s_ddot + cp * s_dddot;
            let ratio = j.abs() / lim.j_max[ax];
            if ratio > SLP9_CUT_PLACEMENT_FRACTION * target_ratio {
                cuts.push(SlpCut::AxisJerk(AxisJerkCut {
                    i,
                    axis: ax,
                    idx,
                    w,
                    b_bars,
                    a_bar_i: result.a[i],
                    cp,
                    cpp,
                    cppp,
                    j_lim_inflated: lim.j_max[ax] * target_ratio,
                }));
            }
        }
        // Junction dual: same stencil triple, right-side geometry and limits.
        for jct in chain.junctions.iter().filter(|jct| jct.idx == i) {
            let jlim = &chain.limits[jct.limits_idx];
            let jgeom = &jct.geom;
            for ax in 0..3 {
                let cp = jgeom.c_prime[ax];
                let cpp = jgeom.c_double_prime[ax];
                let cppp = jgeom.c_triple_prime[ax];
                let j = cppp * s_dot3 + 3.0 * cpp * s_dot * s_ddot + cp * s_dddot;
                let ratio = j.abs() / jlim.j_max[ax];
                if ratio > SLP9_CUT_PLACEMENT_FRACTION * target_ratio {
                    cuts.push(SlpCut::AxisJerk(AxisJerkCut {
                        i,
                        axis: ax,
                        idx,
                        w,
                        b_bars,
                        a_bar_i: result.a[i],
                        cp,
                        cpp,
                        cppp,
                        j_lim_inflated: jlim.j_max[ax] * target_ratio,
                    }));
                }
            }
        }
    }
    cuts
}

/// Path-jerk SLP outer loop for chain grids (non-uniform spacing).
///
/// Clone of `slp_solve` control flow; calls `find_jerk_violators_chain` and
/// emits weight-based path-jerk cuts via `append_path_jerk_cut_weights`.
/// Wired into the schedule entry in Task 8.
pub(crate) fn slp_solve_chain(
    bundle: &ConstraintBundle,
    tol: f64,
    scale: &SolverScale,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let j_path = bundle.j_path;
    debug_assert!(j_path > 0.0);
    let n = bundle.n_grid;

    let mut cuts: Vec<SlpCut> = Vec::new();
    let mut last_result = solve_with_cuts(bundle, &cuts, tol, scale)?;

    if matches!(
        last_result.status,
        SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
    ) {
        return Ok((last_result, SlpOutcome::InnerSolverFailure));
    }

    let violators = find_jerk_violators_chain(&last_result.b, &bundle.h_intervals, j_path);
    if violators.is_empty() {
        return Ok((last_result, SlpOutcome::Converged { outer_iters: 0 }));
    }

    let mut best_result = last_result.clone();
    let mut best_ratio_so_far = max_ratio(&violators);
    let mut max_ratio_history: Vec<f64> = Vec::new();
    let mut best_ratio_history: Vec<f64> = Vec::new();
    max_ratio_history.push(best_ratio_so_far);
    best_ratio_history.push(best_ratio_so_far);
    let b_cut_floor = scale.to_scaled_b(SLP_B_CUT_FLOOR);

    for outer in 1..=SLP_MAX_OUTER_ITERS {
        cuts.clear();
        let mut added = 0_usize;
        for i in 1..n - 1 {
            let b_bar = last_result.b[i];
            if b_bar < b_cut_floor {
                continue;
            }
            let (idx, hl, hr) = crate::topp::stencil::stencil_at(i, n, &bundle.h_intervals);
            let w = crate::topp::stencil::b_dd_weights(hl, hr);
            let h_bar = 0.5 * (bundle.h_intervals[i - 1] + bundle.h_intervals[i]);
            cuts.push(SlpCut::PathJerkWeights {
                i,
                b_bar,
                j_path,
                idx,
                w,
                h_bar,
            });
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

        let new_result = solve_with_cuts(bundle, &cuts, tol, scale)?;
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

        let new_violators =
            find_jerk_violators_chain(&last_result.b, &bundle.h_intervals, j_path);
        if new_violators.is_empty() {
            return Ok((last_result, SlpOutcome::Converged { outer_iters: outer }));
        }

        let cur_max = max_ratio(&new_violators);
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

/// Path-jerk + per-axis-jerk SLP for chain grids (non-uniform spacing).
///
/// Clone of `slp_solve_with_axis_jerk` control flow; calls `slp_solve_chain`
/// then `max_axis_ratio_chain` / `build_axis_jerk_cuts_chain` per iteration.
/// Wired into the schedule entry in Task 8.
#[allow(clippy::too_many_lines)]
pub(crate) fn slp_solve_with_axis_jerk_chain(
    bundle: &ConstraintBundle,
    chain: &crate::topp::chain::ChainGrid,
    tol: f64,
    scale: &SolverScale,
) -> Result<(SolverResult, SlpOutcome), SolverSetupError> {
    let (path_result, path_outcome) = slp_solve_chain(bundle, tol, scale)?;

    if matches!(
        path_outcome,
        SlpOutcome::InnerSolverFailure | SlpOutcome::Diverged { .. } | SlpOutcome::MaxIters { .. }
    ) {
        return Ok((path_result, path_outcome));
    }

    debug_assert_eq!(chain.s.len(), path_result.b.len());

    let mut last_result = path_result.clone();
    let path_outer_iters = match path_outcome {
        SlpOutcome::Converged { outer_iters } => outer_iters,
        _ => 0,
    };

    let initial_max = max_axis_ratio_chain(&last_result, chain);
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
                "slp9_chain warning: per-axis SLP not converged at iter {outer} \
                 (best ratio = {best_ratio:.4})",
            );
        }

        let target_floor = (1.0 + SLP9_EPS_FEAS) * (1.0 - SLP9_TARGET_MARGIN);
        let target_ratio = (best_ratio * SLP9_TARGET_DECAY).max(target_floor);
        let cuts = build_axis_jerk_cuts_chain(&last_result, chain, target_ratio);
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
                scale,
            )?;
            if matches!(
                candidate.status,
                SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
            ) {
                continue;
            }
            let cand_ratio = max_axis_ratio_chain(&candidate, chain);
            if cand_ratio < best_ratio {
                accepted = Some(candidate);
                best_ratio = cand_ratio;
                break;
            }
        }
        if accepted.is_none() {
            let candidate = solve_with_cuts(bundle, &cuts, tol, scale)?;
            if !matches!(
                candidate.status,
                SolverStatus::Infeasible | SolverStatus::MaxIter { .. }
            ) {
                let cand_ratio = max_axis_ratio_chain(&candidate, chain);
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

#[cfg(test)]
mod tests;
