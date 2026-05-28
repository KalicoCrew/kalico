//! Per-axis constraint-bundle builder. Pure data; no solver dependency.
//!
//! Spec §2.2 (constraint forms), §4.3 stage 2, §7.3 (boundary check).
//!
//! # SOCP formulation
//!
//! Based on Consolini & Locatelli, arXiv:2310.07583 (2024). The paper's
//! "tangential jerk" relaxation is 2-D and uses **scalar tangential jerk**,
//! NOT per-axis Cartesian jerk. Per the orchestrator's decision (§11),
//! kalico adopts the paper's scalar-tangential form with
//! `J_path = min(j_max.x, j_max.y, j_max.z)`.
//!
//! The post-solve verifier (`topp::verify::check`) checks the same scalar
//! quantity — verification scope tracks SOCP scope by design. Per-axis
//! Cartesian jerk verification is co-deferred to Step 9 alongside the
//! per-axis SOC relaxation; both are blocked by the same non-convex
//! cross-term `3·c''·v·a` (≡ `q² ≥ b·a²`, indefinite Hessian).
//!
//! ## Decision variables
//!
//! For grid points `i ∈ 0..N` (N = grid size):
//!
//! ```text
//! indices  0..N     : b_i  — squared path-speed  ṡ_i²  (= v_i²)
//! indices  N..2N    : a_i  — path-accel  s̈_i = ½·b'(s_i) (≡ central FD on b
//!                            with coefficient ±1/(4h); see block (b))
//! indices  2N..3N-2 : t_i  — jerk-envelope slack  t_i ≥ h/√b_i  (interior i=1..N-2)
//! indices  3N-2..4N-4: x1_i — rotated-SOC aux, x1_i² ≤ t_i·b_i·h  (interior)
//! indices  4N-4..5N-6: x2_i — rotated-SOC aux, x2_i² ≤ t_i·h      (interior)
//! ```
//!
//! Total `n_vars = N + N + 3·(N-2) = 5N - 6`.
//!
//! ## Constraint block order (matches `cones` vec)
//!
//! 1. **(a) Boundary equalities** — zero cone, 2 rows
//!    `b_0 = v_start²`, `b_{N-1} = v_end²`
//!
//! 2. **(b) Acceleration linkage** — zero cone, N rows
//!    `a_i = (b_{i+1}-b_{i-1})/(4h)` interior; forward/backward diff at
//!    endpoints with coefficient `±1/(2h)`. The factor of ½ vs. the raw
//!    central FD on b is the `s̈ = ½·b'(s)` identity (see block (b) docs).
//!
//! 3. **(c) Per-axis velocity UB** — nonneg cone, up to 3N rows
//!    `(v_max,axis / |c'_axis|)² - b_i ≥ 0`; skip axis when `|c'_axis| < 1e-12`.
//!
//! 4. **(d) Per-axis acceleration two-sided** — nonneg cone, up to 6N rows
//!    `a_max ∓ (c''_axis·b_i + c'_axis·a_i) ≥ 0`; skip when both partials tiny.
//!
//! 5. **(e) Centripetal UB** — nonneg cone, N rows
//!    `b_max_cent[i] - b_i ≥ 0`
//!
//! 6. **(f) Scalar-tangential jerk envelope** — nonneg cone, 2·(N-2) rows
//!    `t_i ± (b_{i-1} - 2b_i + b_{i+1}) / (2h·J_path) ≥ 0` — derived from
//!    `|s⃛| ≤ J_path` via `s⃛ = ½·b''(s)·√b` ⇒ `|b''(s)| ≤ 2J/√b`.
//!
//! 7. **(g) x1,x2 nonnegativity** — nonneg cone, 2·(N-2) rows
//!    `x1_i ≥ 0`, `x2_i ≥ 0`
//!
//! 8. **(h) SOC chain for t·b ≥ h²** — standard SOC, 3·(N-2) blocks of size 3
//!    Using the norm-form encoding `||(2z, u-v)|| ≤ u+v ⟺ z² ≤ u·v` (u,v ≥ 0):
//!      - Block H1: encodes x2² ≤ t·h   → `||(2·x2, t-h)|| ≤ t+h`
//!      - Block H2: encodes x1²/h ≤ t·b → `||(2·x1/√h, t-b)|| ≤ t+b`
//!      - Block H3: encodes h² ≤ x1·x2  → `||(2h, x2-x1)|| ≤ x2+x1`
//!
//! ## Objective
//!
//! `min Σ_{i=1}^{N-2} t_i` (coefficient 1.0 on each t variable, 0 elsewhere).
//! This is the paper's time-integral surrogate: since `t_i ≥ h/√b_i`,
//! minimizing `Σ t_i` approximates `Σ h/√b_i ≈ ∫ ds/v(s) = T_total`.

use crate::Limits;
use crate::topp::path::ArclengthGrid;

/// Cone descriptor in solver-agnostic form. The solver module (Task 5)
/// translates these into Clarabel's vocabulary.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cone {
    /// Equality (zero cone).
    Zero,
    /// Linear inequality `row·x + b_rhs ≥ 0` (nonneg cone).
    Nonneg,
    /// Second-order cone: `||x_tail|| ≤ x_head`.
    SecondOrder,
    /// Rotated SOC: `2 x_0 x_1 ≥ ||x_tail||²`, `x_0, x_1 ≥ 0`.
    RotatedSecondOrder,
}

/// Solver-agnostic constraint bundle produced by [`build`].
///
/// All data is in dense form. Task 5 converts this to Clarabel's sparse format.
#[derive(Debug, Clone)]
pub struct ConstraintBundle {
    /// Total number of decision variables. See module-level layout.
    pub n_vars: usize,
    /// Number of grid points (= `grid.s.len()`).
    pub n_grid: usize,
    /// `(cone_kind, dim)` blocks in row-order; `a_rows` and `b_rhs` rows
    /// concatenate to match this ordering exactly.
    pub cones: Vec<(Cone, usize)>,
    /// Dense constraint matrix `A`, row-major, shape `(Σ dims, n_vars)`.
    /// The SOCP standard form is `Ax + b ∈ K`, i.e. `Ax + b_rhs ≥ 0` for
    /// nonneg rows, `Ax + b_rhs = 0` for zero rows, etc.
    pub a_rows: Vec<Vec<f64>>,
    /// RHS vector `b`, length `Σ dims`.
    pub b_rhs: Vec<f64>,
    /// Linear objective: `min c·x`. Coefficient per variable.
    pub objective: Vec<f64>,
    /// Per-grid-point centripetal MVC `b_max_cent(s_i)` = `a_centripetal_max / κ(s_i)`,
    /// clamped by [`B_MAX_CENT_CAP`]. Used for the boundary-infeasibility check.
    pub b_max_cent: Vec<f64>,
    /// Uniform arclength grid spacing `h = s[1] − s[0]`. Used by the SLP
    /// outer-iteration solver to construct linearized cuts on `1/√b` at
    /// violator grid points (spec §11; Lee 2024 fallback).
    pub h: f64,
    /// Scalar tangential jerk bound `J_path = min(j_max,x, j_max,y, j_max,z)`,
    /// matching block (f). Used by the SLP outer-iteration solver to assemble
    /// the cut RHS and the violator predicate.
    pub j_path: f64,
}

/// Pre-solver boundary infeasibility (start or end velocity exceeds centripetal MVC).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BoundaryInfeasibility {
    StartAboveMvc { mvc_b: f64 },
    EndAboveMvc { mvc_b: f64 },
}

/// Result of [`build`].
#[derive(Debug, Clone)]
pub enum BuildOutcome {
    Ok(ConstraintBundle),
    Boundary(BoundaryInfeasibility),
}

/// Endpoint velocity constraints (path speed, mm/s).
#[derive(Debug, Clone, Copy)]
pub struct EndpointVelocities {
    pub v_start: f64,
    pub v_end: f64,
}

/// Numerical floor on κ for the centripetal MVC; below this the path is treated
/// as locally straight (no centripetal limit). Spec §2.2 / §7.2.
pub const KAPPA_FLOOR: f64 = 1e-12;

/// Cap on `b_max_cent` to guard against κ ≈ 0 noise. Spec §7.2.
pub const B_MAX_CENT_CAP: f64 = 1e8;

/// Small threshold below which an axis tangent or curvature component is
/// considered zero and the corresponding constraint row is skipped (vacuous).
const COMP_FLOOR: f64 = 1e-12;

/// Build the complete constraint bundle for one NURBS segment.
///
/// Returns [`BuildOutcome::Boundary`] if the start or end velocity exceeds the
/// centripetal maximum-velocity curve at that endpoint (§7.3 pre-check).
#[allow(clippy::too_many_lines)]
pub fn build(grid: &ArclengthGrid, limits: &Limits, endpoints: EndpointVelocities) -> BuildOutcome {
    let n = grid.s.len();
    debug_assert!(n >= 2, "ArclengthGrid must have at least 2 points");

    // -------------------------------------------------------------------------
    // Step 1: Compute b_max_cent per grid point.
    //
    // b_max_cent[i] = min(a_centripetal_max / κ(s_i), B_MAX_CENT_CAP)
    // When κ < KAPPA_FLOOR, the path is locally straight and the centripetal
    // constraint is vacuous — cap at B_MAX_CENT_CAP.
    // -------------------------------------------------------------------------
    let b_max_cent: Vec<f64> = grid
        .kappa
        .iter()
        .map(|&k| {
            if k.abs() < KAPPA_FLOOR {
                B_MAX_CENT_CAP
            } else {
                (limits.a_centripetal_max / k.abs()).min(B_MAX_CENT_CAP)
            }
        })
        .collect();

    // -------------------------------------------------------------------------
    // Step 2: Boundary-above-MVC pre-check (§7.3).
    //
    // If v_start² > b_max_cent[0] or v_end² > b_max_cent[N-1], the problem is
    // geometrically infeasible before the solver is invoked.
    // -------------------------------------------------------------------------
    let b_start = endpoints.v_start * endpoints.v_start;
    let b_end = endpoints.v_end * endpoints.v_end;

    if b_start > b_max_cent[0] {
        return BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc {
            mvc_b: b_max_cent[0],
        });
    }
    if b_end > b_max_cent[n - 1] {
        return BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMvc {
            mvc_b: b_max_cent[n - 1],
        });
    }

    // -------------------------------------------------------------------------
    // Step 3: Variable layout constants.
    //
    // n_vars = 5N - 6  (see module-level doc for full derivation).
    //
    // b:  indices [0, N)
    // a:  indices [N, 2N)
    // t:  indices [2N, 3N-2)   — (N-2) interior points, i=1..N-2
    // x1: indices [3N-2, 4N-4) — same (N-2) interior points
    // x2: indices [4N-4, 5N-6) — same (N-2) interior points
    //
    // Helper: given an interior index i ∈ 1..N-1 (one-based in the interior),
    // the t/x1/x2 slot for that point is (i-1).
    // -------------------------------------------------------------------------
    let n_b = n;
    let n_a = n;
    let n_interior = n.saturating_sub(2); // N-2; 0 when N<2 (guarded above)

    let off_b = 0_usize;
    let off_a = n_b;
    let off_t = off_a + n_a;
    let off_x1 = off_t + n_interior;
    let off_x2 = off_x1 + n_interior;

    let n_vars = off_x2 + n_interior; // = 5N - 6 for N ≥ 2

    // Uniform grid spacing h = s[1] - s[0].
    // (The grid is uniform-in-s by construction in sample_arclength_grid.)
    let h = if n >= 2 { grid.s[1] - grid.s[0] } else { 1.0 };
    debug_assert!(h > 0.0, "grid spacing h must be positive");

    // Scalar-tangential jerk bound: J_path = min(j_max,x, j_max,y, j_max,z).
    // Conservative but provably tight for the paper's setup (Cor. 5.1).
    //
    // MAINTAINER WARNING: Do NOT "improve" J_path to a per-axis projected bound
    // (e.g., sum_axis |c'_axis|·j_max,axis), and do NOT add per-axis Cartesian
    // jerk rows here OR in topp::verify::check. The SOC chain in block (h) is
    // only convex for a SINGLE scalar J_path; replacing it with a per-axis
    // bound would silently violate the convexity guarantee from
    // Consolini-Locatelli 2024 §3-§4. Symmetrically, having the verifier check
    // per-axis Cartesian jerk while the SOCP cannot enforce it produces false
    // negatives on every curved fixture (the cross-term `3·c''·v·a` requires
    // bounding the non-convex set `{q² ≥ b·a²}`). Per-axis Cartesian jerk —
    // both SOCP relaxation AND verification — is deferred to Step 9 (likely
    // SLP / Lee-2024 outer iteration); the two MUST land together.
    // BindingConstraint::AxisJerk{axis} is reserved for that path.
    //
    // Note (2026-05-05): the per-axis SLP machinery downstream of this SOC
    // chain (verifier `check` and `append_axis_jerk_cut_to_clarabel`) was
    // unified at width-1 b-FD per
    // `docs/superpowers/specs/2026-05-05-stencil-unification-design.md`.
    // This brings the system's stencil count from 2 to 1 and resolves the
    // prior boundary-adjacent O(1)·b'' bias in the verifier's central-FD-on-`a`
    // estimator. The per-axis Cartesian jerk SOC relaxation in *this* file
    // (block-(h) territory) is still the warning above — that work remains
    // deferred.
    let j_path = limits.j_max[0].min(limits.j_max[1]).min(limits.j_max[2]);
    debug_assert!(j_path > 0.0, "jerk limit must be positive");

    // ---- Builders -----------------------------------------------------------
    // We build cones, a_rows, b_rhs together. Each block emitter pushes into
    // the same three vecs. We use a closure to keep things tidy.

    let mut cones: Vec<(Cone, usize)> = Vec::new();
    let mut a_rows: Vec<Vec<f64>> = Vec::new();
    let mut b_rhs: Vec<f64> = Vec::new();

    // Helper: append a single row with sparse entries.
    // `entries`: (variable_index, coefficient) pairs. All other coefficients 0.
    // `rhs`: the b_rhs scalar for this row.
    // Accumulates into a_rows / b_rhs.
    let push_row =
        |a_rows: &mut Vec<Vec<f64>>, b_rhs: &mut Vec<f64>, entries: &[(usize, f64)], rhs: f64| {
            let mut row = vec![0.0_f64; n_vars];
            for &(idx, coeff) in entries {
                row[idx] = coeff;
            }
            a_rows.push(row);
            b_rhs.push(rhs);
        };

    // -------------------------------------------------------------------------
    // Block (a): Boundary equalities — zero cone, 2 rows.
    //
    // Constraint: A·x + b = 0  (zero cone means Ax + b ∈ {0})
    //   b_0  - v_start² = 0  →  row: +1 on b[0],  rhs: -v_start²
    //   but our standard form is  A·x + b_rhs = 0, i.e. A·x = -b_rhs
    //
    // Convention: the SOCP standard form used here is:
    //   A·x + b_rhs ∈ K
    // For zero cone (equality):  A·x + b_rhs = 0, i.e. A·x = -b_rhs.
    //
    // b_0 = v_start²  ↔  1·b_0 + (-v_start²) = 0
    //   → A-row: +1 on b[0], all others 0;  b_rhs = -v_start²
    //
    // Sign: the row says A·x + b_rhs = 0, so:
    //   x[off_b + 0] + (-v_start²) = 0  ↔  x[off_b + 0] = v_start²  ✓
    // -------------------------------------------------------------------------
    {
        let mut count = 0_usize;
        // b_0 = v_start²
        push_row(&mut a_rows, &mut b_rhs, &[(off_b, 1.0)], -b_start);
        count += 1;
        // b_{N-1} = v_end²
        push_row(&mut a_rows, &mut b_rhs, &[(off_b + n - 1, 1.0)], -b_end);
        count += 1;
        cones.push((Cone::Zero, count));
    }

    // -------------------------------------------------------------------------
    // Block (b): Acceleration linkage — zero cone, N rows.
    //
    // a_i ≡ s̈_i (path acceleration). Since b(s) = ṡ², we have
    // b'(s) = 2·s̈, so s̈_i = ½·b'(s_i). The finite-difference coefficients
    // therefore carry an extra factor of ½ relative to a raw `b'` estimator:
    //
    // a_i = (b_{i+1} - b_{i-1}) / (4h)   for i = 1..N-2  (interior, central)
    // a_0 = (b_1 - b_0) / (2h)            (forward diff)
    // a_{N-1} = (b_{N-1} - b_{N-2}) / (2h) (backward diff)
    //
    // Block (d) and verify::check both consume `a_i` as `s̈` directly via the
    // per-axis Cartesian-acceleration identity `d²x/dt² = c''·b + c'·s̈`.
    // Encoding `a_i` as the unhalved `b'(s)` would silently halve the
    // effective straight-line acceleration limit (it did, for many commits —
    // see Plan changes log, 2026-04-27 entry on the factor-of-2 fix).
    //
    // Rewrite as  A·x + b_rhs = 0:
    //   a_i - (b_{i+1} - b_{i-1}) / (4h) = 0
    //   → row: +1 on a[i],  -1/(4h) on b[i+1],  +1/(4h) on b[i-1],  rhs = 0
    //
    // For i=0 (forward diff):
    //   a_0 - (b_1 - b_0) / (2h) = 0
    //   → row: +1 on a[0],  -1/(2h) on b[1],  +1/(2h) on b[0],  rhs = 0
    //
    // For i=N-1 (backward diff):
    //   a_{N-1} - (b_{N-1} - b_{N-2}) / (2h) = 0
    //   → row: +1 on a[N-1],  -1/(2h) on b[N-1],  +1/(2h) on b[N-2],  rhs = 0
    // -------------------------------------------------------------------------
    {
        let count = n;
        // i = 0: forward difference
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a, 1.0),                  // +1 on a[0]
                (off_b + 1, -1.0 / (2.0 * h)), // -1/(2h) on b[1]
                (off_b, 1.0 / (2.0 * h)),      // +1/(2h) on b[0]
            ],
            0.0,
        );
        // i = 1..N-2: central differences
        for i in 1..n - 1 {
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (off_a + i, 1.0),                  // +1 on a[i]
                    (off_b + i + 1, -1.0 / (4.0 * h)), // -1/(4h) on b[i+1]
                    (off_b + i - 1, 1.0 / (4.0 * h)),  // +1/(4h) on b[i-1]
                ],
                0.0,
            );
        }
        // i = N-1: backward difference
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a + n - 1, 1.0),              // +1 on a[N-1]
                (off_b + n - 1, -1.0 / (2.0 * h)), // -1/(2h) on b[N-1]
                (off_b + n - 2, 1.0 / (2.0 * h)),  // +1/(2h) on b[N-2]
            ],
            0.0,
        );
        cones.push((Cone::Zero, count));
    }

    // -------------------------------------------------------------------------
    // Block (c): Per-axis velocity upper bound — nonneg cone.
    //
    // For each grid point i and each axis ax ∈ {X=0, Y=1, Z=2}:
    //   (v_max,ax / |c'_ax(s_i)|)² - b_i ≥ 0
    //   → A·x + b_rhs ≥ 0  →  row: -1 on b[i],  rhs = (v_max/|g|)²
    //
    // Skip when |g| < COMP_FLOOR (axis not active at this point).
    //
    // NOTE: the nonneg cone uses the same A·x + b_rhs ≥ 0 form.
    //
    // # Numerical-conditioning cap (spec §11, §7.2)
    //
    // Block (c) shares the [`B_MAX_CENT_CAP`] feasibility cap with block (e):
    // when `(v_max/|g|)² > B_MAX_CENT_CAP`, the row is vacuous because block
    // (e) already enforces `b_i ≤ B_MAX_CENT_CAP` for every grid point. The
    // skip is required because finite-difference noise in `c_prime` at
    // endpoints where one Cartesian tangent component is mathematically zero
    // (e.g. a rational-quadratic quarter-arc has `c'_y = 0` at u=0 and
    // `c'_x = 0` at u=1) produces `|g|` on the order of 1e-6. That gives
    // RHS ≈ 1e15 — purely vacuous, but injected into the SOCP it destroys
    // Clarabel's interior-point conditioning and yields `MaxIter` on any
    // non-trivial curved input. The dropped rows are dominated either by
    // block (e)'s cap on `b_i` or by the same-axis row at neighboring grid
    // points where `c'_axis` is well clear of zero, so the feasible region
    // is unchanged.
    // -------------------------------------------------------------------------
    {
        let mut count = 0_usize;
        for i in 0..n {
            for ax in 0..3 {
                let g = grid.c_prime[i][ax];
                if g.abs() < COMP_FLOOR {
                    continue;
                }
                let v_ax = limits.v_max[ax];
                let rhs = (v_ax / g).powi(2); // = (v_max / |g|)²  (g² in denom)
                if rhs > B_MAX_CENT_CAP {
                    // Row is dominated by block (e)'s b ≤ B_MAX_CENT_CAP cap;
                    // keeping it injects FD-noise-driven 1e15 RHS values that
                    // wreck SOCP conditioning. See block-doc above.
                    continue;
                }
                push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], rhs);
                count += 1;
            }
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    // -------------------------------------------------------------------------
    // Block (d): Per-axis acceleration two-sided — nonneg cone.
    //
    // The Cartesian acceleration of the tool is:
    //   d²C_ax/dt² = c''_ax(s)·ṡ² + c'_ax(s)·s̈ = c''_ax·b_i + c'_ax·a_i
    //
    // Two-sided constraint |d²C_ax/dt²| ≤ a_max,ax  expands to:
    //   Positive side: a_max,ax - (c''_ax·b_i + c'_ax·a_i) ≥ 0
    //     → A-row: -c''_ax on b[i],  -c'_ax on a[i];   rhs = a_max,ax
    //   Negative side: a_max,ax + (c''_ax·b_i + c'_ax·a_i) ≥ 0
    //     → A-row: +c''_ax on b[i],  +c'_ax on a[i];   rhs = a_max,ax
    //
    // Skip row when both |c''_ax| < COMP_FLOOR AND |c'_ax| < COMP_FLOOR
    // (constraint is vacuous: 0·x ≤ a_max is always satisfied).
    //
    // # Feasibility-redundancy prune (spec §11)
    //
    // Skip block-(d) rows that cannot bind. Worst-case LHS bounded by triangle
    // inequality (local b_cap + stencil-aware a_cap); if < 10% of a_max, the
    // row is feasibility-redundant. Mirrors block-(c) cap pattern. Spec §11.
    // -------------------------------------------------------------------------
    {
        /// Safety fraction: skip block-(d) rows whose worst-case LHS falls
        /// below this fraction of `a_max`. Mirrors block-(c) cap pattern.
        const BLOCK_D_SAFETY: f64 = 0.1;
        let mut count = 0_usize;
        for i in 0..n {
            for ax in 0..3 {
                let gp = grid.c_prime[i][ax]; // c'_ax
                let gpp = grid.c_double_prime[i][ax]; // c''_ax
                if gp.abs() < COMP_FLOOR && gpp.abs() < COMP_FLOOR {
                    continue;
                }
                let a_ax = limits.a_max[ax];
                // Feasibility-redundancy prune: skip both +cut and -cut row
                // when the worst-case LHS is physically vacuous.
                let b_cap_i = b_max_cent[i].min(B_MAX_CENT_CAP);
                let a_cap_i = b_cap_i / (2.0 * h);
                let worst_case_lhs = gpp.abs() * b_cap_i + gp.abs() * a_cap_i;
                if worst_case_lhs < BLOCK_D_SAFETY * a_ax {
                    continue;
                }
                // Positive side: a_max - c''·b_i - c'·a_i ≥ 0
                push_row(
                    &mut a_rows,
                    &mut b_rhs,
                    &[(off_b + i, -gpp), (off_a + i, -gp)],
                    a_ax,
                );
                // Negative side: a_max + c''·b_i + c'·a_i ≥ 0
                push_row(
                    &mut a_rows,
                    &mut b_rhs,
                    &[(off_b + i, gpp), (off_a + i, gp)],
                    a_ax,
                );
                count += 2;
            }
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    // -------------------------------------------------------------------------
    // Block (e): Centripetal upper bound — nonneg cone, N rows.
    //
    // b_max_cent[i] - b_i ≥ 0
    // → A-row: -1 on b[i];  rhs = b_max_cent[i]
    // -------------------------------------------------------------------------
    {
        let count = n;
        for i in 0..n {
            push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], b_max_cent[i]);
        }
        cones.push((Cone::Nonneg, count));
    }

    // -------------------------------------------------------------------------
    // Block (f): Scalar-tangential jerk envelope — nonneg cone, 2·(N-2) rows.
    //
    // Derivation. With b(s) = ṡ², s̈ = ½·b'(s), and s⃛ = d s̈/dt = ½·b''(s)·ṡ
    // = ½·b''(s)·√b. So |s⃛| ≤ J_path is equivalent to
    //   |b''(s)| ≤ 2·J_path / √b.
    //
    // Discretizing b''(s) by central differences,
    // b''(s_i) ≈ (b_{i-1} - 2 b_i + b_{i+1}) / h², the constraint becomes
    //   |Δ²b_i| / h² ≤ 2·J_path / √b_i.
    //
    // Introduce t_i as the slack for the rotated-SOC chain `t_i ≥ h/√b_i`
    // (paper §8). Multiplying both sides by `h / (2·J_path)` we get
    //   |Δ²b_i| / (2·h·J_path) ≤ h/√b_i ≡ t_i,
    // so t_i ≥ ±Δ²b_i / (2·h·J_path). Letting `hj := 2·h·J_path` keeps the
    // row coefficients identical to the older form modulo the factor-of-2.
    //
    // Encoding `hj := h·J_path` (without the factor of 2) silently enforced
    // |b''(s)| ≤ J_path/√b — half the correct slack — and combined with the
    // block-(b) factor-of-2 to halve every effective limit. See Plan changes
    // log, 2026-04-27 entry on the factor-of-2 fix.
    //
    // In A·x + b_rhs ≥ 0 form, for the positive-side row at interior point i
    // (slot k = i-1):
    //   t_i - (b_{i-1} - 2b_i + b_{i+1}) / hj ≥ 0
    //   → row: +1 on t[k],  -1/hj on b[i-1],  +2/hj on b[i],  -1/hj on b[i+1]
    //      rhs = 0
    //
    // Negative-side row:
    //   t_i + (b_{i-1} - 2b_i + b_{i+1}) / hj ≥ 0
    //   → row: +1 on t[k],  +1/hj on b[i-1],  -2/hj on b[i],  +1/hj on b[i+1]
    //      rhs = 0
    // -------------------------------------------------------------------------
    {
        let hj = 2.0 * h * j_path;
        let mut count = 0_usize;
        for k in 0..n_interior {
            let i = k + 1; // interior grid index (1..N-2 inclusive)
            let t_idx = off_t + k;

            // Positive side: t_i - Δ²b_i / (hJ) ≥ 0
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (t_idx, 1.0),
                    (off_b + i - 1, -1.0 / hj),
                    (off_b + i, 2.0 / hj),
                    (off_b + i + 1, -1.0 / hj),
                ],
                0.0,
            );
            // Negative side: t_i + Δ²b_i / (hJ) ≥ 0
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (t_idx, 1.0),
                    (off_b + i - 1, 1.0 / hj),
                    (off_b + i, -2.0 / hj),
                    (off_b + i + 1, 1.0 / hj),
                ],
                0.0,
            );
            count += 2;
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    // -------------------------------------------------------------------------
    // Block (g): x1, x2 nonnegativity — nonneg cone, 2·(N-2) rows.
    //
    // These are required by the paper's §8.1 SOC chain encoding; the SOC blocks
    // alone do not enforce x1, x2 ≥ 0 in Clarabel's formulation.
    //
    // x1_k ≥ 0 → A-row: +1 on x1[k],  rhs = 0
    // x2_k ≥ 0 → A-row: +1 on x2[k],  rhs = 0
    // -------------------------------------------------------------------------
    {
        let mut count = 0_usize;
        for k in 0..n_interior {
            push_row(&mut a_rows, &mut b_rhs, &[(off_x1 + k, 1.0)], 0.0);
            push_row(&mut a_rows, &mut b_rhs, &[(off_x2 + k, 1.0)], 0.0);
            count += 2;
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    // -------------------------------------------------------------------------
    // Block (h): SOC chain encoding t_i · b_i ≥ h² — standard SOC.
    //
    // Per paper §8.1, the chain t·w ≥ h² (where w = b_i) is encoded via
    // three auxiliary SOC blocks using the norm-form identity:
    //   ||(2z, u-v)|| ≤ u+v  ⟺  z² ≤ u·v  for u,v ≥ 0
    //
    // (Proof: |(2z)|² + (u-v)² ≤ (u+v)² ⟺ 4z²+u²-2uv+v² ≤ u²+2uv+v² ⟺ z²≤uv ✓)
    //
    // For each interior point k (grid index i = k+1):
    //
    //   (H1) x2_k² ≤ t_k · h   (h is a scalar constant, not a variable)
    //        Norm-form with u = t_k, v = h (constant), z = x2_k:
    //        ||(2·x2_k, t_k - h)|| ≤ t_k + h
    //        SOC-3 vector: [ t_k + h,   2·x2_k,   t_k - h ]
    //          → head = t_k + h,  tail = [2·x2_k, t_k - h]
    //
    //        In A·x + b_rhs form for norm SOC `||tail|| ≤ head`:
    //          head row: +1 on t_k;  rhs = h
    //          tail[0]:  +2 on x2_k; rhs = 0
    //          tail[1]:  +1 on t_k;  rhs = -h
    //
    //   (H2) x1_k² / h ≤ t_k · b_i   ↔   (x1_k/√h)² ≤ t_k · b_i
    //        Norm-form with u = t_k, v = b_i, z = x1_k/√h:
    //        ||(2·x1_k/√h, t_k - b_i)|| ≤ t_k + b_i
    //        SOC-3 vector: [ t_k + b_i,  2·x1_k/√h,  t_k - b_i ]
    //
    //        In A·x + b_rhs form:
    //          head row: +1 on t_k,  +1 on b_i;   rhs = 0
    //          tail[0]:  +2/√h on x1_k;            rhs = 0
    //          tail[1]:  +1 on t_k,  -1 on b_i;    rhs = 0
    //
    //   (H3) h² ≤ x1_k · x2_k
    //        Norm-form with u = x1_k, v = x2_k, z = h (constant):
    //        ||(2h, x1_k - x2_k)|| ≤ x1_k + x2_k
    //        SOC-3 vector: [ x1_k + x2_k,  2h,  x1_k - x2_k ]
    //
    //        In A·x + b_rhs form:
    //          head row: +1 on x1_k,  +1 on x2_k;  rhs = 0
    //          tail[0]:  (constant = 2h, no vars);   rhs = 2h
    //            Wait — this tail component is a *constant* 2h, with no variable.
    //            We encode it as rhs = 2h, A-row all zeros. The norms are then
    //            computed against the actual variable values + constants via b_rhs.
    //          tail[1]:  +1 on x1_k,  -1 on x2_k;  rhs = 0
    //
    // NOTE on norm-SOC vector orientation: Clarabel's SecondOrderCone expects
    //   [u, z_1, z_2, ..., z_{n-1}] with the constraint ||z|| ≤ u.
    //   So the head (≥ 0) must appear FIRST. We order accordingly.
    // -------------------------------------------------------------------------
    {
        let sqrt_h = h.sqrt();
        let b_idx = |k: usize| -> usize {
            // Grid index for interior point k is k+1
            off_b + k + 1
        };

        for k in 0..n_interior {
            let t_idx = off_t + k;
            let x1_idx = off_x1 + k;
            let x2_idx = off_x2 + k;
            let bi_idx = b_idx(k);

            // --- Block H1: ||(2·x2_k, t_k - h)|| ≤ t_k + h ---
            // Row 0 (head): (t_k + h) = t_k·1 + h  → coeff +1 on t, rhs = h
            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0)], h);
            // Row 1 (tail[0]): 2·x2_k → coeff +2 on x2, rhs = 0
            push_row(&mut a_rows, &mut b_rhs, &[(x2_idx, 2.0)], 0.0);
            // Row 2 (tail[1]): (t_k - h) = t_k·1 - h → coeff +1 on t, rhs = -h
            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0)], -h);
            cones.push((Cone::SecondOrder, 3));

            // --- Block H2: ||(2·x1_k/√h, t_k - b_i)|| ≤ t_k + b_i ---
            // Row 0 (head): (t_k + b_i) → coeffs +1 on t, +1 on b_i, rhs = 0
            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0), (bi_idx, 1.0)], 0.0);
            // Row 1 (tail[0]): 2·x1_k/√h → coeff +2/√h on x1, rhs = 0
            push_row(&mut a_rows, &mut b_rhs, &[(x1_idx, 2.0 / sqrt_h)], 0.0);
            // Row 2 (tail[1]): (t_k - b_i) → coeffs +1 on t, -1 on b_i, rhs = 0
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(t_idx, 1.0), (bi_idx, -1.0)],
                0.0,
            );
            cones.push((Cone::SecondOrder, 3));

            // --- Block H3: ||(2h, x1_k - x2_k)|| ≤ x1_k + x2_k ---
            // Row 0 (head): (x1_k + x2_k) → coeffs +1 on x1, +1 on x2, rhs = 0
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(x1_idx, 1.0), (x2_idx, 1.0)],
                0.0,
            );
            // Row 1 (tail[0]): constant = 2h → no vars, rhs = 2h
            push_row(&mut a_rows, &mut b_rhs, &[], 2.0 * h);
            // Row 2 (tail[1]): (x1_k - x2_k) → coeffs +1 on x1, -1 on x2, rhs = 0
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(x1_idx, 1.0), (x2_idx, -1.0)],
                0.0,
            );
            cones.push((Cone::SecondOrder, 3));
        }
    }

    // -------------------------------------------------------------------------
    // Objective: min Σ_{k=0}^{N-3} t_k  (= min Σ_{i=1}^{N-2} t_i)
    //
    // This is the paper's time-integral surrogate: t_i ≥ h/√b_i implies
    // Σ t_i ≥ Σ h/v_i ≈ ∫ ds/v(s) = T_total.
    //
    // Objective vector: 1.0 on each t variable, 0.0 everywhere else.
    // -------------------------------------------------------------------------
    let mut objective = vec![0.0_f64; n_vars];
    for k in 0..n_interior {
        objective[off_t + k] = 1.0;
    }

    // Sanity: row count should match cone dimension sum.
    debug_assert_eq!(
        a_rows.len(),
        cones.iter().map(|(_, d)| d).sum::<usize>(),
        "row count / cone dimension mismatch"
    );
    debug_assert_eq!(a_rows.len(), b_rhs.len(), "a_rows / b_rhs length mismatch");
    debug_assert!(
        a_rows.iter().all(|r| r.len() == n_vars),
        "all A rows must have width n_vars"
    );

    BuildOutcome::Ok(ConstraintBundle {
        n_vars,
        n_grid: n,
        cones,
        a_rows,
        b_rhs,
        objective,
        b_max_cent,
        h,
        j_path,
    })
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests;
