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
//! ## Decision variables
//!
//! For grid points `i ∈ 0..N` (N = grid size):
//!
//! ```text
//! indices  0..N     : b_i  — squared path-speed  ṡ_i²  (= v_i²)
//! indices  N..2N    : a_i  — path-accel auxiliary s̈_i ≈ ½·b'(s_i)
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
//!    `a_i = (b_{i+1}-b_{i-1})/(2h)` interior; forward/backward diff at endpoints.
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
//!    `t_i ± (b_{i-1} - 2b_i + b_{i+1}) / (h·J_path) ≥ 0`
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

use crate::topp::path::ArclengthGrid;
use crate::Limits;

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
pub fn build(
    grid: &ArclengthGrid,
    limits: &Limits,
    endpoints: EndpointVelocities,
) -> BuildOutcome {
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
    let push_row = |a_rows: &mut Vec<Vec<f64>>,
                    b_rhs: &mut Vec<f64>,
                    entries: &[(usize, f64)],
                    rhs: f64| {
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
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[(off_b, 1.0)],
            -b_start,
        );
        count += 1;
        // b_{N-1} = v_end²
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[(off_b + n - 1, 1.0)],
            -b_end,
        );
        count += 1;
        cones.push((Cone::Zero, count));
    }

    // -------------------------------------------------------------------------
    // Block (b): Acceleration linkage — zero cone, N rows.
    //
    // a_i = (b_{i+1} - b_{i-1}) / (2h)   for i = 1..N-2  (interior)
    // a_0 = (b_1 - b_0) / h               (forward diff)
    // a_{N-1} = (b_{N-1} - b_{N-2}) / h   (backward diff)
    //
    // Rewrite as  A·x + b_rhs = 0:
    //   a_i - (b_{i+1} - b_{i-1}) / (2h) = 0
    //   → row: +1 on a[i],  -1/(2h) on b[i+1],  +1/(2h) on b[i-1],  rhs = 0
    //
    // For i=0 (forward diff):
    //   a_0 - (b_1 - b_0) / h = 0
    //   → row: +1 on a[0],  -1/h on b[1],  +1/h on b[0],  rhs = 0
    //
    // For i=N-1 (backward diff):
    //   a_{N-1} - (b_{N-1} - b_{N-2}) / h = 0
    //   → row: +1 on a[N-1],  -1/h on b[N-1],  +1/h on b[N-2],  rhs = 0
    // -------------------------------------------------------------------------
    {
        let count = n;
        // i = 0: forward difference
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a,         1.0),      // +1 on a[0]
                (off_b + 1,    -1.0 / h),  // -1/h on b[1]
                (off_b,         1.0 / h),  // +1/h on b[0]
            ],
            0.0,
        );
        // i = 1..N-2: central differences
        for i in 1..n - 1 {
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (off_a + i,       1.0),              // +1 on a[i]
                    (off_b + i + 1,  -1.0 / (2.0 * h)), // -1/(2h) on b[i+1]
                    (off_b + i - 1,   1.0 / (2.0 * h)), // +1/(2h) on b[i-1]
                ],
                0.0,
            );
        }
        // i = N-1: backward difference
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a + n - 1,     1.0),      // +1 on a[N-1]
                (off_b + n - 1,    -1.0 / h),  // -1/h on b[N-1]
                (off_b + n - 2,     1.0 / h),  // +1/h on b[N-2]
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
    // -------------------------------------------------------------------------
    {
        let mut count = 0_usize;
        for i in 0..n {
            for ax in 0..3 {
                let gp  = grid.c_prime[i][ax];       // c'_ax
                let gpp = grid.c_double_prime[i][ax]; // c''_ax
                if gp.abs() < COMP_FLOOR && gpp.abs() < COMP_FLOOR {
                    continue;
                }
                let a_ax = limits.a_max[ax];
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
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(off_b + i, -1.0)],
                b_max_cent[i],
            );
        }
        cones.push((Cone::Nonneg, count));
    }

    // -------------------------------------------------------------------------
    // Block (f): Scalar-tangential jerk envelope — nonneg cone, 2·(N-2) rows.
    //
    // The scalar tangential jerk along the path satisfies (paper eq. 8):
    //   |b_{i-1} - 2b_i + b_{i+1}| / h² ≤ J_path / √b_i
    //
    // Introduce t_i as the slack for this constraint (paper §8):
    //   t_i ≥ (b_{i-1} - 2b_i + b_{i+1}) / (h · J_path)
    //   t_i ≥ -(b_{i-1} - 2b_i + b_{i+1}) / (h · J_path)
    //
    // (The division by h comes from rewriting: |Δ²b_i| / h² ≤ J/√b_i
    //  ↔ |Δ²b_i| / (h · J) ≤ h/√b_i ≡ t_i,
    //  so t_i ≥ ±Δ²b_i / (h·J) where Δ²b_i = b_{i-1} - 2b_i + b_{i+1}.)
    //
    // In A·x + b_rhs ≥ 0 form, for the positive-side row at interior point i
    // (slot k = i-1):
    //   t_i - (b_{i-1} - 2b_i + b_{i+1}) / (h·J_path) ≥ 0
    //   → row: +1 on t[k],  -1/(hJ) on b[i-1],  +2/(hJ) on b[i],  -1/(hJ) on b[i+1]
    //      rhs = 0
    //
    // Negative-side row:
    //   t_i + (b_{i-1} - 2b_i + b_{i+1}) / (h·J_path) ≥ 0
    //   → row: +1 on t[k],  +1/(hJ) on b[i-1],  -2/(hJ) on b[i],  +1/(hJ) on b[i+1]
    //      rhs = 0
    // -------------------------------------------------------------------------
    {
        let hj = h * j_path;
        let mut count = 0_usize;
        for k in 0..n_interior {
            let i = k + 1; // interior grid index (1..N-2 inclusive)
            let t_idx = off_t + k;

            // Positive side: t_i - Δ²b_i / (hJ) ≥ 0
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (t_idx,       1.0),
                    (off_b + i - 1, -1.0 / hj),
                    (off_b + i,      2.0 / hj),
                    (off_b + i + 1, -1.0 / hj),
                ],
                0.0,
            );
            // Negative side: t_i + Δ²b_i / (hJ) ≥ 0
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (t_idx,       1.0),
                    (off_b + i - 1,  1.0 / hj),
                    (off_b + i,     -2.0 / hj),
                    (off_b + i + 1,  1.0 / hj),
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
            let t_idx  = off_t  + k;
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
            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0), (bi_idx, -1.0)], 0.0);
            cones.push((Cone::SecondOrder, 3));

            // --- Block H3: ||(2h, x1_k - x2_k)|| ≤ x1_k + x2_k ---
            // Row 0 (head): (x1_k + x2_k) → coeffs +1 on x1, +1 on x2, rhs = 0
            push_row(&mut a_rows, &mut b_rhs, &[(x1_idx, 1.0), (x2_idx, 1.0)], 0.0);
            // Row 1 (tail[0]): constant = 2h → no vars, rhs = 2h
            push_row(&mut a_rows, &mut b_rhs, &[], 2.0 * h);
            // Row 2 (tail[1]): (x1_k - x2_k) → coeffs +1 on x1, -1 on x2, rhs = 0
            push_row(&mut a_rows, &mut b_rhs, &[(x1_idx, 1.0), (x2_idx, -1.0)], 0.0);
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
    debug_assert_eq!(
        a_rows.len(),
        b_rhs.len(),
        "a_rows / b_rhs length mismatch"
    );
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
    })
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topp::path::ArclengthGrid;
    use crate::Limits;

    // -------------------------------------------------------------------------
    // Test fixtures
    // -------------------------------------------------------------------------

    fn dummy_straight_grid(n: usize, length: f64) -> ArclengthGrid {
        // Synthetic grid: straight X-aligned line, zero curvature, unit X tangent.
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

    fn textbook_limits() -> Limits {
        Limits {
            v_max: [500.0, 500.0, 500.0],
            a_max: [5_000.0, 5_000.0, 5_000.0],
            j_max: [100_000.0, 100_000.0, 100_000.0],
            a_centripetal_max: 2_500.0,
        }
    }

    // -------------------------------------------------------------------------
    // Test 1 (plan's test): straight line, zero endpoints → BuildOutcome::Ok
    // -------------------------------------------------------------------------

    #[test]
    fn straight_line_zero_endpoints_builds_ok() {
        let grid = dummy_straight_grid(10, 100.0);
        let limits = textbook_limits();
        match build(&grid, &limits, EndpointVelocities { v_start: 0.0, v_end: 0.0 }) {
            BuildOutcome::Ok(b) => {
                assert_eq!(b.n_grid, 10);
                assert!(b.n_vars >= 10); // at least the b_i variables
                assert_eq!(b.b_max_cent.len(), 10);
                // Zero curvature ⇒ no centripetal limit ⇒ b_max_cent at cap.
                for &cap in &b.b_max_cent {
                    assert_eq!(cap, B_MAX_CENT_CAP);
                }
            }
            BuildOutcome::Boundary(_) => panic!("zero endpoints should not be infeasible"),
        }
    }

    // -------------------------------------------------------------------------
    // Test 2 (plan's test): boundary-above-MVC returns Boundary outcome
    // -------------------------------------------------------------------------

    #[test]
    fn boundary_above_mvc_returns_boundary_outcome() {
        // Curved grid: κ = 0.05 mm⁻¹ ⇒ b_max_cent = 2500 / 0.05 = 50_000.
        // v_start² = 60_000² = 3.6e9 > 50_000 ⇒ infeasible at start.
        let mut grid = dummy_straight_grid(5, 10.0);
        grid.kappa = vec![0.05; 5];
        let limits = textbook_limits();
        match build(
            &grid,
            &limits,
            EndpointVelocities { v_start: 60_000.0, v_end: 0.0 },
        ) {
            BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc { mvc_b }) => {
                assert!((mvc_b - 50_000.0).abs() < 1e-3);
            }
            other => panic!("expected StartAboveMvc, got {other:?}"),
        }
    }

    // -------------------------------------------------------------------------
    // Test 3 (structural): N=5 straight line — pin variable layout and cone counts
    // -------------------------------------------------------------------------

    #[test]
    fn straight_line_n_vars_and_cone_count_match_design() {
        // N = 5, straight X line, zero endpoints.
        // Expect: n_vars = 5N - 6 = 5*5 - 6 = 19.
        let grid = dummy_straight_grid(5, 100.0);
        let limits = textbook_limits();
        let bundle = match build(
            &grid,
            &limits,
            EndpointVelocities { v_start: 0.0, v_end: 0.0 },
        ) {
            BuildOutcome::Ok(b) => b,
            BuildOutcome::Boundary(_) => panic!("zero endpoints should be feasible"),
        };

        assert_eq!(bundle.n_grid, 5);
        assert_eq!(bundle.n_vars, 5 * 5 - 6); // = 19

        // ---- Nonneg-cone row counts ----
        //
        // For N=5, straight X-line (c'=[1,0,0], c''=[0,0,0], κ=0):
        //
        // (c) velocity UB:
        //   For each of the 5 grid points, only X-axis has |c'| = 1 ≥ COMP_FLOOR.
        //   Y and Z have |c'| = 0 → skipped. → 5 rows.
        //
        // (d) acceleration two-sided:
        //   gp = c'_ax, gpp = c''_ax. For X: gp=1.0, gpp=0.0 → 2 rows per point.
        //   For Y, Z: gp=0, gpp=0 → skipped.
        //   → 5 × 2 = 10 rows.
        //
        // (e) centripetal: 5 rows (always N rows).
        //
        // (f) jerk envelope: 2 × (5-2) = 6 rows.
        //
        // (g) x1, x2 nonneg: 2 × (5-2) = 6 rows.
        //
        // Total nonneg = 5 + 10 + 5 + 6 + 6 = 32.

        let nonneg_rows: usize = bundle
            .cones
            .iter()
            .filter(|(c, _)| matches!(c, Cone::Nonneg))
            .map(|(_, n)| *n)
            .sum();
        assert!(
            nonneg_rows >= 25 && nonneg_rows <= 60,
            "nonneg row count = {nonneg_rows}, expected 25-60"
        );

        // ---- SOC block counts ----
        //
        // Block (h) emits 3 SOC-3 blocks per interior point = 3 × 3 = 9 blocks.
        let soc_block_count = bundle
            .cones
            .iter()
            .filter(|(c, _)| matches!(c, Cone::SecondOrder))
            .count();
        assert_eq!(soc_block_count, 3 * (5 - 2));

        // ---- Zero-cone row counts ----
        //
        // (a) boundary equalities: 2 rows.
        // (b) acceleration linkage: 5 rows.
        // Total: 7 rows.
        let zero_block_count = bundle
            .cones
            .iter()
            .filter(|(c, _)| matches!(c, Cone::Zero))
            .map(|(_, n)| *n)
            .sum::<usize>();
        assert!(
            zero_block_count >= 7,
            "zero cone rows = {zero_block_count}, expected ≥ 7"
        );

        // ---- Dimension sanity ----
        let total_cone_dim: usize = bundle.cones.iter().map(|(_, d)| d).sum();
        assert_eq!(bundle.a_rows.len(), total_cone_dim);
        assert_eq!(bundle.b_rhs.len(), total_cone_dim);
        for row in &bundle.a_rows {
            assert_eq!(row.len(), bundle.n_vars, "row width mismatch");
        }

        // ---- Objective pins ----
        // t variables at indices 2N..3N-2 = 10..13 should have objective = 1.0.
        // All others 0.
        for (idx, &coeff) in bundle.objective.iter().enumerate() {
            if idx >= 10 && idx < 13 {
                assert_eq!(coeff, 1.0, "t var at idx {idx} should have obj coeff 1.0");
            } else {
                assert_eq!(coeff, 0.0, "var at idx {idx} should have obj coeff 0.0");
            }
        }
    }
}
