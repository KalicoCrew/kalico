use crate::Limits;
use crate::topp::path::ArclengthGrid;
use crate::topp::scaling::SolverScale;

/// Cone descriptor in solver-agnostic form.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cone {
    Zero,
    Nonneg,
    SecondOrder,
    RotatedSecondOrder,
}

/// Solver-agnostic constraint bundle produced by [`build`].
#[derive(Debug, Clone)]
pub struct ConstraintBundle {
    pub n_vars: usize,
    pub n_grid: usize,
    pub cones: Vec<(Cone, usize)>,
    /// Dense constraint matrix `A`, row-major. Standard form: `Ax + b_rhs ∈ K`.
    pub a_rows: Vec<Vec<f64>>,
    pub b_rhs: Vec<f64>,
    pub objective: Vec<f64>,
    pub b_max_cent: Vec<f64>,
    pub h: f64,
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

/// Numerical floor on κ; below this the path is treated as locally straight.
pub const KAPPA_FLOOR: f64 = 1e-12;

/// Cap on `b_max_cent` to guard against κ ≈ 0 noise.
pub const B_MAX_CENT_CAP: f64 = 1e8;

/// Threshold below which an axis tangent or curvature component is considered
/// zero and the corresponding constraint row is skipped (vacuous).
const COMP_FLOOR: f64 = 1e-12;

/// Double-S reachable-velocity envelope from rest: returns `v²` at arc-distance
/// `d` from the rest endpoint, under constant-jerk `J` and constant-accel `A`
/// phases.
///
/// Derivation: constant-jerk from rest gives s = J·t³/6, v = J·t²/2, so
/// v² = (6^(4/3)/4)·(J·s²)^(2/3); once a=A is reached (at s = A³/(6J²)),
/// constant-A kinematics extend the envelope as v² = v1² + 2A·(s − s1).
///
/// Used by block (e2) to close the jerk-impulse hole at b→0 where block (f)'s
/// √b weight vanishes; A/J must be the max-over-grid projected caps so the
/// envelope over-estimates reachability (sound, never cuts the true optimum).
pub fn rest_boundary_b_cap(d: f64, a_env: f64, j_env: f64) -> f64 {
    let s1 = a_env * a_env * a_env / (6.0 * j_env * j_env);
    let v1_sq = (a_env * a_env / (2.0 * j_env)).powi(2);
    if d <= s1 {
        (6.0_f64.powf(4.0 / 3.0) / 4.0) * (j_env * d * d).powf(2.0 / 3.0)
    } else {
        v1_sq + 2.0 * a_env * (d - s1)
    }
}

/// `grid`, `limits`, and `endpoints` must share the unit system described by
/// `scale`; the dimensioned guard constants in this file are physical (mm)
/// values converted through `scale` at use.
#[allow(clippy::too_many_lines)]
pub fn build(
    grid: &ArclengthGrid,
    limits: &Limits,
    endpoints: EndpointVelocities,
    scale: &SolverScale,
) -> BuildOutcome {
    let n = grid.s.len();
    debug_assert!(n >= 2, "ArclengthGrid must have at least 2 points");

    let kappa_floor = scale.to_scaled_kappa(KAPPA_FLOOR);
    let b_cap = scale.to_scaled_b(B_MAX_CENT_CAP);
    let b_max_cent: Vec<f64> = grid
        .kappa
        .iter()
        .map(|&k| {
            if k.abs() < kappa_floor {
                b_cap
            } else {
                (limits.a_centripetal_max / k.abs()).min(b_cap)
            }
        })
        .collect();

    // Max-over-grid projected tangential acceleration and jerk caps.
    // Each axis contributes a_max[ax] / |c'[ax]| (resp. j_max); the per-point
    // min over active axes gives the tightest scalar bound at that grid point.
    // The global max over all grid points ensures the envelope over-estimates
    // reachability (sound: never cuts the true optimum).
    let (a_env, j_env) = {
        let mut a_env = f64::NEG_INFINITY;
        let mut j_env = f64::NEG_INFINITY;
        for i in 0..n {
            let mut a_tan_i = f64::INFINITY;
            let mut j_tan_i = f64::INFINITY;
            let mut active = false;
            for ax in 0..3 {
                let g = grid.c_prime[i][ax].abs();
                if g > COMP_FLOOR {
                    a_tan_i = a_tan_i.min(limits.a_max[ax] / g);
                    j_tan_i = j_tan_i.min(limits.j_max[ax] / g);
                    active = true;
                }
            }
            if active {
                a_env = a_env.max(a_tan_i);
                j_env = j_env.max(j_tan_i);
            }
        }
        debug_assert!(
            a_env > 0.0 && j_env > 0.0,
            "A_env/J_env must be positive — corrupt grid tangents"
        );
        (a_env, j_env)
    };

    let b_start = endpoints.v_start * endpoints.v_start;
    let b_end = endpoints.v_end * endpoints.v_end;

    // Tolerance: v_jct set to sqrt(mvc_b_phys) in the joining loop. After
    // dividing v by σ and re-squaring, IEEE 754 may land b_start slightly above
    // b_max_cent. Allow up to 4 ULP slop so the scaled build doesn't reject a
    // physically-feasible starting velocity.
    const B_BOUNDARY_REL_TOL: f64 = f64::EPSILON * 4.0;
    if b_start > b_max_cent[0] * (1.0 + B_BOUNDARY_REL_TOL) {
        return BuildOutcome::Boundary(BoundaryInfeasibility::StartAboveMvc {
            mvc_b: b_max_cent[0],
        });
    }
    if b_end > b_max_cent[n - 1] * (1.0 + B_BOUNDARY_REL_TOL) {
        return BuildOutcome::Boundary(BoundaryInfeasibility::EndAboveMvc {
            mvc_b: b_max_cent[n - 1],
        });
    }

    let n_b = n;
    let n_a = n;
    let n_interior = n.saturating_sub(2);

    let off_b = 0_usize;
    let off_a = n_b;
    let off_t = off_a + n_a;
    let off_x1 = off_t + n_interior;
    let off_x2 = off_x1 + n_interior;

    let n_vars = off_x2 + n_interior;

    let h = if n >= 2 { grid.s[1] - grid.s[0] } else { 1.0 };
    debug_assert!(h > 0.0, "grid spacing h must be positive");

    // J_path = min(j_max per axis): conservative scalar bound for the SOCP's single-J
    // SOC chain. Do NOT replace with a per-axis projected bound — the SOC chain in
    // block (h) is only convex for a single scalar J_path. Per-axis Cartesian jerk
    // relaxation and verification must land together (deferred to the SLP stage).
    let j_path = limits.j_max[0].min(limits.j_max[1]).min(limits.j_max[2]);
    debug_assert!(j_path > 0.0, "jerk limit must be positive");

    let mut cones: Vec<(Cone, usize)> = Vec::new();
    let mut a_rows: Vec<Vec<f64>> = Vec::new();
    let mut b_rhs: Vec<f64> = Vec::new();

    let push_row =
        |a_rows: &mut Vec<Vec<f64>>, b_rhs: &mut Vec<f64>, entries: &[(usize, f64)], rhs: f64| {
            let mut row = vec![0.0_f64; n_vars];
            for &(idx, coeff) in entries {
                row[idx] = coeff;
            }
            a_rows.push(row);
            b_rhs.push(rhs);
        };

    // Block (a): boundary equalities — zero cone, 2 rows.
    // Form: A·x + b_rhs = 0.  b_0 = v_start², b_{N-1} = v_end².
    {
        let mut count = 0_usize;
        push_row(&mut a_rows, &mut b_rhs, &[(off_b, 1.0)], -b_start);
        count += 1;
        push_row(&mut a_rows, &mut b_rhs, &[(off_b + n - 1, 1.0)], -b_end);
        count += 1;
        cones.push((Cone::Zero, count));
    }

    // Block (b): acceleration linkage — zero cone, N rows.
    // a_i = s̈_i = ½·b'(s_i). Coefficients carry the ½ factor.
    // Interior: a_i = (b_{i+1} - b_{i-1}) / (4h).
    // Endpoints: forward/backward diff with coefficient ±1/(2h).
    {
        let count = n;
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a, 1.0),
                (off_b + 1, -1.0 / (2.0 * h)),
                (off_b, 1.0 / (2.0 * h)),
            ],
            0.0,
        );
        for i in 1..n - 1 {
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (off_a + i, 1.0),
                    (off_b + i + 1, -1.0 / (4.0 * h)),
                    (off_b + i - 1, 1.0 / (4.0 * h)),
                ],
                0.0,
            );
        }
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a + n - 1, 1.0),
                (off_b + n - 1, -1.0 / (2.0 * h)),
                (off_b + n - 2, 1.0 / (2.0 * h)),
            ],
            0.0,
        );
        cones.push((Cone::Zero, count));
    }

    // Block (c): per-axis velocity upper bound — nonneg cone.
    // (v_max,ax / |c'_ax|)² - b_i ≥ 0. Skip when |c'_ax| < COMP_FLOOR.
    // Also skip when rhs > B_MAX_CENT_CAP: the row is vacuous (dominated by
    // block (e)) and injecting 1e15-scale RHS from near-zero FD noise wrecks
    // Clarabel's interior-point conditioning.
    {
        let mut count = 0_usize;
        for i in 0..n {
            for ax in 0..3 {
                let g = grid.c_prime[i][ax];
                if g.abs() < COMP_FLOOR {
                    continue;
                }
                let v_ax = limits.v_max[ax];
                let rhs = (v_ax / g).powi(2);
                if rhs > b_cap {
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

    // Block (d): per-axis acceleration two-sided — nonneg cone.
    // |c''_ax·b_i + c'_ax·a_i| ≤ a_max,ax, split into ± rows.
    // Skip when both |c'_ax| and |c''_ax| < COMP_FLOOR (vacuous).
    {
        /// Skip rows whose worst-case LHS falls below this fraction of a_max.
        const BLOCK_D_SAFETY: f64 = 0.1;
        let mut count = 0_usize;
        for i in 0..n {
            for ax in 0..3 {
                let gp = grid.c_prime[i][ax];
                let gpp = grid.c_double_prime[i][ax];
                if gp.abs() < COMP_FLOOR && gpp.abs() < COMP_FLOOR {
                    continue;
                }
                let a_ax = limits.a_max[ax];
                let b_cap_i = b_max_cent[i].min(b_cap);
                let a_cap_i = b_cap_i / (2.0 * h);
                let worst_case_lhs = gpp.abs() * b_cap_i + gp.abs() * a_cap_i;
                if worst_case_lhs < BLOCK_D_SAFETY * a_ax {
                    continue;
                }
                push_row(
                    &mut a_rows,
                    &mut b_rhs,
                    &[(off_b + i, -gpp), (off_a + i, -gp)],
                    a_ax,
                );
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

    // Block (e): centripetal upper bound — nonneg cone, N rows.
    // b_max_cent[i] - b_i ≥ 0.
    {
        let count = n;
        for i in 0..n {
            push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], b_max_cent[i]);
        }
        cones.push((Cone::Nonneg, count));
    }

    // Block (e2): rest-boundary reachable-velocity envelope — nonneg cone.
    // At a true rest boundary a=0 (standstill), so the double-S reachable-b
    // envelope is the exact necessary bound; this closes the jerk-impulse hole
    // at b→0 where block (f)'s √b weight vanishes. A_env/J_env use the
    // max-over-grid projected caps so the envelope over-estimates reachability
    // (sound, never cuts the true optimum). Tsujita et al. arXiv 2202.10029 §III
    // impose exactly such per-grid reachable-envelope rows (their Eq. 17c).
    {
        let mut count = 0_usize;
        if endpoints.v_start == 0.0 {
            for i in 1..n {
                let d = grid.s[i] - grid.s[0];
                let cap = rest_boundary_b_cap(d, a_env, j_env);
                if cap >= b_cap {
                    break;
                }
                push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], cap);
                count += 1;
            }
        }
        if endpoints.v_end == 0.0 {
            for i in (0..n - 1).rev() {
                let d = grid.s[n - 1] - grid.s[i];
                let cap = rest_boundary_b_cap(d, a_env, j_env);
                if cap >= b_cap {
                    break;
                }
                push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], cap);
                count += 1;
            }
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    // Block (f): scalar-tangential jerk envelope — nonneg cone, 2·(N-2) rows.
    // |s⃛| ≤ J_path  →  |Δ²b_i| / (2h·J_path) ≤ h/√b_i ≡ t_i.
    // hj = 2·h·J_path; t_i ≥ ±Δ²b_i / hj.
    {
        let hj = 2.0 * h * j_path;
        let mut count = 0_usize;
        for k in 0..n_interior {
            let i = k + 1;
            let t_idx = off_t + k;

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

    // Block (g): x1, x2 nonnegativity — nonneg cone, 2·(N-2) rows.
    // Required by the SOC chain in block (h); Clarabel's SOC alone does not
    // enforce x1, x2 ≥ 0.
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

    // Block (h): SOC chain encoding t_i · b_i ≥ h² — standard SOC.
    // Uses the norm-form identity z² ≤ u·v ↔ ||(2z, u-v)|| ≤ u+v (u,v ≥ 0).
    // Three 3-element SOC blocks per interior point k:
    //   H1: x2_k² ≤ t_k · h     → ||(2x2_k, t_k-h)|| ≤ t_k+h
    //   H2: x1_k²/h ≤ t_k · b_i → ||(2x1_k/√h, t_k-b_i)|| ≤ t_k+b_i
    //   H3: h² ≤ x1_k · x2_k    → ||(2h, x1_k-x2_k)|| ≤ x1_k+x2_k
    {
        let sqrt_h = h.sqrt();
        let b_idx = |k: usize| -> usize { off_b + k + 1 };

        for k in 0..n_interior {
            let t_idx = off_t + k;
            let x1_idx = off_x1 + k;
            let x2_idx = off_x2 + k;
            let bi_idx = b_idx(k);

            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0)], h);
            push_row(&mut a_rows, &mut b_rhs, &[(x2_idx, 2.0)], 0.0);
            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0)], -h);
            cones.push((Cone::SecondOrder, 3));

            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0), (bi_idx, 1.0)], 0.0);
            push_row(&mut a_rows, &mut b_rhs, &[(x1_idx, 2.0 / sqrt_h)], 0.0);
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(t_idx, 1.0), (bi_idx, -1.0)],
                0.0,
            );
            cones.push((Cone::SecondOrder, 3));

            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(x1_idx, 1.0), (x2_idx, 1.0)],
                0.0,
            );
            push_row(&mut a_rows, &mut b_rhs, &[], 2.0 * h);
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(x1_idx, 1.0), (x2_idx, -1.0)],
                0.0,
            );
            cones.push((Cone::SecondOrder, 3));
        }
    }

    // Objective: min Σ t_k. Time-integral surrogate: t_i ≥ h/√b_i implies
    // Σ t_i ≥ Σ h/v_i ≈ ∫ ds/v(s) = T_total.
    let mut objective = vec![0.0_f64; n_vars];
    for k in 0..n_interior {
        objective[off_t + k] = 1.0;
    }

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

#[cfg(test)]
mod tests;
