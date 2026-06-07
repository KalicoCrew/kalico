use crate::Limits;
use crate::topp::chain::ChainGrid;
use crate::topp::scaling::SolverScale;

/// Cone descriptor in solver-agnostic form.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Cone {
    Zero,
    Nonneg,
    SecondOrder,
    RotatedSecondOrder,
}

/// Solver-agnostic constraint bundle produced by [`build_chain`].
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
    /// Per-interval arclength spacing, len `n_grid − 1`.
    pub h_intervals: Vec<f64>,
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

/// Chain-edge boundary conditions. `a_start = Some(_)` is only legal with
/// `v_start > 0` — pinning accel at a rest start forces `b_1 = 0` (the
/// rejected rest-pin trap); `build_chain` panics on it as a caller bug.
#[derive(Debug, Clone, Copy)]
pub struct EndpointConditions {
    pub v_start: f64,
    pub v_end: f64,
    pub a_start: Option<f64>,
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

/// `chain` and `endpoints` must share the unit system described by `scale`.
#[allow(clippy::too_many_lines)]
pub fn build_chain(
    chain: &ChainGrid,
    endpoints: EndpointConditions,
    scale: &SolverScale,
) -> BuildOutcome {
    let n = chain.n_points();
    debug_assert!(n >= 2, "ChainGrid must have at least 2 points");

    let kappa_floor = scale.to_scaled_kappa(KAPPA_FLOOR);
    let b_cap = scale.to_scaled_b(B_MAX_CENT_CAP);
    let h = &chain.h_intervals;
    let h_bar = |i: usize| -> f64 {
        if i == 0 {
            h[0]
        } else if i == n - 1 {
            h[n - 2]
        } else {
            0.5 * (h[i - 1] + h[i])
        }
    };

    let mut b_max_cent: Vec<f64> = (0..n)
        .map(|i| {
            let k = chain.geom[i].kappa;
            if k.abs() < kappa_floor {
                b_cap
            } else {
                (chain.limits_at(i).a_centripetal_max / k.abs()).min(b_cap)
            }
        })
        .collect();
    for j in &chain.junctions {
        let k = j.geom.kappa;
        let cap = if k.abs() < kappa_floor {
            b_cap
        } else {
            (chain.limits[j.limits_idx].a_centripetal_max / k.abs()).min(b_cap)
        };
        b_max_cent[j.idx] = b_max_cent[j.idx].min(cap);
    }

    let (a_env, j_env) = {
        let mut a_env = f64::NEG_INFINITY;
        let mut j_env = f64::NEG_INFINITY;
        for i in 0..n {
            let lim = chain.limits_at(i);
            let g = &chain.geom[i].c_prime;
            let mut a_tan_i = f64::INFINITY;
            let mut j_tan_i = f64::INFINITY;
            let mut active = false;
            for ax in 0..3 {
                let gabs = g[ax].abs();
                if gabs > COMP_FLOOR {
                    a_tan_i = a_tan_i.min(lim.a_max[ax] / gabs);
                    j_tan_i = j_tan_i.min(lim.j_max[ax] / gabs);
                    active = true;
                }
            }
            if active {
                a_env = a_env.max(a_tan_i);
                j_env = j_env.max(j_tan_i);
            }
        }
        for j in &chain.junctions {
            let lim = &chain.limits[j.limits_idx];
            let g = &j.geom.c_prime;
            let mut a_tan_i = f64::INFINITY;
            let mut j_tan_i = f64::INFINITY;
            let mut active = false;
            for ax in 0..3 {
                let gabs = g[ax].abs();
                if gabs > COMP_FLOOR {
                    a_tan_i = a_tan_i.min(lim.a_max[ax] / gabs);
                    j_tan_i = j_tan_i.min(lim.j_max[ax] / gabs);
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

    let j_path = chain
        .limits
        .iter()
        .flat_map(|l| l.j_max.iter().copied())
        .fold(f64::INFINITY, f64::min);
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

    // Block (a): boundary equalities + optional a_start pin.
    {
        let mut count = 0_usize;
        push_row(&mut a_rows, &mut b_rhs, &[(off_b, 1.0)], -b_start);
        count += 1;
        push_row(&mut a_rows, &mut b_rhs, &[(off_b + n - 1, 1.0)], -b_end);
        count += 1;
        if let Some(a0) = endpoints.a_start {
            assert!(
                endpoints.v_start > 0.0,
                "a_start pin at a rest start forces b_1 = 0 (rejected trap); \
                 rest starts use the (e2) envelope"
            );
            // b_1 − b_0 − 2·h_0·a_0 = 0  (Zero cone; b_0 already pinned).
            push_row(&mut a_rows, &mut b_rhs, &[(off_b + 1, 1.0), (off_b, -1.0)], -2.0 * h[0] * a0);
            count += 1;
        }
        cones.push((Cone::Zero, count));
    }

    // Block (b): acceleration linkage — zero cone, N rows.
    {
        let count = n;
        // Edge i=0: a_0 = (b_1 - b_0) / (2·h[0])
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a, 1.0),
                (off_b + 1, -1.0 / (2.0 * h[0])),
                (off_b, 1.0 / (2.0 * h[0])),
            ],
            0.0,
        );
        for i in 1..n - 1 {
            let w = crate::topp::stencil::b_d_weights(h[i - 1], h[i]);
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[
                    (off_a + i, 1.0),
                    (off_b + i - 1, -w[0] / 2.0),
                    (off_b + i, -w[1] / 2.0),
                    (off_b + i + 1, -w[2] / 2.0),
                ],
                0.0,
            );
        }
        // Edge i=n-1: a_{n-1} = (b_{n-1} - b_{n-2}) / (2·h[n-2])
        push_row(
            &mut a_rows,
            &mut b_rhs,
            &[
                (off_a + n - 1, 1.0),
                (off_b + n - 1, -1.0 / (2.0 * h[n - 2])),
                (off_b + n - 2, 1.0 / (2.0 * h[n - 2])),
            ],
            0.0,
        );
        cones.push((Cone::Zero, count));
    }

    // Block (c): per-axis velocity upper bound — nonneg cone.
    {
        let mut count = 0_usize;
        for i in 0..n {
            let lim = chain.limits_at(i);
            for ax in 0..3 {
                let g = chain.geom[i].c_prime[ax];
                if g.abs() < COMP_FLOOR {
                    continue;
                }
                let v_ax = lim.v_max[ax];
                let rhs = (v_ax / g).powi(2);
                if rhs > b_cap {
                    continue;
                }
                push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], rhs);
                count += 1;
            }
        }
        // Junction duals: the shared point belongs to both segments; emit the
        // right side's per-axis velocity caps too (right geometry AND right limits).
        for j in &chain.junctions {
            let i = j.idx;
            let lims = &chain.limits[j.limits_idx];
            for ax in 0..3 {
                let g = j.geom.c_prime[ax];
                if g.abs() < COMP_FLOOR {
                    continue;
                }
                let rhs = (lims.v_max[ax] / g).powi(2);
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
    {
        /// Skip rows whose worst-case LHS falls below this fraction of a_max.
        const BLOCK_D_SAFETY: f64 = 0.1;
        let mut count = 0_usize;
        for i in 0..n {
            let lim = chain.limits_at(i);
            let b_cap_i = b_max_cent[i].min(b_cap);
            let a_cap_i = b_cap_i / (2.0 * h_bar(i));
            for ax in 0..3 {
                let gp = chain.geom[i].c_prime[ax];
                let gpp = chain.geom[i].c_double_prime[ax];
                if gp.abs() < COMP_FLOOR && gpp.abs() < COMP_FLOOR {
                    continue;
                }
                let a_ax = lim.a_max[ax];
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
        // Junction duals: the shared point belongs to both segments; emit the
        // right side's per-axis accel caps too (right geometry AND right limits).
        for j in &chain.junctions {
            let i = j.idx;
            let lims = &chain.limits[j.limits_idx];
            let b_cap_i = b_max_cent[i].min(b_cap);
            let a_cap_i = b_cap_i / (2.0 * h_bar(i));
            for ax in 0..3 {
                let gp = j.geom.c_prime[ax];
                let gpp = j.geom.c_double_prime[ax];
                if gp.abs() < COMP_FLOOR && gpp.abs() < COMP_FLOOR {
                    continue;
                }
                let a_ax = lims.a_max[ax];
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
    {
        let count = n;
        for i in 0..n {
            push_row(&mut a_rows, &mut b_rhs, &[(off_b + i, -1.0)], b_max_cent[i]);
        }
        cones.push((Cone::Nonneg, count));
    }

    // Block (e2): rest-boundary reachable-velocity envelope — nonneg cone.
    {
        let mut count = 0_usize;
        if endpoints.v_start == 0.0 {
            for i in 1..n {
                let d = chain.s[i] - chain.s[0];
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
                let d = chain.s[n - 1] - chain.s[i];
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
    {
        let mut count = 0_usize;
        for k in 0..n_interior {
            let i = k + 1;
            let t_idx = off_t + k;
            let w = crate::topp::stencil::b_dd_weights(h[i - 1], h[i]);
            let c = h_bar(i) / (2.0 * j_path);
            for sign in [1.0_f64, -1.0] {
                push_row(
                    &mut a_rows,
                    &mut b_rhs,
                    &[
                        (t_idx, 1.0),
                        (off_b + i - 1, -sign * c * w[0]),
                        (off_b + i, -sign * c * w[1]),
                        (off_b + i + 1, -sign * c * w[2]),
                    ],
                    0.0,
                );
            }
            count += 2;
        }
        if count > 0 {
            cones.push((Cone::Nonneg, count));
        }
    }

    // Block (g): x1, x2 nonnegativity — nonneg cone, 2·(N-2) rows.
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

    // Block (h): SOC chain encoding t_i · b_i ≥ h_bar² — standard SOC.
    {
        let b_idx = |k: usize| -> usize { off_b + k + 1 };

        for k in 0..n_interior {
            let t_idx = off_t + k;
            let x1_idx = off_x1 + k;
            let x2_idx = off_x2 + k;
            let bi_idx = b_idx(k);
            let hk = h_bar(k + 1);
            let sqrt_hk = hk.sqrt();

            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0)], hk);
            push_row(&mut a_rows, &mut b_rhs, &[(x2_idx, 2.0)], 0.0);
            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0)], -hk);
            cones.push((Cone::SecondOrder, 3));

            push_row(&mut a_rows, &mut b_rhs, &[(t_idx, 1.0), (bi_idx, 1.0)], 0.0);
            push_row(&mut a_rows, &mut b_rhs, &[(x1_idx, 2.0 / sqrt_hk)], 0.0);
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
            push_row(&mut a_rows, &mut b_rhs, &[], 2.0 * hk);
            push_row(
                &mut a_rows,
                &mut b_rhs,
                &[(x1_idx, 1.0), (x2_idx, -1.0)],
                0.0,
            );
            cones.push((Cone::SecondOrder, 3));
        }
    }

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
        h_intervals: chain.h_intervals.clone(),
        j_path,
    })
}

#[cfg(test)]
mod tests;
