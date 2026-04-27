//! Row-sum identity for the per-axis Cartesian jerk SLP cut (spec §11; Step 9).
//!
//! This is the de-risk gate for Step 9. Before wiring the new cut into
//! Clarabel, we must numerically verify that the cut row coefficients
//! reproduce the verifier-stencil per-axis Cartesian jerk *exactly* at the
//! current iterate `(b̄, ā)`. If this identity fails, the cut math is wrong
//! and Step 9's wire-up will produce garbage; the test must pass for every
//! interior grid point and every axis (and for the boundary one-sided FD
//! variant at i=0 and i=N-1).
//!
//! ## The identity
//!
//! At iterate `(b̄, ā)`, the verifier-stencil per-axis Cartesian jerk is
//!
//! ```text
//!   j_axis(b̄, ā)_i =  C3 · b̄_i^(3/2)
//!                   + 3 · C2 · ā_i · √b̄_i
//!                   + C1 · D̄_i · √b̄_i
//! ```
//!
//! where `C1 = c'_axis(s_i)`, `C2 = c''_axis(s_i)`, `C3 = c'''_axis(s_i)`,
//! and `D̄_i` is the finite-difference of `ā` against `s` at index `i`:
//! central FD `(ā_{i+1} − ā_{i-1})/(2h)` for interior, one-sided
//! `(ā_1 − ā_0)/h` at i=0 and `(ā_{N-1} − ā_{N-2})/h` at i=N-1.
//!
//! The first-order Taylor linearization of `j_axis` at `(b̄, ā)` is
//!
//! ```text
//!   j_lin(b, a) =  α_b · b_i + α_{a-1} · a_{i-1} + α_a · a_i + α_{a+1} · a_{i+1} + K
//! ```
//!
//! By construction the linearization is exact at the iterate:
//!
//! ```text
//!   α_b · b̄_i + α_{a-1} · ā_{i-1} + α_a · ā_i + α_{a+1} · ā_{i+1} + K  ≡  j_axis(b̄, ā)_i.
//! ```
//!
//! This test pins that identity numerically.

#![allow(clippy::doc_markdown)]
#![allow(clippy::too_many_arguments)]
#![allow(clippy::items_after_statements)]

use nurbs::VectorNurbs;
use temporal::topp::path::sample_arclength_grid;
use temporal::{schedule_segment, GridConfig, GridScheme, Limits};

fn textbook_limits() -> Limits {
    Limits::new(
        [500.0, 500.0, 500.0],
        [5_000.0, 5_000.0, 5_000.0],
        [100_000.0, 100_000.0, 100_000.0],
        2_500.0,
    )
}

/// Verifier-stencil per-axis Cartesian jerk at iterate `(b̄, ā)`,
/// matching `topp::verify::check` exactly.
fn j_axis_at_iterate(
    cp: f64,
    cpp: f64,
    cppp: f64,
    b_bar_i: f64,
    a_bar_i: f64,
    da_ds_i: f64,
) -> f64 {
    let s_dot = b_bar_i.max(0.0).sqrt();
    let s_dot3 = s_dot * s_dot * s_dot;
    let s_dddot = da_ds_i * s_dot;
    cppp * s_dot3 + 3.0 * cpp * s_dot * a_bar_i + cp * s_dddot
}

/// One-sided / central FD `da/ds` at grid index `i`, mirroring
/// `topp::verify::da_ds_at`.
fn da_ds_at(a: &[f64], s: &[f64], i: usize) -> f64 {
    let n = s.len();
    if n <= 1 {
        return 0.0;
    }
    if i == 0 {
        let ds = s[1] - s[0];
        if ds.abs() > 1e-15 {
            (a[1] - a[0]) / ds
        } else {
            0.0
        }
    } else if i == n - 1 {
        let ds = s[n - 1] - s[n - 2];
        if ds.abs() > 1e-15 {
            (a[n - 1] - a[n - 2]) / ds
        } else {
            0.0
        }
    } else {
        let ds = s[i + 1] - s[i - 1];
        if ds.abs() > 1e-15 {
            (a[i + 1] - a[i - 1]) / ds
        } else {
            0.0
        }
    }
}

/// Cut-row coefficients at interior i (central FD on `a`).
///
/// Variables touched: `b_i`, `a_{i-1}`, `a_i`, `a_{i+1}` (and a constant K).
/// Returns `(α_b, α_a_im1, α_a_i, α_a_ip1, K)`.
fn interior_cut_coeffs(
    cp: f64,
    cpp: f64,
    cppp: f64,
    b_bar_i: f64,
    a_bar_im1: f64,
    a_bar_i: f64,
    a_bar_ip1: f64,
    h: f64,
) -> (f64, f64, f64, f64, f64) {
    let sqrt_b = b_bar_i.max(0.0).sqrt();
    let b_pow_3_2 = sqrt_b * sqrt_b * sqrt_b;
    // D̄ := (ā_{i+1} − ā_{i-1}) / (2h)  — used implicitly below.

    // α_b  = (3/2)·C3·√b̄  +  3·C2·ā_i / (2·√b̄)  +  C1·D̄ / (2·√b̄)
    //      = (3/2)·C3·√b̄  +  3·C2·ā_i / (2·√b̄)  +  C1·(ā_{i+1} − ā_{i-1}) / (4h·√b̄)
    let alpha_b = if sqrt_b > 0.0 {
        1.5 * cppp * sqrt_b
            + 3.0 * cpp * a_bar_i / (2.0 * sqrt_b)
            + cp * (a_bar_ip1 - a_bar_im1) / (4.0 * h * sqrt_b)
    } else {
        // sqrt_b = 0: the only well-defined contribution is from C3 (which
        // multiplies √b̄ to a positive power); the other partials blow up but
        // are dotted with ā quantities that, by construction at b̄=0, leave
        // the row-sum identity vacuously trivial (j_axis = 0 at b̄=0). Use
        // 0.0 to avoid NaN.
        1.5 * cppp * sqrt_b
    };
    let alpha_a_im1 = -cp * sqrt_b / (2.0 * h);
    let alpha_a_i = 3.0 * cpp * sqrt_b;
    let alpha_a_ip1 = cp * sqrt_b / (2.0 * h);

    // K = −(1/2)·C3·b̄^(3/2)
    //     − (3/2)·C2·ā_i·√b̄
    //     − C1·D̄·√b̄ / 2
    //   = −(1/2)·C3·b̄^(3/2)
    //     − (3/2)·C2·ā_i·√b̄
    //     − C1·(ā_{i+1} − ā_{i-1})·√b̄ / (4h)
    let k = -0.5 * cppp * b_pow_3_2
        - 1.5 * cpp * a_bar_i * sqrt_b
        - cp * (a_bar_ip1 - a_bar_im1) * sqrt_b / (4.0 * h);

    (alpha_b, alpha_a_im1, alpha_a_i, alpha_a_ip1, k)
}

/// Cut-row coefficients at boundary i=0 (forward one-sided FD on `a`).
///
/// Variables touched: `b_0`, `a_0`, `a_1`. Returns `(α_b, α_a_0, α_a_1, K)`.
fn boundary_start_cut_coeffs(
    cp: f64,
    cpp: f64,
    cppp: f64,
    b_bar_0: f64,
    a_bar_0: f64,
    a_bar_1: f64,
    h: f64,
) -> (f64, f64, f64, f64) {
    let sqrt_b = b_bar_0.max(0.0).sqrt();
    let b_pow_3_2 = sqrt_b * sqrt_b * sqrt_b;
    // D̄ = (ā_1 − ā_0) / h
    let d_bar = (a_bar_1 - a_bar_0) / h;

    // α_b = (3/2)·C3·√b̄ + 3·C2·ā_0 / (2·√b̄) + C1·(ā_1 − ā_0) / (2h·√b̄)
    let alpha_b = if sqrt_b > 0.0 {
        1.5 * cppp * sqrt_b
            + 3.0 * cpp * a_bar_0 / (2.0 * sqrt_b)
            + cp * (a_bar_1 - a_bar_0) / (2.0 * h * sqrt_b)
    } else {
        1.5 * cppp * sqrt_b
    };
    // α_a_0 = 3·C2·√b̄ − C1·√b̄ / h
    let alpha_a_0 = 3.0 * cpp * sqrt_b - cp * sqrt_b / h;
    // α_a_1 = +C1·√b̄ / h
    let alpha_a_1 = cp * sqrt_b / h;

    // K = −(1/2)·C3·b̄^(3/2)  −  (3/2)·C2·ā_0·√b̄  −  C1·D̄·√b̄ / 2
    let k = -0.5 * cppp * b_pow_3_2 - 1.5 * cpp * a_bar_0 * sqrt_b - cp * d_bar * sqrt_b / 2.0;

    (alpha_b, alpha_a_0, alpha_a_1, k)
}

/// Cut-row coefficients at boundary i=N-1 (backward one-sided FD on `a`).
///
/// Variables touched: `b_{N-1}`, `a_{N-2}`, `a_{N-1}`. Returns
/// `(α_b, α_a_Nm2, α_a_Nm1, K)`.
fn boundary_end_cut_coeffs(
    cp: f64,
    cpp: f64,
    cppp: f64,
    b_bar_nm1: f64,
    a_bar_nm2: f64,
    a_bar_nm1: f64,
    h: f64,
) -> (f64, f64, f64, f64) {
    let sqrt_b = b_bar_nm1.max(0.0).sqrt();
    let b_pow_3_2 = sqrt_b * sqrt_b * sqrt_b;
    // D̄ = (ā_{N-1} − ā_{N-2}) / h
    let d_bar = (a_bar_nm1 - a_bar_nm2) / h;

    // α_b = (3/2)·C3·√b̄ + 3·C2·ā_{N-1} / (2·√b̄) + C1·D̄ / (2·√b̄)
    let alpha_b = if sqrt_b > 0.0 {
        1.5 * cppp * sqrt_b
            + 3.0 * cpp * a_bar_nm1 / (2.0 * sqrt_b)
            + cp * (a_bar_nm1 - a_bar_nm2) / (2.0 * h * sqrt_b)
    } else {
        1.5 * cppp * sqrt_b
    };
    // α_a_{N-2} = -C1·√b̄ / h
    let alpha_a_nm2 = -cp * sqrt_b / h;
    // α_a_{N-1} = 3·C2·√b̄ + C1·√b̄ / h
    let alpha_a_nm1 = 3.0 * cpp * sqrt_b + cp * sqrt_b / h;

    let k = -0.5 * cppp * b_pow_3_2 - 1.5 * cpp * a_bar_nm1 * sqrt_b - cp * d_bar * sqrt_b / 2.0;

    (alpha_b, alpha_a_nm2, alpha_a_nm1, k)
}

/// Build the same G5 cubic NURBS as fixture 4
/// (`single_g5_emits_one_cubic_fitted_segment` from rust/geometry/tests/g5_reduction.rs).
///
/// G-code: `G1 X0 Y0 F1500` → `G5 X10 Y0 I3 J3 P-3 Q3`
/// Produces degree-3 non-rational NURBS with control points
/// P0=(0,0,0), P1=(3,3,0), P2=(7,3,0), P3=(10,0,0).
fn build_g5_via_geometry() -> VectorNurbs<f64, 3> {
    use geometry::{FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};

    let src = "G1 X0 Y0 F1500\nG5 X10 Y0 I3 J3 P-3 Q3\n";
    let mut pipeline = GeometryPipeline::new(FitterParams::default());
    let mut events: Vec<TelemetryEvent> = vec![];
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        pipeline.process(src, &mut sink).collect()
    };
    items
        .into_iter()
        .find_map(|it| match it {
            Item::Segment(Segment::Fitted(f)) if f.degree == 3 => Some(f.xyz),
            _ => None,
        })
        .expect("G5 reduction must emit exactly one degree-3 FittedSegment")
}

/// Compute `b_max_cent` at the endpoints by sampling κ. Mirrors the helper in
/// `prototype.rs::fixture_4_g5_cubic`.
fn mvc_endpoints(curve: &VectorNurbs<f64, 3>, limits: &Limits) -> (f64, f64) {
    let grid = sample_arclength_grid(curve, 3).expect("arclength grid");
    let kappa_start = grid.kappa[0];
    let kappa_end = *grid.kappa.last().expect("≥ 2 points");
    let b_start = (limits.a_centripetal_max / kappa_start.max(1e-12)).min(1e8);
    let b_end = (limits.a_centripetal_max / kappa_end.max(1e-12)).min(1e8);
    (b_start, b_end)
}

/// The row-sum identity test. For every interior grid point i and every axis
/// (X, Y, Z), the cut row coefficients computed at the iterate `(b̄, ā)` from
/// `schedule_segment` satisfy
///
/// ```text
///   |α_b · b̄_i + α_{a-1} · ā_{i-1} + α_a · ā_i + α_{a+1} · ā_{i+1} + K
///    − j_axis(b̄, ā)_i|  <  1e-9.
/// ```
///
/// The same identity is checked at i=0 (forward FD) and i=N-1 (backward FD)
/// using the boundary-variant coefficients.
#[test]
fn row_sum_identity_holds_on_g5_cubic() {
    let curve = build_g5_via_geometry();
    let limits = textbook_limits();
    let cfg = GridConfig {
        scheme: GridScheme::UniformArclength,
        n: 200,
    };

    // Reproduce fixture 4's endpoint-velocity choice: 50 % of MVC.
    let (mvc_b_start, mvc_b_end) = mvc_endpoints(&curve, &limits);
    let v_start = 0.5 * mvc_b_start.sqrt();
    let v_end = 0.5 * mvc_b_end.sqrt();

    let profile = schedule_segment(&curve, &limits, &cfg, v_start, v_end)
        .expect("schedule_segment must not error at setup");

    // Pull the iterate b̄, ā out of `samples`. (For Step 9 wire-up purposes
    // the identity holds at *any* iterate; the values that come out of
    // `schedule_segment` are the SOCP/SLP-converged iterate, which is what
    // the cut would be linearizing around in practice.)
    let n = profile.samples.len();
    assert_eq!(n, cfg.n);
    let b_bar: Vec<f64> = profile.samples.iter().map(|s| s.b).collect();
    let a_bar: Vec<f64> = profile.samples.iter().map(|s| s.a).collect();

    // Re-sample arclength grid for c', c'', c''' (these aren't in TopProfile).
    let grid = sample_arclength_grid(&curve, cfg.n).expect("arclength");
    let h = grid.s[1] - grid.s[0];

    // Tolerance: the identity is exact in real arithmetic. Floating-point
    // round-off in the chain of multiplications/divisions accumulates to
    // about a few ulps relative to |j_axis|, plus the absolute floor at the
    // C3·b^(3/2) term. 1e-9 absolute is comfortable; we additionally relax
    // to 1e-6 *relative* to |j_axis| when the absolute value is large.
    const TOL_ABS: f64 = 1e-9;
    const TOL_REL: f64 = 1e-9;

    let mut max_residual_abs: f64 = 0.0;
    let mut max_residual_rel: f64 = 0.0;
    let mut worst_locus = (0_usize, 0_usize); // (i, axis)

    for i in 0..n {
        for ax in 0..3 {
            let cp = grid.c_prime[i][ax];
            let cpp = grid.c_double_prime[i][ax];
            let cppp = grid.c_triple_prime[i][ax];

            let da_ds = da_ds_at(&a_bar, &grid.s, i);
            let j_actual = j_axis_at_iterate(cp, cpp, cppp, b_bar[i], a_bar[i], da_ds);

            let j_lin: f64 = if i == 0 {
                let (alpha_b, alpha_a_0, alpha_a_1, k) =
                    boundary_start_cut_coeffs(cp, cpp, cppp, b_bar[0], a_bar[0], a_bar[1], h);
                alpha_b * b_bar[0] + alpha_a_0 * a_bar[0] + alpha_a_1 * a_bar[1] + k
            } else if i == n - 1 {
                let (alpha_b, alpha_a_nm2, alpha_a_nm1, k) = boundary_end_cut_coeffs(
                    cp,
                    cpp,
                    cppp,
                    b_bar[n - 1],
                    a_bar[n - 2],
                    a_bar[n - 1],
                    h,
                );
                alpha_b * b_bar[n - 1] + alpha_a_nm2 * a_bar[n - 2] + alpha_a_nm1 * a_bar[n - 1] + k
            } else {
                let (alpha_b, alpha_a_im1, alpha_a_i, alpha_a_ip1, k) = interior_cut_coeffs(
                    cp,
                    cpp,
                    cppp,
                    b_bar[i],
                    a_bar[i - 1],
                    a_bar[i],
                    a_bar[i + 1],
                    h,
                );
                alpha_b * b_bar[i]
                    + alpha_a_im1 * a_bar[i - 1]
                    + alpha_a_i * a_bar[i]
                    + alpha_a_ip1 * a_bar[i + 1]
                    + k
            };

            let abs_resid = (j_lin - j_actual).abs();
            let rel_resid = abs_resid / j_actual.abs().max(1.0);
            if abs_resid > max_residual_abs {
                max_residual_abs = abs_resid;
                worst_locus = (i, ax);
            }
            if rel_resid > max_residual_rel {
                max_residual_rel = rel_resid;
            }

            assert!(
                abs_resid < TOL_ABS || rel_resid < TOL_REL,
                "row-sum identity broken at (i={}, axis={}): \
                 j_lin = {:.12e}, j_actual = {:.12e}, abs_resid = {:.3e}, rel_resid = {:.3e}, \
                 b̄_i = {:.6e}, ā_i = {:.6e}, h = {:.6e}, cp = {:.6e}, cpp = {:.6e}, cppp = {:.6e}",
                i,
                ax,
                j_lin,
                j_actual,
                abs_resid,
                rel_resid,
                b_bar[i],
                a_bar[i],
                h,
                cp,
                cpp,
                cppp,
            );
        }
    }

    eprintln!(
        "row-sum identity: max abs residual = {:.3e} at (i={}, axis={}); max rel = {:.3e}",
        max_residual_abs, worst_locus.0, worst_locus.1, max_residual_rel,
    );
}
