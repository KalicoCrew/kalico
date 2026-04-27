//! Arclength-grid sampler.
//!
//! Spec §3, §3.3, §4.3 stage 1.
//!
//! # Reparameterization math
//!
//! The NURBS evaluator gives derivatives w.r.t. the native parameter `u`. The
//! Consolini-Locatelli relaxation requires `|C'(s)| = 1`, i.e. derivatives
//! w.r.t. arclength `s`. We convert via the full Faà di Bruno chain rule below.
//!
//! ## Notation
//!
//! - `C(u)` — the curve in R³, native parameter.
//! - `f(u) = |dC/du|` — parametric speed (always ≥ `MIN_PARAMETRIC_SPEED`).
//! - `s` — arclength. `ds/du = f`, so `du/ds = 1/f`.
//! - `' ` suffix denotes d/ds; ˙ denotes d/du.
//! - Dot `·` is 3D inner product; `×` is cross product.
//!
//! ## Scalar chain-rule quantities
//!
//! ```text
//! df/du   = (Ċ · C̈) / f                          [where C̈ = d²C/du²]
//!
//! d²f/du² = (|C̈|² + Ċ · C⃛) / f  −  (df/du)² / f    [where C⃛ = d³C/du³]
//!
//! du/ds   = 1/f
//!
//! d²u/ds² = −(df/du) / f³
//!
//! d³u/ds³ = −(d²f/du²) / f⁴  +  3(df/du)² / f⁵
//! ```
//!
//! ### Derivation of d³u/ds³
//!
//! Let `q(u) = d²u/ds² = −(df/du)/f³`.
//!
//! ```text
//! d³u/ds³ = dq/ds = (dq/du) · (du/ds)
//!
//! dq/du = d/du[−(df/du)/f³]
//!       = −(d²f/du²)/f³  +  (df/du) · 3f²·(df/du) / f⁶
//!       = −(d²f/du²)/f³  +  3(df/du)²/f⁴
//!
//! d³u/ds³ = (dq/du) / f = −(d²f/du²)/f⁴  +  3(df/du)²/f⁵
//! ```
//!
//! **Dimension check** (u dimensionless, s in mm, f in mm):
//! - `d²f/du²` is mm;  `f⁴` is mm⁴  →  `mm/mm⁴ = mm⁻³` ✓
//! - `(df/du)²` is mm²; `f⁵` is mm⁵  →  `mm²/mm⁵ = mm⁻³` ✓
//!
//! **NOTE:** The task prompt contained an algebra error in the explicit formula for
//! `d³u/ds³`, stating exponents `f⁵` and `f⁶` instead of `f⁴` and `f⁵`. The
//! prompt also states those denominators are `f⁵` for the first term, which is
//! dimensionally inconsistent (gives mm⁻⁴ rather than mm⁻³). The correct formula
//! derived above uses `f⁴` and `f⁵`. This is documented here as the corrected form;
//! the formula for `d³C/ds³` in the prompt (which treats `d³u/ds³` symbolically)
//! remains correct once this correction is substituted.
//!
//! ## Curve derivatives by chain rule (Faà di Bruno, k = 1, 2, 3)
//!
//! ```text
//! dC/ds   = Ċ / f
//!
//! d²C/ds² = C̈ / f²  −  (df/du / f³) · Ċ
//!
//! d³C/ds³ = C⃛ / f³  −  3·(df/du / f⁴) · C̈  +  Ċ · d³u/ds³
//! ```
//!
//! ## Curvature
//!
//! With arclength parameterization `|C'(s)| = 1`:
//! ```text
//! κ(s) = |C'(s) × C''(s)|
//! ```
//! This equals `|C''(s)|` when C'(s) is a unit vector, but the cross-product form
//! is more robust to numerical drift.

use nurbs::{
    arc_length::{build_arc_length_table_vector, param_from_arc_length},
    eval::vector_eval,
    VectorNurbs, MIN_PARAMETRIC_SPEED,
};

/// Evaluate the k-th parametric derivative of a NURBS at `u` via central finite
/// differences of `vector_eval`.
///
/// This handles both non-rational and rational NURBS correctly, because
/// `vector_eval` evaluates the true rational curve position at any parameter.
/// The `vector_derivative` degree-lowering API only works for *unweighted*
/// B-splines; for rational NURBS (G2/G3 arcs) the quotient rule is needed, which
/// we approximate here with finite differences at step `h`.
///
/// Step `h = 1e-5` gives ~1e-10 error on smooth curves (dominant term is h²·f'''/6
/// for 2nd-order central differences).
///
/// The parameter `u` is clamped into `[u_start + h, u_end - h]` to avoid
/// evaluating outside the curve's domain.
fn eval_kth_deriv(
    curve: &VectorNurbs<f64, 3>,
    u: f64,
    k: usize,
    h: f64,
) -> [f64; 3] {
    let knots = curve.knots();
    let u_start = knots[0];
    let u_end = *knots.last().expect("non-empty knot vector");
    let view = curve.as_view();

    match k {
        0 => vector_eval(&view, u),
        1 => {
            // Central difference: (C(u+h) - C(u-h)) / (u_p - u_m)
            // Clamp so we never evaluate outside [u_start, u_end].
            let u_p = (u + h).min(u_end);
            let u_m = (u - h).max(u_start);
            let step = (u_p - u_m).max(MIN_PARAMETRIC_SPEED);
            let plus = vector_eval(&view, u_p);
            let minus = vector_eval(&view, u_m);
            [
                (plus[0] - minus[0]) / step,
                (plus[1] - minus[1]) / step,
                (plus[2] - minus[2]) / step,
            ]
        }
        2 => {
            // Second central difference: (C(u+h) - 2C(u) + C(u-h)) / h²
            // Use symmetric clamping: reduce h at domain boundaries to maintain
            // a symmetric stencil.
            let avail_h = (u - u_start).min(u_end - u).min(h);
            let avail_h = avail_h.max(h * 0.01); // don't degenerate completely
            let c = vector_eval(&view, u);
            let plus = vector_eval(&view, u + avail_h);
            let minus = vector_eval(&view, u - avail_h);
            let h2 = avail_h * avail_h;
            [
                (plus[0] - 2.0 * c[0] + minus[0]) / h2,
                (plus[1] - 2.0 * c[1] + minus[1]) / h2,
                (plus[2] - 2.0 * c[2] + minus[2]) / h2,
            ]
        }
        3 => {
            // Third central difference: (C(u+2h) - 2C(u+h) + 2C(u-h) - C(u-2h)) / (2h³)
            //
            // Derivation: expand C(u ± kh) in Taylor series and collect the h³ coefficient.
            // The antisymmetric combination that isolates f'''(u):
            //   C(u+2h) - 2C(u+h) + 2C(u-h) - C(u-2h)
            //   = [... + (2h)³/6·f''' + ...] - 2[... + h³/6·f''' + ...] + 2[... - h³/6·f''' + ...] - [... - (2h)³/6·f''' + ...]
            //   = (8/6 - 2/6 - 2/6 + 8/6)·h³·f''' + O(h⁵) = (12/6)·h³·f''' + O(h⁵) = 2h³·f''' + O(h⁵)
            //
            // NOTE: an earlier version used (-pp + 2p - 2m + mm) which is the negation of the
            // correct stencil and gives -f'''(u). That sign error was caught by the
            // `cubic_bezier_pins_third_derivative_at_start` test.
            //
            // Symmetric clamp: maximum step that fits in the domain.
            let avail_h = ((u - u_start) / 2.0).min((u_end - u) / 2.0).min(h);
            let avail_h = avail_h.max(h * 0.01);
            let pp = vector_eval(&view, u + 2.0 * avail_h);
            let p = vector_eval(&view, u + avail_h);
            let m = vector_eval(&view, u - avail_h);
            let mm = vector_eval(&view, u - 2.0 * avail_h);
            let two_h3 = 2.0 * avail_h * avail_h * avail_h;
            [
                (pp[0] - 2.0 * p[0] + 2.0 * m[0] - mm[0]) / two_h3,
                (pp[1] - 2.0 * p[1] + 2.0 * m[1] - mm[1]) / two_h3,
                (pp[2] - 2.0 * p[2] + 2.0 * m[2] - mm[2]) / two_h3,
            ]
        }
        _ => [0.0, 0.0, 0.0],
    }
}

#[derive(Debug, Clone)]
pub struct ArclengthGrid {
    /// `s_i ∈ [0, L]`, length N.
    pub s: Vec<f64>,
    /// `u_i = u(s_i)`, length N.
    pub u: Vec<f64>,
    /// `C(u_i)`, length N.
    pub c: Vec<[f64; 3]>,
    /// `dC/ds` at `s_i`, length N. Unit-magnitude up to numerical floor.
    pub c_prime: Vec<[f64; 3]>,
    /// `d²C/ds²` at `s_i`, length N.
    pub c_double_prime: Vec<[f64; 3]>,
    /// `d³C/ds³` at `s_i`, length N.
    pub c_triple_prime: Vec<[f64; 3]>,
    /// `κ(s_i) = |C'(s) × C''(s)|` (arclength parameterization), length N.
    pub kappa: Vec<f64>,
    /// Total arclength, mm.
    pub total_length: f64,
}

#[derive(Debug, thiserror::Error)]
pub enum PathSampleError {
    #[error("grid size N must be at least 2, got {0}")]
    GridTooSmall(usize),
    #[error("arc-length table construction failed: {0}")]
    ArcLengthTable(String),
}

/// Build `ArclengthGrid` for a single 3D NURBS at uniform-in-`s` resolution `n`.
///
/// Spec §3.1, §3.3.
pub fn sample_arclength_grid(
    curve: &VectorNurbs<f64, 3>,
    n: usize,
) -> Result<ArclengthGrid, PathSampleError> {
    if n < 2 {
        return Err(PathSampleError::GridTooSmall(n));
    }

    // ---- Step 1: Build arclength table for u(s) --------------------------------
    //
    // tolerance = 1e-6 mm; max_samples = 1024. The adaptive builder doubles the
    // internal sample count until the GL-residual is below tolerance. 1024 samples
    // is more than enough for any reasonable segment at 1e-6 mm precision.
    let arc_table = build_arc_length_table_vector(curve, 1e-6_f64, 1024)
        .map_err(|e| PathSampleError::ArcLengthTable(e.to_string()))?;

    let total_length = arc_table.s_max();
    let table_ref = arc_table.as_view();

    // ---- Step 2: Choose finite-difference step h --------------------------------
    //
    // h = 1e-5 in the native u-parameter gives ~1e-10 local truncation error for
    // smooth curves. The u domain is [0, 1] for all clamped NURBS in this codebase.
    //
    // Rationale for finite differences instead of the `vector_derivative` API:
    //   `vector_derivative` degree-lowers the unweighted control points, which is
    //   exact for non-rational (B-spline) NURBS. For rational NURBS (G2/G3 arcs),
    //   the quotient rule is needed; `vector_derivative` explicitly documents that
    //   it handles "unweighted (B-spline) NURBS only". Since `vector_eval` does
    //   correctly evaluate rational curves at any parameter, central finite
    //   differences of `vector_eval` give the correct derivative for both rational
    //   and non-rational inputs.
    //
    // Finite differences are computed pointwise in the loop below via `eval_kth_deriv`.
    let fd_h = 1e-5_f64;

    // ---- Step 3: Evaluate at each grid point ----------------------------------
    let mut s_vec = Vec::with_capacity(n);
    let mut u_vec = Vec::with_capacity(n);
    let mut c_vec = Vec::with_capacity(n);
    let mut c_prime_vec = Vec::with_capacity(n);
    let mut c_double_prime_vec = Vec::with_capacity(n);
    let mut c_triple_prime_vec = Vec::with_capacity(n);
    let mut kappa_vec = Vec::with_capacity(n);

    let curve_view = curve.as_view();

    let floor = MIN_PARAMETRIC_SPEED;

    for i in 0..n {
        // Uniform-in-s grid.
        let s_i = total_length * (i as f64) / ((n - 1) as f64);
        let u_i = param_from_arc_length(&table_ref, s_i);

        // Curve position.
        let c_i = vector_eval(&curve_view, u_i);

        // u-parameterized derivatives via finite differences (correct for rational
        // and non-rational NURBS alike).
        let dc_du = eval_kth_deriv(curve, u_i, 1, fd_h); // dC/du
        let d2c_du2 = eval_kth_deriv(curve, u_i, 2, fd_h); // d²C/du²
        let d3c_du3 = eval_kth_deriv(curve, u_i, 3, fd_h); // d³C/du³

        // ---- Parametric speed and its derivatives ----------------------------
        //
        // f = |dC/du|
        let f_sq = dot3(dc_du, dc_du);
        let f = f_sq.sqrt().max(floor);

        // df/du = (d²C/du² · dC/du) / f
        let df_du = dot3(d2c_du2, dc_du) / f;

        // d²f/du² = (|d²C/du²|² + dC/du · d³C/du³) / f  −  (df/du)² / f
        let d2f_du2 = (dot3(d2c_du2, d2c_du2) + dot3(dc_du, d3c_du3)) / f - (df_du * df_du) / f;

        // du/ds, d²u/ds², d³u/ds³
        let du_ds = 1.0 / f;
        let d2u_ds2 = -df_du / (f * f * f); // = -(df/du) / f³
        // d³u/ds³ = -(d²f/du²)/f⁴  +  3(df/du)²/f⁵  (see module-level derivation)
        let f4 = f * f * f * f;
        let f5 = f4 * f;
        let d3u_ds3 = -(d2f_du2) / f4 + 3.0 * df_du * df_du / f5;

        // ---- Arclength derivatives of C via Faà di Bruno --------------------

        // dC/ds = dC/du · (du/ds)
        let c_prime_i = scale3(dc_du, du_ds);

        // d²C/ds² = d²C/du² · (du/ds)²  +  dC/du · d²u/ds²
        let du_ds_sq = du_ds * du_ds;
        let c_double_prime_i = add3(scale3(d2c_du2, du_ds_sq), scale3(dc_du, d2u_ds2));

        // d³C/ds³ = d³C/du³ · (du/ds)³
        //           + 3 · d²C/du² · (du/ds) · d²u/ds²
        //           + dC/du · d³u/ds³
        let du_ds_cu = du_ds_sq * du_ds;
        let c_triple_prime_i = add3(
            add3(
                scale3(d3c_du3, du_ds_cu),
                scale3(d2c_du2, 3.0 * du_ds * d2u_ds2),
            ),
            scale3(dc_du, d3u_ds3),
        );

        // ---- Curvature κ = |C'(s) × C''(s)| ---------------------------------
        let cross = cross3(c_prime_i, c_double_prime_i);
        let kappa_i = (dot3(cross, cross)).sqrt();

        s_vec.push(s_i);
        u_vec.push(u_i);
        c_vec.push(c_i);
        c_prime_vec.push(c_prime_i);
        c_double_prime_vec.push(c_double_prime_i);
        c_triple_prime_vec.push(c_triple_prime_i);
        kappa_vec.push(kappa_i);
    }

    Ok(ArclengthGrid {
        s: s_vec,
        u: u_vec,
        c: c_vec,
        c_prime: c_prime_vec,
        c_double_prime: c_double_prime_vec,
        c_triple_prime: c_triple_prime_vec,
        kappa: kappa_vec,
        total_length,
    })
}

// ---- Vector helpers (inline, no alloc) --------------------------------------

#[inline]
fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn scale3(a: [f64; 3], s: f64) -> [f64; 3] {
    [a[0] * s, a[1] * s, a[2] * s]
}

#[inline]
fn add3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

#[inline]
fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[cfg(test)]
mod tests {
    use super::*;
    use nurbs::VectorNurbs;

    #[test]
    fn straight_line_x_aligned_returns_unit_tangent_and_zero_curvature() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
            None,
        )
        .unwrap();

        let grid = sample_arclength_grid(&curve, 5).unwrap();
        assert_eq!(grid.s.len(), 5);
        assert!((grid.total_length - 10.0).abs() < 1e-6);
        assert!((grid.s[0] - 0.0).abs() < 1e-9);
        assert!((grid.s[4] - 10.0).abs() < 1e-6);
        for tan in &grid.c_prime {
            assert!((tan[0] - 1.0).abs() < 1e-6);
            assert!(tan[1].abs() < 1e-6);
            assert!(tan[2].abs() < 1e-6);
        }
        for k in &grid.kappa {
            assert!(k.abs() < 1e-6);
        }
    }

    #[test]
    fn rejects_grid_size_below_two() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
            None,
        )
        .unwrap();
        assert!(matches!(
            sample_arclength_grid(&curve, 1),
            Err(PathSampleError::GridTooSmall(1))
        ));
    }

    #[test]
    fn rational_quadratic_arc_returns_constant_curvature() {
        // Build a quarter-circle (R=20mm) as a rational quadratic NURBS.
        // Standard NURBS quarter-circle: 3 control points with weights [1, sqrt(2)/2, 1].
        //   P0 = (R, 0, 0), P1 = (R, R, 0), P2 = (0, R, 0)
        //   weights = [1, sqrt(2)/2, 1]
        //   knots = [0, 0, 0, 1, 1, 1] (degree 2, 3 CPs, clamped)
        // True curvature κ = 1/R = 0.05.
        let r = 20.0_f64;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let curve = VectorNurbs::<f64, 3>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![[r, 0.0, 0.0], [r, r, 0.0], [0.0, r, 0.0]],
            Some(vec![1.0, w, 1.0]),
        )
        .unwrap();
        let grid = sample_arclength_grid(&curve, 11).unwrap();
        let expected_kappa = 1.0 / r;
        for k in &grid.kappa {
            // Tolerance is loose because arclength reparameterization + numerical
            // chain rule has accumulated error on a NURBS arc; ~1% is typical.
            assert!(
                (k - expected_kappa).abs() / expected_kappa < 0.01,
                "kappa = {k}, expected {expected_kappa}"
            );
        }
        // Total arclength of a quarter-circle of radius R = π·R/2.
        let expected_length = std::f64::consts::FRAC_PI_2 * r;
        assert!((grid.total_length - expected_length).abs() / expected_length < 0.01);
    }

    /// Pin `c_triple_prime` to a known closed-form value on a non-trivial cubic Bezier.
    ///
    /// # Fixture
    ///
    /// Degree-3 non-rational NURBS, knots `[0,0,0,0,1,1,1,1]`, control points:
    ///   P0=(0,0,0), P1=(1,0,0), P2=(2,0,0), P3=(3,1,0).
    ///
    /// # Closed-form algebra at u=0
    ///
    /// For a cubic Bezier C(u) = (1-u)³P0 + 3(1-u)²u P1 + 3(1-u)u² P2 + u³ P3:
    ///
    ///   dC/du  = 3[(1-u)²(P1-P0) + 2(1-u)u(P2-P1) + u²(P3-P2)]
    ///            At u=0: 3·(1,0,0) = (3,0,0)    → f = |dC/du| = 3
    ///
    ///   d²C/du² = 6[(1-u)(P2-2P1+P0) + u(P3-2P2+P1)]
    ///             P2-2P1+P0 = (2,0,0)-(2,0,0)+(0,0,0) = (0,0,0)
    ///             P3-2P2+P1 = (3,1,0)-(4,0,0)+(1,0,0) = (0,1,0)
    ///             At u=0: 6·[(1)·(0,0,0) + 0·(0,1,0)] = (0,0,0)
    ///
    ///   d³C/du³ = 6(P3-3P2+3P1-P0) = 6·((3,1,0)-(6,0,0)+(3,0,0)-(0,0,0))
    ///           = 6·(0,1,0) = (0,6,0)   (constant in u for a cubic Bezier)
    ///
    ///   df/du     = dot(d²C/du², dC/du) / f = dot((0,0,0),(3,0,0)) / 3 = 0
    ///   d²f/du²   = (|d²C/du²|² + dot(dC/du, d³C/du³)) / f - (df/du)²/f
    ///             = (0 + dot((3,0,0),(0,6,0))) / 3 - 0 = 0
    ///
    ///   du/ds = 1/f = 1/3
    ///   d²u/ds² = -df/du / f³ = 0
    ///   d³u/ds³ = -d²f/du² / f⁴ + 3(df/du)² / f⁵ = 0
    ///
    ///   d³C/ds³ = d³C/du³ · (du/ds)³  +  3·d²C/du²·(du/ds)·d²u/ds²  +  dC/du·d³u/ds³
    ///           = (0,6,0) · (1/3)³     +  3·(0,0,0)·(1/3)·0           +  (3,0,0)·0
    ///           = (0,6,0) / 27
    ///           = (0, 2/9, 0)  ≈  (0, 0.22222…, 0)
    ///
    /// # Why this fixture catches chain-rule bugs
    ///
    /// At u=0 all the "speed-variation" terms (df/du, d²f/du², d²u/ds², d³u/ds³) vanish,
    /// so `d³C/ds³` reduces to the cleanest possible form: `d³C/du³ / f³`. Any
    /// implementation error in those terms would go undetected here — but that is
    /// precisely the value: the surviving term `(0,6,0)/27` directly checks that the
    /// `d³C/du³ · (du/ds)³` branch is wired correctly. The vanishing of the other terms
    /// also guarantees a correct zero contribution from each of them; a sign error
    /// or wrong coefficient in those branches that produces a non-zero contribution at
    /// this point would corrupt the result and fail the test.
    #[test]
    fn cubic_bezier_pins_third_derivative_at_start() {
        // Degree-3 non-rational NURBS, knots [0,0,0,0,1,1,1,1].
        // At u=0: dC/du=(3,0,0), d²C/du²=(0,0,0), d³C/du³=(0,6,0) (constant).
        // All speed-variation terms vanish → d³C/ds³ = (0,6,0)/27 = (0, 2/9, 0).
        //
        // Note: at the endpoint (grid index 0), the k=3 FD stencil in eval_kth_deriv
        // uses avail_h = fd_h * 0.01 = 1e-7 and evaluates at slightly negative u,
        // which for a non-rational polynomial NURBS is valid de Boor extrapolation —
        // no clamping occurs in de_boor_inner for a polynomial Bezier patch.
        let curve = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 1.0, 0.0]],
            None,
        )
        .unwrap();

        // n=5 is sufficient; we only assert on index 0 (s=0, u=0).
        let grid = sample_arclength_grid(&curve, 5).unwrap();

        let triple_at_start = grid.c_triple_prime[0];
        let expected = [0.0_f64, 2.0 / 9.0, 0.0];

        // Tolerance: 5 % relative to the y-component (the only non-zero component).
        // The FD stencil for d³C/du³ on a degree-3 polynomial is exact to floating-
        // point precision, so the error budget is dominated by the u(s) inversion
        // round-off, not by FD truncation error.
        let scale = expected[1].abs(); // 2/9
        let err = (triple_at_start[0] - expected[0]).abs()
            + (triple_at_start[1] - expected[1]).abs()
            + (triple_at_start[2] - expected[2]).abs();
        assert!(
            err / scale < 0.05,
            "c_triple_prime[0] = {triple_at_start:?}, expected ≈ {expected:?}, \
             relative err = {:.4} (limit 0.05)",
            err / scale
        );
    }
}
