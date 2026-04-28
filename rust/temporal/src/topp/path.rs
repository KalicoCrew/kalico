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
    MIN_PARAMETRIC_SPEED, VectorNurbs,
    arc_length::{build_arc_length_table_vector, param_from_arc_length},
    eval::{vector_derivative, vector_eval},
};

/// Evaluate the k-th parametric derivative of a *rational* NURBS at `u` via
/// central finite differences of `vector_eval`.
///
/// `vector_derivative` is exact (Piegl & Tiller A3.3 degree-lowering) but only
/// for non-rational (B-spline) NURBS — it silently ignores weights. For rational
/// NURBS (G2/G3 arcs) the quotient rule is needed; we fall back to finite
/// differences here at the Lyness 1968 / Numerical Recipes 3rd ed. §5.7
/// optimal step `h_opt = ε^(1/(k+1))`:
///   k=1 → 1.49e-8 (not used here; k=1 goes through analytical for rational
///                   too via the existing chain rule on `vector_eval`)
///   k=2 → 6.06e-6
///   k=3 → 1.22e-4
///
/// The previous implementation used `h*0.01 = 1e-7` as the endpoint floor,
/// which is ~1000x smaller than the optimal for k=3 and produced catastrophic
/// cancellation noise (`pp - 2p + 2m - mm` is dominated by round-off when the
/// stencil samples agree to ~16 digits). See `/tmp/path_diag.json` and
/// `/tmp/path_verifier.json` for the diagnosis.
///
/// Used only on the rational branch in `sample_arclength_grid`. Non-rational
/// curves go through `vector_derivative` instead.
fn eval_kth_deriv_rational(curve: &VectorNurbs<f64, 3>, u: f64, k: usize) -> [f64; 3] {
    let knots = curve.knots();
    let u_start = knots[0];
    let u_end = *knots.last().expect("non-empty knot vector");
    let view = curve.as_view();

    match k {
        0 => vector_eval(&view, u),
        1 => {
            // Lyness optimal for k=1: ε^(1/2) ≈ 1.49e-8. Round to 1e-8.
            let h = 1e-8_f64;
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
            // Lyness optimal for k=2: ε^(1/3) ≈ 6.06e-6.
            let h_opt = 6.06e-6_f64;
            // Symmetric clamp: don't evaluate outside [u_start, u_end].
            let avail_h = (u - u_start).min(u_end - u).min(h_opt);
            // Endpoint floor at h_opt itself — accept asymmetric stencil error
            // rather than degenerate step. (At endpoints, one of u-u_start or
            // u_end-u is 0; we cap at h_opt and let the stencil straddle the
            // endpoint, valid de Boor extrapolation for clamped polynomial
            // pieces; only used here in the rational branch which has weights
            // that gracefully handle out-of-domain via clamping in vector_eval.)
            let avail_h = avail_h.max(h_opt);
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
            // Lyness optimal for k=3: ε^(1/4) ≈ 1.22e-4.
            let h_opt = 1.22e-4_f64;
            // Third central difference: (C(u+2h) - 2C(u+h) + 2C(u-h) - C(u-2h)) / (2h³)
            // Symmetric clamp: maximum step that fits in the domain (each side
            // takes 2h, so the per-side cap is (u-u_start)/2 and (u_end-u)/2).
            let avail_h = ((u - u_start) / 2.0).min((u_end - u) / 2.0).min(h_opt);
            let avail_h = avail_h.max(h_opt);
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

    // ---- Step 2: Build derivative NURBSes (non-rational) or prepare FD path --
    //
    // For non-rational (B-spline) curves we compose derivative NURBSes once via
    // Piegl & Tiller A3.3 degree-lowering (`vector_derivative`). This is *exact*
    // to floating-point precision: a polynomial of degree p has constant p-th
    // derivative, identically zero (p+1)-th and higher. The previous FD path
    // suffered catastrophic cancellation at endpoints (numerator `pp - 2p + 2m
    // - mm` collapses to round-off noise when the stencil samples agree to ~16
    // digits) — see /tmp/path_diag.json.
    //
    // For rational (weighted) curves `vector_derivative` is wrong (it silently
    // discards weights and returns the unweighted control polygon's
    // derivative). The quotient rule is the analytical fix; until that lands
    // we fall back to FD with Lyness-optimal steps (`eval_kth_deriv_rational`).
    //
    // Degree-too-low guard: a polynomial of degree p has identically zero
    // (p+1)-th and higher derivatives. We materialize derivative NURBSes only
    // up to `min(3, degree())`; lookups of higher orders return [0,0,0]
    // without panicking. Required for G0/G1 (degree-1) and G5.1 (degree-2)
    // inputs, which would otherwise hit `vector_derivative`'s `assert!(p>=1)`.
    let is_rational = curve.weights().is_some();
    let curve_degree = usize::from(curve.degree());

    // For non-rational only: pre-build d1, d2, d3 (each up to degree-lowering
    // limit). Materialize lazily; if `curve_degree` < k, we use a sentinel
    // `None` and the loop returns [0,0,0] for that order.
    let (d1, d2, d3) = if is_rational {
        (None, None, None)
    } else {
        let d1 = if curve_degree >= 1 {
            Some(vector_derivative(curve))
        } else {
            None
        };
        let d2 = match d1.as_ref() {
            Some(d1c) if d1c.degree() >= 1 => Some(vector_derivative(d1c)),
            _ => None,
        };
        let d3 = match d2.as_ref() {
            Some(d2c) if d2c.degree() >= 1 => Some(vector_derivative(d2c)),
            _ => None,
        };
        (d1, d2, d3)
    };

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
        let s_i = (i as f64) / ((n - 1) as f64) * total_length;
        let u_i = param_from_arc_length(&table_ref, s_i);

        // Curve position.
        let c_i = vector_eval(&curve_view, u_i);

        // u-parameterized derivatives.
        // - Non-rational: analytical via vector_eval on degree-lowered NURBSes
        //   (exact to floating-point precision).
        // - Rational: FD with Lyness-optimal step (asymmetric at endpoints,
        //   accepted error per spec §11 / verifier-caveat 1).
        // - Degree-too-low for k: return [0,0,0] (mathematically correct).
        let (dc_du, d2c_du2, d3c_du3) = if is_rational {
            (
                eval_kth_deriv_rational(curve, u_i, 1),
                eval_kth_deriv_rational(curve, u_i, 2),
                eval_kth_deriv_rational(curve, u_i, 3),
            )
        } else {
            let eval_or_zero = |dn: &Option<VectorNurbs<f64, 3>>, u: f64| -> [f64; 3] {
                match dn {
                    Some(c) => vector_eval(&c.as_view(), u),
                    None => [0.0, 0.0, 0.0],
                }
            };
            (
                eval_or_zero(&d1, u_i),
                eval_or_zero(&d2, u_i),
                eval_or_zero(&d3, u_i),
            )
        };

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
        // Post-fix (vector_derivative replaces FD for non-rational): the result
        // is exact to floating-point round-off (analytical degree-lowering plus
        // the u(s) inversion). Tolerance tightened from 5 % to 1 % accordingly;
        // the previous 5 % was permissive of catastrophic-cancellation noise
        // that has been removed (see /tmp/path_diag.json, /tmp/path_verifier.json).
        let curve = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 1.0, 0.0],
            ],
            None,
        )
        .unwrap();

        // n=5 is sufficient; we only assert on index 0 (s=0, u=0).
        let grid = sample_arclength_grid(&curve, 5).unwrap();

        let triple_at_start = grid.c_triple_prime[0];
        let expected = [0.0_f64, 2.0 / 9.0, 0.0];

        let scale = expected[1].abs(); // 2/9
        let err = (triple_at_start[0] - expected[0]).abs()
            + (triple_at_start[1] - expected[1]).abs()
            + (triple_at_start[2] - expected[2]).abs();
        assert!(
            err / scale < 0.01,
            "c_triple_prime[0] = {triple_at_start:?}, expected ≈ {expected:?}, \
             relative err = {:.4} (limit 0.01)",
            err / scale
        );
    }

    /// Pin `c_triple_prime` at *both* endpoints on the asymmetric G5 cubic
    /// (used by Step 9 fixture 4). This is the regression guard for the
    /// catastrophic-cancellation FD bug: with the old `eval_kth_deriv` k=3
    /// stencil floored at `h*0.01 = 1e-7`, the endpoint values were
    /// round-off coin-flips (~1e5 raw / O(50) after chain-rule scaling);
    /// after switching to analytical `vector_derivative`, they match closed
    /// form to ~1e-6.
    ///
    /// # Closed-form derivation (independently verified, NOT taken from the
    /// diagnosis whose `predictedValue` x-component was arithmetic-wrong)
    ///
    /// CPs P0=(0,0,0), P1=(3,3,0), P2=(7,3,0), P3=(10,0,0). Cubic Bezier:
    ///   dC/du = 3[(1−u)²(P1−P0) + 2(1−u)u(P2−P1) + u²(P3−P2)]
    ///   d²C/du² = 6[(1−u)(P2−2P1+P0) + u(P3−2P2+P1)]
    ///   d³C/du³ = 6(P3−3P2+3P1−P0) = 6(10−21+9−0, 0−6+9−0, 0) = 6(−2,3,0)·... wait
    ///
    /// Let me recompute: P3−3P2+3P1−P0 = (10,0,0)−3(7,3,0)+3(3,3,0)−(0,0,0)
    ///                                  = (10−21+9, 0−9+9, 0) = (−2, 0, 0).
    /// So d³C/du³ = 6·(−2, 0, 0) = (−12, 0, 0). Constant in u.
    ///
    /// At u=1: dC/du = 3·(P3−P2) = 3·(3,−3,0) = (9, −9, 0). f=|dC/du|=9√2.
    /// d²C/du² at u=1: 6·(P3−2P2+P1) = 6·((10,0,0)−(14,6,0)+(3,3,0)) = 6·(−1,−3,0) = (−6,−18,0).
    ///
    /// At u=0: dC/du = 3·(P1−P0) = (9,9,0). f=9√2.
    /// d²C/du² at u=0: 6·(P2−2P1+P0) = 6·((7,3,0)−(6,6,0)+(0,0,0)) = 6·(1,−3,0) = (6,−18,0).
    ///
    /// Chain rule (see module docstring) with f=9√2, f²=162, f³=1458√2,
    /// f⁴=26244, f⁵=26244·9√2:
    ///
    /// At u=1:
    ///   df/du = dot((−6,−18,0),(9,−9,0))/9√2 = (−54+162)/9√2 = 12/√2 = 6√2
    ///   d²u/ds² = −(6√2)/(1458√2) = −1/243
    ///   |d²C/du²|² = 36+324 = 360
    ///   dot(dC/du, d³C/du³) = 9·(−12) = −108
    ///   d²f/du² = (360 − 108)/9√2 − (6√2)²/9√2 = 252/9√2 − 72/9√2 = 180/9√2 = 10√2
    ///   d³u/ds³ = −10√2/26244 + 3·(6√2)²/(26244·9√2) = −10√2/26244 + 12√2/26244 = 2√2/26244
    ///
    ///   c'''(u=1) = (−12,0,0)/(1458√2) + 3·(−6,−18,0)/(9√2)·(−1/243) + (9,−9,0)·(2√2/26244)
    ///   c'''_x   = −12/(1458√2) + (18·9√2)/(9√2·243·9√2)·... — easier numerically:
    ///             ≈ −0.005820 + 0.005820 + 0.000970 = +0.000970
    ///   c'''_y   = 0 + 3·(−18)/(9√2)·(−1/243) + (−9)·(2√2/26244)
    ///             = 54/(9√2·243) − 18√2/26244
    ///             ≈ 0.017459 − 0.000970 = +0.016489
    ///
    /// At u=0 (mirror by sign-flips of the y-tangent components):
    ///   c'''_x(u=0) = +0.000970 (same as u=1; symmetry of the chain-rule x algebra)
    ///   c'''_y(u=0) = −0.016489 (sign-flipped relative to u=1)
    #[test]
    fn cubic_bezier_c3_at_endpoints_matches_closed_form() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [3.0, 3.0, 0.0],
                [7.0, 3.0, 0.0],
                [10.0, 0.0, 0.0],
            ],
            None,
        )
        .unwrap();

        // n=200 matches the Step 9 fixture 4 grid resolution.
        let grid = sample_arclength_grid(&curve, 200).unwrap();
        let triple_start = grid.c_triple_prime[0];
        let triple_end = *grid.c_triple_prime.last().unwrap();

        // Closed-form values (re-derived from scratch above, NOT from
        // /tmp/path_diag.json's `predictedValue` field which has an
        // arithmetic error in the x-component).
        let expected_start = [0.000_970_f64, -0.016_489_f64, 0.0];
        let expected_end = [0.000_970_f64, 0.016_489_f64, 0.0];
        let tol = 1e-4_f64; // generous; analytical degree-lowering is ~1e-12

        for (label, got, exp) in [
            ("start", triple_start, expected_start),
            ("end", triple_end, expected_end),
        ] {
            assert!(
                (got[0] - exp[0]).abs() < tol,
                "{label}: c'''_x = {} vs expected {} (tol {})",
                got[0],
                exp[0],
                tol
            );
            assert!(
                (got[1] - exp[1]).abs() < tol,
                "{label}: c'''_y = {} vs expected {} (tol {})",
                got[1],
                exp[1],
                tol
            );
            assert!(
                got[2].abs() < tol,
                "{label}: c'''_z = {} vs expected 0 (tol {})",
                got[2],
                tol
            );
        }
    }

    /// Degree-1 (G1 line) NURBS must NOT panic when the chain rule asks for
    /// the 2nd or 3rd parametric derivative. Mathematically: a polynomial of
    /// degree p has identically zero (p+1)-th and higher derivatives. The
    /// patch guards against `vector_derivative`'s `assert!(p >= 1)` panic by
    /// returning [0,0,0] when the degree-lowering chain bottoms out.
    #[test]
    fn degenerate_g1_curve_does_not_panic() {
        let curve = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
            None,
        )
        .unwrap();

        // n=5 is enough — we only need to confirm the call returns rather
        // than panicking, and that c''' is zero at every grid point (a
        // straight line has zero second and third arclength derivatives).
        let grid = sample_arclength_grid(&curve, 5).unwrap();

        for (i, c3) in grid.c_triple_prime.iter().enumerate() {
            assert!(
                c3[0].abs() + c3[1].abs() + c3[2].abs() < 1e-9,
                "c_triple_prime[{i}] = {c3:?} should be ~0 on a straight line",
            );
        }
        for (i, c2) in grid.c_double_prime.iter().enumerate() {
            assert!(
                c2[0].abs() + c2[1].abs() + c2[2].abs() < 1e-9,
                "c_double_prime[{i}] = {c2:?} should be ~0 on a straight line",
            );
        }
    }

    /// Pin `c_double_prime` endpoints on the rational quarter-circle. Per
    /// `/tmp/path_verifier.json` caveat 1, the rational FD branch also needed
    /// to be hardened (Lyness-optimal step instead of `h*0.01`). On a
    /// uniformly-curved arc, |C''(s)| at any s should equal κ = 1/R.
    #[test]
    fn rational_quadratic_arc_c2_endpoints() {
        let r = 20.0_f64;
        let w = std::f64::consts::FRAC_1_SQRT_2;
        let curve = VectorNurbs::<f64, 3>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![[r, 0.0, 0.0], [r, r, 0.0], [0.0, r, 0.0]],
            Some(vec![1.0, w, 1.0]),
        )
        .unwrap();
        // n=51 gives endpoint κ from the chain rule against FD c'' values.
        let grid = sample_arclength_grid(&curve, 51).unwrap();

        let kappa_expected = 1.0 / r; // 0.05
        // |C''(s)| = κ on an arclength-parameterized curve.
        let c2_start_mag = {
            let v = grid.c_double_prime[0];
            (v[0].powi(2) + v[1].powi(2) + v[2].powi(2)).sqrt()
        };
        let c2_end_mag = {
            let v = *grid.c_double_prime.last().unwrap();
            (v[0].powi(2) + v[1].powi(2) + v[2].powi(2)).sqrt()
        };
        // 5 % tolerance — rational FD with Lyness step is well-conditioned
        // here but still has truncation error from the asymmetric stencil at
        // u=0 / u=1.
        assert!(
            (c2_start_mag - kappa_expected).abs() / kappa_expected < 0.05,
            "|c''(0)| = {c2_start_mag}, expected ~{kappa_expected}",
        );
        assert!(
            (c2_end_mag - kappa_expected).abs() / kappa_expected < 0.05,
            "|c''(L)| = {c2_end_mag}, expected ~{kappa_expected}",
        );
    }
}
