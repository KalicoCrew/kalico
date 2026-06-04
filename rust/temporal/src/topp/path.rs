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

    // ---- Step 2: Build derivative NURBSes via degree-lowering ------------------
    //
    // Compose derivative NURBSes once via Piegl & Tiller A3.3 degree-lowering
    // (`vector_derivative`). This is *exact* to floating-point precision: a
    // polynomial of degree p has constant p-th derivative, identically zero
    // (p+1)-th and higher.
    //
    // Degree-too-low guard: a polynomial of degree p has identically zero
    // (p+1)-th and higher derivatives. We materialize derivative NURBSes only
    // up to `min(3, degree())`; lookups of higher orders return [0,0,0]
    // without panicking. Required for G5.1 (degree-2) inputs, which would
    // otherwise hit `vector_derivative`'s `assert!(p>=1)`.
    let curve_degree = usize::from(curve.degree());

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

        // u-parameterized derivatives via degree-lowered NURBSes.
        // Degree-too-low for k → [0,0,0] (mathematically correct).
        let eval_or_zero = |dn: &Option<VectorNurbs<f64, 3>>, u: f64| -> [f64; 3] {
            match dn {
                Some(c) => vector_eval(&c.as_view(), u),
                None => [0.0, 0.0, 0.0],
            }
        };
        let dc_du = eval_or_zero(&d1, u_i);
        let d2c_du2 = eval_or_zero(&d2, u_i);
        let d3c_du3 = eval_or_zero(&d3, u_i);

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
mod tests;
