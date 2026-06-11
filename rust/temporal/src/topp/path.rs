use nurbs::{
    MIN_PARAMETRIC_SPEED, VectorNurbs,
    arc_length::{build_arc_length_table_vector, param_from_arc_length},
    eval::{vector_derivative, vector_eval},
};

const INTER_INTERVAL_THETA_SAMPLES: &[f64] = &[0.25, 0.5, 0.75];

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
    /// Interior κ samples for each interval `[i, i+1]`, length N−1.
    /// Each entry is a vec of `(θ, κ)` pairs with θ ∈ (0,1) being the
    /// fractional position within the interval (`s = s_i + θ·h`).
    pub inter_kappa: Vec<Vec<(f64, f64)>>,
}

#[derive(Debug, thiserror::Error)]
pub enum PathSampleError {
    #[error("grid size N must be at least 2, got {0}")]
    GridTooSmall(usize),
    #[error("arc-length table construction failed: {0}")]
    ArcLengthTable(String),
}

pub fn sample_arclength_grid(
    curve: &VectorNurbs<f64, 3>,
    n: usize,
) -> Result<ArclengthGrid, PathSampleError> {
    if n < 2 {
        return Err(PathSampleError::GridTooSmall(n));
    }

    let arc_table = build_arc_length_table_vector(curve, 1e-6_f64, 1024)
        .map_err(|e| PathSampleError::ArcLengthTable(e.to_string()))?;

    let total_length = arc_table.s_max();
    let table_ref = arc_table.as_view();

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

    let mut s_vec = Vec::with_capacity(n);
    let mut u_vec = Vec::with_capacity(n);
    let mut c_vec = Vec::with_capacity(n);
    let mut c_prime_vec = Vec::with_capacity(n);
    let mut c_double_prime_vec = Vec::with_capacity(n);
    let mut c_triple_prime_vec = Vec::with_capacity(n);
    let mut kappa_vec = Vec::with_capacity(n);
    let mut inter_kappa_vec: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n.saturating_sub(1));

    let curve_view = curve.as_view();

    let floor = MIN_PARAMETRIC_SPEED;

    let kappa_at_u = |u: f64| -> f64 {
        let eval_or_zero = |dn: &Option<VectorNurbs<f64, 3>>, u: f64| -> [f64; 3] {
            match dn {
                Some(c) => vector_eval(&c.as_view(), u),
                None => [0.0, 0.0, 0.0],
            }
        };
        let dc_du = eval_or_zero(&d1, u);
        let d2c_du2 = eval_or_zero(&d2, u);

        let f_sq = dot3(dc_du, dc_du);
        let f = f_sq.sqrt().max(floor);
        let du_ds = 1.0 / f;
        let df_du = dot3(d2c_du2, dc_du) / f;
        let d2u_ds2 = -df_du / (f * f * f);
        let du_ds_sq = du_ds * du_ds;
        let c_prime = scale3(dc_du, du_ds);
        let c_double_prime = add3(scale3(d2c_du2, du_ds_sq), scale3(dc_du, d2u_ds2));
        let cross = cross3(c_prime, c_double_prime);
        dot3(cross, cross).sqrt()
    };

    for i in 0..n {
        let s_i = (i as f64) / ((n - 1) as f64) * total_length;
        let u_i = param_from_arc_length(&table_ref, s_i);

        let c_i = vector_eval(&curve_view, u_i);

        let eval_or_zero = |dn: &Option<VectorNurbs<f64, 3>>, u: f64| -> [f64; 3] {
            match dn {
                Some(c) => vector_eval(&c.as_view(), u),
                None => [0.0, 0.0, 0.0],
            }
        };
        let dc_du = eval_or_zero(&d1, u_i);
        let d2c_du2 = eval_or_zero(&d2, u_i);
        let d3c_du3 = eval_or_zero(&d3, u_i);

        let f_sq = dot3(dc_du, dc_du);
        let f = f_sq.sqrt().max(floor);

        let df_du = dot3(d2c_du2, dc_du) / f;

        let d2f_du2 = (dot3(d2c_du2, d2c_du2) + dot3(dc_du, d3c_du3)) / f - (df_du * df_du) / f;

        let du_ds = 1.0 / f;
        let d2u_ds2 = -df_du / (f * f * f);
        let f4 = f * f * f * f;
        let f5 = f4 * f;
        let d3u_ds3 = -(d2f_du2) / f4 + 3.0 * df_du * df_du / f5;

        let c_prime_i = scale3(dc_du, du_ds);

        let du_ds_sq = du_ds * du_ds;
        let c_double_prime_i = add3(scale3(d2c_du2, du_ds_sq), scale3(dc_du, d2u_ds2));

        let du_ds_cu = du_ds_sq * du_ds;
        let c_triple_prime_i = add3(
            add3(
                scale3(d3c_du3, du_ds_cu),
                scale3(d2c_du2, 3.0 * du_ds * d2u_ds2),
            ),
            scale3(dc_du, d3u_ds3),
        );

        let cross = cross3(c_prime_i, c_double_prime_i);
        let kappa_i = dot3(cross, cross).sqrt();

        s_vec.push(s_i);
        u_vec.push(u_i);
        c_vec.push(c_i);
        c_prime_vec.push(c_prime_i);
        c_double_prime_vec.push(c_double_prime_i);
        c_triple_prime_vec.push(c_triple_prime_i);
        kappa_vec.push(kappa_i);

        if i + 1 < n {
            let s_next = ((i + 1) as f64) / ((n - 1) as f64) * total_length;
            let h = s_next - s_i;
            let samples: Vec<(f64, f64)> = INTER_INTERVAL_THETA_SAMPLES
                .iter()
                .map(|&theta| {
                    let s_mid = s_i + theta * h;
                    let u_mid = param_from_arc_length(&table_ref, s_mid);
                    (theta, kappa_at_u(u_mid))
                })
                .collect();
            inter_kappa_vec.push(samples);
        }
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
        inter_kappa: inter_kappa_vec,
    })
}

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
