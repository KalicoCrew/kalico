use crate::bezier::binomial;
use crate::{AlgebraError, Float};

#[cfg(feature = "host")]
pub fn scalar_multiply<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    scalar: T,
) -> crate::ScalarNurbs<T> {
    let new_cps: Vec<T> = curve.control_points().iter().map(|c| *c * scalar).collect();
    crate::ScalarNurbs::try_new(curve.degree(), curve.knots().to_vec(), new_cps)
        .expect("scalar_multiply preserves invariants")
}

#[cfg(feature = "host")]
pub fn add<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.degree() != b.degree() {
        return Err(AlgebraError::KnotMismatch);
    }
    if a.knots() != b.knots() {
        return Err(AlgebraError::KnotMismatch);
    }
    let new_cps: Vec<T> = a
        .control_points()
        .iter()
        .zip(b.control_points().iter())
        .map(|(x, y)| *x + *y)
        .collect();
    crate::ScalarNurbs::try_new(a.degree(), a.knots().to_vec(), new_cps)
        .map_err(|_| AlgebraError::KnotMismatch)
}

/// ```rust
/// # use nurbs::algebra::add_with_knot_union;
/// # use nurbs::ScalarNurbs;
/// // X: two Bézier pieces. Y: one piece.
/// let x = ScalarNurbs::try_new(
///     1,
///     vec![0.0_f64, 0.0, 0.5, 1.0, 1.0],
///     vec![0.0, 5.0, 10.0],
/// ).unwrap();
/// let y = ScalarNurbs::try_new(
///     1,
///     vec![0.0_f64, 0.0, 1.0, 1.0],
///     vec![20.0, 20.0],
/// ).unwrap();
/// let sum = add_with_knot_union(&x, &y).unwrap();
/// let v0 = nurbs::eval::eval(&sum.as_view(), 0.0_f64);
/// let v1 = nurbs::eval::eval(&sum.as_view(), 1.0_f64);
/// assert!((v0 - 20.0).abs() < 1e-12);
/// assert!((v1 - 30.0).abs() < 1e-12);
/// ```
#[cfg(feature = "host")]
pub fn add_with_knot_union<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.degree() != b.degree() {
        return Err(AlgebraError::KnotMismatch);
    }

    if a.knots() == b.knots() {
        return add(a, b);
    }

    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    if a_pieces.is_empty() || b_pieces.is_empty() {
        return Err(AlgebraError::KnotMismatch);
    }

    let a_start = a_pieces[0].u_start;
    let a_end = a_pieces[a_pieces.len() - 1].u_end;
    let b_start = b_pieces[0].u_start;
    let b_end = b_pieces[b_pieces.len() - 1].u_end;
    let domain_tol = T::from_f64(1e-12);
    if (a_start - b_start).abs() > domain_tol || (a_end - b_end).abs() > domain_tol {
        return Err(AlgebraError::SupportMismatch);
    }

    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);

    assert_eq!(
        a_refined.len(),
        b_refined.len(),
        "add_with_knot_union: refine produced mismatched piece counts \
         (a_refined={}, b_refined={}); this is an internal invariant violation",
        a_refined.len(),
        b_refined.len(),
    );

    let sum_pieces: Vec<crate::bezier::BezierPiece<T>> = a_refined
        .iter()
        .zip(b_refined.iter())
        .map(|(ap, bp)| {
            assert_eq!(
                ap.coeffs.len(),
                bp.coeffs.len(),
                "add_with_knot_union: CP count mismatch after union refine \
                 (ap.coeffs={}, bp.coeffs={})",
                ap.coeffs.len(),
                bp.coeffs.len(),
            );
            let coeffs: Vec<T> = ap
                .coeffs
                .iter()
                .zip(bp.coeffs.iter())
                .map(|(ac, bc)| *ac + *bc)
                .collect();
            crate::bezier::BezierPiece {
                u_start: ap.u_start,
                u_end: ap.u_end,
                coeffs,
            }
        })
        .collect();

    Ok(crate::bezier::bezier_pieces_to_nurbs(&sum_pieces))
}

#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub enum FitError {
    ToleranceNotReached { achieved_mm: f64, at_degree: u8 },
    DegenerateInput { reason: &'static str },
}

#[cfg(feature = "host")]
pub fn fit_x_to_arc_length_piece<const D: usize>(
    geometry: &crate::VectorNurbs<f64, D>,
    table: &crate::ArcLengthTableRef<'_, f64>,
    s_lo: f64,
    s_hi: f64,
    target_degree: u8,
    max_degree: u8,
    tolerance_mm: f64,
) -> Result<[crate::bezier::BezierPiece<f64>; D], FitError>
where
    [(); D]:,
{
    const RANGE_EPS: f64 = 1e-9;

    if !s_lo.is_finite() || !s_hi.is_finite() {
        return Err(FitError::DegenerateInput {
            reason: "s_lo or s_hi is non-finite",
        });
    }
    if s_hi <= s_lo {
        return Err(FitError::DegenerateInput {
            reason: "s_hi <= s_lo",
        });
    }
    if target_degree > max_degree {
        return Err(FitError::DegenerateInput {
            reason: "target_degree > max_degree",
        });
    }

    let s_max = table.s_max();
    if s_lo < -RANGE_EPS || s_hi > s_max + RANGE_EPS {
        return Err(FitError::DegenerateInput {
            reason: "s_lo/s_hi out of arc-length table range",
        });
    }

    let mut d = target_degree;
    loop {
        let d_usize = d as usize;
        let n_nodes = d_usize + 1;

        let mid = 0.5 * (s_lo + s_hi);
        let half = 0.5 * (s_hi - s_lo);
        let mut s_nodes: Vec<f64> = Vec::with_capacity(n_nodes);
        if d == 0 {
            s_nodes.push(mid);
        } else {
            for i in 0..=d_usize {
                let angle = (i as f64) * std::f64::consts::PI / f64::from(d);
                s_nodes.push(mid + half * angle.cos());
            }
        }

        let mut samples: Vec<[f64; D]> = Vec::with_capacity(n_nodes);
        for &s in &s_nodes {
            let s_clamped = s.clamp(0.0, table.s_max());
            let u = crate::arc_length::param_from_arc_length(table, s_clamped);
            let x = crate::eval::vector_eval(geometry, u);
            samples.push(x);
        }

        let coeffs_per_axis = lagrange_interpolation_pascal_shifted::<D>(&s_nodes, &samples, s_lo);

        let n_check = 4 * n_nodes;
        let mut max_err = 0.0_f64;
        for i in 0..=n_check {
            let t = (i as f64) / (n_check as f64);
            let s = s_lo + (s_hi - s_lo) * t;
            let s_clamped = s.clamp(0.0, table.s_max());
            let u = crate::arc_length::param_from_arc_length(table, s_clamped);
            let truth = crate::eval::vector_eval(geometry, u);
            for axis in 0..D {
                let p_val = horner_pascal_shifted(&coeffs_per_axis[axis], s, s_lo);
                let err = (truth[axis] - p_val).abs();
                if err > max_err {
                    max_err = err;
                }
            }
        }

        if max_err <= tolerance_mm {
            let pieces_vec: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
                .map(|axis| crate::bezier::BezierPiece {
                    u_start: s_lo,
                    u_end: s_hi,
                    coeffs: coeffs_per_axis[axis].clone(),
                })
                .collect();
            return pieces_vec
                .try_into()
                .map_err(|_: Vec<_>| FitError::DegenerateInput {
                    reason: "fit_x_to_arc_length_piece: array length mismatch (unreachable)",
                });
        }

        if d >= max_degree {
            return Err(FitError::ToleranceNotReached {
                achieved_mm: max_err,
                at_degree: d,
            });
        }
        d += 1;
    }
}

#[cfg(feature = "host")]
pub fn fit_hermite_c1<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    tolerance_mm: f64,
    target_degree: u8,
) -> Result<[Vec<crate::bezier::BezierPiece<f64>>; D], FitError> {
    if pieces.is_empty() {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1: empty input",
        });
    }
    if !tolerance_mm.is_finite() || tolerance_mm <= 0.0 {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1: tolerance must be finite and positive",
        });
    }
    if target_degree < 3 {
        return Err(FitError::DegenerateInput {
            reason: "fit_hermite_c1: target_degree must be >= 3 (need 4 Hermite constraints)",
        });
    }

    for w in pieces.windows(2) {
        for axis in 0..D {
            if (w[0][axis].u_end - w[1][axis].u_start).abs() > 1e-12 {
                return Err(FitError::DegenerateInput {
                    reason: "fit_hermite_c1: non-contiguous input pieces",
                });
            }
        }
    }

    let mut result: [Vec<crate::bezier::BezierPiece<f64>>; D] = std::array::from_fn(|_| Vec::new());

    hermite_fit_recursive::<D>(
        pieces,
        0,
        pieces.len(),
        tolerance_mm,
        target_degree,
        &mut result,
    )?;

    Ok(result)
}

#[cfg(feature = "host")]
fn hermite_fit_recursive<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    tolerance_mm: f64,
    target_degree: u8,
    result: &mut [Vec<crate::bezier::BezierPiece<f64>>; D],
) -> Result<(), FitError> {
    debug_assert!(lo < hi);

    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;

    let mut candidate = hermite_fit_one_piece::<D>(pieces, lo, hi, target_degree);
    let max_residual = hermite_check_residual::<D>(pieces, lo, hi, &candidate, target_degree);

    if max_residual <= tolerance_mm {
        for axis in 0..D {
            candidate[axis].u_start = pieces[lo][axis].u_start;
            candidate[axis].u_end = pieces[hi - 1][axis].u_end;
            result[axis].push(candidate[axis].clone());
        }
        return Ok(());
    }

    if hi - lo == 1 {
        return Err(FitError::ToleranceNotReached {
            achieved_mm: max_residual,
            at_degree: target_degree,
        });
    }

    let mid = lo + (hi - lo) / 2;
    let _ = (u_lo, u_hi);

    hermite_fit_recursive::<D>(pieces, lo, mid, tolerance_mm, target_degree, result)?;
    hermite_fit_recursive::<D>(pieces, mid, hi, tolerance_mm, target_degree, result)?;

    Ok(())
}

#[cfg(feature = "host")]
fn hermite_fit_one_piece<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    target_degree: u8,
) -> [crate::bezier::BezierPiece<f64>; D] {
    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;
    let h = u_hi - u_lo;
    let d = target_degree as usize;

    let constraints: Vec<(f64, f64, f64, f64)> = (0..D)
        .map(|axis| {
            let f_lo = pieces[lo][axis].evaluate(u_lo);
            let df_lo = pieces[lo][axis].differentiate().evaluate(u_lo);
            let f_hi = pieces[hi - 1][axis].evaluate(u_hi);
            let df_hi = pieces[hi - 1][axis].differentiate().evaluate(u_hi);
            (f_lo, df_lo, f_hi, df_hi)
        })
        .collect();

    if d <= 3 || h.abs() < 1e-300 {
        return std::array::from_fn(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 0.0)
        });
    }

    let n_check = 4 * (d + 1);
    let mut sample_u: Vec<f64> = Vec::with_capacity(n_check + 1);
    let mut sample_piece_idx: Vec<usize> = Vec::with_capacity(n_check + 1);
    for i in 0..=n_check {
        let t = i as f64 / n_check as f64;
        let u = u_lo + (u_hi - u_lo) * t;
        sample_u.push(u);
        sample_piece_idx.push(hermite_find_piece_at(pieces, lo, hi, u));
    }

    let cand_0: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
        .map(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 0.0)
        })
        .collect();
    let cand_1: Vec<crate::bezier::BezierPiece<f64>> = (0..D)
        .map(|axis| {
            let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
            hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, 1.0)
        })
        .collect();

    let mut a_vals: Vec<f64> = Vec::new();
    let mut b_vals: Vec<f64> = Vec::new();
    for (si, &u) in sample_u.iter().enumerate() {
        let pidx = sample_piece_idx[si];
        for axis in 0..D {
            let ref_val = pieces[pidx][axis].evaluate(u);
            let p0 = cand_0[axis].evaluate(u);
            let p1 = cand_1[axis].evaluate(u);
            a_vals.push(ref_val - p0);
            b_vals.push(p1 - p0);
        }
    }

    let optimal_c2 = minimax_1d(&a_vals, &b_vals);

    std::array::from_fn(|axis| {
        let (f_lo, df_lo, f_hi, df_hi) = constraints[axis];
        hermite_construct_poly(f_lo, df_lo, f_hi, df_hi, u_lo, h, d, optimal_c2)
    })
}

#[cfg(feature = "host")]
fn minimax_1d(a: &[f64], b: &[f64]) -> f64 {
    debug_assert_eq!(a.len(), b.len());

    let max_b = b.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
    if max_b < 1e-30 {
        return 0.0;
    }

    let eval_max_err = |x: f64| -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(&ai, &bi)| (ai - bi * x).abs())
            .fold(0.0_f64, f64::max)
    };

    let mut candidates: Vec<f64> = Vec::new();
    candidates.push(0.0);
    let n = a.len();
    for i in 0..n {
        if b[i].abs() > 1e-30 {
            candidates.push(a[i] / b[i]);
        }
    }
    for i in 0..n {
        for j in 0..n {
            let denom = b[i] + b[j];
            if denom.abs() > 1e-30 {
                candidates.push((a[i] + a[j]) / denom);
            }
            let denom2 = b[i] - b[j];
            if denom2.abs() > 1e-30 {
                candidates.push((a[i] - a[j]) / denom2);
            }
        }
    }

    let mut best_x = 0.0;
    let mut best_err = eval_max_err(0.0);
    for x in candidates {
        if !x.is_finite() {
            continue;
        }
        let err = eval_max_err(x);
        if err < best_err {
            best_err = err;
            best_x = x;
        }
    }

    best_x
}

#[cfg(feature = "host")]
#[allow(clippy::too_many_arguments, clippy::cast_possible_wrap)]
fn hermite_construct_poly(
    f_lo: f64,
    df_lo: f64,
    f_hi: f64,
    df_hi: f64,
    u_lo: f64,
    h: f64,
    d: usize,
    c2_val: f64,
) -> crate::bezier::BezierPiece<f64> {
    let mut coeffs = vec![0.0f64; d + 1];

    coeffs[0] = f_lo;
    coeffs[1] = df_lo;

    if d >= 4 {
        coeffs[2] = c2_val;
    }

    let mut pos_residual = f_hi - coeffs[0] - coeffs[1] * h;
    let mut vel_residual = df_hi - coeffs[1];

    let mut h_pow = h * h;
    let mut h_pow_deriv = h;
    for k in 2..d.saturating_sub(1) {
        pos_residual -= coeffs[k] * h_pow;
        vel_residual -= (k as f64) * coeffs[k] * h_pow_deriv;
        h_pow *= h;
        h_pow_deriv *= h;
    }

    let h_dm2 = h.powi(d as i32 - 2);
    let h_dm1 = h_dm2 * h;
    let h_d = h_dm1 * h;
    let det = h.powi(2 * d as i32 - 2);

    if det.abs() < 1e-300 {
        return crate::bezier::BezierPiece {
            u_start: u_lo,
            u_end: u_lo + h,
            coeffs,
        };
    }

    let d_f = d as f64;
    let dm1_f = (d - 1) as f64;
    let c_dm1 = (pos_residual * d_f * h_dm1 - h_d * vel_residual) / det;
    let c_d = (h_dm1 * vel_residual - dm1_f * h_dm2 * pos_residual) / det;

    coeffs[d - 1] = c_dm1;
    coeffs[d] = c_d;

    crate::bezier::BezierPiece {
        u_start: u_lo,
        u_end: u_lo + h,
        coeffs,
    }
}

#[cfg(feature = "host")]
fn hermite_check_residual<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    candidate: &[crate::bezier::BezierPiece<f64>; D],
    target_degree: u8,
) -> f64 {
    let n_check = 4 * (target_degree as usize + 1);
    let u_lo = pieces[lo][0].u_start;
    let u_hi = pieces[hi - 1][0].u_end;
    let mut max_err = 0.0_f64;

    for i in 0..=n_check {
        let t = i as f64 / n_check as f64;
        let u = u_lo + (u_hi - u_lo) * t;
        let piece_idx = hermite_find_piece_at(pieces, lo, hi, u);

        for axis in 0..D {
            let ref_val = pieces[piece_idx][axis].evaluate(u);
            let fit_val = candidate[axis].evaluate(u);
            let err = (ref_val - fit_val).abs();
            if err > max_err {
                max_err = err;
            }
        }
    }

    max_err
}

#[cfg(feature = "host")]
fn hermite_find_piece_at<const D: usize>(
    pieces: &[[crate::bezier::BezierPiece<f64>; D]],
    lo: usize,
    hi: usize,
    u: f64,
) -> usize {
    for i in lo..hi {
        if u <= pieces[i][0].u_end + 1e-12 {
            return i;
        }
    }
    hi - 1
}

#[cfg(feature = "host")]
fn lagrange_interpolation_pascal_shifted<const D: usize>(
    s_nodes: &[f64],
    samples: &[[f64; D]],
    s_origin: f64,
) -> Vec<Vec<f64>> {
    let n = s_nodes.len();
    debug_assert_eq!(samples.len(), n);

    let mut aug: Vec<Vec<f64>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(n + D);
        let dx = s_nodes[i] - s_origin;
        let mut pow = 1.0;
        for _ in 0..n {
            row.push(pow);
            pow *= dx;
        }
        for axis in 0..D {
            row.push(samples[i][axis]);
        }
        aug.push(row);
    }

    for k in 0..n {
        let mut pivot = k;
        let mut pivot_abs = aug[k][k].abs();
        for i in (k + 1)..n {
            let v = aug[i][k].abs();
            if v > pivot_abs {
                pivot = i;
                pivot_abs = v;
            }
        }
        if pivot != k {
            aug.swap(k, pivot);
        }
        let pivot_val = aug[k][k];
        if pivot_val.abs() < 1e-300 {
            continue;
        }
        for i in (k + 1)..n {
            let factor = aug[i][k] / pivot_val;
            if factor == 0.0 {
                continue;
            }
            for j in k..(n + D) {
                aug[i][j] -= factor * aug[k][j];
            }
        }
    }

    let mut out: Vec<Vec<f64>> = (0..D).map(|_| vec![0.0; n]).collect();
    for axis in 0..D {
        let rhs_col = n + axis;
        for k in (0..n).rev() {
            let mut sum = aug[k][rhs_col];
            for j in (k + 1)..n {
                sum -= aug[k][j] * out[axis][j];
            }
            let pivot_val = aug[k][k];
            if pivot_val.abs() < 1e-300 {
                out[axis][k] = 0.0;
            } else {
                out[axis][k] = sum / pivot_val;
            }
        }
    }
    out
}

#[cfg(feature = "host")]
fn horner_pascal_shifted(coeffs: &[f64], s: f64, s_origin: f64) -> f64 {
    let dx = s - s_origin;
    let mut acc = 0.0;
    for c in coeffs.iter().rev() {
        acc = acc * dx + *c;
    }
    acc
}

#[cfg(feature = "host")]
#[derive(Debug, Clone)]
pub struct PiecewisePolynomialKernel<T: Float> {
    pub pieces: Vec<crate::bezier::BezierPiece<T>>,
}

#[cfg(feature = "host")]
impl<T: Float> PiecewisePolynomialKernel<T> {
    pub fn single_poly(coeffs: Vec<T>, support: (T, T)) -> Self {
        let piece = crate::bezier::BezierPiece {
            u_start: support.0,
            u_end: support.1,
            coeffs,
        };
        Self {
            pieces: vec![piece],
        }
    }

    #[allow(clippy::needless_pass_by_value)]
    pub fn single_poly_from_absolute(coeffs: Vec<T>, support: (T, T)) -> Self {
        let shifted = absolute_to_pascal_shift(&coeffs, support.0);
        Self::single_poly(shifted, support)
    }

    pub fn support(&self) -> (T, T) {
        (
            self.pieces.first().unwrap().u_start,
            self.pieces.last().unwrap().u_end,
        )
    }

    pub fn from_pieces(pieces: Vec<crate::bezier::BezierPiece<T>>) -> Result<Self, AlgebraError> {
        if pieces.is_empty() {
            return Err(AlgebraError::SupportMismatch);
        }
        for w in pieces.windows(2) {
            if w[0].u_end != w[1].u_start {
                return Err(AlgebraError::SupportMismatch);
            }
        }
        Ok(Self { pieces })
    }
}

#[cfg(feature = "host")]
pub fn multiply<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    let a_mults = collect_interior_multiplicities(a);
    let b_mults = collect_interior_multiplicities(b);

    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);
    debug_assert_eq!(a_refined.len(), b_refined.len());

    let mut out_pieces = Vec::with_capacity(a_refined.len());
    for (a_p, b_p) in a_refined.iter().zip(b_refined.iter()) {
        let coeffs = poly_multiply(&a_p.coeffs, &b_p.coeffs);
        out_pieces.push(crate::bezier::BezierPiece {
            u_start: a_p.u_start,
            u_end: a_p.u_end,
            coeffs,
        });
    }

    let mut result = crate::bezier::bezier_pieces_to_nurbs(&out_pieces);

    let d_a = a.degree() as usize;
    let d_b = b.degree() as usize;
    let p = result.degree() as usize;
    let interior_breakpoints = collect_interior_breakpoints(&result);
    let targets: Vec<(T, usize)> = interior_breakpoints
        .into_iter()
        .map(|u| {
            let m_a = a_mults
                .iter()
                .find(|(uu, _)| *uu == u)
                .map_or(0, |(_, m)| *m);
            let m_b = b_mults
                .iter()
                .find(|(uu, _)| *uu == u)
                .map_or(0, |(_, m)| *m);
            let target = morken_multiplicity(d_a, m_a, d_b, m_b);
            debug_assert!(
                target <= p,
                "Mørken target {target} exceeds product degree {p}"
            );
            (u, target)
        })
        .collect();

    knot_remove_to_morken_targets(&mut result, &targets, T::from_f64(1e-12));
    Ok(result)
}

#[cfg(feature = "host")]
fn morken_multiplicity(d_a: usize, m_a: usize, d_b: usize, m_b: usize) -> usize {
    match (m_a > 0, m_b > 0) {
        (true, true) => (d_a + m_b).max(d_b + m_a),
        (false, true) => d_a + m_b,
        (true, false) => d_b + m_a,
        (false, false) => 0,
    }
}

#[cfg(feature = "host")]
fn collect_interior_multiplicities<T: Float>(curve: &crate::ScalarNurbs<T>) -> Vec<(T, usize)> {
    let p = curve.degree() as usize;
    let knots = curve.knots();
    if knots.len() <= 2 * (p + 1) {
        return Vec::new();
    }
    let interior_slice = &knots[p + 1..knots.len() - p - 1];
    let mut out: Vec<(T, usize)> = Vec::new();
    for &k in interior_slice {
        if let Some(entry) = out.iter_mut().find(|(u, _)| *u == k) {
            entry.1 += 1;
        } else {
            out.push((k, 1));
        }
    }
    out
}

#[cfg(feature = "host")]
fn collect_interior_breakpoints<T: Float>(curve: &crate::ScalarNurbs<T>) -> Vec<T> {
    collect_interior_multiplicities(curve)
        .into_iter()
        .map(|(u, _)| u)
        .collect()
}

#[cfg(feature = "host")]
fn knot_remove_to_morken_targets<T: Float>(
    curve: &mut crate::ScalarNurbs<T>,
    target_mults: &[(T, usize)],
    tol: T,
) {
    for &(u, target) in target_mults {
        let current = curve.knots().iter().filter(|k| **k == u).count();
        if current > target {
            let n_to_remove = current - target;
            let (new_curve, _actually_removed) =
                crate::knot::remove_knot(curve, u, n_to_remove, tol);
            *curve = new_curve;
        }
    }
}

#[cfg(feature = "host")]
fn union_breakpoints<T: Float>(
    a: &[crate::bezier::BezierPiece<T>],
    b: &[crate::bezier::BezierPiece<T>],
) -> Vec<T> {
    let mut breaks: Vec<T> = Vec::new();
    let push_unique = |u: T, breaks: &mut Vec<T>| {
        if !breaks.contains(&u) {
            breaks.push(u);
        }
    };
    for piece in a {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    for piece in b {
        push_unique(piece.u_start, &mut breaks);
        push_unique(piece.u_end, &mut breaks);
    }
    breaks.sort_by(|x, y| T::total_cmp(*x, *y));
    breaks
}

#[cfg(feature = "host")]
fn refine_pieces_to_breakpoints<T: Float>(
    pieces: &[crate::bezier::BezierPiece<T>],
    breakpoints: &[T],
) -> Vec<crate::bezier::BezierPiece<T>> {
    let mut result: Vec<crate::bezier::BezierPiece<T>> = Vec::new();
    for piece in pieces {
        let mut current = piece.clone();
        let mut interior: Vec<T> = breakpoints
            .iter()
            .filter(|&&b| b > current.u_start && b < current.u_end)
            .copied()
            .collect();
        interior.sort_by(|x, y| T::total_cmp(*x, *y));
        for u in interior {
            let (left, right) = crate::bezier::split_piece_at(&current, u);
            result.push(left);
            current = right;
        }
        result.push(current);
    }
    result
}

#[cfg(feature = "host")]
pub fn convolve<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    kernel: &PiecewisePolynomialKernel<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    let x_pieces = crate::bezier::extract_bezier_pieces(curve);
    let w_pieces = &kernel.pieces;

    let x_breaks: Vec<T> = {
        let mut v: Vec<T> = Vec::new();
        for p in &x_pieces {
            if !v.contains(&p.u_start) {
                v.push(p.u_start);
            }
        }
        v.push(x_pieces.last().unwrap().u_end);
        v
    };
    let w_breaks: Vec<T> = {
        let mut v: Vec<T> = Vec::new();
        for p in w_pieces {
            if !v.contains(&p.u_start) {
                v.push(p.u_start);
            }
        }
        v.push(w_pieces.last().unwrap().u_end);
        v
    };
    let mut out_breaks: Vec<T> = Vec::new();
    for xb in &x_breaks {
        for wb in &w_breaks {
            let s = *xb + *wb;
            if !out_breaks.contains(&s) {
                out_breaks.push(s);
            }
        }
    }
    out_breaks.sort_by(|a, b| T::total_cmp(*a, *b));

    let degree = x_pieces[0].degree() + w_pieces[0].degree() + 1;

    let mut out_pieces: Vec<crate::bezier::BezierPiece<T>> =
        Vec::with_capacity(out_breaks.len() - 1);
    for win in out_breaks.windows(2) {
        let alpha = win[0];
        let beta = win[1];
        let mut accum = crate::bezier::BezierPiece::<T>::zero(alpha, beta, degree);

        for x_p in &x_pieces {
            for w_p in w_pieces {
                let u_mid = (alpha + beta) * T::from_f64(0.5);
                let s_lo = (x_p.u_start).max(u_mid - w_p.u_end);
                let s_hi = (x_p.u_end).min(u_mid - w_p.u_start);
                if s_lo >= s_hi {
                    continue;
                }

                let contribution = integrate_product_piece(x_p, w_p, alpha, beta);
                accum = (&accum + &contribution).expect("same-support accumulation");
            }
        }
        out_pieces.push(accum);
    }

    let mut result = crate::bezier::bezier_pieces_to_nurbs(&out_pieces);
    knot_remove_redundant(&mut result, T::from_f64(1e-12));
    Ok(result)
}

#[cfg(feature = "host")]
pub fn compose_vector_piece<const D: usize>(
    outer: &[&crate::bezier::BezierPiece<f64>; D],
    inner: &crate::bezier::BezierPiece<f64>,
) -> Result<[crate::bezier::BezierPiece<f64>; D], AlgebraError> {
    const ENDPOINT_TOL: f64 = 1e-9;
    let inner_at_start = inner.evaluate(inner.u_start);
    let inner_at_end = inner.evaluate(inner.u_end);
    for outer_axis in outer {
        if (outer_axis.u_start - inner_at_start).abs() > ENDPOINT_TOL
            || (outer_axis.u_end - inner_at_end).abs() > ENDPOINT_TOL
        {
            return Err(AlgebraError::SupportMismatch);
        }
    }

    let pieces: Vec<crate::bezier::BezierPiece<f64>> = outer
        .iter()
        .map(|outer_axis| {
            let d_outer = outer_axis.degree();

            let mut shifted_inner = inner.coeffs.clone();
            if shifted_inner.is_empty() {
                shifted_inner.push(-outer_axis.u_start);
            } else {
                shifted_inner[0] -= outer_axis.u_start;
            }

            let mut powers: Vec<Vec<f64>> = Vec::with_capacity(d_outer + 1);
            powers.push(vec![1.0]);
            for i in 1..=d_outer {
                let next = poly_multiply(&powers[i - 1], &shifted_inner);
                powers.push(next);
            }

            let d_inner = inner.degree();
            let result_len = d_outer * d_inner + 1;
            let mut result_coeffs = vec![0.0_f64; result_len];
            for (i, c_outer) in outer_axis.coeffs.iter().enumerate() {
                let pow = &powers[i];
                for (k, p_k) in pow.iter().enumerate() {
                    result_coeffs[k] += *c_outer * *p_k;
                }
            }

            crate::bezier::BezierPiece {
                u_start: inner.u_start,
                u_end: inner.u_end,
                coeffs: result_coeffs,
            }
        })
        .collect();

    pieces.try_into().map_err(|_: Vec<_>| {
        AlgebraError::NotImplemented("compose_vector_piece: array length mismatch (unreachable)")
    })
}

#[cfg(feature = "host")]
fn poly_multiply<T: Float>(a: &[T], b: &[T]) -> Vec<T> {
    let mut out = vec![T::ZERO; a.len() + b.len() - 1];
    for (i, ai) in a.iter().enumerate() {
        for (j, bj) in b.iter().enumerate() {
            out[i + j] = out[i + j] + *ai * *bj;
        }
    }
    out
}

#[cfg(feature = "host")]
fn integrate_product_piece<T: Float>(
    x: &crate::bezier::BezierPiece<T>,
    w: &crate::bezier::BezierPiece<T>,
    alpha: T,
    beta: T,
) -> crate::bezier::BezierPiece<T> {
    let d_x = x.degree();
    let d_w = w.degree();
    let out_degree = d_x + d_w + 1;

    // s_lo(u) = max(x.u_start, u - w.u_end); s_hi(u) = min(x.u_end, u - w.u_start).
    // Active branch is constant on [α, β] by construction; determine from midpoint.
    let u_mid = (alpha + beta) * T::from_f64(0.5);
    let lo_branch_curve = u_mid - w.u_end > x.u_start; // true → s_lo(u) = u - w.u_end
    let hi_branch_curve = u_mid - w.u_start < x.u_end; // true → s_hi(u) = u - w.u_start

    // Work in shifted frame v = u − α, r = s − α to avoid catastrophic cancellation
    // when α is large. Result is already in (u − α)^k basis; no re-shift needed.
    let x_abs_r = pascal_shift_to_absolute(&x.coeffs, x.u_start - alpha);
    let w_abs_z = pascal_shift_to_absolute(&w.coeffs, w.u_start);

    let (r_lo_c, r_lo_v): (T, T) = if lo_branch_curve {
        (-w.u_end, T::ONE)
    } else {
        (x.u_start - alpha, T::ZERO)
    };
    let (r_hi_c, r_hi_v): (T, T) = if hi_branch_curve {
        (-w.u_start, T::ONE)
    } else {
        (x.u_end - alpha, T::ZERO)
    };

    let max_m = d_w;
    let max_n = d_x + d_w;
    let mut integrand = vec![vec![T::ZERO; max_n + 1]; max_m + 1];

    for j in 0..=d_w {
        for l in 0..=j {
            let m = j - l;
            let sign = if l % 2 == 0 { T::ONE } else { -T::ONE };
            let c_jl = T::from_f64(binomial(j, l) as f64);
            let coef = sign * c_jl * w_abs_z[j];
            for i in 0..=d_x {
                let n = l + i;
                integrand[m][n] = integrand[m][n] + coef * x_abs_r[i];
            }
        }
    }

    let mut y_v = vec![T::ZERO; out_degree + 1];
    for m in 0..=max_m {
        for n in 0..=max_n {
            if integrand[m][n] == T::ZERO {
                continue;
            }
            let inv = integrand[m][n] / T::from_f64((n + 1) as f64);
            let hi_pow = power_of_linear(r_hi_c, r_hi_v, n + 1);
            let lo_pow = power_of_linear(r_lo_c, r_lo_v, n + 1);
            for k in 0..hi_pow.len() {
                let target = k + m;
                if target <= out_degree {
                    y_v[target] = y_v[target] + inv * (hi_pow[k] - lo_pow[k]);
                }
            }
        }
    }

    crate::bezier::BezierPiece {
        u_start: alpha,
        u_end: beta,
        coeffs: y_v,
    }
}

#[cfg(feature = "host")]
fn power_of_linear<T: Float>(c: T, a: T, p: usize) -> Vec<T> {
    let mut out = vec![T::ZERO; p + 1];
    let mut c_pow = vec![T::ONE; p + 1];
    let mut a_pow = vec![T::ONE; p + 1];
    for k in 1..=p {
        c_pow[k] = c_pow[k - 1] * c;
        a_pow[k] = a_pow[k - 1] * a;
    }
    for k in 0..=p {
        let bin = T::from_f64(binomial(p, k) as f64);
        out[k] = bin * c_pow[p - k] * a_pow[k];
    }
    out
}

#[cfg(feature = "host")]
fn pascal_shift_to_absolute<T: Float>(shifted: &[T], shift: T) -> Vec<T> {
    let d = shifted.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    for k in 0..=d {
        let exp = power_of_linear(-shift, T::ONE, k);
        for n in 0..exp.len() {
            out[n] = out[n] + shifted[k] * exp[n];
        }
    }
    out
}

#[cfg(feature = "host")]
fn absolute_to_pascal_shift<T: Float>(absolute: &[T], shift: T) -> Vec<T> {
    let d = absolute.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    let mut shift_pow = vec![T::ONE; d + 1];
    for k in 1..=d {
        shift_pow[k] = shift_pow[k - 1] * shift;
    }
    for n in 0..=d {
        for k in 0..=n {
            let bin = T::from_f64(binomial(n, k) as f64);
            out[k] = out[k] + absolute[n] * bin * shift_pow[n - k];
        }
    }
    out
}

#[cfg(feature = "host")]
pub fn restrict_to_domain<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    t_lo: T,
    t_hi: T,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    use crate::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, split_piece_at};

    if t_lo >= t_hi {
        return Err(AlgebraError::SupportMismatch);
    }

    let pieces = extract_bezier_pieces(curve);
    let mut result = Vec::new();

    for piece in &pieces {
        if piece.u_end <= t_lo || piece.u_start >= t_hi {
            continue;
        }

        let mut p = piece.clone();

        if p.u_start < t_lo {
            let (_, right) = split_piece_at(&p, t_lo);
            p = right;
        }

        if p.u_end > t_hi {
            let (left, _) = split_piece_at(&p, t_hi);
            p = left;
        }

        result.push(p);
    }

    if result.is_empty() {
        return Err(AlgebraError::SupportMismatch);
    }

    Ok(bezier_pieces_to_nurbs(&result))
}

#[cfg(feature = "host")]
pub(crate) fn knot_remove_redundant<T: Float>(curve: &mut crate::ScalarNurbs<T>, tol: T) {
    let p = curve.degree() as usize;
    loop {
        let knots: Vec<T> = curve.knots().to_vec();
        let interior: Vec<T> = {
            let mut seen: Vec<T> = Vec::new();
            for &k in &knots[p + 1..knots.len() - p - 1] {
                if !seen.contains(&k) {
                    seen.push(k);
                }
            }
            seen
        };

        let mut removed_any = false;
        for u in interior {
            let (new_curve, count) = crate::knot::remove_knot(curve, u, 1, tol);
            if count > 0 {
                *curve = new_curve;
                removed_any = true;
            }
        }
        if !removed_any {
            break;
        }
    }
}

#[cfg(all(test, feature = "host"))]
#[allow(clippy::float_cmp)]
mod tests;
