//! Algebraic operations on NURBS. Host-only.
//! See spec §algebra module.

use crate::bezier::binomial;
use crate::{AlgebraError, Float};

/// Multiply control points by a scalar. Weights, knots, degree unchanged.
#[cfg(feature = "host")]
pub fn scalar_multiply<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    scalar: T,
) -> crate::ScalarNurbs<T> {
    let new_cps: Vec<T> = curve.control_points().iter().map(|c| *c * scalar).collect();
    let weights = curve.weights().map(<[T]>::to_vec);
    crate::ScalarNurbs::try_new(curve.degree(), curve.knots().to_vec(), new_cps, weights)
        .expect("scalar_multiply preserves invariants")
}

/// Add two scalar NURBS pointwise. v1 requires identical degree and identical
/// knot vectors; mismatched cases return `KnotMismatch` and the caller is
/// expected to align via knot insertion (follow-up implementation).
///
/// Weights: v1 supports unweighted-only. Weighted addition is non-trivial
/// (requires homogeneous lift) and is deferred to a follow-up spec.
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
    if a.weights().is_some() || b.weights().is_some() {
        return Err(AlgebraError::NotImplemented(
            "weighted add — homogeneous lift required",
        ));
    }
    let new_cps: Vec<T> = a
        .control_points()
        .iter()
        .zip(b.control_points().iter())
        .map(|(x, y)| *x + *y)
        .collect();
    crate::ScalarNurbs::try_new(a.degree(), a.knots().to_vec(), new_cps, None)
        .map_err(|_| AlgebraError::KnotMismatch)
}

/// Polynomial kernel for convolution. Pieces are contiguous and ordered.
/// Each piece is a polynomial in the Pascal-shifted monomial basis.
#[cfg(feature = "host")]
#[derive(Debug, Clone)]
pub struct PiecewisePolynomialKernel<T: Float> {
    pub pieces: Vec<crate::bezier::BezierPiece<T>>,
}

#[cfg(feature = "host")]
impl<T: Float> PiecewisePolynomialKernel<T> {
    /// Build a single-piece kernel from monomial coefficients
    /// `coeffs[k] * (u - u_start)^k` on the interval `support`.
    pub fn single_poly(coeffs: Vec<T>, support: (T, T)) -> Self {
        let piece = crate::bezier::BezierPiece {
            u_start: support.0,
            u_end: support.1,
            coeffs,
        };
        Self { pieces: vec![piece] }
    }

    /// Total support of the kernel: from first piece's `u_start` to last piece's `u_end`.
    pub fn support(&self) -> (T, T) {
        (self.pieces.first().unwrap().u_start, self.pieces.last().unwrap().u_end)
    }
}

/// Multiply two scalar NURBS pointwise: `c(u) = a(u) * b(u)`.
/// Result degree = `degree(a) + degree(b)`.
///
/// Polynomial inputs only in v1; rational inputs return `RationalNotSupported`.
#[cfg(feature = "host")]
pub fn multiply<T: Float>(
    a: &crate::ScalarNurbs<T>,
    b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if a.weights().is_some() || b.weights().is_some() {
        return Err(AlgebraError::RationalNotSupported {
            operation: "multiply",
            workaround: "use polynomial_refit (Layer 3 utility) before calling",
        });
    }
    let a_pieces = crate::bezier::extract_bezier_pieces(a);
    let b_pieces = crate::bezier::extract_bezier_pieces(b);

    // Refine to common breakpoint set.
    let breakpoints = union_breakpoints(&a_pieces, &b_pieces);
    let a_refined = refine_pieces_to_breakpoints(&a_pieces, &breakpoints);
    let b_refined = refine_pieces_to_breakpoints(&b_pieces, &breakpoints);
    debug_assert_eq!(a_refined.len(), b_refined.len());

    // Per-piece product.
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
    knot_remove_redundant(&mut result, T::from_f64(1e-12));
    Ok(result)
}

/// Compute the union of distinct breakpoints from two piecewise representations.
#[cfg(feature = "host")]
fn union_breakpoints<T: Float>(
    a: &[crate::bezier::BezierPiece<T>],
    b: &[crate::bezier::BezierPiece<T>],
) -> Vec<T> {
    let mut breaks: Vec<T> = Vec::new();
    let push_unique = |u: T, breaks: &mut Vec<T>| {
        if !breaks.iter().any(|x| *x == u) {
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
    breaks.sort_by(|x, y| x.partial_cmp(y).unwrap());
    breaks
}

/// Refine a list of contiguous Bézier pieces so that the piece boundaries
/// coincide with the given (sorted) breakpoints.
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
        interior.sort_by(|x, y| x.partial_cmp(y).unwrap());
        for u in interior {
            let (left, right) = crate::bezier::split_piece_at(&current, u);
            result.push(left);
            current = right;
        }
        result.push(current);
    }
    result
}

/// Convolve a polynomial NURBS with a piecewise polynomial kernel:
/// `y(u) = ∫ x(s) w(u - s) ds`.
///
/// Output domain = Minkowski sum of input and kernel supports. Caller
/// (Layer 3) handles cross-segment stitching for trajectories.
///
/// Polynomial inputs only in v1.
#[cfg(feature = "host")]
pub fn convolve<T: Float>(
    curve: &crate::ScalarNurbs<T>,
    kernel: &PiecewisePolynomialKernel<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    if curve.weights().is_some() {
        return Err(AlgebraError::RationalNotSupported {
            operation: "convolve",
            workaround: "use polynomial_refit (Layer 3 utility) before calling",
        });
    }
    let _ = kernel;
    todo!("convolve: piecewise integration implementation")
}

/// Polynomial coefficient convolution: out[k] = Σ_{i+j=k} a[i] * b[j].
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

/// Integrate `∫ x(s) w(u - s) ds` over the (s, u) region where x's piece i and
/// w's piece j are simultaneously active, for u in `[α, β]`. Returns the
/// contribution as a `BezierPiece` on `[α, β]` with degree `d_x + d_w + 1`.
///
/// Algorithm sketch (per spec §6.4):
/// 1. Re-express `w(u-s)` in s-basis with u-dependent coefficients (binomial expansion).
/// 2. Multiply by `x(s)`; result is a polynomial in s with u-dependent coefficients.
/// 3. Integrate `s^k → s^(k+1)/(k+1)`, evaluate at `s_hi(u)` and `s_lo(u)`.
/// 4. Both `s_lo` and `s_hi` are linear in u, so output is polynomial in u.
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

    // Integration limits as polynomials in u (degree 1, in absolute u, NOT shifted).
    // s_lo(u) = max(x.u_start, u - w.u_end)
    // s_hi(u) = min(x.u_end,   u - w.u_start)
    //
    // For u in [α, β] by construction of out_breaks, the active branch of max/min
    // is constant; we can determine it from the value at u = (α + β) / 2.
    let u_mid = (alpha + beta) * T::from_f64(0.5);
    let lo_branch_curve = u_mid - w.u_end > x.u_start; // true → s_lo(u) = u - w.u_end
    let hi_branch_curve = u_mid - w.u_start < x.u_end; // true → s_hi(u) = u - w.u_start

    // s_lo(u) and s_hi(u) as (constant, linear-in-u-coeff) tuples.
    let (s_lo_c, s_lo_u): (T, T) = if lo_branch_curve {
        (-w.u_end, T::ONE)
    } else {
        (x.u_start, T::ZERO)
    };
    let (s_hi_c, s_hi_u): (T, T) = if hi_branch_curve {
        (-w.u_start, T::ONE)
    } else {
        (x.u_end, T::ZERO)
    };

    // The integrand is x(s) * w(u - s).
    // Step A: Convert x.coeffs to absolute-s monomial basis.
    let x_abs = pascal_shift_to_absolute(&x.coeffs, x.u_start);

    // Step B: Convert w.coeffs to absolute-(u-s) monomial basis (in z = u-s).
    let w_abs_z = pascal_shift_to_absolute(&w.coeffs, w.u_start);
    // Then expand each z^j as (u - s)^j via binomial, giving polynomial in u and s.
    // w_abs_z[j] * (u - s)^j = w_abs_z[j] * Σ_l C(j, l) * u^(j-l) * (-s)^l
    //                        = Σ_l w_abs_z[j] * C(j, l) * (-1)^l * u^(j-l) * s^l

    // Build a 2D coefficient table: integrand[m][n] = coefficient of u^m * s^n.
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
                integrand[m][n] = integrand[m][n] + coef * x_abs[i];
            }
        }
    }

    // Step C: Integrate s^n → s^(n+1) / (n+1), evaluate at s_hi(u) - s_lo(u).
    let mut y_abs = vec![T::ZERO; out_degree + 1];
    for m in 0..=max_m {
        for n in 0..=max_n {
            if integrand[m][n] == T::ZERO { continue; }
            let inv = integrand[m][n] / T::from_f64((n + 1) as f64);
            let hi_pow = power_of_linear(s_hi_c, s_hi_u, n + 1);
            let lo_pow = power_of_linear(s_lo_c, s_lo_u, n + 1);
            for k in 0..hi_pow.len() {
                let target = k + m;
                if target <= out_degree {
                    y_abs[target] = y_abs[target] + inv * (hi_pow[k] - lo_pow[k]);
                }
            }
        }
    }

    // Convert from absolute-u monomial to Pascal-shifted-at-α basis.
    let y_shifted = absolute_to_pascal_shift(&y_abs, alpha);
    crate::bezier::BezierPiece {
        u_start: alpha,
        u_end: beta,
        coeffs: y_shifted,
    }
}

/// Expand `(c + a*u)^p` as a polynomial in u (length `p+1`, ascending power).
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

/// Convert Pascal-shifted-at-`shift` coefficients to absolute monomial.
/// `p(u) = Σ c_k * (u - shift)^k → Σ c'_n * u^n`
#[cfg(feature = "host")]
fn pascal_shift_to_absolute<T: Float>(shifted: &[T], shift: T) -> Vec<T> {
    let d = shifted.len() - 1;
    let mut out = vec![T::ZERO; d + 1];
    for k in 0..=d {
        // (u - shift)^k = (-shift + u)^k
        let exp = power_of_linear(-shift, T::ONE, k);
        for n in 0..exp.len() {
            out[n] = out[n] + shifted[k] * exp[n];
        }
    }
    out
}

/// Inverse: convert absolute monomial to Pascal-shifted-at-`shift`.
/// `Σ c_n * u^n → Σ c'_k * (u - shift)^k` where
/// `u^n = Σ_k C(n, k) * shift^(n-k) * (u - shift)^k`.
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

/// Iterate over interior knots and apply `remove_knot` with the given tolerance,
/// dropping knots whose removal preserves the curve within `tol`. Used by
/// `multiply` and `convolve` to expose natural smoothness of the result.
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
        if !removed_any { break; }
    }
}

#[cfg(all(test, feature = "host"))]
#[allow(clippy::float_cmp)] // tests assert exact stored coords / round-trip values, not arithmetic results
mod tests {
    use super::*;
    use crate::eval::eval;

    #[test]
    fn integrate_product_constant_input_constant_kernel_yields_linear_result() {
        // x(s) = 2 on [0, 1], w(t) = 3 on [-0.5, 0.5].
        // y(u) = ∫ x(s) w(u - s) ds, integration range = intersection of
        // [u - 0.5, u + 0.5] with [0, 1].
        // For u ∈ [0.5, 0.5] (single point), y = 2*3*1 = 6.
        // Generally y(u) = 6 * (length of overlap window).

        let x = crate::bezier::BezierPiece::<f64> {
            u_start: 0.0, u_end: 1.0, coeffs: vec![2.0],  // constant 2
        };
        let w = crate::bezier::BezierPiece::<f64> {
            u_start: -0.5, u_end: 0.5, coeffs: vec![3.0],  // constant 3
        };

        // Integrate over output sub-interval [0.5, 1.0] where the kernel window
        // shrinks linearly (from full overlap to half overlap at u=1.0).
        let contribution = integrate_product_piece(&x, &w, 0.5, 1.0);

        // Expected: y(u) = 6 * (1.0 - (u - 0.5)) for u ∈ [0.5, 1.0]
        //                = 6 * (1.5 - u)
        //                = 9 - 6u
        // In Pascal-shifted basis at α = 0.5: y(u) = 9 - 6u = 9 - 6*(0.5 + (u - 0.5))
        //                                          = 6 - 6 * (u - 0.5)
        // So coeffs at u_start = 0.5 should be [6.0, -6.0].
        assert!((contribution.coeffs[0] - 6.0).abs() < 1e-10);
        assert!((contribution.coeffs[1] - (-6.0)).abs() < 1e-10);
    }

    #[test]
    fn convolve_rejects_rational_input() {
        let curve = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0_f64, 1.0], Some(vec![1.0, 1.0]),
        ).unwrap();
        let kernel = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.1, 0.1));
        let result = convolve(&curve, &kernel);
        assert!(matches!(
            result,
            Err(AlgebraError::RationalNotSupported { operation: "convolve", .. })
        ));
    }

    #[test]
    fn single_poly_kernel_constructs_one_piece() {
        let k = PiecewisePolynomialKernel::single_poly(vec![1.0, 0.5_f64], (-1.0, 1.0));
        assert_eq!(k.pieces.len(), 1);
        assert_eq!(k.pieces[0].u_start, -1.0);
        assert_eq!(k.pieces[0].u_end, 1.0);
        assert_eq!(k.pieces[0].coeffs, vec![1.0, 0.5]);
    }

    #[test]
    fn kernel_support_returns_endpoints() {
        let k = PiecewisePolynomialKernel::single_poly(vec![1.0_f64], (-0.5, 0.5));
        assert_eq!(k.support(), (-0.5, 0.5));
    }

    #[test]
    fn knot_remove_redundant_simplifies_overproduct() {
        let a = crate::ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let b = a.clone();
        let mut c = multiply(&a, &b).unwrap();
        let initial_knot_count = c.knots().len();

        knot_remove_redundant(&mut c, 1e-10);

        // For a single-piece input, no interior knots to remove; result unchanged.
        assert_eq!(c.knots().len(), initial_knot_count);
        // Eval still correct.
        for u in [0.0, 0.5, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-10);
        }
    }

    #[test]
    fn multiply_curves_with_different_interior_knots() {
        // a has interior knot at 0.4, b has interior knot at 0.7.
        let a = crate::ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.4, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0],
            None,
        ).unwrap();
        let b = crate::ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.7, 1.0, 1.0, 1.0],
            vec![1.0, 2.0, 0.0, 1.0],
            None,
        ).unwrap();
        let c = multiply(&a, &b).unwrap();
        assert_eq!(c.degree(), 4);
        for u in [0.0, 0.2, 0.4, 0.5, 0.7, 0.9, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
        }
    }

    #[test]
    fn multiply_two_linear_curves_gives_quadratic() {
        // a(u) = u, b(u) = 2u + 1, expected c(u) = u(2u + 1) = 2u^2 + u.
        let a = crate::ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let b = crate::ScalarNurbs::<f64>::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![1.0, 3.0], None,
        ).unwrap();
        let c = multiply(&a, &b).unwrap();
        assert_eq!(c.degree(), 2);
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let exp = eval(&a.as_view(), u) * eval(&b.as_view(), u);
            let got = eval(&c.as_view(), u);
            assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
        }
    }

    #[test]
    fn scalar_multiply_doubles_evaluation() {
        let curve =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
        let doubled = scalar_multiply(&curve, 2.0_f64);
        assert!((eval(&doubled.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn scalar_multiply_preserves_weights() {
        let curve = crate::ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![1.0, 2.0],
            Some(vec![1.0, 1.0]),
        )
        .unwrap();
        let result = scalar_multiply(&curve, 3.0_f64);
        assert_eq!(result.weights().unwrap(), &[1.0, 1.0]);
        assert_eq!(result.control_points(), &[3.0, 6.0]);
    }

    #[test]
    fn add_two_compatible_curves() {
        let a =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
        let b =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![2.0, 3.0], None).unwrap();
        let sum = add(&a, &b).unwrap();
        // At u=0.5: 0.5 + 2.5 = 3.0
        assert!((eval(&sum.as_view(), 0.5_f64) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn add_rejects_mismatched_degree() {
        let a =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
        let b = crate::ScalarNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.5, 1.0],
            None,
        )
        .unwrap();
        let result = add(&a, &b);
        assert!(matches!(result, Err(crate::AlgebraError::KnotMismatch)));
    }

    #[test]
    fn multiply_rejects_rational_input() {
        let a = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], Some(vec![1.0, 1.0]),
        ).unwrap();
        let b = a.clone();
        let result = multiply(&a, &b);
        assert!(matches!(
            result,
            Err(crate::AlgebraError::RationalNotSupported { operation: "multiply", .. })
        ));
    }

}
