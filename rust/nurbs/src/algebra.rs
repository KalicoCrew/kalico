//! Algebraic operations on NURBS. Host-only.
//! See spec §algebra module.

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

/// Polynomial kernel for convolution. Coefficients are dense, low-to-high.
#[cfg(feature = "host")]
#[derive(Debug, Clone)]
pub struct PolynomialKernel<T: Float> {
    pub coefficients: Vec<T>,
    pub support: (T, T),
}

#[cfg(feature = "host")]
impl<T: Float> PolynomialKernel<T> {
    pub fn degree(&self) -> u8 {
        // Highest non-trivial coefficient; for v1 stub, just length - 1.
        (self.coefficients.len().saturating_sub(1)) as u8
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

/// Convolve a NURBS with a polynomial kernel. Result degree = degree(curve) + `kernel.degree()`.
///
/// Algorithm: deferred to a follow-up spec. Research-flavored (derived from
/// B-spline basis-function math). Lands when smooth shapers come online at
/// CLAUDE.md build step 8.
#[cfg(feature = "host")]
pub fn convolve_with_polynomial_kernel<T: Float>(
    _curve: &crate::ScalarNurbs<T>,
    _kernel: &PolynomialKernel<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    Err(AlgebraError::NotImplemented(
        "convolve_with_polynomial_kernel — research-flavored; lands at build step 8",
    ))
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
mod tests {
    use super::*;
    use crate::eval::eval;

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

    #[test]
    fn convolve_returns_not_implemented_error() {
        let a =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
        let kernel = PolynomialKernel {
            coefficients: vec![1.0, 0.0],
            support: (0.0, 1.0),
        };
        let result = convolve_with_polynomial_kernel(&a, &kernel);
        assert!(matches!(
            result,
            Err(crate::AlgebraError::NotImplemented(_))
        ));
    }
}
