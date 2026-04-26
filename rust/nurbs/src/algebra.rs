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

/// Multiply two scalar NURBS. Result degree = degree(a) + degree(b).
///
/// Algorithm: deferred to a follow-up spec. See spec §algebra module —
/// well-trodden (Piegl & Tiller ch. 5) but verbose with non-uniform knots
/// and weights.
#[cfg(feature = "host")]
pub fn multiply<T: Float>(
    _a: &crate::ScalarNurbs<T>,
    _b: &crate::ScalarNurbs<T>,
) -> Result<crate::ScalarNurbs<T>, AlgebraError> {
    Err(AlgebraError::NotImplemented(
        "multiply — see Piegl & Tiller ch. 5; lands when needed by Layer 3 pre-bake",
    ))
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

#[cfg(all(test, feature = "host"))]
mod tests {
    use super::*;
    use crate::eval::eval;

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
    fn multiply_returns_not_implemented_error() {
        let a =
            crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap();
        let b = a.clone();
        let result = multiply(&a, &b);
        assert!(matches!(
            result,
            Err(crate::AlgebraError::NotImplemented(_))
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
