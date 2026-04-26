//! Algebraic operations on NURBS. Host-only.
//! See spec §algebra module.

use crate::Float;

/// Multiply control points by a scalar. Weights, knots, degree unchanged.
#[cfg(feature = "host")]
pub fn scalar_multiply<T: Float>(curve: &crate::ScalarNurbs<T>, scalar: T) -> crate::ScalarNurbs<T> {
    let new_cps: Vec<T> = curve.control_points().iter().map(|c| *c * scalar).collect();
    let weights = curve.weights().map(|w| w.to_vec());
    crate::ScalarNurbs::try_new(
        curve.degree(), curve.knots().to_vec(), new_cps, weights,
    ).expect("scalar_multiply preserves invariants")
}

#[cfg(all(test, feature = "host"))]
mod tests {
    use super::*;
    use crate::eval::eval;

    #[test]
    fn scalar_multiply_doubles_evaluation() {
        let curve = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None,
        ).unwrap();
        let doubled = scalar_multiply(&curve, 2.0_f64);
        assert!((eval(&doubled.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn scalar_multiply_preserves_weights() {
        let curve = crate::ScalarNurbs::try_new(
            1, vec![0.0, 0.0, 1.0, 1.0], vec![1.0, 2.0], Some(vec![1.0, 1.0]),
        ).unwrap();
        let result = scalar_multiply(&curve, 3.0_f64);
        assert_eq!(result.weights().unwrap(), &[1.0, 1.0]);
        assert_eq!(result.control_points(), &[3.0, 6.0]);
    }
}
