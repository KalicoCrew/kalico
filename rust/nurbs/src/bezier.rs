//! Bézier piece in Pascal-shifted monomial basis. Host-only.
//! See `docs/superpowers/specs/2026-04-26-nurbs-algebra-design.md` §5.

use crate::Float;

/// One Bézier piece as a polynomial in the *Pascal-shifted monomial basis*:
/// p(u) = Σ_{k=0..d} coeffs[k] * (u - u_start)^k
#[derive(Debug, Clone, PartialEq)]
pub struct BezierPiece<T: Float> {
    pub u_start: T,
    pub u_end: T,
    pub coeffs: Vec<T>, // length = degree + 1
}

impl<T: Float> BezierPiece<T> {
    /// Polynomial degree (= coeffs.len() - 1).
    pub fn degree(&self) -> usize {
        self.coeffs.len().saturating_sub(1)
    }

    /// Evaluate p(u) by Horner's method on the Pascal-shifted basis.
    pub fn evaluate(&self, u: T) -> T {
        let dx = u - self.u_start;
        let mut acc = T::ZERO;
        for c in self.coeffs.iter().rev() {
            acc = acc * dx + *c;
        }
        acc
    }

    /// Zero polynomial of the given degree on [u_start, u_end].
    /// Used as the accumulator inside `convolve`.
    pub fn zero(u_start: T, u_end: T, degree: usize) -> Self {
        Self {
            u_start,
            u_end,
            coeffs: vec![T::ZERO; degree + 1],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn evaluate_constant_polynomial_is_constant() {
        let p = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![3.5] };
        assert_eq!(p.evaluate(0.0), 3.5);
        assert_eq!(p.evaluate(0.5), 3.5);
        assert_eq!(p.evaluate(1.0), 3.5);
    }

    #[test]
    fn evaluate_linear_polynomial() {
        // p(u) = 1 + 2 * (u - 0)
        let p = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0] };
        assert_eq!(p.evaluate(0.0), 1.0);
        assert_eq!(p.evaluate(0.5), 2.0);
        assert_eq!(p.evaluate(1.0), 3.0);
    }

    #[test]
    fn evaluate_uses_shifted_basis() {
        // p(u) = 1 + 2 * (u - 5), so p(5) = 1, p(6) = 3.
        let p = BezierPiece::<f64> { u_start: 5.0, u_end: 7.0, coeffs: vec![1.0, 2.0] };
        assert_eq!(p.evaluate(5.0), 1.0);
        assert_eq!(p.evaluate(6.0), 3.0);
        assert_eq!(p.evaluate(7.0), 5.0);
    }

    #[test]
    fn zero_creates_zero_polynomial_of_given_degree() {
        let p = BezierPiece::<f64>::zero(0.0, 1.0, 3);
        assert_eq!(p.coeffs, vec![0.0, 0.0, 0.0, 0.0]);
        assert_eq!(p.degree(), 3);
        assert_eq!(p.evaluate(0.5), 0.0);
    }
}
