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

    /// Convert this monomial-basis polynomial to Bernstein control points on
    /// [u_start, u_end]. Length = degree + 1.
    /// Formula: B_k = Σ_{i=0..k} C(k,i) / C(d,i) * c_i_norm, where c_i_norm = c_i * h^i, h = u_end - u_start.
    /// (Per Farin §5.7.)
    pub fn to_bernstein(&self) -> Vec<T> {
        let d = self.degree();
        let h = self.u_end - self.u_start;
        // Normalize monomial coefficients to the [0, 1] domain.
        let mut h_pow = T::ONE;
        let normalized: Vec<T> = self
            .coeffs
            .iter()
            .map(|c| {
                let v = *c * h_pow;
                h_pow = h_pow * h;
                v
            })
            .collect();

        // Convert normalized monomial to Bernstein.
        let mut bernstein = vec![T::ZERO; d + 1];
        for k in 0..=d {
            let mut acc = T::ZERO;
            for i in 0..=k {
                let num = T::from_f64(binomial(k, i) as f64);
                let den = T::from_f64(binomial(d, i) as f64);
                acc = acc + (num / den) * normalized[i];
            }
            bernstein[k] = acc;
        }
        bernstein
    }

    /// Build a Bézier piece from Bernstein control points on [u_start, u_end].
    /// Inverse of `to_bernstein`. Length of `bernstein` = degree + 1.
    /// Formula: c_k = C(d,k) * Σ_{i=0..k} (-1)^{k-i} * C(k,i) * B_i / h^k.
    pub fn from_bernstein(bernstein: &[T], u_start: T, u_end: T) -> Self {
        let d = bernstein.len() - 1;
        let h = u_end - u_start;

        let mut h_pow = T::ONE;
        let mut coeffs = vec![T::ZERO; d + 1];
        for k in 0..=d {
            let mut acc = T::ZERO;
            for i in 0..=k {
                let sign = if (k - i) % 2 == 0 { T::ONE } else { -T::ONE };
                let c_d_k = T::from_f64(binomial(d, k) as f64);
                let c_k_i = T::from_f64(binomial(k, i) as f64);
                acc = acc + sign * c_d_k * c_k_i * bernstein[i];
            }
            coeffs[k] = acc / h_pow;
            h_pow = h_pow * h;
        }
        Self { u_start, u_end, coeffs }
    }
}

/// Binomial coefficient C(n, k). Integer-valued; safe for k, n ≤ 30 or so.
/// `pub(crate)` so `algebra.rs` can reuse it (DRY — defined here, used in convolve too).
pub(crate) fn binomial(n: usize, k: usize) -> u64 {
    if k > n {
        return 0;
    }
    let k = k.min(n - k);
    let mut result: u64 = 1;
    for i in 0..k {
        result = result * (n - i) as u64 / (i + 1) as u64;
    }
    result
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

    #[test]
    fn bernstein_round_trip_preserves_polynomial() {
        // Quadratic in monomial form: p(u) = 1 + 2u + 3u^2 on [0, 1].
        let monom = BezierPiece::<f64> {
            u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0, 3.0],
        };
        let bernstein = monom.to_bernstein();
        let back = BezierPiece::from_bernstein(&bernstein, 0.0, 1.0);

        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let exp = monom.evaluate(u);
            let got = back.evaluate(u);
            assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
        }
    }

    #[test]
    fn from_bernstein_to_monomial_for_known_case() {
        // Bernstein control points for line from 0 to 1 on [0, 1]: B_0=0, B_1=1.
        let p = BezierPiece::from_bernstein(&[0.0_f64, 1.0], 0.0, 1.0);
        // Equivalent monomial: p(u) = u, so coeffs = [0, 1].
        assert!((p.coeffs[0] - 0.0).abs() < 1e-12);
        assert!((p.coeffs[1] - 1.0).abs() < 1e-12);
    }
}
