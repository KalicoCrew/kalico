//! Bézier piece in Pascal-shifted monomial basis. Host-only.
//! See `docs/superpowers/specs/2026-04-26-nurbs-algebra-design.md` §5.

use crate::{AlgebraError, Float, ScalarNurbs};

/// One Bézier piece as a polynomial in the *Pascal-shifted monomial basis*:
/// `p(u) = Σ_{k=0..d} coeffs[k] * (u - u_start)^k`
#[derive(Debug, Clone, PartialEq)]
pub struct BezierPiece<T: Float> {
    pub u_start: T,
    pub u_end: T,
    pub coeffs: Vec<T>, // length = degree + 1
}

impl<T: Float> BezierPiece<T> {
    /// Polynomial degree (= `coeffs.len()` - 1).
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

    /// Zero polynomial of the given degree on `[u_start, u_end]`.
    /// Used as the accumulator inside `convolve`.
    pub fn zero(u_start: T, u_end: T, degree: usize) -> Self {
        Self {
            u_start,
            u_end,
            coeffs: vec![T::ZERO; degree + 1],
        }
    }

    /// Convert this monomial-basis polynomial to Bernstein control points on
    /// `[u_start, u_end]`. Length = degree + 1.
    /// Formula: `B_k = Σ_{i=0..k} C(k,i) / C(d,i) * c_i_norm`, where `c_i_norm = c_i * h^i`, `h = u_end - u_start`.
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

    /// Build a Bézier piece from Bernstein control points on `[u_start, u_end]`.
    /// Inverse of `to_bernstein`. Length of `bernstein` = degree + 1.
    /// Formula: `c_k = C(d,k) * Σ_{i=0..k} (-1)^{k-i} * C(k,i) * B_i / h^k`.
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

impl<T: Float> std::ops::Add<&BezierPiece<T>> for &BezierPiece<T> {
    type Output = Result<BezierPiece<T>, AlgebraError>;
    fn add(self, rhs: &BezierPiece<T>) -> Self::Output {
        if self.u_start != rhs.u_start || self.u_end != rhs.u_end {
            return Err(AlgebraError::SupportMismatch);
        }
        let max_len = self.coeffs.len().max(rhs.coeffs.len());
        let mut coeffs = vec![T::ZERO; max_len];
        for (i, c) in self.coeffs.iter().enumerate() {
            coeffs[i] = coeffs[i] + *c;
        }
        for (i, c) in rhs.coeffs.iter().enumerate() {
            coeffs[i] = coeffs[i] + *c;
        }
        Ok(BezierPiece {
            u_start: self.u_start,
            u_end: self.u_end,
            coeffs,
        })
    }
}

/// Decompose a polynomial NURBS into its constituent Bézier pieces in the
/// Pascal-shifted monomial basis. Internally raises every interior knot to
/// multiplicity = degree (Boehm), then converts each Bernstein piece to monomial.
pub fn extract_bezier_pieces<T: Float>(curve: &ScalarNurbs<T>) -> Vec<BezierPiece<T>> {
    assert!(
        curve.weights().is_none(),
        "extract_bezier_pieces: rational input not supported in v1"
    );

    let refined = crate::knot::refined_to_full_multiplicity(curve);
    let p = refined.degree() as usize;
    let knots = refined.knots();
    let cps = refined.control_points();

    // Identify unique breakpoints (excluding clamping at endpoints, only counted once).
    let mut breakpoints: Vec<T> = Vec::new();
    let mut last: Option<T> = None;
    for k in knots {
        if last.is_none_or(|l| *k != l) {
            breakpoints.push(*k);
            last = Some(*k);
        }
    }

    let mut pieces = Vec::with_capacity(breakpoints.len() - 1);
    let mut cp_idx = 0;
    for window in breakpoints.windows(2) {
        let u_start = window[0];
        let u_end = window[1];
        let bernstein: Vec<T> = cps[cp_idx..=(cp_idx + p)].to_vec();
        pieces.push(BezierPiece::from_bernstein(&bernstein, u_start, u_end));
        cp_idx += p; // Shared boundary CP between adjacent pieces.
    }

    pieces
}

/// Recompose contiguous Bézier pieces into a single NURBS. Inverse of
/// `extract_bezier_pieces` (modulo knot-multiplicity, which is `degree` at
/// each interior breakpoint = piecewise-Bézier representation).
///
/// Panics if pieces are non-contiguous or have inconsistent degrees.
pub fn bezier_pieces_to_nurbs<T: Float>(pieces: &[BezierPiece<T>]) -> ScalarNurbs<T> {
    assert!(!pieces.is_empty(), "bezier_pieces_to_nurbs: empty input");
    let p = pieces[0].degree();
    for w in pieces.windows(2) {
        assert!(w[0].u_end == w[1].u_start, "non-contiguous Bezier pieces");
        assert!(w[1].degree() == p, "inconsistent degrees");
    }

    // Build knot vector: u_start[0] repeated p+1 times, then each interior
    // boundary repeated p times, then u_end[last] repeated p+1 times.
    let mut knots = Vec::with_capacity((pieces.len() + 1) * p + 2);
    for _ in 0..=p {
        knots.push(pieces[0].u_start);
    }
    for piece in &pieces[..pieces.len() - 1] {
        for _ in 0..p {
            knots.push(piece.u_end);
        }
    }
    for _ in 0..=p {
        knots.push(pieces[pieces.len() - 1].u_end);
    }

    // Build CPs: each piece's Bernstein CPs, with shared boundaries.
    let mut cps: Vec<T> = Vec::with_capacity(pieces.len() * p + 1);
    for (i, piece) in pieces.iter().enumerate() {
        let bernstein = piece.to_bernstein();
        if i == 0 {
            cps.extend_from_slice(&bernstein);
        } else {
            // Skip first CP (shared boundary with previous piece's last).
            cps.extend_from_slice(&bernstein[1..]);
        }
    }

    ScalarNurbs::try_new(p as u8, knots, cps, None)
        .expect("bezier_pieces_to_nurbs: invariants should hold")
}

/// Split a Bézier piece at an interior point `u_split`, producing two pieces
/// covering `[u_start, u_split]` and `[u_split, u_end]` with the same polynomial
/// degree. The polynomial value is preserved on each side.
pub fn split_piece_at<T: Float>(
    piece: &BezierPiece<T>,
    u_split: T,
) -> (BezierPiece<T>, BezierPiece<T>) {
    assert!(
        u_split > piece.u_start && u_split < piece.u_end,
        "u_split must be strictly interior"
    );
    let d = piece.degree();

    // Left piece: same monomial coefficients (basis at u_start unchanged); just narrower support.
    let left = BezierPiece {
        u_start: piece.u_start,
        u_end: u_split,
        coeffs: piece.coeffs.clone(),
    };

    // Right piece: re-shift the basis from u_start to u_split.
    // p(u) = Σ c_k (u - u_start)^k. Substitute (u - u_start) = (u - u_split) + delta where delta = u_split - u_start.
    // Expand via binomial: (u - u_start)^k = Σ_{i=0..k} C(k,i) (u - u_split)^i delta^{k-i}.
    // So new_coeff[i] = Σ_{k=i..d} c_k * C(k,i) * delta^{k-i}.
    let delta = u_split - piece.u_start;
    let mut right_coeffs = vec![T::ZERO; d + 1];
    let mut delta_pow = vec![T::ONE; d + 1];
    for k in 1..=d {
        delta_pow[k] = delta_pow[k - 1] * delta;
    }

    for i in 0..=d {
        let mut acc = T::ZERO;
        for k in i..=d {
            let c_k_i = T::from_f64(binomial(k, i) as f64);
            acc = acc + piece.coeffs[k] * c_k_i * delta_pow[k - i];
        }
        right_coeffs[i] = acc;
    }

    let right = BezierPiece {
        u_start: u_split,
        u_end: piece.u_end,
        coeffs: right_coeffs,
    };

    (left, right)
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
#[allow(clippy::float_cmp)] // tests assert exact stored coords / round-trip values, not arithmetic results
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

    #[test]
    fn add_two_pieces_same_support() {
        let a = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0] };
        let b = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![3.0, 4.0] };
        let sum = (&a + &b).unwrap();
        assert_eq!(sum.coeffs, vec![4.0, 6.0]);
        assert_eq!(sum.u_start, 0.0);
        assert_eq!(sum.u_end, 1.0);
    }

    #[test]
    fn add_two_pieces_mismatched_degrees_pads_with_zero() {
        let a = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0, 3.0] };
        let b = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0] };
        let sum = (&a + &b).unwrap();
        assert_eq!(sum.coeffs, vec![2.0, 2.0, 3.0]);
    }

    #[test]
    fn add_two_pieces_mismatched_support_errors() {
        let a = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0] };
        let b = BezierPiece::<f64> { u_start: 0.5, u_end: 1.0, coeffs: vec![1.0] };
        assert!(matches!(&a + &b, Err(AlgebraError::SupportMismatch)));
    }

    use crate::ScalarNurbs;

    #[test]
    fn extract_single_bezier_piece_from_clamped_curve() {
        // Quadratic with no interior knots — already a single Bezier piece.
        let curve = ScalarNurbs::<f64>::try_new(
            2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], vec![0.0, 1.0, 4.0], None,
        ).unwrap();

        let pieces = extract_bezier_pieces(&curve);
        assert_eq!(pieces.len(), 1);
        let p = &pieces[0];
        assert_eq!(p.u_start, 0.0);
        assert_eq!(p.u_end, 1.0);
        assert_eq!(p.degree(), 2);
        // Eval at sample points matches.
        for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
            let exp = crate::eval::eval(&curve.as_view(), u);
            let got = p.evaluate(u);
            assert!((exp - got).abs() < 1e-12, "u={u}: exp={exp}, got={got}");
        }
    }

    #[test]
    fn extract_two_bezier_pieces_from_curve_with_interior_knot() {
        // Quadratic with an interior knot at 0.5.
        let curve = ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0],
            None,
        ).unwrap();

        let pieces = extract_bezier_pieces(&curve);
        assert_eq!(pieces.len(), 2);
        assert_eq!(pieces[0].u_start, 0.0);
        assert_eq!(pieces[0].u_end, 0.5);
        assert_eq!(pieces[1].u_start, 0.5);
        assert_eq!(pieces[1].u_end, 1.0);
        // Eval continuity: pieces[0].evaluate(0.5) == pieces[1].evaluate(0.5).
        let mid_left = pieces[0].evaluate(0.5);
        let mid_right = pieces[1].evaluate(0.5);
        assert!((mid_left - mid_right).abs() < 1e-12);
        // Each piece evaluates correctly.
        for u in [0.0, 0.25, 0.5] {
            let exp = crate::eval::eval(&curve.as_view(), u);
            let got = pieces[0].evaluate(u);
            assert!((exp - got).abs() < 1e-12);
        }
        for u in [0.5, 0.75, 1.0] {
            let exp = crate::eval::eval(&curve.as_view(), u);
            let got = pieces[1].evaluate(u);
            assert!((exp - got).abs() < 1e-12);
        }
    }

    #[test]
    fn split_piece_at_preserves_evaluation_on_each_side() {
        // p(u) = 1 + 2 * (u - 0) on [0, 1].
        let original = BezierPiece::<f64> { u_start: 0.0, u_end: 1.0, coeffs: vec![1.0, 2.0] };
        let (left, right) = split_piece_at(&original, 0.4);

        assert_eq!(left.u_start, 0.0);
        assert_eq!(left.u_end, 0.4);
        assert_eq!(right.u_start, 0.4);
        assert_eq!(right.u_end, 1.0);

        // Evaluation matches on each side.
        for u in [0.0, 0.2, 0.4] {
            let exp = original.evaluate(u);
            let got = left.evaluate(u);
            assert!((exp - got).abs() < 1e-12);
        }
        for u in [0.4, 0.7, 1.0] {
            let exp = original.evaluate(u);
            let got = right.evaluate(u);
            assert!((exp - got).abs() < 1e-12);
        }
    }

    #[test]
    fn bezier_pieces_to_nurbs_round_trips_extraction() {
        let original = ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.0, 3.0],
            None,
        ).unwrap();

        let pieces = extract_bezier_pieces(&original);
        let recomposed = bezier_pieces_to_nurbs(&pieces);

        // Eval-equivalence at sample points (knot vector may differ in multiplicity).
        for u in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
            let exp = crate::eval::eval(&original.as_view(), u);
            let got = crate::eval::eval(&recomposed.as_view(), u);
            assert!((exp - got).abs() < 1e-10, "u={u}: exp={exp}, got={got}");
        }
    }
}
