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

    /// Compute the derivative polynomial p'(u).
    ///
    /// For `p(u) = Σ_{k=0}^d coeffs[k] * (u - u_start)^k`, the derivative is
    /// `p'(u) = Σ_{k=1}^d k * coeffs[k] * (u - u_start)^{k-1}`, giving
    /// `coeffs'[j] = (j+1) * coeffs[j+1]` for `j = 0..d-1`.
    /// Degree drops by 1. Domain unchanged. A constant returns a zero polynomial.
    pub fn differentiate(&self) -> Self {
        if self.coeffs.len() <= 1 {
            return Self {
                u_start: self.u_start,
                u_end: self.u_end,
                coeffs: vec![T::ZERO],
            };
        }
        let coeffs = (1..self.coeffs.len())
            .map(|k| self.coeffs[k] * T::from_f64(k as f64))
            .collect();
        Self {
            u_start: self.u_start,
            u_end: self.u_end,
            coeffs,
        }
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
        Self {
            u_start,
            u_end,
            coeffs,
        }
    }
}

// ---------------------------------------------------------------------------
// Root-finding for f64 polynomials
// ---------------------------------------------------------------------------

impl BezierPiece<f64> {
    /// Find all real roots of this polynomial within its domain `[u_start, u_end]`.
    ///
    /// The polynomial is in Pascal-shifted monomial basis:
    /// `p(u) = sum_{k=0}^d coeffs[k] * (u - u_start)^k`
    ///
    /// Returns a `Vec<f64>` of roots in the *u*-domain (not the shifted variable).
    /// Roots are included if they lie within `[u_start - eps, u_end + eps]` where
    /// `eps` is a small tolerance for boundary inclusion.
    pub fn real_roots_in_domain(&self) -> Vec<f64> {
        // Tolerance for considering a root to lie on the domain boundary.
        const DOMAIN_TOL: f64 = 1e-10;
        // Tolerance for considering an eigenvalue purely real.
        const IMAG_TOL: f64 = 1e-8;

        // Strip trailing near-zero coefficients to find effective degree.
        let coeffs = self.effective_coeffs();
        let deg = if coeffs.is_empty() {
            0
        } else {
            coeffs.len() - 1
        };

        if deg == 0 {
            // Constant polynomial — no roots.
            return Vec::new();
        }

        let domain_len = self.u_end - self.u_start;

        // Solve in shifted variable x = u - u_start. Roots in x are in [0, domain_len].
        let x_roots = if deg == 1 {
            Self::roots_linear(&coeffs)
        } else if deg == 2 {
            Self::roots_quadratic(&coeffs)
        } else {
            Self::roots_companion_qr(&coeffs, IMAG_TOL)
        };

        // Filter: shift back to u-domain, keep roots within [u_start, u_end].
        let mut result = Vec::new();
        for x in x_roots {
            let u = x + self.u_start;
            if u >= self.u_start - DOMAIN_TOL && u <= self.u_end + DOMAIN_TOL {
                // Clamp to exact domain boundaries.
                let u_clamped = u.clamp(self.u_start, self.u_end);
                // Deduplicate: skip if we already have a root very close.
                if !result
                    .iter()
                    .any(|&existing: &f64| (existing - u_clamped).abs() < DOMAIN_TOL)
                {
                    result.push(u_clamped);
                }
            }
        }

        // Also check domain endpoints if they are roots (may be missed by numerics).
        for &boundary_x in &[0.0, domain_len] {
            let u = boundary_x + self.u_start;
            let val = self.evaluate(u);
            if val.abs() < DOMAIN_TOL * (1.0 + self.coeff_scale())
                && !result
                    .iter()
                    .any(|&existing: &f64| (existing - u).abs() < DOMAIN_TOL)
            {
                result.push(u);
            }
        }

        result
    }

    /// Return the effective coefficients with trailing near-zero entries stripped.
    fn effective_coeffs(&self) -> Vec<f64> {
        let scale = self.coeff_scale();
        let tol = 1e-14 * (1.0 + scale);
        let mut coeffs = self.coeffs.clone();
        while coeffs.len() > 1 && coeffs.last().is_some_and(|c| c.abs() < tol) {
            coeffs.pop();
        }
        coeffs
    }

    /// Scale factor for coefficient magnitude (used for relative tolerances).
    fn coeff_scale(&self) -> f64 {
        self.coeffs.iter().map(|c| c.abs()).fold(0.0_f64, f64::max)
    }

    /// Roots of a linear polynomial: coeffs[0] + coeffs[1] * x = 0.
    fn roots_linear(coeffs: &[f64]) -> Vec<f64> {
        let x = -coeffs[0] / coeffs[1];
        if x.is_finite() { vec![x] } else { Vec::new() }
    }

    /// Roots of a quadratic polynomial: coeffs[0] + coeffs[1]*x + coeffs[2]*x^2 = 0.
    fn roots_quadratic(coeffs: &[f64]) -> Vec<f64> {
        let a = coeffs[2];
        let b = coeffs[1];
        let c = coeffs[0];
        let disc = b * b - 4.0 * a * c;
        if disc < 0.0 {
            return Vec::new();
        }
        if disc == 0.0 {
            let x = -b / (2.0 * a);
            return if x.is_finite() { vec![x] } else { Vec::new() };
        }
        // Numerically stable quadratic formula (avoid catastrophic cancellation).
        let sqrt_disc = disc.sqrt();
        let q = -0.5 * (b + b.signum() * sqrt_disc);
        let x1 = q / a;
        let x2 = c / q;
        let mut roots = Vec::new();
        if x1.is_finite() {
            roots.push(x1);
        }
        if x2.is_finite() {
            roots.push(x2);
        }
        roots
    }

    /// Find roots of a polynomial of degree >= 3 via companion matrix eigenvalues.
    fn roots_companion_qr(coeffs: &[f64], imag_tol: f64) -> Vec<f64> {
        let n = coeffs.len() - 1; // degree
        let leading = coeffs[n];

        // Build companion matrix (n x n), stored row-major.
        // The companion matrix for monic polynomial x^n + a_{n-1}x^{n-1} + ... + a_0:
        //   C[i][j] = 1  if i == j+1  (sub-diagonal)
        //   C[i][n-1] = -a_i / a_n    (last column)
        //   else 0
        let mut c_mat = vec![0.0; n * n];
        for i in 1..n {
            c_mat[i * n + (i - 1)] = 1.0; // sub-diagonal
        }
        for i in 0..n {
            c_mat[i * n + (n - 1)] = -coeffs[i] / leading; // last column
        }

        // QR iteration to find eigenvalues. The companion matrix is already
        // upper Hessenberg.
        let eigenvalues = hessenberg_qr_eigenvalues(&mut c_mat, n, imag_tol);

        // Filter for real eigenvalues.
        eigenvalues
            .into_iter()
            .filter(|(_, imag)| imag.abs() < imag_tol)
            .map(|(real, _)| real)
            .collect()
    }
}

/// QR iteration on an upper Hessenberg matrix to extract eigenvalues.
/// Returns (real, imag) pairs.
///
/// Uses explicit single-shift QR with Givens rotations. For the small matrices
/// we deal with (degree <= 10), this is robust and fast enough.
///
/// `mat` is n x n row-major. Modified in place (reduced to quasi-upper-triangular).
fn hessenberg_qr_eigenvalues(mat: &mut [f64], n: usize, tol: f64) -> Vec<(f64, f64)> {
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![(mat[0], 0.0)];
    }

    let max_iter = 200 * n;
    let mut eigenvalues = Vec::with_capacity(n);
    let mut p = n; // active sub-matrix size (top-left p x p block)

    'outer: for _ in 0..max_iter {
        if p == 0 {
            break;
        }
        if p == 1 {
            eigenvalues.push((mat[0], 0.0));
            break;
        }
        if p == 2 {
            let eigs = eigenvalues_2x2(mat, n, 0);
            eigenvalues.push(eigs.0);
            eigenvalues.push(eigs.1);
            break;
        }

        // Check for deflation from bottom: find smallest k such that
        // H[k, k-1] is negligible.
        for k in (1..p).rev() {
            let sub = mat[k * n + (k - 1)];
            let diag_sum = mat[k * n + k].abs() + mat[(k - 1) * n + (k - 1)].abs();
            let threshold = tol * diag_sum.max(1e-30);
            if sub.abs() <= threshold {
                mat[k * n + (k - 1)] = 0.0;
                if k == p - 1 {
                    // 1x1 block at bottom-right.
                    eigenvalues.push((mat[(p - 1) * n + (p - 1)], 0.0));
                    p -= 1;
                    continue 'outer;
                }
                if k == p - 2 {
                    // 2x2 block at bottom-right.
                    let eigs = eigenvalues_2x2(mat, n, p - 2);
                    eigenvalues.push(eigs.0);
                    eigenvalues.push(eigs.1);
                    p -= 2;
                    continue 'outer;
                }
                // Deflation in the interior — we only chase the bottom block.
                break;
            }
        }

        // Single-shift QR step with Wilkinson shift.
        // Shift = eigenvalue of trailing 2x2 closer to H[p-1][p-1].
        let shift = wilkinson_shift(mat, n, p);
        qr_step_givens(mat, n, p, shift);
    }

    eigenvalues
}

/// Compute the Wilkinson shift: the eigenvalue of the trailing 2x2 block
/// that is closer to H[p-1][p-1].
fn wilkinson_shift(mat: &[f64], n: usize, p: usize) -> f64 {
    let a = mat[(p - 2) * n + (p - 2)];
    let b = mat[(p - 2) * n + (p - 1)];
    let c = mat[(p - 1) * n + (p - 2)];
    let d = mat[(p - 1) * n + (p - 1)];

    let trace = a + d;
    let det = a * d - b * c;
    let disc = trace * trace - 4.0 * det;

    if disc >= 0.0 {
        let sqrt_disc = disc.sqrt();
        let e1 = (trace + sqrt_disc) / 2.0;
        let e2 = (trace - sqrt_disc) / 2.0;
        // Return the one closer to d = H[p-1][p-1].
        if (e1 - d).abs() < (e2 - d).abs() {
            e1
        } else {
            e2
        }
    } else {
        // Complex eigenvalues — use the real part as shift.
        trace / 2.0
    }
}

/// One explicit shifted QR step using Givens rotations on the active p x p
/// upper-Hessenberg sub-matrix. `mat` has stride `n`.
fn qr_step_givens(mat: &mut [f64], n: usize, p: usize, shift: f64) {
    // Apply shift: H <- H - shift * I
    for i in 0..p {
        mat[i * n + i] -= shift;
    }

    // QR factorization via Givens rotations on the Hessenberg matrix.
    // Store the rotations to apply Q^T from the right afterward.
    let mut cs = Vec::with_capacity(p - 1);
    let mut sn = Vec::with_capacity(p - 1);

    for i in 0..p - 1 {
        let a = mat[i * n + i];
        let b = mat[(i + 1) * n + i];
        let (c, s, _r) = givens_rotation(a, b);
        cs.push(c);
        sn.push(s);

        // Apply G(i, i+1, theta) from the left to rows i, i+1.
        for j in i..p {
            let t1 = mat[i * n + j];
            let t2 = mat[(i + 1) * n + j];
            mat[i * n + j] = c * t1 + s * t2;
            mat[(i + 1) * n + j] = -s * t1 + c * t2;
        }
    }

    // Apply rotations from the right: H <- R * Q = R * G_1^T * G_2^T * ...
    for i in 0..p - 1 {
        let c = cs[i];
        let s = sn[i];
        // Apply G(i, i+1, theta)^T from the right to columns i, i+1.
        let row_end = (i + 2).min(p); // Hessenberg: only rows 0..i+2 can be nonzero in col i
        for j in 0..row_end {
            let t1 = mat[j * n + i];
            let t2 = mat[j * n + i + 1];
            mat[j * n + i] = c * t1 + s * t2;
            mat[j * n + i + 1] = -s * t1 + c * t2;
        }
    }

    // Undo shift: H <- H + shift * I
    for i in 0..p {
        mat[i * n + i] += shift;
    }
}

/// Compute Givens rotation parameters (c, s, r) such that:
/// [c  s] [a]   [r]
/// [-s c] [b] = [0]
fn givens_rotation(a: f64, b: f64) -> (f64, f64, f64) {
    if b.abs() < 1e-300 {
        return (1.0, 0.0, a);
    }
    if a.abs() < 1e-300 {
        return (0.0, b.signum(), b.abs());
    }
    let r = a.hypot(b);
    (a / r, b / r, r)
}

/// Extract eigenvalues of the 2x2 block starting at (start, start) in the n-wide matrix.
fn eigenvalues_2x2(mat: &[f64], n: usize, start: usize) -> ((f64, f64), (f64, f64)) {
    let a = mat[start * n + start];
    let b = mat[start * n + start + 1];
    let c = mat[(start + 1) * n + start];
    let d = mat[(start + 1) * n + start + 1];

    let trace = a + d;
    let det = a * d - b * c;
    let disc = trace * trace - 4.0 * det;

    if disc >= 0.0 {
        let sqrt_disc = disc.sqrt();
        let r1 = (trace + sqrt_disc) / 2.0;
        let r2 = (trace - sqrt_disc) / 2.0;
        ((r1, 0.0), (r2, 0.0))
    } else {
        let sqrt_disc = (-disc).sqrt();
        let real = trace / 2.0;
        let imag = sqrt_disc / 2.0;
        ((real, imag), (real, -imag))
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

/// Binomial coefficient C(n, k). Integer-valued.
/// Safe for `n ≤ 50` (largest intermediate ≈ C(50, 25) × 25 ≈ 3e15, well under u64 max).
/// Crate-wide `MAX_DEGREE = 20`; convolve worst case is `n = 2 * MAX_DEGREE = 40`, comfortably safe.
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
mod tests;
