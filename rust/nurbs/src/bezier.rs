use crate::{AlgebraError, Float, ScalarNurbs};

/// One Bézier piece as a polynomial in the Pascal-shifted monomial basis:
/// `p(u) = Σ_{k=0..d} coeffs[k] * (u - u_start)^k`
#[derive(Debug, Clone, PartialEq)]
pub struct BezierPiece<T: Float> {
    pub u_start: T,
    pub u_end: T,
    pub coeffs: Vec<T>,
}

impl<T: Float> BezierPiece<T> {
    pub fn degree(&self) -> usize {
        self.coeffs.len().saturating_sub(1)
    }

    pub fn evaluate(&self, u: T) -> T {
        let dx = u - self.u_start;
        let mut acc = T::ZERO;
        for c in self.coeffs.iter().rev() {
            acc = acc * dx + *c;
        }
        acc
    }

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

    pub fn zero(u_start: T, u_end: T, degree: usize) -> Self {
        Self {
            u_start,
            u_end,
            coeffs: vec![T::ZERO; degree + 1],
        }
    }

    pub fn to_bernstein(&self) -> Vec<T> {
        let d = self.degree();
        let h = self.u_end - self.u_start;
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

impl BezierPiece<f64> {
    pub fn real_roots_in_domain(&self) -> Vec<f64> {
        const DOMAIN_TOL: f64 = 1e-10;
        const IMAG_TOL: f64 = 1e-8;

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

        let x_roots = if deg == 1 {
            Self::roots_linear(&coeffs)
        } else if deg == 2 {
            Self::roots_quadratic(&coeffs)
        } else {
            Self::roots_companion_qr(&coeffs, IMAG_TOL)
        };

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

    fn effective_coeffs(&self) -> Vec<f64> {
        let scale = self.coeff_scale();
        let tol = 1e-14 * (1.0 + scale);
        let mut coeffs = self.coeffs.clone();
        while coeffs.len() > 1 && coeffs.last().is_some_and(|c| c.abs() < tol) {
            coeffs.pop();
        }
        coeffs
    }

    fn coeff_scale(&self) -> f64 {
        self.coeffs.iter().map(|c| c.abs()).fold(0.0_f64, f64::max)
    }

    fn roots_linear(coeffs: &[f64]) -> Vec<f64> {
        let x = -coeffs[0] / coeffs[1];
        if x.is_finite() { vec![x] } else { Vec::new() }
    }

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
        // Numerically stable form — avoids catastrophic cancellation.
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

    fn roots_companion_qr(coeffs: &[f64], imag_tol: f64) -> Vec<f64> {
        let n = coeffs.len() - 1;
        let leading = coeffs[n];

        let mut c_mat = vec![0.0; n * n];
        for i in 1..n {
            c_mat[i * n + (i - 1)] = 1.0;
        }
        for i in 0..n {
            c_mat[i * n + (n - 1)] = -coeffs[i] / leading;
        }

        let eigenvalues = hessenberg_qr_eigenvalues(&mut c_mat, n, imag_tol);

        eigenvalues
            .into_iter()
            .filter(|(_, imag)| imag.abs() < imag_tol)
            .map(|(real, _)| real)
            .collect()
    }
}

fn hessenberg_qr_eigenvalues(mat: &mut [f64], n: usize, tol: f64) -> Vec<(f64, f64)> {
    if n == 0 {
        return Vec::new();
    }
    if n == 1 {
        return vec![(mat[0], 0.0)];
    }

    let max_iter = 200 * n;
    let mut eigenvalues = Vec::with_capacity(n);
    let mut p = n;

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

        for k in (1..p).rev() {
            let sub = mat[k * n + (k - 1)];
            let diag_sum = mat[k * n + k].abs() + mat[(k - 1) * n + (k - 1)].abs();
            let threshold = tol * diag_sum.max(1e-30);
            if sub.abs() <= threshold {
                mat[k * n + (k - 1)] = 0.0;
                if k == p - 1 {
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
                break;
            }
        }

        let shift = wilkinson_shift(mat, n, p);
        qr_step_givens(mat, n, p, shift);
    }

    eigenvalues
}

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

fn qr_step_givens(mat: &mut [f64], n: usize, p: usize, shift: f64) {
    for i in 0..p {
        mat[i * n + i] -= shift;
    }

    let mut cs = Vec::with_capacity(p - 1);
    let mut sn = Vec::with_capacity(p - 1);

    for i in 0..p - 1 {
        let a = mat[i * n + i];
        let b = mat[(i + 1) * n + i];
        let (c, s, _r) = givens_rotation(a, b);
        cs.push(c);
        sn.push(s);

        for j in i..p {
            let t1 = mat[i * n + j];
            let t2 = mat[(i + 1) * n + j];
            mat[i * n + j] = c * t1 + s * t2;
            mat[(i + 1) * n + j] = -s * t1 + c * t2;
        }
    }

    for i in 0..p - 1 {
        let c = cs[i];
        let s = sn[i];
        let row_end = (i + 2).min(p); // Hessenberg: only rows 0..i+2 can be nonzero in col i
        for j in 0..row_end {
            let t1 = mat[j * n + i];
            let t2 = mat[j * n + i + 1];
            mat[j * n + i] = c * t1 + s * t2;
            mat[j * n + i + 1] = -s * t1 + c * t2;
        }
    }

    for i in 0..p {
        mat[i * n + i] += shift;
    }
}

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

pub fn extract_bezier_pieces<T: Float>(curve: &ScalarNurbs<T>) -> Vec<BezierPiece<T>> {
    let refined = crate::knot::refined_to_full_multiplicity(curve);
    let p = refined.degree() as usize;
    let knots = refined.knots();
    let cps = refined.control_points();

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
        cp_idx += p;
    }

    pieces
}

pub fn bezier_pieces_to_nurbs<T: Float>(pieces: &[BezierPiece<T>]) -> ScalarNurbs<T> {
    assert!(!pieces.is_empty(), "bezier_pieces_to_nurbs: empty input");
    let p = pieces[0].degree();
    for w in pieces.windows(2) {
        assert!(w[0].u_end == w[1].u_start, "non-contiguous Bezier pieces");
        assert!(w[1].degree() == p, "inconsistent degrees");
    }

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

    let mut cps: Vec<T> = Vec::with_capacity(pieces.len() * p + 1);
    for (i, piece) in pieces.iter().enumerate() {
        let bernstein = piece.to_bernstein();
        if i == 0 {
            cps.extend_from_slice(&bernstein);
        } else {
            cps.extend_from_slice(&bernstein[1..]);
        }
    }

    ScalarNurbs::try_new(p as u8, knots, cps)
        .expect("bezier_pieces_to_nurbs: invariants should hold")
}

pub fn split_piece_at<T: Float>(
    piece: &BezierPiece<T>,
    u_split: T,
) -> (BezierPiece<T>, BezierPiece<T>) {
    assert!(
        u_split > piece.u_start && u_split < piece.u_end,
        "u_split must be strictly interior"
    );
    let d = piece.degree();

    let left = BezierPiece {
        u_start: piece.u_start,
        u_end: u_split,
        coeffs: piece.coeffs.clone(),
    };

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

// Safe for n ≤ 50; crate MAX_DEGREE = 20, convolve worst case n = 40.
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
#[allow(clippy::float_cmp)]
mod tests;
