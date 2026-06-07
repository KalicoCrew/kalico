/// 3-point stencil weights for b′(s_i) over spacings (hl, hr); exact for
/// quadratics; order is [w_{i−1}, w_i, w_{i+1}].
pub fn b_d_weights(hl: f64, hr: f64) -> [f64; 3] {
    debug_assert!(hl > 0.0 && hr > 0.0);
    let d = hl * hr * (hl + hr);
    [-hr * hr / d, (hr * hr - hl * hl) / d, hl * hl / d]
}

/// 3-point stencil weights for b″(s_i) over spacings (hl, hr); exact for
/// quadratics; O(h) truncation when hl ≠ hr, O(h²) when equal.
pub fn b_dd_weights(hl: f64, hr: f64) -> [f64; 3] {
    debug_assert!(hl > 0.0 && hr > 0.0);
    let d = hl * hr * (hl + hr);
    [2.0 * hr / d, -2.0 * (hl + hr) / d, 2.0 * hl / d]
}

/// Stencil index triple and spacings for point `i` of a grid with
/// per-interval spacings `h_intervals` (len = n−1). Boundary points return
/// the 3-point stencil anchored at the edge — for b″ this matches the legacy
/// one-sided second difference (3-point second-difference weights are
/// anchor-position-independent); for b′ it is a forward/backward 3-point
/// approximation, NOT the 2-point one-sided difference block (b) uses for
/// its edge rows (those stay 2-point on purpose — bit-equivalence with the
/// legacy bundle).
pub fn stencil_at(i: usize, n: usize, h_intervals: &[f64]) -> ([usize; 3], f64, f64) {
    debug_assert!(n >= 3 && i < n && h_intervals.len() == n - 1);
    if i == 0 {
        ([0, 1, 2], h_intervals[0], h_intervals[1])
    } else if i == n - 1 {
        ([n - 3, n - 2, n - 1], h_intervals[n - 3], h_intervals[n - 2])
    } else {
        ([i - 1, i, i + 1], h_intervals[i - 1], h_intervals[i])
    }
}

/// `s‴_i = √b_i · b″(s_i) / 2` with non-uniform-capable weights.
pub fn s_dddot_at_weights(b: &[f64], i: usize, h_intervals: &[f64]) -> f64 {
    let n = b.len();
    let (idx, hl, hr) = stencil_at(i, n, h_intervals);
    let w = b_dd_weights(hl, hr);
    let b_dd = w[0] * b[idx[0]] + w[1] * b[idx[1]] + w[2] * b[idx[2]];
    b[i].max(0.0).sqrt() * b_dd / 2.0
}

#[cfg(test)]
mod tests;
