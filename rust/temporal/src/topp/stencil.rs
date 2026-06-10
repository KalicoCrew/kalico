pub fn b_d_weights(hl: f64, hr: f64) -> [f64; 3] {
    debug_assert!(hl > 0.0 && hr > 0.0);
    let d = hl * hr * (hl + hr);
    [-hr * hr / d, (hr * hr - hl * hl) / d, hl * hl / d]
}

pub fn b_dd_weights(hl: f64, hr: f64) -> [f64; 3] {
    debug_assert!(hl > 0.0 && hr > 0.0);
    let d = hl * hr * (hl + hr);
    [2.0 * hr / d, -2.0 * (hl + hr) / d, 2.0 * hl / d]
}

pub fn stencil_at(i: usize, n: usize, h_intervals: &[f64]) -> ([usize; 3], f64, f64) {
    debug_assert!(n >= 3 && i < n && h_intervals.len() == n - 1);
    if i == 0 {
        ([0, 1, 2], h_intervals[0], h_intervals[1])
    } else if i == n - 1 {
        (
            [n - 3, n - 2, n - 1],
            h_intervals[n - 3],
            h_intervals[n - 2],
        )
    } else {
        ([i - 1, i, i + 1], h_intervals[i - 1], h_intervals[i])
    }
}

pub fn s_dddot_at_weights(b: &[f64], i: usize, h_intervals: &[f64]) -> f64 {
    let n = b.len();
    let (idx, hl, hr) = stencil_at(i, n, h_intervals);
    let w = b_dd_weights(hl, hr);
    let b_dd = w[0] * b[idx[0]] + w[1] * b[idx[1]] + w[2] * b[idx[2]];
    b[i].max(0.0).sqrt() * b_dd / 2.0
}

#[cfg(test)]
mod tests;
