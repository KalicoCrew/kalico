/// Stencil dispatch tag for `s_dddot_at`'s branches.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SDddotStencil {
    StartBoundary,
    Interior,
    EndBoundary,
}

/// Returns the stencil dispatch tag for a given grid index.
///
/// Caller invariant: `n ≥ 3`; `i < n`.
pub fn stencil_for(n: usize, i: usize) -> SDddotStencil {
    debug_assert!(n >= 3);
    debug_assert!(i < n);
    if i == 0 {
        SDddotStencil::StartBoundary
    } else if i == n - 1 {
        SDddotStencil::EndBoundary
    } else {
        SDddotStencil::Interior
    }
}

/// Path-third-derivative `s‴` at grid index `i` via width-1 b-FD.
///
/// `b[i].max(0.0)` guards numerically-borderline iterates where Clarabel may
/// produce slightly-negative `b[i]` due to solver-residual rounding.
///
/// Returns `s‴_i = √b_i · b''(s_i) / 2`.
pub fn s_dddot_at(b: &[f64], i: usize, h: f64) -> f64 {
    debug_assert!(b.len() >= 3, "stencil requires n >= 3");
    debug_assert!(h > 0.0, "h must be positive");
    debug_assert!(i < b.len());
    let n = b.len();
    let s_dot = b[i].max(0.0).sqrt();
    let b_dd = if i == 0 {
        (b[0] - 2.0 * b[1] + b[2]) / (h * h)
    } else if i == n - 1 {
        (b[n - 3] - 2.0 * b[n - 2] + b[n - 1]) / (h * h)
    } else {
        (b[i - 1] - 2.0 * b[i] + b[i + 1]) / (h * h)
    };
    s_dot * b_dd / 2.0
}

#[cfg(test)]
mod tests;
