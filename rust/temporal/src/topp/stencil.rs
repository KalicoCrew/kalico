//! Width-1 b-FD stencil for path-third-derivative `s‴(s_i)`.
//!
//! Single source of truth for the stencil used by `verify::check` and the
//! per-axis Cartesian-jerk SLP machinery (`solver::max_axis_ratio`,
//! `solver::build_axis_jerk_cuts`, `solver::append_axis_jerk_cut_to_clarabel`).
//! The path-jerk SOC chain in `constraints::block_(h)` and the path-jerk
//! SLP cuts in `solver::slp_solve` already use the width-1 b-FD form
//! directly; this module brings everything else into agreement.
//!
//! # Math
//!
//! With `b(s) = ṡ²`, the chain rule gives `s‴(t) = ½ · b''(s) · √b`.
//! Width-1 b-FD estimates `b''(s_i)`:
//!
//! - i = 0:        forward FD  `(b[0] − 2·b[1] + b[2]) / h²`,  O(h)·b''' truncation.
//! - i ∈ [1, n-2]: central FD  `(b[i-1] − 2·b[i] + b[i+1]) / h²`,  O(h²)·b'''' truncation.
//! - i = n-1:      backward FD `(b[n-3] − 2·b[n-2] + b[n-1]) / h²`,  O(h)·b''' truncation.
//!
//! See `docs/superpowers/specs/2026-05-05-stencil-unification-design.md` for
//! the truncation analysis (verifier sign-off + Codex review trail).

/// Stencil dispatch tag mirroring `s_dddot_at`'s branches. Used by the SLP
/// cut linearization to select the correct coefficient formulas.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SDddotStencil {
    /// i = 0, forward FD.
    StartBoundary,
    /// i ∈ [1, n-2], central FD.
    Interior,
    /// i = n-1, backward FD.
    EndBoundary,
}

/// Returns the stencil dispatch tag for a given grid index.
///
/// Caller invariant: `n ≥ 3` (boundary stencils need 3 grid points).
/// Caller invariant: `i < n`.
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
/// Caller-provided invariants: `n ≥ 3` (required for boundary stencils);
/// `h > 0`; `b.len() == n`. The helper applies `.max(0.0)` to `b[i]`
/// defensively before `sqrt` to keep numerically-borderline iterates
/// (where Clarabel may produce slightly-negative `b[i]` due to
/// solver-residual rounding) from producing `NaN`. The b-FD second-
/// difference itself accepts any `b` values; nothing in the stencil
/// arithmetic requires non-negativity beyond the `√b` factor.
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
