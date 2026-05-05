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
pub(crate) enum SDddotStencil {
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
pub(crate) fn stencil_for(n: usize, i: usize) -> SDddotStencil {
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
pub(crate) fn s_dddot_at(b: &[f64], i: usize, h: f64) -> f64 {
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
mod tests {
    use super::*;

    fn build_b_from_polynomial<F: Fn(f64) -> f64>(n: usize, h: f64, f: F) -> Vec<f64> {
        (0..n).map(|i| f(i as f64 * h)).collect()
    }

    /// Quadratic b(s) = α·s² + β·s + γ. b''(s) = 2α everywhere; b''''(s) = 0.
    /// Width-1 stencil's truncation coefficient (h²·b''''/12) vanishes, so
    /// the estimate is exact up to floating-point round-off.
    #[test]
    fn s_dddot_at_quadratic_interior_within_machine_epsilon() {
        let alpha = 0.7;
        let beta = 1.3;
        let gamma = 5.0;
        let h = 0.5;
        let n = 10;
        let b = build_b_from_polynomial(n, h, |s| alpha * s * s + beta * s + gamma);

        // Expected: s‴_i = √b_i · α at all interior indices.
        for i in 1..n - 1 {
            let s = i as f64 * h;
            let b_i = alpha * s * s + beta * s + gamma;
            let expected = b_i.sqrt() * alpha;
            let got = s_dddot_at(&b, i, h);
            assert!(
                (got - expected).abs() < 1e-12,
                "i={i}: got {got}, expected {expected} (diff {})",
                got - expected
            );
        }
    }

    /// Cubic b(s) = α·s³ + β·s² + γ·s + δ. b''(s) = 6α·s + 2β; b''''(s) = 0.
    /// Width-1 stencil exact up to round-off.
    #[test]
    fn s_dddot_at_cubic_interior_within_machine_epsilon() {
        let alpha = 0.4;
        let beta = -0.2;
        let gamma = 1.0;
        let delta = 3.0;
        let h = 0.5;
        let n = 10;
        let b = build_b_from_polynomial(n, h, |s| {
            alpha * s * s * s + beta * s * s + gamma * s + delta
        });

        for i in 1..n - 1 {
            let s = i as f64 * h;
            let b_i = alpha * s * s * s + beta * s * s + gamma * s + delta;
            let b_pp = 6.0 * alpha * s + 2.0 * beta;
            let expected = b_i.sqrt() * b_pp / 2.0;
            let got = s_dddot_at(&b, i, h);
            assert!(
                (got - expected).abs() < 1e-10,
                "i={i}: got {got}, expected {expected} (diff {})",
                got - expected
            );
        }
    }

    /// Quartic b(s) = α·s⁴ + …. b''''(s) = 24α (constant non-zero), so the
    /// width-1 stencil has a leading h²·b''''/12 = 2αh² truncation on b''.
    /// s‴ truncation is √b · h² · α. Pin within that tolerance.
    #[test]
    fn s_dddot_at_quartic_interior_within_truncation_bound() {
        let alpha = 0.1;
        let h = 0.25;
        let n = 12;
        let b = build_b_from_polynomial(n, h, |s| alpha * s * s * s * s + 100.0);

        for i in 1..n - 1 {
            let s = i as f64 * h;
            let b_i = alpha * s * s * s * s + 100.0;
            let b_pp = 12.0 * alpha * s * s;
            let expected = b_i.sqrt() * b_pp / 2.0;
            let got = s_dddot_at(&b, i, h);
            // Truncation tolerance: |error| ≤ √b · h² · α (with safety factor 2).
            let tol = 2.0 * b_i.sqrt() * h * h * alpha;
            assert!(
                (got - expected).abs() < tol,
                "i={i}: got {got}, expected {expected} (diff {}, tol {tol})",
                got - expected
            );
        }
    }

    /// Constant b(s) = c. b''(s) = 0 everywhere, so s‴ = 0 at every grid
    /// index including boundaries. (Forward / backward second-differences of
    /// a constant are also zero.)
    #[test]
    fn s_dddot_at_constant_returns_zero_everywhere() {
        let h = 1.0;
        let n = 8;
        let b = vec![100.0; n];

        for i in 0..n {
            let got = s_dddot_at(&b, i, h);
            assert!(got.abs() < 1e-12, "i={i}: got {got}, expected 0");
        }
    }

    /// b[i] = 0 should produce s_dddot = 0 (the .max(0.0).sqrt() guard
    /// makes s_dot = 0). No NaN/Inf even with non-zero b-FD numerator.
    #[test]
    fn s_dddot_at_handles_zero_b_without_nan() {
        let h = 1.0;
        let mut b = vec![10.0; 5];
        b[1] = 0.0;
        let got = s_dddot_at(&b, 1, h);
        assert_eq!(got, 0.0, "expected exactly 0.0, got {got}");
        assert!(got.is_finite());
    }

    /// b[i] slightly negative (Clarabel residual rounding) should also
    /// produce 0, not NaN.
    #[test]
    fn s_dddot_at_handles_slightly_negative_b_without_nan() {
        let h = 1.0;
        let mut b = vec![10.0; 5];
        b[1] = -1e-15;
        let got = s_dddot_at(&b, 1, h);
        assert_eq!(got, 0.0);
        assert!(got.is_finite());
    }

    /// Boundary stencil at i=0 with b(s) = α·s² + γ (β=0, b''=2α, b''''=0).
    /// Forward FD has O(h)·b''' leading error, but b''' = 0 here too, so
    /// forward FD is also exact for quadratics.
    #[test]
    fn s_dddot_at_boundary_quadratic_exact() {
        let alpha = 0.5;
        let gamma = 4.0;
        let h = 0.3;
        let n = 6;
        let b = build_b_from_polynomial(n, h, |s| alpha * s * s + gamma);

        // i=0
        let b_0 = gamma;
        let expected_0 = b_0.sqrt() * alpha;
        let got_0 = s_dddot_at(&b, 0, h);
        assert!(
            (got_0 - expected_0).abs() < 1e-12,
            "i=0: got {got_0}, expected {expected_0}"
        );

        // i=n-1
        let s_last = (n - 1) as f64 * h;
        let b_last = alpha * s_last * s_last + gamma;
        let expected_last = b_last.sqrt() * alpha;
        let got_last = s_dddot_at(&b, n - 1, h);
        assert!(
            (got_last - expected_last).abs() < 1e-12,
            "i=n-1: got {got_last}, expected {expected_last}"
        );
    }

    /// `stencil_for` dispatch.
    #[test]
    fn stencil_for_dispatches_correctly() {
        assert_eq!(stencil_for(10, 0), SDddotStencil::StartBoundary);
        assert_eq!(stencil_for(10, 1), SDddotStencil::Interior);
        assert_eq!(stencil_for(10, 5), SDddotStencil::Interior);
        assert_eq!(stencil_for(10, 8), SDddotStencil::Interior);
        assert_eq!(stencil_for(10, 9), SDddotStencil::EndBoundary);
        assert_eq!(stencil_for(3, 0), SDddotStencil::StartBoundary);
        assert_eq!(stencil_for(3, 1), SDddotStencil::Interior);
        assert_eq!(stencil_for(3, 2), SDddotStencil::EndBoundary);
    }
}
