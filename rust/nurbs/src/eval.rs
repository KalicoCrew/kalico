//! NURBS evaluation: de Boor, vector eval, derivative, curvature.
//! See spec §eval module.
//!
//! # Safety note
//!
//! This module uses `unsafe { get_unchecked }` for the de Boor index accesses.
//! All such accesses are proved in-bounds by the `find_knot_span` invariant
//! (k ∈ [p, n-1] per Piegl & Tiller Algorithm A4.1); each site is accompanied
//! by a `SAFETY:` comment and a `debug_assert!` of the precondition.
// `unsafe_code` is workspace-denied by default; this module is the sole
// exception because the hot-path MCU de Boor evaluators require provably-safe
// index elimination to avoid panic-symbol contamination in the release firmware.
#![allow(unsafe_code)]

use crate::{Float, MAX_DEGREE, MIN_PARAMETRIC_SPEED, NurbsView, VectorNurbsView, WORKSPACE_SIZE};

// Re-export from knot module for transitional internal use. Eventually
// callers should import directly from `crate::knot::find_knot_span`.
#[cfg(feature = "host")]
pub(crate) use crate::knot::find_knot_span;

// MCU build needs an inline copy since knot module is host-only.
#[cfg(not(feature = "host"))]
#[inline]
pub(crate) fn find_knot_span<T: Float>(knots: &[T], p: usize, n: usize, u: T) -> usize {
    // Pre-conditions: knots.len() == n + p + 1; n >= 1; p >= 0.
    // All index accesses below are in-bounds:
    //   knots[n]     : n < n+p+1 (since p >= 0)
    //   knots[p]     : p < n+p+1 (since n >= 1)
    //   knots[mid]   : mid ∈ [low,high] ⊆ [p,n] ⊂ [0, n+p+1)
    //   knots[mid+1] : mid+1 ≤ n+1 ≤ n+p+1 (since high ≤ n)
    debug_assert!(knots.len() == n + p + 1);
    debug_assert!(n >= 1);
    // SAFETY: n < n+p+1 = knots.len() (p is usize so >= 0, n >= 1)
    if u >= unsafe { *knots.get_unchecked(n) } {
        return n - 1;
    }
    // SAFETY: p < n+p+1 = knots.len() (n >= 1)
    if u <= unsafe { *knots.get_unchecked(p) } {
        return p;
    }
    let mut low = p;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while {
        // SAFETY: mid ∈ [low,high] ⊆ [p,n] < n+p+1 = knots.len();
        //         mid+1 ≤ high+1 ≤ n+1 ≤ n+p+1 = knots.len() (p >= 0).
        let km = unsafe { *knots.get_unchecked(mid) };
        let km1 = unsafe { *knots.get_unchecked(mid + 1) };
        u < km || u >= km1
    } {
        // SAFETY: same bounds — recompute on next iteration.
        let km = unsafe { *knots.get_unchecked(mid) };
        if u < km {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

/// de Boor's algorithm at parameter `u` over `cps` with degree `p`.
/// Stack scratch is `[T; WORKSPACE_SIZE]`. Caller has validated that
/// `p as usize <= MAX_DEGREE`.
///
/// Reference: Piegl & Tiller "The NURBS Book" Algorithm A4.1 (de Boor).
///
/// # Index-safety invariant
///
/// `find_knot_span` returns `k ∈ [p, n-1]` (algorithm A2.1 postcondition).
///
/// For `j ∈ 0..=p`: `k - p + j ∈ [0, k] ⊆ [0, n-1]` — valid index into
/// `cps` (len `n`) and into `knots` (len `n + p + 1`).
///
/// For the recurrence with `r ∈ 1..=p`, `j ∈ r..=p`:
/// `k + 1 + j - r ≤ k + p ≤ (n-1) + p = n + p - 1 < n + p + 1` — valid.
///
/// All accesses below are proved in-bounds by this invariant; `get_unchecked`
/// eliminates the panic paths that LLVM cannot otherwise remove on the MCU.
#[inline]
pub(crate) fn de_boor_inner<T: Float>(cps: &[T], knots: &[T], degree: u8, u: T) -> T {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    // SAFETY: k ∈ [p, n-1] from find_knot_span, so k-p+j ∈ [0, n-1] for j ∈ 0..=p.
    debug_assert!(k >= p && k < n, "find_knot_span invariant: k ∈ [p, n-1]");
    debug_assert!(knots.len() == n + p + 1, "knots len == n + p + 1");

    let mut d = [T::ZERO; WORKSPACE_SIZE];
    for j in 0..=p {
        // SAFETY: k - p + j ∈ [0, k] ⊆ [0, n-1] < cps.len()
        // SAFETY: j ≤ p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
        unsafe { *d.get_unchecked_mut(j) = *cps.get_unchecked(k - p + j) };
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            // SAFETY: k - p + j ∈ [0, n-1] < knots.len();
            //         k + 1 + j - r ≤ k + p ≤ n + p - 1 < knots.len() = n + p + 1.
            // SAFETY: j ≤ p ≤ MAX_DEGREE < WORKSPACE_SIZE; j-1 ≥ r-1 ≥ 0.
            let knot_lo = unsafe { *knots.get_unchecked(k - p + j) };
            let knot_hi = unsafe { *knots.get_unchecked(k + 1 + j - r) };
            let denom = knot_hi - knot_lo;
            let alpha = if denom > T::ZERO {
                (u - knot_lo) / denom
            } else {
                T::ZERO
            };
            // d[j] = (1 - alpha) * d[j-1] + alpha * d[j]
            //      = (d[j] - d[j-1]).mul_add(alpha, d[j-1])
            let dj = unsafe { *d.get_unchecked(j) };
            let djm1 = unsafe { *d.get_unchecked(j - 1) };
            unsafe { *d.get_unchecked_mut(j) = (dj - djm1).mul_add(alpha, djm1) };
        }
    }

    // SAFETY: p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
    unsafe { *d.get_unchecked(p) }
}

/// Evaluate a scalar NURBS at parameter `u`.
/// Hot path. MCU + host. No allocation.
///
/// For non-rational curves: one de Boor walk.
/// For rational curves: two de Boor walks (weighted CPs and weights), then divide.
#[inline]
pub fn eval<T: Float, V: NurbsView<T>>(curve: &V, u: T) -> T {
    debug_assert!((curve.degree() as usize) <= MAX_DEGREE);
    match curve.weights() {
        None => de_boor_inner(curve.control_points(), curve.knots(), curve.degree(), u),
        Some(w) => {
            let numer =
                de_boor_homogeneous(curve.control_points(), w, curve.knots(), curve.degree(), u);
            let denom = de_boor_inner(w, curve.knots(), curve.degree(), u);
            let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
            debug_assert!(denom.abs() > floor);
            numer / denom.max(floor)
        }
    }
}

/// de Boor over `weighted_cps[i] = cps[i] * weights[i]`, computed in a single
/// pass without allocating a weighted-cps vector.
///
/// Reference: Piegl & Tiller "The NURBS Book" §4.4 (rational evaluation via
/// homogeneous coordinates). The weighting is applied at the de Boor
/// initialization step; the recurrence is identical to `de_boor_inner`.
///
/// # Index-safety invariant
///
/// Same as `de_boor_inner`: `find_knot_span` returns `k ∈ [p, n-1]`, so
/// `k - p + j ∈ [0, n-1]` for `j ∈ 0..=p`, and `k + 1 + j - r ≤ k + p ≤ n + p - 1`.
/// All `cps`, `weights`, and `knots` accesses below are in-bounds by this
/// invariant; `get_unchecked` eliminates MCU-build panic paths.
#[inline]
pub(crate) fn de_boor_homogeneous<T: Float>(
    cps: &[T],
    weights: &[T],
    knots: &[T],
    degree: u8,
    u: T,
) -> T {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(cps.len() == weights.len());
    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    // SAFETY: k ∈ [p, n-1] from find_knot_span, so k-p+j ∈ [0, n-1] for j ∈ 0..=p.
    debug_assert!(k >= p && k < n, "find_knot_span invariant: k ∈ [p, n-1]");
    debug_assert!(knots.len() == n + p + 1, "knots len == n + p + 1");

    let mut d = [T::ZERO; WORKSPACE_SIZE];
    for j in 0..=p {
        // SAFETY: k - p + j ∈ [0, n-1] < cps.len() == weights.len()
        // SAFETY: j ≤ p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
        unsafe {
            *d.get_unchecked_mut(j) =
                *cps.get_unchecked(k - p + j) * *weights.get_unchecked(k - p + j);
        }
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            // SAFETY: same knots-index invariant as de_boor_inner.
            // SAFETY: j ≤ p ≤ MAX_DEGREE < WORKSPACE_SIZE; j-1 ≥ r-1 ≥ 0.
            let knot_lo = unsafe { *knots.get_unchecked(k - p + j) };
            let knot_hi = unsafe { *knots.get_unchecked(k + 1 + j - r) };
            let denom = knot_hi - knot_lo;
            let alpha = if denom > T::ZERO {
                (u - knot_lo) / denom
            } else {
                T::ZERO
            };
            let dj = unsafe { *d.get_unchecked(j) };
            let djm1 = unsafe { *d.get_unchecked(j - 1) };
            unsafe { *d.get_unchecked_mut(j) = (dj - djm1).mul_add(alpha, djm1) };
        }
    }

    // SAFETY: p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
    unsafe { *d.get_unchecked(p) }
}

/// Evaluate a vector NURBS at parameter `u`. Shares knot-span lookup and alpha
/// computation across the N axes — meaningfully cheaper than N independent
/// scalar `eval` calls for shared-knot vector NURBS.
#[inline]
pub fn vector_eval<T: Float, V: VectorNurbsView<T, N>, const N: usize>(curve: &V, u: T) -> [T; N] {
    debug_assert!((curve.degree() as usize) <= MAX_DEGREE);
    let p = curve.degree() as usize;
    let knots = curve.knots();
    let cps = curve.control_points();
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    let has_weights = curve.weights().is_some();

    let mut d_axes: [[T; WORKSPACE_SIZE]; N] = [[T::ZERO; WORKSPACE_SIZE]; N];
    let mut d_w = [T::ZERO; WORKSPACE_SIZE];

    // SAFETY: k ∈ [p, n-1] from find_knot_span (same invariant as de_boor_inner).
    debug_assert!(k >= p && k < n, "find_knot_span invariant: k ∈ [p, n-1]");
    debug_assert!(knots.len() == n + p + 1, "knots len == n + p + 1");

    // Initialize active CPs for this span.
    for j in 0..=p {
        // SAFETY: k - p + j ∈ [0, n-1] < cps.len()
        // SAFETY: j ≤ p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
        let cp = unsafe { cps.get_unchecked(k - p + j) };
        if let Some(w) = curve.weights() {
            // SAFETY: same index, w.len() == cps.len()
            let wj = unsafe { *w.get_unchecked(k - p + j) };
            for axis in 0..N {
                unsafe { *d_axes[axis].get_unchecked_mut(j) = cp[axis] * wj };
            }
            unsafe { *d_w.get_unchecked_mut(j) = wj };
        } else {
            for axis in 0..N {
                unsafe { *d_axes[axis].get_unchecked_mut(j) = cp[axis] };
            }
        }
    }

    // de Boor recurrence — shared alphas across axes.
    for r in 1..=p {
        for j in (r..=p).rev() {
            // SAFETY: same knots-index invariant as de_boor_inner.
            // SAFETY: j ≤ p ≤ MAX_DEGREE < WORKSPACE_SIZE; j-1 ≥ r-1 ≥ 0.
            let knot_lo = unsafe { *knots.get_unchecked(k - p + j) };
            let knot_hi = unsafe { *knots.get_unchecked(k + 1 + j - r) };
            let denom = knot_hi - knot_lo;
            let alpha = if denom > T::ZERO {
                (u - knot_lo) / denom
            } else {
                T::ZERO
            };
            for axis in 0..N {
                let dj = unsafe { *d_axes[axis].get_unchecked(j) };
                let djm1 = unsafe { *d_axes[axis].get_unchecked(j - 1) };
                unsafe { *d_axes[axis].get_unchecked_mut(j) = (dj - djm1).mul_add(alpha, djm1) };
            }
            if has_weights {
                let dj = unsafe { *d_w.get_unchecked(j) };
                let djm1 = unsafe { *d_w.get_unchecked(j - 1) };
                unsafe { *d_w.get_unchecked_mut(j) = (dj - djm1).mul_add(alpha, djm1) };
            }
        }
    }

    let mut result = [T::ZERO; N];
    if has_weights {
        // SAFETY: p ≤ MAX_DEGREE < WORKSPACE_SIZE
        let denom = unsafe { *d_w.get_unchecked(p) };
        let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
        debug_assert!(denom.abs() > floor);
        let denom_clamp = denom.max(floor);
        for axis in 0..N {
            // SAFETY: p ≤ MAX_DEGREE < WORKSPACE_SIZE
            result[axis] = unsafe { *d_axes[axis].get_unchecked(p) } / denom_clamp;
        }
    } else {
        for axis in 0..N {
            // SAFETY: p ≤ MAX_DEGREE < WORKSPACE_SIZE
            result[axis] = unsafe { *d_axes[axis].get_unchecked(p) };
        }
    }
    result
}

/// Evaluate `P(u)` and `dP/du` simultaneously from raw cps + knots slices,
/// running the de Boor recurrence and its derivative recurrence in parallel.
/// Saves a second de Boor pyramid pass vs calling `eval_polynomial` and
/// `eval_derivative` separately.
///
/// Per-pass cost is `O(p^2)` arithmetic ops; this function does `~2x` the
/// work of `eval_polynomial` alone, vs `~3x` if you call eval and derivative
/// separately (eval pays for the lowered curve's de Boor, plus its own
/// init / `find_knot_span`). On the H7 at degree 9 / 82 cps / 92 knots, the
/// combined form is materially cheaper than the sum of the separate calls.
///
/// MCU hot path: callers are responsible for ensuring slice shapes satisfy
/// `knots.len() == cps.len() + degree + 1` and that the curve was validated
/// upstream (e.g. `CurvePool` on segment load).
///
/// Polynomial (non-rational) only — for weighted curves go via separate
/// `eval` + appropriate quotient-rule construction.
///
/// Reference: differentiate the de Boor recurrence
/// `d^(r)_j = (1 - α) * d^(r-1)_{j-1} + α * d^(r-1)_j` w.r.t. `u`:
///   `∂_u d^(r)_j = (1 - α) * ∂_u d^(r-1)_{j-1} + α * ∂_u d^(r-1)_j
///                + (d^(r-1)_j - d^(r-1)_{j-1}) / denom`.
/// Initial `∂_u d^(0)_j = 0` since the original cps don't depend on u.
/// After full recurrence, `dd[p] = P'(u)`.
#[inline]
pub fn eval_polynomial_with_derivative<T: Float>(
    cps: &[T],
    knots: &[T],
    degree: u8,
    u: T,
) -> (T, T) {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(knots.len() == cps.len() + (degree as usize) + 1);

    if degree == 0 {
        // Step function: derivative is 0 everywhere, value is the active cp.
        let p = 0;
        let n = cps.len();
        let k = find_knot_span(knots, p, n, u);
        // SAFETY: find_knot_span returns k ∈ [0, n-1] for p=0.
        debug_assert!(k < n);
        return (unsafe { *cps.get_unchecked(k) }, T::ZERO);
    }

    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    // SAFETY: k ∈ [p, n-1] from find_knot_span; k-p+j ∈ [0,n-1] for j ∈ 0..=p;
    //         k+1+j-r ≤ k+p ≤ n+p-1 < knots.len() = n+p+1.
    debug_assert!(k >= p && k < n, "find_knot_span invariant: k ∈ [p, n-1]");
    debug_assert!(knots.len() == n + p + 1, "knots len == n + p + 1");

    let mut d = [T::ZERO; WORKSPACE_SIZE];
    let mut dd = [T::ZERO; WORKSPACE_SIZE];
    for j in 0..=p {
        // SAFETY: k - p + j ∈ [0, n-1] < cps.len()
        // SAFETY: j ≤ p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
        unsafe { *d.get_unchecked_mut(j) = *cps.get_unchecked(k - p + j) };
        // dd[j] = 0 — original cps don't depend on u, already in default.
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            // SAFETY: same knots-index invariant as de_boor_inner.
            // SAFETY: j ≤ p ≤ MAX_DEGREE < WORKSPACE_SIZE; j-1 ≥ r-1 ≥ 0.
            let lo = unsafe { *knots.get_unchecked(k - p + j) };
            let hi = unsafe { *knots.get_unchecked(k + 1 + j - r) };
            let denom = hi - lo;
            // Save old d[j-1] / d[j] / dd[j-1] / dd[j] before any writes.
            // (Reverse-j iteration means d[j-1] hasn't been touched at this r.)
            let old_d_jm1 = unsafe { *d.get_unchecked(j - 1) };
            let old_d_j = unsafe { *d.get_unchecked(j) };
            let old_dd_jm1 = unsafe { *dd.get_unchecked(j - 1) };
            let old_dd_j = unsafe { *dd.get_unchecked(j) };
            if denom > T::ZERO {
                let inv_denom = T::ONE / denom;
                let alpha = (u - lo) * inv_denom;
                let one_minus_alpha = T::ONE - alpha;
                // dd[j] = (1-α) * dd[j-1] + α * dd[j]
                //       + (d[j] - d[j-1]) / denom
                unsafe {
                    *dd.get_unchecked_mut(j) = one_minus_alpha * old_dd_jm1
                        + alpha * old_dd_j
                        + (old_d_j - old_d_jm1) * inv_denom;
                    *d.get_unchecked_mut(j) = (old_d_j - old_d_jm1).mul_add(alpha, old_d_jm1);
                }
            } else {
                // Degenerate knot interval: alpha undefined, freeze d[j] to
                // d[j-1] and dd[j] to dd[j-1] (consistent with the
                // alpha=0 fallback in `de_boor_inner`).
                unsafe {
                    *d.get_unchecked_mut(j) = old_d_jm1;
                    *dd.get_unchecked_mut(j) = old_dd_jm1;
                }
            }
        }
    }

    // SAFETY: p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
    unsafe { (*d.get_unchecked(p), *dd.get_unchecked(p)) }
}

/// Knot-span lookup variant that takes f32 knots but an f64 query parameter.
/// Used by `eval_polynomial_f32_with_pos_vel_accel_f64` to drive the de Boor
/// recurrence in f64 over f32-storage cps/knots without an intermediate
/// per-knot widening pass.
// Same index-safety proof as the MCU `find_knot_span` copy:
//   knots[n]     : n < n+p+1 (p is usize, n >= 1)
//   knots[p]     : p < n+p+1 (n >= 1)
//   knots[mid]   : mid ∈ [low,high] ⊆ [p,n] < n+p+1
//   knots[mid+1] : mid+1 ≤ n+1 ≤ n+p+1 (p >= 0)
#[inline]
fn find_knot_span_f32_with_f64_u(knots: &[f32], p: usize, n: usize, u: f64) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    debug_assert!(n >= 1);
    // SAFETY: n < n+p+1 = knots.len()
    if u >= f64::from(unsafe { *knots.get_unchecked(n) }) {
        return n - 1;
    }
    // SAFETY: p < n+p+1 = knots.len()
    if u <= f64::from(unsafe { *knots.get_unchecked(p) }) {
        return p;
    }
    let mut low = p;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while {
        // SAFETY: mid ∈ [low,high] ⊆ [p,n] < n+p+1; mid+1 ≤ n+1 ≤ n+p+1.
        let km = f64::from(unsafe { *knots.get_unchecked(mid) });
        let km1 = f64::from(unsafe { *knots.get_unchecked(mid + 1) });
        u < km || u >= km1
    } {
        let km = f64::from(unsafe { *knots.get_unchecked(mid) });
        if u < km {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }
    mid
}

/// Same recurrence as the f32→f64 de Boor evaluator with first
/// derivative, but also tracks the second derivative. The position
/// (`d`) and first derivative (`dd`) follow the standard de Boor
/// recurrence; we add a parallel `ddd` array whose update rule is the
/// difference-of-`dd` recurrence — algebraically the second derivative
/// of the same polynomial.
///
/// Cost over the pos+vel variant: one extra triple of f64 ops per
/// inner iteration. Workspace stays bounded by `WORKSPACE_SIZE` (~168 B
/// each × 3 = ~504 B stack).
///
/// Used by `compute_next_step_time` (in the runtime crate) to obtain
/// the second derivative needed for the degree-aware Newton seed that
/// handles `v(0) = 0` cold-start segments — spec
/// `docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md` §3.6.
#[inline]
pub fn eval_polynomial_f32_with_pos_vel_accel_f64(
    cps: &[f32],
    knots: &[f32],
    degree: u8,
    u: f32,
) -> (f64, f64, f64) {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(knots.len() == cps.len() + (degree as usize) + 1);

    let u_f64 = f64::from(u);
    let p = usize::from(degree);
    let n = cps.len();

    if degree == 0 {
        // Step function: position is the active cp, derivatives are zero.
        let k = find_knot_span_f32_with_f64_u(knots, p, n, u_f64);
        // SAFETY: find_knot_span returns k ∈ [0, n-1] for p=0.
        debug_assert!(k < n);
        return (f64::from(unsafe { *cps.get_unchecked(k) }), 0.0, 0.0);
    }
    if degree == 1 {
        // Linear: analytic evaluator. Second derivative is identically zero
        // on each span (the curve is piecewise-linear in u).
        let k = find_knot_span_f32_with_f64_u(knots, p, n, u_f64);
        // SAFETY: k ∈ [1, n-1] for p=1; k-1 ∈ [0, n-2]; k+1 ≤ n < knots.len().
        debug_assert!(k >= 1 && k < n);
        let a = f64::from(unsafe { *cps.get_unchecked(k - 1) });
        let b = f64::from(unsafe { *cps.get_unchecked(k) });
        let knot_lo = f64::from(unsafe { *knots.get_unchecked(k) });
        let knot_hi = f64::from(unsafe { *knots.get_unchecked(k + 1) });
        let denom = knot_hi - knot_lo;
        if denom <= 0.0 {
            return (a, 0.0, 0.0);
        }
        let alpha = (u_f64 - knot_lo) / denom;
        let pos = a + (b - a) * alpha;
        let vel = (b - a) / denom;
        return (pos, vel, 0.0);
    }

    let k = find_knot_span_f32_with_f64_u(knots, p, n, u_f64);

    // SAFETY: k ∈ [p, n-1] from find_knot_span; k-p+j ∈ [0,n-1] for j ∈ 0..=p;
    //         k+1+j-r ≤ k+p ≤ n+p-1 < knots.len() = n+p+1.
    debug_assert!(k >= p && k < n, "find_knot_span invariant");
    debug_assert!(knots.len() == n + p + 1, "knots len == n + p + 1");

    let mut d = [0.0_f64; WORKSPACE_SIZE];
    let mut dd = [0.0_f64; WORKSPACE_SIZE];
    let mut ddd = [0.0_f64; WORKSPACE_SIZE];
    for j in 0..=p {
        // SAFETY: k - p + j ∈ [0, n-1] < cps.len()
        // SAFETY: j ≤ p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
        unsafe { *d.get_unchecked_mut(j) = f64::from(*cps.get_unchecked(k - p + j)) };
        // dd[j] = 0, ddd[j] = 0 — initial cps don't depend on u.
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            // SAFETY: same knots-index invariant as de_boor_inner.
            // SAFETY: j ≤ p ≤ MAX_DEGREE < WORKSPACE_SIZE; j-1 ≥ r-1 ≥ 0.
            let lo = f64::from(unsafe { *knots.get_unchecked(k - p + j) });
            let hi = f64::from(unsafe { *knots.get_unchecked(k + 1 + j - r) });
            let denom = hi - lo;
            let old_d_jm1 = unsafe { *d.get_unchecked(j - 1) };
            let old_d_j = unsafe { *d.get_unchecked(j) };
            let old_dd_jm1 = unsafe { *dd.get_unchecked(j - 1) };
            let old_dd_j = unsafe { *dd.get_unchecked(j) };
            let old_ddd_jm1 = unsafe { *ddd.get_unchecked(j - 1) };
            let old_ddd_j = unsafe { *ddd.get_unchecked(j) };
            if denom > 0.0_f64 {
                let inv_denom = 1.0_f64 / denom;
                let alpha = (u_f64 - lo) * inv_denom;
                let one_minus_alpha = 1.0_f64 - alpha;
                unsafe {
                    *ddd.get_unchecked_mut(j) = one_minus_alpha * old_ddd_jm1
                        + alpha * old_ddd_j
                        + 2.0 * (old_dd_j - old_dd_jm1) * inv_denom;
                    *dd.get_unchecked_mut(j) = one_minus_alpha * old_dd_jm1
                        + alpha * old_dd_j
                        + (old_d_j - old_d_jm1) * inv_denom;
                    *d.get_unchecked_mut(j) = (old_d_j - old_d_jm1) * alpha + old_d_jm1;
                }
            } else {
                unsafe {
                    *d.get_unchecked_mut(j) = old_d_jm1;
                    *dd.get_unchecked_mut(j) = old_dd_jm1;
                    *ddd.get_unchecked_mut(j) = old_ddd_jm1;
                }
            }
        }
    }

    // SAFETY: p ≤ MAX_DEGREE = WORKSPACE_SIZE - 1 < WORKSPACE_SIZE
    unsafe {
        (
            *d.get_unchecked(p),
            *dd.get_unchecked(p),
            *ddd.get_unchecked(p),
        )
    }
}

/// Evaluate a scalar B-spline NURBS at `u` directly from raw cps + knots
/// slices, without going through `ScalarNurbsRef::try_new` (which re-runs the
/// full O(n) NURBS-invariant validation on every call). MCU hot path: callers
/// are responsible for ensuring slice shapes satisfy
/// `knots.len() == cps.len() + degree + 1` and that the curve was validated
/// upstream (e.g. `CurvePool` on segment load).
///
/// Polynomial (non-rational) only — for weighted curves go through `eval`.
#[inline]
pub fn eval_polynomial<T: Float>(cps: &[T], knots: &[T], degree: u8, u: T) -> T {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(knots.len() == cps.len() + (degree as usize) + 1);
    de_boor_inner(cps, knots, degree, u)
}

/// Evaluate `dC/du` for a scalar B-spline NURBS at parameter `u`, without
/// materializing the degree-lowered curve. Computes only the de Boor window
/// of derivative control points (`O(p)` of them) and runs one de Boor walk
/// on a `[T; WORKSPACE_SIZE]` stack scratch.
///
/// MCU hot path: `scalar_derivative_eval` in the runtime crate calls this
/// per-axis at every TIM5 fire (40 kHz × 3 axes). The previous form
/// allocated `[T; MAX_CONTROL_POINTS]` + `[T; MAX_KNOT_VECTOR_LEN]` stack
/// arrays per call (~14.7 KB at the H7 sizing) and got memset-zero'd on
/// every entry; this windowed form keeps stack usage to a couple hundred
/// bytes.
///
/// Polynomial (non-rational) NURBS only — for weighted curves, project to
/// homogeneous coordinates upstream. `degree` must be ≥ 1 (returns
/// `T::ZERO` for `degree == 0`).
///
/// Reference: Piegl & Tiller "The NURBS Book" eq. 3.7 (derivative cps),
/// Algorithm A4.1 (de Boor) on the lowered knot vector.
#[inline]
pub fn eval_derivative<T: Float>(cps: &[T], knots: &[T], degree: u8, u: T) -> T {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    if degree == 0 {
        return T::ZERO;
    }
    let p = degree as usize;
    let n = cps.len();
    if n < 2 || knots.len() < n + p + 1 {
        return T::ZERO;
    }
    let new_p = p - 1;
    let new_n = n - 1;
    // Lowered knot vector drops first and last entries of original.
    // Length = (n + p + 1) - 2 = n + p - 1 = new_n + new_p + 1, the
    // shape `find_knot_span` expects.
    let lowered_knots = &knots[1..n + p];

    let k = find_knot_span(lowered_knots, new_p, new_n, u);

    // SAFETY: k ∈ [new_p, new_n-1] from find_knot_span;
    //         i = k - new_p + j ∈ [0, k] ⊆ [0, new_n-1] = [0, n-2].
    //         cps[i+1] has i+1 ≤ new_n-1+1 = n-1 < cps.len().
    //         knots[i+p+1] has i+p+1 ≤ (n-2)+p+1 = n+p-1 < knots.len() = n+p+1.
    //         lowered_knots has length n+p-1; k+1+j-r ≤ k+new_p ≤ n-2+p-1 = n+p-3 < n+p-1.
    debug_assert!(
        k >= new_p && k < new_n,
        "find_knot_span invariant on lowered knots"
    );

    // Initialize de Boor scratch from the Q-window only. d[j] (j ∈ 0..=new_p)
    // corresponds to derivative cp index i = k - new_p + j in the lowered
    // curve, which maps to original cp index i = k - new_p + j (same offset
    // because Q_i is defined for i = 0..new_n in the original cp space).
    let mut d = [T::ZERO; WORKSPACE_SIZE];
    let p_t = T::from_f64(f64::from(degree));
    for j in 0..=new_p {
        let i = k - new_p + j;
        // SAFETY: i ∈ [0, n-2]; i+1 ∈ [1, n-1] < cps.len(); i+p+1 ≤ n+p-1 < knots.len().
        // SAFETY: j ≤ new_p ≤ p-1 ≤ MAX_DEGREE-1 < WORKSPACE_SIZE
        let denom = unsafe { *knots.get_unchecked(i + p + 1) - *knots.get_unchecked(i + 1) };
        unsafe {
            *d.get_unchecked_mut(j) = if denom > T::ZERO {
                // SAFETY: i ∈ [0, n-2]; i+1 ∈ [1, n-1] < cps.len()
                p_t * (*cps.get_unchecked(i + 1) - *cps.get_unchecked(i)) / denom
            } else {
                T::ZERO
            };
        }
    }

    // de Boor recurrence on lowered_knots, identical shape to de_boor_inner.
    for r in 1..=new_p {
        for j in (r..=new_p).rev() {
            // SAFETY: lowered_knots indices are in-bounds by the invariant above.
            // SAFETY: j ≤ new_p < WORKSPACE_SIZE; j-1 ≥ r-1 ≥ 0.
            let knot_lo = unsafe { *lowered_knots.get_unchecked(k - new_p + j) };
            let knot_hi = unsafe { *lowered_knots.get_unchecked(k + 1 + j - r) };
            let denom = knot_hi - knot_lo;
            let alpha = if denom > T::ZERO {
                (u - knot_lo) / denom
            } else {
                T::ZERO
            };
            let dj = unsafe { *d.get_unchecked(j) };
            let djm1 = unsafe { *d.get_unchecked(j - 1) };
            unsafe { *d.get_unchecked_mut(j) = (dj - djm1).mul_add(alpha, djm1) };
        }
    }

    // SAFETY: new_p = p - 1 ≤ MAX_DEGREE - 1 < WORKSPACE_SIZE
    unsafe { *d.get_unchecked(new_p) }
}

/// Compute the parametric derivative `dP/du` as a new owned NURBS via degree
/// lowering. Result has degree `p - 1`, knot vector with the first and last
/// knots dropped, and control points
///   `Q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})`.
///
/// Host-only — allocates new `Vec`s. For weighted (rational) NURBS, the host
/// pre-bake pipeline should project to homogeneous coordinates first; this
/// function handles unweighted (B-spline) NURBS only. Rational derivative is
/// the consumer's responsibility (composed via the quotient rule downstream).
///
/// Reference: Piegl & Tiller "The NURBS Book" eq. 3.7 / Algorithm A3.3.
#[cfg(feature = "host")]
#[must_use]
pub fn derivative<T: Float>(curve: &crate::ScalarNurbs<T>) -> crate::ScalarNurbs<T> {
    let p = curve.degree();
    assert!(p >= 1, "derivative requires degree >= 1");

    let cps = curve.control_points();
    let knots = curve.knots();
    let new_degree = p - 1;
    let new_n = cps.len() - 1;

    let p_t = T::from_f64(f64::from(p));

    let mut new_cps: Vec<T> = Vec::with_capacity(new_n);
    for i in 0..new_n {
        let denom = knots[i + p as usize + 1] - knots[i + 1];
        let q = if denom > T::ZERO {
            p_t * (cps[i + 1] - cps[i]) / denom
        } else {
            T::ZERO
        };
        new_cps.push(q);
    }

    // New knot vector drops the first and last entries.
    let new_knots: Vec<T> = knots[1..knots.len() - 1].to_vec();

    crate::ScalarNurbs::try_new(new_degree, new_knots, new_cps, None)
        .expect("degree-lowered NURBS satisfies invariants by construction")
}

/// Compute the parametric derivative of a vector NURBS as a new owned NURBS.
/// Same algorithm as scalar `derivative` applied per axis; knot vector and
/// degree handled once.
#[cfg(feature = "host")]
#[must_use]
pub fn vector_derivative<T: Float, const N: usize>(
    curve: &crate::VectorNurbs<T, N>,
) -> crate::VectorNurbs<T, N> {
    let p = curve.degree();
    assert!(p >= 1, "derivative requires degree >= 1");

    let cps = curve.control_points();
    let knots = curve.knots();
    let new_degree = p - 1;
    let new_n = cps.len() - 1;
    let p_t = T::from_f64(f64::from(p));

    let mut new_cps: Vec<[T; N]> = Vec::with_capacity(new_n);
    for i in 0..new_n {
        let denom = knots[i + p as usize + 1] - knots[i + 1];
        let mut q = [T::ZERO; N];
        if denom > T::ZERO {
            for axis in 0..N {
                q[axis] = p_t * (cps[i + 1][axis] - cps[i][axis]) / denom;
            }
        }
        new_cps.push(q);
    }

    let new_knots: Vec<T> = knots[1..knots.len() - 1].to_vec();

    crate::VectorNurbs::try_new(new_degree, new_knots, new_cps, None)
        .expect("degree-lowered NURBS satisfies invariants by construction")
}

/// Compute curvature κ(u) of a 3D path NURBS from its precomputed first and
/// second derivative NURBSes:
///   κ = ||r' × r''|| / ||r'||³
/// The cubed denominator is clamped at `MIN_PARAMETRIC_SPEED` to avoid
/// divide-by-zero at cusps; the clamp engages only on pathological input
/// (well-formed G2/G3 and fitter output never trigger it).
///
/// Caller owns `first_deriv` and `second_deriv` — typically cached on the
/// segment, since TOPP-RA queries many u's per segment.
#[cfg(feature = "host")]
pub fn curvature_from_derivs<T: Float, const N: usize>(
    first_deriv: &crate::VectorNurbs<T, N>,
    second_deriv: &crate::VectorNurbs<T, N>,
    u: T,
) -> T {
    let r_prime = vector_eval(&first_deriv.as_view(), u);
    let r_double = vector_eval(&second_deriv.as_view(), u);

    // Cross product magnitude: works for N=3; for N=2 we'd lift to 3D with z=0.
    // We hardcode 3D here per spec — curvature on path is 3D-only.
    assert!(N == 3, "curvature_from_derivs requires N == 3");

    let cx = r_prime[1] * r_double[2] - r_prime[2] * r_double[1];
    let cy = r_prime[2] * r_double[0] - r_prime[0] * r_double[2];
    let cz = r_prime[0] * r_double[1] - r_prime[1] * r_double[0];
    let cross_norm = (cx * cx + cy * cy + cz * cz).sqrt();

    let speed_sq = r_prime[0] * r_prime[0] + r_prime[1] * r_prime[1] + r_prime[2] * r_prime[2];
    let speed = speed_sq.sqrt();
    let speed_cubed = speed * speed * speed;

    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
    cross_norm / speed_cubed.max(floor)
}

#[cfg(test)]
mod tests;
