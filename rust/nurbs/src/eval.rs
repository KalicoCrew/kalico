//! NURBS evaluation: de Boor, vector eval, derivative, curvature.
//! See spec §eval module.

use crate::{Float, MAX_DEGREE, MIN_PARAMETRIC_SPEED, NurbsView, VectorNurbsView, WORKSPACE_SIZE};

// Re-export from knot module for transitional internal use. Eventually
// callers should import directly from `crate::knot::find_knot_span`.
#[cfg(feature = "host")]
pub(crate) use crate::knot::find_knot_span;

// MCU build needs an inline copy since knot module is host-only.
#[cfg(not(feature = "host"))]
#[inline]
pub(crate) fn find_knot_span<T: Float>(knots: &[T], p: usize, n: usize, u: T) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    if u >= knots[n] {
        return n - 1;
    }
    if u <= knots[p] {
        return p;
    }
    let mut low = p;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
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
#[inline]
pub(crate) fn de_boor_inner<T: Float>(cps: &[T], knots: &[T], degree: u8, u: T) -> T {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    let mut d = [T::ZERO; WORKSPACE_SIZE];
    for j in 0..=p {
        d[j] = cps[k - p + j];
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let denom = knots[k + 1 + j - r] - knots[k - p + j];
            let alpha = if denom > T::ZERO {
                (u - knots[k - p + j]) / denom
            } else {
                T::ZERO
            };
            // d[j] = (1 - alpha) * d[j-1] + alpha * d[j]
            //      = (d[j] - d[j-1]).mul_add(alpha, d[j-1])
            d[j] = (d[j] - d[j - 1]).mul_add(alpha, d[j - 1]);
        }
    }

    d[p]
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

    let mut d = [T::ZERO; WORKSPACE_SIZE];
    for j in 0..=p {
        d[j] = cps[k - p + j] * weights[k - p + j];
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let denom = knots[k + 1 + j - r] - knots[k - p + j];
            let alpha = if denom > T::ZERO {
                (u - knots[k - p + j]) / denom
            } else {
                T::ZERO
            };
            d[j] = (d[j] - d[j - 1]).mul_add(alpha, d[j - 1]);
        }
    }

    d[p]
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

    // Initialize active CPs for this span.
    for j in 0..=p {
        let cp = cps[k - p + j];
        if let Some(w) = curve.weights() {
            for axis in 0..N {
                d_axes[axis][j] = cp[axis] * w[k - p + j];
            }
            d_w[j] = w[k - p + j];
        } else {
            for axis in 0..N {
                d_axes[axis][j] = cp[axis];
            }
        }
    }

    // de Boor recurrence — shared alphas across axes.
    for r in 1..=p {
        for j in (r..=p).rev() {
            let denom = knots[k + 1 + j - r] - knots[k - p + j];
            let alpha = if denom > T::ZERO {
                (u - knots[k - p + j]) / denom
            } else {
                T::ZERO
            };
            for axis in 0..N {
                d_axes[axis][j] =
                    (d_axes[axis][j] - d_axes[axis][j - 1]).mul_add(alpha, d_axes[axis][j - 1]);
            }
            if has_weights {
                d_w[j] = (d_w[j] - d_w[j - 1]).mul_add(alpha, d_w[j - 1]);
            }
        }
    }

    let mut result = [T::ZERO; N];
    if has_weights {
        let denom = d_w[p];
        let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
        debug_assert!(denom.abs() > floor);
        let denom_clamp = denom.max(floor);
        for axis in 0..N {
            result[axis] = d_axes[axis][p] / denom_clamp;
        }
    } else {
        for axis in 0..N {
            result[axis] = d_axes[axis][p];
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
/// init / find_knot_span). On the H7 at degree 9 / 82 cps / 92 knots, the
/// combined form is materially cheaper than the sum of the separate calls.
///
/// MCU hot path: callers are responsible for ensuring slice shapes satisfy
/// `knots.len() == cps.len() + degree + 1` and that the curve was validated
/// upstream (e.g. CurvePool on segment load).
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
        return (cps[k], T::ZERO);
    }

    let p = degree as usize;
    let n = cps.len();
    let k = find_knot_span(knots, p, n, u);

    let mut d = [T::ZERO; WORKSPACE_SIZE];
    let mut dd = [T::ZERO; WORKSPACE_SIZE];
    for j in 0..=p {
        d[j] = cps[k - p + j];
        // dd[j] = 0 — original cps don't depend on u, already in default.
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let lo = knots[k - p + j];
            let hi = knots[k + 1 + j - r];
            let denom = hi - lo;
            // Save old d[j-1] / d[j] / dd[j-1] / dd[j] before any writes.
            // (Reverse-j iteration means d[j-1] hasn't been touched at this r.)
            let old_d_jm1 = d[j - 1];
            let old_d_j = d[j];
            let old_dd_jm1 = dd[j - 1];
            let old_dd_j = dd[j];
            if denom > T::ZERO {
                let inv_denom = T::ONE / denom;
                let alpha = (u - lo) * inv_denom;
                let one_minus_alpha = T::ONE - alpha;
                // dd[j] = (1-α) * dd[j-1] + α * dd[j]
                //       + (d[j] - d[j-1]) / denom
                dd[j] = one_minus_alpha * old_dd_jm1
                    + alpha * old_dd_j
                    + (old_d_j - old_d_jm1) * inv_denom;
                d[j] = (old_d_j - old_d_jm1).mul_add(alpha, old_d_jm1);
            } else {
                // Degenerate knot interval: alpha undefined, freeze d[j] to
                // d[j-1] and dd[j] to dd[j-1] (consistent with the
                // alpha=0 fallback in `de_boor_inner`).
                d[j] = old_d_jm1;
                dd[j] = old_dd_jm1;
            }
        }
    }

    (d[p], dd[p])
}

/// Knot-span lookup variant that takes f32 knots but an f64 query parameter.
/// Used by `eval_polynomial_f32_with_pos_vel_accel_f64` to drive the de Boor
/// recurrence in f64 over f32-storage cps/knots without an intermediate
/// per-knot widening pass.
#[inline]
fn find_knot_span_f32_with_f64_u(knots: &[f32], p: usize, n: usize, u: f64) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    if u >= knots[n] as f64 {
        return n - 1;
    }
    if u <= knots[p] as f64 {
        return p;
    }
    let mut low = p;
    let mut high = n;
    let mut mid = (low + high) / 2;
    while u < knots[mid] as f64 || u >= knots[mid + 1] as f64 {
        if u < knots[mid] as f64 {
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

    let u_f64 = u as f64;
    let p = usize::from(degree);
    let n = cps.len();

    if degree == 0 {
        // Step function: position is the active cp, derivatives are zero.
        let k = find_knot_span_f32_with_f64_u(knots, p, n, u_f64);
        return (cps[k] as f64, 0.0, 0.0);
    }
    if degree == 1 {
        // Linear: analytic evaluator. Second derivative is identically zero
        // on each span (the curve is piecewise-linear in u).
        let k = find_knot_span_f32_with_f64_u(knots, p, n, u_f64);
        let a = cps[k - 1] as f64;
        let b = cps[k] as f64;
        let knot_lo = knots[k] as f64;
        let knot_hi = knots[k + 1] as f64;
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

    let mut d = [0.0_f64; WORKSPACE_SIZE];
    let mut dd = [0.0_f64; WORKSPACE_SIZE];
    let mut ddd = [0.0_f64; WORKSPACE_SIZE];
    for j in 0..=p {
        d[j] = cps[k - p + j] as f64;
        // dd[j] = 0, ddd[j] = 0 — initial cps don't depend on u.
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let lo = knots[k - p + j] as f64;
            let hi = knots[k + 1 + j - r] as f64;
            let denom = hi - lo;
            let old_d_jm1 = d[j - 1];
            let old_d_j = d[j];
            let old_dd_jm1 = dd[j - 1];
            let old_dd_j = dd[j];
            let old_ddd_jm1 = ddd[j - 1];
            let old_ddd_j = ddd[j];
            if denom > 0.0_f64 {
                let inv_denom = 1.0_f64 / denom;
                let alpha = (u_f64 - lo) * inv_denom;
                let one_minus_alpha = 1.0_f64 - alpha;
                ddd[j] = one_minus_alpha * old_ddd_jm1
                    + alpha * old_ddd_j
                    + 2.0 * (old_dd_j - old_dd_jm1) * inv_denom;
                dd[j] = one_minus_alpha * old_dd_jm1
                    + alpha * old_dd_j
                    + (old_d_j - old_d_jm1) * inv_denom;
                d[j] = (old_d_j - old_d_jm1) * alpha + old_d_jm1;
            } else {
                d[j] = old_d_jm1;
                dd[j] = old_dd_jm1;
                ddd[j] = old_ddd_jm1;
            }
        }
    }

    (d[p], dd[p], ddd[p])
}

/// Evaluate a scalar B-spline NURBS at `u` directly from raw cps + knots
/// slices, without going through `ScalarNurbsRef::try_new` (which re-runs the
/// full O(n) NURBS-invariant validation on every call). MCU hot path: callers
/// are responsible for ensuring slice shapes satisfy `knots.len() == cps.len()
/// + degree + 1` and that the curve was validated upstream (e.g. CurvePool
/// on segment load).
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

    // Initialize de Boor scratch from the Q-window only. d[j] (j ∈ 0..=new_p)
    // corresponds to derivative cp index i = k - new_p + j in the lowered
    // curve, which maps to original cp index i = k - new_p + j (same offset
    // because Q_i is defined for i = 0..new_n in the original cp space).
    let mut d = [T::ZERO; WORKSPACE_SIZE];
    let p_t = T::from_f64(f64::from(degree));
    for j in 0..=new_p {
        let i = k - new_p + j;
        let denom = knots[i + p + 1] - knots[i + 1];
        d[j] = if denom > T::ZERO {
            p_t * (cps[i + 1] - cps[i]) / denom
        } else {
            T::ZERO
        };
    }

    // de Boor recurrence on lowered_knots, identical shape to de_boor_inner.
    for r in 1..=new_p {
        for j in (r..=new_p).rev() {
            let denom = lowered_knots[k + 1 + j - r] - lowered_knots[k - new_p + j];
            let alpha = if denom > T::ZERO {
                (u - lowered_knots[k - new_p + j]) / denom
            } else {
                T::ZERO
            };
            d[j] = (d[j] - d[j - 1]).mul_add(alpha, d[j - 1]);
        }
    }

    d[new_p]
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
mod tests {
    use super::*;

    fn linear_curve_f64() -> crate::ScalarNurbs<f64> {
        crate::ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap()
    }

    fn quadratic_curve_f64() -> crate::ScalarNurbs<f64> {
        // Bezier-ish: degree 2, knots {0,0,0,1,1,1}, cps {0, 0.5, 1}.
        crate::ScalarNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.5, 1.0],
            None,
        )
        .unwrap()
    }

    #[test]
    fn find_knot_span_endpoints() {
        let knots = [0.0, 0.0, 1.0, 1.0];
        // n = control_point_count = 2, p = 1
        // u=0 → first span (clamped at start)
        assert_eq!(find_knot_span(&knots, 1, 2, 0.0), 1);
        // u=1 → last span
        assert_eq!(find_knot_span(&knots, 1, 2, 1.0), 1);
    }

    #[test]
    fn find_knot_span_midpoint() {
        let knots = [0.0, 0.0, 0.5, 1.0, 1.0];
        // n = 3, p = 1
        // u=0.25 → span index 1 (between knots[1]=0 and knots[2]=0.5)
        assert_eq!(find_knot_span(&knots, 1, 3, 0.25), 1);
        // u=0.75 → span index 2 (between knots[2]=0.5 and knots[3]=1.0)
        assert_eq!(find_knot_span(&knots, 1, 3, 0.75), 2);
    }

    #[test]
    fn eval_linear_at_endpoints_returns_endpoint_cps() {
        let curve = linear_curve_f64();
        let v = curve.as_view();
        assert!((eval(&v, 0.0_f64) - 0.0).abs() < 1e-12);
        assert!((eval(&v, 1.0_f64) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn eval_linear_at_midpoint_returns_average() {
        let curve = linear_curve_f64();
        let v = curve.as_view();
        assert!((eval(&v, 0.5_f64) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn eval_quadratic_at_endpoints_returns_first_last_cp() {
        let curve = quadratic_curve_f64();
        let v = curve.as_view();
        assert!((eval(&v, 0.0_f64) - 0.0).abs() < 1e-12);
        assert!((eval(&v, 1.0_f64) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn eval_quadratic_at_midpoint_matches_bernstein() {
        // For the bezier-shaped quadratic with cps [0, 0.5, 1] at u=0.5:
        // B_0,2(0.5) * 0 + B_1,2(0.5) * 0.5 + B_2,2(0.5) * 1
        // = 0.25 * 0 + 0.5 * 0.5 + 0.25 * 1 = 0.5
        let curve = quadratic_curve_f64();
        let v = curve.as_view();
        assert!((eval(&v, 0.5_f64) - 0.5).abs() < 1e-12);
    }

    fn rational_quadratic_arc() -> crate::ScalarNurbs<f64> {
        // Rational quadratic: 90° arc from (1,0) to (0,1) projected to scalar X.
        // We model the X channel: cps = [1, 1, 0], weights = [1, sqrt(2)/2, 1].
        // At u=0: X=1; at u=1: X=0; at u=0.5: ~0.707 (approximately cos(45°)).
        crate::ScalarNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![1.0, 1.0, 0.0],
            Some(vec![1.0, std::f64::consts::SQRT_2 / 2.0, 1.0]),
        )
        .unwrap()
    }

    #[test]
    fn eval_rational_at_endpoints() {
        let curve = rational_quadratic_arc();
        let v = curve.as_view();
        assert!((eval(&v, 0.0_f64) - 1.0).abs() < 1e-12);
        assert!((eval(&v, 1.0_f64) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn eval_rational_at_midpoint() {
        let curve = rational_quadratic_arc();
        let v = curve.as_view();
        // Standard rational quadratic formula with symmetric weights yields cos(45°) ≈ 0.7071
        let mid = eval(&v, 0.5_f64);
        let expected = (std::f64::consts::SQRT_2 / 2.0_f64).powi(2)
            / ((std::f64::consts::SQRT_2 / 2.0_f64).powi(2) + 0.5_f64);
        // simpler check: result lies in (0.69, 0.72) for this specific arc
        assert!(mid > 0.69 && mid < 0.72, "got {mid}, expected ~{expected}");
    }

    fn linear_3d_curve_f64() -> crate::VectorNurbs<f64, 3> {
        crate::VectorNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
            None,
        )
        .unwrap()
    }

    #[test]
    fn vector_eval_linear_endpoints() {
        let curve = linear_3d_curve_f64();
        let v = curve.as_view();
        let p0 = vector_eval(&v, 0.0_f64);
        assert!((p0[0] - 0.0).abs() < 1e-12);
        assert!((p0[1] - 0.0).abs() < 1e-12);
        assert!((p0[2] - 0.0).abs() < 1e-12);
        let p1 = vector_eval(&v, 1.0_f64);
        assert!((p1[0] - 1.0).abs() < 1e-12);
        assert!((p1[1] - 2.0).abs() < 1e-12);
        assert!((p1[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn vector_eval_matches_per_axis_scalar() {
        let curve = linear_3d_curve_f64();
        let v = curve.as_view();
        let result = vector_eval(&v, 0.3_f64);

        // Reconstruct each axis as a scalar curve and compare.
        for axis in 0..3 {
            let cps_axis: Vec<f64> = v.control_points().iter().map(|cp| cp[axis]).collect();
            let scalar =
                crate::ScalarNurbs::try_new(v.degree(), v.knots().to_vec(), cps_axis, None)
                    .unwrap();
            let expected = eval(&scalar.as_view(), 0.3_f64);
            assert!(
                (result[axis] - expected).abs() < 1e-12,
                "axis {axis}: got {}, expected {}",
                result[axis],
                expected
            );
        }
    }

    #[cfg(feature = "host")]
    #[test]
    fn derivative_of_linear_is_constant() {
        // Derivative of a linear NURBS is a degree-0 NURBS with control points
        // equal to (cp[1] - cp[0]) / (u_max - u_min) = 1.0 for our linear curve.
        let curve = linear_curve_f64();
        let d = derivative(&curve);
        assert_eq!(d.degree(), 0);
        // Eval at any u should give 1.0
        assert!((eval(&d.as_view(), 0.5_f64) - 1.0).abs() < 1e-12);
    }

    #[cfg(feature = "host")]
    #[test]
    fn derivative_of_quadratic_at_midpoint_matches_central_difference() {
        let curve = quadratic_curve_f64();
        let d = derivative(&curve);
        let v = d.as_view();
        let h = 1e-6_f64;
        let expected =
            (eval(&curve.as_view(), 0.5 + h) - eval(&curve.as_view(), 0.5 - h)) / (2.0 * h);
        let actual = eval(&v, 0.5);
        assert!(
            (actual - expected).abs() < 1e-6,
            "got {actual}, expected {expected}"
        );
    }

    #[test]
    fn eval_polynomial_with_derivative_matches_separate_calls_quadratic() {
        let curve = quadratic_curve_f64();
        for u_pct in 0..=100 {
            let u = u_pct as f64 / 100.0;
            let (v_combined, d_combined) = eval_polynomial_with_derivative(
                curve.control_points(),
                curve.knots(),
                curve.degree(),
                u,
            );
            let v_sep = eval_polynomial(
                curve.control_points(),
                curve.knots(),
                curve.degree(),
                u,
            );
            let d_sep = eval_derivative(
                curve.control_points(),
                curve.knots(),
                curve.degree(),
                u,
            );
            assert!(
                (v_combined - v_sep).abs() < 1e-12,
                "u={u}: combined value {v_combined} vs separate {v_sep}"
            );
            assert!(
                (d_combined - d_sep).abs() < 1e-12,
                "u={u}: combined deriv {d_combined} vs separate {d_sep}"
            );
        }
    }

    #[test]
    fn eval_polynomial_with_derivative_matches_separate_calls_cubic() {
        // Non-uniform 5-cp cubic, exercises a non-trivial knot span.
        let curve = crate::ScalarNurbs::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.5, 4.0, 5.0],
            None,
        )
        .unwrap();
        for u_pct in 0..=100 {
            let u = u_pct as f64 / 100.0;
            let (v_combined, d_combined) = eval_polynomial_with_derivative(
                curve.control_points(),
                curve.knots(),
                curve.degree(),
                u,
            );
            let v_sep = eval_polynomial(
                curve.control_points(),
                curve.knots(),
                curve.degree(),
                u,
            );
            let d_sep = eval_derivative(
                curve.control_points(),
                curve.knots(),
                curve.degree(),
                u,
            );
            assert!(
                (v_combined - v_sep).abs() < 1e-12,
                "u={u}: combined value {v_combined} vs separate {v_sep}"
            );
            assert!(
                (d_combined - d_sep).abs() < 1e-12,
                "u={u}: combined deriv {d_combined} vs separate {d_sep}"
            );
        }
    }

    #[cfg(feature = "host")]
    #[test]
    fn eval_derivative_matches_materialized_derivative_quadratic() {
        // The MCU windowed `eval_derivative` must give the same value as
        // building the lowered curve via `derivative` and evaluating it.
        let curve = quadratic_curve_f64();
        let lowered = derivative(&curve);
        for u_pct in 0..=100 {
            let u = u_pct as f64 / 100.0;
            let materialized = eval(&lowered.as_view(), u);
            let windowed =
                eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
            assert!(
                (materialized - windowed).abs() < 1e-12,
                "u={u}: materialized={materialized}, windowed={windowed}"
            );
        }
    }

    #[cfg(feature = "host")]
    #[test]
    fn eval_derivative_cubic_matches_materialized() {
        // Cubic with a non-uniform knot vector and 5 cps — exercises the
        // de Boor walk on a non-trivial knot span.
        let curve = crate::ScalarNurbs::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0],
            vec![0.0, 1.0, 2.5, 4.0, 5.0],
            None,
        )
        .unwrap();
        let lowered = derivative(&curve);
        for u_pct in 0..=100 {
            let u = u_pct as f64 / 100.0;
            let materialized = eval(&lowered.as_view(), u);
            let windowed =
                eval_derivative(curve.control_points(), curve.knots(), curve.degree(), u);
            assert!(
                (materialized - windowed).abs() < 1e-12,
                "u={u}: materialized={materialized}, windowed={windowed}"
            );
        }
    }

    #[cfg(feature = "host")]
    #[test]
    fn vector_derivative_matches_per_axis_scalar() {
        let curve = linear_3d_curve_f64();
        let d = vector_derivative(&curve);
        assert_eq!(d.degree(), 0);
        let v = d.as_view();
        let result = vector_eval(&v, 0.3_f64);

        for axis in 0..3 {
            let cps_axis: Vec<f64> = curve.control_points().iter().map(|cp| cp[axis]).collect();
            let scalar =
                crate::ScalarNurbs::try_new(curve.degree(), curve.knots().to_vec(), cps_axis, None)
                    .unwrap();
            let scalar_d = derivative(&scalar);
            let expected = eval(&scalar_d.as_view(), 0.3_f64);
            assert!((result[axis] - expected).abs() < 1e-12);
        }
    }

    #[cfg(feature = "host")]
    #[test]
    fn curvature_of_straight_line_is_zero() {
        // Second derivative of a linear curve is zero — but degree-lowering can't
        // produce a degree -1 curve. We need a degree-2 curve to take two derivatives.
        // Use a parabolic 3D curve instead.
        let parabolic = crate::VectorNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
            None,
        )
        .unwrap();
        let first = vector_derivative(&parabolic);
        let second = vector_derivative(&first);
        // The path is straight along X — curvature is 0 everywhere.
        let k = curvature_from_derivs(&first, &second, 0.5_f64);
        assert!(k.abs() < 1e-10, "got {k}");
    }

    #[test]
    fn pos_vel_accel_on_quadratic_polynomial() {
        // f(u) = u² on u ∈ [0,1] as degree-2 Bézier with cps = [0, 0, 1]
        // (knots [0,0,0,1,1,1]).
        // Verify: f(0.5)=0.25, f'(0.5)=1.0, f''(0.5)=2.0.
        let cps = vec![0.0_f32, 0.0, 1.0];
        let knots = vec![0.0_f32, 0.0, 0.0, 1.0, 1.0, 1.0];
        let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 2, 0.5);
        assert!((p - 0.25).abs() < 1e-9, "pos={}", p);
        assert!((v - 1.0_f64).abs() < 1e-9, "vel={}", v);
        assert!((a - 2.0_f64).abs() < 1e-9, "accel={}", a);
    }

    #[test]
    fn pos_vel_accel_on_cubic_polynomial() {
        // f(u) = u³ on u ∈ [0,1] as degree-3 Bézier with cps = [0,0,0,1]
        // (knots [0,0,0,0,1,1,1,1]).
        // Verify at u=0.5: f=0.125, f'=0.75, f''=3.0.
        let cps = vec![0.0_f32, 0.0, 0.0, 1.0];
        let knots = vec![0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 3, 0.5);
        assert!((p - 0.125).abs() < 1e-9, "pos={}", p);
        assert!((v - 0.75_f64).abs() < 1e-9, "vel={}", v);
        assert!((a - 3.0_f64).abs() < 1e-9, "accel={}", a);
    }

    #[test]
    fn pos_vel_accel_on_linear_polynomial_returns_zero_accel() {
        // f(u) = u, degree-1 Bézier cps=[0,1], knots=[0,0,1,1].
        // Note: 0.3_f32 widens to ~0.30000001192 in f64, so position tolerance
        // accommodates the f32→f64 round-trip on u (~1.2e-8). Velocity and
        // acceleration are exact (rational arithmetic on exact knots/cps).
        let cps = vec![0.0_f32, 1.0];
        let knots = vec![0.0_f32, 0.0, 1.0, 1.0];
        let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 1, 0.3);
        assert!((p - 0.3).abs() < 1e-6, "pos={}", p);
        assert!((v - 1.0_f64).abs() < 1e-9, "vel={}", v);
        assert!(
            a.abs() < 1e-9,
            "linear curve must have zero second derivative; got {}",
            a
        );
    }

    #[cfg(feature = "host")]
    #[test]
    fn curvature_of_arc_matches_known_value() {
        // Quadratic Bezier approximating a circular arc: cps [(1,0,0),(1,1,0),(0,1,0)].
        // Not a true circle (rational quadratics with weights are exact), but
        // curvature at u=0.5 should be positive and finite.
        let arc = crate::VectorNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            None,
        )
        .unwrap();
        let first = vector_derivative(&arc);
        let second = vector_derivative(&first);
        let k = curvature_from_derivs(&first, &second, 0.5_f64);
        assert!(k > 0.0, "expected positive curvature, got {k}");
        assert!(k.is_finite(), "curvature should be finite");
    }
}
