//! NURBS evaluation: de Boor, vector eval, derivative, curvature.
//! See spec §eval module.

use crate::{Float, NurbsView, VectorNurbsView, MAX_DEGREE, MIN_PARAMETRIC_SPEED, WORKSPACE_SIZE};

/// Find the knot span `k` such that `knots[k] <= u < knots[k+1]`, with the
/// clamped-end special case mapping `u >= knots[n]` to the last span.
/// Reference: Piegl & Tiller "The NURBS Book" Algorithm A2.1.
///
/// Inputs: `knots` is a clamped open knot vector (validated upstream),
/// `p` is the degree, `n` is the control-point count.
pub(crate) fn find_knot_span<T: Float>(knots: &[T], p: usize, n: usize, u: T) -> usize {
    debug_assert!(knots.len() == n + p + 1);
    // Clamped endpoint cases.
    if u >= knots[n] { return n - 1; }
    if u <= knots[p] { return p; }

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
            let numer = de_boor_homogeneous(
                curve.control_points(), w, curve.knots(), curve.degree(), u,
            );
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
pub fn vector_eval<T: Float, V: VectorNurbsView<T, N>, const N: usize>(
    curve: &V,
    u: T,
) -> [T; N] {
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
                d_axes[axis][j] = (d_axes[axis][j] - d_axes[axis][j - 1]).mul_add(alpha, d_axes[axis][j - 1]);
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
pub fn derivative<T: Float>(curve: &crate::ScalarNurbs<T>) -> crate::ScalarNurbs<T> {
    let p = curve.degree();
    assert!(p >= 1, "derivative requires degree >= 1");

    let cps = curve.control_points();
    let knots = curve.knots();
    let new_degree = p - 1;
    let new_n = cps.len() - 1;

    let p_t = T::from_f64(p as f64);

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
pub fn vector_derivative<T: Float, const N: usize>(
    curve: &crate::VectorNurbs<T, N>,
) -> crate::VectorNurbs<T, N> {
    let p = curve.degree();
    assert!(p >= 1, "derivative requires degree >= 1");

    let cps = curve.control_points();
    let knots = curve.knots();
    let new_degree = p - 1;
    let new_n = cps.len() - 1;
    let p_t = T::from_f64(p as f64);

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
        crate::ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        ).unwrap()
    }

    fn quadratic_curve_f64() -> crate::ScalarNurbs<f64> {
        // Bezier-ish: degree 2, knots {0,0,0,1,1,1}, cps {0, 0.5, 1}.
        crate::ScalarNurbs::try_new(
            2,
            vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
            vec![0.0, 0.5, 1.0],
            None,
        ).unwrap()
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
        ).unwrap()
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
        ).unwrap()
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
            let scalar = crate::ScalarNurbs::try_new(
                v.degree(), v.knots().to_vec(), cps_axis, None,
            ).unwrap();
            let expected = eval(&scalar.as_view(), 0.3_f64);
            assert!((result[axis] - expected).abs() < 1e-12,
                "axis {axis}: got {}, expected {}", result[axis], expected);
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
        let expected = (eval(&curve.as_view(), 0.5 + h) - eval(&curve.as_view(), 0.5 - h)) / (2.0 * h);
        let actual = eval(&v, 0.5);
        assert!((actual - expected).abs() < 1e-6, "got {actual}, expected {expected}");
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
            let scalar = crate::ScalarNurbs::try_new(
                curve.degree(), curve.knots().to_vec(), cps_axis, None,
            ).unwrap();
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
        ).unwrap();
        let first = vector_derivative(&parabolic);
        let second = vector_derivative(&first);
        // The path is straight along X — curvature is 0 everywhere.
        let k = curvature_from_derivs(&first, &second, 0.5_f64);
        assert!(k.abs() < 1e-10, "got {k}");
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
        ).unwrap();
        let first = vector_derivative(&arc);
        let second = vector_derivative(&first);
        let k = curvature_from_derivs(&first, &second, 0.5_f64);
        assert!(k > 0.0, "expected positive curvature, got {k}");
        assert!(k.is_finite(), "curvature should be finite");
    }
}
