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
}
