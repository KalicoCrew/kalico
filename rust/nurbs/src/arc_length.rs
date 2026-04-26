//! Arc-length parameterization.
//! See spec §arc_length module.

use crate::Float;

/// Owned arc-length table. Built on host via `build_arc_length_table_*`,
/// shipped to the MCU as a borrowed view via the wire format.
#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct ArcLengthTable<T: Float> {
    s: Vec<T>,
    u: Vec<T>,
}

#[cfg(feature = "host")]
impl<T: Float> ArcLengthTable<T> {
    /// Construct from monotone non-decreasing s and u sample arrays.
    /// Caller is the builder — already validated.
    pub fn new(s: Vec<T>, u: Vec<T>) -> Self {
        debug_assert_eq!(s.len(), u.len());
        debug_assert!(s.len() >= 2);
        Self { s, u }
    }

    pub fn s(&self) -> &[T] { &self.s }
    pub fn u(&self) -> &[T] { &self.u }
    pub fn s_max(&self) -> T { *self.s.last().expect("table is non-empty") }
    pub fn u_max(&self) -> T { *self.u.last().expect("table is non-empty") }
    pub fn sample_count(&self) -> usize { self.s.len() }

    #[inline]
    pub fn as_view(&self) -> ArcLengthTableRef<'_, T> {
        ArcLengthTableRef { s: &self.s, u: &self.u }
    }

    pub fn into_parts(self) -> (Vec<T>, Vec<T>) { (self.s, self.u) }
}

/// Borrowed arc-length table. Available on host and MCU. Pure lookup.
#[derive(Debug, Clone, Copy)]
pub struct ArcLengthTableRef<'a, T: Float> {
    pub(crate) s: &'a [T],
    pub(crate) u: &'a [T],
}

impl<'a, T: Float> ArcLengthTableRef<'a, T> {
    /// Construct from already-validated slices.
    pub fn new(s: &'a [T], u: &'a [T]) -> Self {
        debug_assert_eq!(s.len(), u.len());
        debug_assert!(s.len() >= 2);
        Self { s, u }
    }

    pub fn s(&self) -> &[T] { self.s }
    pub fn u(&self) -> &[T] { self.u }
    pub fn s_max(&self) -> T { *self.s.last().expect("table is non-empty") }
    pub fn u_max(&self) -> T { *self.u.last().expect("table is non-empty") }
}

/// 5-point Gauss-Legendre nodes (in [-1, 1]) and weights. Exact for polynomials
/// up to degree 9. Sufficient for our integrand magnitudes.
const GAUSS_LEGENDRE_5_NODES: [f64; 5] = [
    -0.906_179_845_938_664_0,
    -0.538_469_310_105_683_1,
     0.0,
     0.538_469_310_105_683_1,
     0.906_179_845_938_664_0,
];
const GAUSS_LEGENDRE_5_WEIGHTS: [f64; 5] = [
    0.236_926_885_056_189_1,
    0.478_628_670_499_366_5,
    0.568_888_888_888_888_9,
    0.478_628_670_499_366_5,
    0.236_926_885_056_189_1,
];

/// Integrate `integrand` over `[u_start, u_end]` via Gauss-Legendre quadrature.
/// `quadrature_points` must be 5; v1 hardcodes 5-point GL — argument reserved
/// for future adaptation (e.g. higher-order for high-degree integrands).
#[cfg(feature = "host")]
pub(crate) fn integrate_arc_length<T: Float, F: Fn(T) -> T>(
    integrand: F,
    u_start: T,
    u_end: T,
    quadrature_points: usize,
) -> T {
    debug_assert_eq!(quadrature_points, 5, "v1 supports only 5-point Gauss-Legendre");

    let half_range = (u_end - u_start) * T::from_f64(0.5);
    let midpoint = (u_start + u_end) * T::from_f64(0.5);

    let mut sum = T::ZERO;
    for i in 0..5 {
        let node = T::from_f64(GAUSS_LEGENDRE_5_NODES[i]);
        let weight = T::from_f64(GAUSS_LEGENDRE_5_WEIGHTS[i]);
        let u = midpoint + half_range * node;
        sum = integrand(u).mul_add(weight, sum);
    }

    sum * half_range
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ref_provides_borrowed_access() {
        let s = [0.0_f64, 0.5, 1.0];
        let u = [0.0_f64, 0.4, 1.0];
        let r = ArcLengthTableRef::new(&s, &u);
        assert_eq!(r.s_max(), 1.0);
        assert_eq!(r.u_max(), 1.0);
    }

    #[cfg(feature = "host")]
    #[test]
    fn owned_as_view_round_trips() {
        let owned = ArcLengthTable::new(vec![0.0, 0.5, 1.0], vec![0.0, 0.4, 1.0]);
        let view = owned.as_view();
        assert_eq!(view.s_max(), 1.0);
    }

    #[cfg(feature = "host")]
    #[test]
    fn integrate_constant_returns_length_times_constant() {
        // ∫_0^1 of f(u)=2 should be 2.
        let result = integrate_arc_length(|_u: f64| 2.0_f64, 0.0, 1.0, 5);
        assert!((result - 2.0).abs() < 1e-12);
    }

    #[cfg(feature = "host")]
    #[test]
    fn integrate_linear_matches_closed_form() {
        // ∫_0^1 of f(u)=u should be 0.5.
        let result = integrate_arc_length(|u: f64| u, 0.0, 1.0, 5);
        assert!((result - 0.5).abs() < 1e-12);
    }

    #[cfg(feature = "host")]
    #[test]
    fn integrate_quadratic_matches_closed_form() {
        // ∫_0^1 of f(u)=u^2 should be 1/3. 5-point Gauss-Legendre is exact for degree <= 9.
        let result = integrate_arc_length(|u: f64| u * u, 0.0, 1.0, 5);
        assert!((result - 1.0 / 3.0).abs() < 1e-12);
    }

    #[cfg(feature = "host")]
    #[test]
    fn build_scalar_table_for_linear_curve() {
        // Linear curve from 0 to 1 over u in [0, 1]: arc length = 1.
        let curve = crate::ScalarNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        ).unwrap();
        let table = build_arc_length_table_scalar(&curve, 1e-6, 64).unwrap();
        assert!((table.s_max() - 1.0).abs() < 1e-6);
        assert!(table.u_max() == 1.0);
        // Monotonicity check
        for w in table.s().windows(2) { assert!(w[1] >= w[0]); }
        for w in table.u().windows(2) { assert!(w[1] >= w[0]); }
    }

    #[cfg(feature = "host")]
    #[test]
    fn build_vector_table_for_3d_linear_curve() {
        // 3D linear curve from origin to (3, 0, 4): arc length = 5.
        let curve = crate::VectorNurbs::try_new(
            1,
            vec![0.0_f64, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [3.0, 0.0, 4.0]],
            None,
        ).unwrap();
        let table = build_arc_length_table_vector(&curve, 1e-5, 64).unwrap();
        assert!((table.s_max() - 5.0).abs() < 1e-4);
    }
}

#[cfg(feature = "host")]
use crate::eval::{eval, vector_eval};
#[cfg(feature = "host")]
use crate::{ArcLengthError, NurbsView, VectorNurbsView, MIN_PARAMETRIC_SPEED};

/// Build an arc-length table for a scalar NURBS via adaptive sampling.
///
/// Strategy: start with a small uniform grid in u; at each step, double the
/// sample count if the linear-interpolation residual against a refined estimate
/// exceeds `tolerance`. Cap at `max_samples`.
///
/// Integrand is `|dP/du|`; for scalar curves we use the absolute value of the
/// scalar derivative evaluated by central difference (we don't take a
/// degree-lowered derivative here because it'd allocate twice for the same
/// information; central difference is cheap on the host).
#[cfg(feature = "host")]
pub fn build_arc_length_table_scalar<T: Float, V: NurbsView<T>>(
    curve: &V,
    tolerance: T,
    max_samples: usize,
) -> Result<ArcLengthTable<T>, ArcLengthError<T>> {
    let h = T::from_f64(1e-6);
    let knots = curve.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let integrand = |u: T| {
        let u_safe = u.max(u_start + h).min(u_end - h);
        let plus = eval(curve, u_safe + h);
        let minus = eval(curve, u_safe - h);
        ((plus - minus) / (h + h)).abs()
    };

    build_table_via_integrand(integrand, u_start, u_end, tolerance, max_samples)
}

/// Build an arc-length table for a vector NURBS in R^3.
#[cfg(feature = "host")]
pub fn build_arc_length_table_vector<T: Float, V: VectorNurbsView<T, 3>>(
    curve: &V,
    tolerance: T,
    max_samples: usize,
) -> Result<ArcLengthTable<T>, ArcLengthError<T>> {
    let h = T::from_f64(1e-6);
    let knots = curve.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    let integrand = |u: T| {
        let u_safe = u.max(u_start + h).min(u_end - h);
        let plus = vector_eval(curve, u_safe + h);
        let minus = vector_eval(curve, u_safe - h);
        let two_h = h + h;
        let dx = (plus[0] - minus[0]) / two_h;
        let dy = (plus[1] - minus[1]) / two_h;
        let dz = (plus[2] - minus[2]) / two_h;
        (dx * dx + dy * dy + dz * dz).sqrt()
    };

    build_table_via_integrand(integrand, u_start, u_end, tolerance, max_samples)
}

/// Adaptive table builder. Doubles sample count until linear-interp residual
/// is below tolerance or we hit the cap.
#[cfg(feature = "host")]
fn build_table_via_integrand<T: Float, F: Fn(T) -> T + Copy>(
    integrand: F,
    u_start: T,
    u_end: T,
    tolerance: T,
    max_samples: usize,
) -> Result<ArcLengthTable<T>, ArcLengthError<T>> {
    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);

    let mut count = 8.max(2);
    loop {
        // Build a table at this sample count by integrating between adjacent u's.
        let mut u_samples: Vec<T> = Vec::with_capacity(count);
        let mut s_samples: Vec<T> = Vec::with_capacity(count);

        let span = u_end - u_start;
        for i in 0..count {
            let frac = T::from_f64(i as f64 / (count - 1) as f64);
            u_samples.push(u_start + span * frac);
        }

        s_samples.push(T::ZERO);
        for i in 1..count {
            // Check for degeneracy at integration sample points.
            let u_mid = (u_samples[i - 1] + u_samples[i]) * T::from_f64(0.5);
            if integrand(u_mid) < floor {
                return Err(ArcLengthError::DegenerateCurve);
            }
            let segment_length = integrate_arc_length(integrand, u_samples[i - 1], u_samples[i], 5);
            let prev = s_samples[i - 1];
            s_samples.push(prev + segment_length);
        }

        // Estimate residual: refine to 2*count and compare s_max.
        let span_full = u_end - u_start;
        let s_refined: T = {
            let count_refined = (count - 1) * 2 + 1;
            let mut acc = T::ZERO;
            for i in 1..count_refined {
                let a = u_start + span_full * T::from_f64((i - 1) as f64 / (count_refined - 1) as f64);
                let b = u_start + span_full * T::from_f64(i as f64 / (count_refined - 1) as f64);
                acc = acc + integrate_arc_length(integrand, a, b, 5);
            }
            acc
        };

        let residual = (s_samples[count - 1] - s_refined).abs();
        if residual <= tolerance {
            return Ok(ArcLengthTable::new(s_samples, u_samples));
        }
        if count * 2 > max_samples {
            return Err(ArcLengthError::ToleranceNotMet {
                achieved_residual: residual,
                samples_used: count,
            });
        }
        count *= 2;
    }
}
