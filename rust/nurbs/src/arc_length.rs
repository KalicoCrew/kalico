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
}
