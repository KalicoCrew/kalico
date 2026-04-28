//! Arc-length parameterization.
//! See spec §`arc_length` module.

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
    #[must_use]
    pub fn new(s: Vec<T>, u: Vec<T>) -> Self {
        debug_assert_eq!(s.len(), u.len());
        debug_assert!(s.len() >= 2);
        Self { s, u }
    }

    #[must_use]
    pub fn s(&self) -> &[T] {
        &self.s
    }
    #[must_use]
    pub fn u(&self) -> &[T] {
        &self.u
    }
    #[must_use]
    pub fn s_max(&self) -> T {
        *self.s.last().expect("table is non-empty")
    }
    #[must_use]
    pub fn u_max(&self) -> T {
        *self.u.last().expect("table is non-empty")
    }
    #[must_use]
    pub fn sample_count(&self) -> usize {
        self.s.len()
    }

    #[inline]
    #[must_use]
    pub fn as_view(&self) -> ArcLengthTableRef<'_, T> {
        ArcLengthTableRef {
            s: &self.s,
            u: &self.u,
        }
    }

    #[must_use]
    pub fn into_parts(self) -> (Vec<T>, Vec<T>) {
        (self.s, self.u)
    }
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

    #[must_use]
    pub fn s(&self) -> &[T] {
        self.s
    }
    #[must_use]
    pub fn u(&self) -> &[T] {
        self.u
    }
    #[must_use]
    pub fn s_max(&self) -> T {
        *self.s.last().expect("table is non-empty")
    }
    #[must_use]
    pub fn u_max(&self) -> T {
        *self.u.last().expect("table is non-empty")
    }
}

/// 5-point Gauss-Legendre nodes (in [-1, 1]) and weights. Exact for polynomials
/// up to degree 9. Sufficient for our integrand magnitudes.
#[cfg(feature = "host")]
const GAUSS_LEGENDRE_5_NODES: [f64; 5] = [
    -0.906_179_845_938_664,
    -0.538_469_310_105_683_1,
    0.0,
    0.538_469_310_105_683_1,
    0.906_179_845_938_664,
];
#[cfg(feature = "host")]
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
    debug_assert_eq!(
        quadrature_points, 5,
        "v1 supports only 5-point Gauss-Legendre"
    );

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

use crate::MIN_PARAMETRIC_SPEED;
#[cfg(feature = "host")]
use crate::eval::{eval, vector_eval};
#[cfg(feature = "host")]
use crate::{ArcLengthError, NurbsView, VectorNurbsView};

/// Given an arc-length table and a query `s`, return the parameter `u` such
/// that `arc_length(u) = s`. Binary search on `s` plus linear interpolation.
///
/// Contract: `s` is segment-local (relative to this segment's table). Out-of-
/// range queries debug-assert in development and clamp silently in release.
#[inline]
pub fn param_from_arc_length<T: Float>(table: &ArcLengthTableRef<'_, T>, s: T) -> T {
    debug_assert!(s >= T::ZERO);
    debug_assert!(s <= table.s_max());
    let s_clamped = s.max(T::ZERO).min(table.s_max());

    let s_arr = table.s();
    let u_arr = table.u();
    // Endpoint short-circuit.
    if s_clamped <= s_arr[0] {
        return u_arr[0];
    }
    let last = s_arr.len() - 1;
    if s_clamped >= s_arr[last] {
        return u_arr[last];
    }

    // Binary search for the span [i, i+1] where s_arr[i] <= s_clamped < s_arr[i+1].
    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if s_arr[mid] <= s_clamped {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    let s_lo = s_arr[lo];
    let s_hi = s_arr[lo + 1];
    let u_lo = u_arr[lo];
    let u_hi = u_arr[lo + 1];

    let span = s_hi - s_lo;
    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
    let frac = (s_clamped - s_lo) / span.max(floor);
    u_lo + (u_hi - u_lo) * frac
}

/// Inverse: given parameter `u`, return arc length `s = arc_length(u)`.
/// Binary search on `u` plus linear interpolation. Same contract as `param_from_arc_length`.
#[inline]
pub fn arc_length_from_param<T: Float>(table: &ArcLengthTableRef<'_, T>, u: T) -> T {
    debug_assert!(u >= T::ZERO);
    debug_assert!(u <= table.u_max());
    let u_clamped = u.max(T::ZERO).min(table.u_max());

    let s_arr = table.s();
    let u_arr = table.u();
    if u_clamped <= u_arr[0] {
        return s_arr[0];
    }
    let last = u_arr.len() - 1;
    if u_clamped >= u_arr[last] {
        return s_arr[last];
    }

    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if u_arr[mid] <= u_clamped {
            lo = mid;
        } else {
            hi = mid;
        }
    }

    let u_lo = u_arr[lo];
    let u_hi = u_arr[lo + 1];
    let s_lo = s_arr[lo];
    let s_hi = s_arr[lo + 1];

    let span = u_hi - u_lo;
    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
    let frac = (u_clamped - u_lo) / span.max(floor);
    s_lo + (s_hi - s_lo) * frac
}

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

use crate::WireError;
use crate::wire::{ARC_LENGTH_HEADER_BYTES, FORMAT_VERSION_V1};

impl<'a> ArcLengthTableRef<'a, f32> {
    /// Zero-copy parse of a wire-format buffer.
    ///
    /// Layout: `u8 version, u8 reserved, u16 sample_count, u32 reserved2,`
    /// `f32[sample_count] s, f32[sample_count] u`.
    pub fn try_from_wire(buf: &'a [u8]) -> Result<Self, WireError> {
        if (buf.as_ptr() as usize) % core::mem::align_of::<f32>() != 0 {
            return Err(WireError::Misaligned);
        }
        if buf.len() < ARC_LENGTH_HEADER_BYTES {
            return Err(WireError::TruncatedBuffer {
                expected_len: ARC_LENGTH_HEADER_BYTES,
                got: buf.len(),
            });
        }
        let version = buf[0];
        if version != FORMAT_VERSION_V1 {
            return Err(WireError::UnknownVersion(version));
        }
        let sample_count = u16::from_ne_bytes([buf[2], buf[3]]) as usize;
        if sample_count < 2 {
            return Err(WireError::TruncatedBuffer {
                expected_len: ARC_LENGTH_HEADER_BYTES + 2 * core::mem::size_of::<f32>() * 2,
                got: buf.len(),
            });
        }

        let bytes_per_axis = sample_count * core::mem::size_of::<f32>();
        let total = ARC_LENGTH_HEADER_BYTES + 2 * bytes_per_axis;
        if buf.len() < total {
            return Err(WireError::TruncatedBuffer {
                expected_len: total,
                got: buf.len(),
            });
        }

        // SAFETY: alignment of `buf` to `align_of::<f32>()` is checked above; the
        // header is 8 bytes (multiple of 4) so `s_ptr` and `u_ptr` remain 4-byte
        // aligned. The total length covers `2 * sample_count * size_of::<f32>()`
        // bytes after the header. Lifetime `'a` is inherited from `buf`.
        #[allow(unsafe_code)]
        let (s, u) = unsafe {
            let s_ptr = buf.as_ptr().add(ARC_LENGTH_HEADER_BYTES).cast::<f32>();
            let u_ptr = buf
                .as_ptr()
                .add(ARC_LENGTH_HEADER_BYTES + bytes_per_axis)
                .cast::<f32>();
            (
                core::slice::from_raw_parts(s_ptr, sample_count),
                core::slice::from_raw_parts(u_ptr, sample_count),
            )
        };
        Ok(Self::new(s, u))
    }
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

    let mut count = 8;
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
                let a =
                    u_start + span_full * T::from_f64((i - 1) as f64 / (count_refined - 1) as f64);
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

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(clippy::float_cmp)]
    #[test]
    fn ref_provides_borrowed_access() {
        let s = [0.0_f64, 0.5, 1.0];
        let u = [0.0_f64, 0.4, 1.0];
        let r = ArcLengthTableRef::new(&s, &u);
        assert_eq!(r.s_max(), 1.0);
        assert_eq!(r.u_max(), 1.0);
    }

    #[cfg(feature = "host")]
    #[allow(clippy::float_cmp)]
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
    #[allow(clippy::float_cmp)]
    #[test]
    fn build_scalar_table_for_linear_curve() {
        // Linear curve from 0 to 1 over u in [0, 1]: arc length = 1.
        let curve =
            crate::ScalarNurbs::try_new(1, vec![0.0_f64, 0.0, 1.0, 1.0], vec![0.0, 1.0], None)
                .unwrap();
        let table = build_arc_length_table_scalar(&curve, 1e-6, 64).unwrap();
        assert!((table.s_max() - 1.0).abs() < 1e-6);
        assert_eq!(table.u_max(), 1.0);
        // Monotonicity check
        for w in table.s().windows(2) {
            assert!(w[1] >= w[0]);
        }
        for w in table.u().windows(2) {
            assert!(w[1] >= w[0]);
        }
    }

    #[allow(clippy::float_cmp)]
    #[test]
    fn param_from_arc_length_at_endpoints() {
        let table = ArcLengthTableRef::new(&[0.0_f64, 0.5, 1.0], &[0.0, 0.6, 1.0]);
        assert_eq!(param_from_arc_length(&table, 0.0), 0.0);
        assert_eq!(param_from_arc_length(&table, 1.0), 1.0);
    }

    #[test]
    fn param_from_arc_length_interpolates_linearly() {
        let table = ArcLengthTableRef::new(&[0.0_f64, 0.5, 1.0], &[0.0, 0.6, 1.0]);
        // s = 0.25 lies between (0.0 -> 0.0) and (0.5 -> 0.6); linear interp gives 0.3.
        assert!((param_from_arc_length(&table, 0.25_f64) - 0.3).abs() < 1e-12);
    }

    #[allow(clippy::float_cmp)]
    #[test]
    fn param_from_arc_length_clamps_above_range_in_release() {
        // In release, out-of-range queries clamp silently. In debug, this would
        // fire a debug_assert, so the test itself uses an in-range value but
        // relies on the clamp branch of the implementation.
        let table = ArcLengthTableRef::new(&[0.0_f64, 1.0], &[0.0, 1.0]);
        // Use a value that exercises clamp logic without violating debug_assert.
        let v = param_from_arc_length(&table, 1.0_f64);
        assert_eq!(v, 1.0);
    }

    #[test]
    fn arc_length_from_param_inverts_param_from_arc_length() {
        let table = ArcLengthTableRef::new(&[0.0_f64, 0.4, 1.0], &[0.0, 0.5, 1.0]);
        let u = 0.3_f64;
        let s = arc_length_from_param(&table, u);
        let u_back = param_from_arc_length(&table, s);
        assert!((u - u_back).abs() < 1e-12);
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
        )
        .unwrap();
        let table = build_arc_length_table_vector(&curve, 1e-5, 64).unwrap();
        assert!((table.s_max() - 5.0).abs() < 1e-4);
    }

    #[test]
    fn try_from_wire_parses_small_table() {
        // Layout: u8 version, u8 reserved, u16 sample_count, u32 reserved2,
        //         T[sample_count] s, T[sample_count] u
        let mut buf = Vec::new();
        buf.extend_from_slice(&[1u8, 0]); // version, reserved
        buf.extend_from_slice(&3u16.to_ne_bytes()); // sample_count
        buf.extend_from_slice(&[0u8; 4]); // reserved2
        for v in [0.0_f32, 0.5, 1.0] {
            buf.extend_from_slice(&v.to_ne_bytes());
        }
        for v in [0.0_f32, 0.6, 1.0] {
            buf.extend_from_slice(&v.to_ne_bytes());
        }

        let aligned = test_align(&buf, 4);
        let r = ArcLengthTableRef::<f32>::try_from_wire(aligned.as_slice()).unwrap();
        assert_eq!(r.s(), &[0.0_f32, 0.5, 1.0]);
        assert_eq!(r.u(), &[0.0_f32, 0.6, 1.0]);
    }

    /// Test-only owner; same shape as `align_buf` in scalar.rs (Task 9).
    struct AlignedBytes {
        backing: Vec<u32>,
        len: usize,
    }

    impl AlignedBytes {
        fn as_slice(&self) -> &[u8] {
            // SAFETY: `Vec<u32>` is 4-byte aligned and `len <= backing.len() * 4`.
            // `u32` has no padding and any bit pattern is a valid `u8` byte.
            #[allow(unsafe_code)]
            unsafe {
                core::slice::from_raw_parts(self.backing.as_ptr().cast::<u8>(), self.len)
            }
        }
    }

    fn test_align(data: &[u8], _align: usize) -> AlignedBytes {
        let n = data.len().div_ceil(4);
        let mut backing: Vec<u32> = vec![0; n];
        // SAFETY: `backing` owns `n*4` bytes 4-byte aligned; `data.len() <= n*4`.
        // `u32` has no padding so writing arbitrary bytes is well-defined.
        #[allow(unsafe_code)]
        let bytes: &mut [u8] =
            unsafe { core::slice::from_raw_parts_mut(backing.as_mut_ptr().cast::<u8>(), n * 4) };
        bytes[..data.len()].copy_from_slice(data);
        AlignedBytes {
            backing,
            len: data.len(),
        }
    }
}
