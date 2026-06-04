//! Arc-length parameterization.
//! See spec §`arc_length` module.
//!
//! # Safety note
//!
//! `param_from_arc_length` and `arc_length_from_param` use `get_unchecked`
//! for hot-path binary-search accesses. All indices are proved in-bounds by
//! the binary-search loop invariants (standard binary-search correctness);
//! each site is annotated with a SAFETY comment. This is the same pattern
//! used in `eval.rs` for the de Boor recurrence.
// `unsafe_code` is workspace-denied; this module is a targeted exception for
// the MCU hot-path arc-length lookup that the release build must not panic in.
#![allow(unsafe_code)]

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
use crate::eval::{eval, vector_derivative, vector_eval};
#[cfg(feature = "host")]
use crate::{ArcLengthError, NurbsView, VectorNurbsView};

/// Given an arc-length table and a query `s`, return the parameter `u` such
/// that `arc_length(u) = s`. Binary search on `s` plus linear interpolation.
///
/// Contract: `s` is segment-local (relative to this segment's table). Out-of-
/// range queries debug-assert in development and clamp silently in release.
/// Index-safety invariant for `param_from_arc_length` and `arc_length_from_param`:
/// The table has `debug_assert!(len >= 2)` at construction. Binary search
/// initialises `lo = 0`, `hi = last = len - 1` and loop terminates with
/// `hi - lo == 1`, so `lo ∈ [0, last-1]` and `lo+1 = hi ≤ last = len-1`.
/// All accesses `arr[0]`, `arr[last]`, `arr[mid]`, `arr[lo]`, `arr[lo+1]`
/// are in `[0, len-1]`.
#[inline]
pub fn param_from_arc_length<T: Float>(table: &ArcLengthTableRef<'_, T>, s: T) -> T {
    debug_assert!(s >= T::ZERO);
    debug_assert!(s <= table.s_max());
    let s_clamped = s.max(T::ZERO).min(table.s_max());

    let s_arr = table.s();
    let u_arr = table.u();
    debug_assert!(s_arr.len() >= 2);
    debug_assert_eq!(s_arr.len(), u_arr.len());
    // Endpoint short-circuit.
    // SAFETY: len >= 2 → index 0 is valid.
    if s_clamped <= unsafe { *s_arr.get_unchecked(0) } {
        return unsafe { *u_arr.get_unchecked(0) };
    }
    let last = s_arr.len() - 1;
    // SAFETY: last = len-1 < len.
    if s_clamped >= unsafe { *s_arr.get_unchecked(last) } {
        return unsafe { *u_arr.get_unchecked(last) };
    }

    // Binary search for the span [i, i+1] where s_arr[i] <= s_clamped < s_arr[i+1].
    // Invariant: s_arr[lo] <= s_clamped < s_arr[hi], lo < hi.
    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = usize::midpoint(lo, hi);
        // SAFETY: mid ∈ (lo, hi) ⊆ [0, last] < len.
        if unsafe { *s_arr.get_unchecked(mid) } <= s_clamped {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    // Loop exits with hi - lo == 1, so lo ∈ [0, last-1] and lo+1 = hi ≤ last < len.
    // SAFETY: lo and lo+1 both in [0, last] < len; same for u_arr (same length).
    let s_lo = unsafe { *s_arr.get_unchecked(lo) };
    let s_hi = unsafe { *s_arr.get_unchecked(lo + 1) };
    let u_lo = unsafe { *u_arr.get_unchecked(lo) };
    let u_hi = unsafe { *u_arr.get_unchecked(lo + 1) };

    let span = s_hi - s_lo;
    let floor = T::from_f64(MIN_PARAMETRIC_SPEED);
    let frac = (s_clamped - s_lo) / span.max(floor);
    u_lo + (u_hi - u_lo) * frac
}

/// Inverse: given parameter `u`, return arc length `s = arc_length(u)`.
/// Binary search on `u` plus linear interpolation. Same contract as `param_from_arc_length`.
/// Same index-safety invariant as `param_from_arc_length` — see its comment.
#[inline]
pub fn arc_length_from_param<T: Float>(table: &ArcLengthTableRef<'_, T>, u: T) -> T {
    debug_assert!(u >= T::ZERO);
    debug_assert!(u <= table.u_max());
    let u_clamped = u.max(T::ZERO).min(table.u_max());

    let s_arr = table.s();
    let u_arr = table.u();
    debug_assert!(u_arr.len() >= 2);
    debug_assert_eq!(s_arr.len(), u_arr.len());
    // SAFETY: len >= 2 → index 0 is valid.
    if u_clamped <= unsafe { *u_arr.get_unchecked(0) } {
        return unsafe { *s_arr.get_unchecked(0) };
    }
    let last = u_arr.len() - 1;
    // SAFETY: last = len-1 < len.
    if u_clamped >= unsafe { *u_arr.get_unchecked(last) } {
        return unsafe { *s_arr.get_unchecked(last) };
    }

    let mut lo = 0usize;
    let mut hi = last;
    while hi - lo > 1 {
        let mid = usize::midpoint(lo, hi);
        // SAFETY: mid ∈ (lo, hi) ⊆ [0, last] < len.
        if unsafe { *u_arr.get_unchecked(mid) } <= u_clamped {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    // SAFETY: lo and lo+1 both in [0, last] < len (same for s_arr).
    let u_lo = unsafe { *u_arr.get_unchecked(lo) };
    let u_hi = unsafe { *u_arr.get_unchecked(lo + 1) };
    let s_lo = unsafe { *s_arr.get_unchecked(lo) };
    let s_hi = unsafe { *s_arr.get_unchecked(lo + 1) };

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

/// One-shot scalar query for the XY-projected arc length of a 3D (or higher)
/// vector NURBS path. Integrates `√(x'(u)² + y'(u)²) du` over the full knot
/// span via 5-point Gauss-Legendre quadrature with adaptive doubling until the
/// residual relative to the previous estimate is below `1e-9 · |estimate|`
/// (capped at 64 subintervals).
///
/// Used by:
/// - Layer 1 E-mode classification (Task 1.6) — compares per-segment XY arc
///   length against a threshold to decide `COUPLED_TO_XY` vs `INDEPENDENT`.
/// - Step-7-pre segment splitter (Task 3.1) — caps per-segment XY length so
///   downstream curve-pool slots stay bounded.
///
/// For a path lying entirely in the XY plane (Z constant), the result matches
/// the full 3D arc length to f64 round-off.
#[cfg(feature = "host")]
#[must_use]
pub fn xy_arc_length<const D: usize>(xyz: &crate::VectorNurbs<f64, D>) -> f64
where
    [(); D]:,
{
    debug_assert!(D >= 2, "xy_arc_length requires D >= 2 (X and Y axes)");

    let knots = xyz.knots();
    let u_start = knots[0];
    let u_end = knots[knots.len() - 1];

    // Compute the parametric derivative once outside the quadrature loop —
    // for cubic input this is degree-2, evaluated 5·subintervals times.
    let deriv = vector_derivative(xyz);

    let xy_speed = |u: f64| -> f64 {
        let d = vector_eval(&deriv, u);
        let dx = d[0];
        let dy = d[1];
        (dx * dx + dy * dy).sqrt()
    };

    // Adaptive doubling: 1, 2, 4, ..., up to 64 subintervals. Stop when the
    // estimate stops moving by more than `1e-9 · |estimate|`.
    let span = u_end - u_start;
    let mut prev_estimate: Option<f64> = None;
    let mut subintervals: usize = 1;

    loop {
        let mut sum = 0.0_f64;
        for i in 0..subintervals {
            let a = u_start + span * (i as f64) / (subintervals as f64);
            let b = u_start + span * ((i + 1) as f64) / (subintervals as f64);
            sum += integrate_arc_length(xy_speed, a, b, 5);
        }

        if let Some(prev) = prev_estimate {
            let tol = 1e-9 * sum.abs().max(1e-300);
            if (sum - prev).abs() < tol {
                return sum;
            }
        }

        if subintervals >= 64 {
            return sum;
        }

        prev_estimate = Some(sum);
        subintervals *= 2;
    }
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
            // Whole-curve degeneracy: a curve whose integrated arc length is below
            // floating-point noise has no meaningful parameterization. (The
            // pre-existing per-midpoint floor was overly aggressive — it rejected
            // geometrically valid cubics with isolated speed minima where central-
            // difference integration noise dropped below 1e-9 even though the
            // analytic min |dr/du| was orders of magnitude higher.)
            let s_total = *s_samples.last().expect("s_samples is non-empty");
            if s_total <= T::from_f64(MIN_PARAMETRIC_SPEED) {
                return Err(ArcLengthError::DegenerateCurve);
            }
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
mod tests;
