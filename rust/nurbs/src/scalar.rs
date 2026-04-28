//! Scalar (1D) NURBS types: `ScalarNurbs` (owned, host) and `ScalarNurbsRef` (borrowed).

use crate::{ConstructError, Float, MAX_DEGREE, NurbsView};

/// Owned, heap-backed scalar NURBS. Host-only.
///
/// Construction validates all spec §Substrate invariants. After construction,
/// the data is trusted; eval algorithms only `debug_assert` invariants.
#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct ScalarNurbs<T: Float> {
    degree: u8,
    knots: crate::knot::KnotVector<T>,
    control_points: Vec<T>,
    weights: Option<Vec<T>>,
}

#[cfg(feature = "host")]
impl<T: Float> ScalarNurbs<T> {
    /// Build a scalar NURBS, validating every spec-listed invariant.
    pub fn try_new(
        degree: u8,
        knots: Vec<T>,
        control_points: Vec<T>,
        weights: Option<Vec<T>>,
    ) -> Result<Self, ConstructError> {
        validate(degree, &knots, control_points.len(), weights.as_deref())?;
        let knot_vector = crate::knot::KnotVector::try_new(knots)
            .expect("validate already ensured monotone + length");
        Ok(Self {
            degree,
            knots: knot_vector,
            control_points,
            weights,
        })
    }

    #[must_use]
    pub fn degree(&self) -> u8 {
        self.degree
    }
    #[must_use]
    pub fn knots(&self) -> &[T] {
        self.knots.as_slice()
    }
    #[must_use]
    pub fn control_points(&self) -> &[T] {
        &self.control_points
    }
    #[must_use]
    pub fn weights(&self) -> Option<&[T]> {
        self.weights.as_deref()
    }

    /// Cheap projection to a borrowed view.
    #[inline]
    #[must_use]
    pub fn as_view(&self) -> ScalarNurbsRef<'_, T> {
        ScalarNurbsRef {
            degree: self.degree,
            knots: self.knots.as_slice(),
            control_points: &self.control_points,
            weights: self.weights.as_deref(),
        }
    }

    /// Consume self into raw parts. Used by host pre-bake pipelines that
    /// build new NURBS by transformation.
    #[must_use]
    pub fn into_parts(self) -> (u8, Vec<T>, Vec<T>, Option<Vec<T>>) {
        (
            self.degree,
            self.knots.into_inner(),
            self.control_points,
            self.weights,
        )
    }
}

#[cfg(feature = "host")]
impl<T: Float> NurbsView<T> for ScalarNurbs<T> {
    #[inline]
    fn degree(&self) -> u8 {
        self.degree
    }
    #[inline]
    fn knots(&self) -> &[T] {
        self.knots.as_slice()
    }
    #[inline]
    fn control_points(&self) -> &[T] {
        &self.control_points
    }
    #[inline]
    fn weights(&self) -> Option<&[T]> {
        self.weights.as_deref()
    }
}

/// Borrowed, slice-backed scalar NURBS. Available on host and MCU.
///
/// Constructed either via `ScalarNurbs::as_view` (host) or
/// `ScalarNurbsRef::try_new` / `try_from_wire` (MCU + zero-copy paths).
#[derive(Debug, Clone, Copy)]
pub struct ScalarNurbsRef<'a, T: Float> {
    pub(crate) degree: u8,
    pub(crate) knots: &'a [T],
    pub(crate) control_points: &'a [T],
    pub(crate) weights: Option<&'a [T]>,
}

impl<'a, T: Float> ScalarNurbsRef<'a, T> {
    /// Build a borrowed NURBS from already-validated slices, re-running invariants.
    /// Use when assembling a `ScalarNurbsRef` outside the wire path.
    pub fn try_new(
        degree: u8,
        knots: &'a [T],
        control_points: &'a [T],
        weights: Option<&'a [T]>,
    ) -> Result<Self, ConstructError> {
        validate(degree, knots, control_points.len(), weights)?;
        Ok(Self {
            degree,
            knots,
            control_points,
            weights,
        })
    }

    #[must_use]
    pub fn degree(&self) -> u8 {
        self.degree
    }
    #[must_use]
    pub fn knots(&self) -> &[T] {
        self.knots
    }
    #[must_use]
    pub fn control_points(&self) -> &[T] {
        self.control_points
    }
    #[must_use]
    pub fn weights(&self) -> Option<&[T]> {
        self.weights
    }
}

impl<T: Float> NurbsView<T> for ScalarNurbsRef<'_, T> {
    #[inline]
    fn degree(&self) -> u8 {
        self.degree
    }
    #[inline]
    fn knots(&self) -> &[T] {
        self.knots
    }
    #[inline]
    fn control_points(&self) -> &[T] {
        self.control_points
    }
    #[inline]
    fn weights(&self) -> Option<&[T]> {
        self.weights
    }
}

/// Shared validation. See spec §Substrate / Validation rules.
pub(crate) fn validate<T: Float>(
    degree: u8,
    knots: &[T],
    control_point_count: usize,
    weights: Option<&[T]>,
) -> Result<(), ConstructError> {
    if (degree as usize) > MAX_DEGREE {
        return Err(ConstructError::DegreeExceeded {
            actual: degree,
            max: MAX_DEGREE as u8,
        });
    }
    let p = degree as usize;
    let expected_knot_count = control_point_count + p + 1;
    if knots.len() != expected_knot_count {
        return Err(ConstructError::KnotCountMismatch {
            expected: expected_knot_count,
            got: knots.len(),
        });
    }
    if knots.len() < 2 * (p + 1) {
        // not enough knots for clamped open of this degree
        return Err(ConstructError::KnotCountMismatch {
            expected: 2 * (p + 1),
            got: knots.len(),
        });
    }

    // Clamped at start: knots[0..=p] all equal.
    let start = knots[0];
    for k in &knots[1..=p] {
        if *k != start {
            return Err(ConstructError::KnotsNotClamped);
        }
    }
    // Clamped at end: knots[len-1-p..] all equal.
    let last_idx = knots.len() - 1;
    let end = knots[last_idx];
    for k in &knots[last_idx - p..last_idx] {
        if *k != end {
            return Err(ConstructError::KnotsNotClamped);
        }
    }

    // Non-decreasing.
    for window in knots.windows(2) {
        if window[1] < window[0] {
            return Err(ConstructError::KnotsNotMonotone);
        }
    }

    // Non-degenerate range.
    if !(end > start) {
        return Err(ConstructError::DegenerateKnotRange);
    }

    if let Some(w) = weights {
        if w.len() != control_point_count {
            return Err(ConstructError::WeightCountMismatch {
                expected: control_point_count,
                got: w.len(),
            });
        }
        for weight in w {
            if !(*weight > T::ZERO) {
                return Err(ConstructError::NonPositiveWeight);
            }
        }
    }

    Ok(())
}

use crate::{
    WireError,
    wire::{FORMAT_VERSION_V1, SCALAR_HEADER_BYTES},
};

impl<'a> ScalarNurbsRef<'a, f32> {
    /// Zero-copy parse of a wire-format buffer into a borrowed scalar NURBS.
    /// See spec §Substrate / Wire format for the byte layout.
    ///
    /// Caller responsibilities (Layer 5 contract):
    /// - `buf` is aligned to `align_of::<f32>()` (4 bytes).
    /// - `buf` is in host-native endianness.
    pub fn try_from_wire(buf: &'a [u8]) -> Result<Self, WireError> {
        if (buf.as_ptr() as usize) % core::mem::align_of::<f32>() != 0 {
            return Err(WireError::Misaligned);
        }
        if buf.len() < SCALAR_HEADER_BYTES {
            return Err(WireError::TruncatedBuffer {
                expected_len: SCALAR_HEADER_BYTES,
                got: buf.len(),
            });
        }
        let version = buf[0];
        if version != FORMAT_VERSION_V1 {
            return Err(WireError::UnknownVersion(version));
        }
        let degree = buf[1];
        let has_weights = buf[2];
        let knot_count = u16::from_ne_bytes([buf[4], buf[5]]) as usize;
        let cp_count = u16::from_ne_bytes([buf[6], buf[7]]) as usize;

        let knots_bytes = knot_count * core::mem::size_of::<f32>();
        let cps_bytes = cp_count * core::mem::size_of::<f32>();
        let weights_bytes = if has_weights == 1 { cps_bytes } else { 0 };
        let total = SCALAR_HEADER_BYTES + knots_bytes + cps_bytes + weights_bytes;
        if buf.len() < total {
            return Err(WireError::TruncatedBuffer {
                expected_len: total,
                got: buf.len(),
            });
        }

        // SAFETY: alignment checked above; lengths checked above; T = f32 has
        // no invalid bit patterns for any 4-byte sequence.
        #[allow(unsafe_code)]
        let (knots, cps, weights) = unsafe {
            let knots_ptr = buf.as_ptr().add(SCALAR_HEADER_BYTES).cast::<f32>();
            let cps_ptr = buf
                .as_ptr()
                .add(SCALAR_HEADER_BYTES + knots_bytes)
                .cast::<f32>();
            let knots = core::slice::from_raw_parts(knots_ptr, knot_count);
            let cps = core::slice::from_raw_parts(cps_ptr, cp_count);
            let weights = if has_weights == 1 {
                let w_ptr = buf
                    .as_ptr()
                    .add(SCALAR_HEADER_BYTES + knots_bytes + cps_bytes)
                    .cast::<f32>();
                Some(core::slice::from_raw_parts(w_ptr, cp_count))
            } else {
                None
            };
            (knots, cps, weights)
        };

        Self::try_new(degree, knots, cps, weights).map_err(WireError::from)
    }
}

#[cfg(all(test, feature = "host"))]
mod tests {
    use super::*;
    use crate::ConstructError;

    fn linear_curve() -> ScalarNurbs<f64> {
        // Degree-1 NURBS, 2 control points, knots {0,0,1,1}.
        ScalarNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, 1.0], None).unwrap()
    }

    #[test]
    fn try_new_accepts_valid_linear() {
        let curve = linear_curve();
        assert_eq!(curve.degree(), 1);
        assert_eq!(curve.control_points(), &[0.0, 1.0]);
    }

    #[test]
    fn try_new_rejects_degree_exceeded() {
        let result = ScalarNurbs::<f64>::try_new(21, vec![0.0; 23], vec![0.0; 1], None);
        assert!(matches!(
            result,
            Err(ConstructError::DegreeExceeded {
                actual: 21,
                max: 20
            })
        ));
    }

    #[test]
    fn try_new_rejects_knot_count_mismatch() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 1.0], // 3 knots, but 2 cps + 1 + 1 = 4 expected
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(
            result,
            Err(ConstructError::KnotCountMismatch { .. })
        ));
    }

    #[test]
    fn try_new_rejects_unclamped_start() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.5, 1.0, 1.0], // not clamped at start
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
    }

    #[test]
    fn try_new_rejects_unclamped_end() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 0.5, 1.0], // not clamped at end
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
    }

    #[test]
    fn try_new_rejects_non_monotone_knots() {
        let result = ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.4, 0.3, 1.0, 1.0, 1.0], // 0.3 < 0.4
            vec![0.0, 0.5, 1.0, 1.5, 2.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotMonotone)));
    }

    #[test]
    fn try_new_rejects_degenerate_knot_range() {
        let result = ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 0.0, 0.0], vec![0.0, 1.0], None);
        assert!(matches!(result, Err(ConstructError::DegenerateKnotRange)));
    }

    #[test]
    fn try_new_rejects_weight_count_mismatch() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            Some(vec![1.0]), // 1 weight for 2 cps
        );
        assert!(matches!(
            result,
            Err(ConstructError::WeightCountMismatch { .. })
        ));
    }

    #[test]
    fn try_new_rejects_non_positive_weight() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            Some(vec![1.0, 0.0]),
        );
        assert!(matches!(result, Err(ConstructError::NonPositiveWeight)));
    }

    #[test]
    fn as_view_provides_borrowed_access() {
        let owned = linear_curve();
        let view = owned.as_view();
        assert_eq!(view.degree(), 1);
        assert_eq!(view.knots(), &[0.0, 0.0, 1.0, 1.0]);
        assert_eq!(view.control_points(), &[0.0, 1.0]);
    }

    #[test]
    fn ref_try_new_accepts_valid_data() {
        let knots = [0.0_f64, 0.0, 1.0, 1.0];
        let cps = [0.0_f64, 1.0];
        let r = ScalarNurbsRef::try_new(1, &knots, &cps, None).unwrap();
        assert_eq!(r.degree(), 1);
    }

    #[test]
    fn try_from_wire_parses_unweighted_linear() {
        // Layout: u8 version, u8 degree, u8 has_weights, u8 reserved,
        //         u16 knot_count, u16 cp_count, then knots + cps (both as f32).
        // Linear curve: degree=1, knots=[0,0,1,1], cps=[0.0, 1.0]
        let mut buf = Vec::new();
        buf.extend_from_slice(&[1, 1, 0, 0]); // version, degree, has_weights, reserved
        buf.extend_from_slice(&4u16.to_ne_bytes()); // knot_count
        buf.extend_from_slice(&2u16.to_ne_bytes()); // cp_count
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());

        // Ensure 4-byte alignment by allocating into an aligned buffer
        let aligned = align_buf(&buf, 4);
        let r = ScalarNurbsRef::<f32>::try_from_wire(aligned.as_slice()).unwrap();
        assert_eq!(r.degree(), 1);
        assert_eq!(r.control_points(), &[0.0_f32, 1.0]);
        assert!(r.weights().is_none());
    }

    #[test]
    fn try_from_wire_rejects_misaligned_buffer() {
        let mut data = [0u8; 32 + 1];
        data[0] = 1;
        // Stack-array layout in release can land on an address where `&buf[1..]`
        // happens to be 4-aligned. Anchor on a 4-aligned base via align_buf, then
        // slice from offset 1 so misalignment for f32 is guaranteed.
        let aligned = align_buf(&data, 4);
        let result = ScalarNurbsRef::<f32>::try_from_wire(&aligned.as_slice()[1..]);
        assert!(matches!(result, Err(crate::WireError::Misaligned)));
    }

    #[test]
    fn try_from_wire_rejects_unknown_version() {
        let buf = align_buf(
            &[
                0xFFu8, 1, 0, 0, 4, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            ],
            4,
        );
        let result = ScalarNurbsRef::<f32>::try_from_wire(buf.as_slice());
        assert!(matches!(
            result,
            Err(crate::WireError::UnknownVersion(0xFF))
        ));
    }

    #[test]
    fn try_from_wire_rejects_truncated_header() {
        let buf = align_buf(&[1u8, 1, 0, 0], 4); // only 4 bytes; 8-byte header missing
        let result = ScalarNurbsRef::<f32>::try_from_wire(buf.as_slice());
        assert!(matches!(
            result,
            Err(crate::WireError::TruncatedBuffer { .. })
        ));
    }

    /// Owns a 4-byte-aligned byte buffer for wire-format tests. The backing
    /// storage is a `Vec<u32>` (alignment 4); we expose its bytes via `as_slice`.
    /// Using a wrapper avoids the layout-mismatch UB that would arise from
    /// transmuting `Vec<u32>` → `Vec<u8>` and letting the latter free with the
    /// wrong alignment.
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

    /// Allocate a buffer aligned to `align` bytes containing `data`.
    fn align_buf(data: &[u8], align: usize) -> AlignedBytes {
        match align {
            4 => {
                let n = data.len().div_ceil(4);
                let mut backing: Vec<u32> = vec![0; n];
                // SAFETY: `backing` owns `n * 4` bytes with 4-byte alignment;
                // we write exactly `data.len() <= n * 4` bytes via the
                // `&mut [u8]` view, then release it before returning.
                #[allow(unsafe_code)]
                let bytes: &mut [u8] = unsafe {
                    core::slice::from_raw_parts_mut(backing.as_mut_ptr().cast::<u8>(), n * 4)
                };
                bytes[..data.len()].copy_from_slice(data);
                AlignedBytes {
                    backing,
                    len: data.len(),
                }
            }
            _ => unimplemented!(),
        }
    }
}
