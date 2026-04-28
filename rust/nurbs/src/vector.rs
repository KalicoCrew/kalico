//! Vector NURBS types in R^N: `VectorNurbs`<T, N> (owned) and `VectorNurbsRef`<T, N> (borrowed).

use crate::{ConstructError, Float, VectorNurbsView, scalar::validate};

#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct VectorNurbs<T: Float, const N: usize> {
    degree: u8,
    knots: crate::knot::KnotVector<T>,
    control_points: Vec<[T; N]>,
    weights: Option<Vec<T>>,
}

#[cfg(feature = "host")]
impl<T: Float, const N: usize> VectorNurbs<T, N> {
    pub fn try_new(
        degree: u8,
        knots: Vec<T>,
        control_points: Vec<[T; N]>,
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
    pub fn control_points(&self) -> &[[T; N]] {
        &self.control_points
    }
    #[must_use]
    pub fn weights(&self) -> Option<&[T]> {
        self.weights.as_deref()
    }

    #[inline]
    #[must_use]
    pub fn as_view(&self) -> VectorNurbsRef<'_, T, N> {
        VectorNurbsRef {
            degree: self.degree,
            knots: self.knots.as_slice(),
            control_points: &self.control_points,
            weights: self.weights.as_deref(),
        }
    }

    #[must_use]
    pub fn into_parts(self) -> (u8, Vec<T>, Vec<[T; N]>, Option<Vec<T>>) {
        (
            self.degree,
            self.knots.into_inner(),
            self.control_points,
            self.weights,
        )
    }
}

#[cfg(feature = "host")]
impl<T: Float, const N: usize> VectorNurbsView<T, N> for VectorNurbs<T, N> {
    #[inline]
    fn degree(&self) -> u8 {
        self.degree
    }
    #[inline]
    fn knots(&self) -> &[T] {
        self.knots.as_slice()
    }
    #[inline]
    fn control_points(&self) -> &[[T; N]] {
        &self.control_points
    }
    #[inline]
    fn weights(&self) -> Option<&[T]> {
        self.weights.as_deref()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct VectorNurbsRef<'a, T: Float, const N: usize> {
    pub(crate) degree: u8,
    pub(crate) knots: &'a [T],
    pub(crate) control_points: &'a [[T; N]],
    pub(crate) weights: Option<&'a [T]>,
}

impl<'a, T: Float, const N: usize> VectorNurbsRef<'a, T, N> {
    pub fn try_new(
        degree: u8,
        knots: &'a [T],
        control_points: &'a [[T; N]],
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
    pub fn control_points(&self) -> &[[T; N]] {
        self.control_points
    }
    #[must_use]
    pub fn weights(&self) -> Option<&[T]> {
        self.weights
    }
}

impl<T: Float, const N: usize> VectorNurbsView<T, N> for VectorNurbsRef<'_, T, N> {
    #[inline]
    fn degree(&self) -> u8 {
        self.degree
    }
    #[inline]
    fn knots(&self) -> &[T] {
        self.knots
    }
    #[inline]
    fn control_points(&self) -> &[[T; N]] {
        self.control_points
    }
    #[inline]
    fn weights(&self) -> Option<&[T]> {
        self.weights
    }
}

use crate::{
    WireError,
    wire::{FORMAT_VERSION_V1, VECTOR_HEADER_BYTES},
};

impl<'a, const N: usize> VectorNurbsRef<'a, f32, N> {
    /// Zero-copy parse of a wire-format buffer. Same alignment / endianness
    /// contract as scalar form. Validates `axes_n` against const generic `N`.
    pub fn try_from_wire(buf: &'a [u8]) -> Result<Self, WireError> {
        if (buf.as_ptr() as usize) % core::mem::align_of::<f32>() != 0 {
            return Err(WireError::Misaligned);
        }
        if buf.len() < VECTOR_HEADER_BYTES {
            return Err(WireError::TruncatedBuffer {
                expected_len: VECTOR_HEADER_BYTES,
                got: buf.len(),
            });
        }
        let version = buf[0];
        if version != FORMAT_VERSION_V1 {
            return Err(WireError::UnknownVersion(version));
        }
        let degree = buf[1];
        let has_weights = buf[2];
        let axes_n = buf[3];
        if axes_n as usize != N {
            return Err(WireError::AxisCountMismatch {
                expected: N,
                got: axes_n,
            });
        }
        let knot_count = u16::from_ne_bytes([buf[4], buf[5]]) as usize;
        let cp_count = u16::from_ne_bytes([buf[6], buf[7]]) as usize;

        let knots_bytes = knot_count * core::mem::size_of::<f32>();
        let cps_bytes = cp_count * N * core::mem::size_of::<f32>();
        let weights_bytes = if has_weights == 1 {
            cp_count * core::mem::size_of::<f32>()
        } else {
            0
        };
        let total = VECTOR_HEADER_BYTES + knots_bytes + cps_bytes + weights_bytes;
        if buf.len() < total {
            return Err(WireError::TruncatedBuffer {
                expected_len: total,
                got: buf.len(),
            });
        }

        // SAFETY: alignment checked above; lengths checked above; f32 has no
        // invalid bit patterns for any 4-byte sequence; `[f32; N]` has the same
        // layout as N consecutive f32 values (Rust guarantees array layout is
        // contiguous with no padding between elements), so `cp_count` such
        // arrays occupy exactly `cp_count * N * 4` contiguous bytes.
        #[allow(unsafe_code)]
        let (knots, cps, weights) = unsafe {
            let knots_ptr = buf.as_ptr().add(VECTOR_HEADER_BYTES).cast::<f32>();
            let cps_ptr = buf
                .as_ptr()
                .add(VECTOR_HEADER_BYTES + knots_bytes)
                .cast::<[f32; N]>();
            let knots = core::slice::from_raw_parts(knots_ptr, knot_count);
            let cps = core::slice::from_raw_parts(cps_ptr, cp_count);
            let weights = if has_weights == 1 {
                let w_ptr = buf
                    .as_ptr()
                    .add(VECTOR_HEADER_BYTES + knots_bytes + cps_bytes)
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
#[allow(clippy::float_cmp)] // tests assert exact stored control-point values, no arithmetic
mod tests {
    use super::*;

    fn linear_3d_curve() -> VectorNurbs<f64, 3> {
        VectorNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
            None,
        )
        .unwrap()
    }

    #[test]
    fn try_new_accepts_valid_linear_3d() {
        let curve = linear_3d_curve();
        assert_eq!(curve.degree(), 1);
        assert_eq!(curve.control_points()[1], [1.0, 2.0, 3.0]);
    }

    #[test]
    fn try_new_rejects_degree_exceeded() {
        let result = VectorNurbs::<f64, 3>::try_new(21, vec![0.0; 23], vec![[0.0; 3]; 1], None);
        assert!(matches!(
            result,
            Err(crate::ConstructError::DegreeExceeded { .. })
        ));
    }

    #[test]
    fn try_new_rejects_knot_count_mismatch() {
        let result =
            VectorNurbs::<f64, 3>::try_new(1, vec![0.0, 0.0, 1.0], vec![[0.0; 3], [1.0; 3]], None);
        assert!(matches!(
            result,
            Err(crate::ConstructError::KnotCountMismatch { .. })
        ));
    }

    #[test]
    fn as_view_provides_borrowed_access() {
        let owned = linear_3d_curve();
        let view = owned.as_view();
        assert_eq!(view.degree(), 1);
        assert_eq!(view.control_points()[1], [1.0, 2.0, 3.0]);
    }

    #[test]
    fn try_from_wire_parses_3d_unweighted_linear() {
        // Layout: u8 version, u8 degree, u8 has_weights, u8 axes_n,
        //         u16 knot_count, u16 cp_count, then knots + cps (interleaved).
        let mut buf = Vec::new();
        buf.extend_from_slice(&[1, 1, 0, 3]); // version, degree, has_weights, axes_n
        buf.extend_from_slice(&4u16.to_ne_bytes()); // knot_count
        buf.extend_from_slice(&2u16.to_ne_bytes()); // cp_count
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&0.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());
        buf.extend_from_slice(&1.0_f32.to_ne_bytes());
        // CPs interleaved: [0,0,0], [1,2,3]
        for &v in &[0.0_f32, 0.0, 0.0, 1.0, 2.0, 3.0] {
            buf.extend_from_slice(&v.to_ne_bytes());
        }
        let aligned = test_align_buf(&buf, 4);
        let r = VectorNurbsRef::<f32, 3>::try_from_wire(aligned.as_slice()).unwrap();
        assert_eq!(r.degree(), 1);
        assert_eq!(r.control_points()[1], [1.0, 2.0, 3.0]);
    }

    #[test]
    fn try_from_wire_rejects_axis_mismatch() {
        // Wire says axes_n=4, but type is 3.
        let mut buf = Vec::new();
        buf.extend_from_slice(&[1, 1, 0, 4]);
        buf.extend_from_slice(&4u16.to_ne_bytes());
        buf.extend_from_slice(&2u16.to_ne_bytes());
        // pad to enough bytes so we get past the axis check
        buf.resize(64, 0);
        let aligned = test_align_buf(&buf, 4);
        let result = VectorNurbsRef::<f32, 3>::try_from_wire(aligned.as_slice());
        assert!(matches!(
            result,
            Err(crate::WireError::AxisCountMismatch {
                expected: 3,
                got: 4
            })
        ));
    }

    /// Test-only owner; same shape as `align_buf` in scalar.rs (see Task 9).
    struct AlignedBytes {
        backing: Vec<u32>,
        len: usize,
    }

    impl AlignedBytes {
        fn as_slice(&self) -> &[u8] {
            // SAFETY: `Vec<u32>` is 4-byte aligned; len <= backing.len()*4.
            #[allow(unsafe_code)]
            unsafe {
                core::slice::from_raw_parts(self.backing.as_ptr().cast::<u8>(), self.len)
            }
        }
    }

    fn test_align_buf(data: &[u8], _align: usize) -> AlignedBytes {
        let n = data.len().div_ceil(4);
        let mut backing: Vec<u32> = vec![0; n];
        // SAFETY: backing owns n*4 bytes 4-byte aligned; we write data.len() <= n*4.
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
