use crate::{ConstructError, Float, MAX_DEGREE, NurbsView};

#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct ScalarNurbs<T: Float> {
    degree: u8,
    knots: crate::knot::KnotVector<T>,
    control_points: Vec<T>,
}

#[cfg(feature = "host")]
impl<T: Float> ScalarNurbs<T> {
    pub fn try_new(
        degree: u8,
        knots: Vec<T>,
        control_points: Vec<T>,
    ) -> Result<Self, ConstructError> {
        validate(degree, &knots, control_points.len())?;
        let knot_vector = crate::knot::KnotVector::try_new(knots)
            .expect("validate already ensured monotone + length");
        Ok(Self {
            degree,
            knots: knot_vector,
            control_points,
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

    #[inline]
    #[must_use]
    pub fn as_view(&self) -> ScalarNurbsRef<'_, T> {
        ScalarNurbsRef {
            degree: self.degree,
            knots: self.knots.as_slice(),
            control_points: &self.control_points,
        }
    }

    #[must_use]
    pub fn into_parts(self) -> (u8, Vec<T>, Vec<T>) {
        (self.degree, self.knots.into_inner(), self.control_points)
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
}

#[derive(Debug, Clone, Copy)]
pub struct ScalarNurbsRef<'a, T: Float> {
    pub(crate) degree: u8,
    pub(crate) knots: &'a [T],
    pub(crate) control_points: &'a [T],
}

impl<'a, T: Float> ScalarNurbsRef<'a, T> {
    pub fn try_new(
        degree: u8,
        knots: &'a [T],
        control_points: &'a [T],
    ) -> Result<Self, ConstructError> {
        validate(degree, knots, control_points.len())?;
        Ok(Self {
            degree,
            knots,
            control_points,
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
}

pub(crate) fn validate<T: Float>(
    degree: u8,
    knots: &[T],
    control_point_count: usize,
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
        return Err(ConstructError::KnotCountMismatch {
            expected: 2 * (p + 1),
            got: knots.len(),
        });
    }

    let start = knots[0];
    for k in &knots[1..=p] {
        if *k != start {
            return Err(ConstructError::KnotsNotClamped);
        }
    }
    let last_idx = knots.len() - 1;
    let end = knots[last_idx];
    for k in &knots[last_idx - p..last_idx] {
        if *k != end {
            return Err(ConstructError::KnotsNotClamped);
        }
    }

    for window in knots.windows(2) {
        if window[1] < window[0] {
            return Err(ConstructError::KnotsNotMonotone);
        }
    }

    if !(end > start) {
        return Err(ConstructError::DegenerateKnotRange);
    }

    Ok(())
}

use crate::{
    WireError,
    wire::{FORMAT_VERSION_V1, SCALAR_HEADER_BYTES},
};

impl<'a> ScalarNurbsRef<'a, f32> {
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
        if buf[2] != 0 {
            return Err(WireError::WeightsUnsupported);
        }
        let knot_count = u16::from_ne_bytes([buf[4], buf[5]]) as usize;
        let cp_count = u16::from_ne_bytes([buf[6], buf[7]]) as usize;

        let knots_bytes = knot_count * core::mem::size_of::<f32>();
        let cps_bytes = cp_count * core::mem::size_of::<f32>();
        let total = SCALAR_HEADER_BYTES + knots_bytes + cps_bytes;
        if buf.len() < total {
            return Err(WireError::TruncatedBuffer {
                expected_len: total,
                got: buf.len(),
            });
        }

        // SAFETY: alignment checked above; lengths checked above; T = f32 has
        // no invalid bit patterns for any 4-byte sequence.
        #[allow(unsafe_code)]
        let (knots, cps) = unsafe {
            let knots_ptr = buf.as_ptr().add(SCALAR_HEADER_BYTES).cast::<f32>();
            let cps_ptr = buf
                .as_ptr()
                .add(SCALAR_HEADER_BYTES + knots_bytes)
                .cast::<f32>();
            let knots = core::slice::from_raw_parts(knots_ptr, knot_count);
            let cps = core::slice::from_raw_parts(cps_ptr, cp_count);
            (knots, cps)
        };

        Self::try_new(degree, knots, cps).map_err(WireError::from)
    }
}

#[cfg(all(test, feature = "host"))]
mod tests;
