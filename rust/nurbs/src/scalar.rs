//! Scalar (1D) NURBS types: ScalarNurbs (owned, host) and ScalarNurbsRef (borrowed).

use crate::{ConstructError, Float, NurbsView, MAX_DEGREE};

/// Owned, heap-backed scalar NURBS. Host-only.
///
/// Construction validates all spec §Substrate invariants. After construction,
/// the data is trusted; eval algorithms only `debug_assert` invariants.
#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct ScalarNurbs<T: Float> {
    degree: u8,
    knots: Vec<T>,
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
        Ok(Self { degree, knots, control_points, weights })
    }

    pub fn degree(&self) -> u8 { self.degree }
    pub fn knots(&self) -> &[T] { &self.knots }
    pub fn control_points(&self) -> &[T] { &self.control_points }
    pub fn weights(&self) -> Option<&[T]> { self.weights.as_deref() }

    /// Cheap projection to a borrowed view.
    #[inline]
    pub fn as_view(&self) -> ScalarNurbsRef<'_, T> {
        ScalarNurbsRef {
            degree: self.degree,
            knots: &self.knots,
            control_points: &self.control_points,
            weights: self.weights.as_deref(),
        }
    }

    /// Consume self into raw parts. Used by host pre-bake pipelines that
    /// build new NURBS by transformation.
    pub fn into_parts(self) -> (u8, Vec<T>, Vec<T>, Option<Vec<T>>) {
        (self.degree, self.knots, self.control_points, self.weights)
    }
}

#[cfg(feature = "host")]
impl<T: Float> NurbsView<T> for ScalarNurbs<T> {
    #[inline] fn degree(&self) -> u8 { self.degree }
    #[inline] fn knots(&self) -> &[T] { &self.knots }
    #[inline] fn control_points(&self) -> &[T] { &self.control_points }
    #[inline] fn weights(&self) -> Option<&[T]> { self.weights.as_deref() }
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
        Ok(Self { degree, knots, control_points, weights })
    }

    pub fn degree(&self) -> u8 { self.degree }
    pub fn knots(&self) -> &[T] { self.knots }
    pub fn control_points(&self) -> &[T] { self.control_points }
    pub fn weights(&self) -> Option<&[T]> { self.weights }
}

impl<'a, T: Float> NurbsView<T> for ScalarNurbsRef<'a, T> {
    #[inline] fn degree(&self) -> u8 { self.degree }
    #[inline] fn knots(&self) -> &[T] { self.knots }
    #[inline] fn control_points(&self) -> &[T] { self.control_points }
    #[inline] fn weights(&self) -> Option<&[T]> { self.weights }
}

/// Shared validation. See spec §Substrate / Validation rules.
pub(crate) fn validate<T: Float>(
    degree: u8,
    knots: &[T],
    control_point_count: usize,
    weights: Option<&[T]>,
) -> Result<(), ConstructError> {
    if (degree as usize) > MAX_DEGREE {
        return Err(ConstructError::DegreeExceeded { actual: degree, max: MAX_DEGREE as u8 });
    }
    let p = degree as usize;
    let expected_knot_count = control_point_count + p + 1;
    if knots.len() != expected_knot_count {
        return Err(ConstructError::KnotCountMismatch {
            expected: expected_knot_count, got: knots.len(),
        });
    }
    if knots.len() < 2 * (p + 1) {
        // not enough knots for clamped open of this degree
        return Err(ConstructError::KnotCountMismatch {
            expected: 2 * (p + 1), got: knots.len(),
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
                expected: control_point_count, got: w.len(),
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

#[cfg(all(test, feature = "host"))]
mod tests {
    use super::*;
    use crate::ConstructError;

    fn linear_curve() -> ScalarNurbs<f64> {
        // Degree-1 NURBS, 2 control points, knots {0,0,1,1}.
        ScalarNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            None,
        ).unwrap()
    }

    #[test]
    fn try_new_accepts_valid_linear() {
        let curve = linear_curve();
        assert_eq!(curve.degree(), 1);
        assert_eq!(curve.control_points(), &[0.0, 1.0]);
    }

    #[test]
    fn try_new_rejects_degree_exceeded() {
        let result = ScalarNurbs::<f64>::try_new(
            21,
            vec![0.0; 23],
            vec![0.0; 1],
            None,
        );
        assert!(matches!(result, Err(ConstructError::DegreeExceeded { actual: 21, max: 20 })));
    }

    #[test]
    fn try_new_rejects_knot_count_mismatch() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 1.0],         // 3 knots, but 2 cps + 1 + 1 = 4 expected
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotCountMismatch { .. })));
    }

    #[test]
    fn try_new_rejects_unclamped_start() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.5, 1.0, 1.0],    // not clamped at start
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
    }

    #[test]
    fn try_new_rejects_unclamped_end() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 0.5, 1.0],    // not clamped at end
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotClamped)));
    }

    #[test]
    fn try_new_rejects_non_monotone_knots() {
        let result = ScalarNurbs::<f64>::try_new(
            2,
            vec![0.0, 0.0, 0.0, 0.4, 0.3, 1.0, 1.0, 1.0],  // 0.3 < 0.4
            vec![0.0, 0.5, 1.0, 1.5, 2.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::KnotsNotMonotone)));
    }

    #[test]
    fn try_new_rejects_degenerate_knot_range() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 0.0, 0.0],
            vec![0.0, 1.0],
            None,
        );
        assert!(matches!(result, Err(ConstructError::DegenerateKnotRange)));
    }

    #[test]
    fn try_new_rejects_weight_count_mismatch() {
        let result = ScalarNurbs::<f64>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 1.0],
            Some(vec![1.0]),         // 1 weight for 2 cps
        );
        assert!(matches!(result, Err(ConstructError::WeightCountMismatch { .. })));
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
}
