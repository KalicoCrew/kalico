//! Vector NURBS types in R^N: VectorNurbs<T, N> (owned) and VectorNurbsRef<T, N> (borrowed).

use crate::{scalar::validate, ConstructError, Float, VectorNurbsView};

#[cfg(feature = "host")]
#[derive(Debug, Clone, PartialEq)]
pub struct VectorNurbs<T: Float, const N: usize> {
    degree: u8,
    knots: Vec<T>,
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
        Ok(Self { degree, knots, control_points, weights })
    }

    pub fn degree(&self) -> u8 { self.degree }
    pub fn knots(&self) -> &[T] { &self.knots }
    pub fn control_points(&self) -> &[[T; N]] { &self.control_points }
    pub fn weights(&self) -> Option<&[T]> { self.weights.as_deref() }

    #[inline]
    pub fn as_view(&self) -> VectorNurbsRef<'_, T, N> {
        VectorNurbsRef {
            degree: self.degree,
            knots: &self.knots,
            control_points: &self.control_points,
            weights: self.weights.as_deref(),
        }
    }

    pub fn into_parts(self) -> (u8, Vec<T>, Vec<[T; N]>, Option<Vec<T>>) {
        (self.degree, self.knots, self.control_points, self.weights)
    }
}

#[cfg(feature = "host")]
impl<T: Float, const N: usize> VectorNurbsView<T, N> for VectorNurbs<T, N> {
    #[inline] fn degree(&self) -> u8 { self.degree }
    #[inline] fn knots(&self) -> &[T] { &self.knots }
    #[inline] fn control_points(&self) -> &[[T; N]] { &self.control_points }
    #[inline] fn weights(&self) -> Option<&[T]> { self.weights.as_deref() }
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
        Ok(Self { degree, knots, control_points, weights })
    }

    pub fn degree(&self) -> u8 { self.degree }
    pub fn knots(&self) -> &[T] { self.knots }
    pub fn control_points(&self) -> &[[T; N]] { self.control_points }
    pub fn weights(&self) -> Option<&[T]> { self.weights }
}

impl<'a, T: Float, const N: usize> VectorNurbsView<T, N> for VectorNurbsRef<'a, T, N> {
    #[inline] fn degree(&self) -> u8 { self.degree }
    #[inline] fn knots(&self) -> &[T] { self.knots }
    #[inline] fn control_points(&self) -> &[[T; N]] { self.control_points }
    #[inline] fn weights(&self) -> Option<&[T]> { self.weights }
}

#[cfg(all(test, feature = "host"))]
mod tests {
    use super::*;

    fn linear_3d_curve() -> VectorNurbs<f64, 3> {
        VectorNurbs::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 2.0, 3.0]],
            None,
        ).unwrap()
    }

    #[test]
    fn try_new_accepts_valid_linear_3d() {
        let curve = linear_3d_curve();
        assert_eq!(curve.degree(), 1);
        assert_eq!(curve.control_points()[1], [1.0, 2.0, 3.0]);
    }

    #[test]
    fn try_new_rejects_degree_exceeded() {
        let result = VectorNurbs::<f64, 3>::try_new(
            21,
            vec![0.0; 23],
            vec![[0.0; 3]; 1],
            None,
        );
        assert!(matches!(result, Err(crate::ConstructError::DegreeExceeded { .. })));
    }

    #[test]
    fn try_new_rejects_knot_count_mismatch() {
        let result = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0],
            vec![[0.0; 3], [1.0; 3]],
            None,
        );
        assert!(matches!(result, Err(crate::ConstructError::KnotCountMismatch { .. })));
    }

    #[test]
    fn as_view_provides_borrowed_access() {
        let owned = linear_3d_curve();
        let view = owned.as_view();
        assert_eq!(view.degree(), 1);
        assert_eq!(view.control_points()[1], [1.0, 2.0, 3.0]);
    }
}
