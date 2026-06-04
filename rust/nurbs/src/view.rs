//! Read-only NURBS view traits. Eval algorithms are generic over these so the
//! same code works against owned (host) and borrowed (MCU) representations.
//! See spec §Substrate / NURBS data model.

use crate::Float;

/// Read-only access to a scalar NURBS curve.
pub trait NurbsView<T: Float> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[T];
    fn control_points(&self) -> &[T];

    /// Number of control points. Convenience derived from slice length.
    #[inline]
    fn control_point_count(&self) -> usize {
        self.control_points().len()
    }
}

/// Read-only access to a vector NURBS curve in R^N.
pub trait VectorNurbsView<T: Float, const N: usize> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[T];
    fn control_points(&self) -> &[[T; N]];

    #[inline]
    fn control_point_count(&self) -> usize {
        self.control_points().len()
    }
}
