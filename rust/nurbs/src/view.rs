use crate::Float;

pub trait NurbsView<T: Float> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[T];
    fn control_points(&self) -> &[T];

    #[inline]
    fn control_point_count(&self) -> usize {
        self.control_points().len()
    }
}

pub trait VectorNurbsView<T: Float, const N: usize> {
    fn degree(&self) -> u8;
    fn knots(&self) -> &[T];
    fn control_points(&self) -> &[[T; N]];

    #[inline]
    fn control_point_count(&self) -> usize {
        self.control_points().len()
    }
}
