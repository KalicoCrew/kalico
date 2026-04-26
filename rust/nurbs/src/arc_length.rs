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
}
