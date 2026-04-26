//! `GeometryPipeline`, `Segments`, `Item`. Phase 1 implementation is filled in
//! across Tasks 18-23.

use crate::{Fatal, FitterParams, Recovery, Segment};

#[derive(Debug)]
pub struct GeometryPipeline {
    #[allow(dead_code)]
    params: FitterParams,
}

impl GeometryPipeline {
    #[must_use]
    pub fn new(params: FitterParams) -> Self {
        Self { params }
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum Item {
    Segment(Segment),
    Recovered(Segment, Recovery),
    Fatal(Fatal),
}

#[derive(Debug)]
pub struct Segments<'a> {
    _lifetime: std::marker::PhantomData<&'a ()>,
}

impl Iterator for Segments<'_> {
    type Item = Item;
    fn next(&mut self) -> Option<Item> {
        None
    }
}
