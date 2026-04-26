//! Segment types — the product of the iterator. Layer 2 reads these.

use nurbs::{ScalarNurbs, VectorNurbs};

#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {}

#[derive(Debug, Clone, PartialEq)]
pub struct FittedSegment {
    pub xyz: VectorNurbs<f64, 3>,
    pub e: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub degree: u8,
    pub max_residual_mm: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ArcSegment {
    pub xyz: VectorNurbs<f64, 3>,
    pub e: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CornerBlendSlot {
    pub position: [f64; 3],
    pub t_in: [f64; 3],
    pub t_out: [f64; 3],
    pub seg_len_in: f64,
    pub seg_len_out: f64,
    pub tolerance_budget_mm: f64,
    pub default_family: BlendFamily,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JunctionDeviation {
    pub position: [f64; 3],
    pub angle_deg: f64,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlendFamily {
    CubicBezier,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceRange {
    pub start_line: u32,
    pub end_line: u32,
}
