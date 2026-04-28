//! Segment types — the product of the iterator. Layer 2 reads these.

use nurbs::{ScalarNurbs, VectorNurbs};

#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {
    Fitted(FittedSegment),
    Arc(ArcSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),
}

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

#[cfg(test)]
mod tests {
    use super::*;
    use nurbs::VectorNurbs;

    #[test]
    fn segment_variants_construct() {
        let xyz = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
            None,
        )
        .expect("valid degree-1 NURBS");
        let f = FittedSegment {
            xyz: xyz.clone(),
            e: None,
            feedrate_mm_s: 100.0,
            degree: 1,
            max_residual_mm: 0.0,
            source: SourceRange {
                start_line: 1,
                end_line: 2,
            },
        };
        let seg_fitted: Segment = Segment::Fitted(f);
        assert!(matches!(seg_fitted, Segment::Fitted(_)));

        let arc = ArcSegment {
            xyz,
            e: None,
            feedrate_mm_s: 100.0,
            source: SourceRange {
                start_line: 3,
                end_line: 3,
            },
        };
        let seg_arc: Segment = Segment::Arc(arc);
        assert!(matches!(seg_arc, Segment::Arc(_)));

        let slot = CornerBlendSlot {
            position: [0.0; 3],
            t_in: [1.0, 0.0, 0.0],
            t_out: [0.0, 1.0, 0.0],
            seg_len_in: 1.0,
            seg_len_out: 1.0,
            tolerance_budget_mm: 0.05,
            default_family: BlendFamily::CubicBezier,
            feedrate_mm_s: 100.0,
            source: SourceRange {
                start_line: 5,
                end_line: 5,
            },
        };
        let seg_slot: Segment = Segment::CornerBlend(slot);
        assert!(matches!(seg_slot, Segment::CornerBlend(_)));

        let jd = JunctionDeviation {
            position: [0.0; 3],
            angle_deg: 90.0,
            feedrate_mm_s: 100.0,
            source: SourceRange {
                start_line: 7,
                end_line: 7,
            },
        };
        let seg_jd: Segment = Segment::Junction(jd);
        assert!(matches!(seg_jd, Segment::Junction(_)));
    }
}
