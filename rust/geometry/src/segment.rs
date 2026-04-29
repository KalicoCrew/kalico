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

/// E-axis classification per CLAUDE.md feature scope. `CubicSegment::try_new`
/// applies the §6.1 classification rules to derive this from raw `(ΔX, ΔY, ΔZ, ΔE)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EMode {
    /// Extrusion proportional to actual XY shaped motion: `e_actual(t) = ratio × ∫|v_xy| dt`.
    /// `extrusion_per_xy_mm` is nonzero and signed (positive for normal extrusion;
    /// negative for retract-during-XY-motion / wipe / coast). Used for moves with
    /// `ΔXY > ε_xyz`, `ΔZ ≤ ε_z`, and `abs(ΔE) > ε_e`.
    CoupledToXy,
    /// Travel move: XY motion with no extrusion. Equivalent to `CoupledToXy` with
    /// `extrusion_per_xy_mm = 0`. Modeled distinctly for clarity in logs/telemetry
    /// and to allow a future plan layer to skip per-sample E integration when the
    /// ratio is definitionally zero.
    Travel,
    /// E motion not coupled to XY: own E NURBS carries the trajectory in time.
    /// In 7-pre's live pipeline, `Independent` always implies null `xyz` motion
    /// (cp_polygon_length and midpoint parametric speed both below thresholds).
    /// Helical extrusion (XYZ + E) is rejected upstream; never produces `Independent`
    /// in the live pipeline.
    Independent,
}

/// Sub-segment provenance, populated by `split_segment_to_cap` (geometry::splitter).
/// `None` when the segment was not split.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitInfo {
    /// 0-indexed position of this child within the parent's sub-segment sequence.
    pub sub_index: u32,
    /// Total sub-segments produced from the parent. May be < the originally-planned
    /// `k` if epsilon-filtering at splitter step 6 dropped near-boundary breakpoints.
    pub sub_count: u32,
    /// Arc-length range this sub-segment occupies in the parent's arc-length domain.
    /// Computed at split time by querying the parent's arc-length table at the child's
    /// `xyz.u_start` and `xyz.u_end`.
    pub s_lo_mm: f64,
    pub s_hi_mm: f64,
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
