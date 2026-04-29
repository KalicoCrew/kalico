//! Segment types — the product of the iterator. Layer 2 reads these.

use nurbs::{ScalarNurbs, VectorNurbs};

#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {
    /// Live-pipeline cubic Bézier segment with E-mode classification. Produced by
    /// `reduce.rs` from G5/G5.1 input.
    Cubic(CubicSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),

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

/// Live-pipeline cubic-Bézier segment. Single-piece cubic Bézier in `xyz` (degree 3,
/// 4 control points, no weights, clamped knot vector). E classification per `EMode`.
#[derive(Debug, Clone, PartialEq)]
pub struct CubicSegment {
    /// XYZ trajectory in u-domain. **Invariant** (enforced by `try_new`): single-piece
    /// cubic Bézier — degree 3, 4 control points, no weights, clamped knot vector.
    pub xyz: VectorNurbs<f64, 3>,
    pub e_mode: EMode,
    /// Valid when `e_mode == CoupledToXy`. Signed: negative for retract-during-XY-motion
    /// / wipe / coast. Zero when `e_mode == Travel`. Unused when `e_mode == Independent`
    /// (use `e_independent` instead).
    pub extrusion_per_xy_mm: f64,
    /// `Some(curve)` iff `e_mode == Independent`; carries the E trajectory for
    /// retraction / prime / filament-change segments.
    pub e_independent: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
    /// `None` on un-split segments; `Some` on splitter output.
    pub split_info: Option<SplitInfo>,
}

impl CubicSegment {
    /// Construct a `CubicSegment`, validating invariants. Returns `Err` on:
    /// - `NotSinglePieceCubic`: xyz is not single-piece cubic (degree != 3,
    ///   != 4 CPs, has weights, or knots are not clamped `[0,0,0,0,1,1,1,1]`).
    /// - `EModeInvariantViolation`: `e_mode` and the corresponding fields disagree.
    pub fn try_new(
        xyz: VectorNurbs<f64, 3>,
        e_mode: EMode,
        extrusion_per_xy_mm: f64,
        e_independent: Option<ScalarNurbs<f64>>,
        feedrate_mm_s: f64,
        source: SourceRange,
        split_info: Option<SplitInfo>,
    ) -> Result<Self, crate::GeometryError> {
        // xyz must be single-piece cubic Bézier.
        if xyz.degree() != 3 {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "degree != 3",
            });
        }
        if xyz.control_points().len() != 4 {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "control_points.len() != 4",
            });
        }
        if xyz.weights().is_some() {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "weights present (must be polynomial)",
            });
        }
        let expected_knots: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        if xyz.knots() != expected_knots.as_slice() {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "knot vector not clamped [0,0,0,0,1,1,1,1]",
            });
        }

        // EMode invariants.
        match e_mode {
            EMode::CoupledToXy => {
                if e_independent.is_some() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "CoupledToXy must have e_independent: None",
                    });
                }
            }
            EMode::Travel => {
                if extrusion_per_xy_mm != 0.0 {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Travel must have extrusion_per_xy_mm == 0.0",
                    });
                }
                if e_independent.is_some() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Travel must have e_independent: None",
                    });
                }
            }
            EMode::Independent => {
                if e_independent.is_none() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Independent must have e_independent: Some(_)",
                    });
                }
                if extrusion_per_xy_mm != 0.0 {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Independent must have extrusion_per_xy_mm == 0.0",
                    });
                }
            }
        }

        Ok(Self {
            xyz,
            e_mode,
            extrusion_per_xy_mm,
            e_independent,
            feedrate_mm_s,
            source,
            split_info,
        })
    }
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
    /// (`cp_polygon_length` and midpoint parametric speed both below thresholds).
    /// Helical extrusion (XYZ + E) is rejected upstream; never produces `Independent`
    /// in the live pipeline.
    Independent,
}

/// Sub-segment provenance, populated by `split_segment_to_cap` (`geometry::splitter`).
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
    fn cubic_variant_constructs() {
        let xyz = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
            None,
        )
        .expect("valid cubic");
        let cs = CubicSegment::try_new(
            xyz,
            EMode::Travel,
            0.0,
            None,
            100.0,
            SourceRange { start_line: 1, end_line: 1 },
            None,
        )
        .expect("valid travel");
        let seg: Segment = Segment::Cubic(cs);
        assert!(matches!(seg, Segment::Cubic(_)));
    }
}
