//! Move classification and CubicSegment construction.

use compat::collinear::to_collinear_bezier;
use geometry::segment::{CubicSegment, EMode, SourceRange};
use nurbs::VectorNurbs;

#[derive(Debug)]
pub enum MoveClass {
    /// XY travel (no Z, no E). Includes pure-X and pure-Y.
    XyTravel,
    /// Z-only move.
    ZOnly,
}

#[derive(Debug)]
pub struct ClassifiedMove {
    pub segment: CubicSegment,
    pub class: MoveClass,
}

/// Classify a G1-style delta move and construct a `CubicSegment`.
///
/// Returns `Err` if `de != 0` (Phase 2 does not support extrusion) or
/// if the move has zero displacement.
pub fn classify_and_build(
    start: [f64; 3],
    dx: f64,
    dy: f64,
    dz: f64,
    de: f64,
    feedrate_mm_s: f64,
) -> Result<ClassifiedMove, ClassifyError> {
    if de.abs() > 1e-9 {
        return Err(ClassifyError::ExtrusionNotSupported);
    }
    let end = [start[0] + dx, start[1] + dy, start[2] + dz];
    let has_xy = dx.abs() > 1e-9 || dy.abs() > 1e-9;
    let has_z = dz.abs() > 1e-9;

    if !has_xy && !has_z {
        return Err(ClassifyError::ZeroDisplacement);
    }

    let class = if has_xy { MoveClass::XyTravel } else { MoveClass::ZOnly };

    let cps = to_collinear_bezier(start, end);
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
        None,
    )
    .map_err(|e| ClassifyError::NurbsConstruction(format!("{e:?}")))?;

    let segment = CubicSegment::try_new(
        xyz,
        EMode::Travel,
        0.0,
        None,
        feedrate_mm_s,
        SourceRange { start_line: 0, end_line: 0 },
        None,
    )
    .map_err(|e| ClassifyError::SegmentConstruction(format!("{e:?}")))?;

    Ok(ClassifiedMove { segment, class })
}

#[derive(Debug)]
pub enum ClassifyError {
    ExtrusionNotSupported,
    ZeroDisplacement,
    NurbsConstruction(String),
    SegmentConstruction(String),
}

impl std::fmt::Display for ClassifyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::ExtrusionNotSupported => write!(f, "extrusion not yet supported (Phase 2)"),
            Self::ZeroDisplacement => write!(f, "zero displacement move"),
            Self::NurbsConstruction(e) => write!(f, "NURBS construction: {e}"),
            Self::SegmentConstruction(e) => write!(f, "segment construction: {e}"),
        }
    }
}

impl std::error::Error for ClassifyError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn xy_travel_classifies_correctly() {
        let m = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 0.0, 100.0).unwrap();
        assert!(matches!(m.class, MoveClass::XyTravel));
        assert_eq!(m.segment.e_mode, EMode::Travel);
        assert_eq!(m.segment.feedrate_mm_s, 100.0);
        let cps = m.segment.xyz.control_points();
        assert_eq!(cps.len(), 4);
        assert_eq!(cps[0], [0.0, 0.0, 0.0]);
        assert!((cps[3][0] - 10.0).abs() < 1e-12);
    }

    #[test]
    fn z_only_classifies_correctly() {
        let m = classify_and_build([0.0, 0.0, 5.0], 0.0, 0.0, 5.0, 0.0, 50.0).unwrap();
        assert!(matches!(m.class, MoveClass::ZOnly));
    }

    #[test]
    fn extrusion_rejected() {
        let r = classify_and_build([0.0; 3], 10.0, 0.0, 0.0, 1.0, 100.0);
        assert!(matches!(r, Err(ClassifyError::ExtrusionNotSupported)));
    }

    #[test]
    fn zero_displacement_rejected() {
        let r = classify_and_build([0.0; 3], 0.0, 0.0, 0.0, 0.0, 100.0);
        assert!(matches!(r, Err(ClassifyError::ZeroDisplacement)));
    }
}
