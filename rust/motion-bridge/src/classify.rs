use compat::collinear::to_collinear_bezier;
use geometry::segment::{CubicSegment, EMode, SourceRange};
use nurbs::VectorNurbs;

#[derive(Debug)]
pub enum MoveClass {
    XyTravel,
    ZOnly,
}

#[derive(Debug)]
pub struct ClassifiedMove {
    pub segment: CubicSegment,
    pub class: MoveClass,
    pub distance_mm: f64,
}

impl ClassifiedMove {
    #[must_use]
    pub fn nominal_duration(&self) -> f64 {
        if self.segment.feedrate_mm_s <= 0.0 {
            return 0.0;
        }
        self.distance_mm / self.segment.feedrate_mm_s
    }
}

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

    let class = if has_xy {
        MoveClass::XyTravel
    } else {
        MoveClass::ZOnly
    };

    let cps = to_collinear_bezier(start, end);
    let xyz = VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
    )
    .map_err(|e| ClassifyError::NurbsConstruction(format!("{e:?}")))?;

    let segment = CubicSegment::try_new(
        xyz,
        EMode::Travel,
        0.0,
        None,
        feedrate_mm_s,
        SourceRange {
            start_line: 0,
            end_line: 0,
        },
        None,
    )
    .map_err(|e| ClassifyError::SegmentConstruction(format!("{e:?}")))?;

    let distance_mm = (dx * dx + dy * dy + dz * dz).sqrt();

    Ok(ClassifiedMove {
        segment,
        class,
        distance_mm,
    })
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
mod tests;
