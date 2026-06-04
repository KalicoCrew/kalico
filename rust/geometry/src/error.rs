use crate::SourceRange;

#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Recovery {
    UnrecognizedCommand {
        line_no: u32,
        head: String,
    },
    MalformedParams {
        line_no: u32,
        raw: String,
    },
    WindowCapHit {
        source: SourceRange,
        run_vertex_count: u32,
    },
    DegenerateSlotFallback {
        line_no: u32,
        reason: SlotDegeneracy,
    },
    ToleranceExceeded {
        source: SourceRange,
        actual_mm: f64,
        budget_mm: f64,
    },
    LspiaNotConverged {
        source: SourceRange,
        last_update_mm: f64,
    },
    G5MissingTangent {
        line_no: u32,
    },
    G5PlaneMismatch {
        line_no: u32,
        active_plane_g_code: u32,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SlotDegeneracy {
    BacktrackingCorner,
    ZeroIncidentLength,
    ColinearTangents,
}

#[derive(Debug)]
pub enum Fatal {
    Internal(Box<InternalDetails>),
    UnsupportedGcode {
        line_no: u32,
        gcode_kind: &'static str,
    },
    HelicalExtrusionUnsupported {
        line_no: u32,
    },
}

#[derive(Debug)]
pub struct InternalDetails {
    pub kind: InternalKind,
    pub context: String,
    pub backtrace: std::backtrace::Backtrace,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InternalKind {
    NonMonotoneKnotVector,
    NaNDetected { stage: &'static str },
    KnotInsertionFailed,
    BasisMatrixSingular,
    DegreeOutOfBounds,
    CubicSegmentInvariantViolation { reason: &'static str },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeometryError {
    UnsupportedGcode { gcode_kind: &'static str },
    HelicalExtrusionUnsupported,
    NotSinglePieceCubic { reason: &'static str },
    EModeInvariantViolation { reason: &'static str },
    ZeroMotion,
}

#[cfg(test)]
mod tests;
