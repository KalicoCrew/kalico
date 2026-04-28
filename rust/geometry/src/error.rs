//! Error model. `Recovery` for anomalies (`#[non_exhaustive]`), `Fatal` for
//! invariant violations (closed; consumers must handle every variant).

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
    /// G5 with both I,J omitted but no previous G5 in modal chain (chain
    /// broken by intervening non-G5 motion). Per RS274NGC §3.5.5, the
    /// implicit-tangent rule requires `prev_g5_pq` to be set; when it is
    /// not, we reject the line rather than fabricate a tangent.
    G5MissingTangent {
        line_no: u32,
    },
    /// G5.1 issued while the active plane (G17/G18/G19) is not the only
    /// supported plane (XY in Phase 1). The G-code number of the active
    /// plane is included for diagnostic clarity (17 = XY, 18 = XZ, 19 = YZ).
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
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[allow(clippy::no_effect_underscore_binding)]
    fn g5_missing_tangent_constructs() {
        let _r = Recovery::G5MissingTangent { line_no: 42 };
    }

    #[test]
    #[allow(clippy::no_effect_underscore_binding)]
    fn g5_plane_mismatch_constructs() {
        let _r = Recovery::G5PlaneMismatch {
            line_no: 42,
            active_plane_g_code: 18,
        };
    }
}
