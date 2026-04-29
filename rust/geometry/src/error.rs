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
    /// G5/G5.1 with simultaneous XY + Z + E motion ("helical extrusion") —
    /// design-rejected per CLAUDE.md (extrusion couples to XY motion only).
    ///
    /// **Currently unused in-tree:** the live pipeline now surfaces helical
    /// extrusion as `Fatal::HelicalExtrusionUnsupported` (round-5 review fix:
    /// reduce-stage commits modal state before the pipeline classifies, so a
    /// recoverable rejection would let subsequent G5s start from the rejected
    /// move's endpoint). Variant retained because `Recovery` is
    /// `#[non_exhaustive]` and future legacy/test paths (Step-13 compat layer
    /// or test fixtures normalizing helical inputs) may want Recovery
    /// semantics.
    #[allow(dead_code)]
    HelicalExtrusionUnsupported {
        line_no: u32,
    },
    /// Live pipeline received G0/G1/G2/G3 — must be normalized via Step-13
    /// compat layer first.
    UnsupportedGcode {
        line_no: u32,
        gcode_kind: &'static str,
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
    /// Live pipeline received an unsupported G-code (G0/G1/G2/G3). Fail-closed
    /// because reduce-stage didn't update modal state from the rejected
    /// command, so subsequent G5 commands would emit cubic segments from
    /// stale position. The caller must abort the file and re-process via
    /// the Step-13 compatibility layer (offline G-code normalizer) before
    /// feeding the live pipeline.
    UnsupportedGcode {
        line_no: u32,
        gcode_kind: &'static str,
    },
    /// Live pipeline received a G5/G5.1 with helical extrusion (XY+Z+E or
    /// Z+E motion in one segment). Per CLAUDE.md design intent, extrusion
    /// couples to XY motion only; combined Z + E motion is design-rejected,
    /// not just unsupported. Fail-closed because reduce-stage already
    /// committed `state.position`/`state.e`/`state.prev_g5_pq` before the
    /// pipeline classified — any continuation would emit subsequent
    /// segments from the rejected move's endpoint.
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
    /// `CubicSegment::try_new` invariant violation — single-piece cubic or
    /// E-mode-fields contract broken. Pipeline didn't validate before
    /// constructing.
    CubicSegmentInvariantViolation {
        reason: &'static str,
    },
}

/// Errors returned by `CubicSegment::try_new` invariant checks. The pipeline
/// translates these to either `Recovery` items (for user-facing surfacing —
/// `HelicalExtrusionUnsupported`, `UnsupportedGcode`) or `Fatal::Internal`
/// (for genuine invariant violations — `NotSinglePieceCubic`, `EModeInvariantViolation`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum GeometryError {
    /// Live pipeline received G0/G1/G2/G3; map to `Recovery::UnsupportedGcode`.
    UnsupportedGcode { gcode_kind: &'static str },
    /// Helical extrusion (XY+Z+E in same segment); map to
    /// `Recovery::HelicalExtrusionUnsupported`.
    HelicalExtrusionUnsupported,
    /// `xyz` not single-piece cubic; map to `Fatal::Internal(InternalKind::CubicSegmentInvariantViolation { ... })`.
    NotSinglePieceCubic { reason: &'static str },
    /// E-mode/E-fields invariant violated; map to `Fatal::Internal`.
    EModeInvariantViolation { reason: &'static str },
    /// Zero-motion segment (all deltas below thresholds). Caller should drop
    /// without emitting (no Recovery / Fatal — silent skip).
    ZeroMotion,
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
