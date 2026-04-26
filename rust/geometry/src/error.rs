//! Error model. `Recovery` for anomalies (`#[non_exhaustive]`), `Fatal` for
//! invariant violations (closed; consumers must handle every variant).

use crate::SourceRange;

#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Recovery {
    UnrecognizedCommand { line_no: u32, head: String },
    MalformedParams { line_no: u32, raw: String },
    WindowCapHit { source: SourceRange, run_vertex_count: u32 },
    DegenerateSlotFallback { line_no: u32, reason: SlotDegeneracy },
    ToleranceExceeded { source: SourceRange, actual_mm: f64, budget_mm: f64 },
    LspiaNotConverged { source: SourceRange, last_update_mm: f64 },
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
    NaNDetected,
    KnotInsertionFailed,
    BasisMatrixSingular,
    DegreeOutOfBounds,
}
