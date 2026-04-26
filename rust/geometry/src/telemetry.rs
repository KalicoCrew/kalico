//! Observability events emitted to the consumer-supplied closure sink.

use crate::Recovery;

#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum TelemetryEvent {
    LayerChange { layer: Option<u32>, line_no: u32 },
    ToolChange { tool: u32, line_no: u32 },
    Retraction { e_delta_mm: f64, line_no: u32 },
    WindowFlush { run_vertex_count: u32, line_no: u32 },
    Recovery(Recovery),
    FitObservation {
        residual_mm: f64,
        tolerance_mm: f64,
        run_vertex_count: u32,
        piece_count: u32,
        degree: u8,
    },
}
