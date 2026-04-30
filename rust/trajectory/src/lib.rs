//! Layer 3 trajectory transformation crate. Pre-bakes time-reparameterization,
//! smooth-shaper convolution, and beta-medium outer iteration onto NURBS segments.

mod beta;
mod e_independent;
pub mod fit;
mod kernel;
mod pad;
mod parallel;
mod partition;
pub mod peak;
mod reparam;
mod shaper;

// ---------------------------------------------------------------------------
// Input types
// ---------------------------------------------------------------------------

/// Top-level input to `shape_batch`.
#[derive(Debug)]
pub struct ShapeBatchInput<'a> {
    /// Segments to shape — must be non-empty.
    pub segments: &'a [ShapeSegmentInput<'a>],
    /// Grid strategy forwarded to `temporal::multi::plan_batch`.
    pub grid_strategy: temporal::multi::GridStrategy,
    /// Worker thread count forwarded to `temporal::multi::plan_batch`.
    pub worker_threads: usize,
    /// Per-axis shaper configuration.
    pub shaper: ShaperConfig,
    /// L-infinity tolerance for the C1-constrained refit (mm).
    pub fit_tolerance_mm: f64,
    /// Maximum number of beta-medium outer iterations.
    pub beta_max_iters: u8,
    /// Convergence ratio threshold for beta-medium iteration.
    pub beta_convergence_ratio: f64,
    /// Extruder axis limits (for independent-E scheduling).
    pub e_limits: ELimits,
}

/// Per-segment input to `shape_batch`.
#[derive(Debug, Clone, Copy)]
pub struct ShapeSegmentInput<'a> {
    /// Temporal (Layer 2) input for this segment.
    pub temporal: temporal::multi::SegmentInput<'a>,
    /// E-axis mode classification.
    pub e_mode: geometry::segment::EMode,
    /// Extrusion ratio (mm E per mm XY arc-length). Meaningful when
    /// `e_mode == CoupledToXy`; zero for `Travel`; unused for `Independent`.
    pub extrusion_per_xy_mm: f64,
    /// Independent E-axis NURBS, present iff `e_mode == Independent`.
    pub e_independent: Option<&'a nurbs::ScalarNurbs<f64>>,
    /// Feedrate from the source G-code (mm/s).
    pub feedrate_mm_s: f64,
}

/// Extruder axis dynamic limits for independent-E scheduling.
#[derive(Debug, Clone, Copy)]
pub struct ELimits {
    /// Maximum E velocity (mm/s).
    pub v_max: f64,
    /// Maximum E acceleration (mm/s^2).
    pub a_max: f64,
}

/// Per-axis shaper configuration for the batch.
#[derive(Debug, Clone)]
pub struct ShaperConfig {
    /// X-axis shaper (required — no passthrough).
    pub x: RequiredShaper,
    /// Y-axis shaper (required — no passthrough).
    pub y: RequiredShaper,
    /// Z-axis shaper (default: passthrough).
    pub z: AxisShaper,
}

/// Shaper family for a required axis (X or Y). No passthrough variant.
#[derive(Debug, Clone, Copy)]
pub enum RequiredShaper {
    /// Smooth ZV shaper at the given resonance frequency.
    SmoothZv { frequency_hz: f64 },
    /// Smooth MZV shaper at the given resonance frequency.
    SmoothMzv { frequency_hz: f64 },
}

/// Shaper family for an optional axis (Z). Includes passthrough.
#[derive(Debug, Clone, Copy)]
pub enum AxisShaper {
    /// Smooth ZV shaper at the given resonance frequency.
    SmoothZv { frequency_hz: f64 },
    /// Smooth MZV shaper at the given resonance frequency.
    SmoothMzv { frequency_hz: f64 },
    /// No shaping — axis passes through unchanged.
    Passthrough,
}

// ---------------------------------------------------------------------------
// Output types
// ---------------------------------------------------------------------------

/// Output of `shape_batch`.
#[derive(Debug)]
pub struct ShapeBatchOutput {
    /// One `ShapedSegment` per input segment (same order).
    pub segments: Vec<ShapedSegment>,
    /// Number of beta-medium outer iterations performed.
    pub beta_iters: u8,
    /// Joining status from the final temporal solve.
    pub temporal_status: temporal::multi::JoiningStatus,
    /// Present if any segment's post-shape peak acceleration exceeds the
    /// machine limit after `beta_max_iters`.
    pub beta_warning: Option<BetaWarning>,
}

/// Diagnostic: beta-medium iteration did not fully converge.
#[derive(Debug, Clone)]
pub struct BetaWarning {
    /// Worst post-shape peak/limit ratio across all segments.
    pub worst_ratio: f64,
    /// Indices of segments whose post-shape peak exceeds the machine limit.
    pub segments_exceeding: Vec<usize>,
}

/// A fully shaped segment — per-axis scalar NURBS in the time domain, with
/// E-mode metadata for MCU dispatch.
#[derive(Debug, Clone)]
pub struct ShapedSegment {
    /// Per-axis shaped NURBS: `[X(t), Y(t), Z(t)]`.
    pub axes: [nurbs::ScalarNurbs<f64>; 3],
    /// E-axis mode, forwarded from input.
    pub e_mode: geometry::segment::EMode,
    /// Extrusion ratio, forwarded from input.
    pub extrusion_per_xy_mm: f64,
    /// Independent E-axis NURBS (owned), present iff `e_mode == Independent`.
    pub e_independent: Option<nurbs::ScalarNurbs<f64>>,
    /// Start time of this segment in the batch timeline (seconds).
    pub t_start: f64,
    /// End time of this segment in the batch timeline (seconds).
    pub t_end: f64,
}

// ---------------------------------------------------------------------------
// Error types
// ---------------------------------------------------------------------------

/// Errors from `shape_batch`.
#[derive(Debug, thiserror::Error)]
pub enum ShapeError {
    /// Temporal batch planning (Layer 2) failed.
    #[error("temporal batch error: {0}")]
    TemporalBatch(#[from] temporal::multi::BatchError),
    /// Temporal joining did not converge.
    #[error("temporal joining: {0:?}")]
    TemporalJoining(temporal::multi::JoiningStatus),
    /// A single segment was unsolvable by the temporal solver.
    #[error("segment {index} unsolvable: {status:?}")]
    SegmentUnsolvable {
        index: usize,
        status: temporal::SolveStatus,
    },
    /// C1 refit or arc-length fit failed on a segment.
    #[error("fit failure on segment {index}: {detail:?}")]
    FitFailure {
        index: usize,
        detail: nurbs::algebra::FitError,
    },
    /// Algebra operation (add, multiply, convolve, compose, restrict) failed.
    #[error("algebra error on segment {index}: {detail}")]
    Algebra {
        index: usize,
        detail: nurbs::AlgebraError,
    },
    /// Input segment buffer was empty.
    #[error("empty segment buffer")]
    EmptySegments,
}

// ---------------------------------------------------------------------------
// Entry point (stub)
// ---------------------------------------------------------------------------

/// Shape a batch of segments: time-reparameterize, convolve with per-axis
/// smooth shaper kernels, and iterate the beta-medium loop to convergence.
///
/// # Errors
/// Returns `ShapeError::EmptySegments` if `input.segments` is empty. Other
/// error variants will be populated as the implementation is filled in.
pub fn shape_batch(input: &ShapeBatchInput<'_>) -> Result<ShapeBatchOutput, ShapeError> {
    if input.segments.is_empty() {
        return Err(ShapeError::EmptySegments);
    }
    todo!("shape_batch implementation")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn shape_batch_rejects_empty_segments() {
        let input = ShapeBatchInput {
            segments: &[],
            grid_strategy: temporal::multi::GridStrategy::Fixed(100),
            worker_threads: 1,
            shaper: ShaperConfig {
                x: RequiredShaper::SmoothZv {
                    frequency_hz: 180.0,
                },
                y: RequiredShaper::SmoothMzv {
                    frequency_hz: 120.0,
                },
                z: AxisShaper::Passthrough,
            },
            fit_tolerance_mm: 0.001,
            beta_max_iters: 5,
            beta_convergence_ratio: 1.02,
            e_limits: ELimits {
                v_max: 100.0,
                a_max: 50_000.0,
            },
        };
        let result = shape_batch(&input);
        assert!(matches!(result, Err(ShapeError::EmptySegments)));
    }
}
