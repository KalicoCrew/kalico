mod beta;
mod e_independent;
pub mod emit_shaped;
pub mod fit;
mod kernel;
mod pad;
mod parallel;
mod partition;
pub mod peak;
pub mod plan_velocity;
mod refit;
mod reparam;
mod shaper;
pub mod streaming;

pub use emit_shaped::{emit_shaped, EmitSegmentMeta, PerAxisHistory};
pub use pad::EHalo;
pub use plan_velocity::{plan_velocity, PlanInput, PlanSegment, PlanShaper, SafetyMode};

#[derive(Debug)]
pub struct ShapeBatchInput<'a> {
    pub segments: &'a [ShapeSegmentInput<'a>],
    pub grid_strategy: temporal::multi::GridStrategy,
    pub worker_threads: usize,
    pub shaper: ShaperConfig,
    pub fit_tolerance_mm: f64,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub e_limits: ELimits,
    /// Velocity at the batch start (mm/s).
    pub initial_v: f64,
    /// Velocity at the batch end (mm/s).
    pub terminal_v: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ShapeSegmentInput<'a> {
    pub temporal: temporal::multi::SegmentInput<'a>,
    pub e_mode: geometry::segment::EMode,
    /// Extrusion ratio (mm E per mm XY arc-length). Meaningful when `e_mode == CoupledToXy`.
    pub extrusion_per_xy_mm: f64,
    pub e_independent: Option<&'a nurbs::ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ELimits {
    pub v_max: f64,
    pub a_max: f64,
}

#[derive(Debug, Clone)]
pub struct ShaperConfig {
    pub x: RequiredShaper,
    pub y: RequiredShaper,
    pub z: AxisShaper,
}

#[derive(Debug, Clone, Copy)]
pub enum RequiredShaper {
    SmoothZv { frequency_hz: f64 },
    SmoothMzv { frequency_hz: f64 },
}

#[derive(Debug, Clone, Copy)]
pub enum AxisShaper {
    SmoothZv { frequency_hz: f64 },
    SmoothMzv { frequency_hz: f64 },
    Passthrough,
}

#[derive(Debug)]
pub struct ShapeBatchOutput {
    pub segments: Vec<ShapedSegment>,
    pub beta_iters: u8,
    pub temporal_status: temporal::multi::JoiningStatus,
    pub beta_warning: Option<BetaWarning>,
}

#[derive(Debug, Clone)]
pub struct BetaWarning {
    pub worst_ratio: f64,
    pub segments_exceeding: Vec<usize>,
}

#[derive(Debug, Clone)]
pub struct ShapedSegment {
    pub axes: [nurbs::ScalarNurbs<f64>; 3],
    pub e_mode: geometry::segment::EMode,
    pub extrusion_per_xy_mm: f64,
    pub e_independent: Option<nurbs::ScalarNurbs<f64>>,
    pub t_start: f64,
    pub t_end: f64,
}

#[derive(Debug, thiserror::Error)]
pub enum ShapeError {
    #[error("temporal batch error: {0}")]
    TemporalBatch(#[from] temporal::multi::BatchError),
    #[error("temporal joining: {0:?}{1}")]
    TemporalJoining(temporal::multi::JoiningStatus, String),
    #[error("segment {index} unsolvable: {status:?}")]
    SegmentUnsolvable {
        index: usize,
        status: temporal::SolveStatus,
    },
    #[error("fit failure on segment {index}: {detail:?}")]
    FitFailure {
        index: usize,
        detail: nurbs::algebra::FitError,
    },
    #[error("algebra error on segment {index}: {detail}")]
    Algebra {
        index: usize,
        detail: nurbs::AlgebraError,
    },
    #[error("arc-length table error on segment {index}: {detail}")]
    ArcLength { index: usize, detail: String },
    #[error("empty segment buffer")]
    EmptySegments,
    #[error("unsupported shaper configuration: Passthrough on X or Y is not supported")]
    UnsupportedShaperOnXY,
    #[error("unsupported boundary velocity: initial_v and terminal_v must be finite and ≥ 0.0")]
    UnsupportedBoundaryVelocity,
}

pub fn shape_batch(input: &ShapeBatchInput<'_>) -> Result<ShapeBatchOutput, ShapeError> {
    if input.segments.is_empty() {
        return Err(ShapeError::EmptySegments);
    }

    let partition = partition::partition_batch(input.segments, &input.e_limits);

    beta::beta_loop(input, &partition)
}

#[cfg(test)]
mod tests;
