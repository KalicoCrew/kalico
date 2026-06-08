use crate::fit::FittedSegment;
use crate::partition::partition_batch;
use crate::{
    AxisShaper, ELimits, RequiredShaper, ShapeBatchInput, ShapeError, ShapeSegmentInput,
    ShaperConfig,
};

pub use crate::beta::{PlanOutput, PlanStats};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyMode {
    TerminalKnown,
    /// Streaming case: the terminal velocity is speculative (decel-to-zero default).
    /// β-medium derates against the worst-case-future bound by tightening the
    /// effective machine accel limit on the trailing region.
    WorstCaseFuture,
}

/// Per-axis shaper for a [`PlanInput`]. Unlike [`ShaperConfig`], allows
/// `None` on every axis — streaming may legitimately plan with passthrough
/// before per-axis shaper config is loaded.
#[derive(Debug, Clone, Copy)]
pub enum PlanShaper {
    SmoothZv { frequency_hz: f64 },
    SmoothMzv { frequency_hz: f64 },
    Passthrough,
}

impl PlanShaper {
    fn into_required(self) -> Result<RequiredShaper, ShapeError> {
        match self {
            Self::SmoothZv { frequency_hz } => Ok(RequiredShaper::SmoothZv { frequency_hz }),
            Self::SmoothMzv { frequency_hz } => Ok(RequiredShaper::SmoothMzv { frequency_hz }),
            Self::Passthrough => Err(ShapeError::UnsupportedShaperOnXY),
        }
    }

    fn into_axis(self) -> AxisShaper {
        match self {
            Self::SmoothZv { frequency_hz } => AxisShaper::SmoothZv { frequency_hz },
            Self::SmoothMzv { frequency_hz } => AxisShaper::SmoothMzv { frequency_hz },
            Self::Passthrough => AxisShaper::Passthrough,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct PlanSegment<'a> {
    pub temporal: temporal::multi::SegmentInput<'a>,
    /// E-axis mode. Used by the partitioning step to identify XY-motion runs.
    pub e_mode: geometry::segment::EMode,
    pub extrusion_per_xy_mm: f64,
    pub e_independent: Option<&'a nurbs::ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
}

#[derive(Debug)]
pub struct PlanInput<'a> {
    pub segments: &'a [PlanSegment<'a>],
    pub grid_strategy: temporal::multi::GridStrategy,
    pub worker_threads: usize,
    /// Per-axis shapers `[X, Y, Z, E]`. E is always passthrough.
    pub kernels: [Option<PlanShaper>; 4],
    pub fit_tolerance_mm: f64,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub e_limits: ELimits,
    pub initial_v: f64,
    pub initial_a: f64,
    pub terminal_v: f64,
    pub safety_mode: SafetyMode,
    /// Axis-wise second derivatives to pin at the first sample of the first fitted
    /// segment. Forwarded verbatim to [`ShapeBatchInput::start_d2_override`].
    pub start_d2_override: Option<[f64; 3]>,
}

///
/// # Errors
///
/// - [`ShapeError::EmptySegments`] — `input.segments` is empty.
/// - [`ShapeError::UnsupportedShaperOnXY`] — X or Y kernel is `None` or `Passthrough`.
/// - [`ShapeError::UnsupportedBoundaryVelocity`] — `initial_v` or `terminal_v` is non-finite or negative.
/// - [`ShapeError::UnsupportedBoundaryAccel`] — `initial_a` is non-finite, or non-zero when `initial_v` is 0.0.
/// - Any error from the underlying β-medium loop.
pub fn plan_velocity(input: &PlanInput<'_>) -> Result<PlanOutput, ShapeError> {
    if input.segments.is_empty() {
        return Err(ShapeError::EmptySegments);
    }

    if !input.initial_v.is_finite() || input.initial_v < 0.0 {
        return Err(ShapeError::UnsupportedBoundaryVelocity);
    }
    if !input.terminal_v.is_finite() || input.terminal_v < 0.0 {
        return Err(ShapeError::UnsupportedBoundaryVelocity);
    }
    if !input.initial_a.is_finite() || (input.initial_v == 0.0 && input.initial_a != 0.0) {
        return Err(ShapeError::UnsupportedBoundaryAccel);
    }

    let shaper = build_shaper_config(&input.kernels)?;
    let segments: Vec<ShapeSegmentInput<'_>> = input
        .segments
        .iter()
        .map(|s| ShapeSegmentInput {
            temporal: s.temporal,
            e_mode: s.e_mode,
            extrusion_per_xy_mm: s.extrusion_per_xy_mm,
            e_independent: s.e_independent,
            feedrate_mm_s: s.feedrate_mm_s,
        })
        .collect();

    let shape_input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: input.grid_strategy,
        worker_threads: input.worker_threads,
        shaper,
        fit_tolerance_mm: input.fit_tolerance_mm,
        beta_max_iters: input.beta_max_iters,
        beta_convergence_ratio: input.beta_convergence_ratio,
        e_limits: input.e_limits,
        initial_v: input.initial_v,
        initial_a: input.initial_a,
        terminal_v: input.terminal_v,
        start_d2_override: input.start_d2_override,
    };

    let partition = partition_batch(&segments, &input.e_limits);
    crate::beta::plan_velocity_inner(&shape_input, &partition, input.safety_mode)
}

fn build_shaper_config(kernels: &[Option<PlanShaper>; 4]) -> Result<ShaperConfig, ShapeError> {
    let x = kernels[0]
        .ok_or(ShapeError::UnsupportedShaperOnXY)?
        .into_required()?;
    let y = kernels[1]
        .ok_or(ShapeError::UnsupportedShaperOnXY)?
        .into_required()?;
    let z = kernels[2].map_or(AxisShaper::Passthrough, PlanShaper::into_axis);
    Ok(ShaperConfig { x, y, z })
}

#[cfg(test)]
mod tests;
