//! Phase-2 Task-2.1: planning half of the streaming-shaper split.
//!
//! `plan_velocity` runs TOPP-RA + β-medium iteration on a multi-axis path and
//! returns the time-domain **planned** (β-converged, unshaped) trajectory as
//! `Vec<FittedSegment>`. It does **not** perform shaping convolution or refit;
//! the shaping half is Task 2.2's `emit_shaped`. Until Task 2.2 lands the
//! existing `shape_batch` keeps doing the shaping inline; this entry point is
//! used by the streaming planner (`ShaperState::append_and_replan` in Phase 3)
//! to re-plan an un-committed path tail without producing wire-bound output.
//!
//! Two safety modes are supported (spec §3.2 / §3.6):
//!
//! - [`SafetyMode::TerminalKnown`] — current `shape_batch` semantics. The
//!   path's terminal velocity is final; the β-medium derate uses constant-pad
//!   future at the path terminus.
//! - [`SafetyMode::WorstCaseFuture`] — streaming case. The terminal velocity
//!   is the speculative decel-to-zero; β-medium derates against the
//!   worst-case-future bound (spec §3.6) by applying a tighter effective
//!   `a_machine` (`0.5·a_machine`) to the trailing region. Output is safe
//!   under any conforming follow-on input arriving after dispatch.
//!
//! See `docs/superpowers/specs/2026-05-10-streaming-shaper-design.md` §3.2 and
//! §3.6 for the full bound derivation and the rationale for the
//! "loose-but-always-safe" model used here.

use crate::fit::FittedSegment;
use crate::partition::partition_batch;
use crate::{
    AxisShaper, ELimits, RequiredShaper, ShapeBatchInput, ShapeError, ShapeSegmentInput,
    ShaperConfig,
};

/// Boundary-future treatment for the β-medium derate test.
///
/// See module docs and spec §3.6.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyMode {
    /// Terminal velocity is the actual final state of the path. β-medium
    /// derates against the post-shape peak computed with constant-position
    /// padding at the path terminus. This matches the current
    /// `shape_batch` behaviour byte-for-byte.
    TerminalKnown,
    /// Streaming case: the terminal velocity is speculative (the planner's
    /// decel-to-zero default) and the trailing-`h` region of the path may be
    /// replaced by a follow-on move at any time. β-medium derates against
    /// the worst-case-future bound (`|ẍ_shaped| ≤ past_term + 0.5·a_machine`
    /// for a symmetric unit-DC kernel) by tightening the effective machine
    /// accel limit on the trailing region.
    WorstCaseFuture,
}

/// Per-axis shaper for a [`PlanInput`]. Mirrors [`AxisShaper`] but allows
/// `None` on every axis (X / Y / Z / E), unlike [`ShaperConfig`] which forces
/// X and Y to be active. Streaming may legitimately plan with passthrough on
/// every axis (e.g., during early bring-up before per-axis shaper config is
/// loaded), so the planning API does not enforce X/Y activeness.
///
/// `E` is structurally always passthrough — the extruder follows shaped XY
/// arc-length on coupled segments and carries its own un-shaped NURBS on
/// independent segments — and we model it here only for symmetry with
/// [`crate::streaming::ShaperState`]'s 4-axis layout.
#[derive(Debug, Clone, Copy)]
pub enum PlanShaper {
    /// Smooth ZV at `frequency_hz`.
    SmoothZv { frequency_hz: f64 },
    /// Smooth MZV at `frequency_hz`.
    SmoothMzv { frequency_hz: f64 },
    /// No shaping for this axis (kernel half-support `h = 0`).
    Passthrough,
}

impl PlanShaper {
    fn into_required(self) -> Result<RequiredShaper, ShapeError> {
        match self {
            Self::SmoothZv { frequency_hz } => Ok(RequiredShaper::SmoothZv { frequency_hz }),
            Self::SmoothMzv { frequency_hz } => Ok(RequiredShaper::SmoothMzv { frequency_hz }),
            // Currently the underlying β-medium loop assumes X and Y are
            // active; passthrough on those axes is not exercised by the
            // existing test suite. We reject early rather than silently
            // producing untested behaviour. Phase 3 may relax this.
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

/// One segment of a multi-segment planning input.
///
/// Mirrors [`ShapeSegmentInput`] without the `e_independent`/`feedrate_mm_s`
/// fields that only matter for the (Task-2.2) shaping half.
#[derive(Debug, Clone, Copy)]
pub struct PlanSegment<'a> {
    /// Layer-2 input for this segment (curve + dynamic limits + junction
    /// chord tolerance).
    pub temporal: temporal::multi::SegmentInput<'a>,
    /// E-axis mode. Used by the partitioning step to identify XY-motion runs.
    pub e_mode: geometry::segment::EMode,
    /// Extrusion ratio (mm E per mm XY arc length); zero for `Travel`.
    pub extrusion_per_xy_mm: f64,
    /// Independent-E NURBS for `Independent` E mode segments. Required by
    /// the partitioner to schedule E-only gaps; `None` for XY-motion segments.
    pub e_independent: Option<&'a nurbs::ScalarNurbs<f64>>,
    /// Feedrate (mm/s) — needed for E-only gap scheduling.
    pub feedrate_mm_s: f64,
}

/// Top-level input to [`plan_velocity`].
///
/// `initial_v` and `terminal_v` are the velocity boundary conditions at the
/// **batch start** (`segments[0]`'s u=0) and **batch end** (`segments[last]`'s
/// u=1) respectively, in mm/s. Phase 3 lifted the prior (0, 0) limitation:
/// both values are now forwarded to [`temporal::multi::plan_batch`] via
/// `BatchInput::{initial_velocity, terminal_velocity}`, which threads them
/// into the joining loop's first-segment `v_start` and last-segment `v_end`
/// seeds. TOPP-RA's per-segment `schedule_segment_with_tolerance` already
/// accepted arbitrary boundary velocities; the lift is purely plumbing.
///
/// The streaming shaper (`ShaperState::append_and_replan`) uses this to plan
/// from the velocity already committed at `t_dispatched` (so the un-committed
/// replan window chains continuously into the in-flight motion) and to
/// always decelerate the replanned tail to zero at the new move's terminal
/// (so the spec's "decel-to-zero default" holds even when no follow-on move
/// arrives in time).
#[derive(Debug)]
pub struct PlanInput<'a> {
    /// Multi-axis planning path; must be non-empty.
    pub segments: &'a [PlanSegment<'a>],
    /// Forwarded to `temporal::multi::plan_batch`.
    pub grid_strategy: temporal::multi::GridStrategy,
    /// Forwarded to `temporal::multi::plan_batch`.
    pub worker_threads: usize,
    /// Per-axis shapers in the order `[X, Y, Z, E]`. `E` is always passthrough
    /// (entry retained for streaming-state symmetry); see [`PlanShaper`].
    pub kernels: [Option<PlanShaper>; 4],
    /// L-infinity tolerance for the C1-constrained fit (mm).
    pub fit_tolerance_mm: f64,
    /// Maximum number of β-medium outer iterations.
    pub beta_max_iters: u8,
    /// Convergence ratio threshold for β-medium iteration.
    pub beta_convergence_ratio: f64,
    /// Extruder axis limits.
    pub e_limits: ELimits,
    /// Velocity at the batch start (mm/s). Must be finite and non-negative.
    /// Phase 3 accepts arbitrary values; the streaming shaper uses this to
    /// chain into the committed velocity at `t_dispatched`.
    pub initial_v: f64,
    /// Velocity at the batch end (mm/s). Must be finite and non-negative.
    /// Phase 3 accepts arbitrary values; the streaming shaper's "decel-to-
    /// zero default" plans always pass `0.0` here so the new move's terminal
    /// is a safe rest point.
    pub terminal_v: f64,
    /// Boundary-future treatment for the trailing region.
    pub safety_mode: SafetyMode,
}

/// Run the planning half of the shaper pipeline.
///
/// Returns the β-converged time-domain **fitted** trajectory: one
/// [`FittedSegment`] per XY-motion input segment, in the same order. E-only
/// gaps are excluded — they are inserted by the shaping half (Task 2.2).
///
/// # Errors
///
/// - [`ShapeError::EmptySegments`] — `input.segments` is empty.
/// - [`ShapeError::UnsupportedShaperOnXY`] — any axis kernel is `None` or
///   `Passthrough` for X or Y (Phase 2 limitation; the underlying β-medium
///   loop assumes both axes are actively shaped).
/// - [`ShapeError::UnsupportedBoundaryVelocity`] — `initial_v` or `terminal_v`
///   is non-finite or negative.
/// - Any error from the underlying β-medium loop (TOPP-RA infeasibility, fit
///   failure, etc.).
pub fn plan_velocity(input: &PlanInput<'_>) -> Result<Vec<FittedSegment>, ShapeError> {
    if input.segments.is_empty() {
        return Err(ShapeError::EmptySegments);
    }

    // Boundary-velocity validation: Phase 3 lifted the (0, 0) limitation —
    // `temporal::multi::plan_batch` now accepts arbitrary `(initial_velocity,
    // terminal_velocity)` and TOPP-RA already handles arbitrary boundary
    // conditions internally via `schedule_segment_with_tolerance(.., v_start,
    // v_end, ..)`. Only basic sanity (finite, non-negative) is enforced here;
    // physical feasibility is the temporal solver's job. The
    // [`ShapeError::UnsupportedBoundaryVelocity`] variant is retained so
    // callers that previously got a hard rejection still see a structured
    // error on out-of-domain inputs.
    if !input.initial_v.is_finite() || input.initial_v < 0.0 {
        return Err(ShapeError::UnsupportedBoundaryVelocity);
    }
    if !input.terminal_v.is_finite() || input.terminal_v < 0.0 {
        return Err(ShapeError::UnsupportedBoundaryVelocity);
    }

    // Build a `ShapeBatchInput` from `PlanInput`. Internally this is the
    // existing β-medium machinery; the only new behaviour is the
    // `safety_mode` interpretation of the post-shape peak vs machine limit
    // in the trailing region.
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
        terminal_v: input.terminal_v,
    };

    let partition = partition_batch(&segments, &input.e_limits);
    crate::beta::plan_velocity_inner(&shape_input, &partition, input.safety_mode)
}

fn build_shaper_config(kernels: &[Option<PlanShaper>; 4]) -> Result<ShaperConfig, ShapeError> {
    // X and Y are required-active per `ShaperConfig`. `None` or `Passthrough`
    // entries in those slots are rejected (see [`PlanShaper`] doc).
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
