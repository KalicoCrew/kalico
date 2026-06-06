use std::collections::VecDeque;

use geometry::segment::CubicSegment;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;

use crate::emit_shaped::EmitSegmentMeta;
use crate::fit::FittedSegment;
use crate::pad::EHalo;
use crate::plan_velocity::{PlanShaper, PlanStats, SafetyMode};
use crate::ELimits;
use crate::ShapedSegment;

#[derive(Debug, Clone, Copy)]
pub struct ReplanReport {
    pub split_us: u64,
    pub solve_us: u64,
    pub rebuild_us: u64,
    pub window_segments: usize,
    pub plan: PlanStats,
}

mod decel_finder;
mod emit;
mod state;

#[cfg(test)]
mod tests;

/// Per-axis unshaped trajectory queue + kernel + half-support.
#[derive(Debug, Clone)]
pub struct AxisShaperQueue {
    /// Unshaped polynomial pieces, in time order.
    pub pieces: VecDeque<BezierPiece<f64>>,
    /// Smooth-shaper kernel for this axis. `None` for passthrough.
    pub kernel: Option<PiecewisePolynomialKernel<f64>>,
    /// Kernel half-support in seconds (`T_sm / 2`; `0.0` for passthrough).
    pub h: f64,
}

/// Source-geometry record for one un-committed move.
#[derive(Debug, Clone)]
pub struct UncommittedMove {
    pub segment: CubicSegment,
    /// Absolute start time in the planner timeline; set by `append_and_replan`.
    pub t_start: f64,
    /// Expected end time as of the most recent replan; updated on every replan.
    pub t_end: f64,
}

/// Configuration the streaming replan needs from the planner caller.
#[derive(Debug, Clone, Copy)]
pub struct ReplanContext {
    /// Per-axis temporal machine limits.
    pub limits: temporal::Limits,
    /// Per-axis shaper kernels `[X, Y, Z, E]`. E is always `Passthrough` or `None`.
    pub kernels: [Option<PlanShaper>; 4],
    /// L-infinity tolerance for the C1-constrained fit (mm).
    pub fit_tolerance_mm: f64,
    /// Maximum number of β-medium outer iterations per replan.
    pub beta_max_iters: u8,
    /// Convergence ratio threshold for β-medium iteration.
    pub beta_convergence_ratio: f64,
    /// Extruder axis dynamic limits.
    pub e_limits: ELimits,
    /// Per-junction chord-error tolerance (mm).
    pub junction_chord_tolerance_mm: f64,
    /// Worker thread count for TOPP-RA's parallel fan-out.
    pub worker_threads: usize,
    /// Grid strategy for `temporal::multi::plan_batch`.
    pub grid_strategy: temporal::multi::GridStrategy,
    /// Fallback path speed at `t_dispatched` when the cursor is outside the `pieces` domain.
    pub fallback_initial_v: f64,
    /// Safety mode for `plan_velocity`.
    pub safety_mode: SafetyMode,
}

/// Configuration the streaming emit half (`emit_committed`) needs from the caller.
///
/// Separate from [`ReplanContext`] because emit needs materialized
/// [`PiecewisePolynomialKernel`]s for the convolution while replan uses the
/// [`PlanShaper`] enum.
#[derive(Debug, Clone, Copy)]
pub struct EmitContext<'a> {
    /// Per-axis shaper kernels `[X, Y, Z, E]`. E slot unused by `emit_shaped`.
    pub kernels: &'a [Option<PiecewisePolynomialKernel<f64>>; 4],
    /// E-gap halo list. Streaming passes `&[]`.
    pub e_halos: &'a [EHalo],
}

/// Stateful streaming-shaper planner state.
#[derive(Debug)]
pub struct ShaperState {
    /// Per-axis queues (X, Y, Z, E).
    pub axes: [AxisShaperQueue; 4],

    /// Source-geometry records for moves not yet fully committed.
    pub uncommitted_moves: VecDeque<UncommittedMove>,

    /// Latest absolute time for which `append_and_replan` has been called.
    pub t_appended: f64,
    /// Start of the most-recently-submitted move's terminal decel-to-zero ramp.
    pub t_decel_start: f64,
    /// Latest absolute time for which a shaped sample has been computed.
    pub t_shaped: f64,
    /// Latest absolute time for which a shaped sample has been dispatched to the wire.
    pub t_dispatched: f64,

    /// Shaped output computed but not yet drained.
    pub pending_dispatch: Vec<ShapedSegment>,

    /// Cached fitted plan from the most recent `append_and_replan`.
    pub(crate) planned_fitted: Vec<FittedSegment>,
    /// Per-segment metadata parallel to `planned_fitted`.
    pub(crate) planned_meta: Vec<EmitSegmentMeta>,
}
