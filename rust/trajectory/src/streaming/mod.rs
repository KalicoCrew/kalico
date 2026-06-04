// Streaming-shaper module: per-axis stateful trajectory queue + look-ahead
// replanning + dispatch-aware emit half.
// Spec: `docs/superpowers/specs/2026-05-10-streaming-shaper-design.md`

use std::collections::VecDeque;

use geometry::segment::CubicSegment;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;

use crate::emit_shaped::EmitSegmentMeta;
use crate::fit::FittedSegment;
use crate::pad::EHalo;
use crate::plan_velocity::{PlanShaper, SafetyMode};
use crate::ELimits;
use crate::ShapedSegment;

mod decel_finder;
mod emit;
mod state;

#[cfg(test)]
mod tests;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Per-axis unshaped trajectory queue + kernel + half-support.
///
/// `pieces` accumulates the unshaped polynomial pieces the convolution must
/// see (history, current, lookahead); trimmed against `t_dispatched` on each
/// `emit_committed` call.
#[derive(Debug, Clone)]
pub struct AxisShaperQueue {
    /// Unshaped polynomial pieces, in time order.
    pub pieces: VecDeque<BezierPiece<f64>>,
    /// Smooth-shaper kernel for this axis. `None` for passthrough.
    pub kernel: Option<PiecewisePolynomialKernel<f64>>,
    /// Kernel half-support in seconds (`T_sm / 2`; `0.0` for passthrough).
    pub h: f64,
}

/// Source-geometry record for one un-committed `submit_move`.
///
/// Retained alongside `axes[i].pieces` so `append_and_replan` can rebuild
/// the planning path (un-committed tail + new move) and run TOPP-RA over it.
/// Dropped once `t_end < t_dispatched`; the `BezierPiece`s remain in
/// `axes[i].pieces` as left-pad history for `emit_shaped`.
#[derive(Debug, Clone)]
pub struct UncommittedMove {
    pub segment: CubicSegment,
    /// Absolute start time in the planner timeline; set by `append_and_replan`.
    pub t_start: f64,
    /// Expected end time as of the most recent replan; updated on every replan.
    pub t_end: f64,
}

/// Configuration the streaming replan needs from the planner caller.
///
/// Not owned by the streaming planner — `motion-bridge::planner.rs` holds it
/// in `PlannerConfig` and passes it on every `append_and_replan` call so live
/// `update_limits` / `update_shaper` reconfigurations take effect immediately.
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
    /// Per-junction chord-error tolerance (mm). No per-move plumb exists in
    /// the streaming path yet; sane default is `0.05` (50 µm).
    pub junction_chord_tolerance_mm: f64,
    /// Worker thread count for TOPP-RA's parallel fan-out.
    pub worker_threads: usize,
    /// Grid strategy for `temporal::multi::plan_batch`.
    pub grid_strategy: temporal::multi::GridStrategy,
    /// Fallback path speed at `t_dispatched` when the cursor is outside the
    /// `pieces` domain (e.g., immediately after construction). Defaults to `0.0`.
    pub fallback_initial_v: f64,
    /// Safety mode for `plan_velocity`. Always `WorstCaseFuture` in the
    /// streaming path: the trailing-h region is β-derated against worst-case
    /// future arrival to keep dispatch safe.
    pub safety_mode: SafetyMode,
}

/// Configuration the streaming emit half (`emit_committed`) needs from the
/// caller.
///
/// Separate from [`ReplanContext`] because the kernel representation differs:
/// [`ReplanContext::kernels`] carries the planner-side [`PlanShaper`] enum,
/// while emit needs the materialized [`PiecewisePolynomialKernel`] for the
/// `emit_shaped` convolution. Both are built from the same `ShaperConfig` at
/// startup.
#[derive(Debug, Clone, Copy)]
pub struct EmitContext<'a> {
    /// Per-axis shaper kernels `[X, Y, Z, E]`. E slot unused by `emit_shaped`.
    pub kernels: &'a [Option<PiecewisePolynomialKernel<f64>>; 4],
    /// E-gap halo list. Streaming passes `&[]`; slot retained for future
    /// Independent-E streaming support.
    pub e_halos: &'a [EHalo],
}

// ---------------------------------------------------------------------------
// ShaperState
// ---------------------------------------------------------------------------

/// Stateful streaming-shaper planner state, sharing one absolute time line
/// across all axes (every append is multi-axis).
///
/// The v4-era fields `t_tentative`, `rest_tentative`, and `generation` were
/// removed when the tentative-rest extension model was eliminated: the planner
/// now appends each move's terminal decel-to-zero outright and tracks where
/// that decel begins in `t_decel_start`. See spec §3.1 ("State invariants").
#[derive(Debug)]
pub struct ShaperState {
    /// Per-axis queues (X, Y, Z, E). Z is typically passthrough; E is followed
    /// off the shaped XY arc-length rather than shaped independently.
    pub axes: [AxisShaperQueue; 4],

    /// Source-geometry records for moves appended but not yet fully committed
    /// (`t_dispatched < move.t_end`). Dropped when `t_end < t_dispatched`;
    /// their `BezierPiece`s remain in `axes[i].pieces` as left-pad history.
    pub uncommitted_moves: VecDeque<UncommittedMove>,

    /// Latest absolute time for which `append_and_replan` has been called.
    pub t_appended: f64,
    /// Start of the most-recently-submitted move's terminal decel-to-zero ramp.
    /// Gates dispatch via `t_decel_start - max_h` in `emit_committed`.
    /// Equals `t_appended` when the queue is empty.
    pub t_decel_start: f64,
    /// Latest absolute time for which a shaped sample has been computed.
    pub t_shaped: f64,
    /// Latest absolute time for which a shaped sample has been dispatched to
    /// the wire.
    pub t_dispatched: f64,

    /// Shaped output computed but not yet drained. Drained via
    /// `drain_committed`.
    pub pending_dispatch: Vec<ShapedSegment>,

    /// Cached fitted plan from the most recent `append_and_replan`. One entry
    /// per `UncommittedMove`, in the same order. Cleared and rebuilt on every
    /// replan; sliced by `emit_committed` and fed to `emit_shaped`.
    pub(crate) planned_fitted: Vec<FittedSegment>,
    /// Per-segment metadata parallel to `planned_fitted` (`e_mode`,
    /// `extrusion_per_xy_mm`), read from `UncommittedMove::segment` at replan
    /// time.
    pub(crate) planned_meta: Vec<EmitSegmentMeta>,
}
