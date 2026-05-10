// Streaming-shaper module: per-axis stateful trajectory queue + look-ahead
// replanning + dispatch-aware emit half.
//
// This module is the new home of the streaming planner that progressively
// replaces the per-batch pad-and-trim shaping driven by `shape_batch`. See:
//
// - Spec: `docs/superpowers/specs/2026-05-10-streaming-shaper-design.md`
// - Plan: `docs/superpowers/plans/2026-05-10-streaming-shaper.md`
//
// **Layout (Phase 3 onward).** The module was split out of a single
// `streaming.rs` file once `append_and_replan` and `emit_committed` made the
// file too dense for a flat layout:
//
// - [`mod`] (this file) — public types and module declarations.
// - [`state`] — `ShaperState` construction, the Phase-1 byte-identity shim,
//   `append_and_replan` (Phase 3 Task 3.1), and the small helpers it owns.
// - [`emit`] — `emit_committed` (Phase 3 Task 3.2) and its helpers.
// - [`decel_finder`] — terminal-decel-ramp localization used by
//   `append_and_replan`.
//
// External callers continue to use the public re-exports through
// `trajectory::streaming::*`; the file split is invisible at the crate boundary.

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
/// `pieces` accumulates the unshaped polynomial pieces that the convolution
/// must see (history, current, lookahead). Phase 1 keeps `pieces` populated
/// only with the `home_pos` rest-extension seed; phase 3 fills it from
/// real `append_and_replan` input and trims it against the dispatched cursor
/// (`emit_committed`).
#[derive(Debug, Clone)]
pub struct AxisShaperQueue {
    /// Unshaped polynomial pieces, in time order. See module docs.
    pub pieces: VecDeque<BezierPiece<f64>>,
    /// Smooth-shaper kernel for this axis. `None` for passthrough.
    pub kernel: Option<PiecewisePolynomialKernel<f64>>,
    /// Kernel half-support (seconds). Equal to `T_sm / 2` for active shapers,
    /// `0.0` for passthrough.
    pub h: f64,
}

/// Source-geometry record for one un-committed `submit_move`. The streaming
/// planner retains these alongside the per-axis `pieces` queue so a
/// follow-on `append_and_replan` call can rebuild the planning path
/// (un-committed tail + new move) and run TOPP-RA over it.
///
/// Once the entry's `t_end` falls strictly below `t_dispatched`, the move's
/// pieces are wholly committed and the source record can be dropped from
/// `uncommitted_moves` — its planned time-domain `BezierPiece`s remain in
/// `axes[i].pieces` as history for `emit_shaped`'s left-pad.
#[derive(Debug, Clone)]
pub struct UncommittedMove {
    /// Source-geometry segment, as classified upstream (`CubicSegment` from
    /// `motion-bridge::classify_and_build` or equivalent).
    pub segment: CubicSegment,
    /// Absolute time at which this move's geometry starts in the planner
    /// timeline. Set by `append_and_replan`. For the first move ever
    /// appended this is `0.0`; for subsequent moves it equals the prior
    /// uncommitted move's `t_end` (continuous time line).
    pub t_start: f64,
    /// Absolute time at which the planner currently expects this move to
    /// end, as of the most recent replan. Updated on every replan.
    pub t_end: f64,
}

/// Configuration the streaming replan needs from the planner caller.
///
/// `kernels` and `e_limits` mirror [`crate::ShapeBatchInput`]; `limits` is
/// the temporal-axis machine limit applied to each move. The streaming
/// shaper does not own this configuration — `motion-bridge::planner.rs`
/// holds it in `PlannerConfig` and threads it through on every
/// `append_and_replan` call so live `update_limits` / `update_shaper`
/// reconfigurations take effect immediately on the next replan.
#[derive(Debug, Clone, Copy)]
pub struct ReplanContext {
    /// Per-axis temporal machine limits (`Limits::new(v, a, j, a_centripetal)`).
    pub limits: temporal::Limits,
    /// Per-axis shaper kernels in the order `[X, Y, Z, E]`. Mirrors
    /// [`crate::plan_velocity::PlanInput::kernels`]. E is structurally
    /// always [`PlanShaper::Passthrough`] or `None`.
    pub kernels: [Option<PlanShaper>; 4],
    /// L-infinity tolerance for the C1-constrained fit (mm).
    pub fit_tolerance_mm: f64,
    /// Maximum number of β-medium outer iterations per replan.
    pub beta_max_iters: u8,
    /// Convergence ratio threshold for β-medium iteration.
    pub beta_convergence_ratio: f64,
    /// Extruder axis dynamic limits.
    pub e_limits: ELimits,
    /// Per-junction chord-error tolerance threaded into each segment's
    /// `temporal::multi::SegmentInput.trailing_junction_chord_tolerance_mm`.
    /// Slicer-supplied per-segment in the full pipeline; the streaming
    /// planner does not currently have a per-move plumb so we accept a
    /// single value here. Sane default: `0.05` (50 µm).
    pub junction_chord_tolerance_mm: f64,
    /// Worker thread count for TOPP-RA's parallel fan-out.
    pub worker_threads: usize,
    /// Grid strategy for `temporal::multi::plan_batch`.
    pub grid_strategy: temporal::multi::GridStrategy,
    /// Reference reading for the path speed at `t_dispatched`. The
    /// streaming planner samples its own `pieces` queue derivative when
    /// available and falls back to this when the cursor is outside the
    /// pieces' domain (e.g., right after `new()` before any append).
    /// Defaults to `0.0` (toolhead at rest).
    pub fallback_initial_v: f64,
    /// `SafetyMode` to pass to `plan_velocity`. The streaming append path
    /// always wants `SafetyMode::WorstCaseFuture` so the trailing-h region
    /// of the un-committed tail is β-derated against the worst-case future
    /// arrival.
    pub safety_mode: SafetyMode,
}

/// Configuration the streaming emit half (`emit_committed`) needs from the
/// caller.
///
/// Separate from [`ReplanContext`] because the kernel representation differs:
/// [`ReplanContext::kernels`] carries the planner-side [`PlanShaper`] enum
/// (which `plan_velocity` consumes), while emit needs the materialized
/// [`PiecewisePolynomialKernel`] for `crate::emit_shaped`'s convolution. The
/// planner thread builds both at startup from the same `ShaperConfig`; the
/// streaming planner does not own this configuration.
///
/// `e_halos` is the same E-gap halo list `crate::emit_shaped` accepts.
/// Streaming callers pass an empty slice — no E gaps exist in the
/// look-ahead replan window (the extruder is followed off the shaped XY
/// arc-length, not scheduled independently in the streaming path).
#[derive(Debug, Clone, Copy)]
pub struct EmitContext<'a> {
    /// Per-axis shaper kernels in the order `[X, Y, Z, E]`. Slot ordering
    /// matches [`PlanInput::kernels`](crate::plan_velocity::PlanInput::kernels)
    /// and [`PerAxisHistory`](crate::emit_shaped::PerAxisHistory). The E slot
    /// is unused by [`crate::emit_shaped`].
    pub kernels: &'a [Option<PiecewisePolynomialKernel<f64>>; 4],
    /// E-gap halo list. Streaming passes `&[]`; the wrapping `shape_batch`
    /// call site (which interleaves E gaps) is the only caller that supplies
    /// non-empty halos. Retained as a slot for forward compatibility with
    /// future Independent-E streaming.
    pub e_halos: &'a [EHalo],
}

// ---------------------------------------------------------------------------
// ShaperState
// ---------------------------------------------------------------------------

/// Stateful streaming-shaper planner state, sharing one absolute time line
/// across all axes (every append is multi-axis).
///
/// Phase 1 only uses `axes` and `pending_dispatch`; the cursors are seeded
/// to zero / left untouched by `append_batch`. Phase 3 (`append_and_replan`
/// / `emit_committed`) drives the cursors (`t_appended`, `t_decel_start`,
/// `t_shaped`, `t_dispatched`).
///
/// **v5 field set.** The v4-era fields `t_tentative`, `rest_tentative`, and
/// `generation` were removed when v5's design eliminated the
/// tentative-rest extension model — the streaming planner now appends each
/// move's terminal decel-to-zero outright and tracks where that decel begins
/// in `t_decel_start`. See spec §3.1 ("State invariants").
#[derive(Debug)]
pub struct ShaperState {
    /// Per-axis queues (X, Y, Z, E). Z is typically passthrough; E is unused
    /// in Phase 1 (extruder is followed off the shaped XY arc-length and is
    /// not a shaped axis in CLAUDE.md's MVP scope).
    pub axes: [AxisShaperQueue; 4],

    /// Source-geometry records for each move that has been appended but not
    /// yet fully committed (`t_dispatched < move.t_end`). Phase 3's
    /// `append_and_replan` reads this to build the planning path
    /// (un-committed tail + new move) for the replan window. Records whose
    /// `t_end < t_dispatched` are dropped by `append_and_replan` (their
    /// planned `BezierPiece`s stay in `axes[i].pieces` as history).
    pub uncommitted_moves: VecDeque<UncommittedMove>,

    /// Latest absolute time for which a real `append_batch` has been received.
    pub t_appended: f64,
    /// Absolute time at which the most-recently-submitted move's terminal
    /// decel-to-zero begins. Phase 3's `append_and_replan` populates this
    /// from the planner's velocity profile so the next `submit_move` can
    /// rewind to it and re-plan the un-committed tail. Initialized to
    /// `0.0` at construction; equal to `t_appended` when the queue is empty.
    pub t_decel_start: f64,
    /// Latest absolute time for which a shaped sample has been computed.
    pub t_shaped: f64,
    /// Latest absolute time for which a shaped sample has been *dispatched*
    /// to the wire.
    pub t_dispatched: f64,

    /// Shaped output computed but not yet drained / dispatched. Populated
    /// transiently by Phase 3's `emit_committed` and by Phase 1's
    /// `append_batch` shim; drained via `drain_committed`.
    pub pending_dispatch: Vec<ShapedSegment>,

    /// Cached time-domain fitted plan produced by the most recent
    /// `append_and_replan`. One [`FittedSegment`] per [`UncommittedMove`], in
    /// the same order. `emit_committed` slices this by absolute-time range
    /// and feeds it to [`crate::emit_shaped`].
    ///
    /// Kept in sync with `axes[i].pieces`'s un-committed tail by
    /// `append_and_replan`; cleared on construction and on the next replan.
    pub(crate) planned_fitted: Vec<FittedSegment>,
    /// Per-segment metadata parallel to `planned_fitted`. Mirrors what
    /// `crate::emit_shaped` requires (`e_mode`, `extrusion_per_xy_mm`); read
    /// from the corresponding [`UncommittedMove::segment`] at replan time.
    pub(crate) planned_meta: Vec<EmitSegmentMeta>,
}
