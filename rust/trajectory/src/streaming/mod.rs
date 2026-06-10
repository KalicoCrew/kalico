use std::collections::VecDeque;

use geometry::segment::CubicSegment;
use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;

use crate::emit_shaped::EmitSegmentMeta;
use crate::fit::FittedSegment;
use crate::pad::EHalo;
use crate::plan_velocity::{PlanShaper, PlanStats, SafetyMode};
use crate::ELimits;

#[derive(Debug, Clone, Copy)]
pub struct ReplanReport {
    pub split_us: u64,
    pub solve_us: u64,
    pub rebuild_us: u64,
    pub window_segments: usize,
    pub plan: PlanStats,
    /// Which fallback rung resolved the plan: 1 = full window succeeded, 2 = Replace-remnant
    /// dropped, 3 = witness preserved and new segment planned alone from rest.
    pub fallback_rung: u8,
}

mod decel_finder;
mod emit;
mod state;

#[cfg(test)]
mod tests;

#[derive(Debug, Clone)]
pub struct AxisShaperQueue {
    /// Unshaped polynomial pieces, in time order.
    pub pieces: VecDeque<BezierPiece<f64>>,
    /// Smooth-shaper kernel for this axis. `None` for passthrough.
    pub kernel: Option<PiecewisePolynomialKernel<f64>>,
    /// Kernel half-support in seconds (`T_sm / 2`; `0.0` for passthrough).
    pub h: f64,
}

#[derive(Debug, Clone)]
pub struct UncommittedMove {
    pub segment: CubicSegment,
    /// Absolute start time in the planner timeline; set by `append_and_replan`.
    pub t_start: f64,
    /// Expected end time as of the most recent replan; updated on every replan.
    pub t_end: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct ReplanContext {
    pub limits: temporal::Limits,
    /// Per-axis shaper kernels `[X, Y, Z, E]`. E is always `Passthrough` or `None`.
    pub kernels: [Option<PlanShaper>; 4],
    /// L-infinity tolerance for the C1-constrained fit (mm).
    pub fit_tolerance_mm: f64,
    pub beta_max_iters: u8,
    pub beta_convergence_ratio: f64,
    pub e_limits: ELimits,
    pub junction_chord_tolerance_mm: f64,
    /// Worker thread count for TOPP-RA's parallel fan-out.
    pub worker_threads: usize,
    pub grid_strategy: temporal::multi::GridStrategy,
    /// Fallback path speed at `t_dispatched` when the cursor is outside the `pieces` domain.
    pub fallback_initial_v: f64,
    pub safety_mode: SafetyMode,
}

#[derive(Debug, Clone, Copy)]
pub struct EmitContext<'a> {
    /// Per-axis shaper kernels `[X, Y, Z, E]`. E slot unused by `emit_shaped`.
    pub kernels: &'a [Option<PiecewisePolynomialKernel<f64>>; 4],
    /// E-gap halo list. Streaming passes `&[]`.
    pub e_halos: &'a [EHalo],
}

#[derive(Debug)]
pub struct ShaperState {
    pub axes: [AxisShaperQueue; 4],

    pub uncommitted_moves: VecDeque<UncommittedMove>,

    pub t_appended: f64,
    /// Start of the most-recently-submitted move's terminal decel-to-zero ramp.
    pub t_decel_start: f64,
    pub t_shaped: f64,
    /// Latest absolute time for which a shaped sample has been dispatched to the wire.
    pub t_dispatched: f64,

    pub(crate) planned_fitted: Vec<FittedSegment>,
    /// Per-segment metadata parallel to `planned_fitted`.
    pub(crate) planned_meta: Vec<EmitSegmentMeta>,
}
