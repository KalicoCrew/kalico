//! Layer 2 multi-segment integration. See spec
//! `docs/superpowers/specs/2026-04-27-layer-2-multi-segment-design.md`.

use crate::{Limits, TopProfile};
use nurbs::VectorNurbs;
use thiserror::Error;

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum GridStrategy {
    /// Fixed-N for every segment. Step 4 backward-compatible.
    Fixed(usize),
    /// Adaptive N per segment per spec §2.5.
    Adaptive {
        min_n: usize,
        max_n: usize,
        target_grid_spacing_mm: f64,
    },
}

#[derive(Debug, Clone, Copy)]
pub struct SegmentInput<'a> {
    pub curve: &'a VectorNurbs<f64, 3>,
    pub limits: Limits,
    /// Per-junction chord-error tolerance for the *trailing* junction
    /// (between this segment and the next). Slicer-supplied for sharp
    /// G1↔G1 corners; ignored for smooth-κ junctions per spec §2.2.
    pub trailing_junction_chord_tolerance_mm: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct BatchInput<'a> {
    pub segments: &'a [SegmentInput<'a>],
    pub grid_strategy: GridStrategy,
    /// Default 3 on Pi 5 per spec §2.6 (avoids Klipper contention on cores 0-1).
    pub worker_threads: usize,
}

#[derive(Debug)]
pub struct BatchOutput {
    pub profiles: Vec<TopProfile>,
    pub junctions: Vec<JunctionInfo>,
    pub joining_sweeps: u32,
    pub joining_status: JoiningStatus,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum JoiningStatus {
    /// Velocities stabilized AND all segments solved cleanly.
    Converged,
    /// Velocity propagation stabilized, but some segments still have
    /// non-success solver status (`Infeasible` / `MaxIter` / `DivergedSlp` /
    /// `MaxIterSlp`). `schedule_segment` is deterministic, so re-solving with
    /// the same inputs would produce the same status — no point continuing.
    /// Diagnostic: indicates pathological segment(s) that need looser
    /// endpoints, finer N, or v2 algorithmic improvement.
    /// (Per round-4 review: split out from `CappedAtMaxSweeps` for caller
    /// diagnostic clarity.)
    StalledOnInfeasibleSegment { last_dirty_count: usize },
    /// Reached `MAX_SWEEPS` without velocity stabilization. Indicates
    /// joining-loop oscillation — different (and worse) failure mode than
    /// `StalledOnInfeasibleSegment`. Should not happen on the test fixtures;
    /// surfacing this means joining algorithm has a bug.
    CappedAtMaxSweeps { last_dirty_count: usize },
}

#[derive(Debug, Clone, Copy)]
pub struct JunctionInfo {
    /// Indices of the two segments this junction sits between.
    pub between_segments: (usize, usize),
    pub v_junction: f64,
    pub binding_cap: JunctionBindingCap,
    pub kappa_left: f64,
    pub kappa_right: f64,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum JunctionBindingCap {
    PerAxisVelocity,
    Centripetal,
    GlobalVMax,
    SharpCornerChord,
}

#[derive(Debug, Error)]
pub enum BatchError {
    #[error("empty segment buffer")]
    EmptySegments,
    #[error("worker_threads must be ≥ 1")]
    InvalidThreads,
    #[error("segment {0}: {1}")]
    Segment(usize, crate::topp::ScheduleError),
}

// Stub — real implementation in Task 9.
pub fn plan_batch(_input: BatchInput<'_>) -> Result<BatchOutput, BatchError> {
    unimplemented!("plan_batch lands in Task 9")
}

mod grid;
mod joining;
mod junction;
mod parallel;
