//! Layer 2 — single-segment time-optimal velocity profile.
//!
//! See `docs/superpowers/specs/2026-04-27-layer-2-topp-prototype-design.md`.

pub mod limits;
pub use limits::Limits;

pub mod topp;
pub use topp::{schedule_segment, ScheduleError};

pub mod multi;
pub use multi::{
    plan_batch, BatchError, BatchInput, BatchOutput, GridStrategy,
    JoiningStatus, JunctionBindingCap, JunctionInfo, SegmentInput,
};

#[derive(Debug, Clone, Copy)]
pub struct GridConfig {
    pub scheme: GridScheme,
    pub n: usize,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GridScheme {
    UniformArclength,
    // Future: Adaptive { … }, KnotAware { … }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Axis {
    X,
    Y,
    Z,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BindingConstraint {
    None,
    Velocity { axis: Axis },
    AxisAccel { axis: Axis },
    AxisJerk { axis: Axis },
    Centripetal,
    Boundary,
}

#[derive(Debug, Clone, Copy)]
pub struct GridSample {
    /// Arclength along the segment, mm.
    pub s: f64,
    /// Path speed, mm/s (= sqrt(b)).
    pub v: f64,
    /// Path acceleration, mm/s² (= s̈).
    pub a: f64,
    /// Raw SOCP primal `b = ṡ²`. Kept for downstream / debug use.
    pub b: f64,
    /// Which constraint, if any, was binding at this grid point.
    pub binding: BindingConstraint,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BoundarySide {
    Start,
    End,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum InfeasibleReason {
    BoundaryAboveMVC { side: BoundarySide, mvc_b: f64 },
    SolverInfeasible,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum SolveStatus {
    Solved,
    SolvedInexact { residual: f64 },
    Infeasible { at_grid: usize, reason: InfeasibleReason },
    MaxIter { last_residual: f64 },
    /// Converged via SLP outer iteration (Lee 2024) after `outer_iters`
    /// iterations of the relaxation-tightening loop. Profile is feasible.
    /// Spec §11 (CL-2024 Conjecture-4.1 counterexample fallback).
    SolvedSlp { outer_iters: u32 },
    /// SLP outer iteration failed to converge — max-violator ratio did not
    /// drop monotonically across the warm-up window, so the loop was aborted
    /// before hitting the iteration cap. Profile is the last iterate;
    /// downstream consumers should treat it as infeasible.
    DivergedSlp {
        last_max_ratio: f64,
        outer_iters: u32,
    },
    /// SLP outer iteration hit `MAX_OUTER_ITERS` without driving the
    /// max-violator ratio below `1 + ε_feas`. Profile is the last iterate.
    MaxIterSlp { last_max_ratio: f64 },
}

#[derive(Debug, Clone)]
pub struct TopProfile {
    pub samples: Vec<GridSample>,
    pub status: SolveStatus,
    pub grid_scheme: GridScheme,
    /// Total trajectory time, seconds.
    pub total_time: f64,
}
