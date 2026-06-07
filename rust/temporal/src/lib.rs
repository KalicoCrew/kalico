pub mod limits;
pub use limits::Limits;

pub mod topp;
pub use topp::{ScheduleError, ToleranceMode, schedule_segment, schedule_segment_with_tolerance};

pub mod multi;
pub use multi::{
    BatchError, BatchInput, BatchOutput, GridStrategy, JoiningStatus, JunctionBindingCap,
    JunctionInfo, SegmentInput, plan_batch,
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
    pub s: f64,
    /// Path speed, mm/s (= sqrt(b)).
    pub v: f64,
    /// Path acceleration, mm/s² (= s̈).
    pub a: f64,
    /// Raw SOCP primal `b = ṡ²`.
    pub b: f64,
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
    SolvedInexact {
        residual: f64,
    },
    Infeasible {
        at_grid: usize,
        reason: InfeasibleReason,
    },
    MaxIter {
        last_residual: f64,
    },
    SolvedSlp {
        outer_iters: u32,
    },
    DivergedSlp {
        last_max_ratio: f64,
        outer_iters: u32,
    },
    MaxIterSlp {
        last_max_ratio: f64,
    },
}

#[derive(Debug, Clone)]
pub struct TopProfile {
    pub samples: Vec<GridSample>,
    pub status: SolveStatus,
    pub grid_scheme: GridScheme,
    pub total_time: f64,
}
