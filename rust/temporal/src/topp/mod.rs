//! TOPP pipeline: path → constraints → solver → verify → output.
//!
//! Spec §4.3.

use crate::{GridConfig, TopProfile, Limits};
use nurbs::VectorNurbs;

pub mod path;
pub mod constraints;
pub(crate) mod solver;
pub(crate) mod verify;

#[derive(Debug, thiserror::Error)]
pub enum ScheduleError {
    #[error("invalid endpoint velocity: {0}")]
    InvalidEndpointVelocity(&'static str),
    #[error("path parameterization failed: {0}")]
    PathParam(String),
    #[error("solver setup failed: {0}")]
    SolverSetup(String),
}

/// Single-segment time-optimal velocity-profile entry point.
///
/// Spec §4.3, §4.4. Solver-runtime infeasibility / max-iter surface as
/// `SolveStatus` on the returned profile, *not* as `ScheduleError`.
/// `ScheduleError` is for setup-time programming errors only.
pub fn schedule_segment(
    _curve: &VectorNurbs<f64, 3>,
    _limits: &Limits,
    _grid: &GridConfig,
    _v_start: f64,
    _v_end: f64,
) -> Result<TopProfile, ScheduleError> {
    unimplemented!("populated in Task 8")
}
