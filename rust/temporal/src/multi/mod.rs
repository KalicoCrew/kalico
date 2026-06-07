use crate::{Limits, TopProfile};
use nurbs::VectorNurbs;
use thiserror::Error;

#[non_exhaustive]
#[derive(Debug, Clone, Copy)]
pub enum GridStrategy {
    Fixed(usize),
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
    pub trailing_junction_chord_tolerance_mm: f64,
}

#[derive(Debug, Clone, Copy)]
pub struct BatchInput<'a> {
    pub segments: &'a [SegmentInput<'a>],
    pub grid_strategy: GridStrategy,
    pub worker_threads: usize,
    pub initial_velocity: f64,
    pub terminal_velocity: f64,
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
    Converged,
    StalledOnInfeasibleSegment { last_dirty_count: usize },
    CappedAtMaxSweeps { last_dirty_count: usize },
}

#[derive(Debug, Clone, Copy)]
pub struct JunctionInfo {
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

/// Run the full multi-segment planning pipeline on a batch of curve segments.
///
/// # Errors
/// - [`BatchError::EmptySegments`] — `input.segments` is empty.
/// - [`BatchError::InvalidThreads`] — `input.worker_threads` is zero.
/// - [`BatchError::Segment`] — a segment-level [`crate::ScheduleError`] was
///   returned by [`crate::topp::schedule_segment_with_tolerance`].
pub fn plan_batch(input: BatchInput<'_>) -> Result<BatchOutput, BatchError> {
    use crate::GridConfig;
    use crate::multi::{grid, joining, junction, parallel};

    if input.segments.is_empty() {
        return Err(BatchError::EmptySegments);
    }
    if input.worker_threads == 0 {
        return Err(BatchError::InvalidThreads);
    }

    let k = input.segments.len();

    let grids: Vec<GridConfig> = input
        .segments
        .iter()
        .map(|s| GridConfig {
            scheme: crate::GridScheme::UniformArclength,
            n: grid::compute_n(&input.grid_strategy, s.curve),
        })
        .collect();

    let junctions: Vec<junction::JunctionResult> = (0..k - 1)
        .map(|i| {
            junction::compute_junction_velocity(
                input.segments[i].curve,
                input.segments[i + 1].curve,
                &input.segments[i].limits,
                &input.segments[i + 1].limits,
                input.segments[i].trailing_junction_chord_tolerance_mm,
            )
        })
        .collect();

    let mut states: Vec<joining::SegmentState> = (0..k)
        .map(|i| {
            let v_start = if i == 0 {
                input.initial_velocity
            } else {
                junctions[i - 1].v_junction
            };
            let v_end = if i == k - 1 {
                input.terminal_velocity
            } else {
                junctions[i].v_junction
            };
            joining::SegmentState {
                v_start,
                v_end,
                profile: None,
                dirty: true,
            }
        })
        .collect();

    parallel::fan_out_solves(input.segments, &mut states, &grids, input.worker_threads)?;

    let (sweeps, joining_status) = joining::join_until_converged(
        input.segments,
        &grids,
        &mut states,
        &junctions,
        input.worker_threads,
    )?;

    let profiles: Vec<_> = states
        .into_iter()
        .map(|s| s.profile.expect("all profiles solved by stage 5"))
        .collect();
    let junction_infos: Vec<JunctionInfo> = junctions
        .into_iter()
        .enumerate()
        .map(|(i, j)| {
            // Use the converged profile endpoint, not the upfront cap: the joining
            // loop may have driven the velocity below the cap on short segments.
            let v_converged = profiles[i].samples.last().map_or(0.0, |s| s.v);
            JunctionInfo {
                between_segments: (i, i + 1),
                v_junction: v_converged,
                binding_cap: j.binding_cap,
                kappa_left: j.kappa_left,
                kappa_right: j.kappa_right,
            }
        })
        .collect();
    Ok(BatchOutput {
        profiles,
        junctions: junction_infos,
        joining_sweeps: sweeps,
        joining_status,
    })
}

#[cfg(test)]
mod tests;

mod chain;
mod grid;
mod joining;
mod junction;
mod parallel;
