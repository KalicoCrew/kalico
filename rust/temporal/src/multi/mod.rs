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
    /// Path accel at the batch start. Pinned in the SOCP only when `initial_velocity > 0`;
    /// at a rest start it MUST be 0.0 (asserted) and the rest envelope governs.
    pub initial_accel: f64,
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
    ChainInterior,
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

pub fn plan_batch(input: BatchInput<'_>) -> Result<BatchOutput, BatchError> {
    use crate::multi::{chain, grid, joining, junction, parallel};
    use crate::topp::chain::ChainGrid;
    use crate::topp::path::sample_arclength_grid;
    use nurbs::VectorNurbs;

    if input.segments.is_empty() {
        return Err(BatchError::EmptySegments);
    }
    if input.worker_threads == 0 {
        return Err(BatchError::InvalidThreads);
    }
    assert!(
        input.initial_velocity > 0.0 || input.initial_accel == 0.0,
        "rest start requires initial_accel == 0"
    );

    let k = input.segments.len();

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

    let kinds: Vec<junction::JunctionKind> = junctions.iter().map(|j| j.kind).collect();
    let chain_ranges = chain::partition_chains(k, &kinds);
    let n_chains = chain_ranges.len();

    let grid_max_n = match input.grid_strategy {
        GridStrategy::Adaptive { max_n, .. } => Some(max_n),
        GridStrategy::Fixed(_) => None,
    };

    let chain_grids: Vec<ChainGrid> = chain_ranges
        .iter()
        .map(|range| {
            let chain_curves: Vec<&VectorNurbs<f64, 3>> =
                range.clone().map(|i| input.segments[i].curve).collect();
            let mut ns: Vec<usize> = chain_curves
                .iter()
                .map(|c| grid::compute_n(&input.grid_strategy, c))
                .collect();
            grid::reconcile_junction_n(&mut ns, &chain_curves, grid_max_n);
            let absorbed = grid::classify_absorbed(&ns, &chain_curves);
            let seg_grids: Result<Vec<_>, _> = range
                .clone()
                .zip(ns)
                .map(|(seg_idx, n)| {
                    sample_arclength_grid(input.segments[seg_idx].curve, n).map_err(|e| {
                        BatchError::Segment(
                            seg_idx,
                            crate::topp::ScheduleError::PathParam(format!("{e}")),
                        )
                    })
                })
                .collect();
            let seg_limits: Vec<_> = range.clone().map(|i| input.segments[i].limits).collect();
            seg_grids.map(|grids| {
                ChainGrid::from_segment_grids_with_absorbed(grids, seg_limits, &absorbed)
            })
        })
        .collect::<Result<Vec<_>, _>>()?;

    let mut states: Vec<joining::ChainState> = chain_ranges
        .iter()
        .enumerate()
        .map(|(c, range)| {
            let lo_c = *range.start();
            let hi_c = *range.end();
            let v_start = if c == 0 {
                input.initial_velocity
            } else {
                junctions[lo_c - 1].v_junction
            };
            let v_end = if c + 1 == n_chains {
                input.terminal_velocity
            } else {
                junctions[hi_c].v_junction
            };
            let a_start = if c == 0 && input.initial_velocity > 0.0 {
                Some(input.initial_accel)
            } else {
                None
            };
            joining::ChainState {
                v_start,
                v_end,
                a_start,
                profile: None,
                dirty: true,
            }
        })
        .collect();

    parallel::fan_out_solves(&chain_grids, &mut states, input.worker_threads)?;

    let corner_caps: Vec<f64> = chain_ranges
        .iter()
        .take(n_chains - 1)
        .map(|range| {
            let hi_c = *range.end();
            junctions[hi_c].v_junction
        })
        .collect();

    let (sweeps, joining_status) = joining::join_until_converged(
        &chain_grids,
        &mut states,
        &corner_caps,
        input.worker_threads,
    )?;

    // Slice each chain profile into per-segment profiles and flatten.
    let profiles: Vec<TopProfile> = states
        .into_iter()
        .zip(chain_grids.iter())
        .flat_map(|(state, cg)| {
            let chain_profile = state.profile.expect("all chain profiles solved by stage 5");
            chain::slice_chain_profile(&chain_profile, &cg.segment_ranges)
        })
        .collect();

    let junction_infos: Vec<JunctionInfo> = junctions
        .into_iter()
        .enumerate()
        .map(|(j, junc)| {
            let v_junction = profiles[j].samples.last().map_or(0.0, |s| s.v);
            let binding = if junc.kind == junction::JunctionKind::Corner {
                junc.binding_cap
            } else if (v_junction - junc.v_junction).abs() < 1e-3 {
                junc.binding_cap
            } else {
                JunctionBindingCap::ChainInterior
            };
            JunctionInfo {
                between_segments: (j, j + 1),
                v_junction,
                binding_cap: binding,
                kappa_left: junc.kappa_left,
                kappa_right: junc.kappa_right,
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
