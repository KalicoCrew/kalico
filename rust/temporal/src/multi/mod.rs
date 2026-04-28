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

/// Run the full multi-segment planning pipeline on a batch of curve segments.
///
/// # Errors
/// - [`BatchError::EmptySegments`] — `input.segments` is empty.
/// - [`BatchError::InvalidThreads`] — `input.worker_threads` is zero.
/// - [`BatchError::Segment`] — a segment-level [`crate::ScheduleError`] was
///   returned by [`crate::topp::schedule_segment_with_tolerance`].
///
/// # Pipeline
/// 1. Validate inputs.
/// 2. Compute per-segment grid sizes via [`multi::grid::compute_n`] and
///    [`BatchInput::grid_strategy`].
/// 3. Compute k−1 junction velocities via
///    [`multi::junction::compute_junction_velocity`].
/// 4. Seed per-segment [`multi::joining::SegmentState`] from junction velocities.
/// 5. Initial [`multi::parallel::fan_out_solves`] (all segments dirty).
/// 6. Joining loop via [`multi::joining::join_until_converged`].
/// 7. Assemble [`BatchOutput`].
pub fn plan_batch(input: BatchInput<'_>) -> Result<BatchOutput, BatchError> {
    use crate::multi::{grid, joining, junction, parallel};
    use crate::GridConfig;

    if input.segments.is_empty() {
        return Err(BatchError::EmptySegments);
    }
    if input.worker_threads == 0 {
        return Err(BatchError::InvalidThreads);
    }

    let k = input.segments.len();

    // Stage 1: per-segment grid sizes.
    let grids: Vec<GridConfig> = input
        .segments
        .iter()
        .map(|s| GridConfig {
            scheme: crate::GridScheme::UniformArclength,
            n: grid::compute_n(&input.grid_strategy, s.curve),
        })
        .collect();

    // Stage 2: junction velocities (k-1 junctions).
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

    // Stage 3: seed per-segment states.
    let mut states: Vec<joining::SegmentState> = (0..k)
        .map(|i| {
            let v_start = if i == 0 { 0.0 } else { junctions[i - 1].v_junction };
            let v_end = if i == k - 1 { 0.0 } else { junctions[i].v_junction };
            joining::SegmentState { v_start, v_end, profile: None, dirty: true }
        })
        .collect();

    // Stage 4: initial fan-out (all dirty).
    parallel::fan_out_solves(input.segments, &mut states, &grids, input.worker_threads)?;

    // Stage 5: joining loop with in-loop re-solves (review-1 corrected algorithm).
    let (sweeps, joining_status) = joining::join_until_converged(
        input.segments,
        &grids,
        &mut states,
        &junctions,
        input.worker_threads,
    )?;

    // Stage 6: assemble output.
    let profiles: Vec<_> = states
        .into_iter()
        .map(|s| s.profile.expect("all profiles solved by stage 5"))
        .collect();
    let junction_infos: Vec<JunctionInfo> = junctions
        .into_iter()
        .enumerate()
        .map(|(i, j)| JunctionInfo {
            between_segments: (i, i + 1),
            v_junction: j.v_junction,
            binding_cap: j.binding_cap,
            kappa_left: j.kappa_left,
            kappa_right: j.kappa_right,
        })
        .collect();
    Ok(BatchOutput { profiles, junctions: junction_infos, joining_sweeps: sweeps, joining_status })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Limits;
    use nurbs::VectorNurbs;

    fn straight_50mm() -> VectorNurbs<f64, 3> {
        VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [50.0, 0.0, 0.0]],
            None,
        )
        .unwrap()
    }

    fn textbook_limits() -> Limits {
        Limits {
            v_max: [500.0; 3],
            a_max: [5_000.0; 3],
            j_max: [100_000.0; 3],
            a_centripetal_max: 2_500.0,
        }
    }

    #[test]
    fn plan_batch_single_segment_works() {
        let curve = straight_50mm();
        let segment = SegmentInput {
            curve: &curve,
            limits: textbook_limits(),
            trailing_junction_chord_tolerance_mm: 0.05,
        };
        let input = BatchInput {
            segments: &[segment],
            grid_strategy: GridStrategy::Adaptive {
                min_n: 10,
                max_n: 200,
                target_grid_spacing_mm: 0.5,
            },
            worker_threads: 1,
        };
        let output = plan_batch(input).expect("should succeed");
        assert_eq!(output.profiles.len(), 1);
        assert!(output.junctions.is_empty());
        // Single segment endpoints both 0.
        assert!(output.profiles[0].samples[0].v < 1e-3);
    }
}

mod grid;
mod joining;
mod junction;
mod parallel;
