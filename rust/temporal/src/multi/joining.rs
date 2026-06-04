use crate::GridConfig;
use crate::TopProfile;
use crate::multi::junction::JunctionResult;
use crate::multi::parallel::fan_out_solves;
use crate::multi::{BatchError, JoiningStatus, SegmentInput};

/// Hard cap on joining sweeps. Typical convergence is 1–3 sweeps.
const MAX_SWEEPS: u32 = 10;

/// Run forward + reverse sweep pairs, re-solving dirty segments between sweeps,
/// until velocity propagation stabilizes or the sweep cap is reached.
///
/// Returns `(sweeps_used, JoiningStatus)`.
pub(crate) fn join_until_converged(
    inputs: &[SegmentInput<'_>],
    grids: &[GridConfig],
    states: &mut [SegmentState],
    junctions: &[JunctionResult],
    n_threads: usize,
) -> Result<(u32, JoiningStatus), BatchError> {
    for sweep in 1..=MAX_SWEEPS {
        let dirty_count = bidirectional_junction_sweep(states, junctions);
        if dirty_count == 0 {
            if states.iter().all(|s| !s.dirty) {
                return Ok((sweep, JoiningStatus::Converged));
            }
            // Velocities stable but some segments still have dirty=true from a
            // non-success solver status. schedule_segment is deterministic, so
            // re-solving with unchanged inputs would produce the same result.
            let last_dirty_count = states.iter().filter(|s| s.dirty).count();
            return Ok((
                sweep,
                JoiningStatus::StalledOnInfeasibleSegment { last_dirty_count },
            ));
        }
        fan_out_solves(inputs, states, grids, n_threads)?;
    }
    let last_dirty = states.iter().filter(|s| s.dirty).count();
    Ok((
        MAX_SWEEPS,
        JoiningStatus::CappedAtMaxSweeps {
            last_dirty_count: last_dirty,
        },
    ))
}

/// Per-segment scratch state during joining.
pub(crate) struct SegmentState {
    pub v_start: f64,
    pub v_end: f64,
    pub profile: Option<TopProfile>,
    pub dirty: bool,
}

/// Propagate each junction as a simultaneous two-sided cap:
/// `v_j = min(v_cap, v_left_end, v_right_start)`.
///
/// Avoids directional overwrite oscillation.
pub(crate) fn bidirectional_junction_sweep(
    states: &mut [SegmentState],
    junctions: &[JunctionResult],
) -> usize {
    const EPS_VEL: f64 = 1e-3;
    let mut dirty_count = 0;

    for k in 0..junctions.len() {
        let target = junctions[k]
            .v_junction
            .min(states[k].v_end)
            .min(states[k + 1].v_start);

        if (target - states[k].v_end).abs() > EPS_VEL {
            states[k].v_end = target;
            states[k].dirty = true;
            dirty_count += 1;
        }
        if (target - states[k + 1].v_start).abs() > EPS_VEL {
            states[k + 1].v_start = target;
            states[k + 1].dirty = true;
            dirty_count += 1;
        }
    }

    dirty_count
}

#[cfg(test)]
pub(crate) fn forward_sweep(states: &mut [SegmentState], junctions: &[JunctionResult]) -> usize {
    const EPS_VEL: f64 = 1e-3;
    let mut dirty_count = 0;
    for k in 1..states.len() {
        let proposed_v_start = junctions[k - 1].v_junction.min(states[k - 1].v_end);
        if (proposed_v_start - states[k].v_start).abs() > EPS_VEL {
            states[k].v_start = proposed_v_start;
            states[k].dirty = true;
            dirty_count += 1;
        }
    }
    dirty_count
}

/// Empty-buffer guard: returns 0 when `states.len() < 2` to prevent usize
/// underflow on `0..states.len() - 1`.
#[cfg(test)]
pub(crate) fn reverse_sweep(states: &mut [SegmentState], junctions: &[JunctionResult]) -> usize {
    const EPS_VEL: f64 = 1e-3;
    if states.len() < 2 {
        return 0;
    }
    let mut dirty_count = 0;
    for k in (0..states.len() - 1).rev() {
        let proposed_v_end = junctions[k].v_junction.min(states[k + 1].v_start);
        if (proposed_v_end - states[k].v_end).abs() > EPS_VEL {
            states[k].v_end = proposed_v_end;
            states[k].dirty = true;
            dirty_count += 1;
        }
    }
    dirty_count
}

#[cfg(test)]
mod tests;
