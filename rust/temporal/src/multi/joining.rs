//! Lookahead joining via SOCP-per-iteration (option A). Spec §2.3.

use crate::GridConfig;
use crate::TopProfile;
use crate::multi::junction::JunctionResult;
use crate::multi::parallel::fan_out_solves;
use crate::multi::{BatchError, JoiningStatus, SegmentInput};

/// Hard cap on joining sweeps. Per spec §2.3 + §6.5: typical convergence is
/// 1–3 sweeps; cap at 10 to detect bugs.
const MAX_SWEEPS: u32 = 10;

/// Run forward + reverse sweep pairs, re-solving dirty segments between sweeps,
/// until velocity propagation stabilizes or the sweep cap is reached.
///
/// Returns `(sweeps_used, JoiningStatus)`. Per spec §2.3 + §6.5.
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
            // Velocity propagation has stabilized — no segment's joining-decided
            // (v_start, v_end) changed in either sweep direction.
            if states.iter().all(|s| !s.dirty) {
                // Velocities stable AND every segment's last fan_out_solves
                // returned a verifier-feasible success status. Done.
                return Ok((sweep, JoiningStatus::Converged));
            }
            // Velocities stable but some segments still have dirty=true,
            // meaning their last fan_out_solves returned a non-success
            // status (Infeasible / MaxIter / DivergedSlp / MaxIterSlp —
            // all return Ok(profile) with non-success SolveStatus, leaving
            // dirty=true). Per kalico-verifier review-3: schedule_segment
            // is deterministic (Clarabel 0.11.1 with kalico's default
            // features uses single-threaded QDLDL; SLP loops have no RNG;
            // constraint construction is deterministic), so re-solving with
            // unchanged inputs would produce the same non-success status.
            // Bail early via the dedicated StalledOnInfeasibleSegment variant
            // (round-4 split — distinct from MAX_SWEEPS-exhaustion below).
            // The caller-side `ShapeError::TemporalJoining(status, detail)`
            // carries per-failing-segment diagnostic info (populated in
            // `beta.rs` after this returns) so this site doesn't need to
            // log directly.
            let last_dirty_count = states.iter().filter(|s| s.dirty).count();
            return Ok((
                sweep,
                JoiningStatus::StalledOnInfeasibleSegment { last_dirty_count },
            ));
        }
        fan_out_solves(inputs, states, grids, n_threads)?;
    }
    // Reached MAX_SWEEPS without velocity stabilization — pathological
    // joining oscillation (shouldn't happen on the test fixtures; if it
    // does, the joining algorithm itself has a bug). Distinct from
    // StalledOnInfeasibleSegment above.
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
/// This avoids directional overwrite oscillation. If a solve lowers the right
/// segment's achievable `v_start`, the same sweep lowers the left segment's
/// `v_end` instead of first raising the right side from stale left state.
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

/// Propagate junction velocities forward, marking dirty any segment whose
/// `v_start` changed beyond `EPS_VEL` since the last forward sweep.
///
/// Returns the number of segments marked dirty.
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

/// Propagate junction velocities backward, marking dirty any segment whose
/// `v_end` changed beyond `EPS_VEL` since the last reverse sweep.
///
/// Returns the number of segments marked dirty.
///
/// **Empty-buffer guard:** `if states.len() < 2` early-returns 0 to prevent
/// `usize` underflow on `0..states.len() - 1` when `states.len()` is 0 or 1.
/// The public `plan_batch` path filters empty buffers via
/// `BatchError::EmptySegments`, but this guard makes the helper safe to call
/// directly. A single-segment buffer has no junctions to propagate, so
/// early-return is correct. (Per round-4 review, Codex NIT.)
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
