//! Lookahead joining via SOCP-per-iteration (option A). Spec §2.3.

use crate::multi::junction::JunctionResult;
use crate::multi::parallel::fan_out_solves;
use crate::multi::{BatchError, JoiningStatus, SegmentInput};
use crate::GridConfig;
use crate::TopProfile;

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
mod tests {
    use super::*;
    use crate::multi::JunctionBindingCap;

    fn make_state(v_start: f64, v_end: f64) -> SegmentState {
        SegmentState {
            v_start,
            v_end,
            profile: None,
            dirty: false,
        }
    }

    fn make_junction(v: f64) -> JunctionResult {
        JunctionResult {
            v_junction: v,
            binding_cap: JunctionBindingCap::Centripetal,
            kappa_left: 0.0,
            kappa_right: 0.0,
        }
    }

    #[test]
    fn forward_propagates_v_end_to_next_v_start() {
        let mut states = vec![make_state(0.0, 100.0), make_state(0.0, 200.0)];
        let junctions = vec![make_junction(150.0)];
        let dirty = forward_sweep(&mut states, &junctions);
        // junctions[0] = 150, states[0].v_end = 100; min = 100. New v_start[1] = 100.
        assert_eq!(dirty, 1);
        assert!((states[1].v_start - 100.0).abs() < 1e-6);
        assert!(states[1].dirty);
    }

    #[test]
    fn forward_no_change_no_dirty() {
        let mut states = vec![make_state(0.0, 150.0), make_state(150.0, 200.0)];
        let junctions = vec![make_junction(150.0)];
        let dirty = forward_sweep(&mut states, &junctions);
        assert_eq!(dirty, 0);
        assert!(!states[1].dirty);
    }

    #[test]
    fn reverse_propagates_v_start_to_prev_v_end() {
        let mut states = vec![make_state(0.0, 200.0), make_state(100.0, 200.0)];
        let junctions = vec![make_junction(150.0)];
        let dirty = reverse_sweep(&mut states, &junctions);
        // junctions[0] = 150, states[1].v_start = 100; min = 100. New v_end[0] = 100.
        assert_eq!(dirty, 1);
        assert!((states[0].v_end - 100.0).abs() < 1e-6);
    }

    #[test]
    fn bidirectional_sweep_uses_lower_achieved_side() {
        let mut states = vec![make_state(0.0, 120.0), make_state(80.0, 0.0)];
        let junctions = vec![make_junction(150.0)];

        let dirty = bidirectional_junction_sweep(&mut states, &junctions);

        assert_eq!(dirty, 1);
        assert!((states[0].v_end - 80.0).abs() < 1e-6);
        assert!((states[1].v_start - 80.0).abs() < 1e-6);
        assert!(states[0].dirty);
        assert!(!states[1].dirty);
    }

    #[test]
    fn converges_in_one_sweep_on_already_consistent() {
        // Stub test — full plan_batch test in Task 9. Direct join_until_converged
        // requires SegmentInput + GridConfig setup, which is integration-test scope.
        // Unit-test path: assert forward_sweep + reverse_sweep both no-op on a
        // pre-balanced state.
        let mut states = vec![make_state(0.0, 150.0), make_state(150.0, 200.0)];
        let junctions = vec![make_junction(150.0)];
        let f_dirty = forward_sweep(&mut states, &junctions);
        let r_dirty = reverse_sweep(&mut states, &junctions);
        assert_eq!(f_dirty, 0);
        assert_eq!(r_dirty, 0);
        // join_until_converged would return Converged in one sweep with no re-solves.
    }
}
