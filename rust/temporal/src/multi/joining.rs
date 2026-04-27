//! Lookahead joining via SOCP-per-iteration (option A). Spec §2.3.

use crate::multi::junction::JunctionResult;
use crate::TopProfile;

/// Per-segment scratch state during joining.
// TODO(task-9): wired in plan_batch
#[allow(dead_code)]
pub(crate) struct SegmentState {
    pub v_start: f64,
    pub v_end: f64,
    pub profile: Option<TopProfile>,
    pub dirty: bool,
}

/// Propagate junction velocities forward, marking dirty any segment whose
/// `v_start` changed beyond `EPS_VEL` since the last forward sweep.
///
/// Returns the number of segments marked dirty.
// TODO(task-9): wired in plan_batch
#[allow(dead_code)]
pub(crate) fn forward_sweep(
    states: &mut [SegmentState],
    junctions: &[JunctionResult],
) -> usize {
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
// TODO(task-9): wired in plan_batch
#[allow(dead_code)]
pub(crate) fn reverse_sweep(
    states: &mut [SegmentState],
    junctions: &[JunctionResult],
) -> usize {
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
        SegmentState { v_start, v_end, profile: None, dirty: false }
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
        let mut states = vec![
            make_state(0.0, 100.0),
            make_state(0.0, 200.0),
        ];
        let junctions = vec![make_junction(150.0)];
        let dirty = forward_sweep(&mut states, &junctions);
        // junctions[0] = 150, states[0].v_end = 100; min = 100. New v_start[1] = 100.
        assert_eq!(dirty, 1);
        assert!((states[1].v_start - 100.0).abs() < 1e-6);
        assert!(states[1].dirty);
    }

    #[test]
    fn forward_no_change_no_dirty() {
        let mut states = vec![
            make_state(0.0, 150.0),
            make_state(150.0, 200.0),
        ];
        let junctions = vec![make_junction(150.0)];
        let dirty = forward_sweep(&mut states, &junctions);
        assert_eq!(dirty, 0);
        assert!(!states[1].dirty);
    }

    #[test]
    fn reverse_propagates_v_start_to_prev_v_end() {
        let mut states = vec![
            make_state(0.0, 200.0),
            make_state(100.0, 200.0),
        ];
        let junctions = vec![make_junction(150.0)];
        let dirty = reverse_sweep(&mut states, &junctions);
        // junctions[0] = 150, states[1].v_start = 100; min = 100. New v_end[0] = 100.
        assert_eq!(dirty, 1);
        assert!((states[0].v_end - 100.0).abs() < 1e-6);
    }
}
