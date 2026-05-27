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
