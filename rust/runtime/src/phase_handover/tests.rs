#![allow(clippy::indexing_slicing)]

use super::*;
use crate::state::SharedState;
use crate::stepping_state::{AxisState, StepMode, StepperRef};
use core::sync::atomic::Ordering;

fn axis_with_stepper(mode: StepMode, oid: u8) -> AxisState {
    let mut axis = AxisState::new_unconfigured();
    axis.mode.store(mode as u8, Ordering::Release);
    axis.microstep_distance = 0.000_625;
    axis.steppers.push(StepperRef::new(oid, Some(7))).unwrap();
    axis
}

#[test]
fn shortest_delta_forward() {
    assert_eq!(shortest_phase_delta(10, 44), 34);
}

#[test]
fn shortest_delta_wraps_backward() {
    // 1000 -> 10 is +34 through the wrap, not -990.
    assert_eq!(shortest_phase_delta(1000, 10), 34);
}

#[test]
fn shortest_delta_wraps_forward_negative() {
    assert_eq!(shortest_phase_delta(10, 1000), -34);
}

#[test]
fn shortest_delta_zero() {
    assert_eq!(shortest_phase_delta(512, 512), 0);
}

#[test]
fn shortest_delta_halfway_is_positive() {
    assert_eq!(shortest_phase_delta(0, 512), 512);
}

#[test]
fn find_stepper_locates_by_oid_across_axes() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    axes[0] = Some(axis_with_stepper(StepMode::Phase, 3));
    axes[2] = Some(axis_with_stepper(StepMode::Pulse, 9));
    let (axis_idx, _, stepper) = find_stepper(&axes, 9).unwrap();
    assert_eq!(axis_idx, 2);
    assert_eq!(stepper.stepper_oid, 9);
    assert!(find_stepper(&axes, 99).is_none());
}

#[test]
fn align_to_sets_both_offsets_and_matches_target_phase() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Pulse, 5);
    axis.last_step_count = 70_000; // 70000 & 0x3FF = 368
    axes[1] = Some(axis);
    assert_eq!(align_to(&axes, 5, 100), 0);
    let axis = axes[1].as_ref().unwrap();
    let stepper = &axis.steppers[0];
    let off = stepper.phase_offset_microsteps.load(Ordering::Acquire);
    assert_eq!(off, stepper.phase_offset_target.load(Ordering::Acquire));
    assert_eq!((axis.last_step_count.wrapping_add(off)) & 0x3FF, 100);
    // Shortest path: |delta| <= 512.
    assert!(off.abs() <= 512);
}

#[test]
fn align_to_rejects_unknown_oid_and_bad_phase() {
    let axes: [Option<AxisState>; 4] = [const { None }; 4];
    assert_ne!(align_to(&axes, 5, 100), 0);
    let mut axes2: [Option<AxisState>; 4] = [const { None }; 4];
    axes2[0] = Some(axis_with_stepper(StepMode::Pulse, 5));
    assert_ne!(align_to(&axes2, 5, 1024), 0);
}

#[test]
fn jog_to_moves_offset_target_by_shortest_path_requires_phase_mode() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Phase, 5);
    axis.last_step_count = 1020; // phase 1020
    axes[0] = Some(axis);
    let shared = SharedState::new();
    assert_eq!(jog_to(&axes, &shared, 5, 4, 1), 0);
    let stepper = &axes[0].as_ref().unwrap().steppers[0];
    // 1020 -> 4 is +8 through the wrap.
    assert_eq!(stepper.phase_offset_target.load(Ordering::Acquire), 8);
    assert_eq!(
        shared.max_phase_offset_ramp_per_sample.load(Ordering::Acquire),
        1
    );
    // Pulse mode is refused.
    axes[0]
        .as_ref()
        .unwrap()
        .mode
        .store(StepMode::Pulse as u8, Ordering::Release);
    assert_ne!(jog_to(&axes, &shared, 5, 4, 1), 0);
}

#[test]
fn jog_to_composes_with_pending_target_not_current_offset() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let axis = axis_with_stepper(StepMode::Phase, 5);
    axes[0] = Some(axis);
    let shared = SharedState::new();
    {
        let stepper = &axes[0].as_ref().unwrap().steppers[0];
        stepper.phase_offset_target.store(100, Ordering::Release);
        stepper.phase_offset_microsteps.store(40, Ordering::Release);
    }
    // last_step_count = 0, pending phase = 100; jog to 110 adds +10 on top
    // of the pending target, not on the in-flight current offset.
    assert_eq!(jog_to(&axes, &shared, 5, 110, 1), 0);
    let stepper = &axes[0].as_ref().unwrap().steppers[0];
    assert_eq!(stepper.phase_offset_target.load(Ordering::Acquire), 110);
}

#[test]
fn query_reports_phase_mode_and_settled() {
    let mut axes: [Option<AxisState>; 4] = [const { None }; 4];
    let mut axis = axis_with_stepper(StepMode::Phase, 5);
    axis.last_step_count = 2048; // phase 0
    axes[3] = Some(axis);
    {
        let stepper = &axes[3].as_ref().unwrap().steppers[0];
        stepper.phase_offset_microsteps.store(5, Ordering::Release);
        stepper.phase_offset_target.store(5, Ordering::Release);
    }
    let q = query(&axes, 5).unwrap();
    assert_eq!(q.axis_idx, 3);
    assert_eq!(q.mode, StepMode::Phase as u8);
    assert_eq!(q.phase, 5);
    assert!(q.settled);
    axes[3].as_ref().unwrap().steppers[0]
        .phase_offset_target
        .store(9, Ordering::Release);
    assert!(!query(&axes, 5).unwrap().settled);
}
