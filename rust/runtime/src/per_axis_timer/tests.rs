#![allow(clippy::expect_used)]

use super::test_hooks::{
    queue_for_axis, reset, set_late_threshold, set_now, set_owned_mask, take_emits, take_late_stats,
};
use super::{MAX_STEPS_PER_EVENT, STEP_OUTPUT_DISABLE, kalico_step_output_event};
use crate::step_queue::{StepEntry, push};

fn entry(cycle_abs: u32, dir: i8) -> StepEntry {
    StepEntry {
        cycle_abs,
        dir,
        _pad: [0; 3],
    }
}

fn enqueue(axis: usize, cycle_abs: u32, dir: i8) {
    let q = queue_for_axis(axis);
    assert!(!q.is_null());
    // SAFETY: host test queue, sole producer here.
    unsafe { push(q, entry(cycle_abs, dir)).expect("queue not full") };
}

#[test]
fn all_empty_returns_disable() {
    reset();
    set_now(1000);
    set_owned_mask(0b0001);
    assert_eq!(kalico_step_output_event(), STEP_OUTPUT_DISABLE);
    assert!(take_emits().is_empty());
}

#[test]
fn far_future_head_not_emitted_returned_as_wake() {
    reset();
    set_now(1000);
    set_owned_mask(0b0001);
    enqueue(0, 5000, 1);
    assert_eq!(kalico_step_output_event(), 5000);
    assert!(take_emits().is_empty());
}

#[test]
fn arrived_head_emitted() {
    reset();
    set_now(2000);
    set_owned_mask(0b0001);
    enqueue(0, 1500, 1);
    let next = kalico_step_output_event();
    let emits = take_emits();
    assert_eq!(emits, vec![(0u8, 1i32)]);
    assert_eq!(next, STEP_OUTPUT_DISABLE);
}

#[test]
fn exactly_now_is_due() {
    reset();
    set_now(2000);
    set_owned_mask(0b0001);
    enqueue(0, 2000, -1);
    let next = kalico_step_output_event();
    assert_eq!(take_emits(), vec![(0u8, -1i32)]);
    assert_eq!(next, STEP_OUTPUT_DISABLE);
}

#[test]
fn soonest_across_owned_axes_selected() {
    reset();
    set_now(1000);
    set_owned_mask(0b0011);
    enqueue(0, 4000, 1);
    enqueue(1, 3000, 1);
    assert_eq!(kalico_step_output_event(), 3000);
    assert!(take_emits().is_empty());
}

#[test]
fn soonest_is_wrap_safe_across_u32_boundary() {
    reset();
    let now = u32::MAX - 100;
    set_now(now);
    set_owned_mask(0b0011);
    let near = now.wrapping_add(50);
    let far = now.wrapping_add(200);
    enqueue(0, near, 1);
    enqueue(1, far, 1);
    assert_eq!(kalico_step_output_event(), near);
    assert!(take_emits().is_empty());

    reset();
    set_now(now);
    set_owned_mask(0b0011);
    enqueue(0, far, 1);
    enqueue(1, near, 1);
    assert_eq!(kalico_step_output_event(), near);
}

#[test]
fn unowned_axis_with_due_head_is_ignored() {
    reset();
    set_now(2000);
    set_owned_mask(0b0001);
    enqueue(1, 1000, 1);
    assert_eq!(kalico_step_output_event(), STEP_OUTPUT_DISABLE);
    assert!(take_emits().is_empty());
}

#[test]
fn single_axis_ordering_emits_all_due_then_returns_future() {
    reset();
    set_now(5000);
    set_owned_mask(0b0001);
    enqueue(0, 1000, 1);
    enqueue(0, 2000, 1);
    enqueue(0, 3000, 1);
    enqueue(0, 9000, 1);
    let next = kalico_step_output_event();
    assert_eq!(take_emits(), vec![(0, 1), (0, 1), (0, 1)]);
    assert_eq!(next, 9000);
}

#[test]
fn per_dispatch_cap_returns_now_with_work_remaining() {
    reset();
    let now = 100_000u32;
    set_now(now);
    set_owned_mask(0b0011);
    let per_axis = crate::step_queue::STEP_QUEUE_DEPTH as u32 - 1;
    for i in 0..per_axis {
        enqueue(0, 1000 + i, 1);
        enqueue(1, 1000 + i, 1);
    }
    let next = kalico_step_output_event();
    let emits = take_emits();
    assert_eq!(emits.len() as u32, MAX_STEPS_PER_EVENT);
    assert_eq!(next, now);
}

#[test]
fn queue_full_push_fails_loud() {
    reset();
    let q = queue_for_axis(0);
    let mut pushed = 0u32;
    loop {
        // SAFETY: host test queue, sole producer.
        let r = unsafe { push(q, entry(pushed, 1)) };
        if r.is_err() {
            break;
        }
        pushed += 1;
        assert!(pushed <= crate::step_queue::STEP_QUEUE_DEPTH as u32);
    }
    assert_eq!(pushed, crate::step_queue::STEP_QUEUE_DEPTH as u32);
}

#[test]
fn mixed_due_and_future_across_axes() {
    reset();
    set_now(2000);
    set_owned_mask(0b0101);
    enqueue(0, 1000, 1);
    enqueue(2, 1500, -1);
    enqueue(0, 8000, 1);
    enqueue(2, 6000, 1);
    let next = kalico_step_output_event();
    let emits = take_emits();
    assert_eq!(emits.len(), 2);
    assert!(emits.contains(&(0u8, 1i32)));
    assert!(emits.contains(&(2u8, -1i32)));
    assert_eq!(next, 6000);
}

// ---- lateness tracking ----

#[test]
fn on_time_emission_produces_zero_late_stats() {
    reset();
    set_now(1000);
    set_owned_mask(0b0001);
    set_late_threshold(500);
    enqueue(0, 1000, 1);
    kalico_step_output_event();
    let (max_late, late_count, max_drained) = take_late_stats();
    assert_eq!(max_late, 0, "on-time emission must not bump max_late");
    assert_eq!(late_count, 0, "on-time emission must not bump late_count");
    assert_eq!(max_drained, 1);
}

#[test]
fn late_emission_exceeding_threshold_increments_stats() {
    reset();
    set_now(5000);
    set_owned_mask(0b0001);
    set_late_threshold(500);
    enqueue(0, 1000, 1);
    kalico_step_output_event();
    let _ = take_emits();
    let (max_late, late_count, _) = take_late_stats();
    assert_eq!(late_count, 1);
    assert_eq!(max_late, 5000u32.wrapping_sub(1000));
}

#[test]
fn late_by_exactly_threshold_not_counted() {
    reset();
    set_now(1500);
    set_owned_mask(0b0001);
    set_late_threshold(500);
    enqueue(0, 1000, 1);
    kalico_step_output_event();
    let _ = take_emits();
    let (max_late, late_count, _) = take_late_stats();
    assert_eq!(late_count, 0, "lateness == threshold is not > threshold");
    assert_eq!(max_late, 0);
}

#[test]
fn max_late_tracks_worst_across_multiple_events() {
    reset();
    set_owned_mask(0b0001);
    set_late_threshold(100);

    set_now(2000);
    enqueue(0, 1500, 1);
    kalico_step_output_event();
    let _ = take_emits();

    set_now(3000);
    enqueue(0, 1000, 1);
    kalico_step_output_event();
    let _ = take_emits();

    let (max_late, late_count, _) = take_late_stats();
    assert_eq!(late_count, 2);
    assert_eq!(max_late, 3000u32.wrapping_sub(1000));
}

#[test]
fn max_drained_tracks_largest_batch() {
    reset();
    set_now(9000);
    set_owned_mask(0b0001);
    set_late_threshold(500);
    enqueue(0, 1000, 1);
    enqueue(0, 2000, 1);
    enqueue(0, 3000, 1);
    enqueue(0, 4000, 1);
    kalico_step_output_event();
    let _ = take_emits();
    let (_, _, max_drained) = take_late_stats();
    assert_eq!(max_drained, 4, "all four entries drain in one event");
}

#[test]
fn future_only_entry_produces_zero_late_and_zero_drained() {
    reset();
    set_now(1000);
    set_owned_mask(0b0001);
    set_late_threshold(500);
    enqueue(0, 5000, 1);
    kalico_step_output_event();
    assert!(take_emits().is_empty());
    let (max_late, late_count, max_drained) = take_late_stats();
    assert_eq!(max_late, 0);
    assert_eq!(late_count, 0);
    assert_eq!(max_drained, 0, "nothing was drained so max_drained stays 0");
}
