//! Host tests for the step-output timer body. Covers: soonest-across-owned
//! scan, wrap-safe selection, due/late emit, per-dispatch cap, all-empty
//! disable, and unowned-axis isolation. Hardware firing is MCU-only.

// Test code: `.expect()` is the intended failure signal for queue-push errors;
// the production expect_used deny does not apply to tests.
#![allow(clippy::expect_used)]

use super::test_hooks::{queue_for_axis, reset, set_now, set_owned_mask, take_emits};
use super::{MAX_STEPS_PER_EVENT, STEP_OUTPUT_DISABLE, kalico_step_output_event};
use crate::step_queue::{StepEntry, push};

fn entry(cycle_abs: u32, dir: i8) -> StepEntry {
    StepEntry {
        cycle_abs,
        dir,
        _pad: [0; 3],
    }
}

/// Push one entry onto axis `axis`'s test queue.
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
    // Head is far in the future → not emitted, returned as the next wake.
    assert_eq!(kalico_step_output_event(), 5000);
    assert!(take_emits().is_empty());
}

#[test]
fn arrived_head_emitted() {
    reset();
    set_now(2000);
    set_owned_mask(0b0001);
    enqueue(0, 1500, 1); // already due (cycle_abs < now)
    let next = kalico_step_output_event();
    let emits = take_emits();
    assert_eq!(emits, vec![(0u8, 1i32)]);
    // Queue now empty → disable.
    assert_eq!(next, STEP_OUTPUT_DISABLE);
}

#[test]
fn exactly_now_is_due() {
    reset();
    set_now(2000);
    set_owned_mask(0b0001);
    enqueue(0, 2000, -1); // cycle_abs == now → due (DUE_WINDOW = 0)
    let next = kalico_step_output_event();
    assert_eq!(take_emits(), vec![(0u8, -1i32)]);
    assert_eq!(next, STEP_OUTPUT_DISABLE);
}

#[test]
fn soonest_across_owned_axes_selected() {
    reset();
    set_now(1000);
    set_owned_mask(0b0011); // axes 0 and 1 owned
    enqueue(0, 4000, 1);
    enqueue(1, 3000, 1); // sooner
    // Both future → none emitted; returns the smaller cycle_abs.
    assert_eq!(kalico_step_output_event(), 3000);
    assert!(take_emits().is_empty());
}

#[test]
fn soonest_is_wrap_safe_across_u32_boundary() {
    reset();
    // now near the top of u32: one head just before wrap, one just after.
    let now = u32::MAX - 100;
    set_now(now);
    set_owned_mask(0b0011);
    // axis 0 head 50 cycles ahead (before wrap), axis 1 head 200 cycles ahead
    // (past the wrap boundary). 50 < 200 so axis 0 is soonest.
    let near = now.wrapping_add(50);
    let far = now.wrapping_add(200); // wraps past 0
    enqueue(0, near, 1);
    enqueue(1, far, 1);
    assert_eq!(kalico_step_output_event(), near);
    assert!(take_emits().is_empty());

    // Reverse the assignment; the post-wrap entry on axis 0 should still be
    // recognised as the *later* one, axis 1's pre-wrap head as soonest.
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
    set_owned_mask(0b0001); // only axis 0 owned
    enqueue(1, 1000, 1); // axis 1 due but UNOWNED
    // Axis 1 must not be emitted nor selected; nothing owned has work → disable.
    assert_eq!(kalico_step_output_event(), STEP_OUTPUT_DISABLE);
    assert!(take_emits().is_empty());
}

#[test]
fn single_axis_ordering_emits_all_due_then_returns_future() {
    reset();
    set_now(5000);
    set_owned_mask(0b0001);
    // Three due, one future.
    enqueue(0, 1000, 1);
    enqueue(0, 2000, 1);
    enqueue(0, 3000, 1);
    enqueue(0, 9000, 1);
    let next = kalico_step_output_event();
    assert_eq!(take_emits(), vec![(0, 1), (0, 1), (0, 1)]);
    // The remaining future head is returned as next wake.
    assert_eq!(next, 9000);
}

#[test]
fn per_dispatch_cap_returns_now_with_work_remaining() {
    reset();
    let now = 100_000u32;
    set_now(now);
    set_owned_mask(0b0011); // two axes
    // A single SPSC ring holds at most STEP_QUEUE_DEPTH-1 (=31) outstanding,
    // which is below the cap. Spread the due backlog across two owned axes so
    // the total due (62) exceeds MAX_STEPS_PER_EVENT (32) and the cap trips.
    let per_axis = crate::step_queue::STEP_QUEUE_DEPTH as u32 - 1; // 31
    for i in 0..per_axis {
        enqueue(0, 1000 + i, 1); // all due (< now)
        enqueue(1, 1000 + i, 1);
    }
    let next = kalico_step_output_event();
    let emits = take_emits();
    assert_eq!(emits.len() as u32, MAX_STEPS_PER_EVENT);
    // Cap hit with work remaining → re-fire immediately at `now`.
    assert_eq!(next, now);
}

#[test]
fn queue_full_push_fails_loud() {
    // The SPSC `push` returns Err once 31 entries are outstanding; the
    // dispatch path (tick.rs) escalates that to FaultCode::StepQueueOverflow
    // (-300). Here we assert the queue API itself fails loud rather than
    // silently overwriting.
    reset();
    let q = queue_for_axis(0);
    // Fill to STEP_QUEUE_DEPTH; the depth-th push must fail.
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
    set_owned_mask(0b0101); // axes 0 and 2 owned (axis 1 not)
    enqueue(0, 1000, 1); // due
    enqueue(2, 1500, -1); // due
    enqueue(0, 8000, 1); // future
    enqueue(2, 6000, 1); // future
    let next = kalico_step_output_event();
    let emits = take_emits();
    // Both due steps emitted (order: axis 0 then axis 2 within a pass).
    assert_eq!(emits.len(), 2);
    assert!(emits.contains(&(0u8, 1i32)));
    assert!(emits.contains(&(2u8, -1i32)));
    // Soonest remaining future head is axis 2 @ 6000.
    assert_eq!(next, 6000);
}
