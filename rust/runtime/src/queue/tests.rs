#![allow(clippy::unwrap_used, clippy::expect_used)]

use super::*;
use crate::segment::EMode;
use crate::segment::CurveHandle;
use crate::segment::*;

fn seg(id: u32, t_start: u64, t_end: u64) -> Segment {
    Segment {
        id,
        x_handle: CurveHandle::new(0, 1),
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start,
        t_end,
        kinematics: KinematicTag::CoreXyAndE,
        e_mode: EMode::CoupledToXy,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    }
}

#[test]
fn capacity_is_seven_for_size_eight() {
    // heapless::spsc::Queue capacity is N - 1.
    let mut q = SegmentQueue::new();
    for i in 0..7 {
        assert!(
            q.try_push(seg(i, 0, 100)).is_ok(),
            "push {i} should succeed"
        );
    }
    assert!(q.try_push(seg(7, 0, 100)).is_err(), "8th push must fail");
}

#[test]
fn fifo_ordering() {
    let mut q = SegmentQueue::new();
    q.try_push(seg(10, 0, 100)).expect("push 10");
    q.try_push(seg(20, 0, 100)).expect("push 20");
    q.try_push(seg(30, 0, 100)).expect("push 30");
    assert_eq!(q.try_pop().expect("pop 10").id, 10);
    assert_eq!(q.try_pop().expect("pop 20").id, 20);
    assert_eq!(q.peek().expect("peek 30").id, 30);
    assert_eq!(q.try_pop().expect("pop 30").id, 30);
    assert!(q.try_pop().is_none());
}

#[test]
fn peek_does_not_consume() {
    let mut q = SegmentQueue::new();
    q.try_push(seg(1, 0, 100)).expect("push 1");
    assert_eq!(q.peek().expect("first peek").id, 1);
    assert_eq!(q.peek().expect("second peek").id, 1); // peek again — same value
    assert_eq!(q.try_pop().expect("pop 1").id, 1);
    assert!(q.peek().is_none());
}
