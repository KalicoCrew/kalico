//! `SegmentQueue` — facade over `heapless::spsc::Queue<Segment, 8>`.
//! Spec §3.1 / §4.7. Capacity 8 → effective 7 (heapless's N-1 rule).
//!
//! Producer half: foreground (test harness at Step 5; comms task at Step 6+).
//! Consumer half: ISR. ARMv7-M atomic ordering ships correct via heapless.

use crate::segment::Segment;
use heapless::spsc::Queue;

// NOTE: Step 5 keeps both producer and consumer accessing `&mut SegmentQueue` directly.
// Step 6 will split into `heapless::spsc::Producer<'a>` and `Consumer<'a>` halves once
// the live comms-task producer lands; the half-split formalizes the SPSC ownership
// at the type level. Step 5's single-threaded test harness doesn't need it.

/// Capacity parameter. Effective capacity = `Q_N - 1` per heapless 0.8.
pub const Q_N: usize = 8;

#[derive(Debug)]
pub struct SegmentQueue {
    inner: Queue<Segment, Q_N>,
}

impl Default for SegmentQueue {
    fn default() -> Self {
        Self::new()
    }
}

impl SegmentQueue {
    pub const fn new() -> Self {
        Self {
            inner: Queue::new(),
        }
    }

    /// Producer side: enqueue a segment.
    /// Returns `Err(seg)` if queue is full.
    #[inline]
    pub fn try_push(&mut self, seg: Segment) -> Result<(), Segment> {
        self.inner.enqueue(seg)
    }

    /// Consumer side: dequeue the next segment.
    #[inline]
    pub fn try_pop(&mut self) -> Option<Segment> {
        self.inner.dequeue()
    }

    /// Consumer side: read the next segment without removing it.
    #[inline]
    pub fn peek(&self) -> Option<&Segment> {
        self.inner.peek()
    }

    /// Returns `true` if there are no segments enqueued.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used)]
mod tests {
    use super::*;
    use crate::curve_pool::CurveHandle;
    use crate::segment::*;

    fn seg(id: u32, t_start: u64, t_end: u64) -> Segment {
        Segment {
            id,
            curve_handle: CurveHandle::new(0, 1),
            t_start,
            t_end,
            kinematics: KinematicTag::CoreXyAndE,
            flags: 0,
            _pad: [0; 2],
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
}
