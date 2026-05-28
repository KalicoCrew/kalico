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
mod tests;
