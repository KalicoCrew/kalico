//! Per-command-queue pair of sorted lists: upcoming (gated by `min_clock`)
//! and ready (ordered by `req_clock`).

use super::entry::{PassthroughEntry, BACKGROUND_PRIORITY_CLOCK};

#[derive(Debug, Default)]
pub struct CommandQueue {
    /// Sorted ascending by `min_clock`. Entries stay here until
    /// `ack_clock >= entry.min_clock()`.
    upcoming: Vec<PassthroughEntry>,
    /// Sorted ascending by `req_clock`. Head is the next to emit.
    ready: Vec<PassthroughEntry>,
    ack_clock: u64,
}

impl CommandQueue {
    pub fn new() -> Self {
        Self::default()
    }

    /// Push an entry, routing it to `ready` (if `min_clock <= ack_clock`) or
    /// `upcoming` (if not yet eligible).
    pub fn push(&mut self, entry: PassthroughEntry) {
        if entry.min_clock() <= self.ack_clock {
            sorted_insert_by_req_clock(&mut self.ready, entry);
        } else {
            sorted_insert_by_min_clock(&mut self.upcoming, entry);
        }
    }

    /// Advance the ack clock and move all newly-eligible entries from
    /// `upcoming` to `ready`.
    pub fn promote(&mut self, ack_clock: u64) {
        self.ack_clock = ack_clock;
        // upcoming is sorted by min_clock ascending, so we can drain from
        // the front until we hit one that isn't eligible yet.
        let split = self.upcoming.partition_point(|e| e.min_clock() <= ack_clock);
        let promoted: Vec<_> = self.upcoming.drain(..split).collect();
        for entry in promoted {
            sorted_insert_by_req_clock(&mut self.ready, entry);
        }
    }

    /// Remove and return the head of the ready queue (lowest `req_clock`).
    pub fn pop_ready(&mut self) -> Option<PassthroughEntry> {
        if self.ready.is_empty() {
            None
        } else {
            Some(self.ready.remove(0))
        }
    }

    /// Peek at the `req_clock` of the ready-queue head without removing it.
    pub fn peek_ready_req_clock(&self) -> Option<u64> {
        self.ready.first().map(PassthroughEntry::req_clock)
    }

    pub fn is_ready_empty(&self) -> bool {
        self.ready.is_empty()
    }

    /// Returns `true` if the ready queue has at least one non-background entry.
    pub fn has_non_background_ready(&self) -> bool {
        self.ready
            .first()
            .map_or(false, |e| e.req_clock() != BACKGROUND_PRIORITY_CLOCK)
    }

    /// True when there are ready entries but all of them are background.
    pub fn has_only_background_ready(&self) -> bool {
        !self.ready.is_empty() && !self.has_non_background_ready()
    }
}

fn sorted_insert_by_req_clock(vec: &mut Vec<PassthroughEntry>, entry: PassthroughEntry) {
    let pos = vec.partition_point(|e| e.req_clock() <= entry.req_clock());
    vec.insert(pos, entry);
}

fn sorted_insert_by_min_clock(vec: &mut Vec<PassthroughEntry>, entry: PassthroughEntry) {
    let pos = vec.partition_point(|e| e.min_clock() <= entry.min_clock());
    vec.insert(pos, entry);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::passthrough_queue::entry::NotifyId;

    fn entry(min_clock: u64, req_clock: u64) -> PassthroughEntry {
        PassthroughEntry::new(vec![0x01], min_clock, req_clock, NotifyId::none())
    }

    #[test]
    fn push_routes_by_min_clock_vs_ack_clock() {
        let mut q = CommandQueue::new();
        // ack_clock is 0, so min_clock=0 goes to ready, min_clock=10 to upcoming
        q.push(entry(0, 50));
        q.push(entry(10, 40));

        assert_eq!(q.peek_ready_req_clock(), Some(50));
        // The min_clock=10 entry is not in ready
        assert_eq!(q.ready.len(), 1);
        assert_eq!(q.upcoming.len(), 1);
    }

    #[test]
    fn ready_orders_by_req_clock_not_min_clock() {
        let mut q = CommandQueue::new();
        // All with min_clock=0 so they go straight to ready.
        q.push(entry(0, 300));
        q.push(entry(0, 100));
        q.push(entry(0, 200));

        assert_eq!(q.pop_ready().unwrap().req_clock(), 100);
        assert_eq!(q.pop_ready().unwrap().req_clock(), 200);
        assert_eq!(q.pop_ready().unwrap().req_clock(), 300);
    }

    #[test]
    fn promote_moves_when_min_clock_reached() {
        let mut q = CommandQueue::new();
        q.push(entry(10, 50));
        q.push(entry(20, 40));

        assert!(q.is_ready_empty());

        q.promote(10);
        // Only the min_clock=10 entry should have moved
        assert_eq!(q.peek_ready_req_clock(), Some(50));
        assert_eq!(q.ready.len(), 1);
        assert_eq!(q.upcoming.len(), 1);

        q.promote(20);
        // Now both are ready — and ordered by req_clock
        assert_eq!(q.pop_ready().unwrap().req_clock(), 40);
        assert_eq!(q.pop_ready().unwrap().req_clock(), 50);
    }

    #[test]
    fn promote_preserves_min_clock_order_for_remaining() {
        let mut q = CommandQueue::new();
        q.push(entry(30, 1));
        q.push(entry(10, 2));
        q.push(entry(20, 3));

        q.promote(15);
        // Only min_clock=10 promoted
        assert_eq!(q.ready.len(), 1);
        // Remaining upcoming should still be sorted by min_clock
        assert_eq!(q.upcoming[0].min_clock(), 20);
        assert_eq!(q.upcoming[1].min_clock(), 30);
    }

    #[test]
    fn peek_ready_req_clock_returns_head_priority() {
        let mut q = CommandQueue::new();
        assert_eq!(q.peek_ready_req_clock(), None);

        q.push(entry(0, 42));
        assert_eq!(q.peek_ready_req_clock(), Some(42));

        q.push(entry(0, 10));
        assert_eq!(q.peek_ready_req_clock(), Some(10));
    }

    #[test]
    fn background_entries_sort_after_normal() {
        let mut q = CommandQueue::new();
        q.push(entry(0, BACKGROUND_PRIORITY_CLOCK));
        q.push(entry(0, 100));
        q.push(entry(0, 200));

        // Normal entries come out first, background last.
        assert_eq!(q.pop_ready().unwrap().req_clock(), 100);
        assert_eq!(q.pop_ready().unwrap().req_clock(), 200);
        assert_eq!(q.pop_ready().unwrap().req_clock(), BACKGROUND_PRIORITY_CLOCK);
    }

    #[test]
    fn has_non_background_ready_distinguishes() {
        let mut q = CommandQueue::new();
        assert!(!q.has_non_background_ready());

        q.push(entry(0, BACKGROUND_PRIORITY_CLOCK));
        assert!(!q.has_non_background_ready());
        assert!(q.has_only_background_ready());

        q.push(entry(0, 50));
        assert!(q.has_non_background_ready());
        assert!(!q.has_only_background_ready());
    }
}
