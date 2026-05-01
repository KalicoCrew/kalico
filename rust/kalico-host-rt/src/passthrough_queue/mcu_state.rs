//! Cross-queue priority emission: owns all `CommandQueue`s for one MCU
//! and picks the ready-head with the lowest `req_clock`.

use indexmap::IndexMap;

use super::command_queue::CommandQueue;
use super::entry::{PassthroughEntry, BACKGROUND_PRIORITY_CLOCK};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CommandQueueId(u32);

impl CommandQueueId {
    pub fn raw(&self) -> u32 {
        self.0
    }

    /// Reconstruct a `CommandQueueId` from a raw `u32` previously obtained
    /// via [`raw()`](Self::raw). The caller is responsible for ensuring the
    /// value refers to an allocated queue.
    pub fn from_raw(raw: u32) -> Self {
        Self(raw)
    }
}

#[derive(Debug)]
pub enum PushError {
    UnknownQueue(CommandQueueId),
}

impl std::fmt::Display for PushError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::UnknownQueue(id) => write!(f, "unknown command queue id {}", id.0),
        }
    }
}

impl std::error::Error for PushError {}

#[derive(Debug, Default)]
pub struct McuState {
    queues: IndexMap<CommandQueueId, CommandQueue>,
    next_id: u32,
}

impl McuState {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn alloc_command_queue(&mut self) -> CommandQueueId {
        let id = CommandQueueId(self.next_id);
        self.next_id += 1;
        self.queues.insert(id, CommandQueue::new());
        id
    }

    pub fn push(&mut self, queue_id: CommandQueueId, entry: PassthroughEntry) -> Result<(), PushError> {
        self.queues
            .get_mut(&queue_id)
            .ok_or(PushError::UnknownQueue(queue_id))?
            .push(entry);
        Ok(())
    }

    /// Run `promote(ack_clock)` across all queues.
    pub fn promote_all(&mut self, ack_clock: u64) {
        for q in self.queues.values_mut() {
            q.promote(ack_clock);
        }
    }

    /// Pick the queue whose ready-head has the lowest `req_clock` and pop it.
    ///
    /// Background-priority entries (`req_clock == BACKGROUND_PRIORITY_CLOCK`)
    /// are only popped when *no* non-background entries exist across any queue.
    pub fn pop_next(&mut self) -> Option<PassthroughEntry> {
        // Check whether any queue has a non-background ready entry.
        let has_non_bg = self.queues.values().any(CommandQueue::has_non_background_ready);

        let best_key = self
            .queues
            .iter()
            .filter_map(|(id, q)| {
                let rc = q.peek_ready_req_clock()?;
                // Skip background entries while non-background exist.
                if has_non_bg && rc == BACKGROUND_PRIORITY_CLOCK {
                    return None;
                }
                Some((*id, rc))
            })
            .min_by_key(|&(_, rc)| rc)
            .map(|(id, _)| id);

        best_key.and_then(|id| self.queues.get_mut(&id).and_then(CommandQueue::pop_ready))
    }

    /// Returns `true` if all ready queues are empty (no entries to emit).
    pub fn is_all_ready_empty(&self) -> bool {
        self.queues.values().all(CommandQueue::is_ready_empty)
    }

    /// Total bytes across all ready queues.
    pub fn total_ready_bytes(&self) -> u64 {
        self.queues.values().map(CommandQueue::ready_bytes).sum()
    }

    /// Total bytes across all upcoming queues.
    pub fn total_upcoming_bytes(&self) -> u64 {
        self.queues.values().map(CommandQueue::upcoming_bytes).sum()
    }

    /// Peek at the lowest `req_clock` across all queues without popping.
    ///
    /// If non-background entries exist, background entries are excluded from
    /// the minimum. If only background entries remain, returns the sentinel.
    pub fn peek_next_req_clock(&self) -> Option<u64> {
        let has_non_bg = self.queues.values().any(CommandQueue::has_non_background_ready);
        self.queues
            .values()
            .filter_map(CommandQueue::peek_ready_req_clock)
            .filter(|&rc| !has_non_bg || rc != BACKGROUND_PRIORITY_CLOCK)
            .min()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::passthrough_queue::entry::NotifyId;

    fn entry(min_clock: u64, req_clock: u64) -> PassthroughEntry {
        PassthroughEntry::new(vec![0x01], min_clock, req_clock, NotifyId::none())
    }

    #[test]
    fn allocates_distinct_command_queue_ids() {
        let mut state = McuState::new();
        let a = state.alloc_command_queue();
        let b = state.alloc_command_queue();
        assert_ne!(a, b);
    }

    #[test]
    fn pop_picks_lowest_req_clock_across_queues() {
        let mut state = McuState::new();
        let qa = state.alloc_command_queue();
        let qb = state.alloc_command_queue();

        state.push(qa, entry(0, 200)).unwrap();
        state.push(qb, entry(0, 100)).unwrap();
        state.push(qa, entry(0, 150)).unwrap();

        assert_eq!(state.pop_next().unwrap().req_clock(), 100);
        assert_eq!(state.pop_next().unwrap().req_clock(), 150);
        assert_eq!(state.pop_next().unwrap().req_clock(), 200);
        assert!(state.pop_next().is_none());
    }

    #[test]
    fn promote_runs_across_all_queues() {
        let mut state = McuState::new();
        let qa = state.alloc_command_queue();
        let qb = state.alloc_command_queue();

        state.push(qa, entry(10, 50)).unwrap();
        state.push(qb, entry(20, 40)).unwrap();

        // Nothing ready yet
        assert!(state.pop_next().is_none());

        state.promote_all(10);
        // Only qa's entry is ready
        assert_eq!(state.pop_next().unwrap().req_clock(), 50);
        assert!(state.pop_next().is_none());

        state.promote_all(20);
        assert_eq!(state.pop_next().unwrap().req_clock(), 40);
    }

    #[test]
    fn push_to_unknown_queue_returns_error() {
        let mut state = McuState::new();
        let bogus = CommandQueueId(999);
        assert!(state.push(bogus, entry(0, 0)).is_err());
    }

    #[test]
    fn background_entries_only_emitted_when_no_non_background_exist() {
        let mut state = McuState::new();
        let qa = state.alloc_command_queue();
        let qb = state.alloc_command_queue();

        // qa has a normal entry, qb has a background entry
        state.push(qa, entry(0, 200)).unwrap();
        state.push(qb, entry(0, BACKGROUND_PRIORITY_CLOCK)).unwrap();

        // Normal entry from qa should come first despite qb also having a
        // ready entry.
        assert_eq!(state.pop_next().unwrap().req_clock(), 200);

        // Now only the background entry remains — it should be emitted.
        let bg = state.pop_next().unwrap();
        assert!(bg.is_background_priority());
        assert!(state.pop_next().is_none());
    }

    #[test]
    fn mixed_queues_normal_preferred_over_background() {
        let mut state = McuState::new();
        let qa = state.alloc_command_queue();
        let qb = state.alloc_command_queue();

        // qa: background, qb: normal
        state.push(qa, entry(0, BACKGROUND_PRIORITY_CLOCK)).unwrap();
        state.push(qb, entry(0, 100)).unwrap();
        state.push(qb, entry(0, 300)).unwrap();

        // Non-background entries first, then background.
        assert_eq!(state.pop_next().unwrap().req_clock(), 100);
        assert_eq!(state.pop_next().unwrap().req_clock(), 300);
        assert_eq!(state.pop_next().unwrap().req_clock(), BACKGROUND_PRIORITY_CLOCK);
    }

    #[test]
    fn peek_next_req_clock_ignores_background_while_normal_exist() {
        let mut state = McuState::new();
        let qa = state.alloc_command_queue();
        let qb = state.alloc_command_queue();

        state.push(qa, entry(0, BACKGROUND_PRIORITY_CLOCK)).unwrap();
        state.push(qb, entry(0, 500)).unwrap();

        // peek should return the non-background value
        assert_eq!(state.peek_next_req_clock(), Some(500));
    }
}
