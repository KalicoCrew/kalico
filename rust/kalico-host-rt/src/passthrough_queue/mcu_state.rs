//! Cross-queue priority emission: owns all `CommandQueue`s for one MCU
//! and picks the ready-head with the lowest `req_clock`.

use indexmap::IndexMap;

use super::command_queue::CommandQueue;
use super::entry::{BACKGROUND_PRIORITY_CLOCK, PassthroughEntry};

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

    pub fn push(
        &mut self,
        queue_id: CommandQueueId,
        entry: PassthroughEntry,
    ) -> Result<(), PushError> {
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
        let has_non_bg = self
            .queues
            .values()
            .any(CommandQueue::has_non_background_ready);

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
        let has_non_bg = self
            .queues
            .values()
            .any(CommandQueue::has_non_background_ready);
        self.queues
            .values()
            .filter_map(CommandQueue::peek_ready_req_clock)
            .filter(|&rc| !has_non_bg || rc != BACKGROUND_PRIORITY_CLOCK)
            .min()
    }
}

#[cfg(test)]
mod tests;
