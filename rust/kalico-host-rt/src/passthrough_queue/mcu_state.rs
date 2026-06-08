use indexmap::IndexMap;

use super::command_queue::CommandQueue;
use super::entry::{BACKGROUND_PRIORITY_CLOCK, PassthroughEntry};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CommandQueueId(u32);

impl CommandQueueId {
    pub fn raw(&self) -> u32 {
        self.0
    }

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

    pub fn promote_all(&mut self, ack_clock: u64) {
        for q in self.queues.values_mut() {
            q.promote(ack_clock);
        }
    }

    pub fn pop_next(&mut self) -> Option<PassthroughEntry> {
        let has_non_bg = self
            .queues
            .values()
            .any(CommandQueue::has_non_background_ready);

        let best_key = self
            .queues
            .iter()
            .filter_map(|(id, q)| {
                let rc = q.peek_ready_req_clock()?;
                if has_non_bg && rc == BACKGROUND_PRIORITY_CLOCK {
                    return None;
                }
                Some((*id, rc))
            })
            .min_by_key(|&(_, rc)| rc)
            .map(|(id, _)| id);

        best_key.and_then(|id| self.queues.get_mut(&id).and_then(CommandQueue::pop_ready))
    }

    pub fn is_all_ready_empty(&self) -> bool {
        self.queues.values().all(CommandQueue::is_ready_empty)
    }

    pub fn total_ready_bytes(&self) -> u64 {
        self.queues.values().map(CommandQueue::ready_bytes).sum()
    }

    pub fn total_upcoming_bytes(&self) -> u64 {
        self.queues.values().map(CommandQueue::upcoming_bytes).sum()
    }

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
