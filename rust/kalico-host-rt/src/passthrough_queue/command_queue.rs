use super::entry::{BACKGROUND_PRIORITY_CLOCK, PassthroughEntry};

#[derive(Debug, Default)]
pub struct CommandQueue {
    upcoming: Vec<PassthroughEntry>,
    ready: Vec<PassthroughEntry>,
    ack_clock: u64,
}

impl CommandQueue {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, entry: PassthroughEntry) {
        if entry.min_clock() <= self.ack_clock {
            sorted_insert_by_req_clock(&mut self.ready, entry);
        } else {
            sorted_insert_by_min_clock(&mut self.upcoming, entry);
        }
    }

    pub fn promote(&mut self, ack_clock: u64) {
        self.ack_clock = ack_clock;
        let split = self
            .upcoming
            .partition_point(|e| e.min_clock() <= ack_clock);
        let promoted: Vec<_> = self.upcoming.drain(..split).collect();
        for entry in promoted {
            sorted_insert_by_req_clock(&mut self.ready, entry);
        }
    }

    pub fn pop_ready(&mut self) -> Option<PassthroughEntry> {
        if self.ready.is_empty() {
            None
        } else {
            Some(self.ready.remove(0))
        }
    }

    pub fn peek_ready_req_clock(&self) -> Option<u64> {
        self.ready.first().map(PassthroughEntry::req_clock)
    }

    pub fn is_ready_empty(&self) -> bool {
        self.ready.is_empty()
    }

    pub fn has_non_background_ready(&self) -> bool {
        self.ready
            .first()
            .map_or(false, |e| e.req_clock() != BACKGROUND_PRIORITY_CLOCK)
    }

    pub fn has_only_background_ready(&self) -> bool {
        !self.ready.is_empty() && !self.has_non_background_ready()
    }

    pub fn ready_bytes(&self) -> u64 {
        self.ready.iter().map(|e| e.bytes().len() as u64).sum()
    }

    pub fn upcoming_bytes(&self) -> u64 {
        self.upcoming.iter().map(|e| e.bytes().len() as u64).sum()
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
mod tests;
