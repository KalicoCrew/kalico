//! UnackedWindow + AwaitingResponse. Spec §3.3.

use std::collections::VecDeque;
use std::sync::mpsc::SyncSender;
use std::time::Instant;

use crate::transport::{MessageParams, TransportError};

pub const MAX_PENDING_BLOCKS: usize = 12;

#[derive(Debug)]
pub struct UnackedEntry {
    pub seq: u64,
    pub frame_bytes: Vec<u8>,
    pub sent_at: Instant,
    pub retry_count: u32,
}

#[derive(Debug, Default)]
pub struct UnackedWindow {
    entries: VecDeque<UnackedEntry>,
}

impl UnackedWindow {
    pub fn len(&self) -> usize {
        self.entries.len()
    }
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
    pub fn is_full(&self) -> bool {
        self.entries.len() >= MAX_PENDING_BLOCKS
    }
    pub fn front(&self) -> Option<&UnackedEntry> {
        self.entries.front()
    }

    pub fn push(&mut self, entry: UnackedEntry) {
        debug_assert!(!self.is_full(), "UnackedWindow overflow");
        self.entries.push_back(entry);
    }

    /// Pop entries with seq < rseq (strict; per Codex finding #2).
    pub fn pop_acked(&mut self, rseq: u64) -> Vec<UnackedEntry> {
        let mut popped = Vec::new();
        while let Some(front) = self.entries.front() {
            if front.seq < rseq {
                popped.push(self.entries.pop_front().unwrap());
            } else {
                break;
            }
        }
        popped
    }

    pub fn iter(&self) -> impl Iterator<Item = &UnackedEntry> {
        self.entries.iter()
    }

    pub fn iter_mut(&mut self) -> impl Iterator<Item = &mut UnackedEntry> {
        self.entries.iter_mut()
    }

    /// Drop every in-flight entry. Called from `transition_closed` per spec §3.11.
    pub fn clear(&mut self) {
        self.entries.clear();
    }
}

#[derive(Debug)]
pub struct AwaitEntry {
    pub call_id: u64,
    pub seq: u64,
    pub expected_response_name: String,
    pub completion: SyncSender<Result<MessageParams, TransportError>>,
    pub submitted_at: Instant,
    pub deadline: Instant,
    pub abandoned: bool,
}

#[derive(Debug, Default)]
pub struct AwaitingResponse {
    entries: VecDeque<AwaitEntry>,
}

const AWAITING_DEFENSIVE_CEILING: usize = 1024;

impl AwaitingResponse {
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    pub fn push(&mut self, entry: AwaitEntry) -> Result<(), TransportError> {
        if self.entries.len() >= AWAITING_DEFENSIVE_CEILING {
            return Err(TransportError::Parse(
                "AwaitingResponse defensive ceiling exceeded".into(),
            ));
        }
        self.entries.push_back(entry);
        Ok(())
    }

    /// FIFO match against expected_response_name, skipping abandoned entries.
    pub fn find_match(&self, name: &str) -> Option<usize> {
        self.entries
            .iter()
            .position(|e| !e.abandoned && e.expected_response_name == name)
    }

    pub fn remove(&mut self, idx: usize) -> AwaitEntry {
        self.entries.remove(idx).expect("idx valid")
    }

    pub fn mark_abandoned(&mut self, call_id: u64) {
        if let Some(e) = self.entries.iter_mut().find(|e| e.call_id == call_id) {
            e.abandoned = true;
        }
    }

    /// GC: evict entries past their deadline. Returns evicted entries.
    pub fn evict_expired(&mut self, now: Instant) -> Vec<AwaitEntry> {
        let mut evicted = Vec::new();
        let mut idx = 0;
        while idx < self.entries.len() {
            if now >= self.entries[idx].deadline {
                evicted.push(self.entries.remove(idx).unwrap());
            } else {
                idx += 1;
            }
        }
        evicted
    }

    /// Drain all entries (for disconnect GC).
    pub fn drain_all(&mut self) -> Vec<AwaitEntry> {
        std::mem::take(&mut self.entries).into_iter().collect()
    }

    pub fn iter(&self) -> impl Iterator<Item = &AwaitEntry> {
        self.entries.iter()
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod awaiting_tests;
