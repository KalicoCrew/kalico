//! UnackedWindow + AwaitingResponse. Spec §3.3.

use std::collections::VecDeque;
use std::sync::mpsc::SyncSender;
use std::time::Instant;

use crate::transport::{MessageParams, TransportError};

pub const MAX_PENDING_BLOCKS: usize = 12;

#[derive(Debug)]
pub struct UnackedEntry {
    pub seq:         u64,
    pub frame_bytes: Vec<u8>,
    pub sent_at:     Instant,
    pub retry_count: u32,
}

#[derive(Debug, Default)]
pub struct UnackedWindow {
    entries: VecDeque<UnackedEntry>,
}

impl UnackedWindow {
    pub fn len(&self) -> usize { self.entries.len() }
    pub fn is_empty(&self) -> bool { self.entries.is_empty() }
    pub fn is_full(&self) -> bool { self.entries.len() >= MAX_PENDING_BLOCKS }
    pub fn front(&self) -> Option<&UnackedEntry> { self.entries.front() }

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
    pub call_id:                u64,
    pub seq:                    u64,
    pub expected_response_name: String,
    pub completion:             SyncSender<Result<MessageParams, TransportError>>,
    pub submitted_at:           Instant,
    pub deadline:               Instant,
    pub abandoned:              bool,
}

#[derive(Debug, Default)]
pub struct AwaitingResponse {
    entries: VecDeque<AwaitEntry>,
}

const AWAITING_DEFENSIVE_CEILING: usize = 1024;

impl AwaitingResponse {
    pub fn len(&self) -> usize { self.entries.len() }

    pub fn push(&mut self, entry: AwaitEntry) -> Result<(), TransportError> {
        if self.entries.len() >= AWAITING_DEFENSIVE_CEILING {
            return Err(TransportError::Parse("AwaitingResponse defensive ceiling exceeded".into()));
        }
        self.entries.push_back(entry);
        Ok(())
    }

    /// FIFO match against expected_response_name, skipping abandoned entries.
    pub fn find_match(&self, name: &str) -> Option<usize> {
        self.entries.iter().position(|e| !e.abandoned && e.expected_response_name == name)
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

    pub fn iter(&self) -> impl Iterator<Item = &AwaitEntry> { self.entries.iter() }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(seq: u64) -> UnackedEntry {
        UnackedEntry {
            seq,
            frame_bytes: vec![],
            sent_at: Instant::now(),
            retry_count: 0,
        }
    }

    #[test]
    fn pop_acked_strict_less_than() {
        let mut w = UnackedWindow::default();
        w.push(entry(1));
        w.push(entry(2));
        w.push(entry(3));
        // ack rseq=2 → pops only seq=1 (strict <).
        let popped = w.pop_acked(2);
        assert_eq!(popped.len(), 1);
        assert_eq!(popped[0].seq, 1);
        assert_eq!(w.len(), 2);
        assert_eq!(w.front().unwrap().seq, 2);
    }
}

#[cfg(test)]
mod awaiting_tests {
    use super::*;
    use std::time::Duration;

    fn make_entry(call_id: u64, name: &str, seq: u64, deadline: Instant)
        -> (AwaitEntry, std::sync::mpsc::Receiver<Result<MessageParams, TransportError>>)
    {
        let (tx, rx) = std::sync::mpsc::sync_channel(1);
        (
            AwaitEntry {
                call_id, seq,
                expected_response_name: name.to_string(),
                completion: tx,
                submitted_at: Instant::now(),
                deadline,
                abandoned: false,
            },
            rx,
        )
    }

    #[test]
    fn fifo_match_finds_oldest() {
        let mut a = AwaitingResponse::default();
        let (e1, _r1) = make_entry(1, "rsp", 1, Instant::now() + Duration::from_secs(60));
        let (e2, _r2) = make_entry(2, "rsp", 2, Instant::now() + Duration::from_secs(60));
        a.push(e1).unwrap();
        a.push(e2).unwrap();
        assert_eq!(a.find_match("rsp"), Some(0));    // oldest first
    }

    #[test]
    fn fifo_skips_abandoned() {
        let mut a = AwaitingResponse::default();
        let (e1, _r1) = make_entry(1, "rsp", 1, Instant::now() + Duration::from_secs(60));
        let (e2, _r2) = make_entry(2, "rsp", 2, Instant::now() + Duration::from_secs(60));
        a.push(e1).unwrap();
        a.push(e2).unwrap();
        a.mark_abandoned(1);
        assert_eq!(a.find_match("rsp"), Some(1));    // skips abandoned 0
    }

    #[test]
    fn evict_expired_returns_past_deadline() {
        let mut a = AwaitingResponse::default();
        let past = Instant::now() - Duration::from_millis(100);
        let future = Instant::now() + Duration::from_secs(60);
        let (e1, _r1) = make_entry(1, "rsp", 1, past);
        let (e2, _r2) = make_entry(2, "rsp", 2, future);
        a.push(e1).unwrap();
        a.push(e2).unwrap();
        let evicted = a.evict_expired(Instant::now());
        assert_eq!(evicted.len(), 1);
        assert_eq!(evicted[0].call_id, 1);
        assert_eq!(a.len(), 1);
    }
}
