//! Rolling debug log for crash diagnostics (`extract_old` parity).
//!
//! Keeps the last N sent and received messages so that `dump_debug`
//! (klippy/serialhdl.py) can inspect what was happening around a crash.

use std::collections::VecDeque;

/// How many sent entries to retain.
const DEBUG_QUEUE_SENT: usize = 100;
/// How many received entries to retain.
const DEBUG_QUEUE_RECEIVE: usize = 100;

/// A single debug-log entry.
#[derive(Debug, Clone)]
pub struct DebugEntry {
    pub seq: u64,
    pub bytes: Vec<u8>,
    pub timestamp: f64,
}

/// Ring buffers of recent sent / received messages.
#[derive(Debug)]
pub struct DebugLog {
    sent: VecDeque<DebugEntry>,
    received: VecDeque<DebugEntry>,
}

impl Default for DebugLog {
    fn default() -> Self {
        Self {
            sent: VecDeque::with_capacity(DEBUG_QUEUE_SENT),
            received: VecDeque::with_capacity(DEBUG_QUEUE_RECEIVE),
        }
    }
}

impl DebugLog {
    pub fn new() -> Self {
        Self::default()
    }

    /// Record a sent message.
    pub fn record_sent(&mut self, seq: u64, bytes: Vec<u8>, timestamp: f64) {
        if self.sent.len() >= DEBUG_QUEUE_SENT {
            self.sent.pop_front();
        }
        self.sent.push_back(DebugEntry { seq, bytes, timestamp });
    }

    /// Record a received message.
    pub fn record_received(&mut self, seq: u64, bytes: Vec<u8>, timestamp: f64) {
        if self.received.len() >= DEBUG_QUEUE_RECEIVE {
            self.received.pop_front();
        }
        self.received.push_back(DebugEntry { seq, bytes, timestamp });
    }

    /// Drain the debug log, returning `(old_sent, old_received)`.
    /// After this call both buffers are empty.
    pub fn extract_old(&mut self) -> (Vec<DebugEntry>, Vec<DebugEntry>) {
        let sent: Vec<_> = self.sent.drain(..).collect();
        let received: Vec<_> = self.received.drain(..).collect();
        (sent, received)
    }

    pub fn sent_count(&self) -> usize {
        self.sent.len()
    }

    pub fn received_count(&self) -> usize {
        self.received.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn entries_are_stored_and_retrievable() {
        let mut log = DebugLog::new();
        log.record_sent(1, vec![0xAA], 1.0);
        log.record_sent(2, vec![0xBB], 2.0);
        log.record_received(10, vec![0xCC], 3.0);

        let (sent, received) = log.extract_old();
        assert_eq!(sent.len(), 2);
        assert_eq!(sent[0].seq, 1);
        assert_eq!(sent[0].bytes, vec![0xAA]);
        assert_eq!(sent[1].seq, 2);
        assert_eq!(received.len(), 1);
        assert_eq!(received[0].seq, 10);
    }

    #[test]
    fn sent_queue_capped_at_100() {
        let mut log = DebugLog::new();
        for i in 0..150 {
            log.record_sent(i, vec![0x01], i as f64);
        }
        assert_eq!(log.sent_count(), DEBUG_QUEUE_SENT);

        let (sent, _) = log.extract_old();
        assert_eq!(sent.len(), DEBUG_QUEUE_SENT);
        // Oldest entry should be seq=50 (first 50 were evicted).
        assert_eq!(sent[0].seq, 50);
        assert_eq!(sent[99].seq, 149);
    }

    #[test]
    fn received_queue_capped_at_100() {
        let mut log = DebugLog::new();
        for i in 0..150 {
            log.record_received(i, vec![0x02], i as f64);
        }
        assert_eq!(log.received_count(), DEBUG_QUEUE_RECEIVE);

        let (_, received) = log.extract_old();
        assert_eq!(received.len(), DEBUG_QUEUE_RECEIVE);
        assert_eq!(received[0].seq, 50);
    }

    #[test]
    fn extract_old_drains_buffers() {
        let mut log = DebugLog::new();
        log.record_sent(1, vec![0x01], 1.0);
        log.record_received(2, vec![0x02], 2.0);

        let _ = log.extract_old();
        assert_eq!(log.sent_count(), 0);
        assert_eq!(log.received_count(), 0);

        // Second extract returns empty.
        let (sent, received) = log.extract_old();
        assert!(sent.is_empty());
        assert!(received.is_empty());
    }
}
