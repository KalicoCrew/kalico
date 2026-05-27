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
        self.sent.push_back(DebugEntry {
            seq,
            bytes,
            timestamp,
        });
    }

    /// Record a received message.
    pub fn record_received(&mut self, seq: u64, bytes: Vec<u8>, timestamp: f64) {
        if self.received.len() >= DEBUG_QUEUE_RECEIVE {
            self.received.pop_front();
        }
        self.received.push_back(DebugEntry {
            seq,
            bytes,
            timestamp,
        });
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
mod tests;
