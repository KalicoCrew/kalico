use std::collections::VecDeque;

const DEBUG_QUEUE_SENT: usize = 100;
const DEBUG_QUEUE_RECEIVE: usize = 100;

#[derive(Debug, Clone)]
pub struct DebugEntry {
    pub seq: u64,
    pub bytes: Vec<u8>,
    pub timestamp: f64,
}

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
