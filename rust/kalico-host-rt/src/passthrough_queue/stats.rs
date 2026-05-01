//! Per-MCU statistics counters — `serialqueue_get_stats` parity.

/// Snapshot of passthrough-queue statistics for one MCU.
#[derive(Debug, Default, Clone, PartialEq, Eq)]
pub struct PassthroughStats {
    pub bytes_write: u64,
    pub bytes_read: u64,
    pub bytes_retransmit: u64,
    pub bytes_invalid: u64,
    pub send_seq: u64,
    pub receive_seq: u64,
    pub retransmit_seq: u64,
    pub ready_bytes: u64,
    pub upcoming_bytes: u64,
    pub stalled_bytes: u64,
}

/// Mutable counters stored inside `McuRecord`.
#[derive(Debug, Default)]
pub struct StatsCounters {
    pub bytes_write: u64,
    pub bytes_read: u64,
    pub bytes_retransmit: u64,
    pub bytes_invalid: u64,
    pub send_seq: u64,
    pub receive_seq: u64,
    pub retransmit_seq: u64,
}

impl StatsCounters {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn snapshot(&self) -> PassthroughStats {
        PassthroughStats {
            bytes_write: self.bytes_write,
            bytes_read: self.bytes_read,
            bytes_retransmit: self.bytes_retransmit,
            bytes_invalid: self.bytes_invalid,
            send_seq: self.send_seq,
            receive_seq: self.receive_seq,
            retransmit_seq: self.retransmit_seq,
            // ready/upcoming/stalled bytes are filled in by the router
            // from live queue state, not from counters.
            ready_bytes: 0,
            upcoming_bytes: 0,
            stalled_bytes: 0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_stats_are_zero() {
        let s = PassthroughStats::default();
        assert_eq!(s.bytes_write, 0);
        assert_eq!(s.bytes_read, 0);
        assert_eq!(s.send_seq, 0);
    }

    #[test]
    fn counters_snapshot_round_trips() {
        let mut c = StatsCounters::new();
        c.bytes_write = 100;
        c.bytes_read = 50;
        c.send_seq = 42;

        let snap = c.snapshot();
        assert_eq!(snap.bytes_write, 100);
        assert_eq!(snap.bytes_read, 50);
        assert_eq!(snap.send_seq, 42);
    }
}
