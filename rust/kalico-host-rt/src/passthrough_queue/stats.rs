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
            ready_bytes: 0,
            upcoming_bytes: 0,
            stalled_bytes: 0,
        }
    }
}

#[cfg(test)]
mod tests;
