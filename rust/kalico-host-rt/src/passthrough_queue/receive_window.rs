const MESSAGE_MAX: u64 = 64;
const DEFAULT_MAX_PENDING_BLOCKS: u64 = 12;
const DEFAULT_RECEIVE_WINDOW: u64 = 192;

#[derive(Debug)]
pub struct ReceiveWindow {
    send_seq: u64,
    receive_seq: u64,
    max_pending_blocks: u64,
    receive_window: u64,
    need_ack_bytes: u64,
    last_ack_bytes: u64,
    last_ack_seq: u64,
}

impl Default for ReceiveWindow {
    fn default() -> Self {
        Self {
            send_seq: 1,
            receive_seq: 1,
            max_pending_blocks: DEFAULT_MAX_PENDING_BLOCKS,
            receive_window: DEFAULT_RECEIVE_WINDOW,
            need_ack_bytes: 0,
            last_ack_bytes: 0,
            last_ack_seq: 0,
        }
    }
}

impl ReceiveWindow {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_limits(max_pending_blocks: u64, receive_window: u64) -> Self {
        Self {
            max_pending_blocks,
            receive_window,
            ..Self::default()
        }
    }

    pub fn can_emit(&self) -> bool {
        if self.send_seq - self.receive_seq >= self.max_pending_blocks {
            return false;
        }
        let carry = if self.last_ack_seq < self.receive_seq {
            self.last_ack_bytes
        } else {
            0
        };
        self.need_ack_bytes + MESSAGE_MAX + carry <= self.receive_window
    }

    pub fn record_emit(&mut self, bytes_len: u64) {
        self.last_ack_bytes = self.need_ack_bytes;
        self.last_ack_seq = self.send_seq - 1;
        self.need_ack_bytes += bytes_len;
        self.send_seq += 1;
    }

    pub fn record_ack(&mut self, acked_bytes: u64) {
        self.need_ack_bytes = self.need_ack_bytes.saturating_sub(acked_bytes);
        self.receive_seq += 1;
    }

    pub fn send_seq(&self) -> u64 {
        self.send_seq
    }

    pub fn receive_seq(&self) -> u64 {
        self.receive_seq
    }
}

#[cfg(test)]
mod tests;
