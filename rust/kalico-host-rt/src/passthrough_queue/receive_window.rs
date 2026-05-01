//! Backpressure tracking with two gates: pending-block count and
//! byte-budget window — both must pass before a message can be emitted.

/// Maximum message size on the wire (from Klipper protocol).
const MESSAGE_MAX: u64 = 64;
/// Default maximum number of in-flight blocks.
const DEFAULT_MAX_PENDING_BLOCKS: u64 = 12;
/// Default receive window size in bytes.
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

    /// Both gates must pass:
    /// 1. `(send_seq - receive_seq) < max_pending_blocks`
    /// 2. `need_ack_bytes + MESSAGE_MAX [+ last_ack_bytes if lagging] <= receive_window`
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

    /// Record that a block of `bytes_len` was emitted.
    pub fn record_emit(&mut self, bytes_len: u64) {
        self.last_ack_bytes = self.need_ack_bytes;
        self.last_ack_seq = self.send_seq - 1;
        self.need_ack_bytes += bytes_len;
        self.send_seq += 1;
    }

    /// Record that an ack was received, freeing `acked_bytes` of window
    /// capacity.
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
mod tests {
    use super::*;

    #[test]
    fn window_default_starts_empty() {
        let w = ReceiveWindow::new();
        assert!(w.can_emit());
        assert_eq!(w.send_seq(), 1);
        assert_eq!(w.receive_seq(), 1);
    }

    #[test]
    fn emit_check_includes_message_max_overhead() {
        // Window of exactly MESSAGE_MAX: one emit should be possible
        let mut w = ReceiveWindow::with_limits(100, MESSAGE_MAX);
        assert!(w.can_emit());

        // After emitting 1 byte, need_ack_bytes=1, check is 1+64 > 64 → fail
        w.record_emit(1);
        assert!(!w.can_emit());
    }

    #[test]
    fn pending_blocks_gate() {
        let mut w = ReceiveWindow::with_limits(2, 10_000);
        assert!(w.can_emit());

        w.record_emit(10);
        assert!(w.can_emit()); // send_seq=2, receive_seq=1, diff=1 < 2

        w.record_emit(10);
        assert!(!w.can_emit()); // send_seq=3, receive_seq=1, diff=2 >= 2

        w.record_ack(10);
        assert!(w.can_emit()); // receive_seq=2, diff=1 < 2
    }

    #[test]
    fn last_ack_bytes_carry_when_acks_lag() {
        // Window is large enough for one message + overhead, but
        // last_ack_bytes carry tips it over.
        let mut w = ReceiveWindow::with_limits(100, MESSAGE_MAX + 20);
        w.record_emit(20);
        // need_ack_bytes=20, last_ack_bytes=0, last_ack_seq=0 < receive_seq=1
        // check: 20 + 64 + 0 = 84 <= 84 ✓
        assert!(w.can_emit());

        w.record_emit(10);
        // need_ack_bytes=30, last_ack_bytes=20, last_ack_seq=1 >= receive_seq=1
        // carry=0 → 30+64+0=94 > 84 → blocked by bytes
        assert!(!w.can_emit());

        // Ack brings receive_seq to 2, last_ack_seq=1 < 2 → carry kicks in
        w.record_ack(20);
        // need_ack_bytes=10, last_ack_bytes=20, last_ack_seq=1 < receive_seq=2
        // check: 10+64+20=94 > 84 → still blocked
        assert!(!w.can_emit());

        // Another ack
        w.record_ack(10);
        // need_ack_bytes=0, last_ack_bytes=20, last_ack_seq=1 < receive_seq=3
        // check: 0+64+20=84 <= 84 ✓
        assert!(w.can_emit());
    }
}
