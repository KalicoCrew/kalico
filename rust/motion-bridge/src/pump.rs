//! Host-side piece pump: merges per-(mcu,axis) piece queues by absolute
//! start_time and streams PushPieces frames in strict time order with
//! per-ring flow control. See
//! `docs/superpowers/specs/2026-05-28-push-pieces-wiring-design.md`.

use std::collections::VecDeque;
use runtime::piece_ring::PieceEntry;

/// Destination ring identity.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct AxisKey {
    pub mcu_id: u32,
    pub axis: u8,
}

/// One axis's outbound queue plus flow-control accounting. `pushed` and
/// `consumed` are wrapping u32 mirrors of the MCU's monotonic ring counter
/// (spec §3.3) — never reset on a time re-anchor, only on an MCU ring reset.
pub struct AxisQueue {
    pub pieces: VecDeque<PieceEntry>,
    pub pushed: u32,
    pub consumed: u32,
    pub ring_depth: u32,
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self { pieces: VecDeque::new(), pushed: 0, consumed: 0, ring_depth }
    }
    /// Free ring slots = depth − in-flight, where in-flight = pushed − consumed
    /// (wrapping). Saturates at 0.
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.consumed);
        self.ring_depth.saturating_sub(in_flight)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[allow(dead_code)]
    fn piece(start: u64) -> PieceEntry {
        PieceEntry { start_time: start, coeffs: [0.0; 4], duration: 0.001, _reserved: 0 }
    }

    #[test]
    fn room_full_then_drains() {
        let mut q = AxisQueue::new(4);
        assert_eq!(q.room(), 4);
        q.pushed = 4;
        assert_eq!(q.room(), 0);          // full
        q.consumed = 1;
        assert_eq!(q.room(), 1);          // one freed
    }

    #[test]
    fn room_correct_across_u32_wrap() {
        let mut q = AxisQueue::new(8);
        q.pushed = 2;                      // wrapped past u32::MAX
        q.consumed = u32::MAX;             // consumed is "behind" pushed by 3
        // in_flight = 2 - (u32::MAX) wrapping = 3
        assert_eq!(q.room(), 5);
    }
}
