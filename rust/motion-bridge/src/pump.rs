//! Host-side piece pump: merges per-(mcu,axis) piece queues by absolute
//! start_time and streams PushPieces frames in strict time order with
//! per-ring flow control. See
//! `docs/superpowers/specs/2026-05-28-push-pieces-wiring-design.md`.

use std::collections::{BTreeMap, BTreeSet, VecDeque};
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
#[derive(Debug)]
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

/// A planned outbound frame: one axis's contiguous run of pieces.
#[derive(Debug)]
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
}

/// Outcome of one scheduling decision.
#[derive(Debug)]
pub enum Schedule {
    /// Send these frames (all on one MCU, a contiguous prefix of global order).
    Send(Vec<FramePlan>),
    /// Global head's ring is full — wait for a heartbeat. Do not send anything.
    StallFull(AxisKey),
    /// No pieces queued anywhere.
    Idle,
}

/// Decide the next action over the queue map. Does **not** mutate the queues;
/// the caller applies the returned plan (pops pieces, bumps `pushed`).
/// `max_per_frame` caps a single PushPieces frame's piece_count (u8 wire field).
#[must_use]
pub fn schedule(queues: &BTreeMap<AxisKey, AxisQueue>, max_per_frame: usize) -> Schedule {
    // Earliest non-empty queue head, tie-broken by (mcu_id, axis).
    let head = queues
        .iter()
        .filter(|(_, q)| !q.pieces.is_empty())
        .min_by(|(ka, qa), (kb, qb)| {
            qa.pieces.front().unwrap().start_time
                .cmp(&qb.pieces.front().unwrap().start_time)
                .then(ka.cmp(kb))
        });
    let (&head_key, head_q) = match head {
        None => return Schedule::Idle,
        Some(h) => h,
    };
    if head_q.room() == 0 {
        return Schedule::StallFull(head_key);
    }

    // Greedily take the contiguous prefix of global time order that stays on
    // head_key.mcu_id and has room. Simulate room locally so a single
    // scheduling pass never plans more than each ring can hold.
    //
    // `maxed` tracks same-MCU axes that have hit their room or frame cap this
    // pass. They are excluded from candidate selection to avoid re-selecting
    // them every iteration (which would infinite-loop), but their saturation
    // does NOT end the batch — other same-MCU axes with room still get pieces.
    let mut taken: BTreeMap<AxisKey, usize> = BTreeMap::new(); // key -> count planned
    let mut maxed: BTreeSet<AxisKey> = BTreeSet::new();
    loop {
        let next = queues
            .iter()
            .filter_map(|(k, q)| {
                if maxed.contains(k) {
                    return None;
                }
                let already = taken.get(k).copied().unwrap_or(0);
                q.pieces.get(already).map(|p| (*k, p.start_time))
            })
            .min_by(|(ka, sa), (kb, sb)| sa.cmp(sb).then(ka.cmp(kb)));
        let (k, _start) = match next {
            Some(n) => n,
            None => break,
        };
        if k.mcu_id != head_key.mcu_id {
            break; // next-earliest is a different MCU — stop the batch
        }
        let already = taken.get(&k).copied().unwrap_or(0);
        let q = &queues[&k];
        let room = q.room() as usize;
        if already >= room || already >= max_per_frame {
            // This axis is saturated for this pass — exclude it from further
            // candidate selection and continue batching other same-MCU axes.
            maxed.insert(k);
            continue;
        }
        *taken.entry(k).or_insert(0) += 1;
    }

    // `taken` entries are always ≥1 by construction (the only path into `taken`
    // is the `+= 1` above, after the saturation check passes), so the filter is
    // a defensive no-op. Kept to make the invariant explicit.
    let frames: Vec<FramePlan> = taken
        .into_iter()
        .filter(|(_, n)| *n > 0)
        .map(|(k, n)| FramePlan {
            key: k,
            pieces: queues[&k].pieces.iter().take(n).copied().collect(),
        })
        .collect();
    // The head-room > 0 check above guarantees at least one piece is planned.
    debug_assert!(!frames.is_empty());
    Schedule::Send(frames)
}

#[cfg(test)]
mod sched_tests {
    use super::*;

    fn q_with(ring_depth: u32, starts: &[u64]) -> AxisQueue {
        let mut q = AxisQueue::new(ring_depth);
        for &s in starts {
            q.pieces.push_back(PieceEntry {
                start_time: s,
                coeffs: [0.0; 4],
                duration: 0.001,
                _reserved: 0,
            });
        }
        q
    }

    #[test]
    fn idle_when_empty() {
        let queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
        assert!(matches!(schedule(&queues, 255), Schedule::Idle));
    }

    #[test]
    fn stalls_when_global_head_ring_full() {
        let mut queues = BTreeMap::new();
        // mcuA/x earliest but full; mcuB/x later but has room → must STALL, not skip.
        let mut a = q_with(2, &[10]);
        a.pushed = 2; // full
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
        queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[20]));
        assert!(matches!(
            schedule(&queues, 255),
            Schedule::StallFull(AxisKey { mcu_id: 1, axis: 0 })
        ));
    }

    #[test]
    fn batches_contiguous_same_mcu_prefix_only() {
        let mut queues = BTreeMap::new();
        // global order: A/x@0, A/y@1, B/x@2, A/x@3
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 3]));
        queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1]));
        queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[2]));
        let s = schedule(&queues, 255);
        // batch stops at B/x@2 → A/x gets [0] only (A/x@3 is after the B boundary),
        // A/y gets [1]. B/x not included.
        match s {
            Schedule::Send(frames) => {
                let ax: Vec<_> = frames.iter().map(|f| (f.key, f.pieces.len())).collect();
                assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 0 }, 1)));
                assert!(ax.contains(&(AxisKey { mcu_id: 1, axis: 1 }, 1)));
                assert!(!ax.iter().any(|(k, _)| k.mcu_id == 2));
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn frame_cap_splits() {
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 1, 2, 3]));
        let s = schedule(&queues, 2);
        match s {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1);
                assert_eq!(frames[0].pieces.len(), 2); // capped at 2 this pass
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn full_axis_does_not_block_same_mcu_sibling() {
        // Scenario: MCU 1, two axes. Y is the global head (Y@0 < X@1).
        // X has depth 1 and pushed==1 (room==0). Y has depth 8 and room.
        // The full X should be excluded from the batch, not cause an early
        // loop exit — Y's pieces must still be planned this pass.
        let mut q: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
        let yq = q_with(8, &[0, 2]); // Y: global head, has room
        let mut xq = q_with(1, &[1]); // X: depth 1, one piece at t=1
        xq.pushed = 1; // room == 0 (full)
        q.insert(AxisKey { mcu_id: 1, axis: 1 }, yq);
        q.insert(AxisKey { mcu_id: 1, axis: 0 }, xq);
        // Y@0 is the global head and has room → top-level StallFull does NOT fire.
        match schedule(&q, 255) {
            Schedule::Send(frames) => {
                // Y must be batched despite full sibling X.
                let yf = frames.iter().find(|f| f.key == AxisKey { mcu_id: 1, axis: 1 });
                assert!(yf.is_some(), "Y should be batched despite full sibling X");
                // X is full → contributes nothing to this batch.
                assert!(
                    !frames.iter().any(|f| f.key == AxisKey { mcu_id: 1, axis: 0 }),
                    "full X must not appear in the batch"
                );
            }
            other => panic!("expected Send, got {other:?}"),
        }
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
