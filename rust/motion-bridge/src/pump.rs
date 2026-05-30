//! Host-side piece pump: merges per-(mcu,axis) piece queues by absolute
//! start_time and streams PushPieces frames in strict time order with
//! per-ring flow control. See
//! `docs/superpowers/specs/2026-05-28-push-pieces-wiring-design.md`.

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::sync::mpsc::{Receiver, RecvError};
use std::sync::Weak;
use std::time::Duration;
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
///
/// `physical_write_cursor` tracks the MCU ring slot that will receive the
/// next piece. It is advanced incrementally modulo `ring_depth` on each
/// successful send — it is NEVER derived as `pushed % ring_depth`, which
/// would produce wrong results across u32 wrap of `pushed`.
#[derive(Debug)]
pub struct AxisQueue {
    pub pieces: VecDeque<PieceEntry>,
    pub pushed: u32,
    pub consumed: u32,
    pub ring_depth: u32,
    /// Physical ring write cursor: the slot index in `[0, ring_depth)` where
    /// the next batch of pieces will be written on the MCU side. Advanced
    /// incrementally (mod ring_depth) on each ACKed send — never reset on
    /// time re-anchor, only on an MCU ring reset.
    pub physical_write_cursor: u32,
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self {
            pieces: VecDeque::new(),
            pushed: 0,
            consumed: 0,
            ring_depth,
            physical_write_cursor: 0,
        }
    }
    /// Free ring slots = depth − in-flight, where in-flight = pushed − consumed
    /// (wrapping). Saturates at 0.
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.consumed);
        self.ring_depth.saturating_sub(in_flight)
    }
    /// Advance the physical write cursor by `n` slots, wrapping at `ring_depth`.
    /// No-op when `ring_depth == 0` (degenerate / uninitialised ring).
    pub fn advance_write_cursor(&mut self, n: u32) {
        if self.ring_depth == 0 {
            return;
        }
        // cursor < ring_depth ≤ 65535, n ≤ 255 → sum ≤ 65789 < u32::MAX; no overflow.
        self.physical_write_cursor = (self.physical_write_cursor + n) % self.ring_depth;
    }
}

/// A planned outbound frame: one axis's contiguous run of pieces.
///
/// `start_slot` is filled in by the send loop (just before sending) from
/// `AxisQueue::physical_write_cursor` — the scheduler does not have mutable
/// queue access at plan time, so this field is set to 0 at construction and
/// overwritten at send time.
#[derive(Debug)]
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    /// Physical ring slot where this frame's first piece will land on the MCU.
    /// Set to 0 at schedule time; overwritten with `q.physical_write_cursor`
    /// just before the send call in `run_pump`.
    pub start_slot: u16,
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
            // start_slot is filled in by run_pump just before sending, once
            // it looks up the queue's physical_write_cursor. Set 0 here.
            start_slot: 0,
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
    use std::sync::{Arc, Mutex};
    use std::sync::mpsc;

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

    #[test]
    fn physical_write_cursor_advances_and_wraps_at_n() {
        let mut q = AxisQueue::new(4); // ring_depth 4
        assert_eq!(q.physical_write_cursor, 0);
        q.advance_write_cursor(3);
        assert_eq!(q.physical_write_cursor, 3);
        // 3 + 3 = 6 → 6 % 4 = 2
        q.advance_write_cursor(3);
        assert_eq!(q.physical_write_cursor, 2);
    }

    // ── Mock sink for run_pump tests ──────────────────────────────────────────

    /// Records every send_frame call's (start_slot, new_head) for assertions.
    #[derive(Clone)]
    struct RecordingSink {
        calls: Arc<Mutex<Vec<(u16, u32)>>>,
    }

    impl RecordingSink {
        fn new() -> Self {
            Self { calls: Arc::new(Mutex::new(Vec::new())) }
        }
        fn recorded(&self) -> Vec<(u16, u32)> {
            self.calls.lock().unwrap().clone()
        }
    }

    impl PieceSink for RecordingSink {
        fn send_frame(
            &self,
            _key: AxisKey,
            _pieces: &[PieceEntry],
            start_slot: u16,
            new_head: u32,
        ) -> Result<i32, String> {
            self.calls.lock().unwrap().push((start_slot, new_head));
            Ok(kalico_protocol::result_codes::OK)
        }
    }

    fn make_piece(t: u64) -> PieceEntry {
        PieceEntry { start_time: t, coeffs: [0.0; 4], duration: 0.001, _reserved: 0 }
    }

    #[test]
    fn run_pump_sets_start_slot_from_cursor_and_advances_it() {
        // ring_depth = 8 (> 2*N), so there is always room for both batches
        // without needing a heartbeat to free slots. N=3 pieces per batch:
        //
        //   First send:  start_slot = 0,        new_head = N=3.
        //                cursor advances to N % 8 = 3.
        //   Second send: start_slot = 3,        new_head = 2*N=6.
        //                cursor advances to (3+3) % 8 = 6.
        //
        // The pump runs in a separate thread (matching pump_loop.rs style)
        // because the burst-drain logic would otherwise see the Shutdown
        // message and exit before sending if everything is pre-queued.
        const RING_DEPTH: u32 = 8; // > 2*N, guarantees room for both batches
        const N: u32 = 3;

        let sink = RecordingSink::new();
        let (tx, rx) = mpsc::channel::<PumpMsg>();
        let sink_clone = sink.clone();
        let handle = std::thread::spawn(move || {
            run_pump(rx, sink_clone, |_key| RING_DEPTH);
        });

        // Enqueue first batch of N pieces and spin-wait until the pump drains it.
        tx.send(PumpMsg::Enqueue(EnqueueMsg {
            key: AxisKey { mcu_id: 1, axis: 0 },
            pieces: (0..N).map(|i| make_piece(i as u64)).collect(),
            fresh_stream: false,
        }))
        .unwrap();
        {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
            while sink.recorded().len() < 1 {
                assert!(
                    std::time::Instant::now() < deadline,
                    "pump did not drain first batch within deadline"
                );
                std::thread::yield_now();
            }
        }

        // Enqueue second batch of N pieces and spin-wait until the pump drains it.
        tx.send(PumpMsg::Enqueue(EnqueueMsg {
            key: AxisKey { mcu_id: 1, axis: 0 },
            pieces: (N..N * 2).map(|i| make_piece(i as u64)).collect(),
            fresh_stream: false,
        }))
        .unwrap();
        {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
            while sink.recorded().len() < 2 {
                assert!(
                    std::time::Instant::now() < deadline,
                    "pump did not drain second batch within deadline"
                );
                std::thread::yield_now();
            }
        }

        tx.send(PumpMsg::Shutdown).unwrap();
        handle.join().unwrap();

        let recorded = sink.recorded();
        assert_eq!(recorded.len(), 2, "expected exactly 2 sends, got {}", recorded.len());

        let (s0, h0) = recorded[0];
        let (s1, h1) = recorded[1];

        // First send: cursor was 0, new_head = N.
        assert_eq!(s0, 0, "first start_slot should be 0");
        assert_eq!(h0, N, "first new_head should be N={N}");

        // Second send: cursor advanced to (0 + N) % RING_DEPTH after first send.
        let expected_s1 = (N % RING_DEPTH) as u16;
        assert_eq!(s1, expected_s1, "second start_slot should be {expected_s1}");
        assert_eq!(h1, N * 2, "second new_head should be {}", N * 2);
        // Sanity: cursor after second send would be (N + N) % RING_DEPTH.
        // We don't assert it here (no access to the queue after run_pump exits),
        // but the new_head values prove both batches were fully sent.
    }
}

// ── Inbound message types ────────────────────────────────────────────────────

/// Pieces handed to the pump for one (mcu, axis), in time order.
pub struct EnqueueMsg {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    /// Set when this batch begins a fresh stream (timeline re-anchor): the
    /// pump leaves flow-control counters alone (spec §3.3) — this flag exists
    /// only so future logic can react; for now it is informational.
    pub fresh_stream: bool,
}

/// Per-MCU heartbeat: consumed counts indexed by axis.
pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub consumed_counts: Vec<u32>,
}

/// Inbound to the pump loop.
pub enum PumpMsg {
    Enqueue(EnqueueMsg),
    Heartbeat(HeartbeatMsg),
    Shutdown,
}

// ── PieceSink trait ──────────────────────────────────────────────────────────

/// Sends one axis's frame to the wire. Returns the MCU's result code
/// (`result_codes::OK` on success).
pub trait PieceSink: Send {
    /// Send `pieces` for `key`, starting at physical ring slot `start_slot`.
    /// `new_head` is `pushed.wrapping_add(pieces.len())` — the post-send
    /// monotonic counter the MCU will see as the new head.
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<i32, String>;
}

// ── run_pump ─────────────────────────────────────────────────────────────────

/// Run the pump until `Shutdown`. `ring_depth_of` supplies each ring's depth
/// the first time its key is seen. `sink` performs the actual wire send.
pub fn run_pump<S, F>(rx: Receiver<PumpMsg>, sink: S, ring_depth_of: F)
where
    S: PieceSink,
    F: Fn(AxisKey) -> u32,
{
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    const MAX_PER_FRAME: usize = 255; // u8 wire piece_count

    let apply = |msg: PumpMsg, queues: &mut BTreeMap<AxisKey, AxisQueue>| -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Enqueue(EnqueueMsg { key, pieces, fresh_stream: _ }) => {
                let q = queues
                    .entry(key)
                    .or_insert_with(|| AxisQueue::new(ring_depth_of(key)));
                let count_added = pieces.len();
                q.pieces.extend(pieces);
                // PIECEDIAG (revert)
                log::info!(
                    "PIECEDIAG enqueue axis={} n={} qlen={}",
                    key.axis, count_added, q.pieces.len()
                );
            }
            PumpMsg::Heartbeat(HeartbeatMsg { mcu_id, consumed_counts }) => {
                // INVARIANT: consumed_counts[i] is axis index i, the SAME axis
                // numbering used by PushPieces.axis_idx and the enqueue adapter's
                // AxisKey.axis. Verified end-to-end on the MCU: the heartbeat FFI
                // writes consumed_counts()[i] = stepping_axes[i].ring.consumed_count()
                // in index order (runtime_ffi.rs), and push_pieces(axis_idx) targets
                // that same stepping_axes[axis_idx]. Do not reorder either side.
                // PIECEDIAG (revert)
                log::info!(
                    "PIECEDIAG HB mcu={} consumed={:?}",
                    mcu_id, consumed_counts
                );
                for (axis, &c) in consumed_counts.iter().enumerate() {
                    let key = AxisKey { mcu_id, axis: axis as u8 };
                    if let Some(q) = queues.get_mut(&key) {
                        q.consumed = c;
                    }
                }
            }
        }
        true
    };

    // Block for the first message, then process bursts.
    loop {
        let first = match rx.recv() {
            Ok(m) => m,
            Err(RecvError) => return,
        };
        if !apply(first, &mut queues) {
            return;
        }
        // Drain anything else already queued (coalesce bursts before sending).
        while let Ok(m) = rx.try_recv() {
            if !apply(m, &mut queues) {
                return;
            }
        }
        // Send as far as the schedule allows. A send failure breaks all the
        // way back to `recv()` (labeled break) instead of re-running schedule
        // on the still-queued pieces — otherwise a persistent failure spins a
        // tight busy-loop. The next inbound message (heartbeat/enqueue) retries.
        'send: loop {
            match schedule(&queues, MAX_PER_FRAME) {
                Schedule::Idle => break 'send,
                Schedule::StallFull(stall_key) => {
                    // PIECEDIAG (revert)
                    let q = queues.get(&stall_key);
                    log::info!(
                        "PIECEDIAG STALL axis={} pushed={} consumed={} ring_depth={} room={}",
                        stall_key.axis,
                        q.map_or(0, |q| q.pushed),
                        q.map_or(0, |q| q.consumed),
                        q.map_or(0, |q| q.ring_depth),
                        q.map_or(0, |q| q.room()),
                    );
                    break 'send;
                }
                Schedule::Send(frames) => {
                    if frames.is_empty() {
                        break 'send;
                    }
                    for mut f in frames {
                        let n = f.pieces.len() as u32;
                        // Look up cursor + compute new_head BEFORE borrowing the
                        // queue mutably after the send. Borrow ends at end of
                        // this block so the mut borrow below doesn't conflict.
                        let new_head = {
                            let q = queues.get(&f.key).expect("planned key exists");
                            debug_assert!(
                                q.ring_depth <= u32::from(u16::MAX),
                                "ring_depth {} exceeds u16::MAX; start_slot cast is lossy",
                                q.ring_depth
                            );
                            f.start_slot = q.physical_write_cursor as u16;
                            q.pushed.wrapping_add(n)
                        };
                        match sink.send_frame(f.key, &f.pieces, f.start_slot, new_head) {
                            Ok(code) => {
                                // PIECEDIAG (revert)
                                log::info!(
                                    "PIECEDIAG SEND axis={} count={} start_slot={} new_head={} -> OK rc={}",
                                    f.key.axis, n, f.start_slot, new_head, code
                                );
                                let q =
                                    queues.get_mut(&f.key).expect("planned key exists");
                                for _ in 0..f.pieces.len() {
                                    q.pieces.pop_front();
                                }
                                q.pushed = q.pushed.wrapping_add(n);
                                q.advance_write_cursor(n);
                            }
                            Err(ref e) => {
                                // PIECEDIAG (revert)
                                log::info!(
                                    "PIECEDIAG SEND axis={} count={} start_slot={} new_head={} -> ERR {}",
                                    f.key.axis, n, f.start_slot, new_head, e
                                );
                                log::error!(
                                    "pump send_frame failed for {:?}: {e}",
                                    f.key
                                );
                                // Leave the pieces queued; retry on next message.
                                break 'send;
                            }
                        }
                    }
                }
            }
        }
    }
}

// ── WireSink ─────────────────────────────────────────────────────────────────

/// Production sink: one `kalico_call(PushPieces)` per frame.
///
/// Holds `Weak<KalicoHostIo>` — NOT `Arc` — mirroring the dispatch_ios design
/// in bridge.rs: `detach_serial` drops the strong `Arc` to tear down the
/// reactor, so the pump must not pin the IO alive. An upgrade failure means
/// the MCU was detached; the frame is dropped with an error (the pump logs
/// and leaves the pieces queued — a detached MCU is a stream teardown).
pub struct WireSink {
    pub ios: HashMap<u32, Weak<kalico_host_rt::host_io::KalicoHostIo>>,
    pub timeout: Duration,
}

impl PieceSink for WireSink {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<i32, String> {
        // schedule() caps frames at MAX_PER_FRAME = 255, so this is unreachable
        // in the production path; the assert guards against callers bypassing
        // schedule() and hitting a silent truncation.
        debug_assert!(
            pieces.len() <= 255,
            "PushPieces frame exceeds u8 piece_count; schedule() must cap at 255"
        );
        let io = self
            .ios
            .get(&key.mcu_id)
            .and_then(Weak::upgrade)
            .ok_or_else(|| format!("KalicoHostIo for mcu {} detached", key.mcu_id))?;
        let mut pieces_bytes =
            Vec::with_capacity(pieces.len() * std::mem::size_of::<PieceEntry>());
        for p in pieces {
            pieces_bytes.extend_from_slice(&p.to_le_bytes());
        }
        let msg = kalico_protocol::messages::PushPieces {
            axis_idx: key.axis,
            piece_count: pieces.len() as u8,
            start_slot,
            new_head,
            pieces_bytes,
        };
        // 2 header bytes (axis_idx + piece_count) + serialised piece data.
        let mut body =
            Vec::with_capacity(2 + pieces.len() * std::mem::size_of::<PieceEntry>());
        kalico_protocol::codec::Encode::encode(&msg, &mut body);
        // PushPieces is sent on KALICO_CHANNEL_PIECES (0x02). The response
        // (PushPiecesResponse) arrives on the control channel matched by
        // correlation_id — no change to the response handling path.
        // DIAG(sip): time the PushPieces round-trip. If this is ~0.25s the
        // first piece is committed to the ring ~that-late (the MCU commits
        // before responding), eating the 0.25s lead → -308. REVERT after.
        let _ptx_t0 = std::time::Instant::now();
        let _ptx_wall = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros())
            .unwrap_or(0);
        let (_kind, resp) = io
            .kalico_call_on_channel(
                kalico_protocol::KALICO_CHANNEL_PIECES,
                kalico_protocol::MessageKind::PushPieces,
                body,
                self.timeout,
            )
            .map_err(|e| format!("kalico_call PushPieces: {e:?}"))?;
        {
            use std::io::Write as _;
            if let Ok(mut fh) = std::fs::OpenOptions::new()
                .create(true)
                .append(true)
                .open("/home/dderg/printer_data/logs/piece-tx.log")
            {
                let _ = writeln!(
                    fh,
                    "[ptx-send] wall_us={} mcu={} axis={} count={} start_slot={} new_head={} call_us={}",
                    _ptx_wall, key.mcu_id, key.axis, pieces.len(), start_slot, new_head,
                    _ptx_t0.elapsed().as_micros(),
                );
            }
        }
        use kalico_protocol::codec::Decode as _;
        let r = kalico_protocol::messages::PushPiecesResponse::decode(&resp)
            .map_err(|e| format!("decode PushPiecesResponse: {e:?}"))?;
        if r.result != kalico_protocol::result_codes::OK {
            return Err(format!(
                "MCU rejected PushPieces (mcu {} axis {}): {}",
                key.mcu_id, key.axis, r.result
            ));
        }
        Ok(r.result)
    }
}
