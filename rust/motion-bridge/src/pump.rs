use runtime::piece_ring::PieceEntry;
use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::sync::Weak;
use std::sync::mpsc::{Receiver, RecvError, RecvTimeoutError};
use std::time::Duration;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug, PartialOrd, Ord)]
pub struct AxisKey {
    pub mcu_id: u32,
    pub axis: u8,
}

#[derive(Debug)]
pub struct AxisQueue {
    pub pieces: VecDeque<(PieceEntry, f64)>,
    pub pushed: u32,
    pub retired: u32,
    pub ring_depth: u32,
    pub physical_write_cursor: u32,
}

impl AxisQueue {
    pub fn new(ring_depth: u32) -> Self {
        Self {
            pieces: VecDeque::new(),
            pushed: 0,
            retired: 0,
            ring_depth,
            physical_write_cursor: 0,
        }
    }
    pub fn room(&self) -> u32 {
        let in_flight = self.pushed.wrapping_sub(self.retired);
        self.ring_depth.saturating_sub(in_flight)
    }
    pub fn advance_write_cursor(&mut self, n: u32) {
        if self.ring_depth == 0 {
            return;
        }
        self.physical_write_cursor = (self.physical_write_cursor + n) % self.ring_depth;
    }
}

#[derive(Debug)]
pub struct FramePlan {
    pub key: AxisKey,
    pub pieces: Vec<PieceEntry>,
    pub start_slot: u16,
}

#[derive(Debug)]
pub enum Schedule {
    Send(Vec<FramePlan>),
    StallFull(AxisKey),
    StallAhead(AxisKey),
    Idle,
}

/// Select the globally earliest-host-time piece across all queues, then emit
/// the same-MCU prefix as one frame batch.
///
/// ## Invariants preserved
///
/// 1. **Global gating is intentional.** A stalled MCU (StallFull / StallAhead)
///    gates issuance to all other MCUs — cross-MCU issue-side coherence requires
///    that a blocked MCU is never overtaken.
///
/// 2. **Ordering must use host time.** `start_time` values are raw MCU clock
///    ticks in per-MCU clock domains (H7: ~520 MHz / own epoch; F446: ~180 MHz /
///    own epoch). Comparing ticks across MCUs is meaningless; F446 values are
///    numerically smaller and would always win, starving the H7.  The `f64`
///    sidecar in each `(PieceEntry, f64)` pair carries the minting host time
///    (`t0 + u_start`, seconds) and is the only valid cross-queue ordering key.
#[must_use]
pub fn schedule(
    queues: &BTreeMap<AxisKey, AxisQueue>,
    max_per_frame: usize,
    horizon_of: impl Fn(u32) -> Option<u64>,
) -> Schedule {
    let mut stall_ahead_candidate: Option<AxisKey> = None;

    // Head selection: earliest host time across all non-empty queues.
    let head = queues
        .iter()
        .filter(|(_, q)| !q.pieces.is_empty())
        .min_by(|(ka, qa), (kb, qb)| {
            let ha = qa.pieces.front().unwrap().1;
            let hb = qb.pieces.front().unwrap().1;
            ha.total_cmp(&hb).then(ka.cmp(kb))
        });
    let (&head_key, head_q) = match head {
        None => return Schedule::Idle,
        Some(h) => h,
    };
    let (head_entry, _head_host) = head_q.pieces.front().unwrap();
    let head_start_ticks = head_entry.start_time;

    if head_q.room() == 0 {
        return Schedule::StallFull(head_key);
    }

    if let Some(horizon) = horizon_of(head_key.mcu_id) {
        if head_start_ticks > horizon {
            return Schedule::StallAhead(head_key);
        }
    }

    let mut taken: BTreeMap<AxisKey, usize> = BTreeMap::new();
    let mut maxed: BTreeSet<AxisKey> = BTreeSet::new();
    loop {
        // Inner batching: order candidates by host time within the same MCU prefix.
        let next = queues
            .iter()
            .filter_map(|(k, q)| {
                if maxed.contains(k) {
                    return None;
                }
                let already = taken.get(k).copied().unwrap_or(0);
                q.pieces.get(already).map(|&(ref p, host)| (*k, p.start_time, host))
            })
            .min_by(|(ka, _, ha), (kb, _, hb)| ha.total_cmp(hb).then(ka.cmp(kb)));
        let (k, start_ticks, _host) = match next {
            Some(n) => n,
            None => break,
        };
        if k.mcu_id != head_key.mcu_id {
            break;
        }
        let already = taken.get(&k).copied().unwrap_or(0);
        let q = &queues[&k];
        let room = q.room() as usize;
        if already >= room || already >= max_per_frame {
            maxed.insert(k);
            continue;
        }
        if let Some(horizon) = horizon_of(k.mcu_id) {
            if start_ticks > horizon {
                if stall_ahead_candidate.is_none() {
                    stall_ahead_candidate = Some(k);
                }
                maxed.insert(k);
                continue;
            }
        }
        *taken.entry(k).or_insert(0) += 1;
    }

    if taken.is_empty() {
        if let Some(k) = stall_ahead_candidate {
            return Schedule::StallAhead(k);
        }
        return Schedule::StallFull(head_key);
    }

    let frames: Vec<FramePlan> = taken
        .into_iter()
        .filter(|(_, n)| *n > 0)
        .map(|(k, n)| FramePlan {
            key: k,
            pieces: queues[&k].pieces.iter().take(n).map(|(p, _)| *p).collect(),
            start_slot: 0,
        })
        .collect();
    debug_assert!(!frames.is_empty());
    Schedule::Send(frames)
}

#[cfg(test)]
mod sched_tests {
    use super::*;

    /// Build a queue from (tick_start, host_time) pairs.
    fn q_with_host(ring_depth: u32, starts: &[(u64, f64)]) -> AxisQueue {
        let mut q = AxisQueue::new(ring_depth);
        for &(s, h) in starts {
            q.pieces.push_back((
                PieceEntry {
                    start_time: s,
                    coeffs: [0.0; 4],
                    duration: 0.001,
                    _reserved: 0,
                },
                h,
            ));
        }
        q
    }

    /// Convenience wrapper for single-MCU tests where host_time == tick as f64.
    fn q_with(ring_depth: u32, starts: &[u64]) -> AxisQueue {
        let pairs: Vec<(u64, f64)> = starts.iter().map(|&s| (s, s as f64)).collect();
        q_with_host(ring_depth, &pairs)
    }

    #[test]
    fn idle_when_empty() {
        let queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
        assert!(matches!(schedule(&queues, 255, |_| None), Schedule::Idle));
    }

    #[test]
    fn stalls_when_global_head_ring_full() {
        let mut queues = BTreeMap::new();
        let mut a = q_with(2, &[10]);
        a.pushed = 2; // full
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, a);
        queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[20]));
        assert!(matches!(
            schedule(&queues, 255, |_| None),
            Schedule::StallFull(AxisKey { mcu_id: 1, axis: 0 })
        ));
    }

    #[test]
    fn batches_contiguous_same_mcu_prefix_only() {
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[0, 3]));
        queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[1]));
        queues.insert(AxisKey { mcu_id: 2, axis: 0 }, q_with(8, &[2]));
        let s = schedule(&queues, 255, |_| None);
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
        let s = schedule(&queues, 2, |_| None);
        match s {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1);
                assert_eq!(frames[0].pieces.len(), 2);
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn full_axis_does_not_block_same_mcu_sibling() {
        let mut q: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
        let yq = q_with(8, &[0, 2]);
        let mut xq = q_with(1, &[1]);
        xq.pushed = 1;
        q.insert(AxisKey { mcu_id: 1, axis: 1 }, yq);
        q.insert(AxisKey { mcu_id: 1, axis: 0 }, xq);
        match schedule(&q, 255, |_| None) {
            Schedule::Send(frames) => {
                let yf = frames
                    .iter()
                    .find(|f| f.key == AxisKey { mcu_id: 1, axis: 1 });
                assert!(yf.is_some(), "Y should be batched despite full sibling X");
                assert!(
                    !frames
                        .iter()
                        .any(|f| f.key == AxisKey { mcu_id: 1, axis: 0 }),
                    "full X must not appear in the batch"
                );
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn time_gate_blocks_piece_beyond_horizon() {
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[100]));
        queues.insert(AxisKey { mcu_id: 1, axis: 1 }, q_with(8, &[200]));
        match schedule(&queues, 255, |_| Some(150)) {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1, "only axis 0 should be batched");
                assert_eq!(frames[0].key, AxisKey { mcu_id: 1, axis: 0 });
                assert_eq!(frames[0].pieces.len(), 1);
            }
            other => panic!("expected Send, got {other:?}"),
        }
    }

    #[test]
    fn all_beyond_horizon_returns_stall_ahead() {
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[1000]));
        assert!(
            matches!(
                schedule(&queues, 255, |_| Some(500)),
                Schedule::StallAhead(AxisKey { mcu_id: 1, axis: 0 })
            ),
            "expected StallAhead when sole piece is beyond horizon"
        );
    }

    #[test]
    fn no_horizon_none_uses_count_only_gate() {
        let mut queues = BTreeMap::new();
        queues.insert(AxisKey { mcu_id: 1, axis: 0 }, q_with(8, &[u64::MAX]));
        match schedule(&queues, 255, |_| None) {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1);
                assert_eq!(frames[0].pieces.len(), 1);
            }
            other => panic!("expected Send (no time gate), got {other:?}"),
        }
    }

    /// Regression: bench shape from 2026-06-04.
    ///
    /// mcu1 (F446, 180 MHz) has a far-future piece whose tick value is numerically
    /// small (~4.8e12) but whose host time is large (beyond mcu1's horizon).
    /// mcu0 (H7, 520 MHz) has past-due pieces whose tick value is numerically large
    /// (~13.8e12) but whose host time is now-ish (within mcu0's horizon, room available).
    ///
    /// Old code: `min_by` on raw ticks → mcu1's small ticks win → StallAhead(mcu1),
    /// H7 starved for up to 2+ seconds.
    /// New code: `min_by` on host time → mcu0's smaller host time wins → Send(mcu0).
    #[test]
    fn cross_mcu_host_time_ordering_bench_regression() {
        let f446_tick: u64 = 4_790_000_000_000;
        let h7_tick: u64 = 13_800_000_000_000;

        // mcu1 (F446): numerically smaller tick, but host time is far in the future
        let f446_host: f64 = 1_000.0; // far future, beyond any horizon
        // mcu0 (H7): numerically larger tick, but host time is now-ish
        let h7_host: f64 = 1.0; // near present

        let mut queues = BTreeMap::new();
        queues.insert(
            AxisKey { mcu_id: 1, axis: 2 }, // F446, Z axis
            q_with_host(8, &[(f446_tick, f446_host)]),
        );
        queues.insert(
            AxisKey { mcu_id: 0, axis: 0 }, // H7, X axis
            q_with_host(8, &[(h7_tick, h7_host)]),
        );

        // mcu0 horizon covers h7_tick; mcu1 horizon does NOT cover f446_tick.
        let horizon_of = |mcu_id: u32| -> Option<u64> {
            if mcu_id == 0 {
                Some(h7_tick + 1_000_000)
            } else {
                Some(f446_tick - 1) // mcu1 piece is ahead of this horizon
            }
        };

        match schedule(&queues, 255, horizon_of) {
            Schedule::Send(frames) => {
                assert_eq!(frames.len(), 1);
                assert_eq!(
                    frames[0].key.mcu_id, 0,
                    "H7 (mcu0) should be selected, not F446 (mcu1)"
                );
            }
            other => panic!(
                "expected Send(mcu0) — cross-MCU host-time ordering regression, got {other:?}"
            ),
        }
    }

    /// The globally host-earliest piece's queue having room==0 must block everything
    /// (intentional global-gating invariant).
    #[test]
    fn stall_full_on_globally_earliest_gates_all() {
        let mut queues = BTreeMap::new();

        // mcu0: host-earliest piece, but ring is full
        let mut mcu0_q = q_with_host(2, &[(100, 1.0)]);
        mcu0_q.pushed = 2; // full
        queues.insert(AxisKey { mcu_id: 0, axis: 0 }, mcu0_q);

        // mcu1: host-later piece, ring has room
        queues.insert(
            AxisKey { mcu_id: 1, axis: 0 },
            q_with_host(8, &[(50, 5.0)]),
        );

        assert!(
            matches!(
                schedule(&queues, 255, |_| None),
                Schedule::StallFull(AxisKey { mcu_id: 0, axis: 0 })
            ),
            "StallFull on the globally host-earliest queue must gate all issuance"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc;
    use std::sync::{Arc, Mutex};

    #[test]
    fn room_full_then_drains() {
        let mut q = AxisQueue::new(4);
        assert_eq!(q.room(), 4);
        q.pushed = 4;
        assert_eq!(q.room(), 0); // full
        q.retired = 1;
        assert_eq!(q.room(), 1); // one freed
    }

    #[test]
    fn room_correct_across_u32_wrap() {
        let mut q = AxisQueue::new(8);
        q.pushed = 2;
        q.retired = u32::MAX;
        assert_eq!(q.room(), 5);
    }

    #[test]
    fn physical_write_cursor_advances_and_wraps_at_n() {
        let mut q = AxisQueue::new(4);
        assert_eq!(q.physical_write_cursor, 0);
        q.advance_write_cursor(3);
        assert_eq!(q.physical_write_cursor, 3);
        q.advance_write_cursor(3);
        assert_eq!(q.physical_write_cursor, 2);
    }

    #[derive(Clone)]
    struct RecordingSink {
        calls: Arc<Mutex<Vec<(u16, u32)>>>,
    }

    impl RecordingSink {
        fn new() -> Self {
            Self {
                calls: Arc::new(Mutex::new(Vec::new())),
            }
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

    fn make_piece(t: u64) -> (PieceEntry, f64) {
        (
            PieceEntry {
                start_time: t,
                coeffs: [0.0; 4],
                duration: 0.001,
                _reserved: 0,
            },
            t as f64,
        )
    }

    #[test]
    fn run_pump_sets_start_slot_from_cursor_and_advances_it() {
        const RING_DEPTH: u32 = 8;
        const N: u32 = 3;

        let sink = RecordingSink::new();
        let (tx, rx) = mpsc::channel::<PumpMsg>();
        let sink_clone = sink.clone();
        let handle = std::thread::spawn(move || {
            run_pump(rx, sink_clone, |_key| RING_DEPTH, |_mcu| None);
        });

        tx.send(PumpMsg::Enqueue(EnqueueMsg {
            key: AxisKey { mcu_id: 1, axis: 0 },
            pieces: (0..N).map(|i| make_piece(i as u64)).collect(),
            fresh_stream: false,
        }))
        .unwrap();
        {
            let deadline = std::time::Instant::now() + std::time::Duration::from_secs(2);
            while sink.recorded().is_empty() {
                assert!(
                    std::time::Instant::now() < deadline,
                    "pump did not drain first batch within deadline"
                );
                std::thread::yield_now();
            }
        }

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
        assert_eq!(
            recorded.len(),
            2,
            "expected exactly 2 sends, got {}",
            recorded.len()
        );

        let (s0, h0) = recorded[0];
        let (s1, h1) = recorded[1];

        assert_eq!(s0, 0, "first start_slot should be 0");
        assert_eq!(h0, N, "first new_head should be N={N}");

        let expected_s1 = (N % RING_DEPTH) as u16;
        assert_eq!(s1, expected_s1, "second start_slot should be {expected_s1}");
        assert_eq!(h1, N * 2, "second new_head should be {}", N * 2);
    }

    #[test]
    fn junction_jumps_math() {
        // Exact gap: first piece starts exactly where previous ended.
        let (tick_us, host_us) = junction_jumps(2000, 2.0e-3, 1000, 1.0e-3, 1_000_000.0);
        assert!((tick_us - 1000.0).abs() < 1e-6, "tick_jump_us={tick_us}");
        assert!((host_us - 1000.0).abs() < 1e-6, "host_jump_us={host_us}");

        // Overlap (negative jump).
        let (tick_us2, host_us2) = junction_jumps(900, 0.9e-3, 1000, 1.0e-3, 1_000_000.0);
        assert!(tick_us2 < 0.0, "overlap should be negative tick jump");
        assert!(host_us2 < 0.0, "overlap should be negative host jump");

        // Cross-domain divergence: tick gap == 0 µs but host gap == 500 µs.
        let freq = 520_000_000.0_f64;
        let prev_end_ticks: u64 = 10_000;
        let (tick_us3, host_us3) =
            junction_jumps(prev_end_ticks, 5.0e-4, prev_end_ticks, 0.0, freq);
        assert!((tick_us3).abs() < 1e-6, "tick gap should be zero, got {tick_us3}");
        assert!((host_us3 - 500.0).abs() < 1e-3, "host_jump_us={host_us3}");
    }
}

pub struct EnqueueMsg {
    pub key: AxisKey,
    /// Each entry pairs the `PieceEntry` with its minting host time (`t0 + u_start`, seconds).
    /// The host time is the ordering key used by `schedule()`; the raw `start_time` tick is
    /// MCU-clock-domain-specific and incomparable across MCUs.
    pub pieces: Vec<(PieceEntry, f64)>,
    pub fresh_stream: bool,
}

pub struct HeartbeatMsg {
    pub mcu_id: u32,
    pub retired_counts: Vec<u32>,
}

pub enum PumpMsg {
    Enqueue(EnqueueMsg),
    Heartbeat(HeartbeatMsg),
    Shutdown,
}

pub trait PieceSink: Send {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<i32, String>;
}

/// Compute the tick-domain and host-domain gaps at a batch boundary.
///
/// Returns `(tick_jump_us, host_jump_us)` where negative values indicate overlap.
/// The difference between the two isolates clock-projection error from planner-intent gaps.
pub fn junction_jumps(
    first_start_ticks: u64,
    first_host: f64,
    prev_end_ticks: u64,
    prev_end_host: f64,
    approx_freq_hz: f64,
) -> (f64, f64) {
    let tick_jump_us =
        (first_start_ticks as i64 - prev_end_ticks as i64) as f64 / approx_freq_hz * 1e6;
    let host_jump_us = (first_host - prev_end_host) * 1e6;
    (tick_jump_us, host_jump_us)
}

const MAX_LEAD_SECS: f64 = 1.0;

pub fn run_pump<S, F, C>(rx: Receiver<PumpMsg>, sink: S, ring_depth_of: F, mcu_clock_of: C)
where
    S: PieceSink,
    F: Fn(AxisKey) -> u32,
    C: Fn(u32) -> Option<(u64, f64)>,
{
    let mut queues: BTreeMap<AxisKey, AxisQueue> = BTreeMap::new();
    // Per-axis junction tracking for clock-projection jitter measurement.
    let mut junction_ends: BTreeMap<AxisKey, (u64, f64)> = BTreeMap::new();
    const MAX_PER_FRAME: usize = 32;

    let apply = |msg: PumpMsg,
                 queues: &mut BTreeMap<AxisKey, AxisQueue>,
                 junction_ends: &mut BTreeMap<AxisKey, (u64, f64)>|
     -> bool {
        match msg {
            PumpMsg::Shutdown => return false,
            PumpMsg::Enqueue(EnqueueMsg {
                key,
                pieces,
                fresh_stream,
            }) => {
                if fresh_stream {
                    junction_ends.remove(&key);
                }
                if !pieces.is_empty() {
                    // Clock not yet synced — skip junction bookkeeping; µs math is
                    // meaningless without a real frequency and end_time needs it too.
                    if let Some((_ack_now, freq)) = mcu_clock_of(key.mcu_id) {
                        let (first_entry, first_host) = &pieces[0];
                        if let Some(&(prev_end_ticks, prev_end_host)) = junction_ends.get(&key) {
                            let (tick_jump_us, host_jump_us) = junction_jumps(
                                first_entry.start_time,
                                *first_host,
                                prev_end_ticks,
                                prev_end_host,
                                freq,
                            );
                            let anomalous = tick_jump_us < -50.0
                                || (tick_jump_us - host_jump_us).abs() > 50.0;
                            if fresh_stream || !anomalous {
                                log::debug!(
                                    "[junction] key={:?} tick_jump_us={:.1} host_jump_us={:.1} fresh={}",
                                    key,
                                    tick_jump_us,
                                    host_jump_us,
                                    fresh_stream,
                                );
                            } else {
                                let reason = if tick_jump_us < -50.0 {
                                    "overlap_risk"
                                } else {
                                    "projection_divergence"
                                };
                                log::warn!(
                                    "[junction] key={:?} tick_jump_us={:.1} host_jump_us={:.1} fresh={} reason={}",
                                    key,
                                    tick_jump_us,
                                    host_jump_us,
                                    fresh_stream,
                                    reason,
                                );
                            }
                        }
                        let (last_entry, last_host) = pieces.last().unwrap();
                        #[allow(clippy::cast_possible_truncation)]
                        let last_end_ticks = last_entry.end_time(freq as f32);
                        let last_end_host = last_host + last_entry.duration as f64;
                        junction_ends.insert(key, (last_end_ticks, last_end_host));
                    }
                }
                let q = queues
                    .entry(key)
                    .or_insert_with(|| AxisQueue::new(ring_depth_of(key)));
                q.pieces.extend(pieces);
            }
            PumpMsg::Heartbeat(HeartbeatMsg {
                mcu_id,
                retired_counts,
            }) => {
                // retired_counts[i] is axis index i; same numbering as PushPieces.axis_idx in runtime_ffi.rs — do not reorder either side.
                for (axis, &c) in retired_counts.iter().enumerate() {
                    let key = AxisKey {
                        mcu_id,
                        axis: axis as u8,
                    };
                    if let Some(q) = queues.get_mut(&key) {
                        q.retired = c;
                    }
                }
            }
        }
        true
    };

    let horizon_of = |mcu_id: u32| -> Option<u64> {
        let (ack_now, freq) = mcu_clock_of(mcu_id)?;
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        Some(ack_now + (MAX_LEAD_SECS * freq) as u64)
    };

    let mut holding_ahead = false;

    loop {
        // If we are holding pieces that are time-gated, poll every 50 ms so
        // the horizon sweeps forward (ack_now advances with the MCU clock).
        // Otherwise block indefinitely — a heartbeat or enqueue will wake us.
        let first = if holding_ahead {
            match rx.recv_timeout(Duration::from_millis(50)) {
                Ok(m) => Some(m),
                Err(RecvTimeoutError::Timeout) => None,
                Err(RecvTimeoutError::Disconnected) => return,
            }
        } else {
            match rx.recv() {
                Ok(m) => Some(m),
                Err(RecvError) => return,
            }
        };

        if let Some(msg) = first {
            if !apply(msg, &mut queues, &mut junction_ends) {
                return;
            }
            while let Ok(m) = rx.try_recv() {
                if !apply(m, &mut queues, &mut junction_ends) {
                    return;
                }
            }
        }

        holding_ahead = false;
        'send: loop {
            match schedule(&queues, MAX_PER_FRAME, &horizon_of) {
                Schedule::Idle => break 'send,
                Schedule::StallFull(_stall_key) => {
                    break 'send;
                }
                Schedule::StallAhead(_stall_key) => {
                    holding_ahead = true;
                    break 'send;
                }
                Schedule::Send(frames) => {
                    if frames.is_empty() {
                        break 'send;
                    }
                    for mut f in frames {
                        let n = f.pieces.len() as u32;
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
                            Ok(_) => {
                                let q = queues.get_mut(&f.key).expect("planned key exists");
                                for _ in 0..f.pieces.len() {
                                    q.pieces.pop_front();
                                }
                                q.pushed = q.pushed.wrapping_add(n);
                                q.advance_write_cursor(n);
                            }
                            Err(ref e) => {
                                log::error!("pump send_frame failed for {:?}: {e}", f.key);
                                break 'send;
                            }
                        }
                    }
                }
            }
        }
    }
}

pub enum McuTransport {
    Serial(Weak<kalico_host_rt::host_io::KalicoHostIo>),
    EtherCat(std::sync::Arc<kalico_host_rt::unix_native_conn::UnixNativeConn>),
}

impl std::fmt::Debug for McuTransport {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Serial(_) => write!(f, "McuTransport::Serial"),
            Self::EtherCat(_) => write!(f, "McuTransport::EtherCat"),
        }
    }
}

pub struct WireSink {
    pub transports: HashMap<u32, McuTransport>,
    pub timeout: Duration,
}

impl WireSink {
    fn call_push_pieces(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<kalico_protocol::messages::PushPiecesResponse, String> {
        let mut pieces_bytes = Vec::with_capacity(std::mem::size_of_val(pieces));
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
        let mut body = Vec::with_capacity(8 + std::mem::size_of_val(pieces));
        kalico_protocol::codec::Encode::encode(&msg, &mut body);

        let transport = self.transports.get(&key.mcu_id).ok_or_else(|| {
            format!(
                "WireSink: no transport for mcu_id {} (axis {}); \
                 this is a logic bug in init_planner — the axis was enqueued \
                 without registering its transport",
                key.mcu_id, key.axis
            )
        })?;

        let resp_body = match transport {
            McuTransport::Serial(weak) => {
                let io = weak
                    .upgrade()
                    .ok_or_else(|| format!("KalicoHostIo for mcu {} detached", key.mcu_id))?;
                let (_kind, b) = io
                    .kalico_call_on_channel(
                        kalico_protocol::KALICO_CHANNEL_PIECES,
                        kalico_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| format!("serial PushPieces mcu {}: {e:?}", key.mcu_id))?;
                b
            }
            McuTransport::EtherCat(conn) => {
                let (_kind, b) = conn
                    .kalico_call_on_channel(
                        kalico_protocol::KALICO_CHANNEL_PIECES,
                        kalico_protocol::MessageKind::PushPieces,
                        body,
                        self.timeout,
                    )
                    .map_err(|e| format!("ethercat PushPieces mcu {}: {e:?}", key.mcu_id))?;
                b
            }
        };

        use kalico_protocol::codec::Decode as _;
        kalico_protocol::messages::PushPiecesResponse::decode(&resp_body)
            .map_err(|e| format!("decode PushPiecesResponse mcu {}: {e:?}", key.mcu_id))
    }
}

impl PieceSink for WireSink {
    fn send_frame(
        &self,
        key: AxisKey,
        pieces: &[PieceEntry],
        start_slot: u16,
        new_head: u32,
    ) -> Result<i32, String> {
        // schedule() caps frames at MAX_PER_FRAME (currently 32); this guards
        // against callers bypassing schedule() and hitting a silent truncation.
        debug_assert!(
            pieces.len() <= 255,
            "PushPieces frame exceeds u8 piece_count; schedule() must cap at MAX_PER_FRAME"
        );

        let host_front_start_time: u64 = pieces.first().map(|p| p.start_time).unwrap_or(0);

        let r = self.call_push_pieces(key, pieces, start_slot, new_head)?;

        {
            let arrival_lead_ticks = r.front_start_time as i64 - r.arrival_clock as i64;
            let approx_freq_hz: f64 = if r.arrival_clock > 1_000_000_000_000 {
                1_000_000_000.0
            } else if key.mcu_id == 0 {
                520_000_000.0
            } else {
                180_000_000.0
            };
            let arrival_lead_us = (arrival_lead_ticks as f64 / approx_freq_hz) * 1e6;
            let host_send_secs = {
                use std::time::{SystemTime, UNIX_EPOCH};
                SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map(|d| d.as_secs_f64())
                    .unwrap_or(0.0)
            };
            let zero_st = host_front_start_time == 0;
            let past_arrival = arrival_lead_ticks < 0;
            if zero_st || past_arrival {
                log::warn!(
                    "[transit-diag] mcu={} axis={} \
                     host_front_start_time={} mcu_front_start_time={} \
                     arrival_clock={} \
                     arrival_lead_ticks={} arrival_lead_us={:.1} \
                     host_send_unix_secs={:.6} \
                     ALERT: {}",
                    key.mcu_id,
                    key.axis,
                    host_front_start_time,
                    r.front_start_time,
                    r.arrival_clock,
                    arrival_lead_ticks,
                    arrival_lead_us,
                    host_send_secs,
                    if zero_st && past_arrival {
                        "host_start_time=0 (clock-sync gap) AND piece in MCU past"
                    } else if zero_st {
                        "host_start_time=0 (router clock_freq=0 at dispatch — clock-sync gap)"
                    } else {
                        "piece arrived in MCU past (arrival_lead<0) — PieceStartInPast risk"
                    },
                );
            } else {
                log::info!(
                    "[transit-diag] mcu={} axis={} \
                     host_front_start_time={} mcu_front_start_time={} \
                     arrival_clock={} \
                     arrival_lead_ticks={} arrival_lead_us={:.1} \
                     host_send_unix_secs={:.6}",
                    key.mcu_id,
                    key.axis,
                    host_front_start_time,
                    r.front_start_time,
                    r.arrival_clock,
                    arrival_lead_ticks,
                    arrival_lead_us,
                    host_send_secs,
                );
            }
        }

        if r.result != kalico_protocol::result_codes::OK {
            return Err(format!(
                "MCU rejected PushPieces (mcu {} axis {}): {}",
                key.mcu_id, key.axis, r.result
            ));
        }
        Ok(r.result)
    }
}
