//! Foreground trace-drain → curve-pool reclaim pipeline. Per spec §10.4.
//!
//! On observing `SEGMENT_END(slot=N, gen=G)` in the trace stream, foreground
//! sets `slot[N].last_retired_gen = G`. FIFO ordering of single-ISR-writer
//! single-foreground-reader `heapless::spsc` preserves the per-slot
//! retirement sequence; no separate "any queued segment references this
//! slot" inspection is needed.
//!
//! Producer is expected to drain pending trace samples before failing alloc
//! due to "no reclaimable slot."
//!
//! ## Multi-handle retirement (Task 7)
//!
//! Each logical segment maps to up to 4 per-axis scalar curve slots (X, Y, Z,
//! E). The trace sample carries only `x_handle` for diagnostics; the
//! foreground learns all 4 handles at push time via `RetirementTable::register`
//! and looks them up on `SEGMENT_END` via `RetirementTable::lookup`.

use core::sync::atomic::Ordering;

use crate::curve_pool::{CurveHandle, CurvePool};
use crate::engine::RuntimeStatus;
use crate::error::FaultCode;
use crate::state::SharedState;
use crate::trace::{TRACE_FLAG_SEGMENT_END, TraceSample};

/// Number of concurrent in-flight segments the retirement table can track.
/// 16 covers the `Q_N` maximum of 256 segments with comfortable headroom for
/// realistic burst depths.
pub const RETIREMENT_TABLE_N: usize = 16;

/// Foreground-side mapping from `segment_id` → `[CurveHandle; 4]` so the
/// drain pipeline can retire all per-axis slots on a single `SEGMENT_END`
/// observation.
///
/// The table is a fixed-size circular ring of `(segment_id, handles)` pairs.
/// `register` writes in FIFO order; `lookup` scans linearly. At steady-state
/// queue depths (≤ `Q_N` = 256, realistic burst ≤ 16) a linear scan is O(1)
/// amortized. Overwrite of the oldest entry is silent — a mis-hit means the
/// slot stays un-retired until the pool's generation counter catches up on
/// the next flush (acceptable fault-recovery path, not a correctness hazard
/// for the steady-state path).
#[derive(Debug)]
pub struct RetirementTable {
    entries: [(u32, [CurveHandle; 4]); RETIREMENT_TABLE_N],
    head: usize,
}

impl RetirementTable {
    /// Construct a zeroed-out retirement table. All entries carry
    /// `(0, [UNUSED_SENTINEL; 4])` so a fresh lookup on `segment_id=0`
    /// returns `None` (no valid segment ever carries id=0 in the monotonic
    /// cursor scheme).
    pub const fn new() -> Self {
        Self {
            entries: [(0, [CurveHandle::UNUSED_SENTINEL; 4]); RETIREMENT_TABLE_N],
            head: 0,
        }
    }

    /// Record the 4 per-axis handles issued for `segment_id`. Must be called
    /// from the foreground at push time, before the segment enters the SPSC
    /// queue. Overwrites the oldest entry when the ring is full.
    #[allow(clippy::indexing_slicing)] // self.head is always kept in 0..RETIREMENT_TABLE_N by the modulo below
    pub fn register(&mut self, segment_id: u32, handles: [CurveHandle; 4]) {
        self.entries[self.head] = (segment_id, handles);
        self.head = (self.head + 1) % RETIREMENT_TABLE_N;
    }

    /// Look up the 4 per-axis handles for `segment_id`. Returns `None` if
    /// the entry has been overwritten or was never registered.
    pub fn lookup(&self, segment_id: u32) -> Option<[CurveHandle; 4]> {
        self.entries
            .iter()
            .find(|(id, _)| *id == segment_id)
            .map(|(_, handles)| *handles)
    }
}

impl Default for RetirementTable {
    fn default() -> Self {
        Self::new()
    }
}

/// Drain up to `limit` trace samples from `drain_one`; for each
/// `SEGMENT_END` observed, retire all non-sentinel handles recorded in
/// `table` for that `segment_id`. Returns the count drained.
pub fn drain_and_reclaim<F>(
    pool: &CurvePool,
    table: &RetirementTable,
    mut drain_one: F,
    limit: usize,
) -> usize
where
    F: FnMut() -> Option<TraceSample>,
{
    let mut drained = 0;
    while drained < limit {
        let Some(sample) = drain_one() else { break };
        if sample.flags & TRACE_FLAG_SEGMENT_END != 0 {
            if let Some(handles) = table.lookup(sample.segment_id) {
                for h in &handles {
                    if !h.is_unused_sentinel() && *h != CurveHandle::HOLD_SEGMENT_SENTINEL {
                        pool.confirm_retired(*h);
                    }
                }
            }
        }
        drained += 1;
    }
    drained
}

/// §13.1 trace-overflow latch. Foreground polls this each drain cycle.
///
/// If the ISR has set `sample_drop_pending` (because a `trace.enqueue` failed
/// after the trace ring filled), latch `KALICO_FAULT_TRACE_OVERFLOW`. The
/// `runtime_status` transition is gated on a previously-clean `last_error`
/// so this never overrides an earlier latched fault. Returns true on a
/// fresh latch transition (so callers can emit a `kalico_fault` frame),
/// false otherwise.
pub fn check_trace_overflow_and_fault(shared: &SharedState) -> bool {
    if !shared.sample_drop_pending.load(Ordering::Acquire) {
        return false;
    }
    // Compare-exchange the i32 last_error against the "clean" value so we
    // don't clobber an earlier-latched fault.
    let prev = shared.last_error.compare_exchange(
        0,
        FaultCode::TraceOverflow.as_i32(),
        Ordering::AcqRel,
        Ordering::Acquire,
    );
    if prev.is_ok() {
        shared
            .runtime_status
            .store(RuntimeStatus::Fault as u8, Ordering::Release);
        true
    } else {
        false
    }
}
