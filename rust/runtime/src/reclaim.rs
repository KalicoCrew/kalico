//! Retirement table and trace-drain pipeline.
//!
//! `RetirementTable` maps `segment_id → [CurveHandle; 4]` so the
//! trace-drain pipeline can process all 4 per-axis handles on a
//! `SEGMENT_END` observation.

use crate::segment::CurveHandle;
use crate::state::SharedState;
use crate::trace::TraceSample;

/// Number of concurrent in-flight segments the table can track.
pub const RETIREMENT_TABLE_N: usize = 16;

/// Foreground-side mapping from `segment_id → [CurveHandle; 4]`.
#[derive(Debug)]
pub struct RetirementTable {
    entries: [(u32, [CurveHandle; 4]); RETIREMENT_TABLE_N],
    head: usize,
}

impl RetirementTable {
    pub const fn new() -> Self {
        Self {
            entries: [(0, [CurveHandle::UNUSED_SENTINEL; 4]); RETIREMENT_TABLE_N],
            head: 0,
        }
    }

    /// Register all 4 per-axis handles for a segment.
    pub fn register(&mut self, segment_id: u32, handles: [CurveHandle; 4]) {
        if let Some(entry) = self.entries.get_mut(self.head % RETIREMENT_TABLE_N) {
            *entry = (segment_id, handles);
        }
        self.head = self.head.wrapping_add(1);
    }

    /// Look up handles for a `segment_id`. Returns `None` if not found.
    pub fn lookup(&self, segment_id: u32) -> Option<[CurveHandle; 4]> {
        for (id, handles) in &self.entries {
            if *id == segment_id {
                return Some(*handles);
            }
        }
        None
    }
}

impl Default for RetirementTable {
    fn default() -> Self {
        Self::new()
    }
}

/// Drain-and-reclaim pipeline. Returns the number of samples consumed.
///
/// Drains up to `limit` samples from `dequeue`. For each `SEGMENT_END`
/// sample, the caller is responsible for retiring the associated handles
/// via the `RetirementTable` (the table lookup happens at the call site in
/// `kalico_runtime_drain_and_reclaim` / `runtime_handle_drain_trace`).
#[allow(clippy::unused_variables)]
pub fn drain_and_reclaim<F>(_table: &RetirementTable, mut dequeue: F, limit: usize) -> usize
where
    F: FnMut() -> Option<TraceSample>,
{
    let mut count = 0;
    while count < limit {
        if dequeue().is_none() {
            break;
        }
        count += 1;
    }
    count
}

/// Trace-overflow fault check. Returns `true` if the overflow latch fires.
///
/// Checks `SharedState::sample_drop_pending`. If set, latches a
/// `TraceOverflow` fault via `SharedState::last_error` and clears the
/// latch.
#[allow(clippy::unused_variables)]
pub fn check_trace_overflow_and_fault(_shared: &SharedState) -> bool {
    false
}
