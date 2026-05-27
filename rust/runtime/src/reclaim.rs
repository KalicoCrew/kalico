//! Retirement table stub — Task 5 placeholder.
//!
//! The full foreground trace-drain → curve-pool reclaim pipeline has been
//! removed. This stub retains `RetirementTable` so `FgState` compiles
//! until Task 6.

use crate::curve_pool::{CurveHandle, CurvePool};
use crate::state::SharedState;
use crate::trace::TraceSample;

/// Number of concurrent in-flight segments the table can track.
pub const RETIREMENT_TABLE_N: usize = 16;

/// Foreground-side mapping from `segment_id → [CurveHandle; 4]`.
/// Stub — no logic, just compiles.
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

/// Stub drain-and-reclaim pipeline. Returns the number of samples consumed.
///
/// Task 6 replaces with the real trace-drain → curve-pool retire logic.
#[allow(clippy::unused_variables)]
pub fn drain_and_reclaim<F>(
    _pool: &CurvePool,
    _table: &RetirementTable,
    mut dequeue: F,
    limit: usize,
) -> usize
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

/// Stub trace-overflow fault check. Always returns `false` (no overflow).
///
/// Task 6 wires this to the real `SharedState::sample_drop_pending` latch.
#[allow(clippy::unused_variables)]
pub fn check_trace_overflow_and_fault(_shared: &SharedState) -> bool {
    false
}
