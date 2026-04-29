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

use crate::curve_pool::CurvePool;
use crate::trace::{TRACE_FLAG_SEGMENT_END, TraceSample};

/// Drain up to `limit` trace samples from `drain_one`; for each
/// `SEGMENT_END` observed, advance `pool.confirm_retired(handle)` so the
/// slot's `last_retired_gen` follows the per-slot retirement sequence.
/// Returns the count drained.
pub fn drain_and_reclaim<F>(pool: &CurvePool, mut drain_one: F, limit: usize) -> usize
where
    F: FnMut() -> Option<TraceSample>,
{
    let mut drained = 0;
    while drained < limit {
        let Some(sample) = drain_one() else { break };
        if sample.flags & TRACE_FLAG_SEGMENT_END != 0 {
            pool.confirm_retired(sample.curve_handle);
        }
        drained += 1;
    }
    drained
}
