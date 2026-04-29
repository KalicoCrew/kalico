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

use core::sync::atomic::Ordering;

use crate::curve_pool::CurvePool;
use crate::engine::RuntimeStatus;
use crate::error::FaultCode;
use crate::state::SharedState;
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
