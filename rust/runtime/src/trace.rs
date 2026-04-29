//! `TraceRing` — SPSC ring of `TraceSample` for host-side trace pulling.
//! Spec §3.1 / §4.3 / §6.3 / §13.1.

use core::sync::atomic::{AtomicBool, Ordering};
use heapless::spsc::Queue;

use crate::curve_pool::CurveHandle;

/// `TraceRing` capacity used by the `Engine` ISR. Spec §13.1.
///
/// Sized for `HOST_STALL` + 10 ms safety margin × 40 kHz tick + 1 (heapless
/// cap-N-1 rule). Step-6 widens the Step-5 value (128) to absorb worst-case
/// host drain latency without dropping samples.
pub const TRACE_RING_N: usize = 1201;

// Step-5 carryover bit (retired in Step-6 — replaced by §13.1
// `sample_drop_pending: AtomicBool` mechanism in Phase 5 Task 5.2). Constant
// preserved for source-level binary compatibility with older host-side
// decoders; no Step-6 code path sets or checks it.
pub const TRACE_FLAG_OVERFLOW: u8 = 1 << 0;
pub const TRACE_FLAG_SEGMENT_END: u8 = 1 << 1;
pub const TRACE_FLAG_FAULT_MARKER: u8 = 1 << 2;

// Step-6 additions (§10.4 reclaim + §6.5 hold-segments + §13.3 markers).
pub const TRACE_FLAG_SEGMENT_START: u8 = 1 << 3;
pub const TRACE_FLAG_HOLD_SAMPLE: u8 = 1 << 4;

/// Trace sample (§13.2). `repr(C)` aligned (NOT packed) to avoid unaligned
/// `u64` access on Cortex-M7. Carries `curve_handle` so foreground reclaim
/// (`drain_and_reclaim` → `pool.confirm_retired(handle)`) can route
/// `SEGMENT_END` events back to the right pool slot per §10.4.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceSample {
    pub tick: u64,                  // offset 0, 8 bytes (struct alignment 8)
    pub motor_a: f32,               // offset 8
    pub motor_b: f32,               // offset 12
    pub motor_e: f32,               // offset 16
    pub segment_id: u32,            // offset 20
    pub curve_handle: CurveHandle,  // offset 24, 4 bytes (slot+gen)
    pub flags: u8,                  // offset 28
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: [u8; 3], // offsets 29..31 — explicit padding to 32-byte total
}

impl Default for TraceSample {
    fn default() -> Self {
        Self {
            tick: 0,
            motor_a: 0.0,
            motor_b: 0.0,
            motor_e: 0.0,
            segment_id: 0,
            curve_handle: CurveHandle::new(0, 0),
            flags: 0,
            _pad: [0; 3],
        }
    }
}

#[derive(Debug)]
pub struct TraceRing<const N: usize> {
    inner: Queue<TraceSample, N>,
    overflow_pending: AtomicBool,
}

impl<const N: usize> Default for TraceRing<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> TraceRing<N> {
    pub const fn new() -> Self {
        Self {
            inner: Queue::new(),
            overflow_pending: AtomicBool::new(false),
        }
    }

    /// Producer side: emit one sample. On full → set `overflow_pending`, drop sample.
    /// On success → OR the pending overflow into the sample's flags before enqueue,
    /// and clear the pending bit.
    #[inline]
    pub fn try_emit(&mut self, mut s: TraceSample) -> Result<(), TraceSample> {
        if self.overflow_pending.load(Ordering::Relaxed) {
            s.flags |= TRACE_FLAG_OVERFLOW;
        }
        match self.inner.enqueue(s) {
            Ok(()) => {
                // Successful enqueue clears the pending overflow.
                self.overflow_pending.store(false, Ordering::Relaxed);
                Ok(())
            }
            Err(rejected) => {
                self.overflow_pending.store(true, Ordering::Relaxed);
                Err(rejected)
            }
        }
    }

    /// Consumer side: drain up to `out.len()` samples in FIFO order.
    /// Returns the count drained.
    pub fn drain_into(&mut self, out: &mut [TraceSample]) -> usize {
        let mut count = 0;
        while count < out.len() {
            let Some(sample) = self.inner.dequeue() else {
                break;
            };
            // Bounded by `count < out.len()`.
            if let Some(slot) = out.get_mut(count) {
                *slot = sample;
            }
            count += 1;
        }
        count
    }

    /// Foreground reads this to know whether to emit a synthetic overflow marker
    /// when drain returned empty (see spec §4.3).
    pub fn has_pending_overflow(&self) -> bool {
        self.overflow_pending.load(Ordering::Relaxed)
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]
mod tests {
    use super::*;
    use core::mem::{align_of, offset_of, size_of};

    #[test]
    fn trace_sample_layout() {
        // Spec §13.2 — these offsets are mirrored in the C smoke build's
        // _Static_assert. Any drift here breaks the C consumer.
        assert_eq!(size_of::<TraceSample>(), 32);
        assert_eq!(align_of::<TraceSample>(), 8);
        assert_eq!(offset_of!(TraceSample, tick), 0);
        assert_eq!(offset_of!(TraceSample, motor_a), 8);
        assert_eq!(offset_of!(TraceSample, motor_b), 12);
        assert_eq!(offset_of!(TraceSample, motor_e), 16);
        assert_eq!(offset_of!(TraceSample, segment_id), 20);
        assert_eq!(offset_of!(TraceSample, curve_handle), 24);
        assert_eq!(offset_of!(TraceSample, flags), 28);
    }

    fn sample(tick: u64, segment_id: u32) -> TraceSample {
        TraceSample {
            tick,
            motor_a: 0.0,
            motor_b: 0.0,
            motor_e: 0.0,
            segment_id,
            curve_handle: CurveHandle::new(0, 0),
            flags: 0,
            _pad: [0; 3],
        }
    }

    #[test]
    fn drain_pulls_in_order() {
        let mut ring = TraceRing::<16>::new();
        for i in 0..5 {
            assert!(ring.try_emit(sample(i, 0)).is_ok());
        }
        let mut out = [TraceSample::default(); 8];
        let n = ring.drain_into(&mut out);
        assert_eq!(n, 5);
        for i in 0..5 {
            assert_eq!(out[i].tick, i as u64);
        }
    }

    #[test]
    fn overflow_carries_into_next_sample() {
        let mut ring = TraceRing::<4>::new(); // effective capacity 3
        // Fill to capacity.
        for i in 0..3 {
            assert!(ring.try_emit(sample(i, 0)).is_ok());
        }
        // 4th emit fails; sets pending overflow flag.
        let r = ring.try_emit(sample(99, 0));
        assert!(r.is_err());
        assert!(ring.has_pending_overflow());

        // Drain everything to free space.
        let mut out = [TraceSample::default(); 8];
        let n = ring.drain_into(&mut out);
        assert_eq!(n, 3);

        // Pending overflow STILL set (drain doesn't clear it).
        assert!(ring.has_pending_overflow());

        // Next successful emit picks up the OVERFLOW flag.
        assert!(ring.try_emit(sample(100, 0)).is_ok());
        let n = ring.drain_into(&mut out);
        assert_eq!(n, 1);
        assert_eq!(out[0].tick, 100);
        assert_ne!(
            out[0].flags & TRACE_FLAG_OVERFLOW,
            0,
            "OVERFLOW must propagate into the next successful sample"
        );

        // After successful enqueue, pending bit cleared.
        assert!(!ring.has_pending_overflow());
    }
}
