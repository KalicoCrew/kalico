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
///
/// Reduced from 1201 to 1199 to give the `RuntimeContext` BSS cell 80 bytes
/// of slack after the `Engine` `CoupledToXy` fields (`prev_x/prev_y/e_accumulator`/
/// `needs_xy_seed`, ~20 bytes) were added and brought it flush against `INIT_DONE`.
pub const TRACE_RING_N: usize = 1199;

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

/// Phase-stepping diagnostic sample (2026-05-18 plan Task 5). When set,
/// this sample carries a packed `(motor, mscount, i_a, i_b, wrote_spi)`
/// payload in the `motor_a` / `motor_b` slots — see `TraceSample::phase_step`
/// for the encoding and `TraceSample::as_phase_step` for the decoder.
///
/// The 40-byte `TraceSample` struct is `#[repr(C)]` and mirrored on the C
/// consumer side (`kalico_runtime.h` + the `_Static_assert` in the C build);
/// reshaping it to a tagged enum would break that ABI. The
/// `TRACE_FLAG_PHASE_STEP` flag plus a reinterpretation of the existing
/// `motor_a` / `motor_b` payload fields lets us add the new sample kind
/// without growing the wire format.
pub const TRACE_FLAG_PHASE_STEP: u8 = 1 << 5;

/// Trace sample (§13.2). `repr(C)` aligned (NOT packed) to avoid unaligned
/// `u64` access on Cortex-M7. Carries `curve_handle` so foreground reclaim
/// (`drain_and_reclaim` → `pool.confirm_retired(handle)`) can route
/// `SEGMENT_END` events back to the right pool slot per §10.4.
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TraceSample {
    pub tick: u64,                 // offset 0, 8 bytes (struct alignment 8)
    pub motor_a: f32,              // offset 8
    pub motor_b: f32,              // offset 12
    pub motor_z: f32,              // offset 16
    pub motor_e: f32,              // offset 20
    pub segment_id: u32,           // offset 24
    pub curve_handle: CurveHandle, // offset 28, 4 bytes (slot+gen)
    pub flags: u8,                 // offset 32
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: [u8; 7], // offsets 33..39 — explicit padding to 40-byte total
}

impl Default for TraceSample {
    fn default() -> Self {
        Self {
            tick: 0,
            motor_a: 0.0,
            motor_b: 0.0,
            motor_z: 0.0,
            motor_e: 0.0,
            segment_id: 0,
            curve_handle: CurveHandle::new(0, 0),
            flags: 0,
            _pad: [0; 7],
        }
    }
}

/// Decoded `(motor, mscount, i_a, i_b, wrote_spi)` payload of a
/// `TRACE_FLAG_PHASE_STEP`-flagged sample. See `TraceSample::phase_step`
/// and `TraceSample::as_phase_step`.
///
/// `tick` is the low 32 bits of the host-widened tick (the upper bits of
/// `TraceSample::tick` are zero-padded on encode and ignored on decode for
/// `PhaseStep` samples — phase-stepping diagnostics don't need 64-bit ticks
/// in a single sample stream).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhaseStepPayload {
    pub tick: u32,
    pub motor: u8,
    pub mscount: u16,
    pub i_a: i16,
    pub i_b: i16,
    pub wrote_spi: bool,
}

impl TraceSample {
    /// Build a `TRACE_FLAG_PHASE_STEP`-flagged sample carrying a packed
    /// `(motor, mscount, i_a, i_b, wrote_spi)` payload.
    ///
    /// The 40-byte `TraceSample` wire format is preserved. Payload encoding:
    /// - `tick` (u32) → `TraceSample.tick` (u64, zero-extended).
    /// - `motor_a` (f32, 4 bytes) carries `[motor, wrote_spi_u8, mscount_lo,
    ///   mscount_hi]` little-endian.
    /// - `motor_b` (f32, 4 bytes) carries `[i_a_lo, i_a_hi, i_b_lo, i_b_hi]`
    ///   little-endian (two i16's).
    /// - `motor_z` / `motor_e` / `segment_id` / `curve_handle` are zeroed.
    /// - `flags` is `TRACE_FLAG_PHASE_STEP`.
    ///
    /// Total payload size: 1 (motor) + 2 (mscount) + 2 (`i_a`) + 2 (`i_b`)
    ///   + 1 (`wrote_spi`) = 8 bytes, comfortably inside the 16 bytes available
    ///     in `motor_a` + `motor_b` — no ring-size changes needed.
    #[must_use]
    pub fn phase_step(
        tick: u32,
        motor: u8,
        mscount: u16,
        i_a: i16,
        i_b: i16,
        wrote_spi: bool,
    ) -> Self {
        let wrote_spi_u8: u8 = u8::from(wrote_spi);
        let mscount_le = mscount.to_le_bytes();
        let motor_a_bytes: [u8; 4] = [motor, wrote_spi_u8, mscount_le[0], mscount_le[1]];

        let i_a_le = i_a.to_le_bytes();
        let i_b_le = i_b.to_le_bytes();
        let motor_b_bytes: [u8; 4] = [i_a_le[0], i_a_le[1], i_b_le[0], i_b_le[1]];

        Self {
            tick: u64::from(tick),
            motor_a: f32::from_le_bytes(motor_a_bytes),
            motor_b: f32::from_le_bytes(motor_b_bytes),
            motor_z: 0.0,
            motor_e: 0.0,
            segment_id: 0,
            curve_handle: CurveHandle::new(0, 0),
            flags: TRACE_FLAG_PHASE_STEP,
            _pad: [0; 7],
        }
    }

    /// Decode a `TRACE_FLAG_PHASE_STEP`-flagged sample's packed payload.
    /// Returns `None` for samples without the flag set. Inverse of
    /// `TraceSample::phase_step`.
    #[must_use]
    pub fn as_phase_step(&self) -> Option<PhaseStepPayload> {
        if self.flags & TRACE_FLAG_PHASE_STEP == 0 {
            return None;
        }
        let motor_a_bytes = self.motor_a.to_le_bytes();
        let motor_b_bytes = self.motor_b.to_le_bytes();

        let motor = motor_a_bytes[0];
        let wrote_spi = motor_a_bytes[1] != 0;
        let mscount = u16::from_le_bytes([motor_a_bytes[2], motor_a_bytes[3]]);

        let i_a = i16::from_le_bytes([motor_b_bytes[0], motor_b_bytes[1]]);
        let i_b = i16::from_le_bytes([motor_b_bytes[2], motor_b_bytes[3]]);

        // PhaseStep samples encode tick as u32 (zero-extended into u64.tick).
        // Truncating back to u32 is the inverse.
        #[allow(clippy::cast_possible_truncation)]
        let tick = self.tick as u32;

        Some(PhaseStepPayload {
            tick,
            motor,
            mscount,
            i_a,
            i_b,
            wrote_spi,
        })
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
mod tests;
