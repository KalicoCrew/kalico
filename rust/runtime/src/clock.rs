//! CYCCNT widening + cycle helpers. Spec §4.1 / §4.2 step 8.
//!
//! `WidenState` is single-producer ISR-only — wrap-handling is testable on host
//! by manually feeding raw values. The real ISR uses a `static mut` instance
//! gated by the SAFETY invariant "only the kalico ISR touches it" (§4.7).
//!
//! `publish_widened_now` / `read_widened_now` are the §11.4 widened-clock
//! seqlock: ARMv7-M lacks a lock-free `AtomicU64`, so a 2 × `AtomicU32` +
//! sequence-counter pattern lets the foreground reader pull the most recent
//! widened `now: u64` published by the ISR with bounded retry.

// `AtomicU32` from `portable_atomic` so that `TickCounter::increment`'s
// `fetch_add` compiles on ARMv6-M (thumbv6m / STM32G0), which has no native
// LDREX/STREX. On thumbv7em the codegen is identical to `core::sync::atomic`.
use core::sync::atomic::Ordering;
use portable_atomic::AtomicU32;

use crate::state::SharedState;

/// TEST/SIM ONLY. Not a production source of truth — the firmware reads the
/// real per-board TIM5 cadence from `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ`
/// (H7 40 kHz default / flashed 10 kHz, F4 20 kHz) via the `runtime_sample_rate_hz`
/// extern in `state.rs`. This constant exists solely so test/sim fixtures have
/// a fixed rate; nothing compiled into the MCU image consults it.
pub const TEST_ONLY_TICK_RATE_HZ: u32 = 40_000;

#[derive(Debug, Default)]
pub struct WidenState {
    pub(crate) last_low: u32,
    pub(crate) high: u64,
}

impl WidenState {
    /// Force-set both halves of the widen state from a known u64 baseline.
    ///
    /// Used by the Linux sim host at boot and by H7/F4 `runtime_tick_enable`
    /// on DRAINED→RUNNING transitions. Both halves must be seeded together —
    /// setting only `high` leaves `last_low` orphaned, causing `widen()` to
    /// spuriously bump `high` by 2³² on the next tick if DWT has wrapped.
    /// See ledger entry for the wrap-bump root cause.
    pub fn seed(&mut self, baseline: u64) {
        self.high = baseline & !0xFFFF_FFFFu64;
        self.last_low = baseline as u32;
    }

    /// Back-compat shim — forwards to `seed`. Existing call sites unchanged.
    #[inline]
    pub fn seed_high(&mut self, baseline: u64) {
        self.seed(baseline);
    }

    pub fn reinit(&mut self, raw: u32, last_widened_now: u64) {
        let captured_low = last_widened_now as u32;
        self.high = last_widened_now & !0xFFFF_FFFFu64;
        if raw < captured_low {
            // At least one wrap since capture. Bump conservatively.
            self.high = self.high.wrapping_add(1u64 << 32);
        }
        self.last_low = raw;
    }

    /// Widen a raw CYCCNT u32 to u64. Caller must invoke at least once per
    /// half-wrap (~3.9 s at 550 MHz) for correctness.
    #[inline]
    pub fn widen(&mut self, raw: u32) -> u64 {
        if raw < self.last_low {
            self.high = self.high.wrapping_add(1u64 << 32);
        }
        self.last_low = raw;
        self.high | u64::from(raw)
    }
}

/// TEST/SIM ONLY. How many CPU cycles make up one `TEST_ONLY_TICK_RATE_HZ`
/// tick at the given clock frequency. Production code derives the tick period
/// from the real per-board sample rate (see `TEST_ONLY_TICK_RATE_HZ`).
///
/// Integer division is intentional here: we want a whole-cycle count, and
/// `TEST_ONLY_TICK_RATE_HZ` (40 000) divides evenly into all supported STM32
/// clock frequencies (multiples of 1 MHz). The truncation is by design.
#[allow(clippy::integer_division)]
#[inline]
pub fn one_tick_cycles(clock_freq: u32) -> u32 {
    clock_freq / TEST_ONLY_TICK_RATE_HZ
}

#[inline]
pub fn min_segment_cycles(clock_freq: u32) -> u32 {
    2 * one_tick_cycles(clock_freq)
}

/// Shared liveness counter — set once by ISR, read by foreground.
///
/// Spec §4.7: u32 chosen over u64 because ARMv7-M lock-free `AtomicU64` is not
/// guaranteed; foreground uses "did the value change?" semantics so wrap (every
/// ~28 hours at 40 kHz) is benign.
#[derive(Debug)]
pub struct TickCounter {
    inner: AtomicU32,
}

impl Default for TickCounter {
    fn default() -> Self {
        Self::new()
    }
}

impl TickCounter {
    pub const fn new() -> Self {
        Self {
            inner: AtomicU32::new(0),
        }
    }

    #[inline]
    pub fn increment(&self) {
        self.inner.fetch_add(1, Ordering::Relaxed);
    }

    #[inline]
    pub fn snapshot(&self) -> u32 {
        self.inner.load(Ordering::Relaxed)
    }

    /// Expose the inner atomic (used by `tick::isr_sample_tick`).
    #[inline]
    pub fn inner_atomic(&self) -> &AtomicU32 {
        &self.inner
    }
}

/// ISR writer: publish the widened u64 `now` to `SharedState` atomics.
///
/// Wait-free and ~3 instructions on Cortex-M7. The seqlock invariant: the
/// sequence counter is even when no write is in flight; the writer makes it
/// odd before storing the two halves and even again after. A reader that
/// observes a stable even sequence around its loads is guaranteed to have a
/// coherent (lo, hi) pair. Spec §11.4.
#[inline]
pub fn publish_widened_now(shared: &SharedState, now: u64) {
    let seq = shared
        .widened_now_seq
        .load(Ordering::Relaxed)
        .wrapping_add(1);
    // → odd (write in progress)
    shared.widened_now_seq.store(seq, Ordering::Release);
    shared.widened_now_lo.store(now as u32, Ordering::Release);
    shared
        .widened_now_hi
        .store((now >> 32) as u32, Ordering::Release);
    // → even (write complete)
    shared
        .widened_now_seq
        .store(seq.wrapping_add(1), Ordering::Release);
}

/// Foreground reader: bounded retry per spec §11.4 analysis.
///
/// Returns the most recently published u64. If the ISR is currently mid-
/// publish (sequence counter is odd) or another publish slipped in between
/// the two atomic-load halves, the loop spins and retries. The probability
/// of a single retry is bounded by the publish window (≤ 3 instructions on
/// Cortex-M7); two retries in a row is statistically negligible at 40 kHz.
#[inline]
pub fn read_widened_now(shared: &SharedState) -> u64 {
    loop {
        let seq_before = shared.widened_now_seq.load(Ordering::Acquire);
        if seq_before & 1 != 0 {
            // Write in progress — spin briefly and retry.
            core::hint::spin_loop();
            continue;
        }
        let lo = u64::from(shared.widened_now_lo.load(Ordering::Acquire));
        let hi = u64::from(shared.widened_now_hi.load(Ordering::Acquire));
        let seq_after = shared.widened_now_seq.load(Ordering::Acquire);
        if seq_after == seq_before {
            return (hi << 32) | lo;
        }
        // Concurrent write slipped in; retry.
    }
}

#[cfg(test)]
mod tests;
