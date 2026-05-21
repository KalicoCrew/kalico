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

use core::sync::atomic::{AtomicU32, Ordering};

use crate::state::SharedState;

pub const TICK_RATE_HZ: u32 = 40_000;

#[derive(Debug, Default)]
pub struct WidenState {
    pub(crate) last_low: u32,
    pub(crate) high: u64,
}

impl WidenState {
    /// Reinitialize widening state across a TIM5 disable→enable transition.
    ///
    /// Key insight (corrected after round-2 verifier review): we cannot
    /// reconstruct CYCCNT epoch from an external clock at u32 resolution alone
    /// (Klipper's `timer_read_time` is u32 too, with the same wrap period as
    /// CYCCNT on ARMCM where both come from `DWT->CYCCNT`). So the backstop
    /// shape is: foreground captures `engine.last_widened_now()` BEFORE
    /// `runtime_tick_disable()`, and passes that u64 value back at re-enable.
    /// Reinit then preserves `high` from the captured high-water mark and
    /// adjusts forward conservatively if `raw < captured_low` (one wrap
    /// detected). Long disables that wrap multiple times are inherently
    /// unrecoverable from CYCCNT alone — but `last_widened_now` carries the
    /// pre-disable high-water across the gap, so the timeline is monotonic
    /// from the foreground's perspective even if we miss exact wrap counts.
    /// Force-set both halves of the widen state from a known u64 baseline.
    ///
    /// Used by both the Linux sim host (at engine boot, before the pthread
    /// spins up) and the H7 / F4 `runtime_tick_enable` path (on
    /// DRAINED→RUNNING transitions, where TIM5 disable→re-enable invalidates
    /// the wrap-counting fast path).
    ///
    /// **Why both halves:** the prior version only set `self.high`, leaving
    /// `self.last_low` whatever stale value was there. The H7 reseed
    /// sequence is `reinit(raw, last_widened_now_pre_drain) → seed_high(klippy_baseline)`,
    /// so `last_low` ended up at the raw cyccnt captured in `reinit` — which
    /// has no relationship to the seeded `high`. On the next ISR tick,
    /// `widen()` compares the fresh DWT against that orphaned `last_low`; if
    /// DWT happens to have rolled past 2³² since `reinit` (or was in a
    /// different relative position than the seeded high implies), `widen`
    /// spuriously bumps `high` by 2³² ≈ 8.26 s at 520 MHz. Engine's `now`
    /// jumps ~8 s into the future, every segment's `t_start` lands in the
    /// past, and the boundary loop retires it without ever evaluating the
    /// curve — silent motion loss with `current_segment_id` advancing
    /// normally (the bench-observed "every other jog ignored after long
    /// idle" symptom, 2026-05-11).
    ///
    /// **Fix:** set `last_low` to the baseline's low 32 bits so the next
    /// `widen()` measures DWT advance from the SAME reference point the
    /// seeded `high` encodes. The seed comes from the host's view of the
    /// MCU's widened clock right before `runtime_tick_enable` returns; the
    /// next ISR fires within microseconds, so DWT will have advanced by
    /// well under one wrap — `widen` correctly detects no wrap and returns
    /// `seeded_high | fresh_raw`, which is the correct current widened
    /// clock.
    pub fn seed(&mut self, baseline: u64) {
        self.high = baseline & !0xFFFF_FFFFu64;
        self.last_low = baseline as u32;
    }

    /// Back-compat shim. Pre-fix code called `seed_high`; the new spelling
    /// is `seed` because it sets both halves. The shim forwards verbatim so
    /// existing call sites (`runtime_handle_seed_widen`) work unchanged.
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

/// How many CPU cycles make up one 40 kHz tick at the given clock frequency.
///
/// Integer division is intentional here: we want a whole-cycle count, and
/// `TICK_RATE_HZ` (40 000) divides evenly into all supported STM32 clock
/// frequencies (multiples of 1 MHz). The truncation is by design.
#[allow(clippy::integer_division)]
#[inline]
pub fn one_tick_cycles(clock_freq: u32) -> u32 {
    clock_freq / TICK_RATE_HZ
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

    /// 2026-05-21 — expose the inner atomic for the `bump_relaxed`
    /// workaround in `tick::isr_sample_tick` (see that function's comment
    /// on the fetch_add codegen symptom for full rationale).
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
mod tests {
    use super::*;

    /// Regression: `seed_high` must align `last_low` with the baseline's
    /// low 32 bits, otherwise the FIRST `widen()` after a reseed spuriously
    /// detects a wrap and inflates `high` by 2³² cycles. Bench-observed
    /// 2026-05-11: segments retired without step pulses after long idle.
    ///
    /// Scenario replayed here:
    /// 1. Engine running, `widen(raw=last_low_pre_drain)` → publishes some
    ///    `last_widened_now`. (Established by the `reinit` arm below.)
    /// 2. TIM5 disabled. DWT advances on its own; widening loop frozen.
    /// 3. Push_segment lands. `reinit(raw_post_drain, last_widened_pre)`
    ///    sets `last_low = raw_post_drain` and `high = pre_drain_high`
    ///    (possibly +2³² if `raw_post_drain < last_low_pre_drain`).
    /// 4. `seed_high(klippy_baseline)` overrides `self.high`.
    /// 5. **The bug:** `self.last_low` still equals `raw_post_drain` from
    ///    step 3, which has no relationship to the seeded `high`. If the
    ///    next `widen(new_raw)` sees `new_raw < self.last_low` → spurious
    ///    `high += 2³²`, breaking the engine's clock by ~8 s.
    ///
    /// With the fix, `seed_high` also sets `last_low = baseline.low`, so
    /// the next `widen` measures advance from the correct reference and
    /// produces the right widened result.
    #[test]
    fn seed_aligns_last_low_so_first_widen_does_not_spuriously_wrap() {
        let mut state = WidenState::default();

        // Simulate the bridge's reseed sequence on DRAINED→RUNNING after
        // a TIM5 disable.
        // Pre-drain state: engine had widened raw=0x4000_0000 to (high=0, low=0x4000_0000).
        state.reinit(0x4000_0000, 0x0000_0000_4000_0000);
        assert_eq!(state.last_low, 0x4000_0000);

        // After TIM5 disable, DWT wraps multiple times. Klippy tracks the
        // wraps and seeds widened-now = 0x0000_0003_1000_0000 (3 wraps +
        // mid-cycle position 0x1000_0000). The raw DWT happens to be at
        // 0x1000_0010 right now (just past the seeded low).
        let baseline: u64 = 0x0000_0003_1000_0000;
        state.seed_high(baseline);

        // With the fix, last_low should now be 0x1000_0000 (baseline's low).
        // Without the fix, last_low would still be 0x4000_0000 from reinit.
        assert_eq!(
            state.last_low, 0x1000_0000,
            "seed must align last_low with baseline.low to prevent spurious wrap detection on next widen"
        );

        // Simulate the next ISR tick — raw advances normally by ~13k cycles.
        let widened = state.widen(0x1000_0010);
        assert_eq!(
            widened, 0x0000_0003_1000_0010,
            "first widen after seed must NOT bump high; should return seeded_high | fresh_raw"
        );
    }

    #[test]
    fn no_wrap_returns_raw_extended() {
        let mut state = WidenState::default();
        // First call after reinit at 0:
        state.reinit(0, 0);
        let now1 = state.widen(100);
        assert_eq!(now1, 100);
        let now2 = state.widen(200);
        assert_eq!(now2, 200);
    }

    #[test]
    fn wrap_increments_high() {
        let mut state = WidenState::default();
        state.reinit(0, 0);
        let _ = state.widen(0xFFFF_FF00);
        let now_post_wrap = state.widen(0x0000_0100);
        assert_eq!(now_post_wrap, (1u64 << 32) | 0x0000_0100);
    }

    #[test]
    fn one_tick_cycles_parametric() {
        // Helper is parametric over runtime_clock_freq. Sanity-check at the
        // H723 Klipper Kconfig default (520 MHz) and a hypothetical 550 MHz.
        assert_eq!(one_tick_cycles(520_000_000), 13_000);
        assert_eq!(one_tick_cycles(550_000_000), 13_750);
        assert_eq!(one_tick_cycles(480_000_000), 12_000);
    }

    #[test]
    fn min_segment_cycles_is_two_ticks() {
        // Spec §4.4 producer rejection threshold = 2 * one_tick_cycles.
        assert_eq!(min_segment_cycles(520_000_000), 26_000);
        assert_eq!(min_segment_cycles(550_000_000), 27_500);
    }
}
