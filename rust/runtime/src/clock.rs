//! CYCCNT widening + cycle helpers. Spec §4.1 / §4.2 step 8.
//!
//! `WidenState` is single-producer ISR-only — wrap-handling is testable on host
//! by manually feeding raw values. The real ISR uses a `static mut` instance
//! gated by the SAFETY invariant "only the kalico ISR touches it" (§4.7).

use core::sync::atomic::{AtomicU32, Ordering};

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
    /// `kalico_h7_disable_tim5()`, and passes that u64 value back at re-enable.
    /// Reinit then preserves `high` from the captured high-water mark and
    /// adjusts forward conservatively if `raw < captured_low` (one wrap
    /// detected). Long disables that wrap multiple times are inherently
    /// unrecoverable from CYCCNT alone — but `last_widened_now` carries the
    /// pre-disable high-water across the gap, so the timeline is monotonic
    /// from the foreground's perspective even if we miss exact wrap counts.
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
}

#[cfg(test)]
mod tests {
    use super::*;

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
        // Helper is parametric over kalico_clock_freq. Sanity-check at the
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
