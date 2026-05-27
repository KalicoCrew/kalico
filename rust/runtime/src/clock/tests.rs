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
