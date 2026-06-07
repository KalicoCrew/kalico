use super::*;

#[test]
fn seed_aligns_last_low_so_first_widen_does_not_spuriously_wrap() {
    let mut state = WidenState::default();

    state.reinit(0x4000_0000, 0x0000_0000_4000_0000);
    assert_eq!(state.last_low, 0x4000_0000);

    let baseline: u64 = 0x0000_0003_1000_0000;
    state.seed_high(baseline);

    assert_eq!(
        state.last_low, 0x1000_0000,
        "seed must align last_low with baseline.low to prevent spurious wrap detection on next widen"
    );

    let widened = state.widen(0x1000_0010);
    assert_eq!(
        widened, 0x0000_0003_1000_0010,
        "first widen after seed must NOT bump high; should return seeded_high | fresh_raw"
    );
}

#[test]
fn no_wrap_returns_raw_extended() {
    let mut state = WidenState::default();
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
    assert_eq!(one_tick_cycles(520_000_000), 13_000);
    assert_eq!(one_tick_cycles(550_000_000), 13_750);
    assert_eq!(one_tick_cycles(480_000_000), 12_000);
}

#[test]
fn min_segment_cycles_is_two_ticks() {
    assert_eq!(min_segment_cycles(520_000_000), 26_000);
    assert_eq!(min_segment_cycles(550_000_000), 27_500);
}
