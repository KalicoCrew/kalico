#![allow(clippy::integer_division)]

use crate::clock::TEST_ONLY_TICK_RATE_HZ;
use crate::engine::Engine;

const CLOCK_FREQ: u32 = 520_000_000;

#[test]
fn engine_new_has_correct_sample_period() {
    let engine = Engine::new(CLOCK_FREQ, TEST_ONLY_TICK_RATE_HZ);
    let expected_cycles = (CLOCK_FREQ + TEST_ONLY_TICK_RATE_HZ / 2) / TEST_ONLY_TICK_RATE_HZ;
    assert_eq!(
        engine.sample_period_cycles, expected_cycles,
        "sample_period_cycles must equal round(clock_freq / sample_rate); \
         got {}, expected {expected_cycles}",
        engine.sample_period_cycles
    );
    assert!(
        engine.sample_period_cycles > 0,
        "sample_period_cycles must be > 0 (a zero value disables the fault-tolerance guard)"
    );
}

#[test]
fn engine_new_starts_idle() {
    let engine = Engine::new(CLOCK_FREQ, TEST_ONLY_TICK_RATE_HZ);
    assert_eq!(
        engine.num_axes, 0,
        "freshly-constructed engine must have 0 axes"
    );
    let rc = engine.retired_counts();
    assert!(
        rc.iter().all(|&c| c == 0),
        "all retired_counts must be 0 at startup; got {rc:?}"
    );
}
