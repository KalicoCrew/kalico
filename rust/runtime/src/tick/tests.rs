#![allow(clippy::integer_division)]

use crate::clock::TEST_ONLY_TICK_RATE_HZ;
use crate::engine::Engine;

const CLOCK_FREQ: u32 = 520_000_000;

/// `Engine::new` with the standard test clock rate must produce an engine
/// with the correct `sample_period_cycles`:
///   `cycles = round(clock_freq / sample_rate)`.
///
/// Guards against the sample-period computation being zeroed or
/// misconfigured, which would silently disable the fault-tolerance check
/// in `get_position_and_velocity` (the `> 2 * sample_period_cycles` guard
/// degenerates to `> 0` when `sample_period_cycles == 0`).
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

/// An `Engine` constructed with `new` and no pieces configured must have
/// `num_axes == 0` and all retired_counts equal to 0.
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
