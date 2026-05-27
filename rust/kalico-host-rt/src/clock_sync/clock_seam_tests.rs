use super::*;
use crate::clock::MockClock;
use std::time::Duration;

#[test]
fn last_sample_age_uses_injected_clock() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(72_000_000.0, clock.clone());
    est.add_piggyback_sample_at_now(0);
    clock.advance(Duration::from_secs(5));
    let age = est.last_sample_age().expect("sample present");
    assert_eq!(age, Duration::from_secs(5));
}

#[test]
fn last_dedicated_sample_age_uses_injected_clock() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(72_000_000.0, clock.clone());
    let t0 = clock.now();
    est.add_dedicated_sample(t0, t0 + Duration::from_millis(2), 1_000_000);
    clock.advance(Duration::from_secs(10));
    let age = est
        .last_dedicated_sample_age()
        .expect("dedicated sample present");
    // Sample recorded at clock.now() at end of add_dedicated_sample,
    // i.e. before our 10s advance. Age should be ~10s.
    assert_eq!(age, Duration::from_secs(10));
}
