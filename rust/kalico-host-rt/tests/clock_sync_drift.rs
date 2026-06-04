use std::time::Duration;

use kalico_host_rt::clock::{Clock, MockClock};
use kalico_host_rt::clock_sync::{ClockSyncEstimator, MAX_DRIFT_PPM_DEFAULT};

#[test]
fn drift_50ppm_stays_bounded_over_24_virtual_hours() {
    let clock = MockClock::new();
    let initial_freq: f64 = 72_000_000.0;
    let mut est = ClockSyncEstimator::new_with_clock(initial_freq, clock.clone());

    let drift_ppm = 50.0_f64;
    let mcu_freq_actual = initial_freq * (1.0 + drift_ppm / 1e6);

    let total_secs: u64 = 24 * 60 * 60;
    let mut mcu_clock_actual: f64 = 0.0;

    for _ in 0..total_secs {
        clock.advance(Duration::from_secs(1));
        mcu_clock_actual += mcu_freq_actual;
        est.add_piggyback_sample_at_now(mcu_clock_actual as u64);
    }

    let drift = est.drift_ppm(initial_freq);
    assert!(
        (drift - drift_ppm).abs() < 5.0,
        "drift estimate {drift} should be near 50 ppm (within 5 ppm)"
    );
    assert!(
        drift.abs() <= MAX_DRIFT_PPM_DEFAULT,
        "drift {drift} exceeds cap {MAX_DRIFT_PPM_DEFAULT}"
    );
}

#[test]
fn last_sample_age_uses_mock_clock() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(72_000_000.0, clock.clone());
    est.add_piggyback_sample_at_now(0);
    clock.advance(Duration::from_secs(60));
    let age = est.last_sample_age().expect("sample present");
    assert_eq!(age, Duration::from_secs(60));
}

#[test]
fn last_dedicated_sample_age_uses_mock_clock() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(72_000_000.0, clock.clone());
    let t0 = clock.now();
    est.add_dedicated_sample(t0, t0 + Duration::from_millis(2), 1_000_000);
    clock.advance(Duration::from_secs(10));
    let age = est.last_dedicated_sample_age().expect("dedicated present");
    assert_eq!(age, Duration::from_secs(10));
}

#[test]
fn request_id_is_strictly_monotonic() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(72_000_000.0, clock.clone());
    let mut prev = est.next_clock_sync_request_id();
    for _ in 0..1000 {
        let next = est.next_clock_sync_request_id();
        assert!(
            next > prev || (prev == u32::MAX && next == 0),
            "request_id non-monotonic: {prev} → {next}"
        );
        prev = next;
    }
}
