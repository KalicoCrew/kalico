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
    assert_eq!(age, Duration::from_secs(10));
}

#[test]
fn wall_time_at_mcu_returns_none_with_zero_samples() {
    let est = ClockSyncEstimator::new(100_000_000.0);
    assert!(est.wall_time_at_mcu(0).is_none());
    assert!(est.wall_time_at_mcu(100_000_000).is_none());
}

#[test]
fn wall_time_at_mcu_inside_window_returns_some_false() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

    for i in 0..30u64 {
        clock.advance(Duration::from_secs(1));
        let mcu_clock = (i + 1) * 100_000_000;
        est.add_piggyback_sample_at_now(mcu_clock);
    }

    let result = est.wall_time_at_mcu(15 * 100_000_000);
    assert!(result.is_some(), "expected Some after 30 samples");
    let (dt, estimated) = result.unwrap();

    assert!(
        !estimated,
        "tick inside window must not be estimated; got estimated={estimated}"
    );

    let now_dt = time::OffsetDateTime::now_utc();
    let diff_secs = (now_dt - dt).whole_seconds().abs();
    assert!(
        diff_secs < 60,
        "returned time {dt} is {diff_secs}s from now_utc {now_dt}",
    );
}

#[test]
fn wall_time_at_mcu_extrapolate_returns_some_true() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

    for i in 0..30u64 {
        clock.advance(Duration::from_secs(1));
        est.add_piggyback_sample_at_now((i + 1) * 100_000_000);
    }

    let far_future = 90 * 100_000_000u64;
    let result = est.wall_time_at_mcu(far_future);
    assert!(result.is_some(), "expected Some after 30 samples");
    let (_dt, estimated) = result.unwrap();
    assert!(
        estimated,
        "extrapolation outside window must set estimated=true",
    );
}

#[test]
fn wall_time_at_mcu_one_sample_returns_some_estimated() {
    let clock = MockClock::new();
    clock.advance(Duration::from_secs(1));
    let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

    let sample_mcu_tick: u64 = 100_000_000;
    est.add_piggyback_sample_at_now(sample_mcu_tick);

    let query_tick: u64 = 200_000_000;
    let result = est.wall_time_at_mcu(query_tick);

    assert!(result.is_some(), "expected Some with one sample, got None");
    let (dt, estimated) = result.unwrap();

    assert!(
        estimated,
        "one-sample path must set estimated=true for a tick != the sample"
    );

    let unix_epoch = time::OffsetDateTime::UNIX_EPOCH;
    assert_ne!(
        dt, unix_epoch,
        "returned time must not be the UNIX epoch; got {dt}"
    );

    let now_dt = time::OffsetDateTime::now_utc();
    let diff_secs = (now_dt - dt).whole_seconds().abs();
    assert!(
        diff_secs < 60,
        "one-sample result {dt} is {diff_secs}s from now_utc {now_dt} — expected < 60 s"
    );
}
