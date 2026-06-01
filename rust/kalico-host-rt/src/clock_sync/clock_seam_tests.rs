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

// ── wall_time_at_mcu tests ────────────────────────────────────────────────────

#[test]
fn wall_time_at_mcu_returns_none_with_zero_samples() {
    let est = ClockSyncEstimator::new(100_000_000.0);
    // No samples added — must return None regardless of the tick value.
    assert!(est.wall_time_at_mcu(0).is_none());
    assert!(est.wall_time_at_mcu(100_000_000).is_none());
}

#[test]
fn wall_time_at_mcu_inside_window_returns_some_false() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

    // Feed 30 samples at 1 s intervals so the window is full.
    // MCU ticks at 100 MHz → 100_000_000 ticks/s.
    for i in 0..30u64 {
        clock.advance(Duration::from_secs(1));
        let mcu_clock = (i + 1) * 100_000_000;
        est.add_piggyback_sample_at_now(mcu_clock);
    }

    // Query a tick inside the regression window (sample #15 → 1.5 Gs).
    let result = est.wall_time_at_mcu(15 * 100_000_000);
    assert!(result.is_some(), "expected Some after 30 samples");
    let (dt, estimated) = result.unwrap();

    // estimated=false because tick is within the window.
    assert!(!estimated, "tick inside window must not be estimated; got estimated={estimated}");

    // The returned time must be within 60 s of the real wall clock.
    // (The mock clock advances but the wall_epoch anchor comes from the
    // real SystemTime at construction — both measure real elapsed time,
    // so the offset should be small.)
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

    // Query a tick 60 s beyond the window's most recent sample
    // (window covers ticks 1..30 × 100 MHz; query is at 90 × 100 MHz).
    let far_future = 90 * 100_000_000u64;
    let result = est.wall_time_at_mcu(far_future);
    assert!(result.is_some(), "expected Some after 30 samples");
    let (_dt, estimated) = result.unwrap();
    assert!(
        estimated,
        "extrapolation outside window must set estimated=true",
    );
}

// ── single-sample degenerate case ────────────────────────────────────────────
//
// With exactly one sample `recompute_regression` returns early (the
// `samples.len() < 2` guard at clock_sync.rs:331), leaving
// `anchor_host_time = 0.0`, `anchor_mcu_clock = 0`, and
// `clock_freq_estimate` at the constructor value.
//
// `wall_time_at_mcu` must:
//   1. Return `Some` (not `None`) — there is a sample.
//   2. Set `estimated = true` for any tick other than the single sample's
//      `mcu_clock`, because `min_mcu == max_mcu` so the tick is either
//      strictly below or strictly above the one-point window.
//   3. Return a plausible wall-clock time — not the UNIX epoch and not a
//      panic.  With the initial_freq fallback the formula is
//      `host_secs = mcu_ticks / initial_freq`, so the result is
//      `wall_epoch + host_secs` which must be close to the real
//      `now_utc()` (within 60 s, matching the existing tolerance used by
//      the 30-sample tests).
#[test]
fn wall_time_at_mcu_one_sample_returns_some_estimated() {
    let clock = MockClock::new();
    // Advance slightly so the sample's `host_time_secs` is non-zero, but
    // the regression is still skipped (only one sample).
    clock.advance(Duration::from_secs(1));
    let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

    // One piggyback sample at MCU tick 100_000_000 (≈ 1 s at 100 MHz).
    let sample_mcu_tick: u64 = 100_000_000;
    est.add_piggyback_sample_at_now(sample_mcu_tick);

    // Query a tick *different* from the single sample so `estimated` is
    // forced true by the `min_mcu == max_mcu` comparison.
    let query_tick: u64 = 200_000_000;
    let result = est.wall_time_at_mcu(query_tick);

    // 1. Must be Some — one sample is enough to not return None.
    assert!(result.is_some(), "expected Some with one sample, got None");
    let (dt, estimated) = result.unwrap();

    // 2. Must be estimated — outside the degenerate single-point window.
    assert!(
        estimated,
        "one-sample path must set estimated=true for a tick != the sample"
    );

    // 3. Must not be the UNIX epoch (which would indicate anchor fallback
    //    without the initial_freq path).
    let unix_epoch = time::OffsetDateTime::UNIX_EPOCH;
    assert_ne!(
        dt, unix_epoch,
        "returned time must not be the UNIX epoch; got {dt}"
    );

    // 4. Plausibility: result should be within 60 s of now_utc, mirroring
    //    the tolerance used by the 30-sample tests.  The MockClock only
    //    advances monotonic time; wall_epoch is captured from the real
    //    SystemTime at construction, so the offset is small.
    let now_dt = time::OffsetDateTime::now_utc();
    let diff_secs = (now_dt - dt).whole_seconds().abs();
    assert!(
        diff_secs < 60,
        "one-sample result {dt} is {diff_secs}s from now_utc {now_dt} — expected < 60 s"
    );
}
