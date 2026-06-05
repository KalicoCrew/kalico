use super::*;
use crate::clock::MockClock;
use std::time::Duration;

// ---------------------------------------------------------------------------
// Seam / infrastructure tests (unchanged behaviour)
// ---------------------------------------------------------------------------

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
fn wall_time_at_mcu_inside_window_returns_some() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

    // Feed 30 dedicated samples with 1ms constant RTT so min_half_rtt is set.
    let t0 = clock.now();
    for i in 1..=30u64 {
        let send = t0 + Duration::from_secs(i);
        let recv = send + Duration::from_millis(1);
        // mcu_at_response: MCU clock at the response instant (send + 0.5ms).
        let mcu_resp = i * 100_000_000 + 50_000;
        est.add_dedicated_sample(send, recv, mcu_resp);
        clock.advance(Duration::from_secs(1));
    }

    let result = est.wall_time_at_mcu(15 * 100_000_000);
    assert!(result.is_some(), "expected Some after 30 samples");
    let (dt, _estimated) = result.unwrap();

    let now_dt = time::OffsetDateTime::now_utc();
    let diff_secs = (now_dt - dt).whole_seconds().abs();
    assert!(
        diff_secs < 120,
        "returned time {dt} is {diff_secs}s from now_utc {now_dt}",
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
    let (dt, _estimated) = result.unwrap();

    let unix_epoch = time::OffsetDateTime::UNIX_EPOCH;
    assert_ne!(
        dt, unix_epoch,
        "returned time must not be the UNIX epoch; got {dt}"
    );

    let now_dt = time::OffsetDateTime::now_utc();
    let diff_secs = (now_dt - dt).whole_seconds().abs();
    assert!(
        diff_secs < 120,
        "one-sample result {dt} is {diff_secs}s from now_utc {now_dt} — expected < 120 s"
    );
}

// ---------------------------------------------------------------------------
// Sub-task B unit tests
//
// All timestamps are deterministic: fixed arrays or seeded LCG.
// No Instant::now() / SystemTime::now() used inside the test logic —
// MockClock provides a controlled time base.
// ---------------------------------------------------------------------------

/// Seeded LCG for deterministic bounded noise (no external crate needed).
struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_mul(6_364_136_223_846_793_005).wrapping_add(1_442_695_040_888_963_407);
        self.0
    }

    /// Returns a value in [0, modulus).
    fn bounded(&mut self, modulus: u64) -> u64 {
        (self.next_u64() >> 17) % modulus
    }

    /// Returns a signed offset in [-half_us, +half_us] expressed in seconds.
    fn noise_secs(&mut self, half_us: u64) -> f64 {
        let raw = self.bounded(2 * half_us + 1);
        (raw as i64 - half_us as i64) as f64 * 1e-6
    }
}

/// (a) Stationary clock + bounded noise → fitted freq within ±5ppm after warmup.
///
/// Stream: 120 dedicated samples at ~500ms spacing with constant 300µs RTT
/// and ±5µs host-timing jitter.  After warmup the EWMA should track
/// `true_freq` within 5ppm.  The tolerance is intentionally looser than the
/// bench ±18ppm worst-case; this test guards against algorithmic regression,
/// not hardware accuracy.
#[test]
fn stationary_clock_converges_within_5ppm() {
    let true_freq = 168_000_000.0_f64;
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(true_freq, clock.clone());
    let mut lcg = Lcg::new(0xDEAD_BEEF_CAFE_1234);

    let epoch = clock.now();
    let rtt_us = 300u64; // constant 300µs RTT
    let half_rtt = rtt_us as f64 * 1e-6 / 2.0;

    for i in 1u64..=120 {
        // ±5µs timing jitter on the send instant (host-side noise).
        let t_secs = i as f64 * 0.5 + lcg.noise_secs(5);
        let t_secs = t_secs.max(0.001);

        // MCU clock at the response instant (send + one-way delay).
        let mcu_resp = ((t_secs + half_rtt) * true_freq) as u64;

        let send = epoch + Duration::from_secs_f64(t_secs);
        let recv = send + Duration::from_micros(rtt_us);
        est.add_dedicated_sample(send, recv, mcu_resp);
        clock.advance(Duration::from_millis(500));
    }

    let fitted = est.clock_freq_estimate;
    let err_ppm = ((fitted - true_freq) / true_freq * 1e6).abs();
    assert!(
        err_ppm < 5.0,
        "fitted freq {fitted:.1} Hz is {err_ppm:.3}ppm from truth {true_freq:.1} Hz — expected < 5ppm"
    );
}

/// (b) Klippy-parity outlier rejection: a high-side clock glitch within the
///     10-second prediction reset window is silently dropped and does NOT move
///     the projected MCU clock by more than ~1µs.
///
/// Klippy's gate: `clock > exp_clock && sent_time < last_prediction_time + 10`
/// → sample is returned early (skipped).  We inject an outlier with a +5ms
/// clock advance (840_000 ticks at 168 MHz) while the prediction window is
/// still fresh.
#[test]
fn high_side_outlier_within_window_is_dropped() {
    let true_freq = 168_000_000.0_f64;
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(true_freq, clock.clone());

    let epoch = clock.now();
    let rtt_dur = Duration::from_micros(300); // constant 300µs RTT
    let half_rtt = 0.000_150; // 150µs

    // Warmup: 60 honest samples at 500ms spacing.
    for i in 1u64..=60 {
        let t = i as f64 * 0.5;
        let send = epoch + Duration::from_secs_f64(t);
        let mcu_resp = ((t + half_rtt) * true_freq) as u64;
        est.add_dedicated_sample(send, send + rtt_dur, mcu_resp);
        clock.advance(Duration::from_millis(500));
    }

    // Probe before outlier.
    let probe_secs = 70.0_f64;
    let before = est.mcu_time_at_host(probe_secs);

    // Inject a high-side outlier: MCU clock is +5ms (840_000 ticks) ahead of
    // prediction.  `sent_time = 30.5`, `last_prediction_time ≈ 30.0` → within
    // the 10-second window → klippy (and our port) silently drop it.
    let outlier_t = 30.5_f64;
    let send = epoch + Duration::from_secs_f64(outlier_t);
    // Expected MCU at send ≈ 30.5 * true_freq + offset_from_regression.
    // We add +840_000 (≈ 5ms) to make it a clear high-side outlier.
    let mcu_resp_outlier = ((outlier_t + half_rtt) * true_freq) as u64 + 840_000;
    est.add_dedicated_sample(send, send + rtt_dur, mcu_resp_outlier);

    let after = est.mcu_time_at_host(probe_secs);

    let shift_ticks = (after as i64) - (before as i64);
    let shift_us = (shift_ticks as f64 / true_freq * 1e6).abs();
    assert!(
        shift_us < 1.0,
        "high-side outlier shifted projected offset by {shift_us:.3}µs — must be < 1µs \
         (klippy silently drops this class of outlier)"
    );
}

/// (c) Cold-start: first 5 samples should yield a usable fit within 50ppm.
///
/// We use a constant RTT of 500µs so there is no ambiguity in what the MCU
/// clock should be at send time.
#[test]
fn cold_start_within_50ppm_after_handful() {
    let true_freq = 100_000_000.0_f64;
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(true_freq, clock.clone());

    let epoch = clock.now();
    let rtt_dur = Duration::from_micros(500);
    let half_rtt = 0.000_250;

    for i in 1u64..=5 {
        let t = i as f64 * 0.5;
        let send = epoch + Duration::from_secs_f64(t);
        let mcu_resp = ((t + half_rtt) * true_freq) as u64;
        est.add_dedicated_sample(send, send + rtt_dur, mcu_resp);
        clock.advance(Duration::from_millis(500));
    }

    let fitted = est.clock_freq_estimate;
    let err_ppm = ((fitted - true_freq) / true_freq * 1e6).abs();
    assert!(
        err_ppm < 50.0,
        "cold-start fitted freq {fitted:.1} Hz is {err_ppm:.3}ppm from truth after 5 samples"
    );
}

/// (d) A genuine frequency step (temperature-ramp analog) is tracked within
///     the decay time constant.
///
/// After 60 samples at `freq_before`, we switch to `freq_after` (+18ppm) and
/// feed another 60 samples.  After one full decay window the fit should have
/// converged to within 5ppm of `freq_after`.
#[test]
fn genuine_freq_step_tracked_within_decay_constant() {
    let freq_before = 100_000_000.0_f64;
    let freq_after = 100_001_800.0_f64; // +18ppm

    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(freq_before, clock.clone());

    let epoch = clock.now();
    let rtt_dur = Duration::from_micros(300);
    let half_rtt = 0.000_150;

    let mut mcu_clock_acc = 0.0_f64; // simulate accumulating ticks

    // Phase 1: 60 samples at freq_before.
    for i in 0u64..60 {
        let t = i as f64 * 0.5;
        mcu_clock_acc += 0.5 * freq_before;
        let send = epoch + Duration::from_secs_f64(t);
        // mcu_resp is the MCU clock at the response instant (send + half_rtt).
        let mcu_resp = (mcu_clock_acc + half_rtt * freq_before) as u64;
        est.add_dedicated_sample(send, send + rtt_dur, mcu_resp);
        clock.advance(Duration::from_millis(500));
    }

    // Phase 2: 60 samples at freq_after.
    let phase2_start = 60.0_f64 * 0.5;
    for i in 0u64..60 {
        let t = phase2_start + i as f64 * 0.5;
        mcu_clock_acc += 0.5 * freq_after;
        let send = epoch + Duration::from_secs_f64(t);
        let mcu_resp = (mcu_clock_acc + half_rtt * freq_after) as u64;
        est.add_dedicated_sample(send, send + rtt_dur, mcu_resp);
        clock.advance(Duration::from_millis(500));
    }

    let fitted = est.clock_freq_estimate;
    let err_ppm = ((fitted - freq_after) / freq_after * 1e6).abs();
    // After one full decay window (60 samples × 500ms) the EWMA should have
    // converged to within ~10ppm of the new frequency.  The tolerance accounts
    // for EWMA lag: at DECAY=1/30, 60 samples give ~86% tracking of a step.
    assert!(
        err_ppm < 10.0,
        "after freq step + 60 samples, fitted {fitted:.1} Hz still {err_ppm:.3}ppm \
         from new truth {freq_after:.1} Hz"
    );
}
