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
fn wall_time_at_mcu_inside_window_returns_some() {
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(100_000_000.0, clock.clone());

    let t0 = clock.now();
    for i in 1..=30u64 {
        let send = t0 + Duration::from_secs(i);
        let recv = send + Duration::from_millis(1);
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

#[test]
fn wall_time_at_mcu_anchor_tick_not_estimated() {
    let clock = MockClock::new();
    let freq = 100_000_000.0_f64;
    let mut est = ClockSyncEstimator::new_with_clock(freq, clock.clone());

    let t0 = clock.now();
    let rtt = Duration::from_micros(300);
    let half_rtt_s = 0.000_150_f64;

    for i in 1u64..=2 {
        let t = i as f64;
        let send = t0 + Duration::from_secs_f64(t);
        let mcu_resp = ((t + half_rtt_s) * freq) as u64;
        est.add_dedicated_sample(send, send + rtt, mcu_resp);
        clock.advance(Duration::from_secs(1));
    }

    let anchor = est.anchor_mcu_clock;
    let (_, estimated) = est
        .wall_time_at_mcu(anchor)
        .expect("must return Some after samples");
    assert!(
        !estimated,
        "tick at exact anchor_mcu_clock should not be estimated (delta_ticks=0)"
    );
}

#[test]
fn wall_time_at_mcu_far_tick_is_estimated() {
    let clock = MockClock::new();
    let freq = 100_000_000.0_f64;
    let mut est = ClockSyncEstimator::new_with_clock(freq, clock.clone());

    let t0 = clock.now();
    let rtt = Duration::from_micros(300);
    let half_rtt_s = 0.000_150_f64;

    for i in 1u64..=2 {
        let t = i as f64;
        let send = t0 + Duration::from_secs_f64(t);
        let mcu_resp = ((t + half_rtt_s) * freq) as u64;
        est.add_dedicated_sample(send, send + rtt, mcu_resp);
        clock.advance(Duration::from_secs(1));
    }

    let anchor = est.anchor_mcu_clock;
    let far_tick = anchor + 200_000_000;
    let (_, estimated) = est
        .wall_time_at_mcu(far_tick)
        .expect("must return Some after samples");
    assert!(
        estimated,
        "tick 2 MCU-seconds from anchor must be estimated (delta=2.0 > 1.0)"
    );
}

struct Lcg(u64);

impl Lcg {
    fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }

    fn bounded(&mut self, modulus: u64) -> u64 {
        (self.next_u64() >> 17) % modulus
    }

    fn noise_secs(&mut self, half_us: u64) -> f64 {
        let raw = self.bounded(2 * half_us + 1);
        (raw as i64 - half_us as i64) as f64 * 1e-6
    }
}

#[test]
fn stationary_clock_converges_within_5ppm() {
    let true_freq = 168_000_000.0_f64;
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(true_freq, clock.clone());
    let mut lcg = Lcg::new(0xDEAD_BEEF_CAFE_1234);

    let epoch = clock.now();
    let rtt_us = 300u64;
    let half_rtt = rtt_us as f64 * 1e-6 / 2.0;

    for i in 1u64..=120 {
        let t_secs = i as f64 * 0.5 + lcg.noise_secs(5);
        let t_secs = t_secs.max(0.001);

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

#[test]
fn high_side_outlier_within_window_is_dropped() {
    let true_freq = 168_000_000.0_f64;
    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(true_freq, clock.clone());

    let epoch = clock.now();
    let rtt_dur = Duration::from_micros(300);
    let half_rtt = 0.000_150;

    for i in 1u64..=60 {
        let t = i as f64 * 0.5;
        let send = epoch + Duration::from_secs_f64(t);
        let mcu_resp = ((t + half_rtt) * true_freq) as u64;
        est.add_dedicated_sample(send, send + rtt_dur, mcu_resp);
        clock.advance(Duration::from_millis(500));
    }

    let probe_secs = 70.0_f64;
    let before = est.mcu_time_at_host(probe_secs);

    let outlier_t = 30.5_f64;
    let send = epoch + Duration::from_secs_f64(outlier_t);
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

#[test]
fn genuine_freq_step_tracked_within_decay_constant() {
    let freq_before = 100_000_000.0_f64;
    let freq_after = 100_001_800.0_f64; // +18ppm

    let clock = MockClock::new();
    let mut est = ClockSyncEstimator::new_with_clock(freq_before, clock.clone());

    let epoch = clock.now();
    let rtt_dur = Duration::from_micros(300);
    let half_rtt = 0.000_150;

    let mut mcu_clock_acc = 0.0_f64;

    for i in 0u64..60 {
        let t = i as f64 * 0.5;
        mcu_clock_acc += 0.5 * freq_before;
        let send = epoch + Duration::from_secs_f64(t);
        let mcu_resp = (mcu_clock_acc + half_rtt * freq_before) as u64;
        est.add_dedicated_sample(send, send + rtt_dur, mcu_resp);
        clock.advance(Duration::from_millis(500));
    }

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
