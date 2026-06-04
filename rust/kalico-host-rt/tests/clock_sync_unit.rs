#![allow(clippy::cast_sign_loss, clippy::cast_lossless, clippy::float_cmp)]

use std::time::{Duration, Instant};

use kalico_host_rt::clock_sync::{ClockSyncEstimator, MAX_RTT_AGE_MS_DEFAULT, MIN_WARMUP_SAMPLES};

#[test]
fn fresh_estimator_quality_gate_fails_under_warmup() {
    let est = ClockSyncEstimator::new(550_000_000.0);
    assert!(est.is_quality_gate_passed(550_000_000.0).is_err());
    assert_eq!(est.sample_count(), 0);
}

#[test]
fn quality_gate_requires_recent_dedicated_sample_per_plan_decision_b() {
    let freq = 550_000_000.0_f64; // 550 MHz baseline
    let mut est = ClockSyncEstimator::new(freq);
    let epoch_offset_mcu: u64 = 1_000_000_000;

    let t0 = Instant::now();
    let n = MIN_WARMUP_SAMPLES + 5;
    for i in 0..n {
        let host_t = t0 + Duration::from_millis(u64::from(i) * 10);
        let host_secs = (i as f64) * 0.010;
        let mcu = epoch_offset_mcu + (host_secs * freq) as u64;
        est.add_piggyback_sample(host_t, mcu);
    }
    assert!(
        est.is_quality_gate_passed(freq).is_err(),
        "must fail without RTT-aware sample (Plan-decision B)"
    );

    let host_send_secs = 0.500_f64;
    let host_send = t0 + Duration::from_millis(500);
    let one_way_secs = 0.000_250_f64; // 500 µs RTT / 2
    let host_recv = host_send + Duration::from_micros(500);
    let mcu_at_send_target = epoch_offset_mcu + (host_send_secs * freq) as u64;
    let mcu_at_response = mcu_at_send_target + (one_way_secs * freq) as u64;
    est.add_dedicated_sample(host_send, host_recv, mcu_at_response);

    assert!(
        est.is_quality_gate_passed(freq).is_ok(),
        "should pass with fresh dedicated sample on regression line; \
         residual_max={} drift_ppm={} samples={}",
        est.residual_max_in_window,
        est.drift_ppm(freq),
        est.sample_count(),
    );
}

#[test]
fn regression_recovers_freq_from_clean_samples() {
    let freq = 600_000_000.0_f64;
    let mut est = ClockSyncEstimator::new(freq * 0.99);

    let t0 = Instant::now();
    for i in 0..MIN_WARMUP_SAMPLES {
        let host_t = t0 + Duration::from_millis(u64::from(i) * 10);
        let host_secs = (i as f64) * 0.010;
        let mcu = (host_secs * freq) as u64 + 12_345;
        est.add_piggyback_sample(host_t, mcu);
    }
    let recovered = est.clock_freq_estimate;
    let drift_ppm = ((recovered - freq) / freq).abs() * 1e6;
    assert!(
        drift_ppm < 1.0,
        "recovered freq {recovered} drifts {drift_ppm} ppm from baseline {freq}"
    );
}

#[test]
fn mcu_time_at_host_uses_anchor_not_zero_offset() {
    let freq = 550_000_000.0_f64;
    let mut est = ClockSyncEstimator::new(freq);
    let t0 = Instant::now();
    let big_offset: u64 = 9_876_543_210;
    let mut sample_xs = Vec::with_capacity(MIN_WARMUP_SAMPLES as usize);
    for i in 0..MIN_WARMUP_SAMPLES {
        let host_t = t0 + Duration::from_millis(u64::from(i) * 10);
        let host_secs = est.host_time_at(host_t);
        sample_xs.push(host_secs);
        let mcu = big_offset + (host_secs * freq) as u64;
        est.add_piggyback_sample(host_t, mcu);
    }
    let target = t0 + Duration::from_millis(150);
    let target_secs = est.host_time_at(target);
    let predicted = est.mcu_time_at_host(target_secs);
    let expected = big_offset + (target_secs * freq) as u64;
    let diff = predicted.abs_diff(expected);
    assert!(
        diff < 1_000,
        "predicted={predicted} expected={expected} diff={diff} (big_offset must \
         not be discarded)"
    );
}

#[test]
fn dedicated_sample_age_check_fails_when_stale() {
    let freq = 550_000_000.0_f64;
    let mut est = ClockSyncEstimator::new(freq);
    let t0 = Instant::now();
    for i in 0..MIN_WARMUP_SAMPLES {
        let host_t = t0 + Duration::from_millis(u64::from(i) * 10);
        let host_secs = (i as f64) * 0.010;
        let mcu = (host_secs * freq) as u64;
        est.add_piggyback_sample(host_t, mcu);
    }
    assert!(est.last_dedicated_sample_age().is_none());
    assert!(est.is_quality_gate_passed(freq).is_err());
    assert_eq!(MAX_RTT_AGE_MS_DEFAULT, 500);
}

#[test]
fn drift_ppm_zero_for_baseline_match() {
    let mut est = ClockSyncEstimator::new(100_000_000.0);
    assert_eq!(est.clock_freq_estimate, 100_000_000.0);
    assert!(est.drift_ppm(100_000_000.0).abs() < 1e-9);
    est.add_piggyback_sample(Instant::now(), 0);
    assert_eq!(est.clock_freq_estimate, 100_000_000.0);
}
