//! Phase 8 Task 8.2 unit tests: `stream::arm_all_mcus` against
//! `MockTransport`.
//!
//! Covers:
//! * happy path (single MCU + dual MCU)
//! * deadline-miss aborts
//! * quality-gate failure aborts
//! * `kalico_stream_arm_response.result != 0` aborts

#![allow(clippy::cast_sign_loss, clippy::cast_lossless)]

mod mock_transport;

use std::time::{Duration, Instant};

use kalico_host_rt::clock_sync::{ClockSyncEstimator, MIN_WARMUP_SAMPLES};
use kalico_host_rt::stream::{
    arm_all_mcus, check_cross_mcu_desync, ArmError, MAX_CROSS_MCU_FREQ_RATIO_OFFSET,
};
use kalico_host_rt::transport::MessageValue;

use mock_transport::{mp_with, MockTransport};

const FREQ: f64 = 550_000_000.0;
const EPOCH_OFFSET: u64 = 1_000_000_000;

/// Pre-warm an estimator with `MIN_WARMUP_SAMPLES` piggyback samples
/// on the synthetic regression line `mcu = EPOCH_OFFSET + freq · t_secs`.
///
/// The estimator's `host_time_at` clamps to zero before its construction-
/// time epoch (`saturating_duration_since`), so we have to place the
/// synthetic samples FORWARD of the epoch — i.e. at small positive
/// offsets from `est.epoch`. We do this by feeding `Instant`s at
/// `Instant::now() + i*cadence`, then encoding the matching `mcu_clock`
/// from the same offsets. Subsequent dedicated samples landing in
/// `arm_all_mcus` use `Instant::now()` (same wall clock), which lies
/// inside the warmed window provided the call follows immediately —
/// any wall-clock jitter shows up as residual.
fn warm_estimator(est: &mut ClockSyncEstimator) {
    let n = MIN_WARMUP_SAMPLES;
    let cadence_ms = 1_u64;
    let now = Instant::now();
    for i in 0..n {
        let host_t = now + Duration::from_millis(u64::from(i) * cadence_ms);
        let host_secs = est.host_time_at(host_t);
        let mcu = EPOCH_OFFSET + (host_secs * FREQ) as u64;
        est.add_piggyback_sample(host_t, mcu);
    }
}

/// Construct an MCU-clock value that, when reported by the mock as
/// `mcu_clock_lo/hi` in a `kalico_clock_sync_response` AND back-
/// calculated by [`ClockSyncEstimator::add_dedicated_sample`] (which
/// subtracts half the RTT in mcu-cycles), lands on the synthetic
/// regression line at `host_send_secs` (estimator-epoch
/// coordinates).
fn make_clock_sync_response(host_send_secs: f64, rtt_us: u32) -> (u32, u32) {
    let one_way = f64::from(rtt_us) * 1e-6 / 2.0;
    let mcu_send = EPOCH_OFFSET + (host_send_secs * FREQ) as u64;
    let mcu_response = mcu_send + (one_way * FREQ) as u64;
    (mcu_response as u32, (mcu_response >> 32) as u32)
}

/// Build a fully-warmed (`mock_io`, estimator) pair whose dedicated-sync
/// mock response is encoded LAZILY at `wait_for_response` call time
/// (via [`MockTransport::install_dynamic_responder`]) so the encoded
/// `mcu_clock` lies on the regression line at the actual `host_send`
/// inside `arm_all_mcus`.
///
/// Encoding the response at queueing time (an earlier `Instant::now()`)
/// produces a residual that grows with wall-clock jitter between
/// fixture setup and the SUT — observable on dual-MCU runs where the
/// second MCU's dedicated sync is hundreds of µs after fixture
/// construction. The dynamic responder closes over the estimator's
/// epoch + the synthetic regression-line constants, so each call
/// computes `mcu_send = freq · (now - epoch) + epoch_offset` fresh.
fn make_warm_mcu(arm_result: i32) -> (MockTransport, ClockSyncEstimator) {
    let mut io = MockTransport::new();
    let mut est = ClockSyncEstimator::new(FREQ);
    warm_estimator(&mut est);
    let epoch = est.epoch();
    io.install_dynamic_responder(
        "kalico_clock_sync_response",
        Box::new(move || {
            let now_secs =
                Instant::now().saturating_duration_since(epoch).as_secs_f64();
            let (lo, hi) = make_clock_sync_response(now_secs, 0);
            mp_with(&[
                ("request_id", MessageValue::U32(1)),
                ("mcu_clock_lo", MessageValue::U32(lo)),
                ("mcu_clock_hi", MessageValue::U32(hi)),
            ])
        }),
    );
    io.enqueue_response(
        "kalico_stream_arm_response",
        mp_with(&[
            ("result", MessageValue::I32(arm_result)),
            ("armed_t_start_lo", MessageValue::U32(0)),
            ("armed_t_start_hi", MessageValue::U32(0)),
        ]),
    );
    (io, est)
}

#[test]
fn happy_path_single_mcu() {
    let pair = make_warm_mcu(0);
    let mut mcus: Vec<(MockTransport, ClockSyncEstimator)> = vec![pair];
    let t_start_wall = Instant::now() + Duration::from_millis(500);
    arm_all_mcus(
        &mut mcus,
        t_start_wall,
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .expect("arm should succeed on warm estimator + clean responses");

    let io = &mcus[0].0;
    assert!(io.sent.iter().any(|c| c.starts_with("kalico_clock_sync_request")));
    assert!(io.sent.iter().any(|c| c.starts_with("kalico_stream_arm")));
}

#[test]
fn happy_path_two_mcus() {
    let mut mcus: Vec<(MockTransport, ClockSyncEstimator)> =
        vec![make_warm_mcu(0), make_warm_mcu(0)];
    let t_start_wall = Instant::now() + Duration::from_millis(500);
    arm_all_mcus(
        &mut mcus,
        t_start_wall,
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .expect("dual-MCU arm should succeed");
}

#[test]
fn quality_gate_failure_aborts() {
    let mut io = MockTransport::new();
    // Fresh estimator with NO warmup samples → quality gate fails.
    let est = ClockSyncEstimator::new(FREQ);
    let (lo, hi) = make_clock_sync_response(1.0, 200);
    io.enqueue_response(
        "kalico_clock_sync_response",
        mp_with(&[
            ("mcu_clock_lo", MessageValue::U32(lo)),
            ("mcu_clock_hi", MessageValue::U32(hi)),
        ]),
    );
    let mut mcus: Vec<(MockTransport, ClockSyncEstimator)> = vec![(io, est)];
    let failure = arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_secs(1),
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .unwrap_err();
    assert!(
        matches!(failure.error, ArmError::QualityGate),
        "expected QualityGate, got {:?}",
        failure.error
    );
    assert!(
        failure.armed_indices.is_empty(),
        "no MCU should be armed when quality gate fails"
    );
    assert!(
        !mcus[0].0.sent.iter().any(|c| c.starts_with("kalico_stream_arm ")),
        "must NOT issue stream_arm if quality gate fails"
    );
}

#[test]
fn mcu_rejected_aborts_with_result_code() {
    let mut mcus: Vec<(MockTransport, ClockSyncEstimator)> =
        vec![make_warm_mcu(-7)];
    let failure = arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_secs(1),
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .unwrap_err();
    match failure.error {
        ArmError::McuRejected(r) => assert_eq!(r, -7),
        other => panic!("expected McuRejected, got {other:?}"),
    }
    assert!(
        failure.armed_indices.is_empty(),
        "single-MCU rejection means nothing is armed"
    );
}

#[test]
fn deadline_missed_when_arm_lead_time_too_short() {
    // arm_lead_time = 0 → deadline is now → first deadline-check fires.
    let io = MockTransport::new();
    let est = ClockSyncEstimator::new(FREQ);
    let mut mcus: Vec<(MockTransport, ClockSyncEstimator)> = vec![(io, est)];
    let failure = arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_secs(1),
        Duration::ZERO,
        0,
        FREQ,
    )
    .unwrap_err();
    assert!(
        matches!(failure.error, ArmError::DeadlineMissed),
        "expected DeadlineMissed, got {:?}",
        failure.error
    );
}

#[test]
fn transport_timeout_propagates() {
    let mut io = MockTransport::new();
    // No response queued → wait_for_response yields Timeout.
    io.force_timeout_after = Some(0);
    let est = ClockSyncEstimator::new(FREQ);
    let mut mcus: Vec<(MockTransport, ClockSyncEstimator)> = vec![(io, est)];
    let failure = arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_secs(1),
        Duration::from_millis(200),
        0,
        FREQ,
    )
    .unwrap_err();
    assert!(
        matches!(failure.error, ArmError::Transport(_)),
        "expected Transport(_), got {:?}",
        failure.error
    );
}

/// Spec §6.3 + §12.4 cross-MCU drift check (GAP-1 fix).
///
/// We test the pure-data form (`check_cross_mcu_desync`) directly: an
/// `arm_all_mcus`-level integration test for this code path is
/// effectively unreachable under spec defaults because the per-MCU
/// drift gate (`MAX_DRIFT_PPM_DEFAULT = 100 ppm`) is tighter than the
/// cross-MCU ratio gate (`MAX_CROSS_MCU_FREQ_RATIO_OFFSET = 1e-3 =
/// 1000 ppm`). Any pair of estimators whose freq divergence exceeds
/// the cross-MCU gate would fail the per-MCU gate first against any
/// sane single baseline. The cross-MCU check is therefore
/// defense-in-depth — we exercise it as a pure function and trust the
/// integration wiring through code review of `arm_all_mcus`.
#[test]
fn cross_mcu_desync_rejects_pair_above_threshold() {
    let freqs = [550_000_000.0, 550_000_000.0 * 1.005];
    let (i, j, offset) = check_cross_mcu_desync(&freqs)
        .expect("0.5% divergence must trip the 1e-3 gate");
    assert_eq!(i, 0);
    assert_eq!(j, 1);
    assert!(
        offset > MAX_CROSS_MCU_FREQ_RATIO_OFFSET,
        "{offset} must exceed {MAX_CROSS_MCU_FREQ_RATIO_OFFSET}"
    );
}

#[test]
fn cross_mcu_desync_passes_within_threshold() {
    // 0.05% divergence (5e-4) sits below the 1e-3 gate.
    let freqs = [550_000_000.0, 550_000_000.0 * 1.0005];
    assert!(
        check_cross_mcu_desync(&freqs).is_none(),
        "tight pair must not trip cross-MCU gate"
    );
}

#[test]
fn cross_mcu_desync_handles_three_or_more_mcus() {
    // First two are tight, third diverges from both.
    let freqs = [
        550_000_000.0,
        550_000_000.0 * 1.0001,
        550_000_000.0 * 1.005,
    ];
    let (i, j, _offset) = check_cross_mcu_desync(&freqs)
        .expect("third MCU diverges → at least one pair trips the gate");
    // First failing pair lexicographically — could be (0, 2) or (1, 2),
    // depending on iteration order. Our impl iterates `i < j`, so the
    // first failing pair is (0, 2).
    assert_eq!(i, 0);
    assert_eq!(j, 2);
}

#[test]
fn cross_mcu_desync_single_mcu_passes() {
    let freqs = [550_000_000.0];
    assert!(check_cross_mcu_desync(&freqs).is_none());
}

#[test]
fn cross_mcu_desync_arm_error_displays() {
    // Smoke-test the Display impl so a real fault produces a useful
    // log line.
    let err = ArmError::CrossMcuDesync {
        mcu_a: 0,
        mcu_b: 1,
        ratio_offset: 5e-3,
    };
    let s = format!("{err}");
    assert!(s.contains("MCU 0"));
    assert!(s.contains("MCU 1"));
    assert!(s.contains("0.005000"));
}

#[test]
fn partial_arm_failure_reports_armed_indices() {
    // MCU 0 arms successfully, MCU 1 rejects the arm. The failure
    // surface must record `armed_indices = [0]` so the caller can
    // flush MCU 0 back to IDLE.
    let mut mcus: Vec<(MockTransport, ClockSyncEstimator)> =
        vec![make_warm_mcu(0), make_warm_mcu(-42)];
    let failure = arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_millis(500),
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .unwrap_err();
    match failure.error {
        ArmError::McuRejected(r) => assert_eq!(r, -42),
        other => panic!("expected McuRejected, got {other:?}"),
    }
    assert_eq!(
        failure.armed_indices,
        vec![0],
        "MCU 0 was armed before MCU 1 failed; caller must flush it"
    );
}
