#![allow(clippy::cast_sign_loss, clippy::cast_lossless)]

mod mock_transport;

use std::time::{Duration, Instant};

use kalico_host_rt::clock_sync::{ClockSyncEstimator, MIN_WARMUP_SAMPLES};
use kalico_host_rt::stream::{
    ArmError, MAX_CROSS_MCU_FREQ_RATIO_OFFSET, arm_all_mcus, check_cross_mcu_desync,
};
use kalico_host_rt::transport::{MessageValue, TransportError};

use mock_transport::{MockTransport, SharedMock, mp_with};

const FREQ: f64 = 550_000_000.0;
const EPOCH_OFFSET: u64 = 1_000_000_000;

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

fn make_clock_sync_response(host_send_secs: f64, rtt_us: u32) -> (u32, u32) {
    let one_way = f64::from(rtt_us) * 1e-6 / 2.0;
    let mcu_send = EPOCH_OFFSET + (host_send_secs * FREQ) as u64;
    let mcu_response = mcu_send + (one_way * FREQ) as u64;
    (mcu_response as u32, (mcu_response >> 32) as u32)
}

fn make_warm_mcu(arm_result: i32) -> (SharedMock, ClockSyncEstimator) {
    let mock = SharedMock::new();
    let mut est = ClockSyncEstimator::new(FREQ);
    warm_estimator(&mut est);
    let epoch = est.epoch();

    mock.install_responder("kalico_clock_sync_response", move |_cmd, call_time| {
        let send_secs = call_time.saturating_duration_since(epoch).as_secs_f64();
        let (lo, hi) = make_clock_sync_response(send_secs, 0);
        mp_with(&[
            ("request_id", MessageValue::U32(1)),
            ("mcu_clock_lo", MessageValue::U32(lo)),
            ("mcu_clock_hi", MessageValue::U32(hi)),
        ])
    });

    let mock_clone = mock.clone();
    std::thread::spawn(move || {
        let _ = mock_clone.wait_for_call("kalico_stream_arm_response");
        mock_clone.complete_call(
            "kalico_stream_arm_response",
            mp_with(&[
                ("result", MessageValue::I32(arm_result)),
                ("armed_t_start_lo", MessageValue::U32(0)),
                ("armed_t_start_hi", MessageValue::U32(0)),
            ]),
        );
    });

    (mock, est)
}

#[test]
fn happy_path_single_mcu() {
    let (mock, est) = make_warm_mcu(0);
    let mut mcus: Vec<(SharedMock, ClockSyncEstimator)> = vec![(mock, est)];
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
    assert!(
        io.any_sent_starting_with("runtime_clock_sync_request"),
        "must have sent runtime_clock_sync_request"
    );
    assert!(
        io.any_sent_starting_with("runtime_stream_arm"),
        "must have sent runtime_stream_arm"
    );
}

#[test]
fn happy_path_two_mcus() {
    let mut mcus: Vec<(SharedMock, ClockSyncEstimator)> = vec![make_warm_mcu(0), make_warm_mcu(0)];
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
    let mock = SharedMock::new();
    let est = ClockSyncEstimator::new(FREQ);

    mock.install_responder("kalico_clock_sync_response", |_cmd, _call_time| {
        let (lo, hi) = make_clock_sync_response(1.0, 200);
        mp_with(&[
            ("request_id", MessageValue::U32(1)),
            ("mcu_clock_lo", MessageValue::U32(lo)),
            ("mcu_clock_hi", MessageValue::U32(hi)),
        ])
    });

    let mut mcus: Vec<(SharedMock, ClockSyncEstimator)> = vec![(mock.clone(), est)];
    let failure = arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_secs(1),
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .unwrap_err();
    assert!(
        matches!(failure.error, ArmError::QualityGate { .. }),
        "expected QualityGate, got {:?}",
        failure.error
    );
    assert!(
        failure.armed_indices.is_empty(),
        "no MCU should be armed when quality gate fails"
    );
    assert!(
        !mock.any_sent_starting_with("runtime_stream_arm "),
        "must NOT issue stream_arm if quality gate fails"
    );
}

#[test]
fn mcu_rejected_aborts_with_result_code() {
    let mut mcus: Vec<(SharedMock, ClockSyncEstimator)> = vec![make_warm_mcu(-7)];
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
    let mock = SharedMock::new();
    let est = ClockSyncEstimator::new(FREQ);
    let mut mcus: Vec<(SharedMock, ClockSyncEstimator)> = vec![(mock, est)];
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
    let mock = SharedMock::new();
    let est = ClockSyncEstimator::new(FREQ);
    let mut mcus: Vec<(SharedMock, ClockSyncEstimator)> = vec![(mock, est)];
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

#[test]
fn cross_mcu_desync_rejects_pair_above_threshold() {
    let freqs = [550_000_000.0, 550_000_000.0 * 1.005];
    let (i, j, offset) =
        check_cross_mcu_desync(&freqs).expect("0.5% divergence must trip the 1e-3 gate");
    assert_eq!(i, 0);
    assert_eq!(j, 1);
    assert!(
        offset > MAX_CROSS_MCU_FREQ_RATIO_OFFSET,
        "{offset} must exceed {MAX_CROSS_MCU_FREQ_RATIO_OFFSET}"
    );
}

#[test]
fn cross_mcu_desync_passes_within_threshold() {
    let freqs = [550_000_000.0, 550_000_000.0 * 1.0005];
    assert!(
        check_cross_mcu_desync(&freqs).is_none(),
        "tight pair must not trip cross-MCU gate"
    );
}

#[test]
fn cross_mcu_desync_handles_three_or_more_mcus() {
    let freqs = [550_000_000.0, 550_000_000.0 * 1.0001, 550_000_000.0 * 1.005];
    let (i, j, _offset) = check_cross_mcu_desync(&freqs)
        .expect("third MCU diverges → at least one pair trips the gate");
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
    let mut mcus: Vec<(SharedMock, ClockSyncEstimator)> =
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

#[test]
fn request_id_is_monotonic_across_arm_attempts() {
    let mock = SharedMock::new();
    let mut est = ClockSyncEstimator::new(FREQ);
    warm_estimator(&mut est);
    let epoch = est.epoch();

    mock.install_responder("kalico_clock_sync_response", move |cmd, call_time| {
        let request_id: u32 = cmd
            .split_whitespace()
            .find_map(|tok| tok.strip_prefix("request_id="))
            .expect("request_id= field present")
            .parse()
            .expect("request_id parses as u32");
        let send_secs = call_time.saturating_duration_since(epoch).as_secs_f64();
        let (lo, hi) = make_clock_sync_response(send_secs, 0);
        mp_with(&[
            ("request_id", MessageValue::U32(request_id)),
            ("mcu_clock_lo", MessageValue::U32(lo)),
            ("mcu_clock_hi", MessageValue::U32(hi)),
        ])
    });

    mock.install_responder("kalico_stream_arm_response", |_cmd, _call_time| {
        mp_with(&[
            ("result", MessageValue::I32(0)),
            ("armed_t_start_lo", MessageValue::U32(0)),
            ("armed_t_start_hi", MessageValue::U32(0)),
        ])
    });

    let mut mcus: Vec<(SharedMock, ClockSyncEstimator)> = vec![(mock.clone(), est)];

    arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_millis(500),
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .expect("first arm should succeed");
    arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_millis(500),
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .expect("second arm should succeed");

    let request_ids: Vec<u32> = mock
        .sent_starting_with("runtime_clock_sync_request")
        .iter()
        .map(|cmd| {
            cmd.split_whitespace()
                .find_map(|tok| tok.strip_prefix("request_id="))
                .expect("request_id= present")
                .parse::<u32>()
                .expect("u32")
        })
        .collect();

    assert_eq!(
        request_ids.len(),
        2,
        "two arm attempts → two clock_sync_requests"
    );
    assert!(
        request_ids[1] > request_ids[0],
        "request_id must be monotonic across arm attempts; got {:?}",
        request_ids
    );
    assert_ne!(
        request_ids[1], 1,
        "second arm must not restart at 1 (regression for Codex finding)"
    );
}

#[test]
fn arm_fails_on_request_id_mismatch() {
    let mock = SharedMock::new();
    let est = ClockSyncEstimator::new(FREQ);

    mock.install_responder("kalico_clock_sync_response", |_cmd, _call_time| {
        let (lo, hi) = make_clock_sync_response(1.0, 0);
        mp_with(&[
            ("request_id", MessageValue::U32(99)),
            ("mcu_clock_lo", MessageValue::U32(lo)),
            ("mcu_clock_hi", MessageValue::U32(hi)),
        ])
    });

    let mut mcus = vec![(mock.clone(), est)];
    let failure = arm_all_mcus(
        &mut mcus,
        Instant::now() + Duration::from_secs(1),
        Duration::from_millis(200),
        50_000,
        FREQ,
    )
    .unwrap_err();

    match failure.error {
        ArmError::Transport(TransportError::Parse(ref msg)) => {
            assert!(
                msg.contains("request_id mismatch"),
                "expected 'request_id mismatch' in error, got: {}",
                msg
            );
        }
        ref other => panic!(
            "expected Transport(Parse(request_id mismatch)), got {:?}",
            other
        ),
    }
    assert!(
        failure.armed_indices.is_empty(),
        "no MCU should be armed on request_id mismatch"
    );
    assert!(
        !mock.any_sent_starting_with("runtime_stream_arm "),
        "runtime_stream_arm must not be sent on request_id mismatch"
    );
}
