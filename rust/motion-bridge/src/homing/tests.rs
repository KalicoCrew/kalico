use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use runtime::piece_ring::PieceEntry;

use kalico_host_rt::passthrough_queue::PassthroughRouter;

use crate::dispatch::{AXIS_X, AXIS_Z, McuAxisConfig, McuCaps};
use crate::homing::{eval_bernstein_cubic, reconstruct_axis_position, trajectory_final_position};
use crate::pump::AxisKey;

fn stub_configs(mcu_id: u32, axes: Vec<usize>) -> Vec<McuAxisConfig> {
    vec![McuAxisConfig {
        mcu_id,
        axes,
        kinematics: 1,
        caps: McuCaps {
            total_piece_memory: 4 * 1024,
        },
    }]
}

fn make_linear_piece(
    start_time: u64,
    duration_secs: f32,
    pos_start: f32,
    pos_end: f32,
) -> PieceEntry {
    PieceEntry {
        start_time,
        coeffs: [pos_start, pos_start, pos_end, pos_end],
        duration: duration_secs,
        _reserved: 0,
    }
}

fn router_with_clock(mcu_id: u32, freq: f64) -> Arc<Mutex<PassthroughRouter>> {
    let clock: Arc<dyn kalico_host_rt::clock::Clock + Send + Sync> =
        Arc::new(kalico_host_rt::clock::RealClock);
    let mut router = PassthroughRouter::with_clock(clock);
    for i in 0..mcu_id {
        let _ = router.claim_mcu(&format!("dummy-{i}"));
    }
    let handle = router.claim_mcu(&format!("mcu-{mcu_id}"));
    assert_eq!(
        handle.raw(),
        mcu_id,
        "handle must equal mcu_id for test correctness"
    );
    let _ =
        router.set_clock_est_from_sample(handle, freq, std::time::Instant::now(), 1_000_000_000);
    Arc::new(Mutex::new(router))
}

#[test]
fn eval_bernstein_cubic_linear_piece_endpoints() {
    let coeffs = [0.0f32, 0.0, 1.0, 1.0];
    let at_start = eval_bernstein_cubic(coeffs, 0.0);
    let at_end = eval_bernstein_cubic(coeffs, 1.0);
    assert!(
        at_start.abs() < 1e-6,
        "u=0 should give pos_start=0, got {at_start}"
    );
    assert!(
        (at_end - 1.0).abs() < 1e-6,
        "u=1 should give pos_end=1, got {at_end}"
    );
}

#[test]
fn eval_bernstein_cubic_midpoint_linear() {
    let coeffs = [0.0f32, 0.0, 100.0, 100.0];
    let at_half = eval_bernstein_cubic(coeffs, 0.5);
    assert!(
        (at_half - 50.0).abs() < 1e-4,
        "midpoint of linear piece should be 50, got {at_half}"
    );
}

#[test]
fn eval_bernstein_cubic_constant_piece() {
    let coeffs = [42.5f32; 4];
    for u in [0.0, 0.25, 0.5, 0.75, 1.0] {
        let v = eval_bernstein_cubic(coeffs, u);
        assert!(
            (v - 42.5).abs() < 1e-5,
            "constant piece: expected 42.5 at u={u}, got {v}"
        );
    }
}

#[test]
fn same_mcu_trip_clock_exact_reconstruction() {
    const MCU_ID: u32 = 1;
    const FREQ: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ);
    let configs = stub_configs(MCU_ID, vec![AXIS_X]);

    let piece_start: u64 = 1_000_000;
    let duration_secs: f32 = 0.025;
    #[allow(clippy::cast_possible_truncation)]
    let duration_ticks = (duration_secs as f64 * FREQ) as u64;

    let piece = make_linear_piece(piece_start, duration_secs, 0.0, 50.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut traj_map: HashMap<AxisKey, Vec<PieceEntry>> = HashMap::new();
    traj_map.insert(key, vec![piece]);
    let homing_traj = Arc::new(Mutex::new(traj_map));

    let trip_clock = piece_start + duration_ticks / 2;

    let result =
        reconstruct_axis_position(MCU_ID, trip_clock, key, &router, &homing_traj, &configs);
    let pos = result.expect("same-MCU reconstruction must succeed");

    assert!(
        (pos - 25.0).abs() < 0.5,
        "midpoint of 0..50mm piece should be ~25mm, got {pos:.4}"
    );
}

#[test]
fn trip_at_piece_start_returns_start_position() {
    const MCU_ID: u32 = 2;
    const FREQ: f64 = 520_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ);
    let configs = stub_configs(MCU_ID, vec![AXIS_Z]);

    let piece_start: u64 = 5_000_000_000;
    let piece = make_linear_piece(piece_start, 0.025, 10.0, 30.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_Z as u8,
    };
    let mut map = HashMap::new();
    map.insert(key, vec![piece]);
    let homing_traj = Arc::new(Mutex::new(map));

    let result =
        reconstruct_axis_position(MCU_ID, piece_start, key, &router, &homing_traj, &configs);
    let pos = result.expect("trip at piece start must succeed");
    assert!(
        (pos - 10.0).abs() < 0.5,
        "expected start position 10mm, got {pos:.4}"
    );
}

#[test]
fn trip_outside_trajectory_window_errors() {
    const MCU_ID: u32 = 3;
    const FREQ: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ);
    let configs = stub_configs(MCU_ID, vec![AXIS_X]);

    let piece_start: u64 = 1_000_000;
    let duration_secs: f32 = 0.025;
    let piece = make_linear_piece(piece_start, duration_secs, 0.0, 10.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut map = HashMap::new();
    map.insert(key, vec![piece]);
    let homing_traj = Arc::new(Mutex::new(map));

    #[allow(clippy::cast_possible_truncation)]
    let way_after = piece_start + (duration_secs as f64 * FREQ) as u64 + 9_999_999;
    let result = reconstruct_axis_position(MCU_ID, way_after, key, &router, &homing_traj, &configs);
    assert!(
        result.is_err(),
        "trip after trajectory window must error, got: {result:?}"
    );
    let msg = result.unwrap_err();
    assert!(
        msg.contains("EndstopTripOutsideTrajectory") || msg.contains("outside"),
        "error must mention outside-trajectory, got: {msg}"
    );
}

#[test]
fn trip_before_trajectory_window_errors() {
    const MCU_ID: u32 = 4;
    const FREQ: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ);
    let configs = stub_configs(MCU_ID, vec![AXIS_X]);

    let piece_start: u64 = 1_000_000_000;
    let piece = make_linear_piece(piece_start, 0.025, 0.0, 10.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut map = HashMap::new();
    map.insert(key, vec![piece]);
    let homing_traj = Arc::new(Mutex::new(map));

    let before = piece_start - 1;
    let result = reconstruct_axis_position(MCU_ID, before, key, &router, &homing_traj, &configs);
    assert!(
        result.is_err(),
        "trip before trajectory window must error, got: {result:?}"
    );
}

#[test]
fn no_trajectory_pieces_errors() {
    const MCU_ID: u32 = 5;
    const FREQ: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ);
    let configs = stub_configs(MCU_ID, vec![AXIS_X]);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let homing_traj: Arc<Mutex<HashMap<AxisKey, Vec<PieceEntry>>>> =
        Arc::new(Mutex::new(HashMap::new()));

    let result =
        reconstruct_axis_position(MCU_ID, 12_345_678, key, &router, &homing_traj, &configs);
    assert!(
        result.is_err(),
        "missing trajectory must error, got: {result:?}"
    );
    let msg = result.unwrap_err();
    assert!(
        msg.contains("NoTrajectoryPieces") || msg.contains("no trajectory"),
        "error must mention missing pieces, got: {msg}"
    );
}

#[test]
fn multiple_pieces_trip_in_second_piece() {
    const MCU_ID: u32 = 6;
    const FREQ: f64 = 180_000_000.0;

    let router = router_with_clock(MCU_ID, FREQ);
    let configs = stub_configs(MCU_ID, vec![AXIS_X]);

    let duration_secs: f32 = 0.025;
    #[allow(clippy::cast_possible_truncation)]
    let duration_ticks = (duration_secs as f64 * FREQ) as u64;

    let piece1_start: u64 = 1_000_000;
    let piece2_start = piece1_start + duration_ticks;

    let piece1 = make_linear_piece(piece1_start, duration_secs, 0.0, 50.0);
    let piece2 = make_linear_piece(piece2_start, duration_secs, 50.0, 100.0);

    let key = AxisKey {
        mcu_id: MCU_ID,
        axis: AXIS_X as u8,
    };
    let mut map = HashMap::new();
    map.insert(key, vec![piece1, piece2]);
    let homing_traj = Arc::new(Mutex::new(map));

    let trip_clock = piece2_start + duration_ticks / 2;
    let result =
        reconstruct_axis_position(MCU_ID, trip_clock, key, &router, &homing_traj, &configs);
    let pos = result.expect("trip in second piece must succeed");
    assert!(
        (pos - 75.0).abs() < 1.0,
        "midpoint of 50..100mm second piece should be ~75mm, got {pos:.4}"
    );
}

#[test]
fn trajectory_final_position_single_piece() {
    let key = AxisKey {
        mcu_id: 10,
        axis: AXIS_X as u8,
    };
    let piece = make_linear_piece(1_000_000, 0.025, 5.0, 45.0);
    let mut map = HashMap::new();
    map.insert(key, vec![piece]);
    let homing_traj = Arc::new(Mutex::new(map));

    let pos =
        trajectory_final_position(key, &homing_traj).expect("single-piece store must succeed");
    assert!(
        (pos - 45.0).abs() < 1e-4,
        "final position must equal last coeffs[3]=45.0, got {pos:.6}"
    );
}

#[test]
fn trajectory_final_position_multi_piece_takes_last() {
    let key = AxisKey {
        mcu_id: 11,
        axis: AXIS_X as u8,
    };
    let piece1 = make_linear_piece(1_000_000, 0.025, 0.0, 50.0);
    let piece2 = make_linear_piece(5_500_000, 0.025, 50.0, 82.5);
    let mut map = HashMap::new();
    map.insert(key, vec![piece1, piece2]);
    let homing_traj = Arc::new(Mutex::new(map));

    let pos = trajectory_final_position(key, &homing_traj).expect("multi-piece store must succeed");
    assert!(
        (pos - 82.5).abs() < 1e-4,
        "final position must equal last piece's coeffs[3]=82.5, got {pos:.6}"
    );
}

#[test]
fn trajectory_final_position_missing_key_errors() {
    let key = AxisKey {
        mcu_id: 12,
        axis: AXIS_X as u8,
    };
    let homing_traj: Arc<Mutex<HashMap<AxisKey, Vec<PieceEntry>>>> =
        Arc::new(Mutex::new(HashMap::new()));

    let result = trajectory_final_position(key, &homing_traj);
    assert!(
        result.is_err(),
        "missing key must return Err, got: {result:?}"
    );
    let msg = result.unwrap_err();
    assert!(
        msg.contains("NoTrajectoryPieces") || msg.contains("no trajectory"),
        "error must mention missing pieces, got: {msg}"
    );
}

#[test]
fn trajectory_final_position_constant_piece() {
    let key = AxisKey {
        mcu_id: 13,
        axis: AXIS_Z as u8,
    };
    let piece = PieceEntry {
        start_time: 0,
        coeffs: [99.0_f32; 4],
        duration: 0.01,
        _reserved: 0,
    };
    let mut map = HashMap::new();
    map.insert(key, vec![piece]);
    let homing_traj = Arc::new(Mutex::new(map));

    let pos =
        trajectory_final_position(key, &homing_traj).expect("constant-piece store must succeed");
    assert!(
        (pos - 99.0).abs() < 1e-4,
        "constant piece endpoint must be 99.0, got {pos:.6}"
    );
}
