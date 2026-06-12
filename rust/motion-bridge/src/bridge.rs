use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

use kalico_host_rt::clock::RealClock;
use kalico_host_rt::host_io::parser::{DataDictionary, FieldValue, MsgProtoParser};
use kalico_host_rt::host_io::{KalicoHostIo, KalicoHostIoConfig};
use kalico_host_rt::passthrough_queue::{NotifyId, PassthroughEntry, PassthroughRouter};
use kalico_host_rt::unix_native_conn::UnixNativeConn;
use trajectory::{AxisShaper, ShaperConfig};

use crate::classify;
use crate::config::{self, PlannerConfig, PlannerLimits, parse_axis_shaper};
use crate::dispatch::{AXIS_E, McuAxisConfig, McuCaps, build_mcu_configs};
use crate::planner::{DispatchError, PlannerError, PlannerHandle};
use crate::types::{cq_id_from_raw, mcu_handle_from_raw, stats_to_pydict};

struct HomingRun {
    cohort: u64,
    endstop_id: u8,
    endstop_mcu: u32,
    axis: u8,
    axis_key: crate::pump::AxisKey,
    all_axis_keys: Vec<crate::pump::AxisKey>,
    window_start_clock: u64,
    notify: crossbeam_channel::Sender<Result<([f64; 3], [f64; 3], u64), String>>,
}

fn abort_after_tracing_appender_drains() {
    let _ = std::io::Write::flush(&mut std::io::stderr());
    std::thread::sleep(std::time::Duration::from_millis(100));
    if std::env::var_os("KALICO_NO_EXIT_ON_FAULT").is_none() {
        std::process::abort();
    }
}

fn trip_position_to_motor_frame(
    axis: u8,
    motor_pos: f64,
    _configs: &[crate::dispatch::McuAxisConfig],
    _axis_mcu: u32,
) -> [f64; 4] {
    let mut frame = [0.0f64; 4];
    if axis < 4 {
        frame[axis as usize] = motor_pos;
    }
    frame
}

struct McuConnection {
    label: String,
    serial_path: String,
    baud: u32,
    host_io: Option<Arc<KalicoHostIo>>,
    runtime_rx: Option<Receiver<kalico_host_rt::host_io::runtime_events::RuntimeEvent>>,
    runtime_caps: Option<kalico_protocol::messages::RuntimeCapsResponse>,
    identify_caps: u64,
    kalico_native_supported: bool,
    ethercat_socket: Option<String>,
    endpoint_process: Option<std::process::Child>,
    endpoint_conn: Option<Arc<UnixNativeConn>>,
}

const DRAIN_TIMEOUT: Duration = Duration::from_secs(60);

const ETHERCAT_CLOCK_FREQ_HZ: u32 = 1_000_000_000;

#[derive(Debug, thiserror::Error)]
enum RuntimeCapsError {
    #[error("kalico_call QueryRuntimeCaps: {0}")]
    Call(String),
    #[error("QueryRuntimeCaps: unexpected response kind {got:?}")]
    UnexpectedKind { got: kalico_protocol::MessageKind },
    #[error("decode RuntimeCapsResponse: {0}")]
    Decode(String),
}

fn decode_runtime_caps_body(
    body: &[u8],
) -> Result<kalico_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use kalico_protocol::codec::{Cursor, Decode};
    use kalico_protocol::messages::RuntimeCapsResponse;
    let mut c = Cursor::new(body);
    RuntimeCapsResponse::decode_from(&mut c).map_err(|e| RuntimeCapsError::Decode(format!("{e:?}")))
}

fn query_runtime_caps(
    io: &KalicoHostIo,
    timeout: std::time::Duration,
) -> Result<kalico_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use kalico_protocol::MessageKind;
    let (kind, body) = io
        .kalico_call(MessageKind::QueryRuntimeCaps, Vec::new(), timeout)
        .map_err(|e| RuntimeCapsError::Call(format!("{e:?}")))?;
    if kind != MessageKind::RuntimeCapsResponse {
        return Err(RuntimeCapsError::UnexpectedKind { got: kind });
    }
    decode_runtime_caps_body(&body)
}

fn query_ethercat_runtime_caps(
    conn: &UnixNativeConn,
    timeout: std::time::Duration,
) -> Result<kalico_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use kalico_host_rt::native_call::NativeCall;
    use kalico_protocol::MessageKind;
    let (kind, body) = conn
        .kalico_call(MessageKind::QueryRuntimeCaps, Vec::new(), timeout)
        .map_err(|e| RuntimeCapsError::Call(format!("{e:?}")))?;
    if kind != MessageKind::RuntimeCapsResponse {
        return Err(RuntimeCapsError::UnexpectedKind { got: kind });
    }
    decode_runtime_caps_body(&body)
}

#[derive(Debug)]
enum EndpointClaimError {
    DriveOffline { slave_idx: u8, fault_code: u16 },
    DriveFault { slave_idx: u8, fault_code: u16 },
    Protocol(String),
}

fn message_for_claim_error(label: &str, interface: &str, e: &EndpointClaimError) -> String {
    match e {
        EndpointClaimError::DriveOffline {
            slave_idx,
            fault_code,
        } => match fault_code {
            1 | 2 => format!(
                "ethercat {label}: EtherCAT bus on {interface}: no slaves responding \
                 (bringup rc=-{fault_code}) — check cable and drive power, then FIRMWARE_RESTART"
            ),
            10..=12 => format!(
                "ethercat {label}: realtime endpoint could not acquire RT scheduling \
                 (bringup rc=-{fault_code}) — grant CAP_SYS_NICE + CAP_IPC_LOCK to \
                 klipper.service and isolate a CPU core, then FIRMWARE_RESTART"
            ),
            // 0 = no bringup rc (e.g. the stub's simulated failure) — omit the suffix.
            0 => format!(
                "ethercat {label}: drive (slave {slave_idx}) offline \
                 — check drive power, then FIRMWARE_RESTART"
            ),
            _ => format!(
                "ethercat {label}: drive (slave {slave_idx}) offline \
                 (bringup rc=-{fault_code}) — check drive power, then FIRMWARE_RESTART"
            ),
        },
        EndpointClaimError::DriveFault {
            slave_idx,
            fault_code,
        } => format!(
            "ethercat {label}: drive (slave {slave_idx}) \
             fault 0x{fault_code:04x} — check drive, then FIRMWARE_RESTART"
        ),
        EndpointClaimError::Protocol(s) => {
            format!("ethercat {label}: endpoint protocol error — {s}")
        }
    }
}

#[cfg(test)]
mod claim_error_message_tests {
    use super::{EndpointClaimError, message_for_claim_error};

    #[test]
    fn bus_dead_ec_init_failure() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 0,
                fault_code: 1,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: EtherCAT bus on eth0: no slaves responding \
             (bringup rc=-1) — check cable and drive power, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn bus_dead_no_slaves() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 0,
                fault_code: 2,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: EtherCAT bus on eth0: no slaves responding \
             (bringup rc=-2) — check cable and drive power, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn drive_offline_with_rc() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 1,
                fault_code: 5,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: drive (slave 1) offline \
             (bringup rc=-5) — check drive power, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn rt_acquisition_failure_names_the_capability() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 1,
                fault_code: 12,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: realtime endpoint could not acquire RT scheduling \
             (bringup rc=-12) — grant CAP_SYS_NICE + CAP_IPC_LOCK to \
             klipper.service and isolate a CPU core, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn drive_offline_stub_fault_code_zero_takes_drive_branch() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveOffline {
                slave_idx: 1,
                fault_code: 0,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: drive (slave 1) offline \
             — check drive power, then FIRMWARE_RESTART"
        );
    }

    #[test]
    fn drive_fault_unchanged() {
        let msg = message_for_claim_error(
            "node_x",
            "eth0",
            &EndpointClaimError::DriveFault {
                slave_idx: 1,
                fault_code: 0x0021,
            },
        );
        assert_eq!(
            msg,
            "ethercat node_x: drive (slave 1) fault 0x0021 — check drive, then FIRMWARE_RESTART"
        );
    }
}

/// The caller (`claim_ethercat_node`) removes any stale socket file at
/// `socket_path` before calling this function. That pre-spawn removal is
/// necessary: `FrameServer::bind` unlinks-and-rebinds on the path, but that
/// happens *after* the process starts — between spawn and bind, a pre-existing
/// file would let `poll_socket_ready` return immediately on existence, racing
/// `handshake_ethercat_endpoint`'s connect ahead of the actual listener.
fn spawn_ethercat_endpoint(
    binary: &str,
    interface: &str,
    socket_path: &str,
    counts_per_mm: f64,
    velocity_ff: bool,
    dynamics_profile: Option<&str>,
    torque_clamp_pct: f64,
    following_error_counts: Option<u32>,
    max_torque_tenth_pct: Option<u16>,
) -> Result<std::process::Child, String> {
    let mut cmd = std::process::Command::new(binary);
    cmd.arg(interface)
        .arg("--socket")
        .arg(socket_path)
        .arg("--counts-per-mm")
        .arg(counts_per_mm.to_string())
        .arg("--torque-clamp-pct")
        .arg(torque_clamp_pct.to_string());
    if velocity_ff {
        cmd.arg("--velocity-ff");
    }
    if let Some(p) = dynamics_profile {
        cmd.arg("--dynamics-profile").arg(p);
    }
    if let Some(ferr) = following_error_counts {
        cmd.arg("--following-error-counts").arg(ferr.to_string());
    }
    if let Some(tq) = max_torque_tenth_pct {
        cmd.arg("--max-torque-tenth-pct").arg(tq.to_string());
    }
    cmd.spawn().map_err(|e| format!("spawn {binary}: {e}"))
}

fn poll_socket_ready(
    path: &str,
    deadline: Instant,
    child: &mut std::process::Child,
) -> Result<(), String> {
    loop {
        if std::path::Path::new(path).exists() {
            return Ok(());
        }
        if Instant::now() >= deadline {
            return Err(format!("endpoint socket {path} did not appear within 15 s"));
        }
        match child.try_wait() {
            Ok(Some(status)) => {
                return Err(format!(
                    "endpoint process exited before socket appeared \
                     (exit status: {status})"
                ));
            }
            Ok(None) => {}
            Err(e) => {
                return Err(format!("try_wait on endpoint process failed: {e}"));
            }
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

fn handshake_ethercat_endpoint(
    socket_path: &str,
    deadline: Instant,
) -> Result<UnixNativeConn, EndpointClaimError> {
    use kalico_host_rt::native_call::NativeCall;
    use kalico_protocol::MessageKind;
    use kalico_protocol::codec::{Cursor, Decode};
    use kalico_protocol::messages::ClaimHandshakeReply;

    // Retry connect until the endpoint's listener is up. ConnectionRefused and
    // NotFound both mean the endpoint hasn't bound yet (bind latency, or the
    // endpoint is mid-unlink-and-rebind of a stale path). Every other error is
    // immediately fatal as a Protocol error — we don't mask real failures.
    let conn = loop {
        match UnixNativeConn::connect(socket_path) {
            Ok(c) => break c,
            Err(e)
                if e.kind() == std::io::ErrorKind::ConnectionRefused
                    || e.kind() == std::io::ErrorKind::NotFound =>
            {
                if Instant::now() >= deadline {
                    return Err(EndpointClaimError::Protocol(format!(
                        "connect to {socket_path}: timed out waiting for listener ({e})"
                    )));
                }
                std::thread::sleep(Duration::from_millis(50));
            }
            Err(e) => {
                return Err(EndpointClaimError::Protocol(format!(
                    "connect to {socket_path}: {e}"
                )));
            }
        }
    };

    let remaining = deadline.saturating_duration_since(Instant::now());
    let (kind, body) = conn
        .kalico_call(MessageKind::ClaimHandshake, Vec::new(), remaining)
        .map_err(|e| EndpointClaimError::Protocol(format!("ClaimHandshake call: {e:?}")))?;

    if kind != MessageKind::ClaimHandshakeReply {
        return Err(EndpointClaimError::Protocol(format!(
            "expected ClaimHandshakeReply (0x{:04x}), got 0x{:04x}",
            MessageKind::ClaimHandshakeReply.as_u16(),
            kind.as_u16(),
        )));
    }

    let reply = ClaimHandshakeReply::decode_from(&mut Cursor::new(&body))
        .map_err(|e| EndpointClaimError::Protocol(format!("decode ClaimHandshakeReply: {e:?}")))?;

    for s in &reply.slave_statuses {
        match s.state {
            kalico_protocol::messages::SlaveState::Offline => {
                return Err(EndpointClaimError::DriveOffline {
                    slave_idx: s.slave_idx,
                    fault_code: s.fault_code,
                });
            }
            kalico_protocol::messages::SlaveState::Fault => {
                return Err(EndpointClaimError::DriveFault {
                    slave_idx: s.slave_idx,
                    fault_code: s.fault_code,
                });
            }
            kalico_protocol::messages::SlaveState::Ok => {}
        }
    }

    Ok(conn)
}

#[derive(Debug, Clone)]
struct BridgeEvent {
    kind: String,
    mcu: u32,
    notify_id: u64,
    response_bytes: Vec<u8>,
    sent_time: f64,
    receive_time: f64,
}

impl BridgeEvent {
    fn to_pydict(&self, py: Python<'_>) -> PyResult<Py<PyDict>> {
        let d = PyDict::new(py);
        d.set_item("type", &self.kind)?;
        d.set_item("mcu", self.mcu)?;
        d.set_item("notify_id", self.notify_id)?;
        d.set_item("data", pyo3::types::PyBytes::new(py, &self.response_bytes))?;
        d.set_item("sent_time", self.sent_time)?;
        d.set_item("receive_time", self.receive_time)?;
        Ok(d.unbind())
    }
}

fn router_err(e: kalico_host_rt::passthrough_queue::RouterError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

fn planner_err(e: PlannerError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

fn build_shaper_config(
    type_x: &str,
    freq_x: f64,
    type_y: &str,
    freq_y: f64,
) -> Result<ShaperConfig, crate::config::ShaperConfigError> {
    Ok(ShaperConfig {
        x: parse_axis_shaper(type_x, freq_x)?,
        y: parse_axis_shaper(type_y, freq_y)?,
        z: AxisShaper::Passthrough,
    })
}

fn resolve_motion_caps(
    caps: Option<kalico_protocol::messages::RuntimeCapsResponse>,
    label: &str,
    handle: u32,
) -> Result<McuCaps, String> {
    caps.map(McuCaps::from).ok_or_else(|| {
        format!(
            "no runtime caps for {label} MCU (handle={handle}) — cannot size piece rings; \
             firmware not flashed or QueryRuntimeCaps failed at attach"
        )
    })
}

fn require_events_dir_for_kalico_native(
    kalico_native: bool,
    events_dir: Option<&std::path::Path>,
    mcu_label: &str,
) -> Result<(), String> {
    if kalico_native && events_dir.is_none() {
        return Err(format!(
            "attach_serial({mcu_label}): init_logging must be called before \
             attach_serial for a kalico-native MCU — the dedicated \
             mcu-*.jsonl writer cannot be installed without an events_dir. \
             All McuLog events would be silently discarded to the general \
             runtime_rx channel with no NDJSON output, which violates the \
             observability spec (§4, Decision C). Call init_logging first."
        ));
    }
    Ok(())
}

#[pyclass(name = "MotionBridge")]
#[allow(missing_debug_implementations)]
pub struct PyMotionBridge {
    router: Arc<Mutex<PassthroughRouter>>,
    parser: Arc<Mutex<Option<Arc<MsgProtoParser>>>>,
    mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    events: Arc<Mutex<VecDeque<BridgeEvent>>>,
    #[allow(dead_code)]
    handlers: Mutex<HashMap<(u32, String, u32), Py<PyAny>>>,
    // `Mutex<Option<..>>` (not `OnceLock`) so `shutdown()` can *take* the handle
    // and join the `kalico-planner` thread. A `OnceLock` cannot be drained, so
    // the planner thread would only be joined when the whole bridge dropped —
    // which never happens on klippy's in-process FIRMWARE_RESTART loop.
    planner: Mutex<Option<PlannerHandle>>,
    planner_config: Mutex<PlannerConfig>,
    commanded_pos: Mutex<[f64; 3]>,
    mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
    dispatched_segments: Arc<AtomicU64>,
    fallback_clock_conversions: Arc<AtomicU64>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    nominal_clock_freqs: Arc<Mutex<HashMap<u32, u32>>>,
    events_dir: Mutex<Option<std::path::PathBuf>>,
    pump_tx: Arc<Mutex<Option<std::sync::mpsc::Sender<crate::pump::PumpMsg>>>>,
    pump_thread: Mutex<Option<JoinHandle<()>>>,
    drain: std::sync::Arc<crate::drain::DrainSync>,
    active_drip_cohort: Arc<Mutex<Option<u64>>>,
    motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    homing_run: Arc<Mutex<Option<HomingRun>>>,
    homing_result:
        Mutex<Option<crossbeam_channel::Receiver<Result<([f64; 3], [f64; 3], u64), String>>>>,
    latched_drive_fault: Arc<Mutex<HashMap<u32, u16>>>,
    remote_triggers: Mutex<HashMap<u8, (u32, kalico_host_rt::host_io::InterceptorId)>>,
    // Latched once `shutdown()` has run a full teardown. Subsequent calls (the
    // Drop backstop, a second `klippy:disconnect`, the failed-connect path) see
    // this and no-op, so double-teardown is provably safe and observable.
    shut_down: AtomicBool,
}

pub(crate) fn axis_ring_depth(total_pieces: u32, num_axes: u32) -> u32 {
    (total_pieces / num_axes.max(1)).max(1)
}

pub(crate) fn drip_cohort_participants(configs: &[McuAxisConfig]) -> Vec<crate::pump::AxisKey> {
    configs
        .iter()
        .flat_map(|cfg| {
            cfg.axes
                .iter()
                .filter(|&&a| a < AXIS_E)
                .map(move |&a| crate::pump::AxisKey {
                    mcu_id: cfg.mcu_id,
                    axis: a as u8,
                })
        })
        .collect()
}

#[cfg(test)]
mod drip_cohort_participants_tests {
    use super::drip_cohort_participants;
    use crate::dispatch::{AXIS_E, AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps};
    use crate::pump::AxisKey;

    fn cfg(mcu_id: u32, axes: Vec<usize>) -> McuAxisConfig {
        McuAxisConfig {
            mcu_id,
            axes,
            caps: McuCaps {
                total_piece_memory: 0,
            },
            kinematics: 1,
        }
    }

    #[test]
    fn excludes_the_extruder_so_the_homing_floor_can_advance() {
        let configs = vec![cfg(0, vec![AXIS_Y, AXIS_Z, AXIS_E]), cfg(1, vec![AXIS_X])];
        let participants = drip_cohort_participants(&configs);
        assert_eq!(
            participants,
            vec![
                AxisKey {
                    mcu_id: 0,
                    axis: AXIS_Y as u8
                },
                AxisKey {
                    mcu_id: 0,
                    axis: AXIS_Z as u8
                },
                AxisKey {
                    mcu_id: 1,
                    axis: AXIS_X as u8
                },
            ]
        );
        assert!(participants.iter().all(|k| k.axis != AXIS_E as u8));
    }
}

#[cfg(test)]
mod axis_ring_depth_tests {
    use super::axis_ring_depth;

    #[test]
    fn typical_two_axis_mcu() {
        assert_eq!(axis_ring_depth(1984, 2), 992);
    }

    #[test]
    fn single_axis_mcu() {
        assert_eq!(axis_ring_depth(1984, 1), 1984);
    }

    #[test]
    fn floor_division() {
        assert_eq!(axis_ring_depth(5, 2), 2);
    }

    #[test]
    fn lower_clamp_on_zero_total() {
        assert_eq!(axis_ring_depth(0, 2), 1);
    }

    #[test]
    fn zero_num_axes_treated_as_one() {
        assert_eq!(axis_ring_depth(1000, 0), 1000);
    }
}

pub(crate) fn ring_depth_for_axis_inner(
    configs: &[crate::dispatch::McuAxisConfig],
    mcu_handle: u32,
    axis: u8,
) -> Result<u16, String> {
    let cfg = configs
        .iter()
        .find(|c| c.mcu_id == mcu_handle)
        .ok_or_else(|| {
            format!(
                "ring_depth_for_axis: unknown mcu_handle {mcu_handle} \
                 (init_planner not yet called?)"
            )
        })?;
    let axis_usize = usize::from(axis);
    if !cfg.axes.contains(&axis_usize) {
        return Err(format!(
            "ring_depth_for_axis: axis {axis} is not configured on mcu_handle \
             {mcu_handle} (configured axes: {:?})",
            cfg.axes
        ));
    }
    let depth = axis_ring_depth(cfg.caps.total_pieces() as u32, cfg.axes.len() as u32);
    if depth > u32::from(u16::MAX) {
        return Err(format!(
            "ring depth {depth} exceeds u16::MAX (65535) for mcu {mcu_handle} axis {axis}; \
             a >65535-piece ring would need >2 MB of SRAM and is impossible here — \
             check total_piece_memory configuration"
        ));
    }
    #[allow(clippy::cast_possible_truncation)]
    Ok(depth as u16)
}

#[cfg(test)]
mod ring_depth_for_axis_tests {
    use super::ring_depth_for_axis_inner;
    use crate::dispatch::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps};

    fn configs() -> Vec<McuAxisConfig> {
        vec![
            McuAxisConfig {
                mcu_id: 1,
                axes: vec![AXIS_X, AXIS_Y],
                kinematics: 0,
                caps: McuCaps {
                    total_piece_memory: 62 * 1024,
                },
            },
            McuAxisConfig {
                mcu_id: 2,
                axes: vec![AXIS_Z],
                kinematics: 1,
                caps: McuCaps {
                    total_piece_memory: 62 * 1024,
                },
            },
        ]
    }

    #[test]
    fn success_two_axis_mcu() {
        assert_eq!(
            ring_depth_for_axis_inner(&configs(), 1, AXIS_X as u8).unwrap(),
            992
        );
        assert_eq!(
            ring_depth_for_axis_inner(&configs(), 1, AXIS_Y as u8).unwrap(),
            992
        );
    }

    #[test]
    fn success_single_axis_mcu() {
        assert_eq!(
            ring_depth_for_axis_inner(&configs(), 2, AXIS_Z as u8).unwrap(),
            1984
        );
    }

    #[test]
    fn unknown_mcu_handle_errors() {
        let e = ring_depth_for_axis_inner(&configs(), 99, AXIS_X as u8).unwrap_err();
        assert!(e.contains("unknown mcu_handle 99"), "got: {e}");
    }

    #[test]
    fn axis_not_on_mcu_errors() {
        let e = ring_depth_for_axis_inner(&configs(), 1, AXIS_Z as u8).unwrap_err();
        assert!(e.contains("not configured"), "got: {e}");
    }

    #[test]
    fn ring_depth_over_u16_is_hard_error_not_clamp() {
        let configs = vec![McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X],
            kinematics: 0,
            caps: McuCaps {
                total_piece_memory: 70_000 * 32,
            },
        }];
        let res = ring_depth_for_axis_inner(&configs, 0, AXIS_X as u8);
        assert!(
            res.is_err(),
            "depth > u16::MAX must be a hard error, not a clamp"
        );
        let e = res.unwrap_err();
        assert!(
            e.contains("exceeds u16::MAX"),
            "error message should mention u16::MAX, got: {e}"
        );
    }
}

#[pymethods]
impl PyMotionBridge {
    #[new]
    fn new() -> Self {
        let clock: Arc<dyn kalico_host_rt::clock::Clock + Send + Sync> = Arc::new(RealClock);
        Self {
            router: Arc::new(Mutex::new(PassthroughRouter::with_clock(clock))),
            parser: Arc::new(Mutex::new(None)),
            mcus: Arc::new(Mutex::new(HashMap::new())),
            events: Arc::new(Mutex::new(VecDeque::new())),
            handlers: Mutex::new(HashMap::new()),
            planner: Mutex::new(None),
            planner_config: Mutex::new(PlannerConfig::default()),
            commanded_pos: Mutex::new([0.0; 3]),
            mcu_axis_configs: Arc::new(Mutex::new(Vec::new())),
            dispatched_segments: Arc::new(AtomicU64::new(0)),
            fallback_clock_conversions: Arc::new(AtomicU64::new(0)),
            clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            nominal_clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            events_dir: Mutex::new(None),
            pump_tx: Arc::new(Mutex::new(None)),
            pump_thread: Mutex::new(None),
            drain: std::sync::Arc::new(crate::drain::DrainSync::new()),
            active_drip_cohort: Arc::new(Mutex::new(None)),
            motion_history: Arc::new(Mutex::new(crate::motion_history::HistoryStore::default())),
            homing_run: Arc::new(Mutex::new(None)),
            homing_result: Mutex::new(None),
            latched_drive_fault: Arc::new(Mutex::new(HashMap::new())),
            remote_triggers: Mutex::new(HashMap::new()),
            shut_down: AtomicBool::new(false),
        }
    }

    fn version(&self) -> &'static str {
        env!("CARGO_PKG_VERSION")
    }

    fn init_logging(&self, events_dir: String) -> PyResult<()> {
        let path = std::path::Path::new(&events_dir);
        crate::logging::init_logging(path).map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("init_logging failed: {e}"))
        })?;
        let mut guard = self.events_dir.lock().unwrap_or_else(|p| p.into_inner());
        *guard = Some(path.to_path_buf());
        Ok(())
    }

    #[pyo3(signature = (session_id, print_id=String::new()))]
    fn set_session_context(&self, session_id: String, print_id: String) {
        crate::logging::set_context(session_id, print_id);
    }

    #[pyo3(signature = (label, serial_path, baud))]
    fn claim_mcu(&self, label: &str, serial_path: &str, baud: u32) -> PyResult<u32> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        self.mcus.lock().unwrap_or_else(|p| p.into_inner()).insert(
            raw,
            McuConnection {
                label: label.to_owned(),
                serial_path: serial_path.to_owned(),
                baud,
                host_io: None,
                runtime_rx: None,
                runtime_caps: None,
                identify_caps: 0,
                kalico_native_supported: false,
                ethercat_socket: None,
                endpoint_process: None,
                endpoint_conn: None,
            },
        );
        Ok(raw)
    }

    #[pyo3(signature = (label, socket_path, interface, endpoint_binary, counts_per_mm, velocity_ff, dynamics_profile, torque_clamp_pct, following_error_counts=None, max_torque_tenth_pct=None))]
    fn claim_ethercat_node(
        &self,
        label: &str,
        socket_path: &str,
        interface: &str,
        endpoint_binary: &str,
        counts_per_mm: f64,
        velocity_ff: bool,
        dynamics_profile: Option<String>,
        torque_clamp_pct: f64,
        following_error_counts: Option<u32>,
        max_torque_tenth_pct: Option<u16>,
    ) -> PyResult<u32> {
        if let Err(e) = std::fs::remove_file(socket_path) {
            if e.kind() != std::io::ErrorKind::NotFound {
                return Err(PyRuntimeError::new_err(format!(
                    "ethercat {label}: failed to remove stale socket {socket_path}: {e}"
                )));
            }
        }

        let mut child = spawn_ethercat_endpoint(
            endpoint_binary,
            interface,
            socket_path,
            counts_per_mm,
            velocity_ff,
            dynamics_profile.as_deref(),
            torque_clamp_pct,
            following_error_counts,
            max_torque_tenth_pct,
        )
        .map_err(|e| {
            PyRuntimeError::new_err(format!("ethercat {label}: endpoint failed to start — {e}"))
        })?;

        let socket_deadline = Instant::now() + Duration::from_secs(15);
        if let Err(detail) = poll_socket_ready(socket_path, socket_deadline, &mut child) {
            let _ = child.kill();
            let _ = child.wait();
            return Err(PyRuntimeError::new_err(format!(
                "ethercat {label}: {detail}"
            )));
        }

        let handshake_deadline = Instant::now() + Duration::from_secs(25);
        let conn = handshake_ethercat_endpoint(socket_path, handshake_deadline).map_err(|e| {
            let _ = child.kill();
            let _ = child.wait();
            PyRuntimeError::new_err(message_for_claim_error(label, interface, &e))
        })?;

        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        drop(router);
        self.register_ethercat_mcu(raw, label, socket_path, child, conn);
        Ok(raw)
    }

    fn set_torque(&self, mcu_handle: u32, value: bool, print_time: f64) -> PyResult<()> {
        let reference_mcu = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            *mcus
                .iter()
                .find(|(_, mc)| mc.label == "mcu")
                .map(|(raw, _)| raw)
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "set_torque: no MCU labeled 'mcu' claimed — \
                         cannot resolve the print_time reference clock",
                    )
                })?
        };
        let execute_at_ns = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            let host_secs = router
                .print_time_to_host_secs(mcu_handle_from_raw(reference_mcu), print_time)
                .ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "set_torque: reference mcu {reference_mcu} clock not synced — \
                         cannot convert print_time {print_time}"
                    ))
                })?;
            router
                .host_time_to_mcu_clock(mcu_handle_from_raw(mcu_handle), host_secs)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "set_torque: no clock mapping for mcu {mcu_handle}: {e:?}"
                    ))
                })?
        };
        if execute_at_ns == 0 {
            return Err(PyRuntimeError::new_err(format!(
                "set_torque: EtherCAT clock for mcu {mcu_handle} not seeded \
                 (init_planner not run?)"
            )));
        }
        let conn = self.ethercat_conn(mcu_handle, "set_torque")?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_torque_command",
            mcu_handle,
            value,
            print_time,
            execute_at_ns,
            "servo torque command"
        );
        let result = crate::servo_torque::send_set_torque(&conn, value, execute_at_ns)
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            tracing::error!(
                subsystem = "bridge",
                event = "servo_torque_rejected",
                mcu_handle,
                value,
                result,
                "servo torque command rejected"
            );
            return Err(PyRuntimeError::new_err(format!(
                "servo torque {} failed: endpoint result {result}",
                if value { "enable" } else { "disable" }
            )));
        }
        Ok(())
    }

    fn start_servo_capture(
        &self,
        mcu_handle: u32,
        path: String,
        started_utc: String,
        drive_name: String,
    ) -> PyResult<()> {
        let conn = self.ethercat_conn(mcu_handle, "start_servo_capture")?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_capture_start",
            mcu_handle,
            path,
            "servo capture start"
        );
        let result =
            crate::servo_capture::send_start_capture(&conn, &path, &started_utc, &drive_name)
                .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "servo capture start failed: endpoint result {result}"
            )));
        }
        Ok(())
    }

    fn stop_servo_capture(&self, mcu_handle: u32) -> PyResult<(i32, u64, Option<u64>)> {
        let conn = self.ethercat_conn(mcu_handle, "stop_servo_capture")?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_capture_stop",
            mcu_handle,
            "servo capture stop"
        );
        let resp =
            crate::servo_capture::send_stop_capture(&conn).map_err(PyRuntimeError::new_err)?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_capture_stopped",
            mcu_handle,
            result = resp.result,
            samples = resp.samples,
            "servo capture stopped"
        );
        let overflow = (resp.overflow_cycle
            != kalico_protocol::messages::StopCaptureResponse::NO_OVERFLOW)
            .then_some(resp.overflow_cycle);
        Ok((resp.result, resp.samples, overflow))
    }

    fn set_drive_limits(
        &self,
        mcu_handle: u32,
        following_error_counts: u32,
        max_torque_tenth_pct: u16,
    ) -> PyResult<()> {
        let conn = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mc = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "set_drive_limits: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            mc.endpoint_conn.clone().ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "set_drive_limits: mcu {mcu_handle} ({}) is not an EtherCAT endpoint",
                    mc.label
                ))
            })?
        };
        tracing::info!(
            subsystem = "bridge",
            event = "servo_drive_limits",
            mcu_handle,
            following_error_counts,
            max_torque_tenth_pct,
            "servo drive limits set"
        );
        let result = crate::servo_torque::send_drive_limits(
            &conn,
            following_error_counts,
            max_torque_tenth_pct,
        )
        .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "set_drive_limits: SDO write failed: endpoint result {result}"
            )));
        }
        Ok(())
    }

    fn restore_drive_limits(&self, mcu_handle: u32) -> PyResult<()> {
        let conn = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mc = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "restore_drive_limits: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            mc.endpoint_conn.clone().ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "restore_drive_limits: mcu {mcu_handle} ({}) is not an EtherCAT endpoint",
                    mc.label
                ))
            })?
        };
        tracing::info!(
            subsystem = "bridge",
            event = "servo_drive_limits",
            mcu_handle,
            "servo drive limits restored"
        );
        let result = crate::servo_torque::send_restore_drive_limits(&conn)
            .map_err(PyRuntimeError::new_err)?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "restore_drive_limits: SDO write failed: endpoint result {result}"
            )));
        }
        Ok(())
    }

    fn take_drive_fault(&self, mcu_handle: u32) -> PyResult<Option<u16>> {
        Ok(self
            .latched_drive_fault
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(&mcu_handle))
    }

    fn sdo_read(&self, mcu_handle: u32, index: u16, subindex: u8) -> PyResult<(u8, u32)> {
        let conn = self.ethercat_conn(mcu_handle, "sdo_read")?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_sdo_read",
            mcu_handle,
            index,
            subindex,
            "servo SDO read"
        );
        let r = crate::servo_sdo::send_sdo_read(&conn, index, subindex)
            .map_err(PyRuntimeError::new_err)?;
        if r.result != 0 {
            tracing::error!(
                subsystem = "bridge",
                event = "servo_sdo_read_failed",
                mcu_handle,
                index,
                subindex,
                result = r.result,
                "servo SDO read failed"
            );
            return Err(PyRuntimeError::new_err(format!(
                "SDO read 0x{index:04x}.{subindex}: {}",
                crate::servo_sdo::failure_text(r.result)
            )));
        }
        Ok((r.size, u32::from_le_bytes(r.data)))
    }

    fn sdo_write(
        &self,
        mcu_handle: u32,
        index: u16,
        subindex: u8,
        size: u8,
        value: i64,
    ) -> PyResult<(u8, u32)> {
        let conn = self.ethercat_conn(mcu_handle, "sdo_write")?;
        tracing::info!(
            subsystem = "bridge",
            event = "servo_sdo_write",
            mcu_handle,
            index,
            subindex,
            size,
            value,
            "servo SDO write"
        );
        let r = crate::servo_sdo::send_sdo_write(&conn, index, subindex, size, value)
            .map_err(PyRuntimeError::new_err)?;
        if r.result != 0 {
            tracing::error!(
                subsystem = "bridge",
                event = "servo_sdo_write_failed",
                mcu_handle,
                index,
                subindex,
                size,
                value,
                result = r.result,
                "servo SDO write failed"
            );
            let readback = u32::from_le_bytes(r.readback_data);
            return Err(PyRuntimeError::new_err(format!(
                "SDO write 0x{index:04x}.{subindex} = {value} (size {size}): {} \
                 (drive reports raw 0x{readback:x})",
                crate::servo_sdo::failure_text(r.result)
            )));
        }
        Ok((r.readback_size, u32::from_le_bytes(r.readback_data)))
    }

    fn release_mcu(&self, handle: u32) -> PyResult<()> {
        // Pull the whole McuConnection out of the map but keep it alive (it owns
        // `host_io`) until *after* the endpoint child is reaped. Teardown order
        // matters: the endpoint must see session-end (socket close + SIGTERM)
        // before we close the host_io pts fd, which is the EBUSY-relevant step.
        //
        // Removing from the map BEFORE closing the endpoint socket (below) is
        // also the ec-heartbeat-poll race guard: the supervision thread confirms
        // every EOF/child-exit fault against `mcus.get(&mcu_id)` under the lock,
        // so by the time the socket close it observes as peer_closed() has
        // happened, the entry is already gone and the fault is read as a clean
        // release rather than fired into std::process::abort().
        let Some(mut conn) = ({
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.remove(&handle)
        }) else {
            // Already released — idempotent no-op (shutdown may call twice, the
            // failed-connect path may call before any attach).
            return Ok(());
        };

        let mut endpoint_process = conn.endpoint_process.take();
        let endpoint_conn = conn.endpoint_conn.take();

        // Drop our Arc on the endpoint connection so the socket closes (signals
        // session end to the endpoint). Router/pump Arcs may still be live;
        // SIGTERM is the authoritative termination signal below.
        drop(endpoint_conn);

        if let Some(ref mut child) = endpoint_process {
            // Capture PID before any wait so it is valid in diagnostic messages
            // (after wait() the OS may reuse the pid_t value).
            let pid = libc::pid_t::try_from(child.id()).expect("child PID exceeds pid_t range");

            // SIGTERM: ask the endpoint to exit gracefully.
            // `libc::kill` is the only stable way to send a specific signal to
            // a child process on Unix; there is no safe std API for this.
            // ESRCH (no such process) = already exited = fine; discard the return value.
            #[allow(unsafe_code)]
            let _ = unsafe { libc::kill(pid, libc::SIGTERM) };

            let reap_deadline = Instant::now() + Duration::from_secs(5);
            loop {
                match child.try_wait() {
                    Ok(Some(_)) => break,
                    Ok(None) => {}
                    Err(_) => break,
                }
                if Instant::now() >= reap_deadline {
                    let _ = child.kill();
                    let _ = child.wait();
                    log::warn!(
                        "release_mcu: ethercat endpoint (pid {}) did not exit \
                         within 5 s after SIGTERM — SIGKILL sent",
                        pid
                    );
                    break;
                }
                std::thread::sleep(Duration::from_millis(50));
            }
        }

        // Endpoint is dead; now close the host_io. Dropping the McuConnection
        // drops its `Arc<KalicoHostIo>` — the last strong ref (pump/heartbeat
        // hold `Weak` only), so `KalicoHostIo::Drop` runs here: it sends the
        // reactor Shutdown and joins the reactor thread, which closes the pts
        // fd and releases TIOCEXCL — clearing the EBUSY for the next process.
        drop(conn);

        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router.release_mcu(mcu_handle_from_raw(handle));
        self.handlers
            .lock()
            .unwrap()
            .retain(|&(mcu, _, _), _| mcu != handle);
        Ok(())
    }

    /// The single, complete, ordered, idempotent teardown primitive.
    ///
    /// It is the authoritative release path on every klippy exit that can leave
    /// state behind (`klippy:disconnect`, the failed-connect arms, and the Drop
    /// backstop). Calling it more than once is a clean no-op — the second call
    /// finds empty maps / `None` handles and the latched `shut_down` flag.
    ///
    /// Ordering — two hazards drive the order, one in each direction:
    ///
    ///   Hazard A (planner → pump): while the planner holds an uncommitted decel
    ///   tail (`t_dispatched < t_appended`, true after essentially any motion),
    ///   its `recv_timeout` fires `run_commit_and_dispatch`, whose dispatch closure
    ///   does `pump_tx.send(..)`. If the pump's `Receiver` were already gone that
    ///   send yields `DispatchError::PumpGone` → the planner calls `fatal()` →
    ///   `std::process::abort()`, which skips every `Drop` — leaking the pts fd.
    ///   Fix: join the planner BEFORE sending `PumpMsg::Shutdown`; once the
    ///   planner thread is joined no further dispatch can fire.
    ///
    ///   Hazard B (pump → EtherCAT conn): the pump may still be draining
    ///   already-queued pieces for an EtherCAT MCU after `release_mcu` drops the
    ///   last strong `Arc<UnixNativeConn>`. In `call_push_pieces` the
    ///   `Weak::upgrade()` then returns `None` → `SendError::Fatal` →
    ///   `on_fatal_transport` → `std::process::abort()` — the same pts-fd leak.
    ///   Fix: join the pump BEFORE calling `release_mcu`; once the pump thread is
    ///   joined no send can be in flight.
    ///
    ///   Together: planner join → pump Shutdown + join → per-MCU release_mcu.
    ///
    ///   Post-join heartbeat sends: the ec-heartbeat-poll thread holds a clone of
    ///   `pump_tx`. After the pump's `Receiver` is dropped (pump joined), those
    ///   sends silently return `Err` and are discarded by the callback — harmless.
    fn shutdown(&self) {
        if self.shut_down.swap(true, Ordering::SeqCst) {
            log::debug!("bridge.shutdown() called twice (idempotent no-op)");
            return;
        }

        // Step 1 — planner: join before the pump receives Shutdown so the planner
        // can never dispatch into a dead pump Receiver (Hazard A).
        let planner = self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .take();
        if let Some(mut p) = planner {
            p.shutdown();
        }

        // Step 2 — pump: join before releasing MCU transports so no queued piece
        // can hit a dead EtherCAT Weak after release_mcu drops the strong Arc
        // (Hazard B). run_pump exits immediately on Shutdown, abandoning queued
        // pieces — safe because the planner is already joined and no new pieces
        // will arrive.
        let pump_join = {
            let tx = self
                .pump_tx
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .take();
            if let Some(tx) = tx {
                let _ = tx.send(crate::pump::PumpMsg::Shutdown);
            }
            self.pump_thread
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .take()
        };
        if let Some(h) = pump_join {
            if let Err(e) = h.join() {
                log::error!("bridge.shutdown(): push-pieces-pump join panicked: {e:?}");
            }
        }

        // Step 3 — per-MCU release_mcu: endpoint socket/child first, then
        // host_io fd (the EBUSY-relevant close), then router prune. The pump is
        // already joined so no send is in flight when the strong Arc drops.
        let handles: Vec<u32> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.keys().copied().collect()
        };
        for h in handles {
            if let Err(e) = self.release_mcu(h) {
                // Fail loud: a release error means an fd / child may be leaked.
                log::error!("bridge.shutdown(): release_mcu({h}) failed: {e}");
            }
        }
    }

    fn alloc_command_queue(&self, handle: u32) -> PyResult<u32> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let qid = router
            .alloc_command_queue(mcu_handle_from_raw(handle))
            .map_err(router_err)?;
        Ok(qid.raw())
    }

    #[pyo3(signature = (mcu, queue, data, min_clock=0, req_clock=0))]
    fn passthrough_send(
        &self,
        mcu: u32,
        queue: u32,
        data: &[u8],
        min_clock: u64,
        req_clock: u64,
    ) -> PyResult<()> {
        let entry = PassthroughEntry::new(data.to_vec(), min_clock, req_clock, NotifyId::none());
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .push(mcu_handle_from_raw(mcu), cq_id_from_raw(queue), entry)
            .map_err(router_err)?;
        Ok(())
    }

    #[pyo3(signature = (mcu, queue, data, min_clock=0, req_clock=0))]
    fn passthrough_query(
        &self,
        mcu: u32,
        queue: u32,
        data: &[u8],
        min_clock: u64,
        req_clock: u64,
    ) -> PyResult<u64> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let mcu_h = mcu_handle_from_raw(mcu);

        let events_ref = Arc::clone(&self.events);
        let mcu_raw = mcu;

        let nid = router
            .register_notify(
                mcu_h,
                Box::new(move |resp| {
                    let ev = BridgeEvent {
                        kind: "query_response".to_owned(),
                        mcu: mcu_raw,
                        notify_id: 0,
                        response_bytes: resp.bytes,
                        sent_time: resp.sent_time,
                        receive_time: resp.receive_time,
                    };
                    events_ref
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .push_back(ev);
                }),
            )
            .map_err(router_err)?;

        let entry = PassthroughEntry::new(data.to_vec(), min_clock, req_clock, nid);
        router
            .push(mcu_h, cq_id_from_raw(queue), entry)
            .map_err(router_err)?;

        Ok(nid.raw())
    }

    #[pyo3(signature = (_mcu, _queue, _data, _timeout))]
    fn passthrough_send_wait_ack(
        &self,
        _mcu: u32,
        _queue: u32,
        _data: &[u8],
        _timeout: f64,
    ) -> PyResult<Vec<u8>> {
        Err(pyo3::exceptions::PyNotImplementedError::new_err(
            "send_wait_ack requires reactor thread — deferred to Phase 2",
        ))
    }

    #[pyo3(signature = (mcu, name, oid, callback))]
    fn passthrough_register_handler(
        &self,
        mcu: u32,
        name: &str,
        oid: u32,
        callback: Py<PyAny>,
    ) -> PyResult<()> {
        self.handlers
            .lock()
            .unwrap()
            .insert((mcu, name.to_owned(), oid), callback);
        Ok(())
    }

    fn passthrough_register_flush_callback(&self, mcu: u32, callback: Py<PyAny>) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let mcu_h = mcu_handle_from_raw(mcu);

        let cb: Box<dyn Fn() + Send> = Box::new(move || {
            Python::attach(|py| {
                if let Err(e) = callback.call0(py) {
                    e.print(py);
                }
            });
        });

        router
            .register_flush_callback(mcu_h, cb)
            .map_err(router_err)?;
        Ok(())
    }

    fn poll_event(&self, py: Python<'_>) -> PyResult<Option<Py<PyDict>>> {
        let mut events = self.events.lock().unwrap_or_else(|p| p.into_inner());
        match events.pop_front() {
            Some(ev) => Ok(Some(ev.to_pydict(py)?)),
            None => Ok(None),
        }
    }

    fn add_config_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_config_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    fn add_init_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_init_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    fn add_restart_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_restart_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    fn begin_config_phase(&self, mcu: u32) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .begin_config_phase(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }

    fn next_config_entry(&self, mcu: u32) -> PyResult<Option<Vec<u8>>> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .next_config_entry(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }

    fn get_stats(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let stats = router
            .get_stats(mcu_handle_from_raw(mcu))
            .map_err(router_err)?;
        stats_to_pydict(py, &stats)
    }

    fn set_msgproto_dict(&self, dict_json: &[u8]) -> PyResult<()> {
        let json_str = std::str::from_utf8(dict_json)
            .map_err(|e| PyRuntimeError::new_err(format!("dict_json utf8: {e}")))?;
        let dict: DataDictionary = serde_json::from_str(json_str)
            .map_err(|e| PyRuntimeError::new_err(format!("dict json parse: {e}")))?;
        let parser = MsgProtoParser::from_dictionary(dict)
            .map_err(|e| PyRuntimeError::new_err(format!("parser build: {e:?}")))?;
        *self.parser.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(parser));
        Ok(())
    }

    // Narrow fd-release hook for the serial arduino-reset path (MCU._disconnect
    // → serial.disconnect()). It only nils host_io/runtime_rx for one MCU; it
    // does NOT touch endpoint_conn/endpoint_process, so it cannot tear an
    // EtherCAT MCU down on its own. The authoritative full teardown is
    // `shutdown()`; detach_serial is harmless before it (shutdown is idempotent).
    fn detach_serial(&self, mcu_handle: u32) -> PyResult<()> {
        let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(conn) = mcus.get_mut(&mcu_handle) {
            conn.runtime_rx = None;
            conn.host_io = None;
        }
        Ok(())
    }

    #[pyo3(signature = (mcu_handle, serial_path, baud, timeout_s = 30.0, klippy_non_critical = false))]
    fn attach_serial(
        &self,
        mcu_handle: u32,
        serial_path: &str,
        baud: u32,
        timeout_s: f64,
        klippy_non_critical: bool,
    ) -> PyResult<()> {
        use std::time::{Duration, Instant};
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s);
        let effective_baud = if baud == 0 { 250_000 } else { baud };
        let config = KalicoHostIoConfig::default();

        {
            let existing_io: Option<Arc<KalicoHostIo>> = {
                let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                mcus.get(&mcu_handle)
                    .and_then(|conn| conn.host_io.as_ref().map(Arc::clone))
            };
            if let Some(io) = existing_io {
                if io.is_alive() {
                    log::info!(
                        "attach_serial: reusing existing connection for {serial_path} \
                         (reactor alive, skipping close/reopen)"
                    );

                    let runtime_rx = io.take_runtime_event_subscription().map_err(|e| {
                        PyRuntimeError::new_err(format!(
                            "attach_serial: runtime_event re-subscribe: {e:?}"
                        ))
                    })?;

                    let (kalico_native_supported, identify_caps) =
                        match io.kalico_identify(std::time::Duration::from_secs(5)) {
                            Ok(out) => {
                                log::info!(
                                    "attach_serial: kalico re-identified — \
                                     reset_epoch=0x{:08x} caps=0x{:016x}",
                                    out.reset_epoch,
                                    out.capabilities,
                                );
                                (true, out.capabilities)
                            }
                            Err(e) => {
                                log::warn!(
                                    "attach_serial: kalico_identify timed out on reuse \
                                     for {serial_path} ({e}); treating as Klipper-protocol-only"
                                );
                                (false, 0u64)
                            }
                        };

                    let runtime_caps = if kalico_native_supported {
                        match query_runtime_caps(&io, std::time::Duration::from_secs(2)) {
                            Ok(caps) => {
                                log::debug!(
                                    "[caps-trace] attach_serial reuse: runtime caps \
                                     for {serial_path}: total_piece_memory={}",
                                    caps.total_piece_memory,
                                );
                                Some(caps)
                            }
                            Err(e) => {
                                return Err(PyRuntimeError::new_err(format!(
                                    "attach_serial: QueryRuntimeCaps failed for {serial_path} \
                                     ({e}) — a kalico-native MCU must report runtime caps; \
                                     firmware is too old, mismatched, or not flashed. \
                                     Refusing to attach with guessed caps."
                                )));
                            }
                        }
                    } else {
                        None
                    };

                    let critical = kalico_native_supported && !klippy_non_critical;
                    io.set_critical(critical);
                    log::info!(
                        "attach_serial: reuse — {serial_path} criticality \
                         critical={critical} (kalico_native={kalico_native_supported} \
                         klippy_non_critical={klippy_non_critical})"
                    );

                    let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
                        PyRuntimeError::new_err(format!(
                            "attach_serial: unknown mcu_handle {mcu_handle}"
                        ))
                    })?;
                    conn.runtime_rx = Some(runtime_rx);
                    conn.runtime_caps = runtime_caps;
                    conn.identify_caps = identify_caps;
                    conn.kalico_native_supported = kalico_native_supported;
                    return Ok(());
                }
            }
        }

        let mcu_label: String = {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "attach_serial: unknown mcu_handle {mcu_handle} (claim_mcu not called)"
                ))
            })?;
            conn.runtime_rx = None;
            conn.host_io = None;
            conn.label.clone()
        };

        let is_pipe = baud == 0
            || serial_path.starts_with("/tmp/")
            || serial_path.starts_with("/dev/pts/")
            || serial_path.contains("klipper_host")
            || serial_path.contains("klipper_sim");

        let host_io = loop {
            let result = if is_pipe {
                #[cfg(target_family = "unix")]
                {
                    KalicoHostIo::open_pipe_with_config(serial_path, config.clone())
                }
                #[cfg(not(target_family = "unix"))]
                {
                    KalicoHostIo::open_with_config(serial_path, effective_baud, config.clone())
                }
            } else {
                KalicoHostIo::open_with_config(serial_path, effective_baud, config.clone())
            };
            match result {
                Ok(io) => break io,
                Err(e) => {
                    if Instant::now() >= deadline {
                        return Err(PyRuntimeError::new_err(format!(
                            "attach_serial: could not open {serial_path} within {timeout_s}s: {e}"
                        )));
                    }
                    log::warn!("attach_serial: retrying {serial_path}: {e}");
                    std::thread::sleep(Duration::from_millis(500));
                }
            }
        };

        let runtime_rx = host_io.take_runtime_event_subscription().map_err(|e| {
            PyRuntimeError::new_err(format!("attach_serial: runtime_event subscribe: {e:?}"))
        })?;

        let (kalico_native_supported, identify_caps) =
            match host_io.kalico_identify(std::time::Duration::from_secs(5)) {
                Ok(out) => {
                    log::info!(
                        "attach_serial: kalico identified — reset_epoch=0x{:08x} \
                         caps=0x{:016x}",
                        out.reset_epoch,
                        out.capabilities,
                    );
                    (true, out.capabilities)
                }
                Err(e) => {
                    log::warn!(
                        "attach_serial: kalico_identify timed out for {serial_path} ({e}); \
                         continuing attach as a Klipper-protocol-only MCU"
                    );
                    (false, 0u64)
                }
            };

        let runtime_caps = if kalico_native_supported {
            match query_runtime_caps(&host_io, std::time::Duration::from_secs(2)) {
                Ok(caps) => {
                    log::debug!(
                        "[caps-trace] attach_serial: runtime caps for {serial_path}: \
                         total_piece_memory={}",
                        caps.total_piece_memory,
                    );
                    Some(caps)
                }
                Err(e) => {
                    return Err(PyRuntimeError::new_err(format!(
                        "attach_serial: QueryRuntimeCaps failed for {serial_path} \
                         ({e}) — a kalico-native MCU must report runtime caps; \
                         firmware is too old, mismatched, or not flashed. \
                         Refusing to attach with guessed caps."
                    )));
                }
            }
        } else {
            None
        };

        let critical = kalico_native_supported && !klippy_non_critical;
        host_io.set_critical(critical);
        log::info!(
            "attach_serial: {serial_path} criticality critical={critical} \
             (kalico_native={kalico_native_supported} \
             klippy_non_critical={klippy_non_critical})"
        );

        let host_io_arc = Arc::new(host_io);

        {
            let events_dir_guard = self
                .events_dir
                .lock()
                .unwrap_or_else(std::sync::PoisonError::into_inner);
            require_events_dir_for_kalico_native(
                kalico_native_supported,
                events_dir_guard.as_deref(),
                &mcu_label,
            )
            .map_err(PyRuntimeError::new_err)?;
        }

        if kalico_native_supported {
            let events_dir_guard = self.events_dir.lock().unwrap_or_else(|p| p.into_inner());
            if let Some(ref dir) = *events_dir_guard {
                use crate::logging::writer::{
                    DEFAULT_BACKUP_COUNT, DEFAULT_MAX_BYTES, FSYNC_INTERVAL, RotatingJsonlWriter,
                };
                let source = mcu_label.clone();
                let jsonl_path = dir.join(format!("{source}.jsonl"));
                match RotatingJsonlWriter::new(
                    &jsonl_path,
                    DEFAULT_MAX_BYTES,
                    DEFAULT_BACKUP_COUNT,
                    FSYNC_INTERVAL,
                ) {
                    Ok(writer) => {
                        let arc_writer = Arc::new(Mutex::new(writer));
                        let mcu_h = mcu_handle_from_raw(mcu_handle);
                        let hook = crate::mcu_log::build_mcu_log_hook(
                            Arc::clone(&self.router),
                            mcu_h,
                            arc_writer,
                            source,
                        );
                        host_io_arc.set_mcu_log_hook(Box::new(hook));
                    }
                    Err(e) => {
                        log::warn!(
                            "attach_serial: mcu-log: failed to open {}: {e}",
                            jsonl_path.display()
                        );
                    }
                }
            } else {
                unreachable!(
                    "attach_serial: events_dir is None for a kalico-native MCU \
                     — require_events_dir_for_kalico_native should have \
                     rejected this call before reaching hook wiring"
                );
            }
        }

        let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!("attach_serial: unknown mcu_handle {mcu_handle}"))
        })?;
        conn.host_io = Some(host_io_arc);
        conn.runtime_rx = Some(runtime_rx);
        conn.runtime_caps = runtime_caps;
        conn.identify_caps = identify_caps;
        conn.kalico_native_supported = kalico_native_supported;
        Ok(())
    }

    fn get_mcu_capabilities(&self, mcu_handle: u32) -> PyResult<u64> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "get_mcu_capabilities: unknown mcu_handle {mcu_handle}"
            ))
        })?;
        Ok(conn.identify_caps)
    }

    fn ring_depth_for_axis(&self, mcu_handle: u32, axis: u8) -> PyResult<u16> {
        let configs = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        ring_depth_for_axis_inner(&configs, mcu_handle, axis).map_err(PyRuntimeError::new_err)
    }

    #[pyo3(signature = (mcu_handle, bus_id, rate, timeout_s = 5.0))]
    fn register_phase_bus(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        bus_id: u8,
        rate: u32,
        timeout_s: f64,
    ) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "register_phase_bus: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            if !conn.kalico_native_supported {
                return Ok(());
            }
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "register_phase_bus: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let msg = format!("runtime_register_phase_bus bus_id={bus_id} rate={rate}");
        let params = py.detach(|| -> PyResult<_> {
            use kalico_host_rt::transport::Transport;
            io.call(&msg, "kalico_register_phase_bus_response", timeout)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!("register_phase_bus: transport error: {e:?}"))
                })
        })?;
        let result = params.try_get_i32("result").ok_or_else(|| {
            PyRuntimeError::new_err(
                "register_phase_bus: response missing or non-integer result field",
            )
        })?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "register_phase_bus: MCU returned error {result} (bus_id={bus_id})"
            )));
        }
        Ok(())
    }

    #[pyo3(signature = (mcu_handle, motor_idx, bus_id, cs_pin_id, slot_idx, timeout_s = 5.0))]
    fn register_phase_motor(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        motor_idx: u8,
        bus_id: u8,
        cs_pin_id: u8,
        slot_idx: u8,
        timeout_s: f64,
    ) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "register_phase_motor: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            if !conn.kalico_native_supported {
                return Ok(());
            }
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "register_phase_motor: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let msg = format!(
            "runtime_register_phase_motor motor_idx={motor_idx} \
             bus_id={bus_id} cs_pin_id={cs_pin_id} slot_idx={slot_idx}"
        );
        let params = py.detach(|| -> PyResult<_> {
            use kalico_host_rt::transport::Transport;
            io.call(&msg, "kalico_register_phase_motor_response", timeout)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!("register_phase_motor: transport error: {e:?}"))
                })
        })?;
        let result = params.try_get_i32("result").ok_or_else(|| {
            PyRuntimeError::new_err(
                "register_phase_motor: response missing or non-integer result field",
            )
        })?;
        if result != 0 {
            return Err(PyRuntimeError::new_err(format!(
                "register_phase_motor: MCU returned error {result} \
                 (motor_idx={motor_idx} bus_id={bus_id} cs_pin_id={cs_pin_id})"
            )));
        }
        Ok(())
    }

    fn get_identify_data(&self, mcu_handle: u32) -> PyResult<Vec<u8>> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "get_identify_data: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "get_identify_data: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        Ok(io.raw_identify_bytes().to_vec())
    }

    #[pyo3(signature = (mcu_handle, msg, response, timeout_s = 5.0))]
    fn bridge_call(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        msg: &str,
        response: &str,
        timeout_s: f64,
    ) -> PyResult<Py<PyDict>> {
        use std::time::Duration;

        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!("bridge_call: unknown mcu_handle {mcu_handle}"))
            })?;
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "bridge_call: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };

        let msg_owned = msg.to_owned();
        let response_owned = response.to_owned();
        let params = py.detach(|| -> PyResult<_> {
            use kalico_host_rt::transport::Transport;
            io.call(
                &msg_owned,
                &response_owned,
                Duration::from_secs_f64(timeout_s),
            )
            .map_err(|e| PyRuntimeError::new_err(format!("bridge_call: {e}")))
        })?;

        let d = PyDict::new(py);
        for (k, v) in &params.fields {
            use kalico_host_rt::transport::MessageValue;
            match v {
                MessageValue::U32(n) => d.set_item(k, n)?,
                MessageValue::I32(n) => d.set_item(k, n)?,
                MessageValue::U64(n) => d.set_item(k, n)?,
                MessageValue::Bytes(b) => {
                    d.set_item(k, pyo3::types::PyBytes::new(py, b.as_slice()))?
                }
                MessageValue::String(s) => d.set_item(k, s)?,
            }
        }
        if params.sent_time_raw != 0.0 {
            d.set_item("#sent_time_raw", params.sent_time_raw)?;
            d.set_item("#receive_time_raw", params.recv_time_raw)?;
        }
        Ok(d.unbind())
    }

    fn take_runtime_event(&self, py: Python<'_>, mcu_handle: u32) -> PyResult<Option<Py<PyDict>>> {
        use kalico_host_rt::host_io::runtime_events::RuntimeEvent;
        use std::sync::mpsc::TryRecvError;

        let event = {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "take_runtime_event: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            match conn.runtime_rx.as_mut() {
                None => return Ok(None),
                Some(rx) => match rx.try_recv() {
                    Ok(ev) => ev,
                    Err(TryRecvError::Empty) => return Ok(None),
                    Err(TryRecvError::Disconnected) => return Ok(None),
                },
            }
        };

        let d = PyDict::new(py);
        match event {
            RuntimeEvent::Status(s) => {
                d.set_item("type", "status")?;
                d.set_item("engine_status", s.engine_status)?;
                d.set_item("queue_depth", s.queue_depth)?;
                d.set_item("current_segment_id", s.current_segment_id)?;
                d.set_item("last_fault", s.last_fault)?;
                d.set_item("fault_detail", s.fault_detail)?;
                d.set_item("retired_through_segment_id", s.retired_through_segment_id)?;
            }
            RuntimeEvent::CreditFreed(c) => {
                d.set_item("type", "credit_freed")?;
                d.set_item("retired_through_segment_id", c.retired_through_segment_id)?;
                d.set_item("free_slots", c.free_slots)?;
            }
            RuntimeEvent::Fault(f) => {
                d.set_item("type", "fault")?;
                d.set_item("fault_code", f.fault_code)?;
                d.set_item("fault_detail", f.fault_detail)?;
                d.set_item("segment_id", f.segment_id)?;
                d.set_item("synthesized", f.synthesized)?;
            }
            RuntimeEvent::Trace(_) => {
                return Ok(None);
            }
            RuntimeEvent::Heartbeat { .. } => {
                return Ok(None);
            }
            RuntimeEvent::EndstopTrip(t) => {
                d.set_item("type", "endstop_trip")?;
                d.set_item("endstop_id", t.endstop_id)?;
                d.set_item("trip_clock", t.trip_clock)?;
                self.handle_endstop_trip(mcu_handle, t.endstop_id, t.trip_clock);
            }
            RuntimeEvent::UnknownOutput { format, msg } => {
                d.set_item("type", "output")?;
                d.set_item("format", format)?;
                d.set_item("msg", msg)?;
            }
            RuntimeEvent::PassthroughResponse { name, params } => {
                d.set_item("type", "response")?;
                d.set_item("name", name)?;
                for (k, v) in &params.fields {
                    use kalico_host_rt::transport::MessageValue;
                    match v {
                        MessageValue::U32(n) => d.set_item(k, *n)?,
                        MessageValue::I32(n) => d.set_item(k, *n)?,
                        MessageValue::U64(n) => d.set_item(k, *n)?,
                        MessageValue::Bytes(b) => {
                            d.set_item(k, pyo3::types::PyBytes::new(py, b.as_slice()))?
                        }
                        MessageValue::String(s) => d.set_item(k, s)?,
                    }
                }
                if params.sent_time_raw != 0.0 {
                    d.set_item("#sent_time_raw", params.sent_time_raw)?;
                    d.set_item("#receive_time_raw", params.recv_time_raw)?;
                }
            }
            RuntimeEvent::McuLog(_) => {
                return Ok(None);
            }
        }
        Ok(Some(d.unbind()))
    }

    fn bridge_get_clock_async(&self, mcu_handle: u32) -> PyResult<()> {
        let io =
            {
                let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                    pyo3::exceptions::PyRuntimeError::new_err(format!(
                        "bridge_get_clock_async: unknown mcu_handle {mcu_handle}"
                    ))
                })?;
                conn.host_io.as_ref().ok_or_else(|| {
                pyo3::exceptions::PyRuntimeError::new_err(
                    "bridge_get_clock_async: attach_serial has not been called for this MCU",
                )
            })?.clone()
            };

        io.get_clock_async().map_err(|e| {
            pyo3::exceptions::PyRuntimeError::new_err(format!("bridge_get_clock_async: {e}"))
        })
    }

    #[pyo3(signature = (mcu_handle, msg))]
    fn bridge_send(&self, mcu_handle: u32, msg: &str) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!("bridge_send: unknown mcu_handle {mcu_handle}"))
            })?;
            conn.host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "bridge_send: attach_serial has not been called for this MCU",
                    )
                })?
                .clone()
        };
        io.send_fire_and_forget(msg)
            .map_err(|e| PyRuntimeError::new_err(format!("bridge_send: {e}")))
    }

    fn bridge_mark_expected_disconnect(&self, mcu_handle: u32) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "bridge_mark_expected_disconnect: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            conn.host_io.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err(
                    "bridge_mark_expected_disconnect: attach_serial has not been called for this MCU",
                )
            })?.clone()
        };
        io.mark_expected_disconnect()
            .map_err(|e| PyRuntimeError::new_err(format!("bridge_mark_expected_disconnect: {e}")))
    }

    #[pyo3(signature = (mcu, freq, offset, last_clock, host_now_raw))]
    fn set_clock_est(
        &self,
        _py: Python<'_>,
        mcu: u32,
        freq: f64,
        offset: f64,
        last_clock: u64,
        host_now_raw: f64,
    ) -> PyResult<()> {
        self.clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, freq);

        use std::sync::atomic::{AtomicUsize, Ordering as AOrd};
        static SET_CLOCK_EST_CALLS: AtomicUsize = AtomicUsize::new(0);
        let call_n = SET_CLOCK_EST_CALLS.fetch_add(1, AOrd::Relaxed);
        if call_n < 5 || call_n % 100 == 0 {
            log::debug!(
                "[bridge-trace] set_clock_est call#{} mcu={} freq={} offset={:.6} last_clock={}",
                call_n,
                mcu,
                freq as u64,
                offset,
                last_clock,
            );
        }
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .set_clock_est_rebased(
                mcu_handle_from_raw(mcu),
                freq,
                offset,
                last_clock,
                host_now_raw,
            )
            .map_err(router_err)?;
        Ok(())
    }

    #[pyo3(signature = (mcu, freq_hz))]
    fn set_nominal_clock_freq(&self, mcu: u32, freq_hz: u32) -> PyResult<()> {
        if freq_hz == 0 {
            return Err(PyRuntimeError::new_err(
                "set_nominal_clock_freq: freq_hz must be nonzero",
            ));
        }
        self.nominal_clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, freq_hz);
        Ok(())
    }

    fn extract_old(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let (sent, received) = router
            .extract_old(mcu_handle_from_raw(mcu))
            .map_err(router_err)?;

        let d = PyDict::new(py);

        let sent_list: Vec<Py<PyDict>> = sent
            .iter()
            .map(|e| {
                let ed = PyDict::new(py);
                ed.set_item("seq", e.seq).unwrap();
                ed.set_item("data", pyo3::types::PyBytes::new(py, &e.bytes))
                    .unwrap();
                ed.set_item("timestamp", e.timestamp).unwrap();
                ed.unbind()
            })
            .collect();

        let received_list: Vec<Py<PyDict>> = received
            .iter()
            .map(|e| {
                let ed = PyDict::new(py);
                ed.set_item("seq", e.seq).unwrap();
                ed.set_item("data", pyo3::types::PyBytes::new(py, &e.bytes))
                    .unwrap();
                ed.set_item("timestamp", e.timestamp).unwrap();
                ed.unbind()
            })
            .collect();

        d.set_item("sent", sent_list)?;
        d.set_item("received", received_list)?;
        Ok(d.unbind())
    }

    #[pyo3(signature = (
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
        square_corner_velocity,
        shaper_type_x,
        shaper_freq_x,
        shaper_type_y,
        shaper_freq_y,
        mcus,
        window_capacity = 32,
        beta_max_iters = 10,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn init_planner(
        &self,
        max_velocity: f64,
        max_accel: f64,
        max_z_velocity: f64,
        max_z_accel: f64,
        square_corner_velocity: f64,
        shaper_type_x: &str,
        shaper_freq_x: f64,
        shaper_type_y: &str,
        shaper_freq_y: f64,
        mcus: Vec<(u32, Vec<u8>, u8)>,
        window_capacity: usize,
        beta_max_iters: u8,
    ) -> PyResult<()> {
        if self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .is_some()
        {
            return Err(PyRuntimeError::new_err("planner already initialized"));
        }

        let shaper =
            build_shaper_config(shaper_type_x, shaper_freq_x, shaper_type_y, shaper_freq_y)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

        let limits = PlannerLimits {
            max_velocity,
            max_accel,
            max_z_velocity,
            max_z_accel,
            square_corner_velocity,
        };

        let mut cfg = config::PlannerConfig::default();
        cfg.limits = limits;
        cfg.shaper = shaper;
        cfg.window_capacity = window_capacity;
        cfg.beta_max_iters = beta_max_iters;

        *self
            .planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = cfg.clone();

        let ec_conns: HashMap<u32, Arc<UnixNativeConn>> = {
            // Collect (handle, conn, socket_path) in one lock acquisition to
            // close the release_mcu race window between separate lookups.
            let ethercat_handles: Vec<(u32, Arc<UnixNativeConn>, String)> = {
                let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                mcus.iter()
                    .filter_map(|(handle, _, _)| {
                        let c = mcus_lock.get(handle)?;
                        let socket = c.ethercat_socket.as_ref()?;
                        let conn = c.endpoint_conn.as_ref()?.clone();
                        Some((*handle, conn, socket.clone()))
                    })
                    .collect()
            };

            let mut out = HashMap::new();
            for (mcu_id, conn, socket) in ethercat_handles {
                let caps = query_ethercat_runtime_caps(&conn, std::time::Duration::from_secs(5))
                    .map_err(|e| {
                        PyRuntimeError::new_err(format!(
                            "init_planner: QueryRuntimeCaps failed for ethercat mcu \
                                 {mcu_id} ({socket}): {e} — endpoint must respond with \
                                 RuntimeCapsResponse; is kalico-ethercat-rt running?"
                        ))
                    })?;
                log::debug!(
                    "[caps-trace] init_planner: ethercat mcu {mcu_id} caps \
                     total_piece_memory={}",
                    caps.total_piece_memory,
                );
                {
                    let mut mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    if let Some(c) = mcus_lock.get_mut(&mcu_id) {
                        c.runtime_caps = Some(caps);
                    }
                }
                out.insert(mcu_id, conn);
            }
            out
        };

        let caps_by_handle: std::collections::HashMap<u32, McuCaps> = {
            let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.iter()
                .map(|(handle, _, _)| {
                    let caps = mcus_lock
                        .get(handle)
                        .and_then(|c| c.runtime_caps)
                        .map(McuCaps::from)
                        .unwrap_or_default();
                    (*handle, caps)
                })
                .collect()
        };
        let mcu_configs = build_mcu_configs(&mcus, &caps_by_handle);
        *self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = mcu_configs.clone();

        let counter = Arc::clone(&self.dispatched_segments);
        let router_arc = Arc::clone(&self.router);

        let ethercat_mcu_ids: HashSet<u32> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcu_configs
                .iter()
                .filter(|c| {
                    mcus.get(&c.mcu_id)
                        .map_or(false, |conn| conn.ethercat_socket.is_some())
                })
                .map(|c| c.mcu_id)
                .collect()
        };

        let host_ios: HashMap<u32, Arc<KalicoHostIo>> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let mut out = HashMap::new();
            for cfg_mcu in &mcu_configs {
                if ethercat_mcu_ids.contains(&cfg_mcu.mcu_id) {
                    continue;
                }
                let conn = mcus.get(&cfg_mcu.mcu_id).ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: unknown mcu_handle {}",
                        cfg_mcu.mcu_id
                    ))
                })?;
                let io = conn.host_io.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err(format!(
                        "init_planner: attach_serial has not been called for MCU {}",
                        cfg_mcu.mcu_id
                    ))
                })?;
                out.insert(cfg_mcu.mcu_id, Arc::clone(io));
            }
            out
        };

        let ring_depth_table: HashMap<crate::pump::AxisKey, u32> = {
            let mut t = HashMap::new();
            for cfg_mcu in &mcu_configs {
                let total = cfg_mcu.caps.total_pieces() as u32;
                let n = cfg_mcu.axes.len() as u32;
                let depth = axis_ring_depth(total, n);
                for &axis in &cfg_mcu.axes {
                    t.insert(
                        crate::pump::AxisKey {
                            mcu_id: cfg_mcu.mcu_id,
                            axis: axis as u8,
                        },
                        depth,
                    );
                }
            }
            t
        };

        {
            let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            let now_ns = crate::motion_node::monotonic_ns();
            for &mcu_id in &ethercat_mcu_ids {
                let mcu_h = mcu_handle_from_raw(mcu_id);
                let _ = router.set_clock_est_from_sample(
                    mcu_h,
                    f64::from(ETHERCAT_CLOCK_FREQ_HZ),
                    Instant::now(),
                    now_ns,
                );
            }
        }

        let (pump_tx_init, pump_rx) = std::sync::mpsc::channel::<crate::pump::PumpMsg>();

        let wire_transports: HashMap<u32, crate::pump::McuTransport> = {
            let mut t = HashMap::new();
            for (&id, io) in &host_ios {
                t.insert(id, crate::pump::McuTransport::Serial(Arc::downgrade(io)));
            }
            for (&id, conn) in &ec_conns {
                t.insert(
                    id,
                    crate::pump::McuTransport::EtherCat(Arc::downgrade(conn)),
                );
            }
            t
        };

        let pump_timeout = Duration::from_secs(5);
        let ring_depth_table_for_pump = ring_depth_table.clone();
        let router_for_pump = Arc::clone(&self.router);
        let drain_for_pump = self.drain.clone();
        let router_for_freq = Arc::clone(&self.router);
        let pump_thread_handle = std::thread::Builder::new()
            .name("push-pieces-pump".into())
            .spawn(move || {
                let sink = crate::pump::WireSink {
                    transports: wire_transports,
                    timeout: pump_timeout,
                    freq_of: Arc::new(move |mcu_id: u32| {
                        let r = router_for_freq.lock().unwrap_or_else(|p| p.into_inner());
                        r.ack_clock_and_freq(mcu_handle_from_raw(mcu_id))
                            .map(|(_, f)| f)
                    }),
                };
                crate::pump::run_pump(
                    pump_rx,
                    sink,
                    move |k| {
                        ring_depth_table_for_pump
                            .get(&k)
                            .copied()
                            .unwrap_or_else(|| {
                                log::error!(
                                    "pump: no ring_depth for {k:?} — axis absent from \
                                 init_planner config; using sentinel depth 1 \
                                 (expect PieceStartInPast fault)"
                                );
                                1
                            })
                    },
                    move |mcu_id: u32| {
                        let r = router_for_pump.lock().unwrap_or_else(|p| p.into_inner());
                        r.ack_clock_and_freq(mcu_handle_from_raw(mcu_id))
                    },
                    |key| {
                        tracing::error!(
                            mcu_id = key.mcu_id,
                            axis = key.axis,
                            "EXIT_ON_FAULT — EtherCAT transport broken-pipe in pump; \
                             aborting klippy so systemd restarts it"
                        );
                        abort_after_tracing_appender_drains();
                    },
                    move |key: crate::pump::AxisKey, n: u32| {
                        drain_for_pump.unsend(key.mcu_id, key.axis, n);
                    },
                    |msg: String| {
                        tracing::error!(
                            msg,
                            "EXIT_ON_FAULT — drip cohort stalled; \
                             aborting klippy so systemd restarts it"
                        );
                        abort_after_tracing_appender_drains();
                    },
                );
            })
            .expect("spawn push-pieces-pump thread");

        *self.pump_tx.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_tx_init.clone());
        *self.pump_thread.lock().unwrap_or_else(|p| p.into_inner()) = Some(pump_thread_handle);

        for cfg_mcu in &mcu_configs {
            let mcu_id = cfg_mcu.mcu_id;
            let pump_tx_hb = pump_tx_init.clone();
            let drain_hb = self.drain.clone();

            if ethercat_mcu_ids.contains(&mcu_id) {
                let conn = ec_conns
                    .get(&mcu_id)
                    .expect("ec_conns built from ethercat_mcu_ids")
                    .clone();

                let mcu_label = {
                    let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    mcus_lock
                        .get(&mcu_id)
                        .map(|c| c.label.clone())
                        .unwrap_or_else(|| format!("mcu-{mcu_id}"))
                };

                let homing_run_hb = Arc::clone(&self.homing_run);
                let active_cohort_hb = Arc::clone(&self.active_drip_cohort);
                let pump_tx_fault = pump_tx_init.clone();
                let latched_fault_hb = Arc::clone(&self.latched_drive_fault);
                let mcu_label_hb = mcu_label.clone();
                conn.attach_heartbeat_callback(Arc::new(
                    move |hb: &kalico_protocol::messages::StatusHeartbeat| {
                        if hb.fault_code != 0 {
                            let run_opt = {
                                let mut guard =
                                    homing_run_hb.lock().unwrap_or_else(|p| p.into_inner());
                                match guard.as_ref().map(|r| r.axis_key.mcu_id) {
                                    Some(axis_mcu)
                                        if crate::homing::route_drive_fault(
                                            mcu_id,
                                            Some(axis_mcu),
                                        ) == crate::homing::DriveFaultRoute::HomingError =>
                                    {
                                        guard.take()
                                    }
                                    _ => None,
                                }
                            };
                            match run_opt {
                                Some(run) => {
                                    latched_fault_hb
                                        .lock()
                                        .unwrap_or_else(|p| p.into_inner())
                                        .insert(mcu_id, hb.fault_code);
                                    *active_cohort_hb.lock().unwrap_or_else(|p| p.into_inner()) =
                                        None;
                                    let _ = pump_tx_fault.send(crate::pump::PumpMsg::Flush(
                                        run.all_axis_keys.clone(),
                                    ));
                                    let _ = pump_tx_fault
                                        .send(crate::pump::PumpMsg::DripDisarm(run.cohort));
                                    let _ = run.notify.send(Err(format!(
                                        "drive fault 0x{:04x} during homing — \
                                     following-error/torque limit exceeded (endstop failure?)",
                                        hb.fault_code
                                    )));
                                }
                                None => {
                                    let prev = latched_fault_hb
                                        .lock()
                                        .unwrap_or_else(|p| p.into_inner())
                                        .insert(mcu_id, hb.fault_code);
                                    if prev != Some(hb.fault_code) {
                                        tracing::error!(
                                            mcu_id,
                                            mcu_label = %mcu_label_hb,
                                            fault_code = hb.fault_code,
                                            "ethercat drive fault — latched for klippy to report"
                                        );
                                    }
                                }
                            }
                            return;
                        }
                        let _ = pump_tx_hb.send(crate::pump::PumpMsg::Heartbeat(
                            crate::pump::HeartbeatMsg {
                                mcu_id,
                                retired_counts: hb.retired_counts.clone(),
                            },
                        ));
                        for (axis, &r) in hb.retired_counts.iter().enumerate() {
                            drain_hb.set_retired(mcu_id, axis as u8, r);
                        }
                    },
                ));

                // Weak so the supervision thread never keeps the conn (and its
                // reader thread / socket) alive past release_mcu: when the last
                // strong Arc drops, upgrade() fails and the thread exits quietly,
                // letting Drop run shutdown(Both)+join. A strong Arc here would
                // pin the reader thread until this loop happened to notice the
                // release, leaking finished-but-unjoined readers across repeated
                // standalone claim/release.
                let conn_for_poll = Arc::downgrade(&conn);
                let mcus_for_supervision = Arc::clone(&self.mcus);
                let label_for_supervision = mcu_label.clone();
                let on_endpoint_death: Box<dyn Fn(&str) + Send + 'static> =
                    Box::new(move |reason: &str| {
                        tracing::error!(
                            mcu_label = label_for_supervision,
                            mcu_id,
                            reason,
                            "EXIT_ON_FAULT — ethercat endpoint died mid-session; \
                             aborting klippy so systemd restarts it"
                        );
                        abort_after_tracing_appender_drains();
                    });

                let _ = std::thread::Builder::new()
                    .name(format!("ec-heartbeat-poll-{mcu_id}"))
                    .spawn(move || {
                        loop {
                            // Released conn -> exit quietly. This is the common
                            // case: release_mcu drops the last strong Arc, the
                            // upgrade fails, and the thread exits before probing.
                            // The residual race — upgrading the Weak while the conn
                            // is still strong but the MCU was already removed from
                            // the map — is closed by the mcus-map re-check below,
                            // which confirms every fault under the lock.
                            let Some(conn) = conn_for_poll.upgrade() else {
                                return;
                            };

                            // The reader thread sets peer_closed on EOF/IO; no poll here.
                            let peer_eof = conn.peer_closed();
                            drop(conn);

                            // Both fault probes (EOF and child-exit) are confirmed
                            // against the mcus map under one lock acquisition, so a
                            // deliberate release can never be misread as a fault.
                            // release_mcu removes the McuConnection from the map
                            // BEFORE it closes the endpoint socket; that socket
                            // close is exactly what sets peer_closed(). So if we
                            // upgraded the Weak in the race window where the conn
                            // was still strong but the MCU was already removed,
                            // `mcus.get(&mcu_id)` is None here and we exit quietly
                            // instead of firing EXIT_ON_FAULT.
                            let fault_reason = {
                                let mut mcus = mcus_for_supervision
                                    .lock()
                                    .unwrap_or_else(|p| p.into_inner());
                                let Some(c) = mcus.get_mut(&mcu_id) else {
                                    // MCU was released — normal shutdown, exit quietly.
                                    return;
                                };
                                if peer_eof {
                                    Some("conn EOF".to_string())
                                } else if let Some(ref mut child) = c.endpoint_process {
                                    match child.try_wait() {
                                        Ok(Some(status)) => Some(format!("child exited: {status}")),
                                        Ok(None) => None,
                                        Err(e) => Some(format!("try_wait error: {e}")),
                                    }
                                } else {
                                    None
                                }
                            };

                            if let Some(reason) = fault_reason {
                                on_endpoint_death(&reason);
                                return;
                            }

                            std::thread::sleep(Duration::from_millis(1));
                        }
                    })
                    .expect("spawn ec-heartbeat-poll thread");
            } else {
                let io = host_ios
                    .get(&mcu_id)
                    .expect("host_io map built from mcu_configs")
                    .clone();
                io.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
                    let _ = pump_tx_hb.send(crate::pump::PumpMsg::Heartbeat(
                        crate::pump::HeartbeatMsg {
                            mcu_id,
                            retired_counts: retired.to_vec(),
                        },
                    ));
                    for (axis, &r) in retired.iter().enumerate() {
                        drain_hb.set_retired(mcu_id, axis as u8, r);
                    }
                }));
            }
        }

        let mcu_configs_for_cb = mcu_configs;
        let router_for_cb = Arc::clone(&router_arc);

        let anchor_mutex = std::sync::Mutex::new(crate::anchor::Anchor::new());
        let pump_tx_for_cb = pump_tx_init.clone();
        let drain_disp = self.drain.clone();
        let counter_for_cb = Arc::clone(&counter);
        let active_drip_cohort_for_cb = Arc::clone(&self.active_drip_cohort);
        let motion_history_for_cb = Arc::clone(&self.motion_history);
        let nominal_freqs_for_cb = Arc::clone(&self.nominal_clock_freqs);

        let dispatch: Arc<
            dyn Fn(&trajectory::ShapedSegment) -> Result<(), DispatchError> + Send + Sync,
        > = Arc::new(
            move |seg: &trajectory::ShapedSegment| -> Result<(), DispatchError> {
                log::debug!(
                    "[bridge-trace] dispatch entered: seg.t_start={:.6} seg.t_end={:.6}",
                    seg.t_start,
                    seg.t_end,
                );

                let host_now = {
                    let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
                    r.host_now_secs()
                };

                let (t0, fresh) = anchor_mutex
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .anchor_segment(seg.t_start, seg.t_end, host_now)
                    .map_err(|late| DispatchError::SegmentLate {
                        gap_s: late.gap_s,
                        seg_t_start: late.seg_t_start,
                    })?;

                if fresh {
                    let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
                    for cfg in mcu_configs_for_cb.iter() {
                        let h = crate::types::mcu_handle_from_raw(cfg.mcu_id);
                        r.log_seg0_deficit(h, t0 + seg.t_start, t0);
                    }
                }

                let project = |mcu_id: u32, host_secs: f64| -> u64 {
                    let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
                    r.host_time_to_mcu_clock(crate::types::mcu_handle_from_raw(mcu_id), host_secs)
                        .unwrap_or(0)
                };

                let active_cohort: Option<u64> = *active_drip_cohort_for_cb
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());

                let max_piece_secs = if active_cohort.is_some() {
                    Some(0.025_f64)
                } else {
                    None::<f64>
                };
                let lead_secs = if active_cohort.is_some() {
                    crate::pump::DRIP_WINDOW_SECS
                } else {
                    crate::pump::MAX_LEAD_SECS
                };

                let msgs = crate::enqueue::enqueue_segment(
                    seg,
                    &mcu_configs_for_cb,
                    t0,
                    fresh,
                    host_now,
                    lead_secs,
                    project,
                    max_piece_secs,
                );

                let nominal_freqs = nominal_freqs_for_cb
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .clone();
                for m in msgs {
                    let nominal_freq = *nominal_freqs
                        .get(&m.key.mcu_id)
                        .ok_or(DispatchError::MissingNominalFreq(m.key.mcu_id))?;
                    {
                        let mut store = motion_history_for_cb
                            .lock()
                            .unwrap_or_else(|p| p.into_inner());
                        for (piece, _host_t) in &m.pieces {
                            store.record(m.key, piece, nominal_freq);
                        }
                    }
                    drain_disp.add_sent(m.key.mcu_id, m.key.axis, m.pieces.len() as u32);
                    pump_tx_for_cb
                        .send(crate::pump::PumpMsg::Enqueue(m))
                        .map_err(|_| DispatchError::PumpGone)?;
                }

                counter_for_cb.fetch_add(1, Ordering::Relaxed);
                Ok(())
            },
        );

        {
            let mut guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
            if guard.is_some() {
                return Err(PyRuntimeError::new_err(
                    "planner already initialized (raced)",
                ));
            }
            *guard = Some(PlannerHandle::spawn(cfg, dispatch));
        }
        Ok(())
    }

    #[pyo3(signature = (dx, dy, dz, de, feedrate))]
    fn submit_move(
        &self,
        py: Python<'_>,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        tracing::debug!(
            subsystem = "motion",
            event = "submit_move_enter",
            dx,
            dy,
            dz,
            de,
            feedrate,
            "bridge.submit_move enter"
        );
        py.detach(|| -> PyResult<()> {
            let pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            let classified = classify::classify_and_build(pos, dx, dy, dz, de, feedrate)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

            {
                let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
                let planner = guard.as_ref().ok_or_else(|| {
                    PyRuntimeError::new_err("planner not initialized — call init_planner first")
                })?;
                planner.submit_move(classified).map_err(planner_err)?;
            }

            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            pos[0] += dx;
            pos[1] += dy;
            pos[2] += dz;
            Ok(())
        })
    }

    fn wait_moves(&self, py: Python<'_>) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)
    }

    fn drain_motion(&self, py: Python<'_>) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)?;
        let drain = self.drain.clone();
        py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
            .map_err(PyRuntimeError::new_err)
    }

    fn motion_drain_poll(&self, py: Python<'_>) -> PyResult<bool> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.detach(|| planner.flush()).map_err(planner_err)?;
        Ok(self.drain.is_drained_now())
    }

    fn motion_drain_finalize(&self) {}

    fn submit_dwell(&self, duration_s: f64) -> PyResult<()> {
        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.dwell(duration_s).map_err(planner_err)
    }

    #[pyo3(signature = (x, y, z, host_now))]
    fn set_position(&self, py: Python<'_>, x: f64, y: f64, z: f64, host_now: f64) -> PyResult<()> {
        {
            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            *pos = [x, y, z];
        }
        let planner_guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(planner) = planner_guard.as_ref() {
            py.detach(|| planner.flush()).map_err(planner_err)?;
            {
                let drain = self.drain.clone();
                py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
                    .map_err(PyRuntimeError::new_err)?;
            }

            planner
                .kalico_stream_open([x, y, z, 0.0])
                .map_err(planner_err)?;

            self.drain.reset();

            let sends = {
                let configs = self
                    .mcu_axis_configs
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                let ethercat_mcu_ids: HashSet<u32> = configs
                    .iter()
                    .filter(|c| {
                        mcus.get(&c.mcu_id)
                            .map_or(false, |conn| conn.ethercat_socket.is_some())
                    })
                    .map(|c| c.mcu_id)
                    .collect();
                crate::dispatch::build_serial_seed_sends(&configs, &ethercat_mcu_ids, x, y, z)
            };
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            for s in sends {
                let conn = mcus.get(&s.mcu_id).unwrap_or_else(|| {
                    panic!(
                        "set_position seed: planner up but mcu_id {} absent \
                         (broken invariant)",
                        s.mcu_id
                    )
                });
                let io = conn.host_io.as_ref().unwrap_or_else(|| {
                    panic!(
                        "set_position seed: serial mcu_id {} has no host_io \
                         (broken invariant — attach_serial not called?)",
                        s.mcu_id
                    )
                });
                io.send_typed(
                    "runtime_seed_position",
                    &[
                        ("x_q16", FieldValue::I32(s.x_q16)),
                        ("y_q16", FieldValue::I32(s.y_q16)),
                        ("z_q16", FieldValue::I32(s.z_q16)),
                    ],
                )
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "set_position seed send to mcu_id {} failed: {e:?}",
                        s.mcu_id
                    ))
                })?;
            }
        }

        {
            let configs: Vec<crate::dispatch::McuAxisConfig> = self
                .mcu_axis_configs
                .lock()
                .unwrap_or_else(|p| p.into_inner())
                .clone();
            let positions = [x, y, z];
            let rebases: Vec<(crate::pump::AxisKey, u64, f64)> = {
                let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
                configs
                    .iter()
                    .flat_map(|cfg| {
                        let handle = crate::types::mcu_handle_from_raw(cfg.mcu_id);
                        let now_clock =
                            router.host_time_to_mcu_clock(handle, host_now).unwrap_or(0);
                        cfg.axes
                            .iter()
                            .filter(|&&a| a < 3)
                            .map(move |&axis| {
                                let key = crate::pump::AxisKey {
                                    mcu_id: cfg.mcu_id,
                                    axis: axis as u8,
                                };
                                (key, now_clock, positions[axis])
                            })
                            .collect::<Vec<_>>()
                    })
                    .collect()
            };
            {
                let mut store = self
                    .motion_history
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                for (key, now_clock, pos) in rebases {
                    store.rebase_axis(key, now_clock, pos);
                }
            }
        }

        Ok(())
    }

    fn update_limits(&self, max_velocity: f64, max_accel: f64) -> PyResult<()> {
        let mut cfg = self
            .planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        cfg.limits.max_velocity = max_velocity;
        cfg.limits.max_accel = max_accel;
        let new_limits = cfg.limits;
        drop(cfg);

        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.update_limits(new_limits).map_err(planner_err)
    }

    fn update_shaper(
        &self,
        shaper_type_x: &str,
        freq_x: f64,
        shaper_type_y: &str,
        freq_y: f64,
    ) -> PyResult<()> {
        let shaper = build_shaper_config(shaper_type_x, freq_x, shaper_type_y, freq_y)
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

        self.planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .shaper = shaper.clone();

        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.update_shaper(shaper).map_err(planner_err)
    }

    fn get_last_move_time(&self) -> f64 {
        match self
            .planner
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .as_ref()
        {
            Some(p) => p.last_move_time(),
            None => 0.0,
        }
    }

    fn dispatched_segment_count(&self) -> u64 {
        self.dispatched_segments.load(Ordering::Relaxed)
    }

    fn fallback_clock_conversions(&self) -> u64 {
        self.fallback_clock_conversions.load(Ordering::Relaxed)
    }

    #[pyo3(signature = (axis, direction, speed_mm_s, max_travel_mm, endstop_id, endstop_mcu))]
    #[allow(clippy::too_many_arguments)]
    fn home_axis_start(
        &self,
        py: Python<'_>,
        axis: u8,
        direction: f64,
        speed_mm_s: f64,
        max_travel_mm: f64,
        endstop_id: u8,
        endstop_mcu: u32,
    ) -> PyResult<()> {
        use crate::planner::HomeDripParams;

        if axis > 2 {
            return Err(PyRuntimeError::new_err(format!(
                "home_axis: axis {axis} out of range (0=X, 1=Y, 2=Z)"
            )));
        }

        let guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        let planner = guard
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("home_axis: planner not initialized"))?;

        let (all_axis_keys, _axis_mcu, axis_key) = {
            let configs = self
                .mcu_axis_configs
                .lock()
                .unwrap_or_else(|p| p.into_inner());

            let all_keys = drip_cohort_participants(&configs);
            let found_mcu = configs
                .iter()
                .find(|cfg| cfg.axes.iter().any(|&a| a == axis as usize))
                .map(|cfg| cfg.mcu_id);

            let mcu = found_mcu.ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "home_axis: axis {axis} not found in mcu_axis_configs \
                     (init_planner not called?)"
                ))
            })?;
            let key = crate::pump::AxisKey { mcu_id: mcu, axis };
            (all_keys, mcu, key)
        };

        let cohort: u64 = {
            use std::sync::atomic::AtomicU64;
            static SEQ: AtomicU64 = AtomicU64::new(1);
            SEQ.fetch_add(1, Ordering::Relaxed)
        };

        let start_pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());

        {
            let mut latched = self
                .latched_drive_fault
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            latched.remove(&axis_key.mcu_id);
        }

        {
            let drain = self.drain.clone();
            py.detach(|| drain.wait_drained(DRAIN_TIMEOUT))
                .map_err(PyRuntimeError::new_err)?;
        }

        let window_start_clock = {
            let store = self
                .motion_history
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            store.last_endpoint_clock(axis_key)
        };

        {
            let mut cohort_guard = self
                .active_drip_cohort
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            *cohort_guard = Some(cohort);
        }

        let pump_tx = self
            .pump_tx
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
            .ok_or_else(|| PyRuntimeError::new_err("home_axis: pump not started"))?;

        pump_tx
            .send(crate::pump::PumpMsg::DripArm(crate::pump::DripArm {
                cohort,
                participants: all_axis_keys.clone(),
                timeout: Duration::from_secs(5),
            }))
            .map_err(|_| PyRuntimeError::new_err("home_axis: pump channel closed"))?;

        let (result_tx, result_rx) =
            crossbeam_channel::bounded::<Result<([f64; 3], [f64; 3], u64), String>>(1);

        {
            let mut run = self.homing_run.lock().unwrap_or_else(|p| p.into_inner());
            *run = Some(HomingRun {
                cohort,
                endstop_id,
                endstop_mcu,
                axis,
                axis_key,
                all_axis_keys: all_axis_keys.clone(),
                window_start_clock,
                notify: result_tx,
            });
        }

        let home_pos_4 = [start_pos[0], start_pos[1], start_pos[2], 0.0];

        let (planner_done_tx, planner_done_rx) =
            crossbeam_channel::bounded::<Result<(), String>>(1);
        planner
            .home_drip(HomeDripParams {
                home_pos: home_pos_4,
                start: start_pos,
                axis,
                direction,
                speed_mm_s,
                max_travel_mm,
                cohort,
                participants: all_axis_keys.clone(),
                notify: planner_done_tx,
            })
            .map_err(|e| {
                self.finish_homing();
                planner_err(e)
            })?;

        let dispatch = py.detach(|| {
            planner_done_rx
                .recv_timeout(Duration::from_secs(5))
                .map_err(|_| "home_axis: planner timed out dispatching homing move".to_owned())
                .and_then(|r| r)
        });
        if let Err(e) = dispatch {
            self.finish_homing();
            return Err(PyRuntimeError::new_err(e));
        }

        *self.homing_result.lock().unwrap_or_else(|p| p.into_inner()) = Some(result_rx);
        Ok(())
    }

    fn home_axis_poll(&self) -> PyResult<Option<([f64; 3], [f64; 3], u64)>> {
        let rx = {
            let guard = self.homing_result.lock().unwrap_or_else(|p| p.into_inner());
            match guard.as_ref() {
                Some(rx) => rx.clone(),
                None => {
                    return Err(PyRuntimeError::new_err(
                        "home_axis_poll: no homing in progress",
                    ));
                }
            }
        };
        match rx.try_recv() {
            Err(crossbeam_channel::TryRecvError::Empty) => Ok(None),
            Err(crossbeam_channel::TryRecvError::Disconnected) => {
                self.finish_homing();
                Err(PyRuntimeError::new_err(
                    "home_axis_poll: homing result channel closed",
                ))
            }
            Ok(result) => {
                self.finish_homing();
                let (trip_pos, final_pos, trip_clock) = result.map_err(PyRuntimeError::new_err)?;
                *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner()) = final_pos;
                Ok(Some((trip_pos, final_pos, trip_clock)))
            }
        }
    }

    fn arm_remote_trigger(&self, mcu_handle: u32, trsync_oid: u32, endstop_id: u8) -> PyResult<()> {
        {
            let armed = self
                .remote_triggers
                .lock()
                .unwrap_or_else(|p| p.into_inner());
            if armed.contains_key(&endstop_id) {
                return Err(PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: endstop_id {endstop_id} is already armed"
                )));
            }
        }
        let host_io = self
            .mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu_handle)
            .and_then(|c| c.host_io.as_ref().map(Arc::clone))
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: mcu {mcu_handle} has no serial transport"
                ))
            })?;
        let deps = self.trip_deps();
        let router = Arc::clone(&self.router);
        let fired = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let id = host_io
            .register_frame_interceptor(
                "trsync_state",
                Some(trsync_oid),
                Box::new(move |params| {
                    let decision = crate::remote_trigger::relay_decision(
                        params.try_get_u32("can_trigger"),
                        fired.load(Ordering::SeqCst),
                    );
                    if decision != crate::remote_trigger::RelayAction::Fire {
                        return;
                    }
                    fired.store(true, Ordering::SeqCst);
                    let clock32 = params.try_get_u32("clock").unwrap_or(0);
                    let reference = router
                        .lock()
                        .unwrap_or_else(|p| p.into_inner())
                        .compute_ack_clock(kalico_host_rt::passthrough_queue::McuHandle::from_raw(
                            mcu_handle,
                        ))
                        .unwrap_or(0);
                    let clock64 = crate::remote_trigger::relay_trip_clock(clock32, reference);
                    tracing::info!(
                        subsystem = "trip-relay",
                        event = "remote_trigger_fired",
                        mcu = mcu_handle,
                        endstop_id,
                        trsync_oid,
                        clock32,
                        clock64,
                        reason = params.try_get_u32("trigger_reason"),
                        "remote trsync terminal report — dispatching endstop trip"
                    );
                    dispatch_endstop_trip(&deps, mcu_handle, endstop_id, clock64);
                }),
            )
            .map_err(|e| {
                PyRuntimeError::new_err(format!(
                    "arm_remote_trigger: interceptor registration failed: {e:?}"
                ))
            })?;
        self.remote_triggers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(endstop_id, (mcu_handle, id));
        Ok(())
    }

    fn disarm_remote_trigger(&self, endstop_id: u8) -> PyResult<()> {
        let entry = self
            .remote_triggers
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .remove(&endstop_id);
        let Some((mcu_handle, id)) = entry else {
            return Err(PyRuntimeError::new_err(format!(
                "disarm_remote_trigger: endstop_id {endstop_id} is not armed"
            )));
        };
        let host_io = self
            .mcus
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu_handle)
            .and_then(|c| c.host_io.as_ref().map(Arc::clone));
        match host_io {
            Some(io) => io.unregister_frame_interceptor(id).map_err(|e| {
                PyRuntimeError::new_err(format!("disarm_remote_trigger: unregister failed: {e:?}"))
            }),
            // MCU detached: its reactor (and the interceptor with it) is
            // already gone. Disarm runs on cleanup paths — don't mask the
            // original error.
            None => Ok(()),
        }
    }

    fn home_abort(&self, py: Python<'_>) {
        struct AbortContext {
            all_axis_keys: Vec<crate::pump::AxisKey>,
            cohort: u64,
            axis_key: crate::pump::AxisKey,
            axis: u8,
        }

        let ctx = {
            let guard = self.homing_run.lock().unwrap_or_else(|p| p.into_inner());
            guard.as_ref().map(|r| AbortContext {
                all_axis_keys: r.all_axis_keys.clone(),
                cohort: r.cohort,
                axis_key: r.axis_key,
                axis: r.axis,
            })
        };

        let Some(ctx) = ctx else {
            self.finish_homing();
            return;
        };

        if let Some(tx) = self
            .pump_tx
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone()
        {
            let _ = tx.send(crate::pump::PumpMsg::Flush(ctx.all_axis_keys));
            let _ = tx.send(crate::pump::PumpMsg::DripDisarm(ctx.cohort));
            let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
            let _ = tx.send(crate::pump::PumpMsg::Barrier(ack_tx));
            let barrier = py.detach(move || ack_rx.recv_timeout(std::time::Duration::from_secs(1)));
            if barrier.is_err() {
                tracing::error!(
                    "home_abort: pump did not acknowledge the flush barrier — \
                     commanded_pos is STALE; a firmware restart is required"
                );
                self.finish_homing();
                return;
            }
        }

        self.finish_homing();

        let final_motor_pos =
            crate::homing::trajectory_final_position(ctx.axis_key, &self.motion_history);

        let final_motor_pos = match final_motor_pos {
            Ok(p) => p,
            Err(e) => {
                tracing::error!(
                    "home_abort: cannot reconcile position after aborted homing move \
                     (trajectory store empty or missing for axis {:?}): {e} — \
                     commanded_pos is STALE; a firmware restart is required to \
                     recover a consistent position",
                    ctx.axis_key
                );
                return;
            }
        };

        let drain = self.drain.clone();
        let drain_result = py.detach(|| drain.wait_drained(DRAIN_TIMEOUT));
        if let Err(e) = drain_result {
            tracing::error!(
                "home_abort: drain timed out after aborted homing move — \
                 commanded_pos is STALE; a firmware restart is required: {e}"
            );
            return;
        }

        let configs = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let kinematics = configs
            .iter()
            .find(|c| c.mcu_id == ctx.axis_key.mcu_id)
            .map_or(1u8, |c| c.kinematics);
        drop(configs);

        let motor_frame =
            trip_position_to_motor_frame(ctx.axis, final_motor_pos, &[], ctx.axis_key.mcu_id);
        let cartesian = crate::kinematics::inverse(kinematics, motor_frame);

        let planner_guard = self.planner.lock().unwrap_or_else(|p| p.into_inner());
        if let Some(planner) = planner_guard.as_ref() {
            let open_result =
                planner.kalico_stream_open([cartesian[0], cartesian[1], cartesian[2], 0.0]);
            if let Err(e) = open_result {
                tracing::error!(
                    "home_abort: kalico_stream_open failed after drain — \
                     commanded_pos is STALE; a firmware restart is required: {e:?}"
                );
                return;
            }
            self.drain.reset();
        }
        drop(planner_guard);

        *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner()) = cartesian;
    }

    #[pyo3(signature = (source_mcu, clock, host_now))]
    fn motion_state_at_clock(
        &self,
        source_mcu: u32,
        clock: u64,
        host_now: f64,
    ) -> PyResult<std::collections::HashMap<String, (f64, f64, f64)>> {
        const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];
        let configs: Vec<crate::dispatch::McuAxisConfig> = self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .clone();
        if configs.is_empty() {
            return Err(PyRuntimeError::new_err(
                "motion_state_at: no axes configured on the bridge",
            ));
        }
        let resolved: Vec<(crate::pump::AxisKey, u64, u64)> = {
            let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
            let source_handle = crate::types::mcu_handle_from_raw(source_mcu);
            let mut acc = Vec::new();
            for cfg in &configs {
                let target_handle = crate::types::mcu_handle_from_raw(cfg.mcu_id);
                let axis_clock = crate::motion_history::clock_between_mcus(
                    &router,
                    source_handle,
                    target_handle,
                    clock,
                )
                .map_err(PyRuntimeError::new_err)?;
                let now_clock = router
                    .host_time_to_mcu_clock(target_handle, host_now)
                    .map_err(|e| {
                        PyRuntimeError::new_err(format!(
                            "motion_state_at: clock unsynced for mcu {}: {e:?}",
                            cfg.mcu_id
                        ))
                    })?;
                for &axis in &cfg.axes {
                    let key = crate::pump::AxisKey {
                        mcu_id: cfg.mcu_id,
                        axis: axis as u8,
                    };
                    acc.push((key, axis_clock, now_clock));
                }
            }
            acc
        };
        let store = self
            .motion_history
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let mut out = std::collections::HashMap::new();
        for (key, axis_clock, now_clock) in resolved {
            let st = store
                .state_at_clock(key, axis_clock, Some(now_clock))
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
            let name = AXIS_NAMES.get(key.axis as usize).ok_or_else(|| {
                PyRuntimeError::new_err(format!("motion_state_at: unnamed axis {}", key.axis))
            })?;
            out.insert(
                (*name).to_string(),
                (st.position, st.velocity, st.acceleration),
            );
        }
        Ok(out)
    }
}

impl Drop for PyMotionBridge {
    // Backstop for the true-process-exit path: SIGTERM → request_exit → the
    // klippy loop breaks → Py_Finalize → pyo3 drops the bridge (if collected).
    // The primary release stays the explicit `klippy:disconnect` → `shutdown()`
    // call so it runs even under `gc.disable()` on the in-process restart loop.
    // `shutdown()` is idempotent, so this never double-tears-down.
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[derive(Clone)]
pub(crate) struct TripDeps {
    homing_run: Arc<Mutex<Option<HomingRun>>>,
    active_drip_cohort: Arc<Mutex<Option<u64>>>,
    pump_tx: Arc<Mutex<Option<std::sync::mpsc::Sender<crate::pump::PumpMsg>>>>,
    mcus: Arc<Mutex<HashMap<u32, McuConnection>>>,
    router: Arc<Mutex<PassthroughRouter>>,
    motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
    mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
}

impl PyMotionBridge {
    pub(crate) fn trip_deps(&self) -> TripDeps {
        TripDeps {
            homing_run: Arc::clone(&self.homing_run),
            active_drip_cohort: Arc::clone(&self.active_drip_cohort),
            pump_tx: Arc::clone(&self.pump_tx),
            mcus: Arc::clone(&self.mcus),
            router: Arc::clone(&self.router),
            motion_history: Arc::clone(&self.motion_history),
            mcu_axis_configs: Arc::clone(&self.mcu_axis_configs),
        }
    }
}

impl PyMotionBridge {
    fn finish_homing(&self) {
        *self
            .active_drip_cohort
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = None;
        *self.homing_run.lock().unwrap_or_else(|p| p.into_inner()) = None;
        *self.homing_result.lock().unwrap_or_else(|p| p.into_inner()) = None;
    }

    fn ethercat_conn(&self, mcu_handle: u32, what: &str) -> PyResult<Arc<UnixNativeConn>> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let mc = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!("{what}: unknown mcu_handle {mcu_handle}"))
        })?;
        mc.endpoint_conn.clone().ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "{what}: mcu {mcu_handle} ({}) is not an EtherCAT endpoint",
                mc.label
            ))
        })
    }

    fn host_io_for_mcu(&self, caller: &str, mcu: u32) -> PyResult<Arc<KalicoHostIo>> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get(&mcu).ok_or_else(|| {
            PyRuntimeError::new_err(format!("{caller}: unknown mcu_handle {mcu}"))
        })?;
        conn.host_io.as_ref().cloned().ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "{caller}: attach_serial has not been called for this MCU"
            ))
        })
    }

    fn handle_endstop_trip(&self, event_mcu: u32, endstop_id: u8, trip_clock: u64) {
        dispatch_endstop_trip(&self.trip_deps(), event_mcu, endstop_id, trip_clock);
    }
}

pub(crate) fn dispatch_endstop_trip(
    deps: &TripDeps,
    event_mcu: u32,
    endstop_id: u8,
    trip_clock: u64,
) {
    let run_opt: Option<HomingRun> = {
        let mut guard = deps.homing_run.lock().unwrap_or_else(|p| p.into_inner());
        guard.take()
    };
    let run = match run_opt {
        None => return,
        Some(r) => r,
    };
    if run.endstop_id != endstop_id || run.endstop_mcu != event_mcu {
        let mut guard = deps.homing_run.lock().unwrap_or_else(|p| p.into_inner());
        *guard = Some(run);
        return;
    }

    {
        let mut cohort_guard = deps
            .active_drip_cohort
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        *cohort_guard = None;
    }

    let pump_tx_opt = deps
        .pump_tx
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();

    let transports: HashMap<u32, Arc<dyn kalico_host_rt::native_call::NativeCall>> = {
        let mcus = deps.mcus.lock().unwrap_or_else(|p| p.into_inner());
        mcus.iter()
            .filter_map(|(&id, conn)| {
                if let Some(io) = conn.host_io.as_ref() {
                    Some((
                        id,
                        Arc::clone(io) as Arc<dyn kalico_host_rt::native_call::NativeCall>,
                    ))
                } else {
                    conn.endpoint_conn.as_ref().map(|ec| {
                        (
                            id,
                            Arc::clone(ec) as Arc<dyn kalico_host_rt::native_call::NativeCall>,
                        )
                    })
                }
            })
            .collect()
    };

    let router_arc = Arc::clone(&deps.router);
    let history_arc = Arc::clone(&deps.motion_history);
    let configs: Vec<McuAxisConfig> = deps
        .mcu_axis_configs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();

    std::thread::Builder::new()
        .name("homing-trip-handler".into())
        .spawn(move || {
            let stop_timeout = Duration::from_secs(3);

            let stepper_mcu_ids: std::collections::HashSet<u32> =
                run.all_axis_keys.iter().map(|k| k.mcu_id).collect();

            if let Some(tx) = pump_tx_opt.as_ref() {
                let _ = tx.send(crate::pump::PumpMsg::Flush(run.all_axis_keys.clone()));
                let _ = tx.send(crate::pump::PumpMsg::DripDisarm(run.cohort));
            }

            use kalico_host_rt::native_call::NativeCall as _;
            use kalico_protocol::codec::Decode as _;
            let stop_call =
                |mcu_id: u32| -> Result<kalico_protocol::messages::StopResponse, String> {
                    let transport = transports
                        .get(&mcu_id)
                        .ok_or_else(|| format!("Stop: no transport for mcu {mcu_id}"))?;
                    let (_kind, body) = transport
                        .kalico_call(kalico_protocol::MessageKind::Stop, Vec::new(), stop_timeout)
                        .map_err(|e| format!("Stop call failed for mcu {mcu_id}: {e:?}"))?;
                    kalico_protocol::messages::StopResponse::decode(&body)
                        .map_err(|e| format!("Stop decode failed for mcu {mcu_id}: {e:?}"))
                };

            let discard_clock = match crate::homing::broadcast_stop(
                &stepper_mcu_ids,
                run.axis_key.mcu_id,
                stop_call,
            ) {
                Ok(c) => c,
                Err(e) => {
                    let _ = run.notify.send(Err(e));
                    return;
                }
            };

            let axis = run.axis;
            let axis_key = run.axis_key;
            let kinematics = configs
                .iter()
                .find(|c| c.mcu_id == axis_key.mcu_id)
                .map_or(1u8, |c| c.kinematics);
            let reconstruct_cartesian = |source_mcu: u32, clock: u64| -> Result<[f64; 3], String> {
                let motor_pos = crate::homing::reconstruct_axis_position(
                    source_mcu,
                    clock,
                    axis_key,
                    &router_arc,
                    &history_arc,
                    run.window_start_clock,
                )?;
                let motor_frame =
                    trip_position_to_motor_frame(axis, motor_pos, &configs, axis_key.mcu_id);
                Ok(crate::kinematics::inverse(kinematics, motor_frame))
            };

            let outcome = reconstruct_cartesian(run.endstop_mcu, trip_clock).and_then(|trip| {
                reconstruct_cartesian(axis_key.mcu_id, discard_clock)
                    .map(|final_pos| (trip, final_pos, trip_clock))
            });

            let outcome = outcome.and_then(|positions| {
                if let Some(tx) = pump_tx_opt.as_ref() {
                    let (ack_tx, ack_rx) = std::sync::mpsc::sync_channel(1);
                    let _ = tx.send(crate::pump::PumpMsg::Barrier(ack_tx));
                    if ack_rx.recv_timeout(Duration::from_secs(1)).is_err() {
                        return Err("EndstopTrip: pump did not acknowledge the flush barrier \
                                 before stream resume"
                            .into());
                    }
                }
                for &mcu_id in &stepper_mcu_ids {
                    let transport = transports
                        .get(&mcu_id)
                        .ok_or_else(|| format!("ResumeStream: no transport for mcu {mcu_id}"))?;
                    let (_kind, body) = transport
                        .kalico_call(
                            kalico_protocol::MessageKind::ResumeStream,
                            Vec::new(),
                            stop_timeout,
                        )
                        .map_err(|e| format!("ResumeStream call failed for mcu {mcu_id}: {e:?}"))?;
                    let resp = kalico_protocol::messages::ResumeStreamResponse::decode(&body)
                        .map_err(|e| {
                            format!("ResumeStream decode failed for mcu {mcu_id}: {e:?}")
                        })?;
                    if resp.result != 0 {
                        return Err(format!(
                            "ResumeStream rejected by mcu {mcu_id}: result={}",
                            resp.result
                        ));
                    }
                }
                Ok(positions)
            });
            let _ = run.notify.send(outcome);
        })
        .expect("spawn homing-trip-handler");
}

impl PyMotionBridge {
    fn register_ethercat_mcu(
        &self,
        raw: u32,
        label: &str,
        socket_path: &str,
        child: std::process::Child,
        conn: UnixNativeConn,
    ) {
        self.mcus.lock().unwrap_or_else(|p| p.into_inner()).insert(
            raw,
            McuConnection {
                label: label.to_owned(),
                serial_path: String::new(),
                baud: 0,
                host_io: None,
                runtime_rx: None,
                runtime_caps: None,
                identify_caps: 0,
                kalico_native_supported: true,
                ethercat_socket: Some(socket_path.to_owned()),
                endpoint_process: Some(child),
                endpoint_conn: Some(Arc::new(conn)),
            },
        );
        self.nominal_clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(raw, ETHERCAT_CLOCK_FREQ_HZ);
    }
}

#[cfg(test)]
mod tests;

#[cfg(test)]
mod require_events_dir_tests {
    use super::require_events_dir_for_kalico_native;
    use std::path::Path;

    #[test]
    fn non_native_no_events_dir_is_ok() {
        assert!(
            require_events_dir_for_kalico_native(false, None, "mcu-stock").is_ok(),
            "non-native MCU must not require events_dir"
        );
    }

    #[test]
    fn non_native_with_events_dir_is_ok() {
        assert!(
            require_events_dir_for_kalico_native(
                false,
                Some(Path::new("/tmp/kalico-events")),
                "mcu-stock",
            )
            .is_ok(),
            "non-native MCU must be Ok regardless of events_dir"
        );
    }

    #[test]
    fn native_with_events_dir_is_ok() {
        assert!(
            require_events_dir_for_kalico_native(
                true,
                Some(Path::new("/tmp/kalico-events")),
                "mcu-h7",
            )
            .is_ok(),
            "native MCU must be Ok when events_dir is set"
        );
    }

    #[test]
    fn native_no_events_dir_is_err_containing_label() {
        let result = require_events_dir_for_kalico_native(true, None, "mcu-h7");
        assert!(
            result.is_err(),
            "native MCU without events_dir must return Err"
        );
        let msg = result.unwrap_err();
        assert!(
            msg.contains("mcu-h7"),
            "error message must contain the MCU label; got: {msg}"
        );
        assert!(
            msg.contains("init_logging"),
            "error message must mention init_logging; got: {msg}"
        );
    }

    #[test]
    fn native_no_events_dir_err_mentions_mculog_discard() {
        let result = require_events_dir_for_kalico_native(true, None, "mcu-f4");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("McuLog") || msg.contains("discarded"),
            "error message must explain McuLog discard; got: {msg}"
        );
    }
}

#[cfg(test)]
mod resolve_motion_caps_tests {
    use super::resolve_motion_caps;
    use crate::dispatch::McuCaps;
    use kalico_protocol::messages::RuntimeCapsResponse;

    #[test]
    fn some_caps_returns_ok_with_correct_value() {
        let caps = Some(RuntimeCapsResponse {
            total_piece_memory: 62 * 1024,
        });
        let result = resolve_motion_caps(caps, "octopus", 1);
        assert_eq!(
            result,
            Ok(McuCaps {
                total_piece_memory: 62 * 1024
            })
        );
    }

    #[test]
    fn none_caps_returns_err_containing_label_and_handle() {
        let result = resolve_motion_caps(None, "f446", 7);
        assert!(result.is_err(), "expected Err for None caps");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("f446"),
            "error message should contain the MCU label; got: {msg}"
        );
        assert!(
            msg.contains('7'),
            "error message should contain the handle; got: {msg}"
        );
    }
}

#[cfg(test)]
mod ethercat_endpoint_tests {
    use super::{handshake_ethercat_endpoint, poll_socket_ready, spawn_ethercat_endpoint};
    use std::io::{Read, Write};
    use std::time::{Duration, Instant};

    #[test]
    fn spawn_nonexistent_binary_errors_with_binary_path() {
        let result = spawn_ethercat_endpoint(
            "/nonexistent/binary/kalico-ec",
            "eth0",
            "/tmp/test.sock",
            1.0,
            false,
            None,
            30.0,
            None,
            None,
        );
        assert!(result.is_err(), "expected Err for nonexistent binary");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("/nonexistent/binary/kalico-ec"),
            "error message should contain the binary path; got: {msg}"
        );
        assert!(
            msg.contains("spawn"),
            "error message should indicate a spawn failure; got: {msg}"
        );
    }

    #[test]
    fn poll_socket_ready_detects_early_child_death() {
        let mut child = std::process::Command::new("sh")
            .args(["-c", "exit 3"])
            .spawn()
            .expect("sh must be available");

        // Give the process time to exit so try_wait will see it on the first
        // poll iteration (poll_socket_ready already does this internally, but
        // a brief spin here makes the test deterministic on loaded CI runners).
        let waited = {
            let start = Instant::now();
            loop {
                if child.try_wait().unwrap().is_some() {
                    break start.elapsed();
                }
                std::thread::sleep(Duration::from_millis(5));
                if start.elapsed() > Duration::from_secs(2) {
                    panic!("child did not exit within 2 s");
                }
            }
        };
        let _ = waited;

        let socket_path = "/tmp/kalico_test_socket_that_will_never_exist_a1b2c3d4";
        let deadline = Instant::now() + Duration::from_secs(30);
        let start = Instant::now();
        let result = poll_socket_ready(socket_path, deadline, &mut child);
        let elapsed = start.elapsed();

        assert!(result.is_err(), "expected Err on early child death");
        let msg = result.unwrap_err();
        assert!(
            msg.contains("exit") || msg.contains("exited"),
            "error message should mention exit status; got: {msg}"
        );
        assert!(
            elapsed < Duration::from_secs(5),
            "poll_socket_ready should return promptly on child death, not burn the deadline; \
             elapsed={elapsed:?}"
        );
    }

    fn encode_claim_handshake_reply(correlation_id: u32) -> Vec<u8> {
        use kalico_native_transport::frame::{CHANNEL_CONTROL, encode_frame};
        use kalico_native_transport::wire_helpers::{
            MESSAGE_VERSION_DEFAULT, encode_message_header,
        };
        use kalico_protocol::codec::Encode as _;
        use kalico_protocol::messages::{
            ClaimHandshakeReply, MessageKind, SlaveState, SlaveStatus,
        };

        let reply = ClaimHandshakeReply {
            slave_statuses: vec![SlaveStatus {
                slave_idx: 0,
                state: SlaveState::Ok,
                fault_code: 0,
            }],
        };
        let mut payload = encode_message_header(
            MessageKind::ClaimHandshakeReply,
            MESSAGE_VERSION_DEFAULT,
            correlation_id,
        )
        .to_vec();
        reply.encode(&mut payload);
        encode_frame(CHANNEL_CONTROL, &payload)
    }

    fn extract_correlation_id(buf: &[u8]) -> u32 {
        use kalico_native_transport::demux::{Demuxer, Frame};
        use kalico_native_transport::wire_helpers::decode_message_header;

        let mut demux = Demuxer::new();
        let (frames, _) = demux.feed_slice(buf);
        for f in frames {
            if let Frame::Kalico { payload, .. } = f {
                if let Some((hdr, _)) = decode_message_header(&payload) {
                    return hdr.correlation_id;
                }
            }
        }
        0
    }

    #[test]
    fn handshake_retries_past_stale_socket_file() {
        use std::os::unix::net::UnixListener;

        // Use pid + thread-id to avoid collisions when tests run in parallel.
        let path = format!(
            "/tmp/kalico_test_stale_{}_handshake.sock",
            std::process::id()
        );
        let _ = std::fs::remove_file(&path);

        {
            let _listener = UnixListener::bind(&path)
                .unwrap_or_else(|e| panic!("bind for stale-file setup failed: {e}"));
        }
        assert!(
            std::path::Path::new(&path).exists(),
            "UnixListener drop must leave the socket file — test precondition violated"
        );

        let (tx, rx) = std::sync::mpsc::channel::<()>();
        let path_bg = path.clone();
        let bg = std::thread::spawn(move || {
            let _ = std::fs::remove_file(&path_bg);
            let listener = UnixListener::bind(&path_bg)
                .unwrap_or_else(|e| panic!("background listener bind failed: {e}"));
            let _ = tx.send(());
            if let Ok((mut stream, _)) = listener.accept() {
                let mut buf = [0u8; 4096];
                if let Ok(n) = stream.read(&mut buf) {
                    let cid = extract_correlation_id(&buf[..n]);
                    let reply = encode_claim_handshake_reply(cid);
                    let _ = stream.write_all(&reply);
                    // Shutdown the write half — sends a clean FIN so the
                    // foreground's kalico_call read loop exits on EOF (Closed)
                    // *after* it has already matched the correlated reply frame
                    // and returned Ok.  Without this, dropping the stream under
                    // parallel load can race with the foreground's read.
                    let _ = stream.shutdown(std::net::Shutdown::Write);
                    // Block on a final drain read so we don't release the fd
                    // (and any kernel-buffered data) until the foreground has
                    // consumed everything.
                    let _ = stream.read(&mut buf);
                }
            }
        });

        rx.recv_timeout(Duration::from_secs(5))
            .expect("background listener must signal within 5 s");

        let deadline = Instant::now() + Duration::from_secs(5);
        let result = handshake_ethercat_endpoint(&path, deadline);
        let _ = std::fs::remove_file(&path);

        let succeeded = result.is_ok();
        // Drop the UnixNativeConn before joining so the foreground side closes,
        // unblocking the background thread's drain read.
        drop(result);
        let _ = bg.join();

        assert!(succeeded, "handshake must succeed once listener is up");
    }

    #[test]
    fn handshake_connect_refused_is_not_immediately_fatal() {
        use std::os::unix::net::UnixListener;
        use std::sync::Arc;
        use std::sync::atomic::{AtomicBool, Ordering};

        let path = format!(
            "/tmp/kalico_test_refused_{}_handshake.sock",
            std::process::id()
        );
        let _ = std::fs::remove_file(&path);

        {
            let _l = UnixListener::bind(&path).unwrap_or_else(|e| panic!("bind failed: {e}"));
        }

        let tried = Arc::new(AtomicBool::new(false));
        let tried_bg = Arc::clone(&tried);

        let (stop_tx, stop_rx) = std::sync::mpsc::channel::<()>();

        let path_hs = path.clone();
        let hs = std::thread::spawn(move || {
            tried_bg.store(true, Ordering::SeqCst);
            let deadline = Instant::now() + Duration::from_secs(4);
            handshake_ethercat_endpoint(&path_hs, deadline)
        });

        while !tried.load(Ordering::SeqCst) {
            std::thread::yield_now();
        }
        std::thread::sleep(Duration::from_millis(100));

        let _ = std::fs::remove_file(&path);
        let listener =
            UnixListener::bind(&path).unwrap_or_else(|e| panic!("late listener bind failed: {e}"));

        let path_lt = path.clone();
        let lt = std::thread::spawn(move || {
            let _ = stop_rx; // keep channel alive
            if let Ok((mut stream, _)) = listener.accept() {
                let mut buf = [0u8; 4096];
                if let Ok(n) = stream.read(&mut buf) {
                    let cid = extract_correlation_id(&buf[..n]);
                    let _ = stream.write_all(&encode_claim_handshake_reply(cid));
                    let _ = stream.shutdown(std::net::Shutdown::Write);
                    let _ = stream.read(&mut buf);
                }
            }
            let _ = std::fs::remove_file(&path_lt);
        });

        let result = hs.join().expect("handshake thread must not panic");

        let error_msg = match &result {
            Ok(_) => None,
            Err(e) => Some(format!("{e:?}")),
        };

        // Drop the UnixNativeConn first so the listener thread's drain read
        // sees EOF and exits. If the handshake never connected at all, the
        // listener is still parked in accept() — unblock it with a throwaway
        // connection so lt.join() cannot hang the test harness.
        let _ = stop_tx.send(());
        drop(result);
        let _ = std::os::unix::net::UnixStream::connect(&path);
        let _ = lt.join();

        if let Some(msg) = error_msg {
            assert!(
                !msg.to_ascii_lowercase().contains("connection refused"),
                "handshake must retry past ConnectionRefused, not fail immediately; got: {msg}"
            );
        }
    }
}

#[cfg(test)]
mod kinematics_calls_tests;
