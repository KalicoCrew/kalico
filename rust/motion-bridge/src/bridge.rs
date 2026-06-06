use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex, OnceLock};
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
use crate::config::{self, PlannerConfig, PlannerLimits, parse_required_shaper};
use crate::dispatch::{McuAxisConfig, McuCaps, build_mcu_configs};
use crate::homing::HomingState;
use crate::planner::{DispatchError, PlannerError, PlannerHandle};
use crate::types::{cq_id_from_raw, mcu_handle_from_raw, stats_to_pydict};

struct RetainedHomingCurve {
    axes: [nurbs::ScalarNurbs<f64>; 3],
    t_start: f64,
    t_end: f64,
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

/// Exceeding this means a wedged MCU — fail loudly.
const DRAIN_TIMEOUT: Duration = Duration::from_secs(60);

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

/// Spawn the EtherCAT endpoint binary.
///
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
) -> Result<std::process::Child, String> {
    std::process::Command::new(binary)
        .arg(interface)
        .arg("--socket")
        .arg(socket_path)
        .arg("--counts-per-mm")
        .arg(counts_per_mm.to_string())
        .spawn()
        .map_err(|e| format!("spawn {binary}: {e}"))
}

/// Poll for the socket file to appear, sleeping 20 ms between checks.
///
/// Early child death is detected at the call site (see `claim_ethercat_node`)
/// between poll iterations.
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
        // Detect early process death so the user gets a fast failure rather
        // than burning the full 15 s.
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
        x: parse_required_shaper(type_x, freq_x)?,
        y: parse_required_shaper(type_y, freq_y)?,
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
    handlers: Mutex<HashMap<(u32, String, u32), PyObject>>,
    planner: OnceLock<PlannerHandle>,
    planner_config: Mutex<PlannerConfig>,
    commanded_pos: Mutex<[f64; 3]>,
    mcu_axis_configs: Mutex<Vec<McuAxisConfig>>,
    dispatched_segments: Arc<AtomicU64>,
    fallback_clock_conversions: Arc<AtomicU64>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    homing: Arc<HomingState>,
    retained_homing_curve: Arc<Mutex<Option<RetainedHomingCurve>>>,
    probe_handles: Mutex<HashMap<u64, crate::probe_homing::ProbeHomingHandle>>,
    probe_handle_counter: AtomicU64,
    events_dir: Mutex<Option<std::path::PathBuf>>,
    pump_tx: Mutex<Option<std::sync::mpsc::Sender<crate::pump::PumpMsg>>>,
    pump_thread: Mutex<Option<JoinHandle<()>>>,
    drain: std::sync::Arc<crate::drain::DrainSync>,
}

pub(crate) fn build_configure_axes_body(
    kinematics: u8,
    present_mask: u8,
    awd_mask: u8,
    invert_mask: u8,
    steps_per_mm: &[f32; 4],
    step_modes: Option<&[u8; 4]>,
    phase_configs: Option<&[(u8, u8, u8)]>,
    phase_capable: u8,
) -> Vec<u8> {
    let mut body = Vec::with_capacity(26 + 3 * 16);
    body.push(kinematics);
    body.push(present_mask);
    body.push(awd_mask);
    body.push(invert_mask);
    for v in steps_per_mm {
        body.extend_from_slice(&v.to_le_bytes());
    }
    if let Some(sm) = step_modes {
        body.push(phase_capable);
        for &m in sm.iter() {
            body.push(m);
        }
    }
    if let Some(pc) = phase_configs {
        assert!(
            step_modes.is_some(),
            "phase_configs requires step_modes (variable-length format extends 25-byte)"
        );
        assert!(
            pc.len() <= 16,
            "phase_configs.len()={} exceeds MAX_STEPPER_OIDS=16",
            pc.len(),
        );
        body.push(pc.len() as u8);
        for &(bus_id, cs_pin_id, slot_idx) in pc.iter() {
            body.push(bus_id);
            body.push(cs_pin_id);
            body.push(slot_idx);
        }
    }
    body
}

pub(crate) fn axis_ring_depth(total_pieces: u32, num_axes: u32) -> u32 {
    (total_pieces / num_axes.max(1)).max(1)
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
            planner: OnceLock::new(),
            planner_config: Mutex::new(PlannerConfig::default()),
            commanded_pos: Mutex::new([0.0; 3]),
            mcu_axis_configs: Mutex::new(Vec::new()),
            dispatched_segments: Arc::new(AtomicU64::new(0)),
            fallback_clock_conversions: Arc::new(AtomicU64::new(0)),
            clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            homing: Arc::new(HomingState::new()),
            retained_homing_curve: Arc::new(Mutex::new(None)),
            probe_handles: Mutex::new(HashMap::new()),
            probe_handle_counter: AtomicU64::new(1),
            events_dir: Mutex::new(None),
            pump_tx: Mutex::new(None),
            pump_thread: Mutex::new(None),
            drain: std::sync::Arc::new(crate::drain::DrainSync::new()),
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

    #[pyo3(signature = (label, socket_path, interface, endpoint_binary, counts_per_mm))]
    fn claim_ethercat_node(
        &self,
        label: &str,
        socket_path: &str,
        interface: &str,
        endpoint_binary: &str,
        counts_per_mm: f64,
    ) -> PyResult<u32> {
        // Remove any stale socket file left by a previous session. The bridge
        // owns this path's lifecycle: anything already there is a dead leftover.
        // NotFound is fine (clean slate); every other error is fatal.
        if let Err(e) = std::fs::remove_file(socket_path) {
            if e.kind() != std::io::ErrorKind::NotFound {
                return Err(PyRuntimeError::new_err(format!(
                    "ethercat {label}: failed to remove stale socket {socket_path}: {e}"
                )));
            }
        }

        let mut child =
            spawn_ethercat_endpoint(endpoint_binary, interface, socket_path, counts_per_mm)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "ethercat {label}: endpoint failed to start — {e}"
                    ))
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
        Ok(raw)
    }

    fn release_mcu(&self, handle: u32) -> PyResult<()> {
        let (mut endpoint_process, endpoint_conn) = {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            match mcus.remove(&handle) {
                Some(mut c) => (c.endpoint_process.take(), c.endpoint_conn.take()),
                None => (None, None),
            }
        };

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
                    // Backstop: force-kill the endpoint.
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

        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router.release_mcu(mcu_handle_from_raw(handle));
        self.handlers
            .lock()
            .unwrap()
            .retain(|&(mcu, _, _), _| mcu != handle);
        Ok(())
    }

    fn shutdown(&self) {
        let handles: Vec<u32> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.keys().copied().collect()
        };
        for h in handles {
            let _ = self.release_mcu(h);
        }

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
            let _ = h.join();
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
        callback: PyObject,
    ) -> PyResult<()> {
        self.handlers
            .lock()
            .unwrap()
            .insert((mcu, name.to_owned(), oid), callback);
        Ok(())
    }

    fn passthrough_register_flush_callback(&self, mcu: u32, callback: PyObject) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let mcu_h = mcu_handle_from_raw(mcu);

        let cb: Box<dyn Fn() + Send> = Box::new(move || {
            Python::with_gil(|py| {
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
            // Wire the MCU log hook.  The hook timestamps MCU log events using
            // the router's clock record — fed by Python clocksync via
            // set_clock_est, which is the single writer for all MCUs.
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

    #[pyo3(signature = (mcu_handle, kinematics, present_mask, awd_mask, invert_mask, steps_per_mm, step_modes = None, phase_configs = None, timeout_s = 2.0))]
    fn configure_axes(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        kinematics: u8,
        present_mask: u8,
        awd_mask: u8,
        invert_mask: u8,
        steps_per_mm: Vec<f32>,
        step_modes: Option<Vec<u8>>,
        phase_configs: Option<Vec<(u8, u8, u8)>>,
        timeout_s: f64,
    ) -> PyResult<()> {
        tracing::info!(
            subsystem = "bridge",
            event = "configure_axes_entry",
            mcu_handle,
            kinematics,
            present_mask,
            awd_mask,
            invert_mask,
            steps_per_mm_len = steps_per_mm.len(),
            step_modes = ?step_modes,
            "configure_axes entry"
        );
        if steps_per_mm.len() != 4 {
            return Err(PyRuntimeError::new_err(
                "configure_axes: steps_per_mm must be a list of 4 floats",
            ));
        }
        if let Some(ref sm) = step_modes {
            if sm.len() != 4 {
                return Err(PyRuntimeError::new_err(
                    "configure_axes: step_modes must be a list of 4 ints (0=Modulated, 1=StepTime)",
                ));
            }
        }
        if let Some(ref pc) = phase_configs {
            if pc.len() > 16 {
                return Err(PyRuntimeError::new_err(format!(
                    "configure_axes: phase_configs.len()={} exceeds MAX_STEPPER_OIDS=16",
                    pc.len(),
                )));
            }
            if step_modes.is_none() {
                return Err(PyRuntimeError::new_err(
                    "configure_axes: phase_configs requires step_modes (variable-length format extends 25-byte)",
                ));
            }
            for (i, &(_, _, slot_idx)) in pc.iter().enumerate() {
                if slot_idx >= 4 {
                    return Err(PyRuntimeError::new_err(format!(
                        "configure_axes: phase_configs[{i}].slot_idx={slot_idx} must be < 4",
                    )));
                }
            }
        }
        let (io, identify_caps) = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!("configure_axes: unknown mcu_handle {mcu_handle}"))
            })?;
            if !conn.kalico_native_supported {
                return Ok(());
            }
            let io = conn
                .host_io
                .as_ref()
                .ok_or_else(|| {
                    PyRuntimeError::new_err(
                        "configure_axes: attach_serial has not been called for this MCU",
                    )
                })?
                .clone();
            (io, conn.identify_caps)
        };
        let phase_capable: u8 = if identify_caps & 0x1 != 0 { 1 } else { 0 };
        let steps_arr: [f32; 4] = [
            steps_per_mm[0],
            steps_per_mm[1],
            steps_per_mm[2],
            steps_per_mm[3],
        ];
        let step_modes_arr: Option<[u8; 4]> =
            step_modes.as_ref().map(|sm| [sm[0], sm[1], sm[2], sm[3]]);
        let body = build_configure_axes_body(
            kinematics,
            present_mask,
            awd_mask,
            invert_mask,
            &steps_arr,
            step_modes_arr.as_ref(),
            phase_configs.as_deref(),
            phase_capable,
        );
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let result = py.allow_threads(|| {
            io.kalico_call(kalico_protocol::MessageKind::ConfigureAxes, body, timeout)
        });
        match result {
            Ok((_, body)) => {
                if body.len() < 4 {
                    return Err(PyRuntimeError::new_err(
                        "configure_axes: short response body",
                    ));
                }
                let r = i32::from_le_bytes([body[0], body[1], body[2], body[3]]);
                if r != 0 {
                    return Err(PyRuntimeError::new_err(format!(
                        "configure_axes: MCU returned error {r}"
                    )));
                }
                Ok(())
            }
            Err(e) => Err(PyRuntimeError::new_err(format!(
                "configure_axes: transport error: {e:?}"
            ))),
        }
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
        let params = py.allow_threads(|| -> PyResult<_> {
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

    #[pyo3(signature = (mcu_handle, motor_idx, bus_id, cs_pin_id, timeout_s = 5.0))]
    fn register_phase_motor(
        &self,
        py: Python<'_>,
        mcu_handle: u32,
        motor_idx: u8,
        bus_id: u8,
        cs_pin_id: u8,
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
             bus_id={bus_id} cs_pin_id={cs_pin_id}"
        );
        let params = py.allow_threads(|| -> PyResult<_> {
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
        let params = py.allow_threads(|| -> PyResult<_> {
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
            RuntimeEvent::EndstopTripped(e) => {
                d.set_item("type", "endstop_tripped")?;
                d.set_item("arm_id", e.arm_id)?;
                d.set_item("trip_clock", e.trip_clock)?;
                d.set_item("trip_source_idx", e.trip_source_idx)?;
                d.set_item("fmt_version", e.fmt_version)?;
                d.set_item("stepper_count", e.stepper_count)?;
                let steppers: Vec<Py<PyDict>> = e
                    .steppers
                    .iter()
                    .map(|s| {
                        let sd = PyDict::new(py);
                        sd.set_item("oid", s.oid).unwrap();
                        sd.set_item("step_count", s.step_count).unwrap();
                        sd.unbind()
                    })
                    .collect();
                d.set_item("steppers", steppers)?;
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

    /// Ask the MCU's reactor to encode+send `get_clock` and stamp the
    /// unsolicited "clock" response with honest CLOCK_MONOTONIC_RAW RTT
    /// measurements. Encoding happens in the reactor with that MCU's own
    /// dictionary — the bridge-level parser holds whichever dict was set
    /// last and must never encode for a specific MCU.
    ///
    /// Called from `serialhdl` in bridge mode for the periodic
    /// `_get_clock_event` loop.  Returns immediately (fire-and-forget); the
    /// response arrives via `take_runtime_event` as a PassthroughResponse with
    /// `sent_time_raw`/`recv_time_raw` baked in.
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
        // Python clocksync is the single writer of the router clock record for
        // all MCUs (including kalico-native H7/F446 and Beacon).  The former
        // Rust periodic sync loop has been retired — clocksync.py feeds every
        // MCU via this path.
        //
        // `offset` arrives as `time_avg + min_half_rtt` in CLOCK_MONOTONIC_RAW
        // seconds (the mirror callback now exports the faithful clock_est triple
        // rather than TRANSMIT_EXTRA-biased values).  `host_now_raw` is captured
        // by the Python callback as its first action (reactor.monotonic() ==
        // CLOCK_MONOTONIC_RAW) so both sides of the epoch translation are in the
        // same domain with no GIL-hop jitter added on the Rust side.
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
        if self.planner.get().is_some() {
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
                // freq=1e9: EtherCAT timestamps are CLOCK_MONOTONIC_RAW nanoseconds.
                let _ = router.set_clock_est_from_sample(
                    mcu_h,
                    1_000_000_000.0_f64,
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
                t.insert(id, crate::pump::McuTransport::EtherCat(Arc::clone(conn)));
            }
            t
        };

        let pump_timeout = Duration::from_secs(5);
        let ring_depth_table_for_pump = ring_depth_table.clone();
        let router_for_pump = Arc::clone(&self.router);
        let pump_thread_handle = std::thread::Builder::new()
            .name("push-pieces-pump".into())
            .spawn(move || {
                let sink = crate::pump::WireSink {
                    transports: wire_transports,
                    timeout: pump_timeout,
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
                        let _ = std::io::Write::flush(&mut std::io::stderr());
                        if std::env::var_os("KALICO_NO_EXIT_ON_FAULT").is_none() {
                            std::process::abort();
                        }
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

                conn.attach_heartbeat_callback(Arc::new(move |retired: &[u32]| {
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

                let conn_for_poll = Arc::clone(&conn);
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
                        let _ = std::io::Write::flush(&mut std::io::stderr());
                        if std::env::var_os("KALICO_NO_EXIT_ON_FAULT").is_none() {
                            std::process::abort();
                        }
                    });

                let _ = std::thread::Builder::new()
                    .name(format!("ec-heartbeat-poll-{mcu_id}"))
                    .spawn(move || {
                        loop {
                            conn_for_poll.poll_events();

                            // Supervision: check for conn EOF before checking the child,
                            // because EOF is cheaper and fires first on clean exit.
                            if conn_for_poll.peer_closed() {
                                on_endpoint_death("conn EOF");
                                return;
                            }

                            // Check child exit status with a brief mutex acquisition.
                            let child_exited = {
                                let mut mcus = mcus_for_supervision
                                    .lock()
                                    .unwrap_or_else(|p| p.into_inner());
                                if let Some(c) = mcus.get_mut(&mcu_id) {
                                    if let Some(ref mut child) = c.endpoint_process {
                                        match child.try_wait() {
                                            Ok(Some(status)) => {
                                                Some(format!("child exited: {status}"))
                                            }
                                            Ok(None) => None,
                                            Err(e) => Some(format!("try_wait error: {e}")),
                                        }
                                    } else {
                                        None
                                    }
                                } else {
                                    // MCU was released — normal shutdown, exit quietly.
                                    return;
                                }
                            };

                            if let Some(reason) = child_exited {
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
                    .anchor_segment(seg.t_start, seg.t_end, host_now);

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

                let msgs = crate::enqueue::enqueue_segment(
                    seg,
                    &mcu_configs_for_cb,
                    t0,
                    fresh,
                    host_now,
                    project,
                );

                for m in msgs {
                    drain_disp.add_sent(m.key.mcu_id, m.key.axis, m.pieces.len() as u32);
                    pump_tx_for_cb
                        .send(crate::pump::PumpMsg::Enqueue(m))
                        .map_err(|_| DispatchError::PumpGone)?;
                }

                counter_for_cb.fetch_add(1, Ordering::Relaxed);
                Ok(())
            },
        );

        self.planner
            .set(PlannerHandle::spawn(cfg, dispatch))
            .map_err(|_| PyRuntimeError::new_err("planner already initialized (raced)"))?;
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
        py.allow_threads(|| -> PyResult<()> {
            let pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            let classified = classify::classify_and_build(pos, dx, dy, dz, de, feedrate)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

            let planner = self.planner.get().ok_or_else(|| {
                PyRuntimeError::new_err("planner not initialized — call init_planner first")
            })?;
            planner.submit_move(classified).map_err(planner_err)?;

            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            pos[0] += dx;
            pos[1] += dy;
            pos[2] += dz;
            Ok(())
        })
    }

    #[pyo3(signature = (newpos, speed, arm_ids))]
    fn submit_homing_move(&self, newpos: Vec<f64>, speed: f64, arm_ids: Vec<u32>) -> PyResult<()> {
        self.submit_homing_move_inner(&newpos, speed, &arm_ids)
    }

    fn wait_moves(&self, py: Python<'_>) -> PyResult<()> {
        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.allow_threads(|| planner.flush()).map_err(planner_err)?;
        self.homing.refresh_after_wait();
        Ok(())
    }

    fn drain_motion(&self, py: Python<'_>) -> PyResult<()> {
        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.allow_threads(|| planner.flush()).map_err(planner_err)?;
        let drain = self.drain.clone();
        py.allow_threads(|| drain.wait_drained(DRAIN_TIMEOUT))
            .map_err(PyRuntimeError::new_err)?;
        self.homing.refresh_after_wait();
        Ok(())
    }

    fn take_trip_event(&self, py: Python<'_>) -> PyResult<Option<Py<PyDict>>> {
        let Some(evt) = self.homing.take_trip_event() else {
            return Ok(None);
        };
        Ok(Some(trip_event_to_pydict(py, evt)?))
    }

    #[pyo3(signature = (mcu, queue, arm_id, arm_clock, sources, stepper_oids, timeout_s=2.0))]
    #[allow(clippy::too_many_arguments)]
    fn endstop_arm(
        &self,
        mcu: u32,
        queue: u32,
        arm_id: u32,
        arm_clock: u64,
        sources: Vec<(u8, u16, bool, u8, u8, u8, u32)>,
        stepper_oids: Vec<u8>,
        timeout_s: f64,
    ) -> PyResult<u8> {
        use kalico_host_rt::endstop;
        let _ = queue;

        let mut source_specs = Vec::with_capacity(sources.len());
        for (kind_byte, gpio, active_high, policy_byte, sample_n, velocity_axis, v_min_q16) in
            sources
        {
            let kind = match kind_byte {
                0 => endstop::SourceKind::Physical,
                1 => endstop::SourceKind::TmcDiag,
                2 => endstop::SourceKind::Software,
                _ => return Err(PyRuntimeError::new_err("invalid source kind")),
            };
            let policy = match policy_byte {
                0 => endstop::ArmPolicy::TripImmediately,
                1 => endstop::ArmPolicy::WaitForClear,
                2 => endstop::ArmPolicy::IgnoreUntilMoving,
                _ => return Err(PyRuntimeError::new_err("invalid arm policy")),
            };
            source_specs.push(endstop::SourceSpec {
                kind,
                gpio,
                active_high,
                policy,
                sample_n,
                velocity_axis,
                v_min_q16,
            });
        }

        let io = self.host_io_for_mcu("endstop_arm", mcu)?;
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let status = endstop::arm_endstop_with_timeout(
            io.as_ref(),
            arm_id,
            arm_clock,
            &source_specs,
            &stepper_oids,
            timeout,
        )
        .map_err(|e| PyRuntimeError::new_err(format!("endstop_arm: {e}")))?;
        tracing::info!(
            subsystem = "homing",
            event = "endstop_arm_result",
            mcu,
            arm_id,
            arm_clock,
            status = status as u8,
            "endstop_arm result"
        );
        Ok(status as u8)
    }

    #[pyo3(signature = (mcu, queue, arm_id, timeout_s=2.0))]
    fn endstop_disarm(&self, mcu: u32, queue: u32, arm_id: u32, timeout_s: f64) -> PyResult<u8> {
        use kalico_host_rt::endstop;
        let _ = queue;
        let io = self.host_io_for_mcu("endstop_disarm", mcu)?;
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let status = endstop::disarm_endstop_with_timeout(io.as_ref(), arm_id, timeout)
            .map_err(|e| PyRuntimeError::new_err(format!("endstop_disarm: {e}")))?;
        Ok(status as u8)
    }

    #[pyo3(signature = (newpos, speed, arm_ids))]
    fn submit_homing_move_async(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        arm_ids: Vec<u32>,
    ) -> PyResult<()> {
        self.submit_homing_move_inner(&newpos, speed, &arm_ids)
    }

    fn is_homing_segment_retired(&self) -> bool {
        matches!(
            self.homing.state(),
            crate::homing::HomingSegmentState::Completed
                | crate::homing::HomingSegmentState::Tripped
                | crate::homing::HomingSegmentState::DeadlineExpired
        )
    }

    fn get_homing_segment_reason(&self) -> u8 {
        match self.homing.state() {
            crate::homing::HomingSegmentState::Completed => 1,
            crate::homing::HomingSegmentState::Tripped => 2,
            crate::homing::HomingSegmentState::DeadlineExpired => 3,
            _ => 0,
        }
    }

    #[pyo3(signature = (mcu, arm_id, timeout_s=2.0))]
    fn software_trip(&self, mcu: u32, arm_id: u32, timeout_s: f64) -> PyResult<u8> {
        let io = self.host_io_for_mcu("software_trip", mcu)?;
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let msg = format!("runtime_software_trip arm_id={arm_id}");
        let params = {
            use kalico_host_rt::transport::Transport;
            io.call(&msg, "kalico_software_trip_response", timeout)
                .map_err(|e| PyRuntimeError::new_err(format!("software_trip: {e}")))?
        };
        let status = params.try_get_u32("status").unwrap_or(1) as u8;
        Ok(status)
    }

    #[pyo3(signature = (mcu, arm_id))]
    fn extend_homing_deadline(&self, mcu: u32, arm_id: u32) -> PyResult<()> {
        let io = self.host_io_for_mcu("extend_homing_deadline", mcu)?;
        let msg = format!("runtime_extend_homing_deadline arm_id={arm_id}");
        io.send_fire_and_forget(&msg)
            .map_err(|e| PyRuntimeError::new_err(format!("extend_homing_deadline: {e}")))?;
        Ok(())
    }

    fn prepare_probe_homing(
        &self,
        beacon_handle: u32,
        beacon_trsync_oid: u8,
        stepper_mcu_handle: u32,
        arm_id: u32,
        sensor_fault_timeout_s: f64,
    ) -> PyResult<u64> {
        let beacon_io = self.host_io_for_mcu("prepare_probe_homing(beacon)", beacon_handle)?;
        let stepper_io =
            self.host_io_for_mcu("prepare_probe_homing(stepper)", stepper_mcu_handle)?;

        let handle = crate::probe_homing::prepare_probe_homing(
            beacon_io,
            stepper_io,
            beacon_trsync_oid,
            arm_id,
            std::time::Duration::from_secs_f64(sensor_fault_timeout_s),
        )
        .map_err(|e| PyRuntimeError::new_err(format!("prepare_probe_homing: {e}")))?;

        let id = self.next_probe_handle_id();
        self.probe_handles.lock().unwrap().insert(id, handle);
        Ok(id)
    }

    #[pyo3(signature = (
        handle_id,
        move_pos,
        speed,
        stepper_oids,
    ))]
    fn run_probe_homing(
        &self,
        py: Python<'_>,
        handle_id: u64,
        move_pos: Vec<f64>,
        speed: f64,
        stepper_oids: Vec<u8>,
    ) -> PyResult<u8> {
        let _ = stepper_oids;

        let handle = self
            .probe_handles
            .lock()
            .unwrap()
            .remove(&handle_id)
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!("run_probe_homing: unknown handle_id {handle_id}"))
            })?;

        let seg_count_before = self.dispatched_segments.load(Ordering::Relaxed);
        self.submit_homing_move_inner(&move_pos, speed, &[handle.arm_id])?;
        let seg_count_after = self.dispatched_segments.load(Ordering::Relaxed);
        tracing::info!(
            subsystem = "homing",
            event = "homing_move_dispatch",
            seg_before = seg_count_before,
            seg_after = seg_count_after,
            dispatched = seg_count_after - seg_count_before,
            "run_probe_homing homing move dispatched"
        );

        let result = py.allow_threads(|| crate::probe_homing::run_probe_homing(&handle));

        crate::probe_homing::cleanup_probe_homing(handle);

        match result {
            Ok(r) => Ok(r as u8),
            Err(e) => Err(PyRuntimeError::new_err(format!(
                "run_probe_homing transport error: {e}"
            ))),
        }
    }

    fn submit_dwell(&self, duration_s: f64) -> PyResult<()> {
        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.dwell(duration_s).map_err(planner_err)
    }

    fn set_position(&self, py: Python<'_>, x: f64, y: f64, z: f64) -> PyResult<()> {
        {
            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            *pos = [x, y, z];
        }
        if let Some(planner) = self.planner.get() {
            py.allow_threads(|| planner.flush()).map_err(planner_err)?;
            {
                let drain = self.drain.clone();
                py.allow_threads(|| drain.wait_drained(DRAIN_TIMEOUT))
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

        *self
            .retained_homing_curve
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = None;

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

        let planner = self.planner.get().ok_or_else(|| {
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

        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.update_shaper(shaper).map_err(planner_err)
    }

    fn get_last_move_time(&self) -> f64 {
        match self.planner.get() {
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

    #[pyo3(signature = (t,))]
    fn get_homing_position_at_time(&self, t: f64) -> PyResult<Vec<f64>> {
        let guard = self
            .retained_homing_curve
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let curve = guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err("get_homing_position_at_time: no homing curve retained")
        })?;
        let t_clamped = t.clamp(curve.t_start, curve.t_end);
        let pos: Vec<f64> = curve
            .axes
            .iter()
            .map(|axis| nurbs::eval::eval(axis, t_clamped))
            .collect();
        Ok(pos)
    }
}

impl PyMotionBridge {
    fn next_probe_handle_id(&self) -> u64 {
        self.probe_handle_counter.fetch_add(1, Ordering::Relaxed)
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

    fn submit_homing_move_inner(
        &self,
        newpos: &[f64],
        speed: f64,
        arm_ids: &[u32],
    ) -> PyResult<()> {
        if newpos.len() < 3 {
            return Err(PyRuntimeError::new_err(
                "submit_homing_move requires newpos with at least 3 axes",
            ));
        }
        let arm_id = arm_ids.first().copied().ok_or_else(|| {
            PyRuntimeError::new_err("submit_homing_move requires at least one arm id")
        })?;
        self.homing.begin(arm_id);

        let pos = *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
        log::info!(
            "[bridge-trace] submit_homing_move arm_id={} pos=[{:.3},{:.3},{:.3}] newpos=[{:.3},{:.3},{:.3}] speed={:.3}",
            arm_id,
            pos[0],
            pos[1],
            pos[2],
            newpos[0],
            newpos[1],
            newpos[2],
            speed,
        );
        let classified = classify::classify_and_build(
            pos,
            newpos[0] - pos[0],
            newpos[1] - pos[1],
            newpos[2] - pos[2],
            0.0,
            speed,
        )
        .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        if let Err(e) = planner.submit_move(classified) {
            self.homing.reset_to_idle();
            return Err(planner_err(e));
        }
        Ok(())
    }
}

#[cfg(test)]
mod build_configure_axes_body_tests;

fn trip_event_to_pydict(py: Python<'_>, evt: runtime::endstop::TripEvent) -> PyResult<Py<PyDict>> {
    let d = PyDict::new(py);
    d.set_item("arm_id", evt.arm_id)?;
    d.set_item("trip_clock", evt.trip_clock)?;
    d.set_item("trip_source_idx", evt.trip_source_idx)?;
    d.set_item("stepper_count", evt.stepper_count)?;
    let steppers: Vec<Py<PyDict>> = evt
        .steppers
        .iter()
        .take(usize::from(evt.stepper_count))
        .map(|s| {
            let sd = PyDict::new(py);
            sd.set_item("oid", s.oid).unwrap();
            sd.set_item("step_count", s.step_count).unwrap();
            sd.unbind()
        })
        .collect();
    d.set_item("steppers", steppers)?;
    Ok(d.unbind())
}

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

    /// `poll_socket_ready` must detect early child death and return an error
    /// well before the deadline rather than burning the full timeout.
    #[test]
    fn poll_socket_ready_detects_early_child_death() {
        // Spawn a process that exits immediately with code 3.
        // The socket path is deliberately one that will never appear.
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
        let _ = waited; // document that the child is confirmed dead

        let socket_path = "/tmp/kalico_test_socket_that_will_never_exist_a1b2c3d4";
        // Deadline is generous (30 s); we expect a fast error.
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

    /// Build a framed ClaimHandshakeReply that `handshake_ethercat_endpoint`
    /// can parse.  Returns the raw bytes to write onto the socket.
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

    /// Parse the first framed message from `buf[..n]`, returning its
    /// correlation_id so the reply can be correlated.
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

    /// A stale socket file — left by a dropped listener — must not prevent
    /// `handshake_ethercat_endpoint` from succeeding once the real listener is
    /// up. The retry loop in `handshake_ethercat_endpoint` must connect past
    /// the ECONNREFUSED / ENOENT window.
    #[test]
    fn handshake_retries_past_stale_socket_file() {
        use std::os::unix::net::UnixListener;

        // Use pid + thread-id to avoid collisions when tests run in parallel.
        let path = format!(
            "/tmp/kalico_test_stale_{}_handshake.sock",
            std::process::id()
        );
        let _ = std::fs::remove_file(&path);

        // Create a socket file that has no listener behind it (bind → drop without
        // removing the file).  On Linux and macOS, dropping UnixListener does NOT
        // unlink the file — assert this as a precondition so the test is
        // self-documenting.
        {
            let _listener = UnixListener::bind(&path)
                .unwrap_or_else(|e| panic!("bind for stale-file setup failed: {e}"));
            // listener drops here; file stays
        }
        assert!(
            std::path::Path::new(&path).exists(),
            "UnixListener drop must leave the socket file — test precondition violated"
        );

        // Background thread: remove the stale file and bind a real listener,
        // then signals via `tx` once the listener is bound.  The foreground
        // waits for that signal before calling handshake — this eliminates the
        // sleep-based timing dependency that flaps under parallel-test load.
        // After writing the reply, the thread calls `shutdown(Write)` so the
        // foreground receives a clean FIN instead of a torn-down fd.
        let (tx, rx) = std::sync::mpsc::channel::<()>();
        let path_bg = path.clone();
        let bg = std::thread::spawn(move || {
            let _ = std::fs::remove_file(&path_bg);
            let listener = UnixListener::bind(&path_bg)
                .unwrap_or_else(|e| panic!("background listener bind failed: {e}"));
            // Signal after bind so the foreground knows the listener is up.
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

        // Wait for the background listener to be bound (with a generous bound).
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

    /// `handshake_ethercat_endpoint` must NOT immediately return
    /// ConnectionRefused as a fatal Protocol error when the socket path has a
    /// dead file but no listener — it must retry until the deadline.
    ///
    /// Structure: the handshake call runs in a background thread while the
    /// foreground waits 100 ms (letting the handshake hit ConnectionRefused at
    /// least once) and then sets up the real listener.  Using a thread for the
    /// handshake removes the timing-sensitivity of the sleep-based approach.
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

        // Bind-then-drop leaves a dead socket file with no listener.
        {
            let _l = UnixListener::bind(&path).unwrap_or_else(|e| panic!("bind failed: {e}"));
        }

        // Flag: set by the handshake thread once it has tried at least once.
        let tried = Arc::new(AtomicBool::new(false));
        let tried_bg = Arc::clone(&tried);

        // Channel for the foreground to signal the listener thread to stop.
        let (stop_tx, stop_rx) = std::sync::mpsc::channel::<()>();

        // Handshake thread: calls handshake with a 4 s deadline.
        // The listener won't be ready for ~100 ms so it will retry.
        let path_hs = path.clone();
        let hs = std::thread::spawn(move || {
            tried_bg.store(true, Ordering::SeqCst);
            let deadline = Instant::now() + Duration::from_secs(4);
            handshake_ethercat_endpoint(&path_hs, deadline)
        });

        // Foreground: wait until the handshake thread has started, then sleep
        // briefly so the first connect attempt hits ConnectionRefused, then
        // set up the real listener.
        while !tried.load(Ordering::SeqCst) {
            std::thread::yield_now();
        }
        std::thread::sleep(Duration::from_millis(100));

        let _ = std::fs::remove_file(&path);
        let listener =
            UnixListener::bind(&path).unwrap_or_else(|e| panic!("late listener bind failed: {e}"));

        // Listener thread: accept one connection and serve the reply.
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

        // The result must be Ok — or if not, must NOT be a ConnectionRefused
        // failure, which would mean the retry loop gave up immediately.
        if let Some(msg) = error_msg {
            assert!(
                !msg.to_ascii_lowercase().contains("connection refused"),
                "handshake must retry past ConnectionRefused, not fail immediately; got: {msg}"
            );
        }
    }
}
