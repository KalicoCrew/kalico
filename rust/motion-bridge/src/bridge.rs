//! `PyMotionBridge` — the PyO3 class that klippy calls.
//!
//! Phase 1: direct wrapper around `PassthroughRouter`. No reactor threads,
//! no real serial I/O. The API surface matches what klippy will need so
//! that the Python-side code can be developed in parallel.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex};

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

use kalico_host_rt::clock::RealClock;
use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::host_io::parser::{DataDictionary, MsgProtoParser};
use kalico_host_rt::host_io::{KalicoHostIo, KalicoHostIoConfig};
use kalico_host_rt::passthrough_queue::{
    NotifyId, PassthroughEntry, PassthroughRouter,
};
use kalico_host_rt::producer;
use trajectory::{AxisShaper, ShaperConfig};

use crate::classify;
use crate::config::{self, parse_required_shaper, PlannerConfig, PlannerLimits};
use crate::dispatch::{build_push_params, McuAxisConfig, AXIS_X, AXIS_Y, AXIS_Z};
use crate::homing::HomingState;
use crate::planner::{PlannerError, PlannerHandle};
use crate::slot_pool::{SlotPool, CURVE_POOL_N};
use crate::types::{cq_id_from_raw, mcu_handle_from_raw, stats_to_pydict};

/// Initial credit seed for the per-MCU `CreditCounter`. The bridge wires
/// `kalico_credit_freed` events into [`CreditCounter::on_credit_freed`] via
/// [`PyMotionBridge::on_credit_freed`] — but the upstream event-routing
/// path (an inbound serial reactor) is not yet hooked up to the bridge,
/// so in practice this seed bounds the in-flight credit budget for the
/// whole print. Sized generously so motion doesn't stall on credit before
/// the routing lands.
const CREDIT_SEED_CAPACITY: i32 = 1024;

// ── Internal types ──────────────────────────────────────────────────────

/// Metadata stored per claimed MCU.
struct McuConnection {
    #[allow(dead_code)]
    label: String,
    serial_path: String,
    baud: u32,
    /// Live I/O handle — populated by `attach_serial`. `None` until attached.
    /// Wrapped in `Arc` so callers can clone the reference out of the mutex
    /// and then call blocking methods without holding the lock.
    host_io: Option<Arc<KalicoHostIo>>,
    /// Runtime event receiver — populated by `attach_serial`. Drained by
    /// `take_runtime_event` for klippy-side dispatch.
    runtime_rx: Option<Receiver<kalico_host_rt::host_io::runtime_events::RuntimeEvent>>,
}

/// An event queued for Python consumption via `poll_event()`.
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

/// Map `RouterError` to a Python `RuntimeError`.
fn router_err(e: kalico_host_rt::passthrough_queue::RouterError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

/// Map `PlannerError` to a Python `RuntimeError`.
fn planner_err(e: PlannerError) -> PyErr {
    PyRuntimeError::new_err(e.to_string())
}

fn build_shaper_config(
    type_x: &str,
    freq_x: f64,
    type_y: &str,
    freq_y: f64,
) -> Result<ShaperConfig, String> {
    Ok(ShaperConfig {
        x: parse_required_shaper(type_x, freq_x)?,
        y: parse_required_shaper(type_y, freq_y)?,
        z: AxisShaper::Passthrough,
    })
}

fn format_push_segment_cmd(params: &producer::SegmentPushParams) -> String {
    format!(
        "kalico_push_segment id={id} x_handle={x_handle} \
         y_handle={y_handle} z_handle={z_handle} e_handle={e_handle} \
         t_start_hi={t_start_hi} t_start_lo={t_start_lo} \
         t_end_hi={t_end_hi} t_end_lo={t_end_lo} \
         kinematics={kin} e_mode={e_mode} extrusion_ratio={extrusion_ratio}",
        id = params.id,
        x_handle = params.x_handle_packed,
        y_handle = params.y_handle_packed,
        z_handle = params.z_handle_packed,
        e_handle = params.e_handle_packed,
        t_start_lo = params.t_start as u32,
        t_start_hi = (params.t_start >> 32) as u32,
        t_end_lo = params.t_end as u32,
        t_end_hi = (params.t_end >> 32) as u32,
        kin = params.kinematics,
        e_mode = params.e_mode,
        extrusion_ratio = params.extrusion_ratio.to_bits(),
    )
}

fn push_segment_fire_and_forget(
    io: &KalicoHostIo,
    credit: &CreditCounter,
    params: &producer::SegmentPushParams,
) -> Result<(), producer::ProducerError> {
    credit
        .try_acquire()
        .ok_or(producer::ProducerError::NoCredit)?;
    let cmd = format_push_segment_cmd(params);
    if let Err(e) = io.send_fire_and_forget(&cmd) {
        credit.release();
        return Err(producer::ProducerError::Transport(e));
    }
    Ok(())
}

// ── PyMotionBridge ──────────────────────────────────────────────────────

#[pyclass(name = "MotionBridge")]
#[allow(missing_debug_implementations)]
pub struct PyMotionBridge {
    /// Shared for passthrough queue state and MCU clock conversion.
    router: Arc<Mutex<PassthroughRouter>>,
    /// MsgProto parser populated via `set_msgproto_dict` for passthrough
    /// compatibility surfaces.
    parser: Arc<Mutex<Option<Arc<MsgProtoParser>>>>,
    mcus: Mutex<HashMap<u32, McuConnection>>,
    /// Shared event queue — callbacks capture an `Arc` clone so they can
    /// push events from any thread without holding a reference to `self`.
    events: Arc<Mutex<VecDeque<BridgeEvent>>>,
    /// Typed-response handlers registered via `passthrough_register_handler`.
    /// Key: `(mcu_handle, name, oid)`.  Phase 1 stores them but does not
    /// dispatch — actual dispatch requires the reactor thread.
    #[allow(dead_code)]
    handlers: Mutex<HashMap<(u32, String, u32), PyObject>>,

    // ── Phase-2 motion-submission state (Task 8) ────────────────────────
    /// Spawned planner thread (None until `init_planner` is called).
    planner: Mutex<Option<PlannerHandle>>,
    /// Current planner config snapshot, mutated by `update_limits` / `update_shaper`.
    planner_config: Mutex<PlannerConfig>,
    /// Last commanded toolhead position (set by `set_position`, advanced by `submit_move`).
    commanded_pos: Mutex<[f64; 3]>,
    /// Per-MCU axis assignment, populated by `init_planner`.
    mcu_axis_configs: Mutex<Vec<McuAxisConfig>>,
    /// Counter of shaped segments observed by the dispatch callback. Used by
    /// tests / sim to verify the planner pipeline ran end-to-end.
    dispatched_segments: Arc<AtomicU64>,
    /// Per-MCU curve-slot allocator. Populated by `init_planner` and
    /// driven by `on_credit_freed` (segment-id retirement → slot release).
    /// `Arc<Mutex<SlotPool>>` so the dispatch closure (planner thread) and
    /// the event-routing thread (klippy reactor, eventually) can share it.
    slot_pools: Arc<Mutex<HashMap<u32, Arc<Mutex<SlotPool>>>>>,
    /// Per-MCU `CreditCounter`. Same sharing pattern as `slot_pools`.
    credit_counters: Arc<Mutex<HashMap<u32, Arc<CreditCounter>>>>,
    /// Total number of times the dispatch closure took the
    /// `host_time_to_mcu_clock` fallback path (because the per-MCU clock
    /// estimate had not yet been installed by `set_clock_est`). Production
    /// integration tests assert this stays zero — non-zero indicates klippy
    /// has not wired SET_CLOCK_EST before motion submission.
    fallback_clock_conversions: Arc<AtomicU64>,
    /// Last Klippy clocksync frequency per MCU, mirrored from `set_clock_est`.
    /// The planner emits batch-local seconds; dispatch uses this to place
    /// those relative times onto the MCU's live clock domain.
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    homing: Arc<HomingState>,
}

#[pymethods]
impl PyMotionBridge {
    // ── Task 31: constructor ────────────────────────────────────────────

    #[new]
    fn new() -> Self {
        let clock: Arc<dyn kalico_host_rt::clock::Clock + Send + Sync> = Arc::new(RealClock);
        Self {
            router: Arc::new(Mutex::new(PassthroughRouter::with_clock(clock))),
            parser: Arc::new(Mutex::new(None)),
            mcus: Mutex::new(HashMap::new()),
            events: Arc::new(Mutex::new(VecDeque::new())),
            handlers: Mutex::new(HashMap::new()),
            planner: Mutex::new(None),
            planner_config: Mutex::new(PlannerConfig::default()),
            commanded_pos: Mutex::new([0.0; 3]),
            mcu_axis_configs: Mutex::new(Vec::new()),
            dispatched_segments: Arc::new(AtomicU64::new(0)),
            slot_pools: Arc::new(Mutex::new(HashMap::new())),
            credit_counters: Arc::new(Mutex::new(HashMap::new())),
            fallback_clock_conversions: Arc::new(AtomicU64::new(0)),
            clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            homing: Arc::new(HomingState::new()),
        }
    }


    /// Crate version.
    fn version(&self) -> &'static str {
        env!("CARGO_PKG_VERSION")
    }

    // ── Task 32: claim_mcu ──────────────────────────────────────────────

    /// Register an MCU with the bridge. Returns the opaque handle as int.
    ///
    /// Phase 1: stores the label/path/baud but does NOT open the port.
    /// The actual serial open + identify handshake is Phase 2+.
    #[pyo3(signature = (label, serial_path, baud))]
    fn claim_mcu(&self, label: &str, serial_path: &str, baud: u32) -> PyResult<u32> {
        let mut router = self.router.lock().unwrap();
        let handle = router.claim_mcu(label);
        let raw = handle.raw();
        self.mcus.lock().unwrap().insert(
            raw,
            McuConnection {
                label: label.to_owned(),
                serial_path: serial_path.to_owned(),
                baud,
                host_io: None,
                runtime_rx: None,
            },
        );
        Ok(raw)
    }

    // ── Task 33: release_mcu ────────────────────────────────────────────

    /// Unregister an MCU. Outstanding notify callbacks are dropped.
    fn release_mcu(&self, handle: u32) -> PyResult<()> {
        let mut router = self.router.lock().unwrap();
        router.release_mcu(mcu_handle_from_raw(handle));
        self.mcus.lock().unwrap().remove(&handle);
        self.handlers
            .lock()
            .unwrap()
            .retain(|&(mcu, _, _), _| mcu != handle);
        Ok(())
    }

    // ── Task 34: alloc_command_queue ─────────────────────────────────────

    /// Allocate a command queue for the given MCU. Returns queue id as int.
    fn alloc_command_queue(&self, handle: u32) -> PyResult<u32> {
        let mut router = self.router.lock().unwrap();
        let qid = router
            .alloc_command_queue(mcu_handle_from_raw(handle))
            .map_err(router_err)?;
        Ok(qid.raw())
    }

    // ── Task 35: passthrough_send (fire-and-forget) ─────────────────────

    /// Push a fire-and-forget command to the router.
    #[pyo3(signature = (mcu, queue, data, min_clock=0, req_clock=0))]
    fn passthrough_send(
        &self,
        mcu: u32,
        queue: u32,
        data: &[u8],
        min_clock: u64,
        req_clock: u64,
    ) -> PyResult<()> {
        let entry = PassthroughEntry::new(
            data.to_vec(),
            min_clock,
            req_clock,
            NotifyId::none(),
        );
        let mut router = self.router.lock().unwrap();
        router
            .push(mcu_handle_from_raw(mcu), cq_id_from_raw(queue), entry)
            .map_err(router_err)?;
        Ok(())
    }

    // ── Task 36: passthrough_query (returns notify_id) ──────────────────

    /// Push a command that expects a response. Returns the notify_id as int.
    ///
    /// When the MCU responds (via `dispatch_response` in the reactor),
    /// the response is placed in the events queue and can be retrieved
    /// via `poll_event()`.
    #[pyo3(signature = (mcu, queue, data, min_clock=0, req_clock=0))]
    fn passthrough_query(
        &self,
        mcu: u32,
        queue: u32,
        data: &[u8],
        min_clock: u64,
        req_clock: u64,
    ) -> PyResult<u64> {
        let mut router = self.router.lock().unwrap();
        let mcu_h = mcu_handle_from_raw(mcu);

        // Clone the Arc so the callback can push to the shared event queue.
        let events_ref = Arc::clone(&self.events);
        let mcu_raw = mcu;

        let nid = router
            .register_notify(
                mcu_h,
                Box::new(move |resp| {
                    let ev = BridgeEvent {
                        kind: "query_response".to_owned(),
                        mcu: mcu_raw,
                        notify_id: 0, // filled below
                        response_bytes: resp.bytes,
                        sent_time: resp.sent_time,
                        receive_time: resp.receive_time,
                    };
                    events_ref.lock().unwrap().push_back(ev);
                }),
            )
            .map_err(router_err)?;

        let entry = PassthroughEntry::new(data.to_vec(), min_clock, req_clock, nid);
        router
            .push(mcu_h, cq_id_from_raw(queue), entry)
            .map_err(router_err)?;

        Ok(nid.raw())
    }

    // ── Task 37: passthrough_send_wait_ack (blocking) ───────────────────

    /// Synchronous blocking send-and-wait. Phase 1: scaffold only.
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

    // ── Task 38: passthrough_register_handler ───────────────────────────

    /// Register a typed-response handler. Phase 1: stores it; actual
    /// dispatch comes when the reactor routes responses.
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

    // ── Task 39: passthrough_register_flush_callback ────────────────────

    /// Register a callback that fires when the MCU's queues transition
    /// from non-empty to empty.
    ///
    /// The callback is a Python callable that takes no arguments.
    fn passthrough_register_flush_callback(
        &self,
        mcu: u32,
        callback: PyObject,
    ) -> PyResult<()> {
        let mut router = self.router.lock().unwrap();
        let mcu_h = mcu_handle_from_raw(mcu);

        // Wrap the Python callback so it acquires the GIL when called.
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

    // ── Task 40: poll_event ─────────────────────────────────────────────

    /// Drain one event from the events queue. Returns None if empty.
    fn poll_event(&self, py: Python<'_>) -> PyResult<Option<Py<PyDict>>> {
        let mut events = self.events.lock().unwrap();
        match events.pop_front() {
            Some(ev) => Ok(Some(ev.to_pydict(py)?)),
            None => Ok(None),
        }
    }

    // ── Additional klippy-expected API ──────────────────────────────────

    /// Add a config command for the given MCU.
    fn add_config_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap();
        router
            .add_config_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    /// Add an init command for the given MCU.
    fn add_init_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap();
        router
            .add_init_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    /// Add a restart command for the given MCU.
    fn add_restart_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap();
        router
            .add_restart_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    /// Transition the MCU to the config-sending phase.
    fn begin_config_phase(&self, mcu: u32) -> PyResult<()> {
        let mut router = self.router.lock().unwrap();
        router
            .begin_config_phase(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }

    /// Get the next config/init entry for the given MCU, or None.
    fn next_config_entry(&self, mcu: u32) -> PyResult<Option<Vec<u8>>> {
        let mut router = self.router.lock().unwrap();
        router
            .next_config_entry(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }

    /// Snapshot statistics for the given MCU as a Python dict.
    fn get_stats(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let router = self.router.lock().unwrap();
        let stats = router
            .get_stats(mcu_handle_from_raw(mcu))
            .map_err(router_err)?;
        stats_to_pydict(py, &stats)
    }

    /// Install the MsgProto data dictionary (klippy already retrieves and
    /// parses this during identify; the bridge needs it to encode/decode
    /// passthrough commands inside `RouterTransport`).
    ///
    /// `dict_json` is the raw `identify_response`-payload JSON bytes.
    /// Calling this multiple times replaces the parser.
    fn set_msgproto_dict(&self, dict_json: &[u8]) -> PyResult<()> {
        let json_str = std::str::from_utf8(dict_json)
            .map_err(|e| PyRuntimeError::new_err(format!("dict_json utf8: {e}")))?;
        let dict: DataDictionary = serde_json::from_str(json_str)
            .map_err(|e| PyRuntimeError::new_err(format!("dict json parse: {e}")))?;
        let parser = MsgProtoParser::from_dictionary(dict)
            .map_err(|e| PyRuntimeError::new_err(format!("parser build: {e:?}")))?;
        *self.parser.lock().unwrap() = Some(Arc::new(parser));
        Ok(())
    }

    // ── Phase 1: serial attach + identify ──────────────────────────────

    /// Open the serial port for `mcu_handle`, run the identify handshake,
    /// and spawn the host-rt reactor thread that owns the FD.
    ///
    /// Blocks until the port is open and identify completes (or the
    /// 30-second retry window expires). The raw identify bytes (zlib
    /// blob from firmware) are stored and can be retrieved via
    /// `get_identify_data`.
    ///
    /// Call once per MCU after `claim_mcu`. Calling again on an already-
    /// attached MCU replaces the existing `KalicoHostIo`.
    #[pyo3(signature = (mcu_handle, serial_path, baud, timeout_s = 30.0))]
    fn attach_serial(
        &self,
        mcu_handle: u32,
        serial_path: &str,
        baud: u32,
        timeout_s: f64,
    ) -> PyResult<()> {
        use std::time::{Duration, Instant};
        let deadline = Instant::now() + Duration::from_secs_f64(timeout_s);
        let effective_baud = if baud == 0 { 250_000 } else { baud };
        let config = KalicoHostIoConfig::default();

        // Determine whether this is a PTY/pipe path (baud=0 signals pipe mode)
        // or a real serial port. Pipe mode uses O_RDWR | O_NOCTTY to open the
        // PTY without configuring baud rate, which serialport::open() would do
        // and which interferes with Linux pseudo-terminals.
        let is_pipe = baud == 0
            || serial_path.starts_with("/tmp/")
            || serial_path.starts_with("/dev/pts/")
            || serial_path.contains("klipper_host")
            || serial_path.contains("klipper_sim");

        let host_io = loop {
            let result = if is_pipe {
                #[cfg(target_family = "unix")]
                { KalicoHostIo::open_pipe_with_config(serial_path, config.clone()) }
                #[cfg(not(target_family = "unix"))]
                { KalicoHostIo::open_with_config(serial_path, effective_baud, config.clone()) }
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

        // Subscribe to runtime events before storing so no events are missed.
        let runtime_rx = host_io
            .take_runtime_event_subscription()
            .map_err(|e| PyRuntimeError::new_err(format!("attach_serial: runtime_event subscribe: {e:?}")))?;

        let mut mcus = self.mcus.lock().unwrap();
        let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "attach_serial: unknown mcu_handle {mcu_handle}"
            ))
        })?;
        conn.host_io = Some(Arc::new(host_io));
        conn.runtime_rx = Some(runtime_rx);
        Ok(())
    }

    /// Return the raw identify bytes (zlib-compressed firmware data-dict)
    /// for the given MCU. `attach_serial` must have been called first.
    ///
    /// Pass the returned bytes to klippy's
    /// `msgproto.MessageParser.process_identify(data)`.
    fn get_identify_data(&self, mcu_handle: u32) -> PyResult<Vec<u8>> {
        let io = {
            let mcus = self.mcus.lock().unwrap();
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "get_identify_data: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            conn.host_io.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err(
                    "get_identify_data: attach_serial has not been called for this MCU",
                )
            })?.clone()
        };
        Ok(io.raw_identify_bytes().to_vec())
    }

    /// Send a human-readable msgproto command and wait for a response.
    ///
    /// Equivalent to klippy's `serial.send_with_response(msg, response)`.
    /// Returns a Python dict of the response parameters.
    ///
    /// `msg` is a command string like `"get_uptime"` or `"get_clock"`.
    /// `response` is the expected response name like `"uptime"` or `"clock"`.
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

        // Get the submission_tx from KalicoHostIo — we need to submit
        // without holding the mutex across a blocking call. KalicoHostIo::call
        // uses mpsc internally and blocks; we must release the mcus lock before
        // calling it. We do this by cloning the sender out while locked, then
        // calling after unlock. Unfortunately KalicoHostIo doesn't expose its
        // sender directly, so we use py.allow_threads with a short-lived lock.
        //
        // Safe because py.allow_threads drops the GIL; the mcus mutex guards
        // McuConnection which is Send (KalicoHostIo is Send).
        // Clone the Arc out of the mutex so we can call blocking I/O without
        // holding the lock.
        let io = {
            let mcus = self.mcus.lock().unwrap();
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "bridge_call: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            conn.host_io.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err(
                    "bridge_call: attach_serial has not been called for this MCU",
                )
            })?.clone()
        };

        let msg_owned = msg.to_owned();
        let response_owned = response.to_owned();
        let params = py.allow_threads(|| -> PyResult<_> {
            use kalico_host_rt::transport::Transport;
            io.call(&msg_owned, &response_owned, Duration::from_secs_f64(timeout_s))
                .map_err(|e| PyRuntimeError::new_err(format!("bridge_call: {e}")))
        })?;

        let d = PyDict::new(py);
        for (k, v) in &params.fields {
            use kalico_host_rt::transport::MessageValue;
            match v {
                MessageValue::U32(n) => d.set_item(k, n)?,
                MessageValue::I32(n) => d.set_item(k, n)?,
                MessageValue::U64(n) => d.set_item(k, n)?,
                MessageValue::Bytes(b) => d.set_item(k, pyo3::types::PyBytes::new(py, b.as_slice()))?,
                MessageValue::String(s) => d.set_item(k, s)?,
            }
        }
        Ok(d.unbind())
    }

    /// Drain one runtime event from the MCU's event queue.
    ///
    /// Returns a Python dict describing the event (with a `"type"` key),
    /// or `None` if no event is pending. Klippy registers a reactor timer
    /// that polls this and dispatches to registered handlers.
    ///
    /// Event types emitted:
    ///   - `"status"`: kalico_status_v6 heartbeat — keys: `engine_status`,
    ///     `current_segment_id`, `last_fault`, `fault_detail`
    ///   - `"credit_freed"`: kalico_credit_freed — keys: `retired_through_segment_id`,
    ///     `free_slots`
    ///   - `"fault"`: kalico_fault — keys: `fault_code`, `fault_detail`,
    ///     `segment_id`, `synthesized`
    ///   - `"output"`: #output / unknown output — keys: `format`, `msg`
    fn take_runtime_event(&self, py: Python<'_>, mcu_handle: u32) -> PyResult<Option<Py<PyDict>>> {
        use kalico_host_rt::host_io::runtime_events::RuntimeEvent;
        use std::sync::mpsc::TryRecvError;

        let event = {
            let mut mcus = self.mcus.lock().unwrap();
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
                d.set_item("current_segment_id", s.current_segment_id)?;
                d.set_item("last_fault", s.last_fault)?;
                d.set_item("fault_detail", s.fault_detail)?;
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
                // Trace events are not klippy-visible; skip silently.
                return Ok(None);
            }
            RuntimeEvent::UnknownOutput { format, msg } => {
                d.set_item("type", "output")?;
                d.set_item("format", format)?;
                d.set_item("msg", msg)?;
            }
        }
        Ok(Some(d.unbind()))
    }

    /// Send a fire-and-forget command to the MCU (no response expected).
    ///
    /// Used for config-phase commands like `allocate_oids`, `config_stepper`,
    /// `finalize_config` where the MCU processes the command but sends no reply.
    /// The frame is still wire-level ACKed; only the application-level response
    /// is absent.
    #[pyo3(signature = (mcu_handle, msg))]
    fn bridge_send(&self, mcu_handle: u32, msg: &str) -> PyResult<()> {
        let io = {
            let mcus = self.mcus.lock().unwrap();
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!(
                    "bridge_send: unknown mcu_handle {mcu_handle}"
                ))
            })?;
            conn.host_io.as_ref().ok_or_else(|| {
                PyRuntimeError::new_err(
                    "bridge_send: attach_serial has not been called for this MCU",
                )
            })?.clone()
        };
        io.send_fire_and_forget(msg)
            .map_err(|e| PyRuntimeError::new_err(format!("bridge_send: {e}")))
    }

    /// Update clock estimation parameters for the given MCU.
    #[pyo3(signature = (mcu, freq, offset, last_clock))]
    fn set_clock_est(
        &self,
        mcu: u32,
        freq: f64,
        offset: f64,
        last_clock: u64,
    ) -> PyResult<()> {
        let mut router = self.router.lock().unwrap();
        router
            .set_clock_est(mcu_handle_from_raw(mcu), freq, offset, last_clock)
            .map_err(router_err)?;
        self.clock_freqs.lock().unwrap().insert(mcu, freq);
        Ok(())
    }

    /// Drain the debug log for crash diagnostics. Returns a dict with
    /// `sent` and `received` lists of dicts.
    fn extract_old(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let mut router = self.router.lock().unwrap();
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

    // ── Task 8: motion-submission methods ───────────────────────────────

    /// Initialize the planner thread with config from `printer.cfg`.
    ///
    /// `octopus_handle` and `f446_handle` are the raw `claim_mcu()` handles
    /// for the two-MCU first-print MVP topology:
    ///   - Octopus drives X+Y (CoreXyAndE = kinematics 0).
    ///   - F446 drives Z (CartesianXyzAndE = kinematics 1).
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
        octopus_handle,
        f446_handle,
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
        octopus_handle: u32,
        f446_handle: u32,
        window_capacity: usize,
        beta_max_iters: u8,
    ) -> PyResult<()> {
        let mut planner_slot = self.planner.lock().unwrap();
        if planner_slot.is_some() {
            return Err(PyRuntimeError::new_err(
                "planner already initialized",
            ));
        }

        let shaper = build_shaper_config(
            shaper_type_x,
            shaper_freq_x,
            shaper_type_y,
            shaper_freq_y,
        )
        .map_err(PyRuntimeError::new_err)?;

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

        // Persist for runtime updates.
        *self.planner_config.lock().unwrap() = cfg.clone();

        // Two-MCU first-print MVP topology.
        let mcu_configs = vec![
            McuAxisConfig {
                mcu_id: octopus_handle,
                axes: vec![AXIS_X, AXIS_Y],
                kinematics: 0, // CoreXyAndE
            },
            McuAxisConfig {
                mcu_id: f446_handle,
                axes: vec![AXIS_Z],
                kinematics: 1, // CartesianXyzAndE
            },
        ];
        *self.mcu_axis_configs.lock().unwrap() = mcu_configs.clone();

        // ── Task 8b: wire the dispatch closure to producer::load_curve /
        // producer::push_segment via KalicoHostIo ─────────────────────────
        //
        // Per-MCU state captured into the closure:
        //   * a CreditCounter pre-seeded to CREDIT_SEED_CAPACITY (option A:
        //     no real `kalico_credit_freed` accounting yet),
        //   * the KalicoHostIo reactor handle that owns this MCU's wire.
        //
        // The closure then, per ShapedSegment:
        //   1. converts `t_start` / `t_end` (print-time seconds) to MCU
        //      clock via `PassthroughRouter::host_time_to_mcu_clock`;
        //   2. builds per-MCU push plans (`build_push_params`);
        //   3. for each plan: `load_curve` per axis, then `push_segment`.
        //
        // Errors are propagated as `Err(String)` so the planner thread
        // surfaces them as `PlannerError::Dispatch`.
        let counter = Arc::clone(&self.dispatched_segments);
        let fallback_counter = Arc::clone(&self.fallback_clock_conversions);
        let clock_freqs = Arc::clone(&self.clock_freqs);
        let homing = Arc::clone(&self.homing);
        let warned_mcus: Arc<Mutex<HashSet<u32>>> =
            Arc::new(Mutex::new(HashSet::new()));
        let router_arc = Arc::clone(&self.router);

        let host_ios: HashMap<u32, Arc<KalicoHostIo>> = {
            let mcus = self.mcus.lock().unwrap();
            let mut out = HashMap::new();
            for cfg_mcu in &mcu_configs {
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

        // Per-MCU dispatch context (host I/O + credit + slot pool) keyed by
        // mcu_id. `dispatch_ios` is the closure-local lookup map; the credit
        // and slot-pool tables on `self` are the persistent ones the
        // event-routing API (`on_credit_freed`) drives.
        let mut dispatch_ios: HashMap<u32, (Arc<KalicoHostIo>, Arc<CreditCounter>, Arc<Mutex<SlotPool>>)> =
            HashMap::new();
        let mut self_credits = self.credit_counters.lock().unwrap();
        let mut self_pools = self.slot_pools.lock().unwrap();
        self_credits.clear();
        self_pools.clear();
        for cfg_mcu in &mcu_configs {
            let io = host_ios
                .get(&cfg_mcu.mcu_id)
                .expect("host_io map built from mcu_configs")
                .clone();
            let credit = Arc::new(CreditCounter::new(CREDIT_SEED_CAPACITY));
            io.attach_credit_counter(Arc::clone(&credit));
            let slot_pool = Arc::new(Mutex::new(SlotPool::new()));
            self_credits.insert(cfg_mcu.mcu_id, Arc::clone(&credit));
            self_pools.insert(cfg_mcu.mcu_id, Arc::clone(&slot_pool));
            dispatch_ios.insert(
                cfg_mcu.mcu_id,
                (io, credit, slot_pool),
            );
        }
        drop(self_credits);
        drop(self_pools);

        let mcu_configs_for_cb = mcu_configs;
        let router_for_cb = Arc::clone(&router_arc);

        // Per-MCU rolling segment id. Allocated alongside the slot to
        // bind the `kalico_credit_freed.retired_through_segment_id`
        // retirement signal to the segment's curve slots.
        let next_seg_id: Arc<Mutex<HashMap<u32, u32>>> =
            Arc::new(Mutex::new(HashMap::new()));
        // Per-MCU schedule state:
        //   (current batch base clock, next available absolute clock).
        // `trajectory::shape_batch` emits batch-local times, with each new
        // batch starting at t=0. Dispatch places those relative seconds onto
        // the MCU's live clock with a small lead so the firmware does not see
        // zero-duration or already-expired segments.
        let schedule_state: Arc<Mutex<HashMap<u32, (u64, u64)>>> =
            Arc::new(Mutex::new(HashMap::new()));

        let dispatch: Arc<
            dyn Fn(&trajectory::ShapedSegment) -> Result<(), String>
                + Send
                + Sync,
        > = Arc::new(move |seg: &trajectory::ShapedSegment| -> Result<(), String> {
            // ── Phase-4 per-axis-per-segment dispatch ─────────────────────
            //
            // The B.1 multi-piece chunker has been retired (see spec
            // `docs/superpowers/specs/2026-05-04-incremental-curve-upload-design.md`
            // §6.3): the wire-fit reason for chunking is gone now that
            // `producer::load_curve` uses the begin/N×chunk/finalize
            // incremental upload protocol, and the §5.0 pool bump
            // accommodates the trajectory layer's worst-case post-shape
            // piece count in a single logical-move dispatch. So K=1
            // load_curve + 1 push_segment per axis per logical move per MCU.
            //
            // Per-MCU clock derivation runs ONCE per logical segment, not
            // once per chunk. `homing.mark_dispatched_segment` and
            // `next_seg_id` allocation also happen once per logical move.

            // Build per-axis-per-segment plans first; we still need clocks
            // before we can fill in the timing fields.
            let mcu_plans = build_push_params(seg, &mcu_configs_for_cb, 0, 0);

            for mut plan in mcu_plans {
                let (io, credit, slot_pool) = match dispatch_ios.get(&plan.mcu_id) {
                    Some(v) => v,
                    None => continue,
                };

                // Per-MCU clock conversion. Falls back to a microsecond
                // approximation if `set_clock_est` has not been called yet.
                let mcu_h = mcu_handle_from_raw(plan.mcu_id);
                let freq = clock_freqs
                    .lock()
                    .unwrap()
                    .get(&plan.mcu_id)
                    .copied()
                    .filter(|f| *f > 0.0)
                    .unwrap_or_else(|| {
                        fallback_counter.fetch_add(1, Ordering::Relaxed);
                        let first_for_mcu = {
                            let mut warned = warned_mcus.lock().unwrap();
                            warned.insert(plan.mcu_id)
                        };
                        if first_for_mcu {
                            log::warn!(
                                "motion-bridge: MCU {} clock frequency not installed; using 1 MHz fallback for relative segment timing. SET_CLOCK_EST not yet wired by klippy?",
                                plan.mcu_id
                            );
                        }
                        1_000_000.0
                    });

                // Compute schedule base ONCE per (mcu, logical-segment).
                let mcu_base_clock: u64 = {
                    let r = router_for_cb.lock().unwrap();
                    let now_clock = r
                        .compute_ack_clock(mcu_h)
                        .map_err(|e| format!("compute_ack_clock: {e}"))?;
                    let lead_cycles = (freq * 0.100).round().max(1.0) as u64;
                    drop(r);

                    let mut schedule = schedule_state.lock().unwrap();
                    let entry = schedule.entry(plan.mcu_id).or_insert((0, 0));
                    if entry.1 == 0 || seg.t_start <= 1.0e-12 {
                        entry.0 = entry.1.max(now_clock.saturating_add(lead_cycles));
                    }
                    if entry.0 < now_clock.saturating_add(lead_cycles) {
                        entry.0 = now_clock.saturating_add(lead_cycles);
                    }
                    entry.0
                };

                // Segment time window in MCU clocks. `seg.t_start` /
                // `seg.t_end` are absolute seconds in the trajectory batch
                // timeline; convert to MCU-clock relative to mcu_base_clock.
                let rel_start = (seg.t_start * freq).round().max(0.0) as u64;
                let rel_end = (seg.t_end * freq).round().max(0.0) as u64;
                let t_start_clock = mcu_base_clock.saturating_add(rel_start);
                let t_end_clock = mcu_base_clock.saturating_add(rel_end);

                // Update tail of schedule so the next logical segment sees
                // the correct end-of-batch.
                {
                    let mut schedule = schedule_state.lock().unwrap();
                    let entry = schedule.entry(plan.mcu_id).or_insert((0, 0));
                    entry.1 = entry.1.max(t_end_clock);
                }

                plan.params.t_start = t_start_clock;
                plan.params.t_end = t_end_clock;

                // Allocate a fresh segment id for this logical move (one per
                // MCU per ShapedSegment, restoring pre-B.1 semantics).
                {
                    let mut ids = next_seg_id.lock().unwrap();
                    let entry = ids.entry(plan.mcu_id).or_insert(1);
                    plan.params.id = *entry;
                    *entry = entry.wrapping_add(1);
                }
                homing.mark_dispatched_segment(plan.params.id);

                // Allocate slots, load curves. On any error, release every
                // slot allocated so far for this segment so the pool doesn't
                // leak. Each `producer::load_curve` call expands internally
                // to begin + N×chunk + finalize over the wire.
                let mut allocated_slots: Vec<u16> =
                    Vec::with_capacity(plan.curves_to_load.len());
                let mut seg_err: Option<String> = None;
                for i in 0..plan.curves_to_load.len() {
                    let axis_idx = plan.curves_to_load[i].0;
                    let curve_params = plan.curves_to_load[i].1.clone();
                    let alloc_result = {
                        let mut pool = slot_pool.lock().unwrap();
                        pool.try_alloc().ok_or_else(|| {
                            format!(
                                "slot pool exhausted for mcu={} (capacity={CURVE_POOL_N}, in_flight={}); \
                                 awaiting kalico_credit_freed retirement events",
                                plan.mcu_id,
                                pool.in_flight_count(),
                            )
                        })
                    };
                    let (slot, _gen) = match alloc_result {
                        Ok(v) => v,
                        Err(e) => {
                            seg_err = Some(e);
                            break;
                        }
                    };
                    allocated_slots.push(slot);
                    match producer::load_curve(
                        io.as_ref(),
                        slot,
                        &curve_params,
                        producer::DEFAULT_LOAD_CURVE_TIMEOUT,
                    ) {
                        Ok(handle) => {
                            plan.set_handle(axis_idx, handle);
                        }
                        Err(e) => {
                            seg_err = Some(format!(
                                "load_curve mcu={}: {e}",
                                plan.mcu_id
                            ));
                            break;
                        }
                    }
                }

                if let Some(err) = seg_err {
                    // Partial failure: release every slot allocated for this
                    // segment before propagating.
                    let mut pool = slot_pool.lock().unwrap();
                    for s in &allocated_slots {
                        pool.release(*s);
                    }
                    return Err(err);
                }

                // Bind every freshly-allocated slot to this segment id so
                // `kalico_credit_freed`-driven retirement can release them.
                {
                    let mut pool = slot_pool.lock().unwrap();
                    for slot in &allocated_slots {
                        pool.register_segment(*slot, plan.params.id);
                    }
                }

                if let Err(e) = push_segment_fire_and_forget(
                    io.as_ref(),
                    credit,
                    &plan.params,
                ) {
                    // Defensive cleanup — release this segment's slots so
                    // the pool doesn't leak (the MCU never accepted them).
                    let mut pool = slot_pool.lock().unwrap();
                    for s in &allocated_slots {
                        pool.release(*s);
                    }
                    return Err(format!(
                        "push_segment mcu={}: {e}",
                        plan.mcu_id
                    ));
                }
            }

            counter.fetch_add(1, Ordering::Relaxed);
            Ok(())
        });

        *planner_slot = Some(PlannerHandle::spawn(cfg, dispatch));
        Ok(())
    }

    /// Submit a travel move. Phase 2: `de` must be 0.
    #[pyo3(signature = (dx, dy, dz, de, feedrate))]
    fn submit_move(
        &self,
        dx: f64,
        dy: f64,
        dz: f64,
        de: f64,
        feedrate: f64,
    ) -> PyResult<()> {
        let pos = *self.commanded_pos.lock().unwrap();
        let classified =
            classify::classify_and_build(pos, dx, dy, dz, de, feedrate)
                .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

        let planner_guard = self.planner.lock().unwrap();
        let planner = planner_guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "planner not initialized — call init_planner first",
            )
        })?;
        planner.submit_move(classified).map_err(planner_err)?;
        drop(planner_guard);

        let mut pos = self.commanded_pos.lock().unwrap();
        pos[0] += dx;
        pos[1] += dy;
        pos[2] += dz;
        Ok(())
    }

    /// Submit one homing-tagged absolute move. MVP watches the first arm id;
    /// multi-arm logical OR is Step 10.
    #[pyo3(signature = (newpos, speed, arm_ids))]
    fn submit_homing_move(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        arm_ids: Vec<u32>,
    ) -> PyResult<()> {
        self.submit_homing_move_inner(&newpos, speed, &arm_ids)
    }

    /// Flush all pending moves and block until the planner has shaped them.
    fn wait_moves(&self) -> PyResult<()> {
        let planner_guard = self.planner.lock().unwrap();
        let planner = planner_guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "planner not initialized — call init_planner first",
            )
        })?;
        planner.flush().map_err(planner_err)?;
        self.homing.refresh_after_wait();
        Ok(())
    }

    fn take_trip_event(&self, py: Python<'_>) -> PyResult<Option<Py<PyDict>>> {
        let Some(evt) = self.homing.take_trip_event() else {
            return Ok(None);
        };
        Ok(Some(trip_event_to_pydict(py, evt)?))
    }

    // ── Step 7-D: endstop arm/disarm/set_homed_state wire surface ──────────
    //
    // These call the kalico-host-rt producer functions over the same
    // KalicoHostIo reactor queue used by bridge_call / bridge_send.
    // Each Python call is one synchronous msgproto round-trip. The Python
    // side (`klippy/motion_bridge.py::BridgeTriggerDispatch`) wraps these
    // and handles async `kalico_endstop_tripped` events via the existing
    // `passthrough_register_handler` plumbing.

    /// Send `kalico_arm_endstop` and wait for the synchronous response.
    /// Returns the status byte (0=Armed, 1=AlreadyTripped, 2=Rejected) per
    /// spec §3.2.
    #[pyo3(signature = (mcu, queue, arm_id, arm_clock, sources, stepper_oids, timeout_s=0.1))]
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
        Ok(status as u8)
    }

    /// Send `kalico_disarm_endstop` and wait for the response. Returns the
    /// status byte (0=Disarmed, 1=AlreadyTripped, 2=Unknown) per spec §3.5.
    #[pyo3(signature = (mcu, queue, arm_id, timeout_s=0.1))]
    fn endstop_disarm(
        &self,
        mcu: u32,
        queue: u32,
        arm_id: u32,
        timeout_s: f64,
    ) -> PyResult<u8> {
        use kalico_host_rt::endstop;
        let _ = queue;
        let io = self.host_io_for_mcu("endstop_disarm", mcu)?;
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let status = endstop::disarm_endstop_with_timeout(io.as_ref(), arm_id, timeout)
            .map_err(|e| PyRuntimeError::new_err(format!("endstop_disarm: {e}")))?;
        Ok(status as u8)
    }

    /// Send `kalico_set_homed_state homed=%c`. Spec §8.
    #[pyo3(signature = (mcu, queue, homed, timeout_s=0.1))]
    fn set_homed_state(
        &self,
        mcu: u32,
        queue: u32,
        homed: bool,
        timeout_s: f64,
    ) -> PyResult<()> {
        use kalico_host_rt::endstop;
        let _ = queue;
        let io = self.host_io_for_mcu("set_homed_state", mcu)?;
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        endstop::set_homed_state_with_timeout(io.as_ref(), homed, timeout)
            .map_err(|e| PyRuntimeError::new_err(format!("set_homed_state: {e}")))
    }

    /// Submit a dwell: flush + advance print time.
    fn submit_dwell(&self, duration_s: f64) -> PyResult<()> {
        let planner_guard = self.planner.lock().unwrap();
        let planner = planner_guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "planner not initialized — call init_planner first",
            )
        })?;
        planner.dwell(duration_s).map_err(planner_err)
    }

    /// Reset commanded position. The planner does not track absolute
    /// position (only print_time), so this is a bridge-local update.
    fn set_position(&self, x: f64, y: f64, z: f64) -> PyResult<()> {
        let mut pos = self.commanded_pos.lock().unwrap();
        *pos = [x, y, z];
        Ok(())
    }

    /// Update velocity / acceleration limits at runtime
    /// (klippy `SET_VELOCITY_LIMIT`).
    fn update_limits(
        &self,
        max_velocity: f64,
        max_accel: f64,
    ) -> PyResult<()> {
        let mut cfg = self.planner_config.lock().unwrap();
        cfg.limits.max_velocity = max_velocity;
        cfg.limits.max_accel = max_accel;
        let new_limits = cfg.limits;
        drop(cfg);

        let planner_guard = self.planner.lock().unwrap();
        let planner = planner_guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "planner not initialized — call init_planner first",
            )
        })?;
        planner.update_limits(new_limits).map_err(planner_err)
    }

    /// Update shaper config at runtime (klippy `SET_INPUT_SHAPER`).
    fn update_shaper(
        &self,
        shaper_type_x: &str,
        freq_x: f64,
        shaper_type_y: &str,
        freq_y: f64,
    ) -> PyResult<()> {
        let shaper = build_shaper_config(
            shaper_type_x,
            freq_x,
            shaper_type_y,
            freq_y,
        )
        .map_err(PyRuntimeError::new_err)?;

        self.planner_config.lock().unwrap().shaper = shaper.clone();

        let planner_guard = self.planner.lock().unwrap();
        let planner = planner_guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "planner not initialized — call init_planner first",
            )
        })?;
        planner.update_shaper(shaper).map_err(planner_err)
    }

    /// Estimated print time of the last queued move, in seconds.
    fn get_last_move_time(&self) -> f64 {
        let planner_guard = self.planner.lock().unwrap();
        match planner_guard.as_ref() {
            Some(p) => p.last_move_time(),
            None => 0.0,
        }
    }

    /// Number of shaped segments observed by the dispatch callback. Test /
    /// sim hook — not part of the klippy-facing API.
    fn dispatched_segment_count(&self) -> u64 {
        self.dispatched_segments.load(Ordering::Relaxed)
    }

    /// Drive the bridge with a `kalico_credit_freed` event.
    ///
    /// `retired_through_segment_id` releases every curve slot bound to a
    /// segment id `<= retired_through_segment_id` in the per-MCU
    /// [`SlotPool`]. `free_slots` is reconciled into the per-MCU
    /// [`CreditCounter`] (the MCU is authoritative — see
    /// [`CreditCounter::on_credit_freed`]).
    ///
    /// Returns the number of curve slots released. Unknown MCU is a no-op
    /// returning 0 (defensive — events for un-claimed MCUs are dropped).
    ///
    /// Wire-routing note: as of HEAD `799bdd867` no host-side serial
    /// reactor inside the bridge calls this. klippy's reactor receives
    /// `kalico_credit_freed` over its existing serial loop and is
    /// expected to forward the event into this method once the routing
    /// hook is wired.
    fn on_credit_freed(
        &self,
        mcu: u32,
        retired_through_segment_id: u32,
        free_slots: u8,
    ) -> PyResult<u32> {
        let n_released = match self.slot_pools.lock().unwrap().get(&mcu) {
            Some(pool_arc) => pool_arc
                .lock()
                .unwrap()
                .retire_through_segment(retired_through_segment_id),
            None => 0,
        };
        if let Some(c) = self.credit_counters.lock().unwrap().get(&mcu) {
            c.on_credit_freed(free_slots);
        }
        self.homing.complete_if_retired(retired_through_segment_id);
        Ok(n_released as u32)
    }

    /// Number of curve slots currently in flight on the given MCU. Test /
    /// diagnostic hook.
    fn slot_pool_in_flight(&self, mcu: u32) -> u32 {
        self.slot_pools
            .lock()
            .unwrap()
            .get(&mcu)
            .map(|p| p.lock().unwrap().in_flight_count() as u32)
            .unwrap_or(0)
    }

    /// Available credit for the given MCU. Test / diagnostic hook.
    fn credit_available(&self, mcu: u32) -> i32 {
        self.credit_counters
            .lock()
            .unwrap()
            .get(&mcu)
            .map(|c| c.available())
            .unwrap_or(0)
    }

    /// Number of times the dispatch closure took the `t * 1e6` fallback
    /// path because `set_clock_est` had not yet been wired for the target
    /// MCU. Production integration tests assert this stays zero — non-zero
    /// indicates SET_CLOCK_EST was not called before motion submission.
    fn fallback_clock_conversions(&self) -> u64 {
        self.fallback_clock_conversions.load(Ordering::Relaxed)
    }
}

impl PyMotionBridge {
    fn host_io_for_mcu(&self, caller: &str, mcu: u32) -> PyResult<Arc<KalicoHostIo>> {
        let mcus = self.mcus.lock().unwrap();
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
        // TODO(Step 10): accept all arm_ids as a logical OR set.
        self.homing.begin(arm_id);

        let pos = *self.commanded_pos.lock().unwrap();
        let classified = classify::classify_and_build(
            pos,
            newpos[0] - pos[0],
            newpos[1] - pos[1],
            newpos[2] - pos[2],
            0.0,
            speed,
        )
        .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;

        let planner_guard = self.planner.lock().unwrap();
        let planner = planner_guard.as_ref().ok_or_else(|| {
            PyRuntimeError::new_err(
                "planner not initialized — call init_planner first",
            )
        })?;
        if let Err(e) = planner.submit_move(classified) {
            self.homing.reset_to_idle();
            return Err(planner_err(e));
        }
        Ok(())
    }
}

#[cfg(test)]
mod credit_freed_tests {
    //! Tests for the `on_credit_freed` PyO3 entry point — the klippy-side
    //! glue that forwards `kalico_credit_freed` MCU events into the
    //! per-MCU `SlotPool` for slot retirement.
    //!
    //! Constructing `PyMotionBridge::new()` doesn't touch Python state
    //! (the `#[new]` body is a plain `Self {...}` literal), and
    //! `on_credit_freed` itself only manipulates Rust mutexes — so we
    //! drive it directly without `Python::with_gil`.

    use super::*;

    /// Inject a slot pool + credit counter for a synthetic MCU handle so
    /// `on_credit_freed` has something to operate on. `init_planner`
    /// normally does this; tests bypass the planner thread.
    fn install_mcu(bridge: &PyMotionBridge, mcu: u32) -> Arc<Mutex<SlotPool>> {
        let pool = Arc::new(Mutex::new(SlotPool::new()));
        bridge.slot_pools.lock().unwrap().insert(mcu, Arc::clone(&pool));
        let credit = Arc::new(CreditCounter::new(CREDIT_SEED_CAPACITY));
        bridge.credit_counters.lock().unwrap().insert(mcu, credit);
        pool
    }

    #[test]
    fn on_credit_freed_releases_eligible_slots() {
        let bridge = PyMotionBridge::new();
        let mcu = 1u32;
        let pool = install_mcu(&bridge, mcu);

        // Allocate three in-flight segments with monotonic ids.
        {
            let mut p = pool.lock().unwrap();
            for seg_id in 1u32..=3 {
                let (slot, _credit) = p
                    .try_alloc()
                    .expect("pool has capacity for three allocs");
                p.register_segment(slot, seg_id);
            }
            assert_eq!(p.in_flight_count(), 3);
        }

        // MCU reports retirement through segment 2 — slots for ids 1,2 free.
        let n = bridge
            .on_credit_freed(mcu, 2, /* free_slots */ 2)
            .expect("on_credit_freed returns Ok");
        assert_eq!(n, 2, "two slots should be released");
        assert_eq!(pool.lock().unwrap().in_flight_count(), 1);

        // Higher-id retirement releases the rest.
        let n = bridge
            .on_credit_freed(mcu, 100, 1)
            .expect("on_credit_freed returns Ok");
        assert_eq!(n, 1);
        assert_eq!(pool.lock().unwrap().in_flight_count(), 0);
    }

    #[test]
    fn on_credit_freed_unknown_mcu_is_noop() {
        // A retirement event for an MCU we don't track must not panic and
        // must report zero released. Defensive — guards against the bridge
        // being mid-teardown when an event arrives.
        let bridge = PyMotionBridge::new();
        let n = bridge
            .on_credit_freed(/* mcu */ 99, /* retired */ 5, /* free */ 1)
            .expect("on_credit_freed must not error on unknown MCU");
        assert_eq!(n, 0);
    }

    #[test]
    fn on_credit_freed_before_any_alloc_is_noop() {
        // Startup race: MCU emits a credit_freed before any segment has
        // been dispatched. retire_through_segment is idempotent on an
        // empty pool — verify the bridge's PyO3 entry point inherits that.
        let bridge = PyMotionBridge::new();
        let mcu = 1u32;
        install_mcu(&bridge, mcu);
        let n = bridge
            .on_credit_freed(mcu, u32::MAX, 0)
            .expect("on_credit_freed must not error on empty pool");
        assert_eq!(n, 0);
    }
}

fn trip_event_to_pydict(
    py: Python<'_>,
    evt: runtime::endstop::TripEvent,
) -> PyResult<Py<PyDict>> {
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
