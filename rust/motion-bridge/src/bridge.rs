//! `PyMotionBridge` — the PyO3 class that klippy calls.
//!
//! Phase 1: direct wrapper around `PassthroughRouter`. No reactor threads,
//! no real serial I/O. The API surface matches what klippy will need so
//! that the Python-side code can be developed in parallel.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

use kalico_host_rt::clock::RealClock;
use kalico_host_rt::passthrough_queue::{
    NotifyId, PassthroughEntry, PassthroughRouter,
};

use crate::types::{cq_id_from_raw, mcu_handle_from_raw, stats_to_pydict};

// ── Internal types ──────────────────────────────────────────────────────

/// Metadata stored per claimed MCU. Phase 1 only stores connection params;
/// actual serial open happens in Phase 2+.
#[derive(Debug)]
struct McuConnection {
    #[allow(dead_code)]
    label: String,
    #[allow(dead_code)]
    serial_path: String,
    #[allow(dead_code)]
    baud: u32,
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

// ── PyMotionBridge ──────────────────────────────────────────────────────

#[pyclass(name = "MotionBridge")]
#[allow(missing_debug_implementations)]
pub struct PyMotionBridge {
    router: Mutex<PassthroughRouter>,
    mcus: Mutex<HashMap<u32, McuConnection>>,
    /// Shared event queue — callbacks capture an `Arc` clone so they can
    /// push events from any thread without holding a reference to `self`.
    events: Arc<Mutex<VecDeque<BridgeEvent>>>,
    /// Typed-response handlers registered via `passthrough_register_handler`.
    /// Key: `(mcu_handle, name, oid)`.  Phase 1 stores them but does not
    /// dispatch — actual dispatch requires the reactor thread.
    #[allow(dead_code)]
    handlers: Mutex<HashMap<(u32, String, u32), PyObject>>,
}

#[pymethods]
impl PyMotionBridge {
    // ── Task 31: constructor ────────────────────────────────────────────

    #[new]
    fn new() -> Self {
        let clock: Arc<dyn kalico_host_rt::clock::Clock + Send + Sync> = Arc::new(RealClock);
        Self {
            router: Mutex::new(PassthroughRouter::with_clock(clock)),
            mcus: Mutex::new(HashMap::new()),
            events: Arc::new(Mutex::new(VecDeque::new())),
            handlers: Mutex::new(HashMap::new()),
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
            .map_err(router_err)
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
}
