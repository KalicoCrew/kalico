//! `PyMotionBridge` — the PyO3 class that klippy calls.
//!
//! Phase 1: direct wrapper around `PassthroughRouter`. No reactor threads,
//! no real serial I/O. The API surface matches what klippy will need so
//! that the Python-side code can be developed in parallel.

use std::collections::{HashMap, HashSet, VecDeque};
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::mpsc::Receiver;
use std::sync::{Arc, Mutex, OnceLock, Weak};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;

use kalico_host_rt::clock::RealClock;
use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::host_io::parser::{DataDictionary, MsgProtoParser};
use kalico_host_rt::host_io::{KalicoHostIo, KalicoHostIoConfig};
use kalico_host_rt::passthrough_queue::{NotifyId, PassthroughEntry, PassthroughRouter};
use kalico_host_rt::producer;
use trajectory::{AxisShaper, ShaperConfig};

use crate::classify;
use crate::config::{self, PlannerConfig, PlannerLimits, parse_required_shaper};
use crate::dispatch::{AXIS_X, AXIS_Y, AXIS_Z, McuAxisConfig, McuCaps, build_push_params};
use crate::homing::HomingState;
use crate::planner::{DispatchError, PlannerError, PlannerHandle};
use crate::slot_pool::{CURVE_POOL_N, SlotPool};
use crate::types::{cq_id_from_raw, mcu_handle_from_raw, stats_to_pydict};

/// Initial credit seed for the per-MCU `CreditCounter`. The bridge wires
/// `kalico_credit_freed` events into [`CreditCounter::on_credit_freed`] via
/// [`PyMotionBridge::on_credit_freed`] — but the upstream event-routing
/// path (an inbound serial reactor) is not yet hooked up to the bridge,
/// so in practice this seed bounds the in-flight credit budget for the
/// whole print. Sized generously so motion doesn't stall on credit before
/// the routing lands.
// MCU segment queue is `Q_N - 1 = 7` deep (see `rust/runtime/src/queue.rs::Q_N`).
// Seeding the host's local credit counter higher than that lets the host fire
// more segments than the MCU can buffer before the first `kalico_credit_freed`
// event arrives — overshoot rejects with `KALICO_ERR_QUEUE_FULL = -1` from
// `push_segment`. Seeding at the same value as the MCU's effective capacity
// keeps the first burst within budget; the very first `credit_freed` then
// reconciles to MCU-authoritative free_slots.
const CREDIT_SEED_CAPACITY: i32 = 7;

// ── Internal types ──────────────────────────────────────────────────────

/// MCU seed position queued by `set_position` and drained by the dispatch
/// closure before the next segment is sent.
///
/// Storing the seed here (rather than firing `runtime_seed_position`
/// immediately in `set_position`) guarantees that the seed arrives at the
/// MCU **after** all previously-dispatched segments (e.g. a retract during
/// homing) have been placed on the wire.  The dispatch closure processes
/// segments sequentially, so draining the pending seed at the head of each
/// dispatch invocation provides the required ordering without any extra
/// synchronisation.
struct SeedPosition {
    x: f64,
    y: f64,
    z: f64,
}

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
    /// Per-MCU runtime capabilities, queried via `QueryRuntimeCaps` after
    /// the kalico-native Identify handshake completes (Task 10). Falls back
    /// to the large-profile defaults if the firmware doesn't reply (older
    /// firmware predates the QueryRuntimeCaps message). Task 11 will move
    /// this onto `McuAxisConfig::caps`; for now the bootstrap stores it here.
    runtime_caps: Option<kalico_protocol::messages::RuntimeCapsResponse>,
    /// Raw `capabilities` bitmap from the `IdentifyResponse` (spec §5 bytes
    /// 61..69). Bit 0 = `PHASE_STEPPING_CAPABLE`. Set during `attach_serial`;
    /// 0 when `kalico_native_supported` is false (stock-Klipper MCU).
    identify_caps: u64,
    /// True when this MCU's kalico-native Identify handshake completed.
    /// False for stock-Klipper firmware that has no kalico runtime — those
    /// MCUs still attach for Klipper-protocol commands but cannot accept
    /// kalico-native bootstrap calls (configure_axes, curve uploads, etc.).
    kalico_native_supported: bool,
    /// Stop flag for the periodic `kalico_clock_sync_request` driver. Set
    /// to `true` on `release_mcu` (or PyMotionBridge drop) so the thread
    /// exits cleanly. `None` when no clock-sync thread is running (stock-
    /// Klipper firmware that doesn't support kalico-native).
    clock_sync_stop: Option<Arc<AtomicBool>>,
    /// Join handle for the clock-sync thread. Joined on `release_mcu`.
    clock_sync_thread: Option<JoinHandle<()>>,
}

/// Default fallback caps used when the MCU doesn't respond to
/// `QueryRuntimeCaps` (older firmware). Matches the large-profile pool sizing
/// the host previously assumed unconditionally.
const FALLBACK_RUNTIME_CAPS: kalico_protocol::messages::RuntimeCapsResponse =
    kalico_protocol::messages::RuntimeCapsResponse {
        curve_pool_n: 16,
        max_pieces_per_curve: 16,
    };

/// Sample interval for the periodic `kalico_clock_sync_request` driver.
/// The host's `compute_ack_clock` extrapolates linearly between samples,
/// so 500 ms is comfortably below the threshold at which clock-drift
/// (typically <100 ppm on H7) accumulates enough error to misschedule a
/// motion segment.
const CLOCK_SYNC_INTERVAL: Duration = Duration::from_millis(500);

/// Per-request timeout for the periodic clock-sync round-trip. USB-CDC
/// RTT is microseconds; 100 ms is generous enough to absorb a transient
/// stall without cascading into the wedge guard.
const CLOCK_SYNC_REQUEST_TIMEOUT: Duration = Duration::from_millis(100);

/// Spawn the bridge's per-MCU periodic clock-sync driver.
///
/// Why this exists: in bridge mode klippy's `clocksync._get_clock_event`
/// short-circuits — `serialhdl.raw_send` is a no-op for bridge MCUs, so
/// the MCU never sees the `get_clock` request, never responds, and
/// `_handle_clock` never runs. The `_bridge_clock_est_cb` registered at
/// connect therefore fires exactly once (on the post-connect refresh)
/// and the router's `(freq, offset, last_clock)` triple is frozen at
/// connect-time. `compute_ack_clock` then linearly extrapolates into
/// the future, producing `t_start` values tens of seconds ahead of the
/// MCU's actual clock — which deadlocks the host's in-flight credit
/// window because the engine waits on `t_start` and never retires.
///
/// This driver issues `runtime_clock_sync_request` directly via the
/// kalico-native transport (the path that ARMING already uses, spec §6.3),
/// maintains a per-MCU `ClockSyncEstimator` for RTT-aware regression,
/// and pushes each fresh estimate into the router via
/// `set_clock_est_from_sample`.
fn spawn_periodic_clock_sync(
    mcu_handle_raw: u32,
    host_io: Arc<KalicoHostIo>,
    router: Arc<Mutex<PassthroughRouter>>,
    clock_freqs: Arc<Mutex<HashMap<u32, f64>>>,
    stop: Arc<AtomicBool>,
) -> JoinHandle<()> {
    use kalico_host_rt::clock_sync::ClockSyncEstimator;
    use kalico_host_rt::transport::Transport;

    let mcu_h = mcu_handle_from_raw(mcu_handle_raw);
    std::thread::Builder::new()
        .name(format!("clock-sync-mcu-{mcu_handle_raw}"))
        .spawn(move || {
            // Initial freq seed: poll `clock_freqs` (populated by klippy's
            // first `set_clock_est`). If klippy hasn't supplied one yet,
            // fall back to 100 MHz — the regression converges within a
            // few samples regardless of the seed; only the very first
            // RTT half-correction depends on it.
            let initial_freq = {
                let guard = clock_freqs.lock().unwrap_or_else(|p| p.into_inner());
                guard.get(&mcu_handle_raw).copied().unwrap_or(100_000_000.0)
            };
            let mut estimator = ClockSyncEstimator::new(initial_freq);

            // Brief startup grace so the reactor's identify/caps round-trips
            // settle before we add foreground call traffic.
            std::thread::sleep(Duration::from_millis(200));

            while !stop.load(Ordering::Relaxed) {
                let request_id = estimator.next_clock_sync_request_id();
                let host_send = Instant::now();
                let cmd = format!(
                    "runtime_clock_sync_request request_id={request_id} \
                     host_send_time_lo=0 host_send_time_hi=0"
                );
                if let Ok(resp) = host_io.call(
                    &cmd,
                    "kalico_clock_sync_response",
                    CLOCK_SYNC_REQUEST_TIMEOUT,
                ) {
                    let host_recv = Instant::now();
                    if let Some(echoed) = resp.try_get_u32("request_id") {
                        if echoed == request_id {
                            let lo = resp.try_get_u32("mcu_clock_lo").unwrap_or(0);
                            let hi = resp.try_get_u32("mcu_clock_hi").unwrap_or(0);
                            let mcu_at_response =
                                (u64::from(hi) << 32) | u64::from(lo);
                            // GUARD: firmware returns `read_widened_now(shared)`
                            // for `kalico_clock_sync_response.mcu_clock_*`.
                            // Before the very first segment-push fires
                            // `runtime_tick_enable`, TIM5 ISR hasn't ticked
                            // and `widened_now=0`. Feeding that 0 sample
                            // into the regression collapses slope→0 →
                            // `set_clock_est_from_sample` overwrites
                            // klippy's valid 520M clock_freq with 0,
                            // making `compute_ack_clock` return 0 and
                            // dispatch wedge waiting for clock-sync.
                            //
                            // Skip the sample if MCU clock looks
                            // uninitialised (well below one wrap of the
                            // wall-clock-equivalent — any printer that
                            // boots in <1s is fictional). The regression
                            // is forward-only; skipping samples is safe.
                            const MCU_CLOCK_INIT_FLOOR: u64 = 100_000_000;
                            if mcu_at_response < MCU_CLOCK_INIT_FLOOR {
                                use std::sync::atomic::{AtomicUsize, Ordering as AOrd};
                                static SKIP_COUNT: AtomicUsize = AtomicUsize::new(0);
                                let n = SKIP_COUNT.fetch_add(1, AOrd::Relaxed);
                                if n < 3 || n % 100 == 0 {
                                    log::debug!(
                                        "[bridge-trace] clock-sync skipping uninit MCU sample #{} mcu_at_response={} (TIM5 likely not yet ticking — pre-first-push)",
                                        n, mcu_at_response,
                                    );
                                }
                            } else {
                                estimator.add_dedicated_sample(
                                    host_send,
                                    host_recv,
                                    mcu_at_response,
                                );
                                let rtt = host_recv.saturating_duration_since(host_send);
                                #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
                                let one_way_cycles = (rtt.as_secs_f64()
                                    * estimator.clock_freq_estimate
                                    / 2.0) as u64;
                                let mcu_at_send =
                                    mcu_at_response.saturating_sub(one_way_cycles);
                                let mut r = router.lock().unwrap_or_else(|p| p.into_inner());
                                let _ = r.set_clock_est_from_sample(
                                    mcu_h,
                                    estimator.clock_freq_estimate,
                                    host_send,
                                    mcu_at_send,
                                );
                            }
                        }
                    }
                }
                // Sleep with poll-on-stop so shutdown is responsive even
                // mid-interval (release_mcu join would otherwise wait up
                // to CLOCK_SYNC_INTERVAL for the loop to come around).
                let mut remaining = CLOCK_SYNC_INTERVAL;
                while remaining > Duration::ZERO && !stop.load(Ordering::Relaxed) {
                    let chunk = remaining.min(Duration::from_millis(50));
                    std::thread::sleep(chunk);
                    remaining = remaining.saturating_sub(chunk);
                }
            }
        })
        .expect("clock-sync thread spawn")
}

/// Errors returned by `query_runtime_caps` / `decode_runtime_caps_body`.
/// The bootstrap path discriminates only via Display today (logged + falls
/// back), but the typed variants make future routing (e.g. distinguishing
/// "old firmware lacks the message" from "transport hiccup") possible
/// without restructuring callers.
#[derive(Debug, thiserror::Error)]
enum RuntimeCapsError {
    #[error("kalico_call QueryRuntimeCaps: {0}")]
    Call(String),
    #[error("QueryRuntimeCaps: unexpected response kind {got:?}")]
    UnexpectedKind { got: kalico_protocol::MessageKind },
    #[error("decode RuntimeCapsResponse: {0}")]
    Decode(String),
}

/// Decode a `RuntimeCapsResponse` from a raw control-channel response body.
/// Extracted so the bootstrap path can be unit-tested without spinning a
/// reactor + serial port (the actual `kalico_call` round-trip is exercised
/// in higher-level integration tests against Renode / hardware).
fn decode_runtime_caps_body(
    body: &[u8],
) -> Result<kalico_protocol::messages::RuntimeCapsResponse, RuntimeCapsError> {
    use kalico_protocol::codec::{Cursor, Decode};
    use kalico_protocol::messages::RuntimeCapsResponse;
    let mut c = Cursor::new(body);
    RuntimeCapsResponse::decode_from(&mut c).map_err(|e| RuntimeCapsError::Decode(format!("{e:?}")))
}

/// Issue a `QueryRuntimeCaps` control-channel call and decode the response
/// body. On any transport / decode error returns `Err` — the bootstrap path
/// logs a warning and falls back to [`FALLBACK_RUNTIME_CAPS`] so older
/// firmware (predating QueryRuntimeCaps) still attaches.
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
) -> Result<ShaperConfig, crate::config::ShaperConfigError> {
    Ok(ShaperConfig {
        x: parse_required_shaper(type_x, freq_x)?,
        y: parse_required_shaper(type_y, freq_y)?,
        z: AxisShaper::Passthrough,
    })
}

/// Push a segment via the kalico-native transport (spec §7.4 / Phase C-B).
/// This was a fire-and-forget Klipper-protocol command pre-Phase-C; the new
/// transport delivers a synchronous `PushSegmentResponse` so the dispatch
/// loop confirms acceptance before moving on. Per-segment latency is one
/// kalico round-trip — measured in microseconds on USB-CDC.
fn dispatch_push_segment(
    io: &KalicoHostIo,
    credit: &CreditCounter,
    params: &producer::SegmentPushParams,
) -> Result<producer::PushedSegmentInfo, producer::ProducerError> {
    producer::push_segment_with_timeout(io, credit, params, producer::DEFAULT_PUSH_RESPONSE_TIMEOUT)
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
    /// Spawned planner thread. `init_planner` sets it exactly once; every
    /// subsequent motion-submission entry point reads it lock-free via
    /// `OnceLock::get`. The previous `Mutex<Option<PlannerHandle>>` form
    /// took an uncontended mutex on every `submit_move` / `flush` /
    /// `wait_moves` call.
    planner: OnceLock<PlannerHandle>,
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
    /// Pending MCU seed position stored by `set_position` and drained by the
    /// dispatch closure before each segment is sent.  `None` when no seed is
    /// outstanding (normal steady-state path).
    pending_seed: Arc<Mutex<Option<SeedPosition>>>,
}

/// Build the kalico-native `ConfigureAxes` wire body.
///
/// Body layouts (length-discriminated; the firmware parser branches on
/// `blob_len`):
///   - 20 bytes when `step_modes` and `phase_configs` are both None
///     (legacy path; kinematics + 3 masks + 4 × f32 steps_per_mm).
///   - 25 bytes when `step_modes` is Some, `phase_configs` is None
///     (Step 7-B: adds phase_capable flag + 4-byte step_mode array).
///   - 26 + 3·N bytes when both `step_modes` and `phase_configs` are Some
///     (true phase stepping): byte 25 is `phase_motor_count = N`,
///     bytes 26 + 3·i .. 26 + 3·i + 2 carry `(bus_id, cs_pin_id, slot_idx)`
///     for motor `i`. `1 ≤ N ≤ MAX_STEPPER_OIDS` (firmware-side cap of 16
///     phase-stepped motors per MCU).
///
/// `phase_capable` is the identify-time PHASE_STEPPING bit (bit 0 of
/// `identify_caps`). It is purely an MCU-side sanity check; the wire
/// position is fixed at byte 20 for the 25-byte and ≥26-byte layouts.
///
/// "No phase stepping" emits the 25-byte body — callers should pass
/// `phase_configs = None` in that case rather than `Some(&[])`.
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
    // Worst-case body length: 26 (header + step_modes + count byte) +
    // 3 × MAX (16) = 74 bytes. Pre-size to that ceiling.
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
        // Promoted from debug_assert! to assert! so release builds also
        // enforce this invariant. The PyO3 wrapper checks at the boundary,
        // but this helper is pub(crate) and could be called from other
        // in-crate sites; a malformed phase-config body without
        // step_modes must never silently leave this function.
        assert!(
            step_modes.is_some(),
            "phase_configs requires step_modes (variable-length format extends 25-byte)"
        );
        // Cap mirrors firmware-side MAX_STEPPER_OIDS=16 (see
        // `runtime::state::MAX_STEPPER_OIDS`).
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
            planner: OnceLock::new(),
            planner_config: Mutex::new(PlannerConfig::default()),
            commanded_pos: Mutex::new([0.0; 3]),
            mcu_axis_configs: Mutex::new(Vec::new()),
            dispatched_segments: Arc::new(AtomicU64::new(0)),
            slot_pools: Arc::new(Mutex::new(HashMap::new())),
            credit_counters: Arc::new(Mutex::new(HashMap::new())),
            fallback_clock_conversions: Arc::new(AtomicU64::new(0)),
            clock_freqs: Arc::new(Mutex::new(HashMap::new())),
            homing: Arc::new(HomingState::new()),
            pending_seed: Arc::new(Mutex::new(None)),
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
                clock_sync_stop: None,
                clock_sync_thread: None,
            },
        );
        Ok(raw)
    }

    // ── Task 33: release_mcu ────────────────────────────────────────────

    /// Unregister an MCU. Outstanding notify callbacks are dropped.
    fn release_mcu(&self, handle: u32) -> PyResult<()> {
        // Stop and join the per-MCU clock-sync thread before releasing
        // the router slot. Holds neither lock during the join so the
        // thread can't deadlock on its final router update.
        let (stop, join) = {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn_opt = mcus.remove(&handle);
            match conn_opt {
                Some(mut c) => (c.clock_sync_stop.take(), c.clock_sync_thread.take()),
                None => (None, None),
            }
        };
        if let Some(stop) = stop {
            stop.store(true, Ordering::Release);
        }
        if let Some(join) = join {
            let _ = join.join();
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
    }

    // ── Task 34: alloc_command_queue ─────────────────────────────────────

    /// Allocate a command queue for the given MCU. Returns queue id as int.
    fn alloc_command_queue(&self, handle: u32) -> PyResult<u32> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
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
        let entry = PassthroughEntry::new(data.to_vec(), min_clock, req_clock, NotifyId::none());
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
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
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
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
    fn passthrough_register_flush_callback(&self, mcu: u32, callback: PyObject) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
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
        let mut events = self.events.lock().unwrap_or_else(|p| p.into_inner());
        match events.pop_front() {
            Some(ev) => Ok(Some(ev.to_pydict(py)?)),
            None => Ok(None),
        }
    }

    // ── Additional klippy-expected API ──────────────────────────────────

    /// Add a config command for the given MCU.
    fn add_config_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_config_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    /// Add an init command for the given MCU.
    fn add_init_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_init_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    /// Add a restart command for the given MCU.
    fn add_restart_cmd(&self, mcu: u32, data: &[u8]) -> PyResult<bool> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .add_restart_cmd(mcu_handle_from_raw(mcu), data.to_vec())
            .map_err(router_err)
    }

    /// Transition the MCU to the config-sending phase.
    fn begin_config_phase(&self, mcu: u32) -> PyResult<()> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .begin_config_phase(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }

    /// Get the next config/init entry for the given MCU, or None.
    fn next_config_entry(&self, mcu: u32) -> PyResult<Option<Vec<u8>>> {
        let mut router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        router
            .next_config_entry(mcu_handle_from_raw(mcu))
            .map_err(router_err)
    }

    /// Snapshot statistics for the given MCU as a Python dict.
    fn get_stats(&self, py: Python<'_>, mcu: u32) -> PyResult<Py<PyDict>> {
        let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
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
        *self.parser.lock().unwrap_or_else(|p| p.into_inner()) = Some(Arc::new(parser));
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

        // ── Reuse path ────────────────────────────────────────────────────────
        // If an existing KalicoHostIo is alive (reactor thread still running),
        // reuse it — skip the drop and reopen entirely. This matches mainline
        // Klipper's behaviour: the serial port stays open through shutdown →
        // FIRMWARE_RESTART cycles. Dropping an alive connection can wedge
        // because the Drop's reactor-thread join blocks on a blocking serial
        // read, and the subsequent reopen gets EBUSY from the kernel's
        // cdc_acm single-open semantics.
        //
        // We do still re-subscribe runtime events (the old channel's buffer
        // is stale after a firmware restart) and re-run the kalico-native
        // identify + caps handshake so the host reflects the new firmware
        // epoch. The clock-sync thread is left running — it holds only a
        // Weak<KalicoHostIo> and will keep ticking without interruption.
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

                    // Re-subscribe so the new runtime-event channel starts
                    // fresh (the firmware restart will have pushed new events).
                    let runtime_rx =
                        io.take_runtime_event_subscription().map_err(|e| {
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

                    let runtime_caps =
                        match query_runtime_caps(&io, std::time::Duration::from_secs(2)) {
                            Ok(caps) => {
                                log::debug!(
                                    "[caps-trace] attach_serial reuse: runtime caps \
                                     for {serial_path}: pool_n={} max_pieces_per_curve={}",
                                    caps.curve_pool_n,
                                    caps.max_pieces_per_curve,
                                );
                                caps
                            }
                            Err(e) => {
                                log::debug!(
                                    "[caps-trace] attach_serial reuse: QueryRuntimeCaps \
                                     failed for {serial_path} ({e}); using large-profile defaults"
                                );
                                FALLBACK_RUNTIME_CAPS
                            }
                        };

                    let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
                    let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
                        PyRuntimeError::new_err(format!(
                            "attach_serial: unknown mcu_handle {mcu_handle}"
                        ))
                    })?;
                    conn.runtime_rx = Some(runtime_rx);
                    conn.runtime_caps = Some(runtime_caps);
                    conn.identify_caps = identify_caps;
                    conn.kalico_native_supported = kalico_native_supported;
                    // clock_sync_stop / clock_sync_thread left intact — the
                    // thread is already running and does not need a restart.
                    return Ok(());
                }
            }
        }

        // ── Fresh open path ───────────────────────────────────────────────────
        // The existing connection is absent or dead. Drop it (if present) to
        // release the kernel FD before reopening.
        //
        // 2026-05-18: drop any existing KalicoHostIo for this mcu_handle
        // BEFORE trying to open the new serial. The Drop impl sends
        // `ReactorCommand::Shutdown` and joins the reactor thread, which
        // is what actually releases the kernel FD. Without this the OLD
        // session's reactor keeps the serial open exclusively and the new
        // `open_with_config` below times out for 30 s with "Device or
        // resource busy" — exactly the wedge klippy's in-process
        // FIRMWARE_RESTART iteration falls into on the F4 (and on the H7
        // when the bridge-mode reset path didn't get a chance to issue
        // `MarkExpectedDisconnect`). Also stop the periodic clock-sync
        // driver so it doesn't keep using the dying io.
        {
            let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            if let Some(conn) = mcus.get_mut(&mcu_handle) {
                if let Some(stop) = conn.clock_sync_stop.take() {
                    stop.store(true, std::sync::atomic::Ordering::Release);
                }
                if let Some(h) = conn.clock_sync_thread.take() {
                    let _ = h.join();
                }
                conn.runtime_rx = None;
                // Drop the Arc<KalicoHostIo>. The dispatch closure in the
                // planner thread holds only Weak<KalicoHostIo> references
                // (downgraded at dispatch_ios insertion in init_planner), so
                // dropping this Arc here drives the refcount to zero, causing
                // the Drop impl to send Shutdown to the reactor and join it —
                // which releases the kernel FD before the re-open loop below
                // runs. No 30-second "Device or resource busy" spin.
                conn.host_io = None;
            }
        }

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

        // Subscribe to runtime events before storing so no events are missed.
        let runtime_rx = host_io.take_runtime_event_subscription().map_err(|e| {
            PyRuntimeError::new_err(format!("attach_serial: runtime_event subscribe: {e:?}"))
        })?;

        // Phase C-B: kalico-native bootstrap-ABI Identify handshake. Stock
        // Klipper firmware (no CONFIG_KALICO_RUNTIME) does not have the
        // kalico-native dispatch path, so this query times out. We treat
        // that case as "no kalico runtime here" rather than refusing the
        // attach — the bridge still routes Klipper-protocol commands fine
        // and the runtime-specific surface (curve uploads, etc.) just
        // stays unused for that MCU.
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

        // Task 10: query per-MCU runtime caps. Older firmware predates this
        // message — on any error fall back to the large-profile defaults so
        // attach still succeeds. Task 11 will route this onto
        // `McuAxisConfig::caps` for sizing decisions; for now we just stash
        // it on the per-MCU connection.
        let runtime_caps = match query_runtime_caps(&host_io, std::time::Duration::from_secs(2)) {
            Ok(caps) => {
                log::debug!(
                    "[caps-trace] attach_serial: runtime caps for {serial_path}: \
                     pool_n={} max_pieces_per_curve={}",
                    caps.curve_pool_n,
                    caps.max_pieces_per_curve,
                );
                caps
            }
            Err(e) => {
                log::debug!(
                    "[caps-trace] attach_serial: QueryRuntimeCaps failed for {serial_path} ({e}); \
                     falling back to large-profile defaults",
                );
                FALLBACK_RUNTIME_CAPS
            }
        };

        let host_io_arc = Arc::new(host_io);

        // Spawn the periodic `kalico_clock_sync_request` driver iff this
        // MCU speaks kalico-native. Without this, klippy's bridge-mode
        // clocksync `_get_clock_event` is a no-op (the legacy serialqueue
        // path is bypassed) — the regression freezes at the connect-time
        // anchor and `compute_ack_clock` linearly drifts into the future,
        // producing motion segments scheduled tens of seconds ahead of
        // MCU time and deadlocking the host's in-flight credit window.
        // Stock-Klipper firmware doesn't accept `kalico_clock_sync_request`,
        // so for those MCUs we leave clock projection on the (admittedly
        // stale) klippy-side anchor — the motion path doesn't execute on
        // them anyway.
        let (clock_sync_stop, clock_sync_thread) = if kalico_native_supported {
            let stop = Arc::new(AtomicBool::new(false));
            let handle = spawn_periodic_clock_sync(
                mcu_handle,
                Arc::clone(&host_io_arc),
                Arc::clone(&self.router),
                Arc::clone(&self.clock_freqs),
                Arc::clone(&stop),
            );
            (Some(stop), Some(handle))
        } else {
            (None, None)
        };

        let mut mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get_mut(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!("attach_serial: unknown mcu_handle {mcu_handle}"))
        })?;
        conn.host_io = Some(host_io_arc);
        conn.runtime_rx = Some(runtime_rx);
        conn.runtime_caps = Some(runtime_caps);
        conn.identify_caps = identify_caps;
        conn.kalico_native_supported = kalico_native_supported;
        conn.clock_sync_stop = clock_sync_stop;
        conn.clock_sync_thread = clock_sync_thread;
        Ok(())
    }

    /// Return the `capabilities` bitmap from the MCU's `IdentifyResponse`
    /// (spec §5, bytes 61..69). Bit 0 = `PHASE_STEPPING_CAPABLE`.
    ///
    /// Returns 0 for stock-Klipper MCUs that don't speak kalico-native.
    /// `claim_mcu` must have been called first; `attach_serial` must have
    /// completed for the value to reflect the real MCU capabilities.
    fn get_mcu_capabilities(&self, mcu_handle: u32) -> PyResult<u64> {
        let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
        let conn = mcus.get(&mcu_handle).ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "get_mcu_capabilities: unknown mcu_handle {mcu_handle}"
            ))
        })?;
        Ok(conn.identify_caps)
    }

    /// Send the kalico-native `ConfigureAxes` message for an attached MCU.
    /// Must be called once after `attach_serial` and before the first
    /// segment is pushed. `kinematics`: 0 = CoreXyAndE, 1 = CartesianXyzAndE.
    /// `steps_per_mm`: 4 entries indexed [A/X, B/Y, Z, E]; entries whose
    /// `present_mask` bit is 0 are ignored. `awd_mask` and `invert_mask`
    /// are 4-bit per-motor flag masks.
    ///
    /// `step_modes`: optional list of 4 `u8` values (0 = Modulated / phase
    /// stepping, 1 = StepTime / classic). When supplied the bridge emits the
    /// 25-byte extended format (spec §4 C1); when omitted it emits the
    /// legacy 20-byte format. Firmware accepts both.
    ///
    /// `phase_configs`: optional variable-length list of
    /// `(bus_id, cs_pin_id, slot_idx)` triples — one entry per
    /// phase-stepped motor. When supplied (and `step_modes` is also Some),
    /// the bridge emits the variable-length format (byte 25 =
    /// phase_motor_count, bytes 26+3·i = per-motor entry). Up to 16 motors
    /// per MCU (mirrors firmware-side `MAX_STEPPER_OIDS`). `slot_idx` must
    /// be in 0..4 (kinematic-slot index) and `step_modes[slot_idx]` must
    /// be 0 (Modulated). Pass `None` (not an empty list) when no motors
    /// are phase stepped — the bridge then emits the 25-byte body.
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
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new()
            .append(true)
            .create(true)
            .open("/tmp/cax-trace.log")
        {
            let _ = writeln!(
                f,
                "configure_axes ENTRY mcu_handle={mcu_handle} kin={kinematics} present=0x{present_mask:x} awd=0x{awd_mask:x} invert=0x{invert_mask:x} steps_per_mm_len={} step_modes={step_modes:?}",
                steps_per_mm.len()
            );
        }
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
            // Cap mirrors firmware-side runtime::state::MAX_STEPPER_OIDS.
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
        log::debug!(
            "[trace-bridge-cax] enter mcu_handle={mcu_handle} kin={kinematics} present=0x{present_mask:x} awd=0x{awd_mask:x} invert=0x{invert_mask:x} step_modes={step_modes:?}"
        );
        // belt-and-suspenders: also force stderr flush
        let _ = std::io::stderr().flush();
        let (io, identify_caps) = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let conn = mcus.get(&mcu_handle).ok_or_else(|| {
                PyRuntimeError::new_err(format!("configure_axes: unknown mcu_handle {mcu_handle}"))
            })?;
            log::debug!(
                "[trace-bridge-cax] conn found mcu_handle={mcu_handle} kalico_supported={} host_io_some={}",
                conn.kalico_native_supported,
                conn.host_io.is_some()
            );
            // Stock-Klipper firmware (no kalico runtime) cannot accept this
            // bootstrap message. Silently no-op so multi-MCU setups where one
            // board runs stock Klipper still complete _configure_axes_per_mcu
            // for the kalico-runtime board(s).
            if !conn.kalico_native_supported {
                log::debug!("[trace-bridge-cax] kalico_native_supported=false -> early Ok(())");
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
        // Always emit the 25-byte extended format when step_modes are provided;
        // fall back to 20-byte legacy when not. Byte 20 carries the phase-
        // stepping capability bit from the identify response so the firmware
        // can double-check the host's understanding. Bytes 21-24 are the per-
        // motor StepMode array (0=Modulated, 1=StepTime).
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

    /// Register a phase-stepping SPI bus (cfg only) with the MCU. Call
    /// once per unique `bus_id` BEFORE any `register_phase_motor` calls
    /// referencing that bus, and before `configure_axes` for that MCU.
    /// Wraps the `runtime_register_phase_bus bus_id=%c rate=%u` wire
    /// command. The firmware-side handler calls
    /// `spi_setup(bus_id, mode=3, rate)` and caches the cfg.
    ///
    /// Per-motor CS GPIOs are registered separately via
    /// `register_phase_motor` — multiple TMC5160 drivers on the same SPI
    /// bus each need their own CS line, so CS state is per-motor, not
    /// per-bus (2026-05-19 fix; see
    /// `docs/superpowers/specs/2026-05-19-phase-stepping-per-motor-cs-design.md`).
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
            // Stock-Klipper firmware (no kalico runtime) does not implement
            // runtime_register_phase_bus. Silently no-op so multi-MCU setups
            // where one board is stock Klipper (e.g. F446 on Z) still complete
            // the per-MCU iteration cleanly.
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
        // Firmware emits `result=%i` (signed i32). `try_get_i32` accepts
        // either I32 or U32 (parser may surface a non-negative %i value as
        // U32 when it fits in u31) and returns None on missing/wrong-type,
        // so a firmware-side schema drift surfaces as an explicit error
        // instead of being silently coerced to 0.
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

    /// Register the CS GPIO for a single phase-stepped motor. Call once
    /// per phase-stepped motor, after `register_phase_bus` for the named
    /// `bus_id` and before `configure_axes` for that MCU. Wraps the
    /// `runtime_register_phase_motor motor_idx=%c bus_id=%c cs_pin_id=%c`
    /// wire command. The firmware-side handler calls
    /// `gpio_out_setup(cs_pin_id, 1 /* idle high */)` and stores the
    /// handle in `phase_motors[motor_idx]` for `write_xdirect` dispatch.
    ///
    /// `motor_idx` is the Rust runtime motor slot index in
    /// `[0, MAX_STEPPER_OIDS=16)`, matching the per-motor
    /// `shared.phase_config[motor_idx]` storage. `cs_pin_id` is the
    /// firmware's GPIO encoding (port * 16 + pin on stm32).
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

    /// Return the raw identify bytes (zlib-compressed firmware data-dict)
    /// for the given MCU. `attach_serial` must have been called first.
    ///
    /// Pass the returned bytes to klippy's
    /// `msgproto.MessageParser.process_identify(data)`.
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
    ///   - `"endstop_tripped"`: kalico_endstop_tripped — keys: `arm_id`,
    ///     `trip_clock`, `trip_source_idx`, `fmt_version`, `stepper_count`
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
                // v2: retirement watermark — host EventDispatcher already
                // synthesizes a CreditFreed from this on watermark advance
                // (events.rs::handle_status_frame), but expose it to klippy
                // too for observability.
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
                // Trace events are not klippy-visible; skip silently.
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
                // Spread params fields directly into the dict so klippy's
                // registered handlers receive them with their original names
                // (e.g. analog_in_state's `oid`, `value`, `next_clock`,
                // `value_avg`). Serial-protocol field names never collide
                // with the keys we set above.
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

    /// 2026-05-18: tell the per-MCU reactor that an imminent transport drop
    /// is expected and must NOT trigger the EXIT_ON_FAULT abort guard.
    /// Klippy calls this from `_restart_via_command` right before sending
    /// the firmware `reset` command — `NVIC_SystemReset` on the MCU drops
    /// USB-CDC and the host reactor would otherwise interpret BrokenPipe as
    /// a wedge and abort the whole klippy process, breaking
    /// FIRMWARE_RESTART recovery on bridge MCUs.
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

    /// Update clock estimation parameters for the given MCU.
    #[pyo3(signature = (mcu, freq, offset, last_clock))]
    fn set_clock_est(
        &self,
        py: Python<'_>,
        mcu: u32,
        freq: f64,
        offset: f64,
        last_clock: u64,
    ) -> PyResult<()> {
        let host_now_same_epoch: f64 = py
            .import("time")?
            .getattr("monotonic")?
            .call0()?
            .extract()?;
        // Diag: log every set_clock_est arrival. Trying to isolate why the
        // dispatch sees `now_clock=0` despite klippy's clocksync showing
        // last_clock in the billions.
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
                host_now_same_epoch,
            )
            .map_err(router_err)?;
        self.clock_freqs
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, freq);
        Ok(())
    }

    /// Drain the debug log for crash diagnostics. Returns a dict with
    /// `sent` and `received` lists of dicts.
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

        // Persist for runtime updates.
        *self
            .planner_config
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = cfg.clone();

        // Two-MCU first-print MVP topology. Pull `runtime_caps` from each
        // `McuConnection` (set during bootstrap by `query_runtime_caps`); fall
        // back to large-profile defaults if the firmware predates
        // `QueryRuntimeCaps`.
        let (octopus_caps, f446_caps) = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            let oc = mcus
                .get(&octopus_handle)
                .and_then(|c| c.runtime_caps)
                .map(McuCaps::from)
                .unwrap_or_default();
            let fc = mcus
                .get(&f446_handle)
                .and_then(|c| c.runtime_caps)
                .map(McuCaps::from)
                .unwrap_or_default();
            (oc, fc)
        };
        let mcu_configs = vec![
            McuAxisConfig {
                mcu_id: octopus_handle,
                axes: vec![AXIS_X, AXIS_Y],
                kinematics: 0, // CoreXyAndE
                caps: octopus_caps,
            },
            McuAxisConfig {
                mcu_id: f446_handle,
                axes: vec![AXIS_Z],
                kinematics: 1, // CartesianXyzAndE
                caps: f446_caps,
            },
        ];
        *self
            .mcu_axis_configs
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = mcu_configs.clone();

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
        let warned_mcus: Arc<Mutex<HashSet<u32>>> = Arc::new(Mutex::new(HashSet::new()));
        let router_arc = Arc::clone(&self.router);
        let pending_seed_for_cb = Arc::clone(&self.pending_seed);

        let host_ios: HashMap<u32, Arc<KalicoHostIo>> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
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

        // Snapshot of which MCUs accept kalico-native producer messages
        // (load_curve / push_segment). MCUs whose firmware is stock Klipper
        // (no kalico runtime) attach for Klipper-protocol traffic but the
        // bridge can't send them planner curves. The dispatch closure below
        // skips plans targeting such MCUs.
        let kalico_native_for_plans: HashMap<u32, bool> = {
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcu_configs
                .iter()
                .map(|cfg| {
                    let supported = mcus
                        .get(&cfg.mcu_id)
                        .map(|c| c.kalico_native_supported)
                        .unwrap_or(false);
                    (cfg.mcu_id, supported)
                })
                .collect()
        };

        // Per-MCU dispatch context (host I/O + credit + slot pool) keyed by
        // mcu_id. `dispatch_ios` is the closure-local lookup map; the credit
        // and slot-pool tables on `self` are the persistent ones the
        // event-routing API (`on_credit_freed`) drives.
        //
        // The KalicoHostIo entry is stored as Weak<KalicoHostIo> so that
        // `attach_serial`'s `conn.host_io = None` (which drops the only
        // strong Arc) immediately drives the refcount to zero — triggering
        // the Drop impl that shuts down the reactor and releases the serial
        // FD — without waiting for the planner thread to be torn down.
        // The closure upgrades to Arc on each dispatch; if the upgrade
        // returns None the connection has been dropped and the dispatch
        // returns ConnectionDropped.
        let mut dispatch_ios: HashMap<
            u32,
            (Weak<KalicoHostIo>, Arc<CreditCounter>, Arc<Mutex<SlotPool>>),
        > = HashMap::new();
        let mut self_credits = self
            .credit_counters
            .lock()
            .unwrap_or_else(|p| p.into_inner());
        let mut self_pools = self.slot_pools.lock().unwrap_or_else(|p| p.into_inner());
        self_credits.clear();
        self_pools.clear();
        for cfg_mcu in &mcu_configs {
            let io = host_ios
                .get(&cfg_mcu.mcu_id)
                .expect("host_io map built from mcu_configs")
                .clone();
            let credit = Arc::new(CreditCounter::new(CREDIT_SEED_CAPACITY));
            io.attach_credit_counter(Arc::clone(&credit));

            // On klippy reconnect without MCU power-cycle the MCU's
            // `CurvePool` still carries generation counters from the prior
            // session. Slots with `current_gen != last_retired_gen` (loaded
            // but never retired because klippy died) will reject every
            // `LoadCurveCubic` with "slot busy". Send `ResetCurvePool`
            // (0x0050) to set `last_retired_gen = current_gen` for every
            // slot before constructing the fresh `SlotPool`.
            //
            // Skip for non-kalico-native MCUs (stock Klipper firmware does
            // not implement the kalico binary protocol).
            if *kalico_native_for_plans
                .get(&cfg_mcu.mcu_id)
                .unwrap_or(&false)
            {
                match producer::reset_curve_pool(&io, producer::DEFAULT_RESET_CURVE_POOL_TIMEOUT) {
                    Ok(()) => {
                        log::debug!("[init-planner] reset_curve_pool mcu={} ok", cfg_mcu.mcu_id)
                    }
                    Err(e) => {
                        // Non-fatal: log and continue. If the MCU was just
                        // power-cycled its pool is already clean (all gens
                        // at 0). Failing here only hurts on a reconnect
                        // where the MCU genuinely has stale state — in that
                        // case the first `load_curve` will surface the
                        // "slot busy" error at dispatch time, same as before
                        // this fix, which is preferable to aborting planner
                        // init entirely.
                        log::warn!(
                            "[init-planner] reset_curve_pool mcu={} failed: {e:?}; \
                             continuing — first load_curve may fail with slot busy \
                             if MCU was not power-cycled",
                            cfg_mcu.mcu_id
                        );
                    }
                }
            }

            // Bench 2026-05-12: size the host slot pool to the MCU's
            // actual `caps.curve_pool_n` (queried at attach_serial). The
            // F446 reports `pool_n=4` under the small profile (per-slot
            // scratch + 128 KB total RAM forces this); the H7 reports 16.
            // Hardcoding 16 here previously caused the 5th sequential jog
            // to allocate slot=4 → F446 rejects with
            // `KALICO_ERR_INVALID_HANDLE`/OutOfBounds.
            let pool_capacity = cfg_mcu.caps.curve_pool_n as usize;
            log::debug!(
                "[slot-trace] init_pool mcu={} capacity={}",
                cfg_mcu.mcu_id,
                pool_capacity,
            );
            let slot_pool = Arc::new(Mutex::new(SlotPool::new(pool_capacity)));
            self_credits.insert(cfg_mcu.mcu_id, Arc::clone(&credit));
            self_pools.insert(cfg_mcu.mcu_id, Arc::clone(&slot_pool));
            // Downgrade to Weak so the dispatch closure does not prevent
            // KalicoHostIo::drop from running when attach_serial clears
            // conn.host_io. The Arc is still live on `self.mcus` until then.
            dispatch_ios.insert(cfg_mcu.mcu_id, (Arc::downgrade(&io), credit, slot_pool));
        }
        drop(self_credits);
        drop(self_pools);

        let mcu_configs_for_cb = mcu_configs;
        let router_for_cb = Arc::clone(&router_arc);

        // Per-MCU rolling segment id. Allocated alongside the slot to
        // bind the `kalico_credit_freed.retired_through_segment_id`
        // retirement signal to the segment's curve slots.
        let next_seg_id: Arc<Mutex<HashMap<u32, u32>>> = Arc::new(Mutex::new(HashMap::new()));
        // Per-MCU schedule state:
        //   (current batch base clock, next available absolute clock).
        // `trajectory::shape_batch` emits batch-local times, with each new
        // batch starting at t=0. Dispatch places those relative seconds onto
        // the MCU's live clock with a small lead so the firmware does not see
        // zero-duration or already-expired segments.
        let schedule_state: Arc<Mutex<HashMap<u32, (u64, u64)>>> =
            Arc::new(Mutex::new(HashMap::new()));

        let dispatch: Arc<
            dyn Fn(&trajectory::ShapedSegment) -> Result<(), DispatchError> + Send + Sync,
        > = Arc::new(
            move |seg: &trajectory::ShapedSegment| -> Result<(), DispatchError> {
                eprintln!(
                    "[move-diag] dispatch closure entered: seg.t_start={} seg.t_end={}",
                    seg.t_start, seg.t_end,
                );

                // ── Pending seed drain ─────────────────────────────────────────
                //
                // `set_position` stores its `runtime_seed_position` payload here
                // rather than sending it immediately, because in-flight segments
                // from a previous move (e.g. a retract during homing) may not
                // have reached the MCU yet.  Sending the seed here — at the head
                // of the next dispatch invocation — guarantees ordering: the seed
                // arrives after all previous segments and before the segment we
                // are about to dispatch.
                if let Some(seed) = pending_seed_for_cb
                    .lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .take()
                {
                    let encode_q16 = |mm: f64| -> i32 {
                        let raw = mm * 65536.0;
                        raw.round().clamp(i32::MIN as f64, i32::MAX as f64) as i32
                    };
                    let x_q16 = encode_q16(seed.x);
                    let y_q16 = encode_q16(seed.y);
                    let z_q16 = encode_q16(seed.z);

                    for cfg in &mcu_configs_for_cb {
                        let (io_weak, _credit, _pool) = match dispatch_ios.get(&cfg.mcu_id) {
                            Some(v) => v,
                            None => continue,
                        };
                        let io = match io_weak.upgrade() {
                            Some(io) => io,
                            None => continue, // connection dropped, skip this MCU
                        };

                        // Motor-frame transform: CoreXY MCUs expect A = X+Y,
                        // B = X-Y; CartesianXyzAndE MCUs receive logical X/Y/Z.
                        let (seed_x_q16, seed_y_q16) =
                            if cfg.kinematics == crate::dispatch::KINEMATICS_COREXY {
                                (encode_q16(seed.x + seed.y), encode_q16(seed.x - seed.y))
                            } else {
                                (x_q16, y_q16)
                            };

                        use kalico_host_rt::host_io::parser::FieldValue;
                        // Ignore send errors — if the channel is closed the
                        // imminent PushSegment will surface the failure through
                        // the normal credit / fault path.
                        let _ = io.send_typed(
                            "runtime_seed_position",
                            &[
                                ("x_q16", FieldValue::I32(seed_x_q16)),
                                ("y_q16", FieldValue::I32(seed_y_q16)),
                                ("z_q16", FieldValue::I32(z_q16)),
                            ],
                        );
                    }
                }

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
                log::info!(
                    "[bridge-trace] mcu_plans built: count={} mcu_ids=[{}]",
                    mcu_plans.len(),
                    mcu_plans
                        .iter()
                        .map(|p| format!("{}({}c)", p.mcu_id, p.curves_to_load.len()))
                        .collect::<Vec<_>>()
                        .join(","),
                );

                // Cap-check each curve against the destination MCU's caps before
                // dispatch. `build_push_params` does not enforce caps; if a planned
                // curve exceeds its target's pool slot, fail-fast (Task 13 reduced
                // scope; full per-segment bisection is a follow-up).
                for plan in &mcu_plans {
                    let caps = mcu_configs_for_cb
                        .iter()
                        .find(|c| c.mcu_id == plan.mcu_id)
                        .map(|c| c.caps)
                        .unwrap_or_default();
                    // Cap source: the MCU's own RuntimeCapsResponse (post-2026-05-20
                    // cubic-only schema), reported at attach via QueryRuntimeCaps.
                    // The earlier `kalico_host_rt::producer::MAX_PIECES_PER_CURVE`
                    // host constant was a placeholder pinned to the firmware
                    // default; with `RUNTIME_MAX_PIECES_PER_CURVE` now a Kconfig
                    // knob (and bumped from 16→64 for bench bring-up), the
                    // authoritative value lives on each MCU and varies per build.
                    let max_pieces = caps.max_pieces_per_curve as usize;
                    for (_axis, curve) in &plan.curves_to_load {
                        let pieces = curve.piece_count();
                        if pieces > max_pieces {
                            let err = DispatchError::CapsExceeded {
                                mcu_id: plan.mcu_id,
                                pieces,
                                max_pieces,
                            };
                            log::error!("{err}");
                            return Err(err);
                        }
                    }
                }

                for mut plan in mcu_plans {
                    // Skip plans targeting MCUs without kalico-runtime — stock
                    // Klipper firmware can't decode load_curve / push_segment.
                    // Their motion has to be driven via the legacy Klipper-
                    // protocol path (out of scope for the kalico planner).
                    if !kalico_native_for_plans
                        .get(&plan.mcu_id)
                        .copied()
                        .unwrap_or(false)
                    {
                        log::info!(
                            "[bridge-trace] skipping plan mcu={} (not kalico-native)",
                            plan.mcu_id,
                        );
                        continue;
                    }
                    let (io_weak, credit, slot_pool) = match dispatch_ios.get(&plan.mcu_id) {
                        Some(v) => v,
                        None => continue,
                    };
                    let io = match io_weak.upgrade() {
                        Some(io) => io,
                        None => {
                            return Err(DispatchError::ConnectionDropped(plan.mcu_id));
                        }
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
                            let mut warned = warned_mcus.lock().unwrap_or_else(|p| p.into_inner());
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

                    // Captured outside the base-clock block for the
                    // post-dispatch diagnostic log (2026-05-11): they tell us
                    // whether the segment activates on-time or late.
                    let _diag_now_clock: u64;
                    let _diag_branch_outer: &'static str;
                    // Compute schedule base ONCE per (mcu, logical-segment).
                    let mcu_base_clock: u64 = {
                        // Block-wait for clock-sync to publish a non-zero
                        // widened MCU clock. If we dispatched against
                        // `now_clock=0` (clock-sync hasn't establish a real
                        // `last_clock` yet — happens on the first dispatch
                        // immediately after klippy restart while the 8-
                        // iteration clocksync calibration is still in
                        // flight), `t_start_clock = 0 + lead_cycles` lands
                        // far below the firmware's widened `now`. The engine
                        // sees the segment as already-in-the-past and the
                        // boundary loop retires it in one tick without
                        // evaluating any intermediate u-values — segment_id
                        // advances, status goes Drained, but zero step
                        // pulses fire ('first jog after restart doesn't
                        // move' bench symptom 2026-05-11).
                        let lead_cycles_init = (freq * 0.250).round().max(1.0) as u64;
                        let wait_start = Instant::now();
                        let mut wait_iter: u32 = 0;
                        let now_clock = loop {
                            let r = router_for_cb.lock().unwrap_or_else(|p| p.into_inner());
                            let n = r
                                .compute_ack_clock(mcu_h)
                                .map_err(|e| DispatchError::ComputeAckClock(e.to_string()))?;
                            drop(r);
                            if n > 0 {
                                break n;
                            }
                            if wait_iter == 0
                                || wait_iter == 50
                                || wait_iter == 250
                                || wait_iter == 499
                            {
                                log::debug!(
                                    "[bridge-trace] dispatch-wait iter={} mcu_id={} mcu_h={:?} now_clock=0 freq_from_map={}",
                                    wait_iter,
                                    plan.mcu_id,
                                    mcu_h,
                                    freq as u64,
                                );
                            }
                            wait_iter += 1;
                            if wait_start.elapsed() > Duration::from_secs(5) {
                                return Err(DispatchError::ClockSyncTimeout {
                                    mcu_id: plan.mcu_id,
                                    mcu_handle: mcu_h,
                                });
                            }
                            std::thread::sleep(Duration::from_millis(10));
                        };
                        let lead_cycles = lead_cycles_init;

                        // One-shot diag (per-session): on the very first
                        // dispatched segment, dump now_clock + freq + lead.
                        use std::sync::atomic::{AtomicBool, Ordering as AOrd};
                        static FIRST_DISPATCH_LOGGED: AtomicBool = AtomicBool::new(false);
                        if !FIRST_DISPATCH_LOGGED.swap(true, AOrd::AcqRel) {
                            log::debug!(
                                "[bridge-trace] first-dispatch mcu={} freq={} now_clock={} lead_cycles={} seg.t_start={:.6} clock_sync_wait_ms={}",
                                plan.mcu_id,
                                freq as u64,
                                now_clock,
                                lead_cycles,
                                seg.t_start,
                                wait_start.elapsed().as_millis(),
                            );
                        }

                        let mut schedule = schedule_state.lock().unwrap_or_else(|p| p.into_inner());
                        let entry = schedule.entry(plan.mcu_id).or_insert((0, 0));
                        let now_plus_lead = now_clock.saturating_add(lead_cycles);
                        let planner_offset_cycles = (seg.t_start * freq).round().max(0.0) as u64;
                        let _diag_prev_entry1 = entry.1;
                        _diag_now_clock = now_clock;
                        if entry.1 == 0 || seg.t_start <= 1.0e-12 {
                            // Fresh trajectory: first ever dispatch on this MCU,
                            // or planner-time-zero (post-stream-open reset).
                            // Anchor base so the first segment lands at now+lead.
                            entry.0 = entry.1.max(now_plus_lead);
                            _diag_branch_outer = "fresh";
                        } else if entry.1 < now_clock {
                            // Continuity break: the previous dispatched segment's
                            // t_end_clock is already in the past, so the MCU
                            // engine has drained and is sitting idle. Rebase so
                            // *this* segment lands at now+lead — that is, choose
                            // a base such that `base + planner_t_start*freq ==
                            // now+lead`. The planner emits cumulative
                            // `seg.t_start` values that grow across moves
                            // (continuous streaming model); without this
                            // subtraction, the dispatch would add the cumulative
                            // planner time on top of wall-clock now, producing
                            // a perceived ~planner_t_start-second delay before
                            // motion starts (bench-observed 2026-05-11: "second
                            // jog is slower" when clicks bracket a drained gap).
                            entry.0 = now_plus_lead.saturating_sub(planner_offset_cycles);
                            _diag_branch_outer = "rebase-drained";
                        } else {
                            // Previous dispatched segment is still in flight
                            // on the MCU (entry.1 >= now_clock). Continuous streaming
                            // — keep the existing base so the new segment threads
                            // onto the end of the previous trajectory at the
                            // planner's intended t_start.
                            _diag_branch_outer = "continuous";
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
                        let mut schedule = schedule_state.lock().unwrap_or_else(|p| p.into_inner());
                        let entry = schedule.entry(plan.mcu_id).or_insert((0, 0));
                        entry.1 = entry.1.max(t_end_clock);
                    }

                    plan.params.t_start = t_start_clock;
                    plan.params.t_end = t_end_clock;

                    // Allocate a fresh segment id for this logical move (one per
                    // MCU per ShapedSegment, restoring pre-B.1 semantics).
                    {
                        let mut ids = next_seg_id.lock().unwrap_or_else(|p| p.into_inner());
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
                    let mut seg_err: Option<DispatchError> = None;
                    for i in 0..plan.curves_to_load.len() {
                        let axis_idx = plan.curves_to_load[i].0;
                        let curve_params = plan.curves_to_load[i].1.clone();
                        let alloc_result = {
                            let mut pool = slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                            let cap = pool.capacity();
                            let in_flight = pool.in_flight_count();
                            pool.try_alloc().ok_or(DispatchError::SlotPoolExhausted {
                                mcu_id: plan.mcu_id,
                                capacity: cap,
                                in_flight,
                            })
                        };
                        let (slot, slot_gen) = match alloc_result {
                            Ok(v) => v,
                            Err(e) => {
                                seg_err = Some(e);
                                break;
                            }
                        };
                        let pool_in_flight_after_alloc = {
                            let p = slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                            p.in_flight_count()
                        };
                        log::debug!(
                            "[slot-trace] try_alloc mcu={} seg_id={} axis={} slot={} gen={} in_flight={}",
                            plan.mcu_id,
                            plan.params.id,
                            axis_idx,
                            slot,
                            slot_gen,
                            pool_in_flight_after_alloc,
                        );
                        allocated_slots.push(slot);
                        match producer::load_curve(
                            io.as_ref(),
                            slot,
                            axis_idx as u8,
                            &curve_params,
                            producer::DEFAULT_LOAD_CURVE_TIMEOUT,
                        ) {
                            Ok(handle) => {
                                plan.set_handle(axis_idx, handle);
                            }
                            Err(e) => {
                                seg_err = Some(DispatchError::LoadCurve {
                                    mcu_id: plan.mcu_id,
                                    slot,
                                    seg_id: plan.params.id,
                                    axis: axis_idx,
                                    host_gen: slot_gen,
                                    detail: e.to_string(),
                                });
                                break;
                            }
                        }
                    }

                    if let Some(err) = seg_err {
                        // Partial failure: release every slot allocated for this
                        // segment before propagating.
                        let mut pool = slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                        for s in &allocated_slots {
                            log::debug!(
                                "[slot-trace] release(on-err) mcu={} seg_id={} slot={}",
                                plan.mcu_id,
                                plan.params.id,
                                s,
                            );
                            pool.release(*s);
                        }
                        return Err(err);
                    }

                    // Bind every freshly-allocated slot to this segment id so
                    // `kalico_credit_freed`-driven retirement can release them.
                    {
                        let mut pool = slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                        for slot in &allocated_slots {
                            pool.register_segment(*slot, plan.params.id);
                            log::debug!(
                                "[slot-trace] register_segment mcu={} slot={} seg_id={}",
                                plan.mcu_id,
                                slot,
                                plan.params.id,
                            );
                        }
                    }

                    // Diagnostic 2026-05-11: dump per-segment dispatch info to
                    // distinguish bridge-side curve discontinuity from
                    // late-activation (u_entry > 0) as the cause of
                    // STEP_BURST_EXCEEDED on segment 3 of the second jog.
                    // Evaluate the shaped curves at their `seg.t_start` and
                    // `seg.t_end` so cross-segment continuity can be verified
                    // from the log alone: if seg N's pos_end matches seg N+1's
                    // pos_start, the planner emits continuous geometry and any
                    // burst must be late-activation; if they differ by the
                    // observed delta (e.g. ~15 mm), the discontinuity is real.
                    let pos_at = |axis: usize, u: f64| -> f64 {
                        let curve = &seg.axes[axis];
                        nurbs::eval::eval(curve, u)
                    };
                    log::debug!(
                        "[bridge-trace] seg-dispatch mcu={} seg_id={} branch={} \
                     now_clock={} mcu_base={} t_start_clock={} t_end_clock={} \
                     t_start_s={:.6} t_end_s={:.6} \
                     pos_start=[{:.4},{:.4},{:.4}] pos_end=[{:.4},{:.4},{:.4}]",
                        plan.mcu_id,
                        plan.params.id,
                        _diag_branch_outer,
                        _diag_now_clock,
                        mcu_base_clock,
                        t_start_clock,
                        t_end_clock,
                        seg.t_start,
                        seg.t_end,
                        pos_at(0, seg.t_start),
                        pos_at(1, seg.t_start),
                        pos_at(2, seg.t_start),
                        pos_at(0, seg.t_end),
                        pos_at(1, seg.t_end),
                        pos_at(2, seg.t_end),
                    );

                    let push_result = dispatch_push_segment(io.as_ref(), credit, &plan.params);
                    match &push_result {
                        Ok(info) => log::info!(
                            "[bridge-trace] push_segment ok: mcu={} sent_id={} accepted_id={} credit_epoch={}",
                            plan.mcu_id,
                            plan.params.id,
                            info.accepted_segment_id,
                            info.credit_epoch,
                        ),
                        Err(e) => log::info!(
                            "[bridge-trace] push_segment err: mcu={} sent_id={} err={:?}",
                            plan.mcu_id,
                            plan.params.id,
                            e,
                        ),
                    }
                    if let Err(e) = push_result {
                        // Defensive cleanup — release this segment's slots so
                        // the pool doesn't leak (the MCU never accepted them).
                        let mut pool = slot_pool.lock().unwrap_or_else(|p| p.into_inner());
                        for s in &allocated_slots {
                            pool.release(*s);
                        }
                        return Err(DispatchError::PushSegment {
                            mcu_id: plan.mcu_id,
                            detail: e.to_string(),
                        });
                    }
                }

                counter.fetch_add(1, Ordering::Relaxed);
                Ok(())
            },
        );

        // `set` returns `Err(handle)` if the slot was concurrently
        // initialized. The early `get().is_some()` check above (combined
        // with klippy's GIL-serialized init path) makes this race a
        // logic bug rather than a benign retry — surface it explicitly.
        self.planner
            .set(PlannerHandle::spawn(cfg, dispatch))
            .map_err(|_| PyRuntimeError::new_err("planner already initialized (raced)"))?;
        Ok(())
    }

    /// Submit a travel move. Phase 2: `de` must be 0.
    //
    // `py.allow_threads` releases the GIL across `classify_and_build`
    // (NURBS construction + validation, real work) and the planner mutex
    // acquisitions, so the clock-sync thread and other Python callers can
    // make progress under sustained motion submission. The channel send
    // inside `planner.submit_move` is unbounded today, but releasing the
    // GIL here also future-proofs against converting it to bounded
    // backpressure without retrofitting every call-site.
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
        eprintln!(
            "[move-diag] bridge.submit_move enter dx={:.3} dy={:.3} dz={:.3} de={:.3} feed={:.1}",
            dx, dy, dz, de, feedrate,
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

    /// Submit one homing-tagged absolute move. MVP watches the first arm id;
    /// multi-arm logical OR is Step 10.
    #[pyo3(signature = (newpos, speed, arm_ids))]
    fn submit_homing_move(&self, newpos: Vec<f64>, speed: f64, arm_ids: Vec<u32>) -> PyResult<()> {
        self.submit_homing_move_inner(&newpos, speed, &arm_ids)
    }

    /// Flush all pending moves and block until every queued segment is on
    /// the wire.
    ///
    /// ## Contract (Phase 6 Task 7.3 — "wait_for_dispatch_to_match_append")
    ///
    /// When this returns, **dispatched time covers queued time**:
    /// every move that was previously submitted via `submit_move` /
    /// `submit_homing_move` has been dispatched all the way through its
    /// trailing decel-to-zero ramp. The bridge atomic
    /// (`last_move_time_bits`) reflects `t_appended` (queued time) under
    /// Phase 6's caller-side-advance semantics; after this call returns,
    /// the dispatched-segment window covers `[0, last_move_time]` up to
    /// the rectification tolerance (1 µs).
    ///
    /// This is what `M400` and homing actually need: a barrier that
    /// blocks until the toolhead has been commanded the full submitted
    /// distance — not just until the planner thread acknowledges the
    /// queue. Phase 4 Task 4.3 ships the mechanism (`PlannerMsg::Flush`
    /// synchronously calls `commit_decel_to_zero` + dispatches the
    /// held-back tail before notifying the waiter); Phase 6 Task 7.3
    /// pins the integration-layer invariant via
    /// `streaming_replan::wait_moves_blocks_until_dispatch_catches_up`.
    ///
    /// Inline-event scheduling (M106, SET_PIN AT_TIME, TMC register
    /// updates, fan transitions) does **not** need this barrier — those
    /// callers read `get_last_move_time` directly and schedule against
    /// the queued timeline, which advances synchronously on
    /// `submit_move`.
    fn wait_moves(&self, py: Python<'_>) -> PyResult<()> {
        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        py.allow_threads(|| planner.flush()).map_err(planner_err)?;
        self.homing.refresh_after_wait();
        Ok(())
    }

    fn take_trip_event(&self, py: Python<'_>) -> PyResult<Option<Py<PyDict>>> {
        let Some(evt) = self.homing.take_trip_event() else {
            return Ok(None);
        };
        Ok(Some(trip_event_to_pydict(py, evt)?))
    }

    // ── Step 7-D: endstop arm/disarm wire surface ──────────────────────────
    //
    // These call the kalico-host-rt producer functions over the same
    // KalicoHostIo reactor queue used by bridge_call / bridge_send.
    // Each Python call is one synchronous msgproto round-trip. The Python
    // side (`klippy/motion_bridge.py::BridgeTriggerDispatch`) wraps these
    // and handles async `kalico_endstop_tripped` events via the existing
    // `passthrough_register_handler` plumbing.

    /// Send `runtime_arm_endstop` and wait for the synchronous response.
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

    /// Send `runtime_disarm_endstop` and wait for the response. Returns the
    /// status byte (0=Disarmed, 1=AlreadyTripped, 2=Unknown) per spec §3.5.
    #[pyo3(signature = (mcu, queue, arm_id, timeout_s=0.1))]
    fn endstop_disarm(&self, mcu: u32, queue: u32, arm_id: u32, timeout_s: f64) -> PyResult<u8> {
        use kalico_host_rt::endstop;
        let _ = queue;
        let io = self.host_io_for_mcu("endstop_disarm", mcu)?;
        let timeout = std::time::Duration::from_secs_f64(timeout_s);
        let status = endstop::disarm_endstop_with_timeout(io.as_ref(), arm_id, timeout)
            .map_err(|e| PyRuntimeError::new_err(format!("endstop_disarm: {e}")))?;
        Ok(status as u8)
    }

    // ── Step 7-E: async homing submission + software trip + deadline ──────────
    //
    // `submit_homing_move_async` submits a homing move without blocking; the
    // Python caller polls `is_homing_segment_retired` in its credit loop.
    // `software_trip` and `extend_homing_deadline` are wire commands that map
    // directly onto the corresponding firmware runtime commands.

    /// Submit one homing-tagged absolute move and return immediately.
    /// Unlike `submit_homing_move`, this does **not** call `wait_moves` —
    /// the caller is expected to poll `is_homing_segment_retired` to detect
    /// completion.
    #[pyo3(signature = (newpos, speed, arm_ids))]
    fn submit_homing_move_async(
        &self,
        newpos: Vec<f64>,
        speed: f64,
        arm_ids: Vec<u32>,
    ) -> PyResult<()> {
        self.submit_homing_move_inner(&newpos, speed, &arm_ids)
        // No wait_moves — returns immediately.
    }

    /// Returns `true` once the homing segment has reached a terminal state:
    /// `Completed`, `Tripped`, or `DeadlineExpired`.
    fn is_homing_segment_retired(&self) -> bool {
        matches!(
            self.homing.state(),
            crate::homing::HomingSegmentState::Completed
                | crate::homing::HomingSegmentState::Tripped
                | crate::homing::HomingSegmentState::DeadlineExpired
        )
    }

    /// Returns a reason code after `is_homing_segment_retired` is `true`.
    ///
    /// | Code | Meaning |
    /// |------|---------|
    /// | 0    | Still active or idle (not yet retired) |
    /// | 1    | Completed — move ran to end time with no trigger |
    /// | 2    | Tripped — software_trip or GPIO trigger fired |
    /// | 3    | DeadlineExpired — deadline elapsed before completion |
    fn get_homing_segment_reason(&self) -> u8 {
        match self.homing.state() {
            crate::homing::HomingSegmentState::Completed => 1,
            crate::homing::HomingSegmentState::Tripped => 2,
            crate::homing::HomingSegmentState::DeadlineExpired => 3,
            _ => 0,
        }
    }

    /// Send `runtime_software_trip arm_id=%u` to the MCU and wait for the
    /// `kalico_software_trip_response`. Returns the status byte from the MCU.
    #[pyo3(signature = (mcu, arm_id, timeout_s=0.1))]
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

    /// Send `runtime_extend_homing_deadline arm_id=%u` to the MCU.
    /// Fire-and-forget — no response is expected.
    #[pyo3(signature = (mcu, arm_id))]
    fn extend_homing_deadline(&self, mcu: u32, arm_id: u32) -> PyResult<()> {
        let io = self.host_io_for_mcu("extend_homing_deadline", mcu)?;
        let msg = format!("runtime_extend_homing_deadline arm_id={arm_id}");
        io.send_fire_and_forget(&msg)
            .map_err(|e| PyRuntimeError::new_err(format!("extend_homing_deadline: {e}")))?;
        Ok(())
    }

    /// Submit a dwell: flush + advance print time.
    fn submit_dwell(&self, duration_s: f64) -> PyResult<()> {
        let planner = self.planner.get().ok_or_else(|| {
            PyRuntimeError::new_err("planner not initialized — call init_planner first")
        })?;
        planner.dwell(duration_s).map_err(planner_err)
    }

    /// Reset commanded position. Klippy calls this on every homing
    /// completion (`SET_KINEMATIC_POSITION`, `G28`, manual stepper moves,
    /// fault-recovery reconnect) so it is the natural hook to re-anchor
    /// the streaming planner's `ShaperState`.
    ///
    /// **Phase 5 Task 5.5 — explicit engine-fault → klippy reset.** Spec
    /// §3.7 ("Engine fault → klippy reset"): "Explicit
    /// `ShaperState::reset(home_pos)` on klippy reconnect." `init_planner`
    /// already does this implicitly by constructing a fresh
    /// `PlannerHandle::spawn(...)` with `ShaperState::new([0.0; 4], &shapers)`
    /// — so the *very first* connect / *clean* reconnect (planner is dropped
    /// and recreated) is already handled. But klippy can also reset the
    /// kinematic position without reinitialising the planner (e.g.,
    /// `SET_KINEMATIC_POSITION` after a homing completion, or a
    /// fault-recovery path that re-uses the existing planner thread). In
    /// those cases `set_position` is the only signal the bridge receives
    /// that the host-side notion of "where the toolhead is" has changed.
    ///
    /// We forward the new position into the planner via
    /// `PlannerHandle::kalico_stream_open`, which re-seeds each axis queue
    /// to `home_pos` at `v = 0` and clears any held-back tail (preserving
    /// kernels). The E axis tracks shaped XY arc-length under the
    /// COUPLED_TO_XY model and is not commanded directly via
    /// `set_position`; we pass `0.0` for the E slot.
    ///
    /// If the planner has not yet been initialised the call is a no-op
    /// (matches the pre-Task-5.5 behaviour — `set_position` worked even
    /// before motion submission was wired). The forward error is
    /// surfaced if the planner channel has closed (planner thread
    /// crashed) so callers see the failure rather than silently losing
    /// the re-anchor.
    fn set_position(&self, x: f64, y: f64, z: f64) -> PyResult<()> {
        {
            let mut pos = self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner());
            *pos = [x, y, z];
        }
        // Forward to the planner so the streaming `ShaperState` is
        // re-anchored to the new home position. See doc above for the
        // Task 5.5 rationale; `kalico_stream_open` is the entry point
        // the planner registers for this lifecycle event (see
        // `PlannerHandle::kalico_stream_open` and
        // `streaming::ShaperState::reset`).
        if let Some(planner) = self.planner.get() {
            planner
                .kalico_stream_open([x, y, z, 0.0])
                .map_err(planner_err)?;
        }

        // Seed the MCU engine's prev_x/y/z so the first segment after
        // SET_KINEMATIC_POSITION computes its delta against the correct
        // origin rather than the boot-time (0, 0, 0). Without this the
        // delta for a move starting at e.g. Y=100 is computed as
        // (Y_end - 0) instead of (Y_end - 100), which exceeds
        // MAX_STEPS_PER_TICK_DEFAULT and raises FaultCode::StepBurstExceeded.
        //
        // We do NOT send `runtime_seed_position` here directly.  In-flight
        // segments from a previous move (e.g. a retract queued during homing)
        // may not have reached the MCU yet.  Firing the seed immediately would
        // overwrite the MCU's `prev_x/y/z` before the retract finishes,
        // corrupting its step-delta computation.
        //
        // Instead, store the seed as `pending_seed`.  The dispatch closure
        // (planner thread) drains it before sending the next segment, which
        // guarantees the seed arrives AFTER all previously-dispatched segments.
        *self
            .pending_seed
            .lock()
            .unwrap_or_else(|p| p.into_inner()) = Some(SeedPosition { x, y, z });

        Ok(())
    }

    /// Update velocity / acceleration limits at runtime
    /// (klippy `SET_VELOCITY_LIMIT`).
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

    /// Update shaper config at runtime (klippy `SET_INPUT_SHAPER`).
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

    /// Estimated print time of the last queued move, in seconds.
    fn get_last_move_time(&self) -> f64 {
        match self.planner.get() {
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
    /// Returns `(n_released, completed_arm)`. `completed_arm` is
    /// `Some(arm_id)` when a homing segment retired without a trip in this
    /// credit-freed cycle; the caller is responsible for firing the matching
    /// dispatch's `_completion`. Unknown MCU is a no-op returning `(0,
    /// None)` (defensive — events for un-claimed MCUs are dropped).
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
    ) -> PyResult<(u32, Option<u32>)> {
        let (n_released, in_flight_after) = match self
            .slot_pools
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu)
        {
            Some(pool_arc) => {
                let mut p = pool_arc.lock().unwrap_or_else(|p| p.into_inner());
                let n = p.retire_through_segment(retired_through_segment_id);
                (n, p.in_flight_count())
            }
            None => (0, 0),
        };
        log::debug!(
            "[slot-trace] on_credit_freed mcu={} retired_through={} free_slots={} \
             n_released={} in_flight_after={}",
            mcu,
            retired_through_segment_id,
            free_slots,
            n_released,
            in_flight_after,
        );
        if let Some(c) = self
            .credit_counters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .get(&mcu)
        {
            c.on_credit_freed(free_slots);
        }
        self.homing.complete_if_retired(retired_through_segment_id);
        let completed_arm = self.homing.take_completion_event();
        Ok((n_released as u32, completed_arm))
    }

    /// Number of curve slots currently in flight on the given MCU. Test /
    /// diagnostic hook.
    fn slot_pool_in_flight(&self, mcu: u32) -> u32 {
        self.slot_pools
            .lock()
            .unwrap()
            .get(&mcu)
            .map(|p| {
                p.lock()
                    .unwrap_or_else(|p| p.into_inner())
                    .in_flight_count() as u32
            })
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
        // TODO(Step 10): accept all arm_ids as a logical OR set.
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
        let pool = Arc::new(Mutex::new(SlotPool::new(crate::slot_pool::CURVE_POOL_N)));
        bridge
            .slot_pools
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, Arc::clone(&pool));
        let credit = Arc::new(CreditCounter::new(CREDIT_SEED_CAPACITY));
        bridge
            .credit_counters
            .lock()
            .unwrap_or_else(|p| p.into_inner())
            .insert(mcu, credit);
        pool
    }

    #[test]
    fn on_credit_freed_releases_eligible_slots() {
        let bridge = PyMotionBridge::new();
        let mcu = 1u32;
        let pool = install_mcu(&bridge, mcu);

        // Allocate three in-flight segments with monotonic ids.
        {
            let mut p = pool.lock().unwrap_or_else(|p| p.into_inner());
            for seg_id in 1u32..=3 {
                let (slot, _credit) = p.try_alloc().expect("pool has capacity for three allocs");
                p.register_segment(slot, seg_id);
            }
            assert_eq!(p.in_flight_count(), 3);
        }

        // MCU reports retirement through segment 2 — slots for ids 1,2 free.
        let (n, _arm) = bridge
            .on_credit_freed(mcu, 2, /* free_slots */ 2)
            .expect("on_credit_freed returns Ok");
        assert_eq!(n, 2, "two slots should be released");
        assert_eq!(
            pool.lock()
                .unwrap_or_else(|p| p.into_inner())
                .in_flight_count(),
            1
        );

        // Higher-id retirement releases the rest.
        let (n, _arm) = bridge
            .on_credit_freed(mcu, 100, 1)
            .expect("on_credit_freed returns Ok");
        assert_eq!(n, 1);
        assert_eq!(
            pool.lock()
                .unwrap_or_else(|p| p.into_inner())
                .in_flight_count(),
            0
        );
    }

    #[test]
    fn on_credit_freed_unknown_mcu_is_noop() {
        // A retirement event for an MCU we don't track must not panic and
        // must report zero released. Defensive — guards against the bridge
        // being mid-teardown when an event arrives.
        let bridge = PyMotionBridge::new();
        let (n, arm) = bridge
            .on_credit_freed(/* mcu */ 99, /* retired */ 5, /* free */ 1)
            .expect("on_credit_freed must not error on unknown MCU");
        assert_eq!(n, 0);
        assert!(arm.is_none());
    }

    #[test]
    fn on_credit_freed_before_any_alloc_is_noop() {
        // Startup race: MCU emits a credit_freed before any segment has
        // been dispatched. retire_through_segment is idempotent on an
        // empty pool — verify the bridge's PyO3 entry point inherits that.
        let bridge = PyMotionBridge::new();
        let mcu = 1u32;
        install_mcu(&bridge, mcu);
        let (n, arm) = bridge
            .on_credit_freed(mcu, u32::MAX, 0)
            .expect("on_credit_freed must not error on empty pool");
        assert_eq!(n, 0);
        assert!(arm.is_none());
    }

    #[test]
    fn on_credit_freed_returns_arm_id_when_homing_segment_retires() {
        // Arrange: install an MCU, begin homing with arm_id=99, mark
        // segment 7 as the dispatched homing segment.
        let bridge = PyMotionBridge::new();
        let mcu = 1u32;
        install_mcu(&bridge, mcu);
        bridge.homing.begin(99);
        bridge.homing.mark_dispatched_segment(7);

        // Act: simulate the MCU reporting that segment 7 retired (without
        // a trip).
        let (_, arm) = bridge
            .on_credit_freed(mcu, 7, 0)
            .expect("on_credit_freed must not error");

        // Assert: the homing terminal surfaces as Some(arm_id).
        assert_eq!(arm, Some(99));

        // Take-once: a second credit-freed for the same segment should
        // not re-fire the completion (the take_completion_event swap
        // gates this).
        let (_, arm2) = bridge
            .on_credit_freed(mcu, 7, 0)
            .expect("on_credit_freed must not error");
        assert_eq!(arm2, None);
    }
}

#[cfg(test)]
mod build_configure_axes_body_tests {
    //! Unit tests for the pure `build_configure_axes_body` byte builder.
    //!
    //! These exercise the three wire layouts (20 / 25 / 26+3N bytes)
    //! without standing up a PyO3 transport or mock MCU.

    use super::*;

    #[test]
    fn build_configure_axes_body_legacy_20() {
        let body = build_configure_axes_body(
            /* kinematics */ 0,
            /* present_mask */ 0x0F,
            /* awd_mask */ 0x03,
            /* invert_mask */ 0,
            &[160.0, 160.0, 800.0, 800.0],
            /* step_modes */ None,
            /* phase_configs */ None,
            /* phase_capable */ 0,
        );
        assert_eq!(body.len(), 20, "legacy body is 20 bytes");
        assert_eq!(body[0], 0);
        assert_eq!(body[1], 0x0F);
        assert_eq!(body[2], 0x03);
        assert_eq!(body[3], 0);
        assert_eq!(&body[4..8], &160.0f32.to_le_bytes());
        assert_eq!(&body[16..20], &800.0f32.to_le_bytes());
    }

    #[test]
    fn build_configure_axes_body_step_modes_25() {
        let body = build_configure_axes_body(
            0,
            0x0F,
            0x03,
            0,
            &[160.0, 160.0, 800.0, 800.0],
            Some(&[0, 0, 1, 1]),
            None,
            /* phase_capable */ 1,
        );
        assert_eq!(body.len(), 25, "step-modes body is 25 bytes");
        assert_eq!(body[20], 1, "byte 20 carries phase_capable");
        assert_eq!(&body[21..25], &[0u8, 0, 1, 1], "step_modes array");
    }

    #[test]
    fn build_configure_axes_body_phase_configs_variable_n1() {
        // Single phase-stepped motor on slot 0.
        let body = build_configure_axes_body(
            1,
            0x0F,
            0x00,
            0,
            &[160.0, 160.0, 800.0, 800.0],
            Some(&[0, 1, 1, 1]),
            Some(&[(3, 5, 0)]),
            /* phase_capable */ 1,
        );
        assert_eq!(body.len(), 26 + 3 * 1, "N=1 body is 29 bytes");
        assert_eq!(body[20], 1, "byte 20 carries phase_capable");
        assert_eq!(&body[21..25], &[0u8, 1, 1, 1], "step_modes array");
        assert_eq!(body[25], 1, "byte 25 is phase_motor_count");
        assert_eq!(&body[26..29], &[3u8, 5, 0], "(bus, cs, slot_idx)");
    }

    #[test]
    fn build_configure_axes_body_phase_configs_variable_n4_corexy_awd() {
        // CoreXY+AWD: 4 motors driving 2 kinematic slots — slot 0 is
        // motor pair (stepper_x, stepper_x1), slot 1 is (stepper_y,
        // stepper_y1). slot_idx layout [0,0,1,1].
        let body = build_configure_axes_body(
            0,
            0x0F,
            0x03,
            0,
            &[160.0, 160.0, 800.0, 800.0],
            Some(&[0, 0, 1, 1]),
            Some(&[(3, 5, 0), (3, 6, 0), (3, 7, 1), (3, 8, 1)]),
            /* phase_capable */ 1,
        );
        assert_eq!(body.len(), 26 + 3 * 4, "N=4 body is 38 bytes");
        assert_eq!(body[20], 1);
        assert_eq!(&body[21..25], &[0u8, 0, 1, 1]);
        assert_eq!(body[25], 4, "phase_motor_count");
        assert_eq!(
            &body[26..38],
            &[3u8, 5, 0, 3, 6, 0, 3, 7, 1, 3, 8, 1],
            "(bus, cs, slot_idx) triples for AWD-paired CoreXY motors",
        );
    }

    #[test]
    fn build_configure_axes_body_phase_configs_variable_n8() {
        // High-motor-count config: 8 motors. Verifies the count byte and
        // full 50-byte body encode correctly.
        let entries: Vec<(u8, u8, u8)> = (0u8..8u8).map(|i| (3, 0x10 + i, i % 4)).collect();
        let body = build_configure_axes_body(
            1,
            0x0F,
            0x00,
            0,
            &[160.0, 160.0, 800.0, 800.0],
            Some(&[0, 0, 0, 0]),
            Some(&entries),
            /* phase_capable */ 1,
        );
        assert_eq!(body.len(), 26 + 3 * 8, "N=8 body is 50 bytes");
        assert_eq!(body[25], 8, "phase_motor_count");
        for (i, (bus, cs, slot)) in entries.iter().enumerate() {
            let off = 26 + i * 3;
            assert_eq!(body[off], *bus, "entry[{i}].bus");
            assert_eq!(body[off + 1], *cs, "entry[{i}].cs");
            assert_eq!(body[off + 2], *slot, "entry[{i}].slot");
        }
    }
}

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
