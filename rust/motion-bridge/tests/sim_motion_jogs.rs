//! Wire-format sim-driven reproduction harness for the live-printer jog bugs.
//!
//! Unlike `bench_repro.rs` (which drives the host engine in-process and bypasses
//! the wire format / clock-sync / MCU widening), this harness:
//!
//!   1. Spawns Renode running the H723 sim firmware (`tools/sim/run_sim.sh`).
//!   2. Connects the production `motion-bridge` stack — `PlannerHandle` +
//!      `producer::load_curve` / `producer::push_segment` over a real
//!      `KalicoHostIo` reactor — to the sim's TCP socket (`localhost:3334`).
//!   3. Mirrors the bridge's own dispatch closure so segments traverse the
//!      same wire path klippy drives in production (clock-sync establishment,
//!      `compute_ack_clock` gating, `kalico_stream_open` reset, etc.).
//!   4. Submits the three jog sequences the user runs on hardware (Tests A/B/C
//!      below) and asserts on `dispatched_segments` + the MCU's StatusEvent
//!      stream (`current_segment_id`).
//!
//! ## What this catches that `bench_repro.rs` does not
//!
//! `bench_repro.rs` reproduces the offline planner bugs (feedrate ignored,
//! mid-segment velocity discontinuity) but it cannot catch wire-format bugs:
//! clock-sync race conditions, `WidenState` seeding, USB-CDC / TCP buffering,
//! `compute_ack_clock` returning 0 on first dispatch. The live-bench symptom
//! "first jog energizes motors but doesn't move" only manifests on the real
//! wire-format boundary — that's the bug this harness is designed to surface.
//!
//! ## Step-pulse observation
//!
//! Renode's H743 platform definition tags GPIOs as opaque memory regions, so
//! we cannot observe per-stepper step pulses directly from the sim. What we
//! CAN observe via the StatusEvent channel:
//!
//!   * `current_segment_id` — advances when the MCU's engine retires a
//!     segment. If the bridge dispatches N segments but `current_segment_id`
//!     never advances past 0, the firmware never accepted the wire frames
//!     (the "first jog" bug signature).
//!   * `engine_status` — Idle → Streaming → Drained. A 0-pulse retirement
//!     still passes through Drained, but the `t_end - t_start` window must
//!     match the feedrate; if the firmware sees a zero-duration segment
//!     because `t_start_clock >> mcu_now`, it retires in one tick (which
//!     `bench_repro::single_segment_has_monotone_velocity_profile` can
//!     detect on the host side, but here it manifests as
//!     `current_segment_id` advancing without the expected time elapsing).
//!
//! ## Prerequisites
//!
//! Run once before invoking these tests:
//!
//! ```bash
//! bash tools/sim/build_sim_firmware.sh   # builds out/klipper.elf
//! ```
//!
//! These tests are `#[ignore]`d by default — they spawn external processes
//! and take ~30 s each. Run explicitly:
//!
//! ```bash
//! cargo test -p motion-bridge --test sim_motion_jogs -- --ignored \
//!     --test-threads=1 --nocapture
//! ```
//!
//! `--test-threads=1` is **mandatory** — every test owns the singleton TCP
//! port 3334. Parallel tests would race for the sim subprocess.

#![allow(clippy::too_many_lines)]
#![allow(clippy::unwrap_used)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_precision_loss)]

use std::collections::HashMap;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

use kalico_host_rt::clock_sync::ClockSyncEstimator;
use kalico_host_rt::credit::CreditCounter;
use kalico_host_rt::host_io::{KalicoHostIo, KalicoHostIoConfig};
use kalico_host_rt::host_io::runtime_events::{RuntimeEvent, StatusEvent};
use kalico_host_rt::producer::{self, DEFAULT_LOAD_CURVE_TIMEOUT};
use kalico_host_rt::transport::Transport;
use trajectory::{AxisShaper, RequiredShaper, ShapedSegment, ShaperConfig};

use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::config::{PlannerConfig, PlannerLimits};
use motion_bridge_native::dispatch::{
    build_push_params, AXIS_X, AXIS_Y, McuAxisConfig, McuCaps,
};
use motion_bridge_native::planner::{DispatchError, PlannerHandle};
use motion_bridge_native::slot_pool::{SlotPool, CURVE_POOL_N};

// ---------------------------------------------------------------------------
// Config constants — mirror the live Trident config (smaller subset; the sim
// is a single-MCU H723 not the dual-MCU live topology).
// ---------------------------------------------------------------------------

/// Sim firmware's `CONFIG_CLOCK_FREQ` (matches `tools/sim/sim.config`).
const SIM_CLOCK_FREQ: u32 = 520_000_000;
/// Renode `CreateServerSocketTerminal` port. Documented in `h723_sim.resc`.
const SIM_TCP_PORT: u16 = 3334;
const SIM_TCP_ADDR: &str = "127.0.0.1:3334";

/// Mock MCU handle for the bridge's clock-sync state. The bridge talks to
/// klippy in terms of opaque u32 handles; in this test we only have one
/// MCU so a single sentinel value suffices.
const MCU_ID_SIM: u32 = 1;

/// Live limits used on the user's Trident — kept tighter than the dispatch
/// closure's wedge guards so we observe the bug, not its workarounds.
fn live_planner_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.limits = PlannerLimits {
        max_velocity: 1000.0,
        max_accel: 70_000.0,
        max_z_velocity: 5.0,
        max_z_accel: 100.0,
        square_corner_velocity: 5.0,
    };
    c.shaper = ShaperConfig {
        x: RequiredShaper::SmoothMzv { frequency_hz: 186.0 },
        y: RequiredShaper::SmoothMzv { frequency_hz: 122.0 },
        z: AxisShaper::Passthrough,
    };
    // Match the bench harness — looser than the production 5 µm because the
    // CoreXY collinear-cubic refit edge cases hit the cap on short moves.
    c.fit_tolerance_mm = 0.05;
    c
}

// ---------------------------------------------------------------------------
// SimProcess — RAII wrapper around the Renode subprocess. Kills the process
// on drop so a panicking test never leaks the sim.
// ---------------------------------------------------------------------------

struct SimProcess {
    child: Option<Child>,
    log_path: PathBuf,
}

impl SimProcess {
    fn spawn() -> Result<Self, String> {
        let repo_root = repo_root();
        let elf = repo_root.join("out").join("klipper.elf");
        if !elf.exists() {
            return Err(format!(
                "missing {elf:?}. Build the sim firmware first: \
                 `bash tools/sim/build_sim_firmware.sh`",
            ));
        }

        // Kill any leftover sim from a prior aborted run. Best-effort — pkill
        // returns non-zero if nothing matched.
        let _ = Command::new("pkill")
            .args(["-f", "renode"])
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status();
        // Give the OS a moment to release port 3334 if pkill killed something.
        std::thread::sleep(Duration::from_millis(500));

        let log_dir = std::env::temp_dir().join("kalico-sim-motion-jogs");
        std::fs::create_dir_all(&log_dir).map_err(|e| format!("mkdir log dir: {e}"))?;
        let log_path = log_dir.join(format!(
            "renode-{}.log",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_secs())
                .unwrap_or(0),
        ));
        let log_file = std::fs::File::create(&log_path)
            .map_err(|e| format!("create renode log: {e}"))?;
        let log_file2 = log_file
            .try_clone()
            .map_err(|e| format!("clone renode log fd: {e}"))?;

        let script = repo_root.join("tools").join("sim").join("run_sim.sh");
        let child = Command::new("bash")
            .arg(&script)
            .current_dir(&repo_root)
            // Renode's --console mode reads stdin; if we don't pipe /dev/null
            // it inherits the cargo-test stdin and exits on EOF.
            .stdin(Stdio::null())
            .stdout(Stdio::from(log_file))
            .stderr(Stdio::from(log_file2))
            .spawn()
            .map_err(|e| format!("spawn renode: {e}"))?;

        eprintln!(
            "[sim] spawned Renode (pid={}) — log: {}",
            child.id(),
            log_path.display(),
        );

        Ok(SimProcess { child: Some(child), log_path })
    }

    fn wait_for_tcp_ready(&self, timeout: Duration) -> Result<(), String> {
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if std::net::TcpStream::connect_timeout(
                &std::net::SocketAddr::new("127.0.0.1".parse().unwrap(), SIM_TCP_PORT),
                Duration::from_millis(500),
            ).is_ok() {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(250));
        }
        Err(format!(
            "Renode TCP port {SIM_TCP_PORT} did not accept connections within {:?}; \
             see {}",
            timeout, self.log_path.display(),
        ))
    }
}

impl Drop for SimProcess {
    fn drop(&mut self) {
        if let Some(mut child) = self.child.take() {
            let pid = child.id();
            // Try graceful shutdown first.
            let _ = child.kill();
            let _ = child.wait();
            // Belt-and-braces — Renode sometimes forks helper threads that
            // hold port 3334 even after the parent dies.
            let _ = Command::new("pkill")
                .args(["-f", "renode"])
                .stdout(Stdio::null())
                .stderr(Stdio::null())
                .status();
            eprintln!("[sim] cleaned up Renode (pid={pid})");
        }
    }
}

fn repo_root() -> PathBuf {
    // CARGO_MANIFEST_DIR for motion-bridge points to .../rust/motion-bridge.
    // Repo root is two levels up.
    let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    p.pop(); // .../rust
    p.pop(); // repo root
    p
}

// ---------------------------------------------------------------------------
// SimHarness — owns the KalicoHostIo + clock-sync thread + status-event
// subscription, mirroring what `PyMotionBridge::attach_serial` sets up in
// production.
// ---------------------------------------------------------------------------

struct SimHarness {
    _sim: SimProcess,
    host_io: Arc<KalicoHostIo>,
    /// Latest StatusEvent observed by the background watcher thread. The
    /// reactor publishes a new event each time `kalico_status_v6` arrives
    /// (~10 Hz).
    status: Arc<Mutex<StatusEvent>>,
    /// Clock-sync state — same regression as `spawn_periodic_clock_sync`,
    /// kept here so the dispatch closure can call `mcu_time_at_host` for
    /// segment-clock computation without a `PassthroughRouter`.
    clock_sync: Arc<Mutex<ClockSyncEstimator>>,
    /// Number of successful `runtime_clock_sync_request` round-trips. The
    /// dispatch closure waits for this to reach ≥2 (so the regression has at
    /// least two anchor points and `mcu_time_at_host` is meaningful).
    clock_sync_samples: Arc<AtomicU64>,
    clock_sync_stop: Arc<std::sync::atomic::AtomicBool>,
    clock_sync_handle: Option<thread::JoinHandle<()>>,
    /// Snapshot of the most recently observed engine `current_segment_id`.
    /// Tests poll this to assert "segment id advanced past the dispatched
    /// count" (i.e. the MCU retired the work, not just received it).
    last_seg_id: Arc<AtomicU64>,
    /// Per-tag maximum value observed in `StatusEvent::fault_detail`'s low
    /// 24 bits. The runtime_tick.c rotation publishes a different tag in
    /// the high byte each status frame (~14s wall to cycle through all
    /// 23 inner phases). Tests can read this to diagnose WHERE the
    /// pipeline broke without reading per-frame raw values.
    ///
    /// Key = high-byte tag (0xB0..0xEF). Value = max-seen low-24-bit
    /// payload. Tag encoding documented inline in
    /// `src/runtime_tick.c::runtime_tick`.
    fault_detail_by_tag: Arc<Mutex<HashMap<u8, u32>>>,
}

impl SimHarness {
    fn new() -> Result<Self, String> {
        let sim = SimProcess::spawn()?;
        sim.wait_for_tcp_ready(Duration::from_secs(15))?;

        // Give the firmware ~2s after TCP is up to finish its boot path
        // (sched_main → command_task_init → kalico_runtime_init → enable
        // periodic emits). The Klipper-protocol identify request will land
        // mid-boot if we issue it immediately; the firmware then NAKs
        // every retry until its command table is registered, which is
        // exactly the wedge we saw at offset=3720 in early test runs.
        thread::sleep(Duration::from_secs(2));

        // Match production identify/timeouts — these are tight but the sim
        // boots fast on a modern host. Extend the identify timeout because
        // Renode's quantum=1µs simulation is ~5x slower than wall-clock.
        let mut config = KalicoHostIoConfig::default();
        config.identify_timeout = Duration::from_secs(120);

        let host_io = KalicoHostIo::open_tcp(SIM_TCP_ADDR, config)
            .map_err(|e| format!("KalicoHostIo::open_tcp: {e:?}"))?;
        eprintln!("[sim] identify handshake ok");

        // Bootstrap the kalico-native control channel (matches the bridge's
        // `attach_serial` path — see `bridge.rs` ~L745).
        let identify_outcome = host_io
            .kalico_identify(Duration::from_secs(10))
            .map_err(|e| format!("kalico_identify: {e:?}"))?;
        eprintln!(
            "[sim] kalico-native identified — reset_epoch=0x{:08x}",
            identify_outcome.reset_epoch,
        );

        let host_io = Arc::new(host_io);

        // Subscribe to runtime events before any motion is dispatched so we
        // don't miss the first status emit.
        let runtime_rx = host_io
            .take_runtime_event_subscription()
            .map_err(|e| format!("take_runtime_event_subscription: {e:?}"))?;
        let status_shared = Arc::new(Mutex::new(StatusEvent::default()));
        let last_seg_id = Arc::new(AtomicU64::new(0));
        let fault_detail_by_tag: Arc<Mutex<HashMap<u8, u32>>> =
            Arc::new(Mutex::new(HashMap::new()));
        {
            let status_shared = Arc::clone(&status_shared);
            let last_seg_id = Arc::clone(&last_seg_id);
            let fault_detail_by_tag = Arc::clone(&fault_detail_by_tag);
            thread::Builder::new()
                .name("sim-status-watcher".to_string())
                .spawn(move || {
                    while let Ok(evt) = runtime_rx.recv() {
                        match &evt {
                            RuntimeEvent::Status(s) => {
                                last_seg_id
                                    .store(u64::from(s.current_segment_id), Ordering::Release);
                                *status_shared.lock().unwrap() = s.clone();
                                // Decompose fault_detail into (tag, payload)
                                // and track max-seen payload per tag — this
                                // is how tests inspect the rotation
                                // (0xB0..0xEF) without polling.
                                if s.fault_detail != 0 {
                                    let tag = (s.fault_detail >> 24) as u8;
                                    let payload = s.fault_detail & 0x00FF_FFFF;
                                    let mut map = fault_detail_by_tag.lock().unwrap();
                                    let entry = map.entry(tag).or_insert(0);
                                    if payload > *entry {
                                        *entry = payload;
                                    }
                                }
                                eprintln!(
                                    "[sim-status] engine_status={} seg_id={} last_fault={} fault_detail=0x{:08x}",
                                    s.engine_status, s.current_segment_id, s.last_fault, s.fault_detail,
                                );
                            }
                            RuntimeEvent::Fault(f) => {
                                eprintln!(
                                    "[sim-fault] code={} detail=0x{:08x} seg_id={} synth={}",
                                    f.fault_code, f.fault_detail, f.segment_id, f.synthesized,
                                );
                            }
                            RuntimeEvent::CreditFreed(c) => {
                                eprintln!(
                                    "[sim-credit] retired_through={} free_slots={}",
                                    c.retired_through_segment_id, c.free_slots,
                                );
                            }
                            RuntimeEvent::UnknownOutput { msg, .. } => {
                                eprintln!("[sim-output] {msg}");
                            }
                            _ => {}
                        }
                    }
                })
                .expect("spawn status watcher");
        }

        // Clock-sync thread — identical structure to bridge.rs's
        // `spawn_periodic_clock_sync`, just inlined.
        let clock_sync = Arc::new(Mutex::new(
            ClockSyncEstimator::new(f64::from(SIM_CLOCK_FREQ)),
        ));
        let clock_sync_stop = Arc::new(std::sync::atomic::AtomicBool::new(false));
        let clock_sync_samples = Arc::new(AtomicU64::new(0));
        let clock_sync_handle = {
            let host_io_for_cs = Arc::clone(&host_io);
            let clock_sync = Arc::clone(&clock_sync);
            let stop = Arc::clone(&clock_sync_stop);
            let samples_counter = Arc::clone(&clock_sync_samples);
            thread::Builder::new()
                .name("sim-clock-sync".to_string())
                .spawn(move || {
                    // Mirror `spawn_periodic_clock_sync`'s 200 ms startup grace.
                    thread::sleep(Duration::from_millis(200));
                    while !stop.load(Ordering::Relaxed) {
                        let request_id = clock_sync
                            .lock()
                            .unwrap()
                            .next_clock_sync_request_id();
                        let host_send = Instant::now();
                        let cmd = format!(
                            "runtime_clock_sync_request request_id={request_id} \
                             host_send_time_lo=0 host_send_time_hi=0"
                        );
                        match host_io_for_cs.call(
                            &cmd,
                            "kalico_clock_sync_response",
                            Duration::from_millis(2000),
                        ) {
                            Ok(resp) => {
                                let host_recv = Instant::now();
                                let echoed = resp.try_get_u32("request_id").unwrap_or(0);
                                if echoed == request_id {
                                    let lo = resp.try_get_u32("mcu_clock_lo").unwrap_or(0);
                                    let hi = resp.try_get_u32("mcu_clock_hi").unwrap_or(0);
                                    let mcu_at_response =
                                        (u64::from(hi) << 32) | u64::from(lo);
                                    clock_sync.lock().unwrap().add_dedicated_sample(
                                        host_send,
                                        host_recv,
                                        mcu_at_response,
                                    );
                                    let prev = samples_counter
                                        .fetch_add(1, Ordering::Relaxed);
                                    eprintln!(
                                        "[sim-clock-sync] sample #{} mcu_at_response={mcu_at_response} rtt_ms={:.2}",
                                        prev + 1,
                                        host_recv.saturating_duration_since(host_send).as_secs_f64() * 1000.0,
                                    );
                                }
                            }
                            Err(e) => {
                                eprintln!("[sim-clock-sync] request failed: {e:?}");
                            }
                        }
                        thread::sleep(Duration::from_millis(250));
                    }
                })
                .expect("spawn clock-sync")
        };

        let harness = SimHarness {
            _sim: sim,
            host_io,
            status: status_shared,
            clock_sync,
            clock_sync_samples,
            clock_sync_stop,
            clock_sync_handle: Some(clock_sync_handle),
            last_seg_id,
            fault_detail_by_tag,
        };

        // Block until clock-sync has produced at least two regression samples
        // (enough for `mcu_time_at_host`'s regression to land on the real
        // line rather than the construction-time anchor `(0, 0)`).
        // Renode + the H7 sim is slow on first-startup — give it 30 s wall.
        harness.wait_for_clock_sync(2, Duration::from_secs(30))?;
        Ok(harness)
    }

    fn wait_for_clock_sync(&self, min_samples: u64, timeout: Duration) -> Result<(), String> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            if self.clock_sync_samples.load(Ordering::Relaxed) >= min_samples {
                eprintln!(
                    "[sim] clock-sync ready: {} samples (waited {:?})",
                    self.clock_sync_samples.load(Ordering::Relaxed),
                    start.elapsed(),
                );
                return Ok(());
            }
            thread::sleep(Duration::from_millis(100));
        }
        Err(format!(
            "clock-sync did not collect {min_samples} samples within {timeout:?} \
             (got {}). Last status: {:?}",
            self.clock_sync_samples.load(Ordering::Relaxed),
            self.status.lock().unwrap(),
        ))
    }

    /// Snapshot of the last StatusEvent observed.
    fn status(&self) -> StatusEvent {
        self.status.lock().unwrap().clone()
    }

    fn last_segment_id(&self) -> u32 {
        self.last_seg_id.load(Ordering::Acquire) as u32
    }

    /// Return the maximum payload (low 24 bits of `fault_detail`) observed
    /// for the given tag (high byte). Returns `None` if the tag has never
    /// appeared in a status frame.
    ///
    /// Common tags (see `src/runtime_tick.c` for full encoding):
    ///   - `0xB2` low 16 bits = 4-bit ring_high_water per motor (>0 means
    ///     producer pushed at least one step for that motor).
    ///   - `0xB3` bits 16..23 = `producer_steps_pushed_total & 0xFF`.
    ///   - `0xE1` low 24 bits = `runtime_emit_calls`.
    ///   - `0xE2` bits 16..23 = `runtime_emit_pulses & 0xFF`.
    ///   - `0xB8` low 8 bits = `producer_primary_resolved_total & 0xFF`.
    fn fault_detail_max(&self, tag: u8) -> Option<u32> {
        self.fault_detail_by_tag
            .lock()
            .unwrap()
            .get(&tag)
            .copied()
    }

    /// Snapshot every observed tag/payload pair, sorted by tag for stable
    /// diagnostic output.
    fn fault_detail_summary(&self) -> Vec<(u8, u32)> {
        let map = self.fault_detail_by_tag.lock().unwrap();
        let mut v: Vec<(u8, u32)> = map.iter().map(|(k, v)| (*k, *v)).collect();
        v.sort_by_key(|(k, _)| *k);
        v
    }

    /// Block until the MCU's `current_segment_id` reaches at least `target`,
    /// or `timeout` elapses. Returns `Ok(())` on hit, `Err(observed)` on
    /// timeout.
    fn wait_for_segment_id(&self, target: u32, timeout: Duration) -> Result<(), u32> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            let cur = self.last_segment_id();
            if cur >= target {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(20));
        }
        Err(self.last_segment_id())
    }
}

impl Drop for SimHarness {
    fn drop(&mut self) {
        self.clock_sync_stop.store(true, Ordering::Release);
        if let Some(h) = self.clock_sync_handle.take() {
            let _ = h.join();
        }
        // host_io is Arc'd; the dispatch closure may still hold a clone if
        // the test panicked mid-flush. Best-effort: when the last Arc drops,
        // `KalicoHostIo::drop` shuts the reactor. Nothing more to do here.
    }
}

// ---------------------------------------------------------------------------
// Dispatch closure — mirrors the bridge's bridge.rs::init_planner closure
// (~L1407–L1708), simplified for the single-MCU sim topology.
//
// Per ShapedSegment:
//   1. `build_push_params` for the (single) MCU.
//   2. Allocate one slot per non-trivial axis curve from the SlotPool.
//   3. `producer::load_curve(host_io, slot, ...)` for each axis.
//   4. Convert seg.t_start/t_end seconds to MCU clocks via clock-sync's
//      `mcu_time_at_host`. Same +250 ms lead the production dispatch applies
//      so the firmware doesn't see "already in the past" segments.
//   5. `producer::push_segment(host_io, credit, &params)`.
//
// Errors propagate back to the planner via `Err(String)` — that's exactly the
// same error channel the bridge uses; the planner thread surfaces it as
// `PlannerError::Dispatch`.
// ---------------------------------------------------------------------------

#[derive(Default)]
struct DispatchStats {
    segments_seen: AtomicU64,
    last_error: Mutex<Option<String>>,
}

fn build_dispatch(
    host_io: Arc<KalicoHostIo>,
    credit: Arc<CreditCounter>,
    slot_pool: Arc<Mutex<SlotPool>>,
    clock_sync: Arc<Mutex<ClockSyncEstimator>>,
    clock_sync_samples: Arc<AtomicU64>,
    mcu_axis_configs: Vec<McuAxisConfig>,
    stats: Arc<DispatchStats>,
) -> Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> {
    let next_seg_id = Arc::new(Mutex::new(1_u32));
    Arc::new(move |seg: &ShapedSegment| -> Result<(), DispatchError> {
        // Build per-MCU plans. `t_start/t_end` get overwritten below.
        let mut plans = build_push_params(seg, &mcu_axis_configs, 0, 0);
        if plans.is_empty() {
            // All axes trivially constant — bridge ignores these too.
            stats.segments_seen.fetch_add(1, Ordering::Relaxed);
            return Ok(());
        }

        for mut plan in plans.drain(..) {
            // Block until clock-sync has at least 2 samples in the regression
            // window. Mirrors `bridge.rs`'s `while compute_ack_clock <= 0`
            // wedge guard added in commit 365087203 ("block dispatch until
            // clock-sync establishes"). Without this, `mcu_time_at_host`
            // returns a value projected from the construction-time anchor
            // `(0, 0)` — i.e. a value far in the future — and the segment
            // lands beyond the firmware's wrap-around horizon.
            let cs = clock_sync.clone();
            let mcu_base_clock = {
                let wait_start = Instant::now();
                loop {
                    if clock_sync_samples.load(Ordering::Relaxed) >= 2 {
                        break;
                    }
                    if wait_start.elapsed() > Duration::from_secs(10) {
                        let msg = format!(
                            "clock-sync hadn't established (≥2 samples) after \
                             10s — got {} for mcu {}",
                            clock_sync_samples.load(Ordering::Relaxed),
                            plan.mcu_id,
                        );
                        *stats.last_error.lock().unwrap() = Some(msg.clone());
                        return Err(DispatchError::ComputeAckClock(msg));
                    }
                    thread::sleep(Duration::from_millis(20));
                }
                let cs_lock = cs.lock().unwrap();
                let host_time = cs_lock.host_time_at(Instant::now());
                let now_clock = cs_lock.mcu_time_at_host(host_time);
                drop(cs_lock);
                // 250 ms lead — matches the production lead in bridge.rs.
                let lead_cycles = (f64::from(SIM_CLOCK_FREQ) * 0.250) as u64;
                now_clock.saturating_add(lead_cycles)
            };

            // Map seg.t_start/t_end (absolute trajectory seconds) onto MCU
            // clocks relative to mcu_base_clock. This is what
            // `host_time_to_mcu_clock` does on the router side, just without
            // routing through a `PassthroughRouter` mutex.
            let rel_start_cycles =
                (seg.t_start * f64::from(SIM_CLOCK_FREQ)).max(0.0) as u64;
            let rel_end_cycles =
                (seg.t_end * f64::from(SIM_CLOCK_FREQ)).max(0.0) as u64;
            plan.params.t_start = mcu_base_clock.saturating_add(rel_start_cycles);
            plan.params.t_end = mcu_base_clock.saturating_add(rel_end_cycles);

            // Allocate a per-MCU rolling segment id.
            {
                let mut id = next_seg_id.lock().unwrap();
                plan.params.id = *id;
                *id = id.wrapping_add(1);
            }

            eprintln!(
                "[planner-trace] dispatching seg id={} mcu={} t_start={} t_end={} (rel {:.6}–{:.6} s) curves_to_load.len={}",
                plan.params.id, plan.mcu_id, plan.params.t_start, plan.params.t_end,
                seg.t_start, seg.t_end, plan.curves_to_load.len(),
            );
            for (i, (axis_idx, cp)) in plan.curves_to_load.iter().enumerate() {
                eprintln!(
                    "[planner-trace]   curve[{}]: axis={} n_cps={} n_knots={} body_estimate={}",
                    i, axis_idx, cp.cps_f32.len(), cp.knots_f32.len(),
                    11 + 4 * (cp.cps_f32.len() + cp.knots_f32.len()),
                );
            }

            // Load each axis curve, then push the segment. Mirror bridge.rs's
            // partial-failure slot release.
            let mut allocated_slots: Vec<u16> = Vec::with_capacity(plan.curves_to_load.len());
            let mut load_err: Option<DispatchError> = None;
            eprintln!("[planner-trace] seg={} entering load loop, n_curves={}",
                plan.params.id, plan.curves_to_load.len());
            for i in 0..plan.curves_to_load.len() {
                let axis_idx = plan.curves_to_load[i].0;
                let curve_params = plan.curves_to_load[i].1.clone();
                eprintln!("[planner-trace] seg={} curve_iter={} axis={} start", plan.params.id, i, axis_idx);
                let alloc = {
                    let mut pool = slot_pool.lock().unwrap();
                    let cap = pool.capacity();
                    let in_flight = pool.in_flight_count();
                    pool.try_alloc()
                        .ok_or(DispatchError::SlotPoolExhausted {
                            mcu_id: plan.mcu_id,
                            capacity: cap,
                            in_flight,
                        })
                };
                let (slot, slot_gen) = match alloc {
                    Ok(v) => v,
                    Err(e) => {
                        eprintln!("[planner-trace] seg={} curve_iter={} alloc FAILED: {:?}", plan.params.id, i, e);
                        load_err = Some(e);
                        break;
                    }
                };
                eprintln!("[planner-trace] seg={} curve_iter={} alloc OK slot={} gen={}", plan.params.id, i, slot, slot_gen);
                allocated_slots.push(slot);
                match producer::load_curve(
                    host_io.as_ref(),
                    slot,
                    &curve_params,
                    DEFAULT_LOAD_CURVE_TIMEOUT,
                ) {
                    Ok(handle) => {
                        eprintln!("[planner-trace] seg={} curve_iter={} load_curve OK handle=0x{:08x}", plan.params.id, i, handle);
                        plan.set_handle(axis_idx, handle);
                    }
                    Err(e) => {
                        eprintln!("[planner-trace] seg={} curve_iter={} load_curve ERR: {:?}", plan.params.id, i, e);
                        load_err = Some(DispatchError::LoadCurve {
                            mcu_id: plan.mcu_id,
                            slot,
                            seg_id: plan.params.id,
                            axis: axis_idx,
                            host_gen: slot_gen,
                            detail: format!("{e:?}"),
                        });
                        break;
                    }
                }
            }
            eprintln!("[planner-trace] seg={} exited load loop, load_err={:?}", plan.params.id, load_err.as_ref().map(|e| format!("{:?}", e)));

            if let Some(err) = load_err {
                let mut pool = slot_pool.lock().unwrap();
                for s in &allocated_slots {
                    pool.release(*s);
                }
                *stats.last_error.lock().unwrap() = Some(err.to_string());
                return Err(err);
            }

            {
                let mut pool = slot_pool.lock().unwrap();
                for s in &allocated_slots {
                    pool.register_segment(*s, plan.params.id);
                }
            }

            match producer::push_segment_with_timeout(
                host_io.as_ref(),
                credit.as_ref(),
                &plan.params,
                producer::DEFAULT_PUSH_RESPONSE_TIMEOUT,
            ) {
                Ok(info) => {
                    eprintln!(
                        "[bridge-trace] push_segment ok: mcu={} sent_id={} accepted_id={} epoch={}",
                        plan.mcu_id, plan.params.id, info.accepted_segment_id, info.credit_epoch,
                    );
                }
                Err(e) => {
                    let mut pool = slot_pool.lock().unwrap();
                    for s in &allocated_slots {
                        pool.release(*s);
                    }
                    let err = DispatchError::PushSegment {
                        mcu_id: plan.mcu_id,
                        detail: format!("{e:?}"),
                    };
                    *stats.last_error.lock().unwrap() = Some(err.to_string());
                    return Err(err);
                }
            }
        }

        stats.segments_seen.fetch_add(1, Ordering::Relaxed);
        Ok(())
    })
}

// ---------------------------------------------------------------------------
// Test harness boilerplate — build planner + dispatch + return everything the
// jog tests need.
// ---------------------------------------------------------------------------

struct PlannerCtx {
    harness: SimHarness,
    planner: PlannerHandle,
    stats: Arc<DispatchStats>,
}

impl PlannerCtx {
    fn build() -> Result<Self, String> {
        Self::build_with_spm([80.0_f32, 80.0_f32, 0.0_f32, 0.0_f32])
    }

    fn build_with_spm(steps_per_mm: [f32; 4]) -> Result<Self, String> {
        let harness = SimHarness::new()?;

        // Configure the firmware's kinematics + steps_per_mm. Default tests
        // use CoreXY @ 80 spm on A+B; the live-mirror test bumps that to
        // 160 spm to match the user's Trident. `configure_axes` is a
        // kalico-native control-channel call (not a runtime command).
        configure_sim_axes_with_spm(&harness.host_io, steps_per_mm)?;

        let host_io = Arc::clone(&harness.host_io);
        let credit = Arc::new(CreditCounter::new(1024));
        host_io.attach_credit_counter(Arc::clone(&credit));
        let slot_pool = Arc::new(Mutex::new(SlotPool::new(CURVE_POOL_N)));
        let clock_sync = Arc::clone(&harness.clock_sync);
        let clock_sync_samples = Arc::clone(&harness.clock_sync_samples);

        let mcu_axis_configs = vec![McuAxisConfig {
            mcu_id: MCU_ID_SIM,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: 0, // CoreXyAndE
            caps: McuCaps::default(),
        }];

        let stats = Arc::new(DispatchStats::default());
        let dispatch = build_dispatch(
            Arc::clone(&host_io),
            Arc::clone(&credit),
            Arc::clone(&slot_pool),
            Arc::clone(&clock_sync),
            clock_sync_samples,
            mcu_axis_configs,
            Arc::clone(&stats),
        );

        let planner = PlannerHandle::spawn(live_planner_config(), dispatch);

        // Send `kalico_stream_open` so subsequent submits go through the
        // streaming-state path (matches what the bridge does on klippy
        // connect — `set_position` → `kalico_stream_open`).
        planner
            .kalico_stream_open([0.0; 4])
            .map_err(|e| format!("kalico_stream_open: {e}"))?;

        Ok(PlannerCtx { harness, planner, stats })
    }

    fn submit_jog(
        &self,
        start_pos: [f64; 3],
        dx: f64,
        dy: f64,
        dz: f64,
        feedrate: f64,
    ) -> Result<(), String> {
        let m = classify_and_build(start_pos, dx, dy, dz, 0.0, feedrate)
            .map_err(|e| format!("classify_and_build: {e:?}"))?;
        self.planner
            .submit_move(m)
            .map_err(|e| format!("submit_move: {e}"))?;
        Ok(())
    }

    fn flush(&self) -> Result<(), String> {
        self.planner.flush().map_err(|e| format!("flush: {e}"))
    }

    fn dispatched_segments(&self) -> u64 {
        self.stats.segments_seen.load(Ordering::Relaxed)
    }

    fn last_dispatch_error(&self) -> Option<String> {
        self.stats.last_error.lock().unwrap().clone()
    }
}

/// Send `ConfigureAxes` to the sim — CoreXY with caller-supplied
/// per-axis steps_per_mm. Present mask hardcoded to motors 0+1 (A+B).
/// Default 25mm-jog tests pass `[80, 80, 0, 0]`; the live-mirror test
/// passes `[160, 160, 0, 0]` to match the user's Trident config.
fn configure_sim_axes_with_spm(
    host_io: &KalicoHostIo,
    steps_per_mm: [f32; 4],
) -> Result<(), String> {
    let kinematics = 0_u8; // CoreXyAndE
    let present_mask = 0b0011_u8; // A=motor0, B=motor1; Z/E absent
    let awd_mask = 0_u8;
    let invert_mask = 0_u8;

    let mut body = Vec::with_capacity(20);
    body.push(kinematics);
    body.push(present_mask);
    body.push(awd_mask);
    body.push(invert_mask);
    for v in &steps_per_mm {
        body.extend_from_slice(&v.to_le_bytes());
    }

    let (_kind, resp_body) = host_io
        .kalico_call(
            kalico_protocol::MessageKind::ConfigureAxes,
            body,
            Duration::from_secs(2),
        )
        .map_err(|e| format!("kalico_call ConfigureAxes: {e:?}"))?;
    if resp_body.len() < 4 {
        return Err(format!(
            "ConfigureAxes: short response body ({} bytes)",
            resp_body.len(),
        ));
    }
    let r = i32::from_le_bytes([resp_body[0], resp_body[1], resp_body[2], resp_body[3]]);
    if r != 0 {
        return Err(format!("ConfigureAxes returned error code {r}"));
    }
    eprintln!(
        "[sim] ConfigureAxes ok (CoreXY, steps_per_mm = [{:.1}, {:.1}, {:.1}, {:.1}])",
        steps_per_mm[0], steps_per_mm[1], steps_per_mm[2], steps_per_mm[3],
    );
    Ok(())
}

// ===========================================================================
//
// Test A — first 25 mm pure-X jog at F=100 immediately after stream_open.
//
// Live-bench symptom: ~50% of the time the first dispatched segment after a
// fresh klippy connect produces zero step pulses even though `segment_id`
// advances and status reaches Drained.
//
// Pass condition: planner emits ≥1 segment AND the MCU's StatusEvent
// `current_segment_id` advances to at least that count within a reasonable
// window. If `dispatched_segments > 0` but `last_segment_id == 0`, the
// firmware never accepted the dispatched curve frames — that's the bug.
//
// ===========================================================================

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn first_jog_after_stream_open_runs_on_sim() {
    let ctx = PlannerCtx::build().expect("build sim harness");

    ctx.submit_jog([0.0; 3], 25.0, 0.0, 0.0, 100.0)
        .expect("submit 25mm X");
    ctx.flush().expect("flush after first jog");

    let dispatched = ctx.dispatched_segments();
    eprintln!(
        "[test-A] dispatched_segments={} last_seg_id_observed={}",
        dispatched, ctx.harness.last_segment_id(),
    );

    // Allow the firmware up to 5 s real-time to process the dispatched curves.
    // At 250 ms lead + ~0.25 s expected trajectory + status-emit 10 Hz, we
    // expect to see `current_segment_id >= 1` well within this window.
    let wait_outcome = ctx
        .harness
        .wait_for_segment_id(dispatched.max(1) as u32, Duration::from_secs(5));

    let status = ctx.harness.status();
    eprintln!(
        "[test-A] final status: engine_status={} seg_id={} last_fault={} fault_detail=0x{:08x}",
        status.engine_status, status.current_segment_id, status.last_fault, status.fault_detail,
    );
    if let Some(err) = ctx.last_dispatch_error() {
        eprintln!("[test-A] dispatch error captured: {err}");
    }

    assert!(
        dispatched > 0,
        "BENCH BUG (planner-side): first 25mm jog dispatched zero segments. \
         Last dispatch error: {:?}",
        ctx.last_dispatch_error(),
    );

    match wait_outcome {
        Ok(()) => {
            eprintln!(
                "[test-A] PASS: segment_id reached {} (dispatched {})",
                ctx.harness.last_segment_id(),
                dispatched,
            );
        }
        Err(observed) => {
            panic!(
                "BENCH BUG #3 (first-jog no-motion): dispatched {} segments \
                 but MCU `current_segment_id` only reached {} after 5s. \
                 Final status = {:?}. Last dispatch error = {:?}. \
                 This is the wire-format symptom — segments accepted on the \
                 wire (push_segment OK) but engine never executed them.",
                dispatched, observed, status, ctx.last_dispatch_error(),
            );
        }
    }
}

// ===========================================================================
//
// Test B — 10 alternating ±25 mm jogs at ~1 s intervals.
//
// Live-bench symptom: "speed all over the place / huge delays" on subsequent
// jogs after the first one. Hypothesis: feedrate-ignored bug makes each jog
// take 0.138 s instead of 0.5 s, so klippy's lookahead gets confused; or
// widen-state drift accumulates on subsequent dispatches.
//
// Pass condition: all 10 jogs dispatch; MCU's `current_segment_id` advances
// to at least the dispatched count.
//
// ===========================================================================

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn ten_alternating_jogs_run_on_sim() {
    let ctx = PlannerCtx::build().expect("build sim harness");

    let n_jogs = 10_usize;
    let jog_mm = 25.0;
    let mut pos = [0.0_f64; 3];
    for i in 0..n_jogs {
        let dx = if i % 2 == 0 { jog_mm } else { -jog_mm };
        ctx.submit_jog(pos, dx, 0.0, 0.0, 100.0)
            .unwrap_or_else(|e| panic!("submit jog {i}: {e}"));
        pos[0] += dx;
        // Loose pacing — matches the user's manual button cadence on the
        // bench (1–3 s between presses). We don't flush per-jog; the planner
        // batches into shape_batch.
        thread::sleep(Duration::from_millis(1000));
    }
    ctx.flush().expect("final flush");

    let dispatched = ctx.dispatched_segments();
    let target = dispatched.max(1) as u32;
    eprintln!("[test-B] dispatched_segments={} target_seg_id={}", dispatched, target);

    let wait_outcome = ctx
        .harness
        .wait_for_segment_id(target, Duration::from_secs(30));
    let status = ctx.harness.status();
    eprintln!("[test-B] final status: {status:?}");
    if let Some(err) = ctx.last_dispatch_error() {
        eprintln!("[test-B] dispatch error captured: {err}");
    }

    assert!(
        dispatched as usize >= n_jogs,
        "BENCH BUG: 10 alternating jogs only produced {} dispatch events. \
         Last dispatch error: {:?}",
        dispatched,
        ctx.last_dispatch_error(),
    );

    if let Err(observed) = wait_outcome {
        panic!(
            "BENCH BUG #4 (subsequent jogs slow / no motion): dispatched {} \
             segments but MCU `current_segment_id` only reached {} after 30s. \
             Status = {:?}. Last dispatch error = {:?}",
            dispatched, observed, status, ctx.last_dispatch_error(),
        );
    }
}

// ===========================================================================
//
// Test C — rapid burst of 20 × 5 mm jogs with no pacing.
//
// Live-bench symptom: rapid presses sometimes trip
// `KALICO_ERR_STEP_BURST_EXCEEDED` or produce intermittent no-motion. We
// submit 20 jogs back-to-back (no inter-jog sleep) and check:
//
//   1. All 20 dispatched (planner didn't refuse mid-batch).
//   2. `current_segment_id` eventually catches up.
//   3. `last_fault` stays 0 throughout (no `kalico_fault` event for
//      step-burst or pool-exhaustion).
//
// ===========================================================================

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn rapid_short_jogs_burst_no_fault() {
    let ctx = PlannerCtx::build().expect("build sim harness");

    let n_jogs = 20_usize;
    let jog_mm = 5.0;
    let mut pos = [0.0_f64; 3];
    for i in 0..n_jogs {
        let dx = if i % 2 == 0 { jog_mm } else { -jog_mm };
        ctx.submit_jog(pos, dx, 0.0, 0.0, 100.0)
            .unwrap_or_else(|e| panic!("submit jog {i}: {e}"));
        pos[0] += dx;
    }
    ctx.flush().expect("flush rapid burst");

    let dispatched = ctx.dispatched_segments();
    let target = dispatched.max(1) as u32;
    eprintln!("[test-C] dispatched={} target_seg_id={}", dispatched, target);

    let wait_outcome = ctx
        .harness
        .wait_for_segment_id(target, Duration::from_secs(30));
    let status = ctx.harness.status();
    eprintln!("[test-C] final status: {status:?}");
    if let Some(err) = ctx.last_dispatch_error() {
        eprintln!("[test-C] dispatch error captured: {err}");
    }

    assert_eq!(
        status.last_fault, 0,
        "BENCH BUG #2 (step burst): MCU latched fault {} (detail=0x{:08x}, seg_id={}). \
         dispatched_segments={} last_dispatch_error={:?}",
        status.last_fault, status.fault_detail, status.current_segment_id,
        dispatched, ctx.last_dispatch_error(),
    );

    if let Err(observed) = wait_outcome {
        // Make the failure detail explicit: a no-fault stall with segments
        // dispatched but not retired is the "intermittent no-motion" bug.
        panic!(
            "BENCH BUG (rapid no-motion or stall): dispatched {} segments but \
             MCU `current_segment_id` only reached {} after 30s. last_fault=0, \
             status={:?}. dispatch error={:?}",
            dispatched, observed, status, ctx.last_dispatch_error(),
        );
    }

    eprintln!(
        "[test-C] PASS: 20 rapid jogs dispatched and retired without fault \
         (seg_id={})",
        ctx.harness.last_segment_id(),
    );
}

// ---------------------------------------------------------------------------
// Smoke test — confirm the harness can connect, identify, and observe the
// sim's idle StatusEvent stream without driving any motion. Useful to debug
// the sim infrastructure independently of the bridge dispatch closure.
// ---------------------------------------------------------------------------

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn sim_harness_boots_and_emits_status() {
    let harness = SimHarness::new().expect("build sim harness");
    // Wait 1.5 s — the firmware emits `kalico_status_v6` at 10 Hz, so we
    // should see ≥10 status events in that window.
    thread::sleep(Duration::from_millis(1500));
    let status = harness.status();
    eprintln!("[smoke] status after 1.5s idle: {status:?}");
    // Engine status is u8; default = 0 = Idle. We don't insist on a specific
    // value — just that the watcher saw at least one event, which is implicit
    // in `wait_for_clock_sync` having returned during construction.
    assert_eq!(
        status.last_fault, 0,
        "sim boot emitted fault {} (detail=0x{:08x}) before any motion was \
         submitted — firmware is unhealthy",
        status.last_fault, status.fault_detail,
    );
}

// ===========================================================================
//
// Test D — live Trident reproduction. CoreXY @ 160 steps/mm (vs the 80 spm
// other tests use), start position [125, 100, 10], 0.5 mm pure-X jog at
// F=600 mm/min.
//
// Live-bench signature: bridge dispatches the segment, push_segment is
// accepted, motors resolve their primary curves, but the producer's inner
// loop returns `SegmentExhausted` on every piece — `producer_steps_pushed_total`
// stays 0, `runtime_emit_pulses` stays 0, ring_high_water stays 0 across
// every motor. End result on the printer is "steppers energize but
// toolhead doesn't move."
//
// What this test catches that A/B/C don't:
//   * `25 mm at F=100` (1.67 mm/s) and `5 mm at F=100` (also 1.67 mm/s)
//     are long, slow moves. The live failure is a 0.5 mm move at 10 mm/s —
//     much shorter trajectory, much higher peak velocity, ~80 expected
//     steps per motor instead of thousands. If the producer's
//     SegmentExhausted bug is dependent on segment duration vs piece
//     count, A/B/C may quietly pass.
//   * The host-side bench reproducer
//     `live_corexy_jog_short_distance_with_real_handle_y_emits_step_pulses`
//     in `bench_repro.rs` produces step_counts=[79, 79, 0, 0] for these
//     exact inputs — so the bug must be on the MCU side, isolated to
//     either wire-level curve corruption OR MCU-runtime-state divergence.
//
// Pass condition:
//   1. Planner emits ≥1 dispatched segment.
//   2. `current_segment_id` reaches the dispatched count within 10 s.
//   3. `runtime_emit_pulses` (tag 0xE2, bits 16..23) > 0 — the firmware
//      actually emitted at least one STEP toggle. This is the assertion
//      that the live bench fails on; the prior tests only checked
//      retirement, which can advance without any pulses if the producer
//      exhausts every piece.
//
// If the test fails, the diagnostic summary dumps every observed
// `fault_detail` tag with its max-seen payload — that's enough to pin
// where the pipeline broke (handle routing? curve resolution? push
// path?) without re-flashing.
//
// ===========================================================================

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn live_jog_mirror_corexy_160spm_xonly_pushes_steps_on_sim() {
    let ctx = PlannerCtx::build_with_spm([160.0, 160.0, 0.0, 0.0])
        .expect("build sim harness");

    // Mirror the live test exactly: start at [125, 100, 10], jog +0.5mm X
    // at F=600 (10 mm/s). bench_repro.rs's host runtime test with these
    // exact inputs returns step_counts = [79, 79, 0, 0] (motor A + motor B,
    // CoreXY decomposition of a pure-X jog).
    ctx.submit_jog([125.0, 100.0, 10.0], 0.5, 0.0, 0.0, 600.0)
        .expect("submit 0.5mm X jog");
    ctx.flush().expect("flush after 0.5mm jog");

    let dispatched = ctx.dispatched_segments();
    eprintln!(
        "[test-D] dispatched_segments={} last_seg_id_observed={}",
        dispatched, ctx.harness.last_segment_id(),
    );

    assert!(
        dispatched > 0,
        "BENCH BUG (planner-side): 0.5mm X jog at F=600 dispatched zero \
         segments. Last dispatch error: {:?}",
        ctx.last_dispatch_error(),
    );

    // Wait for the MCU to retire the dispatched segment(s). Use a longer
    // window than tests A/B/C because we also need the fault_detail tag
    // rotation to cycle through 0xE2 (pulses) at least once. The rotation
    // period is ~14 s wall (23 inner phases × 6 outer × 100 ms emit, gated
    // on outer == 0), so 25 s gives us ~1.8 full rotations.
    let seg_wait = ctx
        .harness
        .wait_for_segment_id(dispatched.max(1) as u32, Duration::from_secs(25));

    // Continue observing fault_detail tags for a few more seconds even if
    // the segment id advanced quickly, so the diagnostic summary is
    // populated regardless of pass/fail.
    let observation_extra = if seg_wait.is_ok() {
        Duration::from_secs(20)
    } else {
        Duration::from_secs(5)
    };
    thread::sleep(observation_extra);

    let status = ctx.harness.status();
    let summary = ctx.harness.fault_detail_summary();
    eprintln!(
        "[test-D] final status: engine_status={} seg_id={} last_fault={} fault_detail=0x{:08x}",
        status.engine_status, status.current_segment_id, status.last_fault, status.fault_detail,
    );
    eprintln!("[test-D] fault_detail tag summary (tag → max payload):");
    for (tag, payload) in &summary {
        eprintln!("  0x{:02X} → 0x{:06X} ({})", tag, payload, payload);
    }
    if let Some(err) = ctx.last_dispatch_error() {
        eprintln!("[test-D] dispatch error captured: {err}");
    }

    if let Err(observed) = seg_wait {
        panic!(
            "BENCH BUG #3 (first-jog no-motion): dispatched {} segments \
             but MCU `current_segment_id` only reached {} after 25s. \
             Final status = {:?}. Tag summary = {:?}. Last dispatch \
             error = {:?}.",
            dispatched, observed, status, summary, ctx.last_dispatch_error(),
        );
    }

    // Tag 0xE2's bits 16..23 carry `runtime_emit_pulses & 0xFF`.
    // > 0 means the runtime called runtime_emit_step at least once,
    // which only happens once a step_time_event ISR popped a ring entry
    // the producer pushed. If this is zero, the producer exhausted every
    // piece on every motor without emitting a single step — the exact
    // live-bench signature.
    let e2 = ctx.harness.fault_detail_max(0xE2);
    let pulses_lo = e2.map(|p| (p >> 16) & 0xFF).unwrap_or(0);

    // Tag 0xB3 bits 16..23 carry `producer_steps_pushed_total & 0xFF`.
    let b3 = ctx.harness.fault_detail_max(0xB3);
    let pushed = b3.map(|p| (p >> 16) & 0xFF).unwrap_or(0);

    // Tag 0xB2 low 16 bits carry 4-bit ring_high_water for motors 0..3.
    let b2 = ctx.harness.fault_detail_max(0xB2);
    let ring_high_waters = b2.map(|p| {
        let hws = p & 0xFFFF;
        [
            (hws & 0xF) as u32,
            ((hws >> 4) & 0xF) as u32,
            ((hws >> 8) & 0xF) as u32,
            ((hws >> 12) & 0xF) as u32,
        ]
    });

    eprintln!(
        "[test-D] derived diagnostics: pulses_lo={} pushed={} \
         ring_high_water={:?}",
        pulses_lo, pushed, ring_high_waters,
    );

    assert!(
        pulses_lo > 0 || pushed > 0
            || ring_high_waters
                .map(|hws| hws.iter().any(|h| *h > 0))
                .unwrap_or(false),
        "LIVE BUG REPRODUCED IN SIM: 0.5mm X jog at F=600 (CoreXY 160 spm) \
         dispatched {} segment(s), `current_segment_id` reached {} (retirement \
         happened), BUT `runtime_emit_pulses = {}`, `producer_steps_pushed_total \
         = {}`, ring_high_water = {:?}. The producer exhausts every piece \
         without pushing a single step entry — same signature as the live \
         hardware bench. Tag summary: {:?}",
        dispatched, ctx.harness.last_segment_id(),
        pulses_lo, pushed, ring_high_waters, summary,
    );

    eprintln!(
        "[test-D] PASS: 0.5mm X jog dispatched {} segments, seg_id reached \
         {}, runtime_emit_pulses_lo = {}, producer_steps_pushed_lo = {}, \
         ring_high_water = {:?}.",
        dispatched, ctx.harness.last_segment_id(),
        pulses_lo, pushed, ring_high_waters,
    );
}

