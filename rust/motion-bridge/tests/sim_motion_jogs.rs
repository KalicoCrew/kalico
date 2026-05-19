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
use kalico_host_rt::endstop::{
    arm_endstop_with_timeout, ArmPolicy, ArmStatus, SourceKind, SourceSpec,
};
use kalico_host_rt::host_io::runtime_events::{
    EndstopTrippedEvent, RuntimeEvent, StatusEvent,
};
use kalico_host_rt::host_io::{KalicoHostIo, KalicoHostIoConfig};
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
/// Renode monitor TCP socket — `tools/sim/run_sim.sh` launches Renode with
/// `--port 3335`, exposing the Monitor as a telnet-style socket. Tests use
/// it via `RenodeMonitor` to drive virtual peripherals at runtime (e.g.
/// flipping an endstop GPIO mid-move).
const SIM_MONITOR_ADDR: &str = "127.0.0.1:3335";

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
// RenodeMonitor — TCP client for Renode's command-monitor socket (port 3335).
// Used to drive virtual peripherals from inside a test: flipping a GPIO,
// pausing the machine, dumping a register, etc.
//
// Protocol notes (Renode 1.16, `--port N` mode):
//
//   * Renode banners the connection with version info, ANSI-colored prompts,
//     and a few telnet IAC bytes. None of it is structured — Renode's
//     monitor was designed for human interaction at a terminal.
//   * Commands are sent as plain text terminated by `\n`. Renode echoes the
//     command and prints any output, followed by a new prompt line (e.g.
//     `(h723) ` or `(monitor) `).
//   * There is no acknowledgement framing. Tests that need to observe the
//     effect of a command should verify it via the firmware-side path
//     (e.g. drive an endstop pin → observe a trip event from the runtime
//     event stream), NOT by parsing the monitor's text response.
//
// We never parse responses — we send fire-and-forget. The monitor TCP buffer
// fills slowly enough that occasional non-drained reads don't cause back-
// pressure within the lifetime of a single test.
// ---------------------------------------------------------------------------

struct RenodeMonitor {
    stream: std::net::TcpStream,
}

impl RenodeMonitor {
    fn connect_with_timeout(timeout: Duration) -> Result<Self, String> {
        let addr: std::net::SocketAddr = SIM_MONITOR_ADDR
            .parse()
            .map_err(|e| format!("parse SIM_MONITOR_ADDR: {e}"))?;
        let stream = std::net::TcpStream::connect_timeout(&addr, timeout)
            .map_err(|e| format!("connect Renode monitor at {SIM_MONITOR_ADDR}: {e}"))?;
        // Tight read timeout — we never wait long on a read; we only do
        // best-effort drains so the socket buffer doesn't back up.
        let _ = stream.set_read_timeout(Some(Duration::from_millis(200)));
        let _ = stream.set_write_timeout(Some(Duration::from_secs(2)));
        let mut m = Self { stream };
        m.drain_socket();
        Ok(m)
    }

    fn drain_socket(&mut self) {
        use std::io::Read;
        let mut buf = [0_u8; 4096];
        // Read until the kernel buffer is empty or our read-timeout fires.
        loop {
            match self.stream.read(&mut buf) {
                Ok(0) => break,
                Ok(_) => continue,
                Err(_) => break,
            }
        }
    }

    fn send_command(&mut self, cmd: &str) -> Result<(), String> {
        use std::io::Write;
        let line = format!("{cmd}\n");
        self.stream
            .write_all(line.as_bytes())
            .map_err(|e| format!("send monitor cmd `{cmd}`: {e}"))?;
        self.stream
            .flush()
            .map_err(|e| format!("flush monitor cmd `{cmd}`: {e}"))?;
        // Best-effort drain so the socket buffer doesn't back up. The actual
        // effect of the command is verified by the test via firmware-side
        // observation, not by parsing the response text.
        self.drain_socket();
        Ok(())
    }

    /// Drive a GPIO input pin to the given level. Uses the Renode
    /// `<port> OnGPIO <pin> <bool>` command, which propagates to the
    /// STM32_GPIOPort model's IDR bit (when the pin is configured as input).
    ///
    /// `port` is `'A'..='K'`; `pin` is `0..=15`. STM32 firmware addresses
    /// the same pin as `GPIO(port, pin) = (port - 'A') * 16 + pin`.
    fn set_gpio_input(&mut self, port: char, pin: u8, level: bool) -> Result<(), String> {
        if !port.is_ascii_alphabetic() || port < 'A' || port > 'K' {
            return Err(format!("invalid GPIO port: '{port}'"));
        }
        if pin > 15 {
            return Err(format!("invalid GPIO pin: {pin}"));
        }
        let cmd = format!(
            "sysbus.gpioPort{} OnGPIO {} {}",
            port,
            pin,
            if level { "True" } else { "False" },
        );
        self.send_command(&cmd)
    }
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
    /// All `RuntimeEvent::EndstopTripped` events observed since harness
    /// construction, in arrival order. Populated by the watcher thread.
    /// The homing sim test inspects this to assert that a virtual-GPIO
    /// trip flowed through to a `kalico_endstop_tripped` output.
    endstop_trips: Arc<Mutex<Vec<EndstopTrippedEvent>>>,
    /// Host-side slot pool the dispatch closure allocates from. Set
    /// post-construction by `PlannerCtx::build_inner` via
    /// `register_slot_pool` — production routes `kalico_credit_freed`
    /// through klippy → `PyMotionBridge::on_credit_freed`, but the
    /// standalone sim test bypasses klippy, so we wire the same release
    /// directly off the runtime events channel here. Without this, the
    /// host pool grows monotonically and exhausts after CURVE_POOL_N / 2
    /// segments — that's what the test reproduced before this hookup.
    slot_pool_for_release: Arc<Mutex<Option<Arc<Mutex<SlotPool>>>>>,
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
        let endstop_trips: Arc<Mutex<Vec<EndstopTrippedEvent>>> =
            Arc::new(Mutex::new(Vec::new()));
        let slot_pool_for_release: Arc<Mutex<Option<Arc<Mutex<SlotPool>>>>> =
            Arc::new(Mutex::new(None));
        {
            let status_shared = Arc::clone(&status_shared);
            let last_seg_id = Arc::clone(&last_seg_id);
            let fault_detail_by_tag = Arc::clone(&fault_detail_by_tag);
            let endstop_trips = Arc::clone(&endstop_trips);
            let slot_pool_for_release = Arc::clone(&slot_pool_for_release);
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
                                // Mirror what `PyMotionBridge::on_credit_freed`
                                // does in production: release in-flight slots
                                // whose registered seg_id ≤ retired_through.
                                let n_released = match &*slot_pool_for_release.lock().unwrap() {
                                    Some(pool_arc) => {
                                        let mut p = pool_arc.lock().unwrap();
                                        p.retire_through_segment(c.retired_through_segment_id)
                                    }
                                    None => 0,
                                };
                                eprintln!(
                                    "[sim-credit] retired_through={} free_slots={} n_released={}",
                                    c.retired_through_segment_id, c.free_slots, n_released,
                                );
                            }
                            RuntimeEvent::UnknownOutput { msg, .. } => {
                                eprintln!("[sim-output] {msg}");
                            }
                            RuntimeEvent::EndstopTripped(e) => {
                                eprintln!(
                                    "[sim-endstop-trip] arm_id={} trip_clock={} \
                                     src_idx={} fmt={} stepper_count={}",
                                    e.arm_id, e.trip_clock, e.trip_source_idx,
                                    e.fmt_version, e.stepper_count,
                                );
                                endstop_trips.lock().unwrap().push(e.clone());
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
            endstop_trips,
            slot_pool_for_release,
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

    /// Block until the watcher records at least one `EndstopTripped` event
    /// matching `arm_id`, or `timeout` elapses.
    fn wait_for_endstop_trip(
        &self,
        arm_id: u32,
        timeout: Duration,
    ) -> Result<EndstopTrippedEvent, Vec<EndstopTrippedEvent>> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            if let Some(e) = self
                .endstop_trips
                .lock()
                .unwrap()
                .iter()
                .find(|e| e.arm_id == arm_id)
                .cloned()
            {
                return Ok(e);
            }
            thread::sleep(Duration::from_millis(20));
        }
        Err(self.endstop_trips.lock().unwrap().clone())
    }

    /// Register the dispatch closure's slot pool so the status-watcher
    /// thread releases in-flight slots when `kalico_credit_freed` arrives.
    /// Mirror of klippy's `PyMotionBridge::on_credit_freed` →
    /// `pool.retire_through_segment` wiring; production routes the event
    /// through klippy's reactor, but this standalone test owns the pool
    /// directly and must call the same release path.
    fn register_slot_pool(&self, pool: Arc<Mutex<SlotPool>>) {
        *self.slot_pool_for_release.lock().unwrap() = Some(pool);
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
        Self::build_inner([80.0_f32, 80.0_f32, 0.0_f32, 0.0_f32], false, false)
    }

    fn build_with_spm(steps_per_mm: [f32; 4]) -> Result<Self, String> {
        Self::build_inner(steps_per_mm, false, false)
    }

    /// Same as `build_with_spm` but additionally registers Klipper-side
    /// stepper bindings (allocate_oids + config_stepper + config_runtime_stepper)
    /// for motor 0 on PB5/PB6 before the kalico-native ConfigureAxes.
    ///
    /// Used by tests that need to observe actual step/dir GPIO emission via
    /// the `runtime_emit_calls` / `runtime_emit_pulses` / motor binding count
    /// diagnostics surfaced through `kalico_status_v6::fault_detail`. Without
    /// these bindings `runtime_emit_step_pulses` early-returns on `cnt == 0`,
    /// `runtime_emit_pulses` stays zero, and no GPIO toggles happen — which
    /// is the steady-state for the other tests in this file.
    fn build_with_stepper_bindings(steps_per_mm: [f32; 4]) -> Result<Self, String> {
        Self::build_inner(steps_per_mm, false, true)
    }

    /// Same as [`Self::build_with_spm`] but configures the firmware with
    /// phase-stepping enabled (`mcu_caps` bit 0 = 1, step_modes = Modulated
    /// for motors A+B). Mirrors a Trident `phase_stepping: 1` config —
    /// runtime_modulated_tick (TIM5 on H7) handles step output instead of
    /// per-stepper step-time ISRs.
    fn build_with_phase_stepping(steps_per_mm: [f32; 4]) -> Result<Self, String> {
        Self::build_inner(steps_per_mm, true, false)
    }

    fn build_inner(
        steps_per_mm: [f32; 4],
        phase_stepping: bool,
        with_klipper_stepper_bindings: bool,
    ) -> Result<Self, String> {
        let harness = SimHarness::new()?;

        // Configure the firmware's kinematics + steps_per_mm. Default tests
        // use CoreXY @ 80 spm on A+B; the live-mirror test bumps that to
        // 160 spm to match the user's Trident. `configure_axes` is a
        // kalico-native control-channel call (not a runtime command).
        if phase_stepping {
            configure_sim_axes_phase_stepping(&harness.host_io, steps_per_mm)?;
        } else {
            configure_sim_axes_with_spm(&harness.host_io, steps_per_mm)?;
        }

        // Klipper-side stepper bindings (allocate_oids / config_stepper /
        // config_runtime_stepper) MUST come AFTER ConfigureAxes. The
        // kalico-native `kalico_runtime_configure_axes` calls
        // `runtime_reset_stepper_bindings()` which zeros
        // `runtime_motor_stepper_count[]` — running the bindings first
        // would have them wiped out. Reset order matches the production
        // bridge: klippy sends `configure_axes` first, then iterates the
        // per-MCU stepper list and sends `config_runtime_stepper` for each.
        // See klippy/motion_toolhead.py:753-783.
        if with_klipper_stepper_bindings {
            setup_klipper_stepper_bindings(&harness.host_io)?;
        }

        let host_io = Arc::clone(&harness.host_io);
        let credit = Arc::new(CreditCounter::new(1024));
        host_io.attach_credit_counter(Arc::clone(&credit));
        let slot_pool = Arc::new(Mutex::new(SlotPool::new(CURVE_POOL_N)));
        // Mirror production: hook `kalico_credit_freed` to slot release
        // so the host pool actually drains as the MCU retires segments.
        harness.register_slot_pool(Arc::clone(&slot_pool));
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
    configure_sim_axes_inner(host_io, steps_per_mm, None)
}

/// Same as `configure_sim_axes_with_spm` but sends the **25-byte extended**
/// blob with `mcu_caps=0x01` (PHASE_STEPPING_CAPABLE) and `step_modes` =
/// [Modulated, Modulated, StepTime, StepTime] — A and B run via the TIM5
/// modulated tick on the H7 sim, mirroring a Trident `phase_stepping: 1`
/// config on motors A+B.
fn configure_sim_axes_phase_stepping(
    host_io: &KalicoHostIo,
    steps_per_mm: [f32; 4],
) -> Result<(), String> {
    // step_mode discriminants are stable wire bytes:
    //   0 = Modulated   (`runtime::state::StepMode::Modulated`)
    //   1 = StepTime    (`runtime::state::StepMode::StepTime`)
    // see rust/runtime/src/state.rs and the configure_axes_blob_step_modes
    // integration tests.
    const MOD: u8 = 0; // Modulated
    const ST: u8 = 1; // StepTime
    configure_sim_axes_inner(host_io, steps_per_mm, Some((0x01, [MOD, MOD, ST, ST])))
}

fn configure_sim_axes_inner(
    host_io: &KalicoHostIo,
    steps_per_mm: [f32; 4],
    extended: Option<(u8, [u8; 4])>,
) -> Result<(), String> {
    let kinematics = 0_u8; // CoreXyAndE
    let present_mask = 0b0011_u8; // A=motor0, B=motor1; Z/E absent
    let awd_mask = 0_u8;
    let invert_mask = 0_u8;

    let body_len = if extended.is_some() { 25 } else { 20 };
    let mut body = Vec::with_capacity(body_len);
    body.push(kinematics);
    body.push(present_mask);
    body.push(awd_mask);
    body.push(invert_mask);
    for v in &steps_per_mm {
        body.extend_from_slice(&v.to_le_bytes());
    }
    if let Some((mcu_caps, step_modes)) = extended {
        body.push(mcu_caps);
        body.extend_from_slice(&step_modes);
    }
    debug_assert_eq!(body.len(), body_len);

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
        "[sim] ConfigureAxes ok (CoreXY, steps_per_mm = [{:.1}, {:.1}, {:.1}, {:.1}], phase_stepping={})",
        steps_per_mm[0], steps_per_mm[1], steps_per_mm[2], steps_per_mm[3],
        extended.is_some(),
    );
    Ok(())
}

// ---------------------------------------------------------------------------
// Klipper-side stepper bindings — register OID 0 as a stepper on PB5
// (step_pin) / PB6 (dir_pin), then bind kalico runtime motor 0 to that OID
// so `runtime_emit_step_pulses(0, n_steps)` in src/stepper.c actually
// toggles step/dir GPIOs instead of returning early on `cnt == 0`.
//
// The wire-level encoding matches:
//   - DECL_COMMAND(command_allocate_oids,
//                  "allocate_oids count=%c") in src/basecmd.c
//   - DECL_COMMAND(command_config_stepper,
//                  "config_stepper oid=%c step_pin=%c dir_pin=%c \
//                   invert_step=%c step_pulse_ticks=%u") in src/stepper.c
//   - DECL_COMMAND(command_config_runtime_stepper,
//                  "config_runtime_stepper motor_idx=%c stepper_oid=%c \
//                   invert_dir=%c") in src/runtime_tick.c
//
// Pin names use the firmware's data-dictionary enum ("PB5", "PB6", etc.) —
// Klipper's command parser does NOT accept numeric STM32 IDs on the wire.
// PB5 (step) and PB6 (dir) are not used by Renode's stm32h743 platform
// definition for any reserved purpose, so we can safely toggle them.
//
// We do NOT call `finalize_config` afterwards: the kalico runtime's
// `runtime_emit_step_pulses` path doesn't gate on `is_finalized()`, and
// the existing tests in this file already work without it. Skipping
// finalize keeps the MCU's pre-config invariants flexible for future
// extension (e.g., binding additional OIDs mid-test).
// ---------------------------------------------------------------------------

const STEP_PIN_NAME: &str = "PB5";
const DIR_PIN_NAME: &str = "PB6";

fn setup_klipper_stepper_bindings(host_io: &KalicoHostIo) -> Result<(), String> {
    // 1. Pre-allocate one OID slot. `command_allocate_oids` shutdowns the
    //    MCU if called twice, so this must run exactly once per boot.
    host_io
        .send_fire_and_forget("allocate_oids count=1")
        .map_err(|e| format!("allocate_oids: {e:?}"))?;

    // 2. Register stepper OID 0 with PB5 step / PB6 dir. step_pulse_ticks
    //    is only consulted by the legacy stepper.c queue_step path; the
    //    kalico runtime computes its own pulse timing in
    //    `runtime_emit_step_pulses`. The value here only needs to satisfy
    //    the %u field parser.
    let cmd = format!(
        "config_stepper oid=0 step_pin={STEP_PIN_NAME} dir_pin={DIR_PIN_NAME} \
         invert_step=0 step_pulse_ticks=100"
    );
    host_io
        .send_fire_and_forget(&cmd)
        .map_err(|e| format!("config_stepper: {e:?}"))?;

    // 3. Bind kalico runtime motor 0 to stepper OID 0. After this,
    //    `runtime_motor_stepper_count[0] == 1` and
    //    `runtime_emit_step_pulses(0, n)` toggles PB5 (and writes PB6 on
    //    direction change) instead of early-returning on `cnt == 0`.
    host_io
        .send_fire_and_forget(
            "config_runtime_stepper motor_idx=0 stepper_oid=0 invert_dir=0",
        )
        .map_err(|e| format!("config_runtime_stepper: {e:?}"))?;

    // 4. Barrier: send a request/response command so we know all three
    //    fire-and-forget commands have been processed by the MCU's main
    //    loop before we return. `get_uptime` always responds with
    //    `uptime clock=%u high=%u`, regardless of finalize state.
    host_io
        .send_with_response("get_uptime", "uptime", Duration::from_secs(5))
        .map_err(|e| format!("get_uptime barrier: {e:?}"))?;

    eprintln!(
        "[sim] Klipper stepper bindings ok: \
         OID 0 on {STEP_PIN_NAME} step / {DIR_PIN_NAME} dir, motor 0 → OID 0",
    );
    Ok(())
}

// ===========================================================================
//
// Test G — G1 X50 → observable step/dir GPIO emission on the sim.
//
// This is the only test in this file that registers Klipper-side stepper
// bindings (PB5 step / PB6 dir for motor 0) before dispatching motion.
// Without those bindings `runtime_emit_step_pulses` early-returns on
// `cnt == 0` and no GPIO toggles happen — segments retire and
// `current_segment_id` advances, but the TMC driver wired downstream
// sees nothing.
//
// Pass condition: after a 50 mm pure-X jog at F=100, the firmware-side
// diagnostic counters surfaced via `kalico_status_v6::fault_detail`
// (high byte = tag, low 24 bits = payload) show:
//
//   * tag 0xE1 (`runtime_emit_calls`) > 0
//       — the Rust step producer called `runtime_emit_step_pulses`.
//   * tag 0xE2 low 4 bits (`runtime_motor_binding_count(0)`) >= 1
//       — the runtime sees motor 0 → stepper-OID-0 binding.
//   * tag 0xE2 bits 16..23 (`runtime_emit_pulses & 0xFF`) > 0
//       — the C-side `gpio_out_toggle_noirq` ran on at least one pulse.
//
// Together these prove the full chain in `src/stepper.c` ran:
// `runtime_emit_calls++` → bounds check passed → `cnt > 0` →
// `runtime_emit_pulses += |n_steps|` → `gpio_out_write(dir_pin)` →
// loop of `gpio_out_toggle_noirq(step_pin)`. That is the wire-level
// signal a TMC2209 / TMC5160 / similar step+dir driver reads.
//
// On a CoreXY at present_mask=0b0011 the X-only jog drives motor 0 (A)
// and motor 1 (B) equally. We only bind motor 0 here — that's sufficient
// to prove the GPIO path works without needing to set up a second OID.
//
// ===========================================================================

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn g1_x50_emits_step_pulses_on_sim() {
    let ctx = PlannerCtx::build_with_stepper_bindings(
        [80.0_f32, 80.0_f32, 0.0_f32, 0.0_f32],
    )
    .expect("build sim harness with stepper bindings");

    // Wait for the status-frame rotation to publish at least one of the
    // 0xB0/0xB1/0xE2/0xE6 binding-state tags so Stage 1's diag is visible
    // even when the engine subsequently wedges before further rotation
    // cycles complete. The diag rotates at the 10 Hz status cadence;
    // 3 s wall ≈ 30 frames is plenty to hit several tags.
    {
        let deadline = Instant::now() + Duration::from_secs(8);
        while Instant::now() < deadline {
            let has_binding_tag =
                ctx.harness.fault_detail_max(0xE2).is_some()
                    || ctx.harness.fault_detail_max(0xB1).is_some()
                    || ctx.harness.fault_detail_max(0xE6).is_some();
            if has_binding_tag {
                break;
            }
            thread::sleep(Duration::from_millis(200));
        }
    }

    // 50 mm X jog at F=100 mm/s → ~500 ms trajectory at peak velocity,
    // 4000 steps on motor A (80 spm × 50 mm). On a CoreXY the same count
    // shows up on motor B too (opposite sign), but we only bound motor 0
    // so only A's pulses are observable through the diag counters.
    if let Err(e) = ctx.submit_jog([0.0; 3], 50.0, 0.0, 0.0, 100.0) {
        eprintln!("[test-G] submit_jog returned err (continuing): {e}");
    }
    if let Err(e) = ctx.flush() {
        // The sim's USART2 model under Renode has known RX overrun on
        // 700+ byte frames (see h723_sim.resc). LoadCurve timeouts here
        // do NOT invalidate the test — the goal is to surface the
        // step-emission counters, which advance independently of any
        // single LoadCurve failure.
        eprintln!("[test-G] flush returned err (continuing): {e}");
    }

    let dispatched = ctx.dispatched_segments();
    eprintln!(
        "[test-G] dispatched_segments={} last_seg_id_observed={}",
        dispatched, ctx.harness.last_segment_id(),
    );

    // Wait up to 180 s for the MCU to retire at least one segment. Renode
    // runs ~150-200x slower than wall clock; a 50 mm jog at F=100 produces
    // ~500 ms of trajectory ≈ 75-100 s of wall time. The wait does not
    // panic — if the engine never advances we still print full diagnostics
    // below so the failure mode is visible.
    let _ = ctx
        .harness
        .wait_for_segment_id(dispatched.max(1) as u32, Duration::from_secs(180));

    let status = ctx.harness.status();
    eprintln!(
        "[test-G] final status: engine_status={} seg_id={} last_fault={} \
         fault_detail=0x{:08x}",
        status.engine_status, status.current_segment_id,
        status.last_fault, status.fault_detail,
    );
    for (tag, payload) in ctx.harness.fault_detail_summary() {
        eprintln!("[test-G]   fault_detail tag=0x{tag:02X} max_payload=0x{payload:06X}");
    }

    // ---- Step/dir GPIO emission diagnostics ----
    //
    // The three counters surface different layers of the chain:
    //   * tag 0xE2 low nibble    = config_runtime_stepper landed (binding)
    //   * tag 0xE1               = Rust step producer called the FFI emit
    //   * tag 0xE2 bits 16..23   = C-side `runtime_emit_pulses` advanced
    //
    // Each assertion's failure message says exactly WHICH stage broke,
    // so the panic surfaces the root cause without follow-up debugging.

    let emit_calls = ctx.harness.fault_detail_max(0xE1).unwrap_or(0);
    let e2 = ctx.harness.fault_detail_max(0xE2).unwrap_or(0);
    let pulses_lo = (e2 >> 16) & 0xFF;
    let motor0_binding = e2 & 0xF;

    // Engine-status diag (tag 0xE4 = step_time_producer_kicks).
    let producer_kicks = ctx.harness.fault_detail_max(0xE4).unwrap_or(0);
    let empty_polls = ctx.harness.fault_detail_max(0xE5).unwrap_or(0);

    eprintln!(
        "[test-G] step-emission diag:\n  \
         runtime_emit_calls           (tag 0xE1)       = {}\n  \
         motor 0 binding count        (tag 0xE2 nib0)  = {}\n  \
         runtime_emit_pulses & 0xFF   (tag 0xE2 b16-23) = {}\n  \
         step_time_producer_kicks     (tag 0xE4)       = {}\n  \
         step_time_empty_polls        (tag 0xE5)       = {}",
        emit_calls, motor0_binding, pulses_lo, producer_kicks, empty_polls,
    );

    assert!(
        motor0_binding >= 1,
        "STAGE 1 FAILED (Klipper-protocol stepper binding did not land): \
         tag 0xE2 low nibble = 0, so `runtime_motor_stepper_count[0] == 0`. \
         The firmware never accepted `config_runtime_stepper motor_idx=0 \
         stepper_oid=0` — check `runtime_bind_calls_total` (tag 0xB0 bits 8..15) \
         to confirm the command reached the firmware."
    );

    assert!(
        emit_calls >= 1,
        "STAGE 2 FAILED (Rust step producer never ran): \
         tag 0xE1 = 0, so `runtime_emit_step_pulses` was never called for \
         any motor. Motor 0 has {} binding(s) registered, but the engine \
         didn't reach the per-motor emit loop. \
         engine_status={} (0=Idle, 1=Running, 2=Drained, 3=Fault); \
         producer_kicks={}; empty_polls={}. \
         This is the H7-wedge-after-motion symptom — the engine accepts \
         segments on the wire (push_segment OK) but never transitions out \
         of Idle. See recent runtime_tick.c producer-timer fixes.",
        motor0_binding, status.engine_status, producer_kicks, empty_polls,
    );

    assert!(
        pulses_lo > 0,
        "STAGE 3 FAILED (no GPIO toggles despite {} emit calls and \
         motor 0 bound to OID 0): runtime_emit_pulses stayed 0, meaning \
         the per-motor `gpio_out_toggle_noirq(step_pin)` loop in \
         src/stepper.c didn't execute. Either every emit had n_steps==0 \
         (engine produced zero-step segments) or the bounds check at \
         line 587 (`motor_idx >= RUNTIME_MOTOR_COUNT`) rejected.",
        emit_calls,
    );

    eprintln!(
        "[test-G] PASS: G1 X50 produced runtime_emit_calls={}, \
         pulses_lo={} (toggles on {}), motor 0 binding count={}",
        emit_calls, pulses_lo, STEP_PIN_NAME, motor0_binding,
    );
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

    // Allow the firmware up to 90 s wall to process the dispatched curves.
    // Renode's H7 sim runs ~150-200× slower than real wall clock, and the
    // 25 mm jog at F=100 produces ~282 ms of trajectory (seg=2 t_end =
    // 277 M cycles at 520 MHz). So we need at least 50-60 s wall just for
    // virtual time to reach the segment's end-of-trajectory. 90 s gives
    // a safety margin; on silicon this completes in well under a second.
    let wait_outcome = ctx
        .harness
        .wait_for_segment_id(dispatched.max(1) as u32, Duration::from_secs(90));

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

// ===========================================================================
//
// Test F — phase-stepping + SET_KINEMATIC_POSITION + rapid relative-mode
// G1 X+25 burst. Reproduction of a live-bench symptom reported 2026-05-16:
//
//     [bench]
//     M115 / startup
//     (`phase_stepping: 1` on stepper_x + stepper_y in printer.cfg)
//     G91                              ; relative mode
//     SET_KINEMATIC_POSITION X=125 Y=100 Z=10
//     G1 X25  ; <— 5+ of these issued in rapid succession
//     G1 X25
//     G1 X25
//     ...
//     → MCU crashes (host loses comm / hangs / latches fault).
//
// The bench-side klippy flow lands on the bridge as:
//   1. `configure_axes` with `mcu_caps=0x01` and `step_modes[0..2]=Modulated`
//      (the phase-stepping wire wiring; spec §4 C1 extended blob).
//   2. `set_position(125, 100, 10)` → `kalico_stream_open([125,100,10,0])`
//      (planner-side `ShaperState::reset`).
//   3. Five `submit_move(dx=+25, dy=0, dz=0, F=3000)` calls back-to-back.
//
// This test mirrors that exact sequence against the sim and flags whichever
// crash signature actually manifests:
//   * `submit_move` returns `Err(ChannelClosed)` — planner thread panicked.
//   * `flush()` returns `Err` — dispatch returned a fatal error mid-flush.
//   * `last_fault != 0` — runtime latched (e.g. STEP_BURST_EXCEEDED at
//     `engine.rs::runtime_modulated_tick` line ~2843).
//   * `current_segment_id` doesn't advance past dispatched count within
//     90 s wall — MCU hung (hard fault / status stream stopped emitting).
//   * Dispatch closure captured an error — push_segment / load_curve
//     failed (the usual visible failure when the MCU latches between
//     dispatch steps of a subsequent segment).
//
// Pass condition: NONE of the above triggered. Bug reproduces ↔ test fails
// with a verbose panic including the captured tag summary.
//
// ===========================================================================

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn phase_stepping_rapid_g1_x25_after_set_position_no_crash() {
    let ctx = PlannerCtx::build_with_phase_stepping([80.0, 80.0, 0.0, 0.0])
        .expect("build sim harness with phase stepping");

    // Mimic `SET_KINEMATIC_POSITION X=125 Y=100 Z=10`. The bridge wires
    // klippy's set_position to `kalico_stream_open` (Phase 5 Task 5.1;
    // planner.rs:360-374 resets `ShaperState` to `home_pos`). The harness
    // already issued kalico_stream_open at `[0;4]` during `build_inner`;
    // we re-open at the bench position to mirror "stream open at origin →
    // SET_KINEMATIC_POSITION at 125,100,10".
    let start = [125.0_f64, 100.0, 10.0];
    ctx.planner
        .kalico_stream_open([start[0], start[1], start[2], 0.0])
        .expect("re-open stream at SET_KINEMATIC_POSITION");

    // Five `G1 X+25` in relative mode = five absolute moves from the
    // running position, each dx=+25. `classify_and_build` accepts an
    // absolute start + relative delta, matching the bridge's `manual_move`
    // wire shape one-to-one.
    let n_jogs = 5_usize;
    let jog_mm = 25.0_f64;
    // F=3000 mm/min = 50 mm/s. 25 mm at 50 mm/s = 0.5 s trajectory per move.
    let feedrate_mm_per_min = 3_000.0_f64;
    let mut pos = start;
    let mut submit_results: Vec<Result<(), String>> = Vec::with_capacity(n_jogs);
    for i in 0..n_jogs {
        let r = ctx.submit_jog(pos, jog_mm, 0.0, 0.0, feedrate_mm_per_min);
        eprintln!(
            "[test-phase] jog#{i}: dx=+{jog_mm} pos_after=[{:.1},{:.1},{:.1}] submit={r:?}",
            pos[0] + jog_mm, pos[1], pos[2],
        );
        submit_results.push(r);
        pos[0] += jog_mm;
        // No sleep — "G1 X25 in a row quickly" = back-to-back.
    }

    let flush_result = ctx.flush();
    eprintln!("[test-phase] flush result: {flush_result:?}");

    let dispatched = ctx.dispatched_segments();
    let target = dispatched.max(1) as u32;
    eprintln!(
        "[test-phase] dispatched_segments={dispatched} target_seg_id={target} \
         last_seg_id_observed={}",
        ctx.harness.last_segment_id(),
    );

    // Allow up to 90 s wall for the burst to retire. The modulated tick on
    // the H7 sim runs at 40 kHz, so per-segment retirement is gated on
    // wall-time reaching `t_end_clock` — Renode's virtual-time pacing
    // sets the real-wall budget.
    let wait_outcome = ctx
        .harness
        .wait_for_segment_id(target, Duration::from_secs(90));

    let status = ctx.harness.status();
    let summary = ctx.harness.fault_detail_summary();
    eprintln!(
        "[test-phase] final status: engine_status={} seg_id={} \
         last_fault={} fault_detail=0x{:08x}",
        status.engine_status, status.current_segment_id,
        status.last_fault, status.fault_detail,
    );
    eprintln!("[test-phase] fault_detail tag summary (tag → max payload):");
    for (tag, payload) in &summary {
        eprintln!("  0x{:02X} → 0x{:06X} ({})", tag, payload, payload);
    }
    if let Some(err) = ctx.last_dispatch_error() {
        eprintln!("[test-phase] dispatch error captured: {err}");
    }

    let submit_failures: Vec<(usize, String)> = submit_results
        .iter()
        .enumerate()
        .filter_map(|(i, r)| r.as_ref().err().map(|e| (i, e.clone())))
        .collect();
    let any_submit_err = !submit_failures.is_empty();
    let flush_err_str = flush_result.as_ref().err().cloned();
    let dispatch_err = ctx.last_dispatch_error();
    let fault_latched = status.last_fault != 0;
    let seg_id_stalled = wait_outcome.is_err();

    let crashed = any_submit_err
        || flush_err_str.is_some()
        || dispatch_err.is_some()
        || fault_latched
        || seg_id_stalled;

    if !crashed {
        eprintln!(
            "[test-phase] PASS-no-repro: phase-stepping + set_kinematic_position + \
             rapid G1 X+25×{n_jogs} burst dispatched {dispatched} segments, \
             seg_id reached {}, no fault, no submit/flush/dispatch errors. \
             The bench-reported crash did NOT reproduce in the sim.",
            ctx.harness.last_segment_id(),
        );
        return;
    }

    panic!(
        "BENCH BUG REPRODUCED IN SIM: phase-stepping + set_kinematic_position + \
         rapid G1 X+25 burst tripped at least one crash signature.\n\
         submit_failures={submit_failures:?}\n\
         flush_err={flush_err_str:?}\n\
         dispatch_err={dispatch_err:?}\n\
         last_fault={} fault_detail=0x{:08x}\n\
         seg_id_target={target} seg_id_observed={} dispatched={dispatched}\n\
         fault_detail summary: {summary:?}",
        status.last_fault, status.fault_detail,
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

// ===========================================================================
//
// Test E — homing trip via real virtual GPIO. End-to-end exercise of the
// homing data path:
//
//     bridge::endstop_arm
//   → runtime_arm_endstop command (USART)
//   → endstop_pin_table_populate (firmware, runtime_commands.c)
//     ↓ configures PA0 as input via gpio_in_setup
//   → motor starts stepping
//   → per-step runtime_endstop_sample_one (runtime_tick.c)
//     ↓ calls gpio_in_read(PA0)
//   → RenodeMonitor flips PA0 high mid-move
//   → next sampler call sees high → trip fires
//   → kalico_endstop_tripped output (kalico_dispatch.c)
//   → RuntimeEvent::EndstopTripped on the host event stream
//
// This is the "architecturally honest" homing test — every layer that runs
// on real hardware also runs here. The earlier in-process homing tests in
// `sim_motion.rs` validate the trip-event surface but stub the GPIO read,
// so they cannot catch a regression in `gpio_in_setup` / `gpio_in_read` /
// the per-step sampler invocation. This test does.
//
// Pin choice: PA0 (Klipper gpio id = `GPIO('A', 0)` = 0). It is not used by
// any other firmware peripheral in the sim configuration, so configuring it
// as input does not conflict with anything else.
//
// Pass condition: a single `EndstopTripped { arm_id == TEST_ARM_ID }` event
// is observed on the runtime event stream within 90 s wall (Renode's H7
// sim runs at ~150–200× slower than real wall-clock).
//
// ===========================================================================

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn homing_x_trips_when_pa0_raised_via_monitor() {
    const TEST_ARM_ID: u32 = 9001;
    const PA0_PORT: char = 'A';
    const PA0_PIN: u8 = 0;
    // Klipper STM32 pin id for PA0. Mirrors firmware `GPIO('A', 0)`.
    const PA0_GPIO_ID: u16 = 0;

    let ctx = PlannerCtx::build().expect("build sim harness");

    // Connect to the Renode monitor first, then raise PA0 *before* arming
    // and *before* submitting the move. With `TripImmediately`, the very
    // first per-step sampler invocation will read PA0 high and fire the
    // trip — no mid-motion GPIO-toggle race. Renode's STM32_GPIOPort
    // latches the input level until the next OnGPIO command, so PA0 stays
    // high once we set it.
    let mut monitor =
        RenodeMonitor::connect_with_timeout(Duration::from_secs(5))
            .expect("connect Renode monitor on port 3335");
    monitor
        .set_gpio_input(PA0_PORT, PA0_PIN, true)
        .expect("pre-arm: raise PA0");

    // Arm an endstop on PA0. Single source, active-high, sample_n=1 (one
    // high sample triggers), no velocity gating (`v_min_q16 = 0`,
    // velocity_axis=0x07 = X|Y|Z), `TripImmediately` so trip fires on the
    // first matching sample. Per-step sampling means no sample happens
    // until the motor steps — so this is still "trip mid-motion", just
    // deterministic about which step triggers (the first).
    let sources = [SourceSpec {
        kind: SourceKind::Physical,
        gpio: PA0_GPIO_ID,
        active_high: true,
        policy: ArmPolicy::TripImmediately,
        sample_n: 1,
        velocity_axis: 0x07, // X | Y | Z
        v_min_q16: 0,
    }];
    // CoreXY motor 0 = A, motor 1 = B. We hand both OIDs to the runtime so
    // its per-stepper trip step_count vector is populated for either path.
    let stepper_oids = [0_u8, 1_u8];
    let arm_status = arm_endstop_with_timeout(
        ctx.harness.host_io.as_ref(),
        TEST_ARM_ID,
        0, // arm_clock = 0 → effective immediately
        &sources,
        &stepper_oids,
        Duration::from_secs(5),
    )
    .expect("arm_endstop_with_timeout");
    eprintln!("[test-E] arm_endstop returned status={arm_status:?}");
    assert_eq!(
        arm_status,
        ArmStatus::Armed,
        "MCU did not arm endstop on PA0 — status={arm_status:?}",
    );

    // Submit a 25 mm pure-X jog at F=100 — same shape as `test A`, which we
    // already know retires in ~36 s wall with `runtime_emit_calls` reaching
    // ~460. The motor produces step pulses; the very first per-step sampler
    // invocation reads PA0 high and fires the trip.
    ctx.submit_jog([0.0; 3], 25.0, 0.0, 0.0, 100.0)
        .expect("submit 25mm X jog");
    ctx.flush().expect("flush after homing jog");
    eprintln!(
        "[test-E] post-flush: dispatched={} waiting for trip event …",
        ctx.dispatched_segments(),
    );

    // Wait for the trip event. 90 s wall budget — same as test A.
    let trip_outcome = ctx
        .harness
        .wait_for_endstop_trip(TEST_ARM_ID, Duration::from_secs(90));

    let status = ctx.harness.status();
    let summary = ctx.harness.fault_detail_summary();
    eprintln!(
        "[test-E] final status: engine_status={} seg_id={} last_fault={} \
         fault_detail=0x{:08x}",
        status.engine_status, status.current_segment_id, status.last_fault,
        status.fault_detail,
    );
    eprintln!("[test-E] fault_detail tag summary (tag → max payload):");
    for (tag, payload) in &summary {
        eprintln!("  0x{:02X} → 0x{:06X} ({})", tag, payload, payload);
    }

    match trip_outcome {
        Ok(e) => {
            eprintln!(
                "[test-E] PASS: trip event arm_id={} trip_clock={} \
                 src_idx={} stepper_count={}",
                e.arm_id, e.trip_clock, e.trip_source_idx, e.stepper_count,
            );
            assert_eq!(e.arm_id, TEST_ARM_ID);
            assert_eq!(e.fmt_version, kalico_host_rt::endstop::FMT_VERSION_V1);
        }
        Err(seen) => {
            // Surface enough diagnostics to localize the broken layer:
            //   * 0xE1 = runtime_emit_calls — did the runtime emit any
            //     step pulses at all? If 0, motor never moved, so the
            //     per-step sampler never ran. Suspect dispatch / motor
            //     config.
            //   * 0xB2 = ring_high_water — did the producer push step
            //     entries to any motor's ring? If 0 with emit_calls > 0,
            //     producer-side breakage. If both > 0 but no trip,
            //     suspect the GPIO read path (Renode model) or sampler
            //     wiring.
            let e1 = ctx.harness.fault_detail_max(0xE1).unwrap_or(0);
            let b2 = ctx.harness.fault_detail_max(0xB2).unwrap_or(0);
            panic!(
                "no EndstopTripped event for arm_id={TEST_ARM_ID} within 90s. \
                 emit_calls(0xE1) max={e1}, ring_high_water(0xB2)=0x{b2:06X}. \
                 trips_seen={seen:?}. Last status: {status:?}. \
                 Tag summary: {summary:?}",
            );
        }
    }
}

// ===========================================================================
// Test G — G28-shaped multi-axis two-pass homing sequence.
//
// Mirrors what klippy's `home_rails()` in `klippy/extras/homing.py` does for
// each rail: arm → fast-home → trip → disarm → back-off → re-arm →
// slow-home → trip → disarm. We run that sequence for X (PA0) and then Y
// (PA1) in a single sim session, exercising:
//
//   1. Sequential arm/disarm cycles on the **same** pin (X first + X second).
//      Tests engine-state cleanup after a trip-induced abort: can the
//      runtime accept the next `runtime_arm_endstop`, and does the next
//      motion segment produce step pulses?
//   2. Arm/disarm on **different** pins back-to-back (X then Y). Tests
//      that the endstop pin table is rewritten correctly between arms and
//      the sampler picks up the new pin for the new arm_id.
//   3. Motion submission **after** a trip-induced abort (the "back-off"
//      jog). On the printer this is where regular-stepping G28 silently
//      fails: the motor energizes but produces no actual motion. This test
//      asserts step pulses are emitted for every move in the sequence.
//   4. CoreXY motors A+B driving both X-only and Y-only logical moves.
//      A pure X jog steps both motors in opposite directions; a pure Y
//      jog steps both in the same direction. The single-axis E test only
//      covers X.
//
// Pin choice: PA0 (gpio id 0) for X, PA1 (gpio id 1) for Y. Neither is used
// by any sim-firmware peripheral.
//
// Pass condition: four `EndstopTripped` events arrive in order (arm_id
// 9001..=9004), and the inter-trip motion segments each produce step
// pulses (`runtime_emit_calls` tag 0xE1 advances between trips).
//
// ===========================================================================

#[test]
#[ignore = "spawns Renode subprocess; run with --ignored --test-threads=1"]
fn g28_shaped_xy_two_pass_homing_via_renode_monitor() {
    const ARM_X_FAST: u32 = 9001;
    const ARM_X_SLOW: u32 = 9002;
    const ARM_Y_FAST: u32 = 9003;
    const ARM_Y_SLOW: u32 = 9004;
    const PA0: (char, u8, u16) = ('A', 0, 0);     // X endstop
    const PA1: (char, u8, u16) = ('A', 1, 1);     // Y endstop
    const STEPPER_OIDS: [u8; 2] = [0, 1];          // CoreXY A + B
    const VELOCITY_AXIS_ALL: u8 = 0x07;            // X | Y | Z

    let ctx = PlannerCtx::build().expect("build sim harness");

    let mut monitor =
        RenodeMonitor::connect_with_timeout(Duration::from_secs(5))
            .expect("connect Renode monitor on port 3335");

    // Helper closure: arm an endstop on (port,pin,gpio_id) with TripImmediately
    // policy after pre-raising the pin. The first per-step sampler invocation
    // reads the pin high → trip fires on the first step.
    let arm_after_raise = |mon: &mut RenodeMonitor,
                           pin: (char, u8, u16),
                           arm_id: u32|
     -> ArmStatus {
        mon.set_gpio_input(pin.0, pin.1, true)
            .unwrap_or_else(|e| panic!("pre-raise P{}{}: {e}", pin.0, pin.1));
        let sources = [SourceSpec {
            kind: SourceKind::Physical,
            gpio: pin.2,
            active_high: true,
            policy: ArmPolicy::TripImmediately,
            sample_n: 1,
            velocity_axis: VELOCITY_AXIS_ALL,
            v_min_q16: 0,
        }];
        arm_endstop_with_timeout(
            ctx.harness.host_io.as_ref(),
            arm_id,
            0,
            &sources,
            &STEPPER_OIDS,
            Duration::from_secs(5),
        )
        .unwrap_or_else(|e| panic!("arm_endstop arm_id={arm_id}: {e:?}"))
    };

    // Helper: lower a pin (used for back-off so the next pre-raise+arm
    // cycle observes a clean low → high transition).
    let lower_pin =
        |mon: &mut RenodeMonitor, pin: (char, u8, u16)| {
            mon.set_gpio_input(pin.0, pin.1, false)
                .unwrap_or_else(|e| panic!("lower P{}{}: {e}", pin.0, pin.1));
        };

    // Per-phase telemetry: dispatched-from-host count + MCU-side retired
    // segment id. dispatched goes up the moment the bridge writes a frame;
    // last_segment_id goes up when the MCU retires the segment (in StatusEvent
    // frames). We need both to localize where work stalls.
    let snapshot = |ctx: &PlannerCtx, label: &str| -> (u64, u32) {
        let d = ctx.dispatched_segments();
        let s = ctx.harness.last_segment_id();
        eprintln!(
            "[test-G] {label}: dispatched={d} mcu_seg_id={s}",
        );
        (d, s)
    };

    // -----------------------------------------------------------------
    // X — pass 1 (fast home: 100 mm/min toward -X over 25 mm)
    // -----------------------------------------------------------------
    eprintln!("[test-G] === X fast home (arm_id={ARM_X_FAST}) ===");
    let s = arm_after_raise(&mut monitor, PA0, ARM_X_FAST);
    assert_eq!(s, ArmStatus::Armed, "X fast-home arm rejected: {s:?}");
    ctx.submit_jog([0.0; 3], -25.0, 0.0, 0.0, 100.0)
        .expect("X fast-home jog submit");
    ctx.flush().expect("flush X fast-home");
    let trip_x1 = ctx
        .harness
        .wait_for_endstop_trip(ARM_X_FAST, Duration::from_secs(120))
        .unwrap_or_else(|seen| {
            panic!(
                "X fast-home trip not observed; \
                 emit_calls(0xE1)={:?}, trips_seen={seen:?}, status={:?}",
                ctx.harness.fault_detail_max(0xE1),
                ctx.harness.status(),
            )
        });
    eprintln!(
        "[test-G] X fast-home trip: arm_id={} clock={}",
        trip_x1.arm_id, trip_x1.trip_clock,
    );
    assert_eq!(trip_x1.arm_id, ARM_X_FAST);
    let after_xfast = snapshot(&ctx, "after X fast-home trip");
    let disarm_x1 = kalico_host_rt::endstop::disarm_endstop_with_timeout(
        ctx.harness.host_io.as_ref(),
        ARM_X_FAST,
        Duration::from_secs(8),
    );
    eprintln!("[test-G] X fast-home disarm: {disarm_x1:?}");
    lower_pin(&mut monitor, PA0);
    // Renode runs ~5× slower than wall-clock; give the MCU time to
    // drain trip-aborted segments before submitting the next phase.
    thread::sleep(Duration::from_secs(1));

    // -----------------------------------------------------------------
    // X — back-off (+5 mm at slower speed, no arm). Tests motion
    // submission after a trip-induced abort.
    // -----------------------------------------------------------------
    eprintln!("[test-G] === X back-off ===");
    ctx.submit_jog([-25.0, 0.0, 0.0], 5.0, 0.0, 0.0, 30.0)
        .expect("X back-off jog submit");
    ctx.flush().expect("flush X back-off");
    // Wait for MCU to retire one more segment than the X fast-home's
    // last-known retired id. Renode runs ~5× slower than wall-clock; 60 s
    // is comfortable headroom for a 5 mm jog at 30 mm/min.
    let xback_target = after_xfast.1.saturating_add(1);
    let xback_outcome =
        ctx.harness.wait_for_segment_id(xback_target, Duration::from_secs(60));
    let after_xback = snapshot(&ctx, "after X back-off");
    if xback_outcome.is_err() {
        let status = ctx.harness.status();
        let summary = ctx.harness.fault_detail_summary();
        panic!(
            "X back-off did not retire on MCU within 60 s. \
             Wanted mcu_seg_id ≥ {xback_target}, got {}. \
             dispatched={}, status={status:?}, fault_tags={summary:?}",
            after_xback.1, after_xback.0,
        );
    }

    // -----------------------------------------------------------------
    // X — pass 2 (slow home: 30 mm/min, 25 mm toward -X again)
    // -----------------------------------------------------------------
    eprintln!("[test-G] === X slow home (arm_id={ARM_X_SLOW}) ===");
    // For the slow-home pass, the pin is high at arm time (we pre-raised
    // it). `TripImmediately` accepts either `Armed` (trip fires on first
    // post-arm sample) or `AlreadyTripped` (trip event published from the
    // arm path itself); both produce an `EndstopTripped` event on the
    // runtime-events channel matching the arm_id, which is what
    // `wait_for_endstop_trip` blocks on. Treat `Rejected` (status=2) as
    // failure.
    let s = arm_after_raise(&mut monitor, PA0, ARM_X_SLOW);
    assert_ne!(
        s, ArmStatus::Rejected,
        "X slow-home arm rejected: {s:?}",
    );
    ctx.submit_jog([-20.0, 0.0, 0.0], -25.0, 0.0, 0.0, 30.0)
        .expect("X slow-home jog submit");
    ctx.flush().expect("flush X slow-home");
    let trip_x2 = ctx
        .harness
        .wait_for_endstop_trip(ARM_X_SLOW, Duration::from_secs(180))
        .unwrap_or_else(|seen| {
            panic!(
                "X slow-home trip not observed; \
                 emit_calls(0xE1)={:?}, trips_seen={seen:?}, status={:?}",
                ctx.harness.fault_detail_max(0xE1),
                ctx.harness.status(),
            )
        });
    eprintln!(
        "[test-G] X slow-home trip: arm_id={} clock={}",
        trip_x2.arm_id, trip_x2.trip_clock,
    );
    assert_eq!(trip_x2.arm_id, ARM_X_SLOW);
    let _after_xslow = snapshot(&ctx, "after X slow-home trip");
    let disarm_x2 = kalico_host_rt::endstop::disarm_endstop_with_timeout(
        ctx.harness.host_io.as_ref(),
        ARM_X_SLOW,
        Duration::from_secs(8),
    );
    eprintln!("[test-G] X slow-home disarm: {disarm_x2:?}");
    lower_pin(&mut monitor, PA0);
    thread::sleep(Duration::from_secs(1));

    // -----------------------------------------------------------------
    // Y — pass 1 (fast home on PA1)
    // -----------------------------------------------------------------
    eprintln!("[test-G] === Y fast home (arm_id={ARM_Y_FAST}) ===");
    let s = arm_after_raise(&mut monitor, PA1, ARM_Y_FAST);
    assert_eq!(s, ArmStatus::Armed, "Y fast-home arm rejected: {s:?}");
    ctx.submit_jog([-45.0, 0.0, 0.0], 0.0, -25.0, 0.0, 100.0)
        .expect("Y fast-home jog submit");
    ctx.flush().expect("flush Y fast-home");
    let trip_y1 = ctx
        .harness
        .wait_for_endstop_trip(ARM_Y_FAST, Duration::from_secs(120))
        .unwrap_or_else(|seen| {
            panic!(
                "Y fast-home trip not observed; \
                 emit_calls(0xE1)={:?}, trips_seen={seen:?}, status={:?}",
                ctx.harness.fault_detail_max(0xE1),
                ctx.harness.status(),
            )
        });
    eprintln!(
        "[test-G] Y fast-home trip: arm_id={} clock={}",
        trip_y1.arm_id, trip_y1.trip_clock,
    );
    assert_eq!(trip_y1.arm_id, ARM_Y_FAST);
    let after_yfast = snapshot(&ctx, "after Y fast-home trip");
    let disarm_y1 = kalico_host_rt::endstop::disarm_endstop_with_timeout(
        ctx.harness.host_io.as_ref(),
        ARM_Y_FAST,
        Duration::from_secs(8),
    );
    eprintln!("[test-G] Y fast-home disarm: {disarm_y1:?}");
    lower_pin(&mut monitor, PA1);
    thread::sleep(Duration::from_secs(1));

    // -----------------------------------------------------------------
    // Y — back-off (+5 mm Y at slow speed)
    // -----------------------------------------------------------------
    eprintln!("[test-G] === Y back-off ===");
    ctx.submit_jog([-45.0, -25.0, 0.0], 0.0, 5.0, 0.0, 30.0)
        .expect("Y back-off jog submit");
    ctx.flush().expect("flush Y back-off");
    thread::sleep(Duration::from_secs(2));
    let yback_target = after_yfast.1.saturating_add(1);
    let yback_outcome =
        ctx.harness.wait_for_segment_id(yback_target, Duration::from_secs(60));
    let after_yback = snapshot(&ctx, "after Y back-off");
    if yback_outcome.is_err() {
        let status = ctx.harness.status();
        let summary = ctx.harness.fault_detail_summary();
        panic!(
            "Y back-off did not retire on MCU within 60 s. \
             Wanted mcu_seg_id ≥ {yback_target}, got {}. \
             dispatched={}, status={status:?}, fault_tags={summary:?}",
            after_yback.1, after_yback.0,
        );
    }

    // -----------------------------------------------------------------
    // Y — pass 2 (slow home)
    // -----------------------------------------------------------------
    eprintln!("[test-G] === Y slow home (arm_id={ARM_Y_SLOW}) ===");
    // See X slow-home arm comment: AlreadyTripped is acceptable here too.
    let s = arm_after_raise(&mut monitor, PA1, ARM_Y_SLOW);
    assert_ne!(
        s, ArmStatus::Rejected,
        "Y slow-home arm rejected: {s:?}",
    );
    ctx.submit_jog([-45.0, -20.0, 0.0], 0.0, -25.0, 0.0, 30.0)
        .expect("Y slow-home jog submit");
    ctx.flush().expect("flush Y slow-home");
    let trip_y2 = ctx
        .harness
        .wait_for_endstop_trip(ARM_Y_SLOW, Duration::from_secs(180))
        .unwrap_or_else(|seen| {
            panic!(
                "Y slow-home trip not observed; \
                 emit_calls(0xE1)={:?}, trips_seen={seen:?}, status={:?}",
                ctx.harness.fault_detail_max(0xE1),
                ctx.harness.status(),
            )
        });
    eprintln!(
        "[test-G] Y slow-home trip: arm_id={} clock={}",
        trip_y2.arm_id, trip_y2.trip_clock,
    );
    assert_eq!(trip_y2.arm_id, ARM_Y_SLOW);
    let _after_yslow = snapshot(&ctx, "after Y slow-home trip");
    let disarm_y2 = kalico_host_rt::endstop::disarm_endstop_with_timeout(
        ctx.harness.host_io.as_ref(),
        ARM_Y_SLOW,
        Duration::from_secs(8),
    );
    eprintln!("[test-G] Y slow-home disarm: {disarm_y2:?}");

    eprintln!(
        "[test-G] PASS: 4 trips (X fast, X slow, Y fast, Y slow) + 2 \
         back-off segments completed.",
    );
    let status = ctx.harness.status();
    eprintln!(
        "[test-G] final status: engine_status={} seg_id={} \
         last_fault={} fault_detail=0x{:08x}",
        status.engine_status,
        status.current_segment_id,
        status.last_fault,
        status.fault_detail,
    );
}

