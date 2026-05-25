# Rust Homing Loop for External Probes — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the external-probe (Beacon) Z homing loop from Python into Rust so the trigger path is Beacon wire → Rust reactor → bottom MCU wire, with zero Python in the critical path.

**Architecture:** A generic frame interceptor table on the Rust reactor fires `software_trip` at wire speed when the Beacon trigger arrives. A separate Rust homing loop thread handles deadline extension, sensor-fault timeout, and result reporting. Python calls one blocking method and gets back a result code.

**Tech Stack:** Rust (kalico-host-rt reactor, motion-bridge PyO3), Python (klippy/motion_toolhead.py)

**Spec:** `docs/superpowers/specs/2026-05-25-rust-homing-loop-design.md`

---

## File Map

| File | Action | Responsibility |
|------|--------|---------------|
| `rust/kalico-host-rt/src/host_io/interceptor.rs` | Create | `InterceptorId`, `InterceptorEntry`, `InterceptorTable` — generic frame interceptor infra |
| `rust/kalico-host-rt/src/host_io/reactor.rs` | Modify | Add `InterceptorTable` field to `Reactor`, call interceptors in `handle_inbound_frame` |
| `rust/kalico-host-rt/src/host_io/mod.rs` | Modify | Add `ReactorCommand` variants for register/unregister, add public methods on `KalicoHostIo` |
| `rust/kalico-host-rt/src/host_io.rs` or `mod.rs` | Modify | Add `pub mod interceptor;` declaration |
| `rust/motion-bridge/src/probe_homing.rs` | Create | `ProbeHomingResult` enum, `run_probe_homing` function (loop thread + interceptor setup) |
| `rust/motion-bridge/src/lib.rs` | Modify | Add `pub mod probe_homing;` |
| `rust/motion-bridge/src/bridge.rs` | Modify | Add `run_probe_homing` pymethod delegating to `probe_homing::run_probe_homing` |
| `klippy/motion_toolhead.py` | Modify | Simplify `_drip_move_software_trip` to call `bridge.run_probe_homing(...)` |

---

### Task 1: Frame interceptor table data structures

**Files:**
- Create: `rust/kalico-host-rt/src/host_io/interceptor.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs` (add module declaration)

- [ ] **Step 1: Create the interceptor module**

Create `rust/kalico-host-rt/src/host_io/interceptor.rs`:

```rust
use std::collections::HashMap;
use std::sync::atomic::{AtomicU64, Ordering};

use crate::transport::MessageParams;

static NEXT_ID: AtomicU64 = AtomicU64::new(1);

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct InterceptorId(u64);

impl InterceptorId {
    fn next() -> Self {
        Self(NEXT_ID.fetch_add(1, Ordering::Relaxed))
    }
}

pub(crate) struct InterceptorEntry {
    pub id: InterceptorId,
    pub callback: Box<dyn Fn(&MessageParams) + Send + Sync>,
}

#[derive(Debug, Clone, PartialEq, Eq, Hash)]
struct InterceptorKey {
    msg_name: String,
    oid: Option<u32>,
}

pub(crate) struct InterceptorTable {
    entries: HashMap<InterceptorKey, Vec<InterceptorEntry>>,
}

impl InterceptorTable {
    pub fn new() -> Self {
        Self {
            entries: HashMap::new(),
        }
    }

    pub fn register(
        &mut self,
        msg_name: String,
        oid: Option<u32>,
        callback: Box<dyn Fn(&MessageParams) + Send + Sync>,
    ) -> InterceptorId {
        let id = InterceptorId::next();
        let key = InterceptorKey { msg_name, oid };
        self.entries
            .entry(key)
            .or_default()
            .push(InterceptorEntry { id, callback });
        id
    }

    pub fn unregister(&mut self, id: InterceptorId) {
        self.entries.retain(|_, entries| {
            entries.retain(|e| e.id != id);
            !entries.is_empty()
        });
    }

    pub fn dispatch(&self, msg_name: &str, oid: Option<u32>, params: &MessageParams) {
        let key = InterceptorKey {
            msg_name: msg_name.to_owned(),
            oid,
        };
        if let Some(entries) = self.entries.get(&key) {
            for entry in entries {
                (entry.callback)(params);
            }
        }
    }
}

impl std::fmt::Debug for InterceptorEntry {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("InterceptorEntry")
            .field("id", &self.id)
            .finish_non_exhaustive()
    }
}
```

- [ ] **Step 2: Add module declaration**

In `rust/kalico-host-rt/src/host_io/mod.rs`, add near the other `pub mod` / `mod` declarations:

```rust
pub(crate) mod interceptor;
```

Also re-export the public types:

```rust
pub use interceptor::InterceptorId;
```

- [ ] **Step 3: Build and verify**

Run: `cd rust && cargo build -p kalico-host-rt 2>&1 | tail -5`
Expected: compiles with no errors (warnings OK)

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/interceptor.rs rust/kalico-host-rt/src/host_io/mod.rs
git commit -m "feat: add frame interceptor table data structures (kalico-host-rt)"
```

---

### Task 2: Wire interceptor table into the reactor

**Files:**
- Modify: `rust/kalico-host-rt/src/host_io/reactor.rs`
- Modify: `rust/kalico-host-rt/src/host_io/mod.rs`

- [ ] **Step 1: Add interceptor table to Reactor struct**

In `rust/kalico-host-rt/src/host_io/reactor.rs`, add a field to `struct Reactor` (line ~28):

```rust
pub(crate) interceptors: crate::host_io::interceptor::InterceptorTable,
```

Initialize it in every `Reactor::new` / constructor call site with:

```rust
interceptors: crate::host_io::interceptor::InterceptorTable::new(),
```

- [ ] **Step 2: Call interceptors in handle_inbound_frame**

In `reactor.rs`, in the `handle_inbound_frame` method, in the unsolicited response branch (around line 799 where `PassthroughResponse` is constructed), add the interceptor dispatch **before** `self.dispatch_runtime_event(event)`:

```rust
// --- existing code around line 792-803 ---
// Unsolicited Klipper-protocol Response frames ...
let oid = params.fields.get("oid").and_then(|v| match v {
    crate::transport::MessageValue::U32(n) => Some(*n),
    crate::transport::MessageValue::I32(n) => Some(*n as u32),
    _ => None,
});
self.interceptors.dispatch(&name, oid, &params);
let event = crate::host_io::runtime_events::RuntimeEvent::PassthroughResponse {
    name,
    params,
};
self.dispatch_runtime_event(event);
```

Note: this replaces the existing 4-line block that creates and dispatches the `PassthroughResponse`. The `name` and `params` must not be moved before the interceptor call — extract `oid` first, dispatch interceptors, then create the event.

- [ ] **Step 3: Add ReactorCommand variants for register/unregister**

In `rust/kalico-host-rt/src/host_io/mod.rs`, add to `enum ReactorCommand`:

```rust
RegisterInterceptor {
    msg_name: String,
    oid: Option<u32>,
    callback: Box<dyn Fn(&crate::transport::MessageParams) + Send + Sync>,
    reply: SyncSender<crate::host_io::InterceptorId>,
},
UnregisterInterceptor {
    id: crate::host_io::InterceptorId,
},
```

- [ ] **Step 4: Handle the new commands in the reactor**

In `reactor.rs` `handle_command`, add match arms:

```rust
ReactorCommand::RegisterInterceptor {
    msg_name,
    oid,
    callback,
    reply,
} => {
    let id = self.interceptors.register(msg_name, oid, callback);
    let _ = reply.send(id);
}
ReactorCommand::UnregisterInterceptor { id } => {
    self.interceptors.unregister(id);
}
```

- [ ] **Step 5: Add public methods on KalicoHostIo**

In `rust/kalico-host-rt/src/host_io/mod.rs`, add methods on `impl KalicoHostIo`:

```rust
pub fn register_frame_interceptor(
    &self,
    msg_name: &str,
    oid: Option<u32>,
    callback: Box<dyn Fn(&crate::transport::MessageParams) + Send + Sync>,
) -> Result<InterceptorId, TransportError> {
    let (reply_tx, reply_rx) = std::sync::mpsc::sync_channel(1);
    self.submission_tx
        .send(ReactorCommand::RegisterInterceptor {
            msg_name: msg_name.to_owned(),
            oid,
            callback,
            reply: reply_tx,
        })
        .map_err(|_| TransportError::Closed)?;
    reply_rx.recv().map_err(|_| TransportError::Closed)
}

pub fn unregister_frame_interceptor(
    &self,
    id: InterceptorId,
) -> Result<(), TransportError> {
    self.submission_tx
        .send(ReactorCommand::UnregisterInterceptor { id })
        .map_err(|_| TransportError::Closed)
}
```

- [ ] **Step 6: Build and verify**

Run: `cd rust && cargo build -p kalico-host-rt 2>&1 | tail -5`
Expected: compiles

- [ ] **Step 7: Commit**

```bash
git add rust/kalico-host-rt/src/host_io/reactor.rs rust/kalico-host-rt/src/host_io/mod.rs
git commit -m "feat: wire interceptor table into reactor + register/unregister API"
```

---

### Task 3: Probe homing loop module

**Files:**
- Create: `rust/motion-bridge/src/probe_homing.rs`
- Modify: `rust/motion-bridge/src/lib.rs`

- [ ] **Step 1: Create the probe_homing module**

Create `rust/motion-bridge/src/probe_homing.rs`:

```rust
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use kalico_host_rt::host_io::{InterceptorId, KalicoHostIo};
use kalico_host_rt::transport::TransportError;

use crate::homing::HomingSegmentState;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ProbeHomingResult {
    ProbeTriggered = 0,
    SegmentRetired = 1,
    SensorFault = 2,
    DeadlineExpired = 3,
}

pub struct ProbeHomingParams {
    pub beacon_io: Arc<KalicoHostIo>,
    pub stepper_io: Arc<KalicoHostIo>,
    pub beacon_trsync_oid: u8,
    pub arm_id: u32,
    pub sensor_fault_timeout: Duration,
}

const TICK_INTERVAL: Duration = Duration::from_millis(25);

pub fn run_probe_homing(
    params: &ProbeHomingParams,
    homing: &crate::homing::HomingState,
) -> Result<ProbeHomingResult, TransportError> {
    let triggered = Arc::new(AtomicBool::new(false));

    // Register interceptor: when Beacon sends trsync_state(can_trigger=0),
    // fire software_trip to the stepper MCU immediately in the reactor thread.
    let interceptor_id = {
        let triggered_clone = Arc::clone(&triggered);
        let stepper_io_clone = Arc::clone(&params.stepper_io);
        let arm_id = params.arm_id;

        params.beacon_io.register_frame_interceptor(
            "trsync_state",
            Some(u32::from(params.beacon_trsync_oid)),
            Box::new(move |msg_params| {
                let can_trigger = msg_params.get_u32("can_trigger");
                if can_trigger == 0 {
                    let cmd = format!("runtime_software_trip arm_id={arm_id}");
                    let _ = stepper_io_clone.send_fire_and_forget(&cmd);
                    triggered_clone.store(true, Ordering::Release);
                }
            }),
        )?
    };

    // Send an immediate extend before entering the loop — provides margin
    // against the MCU's initial 50ms grant window.
    let extend_cmd = format!("runtime_extend_homing_deadline arm_id={}", params.arm_id);
    params.stepper_io.send_fire_and_forget(&extend_cmd)?;

    let result = run_loop(params, homing, &triggered, &extend_cmd);

    // Always unregister, even on error.
    let _ = params.beacon_io.unregister_frame_interceptor(interceptor_id);

    result
}

fn run_loop(
    params: &ProbeHomingParams,
    homing: &crate::homing::HomingState,
    triggered: &AtomicBool,
    extend_cmd: &str,
) -> Result<ProbeHomingResult, TransportError> {
    let start = Instant::now();

    loop {
        std::thread::sleep(TICK_INTERVAL);
        let elapsed = start.elapsed();

        if triggered.load(Ordering::Acquire) {
            log::info!(
                "[probe-homing] probe triggered elapsed={:.3}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::ProbeTriggered);
        }

        homing.refresh_after_wait();
        let state = homing.state();
        if matches!(
            state,
            HomingSegmentState::Tripped | HomingSegmentState::DeadlineExpired
        ) {
            log::info!(
                "[probe-homing] segment terminal state={:?} elapsed={:.3}s",
                state,
                elapsed.as_secs_f64(),
            );
            return Ok(match state {
                HomingSegmentState::DeadlineExpired => ProbeHomingResult::DeadlineExpired,
                _ => ProbeHomingResult::ProbeTriggered,
            });
        }
        if state == HomingSegmentState::Completed {
            log::info!(
                "[probe-homing] segment retired (no trigger) elapsed={:.3}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::SegmentRetired);
        }

        if elapsed > params.sensor_fault_timeout {
            log::error!(
                "[probe-homing] SENSOR FAULT: no trigger after {:.1}s",
                elapsed.as_secs_f64(),
            );
            return Ok(ProbeHomingResult::SensorFault);
        }

        params.stepper_io.send_fire_and_forget(extend_cmd)?;
    }
}
```

- [ ] **Step 2: Add module declaration**

In `rust/motion-bridge/src/lib.rs`, add:

```rust
pub mod probe_homing;
```

- [ ] **Step 3: Build and verify**

Run: `cd rust && cargo build -p motion-bridge 2>&1 | tail -5`
Expected: compiles

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/probe_homing.rs rust/motion-bridge/src/lib.rs
git commit -m "feat: probe homing loop module with interceptor + deadline extension"
```

---

### Task 4: Bridge pymethod `run_probe_homing`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

- [ ] **Step 1: Add the pymethod**

In `bridge.rs`, add inside the `#[pymethods]` impl block for `PyMotionBridge`, near the existing homing methods:

```rust
#[pyo3(signature = (
    beacon_handle,
    beacon_trsync_oid,
    stepper_mcu_handle,
    arm_id,
    move_pos,
    speed,
    sensor_fault_timeout_s,
    stepper_oids,
))]
fn run_probe_homing(
    &self,
    py: Python<'_>,
    beacon_handle: u32,
    beacon_trsync_oid: u8,
    stepper_mcu_handle: u32,
    arm_id: u32,
    move_pos: Vec<f64>,
    speed: f64,
    sensor_fault_timeout_s: f64,
    stepper_oids: Vec<u8>,
) -> PyResult<u8> {
    let beacon_io = self.host_io_for_mcu("run_probe_homing(beacon)", beacon_handle)?;
    let stepper_io = self.host_io_for_mcu("run_probe_homing(stepper)", stepper_mcu_handle)?;

    // Submit the homing move (reuses existing infra).
    self.submit_homing_move_inner(
        &move_pos,
        speed,
        &[arm_id],
    )?;

    let params = crate::probe_homing::ProbeHomingParams {
        beacon_io,
        stepper_io,
        beacon_trsync_oid,
        arm_id,
        sensor_fault_timeout: std::time::Duration::from_secs_f64(sensor_fault_timeout_s),
    };

    // Release GIL so Python's reactor timers (if any) can still fire,
    // and so we don't hold the GIL for seconds during the homing move.
    let result = py.allow_threads(|| {
        crate::probe_homing::run_probe_homing(&params, &self.homing)
    });

    match result {
        Ok(r) => Ok(r as u8),
        Err(e) => Err(PyRuntimeError::new_err(format!(
            "run_probe_homing transport error: {e}"
        ))),
    }
}
```

Note: `submit_homing_move_inner` is an existing private method on `PyMotionBridge`. Check that it's accessible from within the same impl block. If it's in a separate impl block, move the call or make it `pub(crate)`.

- [ ] **Step 2: Build and verify**

Run: `cd rust && cargo build -p motion-bridge 2>&1 | tail -5`
Expected: compiles. If `submit_homing_move_inner` has visibility issues, adjust accordingly.

- [ ] **Step 3: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat: run_probe_homing pymethod on PyMotionBridge"
```

---

### Task 5: Simplify Python `_drip_move_software_trip`

**Files:**
- Modify: `klippy/motion_toolhead.py`

- [ ] **Step 1: Replace the extension loop with `run_probe_homing`**

Replace the body of `_drip_move_software_trip` (lines 484-679 in `motion_toolhead.py`). The new version keeps the motor-enable and endstop-arm preamble, replaces the loop with one call, and simplifies the cleanup:

```python
def _drip_move_software_trip(self, newpos, speed, drip_completion):
    from . import motion_bridge as _mb
    from . import motion_kinematics

    self.bridge.wait_moves()
    self._ground_pending_end_time_after_bridge_drain()

    pos3 = list(newpos[:3]) + [0.0] * max(0, 3 - len(newpos[:3]))
    dx = pos3[0] - self.commanded_pos[0]
    dy = pos3[1] - self.commanded_pos[1]
    dz = pos3[2] - self.commanded_pos[2]

    kin_name = self.kinematics_name or ""
    motor_d = motion_kinematics.motor_deltas(kin_name, dx, dy, dz, 0.0)
    slot_prefixes = ["stepper_x", "stepper_y", "stepper_z", "extruder"]
    moving_steppers = []
    for slot_idx, delta in enumerate(motor_d):
        if abs(delta) < 1e-9:
            continue
        prefix = slot_prefixes[slot_idx]
        for s in self.kin.get_steppers():
            if s.get_name().startswith(prefix):
                moving_steppers.append(s)

    if not moving_steppers:
        self.move(newpos, speed)
        return

    stepper_mcus = set(s.get_mcu() for s in moving_steppers)
    if len(stepper_mcus) > 1:
        raise self.printer.command_error(
            "External probe homing across multiple bridge MCUs "
            "is not supported"
        )
    stepper_mcu = next(iter(stepper_mcus))
    mcu_handle = stepper_mcu._bridge_handle
    queue = self.bridge.alloc_command_queue(mcu_handle)

    arm_id = _mb._alloc_arm_id()
    stepper_oids = [s.get_oid() for s in moving_steppers]
    source = (_mb.SOURCE_KIND_SOFTWARE, 0, False, 0, 1, 0, 0)

    ENABLE_HEADROOM = 2.000
    lmt = self.get_last_move_time()
    est_now = 0.0
    if self.mcu is not None:
        est_now = self.mcu.estimated_print_time(
            self.reactor.monotonic())
        needed = est_now + ENABLE_HEADROOM
        if lmt < needed:
            self.dwell(needed - lmt)
            lmt = self.get_last_move_time()

    logging.info(
        "[probe-homing] pre-enable: lmt=%.6f est_now=%.6f "
        "stepper_mcu=%s",
        lmt, est_now, stepper_mcu.get_name(),
    )

    self._fire_active_callbacks(dx, dy, dz, 0.0, lmt)

    if self.mcu is not None:
        est_now = self.mcu.estimated_print_time(
            self.reactor.monotonic())
    arm_clock = int(stepper_mcu.print_time_to_clock(
        max(lmt, est_now + BUFFER_TIME_START)
    ))

    logging.info(
        "[probe-homing] post-enable: est_now=%.6f arm_clock=%d",
        est_now, arm_clock,
    )

    self.active_homing_arms.add(arm_id)
    self.bridge.register_homing_dispatch(arm_id, None)
    self.bridge._software_trip_active = True

    bridge_lmt_before = self.bridge.get_last_move_time()
    try:
        self.bridge.endstop_arm(
            mcu_handle, queue, arm_id, arm_clock,
            [source], stepper_oids,
        )
        self.bridge._homing_print_time_base = bridge_lmt_before

        # Resolve Beacon MCU handle and trsync OID.
        # homing.py stashes self.endstops on the toolhead before
        # calling drip_move. Find the endstop whose MCU differs
        # from the stepper MCU — that's the external probe.
        beacon_mcu = None
        beacon_trsync_oid = 0
        for mcu_endstop, name in getattr(self, '_homing_endstops', []):
            es_mcu = mcu_endstop.get_mcu()
            if es_mcu != stepper_mcu:
                beacon_mcu = es_mcu
                # BeaconEndstopWrapper._shared._trsync is the
                # MCU_trsync on the Beacon MCU.
                beacon_trsync_oid = (
                    mcu_endstop._shared._trsync.get_oid()
                )
                break

        if beacon_mcu is None or beacon_mcu._bridge_handle is None:
            raise self.printer.command_error(
                "Cannot resolve Beacon MCU for probe homing"
            )

        # Compute sensor-fault timeout.
        nominal_dist = abs(pos3[2] - self.commanded_pos[2])
        axis_rails = self.kin._axis_rails()
        z_rail = axis_rails.get(2)
        if z_rail is not None:
            z_min, z_max = z_rail.get_range()
            actual_range = abs(z_max - z_min)
            move_dist = min(nominal_dist, actual_range)
        else:
            move_dist = nominal_dist
        sensor_fault_timeout = move_dist / max(speed, 0.1) + 5.0

        logging.info(
            "[probe-homing] calling run_probe_homing: "
            "beacon_handle=%s trsync_oid=%d stepper_handle=%s "
            "arm_id=%d speed=%.1f sensor_fault_timeout=%.1f",
            beacon_mcu._bridge_handle, beacon_trsync_oid,
            mcu_handle, arm_id, speed, sensor_fault_timeout,
        )

        result = self.bridge.run_probe_homing(
            beacon_mcu._bridge_handle,
            beacon_trsync_oid,
            mcu_handle,
            arm_id,
            pos3,
            speed,
            sensor_fault_timeout,
            stepper_oids,
        )

        PROBE_TRIGGERED = 0
        SEGMENT_RETIRED = 1
        SENSOR_FAULT = 2
        DEADLINE_EXPIRED = 3

        if result == SENSOR_FAULT:
            raise self.printer.command_error(
                "Probe sensor fault: no trigger after %.1fmm "
                "of Z travel (%.1fs). Check probe wiring and "
                "threshold." % (move_dist, sensor_fault_timeout)
            )
        if result == DEADLINE_EXPIRED:
            raise self.printer.command_error(
                "Homing deadline expired: MCU dead-man switch "
                "fired (host extension loop may have stalled)"
            )

        self.bridge.wait_moves()
        bridge_lmt_after = self.bridge.get_last_move_time()
        duration = bridge_lmt_after - bridge_lmt_before
        self._bump_pending_end_time(duration)
    finally:
        self.bridge._software_trip_active = False
        self.active_homing_arms.discard(arm_id)
        self.bridge.unregister_homing_dispatch(arm_id)
        try:
            self.bridge.endstop_disarm(mcu_handle, queue, arm_id)
        except Exception:
            pass
```

- [ ] **Step 2: Stash endstops on toolhead from homing.py**

In `klippy/extras/homing.py`, in `homing_move()` (around line 144, before the `drip_move` call), add:

```python
self.toolhead._homing_endstops = self.endstops
```

And in the finally/cleanup path (after `drip_move` returns or errors), clear it:

```python
self.toolhead._homing_endstops = []
```

This makes the `(mcu_endstop, name)` list available to `_drip_move_software_trip`.

- [ ] **Step 3: Verify the Beacon trsync OID resolution path**

The Beacon endstop is a `BeaconEndstopWrapper`. Its trsync is at:
```
mcu_endstop._shared._trsync  →  MCU_trsync instance
mcu_endstop._shared._trsync.get_oid()  →  trsync OID (u8)
mcu_endstop.get_mcu()  →  Beacon MCU (has ._bridge_handle)
```

The Python code in Step 1 resolves this via:
```python
for mcu_endstop, name in self._homing_endstops:
    es_mcu = mcu_endstop.get_mcu()
    if es_mcu != stepper_mcu:
        beacon_mcu = es_mcu
        beacon_trsync_oid = mcu_endstop._shared._trsync.get_oid()
        break
```

This finds the endstop whose MCU differs from the stepper MCU (i.e., the external probe).

- [ ] **Step 4: Build and basic smoke test**

Run on the dev machine:
```bash
cd rust && cargo build -p motion-bridge 2>&1 | tail -5
```
Expected: compiles

Then verify Python syntax:
```bash
python3 -c "import ast; ast.parse(open('klippy/motion_toolhead.py').read())"
```
Expected: no syntax errors

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "feat: simplify _drip_move_software_trip to use run_probe_homing"
```

---

### Task 6: Integration test with klipper-sim

**Files:**
- No new files — uses existing sim infrastructure

- [ ] **Step 1: Run the existing beacon Z homing sim test**

The repo has a beacon Z homing simulation test. Run it to verify the new code path works end-to-end:

```bash
cd tools/sim && python3 run_homing_test.py --test beacon_z 2>&1 | tail -30
```

If this test doesn't exist or uses a different invocation, check:
```bash
ls tools/sim/*.py
grep -r 'beacon.*hom\|homing.*beacon\|G28.*Z' tools/sim/
```

- [ ] **Step 2: Check log output for the new Rust path**

In the sim output or log, look for:
- `[probe-homing] pre-enable:` — Python preamble ran
- `[probe-homing] post-enable:` — TMC callbacks completed
- `[probe-homing] calling run_probe_homing:` — handoff to Rust
- `[probe-homing] probe triggered elapsed=` — Rust loop detected trigger

If the old `[homing-loop]` log lines appear instead, the code path didn't switch — the `_drip_move_software_trip` changes didn't take effect.

- [ ] **Step 3: Commit test results or fixes**

If any adjustments were needed (attribute paths, trsync OID resolution, etc.), commit them:

```bash
git add -u
git commit -m "fix: integration adjustments for run_probe_homing"
```

---

### Task 7: Bench test on Trident

**Files:**
- No code changes — live hardware verification

**Prerequisites:** Commit, push, pull on Pi, compile. Follow the standard bench flow:
`commit → push → pull → make clean → compile host + MCU → flash`

- [ ] **Step 1: Flash and start**

Follow the flashing-trident-mcus skill or the standard bench firmware flow. Start klippy and verify it connects.

- [ ] **Step 2: Home X and Y**

From the console, issue `G28 X` then `G28 Y`. Verify both succeed (sensorless homing, bridge-native path — should be unaffected).

- [ ] **Step 3: Home Z with Beacon**

**Get explicit user permission before issuing G28 Z.** This is a motion command on live hardware.

After permission: issue `G28 Z` and observe:
- Z should move down at homing speed
- Beacon should trigger → Z stops within ~1mm of contact
- No crash, no pull-the-plug
- klippy.log should show `[probe-homing] probe triggered`

- [ ] **Step 4: Verify log output**

Pull klippy.log and check for the full probe-homing sequence:
```
[probe-homing] pre-enable: ...
[probe-homing] post-enable: ...
[probe-homing] calling run_probe_homing: ...
[probe-homing] probe triggered elapsed=...
```

Verify no `[homing-loop]` entries (old Python path should be gone).
