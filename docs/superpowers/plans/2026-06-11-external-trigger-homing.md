# External-Trigger Homing (Spec B) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Homing/probing moves triggered by a non-GPIO source (Beacon-class probe firing a stock trsync on its own MCU), per `docs/kalico-rewrite/external-trigger-homing.md`.

**Architecture:** A Rust reactor interceptor on the probe MCU's RX thread translates terminal `trsync_state` reports into the existing `handle_endstop_trip` flow (extracted into a `TripDeps` context so a closure can call it). The firmware endstop latch becomes queryable (`tripped` + `trip_clock` in `endstop_state`); Python cross-checks latch vs doorbell after every GPIO trip and diagnoses lost doorbells. A `RemoteBridgeEndstop` Python class and two provider-contract hooks (`measured_trip_position`, `disarm`) complete the provider surface. No MCU-side deadman is added — the pump drip cohort already bounds MCU authority.

**Tech Stack:** Rust (pyo3 bridge in `rust/motion-bridge`, reactor in `rust/kalico-host-rt`), C firmware (`src/endstop.c`), Python klippy host, pytest (`test/`), `cargo nextest`, kalico-sim.

**Verification commands:**
- Rust: `cd rust && cargo nextest run` (all crates) — scope with `-p motion-bridge`.
- Python: `python3 -m pytest test/<file> -v` from repo root.
- Format gate before any PR: `cd rust && cargo fmt --all --check`.
- Sim (Docker): `./tools/kalico-sim/run.sh --probe-test remote` (unknown args pass through run.sh to runner.py as EXTRA_ARGS).

**Wire-format warning:** Tasks 1 changes a klipper-msgproto response format shared by H7 and F446. Host templates and firmware must change in the same commit, and on the bench BOTH MCUs must be reflashed together (`make clean` between builds, per repo convention).

---

### Task 1: Firmware trip latch — `tripped` + `trip_clock` in `endstop_state`

**Files:**
- Modify: `src/endstop.c`
- Modify: `klippy/bridge_endstop.py` (response template + `query_trip_state()`)
- Test: `test/test_bridge_endstop.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_bridge_endstop.py`. The existing `FakeMcu.state_cmd` response dict gains the new fields (update the existing line, shown below), and two new tests cover `query_trip_state`:

```python
# In FakeMcu.__init__, replace the existing state_cmd line with:
        self.state_cmd = FakeCommand(
            {"oid": 0, "armed": 0, "pin_value": 0, "tripped": 0, "trip_clock": 0}
        )
```

```python
def test_query_trip_state_not_tripped():
    mcu = FakeMcu()
    es = BridgeEndstop(_pin_params(mcu), 7)
    for cb in mcu.config_callbacks:
        cb()
    assert es.query_trip_state() == {"tripped": False, "trip_clock": 0}


def test_query_trip_state_tripped_returns_latched_clock():
    mcu = FakeMcu()
    es = BridgeEndstop(_pin_params(mcu), 7)
    for cb in mcu.config_callbacks:
        cb()
    mcu.state_cmd.response = {
        "oid": 0,
        "armed": 0,
        "pin_value": 1,
        "tripped": 1,
        "trip_clock": 0xDEADBEEF,
    }
    assert es.query_trip_state() == {
        "tripped": True,
        "trip_clock": 0xDEADBEEF,
    }
```

Use the existing `_pin_params` helper already defined in the file.

- [ ] **Step 2: Run tests to verify the new ones fail**

Run: `python3 -m pytest test/test_bridge_endstop.py -v`
Expected: the two new tests FAIL with `AttributeError: 'BridgeEndstop' object has no attribute 'query_trip_state'`; all pre-existing tests PASS.

- [ ] **Step 3: Implement firmware latch in `src/endstop.c`**

Three edits:

(a) Add a `tripped` field to the struct (after `trip_pending`):

```c
struct endstop {
    struct timer time;
    uint32_t rest_ticks;
    uint32_t pin_id;
    struct gpio_in pin;
    uint64_t trip_clock;
    uint8_t endstop_id;
    uint8_t invert;
    uint8_t armed;
    uint8_t trip_pending;
    uint8_t tripped;
};
```

(b) In `endstop_event` (the IRQ timer), set the latch where `trip_clock` is captured:

```c
    if (active && e->armed) {
        e->trip_clock = kalico_runtime_now_ticks(runtime_handle);
        e->armed = 0;
        e->trip_pending = 1;
        e->tripped = 1;
        sched_wake_task(&endstop_trip_wake);
        return SF_DONE;
    }
```

(c) Clear the latch on arm in `command_query_endstop` (the latch persists until the *next* arm), initialize it in `command_config_endstop`, and extend the query response in `command_endstop_query_state`:

```c
void
command_config_endstop(uint32_t *args)
{
    /* ... existing body unchanged, add after e->trip_pending = 0; */
    e->tripped = 0;
    e->trip_clock = 0;
```

```c
void
command_query_endstop(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    sched_del_timer(&e->time);
    e->rest_ticks = args[1];
    if (!e->rest_ticks) {
        e->armed = 0;
        return;
    }
    e->tripped = 0;
    e->trip_clock = 0;
    e->armed = 1;
    e->time.waketime = timer_read_time() + e->rest_ticks;
    sched_add_timer(&e->time);
}
```

```c
void
command_endstop_query_state(uint32_t *args)
{
    struct endstop *e = oid_lookup(args[0], command_config_endstop);
    uint8_t raw = gpio_in_read(e->pin) ? 1 : 0;
    sendf("endstop_state oid=%c armed=%c pin_value=%c tripped=%c"
          " trip_clock=%u",
          args[0], e->armed, raw, e->tripped, (uint32_t)e->trip_clock);
}
DECL_COMMAND(command_endstop_query_state, "endstop_query_state oid=%c");
```

`trip_clock` is the low 32 bits of the latched 64-bit tick count; the host compares it against the low 32 bits of the doorbell clock (Task 3) — no expansion needed.

- [ ] **Step 4: Update `klippy/bridge_endstop.py` template and accessor**

In `_build_config`, the response template must match the new firmware format exactly:

```python
        self._state_cmd = self.mcu.lookup_query_command(
            "endstop_query_state oid=%c",
            "endstop_state oid=%c armed=%c pin_value=%c tripped=%c"
            " trip_clock=%u",
            oid=self.oid,
        )
```

Add after `is_triggered`:

```python
    def query_trip_state(self):
        params = self._state_cmd.send([self.oid])
        return {
            "tripped": bool(params["tripped"]),
            "trip_clock": params["trip_clock"],
        }
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `python3 -m pytest test/test_bridge_endstop.py -v`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add src/endstop.c klippy/bridge_endstop.py test/test_bridge_endstop.py
git commit -m "feat(endstop): queryable trip latch — tripped + trip_clock in endstop_state"
```

---

### Task 2: Plumb the doorbell `trip_clock` through the homing result

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (lines 33, 484, 3257, ~3309, ~3683-3724)
- Modify: `klippy/extras/homing.py:348`

The homing result tuple `([f64; 3], [f64; 3])` becomes `([f64; 3], [f64; 3], u64)` — (trip_pos, final_pos, trip_clock). The doorbell clock is what `handle_endstop_trip` received; Python needs it for the latch cross-check.

- [ ] **Step 1: Change the Rust types and let the compiler find every site**

Four declared sites (plus any the compiler finds):

`bridge.rs:33` (HomingRun):
```rust
    notify: crossbeam_channel::Sender<Result<([f64; 3], [f64; 3], u64), String>>,
```

`bridge.rs:484`:
```rust
    homing_result:
        Mutex<Option<crossbeam_channel::Receiver<Result<([f64; 3], [f64; 3], u64), String>>>>,
```

`bridge.rs:3257` (in `home_axis_start`):
```rust
        let (result_tx, result_rx) =
            crossbeam_channel::bounded::<Result<([f64; 3], [f64; 3], u64), String>>(1);
```

In `handle_endstop_trip`'s worker thread (~line 3683), include the clock in the outcome:
```rust
                let outcome = reconstruct_cartesian(run.endstop_mcu, trip_clock).and_then(|trip| {
                    reconstruct_cartesian(axis_key.mcu_id, discard_clock)
                        .map(|final_pos| (trip, final_pos, trip_clock))
                });
```
The `ResumeStream` `and_then` block below it passes `positions` through unchanged — only its type changes; no edit needed beyond what the compiler demands.

`home_axis_poll` (~3309):
```rust
    fn home_axis_poll(&self) -> PyResult<Option<([f64; 3], [f64; 3], u64)>> {
            /* ... */
            Ok(result) => {
                self.finish_homing();
                let (trip_pos, final_pos, trip_clock) =
                    result.map_err(PyRuntimeError::new_err)?;
                *self.commanded_pos.lock().unwrap_or_else(|p| p.into_inner()) = final_pos;
                Ok(Some((trip_pos, final_pos, trip_clock)))
            }
```

- [ ] **Step 2: Build and run the Rust suite**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: compile errors first guide remaining tuple sites (fix them the same way — `home_abort`'s `Err(String)` sends are unaffected); then all tests PASS. If `bridge/tests.rs` constructs the old tuple, extend those constructions with a `0u64` clock.

- [ ] **Step 3: Update the Python unpack site**

`klippy/extras/homing.py:348` in `trip_move`:

```python
        trip_pos, final_pos, trip_clock = result
```

`trip_move`'s return value stays `(trip_pos, final_pos)` — `trip_clock` is consumed in Task 3. To keep this commit green, add the verification call now as a no-op placeholder is NOT allowed; instead just leave `trip_clock` unused (named, not `_`) — Task 3 wires it immediately after.

- [ ] **Step 4: Run the Python suite**

Run: `python3 -m pytest test/ -x -q`
Expected: PASS (no test drives `trip_move` end-to-end today).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs klippy/extras/homing.py
git commit -m "feat(bridge): return doorbell trip_clock from home_axis_poll"
```

---

### Task 3: Latch cross-check + doorbell-lost diagnosis in `trip_move`

**Files:**
- Modify: `klippy/extras/homing.py`
- Test: `test/test_homing_trip_verify.py` (create)

Two module-level helpers (unit-testable without a `Homing` instance, matching the existing helper style in `homing.py`), called from `trip_move`. They apply only to endstops exposing `query_trip_state` (GPIO-backed `BridgeEndstop`); remote endstops (Task 6) don't have it and are skipped by construction.

- [ ] **Step 1: Write the failing tests**

Create `test/test_homing_trip_verify.py`:

```python
import pytest

from klippy.extras.homing import (
    _no_trigger_error_message,
    _verify_latched_trip,
)


class FakeGcmd:
    def error(self, msg):
        return RuntimeError(msg)


class FakeLatchEndstop:
    def __init__(self, tripped, trip_clock):
        self._state = {"tripped": tripped, "trip_clock": trip_clock}

    def query_trip_state(self):
        return dict(self._state)


class FakeRemoteEndstop:
    pass


def test_verify_passes_on_matching_low32():
    es = FakeLatchEndstop(True, 0xDEADBEEF)
    _verify_latched_trip(FakeGcmd(), 2, es, 0x1_DEAD_BEEF)


def test_verify_raises_on_clock_mismatch():
    es = FakeLatchEndstop(True, 0x1111)
    with pytest.raises(RuntimeError, match="latch/doorbell clock mismatch"):
        _verify_latched_trip(FakeGcmd(), 2, es, 0x2222)


def test_verify_raises_when_latch_not_tripped():
    es = FakeLatchEndstop(False, 0)
    with pytest.raises(RuntimeError, match="latch shows no trip"):
        _verify_latched_trip(FakeGcmd(), 2, es, 0x2222)


def test_verify_skips_endstops_without_latch():
    _verify_latched_trip(FakeGcmd(), 2, FakeRemoteEndstop(), 0x2222)


def test_no_trigger_message_plain():
    msg = _no_trigger_error_message(2, FakeLatchEndstop(False, 0), 40.0)
    assert "did not trigger within 40.0mm" in msg
    assert "doorbell" not in msg


def test_no_trigger_message_reports_lost_doorbell():
    msg = _no_trigger_error_message(2, FakeLatchEndstop(True, 1234), 40.0)
    assert "trip event was lost" in msg
    assert "1234" in msg


def test_no_trigger_message_remote_endstop():
    msg = _no_trigger_error_message(2, FakeRemoteEndstop(), 40.0)
    assert "did not trigger within 40.0mm" in msg
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_homing_trip_verify.py -v`
Expected: FAIL with `ImportError: cannot import name '_no_trigger_error_message'`.

- [ ] **Step 3: Implement the helpers and wire them into `trip_move`**

Add to `klippy/extras/homing.py` (module level, after the existing helpers):

```python
def _verify_latched_trip(gcmd, axis, endstop, doorbell_clock):
    query = getattr(endstop, "query_trip_state", None)
    if query is None:
        return
    latch = query()
    if not latch["tripped"]:
        raise gcmd.error(
            "%s endstop: doorbell event arrived but the MCU latch shows no"
            " trip — duplicate or stale trip event" % ("XYZ"[axis],)
        )
    if latch["trip_clock"] != (doorbell_clock & 0xFFFFFFFF):
        raise gcmd.error(
            "%s endstop: latch/doorbell clock mismatch — latch=%d"
            " doorbell_low32=%d" % (
                "XYZ"[axis],
                latch["trip_clock"],
                doorbell_clock & 0xFFFFFFFF,
            )
        )


def _no_trigger_error_message(axis, endstop, max_travel):
    base = "%s endstop did not trigger within %.1fmm of travel" % (
        "XYZ"[axis],
        max_travel,
    )
    query = getattr(endstop, "query_trip_state", None)
    if query is None:
        return base
    latch = query()
    if latch["tripped"]:
        return (
            "%s endstop tripped (latched clock %d) but the trip event was"
            " lost — doorbell never reached the host"
            % ("XYZ"[axis], latch["trip_clock"])
        )
    return base
```

In `trip_move`, replace the deadline-expiry raise:

```python
                if reactor.monotonic() > deadline:
                    bridge.home_abort()
                    raise gcmd.error(
                        _no_trigger_error_message(axis, endstop, max_travel)
                    )
```

and after the result unpack (`trip_pos, final_pos, trip_clock = result`), before the no-movement check:

```python
        _verify_latched_trip(gcmd, axis, endstop, trip_clock)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_homing_trip_verify.py test/test_bridge_endstop.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/homing.py test/test_homing_trip_verify.py
git commit -m "feat(homing): latch/doorbell cross-check and lost-doorbell diagnosis"
```

---

### Task 4: Extract `TripDeps` so a closure can dispatch a trip (pure refactor)

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

`handle_endstop_trip(&self, ...)` (bridge.rs:3560) only touches `Arc`/`Mutex` fields. Extract its state into a cloneable context so the Task 5/6 interceptor closure (which cannot hold `&self`) can call it. Two fields need an `Arc` wrapper added.

- [ ] **Step 1: Wrap two fields in `Arc`**

In `struct PyMotionBridge` (bridge.rs:458):

```rust
    mcu_axis_configs: Arc<Mutex<Vec<McuAxisConfig>>>,
    /* ... */
    pump_tx: Arc<Mutex<Option<std::sync::mpsc::Sender<crate::pump::PumpMsg>>>>,
```

Update the constructor (`bridge.rs:765` region) to `Arc::new(Mutex::new(...))`. All existing `self.mcu_axis_configs.lock()` / `self.pump_tx.lock()` call sites compile unchanged (`Arc<Mutex<T>>` derefs to `Mutex<T>`).

- [ ] **Step 2: Define `TripDeps` and extract the function**

Above `impl PyMotionBridge` containing `handle_endstop_trip`:

```rust
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
```

Move the entire body of `handle_endstop_trip` into a free function, replacing each `self.<field>` access with `deps.<field>` (the body also reads `self.active_drip_cohort`, `self.pump_tx`, `self.mcus`, `self.router`, `self.motion_history`, `self.mcu_axis_configs` — all present on `TripDeps`):

```rust
pub(crate) fn dispatch_endstop_trip(
    deps: &TripDeps,
    event_mcu: u32,
    endstop_id: u8,
    trip_clock: u64,
) {
    /* moved body of handle_endstop_trip, self.* → deps.* */
}
```

Keep the method as a delegation so the existing `take_runtime_event` call site (bridge.rs:2142) is untouched:

```rust
    fn handle_endstop_trip(&self, event_mcu: u32, endstop_id: u8, trip_clock: u64) {
        dispatch_endstop_trip(&self.trip_deps(), event_mcu, endstop_id, trip_clock);
    }
```

Note: inside the moved body, `self.finish_homing()` is NOT called (it never was — only `home_axis_start`'s error paths call it); the body's only `self` uses are the seven fields above. If the compiler reveals another `self` use, stop and re-read it before adapting — do not paper over.

- [ ] **Step 3: Build and run the full Rust suite**

Run: `cd rust && cargo nextest run`
Expected: all PASS (behavior-preserving refactor).

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "refactor(bridge): extract dispatch_endstop_trip + TripDeps from handle_endstop_trip"
```

---

### Task 5: Remote-trigger relay decision logic (pure functions + tests)

**Files:**
- Create: `rust/motion-bridge/src/remote_trigger.rs`
- Create: `rust/motion-bridge/src/remote_trigger/tests.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (add `pub mod remote_trigger;` alongside the existing module list)

The interceptor closure's decision logic, kept free of I/O so it unit-tests cleanly. Module layout follows the existing convention (`pump.rs` + `pump/`, `homing.rs` + `homing/`).

- [ ] **Step 1: Write the failing tests**

Create `rust/motion-bridge/src/remote_trigger/tests.rs`:

```rust
use super::{relay_decision, relay_trip_clock, RelayAction};

#[test]
fn non_terminal_report_is_ignored() {
    assert_eq!(relay_decision(Some(1), false), RelayAction::Ignore);
}

#[test]
fn terminal_report_fires() {
    assert_eq!(relay_decision(Some(0), false), RelayAction::Fire);
}

#[test]
fn second_terminal_report_is_ignored() {
    assert_eq!(relay_decision(Some(0), true), RelayAction::Ignore);
}

#[test]
fn malformed_report_without_can_trigger_is_ignored() {
    assert_eq!(relay_decision(None, false), RelayAction::Ignore);
}

#[test]
fn nonzero_report_clock_expands_against_reference() {
    // reference 0x1_0000_1000, clock32 just below the low-32 reference:
    // small negative delta, same epoch.
    assert_eq!(relay_trip_clock(0x0000_0F00, 0x1_0000_1000), 0x1_0000_0F00);
}

#[test]
fn clock32_ahead_of_reference_expands_forward() {
    assert_eq!(relay_trip_clock(0x0000_2000, 0x1_0000_1000), 0x1_0000_2000);
}

#[test]
fn expansion_handles_wrap_boundary() {
    // reference just past a 32-bit wrap; clock32 from just before it.
    assert_eq!(
        relay_trip_clock(0xFFFF_FF00, 0x2_0000_0010),
        0x1_FFFF_FF00
    );
}

#[test]
fn zero_clock_means_host_commanded_trigger_substitute_reference() {
    // trsync_trigger path reports clock=0 (trsync.c:176); substitute the
    // router's current estimate.
    assert_eq!(relay_trip_clock(0, 0x1_0000_1000), 0x1_0000_1000);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(remote_trigger)'`
Expected: compile FAILURE (module does not exist).

- [ ] **Step 3: Implement the module**

Create `rust/motion-bridge/src/remote_trigger.rs`:

```rust
//! Decision logic for the remote-trigger relay: a reactor interceptor on a
//! probe MCU's RX thread translating terminal `trsync_state` reports into
//! the bridge's endstop-trip dispatch. Kept free of I/O for testability;
//! the interceptor closure lives in bridge.rs.

#[derive(Debug, PartialEq, Eq)]
pub enum RelayAction {
    Fire,
    Ignore,
}

pub fn relay_decision(can_trigger: Option<u32>, already_fired: bool) -> RelayAction {
    match can_trigger {
        Some(0) if !already_fired => RelayAction::Fire,
        _ => RelayAction::Ignore,
    }
}

/// `trsync_state.clock` is a report-time clock, not a trip timestamp
/// (`trsync.c:190`), and the host-commanded `trsync_trigger` path sends 0
/// (`trsync.c:176`). Expand a nonzero report clock to 64 bits against the
/// router's current clock estimate for the probe MCU; substitute that
/// estimate outright for the zero case. The result is provisional-only —
/// precise trigger timestamps come from the probe's own latched record.
pub fn relay_trip_clock(clock32: u32, reference_clock64: u64) -> u64 {
    if clock32 == 0 {
        return reference_clock64;
    }
    let delta = clock32.wrapping_sub(reference_clock64 as u32) as i32 as i64;
    reference_clock64.wrapping_add(delta as u64)
}

#[cfg(test)]
mod tests;
```

Add to `rust/motion-bridge/src/lib.rs`, in the module list:

```rust
pub mod remote_trigger;
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(remote_trigger)'`
Expected: 8 PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/remote_trigger.rs rust/motion-bridge/src/remote_trigger/tests.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(bridge): remote-trigger relay decision logic"
```

---

### Task 6: `arm_remote_trigger` / `disarm_remote_trigger` bridge API

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (new field + two pymethods)
- Modify: `klippy/motion_bridge.py` (wrappers)

- [ ] **Step 1: Add the registration map field**

In `struct PyMotionBridge`:

```rust
    remote_triggers:
        Mutex<HashMap<u8, (u32, kalico_host_rt::host_io::InterceptorId)>>,
```

Initialize in the constructor: `remote_triggers: Mutex::new(HashMap::new()),`.

- [ ] **Step 2: Implement the pymethods**

Add to the `#[pymethods] impl PyMotionBridge` block that contains `home_axis_start` (terminal trsync semantics: `can_trigger == 0` is the terminal marker; any terminal reason — trigger, comms timeout, host abort — must stop motion, and reason discrimination stays in Python, which still receives the same `trsync_state` via the unchanged passthrough path):

```rust
    fn arm_remote_trigger(
        &self,
        mcu_handle: u32,
        trsync_oid: u32,
        endstop_id: u8,
    ) -> PyResult<()> {
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
                        .compute_ack_clock(
                            kalico_host_rt::passthrough_queue::McuHandle::from_raw(
                                mcu_handle,
                            ),
                        )
                        .unwrap_or(0);
                    let clock64 = crate::remote_trigger::relay_trip_clock(
                        clock32, reference,
                    );
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
                PyRuntimeError::new_err(format!(
                    "disarm_remote_trigger: unregister failed: {e:?}"
                ))
            }),
            // MCU detached: its reactor (and the interceptor with it) is
            // already gone. Disarm runs on cleanup paths — don't mask the
            // original error.
            None => Ok(()),
        }
    }
```

Imports needed at the top of bridge.rs (check which already exist): `std::sync::atomic::Ordering` is already imported (used by `dispatched_segments`); `McuHandle` is referenced by full path above so no new import is required. Note: if the router import path differs (check how `motion_history.rs` names `PassthroughRouter`), use the same path style as the existing code.

`McuHandle::from_raw` exists (`router.rs:24`). `compute_ack_clock` returns `Ok(0)` when the clock is unsynced — `relay_trip_clock(clock32, 0)` then produces a near-zero clock64, which fails `reconstruct_axis_position`'s window bounds check and surfaces as a loud error through the homing result channel, *after* motion is stopped. That is the spec's intended fail-loudly path: stop unconditionally, then error.

- [ ] **Step 3: Build and test**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: compiles, all PASS.

- [ ] **Step 4: Add the Python wrappers**

In `klippy/motion_bridge.py`, next to `home_axis_poll` (line ~432):

```python
    def arm_remote_trigger(self, mcu_handle, trsync_oid, endstop_id):
        return self._bridge.arm_remote_trigger(
            mcu_handle, trsync_oid, endstop_id
        )

    def disarm_remote_trigger(self, endstop_id):
        return self._bridge.disarm_remote_trigger(endstop_id)
```

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs klippy/motion_bridge.py
git commit -m "feat(bridge): arm/disarm_remote_trigger — trsync_state relay via reactor interceptor"
```

---

### Task 7: `RemoteBridgeEndstop` + disarm wiring in `trip_move`

**Files:**
- Modify: `klippy/bridge_endstop.py`
- Modify: `klippy/extras/homing.py` (`trip_move` finally block)
- Test: `test/test_bridge_endstop.py`

- [ ] **Step 1: Write the failing tests**

Append to `test/test_bridge_endstop.py`:

```python
from klippy.bridge_endstop import RemoteBridgeEndstop


class FakeBridge:
    def __init__(self):
        self.calls = []

    def arm_remote_trigger(self, mcu_handle, trsync_oid, endstop_id):
        self.calls.append(("arm", mcu_handle, trsync_oid, endstop_id))

    def disarm_remote_trigger(self, endstop_id):
        self.calls.append(("disarm", endstop_id))


class FakeRemoteMcu:
    _bridge_handle = 42


def _remote_setup():
    printer = FakePrinter()
    bridge = FakeBridge()
    printer.add_object("motion_bridge", bridge)
    es = RemoteBridgeEndstop(printer, FakeRemoteMcu(), trsync_oid=9)
    return bridge, es


def test_remote_endstop_allocates_provider_id():
    _, es = _remote_setup()
    assert es.endstop_id >= PROVIDER_ID_FIRST


def test_remote_endstop_arm_and_disarm_delegate_to_bridge():
    bridge, es = _remote_setup()
    es.arm(0.001)
    es.disarm()
    assert bridge.calls == [
        ("arm", 42, 9, es.endstop_id),
        ("disarm", es.endstop_id),
    ]


def test_remote_endstop_default_query_state():
    _, es = _remote_setup()
    assert es.is_triggered() is False
    assert es.query_endstop(0.0) is False
    assert es.bridge_mcu_handle() == 42
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_bridge_endstop.py -v`
Expected: new tests FAIL with `ImportError: cannot import name 'RemoteBridgeEndstop'`.

- [ ] **Step 3: Implement `RemoteBridgeEndstop`**

Append to `klippy/bridge_endstop.py` (before `_ProviderIdAllocator`):

```python
class RemoteBridgeEndstop:
    """Endstop whose trigger is a trsync on a non-bridge-driven MCU (e.g. a
    Beacon-class probe). Arming registers a Rust-side relay that translates
    the trsync's terminal report into a bridge endstop trip; the device-side
    arming dance (trsync_start, heartbeats, probe commands) is the
    provider's job, via trip_move_begin/trip_move_end."""

    def __init__(self, printer, mcu, trsync_oid):
        # Constructed at provider config-load time, possibly before
        # motion_bridge exists — look the bridge up lazily at arm time.
        self._printer = printer
        self.mcu = mcu
        self.trsync_oid = trsync_oid
        self.endstop_id = allocate_provider_id(printer)

    def bridge_mcu_handle(self):
        return getattr(self.mcu, "_bridge_handle", None)

    def is_triggered(self):
        return False

    def arm(self, poll_period):
        del poll_period
        bridge = self._printer.lookup_object("motion_bridge")
        bridge.arm_remote_trigger(
            self.bridge_mcu_handle(), self.trsync_oid, self.endstop_id
        )

    def disarm(self):
        bridge = self._printer.lookup_object("motion_bridge")
        bridge.disarm_remote_trigger(self.endstop_id)

    def query_endstop(self, print_time):
        return False
```

- [ ] **Step 4: Wire disarm into `trip_move`'s cleanup**

In `klippy/extras/homing.py`, the `finally` block of `trip_move` becomes:

```python
        finally:
            disarm = getattr(endstop, "disarm", None)
            if disarm is not None:
                try:
                    disarm()
                except Exception:
                    logging.exception(
                        "trip_move: remote trigger disarm failed during unwind"
                    )
            if provider is not None and hasattr(provider, "trip_move_end"):
                provider.trip_move_end(entry)
```

(`disarm` failure must not mask a real homing error; the relay dies with the reactor anyway if the MCU is gone.)

- [ ] **Step 5: Run tests to verify they pass**

Run: `python3 -m pytest test/test_bridge_endstop.py test/test_homing_trip_verify.py -v`
Expected: all PASS.

- [ ] **Step 6: Commit**

```bash
git add klippy/bridge_endstop.py klippy/extras/homing.py test/test_bridge_endstop.py
git commit -m "feat(homing): RemoteBridgeEndstop provider variant with relay arm/disarm"
```

---

### Task 8: `measured_trip_position` provider hook

**Files:**
- Modify: `klippy/extras/homing.py` (`_home_axis`)
- Test: `test/test_homing_trip_verify.py`

The hook returns the toolhead's actual axis coordinate *at rest at final_pos* (eddy: post-home sample; contact: detect-time evaluation plus overshoot — both computed by the provider). When it returns a float, G28 sets the axis to it directly, replacing `trigger_height + overshoot`.

- [ ] **Step 1: Write the failing tests**

The position arithmetic lives in a new module-level helper so it tests without a toolhead. Append to `test/test_homing_trip_verify.py`:

```python
from klippy.extras.homing import _homed_axis_position


class FakeProviderNoHook:
    pass


class FakeProviderMeasures:
    def measured_trip_position(self, axis, trip_pos, final_pos):
        return 3.25


class FakeProviderDeclines:
    def measured_trip_position(self, axis, trip_pos, final_pos):
        return None


def test_homed_position_default_is_trigger_height_plus_overshoot():
    pos = _homed_axis_position(
        FakeProviderNoHook(), 2, [0, 0, 1.0], [0, 0, 0.9], 0.5
    )
    assert pos == pytest.approx(0.5 + (0.9 - 1.0))


def test_homed_position_none_provider_uses_default():
    pos = _homed_axis_position(None, 2, [0, 0, 1.0], [0, 0, 0.9], 0.5)
    assert pos == pytest.approx(0.4)


def test_homed_position_uses_provider_measurement():
    pos = _homed_axis_position(
        FakeProviderMeasures(), 2, [0, 0, 1.0], [0, 0, 0.9], 0.5
    )
    assert pos == 3.25


def test_homed_position_provider_declining_falls_back():
    pos = _homed_axis_position(
        FakeProviderDeclines(), 2, [0, 0, 1.0], [0, 0, 0.9], 0.5
    )
    assert pos == pytest.approx(0.4)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `python3 -m pytest test/test_homing_trip_verify.py -v`
Expected: FAIL with `ImportError: cannot import name '_homed_axis_position'`.

- [ ] **Step 3: Implement the helper and use it in `_home_axis`**

Module-level in `klippy/extras/homing.py`:

```python
def _homed_axis_position(provider, axis, trip_pos, final_pos, trigger_height):
    if provider is not None and hasattr(provider, "measured_trip_position"):
        measured = provider.measured_trip_position(axis, trip_pos, final_pos)
        if measured is not None:
            return measured
    return trigger_height + (final_pos[axis] - trip_pos[axis])
```

In `_home_axis`, replace the position-set block (currently `overshoot = ...; newpos[axis] = trigger_height + overshoot`):

```python
            newpos = list(toolhead.get_position())
            newpos[axis] = _homed_axis_position(
                entry["provider"], axis, trip_pos, final_pos, trigger_height
            )
            toolhead.set_position(newpos, homing_axes=[axis])
            logging.info(
                "homing: %s trigger=%.4f overshoot=%+.4f set %s=%.4f",
                "XYZ"[axis],
                trigger_height,
                final_pos[axis] - trip_pos[axis],
                "XYZ"[axis],
                newpos[axis],
            )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `python3 -m pytest test/test_homing_trip_verify.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/homing.py test/test_homing_trip_verify.py
git commit -m "feat(homing): measured_trip_position provider hook"
```

---

### Task 9: Delete dead `runtime_stop_on_trigger` scaffolding

**Files:**
- Modify: `klippy/mcu.py` (`MCU_trsync.start`, lines ~358-368)

The sender at `mcu.py:365-367` targets a firmware command that does not exist; `_bridge_arm_id` (read at line 358) is never assigned anywhere, so `start()` raises unconditionally today. The log_codes.rs entry `EVENT_ENDSTOP_STOP_CB_ENTER` stays — that table is wire-stable and must keep decoding historical logs.

- [ ] **Step 1: Delete the dead block**

In `MCU_trsync.start()`, remove:

```python
        arm_id = getattr(self, "_bridge_arm_id", None)
        if arm_id is None:
            raise self._mcu.error(
                "bridge MCU_trsync.start: _bridge_arm_id not set "
                "(homing glue must assign it before start)"
            )
        if self._steppers:
            serial.send(
                "runtime_stop_on_trigger arm_id=%d trsync_oid=%d"
                % (arm_id, self._oid)
            )
```

`start()` now sends `trsync_start` + `trsync_set_timeout` only — exactly what a probe-MCU trsync (the Spec D fork's `BeaconEndstopShared`) needs. Verify nothing else references the deleted names:

Run: `grep -rn "_bridge_arm_id\|runtime_stop_on_trigger" klippy/ rust/ src/ --include="*.py" --include="*.rs" --include="*.c"`
Expected: only the wire-stable `log_codes.rs:206` template string remains.

- [ ] **Step 2: Run the Python suite**

Run: `python3 -m pytest test/ -x -q`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add klippy/mcu.py
git commit -m "chore(mcu): delete dead runtime_stop_on_trigger scaffolding from MCU_trsync.start"
```

---

### Task 10: Sim end-to-end — synthetic remote-trsync provider + runner variant

**Files:**
- Create: `klippy/extras/sim_remote_endstop.py`
- Modify: `tools/kalico-sim/runner.py` (`PROBE_TEST_VARIANTS`, `_generate_probe_config`, `run_probe_test`)

The provider doubles as the reference implementation of the full Spec B contract for Spec D. It allocates a trsync oid on the (bridge) sim MCU — `config_trsync` is a harmless idle allocation there — arms it in `trip_move_begin`, schedules a `trsync_trigger reason=1` after a configurable descent delay (exercising the relay's clock=0 substitution path), verifies the terminal reason in `trip_move_end`, and returns a fixed `measured_trip_position` so the test can assert the override plumbing end to end.

- [ ] **Step 1: Write the provider extra**

Create `klippy/extras/sim_remote_endstop.py`:

```python
# Sim-only virtual-endstop provider exercising the Spec B remote-trigger
# contract end to end: RemoteBridgeEndstop arming, trsync relay, terminal
# reason verification, and the measured-position override. Reference
# implementation for external-probe providers (Spec D).
import logging

from klippy.bridge_endstop import RemoteBridgeEndstop

REASON_ENDSTOP_HIT = 1
REASON_COMMS_TIMEOUT = 4


class SimRemoteEndstop:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.trigger_delay = config.getfloat("trigger_delay", 1.0, above=0.0)
        self.measured_z = config.getfloat("measured_z", None)
        self.trigger_height = config.getfloat("trigger_height", 0.0)
        mcu_name = config.get("mcu", "mcu")
        self.mcu = self.printer.lookup_object(
            "mcu" if mcu_name == "mcu" else "mcu " + mcu_name
        )
        self.oid = self.mcu.create_oid()
        self._trsync_start_cmd = None
        self._trsync_trigger_cmd = None
        self._last_reason = None
        self._trigger_timer = None
        self.mcu.register_config_callback(self._build_config)
        self.mcu.register_response(
            self._handle_trsync_state, "trsync_state", self.oid
        )
        self._endstop = RemoteBridgeEndstop(
            self.printer, self.mcu, trsync_oid=self.oid
        )

    def _build_config(self):
        self.mcu.add_config_cmd("config_trsync oid=%d" % (self.oid,))
        self._trsync_start_cmd = self.mcu.lookup_command(
            "trsync_start oid=%c report_clock=%u report_ticks=%u"
            " expire_reason=%c"
        )
        self._trsync_trigger_cmd = self.mcu.lookup_command(
            "trsync_trigger oid=%c reason=%c"
        )

    def _handle_trsync_state(self, params):
        if not params["can_trigger"]:
            self._last_reason = params["trigger_reason"]

    # --- virtual endstop provider contract -------------------------------

    def setup_bridge_endstop(self, pin_params, axis):
        if pin_params["pin"] != "z_virtual_endstop" or axis != 2:
            raise self.printer.config_error(
                "sim_remote_endstop only provides z_virtual_endstop on Z"
            )
        if pin_params["invert"] or pin_params["pullup"]:
            raise self.printer.config_error(
                "Can not pullup/invert sim_remote_endstop virtual endstop"
            )
        return self._endstop

    def get_position_endstop(self):
        return self.trigger_height

    def trip_move_begin(self, entry):
        self._last_reason = None
        self._trsync_start_cmd.send([self.oid, 0, 0, REASON_COMMS_TIMEOUT])
        reactor = self.printer.get_reactor()
        self._trigger_timer = reactor.register_timer(
            self._fire_trigger, reactor.monotonic() + self.trigger_delay
        )

    def _fire_trigger(self, eventtime):
        logging.info("sim_remote_endstop: firing trsync_trigger")
        self._trsync_trigger_cmd.send([self.oid, REASON_ENDSTOP_HIT])
        return self.printer.get_reactor().NEVER

    def trip_move_end(self, entry):
        reactor = self.printer.get_reactor()
        if self._trigger_timer is not None:
            reactor.unregister_timer(self._trigger_timer)
            self._trigger_timer = None
        deadline = reactor.monotonic() + 2.0
        while self._last_reason is None:
            if reactor.monotonic() > deadline:
                raise self.printer.command_error(
                    "sim_remote_endstop: no terminal trsync_state received"
                )
            reactor.pause(reactor.monotonic() + 0.010)
        if self._last_reason != REASON_ENDSTOP_HIT:
            raise self.printer.command_error(
                "sim_remote_endstop: trsync terminated with reason %d"
                % (self._last_reason,)
            )

    def measured_trip_position(self, axis, trip_pos, final_pos):
        return self.measured_z


def load_config(config):
    return SimRemoteEndstop(config)
```

Check the exact `register_response` signature in `klippy/mcu.py` (`register_response(self, cb, msg, oid=None)`) and `config.getfloat` option style against a neighboring extra before assuming; adjust to match.

- [ ] **Step 2: Add the runner variant**

In `tools/kalico-sim/runner.py`:

(a) `PROBE_TEST_VARIANTS` (line ~1184): add `"remote"`.

(b) `_generate_probe_config` (line ~1200): add a branch and a remote section. In the variant ladder:

```python
    elif variant == "remote":
        z_endstop = "endstop_pin: sim_remote_endstop:z_virtual_endstop"
        probe_pin = "gpiochip0/gpio203"
```

and below `safe_z_section`:

```python
    remote_section = ""
    if variant == "remote":
        probe_section = ""
        remote_section = """
[sim_remote_endstop]
trigger_delay: 1.0
measured_z: 3.25
trigger_height: 0
"""
```

Append `{remote_section}` to the f-string right after `{safe_z_section}{probe_section}`.

(c) In `run_probe_test`, after the `boot-ready` check, add an early variant branch (before the `QUERY_PROBE` flow):

```python
                if variant == "remote":
                    offset = len(klippy_log.read_bytes())
                    resp = send_gcode(api_socket, "G28 Z", timeout=120)
                    out, offset = _log_tail_since(klippy_log, offset)
                    g28_err = (
                        resp.get("error")
                        if isinstance(resp, dict)
                        else None
                    )
                    check("remote-g28-z", not g28_err, g28_err or "G28 Z ok")
                    check(
                        "remote-relay-fired",
                        "remote trsync terminal report" in out
                        or "sim_remote_endstop: firing" in out,
                        "relay/trigger evidence in logs",
                    )
                    check(
                        "remote-measured-override",
                        "set Z=3.25" in out,
                        "homing log should show measured override 3.25",
                    )
                    resp = send_gcode(api_socket, "M114", timeout=30)
                    check(
                        "remote-final-position",
                        resp is not None,
                        "M114: %s" % (resp,),
                    )
                    raise SystemExit(_summarize(checks))
```

NOTE for the implementer: `run_probe_test` is a long function — read its tail to see how existing variants summarize and return (the `checks` list + return code); replicate that pattern instead of `raise SystemExit` if a `_summarize` helper does not exist. The Rust relay's `tracing::info!` line goes to the bridge log, not klippy.log — if `remote-relay-fired` can't see it in klippy.log, assert on the `sim_remote_endstop: firing` line plus successful G28 only. After Z homes with retract (`homing_retract_dist` defaults to 5.0), the expected M114 Z is `3.25 + 5.0`; assert `"Z:8.25" in str(resp)` if M114 formatting allows, otherwise drop to presence-check.

- [ ] **Step 3: Run the sim variant**

Run: `./tools/kalico-sim/run.sh --probe-test remote`
Expected: all CHECK lines PASS, exit 0. Iterate here — this is the integration test for Tasks 1-8; failures localize as: boot failure → config/extra bug; G28 hang → relay never fired (check interceptor registration / trsync oid); G28 error mentioning "latch" → cross-check firing on remote endstop (it must not — remote has no `query_trip_state`); wrong final Z → override plumbing.

- [ ] **Step 4: Run the GPIO regression variants**

Run: `./tools/kalico-sim/run.sh --probe-test virtual` and `./tools/kalico-sim/run.sh --probe-test gpio-z`
Expected: PASS — these now implicitly exercise the Task 1 latch + Task 3 cross-check on every GPIO trip.

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/sim_remote_endstop.py tools/kalico-sim/runner.py
git commit -m "test(sim): remote-trigger homing end-to-end variant"
```

---

### Task 11: Full-suite gate + format check

- [ ] **Step 1: Run everything**

```bash
cd rust && cargo nextest run && cargo test --doc && cargo fmt --all --check
cd .. && python3 -m pytest test/ -q
```
Expected: all PASS, fmt clean. Re-run `cargo fmt --all --check` after ANY late edit.

- [ ] **Step 2: Update the survey doc status line**

In `docs/kalico-rewrite/beacon-fork-survey.md`, the Spec B entry already links the design doc; no further edit unless implementation deviated from the spec — if it did, update `docs/kalico-rewrite/external-trigger-homing.md` to match reality and say so in the commit.

- [ ] **Step 3: Commit any stragglers and prepare the PR**

```bash
git status   # expect clean or only intended files
```
PR description: implements `docs/kalico-rewrite/external-trigger-homing.md` (Spec B); note the wire-format change in `endstop_state` requiring both bench MCUs to be reflashed together.

---

## Deviations from the spec (intentional, pre-approved by design review)

- The spec's "stale log-code template" deletion is narrowed: `log_codes.rs` is a wire-stable decode table and keeps the `EVENT_ENDSTOP_STOP_CB_ENTER` entry so historical logs still decode. Only the dead Python sender is deleted (Task 9).
- The spec's "doorbell-lost (emulator suppresses event)" sim scenario is covered at unit level (`test_no_trigger_message_reports_lost_doorbell`) — the GPIO event path can't be selectively suppressed in sim without a new shim feature. The terminal-error-reason scenario is covered by the provider's `trip_move_end` reason check; a dedicated sim variant for it is deferred to Spec E's emulator work.
- The sim happy path triggers via host-commanded `trsync_trigger` (clock=0 → substitution path). A realistic nonzero report clock arrives only from a device-fired trsync — that's Spec E with the beacon emulator. The nonzero-clock expansion is unit-tested in Task 5.
