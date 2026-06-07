# Trsync Cross-MCU Homing — Part A (Trip Relay) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make an endstop trip on any MCU participating in a homing move freeze the curve evaluator on *all* participating MCUs, via a host-side relay in the bridge reactor (the `trdispatch` analog) and a new firmware trsync signal.

**Architecture:** Detection (runtime `endstop.rs` / Beacon firmware) emits a trip *report* without firing the local trsync. A per-move `TripDispatch` in the bridge reactor listens for those reports (`kalico_endstop_tripped` and `trsync_state`) and broadcasts `trsync_trigger` to every participating bridge MCU's firmware trsync, which carries a new `runtime_stop_on_trigger` signal that calls `kalico_software_trip` → `AbortNow` → freeze. The detecting MCU's local "siren" (immediate self-freeze) is disabled with a marker comment so the relay is testable on a single board.

**Tech Stack:** C (MCU firmware), Rust (`kalico-host-rt`, `runtime`, `motion-bridge`, PyO3 FFI), Python (klippy). Verification: `cargo test` for Rust logic; Pi firmware build (`make clean` between H7/F446 per project rule); dual-MCU Renode sim (`tools/sim/dual_mcu_docker.resc`) for cross-MCU; single-board hardware bring-up last.

**Spec:** `docs/superpowers/specs/2026-05-31-trsync-cross-mcu-homing-design.md` (Part A; Part B is a separate plan).

**Scope note — Part A keeps the existing deadline net.** Part A does NOT delete the `extend_homing_deadline` machinery (that is Part B). Sensorless GPIO homing already submits the whole move + `wait_moves`; the relay's freeze retires the segment early. So Part A is safe and testable standalone.

---

## File structure

| File | Responsibility | Change |
|------|----------------|--------|
| `src/runtime_commands.c` | MCU command glue | **Add** `runtime_stop_on_trigger` command + `trsync_signal` callback → `kalico_software_trip`. |
| `rust/runtime/src/endstop.rs` | runtime endstop detection | **Disable** the local `AbortNow` siren on fresh GPIO trip (marker comment); keep trip-report queueing. |
| `rust/runtime/tests/endstop_siren_disabled.rs` | unit test | **Create** test proving detection reports without freezing. |
| `rust/motion-bridge/src/trip_dispatch.rs` | the relay | **Create** `TripDispatch` (sources → fan out `trsync_trigger` to sinks). |
| `rust/motion-bridge/src/trip_dispatch/tests.rs` | unit test | **Create** fan-out tests. |
| `rust/motion-bridge/src/bridge.rs` | PyO3 FFI | **Add** `trip_dispatch_prepare` / `_cleanup`; resolve mcu handles → `KalicoHostIo`. |
| `rust/motion-bridge/src/lib.rs` | module wiring | **Add** `mod trip_dispatch;`. |
| `klippy/mcu.py` | host trsync arming | Bridge `MCU_trsync` arms with `runtime_stop_on_trigger` (reverse the ceremonial no-op). |
| `klippy/motion_bridge.py` | host bridge wrappers | **Add** `trip_dispatch_prepare`/`_cleanup` wrappers. |
| `klippy/motion_toolhead.py` | homing glue | `drip_move` GPIO branch arms per-MCU trsyncs + `TripDispatch`, waits on completion. |
| `tools/test_renode_endstop_e2e.py` | sim integration | Extend with a cross-MCU relay assertion. |

---

## Task 1: Firmware — `runtime_stop_on_trigger` command + signal

**Files:**
- Modify: `src/runtime_commands.c` (beside `command_runtime_software_trip`, ~line 244)

- [ ] **Step 1: Add the include for trsync + container_of**

At the top of `src/runtime_commands.c`, with the other includes (after line 17), add:

```c
#include "trsync.h"               // trsync_add_signal, trsync_oid_lookup
#include "compiler.h"             // container_of
```

(If `compiler.h` is already transitively included via `command.h`, the second
line is a harmless duplicate; keep it explicit for legibility.)

- [ ] **Step 2: Add the signal struct + callback + command**

Insert immediately after `DECL_COMMAND(command_runtime_software_trip, ...)`
(after line 243), BEFORE `command_runtime_extend_homing_deadline`:

```c
// ---- runtime_stop_on_trigger: trsync signal that freezes the curve evaluator
//
// This is the bridge twin of stepper.c's stepper_stop_on_trigger. Where
// stepper_stop clears the (unused-in-bridge) C step queue, this freezes the
// curve evaluator via kalico_software_trip. The bridge reactor's TripDispatch
// relays `trsync_trigger` here; trsync_do_trigger fires this signal.
//
// One active homing arm per MCU at a time, so a single static instance is
// sufficient. (Multiple concurrent arms would need an array keyed by trsync.)
static struct runtime_stop_binding {
    struct trsync_signal signal;
    uint32_t arm_id;
} runtime_stop_binding;

static void
runtime_stop_on_trigger_cb(struct trsync_signal *tss, uint8_t reason)
{
    (void)reason;
    struct runtime_stop_binding *b =
        container_of(tss, struct runtime_stop_binding, signal);
    uint32_t clock_lo = timer_read_time();
    uint32_t clock_hi = stats_send_time_high + (clock_lo < stats_send_time);
    uint8_t status = 1; // NotArmed default
    (void)kalico_software_trip(b->arm_id, clock_lo, clock_hi, &status);
}

void
command_runtime_stop_on_trigger(uint32_t *args)
{
    uint32_t arm_id = args[0];
    struct trsync *ts = trsync_oid_lookup(args[1]);
    runtime_stop_binding.arm_id = arm_id;
    trsync_add_signal(ts, &runtime_stop_binding.signal,
                      runtime_stop_on_trigger_cb);
}
DECL_COMMAND(command_runtime_stop_on_trigger,
    "runtime_stop_on_trigger arm_id=%u trsync_oid=%c");
```

- [ ] **Step 3: Build H7 firmware on the Pi**

Per project rule (build on Pi, `make clean` between MCUs, use all cores):

Run (on the Pi, after commit+push+pull):
```bash
make clean && make menuconfig  # ensure H7 config (or: cp .config.h7.bak .config)
cp .config.h7.bak .config && make -j$(nproc)
```
Expected: compiles clean; `out/klipper.bin` produced. The new `runtime_stop_on_trigger` command appears in the dictionary.

- [ ] **Step 4: Build F446 firmware on the Pi**

Run:
```bash
make clean
cp .config.f446.test .config && make -j$(nproc)
```
Expected: compiles clean for F446 too (both MCUs must carry the command —
the relay may target either).

- [ ] **Step 5: Commit**

```bash
git add src/runtime_commands.c
git commit -m "feat(mcu): runtime_stop_on_trigger trsync signal freezes curve evaluator

Bridge twin of stepper_stop_on_trigger: relayed trsync_trigger fires this
signal, which calls kalico_software_trip(arm_id) to freeze the curve
evaluator. Lives beside command_runtime_software_trip in runtime_commands.c."
```

---

## Task 2: Runtime — disable the local siren (marker comment)

The detecting MCU must NOT self-freeze on a fresh GPIO trip, so the firmware
trsync stays armed and the relayed `trsync_trigger` is what stops it. Detection
still publishes the snapshot and queues the trip report.

**Files:**
- Modify: `rust/runtime/src/endstop.rs` (the `tick` GPIO-trip transition, ~line 660-678, where `compare_exchange(Armed→Tripping)` succeeds and returns `TripAction::AbortNow`)
- Test: `rust/runtime/tests/endstop_siren_disabled.rs` (create)

- [ ] **Step 1: Read the exact current trip-return site**

Run:
```bash
sed -n '578,700p' rust/runtime/src/endstop.rs
```
Identify the block where a fresh GPIO source asserts: `compare_exchange(Armed,
Tripping)` succeeds → `publish_snapshot(...)` → store `TrippedReady` → set
`TRIP_EVENT_QUEUED` → `return TripAction::AbortNow;`. This is the siren site.
(Note: the *separate* early return at the top of `tick` — `if state ==
TrippedReady || Tripping { return AbortNow }` — must be LEFT INTACT; that is how
the relayed `software_trip` freeze takes effect. Only the *fresh GPIO detection*
return is suppressed.)

- [ ] **Step 2: Write the failing test**

Create `rust/runtime/tests/endstop_siren_disabled.rs`. Mirror the arm/tick
setup of the existing endstop tests (see `rust/runtime/tests/` for the helper
pattern; reuse the same `ArmMsg`/source construction). The test arms a GPIO
source, drives the pin asserted, ticks, and asserts that:
- `tick(...)` returns `TripAction::Continue` (siren disabled — no local freeze),
- the trip event is still queued (`poll_trip()` returns `Some` with the snapshot).

```rust
// rust/runtime/tests/endstop_siren_disabled.rs
//
// With the local siren disabled (Part A bring-up), a fresh GPIO detection must
// REPORT (queue a trip event) but NOT self-freeze (tick returns Continue). The
// relayed software_trip is what freezes; see endstop.rs siren marker.

mod common; // if the test dir has a shared helper module; otherwise inline setup

use runtime::endstop::{self, TripAction};

#[test]
fn fresh_gpio_detection_reports_without_freezing() {
    // ... arm a single GPIO source (active_high, TripImmediately, sample_n=1)
    //     using the same helper the other endstop tests use ...
    let arm_id = 7;
    common::arm_single_gpio(arm_id, /*gpio*/ 0, /*active_high*/ true);

    // Drive the pin asserted in the test pin backend.
    common::set_pin(0, true);

    // Tick at/after arm_clock with a nonzero step count snapshot.
    let action = endstop::tick(/*clock*/ 1000, [0; 3], &[10, 20]);

    // Siren disabled: no local freeze.
    assert_eq!(action, TripAction::Continue,
        "fresh GPIO detection must not self-freeze while siren is disabled");

    // But the trip is still reported.
    let ev = endstop::poll_trip().expect("trip event must be queued");
    assert_eq!(ev.arm_id, arm_id);
}
```

(If `rust/runtime/tests/` has no `common` module, copy the minimal arm/pin
helpers from the nearest existing endstop test file into this file — repeat the
code; do not `use` across test crates.)

- [ ] **Step 3: Run the test — expect FAIL**

Run:
```bash
cargo test -p runtime --test endstop_siren_disabled -- --nocapture
```
Expected: FAIL — `tick` currently returns `AbortNow`, so the `Continue`
assertion fails.

- [ ] **Step 4: Disable the siren at the marked site**

In `rust/runtime/src/endstop.rs`, at the fresh-GPIO-trip block from Step 1,
replace the `return TripAction::AbortNow;` with a report-only return and a
marker. Keep the `publish_snapshot` + `TrippedReady` + `TRIP_EVENT_QUEUED`
lines so the report still happens:

```rust
        // ... compare_exchange(Armed -> Tripping) succeeded ...
        publish_snapshot(clock, idx as u8, stepper_counts);
        ARM.state.store(ArmState::TrippedReady as u8, Ordering::Release);
        TRIP_EVENT_QUEUED.store(true, Ordering::Release);
        // DISABLED FOR TESTING: local siren. The detecting MCU intentionally
        // does NOT self-freeze here — it only reports the trip. The cross-MCU
        // relay (bridge reactor TripDispatch) sends trsync_trigger, which
        // freezes via runtime_stop_on_trigger. Suppressing the local freeze
        // lets us verify the relay on a single board. Re-enable as the
        // same-MCU fast-path once the relay is confirmed.
        // See docs/superpowers/specs/2026-05-31-trsync-cross-mcu-homing-design.md
        // return TripAction::AbortNow;
        return TripAction::Continue;
```

Leave the early-return at the top of `tick` (`TrippedReady | Tripping ->
AbortNow`) UNCHANGED — that path is how the relayed `software_trip` freezes.

- [ ] **Step 5: Run the test — expect PASS**

Run:
```bash
cargo test -p runtime --test endstop_siren_disabled -- --nocapture
```
Expected: PASS.

- [ ] **Step 6: Run the full runtime suite (no regressions)**

Run:
```bash
cargo test -p runtime
```
Expected: PASS. (Existing endstop tests that asserted `AbortNow` on fresh GPIO
detection will need updating — if any fail, they encode the now-disabled siren;
update them to expect `Continue` + a queued trip, matching Step 2's intent, and
note the siren-disabled reason in a comment.)

- [ ] **Step 7: Commit**

```bash
git add rust/runtime/src/endstop.rs rust/runtime/tests/endstop_siren_disabled.rs
git commit -m "feat(runtime): disable local endstop siren for cross-MCU bring-up

Fresh GPIO detection now reports the trip (queues the event) but returns
Continue instead of AbortNow, so the firmware trsync stays armed and the
relayed trsync_trigger is what freezes. Marker comment records the re-enable
point for the same-MCU fast-path. Top-of-tick AbortNow (relayed-freeze path)
left intact."
```

---

## Task 3: Bridge — `TripDispatch` relay (the trdispatch analog)

**Files:**
- Create: `rust/motion-bridge/src/trip_dispatch.rs`
- Create: `rust/motion-bridge/src/trip_dispatch/tests.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (add `mod trip_dispatch;`)

- [ ] **Step 1: Add the module declaration**

In `rust/motion-bridge/src/lib.rs`, with the other `mod` lines, add:
```rust
mod trip_dispatch;
```

- [ ] **Step 2: Write the failing fan-out test**

Create `rust/motion-bridge/src/trip_dispatch/tests.rs`. Test the pure fan-out
logic without real transport, via a recording sender:

```rust
use super::*;
use std::cell::RefCell;

#[test]
fn first_trip_fans_trigger_to_all_sinks_once() {
    // Three sink trsyncs across (logically) different MCUs.
    let sinks = vec![
        SinkSpec { mcu: 1, trsync_oid: 10 },
        SinkSpec { mcu: 2, trsync_oid: 11 },
        SinkSpec { mcu: 3, trsync_oid: 12 },
    ];
    let sent = RefCell::new(Vec::<(u32, String)>::new());
    let dispatch = FanOut::new(sinks);

    // First trip → one trigger per sink.
    dispatch.on_trip(|mcu, cmd| sent.borrow_mut().push((mcu, cmd.to_string())));
    // Second trip (e.g. a duplicate report) → no further sends (one-shot).
    dispatch.on_trip(|mcu, cmd| sent.borrow_mut().push((mcu, cmd.to_string())));

    let sent = sent.into_inner();
    assert_eq!(sent.len(), 3, "exactly one trigger per sink, one-shot");
    assert_eq!(sent[0], (1, "trsync_trigger oid=10 reason=1".to_string()));
    assert_eq!(sent[1], (2, "trsync_trigger oid=11 reason=1".to_string()));
    assert_eq!(sent[2], (3, "trsync_trigger oid=12 reason=1".to_string()));
}

#[test]
fn build_trigger_cmd_formats_reason_endstop_hit() {
    assert_eq!(build_trigger_cmd(42), "trsync_trigger oid=42 reason=1");
}
```

`reason=1` = `REASON_ENDSTOP_HIT` (matches `klippy/mcu.py`'s
`MCU_trsync.REASON_ENDSTOP_HIT = 1`).

- [ ] **Step 3: Run the test — expect FAIL (no such module)**

Run:
```bash
cargo test -p motion-bridge trip_dispatch -- --nocapture
```
Expected: FAIL — `trip_dispatch` types not defined.

- [ ] **Step 4: Implement the module**

Create `rust/motion-bridge/src/trip_dispatch.rs`:

```rust
//! Cross-MCU homing trip relay — the bridge reactor's analog of mainline
//! Klipper's C `trdispatch`. On the first trip report from any participating
//! source, broadcast `trsync_trigger` to every participating sink trsync.
//!
//! Sources report via either `kalico_endstop_tripped` (bridge GPIO) or
//! `trsync_state` with `can_trigger==0` (classic/Beacon). Sinks are firmware
//! trsyncs armed with `runtime_stop_on_trigger` (Task 1) whose signal freezes
//! the curve evaluator.

use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use kalico_host_rt::host_io::{InterceptorId, KalicoHostIo};
use kalico_host_rt::transport::TransportError;

/// Reason carried by a relayed trigger. Matches `MCU_trsync.REASON_ENDSTOP_HIT`.
pub const REASON_ENDSTOP_HIT: u8 = 1;

/// A sink: a firmware trsync to trigger (freeze) on any trip.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SinkSpec {
    pub mcu: u32,
    pub trsync_oid: u8,
}

/// The trip report a source emits.
pub enum SourceSpec {
    /// Bridge GPIO endstop — listens for `kalico_endstop_tripped` (by arm_id).
    BridgeGpio { mcu: u32, arm_id: u32 },
    /// Classic/Beacon trsync — listens for `trsync_state` (by oid, can_trigger==0).
    Trsync { mcu: u32, trsync_oid: u8 },
}

/// `trsync_trigger oid=<oid> reason=<REASON_ENDSTOP_HIT>` — the sink command.
pub fn build_trigger_cmd(oid: u8) -> String {
    format!("trsync_trigger oid={oid} reason={REASON_ENDSTOP_HIT}")
}

/// Pure one-shot fan-out, unit-testable without real transport.
pub struct FanOut {
    sinks: Vec<SinkSpec>,
    fired: AtomicBool,
}

impl FanOut {
    pub fn new(sinks: Vec<SinkSpec>) -> Self {
        Self { sinks, fired: AtomicBool::new(false) }
    }

    /// On the first call, invoke `send(mcu, cmd)` once per sink. Subsequent
    /// calls are no-ops (one-shot, like trdispatch clearing `can_trigger`).
    pub fn on_trip(&self, mut send: impl FnMut(u32, &str)) {
        if self.fired
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return; // already fired
        }
        for s in &self.sinks {
            send(s.mcu, &build_trigger_cmd(s.trsync_oid));
        }
    }
}

/// Live handle: holds interceptor registrations + the io handles needed to
/// relay. Drop / `cleanup` unregisters the interceptors.
pub struct TripDispatchHandle {
    pub(crate) triggered: Arc<AtomicBool>,
    pub(crate) registrations: Vec<(Arc<KalicoHostIo>, InterceptorId)>,
}

/// Wire up the relay: register an interceptor per source; each interceptor's
/// closure fans `trsync_trigger` out to all sinks and sets `triggered`.
///
/// `sink_ios` maps a sink's `mcu` id to its `KalicoHostIo` (resolved by the
/// caller via `host_io_for_mcu`).
pub fn prepare(
    sources: Vec<(SourceSpec, Arc<KalicoHostIo>)>,
    sinks: Vec<SinkSpec>,
    sink_ios: Vec<(u32, Arc<KalicoHostIo>)>,
) -> Result<TripDispatchHandle, TransportError> {
    let triggered = Arc::new(AtomicBool::new(false));
    let fan = Arc::new(FanOut::new(sinks));
    let mut registrations = Vec::new();

    for (spec, src_io) in sources {
        let fan = Arc::clone(&fan);
        let triggered = Arc::clone(&triggered);
        let sink_ios = sink_ios.clone();
        let (name, oid_filter, want_arm_id) = match &spec {
            SourceSpec::BridgeGpio { arm_id, .. } =>
                ("kalico_endstop_tripped", None, Some(*arm_id)),
            SourceSpec::Trsync { trsync_oid, .. } =>
                ("trsync_state", Some(u32::from(*trsync_oid)), None),
        };
        let id = src_io.register_frame_interceptor(
            name,
            oid_filter,
            Box::new(move |params| {
                // Filter: bridge GPIO matches arm_id; trsync matches can_trigger==0.
                if let Some(want) = want_arm_id {
                    if params.get_u32("arm_id") != want { return; }
                } else if params.get_u32("can_trigger") != 0 {
                    return;
                }
                fan.on_trip(|mcu, cmd| {
                    if let Some((_, io)) =
                        sink_ios.iter().find(|(m, _)| *m == mcu)
                    {
                        let _ = io.send_fire_and_forget(cmd);
                    }
                });
                triggered.store(true, Ordering::Release);
            }),
        )?;
        registrations.push((src_io, id));
    }

    Ok(TripDispatchHandle { triggered, registrations })
}

pub fn cleanup(handle: TripDispatchHandle) {
    for (io, id) in handle.registrations {
        let _ = io.unregister_frame_interceptor(id);
    }
}

#[cfg(test)]
mod tests;
```

- [ ] **Step 5: Run the test — expect PASS**

Run:
```bash
cargo test -p motion-bridge trip_dispatch -- --nocapture
```
Expected: PASS (both `first_trip_fans_trigger_to_all_sinks_once` and
`build_trigger_cmd_formats_reason_endstop_hit`).

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/trip_dispatch.rs \
        rust/motion-bridge/src/trip_dispatch/tests.rs \
        rust/motion-bridge/src/lib.rs
git commit -m "feat(bridge): TripDispatch cross-MCU homing relay

trdispatch analog in the bridge reactor: registers interceptors on source
MCUs (kalico_endstop_tripped or trsync_state), and on the first trip fans
trsync_trigger out to every participating sink trsync (one-shot). Pure FanOut
unit-tested without transport."
```

---

## Task 4: Bridge FFI — expose `trip_dispatch_prepare` / `_cleanup`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (add PyO3 methods near `software_trip`, ~line 2566)

- [ ] **Step 1: Add the FFI methods**

In `rust/motion-bridge/src/bridge.rs`, in the `#[pymethods]` impl block, near
`software_trip`, add. The signature takes parallel arrays the Python side can
build from mcu handles + oids/arm_ids; the body resolves handles to
`KalicoHostIo` via the existing `host_io_for_mcu`, builds `SourceSpec`/`SinkSpec`,
calls `trip_dispatch::prepare`, stores the handle keyed by a returned id, and
returns that id.

```rust
    /// Prepare the cross-MCU trip relay for one homing move.
    ///
    /// `sources`: list of (kind, mcu, id) where kind 0 = BridgeGpio (id is
    /// arm_id), kind 1 = Trsync (id is trsync_oid). `sinks`: list of
    /// (mcu, trsync_oid). Returns an opaque handle id for `trip_dispatch_cleanup`.
    fn trip_dispatch_prepare(
        &self,
        sources: Vec<(u8, u32, u32)>,
        sinks: Vec<(u32, u8)>,
    ) -> PyResult<u64> {
        use crate::trip_dispatch::{self, SinkSpec, SourceSpec};

        let mut src_specs = Vec::new();
        for (kind, mcu, id) in sources {
            let io = self.host_io_for_mcu("trip_dispatch_prepare", mcu)?;
            let spec = match kind {
                0 => SourceSpec::BridgeGpio { mcu, arm_id: id },
                1 => SourceSpec::Trsync { mcu, trsync_oid: id as u8 },
                other => {
                    return Err(PyRuntimeError::new_err(format!(
                        "trip_dispatch_prepare: bad source kind {other}"
                    )));
                }
            };
            src_specs.push((spec, io));
        }

        let sink_specs: Vec<SinkSpec> = sinks
            .iter()
            .map(|(mcu, oid)| SinkSpec { mcu: *mcu, trsync_oid: *oid })
            .collect();
        let mut sink_ios = Vec::new();
        for (mcu, _) in &sinks {
            let io = self.host_io_for_mcu("trip_dispatch_prepare", *mcu)?;
            sink_ios.push((*mcu, io));
        }

        let handle = trip_dispatch::prepare(src_specs, sink_specs, sink_ios)
            .map_err(|e| {
                PyRuntimeError::new_err(format!("trip_dispatch_prepare: {e}"))
            })?;

        // Store keyed by a monotonic id. Reuse the same handle-registry pattern
        // probe_homing used (a Mutex<HashMap<u64, TripDispatchHandle>> on self,
        // plus a counter). If probe_homing's registry is generic, reuse it;
        // otherwise add `trip_dispatch_handles: Mutex<HashMap<u64, _>>` and
        // `trip_dispatch_next_id: AtomicU64` to the bridge struct.
        let id = self.trip_dispatch_next_id.fetch_add(1, Ordering::AcqRel);
        self.trip_dispatch_handles.lock().unwrap().insert(id, handle);
        Ok(id)
    }

    /// Tear down the relay (unregister interceptors). Idempotent.
    fn trip_dispatch_cleanup(&self, handle_id: u64) -> PyResult<()> {
        if let Some(handle) =
            self.trip_dispatch_handles.lock().unwrap().remove(&handle_id)
        {
            crate::trip_dispatch::cleanup(handle);
        }
        Ok(())
    }
```

- [ ] **Step 2: Add the handle registry fields to the bridge struct**

Find the bridge `#[pyclass]` struct definition (search `struct` near the top of
`bridge.rs` for the type whose `impl` holds `software_trip`). Add fields:

```rust
    trip_dispatch_handles:
        std::sync::Mutex<std::collections::HashMap<u64, crate::trip_dispatch::TripDispatchHandle>>,
    trip_dispatch_next_id: std::sync::atomic::AtomicU64,
```

And initialize them in the constructor (the `new`/`__new__` that builds the
struct) with `Mutex::new(HashMap::new())` and `AtomicU64::new(0)`. Add
`use std::sync::atomic::Ordering;` if not already imported in this file.

- [ ] **Step 3: Build the crate**

Run:
```bash
cargo build -p motion-bridge
```
Expected: compiles. (No new unit test here — Task 3 covered the logic; this is
FFI plumbing verified by compile + the sim test in Task 8.)

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "feat(bridge): FFI trip_dispatch_prepare/_cleanup

Resolve mcu handles to KalicoHostIo, build TripDispatch source/sink specs,
register the relay, return an opaque handle id. Handle registry on the bridge
struct mirrors probe_homing's."
```

---

## Task 5: Host — bridge `MCU_trsync` arms with `runtime_stop_on_trigger`

Reverse Piece A's ceremonial no-op: a bridge-driven MCU's trsync must actually
arm and register the curve-evaluator-freeze signal.

**Files:**
- Modify: `klippy/mcu.py` (`MCU_trsync.start`, ~line 346-419; `_build_config`, ~275)

- [ ] **Step 1: Add the `runtime_stop_on_trigger` command lookup**

In `MCU_trsync._build_config` (`mcu.py:275`), alongside the existing
`self._stepper_stop_cmd` lookup (~line 302), add a bridge-mode equivalent. For
bridge MCUs the commands are sent as text via the serial shim, so a lookup
object is not required; instead record the arm_id channel. No command-object
lookup is needed — Step 2 sends text. (Leave `_build_config` otherwise as-is;
`config_trsync` + the on-restart `trsync_start` already allocate the oid.)

- [ ] **Step 2: Arm with `runtime_stop_on_trigger` in the bridge branch**

In `MCU_trsync.start` (`mcu.py:346`), the `_bridge_drives_steppers` branch is
currently a no-op `return` (line 348-353). Replace it with real arming that
sends `trsync_start` + `runtime_stop_on_trigger` (one per the arm's `arm_id`)
via the serial shim. The `arm_id` is supplied by the homing glue (Task 7); store
it on the trsync when the dispatch is built. Concretely:

```python
    def start(self, print_time, report_offset, trigger_completion,
              expire_timeout):
        self._trigger_completion = trigger_completion
        if self._mcu._bridge_drives_steppers:
            # Bridge-driven MCU: the firmware trsync is a real SINK. Arm it and
            # register the runtime_stop_on_trigger signal so a relayed
            # trsync_trigger freezes the curve evaluator. No periodic report and
            # no expire are needed here — the bridge owns trip distribution
            # (TripDispatch) and host-death safety (Part B drip drain).
            self._home_end_clock = None
            clock = self._mcu.print_time_to_clock(print_time)
            serial = self._mcu._serial
            serial.send(
                "trsync_start oid=%d report_clock=%d report_ticks=0"
                " expire_reason=%d"
                % (self._oid, clock, self.REASON_COMMS_TIMEOUT)
            )
            arm_id = getattr(self, "_bridge_arm_id", None)
            if arm_id is None:
                raise self._mcu.error(
                    "bridge MCU_trsync.start: _bridge_arm_id not set "
                    "(homing glue must assign it before start)"
                )
            for s in self._steppers:
                serial.send(
                    "runtime_stop_on_trigger arm_id=%d trsync_oid=%d"
                    % (arm_id, self._oid)
                )
            logging.info(
                "[trsync-diag] bridge sink armed mcu=%s oid=%d arm_id=%d",
                self._mcu._name, self._oid, arm_id,
            )
            return
        # ... existing non-bridge (Beacon/Eddy) path unchanged ...
```

Note: a bridge sink trsync needs the freeze signal registered exactly once
regardless of how many steppers are on it; the per-stepper loop above sends one
`runtime_stop_on_trigger` per stepper but they all bind the same single static
firmware binding (Task 1) to the same `arm_id` — harmless (idempotent rebind to
the same arm). If you prefer exactly-once, send a single
`runtime_stop_on_trigger` when `self._steppers` is non-empty. Keep it simple:
send once:

```python
            if self._steppers:
                serial.send(
                    "runtime_stop_on_trigger arm_id=%d trsync_oid=%d"
                    % (arm_id, self._oid)
                )
```

- [ ] **Step 3: Syntax check**

Run:
```bash
python -c "import ast; ast.parse(open('klippy/mcu.py').read()); print('ok')"
```
Expected: `ok`.

- [ ] **Step 4: Commit**

```bash
git add klippy/mcu.py
git commit -m "feat(host): bridge MCU_trsync arms runtime_stop_on_trigger

Reverse the ceremonial no-op: a bridge-driven MCU's firmware trsync becomes a
real sink — armed via trsync_start and bound to the curve-evaluator freeze via
runtime_stop_on_trigger(arm_id). No report/expire; TripDispatch owns
distribution, drip drain owns host-death."
```

---

## Task 6: Host — create + arm sink trsyncs and the `TripDispatch` relay in `BridgeTriggerDispatch`

> **CORRECTION (2026-05-31, during execution).** The original Task 6 below assumed
> `drip_move` could enumerate per-MCU sink `MCU_trsync` objects. It cannot: the
> bridge GPIO endstop uses `BridgeTriggerDispatch` (`motion_bridge.py`), which owns
> **no** `MCU_trsync` objects (it arms stepper OIDs directly via `endstop_arm`), and
> the legacy `TriggerDispatch` that *does* create them hard-rejects bridge MCUs
> (`mcu.py:506`). Firmware trsync OIDs are **config-time** allocations
> (`MCU_trsync.__init__` → `mcu.create_oid()` + `register_config_callback`), so the
> sink trsyncs must be **created in `BridgeTriggerDispatch.add_stepper`** (config
> phase, one per distinct stepper MCU, mirroring legacy `TriggerDispatch.add_stepper`)
> and **armed + relayed in `BridgeTriggerDispatch.start`**, cleaned up in `stop`.
> This matches spec A4/A5 ("`MCU_endstop.home_start` arms the per-MCU trsyncs + the
> `TripDispatch`"). `drip_move` is left **unchanged**. The `motion_bridge.py`
> `trip_dispatch_prepare`/`_cleanup` wrappers are still added as written.
>
> The corrected scope is implemented per the re-dispatch instructions; the original
> drip_move-centric steps below are retained for context but superseded.

### (superseded) Original Task 6 — wire `TripDispatch` into the GPIO `drip_move` branch

**Files:**
- Modify: `klippy/motion_bridge.py` (add `trip_dispatch_prepare`/`_cleanup` wrappers)
- Modify: `klippy/motion_toolhead.py` (`drip_move` GPIO branch, ~line 464-479)

- [ ] **Step 1: Add the bridge wrappers**

In `klippy/motion_bridge.py`, beside `software_trip` (~line 371), add:

```python
    def trip_dispatch_prepare(self, sources, sinks):
        # sources: [(kind, mcu_handle, id)]  kind 0=BridgeGpio(id=arm_id),
        #                                    kind 1=Trsync(id=trsync_oid)
        # sinks:   [(mcu_handle, trsync_oid)]
        return self._bridge.trip_dispatch_prepare(sources, sinks)

    def trip_dispatch_cleanup(self, handle_id):
        return self._bridge.trip_dispatch_cleanup(handle_id)
```

- [ ] **Step 2: Assign `arm_id` to participating trsyncs + prepare the relay**

In `klippy/motion_toolhead.py`, the GPIO branch of `drip_move` (where
`active_homing_arms` is non-empty, ~line 464). Before `submit_homing_move`,
build the participant set and prepare the relay. The endstop's
`BridgeTriggerDispatch` already created the per-MCU `MCU_trsync` sinks and added
the arm_id to `active_homing_arms`; here we (a) stamp `_bridge_arm_id` on each
sink trsync so `MCU_trsync.start` (Task 5) can arm it, and (b) prepare the
`TripDispatch` with the detecting MCU as a `BridgeGpio` source and every
participating bridge MCU as a sink:

```python
        arm_ids = list(self.active_homing_arms)
        if arm_ids:
            arm_id = arm_ids[0]
            # Sinks: every bridge MCU with a moving stepper in this move.
            sinks = []           # [(mcu_handle, trsync_oid)]
            for trsync in self._homing_sink_trsyncs():   # see Step 3
                trsync._bridge_arm_id = arm_id
                sinks.append(
                    (trsync.get_mcu()._bridge_handle, trsync.get_oid())
                )
            # Source: the detecting (endstop) MCU reports via kalico_endstop_tripped.
            endstop_mcu_handle = self._homing_endstop_mcu()._bridge_handle  # Step 3
            sources = [(0, endstop_mcu_handle, arm_id)]
            self._trip_handle_id = self.bridge.trip_dispatch_prepare(
                sources, sinks
            )

            pos3 = list(newpos[:3]) + [0.0] * max(0, 3 - len(newpos[:3]))
            dx = pos3[0] - self.commanded_pos[0]
            dy = pos3[1] - self.commanded_pos[1]
            dz = pos3[2] - self.commanded_pos[2]
            self._fire_active_callbacks(dx, dy, dz, 0.0,
                                        self.get_last_move_time())
            self.bridge._software_trip_active = False
            bridge_lmt_before = self.bridge.get_last_move_time()
            try:
                self.bridge.submit_homing_move(pos3, speed, arm_ids)
                self.bridge.wait_moves()
            finally:
                if getattr(self, "_trip_handle_id", None) is not None:
                    self.bridge.trip_dispatch_cleanup(self._trip_handle_id)
                    self._trip_handle_id = None
            return
```

- [ ] **Step 3: Add the participant-resolution helpers**

Still in `motion_toolhead.py`, add small helpers that read the active homing
dispatch to find the sink trsyncs and the endstop MCU. The
`BridgeTriggerDispatch` for the active arm holds the `MCU_trsync` list and the
endstop's MCU; expose them:

```python
    def _homing_sink_trsyncs(self):
        # The active BridgeTriggerDispatch(es) hold the per-MCU MCU_trsync
        # sinks. Collect them across all active endstop dispatches.
        trsyncs = []
        for disp in self._active_endstop_dispatches():   # existing registry
            trsyncs.extend(disp.get_trsyncs())
        return trsyncs

    def _homing_endstop_mcu(self):
        for disp in self._active_endstop_dispatches():
            return disp.get_endstop_mcu()
        raise self.printer.command_error("no active homing dispatch")
```

If `BridgeTriggerDispatch` does not already expose `get_trsyncs()` /
`get_endstop_mcu()` / a registry of active dispatches, add those accessors in
`motion_bridge.py` (they return `self._trsyncs` and the endstop's `_mcu`). Keep
them thin.

- [ ] **Step 4: Syntax check both files**

Run:
```bash
python -c "import ast; [ast.parse(open(f).read()) for f in ['klippy/motion_bridge.py','klippy/motion_toolhead.py']]; print('ok')"
```
Expected: `ok`.

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_bridge.py klippy/motion_toolhead.py
git commit -m "feat(host): wire TripDispatch into GPIO drip_move

Stamp arm_id on each sink trsync, prepare the cross-MCU relay (detecting MCU as
BridgeGpio source, all participating bridge MCUs as sinks), and clean it up
after the move. Completion handling stays on BridgeTriggerDispatch."
```

---

## Task 7: Build the Python extension + smoke-load

**Files:** none (build/verify only)

- [ ] **Step 1: Build the motion-bridge Python extension**

Run (the repo's normal bridge build — match how the project builds the PyO3
module; commonly a maturin/cargo step wired into the klippy build):
```bash
cargo build -p motion-bridge --release
```
Expected: compiles. Resolve any FFI signature mismatches between Task 4 and
Tasks 5-6 (argument tuple shapes for `trip_dispatch_prepare`).

- [ ] **Step 2: Commit (if any build glue changed)**

```bash
git add -A && git commit -m "chore(bridge): build trip_dispatch FFI" || echo "nothing to commit"
```

---

## Task 8: Dual-MCU Renode sim — cross-MCU relay assertion

This is the genuine cross-MCU test the hardware can't give (X is H7-only).

**Files:**
- Modify: `tools/test_renode_endstop_e2e.py`

- [ ] **Step 1: Read the existing sim harness**

Run:
```bash
sed -n '1,80p' tools/test_renode_endstop_e2e.py
cat tools/sim/dual_mcu_docker.resc
```
Understand how it boots H723+F446, arms an endstop, and injects a GPIO level.

- [ ] **Step 2: Add a cross-MCU relay test**

Append a test that: arms a homing move with the endstop GPIO on MCU A and a
stepper (sink trsync) on MCU B; starts motion on both; injects the GPIO trip on
A; asserts B's curve evaluator freezes (B's step count stops advancing) within
a bound (e.g. 10 ms). Use the harness's existing assertion helpers. Pseudocode
to adapt to the harness API:

```python
def test_cross_mcu_relay_freezes_remote(sim):
    a, b = sim.mcu("h723"), sim.mcu("f446")
    arm_id = 1
    sim.arm_homing(endstop_mcu=a, sink_mcus=[a, b], arm_id=arm_id)
    sim.start_homing_move(speed=50)
    b_steps_before = sim.step_count(b)
    sim.inject_gpio_trip(a)                       # endstop asserts on A
    sim.wait(0.010)                               # 10 ms budget
    b_steps_after = sim.step_count(b)
    # B must have stopped advancing (relay → trsync_trigger → freeze).
    assert sim.is_frozen(b), "remote MCU did not freeze on relayed trigger"
```

- [ ] **Step 3: Run the sim test**

Run:
```bash
python tools/test_renode_endstop_e2e.py   # or the repo's pytest invocation
```
Expected: PASS — A's trip freezes B via the relay.

- [ ] **Step 4: Commit**

```bash
git add tools/test_renode_endstop_e2e.py
git commit -m "test(sim): cross-MCU trip relay freezes remote MCU"
```

---

## Task 9: Single-board hardware bring-up (siren disabled)

Verify the full report→relay→`trsync_trigger`→freeze loop on H7 alone. Safe:
sensorless X homes into the frame in free air; if the relay fails the move just
pushes the carriage against the frame at homing speed (normal StallGuard
behavior), no collision.

**Files:** none (hardware procedure)

- [ ] **Step 1: Flash both MCUs (Pi flow)**

Per project rule: commit + push, then on the Pi pull, `make clean` between
configs, build H7 (`.config.h7.bak`) and F446 (`.config.f446.test`), flash both.

- [ ] **Step 2: Arm tracing**

Confirm `/tmp/interceptor_trace.log` is being written and tail it. Ensure the
new diag lines (CALLBACK on `kalico_endstop_tripped`, the `trsync_trigger` send)
appear.

- [ ] **Step 3: Home X (ask the user to issue the G-code)**

Do NOT issue motion G-code without explicit per-command permission. Ask the user
to run `G28 X` and watch:
- Expected: X drives toward the frame; StallGuard trips; `endstop.rs` reports
  (`kalico_endstop_tripped`); the reactor relays `trsync_trigger oid=…` to H7;
  the curve evaluator freezes; X stops. `home_wait` returns a trigger time.
- Failure to capture: no `CALLBACK` / no `trsync_trigger` send in the trace →
  relay didn't fire; compare against the Task 8 sim path.

- [ ] **Step 4: Record result**

If it stops: Part A is verified end-to-end. If not, the trace pinpoints the dead
link (report missing vs trigger send missing vs freeze missing). Do not
re-enable the siren yet — that is the optimization step, after the relay is
trusted.

---

## Self-review

- **Spec coverage:** A1 (signal) → Task 1. A2 (siren off + report) → Task 2.
  A3 (host arming) → Task 5. A4 (reactor relay) → Tasks 3-4, wired in Task 6.
  A5 (homing contract) → Task 6 preserves `submit_homing_move`/`wait_moves` +
  `BridgeTriggerDispatch` completion. A6 (Beacon source) → `SourceSpec::Trsync`
  exists in Task 3; full Beacon wiring is exercised in Part B / a follow-up
  (Part A proves the bridge-GPIO source path on hardware). Part B (drip) is a
  separate plan — explicitly out of scope here; the existing deadline net stays.
- **Placeholder scan:** sim test (Task 8) and participant helpers (Task 6 Step 3)
  are written against harness/registry APIs that must be confirmed in-code; both
  steps begin with a read step and give concrete adapt-to-API pseudocode rather
  than a bare "TODO". The `host_io_for_mcu`, `register_frame_interceptor`,
  `send_fire_and_forget`, and `_serial.send` calls are all used verbatim from
  existing code (`bridge.rs::software_trip`, `probe_homing.rs`, `mcu.py:404`).
- **Type consistency:** `REASON_ENDSTOP_HIT = 1` used identically in Task 3
  (Rust) and implied by `mcu.py` (`MCU_trsync.REASON_ENDSTOP_HIT`). FFI tuple
  shapes: `trip_dispatch_prepare(sources=[(kind,mcu,id)], sinks=[(mcu,oid)])`
  consistent across Task 4 (Rust) and Task 6 (Python). `_bridge_arm_id` set in
  Task 6, read in Task 5.

## Risks carried into execution

- **`BridgeTriggerDispatch` accessor surface** (`get_trsyncs`, `get_endstop_mcu`,
  active-dispatch registry) may need adding — Task 6 Step 3 notes this.
- **Single static firmware binding** (Task 1) assumes one active homing arm per
  MCU. True today; revisit if concurrent multi-arm homing is added.
- **Beacon arming through the bridge serial shim** is not exercised by Part A's
  hardware test (bridge-GPIO source instead). Confirm separately before relying
  on Beacon+Z.

## Post-implementation notes (2026-05-31)

Tasks 1–7 implemented + reviewed (spec + code-quality). Validation done:
`cargo build -p motion-bridge` clean; trip_dispatch (4) + endstop (30) unit
tests pass; H723 sim firmware builds with the changes and exports
`runtime_stop_on_trigger arm_id=%u trsync_oid=%c` in `out/klipper.dict`
alongside `trsync_trigger`/`trsync_start`. Final whole-branch review confirmed
the arm_id (u32) / oid (u8) / reason (u8=1) contract is consistent across C
(`runtime_commands.c`), Rust (`trip_dispatch.rs`/FFI), and Python
(`mcu.py`/`motion_bridge.py`), with no legacy `stepper_stop_on_trigger`
double-stop on the bridge path (bridge steppers get `runtime_stop_on_trigger`
exclusively).

- **Task 8 divergence (intentional):** the plan's Task 8 modified
  `tools/test_renode_endstop_e2e.py` (Python `KalicoHostIO` + `sim.is_frozen`
  helpers) — but that harness drives the firmware directly and **never calls the
  relay**, so it would prove nothing about the new code. Replaced with a
  pure-Rust live-reactor integration test
  (`rust/motion-bridge/tests/relay_reactor_integration.rs`, commit `9bcb688a8`):
  feeds an inbound `kalico_endstop_tripped` frame through a real `Reactor`
  (`ReactorHarness`), and asserts the interceptor fires and emits a real
  outbound `trsync_trigger oid=S reason=1` on the sink wire (positive +
  wrong-arm_id-filtered + FanOut-one-shot cases). Covers the reactor-decode →
  InterceptorTable → relay closure (real `FanOut`/`build_trigger_cmd`) → wire
  seam — the gap the unit tests left. Does NOT cover the firmware half
  (`trsync_do_trigger → runtime_stop_on_trigger → software_trip`); that is
  Task 9. Added two minimal additive `ReactorHarness` helpers
  (`new_with_parser`, `register_interceptor`). (Pre-existing, unrelated: 2
  `kalico-host-rt` `arm_flow_unit` tests fail at the merge-base too —
  clock-sync-request harness issue, not from this branch.)

- **Follow-up F1 (hygiene, deferrable past bench):** `BridgeTriggerDispatch.stop()`
  tears down the host relay + GPIO endstop but does NOT disarm sink firmware
  trsyncs on the no-trip path (`MCU_trsync.stop()` bridge branch is a no-op). A
  sink that didn't fire stays armed (with its `runtime_stop_binding`) until the
  next `trsync_start` clears it. Benign — no report/expire timer is armed
  (`report_ticks=0`, no `trsync_set_timeout`), so it cannot spuriously fire, and
  each homing move re-sends `trsync_start` (→ `trsync_clear`) first. A clean
  disarm has no existing firmware command (force-`trsync_trigger` would wrongly
  fire the freeze; `trsync_start`-to-clear is hacky), so the proper fix is a
  design piece for Part B / the same-MCU fast-path re-enable, not a bench
  blocker.

- **Follow-up F2 (Task 9 watch / Part B):** sinks arm no `trsync_set_timeout`, so
  with `report_ticks=0` there is no firmware expire timer — if the host dies
  mid-homing the sink does NOT auto-freeze; the metered drip-drain (Part B) is
  the only host-death safety net. Validate the drain actually freezes on
  host-death during a homing move when Part B lands.
