# Homing: Trsync Liveness + Metered Drip — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Working homing per `docs/superpowers/specs/2026-06-07-homing-trsync-metered-drip-design.md`: merge Part A (cross-MCU trsync stop), port trdispatch's timeout-extension into `TripDispatch`, and meter homing motion as ≤25 ms curve pieces with a 50 ms pump horizon.

**Architecture:** Stock trsync is the liveness + stop protocol for every participant (bridge sinks and classic sources like Beacon). `TripDispatch` in the bridge reactor is a complete port of mainline `klippy/chelper/trdispatch.c` (trigger fan-out — already built on branch `trsync-cross-mcu-A` — plus timeout extension, this plan). The drip is bounded buffering only: piece slicing in `enqueue.rs`, per-move-class lead in `pump.rs`.

**Tech Stack:** Rust (motion-bridge, kalico-host-rt, runtime), Python (klippy), C firmware (already complete via Part A merge). Worktree: `/Users/daniladergachev/Developer/kalico/.worktrees/homing-rework`, branch `homing-rework`.

**Reference implementations (read before the tasks that port them):**
- `git show main:klippy/chelper/trdispatch.c` — the extension algorithm being ported (Task 3)
- `git show main:klippy/mcu.py` lines 150–390 — `MCU_trsync.start` report/timeout params (Task 5)

---

### Task 1: Merge `trsync-cross-mcu-A`

**Files:**
- Merge-modify: `klippy/mcu.py`, `klippy/motion_bridge.py`, `rust/motion-bridge/src/bridge.rs`, `rust/kalico-host-rt/src/host_io/test_harness.rs`, `rust/motion-bridge/Cargo.toml`, `docs/superpowers/plans/2026-05-31-trsync-cross-mcu-homing-A.md`
- Merge-clean (auto): `src/runtime_commands.c`, `rust/runtime/src/endstop.rs`, `rust/runtime/src/endstop/tests.rs`, `rust/motion-bridge/src/lib.rs`, `rust/motion-bridge/src/trip_dispatch.rs` (+ tests), `rust/motion-bridge/tests/relay_reactor_integration.rs`

- [ ] **Step 1: Start the merge**

```bash
cd /Users/daniladergachev/Developer/kalico/.worktrees/homing-rework
git merge trsync-cross-mcu-A
```

Expected: conflicts in the six merge-modify files above; everything else auto-merges.

- [ ] **Step 2: Resolve per-file**

Resolution intents (the branch's Part A features land on top of homing-rework's deadline rip-out; where the two sides touch the same region, keep BOTH the rip-out deletions and the Part A additions):

- `docs/superpowers/plans/2026-05-31-trsync-cross-mcu-homing-A.md` (modify/delete): keep the branch's version (`git checkout trsync-cross-mcu-A -- <path>`). Historical plan record.
- `klippy/motion_bridge.py`: keep Part A's `trip_dispatch_prepare`/`trip_dispatch_cleanup` wrapper methods and all `BridgeTriggerDispatch` changes (sink-trsync creation in `add_stepper`, arming + relay in `start()`, relay teardown in `stop()`); drop the `extend_homing_deadline` wrapper the branch still carries (deleted on our side in commit `208a186ed`), and the `"extend_homing_deadline"` whitelist entry if the branch reintroduces it.
- `klippy/mcu.py`: take Part A's bridge-sink arming branch in `MCU_trsync.start` (the `_bridge_drives_steppers` block sending `trsync_start` + `runtime_stop_on_trigger`) over the no-op logging branch; keep all unrelated sota-motion drift on our side.
- `rust/motion-bridge/src/bridge.rs`: keep Part A's `trip_dispatch_prepare`/`trip_dispatch_cleanup` pyfunctions and handle registry; do NOT reintroduce `extend_homing_deadline` (deleted on our side).
- `rust/motion-bridge/Cargo.toml` / `test_harness.rs`: union — keep both sides' additions (branch adds dev-deps/harness helpers `new_with_parser`, `register_interceptor`).

- [ ] **Step 3: Verify and commit**

```bash
python3 -m py_compile klippy/mcu.py klippy/motion_bridge.py klippy/motion_toolhead.py
cd rust && cargo test --workspace 2>&1 | tail -5
git add -A && git commit  # merge commit, default message fine
```

Expected: py_compile silent; cargo zero failures (the branch's trip_dispatch unit tests + `relay_reactor_integration` must pass post-merge). Known pre-existing failures at the old merge-base (`kalico-host-rt` `arm_flow_unit` clock-sync harness) were fixed since; if any test fails, fix the merge — do not skip.

---

### Task 2: `ExtensionEngine` — pure trdispatch timeout-extension port

**Files:**
- Create: `rust/motion-bridge/src/trip_dispatch/extension.rs`
- Create: `rust/motion-bridge/src/trip_dispatch/extension_tests.rs`
- Modify: `rust/motion-bridge/src/trip_dispatch.rs` (add `pub mod extension;` + `#[cfg(test)] mod extension_tests;` — match the file's existing test-module idiom)

Pure logic, no I/O — the port target is `handle_trsync_state`'s extension half in mainline `trdispatch.c`. Times are f64 host seconds (the reactor analog of trdispatch's `clock_to_time`); tick conversion happens at the wiring layer (Task 4).

- [ ] **Step 1: Write the failing tests**

`extension_tests.rs`:

```rust
use super::extension::{ExtensionEngine, Participant};

fn engine(n: usize, expire_s: f64, min_extend_s: f64, start: f64) -> ExtensionEngine {
    ExtensionEngine::new(
        (0..n)
            .map(|_| Participant {
                last_status_time: start,
                expire_time: start + expire_s,
            })
            .collect(),
        expire_s,
        min_extend_s,
    )
}

#[test]
fn report_extends_others_anchored_to_minimum() {
    // Port of trdispatch's min/next-min anchoring. Two participants, 25ms
    // expire, no hysteresis. P0 reports t=1.0 while P1 is still at t=0.0:
    // P0's anchor is min-of-others = 0.0 (it may NOT use its own report);
    // P1's anchor is min-of-others = 1.0.
    let mut e = engine(2, 0.025, 0.0, 0.0);
    let sends = e.on_report(0, 1.0);
    let find = |s: &[(usize, f64)], i| s.iter().find(|(p, _)| *p == i).map(|(_, t)| *t);
    assert_eq!(find(&sends, 1), Some(1.0 + 0.025));
    // P0's own expire stays anchored at P1's stale time — extending it would
    // need 0.0 + 0.025 which is not an advance, so no send for P0.
    assert_eq!(find(&sends, 0), None);
}

#[test]
fn participant_cannot_extend_itself() {
    // Single participant: its own report must never extend its own expire
    // (trdispatch: min_tdm is anchored to next_min_time, which with one
    // participant is its own time — mainline still sends, matching
    // single-MCU behavior where the host's report echo IS the extension).
    // Two-participant asymmetry is the load-bearing property:
    let mut e = engine(2, 0.025, 0.0, 0.0);
    // P0 reports repeatedly; P1 stays silent at t=0.
    e.on_report(0, 1.0);
    let sends = e.on_report(0, 2.0);
    // P0's expire is still anchored to P1's t=0 — never advances.
    assert!(sends.iter().all(|(p, _)| *p != 0));
    // And P1's expire is re-anchored to P0's fresh time each report.
    assert_eq!(sends, vec![(1, 2.0 + 0.025)]);
}

#[test]
fn silence_means_no_extension_for_anyone_anchored_to_the_silent_one() {
    let mut e = engine(3, 0.025, 0.0, 0.0);
    e.on_report(0, 1.0);
    e.on_report(1, 1.0);
    // P2 never reports. P0/P1 keep reporting; their anchors include P2's
    // t=0.0, so their expire stays 0.025 — they WILL time out. P2's own
    // expire keeps advancing (anchored to min of P0/P1).
    let sends = e.on_report(0, 2.0);
    assert!(sends.iter().any(|(p, t)| *p == 2 && *t > 1.0));
    assert!(sends.iter().all(|(p, _)| *p != 0 && *p != 1));
}

#[test]
fn hysteresis_suppresses_small_advances() {
    // min_extend = 6ms (0.8 × 0.3 × 25ms ≈ mainline's min_extend_ticks).
    let mut e = engine(2, 0.025, 0.006, 0.0);
    let sends = e.on_report(0, 0.004);
    // P1's new expire would be 0.029 vs current 0.025 → +4ms < 6ms → no send.
    assert!(sends.is_empty());
    let sends = e.on_report(0, 0.010);
    assert_eq!(sends, vec![(1, 0.035)]);
}

#[test]
fn single_participant_extends_on_own_report() {
    // Mainline single-MCU homing: one trsync, its own state reports drive
    // its extension (next_min_time == its own time).
    let mut e = engine(1, 0.25, 0.0, 0.0);
    let sends = e.on_report(0, 1.0);
    assert_eq!(sends, vec![(0, 1.25)]);
}
```

- [ ] **Step 2: Run to verify failure**

```bash
cd rust && cargo test -p motion-bridge extension 2>&1 | tail -3
```

Expected: compile error — `extension` module does not exist.

- [ ] **Step 3: Implement**

`extension.rs`:

```rust
//! Timeout-extension half of mainline `trdispatch.c::handle_trsync_state`,
//! ported as pure logic over f64 host-time seconds. The wiring layer
//! converts participant report clocks to host time and expire times back to
//! MCU ticks.

pub struct Participant {
    pub last_status_time: f64,
    pub expire_time: f64,
}

pub struct ExtensionEngine {
    participants: Vec<Participant>,
    expire_secs: f64,
    min_extend_secs: f64,
}

impl ExtensionEngine {
    pub fn new(
        participants: Vec<Participant>,
        expire_secs: f64,
        min_extend_secs: f64,
    ) -> Self {
        Self { participants, expire_secs, min_extend_secs }
    }

    /// Record a `can_trigger=1` report from `idx` at host time `status_time`
    /// and return the `(participant_idx, new_expire_time)` set_timeout sends
    /// due. Mirrors trdispatch: each participant's anchor is the minimum
    /// status time among the OTHERS (the minimum-holder anchors to the
    /// second minimum), so no participant ever extends itself; sends are
    /// suppressed unless the expire advances by at least `min_extend_secs`.
    pub fn on_report(&mut self, idx: usize, status_time: f64) -> Vec<(usize, f64)> {
        self.participants[idx].last_status_time = status_time;

        let mut min_time = f64::INFINITY;
        let mut next_min_time = f64::INFINITY;
        let mut min_idx = usize::MAX;
        for (i, p) in self.participants.iter().enumerate() {
            let t = p.last_status_time;
            if t < next_min_time {
                next_min_time = t;
                if t < min_time {
                    next_min_time = min_time;
                    min_time = t;
                    min_idx = i;
                }
            }
        }
        if next_min_time == f64::INFINITY {
            next_min_time = min_time;
        }

        let mut sends = Vec::new();
        for (i, p) in self.participants.iter_mut().enumerate() {
            let anchor = if i == min_idx { next_min_time } else { min_time };
            let expire = anchor + self.expire_secs;
            if expire - p.expire_time >= self.min_extend_secs
                && expire > p.expire_time
            {
                p.expire_time = expire;
                sends.push((i, expire));
            }
        }
        sends
    }
}
```

Note the `expire > p.expire_time` guard alongside hysteresis: with `min_extend_secs = 0.0`, trdispatch's `>= min_extend_ticks` would re-send equal expires forever; mainline avoids this because `min_extend_ticks` is never 0 in practice. The strict guard keeps the `0.0` test configuration meaningful without changing real-cadence behavior.

- [ ] **Step 4: Run to verify pass**

```bash
cd rust && cargo test -p motion-bridge extension 2>&1 | tail -3
```

Expected: all 5 tests pass. If `report_extends_others_anchored_to_minimum` fails on the P0-no-send assertion, re-check the min/next-min swap against `trdispatch.c` lines 96–110 — the inner `next_min_time = min_time` before overwriting `min_time` is the subtle part.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/trip_dispatch.rs rust/motion-bridge/src/trip_dispatch/extension.rs rust/motion-bridge/src/trip_dispatch/extension_tests.rs
git commit -m "feat(bridge): ExtensionEngine — pure port of trdispatch timeout extension"
```

---

### Task 3: Clock conversion helpers for participant reports

**Files:**
- Modify: `rust/motion-bridge/src/trip_dispatch/extension.rs` (append)
- Modify: `rust/motion-bridge/src/trip_dispatch/extension_tests.rs` (append)

`trsync_state` carries a 32-bit MCU clock. The router gives a projected 64-bit "now" + frequency per MCU (`router.rs::ack_clock_and_freq`). Port of mainline's `clock_from_clock32` + `clock_to_time`.

- [ ] **Step 1: Failing tests**

Append to `extension_tests.rs`:

```rust
use super::extension::{clock32_to_64, ticks_to_host_time, host_time_to_ticks};

#[test]
fn clock32_reconstruction_handles_wrap() {
    // now64 = 0x1_0000_0010, report clock32 slightly in the past across the
    // 32-bit boundary.
    assert_eq!(clock32_to_64(0x1_0000_0010, 0xFFFF_FFF0), 0x0_FFFF_FFF0);
    // And slightly in the future.
    assert_eq!(clock32_to_64(0x1_0000_0010, 0x0000_0020), 0x1_0000_0020);
}

#[test]
fn tick_time_round_trip() {
    let freq = 520_000_000.0;
    let host_now = 100.0;
    let now_ticks: u64 = 52_000_000_000;
    let t = ticks_to_host_time(now_ticks + 5_200_000, now_ticks, host_now, freq);
    assert!((t - 100.01).abs() < 1e-9);
    let back = host_time_to_ticks(t, now_ticks, host_now, freq);
    assert_eq!(back, now_ticks + 5_200_000);
}
```

- [ ] **Step 2: Verify failure**

```bash
cd rust && cargo test -p motion-bridge extension 2>&1 | tail -3
```

Expected: compile error — functions not defined.

- [ ] **Step 3: Implement** (append to `extension.rs`)

```rust
/// Reconstruct a full 64-bit clock from a 32-bit report value, anchored to
/// the projected 64-bit now (mainline `clock_from_clock32`).
pub fn clock32_to_64(now64: u64, clock32: u32) -> u64 {
    let delta = clock32.wrapping_sub(now64 as u32) as i32;
    now64.wrapping_add(delta as i64 as u64)
}

pub fn ticks_to_host_time(ticks: u64, now_ticks: u64, host_now: f64, freq: f64) -> f64 {
    host_now + (ticks as i64 - now_ticks as i64) as f64 / freq
}

pub fn host_time_to_ticks(t: f64, now_ticks: u64, host_now: f64, freq: f64) -> u64 {
    let delta = (t - host_now) * freq;
    (now_ticks as i64 + delta.round() as i64) as u64
}
```

- [ ] **Step 4: Verify pass**

```bash
cd rust && cargo test -p motion-bridge extension 2>&1 | tail -3
```

- [ ] **Step 5: Commit**

```bash
git add -u && git commit -m "feat(bridge): clock32 reconstruction + tick/host-time conversion for trsync reports"
```

---

### Task 4: Wire the liveness web into `TripDispatch`

**Files:**
- Modify: `rust/motion-bridge/src/trip_dispatch.rs`
- Modify: `rust/motion-bridge/src/trip_dispatch/tests.rs`
- Modify: `rust/motion-bridge/src/bridge.rs` (the `trip_dispatch_prepare` pyfunction)
- Modify: `rust/motion-bridge/tests/relay_reactor_integration.rs`

Every participant now also has its `can_trigger=1` reports feed the `ExtensionEngine`, and due extensions go out as `trsync_set_timeout` fire-and-forget sends. Bridge sinks report too (arming changes in Task 5 turn their reports on).

- [ ] **Step 1: Read the current wiring**

Read `rust/motion-bridge/src/trip_dispatch.rs` (post-merge) and the `trip_dispatch_prepare` pyfunction in `bridge.rs` to confirm the registration and handle-registry shapes; the steps below adapt to them.

- [ ] **Step 2: Extend `prepare`'s signature and interceptor closures**

Changes to `trip_dispatch.rs`:

```rust
pub struct ParticipantSpec {
    /// Which io this participant's trsync lives on (and where its
    /// trsync_state reports arrive).
    pub mcu: u32,
    pub trsync_oid: u8,
}

pub fn prepare(
    sources: Vec<(SourceSpec, Arc<KalicoHostIo>)>,
    sinks: Vec<SinkSpec>,
    sink_ios: Vec<(u32, Arc<KalicoHostIo>)>,
    // NEW: the liveness web. Every participating trsync (sinks AND classic
    // sources like Beacon), with the timeout chosen by the caller
    // (single- vs multi-MCU constant from danger_options).
    participants: Vec<(ParticipantSpec, Arc<KalicoHostIo>)>,
    expire_timeout_s: f64,
    clock_of: impl Fn(u32) -> Option<(u64, f64)> + Send + Sync + 'static,
) -> Result<TripDispatchHandle, TransportError> {
```

Inside, after the existing trip-interceptor loop, register one additional
`trsync_state` interceptor per participant whose closure does the
extension half (the existing `SourceSpec::Trsync` interceptor keeps doing
the `can_trigger==0` trip half; a participant that is also a Trsync source
gets both behaviors from its two interceptors — registration keys differ
only if the io layer requires it; if `register_frame_interceptor` rejects
duplicate (name, oid) keys, merge both behaviors into one closure that
branches on `can_trigger`):

```rust
let engine = Arc::new(std::sync::Mutex::new(ExtensionEngine::new(
    participants
        .iter()
        .map(|_| Participant { last_status_time: 0.0, expire_time: 0.0 })
        .collect(),
    expire_timeout_s,
    0.8 * 0.3 * expire_timeout_s, // mainline: min_extend = 0.8 × report_ticks
)));
let clock_of = Arc::new(clock_of);
let participant_io: Vec<(u32, u8, Arc<KalicoHostIo>)> = participants
    .iter()
    .map(|(p, io)| (p.mcu, p.trsync_oid, Arc::clone(io)))
    .collect();

for (idx, (p, io)) in participants.iter().enumerate() {
    let engine = Arc::clone(&engine);
    let clock_of = Arc::clone(&clock_of);
    let participant_io = participant_io.clone();
    let mcu = p.mcu;
    let host_epoch = std::time::Instant::now();
    let id = io.register_frame_interceptor(
        "trsync_state",
        Some(u32::from(p.trsync_oid)),
        Box::new(move |params| {
            if params.get_u32("can_trigger") == 0 {
                return; // trip half handled by the source interceptor
            }
            let Some((now_ticks, freq)) = clock_of(mcu) else { return };
            let report64 = clock32_to_64(now_ticks, params.get_u32("clock"));
            let host_now = host_epoch.elapsed().as_secs_f64();
            let t = ticks_to_host_time(report64, now_ticks, host_now, freq);
            let sends = engine.lock().unwrap_or_else(|e| e.into_inner()).on_report(idx, t);
            for (target_idx, expire_t) in sends {
                let (tmcu, toid, tio) = &participant_io[target_idx];
                let Some((tnow, tfreq)) = clock_of(*tmcu) else { continue };
                let expire_ticks = host_time_to_ticks(expire_t, tnow, host_now, *tfreq);
                let _ = tio.send_fire_and_forget(&format!(
                    "trsync_set_timeout oid={} clock={}",
                    toid,
                    (expire_ticks & 0xFFFF_FFFF)
                ));
            }
        }),
    )?;
    registrations.push((Arc::clone(io), id));
}
```

Caveat baked into the code above: each participant's host time must come
from one shared epoch — hoist a single `host_epoch` (or better, reuse the
reactor's existing monotonic-now accessor if one exists; check
`KalicoHostIo` for a `host_now()` helper during Step 1 and prefer it) above
the loop so all participants share it. Cross-MCU time comparison is the
whole point; per-closure epochs would break the minimum computation.

- [ ] **Step 3: Update unit + integration tests**

In `trip_dispatch/tests.rs`, existing `prepare` calls gain
`participants: vec![]`, `expire_timeout_s: 0.25`, and a `clock_of` stub
returning `Some((0, 520e6))` — empty web preserves their old behavior
exactly. Add one new unit test with a fake io (whatever fake the existing
tests use for `send_fire_and_forget` capture) asserting: a `trsync_state
can_trigger=1 clock=N` frame on participant 0 of a 2-participant web emits
a `trsync_set_timeout` on participant 1's io and none on participant 0's.

In `relay_reactor_integration.rs`, add the live-reactor version: feed an
inbound `trsync_state can_trigger=1` frame through the `ReactorHarness` and
assert a `trsync_set_timeout oid=<sink> clock=…` frame appears on the other
participant's wire.

- [ ] **Step 4: Update the `bridge.rs` pyfunction**

`trip_dispatch_prepare(sources, sinks)` becomes
`trip_dispatch_prepare(sources, sinks, participants, expire_timeout_s)` with
`participants: Vec<(u32 /* mcu handle */, u8 /* trsync_oid */)>`; resolve
ios from the same table the sinks use, pass
`clock_of = router.ack_clock_and_freq` (clone the router Arc the same way
`bridge.rs` does for the pump at its `run_pump` call). Update
`klippy/motion_bridge.py::trip_dispatch_prepare` wrapper to pass the two new
arguments through.

- [ ] **Step 5: Run, fix, commit**

```bash
cd rust && cargo test -p motion-bridge 2>&1 | tail -5
python3 -m py_compile klippy/motion_bridge.py
git add -u && git commit -m "feat(bridge): TripDispatch liveness web — trsync_state reports drive timeout extension"
```

---

### Task 5: Arming — bridge sinks get real reports + expire timeouts

**Files:**
- Modify: `klippy/mcu.py` (`MCU_trsync.start`, bridge branch)
- Modify: `klippy/motion_bridge.py` (`BridgeTriggerDispatch.start`)

Reverse Part A's "no report, no expire" arming per spec §A-rev. Mainline reference: the non-bridge branch of the same function.

- [ ] **Step 1: Rewrite the bridge branch of `MCU_trsync.start`**

Replace the body of the `if self._mcu._bridge_drives_steppers:` block (keep the `_bridge_arm_id` fail-loud check and the `runtime_stop_on_trigger` send):

```python
            self._home_end_clock = None
            clock = self._mcu.print_time_to_clock(print_time)
            expire_ticks = self._mcu.seconds_to_clock(expire_timeout)
            expire_clock = clock + expire_ticks
            report_ticks = self._mcu.seconds_to_clock(expire_timeout * 0.3)
            report_clock = clock + int(report_ticks * report_offset + 0.5)
            serial = self._mcu._serial
            serial.send(
                "trsync_start oid=%d report_clock=%d report_ticks=%d"
                " expire_reason=%d"
                % (self._oid, report_clock, report_ticks,
                   self.REASON_COMMS_TIMEOUT)
            )
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
            serial.send(
                "trsync_set_timeout oid=%d clock=%d"
                % (self._oid, expire_clock & 0xFFFFFFFF)
            )
            logging.info(
                "[trsync-diag] bridge sink armed mcu=%s oid=%d arm_id=%d"
                " timeout=%.3fs",
                self._mcu._name, self._oid, arm_id, expire_timeout,
            )
            return
```

- [ ] **Step 2: `BridgeTriggerDispatch.start` — timeout selection + web wiring**

In `klippy/motion_bridge.py`, where the sinks are armed (`trsync.start(arm_print_time, 0., self._completion, 0.)`):

```python
        from .mcu import get_danger_options
        participants = []   # (mcu_handle, trsync_oid) for the liveness web
        n_participants = len(self._sink_trsyncs)  # + classic sources, Task 7
        expire_timeout = get_danger_options().multi_mcu_trsync_timeout
        if n_participants == 1:
            expire_timeout = get_danger_options().single_mcu_trsync_timeout
        try:
            sinks = []
            for i, trsync in enumerate(self._sink_trsyncs.values()):
                trsync._bridge_arm_id = self._arm_id
                report_offset = float(i) / n_participants
                trsync.start(
                    arm_print_time, report_offset,
                    self._completion, expire_timeout,
                )
                handle = trsync.get_mcu()._bridge_handle
                sinks.append((handle, trsync.get_oid()))
                participants.append((handle, trsync.get_oid()))
            sources = [(0, self._mcu, self._arm_id)]
            self._trip_handle_id = self._bridge.trip_dispatch_prepare(
                sources, sinks, participants, expire_timeout
            )
```

(Confirm `get_danger_options` import path against `klippy/mcu.py`'s own import of it; reuse the same.)

- [ ] **Step 3: Verify + commit**

```bash
python3 -m py_compile klippy/mcu.py klippy/motion_bridge.py
cd rust && cargo test -p motion-bridge 2>&1 | tail -3
git add -u && git commit -m "feat(host): arm bridge sink trsyncs with mainline report cadence + expire timeout"
```

Note for review: after this task, an armed-but-unextended sink WILL freeze
its evaluator via `REASON_COMMS_TIMEOUT` ~25–250 ms after arming if the web
isn't delivering extensions — that is the designed dead-man working. Any
"homing aborts immediately" symptom in later testing starts its diagnosis
here (are reports flowing? is Task 4's interceptor firing?).

---

### Task 6: Pump — per-move-class lead, 10 ms homing re-poll, `Flush`

**Files:**
- Modify: `rust/motion-bridge/src/pump.rs`
- Modify: `rust/motion-bridge/src/pump/tests.rs`, `rust/motion-bridge/src/pump/sched_tests.rs`
- Modify: `rust/motion-bridge/src/enqueue.rs` (EnqueueMsg construction — `lead_secs` field)
- Modify: callers of `enqueue_segment` (found via `grep -rn 'enqueue_segment' rust/motion-bridge/src/`)

- [ ] **Step 1: Failing tests** (`pump/tests.rs` style follows existing tests — adapt setup helpers)

```rust
#[test]
fn homing_lead_gates_release_to_50ms() {
    // Queue two pieces: one inside ack_now+50ms, one beyond. With
    // lead_secs=0.05 only the first is schedulable; the second StallAheads.
}

#[test]
fn flush_clears_queued_pieces_and_junctions() {
    // Enqueue 4 pieces, Flush the key, assert queue empty, pushed/retired
    // counters untouched, junction_ends cleared (next enqueue with
    // fresh_stream=false must not emit an overlap warning against a
    // flushed junction).
}
```

Write them concretely against the existing test harness in those files (they construct `AxisQueue`/`schedule` directly — follow `sched_tests.rs` patterns for the first, `tests.rs` `apply`-loop patterns for the second).

- [ ] **Step 2: Implement**

`pump.rs` changes:

```rust
pub struct AxisQueue {
    pub pieces: VecDeque<(PieceEntry, f64)>,
    pub pushed: u32,
    pub retired: u32,
    pub ring_depth: u32,
    pub physical_write_cursor: u32,
    pub lead_secs: f64,            // NEW — horizon for this queue's pieces
}
// AxisQueue::new gains `lead_secs: MAX_LEAD_SECS` default.

pub struct EnqueueMsg {
    pub key: AxisKey,
    pub pieces: Vec<(PieceEntry, f64)>,
    pub fresh_stream: bool,
    pub lead_secs: f64,            // NEW — planner sets 0.05 for homing moves
}

pub enum PumpMsg {
    Enqueue(EnqueueMsg),
    Heartbeat(HeartbeatMsg),
    Flush(Vec<AxisKey>),           // NEW — drop queued (un-pushed) pieces
    Shutdown,
}
```

In `apply`: `PumpMsg::Enqueue` sets `q.lead_secs = msg.lead_secs`;
`PumpMsg::Flush(keys)` does, per key: `q.pieces.clear(); junction_ends.remove(&key);`.

`schedule()`'s `horizon_of` parameter changes from `Fn(u32) -> Option<u64>`
to `Fn(&AxisKey, &AxisQueue) -> Option<u64>` computed in `run_pump` as:

```rust
let horizon_of = |key: &AxisKey, q: &AxisQueue| -> Option<u64> {
    let (ack_now, freq) = mcu_clock_of(key.mcu_id)?;
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    Some(ack_now + (q.lead_secs * freq) as u64)
};
```

(Mechanical update at both `horizon_of` call sites inside `schedule`.)

Re-poll: the `holding_ahead` branch's `recv_timeout` becomes

```rust
let poll = if queues.values().any(|q| q.lead_secs < 0.1 && !q.pieces.is_empty()) {
    Duration::from_millis(10)
} else {
    Duration::from_millis(50)
};
```

- [ ] **Step 3: Thread `lead_secs` through `enqueue_segment`**

`enqueue.rs::enqueue_segment` gains a `lead_secs: f64` parameter copied
into each `EnqueueMsg`. Grep callers (`planner.rs` / `dispatch.rs`), pass
`MAX_LEAD_SECS` (export it from `pump.rs`) everywhere except the homing
call path, which Task 8 switches to `0.05`. All existing pump tests gain
the new field with `lead_secs: 1.0`.

- [ ] **Step 4: Run, fix, commit**

```bash
cd rust && cargo test -p motion-bridge pump 2>&1 | tail -3
cd rust && cargo test --workspace 2>&1 | tail -3
git add -u && git commit -m "feat(pump): per-move-class lead horizon, homing re-poll, Flush message"
```

---### Task 7: Homing piece slicing (≤25 ms)

**Files:**
- Modify: `rust/motion-bridge/src/enqueue.rs`
- Modify: `rust/motion-bridge/src/enqueue/tests.rs`

- [ ] **Step 1: Failing tests**

```rust
#[test]
fn subdivide_preserves_curve_and_continuity() {
    // Cubic with distinct coeffs over 0.2s, max piece 0.025 → 8 pieces.
    let coeffs = [1.0_f64, 4.0, 2.0, 8.0];
    let pieces = subdivide_bernstein(coeffs, 0.2, 0.025);
    assert_eq!(pieces.len(), 8);
    let total: f64 = pieces.iter().map(|(_, d)| *d).sum();
    assert!((total - 0.2).abs() < 1e-12);
    // C0 continuity at every boundary.
    for w in pieces.windows(2) {
        assert!((w[0].0[3] - w[1].0[0]).abs() < 1e-12);
    }
    // Value match against the original at arbitrary sample points.
    let eval = |c: &[f64; 4], u: f64| {
        let v = 1.0 - u;
        c[0]*v*v*v + 3.0*c[1]*v*v*u + 3.0*c[2]*v*u*u + c[3]*u*u*u
    };
    for s in [0.0, 0.07, 0.13, 0.2] {
        let direct = eval(&coeffs, s / 0.2);
        let mut acc = 0.0;
        let mut found = None;
        for (c, d) in &pieces {
            if s <= acc + d + 1e-12 {
                found = Some(eval(c, ((s - acc) / d).clamp(0.0, 1.0)));
                break;
            }
            acc += d;
        }
        assert!((found.unwrap() - direct).abs() < 1e-9);
    }
}

#[test]
fn short_pieces_pass_through_unsplit() {
    let pieces = subdivide_bernstein([0.0, 1.0, 2.0, 3.0], 0.02, 0.025);
    assert_eq!(pieces.len(), 1);
}
```

- [ ] **Step 2: Verify failure, then implement**

```rust
/// Split a cubic Bernstein segment of length `duration` into equal
/// sub-pieces of at most `max_piece_secs`, via repeated de Casteljau
/// subdivision at the running left edge.
pub fn subdivide_bernstein(
    coeffs: [f64; 4],
    duration: f64,
    max_piece_secs: f64,
) -> Vec<([f64; 4], f64)> {
    if duration <= max_piece_secs {
        return vec![(coeffs, duration)];
    }
    let n = (duration / max_piece_secs).ceil() as usize;
    let sub = duration / n as f64;
    let mut out = Vec::with_capacity(n);
    let mut rest = coeffs;
    for i in 0..n - 1 {
        // Split `rest` (spanning [i*sub, duration]) at local parameter u.
        let u = sub / (duration - i as f64 * sub);
        let (left, right) = de_casteljau_split(rest, u);
        out.push((left, sub));
        rest = right;
    }
    out.push((rest, sub));
    out
}

fn de_casteljau_split(c: [f64; 4], u: f64) -> ([f64; 4], [f64; 4]) {
    let b01 = lerp(c[0], c[1], u);
    let b12 = lerp(c[1], c[2], u);
    let b23 = lerp(c[2], c[3], u);
    let b012 = lerp(b01, b12, u);
    let b123 = lerp(b12, b23, u);
    let b = lerp(b012, b123, u);
    ([c[0], b01, b012, b], [b, b123, b23, c[3]])
}

fn lerp(a: f64, b: f64, u: f64) -> f64 {
    a + (b - a) * u
}
```

In `flatten_axis`, after `let bern = bp.to_bernstein();` and the existing
degree assertions, route through the splitter when a `max_piece_secs:
Option<f64>` parameter (added to `flatten_axis` and `enqueue_segment`,
`None` for normal moves) is set:

```rust
let span = (bp.u_end - bp.u_start) as f64;
let subs = match max_piece_secs {
    Some(m) => subdivide_bernstein(coeffs_f64, span, m),
    None => vec![(coeffs_f64, span)],
};
let mut offset = 0.0;
for (sub_coeffs, sub_dur) in subs {
    let host_secs = t0 + bp.u_start + offset;
    // … existing PieceEntry construction with sub_coeffs (cast to f32)
    // and duration = sub_dur as f32 …
    offset += sub_dur;
}
```

(`coeffs_f64` is the `[f64; 4]` Bernstein array before the existing f32
cast; move the cast inside the loop.)

- [ ] **Step 3: Run, commit**

```bash
cd rust && cargo test -p motion-bridge enqueue 2>&1 | tail -3
git add -u && git commit -m "feat(enqueue): de Casteljau subdivision to <=25ms pieces for homing moves"
```

---

### Task 8: Homing move classification + flush-on-completion wiring

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs` (homing submit path), `rust/motion-bridge/src/bridge.rs`
- Modify: `rust/motion-bridge/src/homing.rs` if hook signatures need it
- Test: extend `rust/motion-bridge/src/homing/tests.rs` only if logic (not wiring) is added

This is the integration task; it begins with reading.

- [ ] **Step 1: Read the homing motion path**

Read `bridge.rs::submit_homing_move` and follow it through `planner.rs` to
the `enqueue_segment` call; identify where a homing-class move is
distinguishable (the submit path sets `homing.begin(arm_id)` — the same
scope knows the move is homing).

- [ ] **Step 2: Pass the class down**

At the homing `enqueue_segment` call site: `max_piece_secs = Some(0.025)`,
`lead_secs = 0.05`. Normal path: `None` / `MAX_LEAD_SECS`. Define both
constants in one place:

```rust
// rust/motion-bridge/src/homing.rs
pub const DRIP_PIECE_SECS: f64 = 0.025;
pub const DRIP_MAX_AHEAD_SECS: f64 = 0.05;
```

- [ ] **Step 3: Flush on homing end**

`bridge.rs` keeps a `pump_tx` clone on the bridge struct (it currently only
moves clones into closures at `init_planner` — add `self.pump_tx = Some(pump_tx_init.clone())`).
At the two places homing ends — the trip event being recorded
(`homing.rs::take_trip_event`'s caller / the `kalico_endstop_tripped`
handler in `bridge.rs`) and natural completion (`wait_moves` path /
`refresh_after_wait`) — send `PumpMsg::Flush(homing_axis_keys)` where
`homing_axis_keys` are the `AxisKey`s of the MCUs/axes the homing move was
enqueued to (the submit path knows them; store on `HomingState` as a
`Vec<(u32, u8)>` alongside `active_segment_id`).

Late or duplicate flushes are harmless by design (clearing an empty queue);
do NOT add guards against them.

- [ ] **Step 4: Verify + commit**

```bash
cd rust && cargo test --workspace 2>&1 | tail -3
python3 -m py_compile klippy/motion_toolhead.py
git add -u && git commit -m "feat(bridge): homing moves slice to 25ms pieces, 50ms lead, flush on completion"
```

---

### Task 9: Beacon fold-in — delete `probe_homing.rs`

**Files:**
- Delete: `rust/motion-bridge/src/probe_homing.rs` (+ its tests, `mod` decl in `lib.rs`)
- Modify: `rust/motion-bridge/src/bridge.rs` (delete `prepare_probe_homing`/`run_probe_homing`/`cleanup_probe_homing` pyfunctions)
- Modify: `klippy/motion_bridge.py` (delete wrappers + whitelist entries), `klippy/motion_toolhead.py` (probe branch)

- [ ] **Step 1: Read the probe branch**

Read `klippy/motion_toolhead.py` lines ~547–713 (the software-trip /
`run_probe_homing` flow) and the Beacon arming path it relies on, plus how
Beacon's `MCU_trsync` is created (non-bridge branch — it sends real
firmware commands; confirm whether they go via klippy serialqueue or the
bridge serial shim, and confirm the chelper `trdispatch_mcu_alloc` in
`MCU_trsync._build_config` is skipped or inert for shim-backed MCUs — if it
would bind a real C trdispatch to a serialqueue the MCU doesn't use, guard
it out for `_bridge_drives_steppers` is already false for Beacon, so check
what `mcu._serial.get_serialqueue()` returns there and guard if needed).

- [ ] **Step 2: Re-route the probe flow through the common path**

The probe branch becomes the same shape as the GPIO branch:
`BridgeTriggerDispatch` for the Z arm gains Beacon as an extra participant —
in `start()`, when an external probe trsync is registered (new optional
field `self._classic_participants: list[(mcu_handle, trsync_oid, MCU_trsync)]`
populated by the probe glue before `home_start`):

- arm it via its existing `MCU_trsync.start(print_time, report_offset, completion, expire_timeout)` non-bridge branch (real firmware commands — mainline behavior, stock Beacon contract),
- add `(1, beacon_mcu_handle, trsync_oid)` to `sources` (kind 1 = `SourceSpec::Trsync`),
- add it to `participants` so the web extends its timeout,
- count it in `n_participants` for the single/multi timeout choice.

Delete from `motion_toolhead.py`: the `run_probe_homing` call loop, the
`PROBE_TRIGGERED`/`SEGMENT_RETIRED`/`SENSOR_FAULT` constants and branches —
the flow is now: arm (with Beacon participant) → `submit_homing_move` →
`wait_moves` → read homing reason, identical to GPIO homing. The trip
arrives as a relayed `trsync_trigger` freezing F446 via
`runtime_stop_on_trigger`; a hung Beacon stalls the web and times the group
out (`REASON_COMMS_TIMEOUT` → completion failure → homing error).

- [ ] **Step 3: Delete and verify**

```bash
git rm rust/motion-bridge/src/probe_homing.rs
# remove `mod probe_homing;` from lib.rs, pyfunctions from bridge.rs,
# wrappers from motion_bridge.py
grep -rn 'probe_homing\|run_probe_homing\|PROBE_TRIGGERED\|SENSOR_FAULT' rust/ klippy/ --include='*.rs' --include='*.py'
```

Expected after cleanup: zero hits outside docs. Then:

```bash
cd rust && cargo test --workspace 2>&1 | tail -3
python3 -m py_compile klippy/motion_toolhead.py klippy/motion_bridge.py klippy/mcu.py
git add -A && git commit -m "refactor(homing): fold Beacon into TripDispatch; delete probe_homing special case"
```

---

### Task 10: `software_trip` disarm-ordering contract

**Files:**
- Read: `rust/runtime/src/endstop.rs` (`software_trip` handler)
- Test: `rust/runtime/src/endstop/tests.rs`
- Modify: `klippy/motion_bridge.py` (`BridgeTriggerDispatch.stop` ordering) if needed

- [ ] **Step 1: Write the contract test**

In `endstop/tests.rs` (follow the file's existing arm/tick/trip helpers):

```rust
#[test]
fn software_trip_on_disarmed_arm_is_a_no_op() {
    // Arm, disarm, then software_trip with the stale arm_id: no snapshot
    // published, no TripAction::AbortNow on subsequent ticks.
}

#[test]
fn software_trip_with_mismatched_arm_id_is_a_no_op() {
    // Armed with arm_id=7; software_trip(arm_id=6) is ignored.
}
```

Write the bodies against the real helpers in that file (`sw_msg`/`arm`/
`drain_trip` style seen at lines 27–60).

- [ ] **Step 2: Run; implement the guard only if a test fails**

If `software_trip` already guards on `ArmState`/`arm_id` (likely — check
around the `TRIP_SOURCE_SOFTWARE` path), the tests pass as-is and document
the contract. If not, add the guard: mismatched/inactive → return without
publishing (this is the one deliberate exception to fail-loud: mainline's
`stepper_stop` on stopped steppers is equally silent, and the host-request
disarm path depends on it).

- [ ] **Step 3: Verify stop ordering in Python**

In `BridgeTriggerDispatch.stop()` confirm `endstop_disarm` precedes any
sink-trsync query/trigger teardown; reorder if not.

```bash
cd rust && cargo test -p runtime endstop 2>&1 | tail -3
git add -u && git commit -m "test(runtime): software_trip disarm-ordering contract"
```

---

### Task 11: Full verification + spec status

- [ ] **Step 1: Suites**

```bash
cd rust && cargo test --workspace 2>&1 | tail -5
cd rust && cargo clippy --workspace 2>&1 | tail -3
python3 -m py_compile klippy/mcu.py klippy/motion_bridge.py klippy/motion_toolhead.py
grep -rn -E 'extend_homing_deadline|grant_ticks|deadline_clock|probe_homing' rust/ src/ klippy/ --include='*.rs' --include='*.c' --include='*.py'
```

Expected: zero failures, zero clippy warnings, zero grep hits.

- [ ] **Step 2: Firmware dict check (compile-only, host-side cross-check deferred to bench)**

Confirm `src/runtime_commands.c` still declares `runtime_stop_on_trigger`
(from the merge) and nothing references deleted symbols:

```bash
grep -n 'runtime_stop_on_trigger\|trsync' src/runtime_commands.c | head
```

- [ ] **Step 3: Commit any stragglers; update spec**

Append to the spec's Testing section: "Rungs 1–2 (unit + integration)
complete as of <commit>." Commit.

---

## Deferred to the bench (separate session, `flashing-trident-mcus` + ladder)

Per the spec's testing ladder and the bench-flow memory (commit → push →
pull → build on Pi → flash, both MCUs, `make clean` between):

1. Renode dual-MCU sim (`tools/sim/dual_mcu_docker.resc`): trip on A freezes
   B; silence on A expires B (`REASON_COMMS_TIMEOUT`).
2. Sensorless X on H7 through the relay (siren disabled, free air). If
   homing aborts with comms-timeout immediately: check report flow first
   (Task 5 note); the escape hatch is raising
   `multi_mcu_trsync_timeout`/`single_mcu_trsync_timeout` in the danger
   options — a config change, not a code change.
3. Beacon + F446 Z homing.

## Self-review notes

- Spec coverage: liveness web → Tasks 2–5; metered drip → Tasks 6–8; A-rev
  arming → Task 5; A-rev probe deletion → Task 9; disarm contract → Task 10;
  natural-end loud failure → existing `HomingState::Completed` path,
  exercised via Task 8's wait/refresh wiring; constants table → Tasks 5
  (timeouts, report cadence), 2 (hysteresis), 8 (drip constants).
- Known adapt-points (read-first steps): exact `register_frame_interceptor`
  duplicate-key semantics (Task 4), `enqueue_segment` caller list (Task 6),
  homing submit path shape (Task 8), Beacon serial path (Task 9). Each task
  names the file and the question; none invents new mechanisms beyond the
  spec.
