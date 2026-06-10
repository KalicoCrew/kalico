# Executed-Motion History Service Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Always-on bounded history of every dispatched motion curve, queryable as commanded (position, velocity, acceleration) per axis at a past time — exact, resync-immune; homing trip reconstruction migrates onto it.

**Architecture:** A `HistoryStore` (ring of `HistoryPiece` per `AxisKey` + hold-state endpoint registers) lives on the bridge, fed from the dispatch callback that already mirrors homing pieces. End clocks are precomputed at record time with the per-MCU **nominal** clock frequency (the same constant the MCU ISR uses — `src/runtime_tick.c:22` passes `CONFIG_CLOCK_FREQ`), replicating `PieceEntry::end_time` bit-for-bit, so within-domain queries touch zero sync state. The `(mcu, clock)` FFI query converts cross-MCU via the router path homing already uses; Python resolves `print_time` via existing clocksync.

**Tech Stack:** Rust (motion-bridge crate, PyO3), Python (klippy), `cargo nextest` for tests.

**Spec:** `docs/superpowers/specs/2026-06-10-motion-history-service-design.md`

**House rules that bind every task:** unit tests in a separate file from the tested code; no narrative comments (encode invariants in code/asserts); fail loudly — never recover silently; run `cargo fmt --all --check` before any push.

---

## File structure

| File | Responsibility |
|---|---|
| Create `rust/motion-bridge/src/motion_history.rs` | `HistoryPiece`, `HistoryStore`, `AxisState`, `HistoryError`, Bernstein eval + derivatives, cross-MCU clock helper |
| Create `rust/motion-bridge/src/motion_history/tests.rs` | All unit tests for the above |
| Modify `rust/motion-bridge/src/lib.rs` | register `pub mod motion_history;` |
| Modify `rust/motion-bridge/src/bridge.rs` | store field + nominal-freq map + recording + rebase + `motion_state_at_clock`/`set_nominal_clock_freq` pymethods; delete `homing_trajectory` |
| Modify `rust/motion-bridge/src/dispatch.rs` | `DispatchError::MissingNominalFreq` |
| Modify `rust/motion-bridge/src/homing.rs` + `homing/tests.rs` | reconstruct via `HistoryStore` + stale-trip window guard; eval fn moves to motion_history |
| Modify `klippy/motion_bridge.py` | wrapper: `set_nominal_clock_freq`, `motion_state_at`, `set_position` gains host_now |
| Modify `klippy/mcu.py` | push nominal freq after claim+identify |
| Modify `klippy/kinematics/extruder.py` | `find_past_position` → bridge query |
| Modify `klippy/stepper.py` | fail-loud `get_past_mcu_position` |
| Modify `klippy/motion_toolhead.py` | `KALICO_SIM_MOTION_STATE` debug gcode |
| Modify `tools/kalico-sim/runner.py` | `--test-motion-state` validation block |

---

### Task 1: `motion_history.rs` core — store, eval, semantics

**Files:**
- Create: `rust/motion-bridge/src/motion_history.rs`
- Create: `rust/motion-bridge/src/motion_history/tests.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (add `pub mod motion_history;` alongside the existing `pub mod homing;`)

- [ ] **Step 1: Write the failing tests**

`rust/motion-bridge/src/motion_history/tests.rs`:

```rust
use runtime::piece_ring::PieceEntry;

use crate::motion_history::{HISTORY_CAPACITY, HistoryError, HistoryPiece, HistoryStore};
use crate::pump::AxisKey;

const FREQ: u32 = 520_000_000;

fn key() -> AxisKey {
    AxisKey { mcu_id: 7, axis: 2 }
}

fn entry(start_time: u64, duration: f32, coeffs: [f32; 4]) -> PieceEntry {
    PieceEntry { start_time, coeffs, duration, _reserved: 0 }
}

fn linear(start_time: u64, duration: f32, p0: f32, p1: f32) -> PieceEntry {
    let third = (p1 - p0) / 3.0;
    entry(start_time, duration, [p0, p0 + third, p0 + 2.0 * third, p1])
}

#[test]
fn end_clock_matches_isr_formula() {
    let e = entry(1_000, 0.0123, [0.0; 4]);
    let h = HistoryPiece::from_entry(&e, FREQ);
    assert_eq!(h.end_clock, e.end_time(FREQ as f32));
    assert_eq!(h.start_clock, 1_000);
}

#[test]
fn linear_piece_position_velocity_acceleration() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 1.0, 0.0, 10.0), FREQ);
    let mid = FREQ as u64 / 2;
    let st = store.state_at_clock(key(), mid, Some(u64::MAX)).unwrap();
    assert!((st.position - 5.0).abs() < 1e-6);
    assert!((st.velocity - 10.0).abs() < 1e-6);
    assert!(st.acceleration.abs() < 1e-6);
}

#[test]
fn quadratic_piece_derivatives() {
    let mut store = HistoryStore::default();
    store.record(key(), &entry(0, 1.0, [0.0, 0.0, 5.0, 15.0]), FREQ);
    let mid = FREQ as u64 / 2;
    let st = store.state_at_clock(key(), mid, Some(u64::MAX)).unwrap();
    assert!((st.position - 3.75).abs() < 1e-5);
    assert!((st.velocity - 15.0).abs() < 1e-5);
    assert!((st.acceleration - 30.0).abs() < 1e-4);
}

#[test]
fn gap_between_pieces_holds_previous_endpoint() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 0.001, 0.0, 10.0), FREQ);
    let gap_start = HistoryPiece::from_entry(&linear(0, 0.001, 0.0, 10.0), FREQ).end_clock;
    store.record(key(), &linear(gap_start + 1_000_000, 0.001, 10.0, 20.0), FREQ);
    let st = store
        .state_at_clock(key(), gap_start + 500_000, Some(u64::MAX))
        .unwrap();
    assert!((st.position - 10.0).abs() < 1e-6);
    assert_eq!(st.velocity, 0.0);
    assert_eq!(st.acceleration, 0.0);
}

#[test]
fn after_last_piece_holds_when_not_future() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 0.001, 0.0, 10.0), FREQ);
    let end = store.state_at_clock(key(), 519_999, Some(u64::MAX)).unwrap();
    assert!((end.position - 10.0).abs() < 1e-4);
    let held = store
        .state_at_clock(key(), 5_000_000, Some(10_000_000))
        .unwrap();
    assert!((held.position - 10.0).abs() < 1e-6);
}

#[test]
fn hold_in_the_future_is_an_error() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 0.001, 0.0, 10.0), FREQ);
    let err = store
        .state_at_clock(key(), 5_000_000, Some(1_000_000))
        .unwrap_err();
    assert!(matches!(err, HistoryError::QueryInFuture { .. }));
}

#[test]
fn inside_committed_future_piece_evaluates() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 1.0, 0.0, 10.0), FREQ);
    let st = store
        .state_at_clock(key(), FREQ as u64 / 2, Some(1_000))
        .unwrap();
    assert!((st.position - 5.0).abs() < 1e-6);
}

#[test]
fn before_window_is_an_error() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(1_000_000, 0.001, 0.0, 10.0), FREQ);
    let err = store.state_at_clock(key(), 500, Some(u64::MAX)).unwrap_err();
    assert!(matches!(err, HistoryError::BeforeRetainedWindow { .. }));
}

#[test]
fn unknown_axis_is_an_error() {
    let store = HistoryStore::default();
    let err = store.state_at_clock(key(), 0, Some(u64::MAX)).unwrap_err();
    assert!(matches!(err, HistoryError::NoHistoryForAxis(_)));
}

#[test]
fn rebase_clears_ring_and_answers_from_register() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(0, 1.0, 0.0, 10.0), FREQ);
    store.rebase_axis(key(), 2_000_000_000, 42.0);
    let held = store
        .state_at_clock(key(), 2_000_000_500, Some(3_000_000_000))
        .unwrap();
    assert!((held.position - 42.0).abs() < 1e-9);
    let err = store
        .state_at_clock(key(), 1_000, Some(u64::MAX))
        .unwrap_err();
    assert!(matches!(err, HistoryError::BeforeRetainedWindow { .. }));
}

#[test]
fn eviction_keeps_capacity_and_reports_true_window() {
    let mut store = HistoryStore::default();
    let dur = 0.001_f32;
    let dur_ticks = (dur * FREQ as f32) as u64;
    for i in 0..(HISTORY_CAPACITY as u64 + 10) {
        store.record(key(), &linear(i * dur_ticks, dur, 0.0, 1.0), FREQ);
    }
    let err = store.state_at_clock(key(), 0, Some(u64::MAX)).unwrap_err();
    match err {
        HistoryError::BeforeRetainedWindow { window_start, .. } => {
            assert_eq!(window_start, 10 * dur_ticks);
        }
        other => panic!("expected BeforeRetainedWindow, got {other:?}"),
    }
}
```

- [ ] **Step 2: Run tests to verify they fail to compile**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(motion_history)'`
Expected: compile error — `motion_history` module not found.

- [ ] **Step 3: Write the implementation**

`rust/motion-bridge/src/motion_history.rs`:

```rust
use std::collections::{HashMap, VecDeque};

use runtime::piece_ring::PieceEntry;

use crate::pump::AxisKey;

pub const HISTORY_CAPACITY: usize = 4096;

#[derive(Debug, thiserror::Error)]
pub enum HistoryError {
    #[error(
        "query clock {queried} precedes retained motion history for axis \
         {key:?} (window {window_start}..{window_end})"
    )]
    BeforeRetainedWindow {
        key: AxisKey,
        queried: u64,
        window_start: u64,
        window_end: u64,
    },

    #[error(
        "query clock {queried} is in the future for axis {key:?} \
         (now≈{now_clock}) — motion history answers the past only"
    )]
    QueryInFuture {
        key: AxisKey,
        queried: u64,
        now_clock: u64,
    },

    #[error("no motion history recorded for axis {0:?}")]
    NoHistoryForAxis(AxisKey),
}

#[derive(Debug, Clone, Copy)]
pub struct HistoryPiece {
    pub start_clock: u64,
    pub end_clock: u64,
    pub duration_secs: f32,
    pub coeffs: [f32; 4],
}

impl HistoryPiece {
    pub fn from_entry(entry: &PieceEntry, nominal_freq_hz: u32) -> Self {
        #[allow(clippy::cast_precision_loss)]
        let end_clock = entry.end_time(nominal_freq_hz as f32);
        Self {
            start_clock: entry.start_time,
            end_clock,
            duration_secs: entry.duration,
            coeffs: entry.coeffs,
        }
    }

    fn endpoint(&self) -> AxisEndpoint {
        AxisEndpoint {
            clock: self.end_clock,
            position: f64::from(self.coeffs[3]),
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct AxisState {
    pub position: f64,
    pub velocity: f64,
    pub acceleration: f64,
}

#[derive(Debug, Clone, Copy)]
struct AxisEndpoint {
    clock: u64,
    position: f64,
}

impl AxisEndpoint {
    fn hold_state(&self) -> AxisState {
        AxisState {
            position: self.position,
            velocity: 0.0,
            acceleration: 0.0,
        }
    }
}

#[inline]
pub fn eval_bernstein_cubic(coeffs: [f32; 4], u: f64) -> f64 {
    let v = 1.0 - u;
    let b0 = f64::from(coeffs[0]);
    let b1 = f64::from(coeffs[1]);
    let b2 = f64::from(coeffs[2]);
    let b3 = f64::from(coeffs[3]);
    v * v * v * b0 + 3.0 * v * v * u * b1 + 3.0 * v * u * u * b2 + u * u * u * b3
}

fn eval_state(piece: &HistoryPiece, clock: u64) -> AxisState {
    #[allow(clippy::cast_precision_loss)]
    let dur_ticks = piece.end_clock.saturating_sub(piece.start_clock) as f64;
    #[allow(clippy::cast_precision_loss)]
    let u = if dur_ticks > 0.0 {
        ((clock - piece.start_clock) as f64 / dur_ticks).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let v = 1.0 - u;
    let b0 = f64::from(piece.coeffs[0]);
    let b1 = f64::from(piece.coeffs[1]);
    let b2 = f64::from(piece.coeffs[2]);
    let b3 = f64::from(piece.coeffs[3]);
    let t = f64::from(piece.duration_secs);
    let (velocity, acceleration) = if t > 0.0 {
        let db = 3.0 * ((b1 - b0) * v * v + 2.0 * (b2 - b1) * v * u + (b3 - b2) * u * u);
        let d2b = 6.0 * ((b2 - 2.0 * b1 + b0) * v + (b3 - 2.0 * b2 + b1) * u);
        (db / t, d2b / (t * t))
    } else {
        (0.0, 0.0)
    };
    AxisState {
        position: eval_bernstein_cubic(piece.coeffs, u),
        velocity,
        acceleration,
    }
}

#[derive(Debug, Default)]
pub struct HistoryStore {
    rings: HashMap<AxisKey, VecDeque<HistoryPiece>>,
    endpoints: HashMap<AxisKey, AxisEndpoint>,
}

impl HistoryStore {
    pub fn record(&mut self, key: AxisKey, entry: &PieceEntry, nominal_freq_hz: u32) {
        let piece = HistoryPiece::from_entry(entry, nominal_freq_hz);
        let ring = self.rings.entry(key).or_default();
        if let Some(last) = ring.back() {
            debug_assert!(
                piece.start_clock >= last.start_clock,
                "out-of-order piece for {key:?}: {} < {}",
                piece.start_clock,
                last.start_clock
            );
        }
        if ring.len() == HISTORY_CAPACITY {
            ring.pop_front();
        }
        self.endpoints.insert(key, piece.endpoint());
        ring.push_back(piece);
    }

    pub fn rebase_axis(&mut self, key: AxisKey, clock: u64, position: f64) {
        self.rings.entry(key).or_default().clear();
        self.endpoints.insert(key, AxisEndpoint { clock, position });
    }

    pub fn last_endpoint_clock(&self, key: AxisKey) -> u64 {
        self.endpoints.get(&key).map_or(0, |e| e.clock)
    }

    pub fn state_at_clock(
        &self,
        key: AxisKey,
        clock: u64,
        now_clock: Option<u64>,
    ) -> Result<AxisState, HistoryError> {
        let ring = self.rings.get(&key).filter(|r| !r.is_empty());
        let hold = match ring {
            Some(ring) => {
                let idx = ring.partition_point(|p| p.start_clock <= clock);
                if idx == 0 {
                    return Err(HistoryError::BeforeRetainedWindow {
                        key,
                        queried: clock,
                        window_start: ring.front().map_or(0, |p| p.start_clock),
                        window_end: ring.back().map_or(0, |p| p.end_clock),
                    });
                }
                let piece = &ring[idx - 1];
                if clock < piece.end_clock {
                    return Ok(eval_state(piece, clock));
                }
                piece.endpoint()
            }
            None => {
                let endpoint = self
                    .endpoints
                    .get(&key)
                    .ok_or(HistoryError::NoHistoryForAxis(key))?;
                if clock < endpoint.clock {
                    return Err(HistoryError::BeforeRetainedWindow {
                        key,
                        queried: clock,
                        window_start: endpoint.clock,
                        window_end: endpoint.clock,
                    });
                }
                *endpoint
            }
        };
        if let Some(now_clock) = now_clock {
            if clock > now_clock {
                return Err(HistoryError::QueryInFuture {
                    key,
                    queried: clock,
                    now_clock,
                });
            }
        }
        Ok(hold.hold_state())
    }
}

#[cfg(test)]
mod tests;
```

In `rust/motion-bridge/src/lib.rs`, add `pub mod motion_history;` next to `pub mod homing;`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(motion_history)'`
Expected: all 11 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/motion_history.rs rust/motion-bridge/src/motion_history/tests.rs rust/motion-bridge/src/lib.rs
git commit -m "feat(bridge): motion-history store — dispatched-piece ring with hold-state and exact tick anchoring"
```

---

### Task 2: cross-MCU clock helper — extract from homing, share

**Files:**
- Modify: `rust/motion-bridge/src/motion_history.rs` (add `clock_between_mcus`)
- Modify: `rust/motion-bridge/src/motion_history/tests.rs`
- Modify: `rust/motion-bridge/src/homing.rs:109-143` (use the helper)

- [ ] **Step 1: Write the failing test**

Append to `rust/motion-bridge/src/motion_history/tests.rs` (mirror the mocked-router pattern already used in `rust/motion-bridge/src/homing/tests.rs` — copy its router construction helper verbatim if one exists there; otherwise construct `PassthroughRouter` and feed `set_clock_est`-equivalent state the same way that file does):

```rust
#[test]
fn clock_between_mcus_round_trips_through_host_secs() {
    // Construct a PassthroughRouter with two MCUs whose sync state is:
    //   mcu A: freq 500 MHz, offset such that clock 1_000_000 == host 2.0 s
    //   mcu B: freq 400 MHz, offset such that host 2.0 s == clock 3_000_000
    // (use the same router-construction helper as homing/tests.rs)
    let router = stub_router_two_mcus();
    let got = crate::motion_history::clock_between_mcus(
        &router,
        crate::types::mcu_handle_from_raw(1),
        crate::types::mcu_handle_from_raw(2),
        1_000_000,
    )
    .unwrap();
    assert_eq!(got, 3_000_000);
}
```

If `homing/tests.rs` has no reusable router stub, build `stub_router_two_mcus()` in this test file using the public `PassthroughRouter` API (`set_clock_est`-equivalent setter used by `bridge.rs:set_clock_est`) — read `rust/kalico-host-rt/src/passthrough_queue/router.rs:423-490` for the record fields (`clock_freq`, `clock_offset`, `last_clock`).

- [ ] **Step 2: Run test to verify it fails**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(clock_between_mcus)'`
Expected: compile error — function not defined.

- [ ] **Step 3: Implement the helper and refactor homing.rs**

In `motion_history.rs`:

```rust
use kalico_host_rt::passthrough_queue::PassthroughRouter;
use kalico_host_rt::types::McuHandle;

pub fn clock_between_mcus(
    router: &PassthroughRouter,
    source: McuHandle,
    target: McuHandle,
    clock: u64,
) -> Result<u64, String> {
    if source == target {
        return Ok(clock);
    }
    let host_secs = router.clock_to_host_secs(source, clock).ok_or_else(|| {
        format!("clock_to_host_secs returned None for source mcu {source:?}")
    })?;
    router
        .host_time_to_mcu_clock(target, host_secs)
        .map_err(|e| format!("host_time_to_mcu_clock failed for target mcu {target:?}: {e:?}"))
}
```

(Adjust the `McuHandle` import path to whatever `homing.rs` uses today — it calls `crate::types::mcu_handle_from_raw`.) Then replace the inline conversion in `homing.rs:109-143` (`reconstruct_axis_position`'s `axis_clock` computation) with a call to this helper, preserving the `ReconstructError::ClockUnsynced` wrapping:

```rust
let axis_clock = {
    let router_guard = router.lock().unwrap_or_else(|p| p.into_inner());
    crate::motion_history::clock_between_mcus(
        &router_guard,
        crate::types::mcu_handle_from_raw(endstop_mcu),
        crate::types::mcu_handle_from_raw(axis_mcu),
        trip_clock,
    )
    .map_err(|description| {
        ReconstructError::ClockUnsynced {
            description,
            endstop_mcu,
            axis_mcu,
            trip_clock,
        }
        .to_string()
    })?
};
```

- [ ] **Step 4: Run the crate tests**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: PASS (including existing homing tests — behavior unchanged).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/motion_history.rs rust/motion-bridge/src/motion_history/tests.rs rust/motion-bridge/src/homing.rs
git commit -m "refactor(bridge): shared cross-MCU clock conversion in motion_history"
```

---

### Task 3: nominal clock-frequency plumbing

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (field + pymethod)
- Modify: `klippy/motion_bridge.py` (wrapper method)
- Modify: `klippy/mcu.py` (call after claim + identify)

- [ ] **Step 1: Add the bridge field and pymethod**

In `bridge.rs`, next to the `clock_freqs` field (line 465):

```rust
nominal_clock_freqs: Arc<Mutex<HashMap<u32, u32>>>,
```

Init next to line 693: `nominal_clock_freqs: Arc::new(Mutex::new(HashMap::new())),`

New pymethod inside the `#[pymethods]` block (near `set_clock_est`, line ~2070):

```rust
#[pyo3(signature = (mcu, freq_hz))]
fn set_nominal_clock_freq(&self, mcu: u32, freq_hz: u32) -> PyResult<()> {
    if freq_hz == 0 {
        return Err(PyRuntimeError::new_err(
            "set_nominal_clock_freq: freq_hz must be nonzero",
        ));
    }
    self.nominal_clock_freqs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert(mcu, freq_hz);
    Ok(())
}
```

- [ ] **Step 2: Wrapper method in `klippy/motion_bridge.py`**

Next to `set_drive_limits` (line ~155):

```python
def set_nominal_clock_freq(self, mcu_handle, freq_hz):
    return self._bridge.set_nominal_clock_freq(mcu_handle, int(freq_hz))
```

- [ ] **Step 3: Call site in `klippy/mcu.py`**

`self._mcu_freq` is set from identify constants at `mcu.py:1174`; the bridge handle claim block is at `mcu.py:1230-1237`. Insert immediately after the claim block (where `handle = self._bridge_handle` is available), verifying ordering at edit time — `_mcu_freq` must already be parsed at that point; if it is not, move the call to directly after line 1174 instead and use `self._bridge_handle`:

```python
if not self._mcu_freq:
    raise error("MCU '%s': CLOCK_FREQ unknown at bridge claim time" % (self._name,))
self._motion_bridge.set_nominal_clock_freq(handle, int(self._mcu_freq))
```

- [ ] **Step 4: Build check**

Run: `cd rust && cargo nextest run -p motion-bridge` and `ruff check klippy/mcu.py klippy/motion_bridge.py`
Expected: PASS / clean.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs klippy/motion_bridge.py klippy/mcu.py
git commit -m "feat(bridge): per-MCU nominal clock frequency plumbing"
```

---

### Task 4: record every dispatched piece; rebase on set_position

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (store field, dispatch callback at 2695-2733, `set_position` at 2826)
- Modify: `rust/motion-bridge/src/dispatch.rs` (new error variant)
- Modify: `klippy/motion_bridge.py` (`set_position` passes host_now)

- [ ] **Step 1: Add the store field**

In `bridge.rs` next to `homing_trajectory` (line 471):

```rust
motion_history: Arc<Mutex<crate::motion_history::HistoryStore>>,
```

Init (line ~699): `motion_history: Arc::new(Mutex::new(crate::motion_history::HistoryStore::default())),`

- [ ] **Step 2: New `DispatchError` variant**

In `dispatch.rs` where `DispatchError` is defined (search `enum DispatchError`):

```rust
MissingNominalFreq(u32),
```

with display text `"no nominal clock frequency registered for mcu {0} — set_nominal_clock_freq was not called"` (match the enum's existing error-derive style).

- [ ] **Step 3: Record in the dispatch callback**

At `bridge.rs:2653`, alongside `homing_traj_for_cb`, clone the new handles:

```rust
let motion_history_for_cb = Arc::clone(&self.motion_history);
let nominal_freqs_for_cb = Arc::clone(&self.nominal_clock_freqs);
```

In the message loop (lines 2715-2727), record unconditionally **in addition to** the existing drip-gated `homing_trajectory` insert (the old map is deleted in Task 6):

```rust
for mut m in msgs {
    let nominal_freq = {
        let freqs = nominal_freqs_for_cb.lock().unwrap_or_else(|p| p.into_inner());
        *freqs
            .get(&m.key.mcu_id)
            .ok_or(DispatchError::MissingNominalFreq(m.key.mcu_id))?
    };
    {
        let mut store = motion_history_for_cb.lock().unwrap_or_else(|p| p.into_inner());
        for (piece, _host_t) in &m.pieces {
            store.record(m.key, piece, nominal_freq);
        }
    }
    if let Some(cohort) = active_cohort {
        m.drip_cohort = Some(cohort);
        let mut traj = homing_traj_for_cb.lock().unwrap_or_else(|p| p.into_inner());
        let entry = traj.entry(m.key).or_default();
        for (piece, _host_t) in &m.pieces {
            entry.push(*piece);
        }
    }
    drain_disp.add_sent(m.key.mcu_id, m.key.axis, m.pieces.len() as u32);
    pump_tx_for_cb
        .send(crate::pump::PumpMsg::Enqueue(m))
        .map_err(|_| DispatchError::PumpGone)?;
}
```

- [ ] **Step 4: Rebase on `set_position`**

Change the pymethod at `bridge.rs:2826` to accept host time and rebase X/Y/Z rings (E is planner-continuous across G92 and is not rebased):

```rust
#[pyo3(signature = (x, y, z, host_now))]
fn set_position(&self, py: Python<'_>, x: f64, y: f64, z: f64, host_now: f64) -> PyResult<()> {
    // ...existing commanded_pos update stays...
    let configs: Vec<McuAxisConfig> = self
        .mcu_axis_configs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();
    let positions = [x, y, z];
    let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
    let mut store = self.motion_history.lock().unwrap_or_else(|p| p.into_inner());
    for cfg in &configs {
        let handle = crate::types::mcu_handle_from_raw(cfg.mcu_id);
        let now_clock = router.host_time_to_mcu_clock(handle, host_now).unwrap_or(0);
        for &axis in cfg.axes.iter().filter(|&&a| a < 3) {
            let key = crate::pump::AxisKey { mcu_id: cfg.mcu_id, axis: axis as u8 };
            store.rebase_axis(key, now_clock, positions[axis]);
        }
    }
    // ...rest of existing body...
    Ok(())
}
```

In `klippy/motion_bridge.py`, find the `set_position` passthrough (or add one if calls go through `__getattr__`) so the Python side supplies the time:

```python
def set_position(self, x, y, z):
    return self._bridge.set_position(x, y, z, self._reactor.monotonic())
```

Verify the only Rust-side caller signature change compiles; the Python caller is `klippy/motion_toolhead.py:196` (`BridgeKinematics.set_position`) and goes through the wrapper, so it needs no change.

- [ ] **Step 5: Run tests + commit**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: PASS.

```bash
git add rust/motion-bridge/src/bridge.rs rust/motion-bridge/src/dispatch.rs klippy/motion_bridge.py
git commit -m "feat(bridge): record all dispatched pieces into motion history; rebase XYZ on set_position"
```

---

### Task 5: the query — `motion_state_at_clock` FFI + Python `motion_state_at`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (pymethod, near `home_axis_poll` at 3121)
- Modify: `klippy/motion_bridge.py` (wrapper)

- [ ] **Step 1: Pymethod**

```rust
#[pyo3(signature = (source_mcu, clock, host_now))]
fn motion_state_at_clock(
    &self,
    source_mcu: u32,
    clock: u64,
    host_now: f64,
) -> PyResult<std::collections::HashMap<String, (f64, f64, f64)>> {
    const AXIS_NAMES: [&str; 4] = ["x", "y", "z", "e"];
    let configs: Vec<McuAxisConfig> = self
        .mcu_axis_configs
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .clone();
    if configs.is_empty() {
        return Err(PyRuntimeError::new_err(
            "motion_state_at: no axes configured on the bridge",
        ));
    }
    let mut resolved: Vec<(crate::pump::AxisKey, u64, u64)> = Vec::new();
    {
        let router = self.router.lock().unwrap_or_else(|p| p.into_inner());
        let source_handle = crate::types::mcu_handle_from_raw(source_mcu);
        for cfg in &configs {
            let target_handle = crate::types::mcu_handle_from_raw(cfg.mcu_id);
            let axis_clock = crate::motion_history::clock_between_mcus(
                &router,
                source_handle,
                target_handle,
                clock,
            )
            .map_err(PyRuntimeError::new_err)?;
            let now_clock = router
                .host_time_to_mcu_clock(target_handle, host_now)
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "motion_state_at: clock unsynced for mcu {}: {e:?}",
                        cfg.mcu_id
                    ))
                })?;
            for &axis in &cfg.axes {
                let key = crate::pump::AxisKey { mcu_id: cfg.mcu_id, axis: axis as u8 };
                resolved.push((key, axis_clock, now_clock));
            }
        }
    }
    let store = self.motion_history.lock().unwrap_or_else(|p| p.into_inner());
    let mut out = std::collections::HashMap::new();
    for (key, axis_clock, now_clock) in resolved {
        let st = store
            .state_at_clock(key, axis_clock, Some(now_clock))
            .map_err(|e| PyRuntimeError::new_err(e.to_string()))?;
        let name = AXIS_NAMES
            .get(key.axis as usize)
            .ok_or_else(|| {
                PyRuntimeError::new_err(format!("motion_state_at: unnamed axis {}", key.axis))
            })?;
        out.insert((*name).to_string(), (st.position, st.velocity, st.acceleration));
    }
    Ok(out)
}
```

- [ ] **Step 2: Python wrapper**

In `klippy/motion_bridge.py`:

```python
def motion_state_at(self, mcu, clock=None, print_time=None):
    if (clock is None) == (print_time is None):
        raise ValueError(
            "motion_state_at: specify exactly one of clock= or print_time="
        )
    if print_time is not None:
        clock = mcu.print_time_to_clock(print_time)
    return self._bridge.motion_state_at_clock(
        mcu._bridge_handle, int(clock), self._reactor.monotonic()
    )
```

- [ ] **Step 3: Run + commit**

Run: `cd rust && cargo nextest run -p motion-bridge && cargo fmt --all --check`
Expected: PASS / clean.

```bash
git add rust/motion-bridge/src/bridge.rs klippy/motion_bridge.py
git commit -m "feat(bridge): motion_state_at query — commanded (pos, vel, accel) at a past time"
```

---

### Task 6: homing migration — reconstruct from the ring, delete `homing_trajectory`

**Files:**
- Modify: `rust/motion-bridge/src/homing.rs` (reconstruct via store + stale-trip guard; delete `eval_piece_at_clock`, `eval_bernstein_cubic`, `NoTrajectoryPieces`, `UnknownClockFreq`)
- Modify: `rust/motion-bridge/src/homing/tests.rs`
- Modify: `rust/motion-bridge/src/bridge.rs` (HomingRun, home_axis_start, trip handler; delete `homing_trajectory` at lines 471, 699, 2653/2718-gated insert, 3033 clear, 3257 clone)

- [ ] **Step 1: Update homing tests first**

Rewrite `rust/motion-bridge/src/homing/tests.rs` to drive `reconstruct_axis_position` through a `HistoryStore` (`store.record(...)` with `FREQ`) instead of `HashMap<AxisKey, Vec<PieceEntry>>`, preserving every existing scenario (same-MCU trip, cross-MCU trip, trip-outside-trajectory). Add the stale-trip case:

```rust
#[test]
fn trip_before_homing_window_is_rejected() {
    let mut store = HistoryStore::default();
    store.record(key(), &linear(1_000_000, 0.01, 0.0, 5.0), FREQ);
    let window_start = 2_000_000;
    let err = reconstruct_axis_position(
        7, 1_500_000, key(), &router(), &shared(store), window_start,
    )
    .unwrap_err();
    assert!(err.contains("stale"));
}
```

(Adapt helper names to that file's existing fixtures; `shared(store)` wraps in `Arc<Mutex<_>>`.)

- [ ] **Step 2: Run to verify failure**

Run: `cd rust && cargo nextest run -p motion-bridge -E 'test(homing)'`
Expected: compile failure (signature mismatch).

- [ ] **Step 3: Rewrite `reconstruct_axis_position`**

```rust
#[allow(clippy::implicit_hasher)]
pub fn reconstruct_axis_position(
    endstop_mcu: u32,
    trip_clock: u64,
    axis_key: AxisKey,
    router: &Arc<Mutex<PassthroughRouter>>,
    history: &Arc<Mutex<crate::motion_history::HistoryStore>>,
    window_start_clock: u64,
) -> Result<f64, String> {
    let axis_mcu = axis_key.mcu_id;
    let axis_clock = {
        let router_guard = router.lock().unwrap_or_else(|p| p.into_inner());
        crate::motion_history::clock_between_mcus(
            &router_guard,
            crate::types::mcu_handle_from_raw(endstop_mcu),
            crate::types::mcu_handle_from_raw(axis_mcu),
            trip_clock,
        )
        .map_err(|description| {
            ReconstructError::ClockUnsynced {
                description,
                endstop_mcu,
                axis_mcu,
                trip_clock,
            }
            .to_string()
        })?
    };
    if axis_clock <= window_start_clock {
        return Err(format!(
            "endstop trip clock {axis_clock} predates this homing move \
             (window starts at {window_start_clock}) — stale trip or \
             mis-synced clock"
        ));
    }
    let store = history.lock().unwrap_or_else(|p| p.into_inner());
    store
        .state_at_clock(axis_key, axis_clock, None)
        .map(|st| st.position)
        .map_err(|e| e.to_string())
}
```

Delete `eval_piece_at_clock`, the local `eval_bernstein_cubic` (now in motion_history), and the dead `ReconstructError` variants (`NoTrajectoryPieces`, `UnknownClockFreq`, `EndstopTripOutsideTrajectory` if no longer constructed — keep `ClockUnsynced`). The `configs` parameter drops (frequency now baked into the ring); update the caller.

- [ ] **Step 4: bridge.rs surgery**

1. `HomingRun` (bridge.rs:25) gains `window_start_clock: u64`.
2. In `home_axis_start`, replace the `homing_trajectory` clear block (lines ~3031-3037) with a window capture for the moving axis, before pieces are dispatched:

```rust
let window_start_clock = {
    let store = self.motion_history.lock().unwrap_or_else(|p| p.into_inner());
    store.last_endpoint_clock(axis_key)
};
```

and store it in the `HomingRun`.
3. In the trip-handler thread (lines ~3257, 3316): replace `homing_traj` clone with `Arc::clone(&self.motion_history)`; call the new `reconstruct_axis_position(source_mcu, clock, axis_key, &router_arc, &history, run.window_start_clock)`; drop the now-unused `configs` argument from the call but keep `configs` for `trip_position_to_motor_frame`/`kinematics` lookups that follow.
4. Delete the `homing_trajectory` field (471), its init (699), the `homing_traj_for_cb` clone + drip-gated insert in the dispatch callback (2653, 2718-2723 — recording is now solely the unconditional Task-4 block).

- [ ] **Step 5: Run the full crate suite**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: PASS — homing tests now exercise the ring path.

- [ ] **Step 6: Sim homing regression**

Run the existing homing sim validation (kalico-sim skill: `_HOME_TEST` variants) and confirm green before committing.

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/homing.rs rust/motion-bridge/src/homing/tests.rs rust/motion-bridge/src/bridge.rs
git commit -m "refactor(homing): trip reconstruction reads the motion-history ring; delete homing_trajectory"
```

---

### Task 7: Python consumers — extruder rewire, stepper fail-loud

**Files:**
- Modify: `klippy/kinematics/extruder.py:83-85`
- Modify: `klippy/stepper.py` (new method on `MCU_stepper`, near `calc_position_from_coord` at line 131)

- [ ] **Step 1: Rewire `find_past_position`**

```python
def find_past_position(self, print_time):
    bridge = self.printer.lookup_object("motion_bridge")
    state = bridge.motion_state_at(self.stepper.get_mcu(), print_time=print_time)
    if "e" not in state:
        raise self.printer.command_error(
            "find_past_position: extruder axis is not bridge-dispatched"
        )
    return state["e"][0]
```

- [ ] **Step 2: Fail-loud `get_past_mcu_position`**

On `MCU_stepper`, mirroring the raise style of `calc_position_from_coord` (stepper.py:131-135):

```python
def get_past_mcu_position(self, print_time):
    raise self._error(
        "MCU_stepper.get_past_mcu_position is host step history; the bridge"
        " keeps no motor-space step history. Use motion_bridge"
        ".motion_state_at for toolhead-space history."
    )
```

(Use the same error mechanism `calc_position_from_coord` uses — read it first and copy the idiom exactly.)

- [ ] **Step 3: Lint + commit**

Run: `ruff check klippy/kinematics/extruder.py klippy/stepper.py`
Expected: clean.

```bash
git add klippy/kinematics/extruder.py klippy/stepper.py
git commit -m "feat(klippy): find_past_position via motion history; fail-loud motor-space past-position"
```

---

### Task 8: sim validation — `KALICO_SIM_MOTION_STATE` + runner test

**Files:**
- Modify: `klippy/motion_toolhead.py` (register near the `KALICO_SIM_*` cluster at lines 262-284)
- Modify: `tools/kalico-sim/runner.py` (flag + validation block following the `homing_test` pattern at lines 566-600)

- [ ] **Step 1: Debug gcode command**

```python
gcode.register_command(
    "KALICO_SIM_MOTION_STATE",
    self.cmd_KALICO_SIM_MOTION_STATE,
    desc="[sim] Query commanded motion state at a past print_time",
)
```

```python
def cmd_KALICO_SIM_MOTION_STATE(self, gcmd):
    print_time = gcmd.get_float("PRINT_TIME", None)
    t_ago = gcmd.get_float("T_AGO", None)
    if (print_time is None) == (t_ago is None):
        raise gcmd.error("specify exactly one of PRINT_TIME or T_AGO")
    if t_ago is not None:
        print_time = self.get_last_move_time() - t_ago
    bridge = self.printer.lookup_object("motion_bridge")
    state = bridge.motion_state_at(self.mcu, print_time=print_time)
    parts = [
        "%s: pos=%.6f vel=%.6f accel=%.6f" % (name, p, v, a)
        for name, (p, v, a) in sorted(state.items())
    ]
    gcmd.respond_info(
        "motion_state @%.6f: %s" % (print_time, " | ".join(parts))
    )
```

(`self.mcu` is the attribute `get_last_move_time` already uses at motion_toolhead.py:427-434.)

- [ ] **Step 2: Runner block**

Add `--test-motion-state` argument next to the existing test flags in `main()` (runner.py:1897+), ensure `import re` exists at the top of runner.py, and add a block modeled on `homing_test` (runner.py:566-600):

```python
if motion_state_test:
    log.info("Motion-state test: move, then query mid-move state")
    send_gcode(api_socket, "SET_KINEMATIC_POSITION X=150 Y=150 Z=100", timeout=10)
    send_gcode(api_socket, "G4 P500", timeout=15)
    send_gcode(api_socket, "G1 X170 F600", timeout=30)
    send_gcode(api_socket, "M400", timeout=30)
    resp = send_gcode(api_socket, "KALICO_SIM_MOTION_STATE T_AGO=1.0", timeout=15)
    log.info("KALICO_SIM_MOTION_STATE: %s", resp)
    text = str(resp)
    ms_error = None
    m = re.search(r"x: pos=([0-9.eE+-]+) vel=([0-9.eE+-]+)", text)
    if not m:
        ms_error = "no x-axis state in response: %s" % (text,)
    else:
        pos, vel = float(m.group(1)), float(m.group(2))
        if not (150.0 - 1e-3 <= pos <= 170.0 + 1e-3):
            ms_error = "x pos %.4f outside move span 150..170" % (pos,)
        elif not (0.0 <= vel <= 10.0 + 1e-3):
            ms_error = "x vel %.4f outside 0..10 mm/s" % (vel,)
    success = ms_error is None
    error = ms_error
```

The 20 mm move at 10 mm/s takes ~2 s, so `T_AGO=1.0` after `M400` lands mid-move: x strictly inside (150, 170) and vel in (0, 10]. Exact assertion bounds are intentionally loose — accel/decel phases make the midpoint inexact; the test proves the pipeline (clock conversion, ring lookup, evaluation), not the planner.

- [ ] **Step 3: Run the sim test**

Use the kalico-sim skill to run with `--test-motion-state` against the standard sim config.
Expected: success=True, no error.

- [ ] **Step 4: Commit**

```bash
git add klippy/motion_toolhead.py tools/kalico-sim/runner.py
git commit -m "test(sim): KALICO_SIM_MOTION_STATE command + motion-state validation run"
```

---

### Task 9: full verification

- [ ] **Step 1:** `cd rust && cargo nextest run` — full workspace suite green.
- [ ] **Step 2:** `cd rust && cargo test --doc` — doc-tests (nextest skips them) green.
- [ ] **Step 3:** `cd rust && cargo fmt --all --check` and `cargo clippy -p motion-bridge` — clean.
- [ ] **Step 4:** `ruff check klippy/` — clean.
- [ ] **Step 5:** Re-run sim homing variant + motion-state variant (kalico-sim skill) — both green.
- [ ] **Step 6:** Final commit if any cleanups; do not push without the fmt check passing.
