# Faithful Position-at-Time Homing Seam — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. Rust work MUST go to a `rust-engineer` subagent.

**Goal:** Build the Rust `eval`-based position-at-time seam and repoint the five fail-loud Python holes left by commit `bd9e16161`, so homing/probing distance and trigger position come from one pure evaluation of the planner's motor-frame trajectory, with kinematics living only in Rust.

**Architecture:** A new `kinematics.rs` owns forward (toolhead→motor) and inverse (motor→toolhead). The enqueue path retains the motor-frame NURBS it already builds, per `(mcu, slot)`, truncated at trip. New PyO3 methods evaluate that retention at a clock or at "now" (the committed endpoint). `get_mcu_position`/`get_past_mcu_position`/`calc_position`/`set_position`/`_fire_active_callbacks` become thin delegators. Reference design: `docs/superpowers/specs/2026-06-08-faithful-position-at-time-homing-seam-design.md`.

**Tech stack:** Rust (`rust/motion-bridge`, `rust/nurbs`) → `motion_bridge_native` PyO3 cdylib; Python host (`klippy/stepper.py`, `klippy/motion_toolhead.py`, `klippy/motion_bridge.py`, `klippy/mcu.py`). `cargo nextest run` for Rust; `python3 -m pytest` for host.

**Pre-state (already done):** Commit `bd9e16161` deleted the four host kinematics copies, the software-trip stash, and the step-count snapshot path. The five seam methods raise `NotImplementedError`. The tree is intentionally non-functional until this plan completes.

**Key invariants this plan must preserve:**
1. `get_mcu_position()` (no clock) = the **latest committed motor position** = the retained curve's endpoint, or the grounded anchor when no curve. NO print_time→clock conversion on this path (avoids the lossy-epoch trap).
2. After a homing trip, the retained motor curve is **truncated at `trip_clock`**, so its endpoint equals the trip position. Then `halt_pos` (= `get_mcu_position()` clamped to endpoint) equals `trig_pos` (= `get_past_mcu_position(trigger_time)`), and `(halt − start) × step_dist` is the true traveled distance.
3. `eval` is a pure function of `(retained curve, time)`; outside the curve's `[t_abs_start, t_abs_end]` it clamps to the nearest endpoint. The resting state is a degenerate one-point curve.

---

## Task 1: Kinematics module (forward + inverse, pure)

**Files:**
- Create: `rust/motion-bridge/src/kinematics.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (add `pub mod kinematics;` after the existing module declarations, ~line 11)
- Modify: `rust/motion-bridge/src/dispatch.rs:72-78` (make `motor_frame_xy` delegate to the new module)
- Test: `rust/motion-bridge/src/kinematics/tests.rs`

- [ ] **Step 1: Write the failing round-trip + transform tests**

`rust/motion-bridge/src/kinematics/tests.rs`:
```rust
use super::*;

const CARTESIAN: u8 = 1; // KINEMATICS_* tags: 0=CoreXY, 1=Cartesian (dispatch.rs)
const COREXY: u8 = 0;

#[test]
fn corexy_forward_is_sum_and_difference() {
    assert_eq!(forward_corexy(3.0, 1.0), (4.0, 2.0));
}

#[test]
fn corexy_inverse_recovers_toolhead() {
    assert_eq!(inverse_corexy(4.0, 2.0), (3.0, 1.0));
}

#[test]
fn corexy_round_trip_identity() {
    for (x, y) in [(0.0, 0.0), (10.0, -7.5), (-3.25, 100.0), (0.1, 0.2)] {
        let (a, b) = forward_corexy(x, y);
        let (rx, ry) = inverse_corexy(a, b);
        assert!((rx - x).abs() < 1e-12 && (ry - y).abs() < 1e-12);
    }
}

#[test]
fn forward_inverse_round_trip_by_tag() {
    for tag in [CARTESIAN, COREXY] {
        let p = [12.0, -4.0, 3.0];
        let motor = forward(tag, p);
        let back = inverse(tag, motor);
        for i in 0..3 {
            assert!((back[i] - p[i]).abs() < 1e-12, "tag {tag} axis {i}");
        }
    }
}
```

- [ ] **Step 2: Run, verify it fails to compile (module missing)**

Run: `cd rust && cargo test -p motion-bridge kinematics`
Expected: compile error, `kinematics` not found.

- [ ] **Step 3: Implement `kinematics.rs`**

```rust
//! Single host-side source of the toolhead<->motor transform. enqueue.rs builds
//! the per-motor NURBS curves; this module owns the scalar forward/inverse used
//! for reporting, grounding, and active-callback gating. Slot order: 0=X 1=Y 2=Z 3=E.
use crate::dispatch::KINEMATICS_COREXY;

#[inline]
pub fn forward_corexy(x: f64, y: f64) -> (f64, f64) {
    (x + y, x - y)
}

#[inline]
pub fn inverse_corexy(motor_a: f64, motor_b: f64) -> (f64, f64) {
    (0.5 * (motor_a + motor_b), 0.5 * (motor_a - motor_b))
}

/// Cartesian toolhead position -> per-slot motor position (E left to caller).
pub fn forward(tag: u8, xyz: [f64; 3]) -> [f64; 4] {
    if tag == KINEMATICS_COREXY {
        let (a, b) = forward_corexy(xyz[0], xyz[1]);
        [a, b, xyz[2], 0.0]
    } else {
        [xyz[0], xyz[1], xyz[2], 0.0]
    }
}

/// Per-slot motor position -> Cartesian toolhead position.
pub fn inverse(tag: u8, motor: [f64; 4]) -> [f64; 3] {
    if tag == KINEMATICS_COREXY {
        let (x, y) = inverse_corexy(motor[0], motor[1]);
        [x, y, motor[2]]
    } else {
        [motor[0], motor[1], motor[2]]
    }
}

#[cfg(test)]
mod tests;
```

- [ ] **Step 4: Refactor `dispatch.rs::motor_frame_xy` to delegate**

In `rust/motion-bridge/src/dispatch.rs:72-78`, replace the body of `motor_frame_xy` so the CoreXY branch returns `crate::kinematics::forward_corexy(x, y)` (keep the `cfg_is_corexy(cfg)` guard and identity else-branch). The existing dispatch tests (`dispatch.rs:219-225`) must still pass unchanged.

- [ ] **Step 5: Run all motion-bridge tests, verify green**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: PASS, including `dispatch` tests.

- [ ] **Step 6: Commit**
```bash
git add rust/motion-bridge/src/kinematics.rs rust/motion-bridge/src/kinematics/tests.rs rust/motion-bridge/src/lib.rs rust/motion-bridge/src/dispatch.rs
git commit -m "homing: single Rust kinematics module (forward+inverse), round-trip tested"
```

---

## Task 2: oid→slot registration

The bridge has no oid→slot map (`McuAxisConfig.axes` is per-MCU slot list, not keyed by oid; oid↔slot lives only in Python `bind_list`, `motion_bridge.py:766`). `eval_motor_position_at_clock(mcu, oid, …)` needs it.

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (add field + `#[pymethods]`)
- Modify: `klippy/motion_bridge.py` (`_configure_axes_per_mcu`, add to `_STUB_MOTION_METHODS`)
- Test: `rust/motion-bridge/src/bridge/stepper_oid_map_tests.rs`

- [ ] **Step 1: Write the failing test**

`rust/motion-bridge/src/bridge/stepper_oid_map_tests.rs`:
```rust
use super::*;
#[test]
fn register_and_resolve_oid_slot() {
    let b = PyMotionBridge::new_for_test(); // see test_support.rs; if absent, use the existing test ctor pattern
    b.register_stepper_slot(7, 12, 1).unwrap();
    let map = b.stepper_oid_map.lock().unwrap();
    assert_eq!(map.get(&(7u32, 12u8)).copied(), Some(1u8));
}
```
(If `new_for_test` does not exist, mirror the construction used by `retained_homing_curve_tests.rs`.)

- [ ] **Step 2: Run, verify failure**

Run: `cd rust && cargo test -p motion-bridge stepper_oid_map`
Expected: FAIL (no field/method).

- [ ] **Step 3: Add field + method**

In `PyMotionBridge` struct (`bridge.rs`, near the `mcu_axis_configs` field ~line 448):
```rust
stepper_oid_map: std::sync::Mutex<std::collections::HashMap<(u32, u8), u8>>,
```
Initialize it in every `PyMotionBridge` constructor to `Mutex::new(HashMap::new())`.

In the `#[pymethods] impl PyMotionBridge` block:
```rust
#[pyo3(signature = (mcu, oid, slot))]
fn register_stepper_slot(&self, mcu: u32, oid: u8, slot: u8) -> PyResult<()> {
    self.stepper_oid_map
        .lock()
        .unwrap_or_else(|p| p.into_inner())
        .insert((mcu, oid), slot);
    Ok(())
}
```
Declare `mod stepper_oid_map_tests;` near the other `#[cfg(test)] mod ...;` lines (~3487).

- [ ] **Step 4: Call it from Python during axis config**

In `klippy/motion_bridge.py`, `_configure_axes_per_mcu`, after `bind_list.append((i, sname, s.get_oid(), inv))` (line ~766), call `self._bridge.register_stepper_slot(mcu_handle, s.get_oid(), i)` where `i` is the slot index already in scope. Add `"register_stepper_slot"` to `_STUB_MOTION_METHODS`.

- [ ] **Step 5: Run Rust tests, verify green**

Run: `cd rust && cargo nextest run -p motion-bridge stepper_oid_map`
Expected: PASS.

- [ ] **Step 6: Commit**
```bash
git commit -am "homing: register stepper oid->slot map in the bridge"
```

---

## Task 3: Motor-frame retention

Replace the Cartesian `RetainedHomingPiece` with per-`(mcu, slot)` motor-frame retention. `enqueue.rs` builds the motor NURBS (`enqueue.rs:25-35`) and discards them; thread them back to the dispatch closure (`bridge.rs:2358-2445`) and store them. Truncate at trip so the endpoint is the trip position.

**Files:**
- Modify: `rust/motion-bridge/src/enqueue.rs` (return motor curves alongside `EnqueueMsg`s)
- Modify: `rust/motion-bridge/src/bridge.rs` (new struct, field, populate in dispatch closure, clear in `set_position`)
- Test: `rust/motion-bridge/src/bridge/retained_motor_curve_tests.rs`

- [ ] **Step 1: Write the failing retention-lifecycle test**

`rust/motion-bridge/src/bridge/retained_motor_curve_tests.rs`:
```rust
use super::*;
use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};

fn motor_curve(p0: f64, p1: f64, t0: f64, t1: f64) -> nurbs::ScalarNurbs<f64> {
    let d = (p1 - p0) / 3.0;
    let bern = [p0, p0 + d, p0 + 2.0 * d, p1];
    bezier_pieces_to_nurbs(&[BezierPiece::from_bernstein(&bern, t0, t1)])
}

#[test]
fn retain_then_eval_midpoint_and_clamp() {
    let mut ret = RetainedMotorCurve::default();
    ret.push_piece(0 /*mcu*/, 0 /*slot*/, motor_curve(0.0, 10.0, 100.0, 102.0), 100.0, 102.0);
    // midpoint of a uniform cubic from 0->10 over [100,102] is 5.0 at t=101
    assert!((ret.eval(0, 0, 101.0).unwrap() - 5.0).abs() < 1e-9);
    // clamp before start and after end -> endpoints
    assert!((ret.eval(0, 0, 99.0).unwrap() - 0.0).abs() < 1e-9);
    assert!((ret.eval(0, 0, 200.0).unwrap() - 10.0).abs() < 1e-9);
}
```

- [ ] **Step 2: Run, verify failure**

Run: `cd rust && cargo test -p motion-bridge retained_motor_curve`
Expected: FAIL (type missing).

- [ ] **Step 3: Define `RetainedMotorCurve`**

In `bridge.rs`, alongside the existing `RetainedHomingPiece` (which this replaces; remove `RetainedHomingPiece`/`RetainedHomingCurve` and `get_homing_position_at_clock`/`eval_retained_curve` once nothing references them — see Task 4 Step 5):
```rust
#[derive(Default)]
struct RetainedMotorCurve {
    // key (mcu_id, slot) -> ordered pieces; eval clamps outside [t_abs_start, t_abs_end]
    pieces: std::collections::HashMap<(u32, u8), Vec<RetainedMotorPiece>>,
}
struct RetainedMotorPiece {
    curve: nurbs::ScalarNurbs<f64>,
    t_abs_start: f64,
    t_abs_end: f64,
    t0: f64,
}
impl RetainedMotorCurve {
    fn push_piece(&mut self, mcu: u32, slot: u8, curve: nurbs::ScalarNurbs<f64>, ts: f64, te: f64) {
        self.pieces.entry((mcu, slot)).or_default().push(RetainedMotorPiece {
            curve, t_abs_start: ts, t_abs_end: te, t0: ts,
        });
    }
    fn eval(&self, mcu: u32, slot: u8, t_abs: f64) -> Option<f64> {
        let v = self.pieces.get(&(mcu, slot))?;
        // pick the piece whose window contains t_abs, else clamp to first/last
        let piece = v.iter().find(|p| t_abs >= p.t_abs_start - 1e-9 && t_abs <= p.t_abs_end + 1e-9)
            .or_else(|| if t_abs < v.first()?.t_abs_start { v.first() } else { v.last() })?;
        let u = (t_abs - piece.t0).clamp(piece.t_abs_start - piece.t0, piece.t_abs_end - piece.t0);
        Some(nurbs::eval::eval(&piece.curve, u))
    }
    fn endpoint(&self, mcu: u32, slot: u8) -> Option<f64> {
        let p = self.pieces.get(&(mcu, slot))?.last()?;
        Some(nurbs::eval::eval(&p.curve, p.t_abs_end - p.t0))
    }
}
```
Change the `PyMotionBridge` field (was `retained_homing_curve: Arc<Mutex<Option<RetainedHomingCurve>>>`, ~line 453) to `retained_motor_curve: Arc<Mutex<RetainedMotorCurve>>`. Declare `mod retained_motor_curve_tests;`.

- [ ] **Step 4: Thread motor curves out of enqueue into retention**

In `enqueue.rs`, have `enqueue_segment` additionally return the per-`(mcu_id, slot)` motor NURBS it constructs (the `motor_a`/`motor_b` at lines 25-35, plus the identity curves for non-CoreXY axes and Z/E). Recommended: change the return to `(Vec<EnqueueMsg>, Vec<(AxisKey, ScalarNurbs<f64>)>)` rather than recomputing. In the dispatch closure (`bridge.rs:2358-2445`), where it currently pushes a `RetainedHomingPiece` under `HomingSegmentState::Active` (lines 2418-2433), instead push each `(AxisKey, curve)` via `retained_motor_curve.push_piece(key.mcu_id, key.axis, curve, t0 + seg.t_start, t0 + seg.t_end)`. Keep the `fresh`-boundary clear (clear the map on a new homing stream).

- [ ] **Step 5: Truncate at trip**

The retained curve's last piece must end at `trip_clock`. The trip path (`take_trip_event` → `flush_homing_pieces`) already truncates the dispatched stream; ensure the **retained** curve is correspondingly trimmed: in the trip handler, after computing `t_abs_trip = router.clock_to_host_secs(mcu, trip_clock)`, drop retained pieces starting after `t_abs_trip` and set the containing piece's `t_abs_end = t_abs_trip` (its `endpoint()` then returns the trip position). Add a test: build a 0→10 move over `[100,102]`, "trip" at `t_abs=101`, assert `endpoint()==5.0`.

- [ ] **Step 6: Clear on `set_position`**

In `set_position` (`bridge.rs:2774`, where it currently sets `retained_homing_curve=None` ~2842-2845): clear the motor map and install a **degenerate constant piece** per present slot at the grounded motor position (see Task 7 for the value), so `eval`/`endpoint` return the set position. For this task, just clear; Task 7 installs the constant.

- [ ] **Step 7: Run, verify green**

Run: `cd rust && cargo nextest run -p motion-bridge`
Expected: PASS.

- [ ] **Step 8: Commit**
```bash
git commit -am "homing: retain motor-frame curves per (mcu,slot), truncated at trip"
```

---

## Task 4: Eval primitives (PyO3)

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (`#[pymethods]`, after old `get_homing_position_at_clock` site ~line 2923)
- Modify: `klippy/motion_bridge.py` (`_STUB_MOTION_METHODS`)
- Test: `rust/motion-bridge/src/bridge/eval_motor_position_tests.rs`

- [ ] **Step 1: Write failing eval tests** (build a known CoreXY move, resolve oid→slot, eval at a clock and at "now")

`eval_motor_position_tests.rs`: register oid→slot, push a motor curve via the retention API, then assert `eval_motor_position_at_clock` matches the midpoint and `eval_motor_position_now` matches the endpoint. Convert a known `t_abs` to a `trip_clock` using the inverse of `router.clock_to_host_secs` available in the test router (mirror `retained_homing_curve_tests.rs` clock setup).

- [ ] **Step 2: Run, verify failure.** `cd rust && cargo test -p motion-bridge eval_motor_position` → FAIL.

- [ ] **Step 3: Implement the two methods**
```rust
#[pyo3(signature = (mcu, oid, trip_clock))]
fn eval_motor_position_at_clock(&self, mcu: u32, oid: u8, trip_clock: u64) -> PyResult<f64> {
    let slot = self.resolve_slot(mcu, oid)?;          // helper: read stepper_oid_map
    let t_abs = self.router.lock()...clock_to_host_secs(mcu, trip_clock); // reuse get_homing_position_at_clock's path
    self.retained_motor_curve.lock().unwrap().eval(mcu, slot, t_abs)
        .ok_or_else(|| PyRuntimeError::new_err("eval_motor_position_at_clock: no retained curve for slot"))
}

#[pyo3(signature = (mcu, oid))]
fn eval_motor_position_now(&self, mcu: u32, oid: u8) -> PyResult<f64> {
    let slot = self.resolve_slot(mcu, oid)?;
    self.retained_motor_curve.lock().unwrap().endpoint(mcu, slot)
        .ok_or_else(|| PyRuntimeError::new_err("eval_motor_position_now: no retained curve for slot"))
}
```
`resolve_slot` is a small private (non-`#[pymethods]`) helper reading `stepper_oid_map`. Reuse the exact `clock_to_host_secs` invocation from the (about-to-be-removed) `get_homing_position_at_clock` at `bridge.rs:2907`.

- [ ] **Step 4: Add both names to `_STUB_MOTION_METHODS`.**

- [ ] **Step 5: Delete the dead Cartesian path.** Remove `get_homing_position_at_clock` (Rust), `eval_retained_curve`, `RetainedHomingPiece`, `RetainedHomingCurve` now that nothing references them. `grep -rn get_homing_position_at_clock rust/` must return zero.

- [ ] **Step 6: Run, verify green.** `cd rust && cargo nextest run -p motion-bridge`.

- [ ] **Step 7: Commit** `git commit -am "homing: eval_motor_position_{at_clock,now} on the motor-frame retention"`

---

## Task 5: Inverse + forward PyO3 calls

**Files:** Modify `rust/motion-bridge/src/bridge.rs` (`#[pymethods]`); `klippy/motion_bridge.py` (`_STUB_MOTION_METHODS`). Test: `rust/motion-bridge/src/bridge/kinematics_calls_tests.rs`.

- [ ] **Step 1: Write failing tests** asserting `motor_positions_to_toolhead(mcu, 4.0, 2.0)` → `[3.0, 1.0]` for a CoreXY-configured test bridge, and `toolhead_delta_to_motor_slots(mcu, 1.0, 1.0, 0.0)` → `[(0, 2.0), (1, 0.0)]`-style mapping (CoreXY: A moves 2, B moves 0).

- [ ] **Step 2: Run, verify failure.**

- [ ] **Step 3: Implement** (look up kinematics tag from `mcu_axis_configs` for `mcu`, call `crate::kinematics::inverse`/`forward`):
```rust
#[pyo3(signature = (mcu, motor_a_mm, motor_b_mm))]
fn motor_positions_to_toolhead(&self, mcu: u32, motor_a_mm: f64, motor_b_mm: f64) -> PyResult<Vec<f64>> {
    let tag = self.kin_tag_for(mcu)?;
    let xyz = crate::kinematics::inverse(tag, [motor_a_mm, motor_b_mm, 0.0, 0.0]);
    Ok(vec![xyz[0], xyz[1]])
}

#[pyo3(signature = (mcu, dx, dy, dz))]
fn toolhead_delta_to_motor_slots(&self, mcu: u32, dx: f64, dy: f64, dz: f64) -> PyResult<Vec<(u8, f64)>> {
    let tag = self.kin_tag_for(mcu)?;
    let m = crate::kinematics::forward(tag, [dx, dy, dz]);
    Ok((0u8..4).filter(|&s| m[s as usize].abs() > 1e-9).map(|s| (s, m[s as usize])).collect())
}
```
`kin_tag_for(mcu)` is a private helper reading `mcu_axis_configs`. Add a `forward_motor_positions(mcu, x, y, z) -> Vec<(u8, f64)>` (all present slots, not just moving ones) for `set_position` grounding (Task 7).

- [ ] **Step 4: Add names to `_STUB_MOTION_METHODS`.**

- [ ] **Step 5: Run, verify green. Commit** `git commit -am "homing: inverse/forward kinematics PyO3 calls"`

---

## Task 6: Repoint the five Python holes

**Files:** Modify `klippy/stepper.py`, `klippy/motion_toolhead.py`, `klippy/mcu.py`, `klippy/motion_bridge.py`. Test: `test/test_bridge_position_seam.py` (new).

- [ ] **Step 1: Write failing host tests** using the fake-native injection pattern from `test/test_motion_bridge_trip_routing.py` (inject a fake `motion_bridge_native` with a `MotionBridge` whose `eval_motor_position_now`/`_at_clock`/`motor_positions_to_toolhead`/`toolhead_delta_to_motor_slots` return scripted values). Assert:
  - `get_mcu_position()` returns `round((eval_now + offset) / step_dist)`.
  - `get_past_mcu_position(pt)` calls `eval_motor_position_at_clock` with `mcu.print_time_to_clock(pt)` and returns `round((eval + offset)/step_dist)`.
  - `BridgeKinematics.calc_position({sx: a, sy: b})` returns the inverse-mapped toolhead `[x, y, z]`.

- [ ] **Step 2: Run, verify failure** (methods still raise).

- [ ] **Step 3: Repoint `stepper.py`**
```python
def get_mcu_position(self, cmd_pos=None):
    if cmd_pos is not None:
        mcu_pos_dist = cmd_pos + self._mcu_position_offset
    else:
        bridge = self._mcu._motion_bridge
        motor_mm = bridge.eval_motor_position_now(self._mcu._bridge_handle, self.get_oid())
        mcu_pos_dist = motor_mm + self._mcu_position_offset
    mcu_pos = mcu_pos_dist / self._step_dist
    return int(mcu_pos + 0.5) if mcu_pos >= 0.0 else int(mcu_pos - 0.5)

def get_past_mcu_position(self, print_time):
    bridge = self._mcu._motion_bridge
    clock = self._mcu.print_time_to_clock(print_time)
    motor_mm = bridge.eval_motor_position_at_clock(self._mcu._bridge_handle, self.get_oid(), clock)
    mcu_pos = (motor_mm + self._mcu_position_offset) / self._step_dist
    return int(mcu_pos + 0.5) if mcu_pos >= 0.0 else int(mcu_pos - 0.5)
```
(`print_time_to_clock` exists on `MCU`; confirm and use it.)

- [ ] **Step 4: Repoint `BridgeKinematics.calc_position`** — collect per-rail motor mm from `stepper_positions` (the dict is keyed by stepper name → motor step position; multiply by that stepper's `get_step_dist()` to get mm), then call `bridge.motor_positions_to_toolhead(mcu, motor_a_mm, motor_b_mm)` for X/Y and pass Z straight through. Return `[x, y, z]`. (For Cartesian the inverse is identity; the Rust call handles both via tag.)

- [ ] **Step 5: Repoint `set_position`** — replace the fail-loud raise: after `bridge.set_position(...)`, call `bridge.forward_motor_positions(mcu, newpos[0], newpos[1], newpos[2])` → `[(slot, motor_mm), ...]`, and for each rail's steppers set `s._set_mcu_position(int(motor_mm / s.get_step_dist() + 0.5))`. Keep the homing_axes→limits update. (Grounding correctness is pinned in Task 7.)

- [ ] **Step 6: Repoint `_fire_active_callbacks`** — replace the raise: `moved = bridge.toolhead_delta_to_motor_slots(mcu, dx, dy, dz)`; build a `set` of moving slot indices; fire a stepper's callbacks when `_stepper_motor_slot(s)` is in that set (the existing `_stepper_motor_slot` helper maps a stepper to its slot). Keep the ServoRail branch unchanged.

- [ ] **Step 7: Run host tests, verify green.**

Run: `python3 -m pytest test/test_bridge_position_seam.py test/test_motion_bridge_trip_routing.py -q`
Expected: PASS.

- [ ] **Step 8: Commit** `git commit -am "homing: repoint the 5 host seam methods at the Rust eval/kinematics"`

---

## Task 7: set_position grounding round-trip

Pin the invariant: after `set_position(p)`, `get_mcu_position()` returns exactly the steps for `p`.

**Files:** Modify `rust/motion-bridge/src/bridge.rs` (`set_position` installs a constant motor piece); Test: `rust/motion-bridge/src/bridge/set_position_grounding_tests.rs` + a host assertion in `test/test_bridge_position_seam.py`.

- [ ] **Step 1: Write failing test** — call `set_position`-equivalent on a test bridge, then `eval_motor_position_now` must equal the grounded motor position for each slot.

- [ ] **Step 2: Run, verify failure.**

- [ ] **Step 3: Install a degenerate constant piece.** In `set_position` (`bridge.rs:2774`), after clearing the retained map (Task 3 Step 6), for each present `(mcu, slot)` compute the grounded motor position `m = kinematics::forward(tag, [x,y,z])[slot]` and `push_piece(mcu, slot, constant_curve(m), t_now, t_now)` where `constant_curve(m)` is a degree-0 (or repeated-control-point cubic) NURBS evaluating to `m` everywhere. Then `endpoint()`/`eval()` return `m`.

- [ ] **Step 4: Verify the Python offset reconciliation.** With the constant curve at `m` (mm) and `_mcu_position_offset` unchanged, `get_mcu_position()` returns `round((m + offset)/step_dist)`. Confirm in the host test that this equals the step position for `p`; if an offset adjustment is required, set `_mcu_position_offset` in `set_position` so the identity holds, and document the exact arithmetic in a comment-free, test-pinned way.

- [ ] **Step 5: Run Rust + host tests, verify green. Commit** `git commit -am "homing: set_position installs grounded constant curve; position round-trips"`

---

## Task 8: Homing distance integration tests

**Files:** Test: `test/test_homing_distance_bridge.py` (new), using the fake-native pattern.

- [ ] **Step 1: Write tests** that drive a `HomingMove`-like flow with scripted `eval` returns and assert: (a) `distance_elapsed` equals `(trip − start)` mapped through `calc_position`; (b) a short trip (< `min_home_dist`) makes `moved_less_than_dist` true and arms the second approach; (c) `halt_pos == trig_pos` when the retained curve is truncated at trip.

- [ ] **Step 2: Run, iterate to green.** `python3 -m pytest test/test_homing_distance_bridge.py -q`

- [ ] **Step 3: Commit** `git commit -am "test(homing): distance + min_home_dist via the eval seam"`

---

## Task 9: Remove unused trip step_count reporting (cleanup)

Per spec §10 — the per-stepper `step_count` in trip events is now unconsumed.

**Files:** `rust/runtime/src/endstop.rs` (`publish_snapshot`, `TripSnapshot.step_counts`, `StepperSnapshot.step_count`), `rust/motion-bridge/src/bridge.rs` (`trip_event_to_pydict:3077`, `take_runtime_event:1780`), `kalico-c-api/src/runtime_ffi.rs:652`.

- [ ] **Step 1:** Remove the `step_count` field from the serialized trip dict (both sites) and confirm no Python reads it (`grep -rn 'step_count' klippy/` → only unrelated hits).
- [ ] **Step 2:** Remove the snapshot plumbing in `endstop.rs` and the empty-slice FFI arg. Keep `trip_clock`/`trip_source_idx`.
- [ ] **Step 3:** `cd rust && cargo nextest run` (workspace). PASS.
- [ ] **Step 4: Commit** `git commit -am "homing: drop unused per-stepper step_count from trip events"`

---

## Task 10: Build, full verification, flash

- [ ] **Step 1: Workspace tests + lint.** `cd rust && cargo nextest run && cargo clippy --all-targets -- -D warnings`. Host: `python3 -m pytest test/ -q`.
- [ ] **Step 2: Build the cdylib** via the flashing-trident-mcus skill's host-rebuild path (`make -f Makefile.kalico motion-bridge`) on the Pi; confirm `motion_bridge_native.so` loads and klippy reaches "ready".
- [ ] **Step 3: Flash both MCUs** (H7 from `.config.h7.bak`, F446 from `.config.f446.test`) per the flashing-trident-mcus skill — only after host tests are green.
- [ ] **Step 4:** Hand back to the user for the live G28 / probe / Beacon-home test (no G-code issued by the agent).

---

## Risks (carry into execution)

- **Clock-domain "now" (highest).** `get_mcu_position()` must NOT round-trip through a wall-clock; it uses `eval_motor_position_now` (committed endpoint). Only `get_past_mcu_position(print_time)` converts, via `print_time_to_clock`. Tests must cover a "now" read after a truncated homing move equalling the trip position.
- **Retention truncation at trip.** If the retained curve isn't trimmed to `trip_clock`, `halt_pos != trig_pos` and the original distance bug returns. Task 3 Step 5 + Task 8(c) guard this.
- **enqueue return-shape change** ripples to `pump.rs`/dispatch callers — keep the `EnqueueMsg` contract, only add the parallel motor-curve return.
- **set_position offset arithmetic** — pin by test (Task 7) before trusting it.
