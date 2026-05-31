# Config-driven MCU topology in `init_planner` — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the hardcoded two-MCU (octopus/f446) planner topology with a topology derived from the printer's existing config — global `kinematics` plus the stepper→MCU assignment — and passed through `init_planner` as a per-MCU descriptor list.

**Architecture:** The host derives a `[(handle, axes, kinematics_tag), …]` list (Python, pure function `_derive_mcu_topology`) from the axis→MCU map and the global kinematics name, then passes it across the PyO3 boundary. Rust's `init_planner` builds `mcu_configs` from that list via a pure helper `build_mcu_configs`. All downstream code already iterates `mcu_configs`, so nothing else changes. The E axis is included now (placed on the extruder's MCU); it is inert until E-curve shaping lands because the dispatch loop range-skips axes beyond `shaped.axes.len()`.

**Tech Stack:** Rust (PyO3 cdylib `motion_bridge_native`), Python (klippy), cargo + pytest.

**Reference spec:** `docs/superpowers/specs/2026-05-31-config-driven-mcu-topology-design.md`

---

## File Structure

- `rust/motion-bridge/src/dispatch.rs` — add `AXIS_E` const + pure helper `build_mcu_configs`.
- `rust/motion-bridge/src/dispatch/tests.rs` — unit tests for `build_mcu_configs`.
- `rust/motion-bridge/src/bridge.rs` — rewire `init_planner` signature + body to consume the descriptor list.
- `klippy/motion_toolhead.py` — module-level `_derive_mcu_topology`; rewire `_init_planner`.
- `klippy/motion_bridge.py` — forward the descriptor list through the wrapper.
- `tools/test_renode_phase2_gate.py` — update the direct `init_planner` call site.
- `tools/test_mcu_topology_derive.py` — new pure-Python unit test for `_derive_mcu_topology`.

**Not touched:** `_configure_axes_per_mcu`, the dispatch closure, `build_push_params`, the corexy transform, the wire ABI, `bridge_to_runtime_step_chain.rs` / `sim_motion_jogs.rs` (they construct `McuAxisConfig` directly — the struct is unchanged), `test_motion_toolhead_static.py` (the derivation is a module-level function, not a new method, so the override-surface baseline is unaffected).

---

## Task 1: Rust — `AXIS_E` constant + `build_mcu_configs` helper

**Files:**
- Modify: `rust/motion-bridge/src/dispatch.rs` (add `AXIS_E` near `:36`; add `build_mcu_configs` near the `McuAxisConfig` def `:44`)
- Test: `rust/motion-bridge/src/dispatch/tests.rs`

- [ ] **Step 1: Write the failing tests**

Append to `rust/motion-bridge/src/dispatch/tests.rs`. (The file already has `use super::*;` at the top — confirm it does; if not, these reference `super::build_mcu_configs`, `super::AXIS_*`, `super::McuCaps`.)

```rust
#[test]
fn axis_e_is_three() {
    assert_eq!(AXIS_E, 3);
}

#[test]
fn build_mcu_configs_two_mcu_corexy_with_e() {
    use std::collections::HashMap;
    let mut caps = HashMap::new();
    caps.insert(7u32, McuCaps { curve_pool_n: 12, max_pieces_per_curve: 4 });
    caps.insert(9u32, McuCaps { curve_pool_n: 8, max_pieces_per_curve: 2 });
    // octopus(7) carries X,Y,E corexy; f446(9) carries Z cartesian.
    let mcus = vec![
        (7u32, vec![AXIS_X as u8, AXIS_Y as u8, AXIS_E as u8], 0u8),
        (9u32, vec![AXIS_Z as u8], 1u8),
    ];
    let cfgs = build_mcu_configs(&mcus, &caps);
    assert_eq!(cfgs.len(), 2);
    assert_eq!(cfgs[0].mcu_id, 7);
    assert_eq!(cfgs[0].axes, vec![AXIS_X, AXIS_Y, AXIS_E]);
    assert_eq!(cfgs[0].kinematics, 0);
    assert_eq!(cfgs[0].caps, McuCaps { curve_pool_n: 12, max_pieces_per_curve: 4 });
    assert_eq!(cfgs[1].mcu_id, 9);
    assert_eq!(cfgs[1].axes, vec![AXIS_Z]);
    assert_eq!(cfgs[1].kinematics, 1);
}

#[test]
fn build_mcu_configs_missing_caps_falls_back_to_default() {
    use std::collections::HashMap;
    let caps: HashMap<u32, McuCaps> = HashMap::new();
    let mcus = vec![(7u32, vec![AXIS_X as u8, AXIS_Y as u8], 0u8)];
    let cfgs = build_mcu_configs(&mcus, &caps);
    assert_eq!(cfgs.len(), 1);
    assert_eq!(cfgs[0].caps, McuCaps::default());
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd rust && cargo test -p motion-bridge-native --lib build_mcu_configs axis_e_is_three 2>&1 | tail -20`
(Adjust package name if `cargo test -p` errors — use the name from `rust/motion-bridge/Cargo.toml`'s `[package] name`.)
Expected: FAIL — `cannot find function 'build_mcu_configs'` and `cannot find value 'AXIS_E'`.

- [ ] **Step 3: Add the `AXIS_E` constant**

In `rust/motion-bridge/src/dispatch.rs`, immediately after the `AXIS_Z` line (`:36`):

```rust
pub const AXIS_X: usize = 0;
pub const AXIS_Y: usize = 1;
pub const AXIS_Z: usize = 2;
/// Extruder axis. Reserved in the axis vocabulary so the host can place E on
/// the MCU that carries the extruder stepper. Inert until E-curve shaping
/// lands: `build_push_params` range-skips it while `shaped.axes` has only
/// X/Y/Z entries.
pub const AXIS_E: usize = 3;
```

- [ ] **Step 4: Add the `build_mcu_configs` helper**

In `rust/motion-bridge/src/dispatch.rs`, after the `McuAxisConfig` struct (after `:53`). Ensure `use std::collections::HashMap;` is present at the top of the file (add it if missing).

```rust
/// Build the per-MCU planner topology from a host-supplied descriptor list.
///
/// Each `mcus` entry is `(bridge_handle, axes, kinematics_tag)` where `axes`
/// holds `AXIS_*` indices as `u8` and `kinematics_tag` is a
/// [`KinematicTag`] discriminant. `caps_by_handle` supplies the per-MCU
/// runtime capabilities; a handle absent from the map gets `McuCaps::default()`
/// (large-profile fallback for firmware predating `QueryRuntimeCaps`).
///
/// Order is preserved from `mcus`. No hardcoded MCU identity, axis set, or
/// kinematics — every field comes from the caller.
pub fn build_mcu_configs(
    mcus: &[(u32, Vec<u8>, u8)],
    caps_by_handle: &HashMap<u32, McuCaps>,
) -> Vec<McuAxisConfig> {
    mcus.iter()
        .map(|(handle, axes, tag)| McuAxisConfig {
            mcu_id: *handle,
            axes: axes.iter().map(|&a| a as usize).collect(),
            kinematics: *tag,
            caps: caps_by_handle.get(handle).copied().unwrap_or_default(),
        })
        .collect()
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd rust && cargo test -p motion-bridge-native --lib build_mcu_configs axis_e_is_three 2>&1 | tail -20`
Expected: PASS (3 tests).

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/dispatch.rs rust/motion-bridge/src/dispatch/tests.rs
git commit -m "feat(bridge): add AXIS_E and build_mcu_configs topology helper"
```

---

## Task 2: Rust — rewire `init_planner` to the descriptor list

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs:1843-1938` (signature + caps fetch + `mcu_configs` construction + doc comment)

- [ ] **Step 1: Rewrite the `#[pyo3(signature = …)]` and the doc comment**

Replace the doc comment at `bridge.rs:1837-1842` and the signature at `:1843-1857` with:

```rust
    /// Initialize the planner thread with config from `printer.cfg`.
    ///
    /// `mcus` is the host-derived topology: one `(bridge_handle, axes,
    /// kinematics_tag)` entry per motion MCU, where `axes` holds `AXIS_*`
    /// indices and `kinematics_tag` is a `KinematicTag` discriminant. The
    /// host builds this from the global `kinematics` setting plus the
    /// stepper→MCU assignment — no MCU identity is hardcoded here. Supports
    /// 1..N MCUs and any axis partition.
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
        mcus,
        window_capacity = 32,
        beta_max_iters = 10,
    ))]
```

- [ ] **Step 2: Rewrite the fn parameter list**

Replace `bridge.rs:1870-1871` (the two handle params) so the params become (keeping the rest unchanged):

```rust
        shaper_freq_y: f64,
        mcus: Vec<(u32, Vec<u8>, u8)>,
        window_capacity: usize,
        beta_max_iters: u8,
```

- [ ] **Step 3: Replace the caps fetch + `mcu_configs` construction**

Replace `bridge.rs:1903-1934` (the `// Two-MCU first-print MVP topology.` comment block through the end of the `let mcu_configs = vec![ … ];`) with:

```rust
        // Host-supplied N-MCU topology. Pull `runtime_caps` from each
        // `McuConnection` (set during bootstrap by `query_runtime_caps`);
        // fall back to large-profile defaults if the firmware predates
        // `QueryRuntimeCaps`.
        let caps_by_handle: std::collections::HashMap<u32, McuCaps> = {
            let mcus_lock = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            mcus.iter()
                .map(|(handle, _, _)| {
                    let caps = mcus_lock
                        .get(handle)
                        .and_then(|c| c.runtime_caps)
                        .map(McuCaps::from)
                        .unwrap_or_default();
                    (*handle, caps)
                })
                .collect()
        };
        let mcu_configs = build_mcu_configs(&mcus, &caps_by_handle);
```

- [ ] **Step 4: Verify `build_mcu_configs` is imported in `bridge.rs`**

Run: `grep -n "use crate::dispatch::\|use super::dispatch::\|build_mcu_configs\|McuAxisConfig" rust/motion-bridge/src/bridge.rs | head`
If `McuAxisConfig` is imported but `build_mcu_configs` is not, add `build_mcu_configs` to that same `use` line. (`McuAxisConfig` must already be in scope since `bridge.rs` referenced it at the old `:1922`.)

- [ ] **Step 5: Build to verify it compiles**

Run: `cd rust && cargo build -p motion-bridge-native 2>&1 | tail -20`
Expected: compiles clean. (If `mcu_configs.clone()` at the old `:1938` and later uses still reference `mcu_configs`, they remain valid — `build_mcu_configs` returns the same `Vec<McuAxisConfig>` type.)

- [ ] **Step 6: Run the full bridge test suite to confirm no regressions**

Run: `cd rust && cargo test -p motion-bridge-native 2>&1 | tail -25`
Expected: PASS. The `bridge_to_runtime_step_chain` and `sim_motion_jogs` tests construct `McuAxisConfig` directly and are unaffected.

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "refactor(bridge): init_planner consumes host-supplied MCU topology list"
```

---

## Task 3: Python — `_derive_mcu_topology` pure helper + unit test

**Files:**
- Modify: `klippy/motion_toolhead.py` (add module-level constants + `_derive_mcu_topology` near the other module-level helpers, after `_stepper_motor_slot` at `:55`)
- Test: `tools/test_mcu_topology_derive.py` (new)

- [ ] **Step 1: Write the failing test**

Create `tools/test_mcu_topology_derive.py`:

```python
#!/usr/bin/env python3
"""Unit tests for motion_toolhead._derive_mcu_topology (pure, no klippy boot)."""
from __future__ import annotations

from klippy.motion_toolhead import _derive_mcu_topology


def test_corexy_two_mcu_extruder_on_octopus():
    # X,Y,E on handle 7 (octopus); Z on handle 9 (f446).
    axis_to_handle = {0: 7, 1: 7, 3: 7, 2: 9}
    topo = _derive_mcu_topology(axis_to_handle, "corexy")
    assert topo == [(7, [0, 1, 3], 0), (9, [2], 1)]


def test_cartesian_single_mcu_all_axes():
    axis_to_handle = {0: 5, 1: 5, 2: 5, 3: 5}
    topo = _derive_mcu_topology(axis_to_handle, "cartesian")
    assert topo == [(5, [0, 1, 2, 3], 1)]


def test_corexy_single_mcu_gets_corexy_tag():
    axis_to_handle = {0: 5, 1: 5, 2: 5, 3: 5}
    topo = _derive_mcu_topology(axis_to_handle, "corexy")
    assert topo == [(5, [0, 1, 2, 3], 0)]


def test_corexy_z_only_mcu_is_cartesian():
    # An MCU lacking the X/Y pair is cartesian even on a corexy printer.
    axis_to_handle = {2: 9}
    topo = _derive_mcu_topology(axis_to_handle, "corexy")
    assert topo == [(9, [2], 1)]


if __name__ == "__main__":
    test_corexy_two_mcu_extruder_on_octopus()
    test_cartesian_single_mcu_all_axes()
    test_corexy_single_mcu_gets_corexy_tag()
    test_corexy_z_only_mcu_is_cartesian()
    print("all passed")
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest tools/test_mcu_topology_derive.py -v 2>&1 | tail -20`
Expected: FAIL — `ImportError: cannot import name '_derive_mcu_topology'`.

- [ ] **Step 3: Add the constants and helper**

In `klippy/motion_toolhead.py`, after `_stepper_motor_slot` (ends at `:55`), add:

```python
# Axis indices — mirror rust/motion-bridge/src/dispatch.rs AXIS_* and the
# _MOTOR_SLOT_PREFIXES slot order (X=0, Y=1, Z=2, E=3).
_AXIS_X = 0
_AXIS_Y = 1

# Kinematics tags — mirror rust KinematicTag discriminants
# (CoreXyAndE = 0, CartesianXyzAndE = 1).
_KIN_COREXY = 0
_KIN_CARTESIAN = 1


def _derive_mcu_topology(axis_to_handle, kinematics_name):
    """Derive the per-MCU planner topology from the axis->MCU assignment.

    `axis_to_handle` maps each present axis index (0=X, 1=Y, 2=Z, 3=E) to its
    MCU's bridge handle. `kinematics_name` is the printer's global kinematics
    (e.g. "corexy", "cartesian").

    Returns a list of `(handle, sorted_axes, kinematics_tag)` tuples — one per
    distinct handle, ordered by handle. An MCU's tag is COREXY iff the printer
    is corexy AND that MCU carries both X and Y; otherwise CARTESIAN. This
    reproduces the historical hardcoded topology (XY-MCU -> corexy, Z-MCU ->
    cartesian) without hardcoding MCU identity.
    """
    by_handle = {}
    for axis_idx, handle in axis_to_handle.items():
        by_handle.setdefault(handle, []).append(axis_idx)
    is_corexy = (kinematics_name or "").lower() == "corexy"
    topo = []
    for handle in sorted(by_handle):
        axes = sorted(by_handle[handle])
        if is_corexy and _AXIS_X in axes and _AXIS_Y in axes:
            tag = _KIN_COREXY
        else:
            tag = _KIN_CARTESIAN
        topo.append((handle, axes, tag))
    return topo
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest tools/test_mcu_topology_derive.py -v 2>&1 | tail -20`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_toolhead.py tools/test_mcu_topology_derive.py
git commit -m "feat(motion): add _derive_mcu_topology host-side topology derivation"
```

---

## Task 4: Python — rewire `_init_planner` + wrapper + gate test call site

**Files:**
- Modify: `klippy/motion_toolhead.py:831-894` (`_init_planner` body + the `init_planner` call)
- Modify: `klippy/motion_bridge.py:270-300` (wrapper signature + forward)
- Modify: `tools/test_renode_phase2_gate.py:198-207` (direct call site)

- [ ] **Step 1: Replace the octopus/f446 derivation in `_init_planner`**

In `klippy/motion_toolhead.py`, replace the block at `:831-858` (from `def _init_planner(self):` through the `if f446 is None: f446 = octopus` lines) with:

```python
    def _init_planner(self):
        # Build the planner topology from existing config: each kinematic
        # stepper's MCU (via get_mcu()._bridge_handle) gives axis->MCU, and
        # the global kinematics name gives each MCU's kinematics tag. No MCU
        # identity is hardcoded.
        bridge_mcus = []
        for name, mcu in self.printer.lookup_objects(module="mcu"):
            handle = getattr(mcu, "_bridge_handle", None)
            if handle is None:
                continue
            bridge_mcus.append((name, mcu, handle))
        if not bridge_mcus:
            logging.warning(
                "MotionToolhead: no MCU bridge handles available; "
                "skipping init_planner"
            )
            return

        # axis index (0=X,1=Y,2=Z,3=E) -> bridge handle of the MCU that
        # drives that axis's primary stepper. AWD partners share their
        # primary's axis/MCU, so only primaries contribute.
        axis_to_handle = {}
        fm = self.printer.lookup_object("force_move", None)
        if fm is not None:
            for sname, s in fm.steppers.items():
                info = _name_motor_slot(sname)
                if info is None:
                    continue
                slot_idx, is_primary = info
                if not is_primary:
                    continue
                s_handle = getattr(s.get_mcu(), "_bridge_handle", None)
                if s_handle is None:
                    continue
                axis_to_handle[slot_idx] = s_handle

        topology = _derive_mcu_topology(axis_to_handle, self.kinematics_name)
        if not topology:
            logging.warning(
                "MotionToolhead: no axis->MCU assignment resolved; "
                "skipping init_planner"
            )
            return
```

- [ ] **Step 2: Replace the `init_planner` call**

In the same method, replace the call at `:882-894` (the `self.bridge.init_planner( … octopus, f446, )` call and its surrounding `try:`) so the positional handles become `topology`:

```python
        try:
            self.bridge.init_planner(
                self.max_velocity,
                self.max_accel,
                self.max_z_velocity,
                self.max_z_accel,
                self.square_corner_velocity,
                shaper_type_x,
                shaper_freq_x,
                shaper_type_y,
                shaper_freq_y,
                topology,
            )
            self._configure_axes_per_mcu(bridge_mcus)

        except Exception:
            logging.exception("MotionToolhead: init_planner failed")
            raise
```

(The shaper-param block at `:860-879` between Step 1's new code and this `try:` is unchanged.)

- [ ] **Step 3: Update the `motion_bridge.py` wrapper**

In `klippy/motion_bridge.py`, replace `init_planner` (`:270-300`):

```python
    def init_planner(
        self,
        max_velocity,
        max_accel,
        max_z_velocity,
        max_z_accel,
        square_corner_velocity,
        shaper_type_x,
        shaper_freq_x,
        shaper_type_y,
        shaper_freq_y,
        mcus,
        window_capacity=32,
        beta_max_iters=10,
    ):
        return self._bridge.init_planner(
            max_velocity,
            max_accel,
            max_z_velocity,
            max_z_accel,
            square_corner_velocity,
            shaper_type_x,
            shaper_freq_x,
            shaper_type_y,
            shaper_freq_y,
            mcus,
            window_capacity,
            beta_max_iters,
        )
```

- [ ] **Step 4: Update the renode gate test call site**

In `tools/test_renode_phase2_gate.py`, replace the comment + claim + call at `:187-207`:

```python
        # The bridge derives its MCU topology from the host-supplied
        # descriptor list. This harness fabricates two handles off one serial
        # connection; since it does not pump bytes through RouterTransport,
        # the handles' practical role here is just to make init_planner happy.
        octopus = bridge.claim_mcu("octopus", args.port, 0)
        f446 = bridge.claim_mcu("f446", args.port, 0)
        print("[gate] bridge.claim_mcu ok (octopus=%d, f446=%d)"
              % (octopus, f446))

        # corexy topology: octopus drives X,Y,E (tag 0); f446 drives Z (tag 1).
        topology = [
            (octopus, [0, 1, 3], 0),
            (f446, [2], 1),
        ]
        bridge.init_planner(
            300.0,   # max_velocity (mm/s)
            5000.0,  # max_accel
            10.0,    # max_z_velocity
            100.0,   # max_z_accel
            5.0,     # square_corner_velocity
            "smooth_zv", 40.0,  # X shaper
            "smooth_zv", 40.0,  # Y shaper
            topology,
        )
        print("[gate] bridge.init_planner ok")
```

- [ ] **Step 5: Rebuild the native cdylib so Python sees the new signature**

This must happen on the Pi per the bench firmware flow, but for local signature validation the cdylib can be built locally if the dev env supports it. Local check:
Run: `cd rust && cargo build -p motion-bridge-native 2>&1 | tail -5`
Expected: compiles. (The actual `.so` used by the running printer is rebuilt via the `flashing-trident-mcus` skill's host-build step — out of scope for unit verification.)

- [ ] **Step 6: Verify the static override-surface baseline still passes**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest tools/test_motion_toolhead_static.py -v 2>&1 | tail -20`
Expected: PASS — `_init_planner` is still the only init method in `EXPECTED_LOCAL_METHODS`; the new `_derive_mcu_topology` is module-level, not a method.

- [ ] **Step 7: Verify the topology unit test still passes**

Run: `cd /Users/daniladergachev/Developer/kalico && python -m pytest tools/test_mcu_topology_derive.py -v 2>&1 | tail -10`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add klippy/motion_toolhead.py klippy/motion_bridge.py tools/test_renode_phase2_gate.py
git commit -m "refactor(motion): _init_planner derives MCU topology from config"
```

---

## Final verification

- [ ] **Rust suite:** `cd rust && cargo test -p motion-bridge-native 2>&1 | tail -25` — all pass.
- [ ] **Python topology + static tests:** `python -m pytest tools/test_mcu_topology_derive.py tools/test_motion_toolhead_static.py -v 2>&1 | tail -25` — all pass.
- [ ] **Grep for residual hardcoding:** `grep -rn "octopus_handle\|f446_handle" rust/motion-bridge/src klippy/` — expect no matches in `init_planner` signatures/bodies (local-variable `octopus`/`f446` names in the gate test are fine).
- [ ] **End-to-end (bench, when ready):** flash both MCUs + rebuild host `.so` via the `flashing-trident-mcus` skill, confirm `[gate] bridge.init_planner ok` and that a real corexy print plans X/Y on the octopus and Z on the f446 exactly as before.
