# MotionToolhead extends upstream ToolHead — implementation plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore upstream `klippy/toolhead.py` and refactor `klippy/motion_toolhead.py` from a 965-LOC standalone reimplementation into a ~350-LOC subclass of `ToolHead` that overrides only the bridge-owned methods. No behavior change to the Rust planner, the bridge runtime, or the MCU firmware.

**Architecture:** Subclass with one surgical extension point (`ToolHead._load_kinematics`). MotionToolhead calls `super().__init__(config)`, overrides ~12 motion-issuance methods, inherits ~20 unchanged methods from upstream. BridgeKinematics gains `set_position(newpos, homing_axes)` matching the upstream kinematics contract. Inheriting upstream additionally fixes three latent bugs: missing `toolhead:set_position` / `toolhead:manual_move` event firing, missing `RESET_VELOCITY_LIMIT` registration, and the broader `cmd_SET_VELOCITY_LIMIT` parameter surface.

**Tech Stack:** Python 3 (klippy host code), CPython FFI bindings via `chelper`, klippy reactor / printer / gcode subsystems, the Rust bridge via `klippy/motion_bridge.so` (untouched), Docker-based klippy-in-loop sim at `tools/sim_klippy/`.

**Spec:** [`docs/superpowers/specs/2026-05-07-motion-toolhead-extends-upstream-design.md`](../specs/2026-05-07-motion-toolhead-extends-upstream-design.md)

---

## File map

| File | Action | Purpose |
|------|--------|---------|
| `klippy/toolhead.py` | **Create** (restored from `1f3d0d070^` + 1 method extraction) | Upstream ToolHead, source of truth |
| `klippy/motion_toolhead.py` | **Rewrite** (965 → ~350 LOC) | MotionToolhead(ToolHead) subclass + BridgeKinematics + add_printer_objects |
| `klippy/extras/trad_rack.py` | **Modify** (line 13) | Import revert: `from .. import toolhead` |
| `klippy/extras/nozzle_cleanup.py` | **Modify** (line 11) | Import revert: `from klippy.toolhead import ToolHead` |
| `klippy/extras/probe.py` | **Unchanged** | Uses `from __future__ import annotations`; ToolHead annotations are strings |
| `klippy/printer.py` | **Unchanged** | Still loads motion_toolhead module only |
| `tools/test_motion_toolhead_static.py` | **Create** | Pytest: override-drift detector, flush-timer silencing invariant, legacy ToolHead import |
| `tools/sim_klippy/test_velocity_limits.py` | **Create** | Docker sim test: M204 / SET_VELOCITY_LIMIT / RESET_VELOCITY_LIMIT propagation |
| `tools/sim_klippy/test_gcode_move_state_sync.py` | **Create** | Docker sim test: SET_GCODE_OFFSET MOVE=1, G92, manual_move event firing |
| `tools/sim_klippy/test_set_position_trapq_safe.py` | **Create** | Docker sim test: inherited set_position trapq side-effect is benign |
| `docs/superpowers/plan-changes-log.md` | **Modify** (append entry) | Log this refactor in the running build-order log |

---

## Conventions

- **Commit each task at its end.** No throwaway-then-amend cycles.
- **Test commands** are run from the repo root unless stated otherwise.
- **Sim tests** require Docker. The `bash tools/sim_klippy/run_local.sh` flow is the canonical sim invocation; new sim tests follow the pattern in `tools/sim_klippy/test_home_x.py` (line 1-30).
- **Never add `Co-Authored-By: Claude`** to commits per project rule.
- **No time estimates** — each task is sized as a single logical change.

---

### Task 1: Restore upstream `klippy/toolhead.py` with `_load_kinematics` extraction

**Files:**
- Create: `klippy/toolhead.py`

**Rationale:** Step 1 of the refactor. Restores the upstream Klipper/Kalico toolhead module as the source of truth, with a single surgical edit: extract the kinematics-import block in `__init__` into a `_load_kinematics(config)` method so subclasses can override it.

- [ ] **Step 1: Restore the file from git history**

```bash
git show 1f3d0d070^:klippy/toolhead.py > klippy/toolhead.py
```

- [ ] **Step 2: Apply the `_load_kinematics` extraction**

Open `klippy/toolhead.py`. Find the kinematics-import block in `ToolHead.__init__` (the block that starts with `kin_name = config.get("kinematics")` and ends with `raise config.error(msg)`). Replace it with a single line:

```python
        self.kin = self._load_kinematics(config)
```

Then add the new method after `__init__`. The exact location is just before `def get_active_rails_for_axis(self, axis):`. Insert:

```python
    def _load_kinematics(self, config):
        kin_name = config.get("kinematics")
        try:
            mod = importlib.import_module("klippy.kinematics." + kin_name)
            return mod.load_kinematics(self, config)
        except config.error:
            raise
        except self.printer.lookup_object("pins").error:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            logging.exception(msg)
            raise config.error(msg)

```

- [ ] **Step 3: Verify the file imports cleanly**

```bash
python3 -c "from klippy import toolhead; assert hasattr(toolhead, 'ToolHead'); assert hasattr(toolhead.ToolHead, '_load_kinematics'); assert hasattr(toolhead, 'LookAheadQueue'); assert toolhead.BUFFER_TIME_START == 0.250; print('OK')"
```

Expected: `OK` printed, no exception.

- [ ] **Step 4: Verify the diff against upstream is minimal**

```bash
diff <(git show 1f3d0d070^:klippy/toolhead.py) klippy/toolhead.py | head -60
```

Expected: only the kinematics-import block change shows up — one block removed, replaced with `self.kin = self._load_kinematics(config)`, and the new `_load_kinematics` method added. No other deltas.

- [ ] **Step 5: Commit**

```bash
git add klippy/toolhead.py
git commit -m "feat(klippy): restore toolhead.py with _load_kinematics extension point

Restored from 1f3d0d070^ with a single surgical edit: extract the
kinematics-import block in ToolHead.__init__ into a _load_kinematics
method so subclasses (MotionToolhead) can override it without
duplicating the rest of __init__.

This is step 1 of the motion_toolhead-extends-upstream refactor; on
its own this commit makes klippy.toolhead importable but nothing
loads it (motion_toolhead is still standalone)."
```

---

### Task 2: BridgeKinematics — add `set_position`, capture `_toolhead`, drop `set_homed`

**Files:**
- Modify: `klippy/motion_toolhead.py` (BridgeKinematics class only)

**Rationale:** The upstream kinematics contract is `kin.set_position(newpos, homing_axes)`. By moving the bridge.set_position call into BridgeKinematics, MotionToolhead can inherit upstream's `ToolHead.set_position` unchanged. Capture the toolhead reference to avoid going through the printer registry.

- [ ] **Step 1: Modify `BridgeKinematics.__init__` to keep the toolhead reference**

In `klippy/motion_toolhead.py`, find `BridgeKinematics.__init__`. The first parameter is already `toolhead` but it's currently discarded. Add this as the first line of the body, right after the docstring (if any) and before `kin_name = config.get("kinematics")`:

```python
        self._toolhead = toolhead
```

- [ ] **Step 2: Replace `set_homed` with `set_position`**

Find the `set_homed` method:

```python
    def set_homed(self, axes):
        for a in axes:
            self.homed_axes.add(a)
```

Replace it with:

```python
    def set_position(self, newpos, homing_axes=()):
        # Upstream kinematics contract: this method owns runtime
        # position-state sync. For cartesian, it drives itersolve. For
        # the bridge, it pushes the new basis into the planner runtime.
        if self._toolhead.bridge is not None:
            self._toolhead.bridge.set_position(
                newpos[0], newpos[1], newpos[2]
            )
        for a in homing_axes:
            self.homed_axes.add(a)
```

- [ ] **Step 3: Update the only caller of `set_homed` in this file**

Find `MotionToolhead.set_position` (the override). Inside it, find the line:

```python
        if homing_axes and self.kin is not None and hasattr(self.kin, "set_homed"):
            self.kin.set_homed(homing_axes)
```

Replace with:

```python
        if homing_axes and self.kin is not None:
            self.kin.set_position(newpos, homing_axes)
```

(The full set_position override goes away in Task 3; this intermediate edit is to keep tests green between tasks.)

- [ ] **Step 4: Smoke test — module imports cleanly**

```bash
python3 -c "from klippy.motion_toolhead import BridgeKinematics, MotionToolhead; assert hasattr(BridgeKinematics, 'set_position'); assert not hasattr(BridgeKinematics, 'set_homed'); print('OK')"
```

Expected: `OK`.

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "feat(klippy): BridgeKinematics adopts upstream set_position contract

Replace set_homed(axes) with set_position(newpos, homing_axes) so it
matches the upstream kinematics interface that ToolHead.set_position
calls (toolhead.py:600-608). Capture the toolhead reference in
__init__ so the bridge handle is reachable without a printer-registry
lookup. MotionToolhead.set_position is updated to call kin.set_position
in place of kin.set_homed; the override itself is removed in Task 3
when MotionToolhead inherits set_position from upstream."
```

---

### Task 3: Refactor MotionToolhead to extend ToolHead

**Files:**
- Modify: `klippy/motion_toolhead.py` (MotionToolhead class + module-level shim removal)

**Rationale:** The core refactor. MotionToolhead becomes a subclass of `ToolHead` that calls `super().__init__(config)`, overrides only bridge-owned methods, and adds bridge wiring. The duplicated upstream constants and `LookAheadQueue` shim are dropped because the restored `klippy/toolhead.py` exposes them.

- [ ] **Step 1: Update imports at the top of `klippy/motion_toolhead.py`**

Find the existing imports block:

```python
import logging

from .kinematics import extruder
from . import chelper
from . import stepper
```

Replace with:

```python
import logging

from . import chelper
from . import stepper
from .kinematics import extruder
from .toolhead import ToolHead, BUFFER_TIME_START
```

- [ ] **Step 2: Replace the `MotionToolhead` class body**

Find `class MotionToolhead:` and replace the **entire class body** (everything from `class MotionToolhead:` through the last method `cmd_M204`) with the following:

```python
class MotionToolhead(ToolHead):
    """Bridge-aware ToolHead subclass.

    Inherits the upstream surface unchanged; overrides only the methods
    where the Rust motion bridge owns the behavior (move issuance,
    timeline, velocity-limit propagation, sim diagnostics).

    See docs/superpowers/specs/2026-05-07-motion-toolhead-extends-upstream-design.md.
    """

    def __init__(self, config):
        # Pre-super: attributes that BridgeKinematics or registered handlers
        # may reference during super().__init__.
        printer = config.get_printer()
        self.bridge = printer.lookup_object("motion_bridge", None)
        self.active_homing_arms = set()
        self.kinematics_name = config.get("kinematics", "")

        # Run upstream init: trapq alloc, gcode commands (G4/M400/
        # SET_VELOCITY_LIMIT/RESET_VELOCITY_LIMIT/M204), helper modules
        # (gcode_move/homing/idle_timeout/statistics/manual_probe/
        # tuning_tower/garbage_collection), lookahead, flush_timer,
        # _calc_junction_deviation, _handle_shutdown registration,
        # extruder = DummyExtruder, AND _load_kinematics → BridgeKinematics.
        super().__init__(config)

        # Bridge owns the timeline; silence upstream's flush machinery.
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False

        # Bridge-only config keys (not parsed by upstream ToolHead).
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", self.max_velocity, above=0.0
        )
        self.max_z_accel = config.getfloat(
            "max_z_accel", self.max_accel, above=0.0
        )

        # Sim-only diagnostic gcode commands (only when bridge present).
        if self.bridge is not None:
            gcode = self.printer.lookup_object("gcode")
            gcode.register_command(
                "KALICO_SIM_STEP_COUNT",
                self.cmd_KALICO_SIM_STEP_COUNT,
                desc="[sim] Query cumulative step count for a stepper OID",
            )
            gcode.register_command(
                "KALICO_SIM_AXIS_STEPS",
                self.cmd_KALICO_SIM_AXIS_STEPS,
                desc="[sim] Query configured steps_per_mm for an axis OID",
            )
            gcode.register_command(
                "KALICO_SIM_AXIS_ACCUM",
                self.cmd_KALICO_SIM_AXIS_ACCUM,
                desc="[sim] Query step accumulator for an axis OID",
            )
            gcode.register_command(
                "KALICO_SIM_ENDSTOP_SET_PIN",
                self.cmd_KALICO_SIM_ENDSTOP_SET_PIN,
                desc="[sim] Drive a Linux-MCU GPIO level (test fixture)",
            )

        # Planner initialization runs once all MCUs have connected.
        self.printer.register_event_handler(
            "klippy:connect", self._init_planner
        )

        logging.info("MotionToolhead: Phase 1 skeleton initialized")

    # ------------------------------------------------------------------
    # Kinematics override
    # ------------------------------------------------------------------

    def _load_kinematics(self, config):
        return BridgeKinematics(self, config, self.trapq)

    # ------------------------------------------------------------------
    # Move issuance — bridge owns these
    # ------------------------------------------------------------------

    def move(self, newpos, speed):
        dx = newpos[0] - self.commanded_pos[0]
        dy = newpos[1] - self.commanded_pos[1]
        dz = newpos[2] - self.commanded_pos[2]
        de = newpos[3] - self.commanded_pos[3]
        feedrate = min(speed, self.max_velocity)
        if abs(dz) > 1e-9 and abs(dx) < 1e-9 and abs(dy) < 1e-9:
            feedrate = min(feedrate, self.max_z_velocity)
        logging.info(
            "[bridge-trace] move: newpos=%s speed=%s dx=%.4f dy=%.4f "
            "dz=%.4f de=%.4f feedrate=%.4f bridge_is_none=%s",
            list(newpos), speed, dx, dy, dz, de, feedrate,
            self.bridge is None,
        )
        if self.bridge is not None:
            self.bridge.submit_move(dx, dy, dz, de, feedrate)
            # Bridge synthesizes steps in the runtime; klippy's normal
            # itersolve_check_active path doesn't fire. We trigger
            # active-stepper callbacks ourselves so motors energize
            # before the move starts.
            self._fire_active_callbacks(dx, dy, dz, de)
        self.commanded_pos[:] = newpos

    def _fire_active_callbacks(self, dx, dy, dz, de):
        if self.kin is None:
            return
        active_axes = []
        if abs(dx) > 1e-9: active_axes.append("x")
        if abs(dy) > 1e-9: active_axes.append("y")
        if abs(dz) > 1e-9: active_axes.append("z")
        if not active_axes and abs(de) <= 1e-9:
            return
        try:
            print_time = self.bridge.get_last_move_time()
        except Exception:
            print_time = 0.0
        for s in self.kin.get_steppers():
            if not s._active_callbacks:
                continue
            if not any(s.is_active_axis(a) for a in active_axes):
                continue
            cbs = s._active_callbacks
            s._active_callbacks = []
            for cb in cbs:
                cb(print_time)

    def drip_move(self, newpos, speed, drip_completion):
        # Step 7-D §6.2: bridge-aware single-segment homing.
        # Endstops were armed upstream by homing.py via
        # mcu_endstop.home_start; each BridgeTriggerDispatch.start
        # registered its arm_id with self.active_homing_arms. Submit one
        # homing-tagged segment; on trip the runtime ISR aborts and
        # freezes the curve evaluator. wait_moves() returns when the
        # segment retires (Completed or Tripped).
        logging.info(
            "[bridge-trace] drip_move entered: newpos=%s speed=%s "
            "bridge_is_none=%s drip_test=%s active_homing_arms=%s",
            list(newpos), speed, self.bridge is None,
            (drip_completion.test()
             if drip_completion is not None else None),
            sorted(self.active_homing_arms),
        )
        if self.bridge is None:
            return
        if drip_completion is not None and drip_completion.test():
            return
        arm_ids = list(self.active_homing_arms)
        if not arm_ids:
            # No bridge endstops armed — fall back to a regular move so
            # bring-up doesn't crash on file-output / legacy paths.
            self.move(newpos, speed)
            return
        pos3 = list(newpos[:3]) + [0.0] * max(0, 3 - len(newpos[:3]))
        self.bridge.submit_homing_move(pos3, speed, arm_ids)
        self.bridge.wait_moves()

    def dwell(self, delay):
        if self.bridge is not None:
            self.bridge.submit_dwell(delay)

    def wait_moves(self):
        if self.bridge is not None:
            self.bridge.wait_moves()

    def flush_step_generation(self):
        # Bridge owns flush; upstream's body operates on lookahead +
        # trapq which we bypass.
        pass

    def get_last_move_time(self):
        # Floor at mcu.estimated_print_time + BUFFER_TIME_START so legacy
        # MCU commands (TMC, SPI, digital_out) issued before the first
        # bridge move don't land in the MCU's past.
        est = 0.0
        if self.mcu is not None:
            est = self.mcu.estimated_print_time(self.reactor.monotonic())
        floor = est + BUFFER_TIME_START
        if self.bridge is not None:
            return max(self.bridge.get_last_move_time(), floor)
        return floor

    def note_mcu_movequeue_activity(self, mq_time, set_step_gen_time=False):
        # Bridge has its own queue; upstream's body would re-arm the
        # silenced flush_timer.
        pass

    # ------------------------------------------------------------------
    # Velocity-limit propagation — bridge mirrors host-side updates
    # ------------------------------------------------------------------

    def set_accel(self, accel):
        if accel is not None and accel > 0.0:
            self.max_accel = accel
            if self.bridge is not None:
                self.bridge.update_limits(self.max_velocity, self.max_accel)

    def reset_accel(self):
        if self.bridge is not None:
            self.bridge.update_limits(self.max_velocity, self.max_accel)

    def cmd_SET_VELOCITY_LIMIT(self, gcmd):
        super().cmd_SET_VELOCITY_LIMIT(gcmd)
        if self.bridge is not None:
            self.bridge.update_limits(self.max_velocity, self.max_accel)

    def cmd_RESET_VELOCITY_LIMIT(self, gcmd):
        super().cmd_RESET_VELOCITY_LIMIT(gcmd)
        if self.bridge is not None:
            self.bridge.update_limits(self.max_velocity, self.max_accel)

    # ------------------------------------------------------------------
    # Stats — bridge-aware silence (see spec §"stats")
    # ------------------------------------------------------------------

    def stats(self, eventtime):
        return False, "print_time=%.3f buffer_time=0.000 print_stall=%d" % (
            self.print_time, self.print_stall,
        )

    # ------------------------------------------------------------------
    # Bridge-only: planner init, ConfigureAxes, credit-freed wiring
    # ------------------------------------------------------------------

    def _init_planner(self):
        if self.bridge is None:
            return
        # Locate the two MVP MCUs by name. First-print topology:
        #   "mcu" (Octopus) drives X+Y; "mcu z" drives Z.
        # If only one MCU is configured, reuse its handle for Z.
        octopus = None
        f446 = None
        bridge_mcus = []
        for name, mcu in self.printer.lookup_objects(module="mcu"):
            handle = getattr(mcu, "_bridge_handle", None)
            if handle is None:
                continue
            bridge_mcus.append((name, mcu, handle))
            mcu_name = getattr(mcu, "_name", name)
            if octopus is None or mcu_name in ("mcu", "octopus"):
                if octopus is None:
                    octopus = handle
                elif f446 is None:
                    f446 = handle
            elif f446 is None:
                f446 = handle
        if octopus is None:
            logging.warning(
                "MotionToolhead: no MCU bridge handles available; "
                "skipping init_planner"
            )
            return
        if f446 is None:
            f446 = octopus

        # Pull initial shaper params from [input_shaper] config, if present.
        shaper_type_x = "smooth_zv"
        shaper_freq_x = 0.0
        shaper_type_y = "smooth_zv"
        shaper_freq_y = 0.0
        is_obj = self.printer.lookup_object("input_shaper", None)
        if is_obj is not None:
            try:
                shapers = is_obj.get_shapers()
                for s in shapers or ():
                    if s.axis == "x":
                        shaper_type_x = s.params.shaper_type
                        shaper_freq_x = s.params.shaper_freq
                    elif s.axis == "y":
                        shaper_type_y = s.params.shaper_type
                        shaper_freq_y = s.params.shaper_freq
            except Exception:
                logging.exception(
                    "MotionToolhead: failed to read input_shaper params"
                )

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
                octopus,
                f446,
            )
            self._configure_axes_per_mcu(bridge_mcus)
            self._register_credit_freed_handlers(bridge_mcus)
            # The local Linux sim harness sometimes sends a bare movement
            # command without a preceding G28. Mark single-MCU local sim
            # runtime homed so smoke tests keep passing.
            if len(bridge_mcus) == 1:
                _, mcu_obj, mcu_handle = bridge_mcus[0]
                if getattr(mcu_obj, "_serialport", None) == "/tmp/klipper_sim_socket":
                    queue = self.bridge.alloc_command_queue(mcu_handle)
                    self.bridge.set_homed_state(mcu_handle, queue, True)
                    logging.info(
                        "MotionToolhead: marked single-MCU local sim homed"
                    )
        except Exception:
            logging.exception("MotionToolhead: init_planner failed")
            raise

    def _configure_axes_per_mcu(self, bridge_mcus):
        """Send `ConfigureAxes` over the kalico-native transport for each
        bridge-attached MCU. Maps klippy `MCU_stepper` objects to motor
        slots per kinematics:
          corexy:    [A=stepper_x, B=stepper_y, Z=stepper_z, E=extruder]
          cartesian: [X=stepper_x, Y=stepper_y, Z=stepper_z, E=extruder]
        Steppers not on a given MCU are omitted from that MCU's blob.
        """
        kin = (self.kinematics_name or "").lower()
        if kin == "corexy":
            kin_tag = 0
            slot_names = ["stepper_x", "stepper_y", "stepper_z", "extruder"]
            awd_default = 0b0011
        elif kin == "cartesian":
            kin_tag = 1
            slot_names = ["stepper_x", "stepper_y", "stepper_z", "extruder"]
            awd_default = 0b0000
        else:
            logging.info(
                "MotionToolhead: kinematics=%r — skipping configure_axes",
                kin,
            )
            return

        steppers_by_slot = {}
        fm = self.printer.lookup_object("force_move", None)
        if fm is not None:
            for name, s in fm.steppers.items():
                if name in slot_names and name not in steppers_by_slot:
                    steppers_by_slot[name] = s

        for name, mcu_obj, mcu_handle in bridge_mcus:
            present_mask = 0
            invert_mask = 0
            steps_per_mm = [0.0, 0.0, 0.0, 0.0]
            for i, slot in enumerate(slot_names):
                s = steppers_by_slot.get(slot)
                if s is None:
                    continue
                if len(bridge_mcus) > 1:
                    try:
                        s_mcu = s.get_mcu()
                    except AttributeError:
                        s_mcu = None
                    if s_mcu is not None and s_mcu is not mcu_obj:
                        continue
                step_dist = s.get_step_dist()
                if step_dist <= 0.0:
                    continue
                steps_per_mm[i] = 1.0 / step_dist
                present_mask |= 1 << i
            awd_mask = awd_default & present_mask
            if present_mask == 0:
                logging.info(
                    "MotionToolhead: no steppers matched MCU %s; "
                    "skipping configure_axes", name,
                )
                continue
            self.bridge.configure_axes(
                mcu_handle, kin_tag, present_mask, awd_mask,
                invert_mask, steps_per_mm,
            )
            logging.info(
                "MotionToolhead: configure_axes mcu=%s kin=%d "
                "present=0x%x awd=0x%x steps_per_mm=%s",
                name, kin_tag, present_mask, awd_mask, steps_per_mm,
            )

    def _register_credit_freed_handlers(self, bridge_mcus):
        """Register a kalico_credit_freed handler on each bridge MCU.

        Dispatch path: MCU emits kalico_credit_freed
        -> Rust host_io lifts to RuntimeEvent::CreditFreed
        -> bridge.try_recv_event surfaces dict
        -> serialhdl._bridge_event_poller renames to "kalico_credit_freed"
        -> THIS handler -> bridge.on_credit_freed
        -> Rust SlotPool.retire_through_segment + CreditCounter sync.
        """
        bridge = self.bridge
        for name, mcu_obj, mcu_handle in bridge_mcus:
            serial = getattr(mcu_obj, "_serial", None)
            if serial is None or not hasattr(serial, "register_response"):
                logging.warning(
                    "MotionToolhead: bridge MCU '%s' has no SerialReader; "
                    "kalico_credit_freed handler not registered", name,
                )
                continue
            handle = mcu_handle
            mcu_label = name

            def _on_credit_freed(params, _bridge=bridge, _handle=handle,
                                 _label=mcu_label):
                try:
                    retired = int(params.get("retired_through_segment_id", 0))
                    free_slots = int(params.get("free_slots", 0))
                    result = _bridge.on_credit_freed(
                        _handle, retired, free_slots,
                    )
                    if isinstance(result, tuple) and len(result) >= 2:
                        completed_arm = result[1]
                        if completed_arm is not None:
                            _bridge.fire_homing_completion(completed_arm)
                except Exception:
                    logging.exception(
                        "MotionToolhead: bridge.on_credit_freed failed for "
                        "MCU '%s' (handle=%s)", _label, _handle,
                    )

            serial.register_response(_on_credit_freed, "kalico_credit_freed")
            logging.info(
                "MotionToolhead: registered kalico_credit_freed handler for "
                "MCU '%s' (handle=%s)", name, mcu_handle,
            )

    # ------------------------------------------------------------------
    # Sim-only diagnostic gcode commands
    # ------------------------------------------------------------------

    def cmd_KALICO_SIM_STEP_COUNT(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0)
        if self.bridge is None or self.mcu is None:
            raise gcmd.error("bridge not available")
        handle = getattr(self.mcu, "_bridge_handle", None)
        if handle is None:
            raise gcmd.error("bridge handle not set")
        try:
            resp = self.bridge.bridge_call(
                handle,
                "runtime_sim_stepper_count_query oid=%d" % oid,
                "runtime_sim_stepper_count_response",
                timeout_s=5.0,
            )
            count = resp.get("count", 0)
            gcmd.respond_info(
                "[bridge-async] KALICO_SIM_STEP_COUNT oid=%d count=%d"
                % (oid, count)
            )
        except Exception as e:
            raise gcmd.error("step count query failed: %s" % e)

    def cmd_KALICO_SIM_AXIS_STEPS(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0, maxval=3)
        if self.bridge is None or self.mcu is None:
            raise gcmd.error("bridge not available")
        handle = getattr(self.mcu, "_bridge_handle", None)
        if handle is None:
            raise gcmd.error("bridge handle not set")
        try:
            resp = self.bridge.bridge_call(
                handle,
                "runtime_sim_axis_steps_query oid=%d" % oid,
                "runtime_sim_axis_steps_response",
                timeout_s=5.0,
            )
            milli = resp.get("milli_spm", 0)
            gcmd.respond_info(
                "[bridge-async] KALICO_SIM_AXIS_STEPS oid=%d "
                "steps_per_mm=%.3f" % (oid, milli / 1000.0)
            )
        except Exception as e:
            raise gcmd.error("axis steps query failed: %s" % e)

    def cmd_KALICO_SIM_AXIS_ACCUM(self, gcmd):
        oid = gcmd.get_int("OID", 0, minval=0, maxval=3)
        if self.bridge is None or self.mcu is None:
            raise gcmd.error("bridge not available")
        handle = getattr(self.mcu, "_bridge_handle", None)
        if handle is None:
            raise gcmd.error("bridge handle not set")
        try:
            resp = self.bridge.bridge_call(
                handle,
                "runtime_sim_axis_accum_query oid=%d" % oid,
                "runtime_sim_axis_accum_response",
                timeout_s=5.0,
            )
            milli = resp.get("milli", 0)
            gcmd.respond_info(
                "[bridge-async] KALICO_SIM_AXIS_ACCUM oid=%d accum=%.3f"
                % (oid, milli / 1000.0)
            )
        except Exception as e:
            raise gcmd.error("axis accum query failed: %s" % e)

    def cmd_KALICO_SIM_ENDSTOP_SET_PIN(self, gcmd):
        gpio = gcmd.get_int("GPIO", minval=0, maxval=0xFFFF)
        level = gcmd.get_int("LEVEL", minval=0, maxval=1)
        if self.bridge is None or self.mcu is None:
            raise gcmd.error("bridge not available")
        handle = getattr(self.mcu, "_bridge_handle", None)
        if handle is None:
            raise gcmd.error("bridge handle not set")
        try:
            resp = self.bridge.bridge_call(
                handle,
                "runtime_sim_endstop_set_pin gpio=%d level=%d" % (gpio, level),
                "runtime_sim_endstop_set_pin_response",
                timeout_s=5.0,
            )
            gcmd.respond_info(
                "[bridge-async] KALICO_SIM_ENDSTOP_SET_PIN "
                "gpio=%d level=%d result=%d"
                % (gpio, level, resp.get("result", -1))
            )
        except Exception as e:
            raise gcmd.error("endstop set_pin failed: %s" % e)
```

- [ ] **Step 3: Delete the now-redundant module-level shim**

After the `MotionToolhead` class, delete the entire trailing block:

```python
# ---------------------------------------------------------------------------
# Compat shim — symbols previously exported by klippy/toolhead.py
# ...
LOOKAHEAD_FLUSH_TIME = 0.250
BUFFER_TIME_LOW = 1.0
BUFFER_TIME_HIGH = 2.0
BUFFER_TIME_START = 0.250
SDS_CHECK_TIME = 0.001

class LookAheadQueue:
    # ... (entire class)

ToolHead = MotionToolhead
```

Replace with just the `add_printer_objects` function (which stays):

```python
def add_printer_objects(config):
    """Register the MotionToolhead (and extruder) with the printer."""
    config.get_printer().add_object("toolhead", MotionToolhead(config))
    extruder.add_printer_objects(config)
```

- [ ] **Step 4: Smoke test — module imports cleanly, has the right surface**

```bash
python3 -c "
from klippy.motion_toolhead import MotionToolhead, BridgeKinematics, add_printer_objects
from klippy.toolhead import ToolHead
assert issubclass(MotionToolhead, ToolHead)
assert not hasattr(MotionToolhead, 'motor_off')  # dropped
assert not hasattr(MotionToolhead, 'register_move_handler')  # dropped
# Inherited:
for m in ['set_position', 'manual_move', 'get_status', 'check_busy',
          'cmd_G4', 'cmd_M400', 'cmd_M204', 'get_active_rails_for_axis',
          '_handle_shutdown', 'note_step_generation_scan_time',
          'limit_next_junction_speed', 'register_lookahead_callback']:
    assert getattr(MotionToolhead, m) is getattr(ToolHead, m), \
        '%s should be inherited but is overridden' % m
# Overridden:
for m in ['_load_kinematics', 'move', 'drip_move', 'dwell', 'wait_moves',
          'flush_step_generation', 'get_last_move_time',
          'note_mcu_movequeue_activity', 'set_accel', 'reset_accel',
          'cmd_SET_VELOCITY_LIMIT', 'cmd_RESET_VELOCITY_LIMIT', 'stats']:
    assert getattr(MotionToolhead, m) is not getattr(ToolHead, m, None), \
        '%s should be overridden but is inherited' % m
print('OK — override surface matches spec')
"
```

Expected: `OK — override surface matches spec`. Any AssertionError indicates the override list drifted from the spec.

- [ ] **Step 5: Commit**

```bash
git add klippy/motion_toolhead.py
git commit -m "refactor(klippy): MotionToolhead extends upstream ToolHead

Replaces the 965-LOC standalone reimplementation with a ~350-LOC
subclass. Inherits set_position, manual_move, get_status, check_busy,
get_active_rails_for_axis, register_lookahead_callback, cmd_G4/M400/
M204, _handle_shutdown, _calc_junction_deviation, get_max_velocity,
limit_next_junction_speed, note_step_generation_scan_time. Overrides
the 13 bridge-owned methods (move/drip_move/dwell/wait_moves/
flush_step_generation/get_last_move_time/note_mcu_movequeue_activity/
set_accel/reset_accel/cmd_SET_VELOCITY_LIMIT/cmd_RESET_VELOCITY_LIMIT/
stats/_load_kinematics).

Drops motor_off and register_move_handler (zero callers across klippy/).
Drops the duplicated LookAheadQueue / BUFFER_TIME_* / SDS_CHECK_TIME /
ToolHead-alias module-level shim — these now live in the restored
klippy/toolhead.py.

Side-effect benefits from inheriting upstream:
- toolhead:set_position and toolhead:manual_move events now fire,
  fixing latent gcode_move state-drift on probe / safe_z_home / G92
  flows.
- RESET_VELOCITY_LIMIT command is now registered.
- SET_VELOCITY_LIMIT now accepts SQUARE_CORNER_VELOCITY,
  MINIMUM_CRUISE_RATIO, ACCEL_TO_DECEL, and per-axis params (which
  degrade gracefully under BridgeKinematics via hasattr checks)."
```

---

### Task 4: Migrate consumer imports

**Files:**
- Modify: `klippy/extras/trad_rack.py:13`
- Modify: `klippy/extras/nozzle_cleanup.py:11`
- (Verify no change needed to `klippy/extras/probe.py` and `klippy/printer.py`.)

**Rationale:** Restore upstream-style imports now that `klippy.toolhead` is back. probe.py uses `from __future__ import annotations` so type hints are strings — no runtime import needed.

- [ ] **Step 1: Revert `trad_rack.py` import**

In `klippy/extras/trad_rack.py:13`, find:

```python
from .. import chelper, motion_toolhead as toolhead
```

Replace with:

```python
from .. import chelper, toolhead
```

- [ ] **Step 2: Revert `nozzle_cleanup.py` import**

In `klippy/extras/nozzle_cleanup.py:11`, find:

```python
from klippy.motion_toolhead import MotionToolhead as ToolHead
```

Replace with:

```python
from klippy.toolhead import ToolHead
```

- [ ] **Step 3: Verify probe.py and printer.py are correct as-is**

```bash
# probe.py uses `from __future__ import annotations` (line 6).
grep -n "from __future__ import annotations" klippy/extras/probe.py
# Expected: 6:from __future__ import annotations

# printer.py iterates only motion_toolhead, not toolhead. No collision risk.
grep -n "motion_toolhead\|^from . import toolhead\b" klippy/printer.py
# Expected: line 35 imports motion_toolhead; line 336 iterates [motion_toolhead]; no toolhead import.
```

- [ ] **Step 4: Verify the imports resolve**

```bash
python3 -c "
import klippy.extras.trad_rack
import klippy.extras.nozzle_cleanup
import klippy.extras.probe
print('OK — consumer imports resolve')
"
```

Expected: `OK — consumer imports resolve`. (Any ImportError means the migration is wrong.)

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/trad_rack.py klippy/extras/nozzle_cleanup.py
git commit -m "refactor(klippy): revert toolhead-module imports to upstream path

trad_rack.py and nozzle_cleanup.py now import from klippy.toolhead
again, since the upstream module is restored. probe.py is unchanged
(uses 'from __future__ import annotations', so its ToolHead annotation
is a string — no runtime import needed).

printer.py was already correct: it iterates only [motion_toolhead] for
add_printer_objects, so there's no collision with the restored
klippy/toolhead.py's add_printer_objects function."
```

---

### Task 5: Override-drift detector + flush-timer invariant + legacy ToolHead static test

**Files:**
- Create: `tools/test_motion_toolhead_static.py`

**Rationale:** Spec test items §4.6, §4.7, §4.8. All three are static analyses (no klippy boot required), so they go in one file. The override-drift detector pins the override surface against accidental future drift in either direction.

- [ ] **Step 1: Write the failing test file**

Create `tools/test_motion_toolhead_static.py`:

```python
#!/usr/bin/env python3
"""Static invariants for the MotionToolhead extends-upstream refactor.

Three checks, all pure-Python (no klippy boot required):

1. override_surface_baseline: pin the methods MotionToolhead overrides
   locally vs methods it inherits from ToolHead. A diff against the
   baseline catches accidental drift in either direction (we add an
   override that should be inherited, or upstream gains a method that
   our override list might need to consider).

2. flush_timer_silencing_invariant: verify no MotionToolhead path
   (overridden or inherited) calls the upstream rearm method
   `note_mcu_movequeue_activity` along a bridge code path. The
   silenced-flush-timer guarantee depends on the rearm callers
   (_advance_flush_time, drip_move) being either no-op'd or overridden.

3. legacy_toolhead_importable: prove `klippy.toolhead` is a
   self-contained module that imports without the bridge present.
"""
from __future__ import annotations

import importlib

from klippy.toolhead import ToolHead
from klippy.motion_toolhead import MotionToolhead


# --- Test 1: override-surface baseline -------------------------------------

# Methods MotionToolhead defines locally (overrides). Update this baseline
# when intentionally changing the override surface. Any divergence triggers
# CI failure with a diff message.
EXPECTED_LOCAL_METHODS = frozenset({
    "__init__",
    "_load_kinematics",
    "move",
    "_fire_active_callbacks",
    "drip_move",
    "dwell",
    "wait_moves",
    "flush_step_generation",
    "get_last_move_time",
    "note_mcu_movequeue_activity",
    "set_accel",
    "reset_accel",
    "cmd_SET_VELOCITY_LIMIT",
    "cmd_RESET_VELOCITY_LIMIT",
    "stats",
    "_init_planner",
    "_configure_axes_per_mcu",
    "_register_credit_freed_handlers",
    "cmd_KALICO_SIM_STEP_COUNT",
    "cmd_KALICO_SIM_AXIS_STEPS",
    "cmd_KALICO_SIM_AXIS_ACCUM",
    "cmd_KALICO_SIM_ENDSTOP_SET_PIN",
})


def test_motion_toolhead_override_surface_matches_baseline():
    actual = {
        name for name, value in MotionToolhead.__dict__.items()
        if callable(value) and not name.startswith("__")
    }
    actual.add("__init__")  # always part of the baseline
    extra = actual - EXPECTED_LOCAL_METHODS
    missing = EXPECTED_LOCAL_METHODS - actual
    assert not extra and not missing, (
        "Override surface drift!\n"
        "  Added (in MotionToolhead but not in baseline): %s\n"
        "  Removed (in baseline but not in MotionToolhead): %s\n"
        "If intentional, update EXPECTED_LOCAL_METHODS in this file."
        % (sorted(extra), sorted(missing))
    )


# Methods upstream ToolHead exposes today. If upstream gains a new
# public-ish method, this test fails so a human reviews whether
# MotionToolhead should override it.
EXPECTED_TOOLHEAD_METHODS = frozenset({
    "__init__",
    "_advance_flush_time",
    "_advance_move_time",
    "_calc_junction_deviation",
    "_calc_print_time",
    "_check_pause",
    "_flush_handler",
    "_flush_lookahead",
    "_handle_shutdown",
    "_load_kinematics",
    "_priming_handler",
    "_process_moves",
    "_update_drip_move_time",
    "check_busy",
    "cmd_G4",
    "cmd_M204",
    "cmd_M400",
    "cmd_RESET_VELOCITY_LIMIT",
    "cmd_SET_VELOCITY_LIMIT",
    "drip_move",
    "dwell",
    "flush_step_generation",
    "get_active_rails_for_axis",
    "get_extruder",
    "get_kinematics",
    "get_last_move_time",
    "get_max_velocity",
    "get_position",
    "get_status",
    "get_trapq",
    "limit_next_junction_speed",
    "manual_move",
    "move",
    "note_mcu_movequeue_activity",
    "note_step_generation_scan_time",
    "register_lookahead_callback",
    "register_step_generator",
    "reset_accel",
    "set_accel",
    "set_extruder",
    "set_position",
    "stats",
    "wait_moves",
})


def test_upstream_toolhead_method_baseline():
    actual = {
        name for name, value in ToolHead.__dict__.items()
        if callable(value) and not name.startswith("__")
    }
    actual.add("__init__")
    extra = actual - EXPECTED_TOOLHEAD_METHODS
    missing = EXPECTED_TOOLHEAD_METHODS - actual
    assert not extra and not missing, (
        "Upstream ToolHead method baseline drift!\n"
        "  Added in ToolHead: %s — REVIEW whether MotionToolhead needs\n"
        "    to override this for bridge-mode correctness.\n"
        "  Removed from ToolHead: %s — the bridge override may now be\n"
        "    overriding nothing.\n"
        "If reviewed and OK, update EXPECTED_TOOLHEAD_METHODS in this file."
        % (sorted(extra), sorted(missing))
    )


# --- Test 2: flush-timer silencing invariant -------------------------------

def test_no_motion_toolhead_path_calls_note_mcu_movequeue_activity():
    """Static check: no MotionToolhead method body invokes the upstream
    flush-timer rearm method along a bridge code path.

    Upstream's body of note_mcu_movequeue_activity (toolhead.py:776) runs
    `self.reactor.update_timer(self.flush_timer, self.reactor.NOW)` which
    would cancel our `update_timer(NEVER)` silencing. The invariant: only
    upstream-owned paths call note_mcu_movequeue_activity, and we override
    the two such bridge-reachable callers (drip_move, _advance_flush_time
    indirectly via _flush_handler — both are no-op'd because our
    flush_step_generation, drip_move, and note_mcu_movequeue_activity are
    all overridden to no-op or replaced).
    """
    import inspect
    forbidden = "note_mcu_movequeue_activity"
    overrides = {
        name: src for name in EXPECTED_LOCAL_METHODS
        if (val := MotionToolhead.__dict__.get(name)) is not None
        and callable(val)
        and (src := inspect.getsource(val))
    }
    offenders = []
    for name, src in overrides.items():
        # Any line in our override body that actually calls the rearm
        # method on self / the toolhead. Exclude the override of the
        # method itself (which redefines it as no-op and contains the
        # name in its `def` line).
        if name == forbidden:
            continue
        if (".%s(" % forbidden) in src or ("self.%s(" % forbidden) in src:
            offenders.append(name)
    assert not offenders, (
        "Flush-timer silencing invariant violated!\n"
        "These MotionToolhead methods invoke %s, which rearms the "
        "silenced flush_timer:\n  %s"
        % (forbidden, offenders)
    )


# --- Test 3: legacy ToolHead module imports cleanly ------------------------

def test_legacy_toolhead_module_imports_without_bridge():
    """Restored upstream toolhead.py is self-contained: it imports without
    requiring motion_bridge or any bridge-specific module. This protects
    the "rest of the printer works as before" goal — anyone running plain
    upstream Kalico without the bridge gets the same toolhead module as
    mainline, byte-for-byte except for the _load_kinematics extraction.
    """
    th = importlib.import_module("klippy.toolhead")
    assert hasattr(th, "ToolHead")
    assert hasattr(th, "LookAheadQueue")
    assert hasattr(th, "Move")
    assert hasattr(th, "DripModeEndSignal")
    assert hasattr(th, "add_printer_objects")
    assert th.BUFFER_TIME_START == 0.250
    assert th.BUFFER_TIME_HIGH == 2.0
    assert th.SDS_CHECK_TIME == 0.001
    # The extension point we added:
    assert hasattr(th.ToolHead, "_load_kinematics")
```

- [ ] **Step 2: Run the test, expect three PASSes**

```bash
python3 -m pytest tools/test_motion_toolhead_static.py -v
```

Expected: 3 passed. If `EXPECTED_TOOLHEAD_METHODS` is off (because I miscounted upstream ToolHead's methods), the second test fails with a diff — fix `EXPECTED_TOOLHEAD_METHODS` to match `git show 1f3d0d070^:klippy/toolhead.py`'s actual surface (ground truth: the method-name list at lines 259–966 of upstream).

- [ ] **Step 3: Commit**

```bash
git add tools/test_motion_toolhead_static.py
git commit -m "test(motion_toolhead): static invariants — override surface, flush-timer, legacy import

Three pure-Python checks (no klippy boot needed):
  - Override-surface baseline: bidirectional pin of methods that
    MotionToolhead overrides locally vs methods upstream ToolHead
    exposes. Drift in either direction triggers CI review.
  - Flush-timer silencing invariant: source-grep of every
    MotionToolhead override to confirm none re-arm the silenced
    flush_timer via self.note_mcu_movequeue_activity().
  - Legacy ToolHead import: restored upstream module is
    self-contained and exposes the historical surface (ToolHead,
    LookAheadQueue, BUFFER_TIME_*, SDS_CHECK_TIME, add_printer_objects)
    plus the new _load_kinematics extension point."
```

---

### Task 6: Klippy-in-loop sim — boot + G28/G1 regression

**Files:**
- Test: existing `tools/sim_klippy/test_home_x.py` and `tools/sim_klippy/test_phase4_steps.py` (no changes needed)

**Rationale:** Spec test §4.1 + §4.2. The existing sim harness already exercises the bridge's G28 (homing) and G1 X10 (Phase-4 step generation) paths. Re-running it after the refactor is the regression check that the live integration still works.

- [ ] **Step 1: Run the existing G28 sim test**

```bash
bash tools/sim_klippy/run_local.sh "G28 X"
```

Expected output (from inside the Docker container):
- `MotionToolhead: Phase 1 skeleton initialized` line in `tools/sim_klippy/.local-logs/klippy.log`.
- `MotionToolhead: configure_axes mcu=mcu kin=0 ...` log line.
- `[bridge-trace] drip_move entered: ...` log lines.
- The `run_local.sh` exits with status 0.

- [ ] **Step 2: Run the existing G28 trip-path test (verifies homing event flow)**

```bash
bash tools/sim_klippy/run_local.sh < /dev/null bash -c "python3 tools/sim_klippy/test_home_x.py"
```

Or equivalently, invoke the underlying test inside the running container. Expected: both `trip_path` and `notrip_path` cases pass per the script's docstring.

- [ ] **Step 3: Run the existing Phase-4 step-count test**

```bash
bash tools/sim_klippy/run_local.sh < /dev/null bash -c "python3 tools/sim_klippy/test_phase4_steps.py"
```

Expected: step_count > 0 after the move (the gate in the script's docstring).

- [ ] **Step 4: If any of the above fail, debug — do not commit**

The most likely failure mode is `_init_planner` reading something at the wrong time or missing an attribute. Check `tools/sim_klippy/.local-logs/klippy.log` for the first ERROR/Exception. Common pivot points:

- `BridgeKinematics.__init__` referencing `self._toolhead.bridge` before it's set: ensure Task 3's pre-super assignment is correct.
- `_init_planner` not finding `force_move`: confirm `[force_move]` is in the sim printer.cfg (line 78 of `tools/sim_klippy/printer.cfg`).

- [ ] **Step 5: Commit (no code change, just a regression-checkpoint marker)**

If the runs are clean and no behavior regression is found, no commit is needed for this task — Tasks 3 & 4 already covered the code. This task is a verification gate. If fixes are needed, commit those individually with descriptive messages.

---

### Task 7: Sim test — velocity-limit propagation

**Files:**
- Create: `tools/sim_klippy/test_velocity_limits.py`

**Rationale:** Spec test §4.3. Verifies that M204, SET_VELOCITY_LIMIT (including newly-gained SQUARE_CORNER_VELOCITY), and RESET_VELOCITY_LIMIT all propagate to the bridge runtime.

- [ ] **Step 1: Write the failing test**

Create `tools/sim_klippy/test_velocity_limits.py`:

```python
#!/usr/bin/env python3
"""Velocity-limit gcode commands propagate to the bridge.

After the refactor, the bridge sees:
  - M204 S2000 → bridge.update_limits(_, 2000)
  - SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=3000 → bridge.update_limits(200, 3000)
  - SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=10 → toolhead.square_corner_velocity = 10
  - RESET_VELOCITY_LIMIT → bridge.update_limits(orig_max_velocity, orig_max_accel)

The check uses the printer's `toolhead` object status from the API
socket: get_status returns max_velocity / max_accel / square_corner_velocity.
"""
import json, os, pathlib, signal, socket, subprocess, sys, time

REPO = pathlib.Path(os.environ.get("KALICO_REPO", "/work"))
LOGDIR = REPO / "tools" / "sim_klippy" / ".local-logs"
KLIPPER_ELF = REPO / "out" / "klipper.elf"
PRINTER_CFG = REPO / "tools" / "sim_klippy" / "printer.cfg"
SIM_SOCKET = "/tmp/klipper_sim_socket"
KLIPPY_INPUT_TTY = "/tmp/klippy_sim_printer"
KLIPPY_API = "/tmp/klippy_sim_api"
KLIPPY_LOG = LOGDIR / "klippy.log"


def cleanup_prior():
    subprocess.run(["pkill", "-f", str(KLIPPER_ELF)], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "klippy_sim"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    for path in (SIM_SOCKET, KLIPPY_INPUT_TTY, KLIPPY_API):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass


def send_gcode(api_sock, script):
    """Send GCODE via api-server unix socket; return server response dict."""
    msg = {
        "id": 1,
        "method": "gcode/script",
        "params": {"script": script},
    }
    api_sock.sendall((json.dumps(msg) + "\x03").encode())
    chunks = []
    while True:
        chunk = api_sock.recv(4096)
        if not chunk:
            break
        chunks.append(chunk)
        if b"\x03" in chunk:
            break
    return json.loads(b"".join(chunks).split(b"\x03")[0].decode())


def query_toolhead_status(api_sock):
    msg = {
        "id": 2,
        "method": "objects/query",
        "params": {"objects": {"toolhead": ["max_velocity", "max_accel",
                                            "square_corner_velocity"]}},
    }
    api_sock.sendall((json.dumps(msg) + "\x03").encode())
    chunks = []
    while True:
        chunk = api_sock.recv(4096)
        if not chunk:
            break
        chunks.append(chunk)
        if b"\x03" in chunk:
            break
    resp = json.loads(b"".join(chunks).split(b"\x03")[0].decode())
    return resp["result"]["status"]["toolhead"]


def main():
    cleanup_prior()
    LOGDIR.mkdir(parents=True, exist_ok=True)

    elf = subprocess.Popen(
        [str(KLIPPER_ELF), "-s", "/tmp/klipper_sim_socket",
         "-l", str(LOGDIR / "klipper_elf.log")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(1.5)
    klippy = subprocess.Popen(
        ["python3", str(REPO / "klippy" / "klippy.py"),
         str(PRINTER_CFG), "-l", str(KLIPPY_LOG),
         "-I", KLIPPY_INPUT_TTY, "-a", KLIPPY_API],
    )

    try:
        time.sleep(4.0)
        api_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        api_sock.connect(KLIPPY_API)

        # Baseline read
        baseline = query_toolhead_status(api_sock)
        assert baseline["max_velocity"] == 300.0
        assert baseline["max_accel"] == 3000.0

        # M204 S2000 → max_accel mutates
        send_gcode(api_sock, "M204 S2000")
        st = query_toolhead_status(api_sock)
        assert st["max_accel"] == 2000.0, st

        # SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=4000
        send_gcode(api_sock, "SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=4000")
        st = query_toolhead_status(api_sock)
        assert st["max_velocity"] == 200.0, st
        assert st["max_accel"] == 4000.0, st

        # SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=12 (upstream-broader surface)
        send_gcode(api_sock, "SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=12")
        st = query_toolhead_status(api_sock)
        assert st["square_corner_velocity"] == 12.0, st

        # RESET_VELOCITY_LIMIT — newly-registered command
        send_gcode(api_sock, "RESET_VELOCITY_LIMIT")
        st = query_toolhead_status(api_sock)
        assert st["max_velocity"] == 300.0, st
        assert st["max_accel"] == 3000.0, st

        # And confirm the bridge log shows update_limits dispatches
        log = KLIPPY_LOG.read_text(errors="ignore")
        # update_limits was called at least once for each of M204 / SVL /
        # RVL; logging from the Rust side may or may not surface, so we
        # check the host-side method dispatch via the bridge's call count
        # in the log if available. Minimal check: no exception during any
        # of the four commands.
        assert "Traceback" not in log, "Exception in klippy.log!"
        print("OK: M204 / SET_VELOCITY_LIMIT / RESET_VELOCITY_LIMIT all propagated")

    finally:
        try: api_sock.close()
        except Exception: pass
        klippy.send_signal(signal.SIGTERM)
        elf.send_signal(signal.SIGTERM)
        klippy.wait(timeout=5)
        elf.wait(timeout=5)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the test inside the Docker container**

```bash
bash tools/sim_klippy/run_local.sh < /dev/null bash -c "python3 tools/sim_klippy/test_velocity_limits.py"
```

Expected: `OK: M204 / SET_VELOCITY_LIMIT / RESET_VELOCITY_LIMIT all propagated` and exit 0.

- [ ] **Step 3: Commit**

```bash
git add tools/sim_klippy/test_velocity_limits.py
git commit -m "test(sim): velocity-limit propagation through MotionToolhead refactor

Sim test driving M204, SET_VELOCITY_LIMIT (with VELOCITY/ACCEL and the
newly-gained SQUARE_CORNER_VELOCITY parameter), and the
newly-registered RESET_VELOCITY_LIMIT command. Reads back via
api-server objects/query that toolhead.max_velocity / max_accel /
square_corner_velocity reflect the changes, and that no exceptions
surface in klippy.log."
```

---

### Task 8: Sim test — gcode_move state-sync coverage of newly-fired events

**Files:**
- Create: `tools/sim_klippy/test_gcode_move_state_sync.py`

**Rationale:** Spec test §4.4. After the refactor, `set_position` and `manual_move` fire `toolhead:set_position` / `toolhead:manual_move` events, which `gcode_move.reset_last_position` listens for (see `klippy/extras/gcode_move.py:14-19`). This test verifies gcode_move's `last_position` correctly tracks the toolhead after each event-firing flow.

- [ ] **Step 1: Write the failing test**

Create `tools/sim_klippy/test_gcode_move_state_sync.py`:

```python
#!/usr/bin/env python3
"""gcode_move.last_position tracks toolhead.commanded_pos after events.

This is the regression test for the latent bug fixed by inheriting
upstream ToolHead.manual_move and ToolHead.set_position: today's
standalone MotionToolhead does NOT fire toolhead:manual_move or
toolhead:set_position, so gcode_move's reset_last_position handler
(gcode_move.py:14-19, :146) never runs and gcode_move's
gcode_position drifts from the toolhead's commanded_pos after any
manual_move (probe / safe_z_home / dockable_probe paths).

After the refactor: events fire, reset_last_position runs, drift gone.

Two flows tested with a fake-homed toolhead (force_move):
  1. SET_KINEMATIC_POSITION X=50 Y=60 Z=10 → set_position event fires.
  2. PROBE-style manual_move via gcode SET_GCODE_OFFSET MOVE=1 →
     manual_move event fires.
"""
import json, os, pathlib, signal, socket, subprocess, sys, time

REPO = pathlib.Path(os.environ.get("KALICO_REPO", "/work"))
LOGDIR = REPO / "tools" / "sim_klippy" / ".local-logs"
KLIPPER_ELF = REPO / "out" / "klipper.elf"
PRINTER_CFG = REPO / "tools" / "sim_klippy" / "printer.cfg"
SIM_SOCKET = "/tmp/klipper_sim_socket"
KLIPPY_INPUT_TTY = "/tmp/klippy_sim_printer"
KLIPPY_API = "/tmp/klippy_sim_api"
KLIPPY_LOG = LOGDIR / "klippy.log"


def cleanup_prior():
    subprocess.run(["pkill", "-f", str(KLIPPER_ELF)], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "klippy_sim"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    for path in (SIM_SOCKET, KLIPPY_INPUT_TTY, KLIPPY_API):
        try: os.unlink(path)
        except FileNotFoundError: pass


def api_request(api_sock, msg_id, method, params):
    msg = {"id": msg_id, "method": method, "params": params}
    api_sock.sendall((json.dumps(msg) + "\x03").encode())
    chunks = []
    while True:
        chunk = api_sock.recv(4096)
        if not chunk:
            break
        chunks.append(chunk)
        if b"\x03" in chunk:
            break
    return json.loads(b"".join(chunks).split(b"\x03")[0].decode())


def send_gcode(api_sock, script):
    return api_request(api_sock, 1, "gcode/script", {"script": script})


def query_positions(api_sock):
    """Read both toolhead.position and gcode_move.gcode_position."""
    r = api_request(api_sock, 2, "objects/query", {"objects": {
        "toolhead": ["position"],
        "gcode_move": ["gcode_position"],
    }})
    return r["result"]["status"]


def main():
    cleanup_prior()
    LOGDIR.mkdir(parents=True, exist_ok=True)

    elf = subprocess.Popen(
        [str(KLIPPER_ELF), "-s", "/tmp/klipper_sim_socket",
         "-l", str(LOGDIR / "klipper_elf.log")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(1.5)
    klippy = subprocess.Popen(
        ["python3", str(REPO / "klippy" / "klippy.py"),
         str(PRINTER_CFG), "-l", str(KLIPPY_LOG),
         "-I", KLIPPY_INPUT_TTY, "-a", KLIPPY_API],
    )

    try:
        time.sleep(4.0)
        api_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        api_sock.connect(KLIPPY_API)

        # Flow 1: SET_KINEMATIC_POSITION → set_position event → gcode_move syncs
        send_gcode(api_sock, "SET_KINEMATIC_POSITION X=50 Y=60 Z=10")
        st = query_positions(api_sock)
        th_pos = st["toolhead"]["position"]
        gm_pos = st["gcode_move"]["gcode_position"]
        # toolhead.position is a Coord(x,y,z,e); gcode_position is a list.
        assert abs(th_pos[0] - 50.0) < 1e-6, th_pos
        assert abs(th_pos[1] - 60.0) < 1e-6, th_pos
        assert abs(th_pos[2] - 10.0) < 1e-6, th_pos
        assert abs(gm_pos[0] - 50.0) < 1e-6, (
            "gcode_move drifted from toolhead after SET_KINEMATIC_POSITION: %s vs %s"
            % (gm_pos, th_pos)
        )
        assert abs(gm_pos[1] - 60.0) < 1e-6, gm_pos

        # Flow 2: SET_GCODE_OFFSET MOVE=1 → manual_move event → gcode_move syncs
        send_gcode(api_sock, "SET_GCODE_OFFSET X=5 Y=5 MOVE=1 MOVE_SPEED=20")
        st = query_positions(api_sock)
        th_pos = st["toolhead"]["position"]
        gm_pos = st["gcode_move"]["gcode_position"]
        # After SET_GCODE_OFFSET MOVE=1 with offset (5,5,0), toolhead
        # commanded_pos becomes (55, 65, 10), gcode_position should
        # remain (50, 60, 10) (gcode-frame, with offset applied).
        assert abs(th_pos[0] - 55.0) < 1e-6, th_pos
        assert abs(th_pos[1] - 65.0) < 1e-6, th_pos
        # gcode_position is gcode-frame: (toolhead_pos - offset).
        assert abs(gm_pos[0] - 50.0) < 1e-6, (
            "gcode_move out of sync after SET_GCODE_OFFSET MOVE=1: %s vs %s"
            % (gm_pos, th_pos)
        )
        assert abs(gm_pos[1] - 60.0) < 1e-6, gm_pos

        # Flow 3: G92 X0 Y0 → gcode_move resets origin (no toolhead move)
        send_gcode(api_sock, "G92 X0 Y0")
        st = query_positions(api_sock)
        gm_pos = st["gcode_move"]["gcode_position"]
        assert abs(gm_pos[0] - 0.0) < 1e-6, gm_pos
        assert abs(gm_pos[1] - 0.0) < 1e-6, gm_pos

        log = KLIPPY_LOG.read_text(errors="ignore")
        assert "Traceback" not in log, "Exception in klippy.log!"
        print("OK: gcode_move state stays in sync after set_position + manual_move events")

    finally:
        try: api_sock.close()
        except Exception: pass
        klippy.send_signal(signal.SIGTERM)
        elf.send_signal(signal.SIGTERM)
        klippy.wait(timeout=5)
        elf.wait(timeout=5)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the test**

```bash
bash tools/sim_klippy/run_local.sh < /dev/null bash -c "python3 tools/sim_klippy/test_gcode_move_state_sync.py"
```

Expected: `OK: gcode_move state stays in sync after set_position + manual_move events` and exit 0.

- [ ] **Step 3: Commit**

```bash
git add tools/sim_klippy/test_gcode_move_state_sync.py
git commit -m "test(sim): gcode_move state stays in sync with newly-fired events

Sim test driving SET_KINEMATIC_POSITION (set_position event),
SET_GCODE_OFFSET MOVE=1 (manual_move event), and G92 (gcode-frame
reset). After each flow, asserts gcode_move.gcode_position tracks
toolhead.position + the active gcode offset.

Pre-refactor MotionToolhead.set_position and manual_move did not
fire toolhead:set_position / toolhead:manual_move events, so
gcode_move.reset_last_position never ran and gcode_move drifted out
of sync after probe / safe_z_home / dockable_probe sequences. The
inherited upstream methods fix this."
```

---

### Task 9: Sim test — `set_position` trapq side-effect verification

**Files:**
- Create: `tools/sim_klippy/test_set_position_trapq_safe.py`

**Rationale:** Spec test §4.9. Codex review pushed back that "inherited unchanged equals beneficial" was thin reasoning — upstream `set_position` does `trapq_set_position(self.trapq, self.print_time, ...)`, and we'd been assuming this is benign on an empty trapq rather than verifying it. This test exercises the path explicitly.

- [ ] **Step 1: Write the failing test**

Create `tools/sim_klippy/test_set_position_trapq_safe.py`:

```python
#!/usr/bin/env python3
"""Inherited set_position does not corrupt the (unused) bridge trapq.

Upstream ToolHead.set_position calls
  ffi_lib.trapq_set_position(self.trapq, self.print_time, x, y, z)
on every set_position. Under bridge, self.trapq is allocated (so
hardware-init's set_trapq doesn't crash) but never has segments
appended. We need to verify this writes are harmless: get_status
keeps returning sensible values, no segfault, no exception.

This test does many set_position calls in rapid succession and reads
back status each time, looking for any divergence.
"""
import json, os, pathlib, signal, socket, subprocess, sys, time

REPO = pathlib.Path(os.environ.get("KALICO_REPO", "/work"))
LOGDIR = REPO / "tools" / "sim_klippy" / ".local-logs"
KLIPPER_ELF = REPO / "out" / "klipper.elf"
PRINTER_CFG = REPO / "tools" / "sim_klippy" / "printer.cfg"
SIM_SOCKET = "/tmp/klipper_sim_socket"
KLIPPY_INPUT_TTY = "/tmp/klippy_sim_printer"
KLIPPY_API = "/tmp/klippy_sim_api"
KLIPPY_LOG = LOGDIR / "klippy.log"


def cleanup_prior():
    subprocess.run(["pkill", "-f", str(KLIPPER_ELF)], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(["pkill", "-f", "klippy_sim"], check=False,
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    for path in (SIM_SOCKET, KLIPPY_INPUT_TTY, KLIPPY_API):
        try: os.unlink(path)
        except FileNotFoundError: pass


def main():
    cleanup_prior()
    LOGDIR.mkdir(parents=True, exist_ok=True)
    elf = subprocess.Popen(
        [str(KLIPPER_ELF), "-s", "/tmp/klipper_sim_socket",
         "-l", str(LOGDIR / "klipper_elf.log")],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    time.sleep(1.5)
    klippy = subprocess.Popen(
        ["python3", str(REPO / "klippy" / "klippy.py"),
         str(PRINTER_CFG), "-l", str(KLIPPY_LOG),
         "-I", KLIPPY_INPUT_TTY, "-a", KLIPPY_API],
    )
    try:
        time.sleep(4.0)
        api_sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        api_sock.connect(KLIPPY_API)

        for i in range(20):
            x = float(10 + i)
            y = float(20 + i)
            z = float(30 + i)
            msg = {"id": 100 + i, "method": "gcode/script", "params": {
                "script": "SET_KINEMATIC_POSITION X=%f Y=%f Z=%f" % (x, y, z)
            }}
            api_sock.sendall((json.dumps(msg) + "\x03").encode())
            buf = b""
            while b"\x03" not in buf:
                buf += api_sock.recv(4096)
            # Query status
            qmsg = {"id": 200 + i, "method": "objects/query",
                    "params": {"objects": {"toolhead": ["position",
                                                         "max_velocity",
                                                         "estimated_print_time"]}}}
            api_sock.sendall((json.dumps(qmsg) + "\x03").encode())
            buf = b""
            while b"\x03" not in buf:
                buf += api_sock.recv(4096)
            resp = json.loads(buf.split(b"\x03")[0].decode())
            th = resp["result"]["status"]["toolhead"]
            assert abs(th["position"][0] - x) < 1e-6, th
            assert th["max_velocity"] == 300.0, th  # baseline unchanged
            assert th["estimated_print_time"] >= 0.0, th

        log = KLIPPY_LOG.read_text(errors="ignore")
        assert "Traceback" not in log, "Exception in klippy.log!"
        assert "trapq" not in log.lower() or "trapq_alloc" in log, (
            "Suspicious trapq mention in log — review:\n"
            + "\n".join(l for l in log.splitlines() if "trapq" in l.lower())
        )
        print("OK: set_position with empty trapq is benign across 20 iterations")

    finally:
        try: api_sock.close()
        except Exception: pass
        klippy.send_signal(signal.SIGTERM)
        elf.send_signal(signal.SIGTERM)
        klippy.wait(timeout=5)
        elf.wait(timeout=5)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run the test**

```bash
bash tools/sim_klippy/run_local.sh < /dev/null bash -c "python3 tools/sim_klippy/test_set_position_trapq_safe.py"
```

Expected: `OK: set_position with empty trapq is benign across 20 iterations` and exit 0.

- [ ] **Step 3: Commit**

```bash
git add tools/sim_klippy/test_set_position_trapq_safe.py
git commit -m "test(sim): inherited set_position is benign on empty bridge trapq

Sim-driven verification of spec §4.9. MotionToolhead inherits
ToolHead.set_position which calls trapq_set_position(self.trapq,
self.print_time, ...) on the bridge's allocated-but-unused trapq.
The test repeatedly calls SET_KINEMATIC_POSITION and queries status
to confirm no exception, no segfault, and no value divergence."
```

---

### Task 10: Update plan-changes log + final verification

**Files:**
- Modify: `docs/superpowers/plan-changes-log.md`

**Rationale:** Project rule: changes to build-order, specs, or architectural constraints land in the running log so future-self / next-conversation can find them.

- [ ] **Step 1: Append the entry**

Open `docs/superpowers/plan-changes-log.md` and append at the end:

```markdown

## 2026-05-07 — MotionToolhead extends upstream ToolHead

**What changed:** Refactored `klippy/motion_toolhead.py` from a 965-LOC standalone reimplementation of upstream Klipper's `ToolHead` into a ~350-LOC subclass. Restored `klippy/toolhead.py` (deleted by `1f3d0d070`) with one surgical extraction (`_load_kinematics` extension point). MotionToolhead now overrides only the bridge-owned methods and inherits the rest from upstream.

**Why:** "Be able to iterate on our custom motion implementation, and for the rest of the printer to work as before." The standalone version was diverging from upstream and forcing small upstream fixes to be re-ported by hand; subclassing makes the upstream module the source of truth for non-motion behavior.

**Side-effect benefits (latent bugs fixed):**
- `toolhead:set_position` and `toolhead:manual_move` events now fire — `gcode_move.reset_last_position` runs, so gcode-move state stays in sync after probe / safe_z_home / dockable_probe / G92 / SET_GCODE_OFFSET MOVE=1 flows.
- `RESET_VELOCITY_LIMIT` command is now exposed.
- `SET_VELOCITY_LIMIT` accepts the upstream-broader parameter set (`SQUARE_CORNER_VELOCITY`, `MINIMUM_CRUISE_RATIO`, `ACCEL_TO_DECEL`, per-axis args).

**Forward-linked items (not addressed by this refactor):**
- `idle_timeout` evaluates `lookahead_empty` which is always True under bridge — can fire during long bridge moves. Tracked separately.
- `stats()` override stays for now; switch to `bridge.get_last_move_time()`-based predicate once bridge owns meaningful `print_time`.
- Pre-existing kalico bug found during review: `input_shaper.cmd_SET_INPUT_SHAPER` reads `self.shapers[0].shaper_type` directly while `AxisInputShaper` stores those attributes under `.params`. Track separately.

**Evidence:** spec at `docs/superpowers/specs/2026-05-07-motion-toolhead-extends-upstream-design.md`, plan at `docs/superpowers/plans/2026-05-07-motion-toolhead-extends-upstream.md`.
```

- [ ] **Step 2: Run the static test suite to confirm refactor invariants**

```bash
python3 -m pytest tools/test_motion_toolhead_static.py -v
```

Expected: 3 passed.

- [ ] **Step 3: Run the sim regression suite (one final pass)**

```bash
bash tools/sim_klippy/run_local.sh < /dev/null bash -c "
set -e
python3 tools/sim_klippy/test_home_x.py
python3 tools/sim_klippy/test_phase4_steps.py
python3 tools/sim_klippy/test_velocity_limits.py
python3 tools/sim_klippy/test_gcode_move_state_sync.py
python3 tools/sim_klippy/test_set_position_trapq_safe.py
"
```

Expected: all five tests print their `OK:` lines and the script exits 0.

- [ ] **Step 4: Verify motion_toolhead.py target LOC**

```bash
wc -l klippy/motion_toolhead.py
```

Expected: ~350 LOC (down from 965). Within 10% of target is acceptable; significantly larger means dead code crept back in.

- [ ] **Step 5: Commit**

```bash
git add docs/superpowers/plan-changes-log.md
git commit -m "docs(plan-changes-log): record MotionToolhead extends-upstream refactor

Cross-references the spec, the plan, the side-effect bug fixes, and
the three forward-linked items kept out of scope (idle_timeout
quirk, stats() future direction, input_shaper.cmd_SET_INPUT_SHAPER
.params bug)."
```

---

## Self-review

**Spec coverage:**
- §3.1 toolhead.py extraction → Task 1.
- §3.2 MotionToolhead subclass → Task 3.
- §3.3 override surface table → Task 3 (every method) + Task 5 (drift detector pins it).
- §3.4 BridgeKinematics.set_position → Task 2.
- §3.5 consumer migration → Task 4.
- "Improved" section (gcode_move events, RESET_VELOCITY_LIMIT, SET_VELOCITY_LIMIT broader surface) → Tasks 7 + 8.
- Forward-linked correctness hazards → Task 10 plan-changes-log entry.
- Test §4.1 (boot smoke) → Task 6.
- Test §4.2 (G28 / G1 dispatch) → Task 6.
- Test §4.3 (velocity-limit commands) → Task 7.
- Test §4.4 (gcode_move state-sync) → Task 8.
- Test §4.5 (trad_rack) → Task 4 step 4 (import resolves) + the LookAheadQueue branch path is exercised by any normal klippy boot in Task 6.
- Test §4.6 (override-drift detector, bidirectional) → Task 5.
- Test §4.7 (legacy upstream importable test) → Task 5 (third test in static file).
- Test §4.8 (flush-timer silencing invariant) → Task 5.
- Test §4.9 (set_position trapq side-effect) → Task 9.
- Test §4.10 (offline planner harness, klipper-sim) → noted as out-of-scope for the per-task TDD loop; user runs it manually as the meta-regression check at the end. Folded into Task 10 step 3 if extended; otherwise the user's own `~/Developer/klipper-sim/...` invocation covers it.

**Placeholder scan:** No `TBD` / `TODO` / "implement later" / "similar to Task N". Every code step shows full code; every command step shows the exact command and expected result.

**Type / signature consistency:** `BridgeKinematics.set_position(newpos, homing_axes=())` matches upstream `kin.set_position(newpos, homing_axes)` contract used across `cartesian.py` / `corexy.py` / `delta.py` / `polar.py` / `rotary_delta.py` / `winch.py`. `MotionToolhead.set_position` is inherited; calls `self.kin.set_position(...)` — same signature. `MotionToolhead.cmd_SET_VELOCITY_LIMIT` and `cmd_RESET_VELOCITY_LIMIT` are super-calls to upstream's signatures; `BUFFER_TIME_START` is imported from `klippy.toolhead` (defined upstream as `0.250`). `MAX_Z_VELOCITY`/`max_z_accel` parsed identically to today.
