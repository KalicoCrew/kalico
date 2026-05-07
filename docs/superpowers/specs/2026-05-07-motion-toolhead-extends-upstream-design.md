# MotionToolhead extends upstream ToolHead — design

**Status:** Draft, awaiting user approval. Revised 2026-05-07 after multi-agent review (architect-reviewer, codebase auditor, codex independent reviewer).
**Date:** 2026-05-07.
**Scope:** Refactor of `klippy/motion_toolhead.py` and restoration of `klippy/toolhead.py`. No behavior change to the Rust planner, the bridge runtime, or the MCU firmware.

## Goal

Be able to iterate on the bridge-aware motion path (`move`, `drip_move`, `wait_moves`, `dwell`, `set_accel`, planner init, credit-freed routing, sim diagnostics) without re-implementing or maintaining the upstream `ToolHead` surface. Everything outside motion (status reporting, gcode-move state, idle timeout, helper-module loading, velocity-limit commands, RESET_VELOCITY_LIMIT, manual moves, shutdown handling) should behave the same as upstream Klipper / Kalico — no regressions, no bridge-mode quirks where upstream's behavior already matches what the user expects.

The deletion of `klippy/toolhead.py` in `1f3d0d070` made `motion_toolhead.py` the single source of truth for both the bridge implementation AND a hand-maintained reimplementation of upstream's `LookAheadQueue`, buffer-time constants, and `add_printer_objects`. That diverges over time and forces small upstream fixes to be re-ported. This refactor restores upstream toolhead as the reference and reduces `motion_toolhead.py` to "the parts where the bridge replaces upstream behavior".

## Non-goals

- No behavioral change to the Rust planner or bridge runtime.
- No change to `BridgeKinematics`'s rail-registration logic, homing flow, or itersolve allocation.
- No change to MCU firmware.
- No change to the MVP build-order (Step 7-D continues from this refactored baseline).
- No real-hardware testing introduced by this work — the surface change is verified via the existing klippy-in-loop Renode harness and offline planner harness.

## Architecture

```
klippy/toolhead.py                   restored from 1f3d0d070^ verbatim,
                                     with one surgical extraction (§3.1).

klippy/motion_toolhead.py            ~350 LOC. Contains:
                                       BridgeKinematics
                                       MotionToolhead(ToolHead)
                                       add_printer_objects()
                                     DELETED: LookAheadQueue stub,
                                       BUFFER_TIME_*, SDS_CHECK_TIME,
                                       LOOKAHEAD_FLUSH_TIME,
                                       `ToolHead = MotionToolhead` alias.

klippy/printer.py                    unchanged; loads motion_toolhead.

klippy/extras/trad_rack.py           import reverted to klippy.toolhead.
klippy/extras/probe.py               UNCHANGED. Uses `from __future__ import
                                     annotations`, so `ToolHead` annotations
                                     are strings — no runtime import needed.
klippy/extras/nozzle_cleanup.py      type-hint import switched to klippy.toolhead
                                     (runtime instance is still MotionToolhead
                                     via the printer registry; subclass relationship
                                     keeps the type alias correct).
```

Class hierarchy:

```
ToolHead (legacy, unchanged behavior; owns lookahead, flush_timer,
          priming_timer, trapq alloc, gcode-command registration,
          helper-module loading, _calc_junction_deviation, get_status,
          manual_move, _handle_shutdown, ...)
   └── MotionToolhead (bridge-aware; replaces motion-issuance and
                       time-accounting where the bridge owns them)
```

## Edits

### §3.1 `klippy/toolhead.py` — single extraction

In `ToolHead.__init__`, the kinematics import block becomes a method so subclasses can override it. Behavior-identical for legacy users.

```python
# Before:
kin_name = config.get("kinematics")
try:
    mod = importlib.import_module("klippy.kinematics." + kin_name)
    self.kin = mod.load_kinematics(self, config)
except config.error:
    raise
except self.printer.lookup_object("pins").error:
    raise
except:
    msg = "Error loading kinematics '%s'" % (kin_name,)
    logging.exception(msg)
    raise config.error(msg)

# After (in __init__):
self.kin = self._load_kinematics(config)

# New method on ToolHead:
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

Everything else in `klippy/toolhead.py` is byte-for-byte from `1f3d0d070^`.

### §3.2 `klippy/motion_toolhead.py` — subclass

`MotionToolhead` extends `ToolHead`. It overrides `_load_kinematics` to install `BridgeKinematics`, overrides only the methods the bridge owns, and adds bridge-only initialization after `super().__init__`.

```python
class MotionToolhead(ToolHead):
    def __init__(self, config):
        # Pre-super: attributes that BridgeKinematics or registered handlers
        # may reference during super init.
        self.bridge = config.get_printer().lookup_object("motion_bridge", None)
        self.active_homing_arms = set()
        self.kinematics_name = config.get("kinematics", "")

        # Run upstream init: trapq alloc, gcode commands (G4/M400/
        # SET_VELOCITY_LIMIT/RESET_VELOCITY_LIMIT/M204), helper modules
        # (gcode_move/homing/idle_timeout/statistics/manual_probe/
        # tuning_tower/garbage_collection), lookahead, flush_timer,
        # _calc_junction_deviation, _handle_shutdown registration,
        # extruder = DummyExtruder, AND _load_kinematics → BridgeKinematics.
        super().__init__(config)

        # Bridge owns the timeline; silence upstream's flush machinery so
        # the reactor doesn't tick a no-op handler forever and so the
        # inertness is explicit to anyone debugging.
        self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
        self.do_kick_flush_timer = False

        # Bridge-only config keys (not parsed by upstream ToolHead).
        self.max_z_velocity = config.getfloat(
            "max_z_velocity", self.max_velocity, above=0.0)
        self.max_z_accel = config.getfloat(
            "max_z_accel", self.max_accel, above=0.0)

        # Sim-only diagnostic gcode commands (only when bridge present).
        if self.bridge is not None:
            gcode = self.printer.lookup_object("gcode")
            gcode.register_command(
                "KALICO_SIM_STEP_COUNT", self.cmd_KALICO_SIM_STEP_COUNT,
                desc="[sim] Query cumulative step count for a stepper OID")
            gcode.register_command(
                "KALICO_SIM_AXIS_STEPS", self.cmd_KALICO_SIM_AXIS_STEPS,
                desc="[sim] Query configured steps_per_mm for an axis OID")
            gcode.register_command(
                "KALICO_SIM_AXIS_ACCUM", self.cmd_KALICO_SIM_AXIS_ACCUM,
                desc="[sim] Query step accumulator for an axis OID")
            gcode.register_command(
                "KALICO_SIM_ENDSTOP_SET_PIN",
                self.cmd_KALICO_SIM_ENDSTOP_SET_PIN,
                desc="[sim] Drive a Linux-MCU GPIO level (test fixture)")

        # Planner initialization runs once all MCUs have connected.
        self.printer.register_event_handler(
            "klippy:connect", self._init_planner)

        logging.info("MotionToolhead: Phase 1 skeleton initialized")

    def _load_kinematics(self, config):
        # Replace the legacy kinematics-module import path: bridge owns
        # motion, BridgeKinematics handles rail registration / itersolve
        # for hardware init / homing-rail mapping only.
        return BridgeKinematics(self, config, self.trapq)
```

### §3.3 Override surface

**Inherited unchanged from `ToolHead`** (verified equivalent or beneficial under bridge):

- `__init__` (we call `super().__init__(config)`).
- `get_position`, `set_position`, `set_extruder`, `get_extruder`, `get_kinematics`, `get_trapq`.
- `manual_move` — fires `toolhead:manual_move` event (today's bridge omits it; inheriting fixes a latent gcode_move state-drift bug). Inherited `manual_move` calls overridden `move()`, which still bypasses the lookahead queue — net effect: manual moves go straight to the bridge, identical to today's behavior plus the corrective event firing.
- `register_step_generator`, `register_lookahead_callback`, `note_step_generation_scan_time`.
- `get_max_velocity`, `_calc_junction_deviation`, `limit_next_junction_speed`.
- `check_busy`, `get_status`, `get_active_rails_for_axis`, `_handle_shutdown`.
- `cmd_G4`, `cmd_M400`, `cmd_M204` — dispatch to overridden `dwell`/`wait_moves`/`set_accel`.

**Overridden (bridge owns these):**

| Method | Behavior |
|---|---|
| `_load_kinematics(config)` | Returns `BridgeKinematics(self, config, self.trapq)`. |
| `move(newpos, speed)` | `bridge.submit_move(...)` + `_fire_active_callbacks(...)`. Updates `self.commanded_pos`. No lookahead. |
| `drip_move(newpos, speed, drip_completion)` | Bridge homing model: `bridge.submit_homing_move(pos3, speed, arm_ids)` + `wait_moves()`. Falls back to regular `move()` when no arms are armed (file-output / legacy path safety net). |
| `dwell(delay)` | `bridge.submit_dwell(delay)`. |
| `wait_moves()` | `bridge.wait_moves()`. |
| `flush_step_generation()` | No-op. Bridge owns flush. |
| `get_last_move_time()` | `max(bridge.get_last_move_time(), mcu.estimated_print_time + toolhead.BUFFER_TIME_START)`. Reuses upstream's `BUFFER_TIME_START = 0.250` constant (imported from the restored `toolhead` module) — defining a duplicate `BUFFER_LEAD` would defeat the source-of-truth restoration. The floor keeps legacy MCU commands (TMC, SPI, digital_out) issued before the first move from landing in the MCU's past. |
| `note_mcu_movequeue_activity(mq_time, set_step_gen_time=False)` | No-op. Bridge has its own queue. |
| `set_accel(accel)` | Mutate `self.max_accel`; `bridge.update_limits(...)`. |
| `reset_accel()` | `bridge.update_limits(self.max_velocity, self.max_accel)`. |
| `cmd_SET_VELOCITY_LIMIT(gcmd)` | `super().cmd_SET_VELOCITY_LIMIT(gcmd)`; if bridge present, `bridge.update_limits(...)`. |
| `cmd_RESET_VELOCITY_LIMIT(gcmd)` | `super().cmd_RESET_VELOCITY_LIMIT(gcmd)`; if bridge present, `bridge.update_limits(...)`. New under refactor — was missing from current bridge. |
| `stats(eventtime)` | Returns `(False, "print_time=%.3f buffer_time=0.000 print_stall=%d" % ...)`. Preserves current bridge silence (upstream's `is_active = buffer_time > -60` would intermittently fire under stuck-at-zero `print_time`). |

**Bridge-only methods added after super:**

- `_init_planner()` — `klippy:connect` handler; locates bridge MCUs, reads input_shaper params, calls `bridge.init_planner(...)`, runs `_configure_axes_per_mcu` and `_register_credit_freed_handlers`, marks single-MCU local sim as homed.
- `_configure_axes_per_mcu(bridge_mcus)` — sends `ConfigureAxes` per bridge MCU (kin_tag, present_mask, awd_mask, invert_mask, steps_per_mm).
- `_register_credit_freed_handlers(bridge_mcus)` — registers per-MCU `kalico_credit_freed` handler that dispatches into `bridge.on_credit_freed` and fires homing completions.
- `_fire_active_callbacks(dx, dy, dz, de)` — energizes motors before bridge moves (upstream cartesian/corexy do this via `itersolve_check_active`; bridge synthesizes steps in the runtime so we fire the callbacks ourselves).
- `cmd_KALICO_SIM_STEP_COUNT`, `cmd_KALICO_SIM_AXIS_STEPS`, `cmd_KALICO_SIM_AXIS_ACCUM`, `cmd_KALICO_SIM_ENDSTOP_SET_PIN` — sim diagnostics.
- Instance attribute: `active_homing_arms: set` — printer-side registry written by `BridgeTriggerDispatch.start/stop`, read by `drip_move`.

**Removed (no callers — confirmed by grep):**

- `motor_off()` — all four `motor_off()` call sites in the codebase target `stepper_enable.motor_off()`, none target `toolhead.motor_off()`.
- `register_move_handler(handler)` — no callers.

### §3.4 `BridgeKinematics` — `set_position` replaces `set_homed`

Today's `BridgeKinematics.set_homed(axes)` is called from `MotionToolhead.set_position`. To let `MotionToolhead` inherit upstream `set_position` (which calls `self.kin.set_position(newpos, homing_axes)` per the kinematics contract), move the bridge-side position update INTO `BridgeKinematics`. Capture the toolhead reference in `__init__` so we don't go through the printer registry on every call:

```python
class BridgeKinematics:
    def __init__(self, toolhead, config, trapq):
        self._toolhead = toolhead     # already passed; previously discarded
        # ... rest of existing __init__ ...

    def set_position(self, newpos, homing_axes=()):
        # Upstream contract: kinematics owns runtime position-state sync.
        # For cartesian, this drives itersolve. For us, this drives the
        # bridge runtime's planner basis.
        if self._toolhead.bridge is not None:
            self._toolhead.bridge.set_position(newpos[0], newpos[1], newpos[2])
        for a in homing_axes:
            self.homed_axes.add(a)
```

Why direct toolhead reference rather than `self._printer.lookup_object("motion_bridge", None)`: the toolhead already holds the bridge handle (set pre-super in `MotionToolhead.__init__`), so the kinematics doesn't need a backward registry lookup. Removes the silent-`None` failure mode.

`set_homed` is dropped — its sole caller (`MotionToolhead.set_position`) is going away.

`clear_homing_state(axes)` stays (called from `_handle_motor_off`).

### §3.5 Consumer migration

```python
# klippy/extras/trad_rack.py
- from .. import chelper, motion_toolhead as toolhead
+ from .. import chelper, toolhead

# klippy/extras/probe.py — UNCHANGED
# Uses `from __future__ import annotations` (line 6); ToolHead annotations
# on lines 516, 600 are strings, no runtime import needed.

# klippy/extras/nozzle_cleanup.py
- from klippy.motion_toolhead import MotionToolhead as ToolHead
+ from klippy.toolhead import ToolHead

# klippy/printer.py — UNCHANGED
# Verified: printer.py:336 iterates only `[motion_toolhead]`, NOT both
# toolhead and motion_toolhead. So `motion_toolhead.add_printer_objects`
# is the single registrar of the "toolhead" object — no collision with
# the restored `toolhead.add_printer_objects`.
```

**`trad_rack` compatibility note:** `TradRackToolHead.__init__` (line 2316) has a `hasattr(toolhead, "LookAheadQueue")` branch. Restored upstream toolhead exposes `LookAheadQueue`, so the live branch is taken and the `else: toolhead.MoveQueue(...)` path remains dead code (it predates Klipper's MoveQueue → LookAheadQueue rename and is irrelevant to our environment).

`MotionToolhead` is-a `ToolHead`, so existing `isinstance(x, ToolHead)` checks and type hints remain correct. The runtime instance registered as the `"toolhead"` printer object is still `MotionToolhead`, installed by `motion_toolhead.add_printer_objects(config)`.

## Behavioral comparison

### Same as today's bridge

- All bridge-owned move issuance paths.
- BridgeKinematics rail/itersolve setup.
- Planner init, ConfigureAxes per-MCU, credit-freed handler registration.
- Sim-only diagnostic commands.
- `active_homing_arms` registry semantics for `drip_move`.
- `stats()` silence policy.
- `get_last_move_time()` floor-with-BUFFER_LEAD policy.

### Improved (latent bugs fixed by inheriting upstream)

- `manual_move` now fires `toolhead:manual_move` event → `gcode_move.reset_last_position` runs → no more state drift on probe / safe-z-home / dockable-probe paths.
- `set_position` now fires `toolhead:set_position` event → `gcode_move.reset_last_position` runs → no more state drift on G28 / SET_KINEMATIC_POSITION.
- `RESET_VELOCITY_LIMIT` is registered (was previously absent under bridge) and propagates to the bridge runtime via the override.
- `cmd_SET_VELOCITY_LIMIT` now respects upstream's full surface (e.g., `MINIMUM_CRUISE_RATIO`, `SQUARE_CORNER_VELOCITY` parameters) — previously the bridge's reimplementation handled only `VELOCITY` and `ACCEL`.

### Same as upstream Klipper / Kalico

- All non-motion gcode commands (G4, M400, M204, RESET_VELOCITY_LIMIT, SET_VELOCITY_LIMIT body).
- Helper-module loading (gcode_move, homing, idle_timeout, statistics, manual_probe, tuning_tower, garbage_collection).
- `_handle_shutdown` semantics.
- `get_status` field set.
- `_calc_junction_deviation`, `get_max_velocity`, `note_step_generation_scan_time`.

### Forward-linked correctness hazards (NOT a behavioral change introduced by this refactor — flagged for separate followup)

- **`idle_timeout` firing during long bridge moves.** Upstream's `check_busy` returns `lookahead_empty = not self.lookahead.queue`. Under bridge, lookahead is always empty, so `idle_timeout` evaluates the printer as idle even while the bridge is mid-segment. This is pre-existing today and unchanged by this refactor, but it is a real correctness hazard (motors can de-energize during a long curve). Tracked separately.
- **`stats()` future direction.** This refactor keeps the bridge's silence-the-stats override because upstream's `is_active = buffer_time > -60` predicate is fed by `print_time = 0` under bridge — wrong inputs, wrong answer. Once the bridge owns a meaningful `print_time` (post-MVP), the override should change to use `bridge.get_last_move_time()` in the predicate rather than inherit upstream verbatim. Captured here so the override doesn't get adopted permanently.

### Pre-existing limitations (unchanged by this refactor — known and accepted)

- `BridgeKinematics.calc_position` returns `[0,0,0]` stub; bridge homing relies on the trip-snapshot, not stepper-count → kinematics inversion. Affects `GET_POSITION` kin reporting and any consumer that reads stepper-count-derived position.
- Trapq is allocated but never has moves appended → trapq-history queries return empty. Inherited `set_position` calls `trapq_set_position(self.trapq, self.print_time, ...)`; with an empty trapq this is a metadata write only (no segment list to invalidate). Verified harmless via Test §4.9.
- Step generators registered via `register_step_generator` are never invoked → bridge runtime synthesizes steps directly; itersolve generators are dormant.

### Pre-existing kalico bug found during review (out of scope, separate ticket)

- **`input_shaper.cmd_SET_INPUT_SHAPER`** reads `self.shapers[0].shaper_type` and `self.shapers[0].shaper_freq` directly, but `AxisInputShaper` stores those attributes under `.params` (`input_shaper.py:79-82`). This was found during this spec's review but is unrelated to the toolhead refactor. Track separately.

## Test plan

1. **Static / boot smoke**
   - `python -c "from klippy import motion_toolhead, toolhead"` succeeds.
   - `MotionToolhead.__mro__` contains `ToolHead`.
   - klippy-in-loop boot reaches "MotionToolhead: Phase 1 skeleton initialized".

2. **Live G28 / G1 dispatch (existing klippy-in-loop Renode harness)**
   - `G28` → `BridgeTriggerDispatch.start` populates `active_homing_arms` → `drip_move` reads the set, calls `bridge.submit_homing_move(pos3, speed, arm_ids)`.
   - `G1 X10` → `move()` calls `bridge.submit_move(...)` and fires active-callbacks; commanded_pos updates.
   - `G1 Z5` → same dispatch, Z axis present_mask honored.
   - Renode tick_counter remains in spec post-refactor.

3. **Velocity-limit commands**
   - `M204 S2000` → `max_accel` updates AND `bridge.update_limits` called.
   - `SET_VELOCITY_LIMIT VELOCITY=200 ACCEL=3000` → both update AND propagate.
   - `SET_VELOCITY_LIMIT SQUARE_CORNER_VELOCITY=10` → upstream parameter handled (verifies the broader-surface gain).
   - `RESET_VELOCITY_LIMIT` → newly-registered command resets to `orig_cfg` AND propagates.

4. **gcode_move state sync — explicit behavioral coverage of the new event firing**

   Each of the following must produce a gcode_move state where `last_position` matches `toolhead.commanded_pos` after the move completes (today's bridge fails some of these silently):
   - `G28` then `G92 X0 Y0`
   - `SET_GCODE_OFFSET MOVE=1 X=5 Y=5`
   - `PROBE` (drives `manual_move` for nozzle approach)
   - `safe_z_home` macro path (drives `manual_move` for z-hop and home XY)
   - manual_probe reaching its final adjustment (`manual_move` z-bob path)
   - `dockable_probe` attach/detach sequence (multiple `manual_move` calls in series)

5. **`trad_rack` import path**
   - `import klippy.extras.trad_rack` succeeds. `TradRackToolHead.__init__` resolves `toolhead.LookAheadQueue`, `toolhead.BUFFER_TIME_HIGH`, `toolhead.SDS_CHECK_TIME` against the restored upstream module.
   - The `hasattr(toolhead, "LookAheadQueue")` branch is taken; the `else: toolhead.MoveQueue(...)` branch remains dead.

6. **Override-surface drift detector — bidirectional**
   - Asserts `{m for m in MotionToolhead.__dict__ if not m.startswith("__")}` matches a frozen baseline. Catches accidental new overrides.
   - Asserts `{m for m in ToolHead.__dict__ if not m.startswith("__")}` matches a frozen baseline. Catches NEW upstream additions that the refactor's override list might need to consider — failure means human review, not necessarily a code change.

7. **Legacy upstream path doesn't regress (real test, not "mechanically reviewed")**
   - Unit test imports `klippy.toolhead` and instantiates `ToolHead(config)` against a minimal printer config without `[motion_bridge]`. Verifies upstream init runs to completion (kinematics module resolves, gcode commands register, helper modules load). The trad_rack subclass test (§4.5) covers part of this surface; this is the explicit case.

8. **Flush-timer silencing invariant**
   - Test asserts no MotionToolhead method (overridden or inherited) calls `note_mcu_movequeue_activity` along a bridge path; if it did, upstream's body would re-arm `flush_timer` despite our `update_timer(NEVER)` call. Static check via override-list closure: the only methods that call `note_mcu_movequeue_activity` upstream are `_advance_flush_time` and `drip_move`, both of which are no-op'd or overridden under bridge.

9. **`set_position` trapq side-effect under bridge (verify the inherited path is benign)**
   - Direct test: instantiate the bridge, call `toolhead.set_position([1, 2, 3, 0], homing_axes=[0,1,2])`. Verify (a) `bridge.set_position(1, 2, 3)` was called, (b) `homed_axes` set updated, (c) `commanded_pos` updated, (d) `toolhead:set_position` event fired and `gcode_move.reset_last_position` ran, (e) the inherited `trapq_set_position(self.trapq, self.print_time=0, ...)` did not corrupt anything (subsequent `get_status` returns sensible values, no segfault, no exception).

10. **Offline planner harness (klipper-sim)**
    - `~/Developer/klipper-sim/...` runs the planner against a representative G-code file using `--klipper-root` pointed at this branch. Output diff vs pre-refactor branch should be byte-identical (no planner-level behavior change).

## Risks and mitigations

- **Risk:** A consumer relies on `toolhead.motor_off()` or `toolhead.register_move_handler(...)`. Mitigation: grep confirms zero callers; if a caller materializes after merge, restore as a no-op.
- **Risk:** `manual_move` event firing changes timing for some probe sequence. Mitigation: gcode_move's listener is fast and idempotent; if a regression appears, the event firing can be conditionally suppressed in the override.
- **Risk:** `super().__init__` fires upstream's `_handle_shutdown` registration AND any klippy:connect handlers in helper modules in a different order than today's MotionToolhead. Mitigation: registration order is preserved (helper modules load in identical order); event-handler dispatch is FIFO so behavior is stable.
- **Risk:** A future upstream sync brings a new `ToolHead` change that BridgeKinematics or `MotionToolhead` overrides incompatibly. Mitigation: the override-surface drift test (§4.6) and a code comment on each override pointing back to this spec.

## Rollback

Revert the refactor commit. Files restored: `klippy/toolhead.py` re-deleted, `klippy/motion_toolhead.py` reverted to current state, consumer imports reverted. No data-format or wire-protocol changes; rollback is mechanical.
