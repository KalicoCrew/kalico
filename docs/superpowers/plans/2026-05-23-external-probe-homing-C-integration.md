# Plan C: Integration — Python Credit Loop + Position Dispatch

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Depends on Plan A + Plan B.** Both must be merged to the working branch before starting this plan. Plan A provides the trsync re-enable; Plan B provides the firmware commands, bridge FFI, and curve evaluation.

**Goal:** Wire the software-trip credit loop into `drip_move`, add Python bridge wrappers, and implement `get_past_mcu_position` dispatch for trigger-time accuracy. After this plan, G28 Z with Beacon works end-to-end.

**Architecture:** Three layers: (1) Python wrappers for the bridge FFI surface from Plan B, (2) the credit-extension loop in `drip_move` with virtual arm creation, (3) `get_past_mcu_position` dispatch between curve evaluation (software-trip) and MCU snapshot (GPIO trip).

**Tech Stack:** Python (klippy)

**Spec:** `docs/kalico-rewrite/external-probe-homing.md`, Pieces B+C host-side

---

## File Map

| File | Change |
|------|--------|
| `klippy/motion_bridge.py` | `BridgeHomingReason` constants, `SOURCE_KIND_SOFTWARE`, wrapper methods for `software_trip`/`extend_homing_deadline`/`submit_homing_move_async`/`get_homing_position_at_time` |
| `klippy/motion_toolhead.py` | `_drip_move_software_trip` method, three-way dispatch in `drip_move`, `_software_trip_active` flag |
| `klippy/motion_kinematics.py` | Read-only (used for stepper selection via `motor_deltas`) |
| `klippy/stepper.py` | `get_past_mcu_position` conditional dispatch |

---

### Task 1: Python bridge wrappers + constants

**Files:**
- Modify: `klippy/motion_bridge.py`

- [ ] **Step 1:** Add bridge-private reason constants (NOT in MCU_trsync namespace). Place near the existing `REASON_ENDSTOP_HIT` / `ARM_STATUS_*` constants:
```python
# Bridge-private homing segment reasons. These live in a separate
# namespace from MCU_trsync reasons (which use values 1-4 and probes
# use 5+ for sensor-specific errors).
BRIDGE_REASON_PAST_END_TIME = 1
BRIDGE_REASON_TRIPPED = 2
BRIDGE_REASON_DEADLINE_EXPIRED = 3
```

- [ ] **Step 2:** Add source kind constant:
```python
SOURCE_KIND_SOFTWARE = 2
```

- [ ] **Step 3:** Add wrapper methods on `MotionBridgeWrapper` class, near the existing `submit_homing_move` / `endstop_arm` / `endstop_disarm` wrappers:
```python
def submit_homing_move_async(self, newpos, speed, arm_ids):
    return self._bridge.submit_homing_move_async(newpos, speed, arm_ids)

def is_homing_segment_retired(self):
    return self._bridge.is_homing_segment_retired()

def get_homing_segment_reason(self):
    return self._bridge.get_homing_segment_reason()

def software_trip(self, arm_id):
    return self._bridge.software_trip(arm_id)

def extend_homing_deadline(self, arm_id):
    return self._bridge.extend_homing_deadline(arm_id)

def get_homing_position_at_time(self, print_time):
    return self._bridge.get_homing_position_at_time(print_time)
```

- [ ] **Step 4:** Add `_software_trip_active` flag on the wrapper, default `False`:
```python
self._software_trip_active = False
```

- [ ] **Step 5:** Commit: `feat: Python bridge wrappers for software-trip homing`

---

### Task 2: drip_move three-way dispatch

**Files:**
- Modify: `klippy/motion_toolhead.py` — `drip_move` method

- [ ] **Step 1:** In `drip_move` (motion_toolhead.py:432), restructure the existing body into a three-way dispatch. The existing GPIO path (lines 456-481) becomes the `if arm_ids:` branch. Add the new external-probe branch. The no-endstop fallback (lines 451-454) stays as `else`:

```python
def drip_move(self, newpos, speed, drip_completion):
    logging.info(
        "[bridge-trace] drip_move entered: newpos=%s speed=%s "
        "drip_test=%s active_homing_arms=%s",
        list(newpos), speed,
        (drip_completion.test()
         if drip_completion is not None else None),
        sorted(self.active_homing_arms),
    )
    if drip_completion is not None and drip_completion.test():
        return
    arm_ids = list(self.active_homing_arms)
    if arm_ids:
        # Bridge-native GPIO/sensorless path (existing, unchanged)
        pos3 = list(newpos[:3]) + [0.0] * max(0, 3 - len(newpos[:3]))
        dx = pos3[0] - self.commanded_pos[0]
        dy = pos3[1] - self.commanded_pos[1]
        dz = pos3[2] - self.commanded_pos[2]
        self._fire_active_callbacks(
            dx, dy, dz, 0.0, self.get_last_move_time()
        )
        self.bridge._software_trip_active = False
        bridge_lmt_before = self.bridge.get_last_move_time()
        self.bridge.submit_homing_move(pos3, speed, arm_ids)
        self.bridge.wait_moves()
        bridge_lmt_after = self.bridge.get_last_move_time()
        duration = bridge_lmt_after - bridge_lmt_before
        self._bump_pending_end_time(duration)
    elif drip_completion is not None and not drip_completion.test():
        # External probe software-trip path
        self._drip_move_software_trip(newpos, speed, drip_completion)
    else:
        # No endstop armed — regular move fallback
        self.move(newpos, speed)
```

Note: the existing GPIO path now explicitly sets `bridge._software_trip_active = False` to ensure the flag is cleared if a prior software-trip homing left it set.

- [ ] **Step 2:** Commit: `refactor: drip_move three-way dispatch (GPIO / software-trip / fallback)`

---

### Task 3: _drip_move_software_trip implementation

**Files:**
- Modify: `klippy/motion_toolhead.py`

This is the core implementation — the credit-extension loop with virtual arm.

- [ ] **Step 1:** Add the method. Place it right after `drip_move`:

```python
def _drip_move_software_trip(self, newpos, speed, drip_completion):
    from . import motion_bridge as _mb
    from . import motion_kinematics

    pos3 = list(newpos[:3]) + [0.0] * max(0, 3 - len(newpos[:3]))
    dx = pos3[0] - self.commanded_pos[0]
    dy = pos3[1] - self.commanded_pos[1]
    dz = pos3[2] - self.commanded_pos[2]

    # Select moving steppers via kinematic motor-delta mapping
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

    # Resolve MCU handle + queue from first stepper's MCU
    stepper_mcus = set(s.get_mcu() for s in moving_steppers)
    if len(stepper_mcus) > 1:
        raise self.printer.command_error(
            "External probe homing across multiple bridge MCUs "
            "is not supported"
        )
    stepper_mcu = next(iter(stepper_mcus))
    mcu_handle = stepper_mcu._bridge_handle
    queue = self.bridge.alloc_command_queue(mcu_handle)

    # Create virtual arm
    arm_id = _mb._alloc_arm_id()
    stepper_oids = [s.get_oid() for s in moving_steppers]
    source = (_mb.SOURCE_KIND_SOFTWARE, 0, 0, 0, 1, 0, 0)
    arm_clock = int(stepper_mcu.print_time_to_clock(
        self.get_last_move_time()
    ))

    # Energize motors
    self._fire_active_callbacks(
        dx, dy, dz, 0.0, self.get_last_move_time()
    )

    # Arm + submit
    self.active_homing_arms.add(arm_id)
    self.bridge.register_homing_dispatch(arm_id, None)
    self.bridge._software_trip_active = True

    bridge_lmt_before = self.bridge.get_last_move_time()
    try:
        self.bridge.endstop_arm(
            mcu_handle, queue, arm_id, arm_clock,
            [source], stepper_oids,
        )
        self.bridge.submit_homing_move_async(pos3, speed, [arm_id])

        # Credit-extension loop
        while True:
            drip_completion.wait(
                waketime=self.reactor.monotonic() + 0.025
            )
            if drip_completion.test():
                self.bridge.software_trip(arm_id)
                break
            if self.bridge.is_homing_segment_retired():
                reason = self.bridge.get_homing_segment_reason()
                if reason == _mb.BRIDGE_REASON_DEADLINE_EXPIRED:
                    raise self.printer.command_error(
                        "Homing deadline expired: host failed to "
                        "extend credit within 50ms"
                    )
                break  # natural no-trigger
            self.bridge.extend_homing_deadline(arm_id)

        self.bridge.wait_moves()
        bridge_lmt_after = self.bridge.get_last_move_time()
        duration = bridge_lmt_after - bridge_lmt_before
        self._bump_pending_end_time(duration)
    finally:
        self.active_homing_arms.discard(arm_id)
        self.bridge.unregister_homing_dispatch(arm_id)
        try:
            self.bridge.endstop_disarm(mcu_handle, queue, arm_id)
        except Exception:
            pass  # best-effort cleanup
```

- [ ] **Step 2:** Verify the `_alloc_arm_id` import works — it's a module-level function in `motion_bridge.py`.

- [ ] **Step 3:** Commit: `feat: drip_move software-trip credit loop for external probes`

---

### Task 4: get_past_mcu_position dispatch

**Files:**
- Modify: `klippy/stepper.py:183`
- Modify: `klippy/motion_toolhead.py` — clear flag on `set_position`

- [ ] **Step 1:** In `stepper.py`, modify `get_past_mcu_position` (line 183):

```python
def get_past_mcu_position(self, print_time):
    bridge = getattr(self._mcu, '_motion_bridge', None)
    if bridge is not None and getattr(bridge, '_software_trip_active', False):
        try:
            pos_xyz = bridge.get_homing_position_at_time(print_time)
        except Exception:
            # Fallback if no retained curve or eval fails
            return getattr(self, "_bridge_last_trip_step_count",
                           self.get_mcu_position())
        # Apply kinematics + stepper conversion
        motor_pos = self._calc_motor_position_from_xyz(pos_xyz)
        mcu_pos_dist = motor_pos + self._mcu_position_offset
        mcu_pos = mcu_pos_dist / self._step_dist
        if mcu_pos >= 0.0:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)
    # GPIO bridge homing: MCU snapshot is authoritative
    return getattr(self, "_bridge_last_trip_step_count",
                   self.get_mcu_position())
```

- [ ] **Step 2:** Add `_calc_motor_position_from_xyz` helper on `MCU_stepper`. This applies the per-stepper kinematic transform. The stepper knows its axis from `setup_itersolve` (the `axis` parameter passed to `cartesian_stepper_alloc`).

For simple axes, the axis byte determines the index:
```python
def _calc_motor_position_from_xyz(self, pos_xyz):
    # The stepper's axis is set during setup_itersolve.
    # For cartesian: axis b'x' → pos[0], b'y' → pos[1], b'z' → pos[2]
    # For corexy: stepper_x is motor A (not axis X), but
    # setup_itersolve('cartesian_stepper_alloc', b'x') means this
    # stepper tracks the X axis in cartesian space.
    #
    # However, for CoreXY the bridge already evaluates the curve in
    # toolhead space [X, Y, Z]. The kinematics forward transform
    # (A=X+Y, B=X-Y) must be applied here.
    #
    # Use the bridge's kinematics type to decide:
    bridge = self._mcu._motion_bridge
    kin = getattr(bridge, '_kinematics_name', 'cartesian')
    axis = getattr(self, '_itersolve_axis', b'x')
    if kin == 'corexy':
        if axis == b'x':  # motor A = X + Y
            return pos_xyz[0] + pos_xyz[1]
        elif axis == b'y':  # motor B = X - Y
            return pos_xyz[0] - pos_xyz[1]
        else:  # Z, E — direct
            idx = {b'z': 2}.get(axis, 2)
            return pos_xyz[idx]
    else:  # cartesian
        idx = {b'x': 0, b'y': 1, b'z': 2}.get(axis, 0)
        return pos_xyz[idx]
```

Note: `_itersolve_axis` needs to be stored during `setup_itersolve`. Check if it's already stored; if not, save the axis parameter.

- [ ] **Step 3:** In `setup_itersolve` (stepper.py), store the axis parameter:
```python
def setup_itersolve(self, alloc_type, *params):
    # Bridge: position tracking lives in Rust.
    if params:
        self._itersolve_axis = params[0]
    return
```
Check if this attribute is already stored somewhere. If `setup_itersolve` already saves enough info, skip this step.

- [ ] **Step 4:** Clear `_software_trip_active` on planner reset. In `MotionToolhead.set_position` (motion_toolhead.py), add:
```python
self.bridge._software_trip_active = False
```
This ensures stale software-trip state doesn't leak across homing sequences.

- [ ] **Step 5:** Commit: `feat: get_past_mcu_position dispatch for software-trip trigger-time accuracy`

---

### Task 5: End-to-end testing

- [ ] **Step 1:** Flash firmware (Plan B), restart klippy. Verify sensorless X/Y homing still works:
```
G28 X
G28 Y
```
Expected: homes normally, no regressions.

- [ ] **Step 2:** Attempt Z homing with Beacon:
```
G28 Z
```
Expected: nozzle moves down, Beacon triggers, motion stops, Z position is set.
Check klippy.log for `[bridge-trace]` messages showing:
- `drip_move entered` with empty `active_homing_arms`
- Software-trip path entered
- Credit extension messages
- `software_trip` sent
- Successful position report

- [ ] **Step 3:** Full homing sequence:
```
G28
```
Expected: X homes (sensorless), Y homes (sensorless), Z homes (Beacon). All three axes homed.

- [ ] **Step 4:** Repeated Z homing (tests second-pass completion reset):
```
G28 Z
G28 Z
G28 Z
```
Expected: all three succeed. No "Endstop still triggered after retract" errors.

- [ ] **Step 5:** Test no-trigger case. Move nozzle well above Beacon range (if safe), attempt:
```
G28 Z
```
Expected: nozzle travels full distance, "No trigger on z after full movement" error. No bed crash (credit deadline prevents unlimited travel if host issues arise).

- [ ] **Step 6:** Verify trigger position accuracy. After G28 Z, check reported Z position:
```
GET_POSITION
```
Compare against Beacon's post-homing measurement. For eddy-current mode, Beacon overrides the position — values should be consistent. For contact mode, the curve-evaluated trigger position should be within step resolution of the actual contact point.

- [ ] **Step 7:** Commit any test-driven fixes.

---

## Verification Checklist

- [ ] G28 X (sensorless) — works, unchanged
- [ ] G28 Y (sensorless) — works, unchanged
- [ ] G28 Z (Beacon eddy-current) — works, nozzle stops on trigger
- [ ] G28 (full) — all three axes home
- [ ] Repeated G28 Z — no stale state between runs
- [ ] No-trigger G28 Z — error message, no crash or bed damage
- [ ] Position accuracy — GET_POSITION shows correct Z after homing
- [ ] klippy.log — no Python exceptions, no MCU communication errors
- [ ] Credit deadline — visible in klippy.log as periodic extension messages
