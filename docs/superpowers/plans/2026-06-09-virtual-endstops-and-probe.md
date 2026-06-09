# Virtual Endstops + Bridge-Native [probe] Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Generic virtual-endstop support in the new `homing.py` plus a bridge-native rewrite of `klippy/extras/probe.py`, so the Neptune boots and homes Z via `probe:z_virtual_endstop`.

**Architecture:** Host-side Python only — the firmware (`src/endstop.c`) and bridge (`bridge.rs`) already treat `endstop_id` as an opaque token. A shared `BridgeEndstop` class (extracted from `homing.py`) wraps the `config_endstop` firmware object; `homing.py` resolves `endstop_pin` through the pins registry and accepts "provider" chips that hand back a `BridgeEndstop`; the rewritten `probe.py` is the first provider and implements `PROBE`/`QUERY_PROBE`/`PROBE_ACCURACY` on `Homing.trip_move`, the extracted descend-until-trip primitive.

**Tech Stack:** Python (klippy), pytest (`uv run python -m pytest`), kalico-sim for integration, Neptune bench for final validation.

**Spec:** `docs/kalico-rewrite/virtual-endstops-and-probe.md` — read it first.

**Conventions that apply to every task:**
- Comments are a failure of expression (project CLAUDE.md). The code blocks below contain none; do not add any.
- Fail loudly: every unexpected state is a hard error, never a fallback.
- Run tests with `uv run python -m pytest <path> -q` from the repo root.
- Commit messages: no `Co-Authored-By` trailers of any kind.

---

### Task 1: Shared `BridgeEndstop` + provider id allocator

**Files:**
- Create: `klippy/bridge_endstop.py`
- Test: `test/test_bridge_endstop.py`

- [ ] **Step 1: Write the failing test**

Create `test/test_bridge_endstop.py`:

```python
from klippy.bridge_endstop import (
    PROVIDER_ID_FIRST,
    BridgeEndstop,
    allocate_provider_id,
)


class FakeCommand:
    def __init__(self, response=None):
        self.sent = []
        self.response = response

    def send(self, args):
        self.sent.append(list(args))
        return self.response


class FakeMcu:
    def __init__(self):
        self.oids = 0
        self.config_cmds = []
        self.config_callbacks = []
        self.query_cmd = FakeCommand()
        self.state_cmd = FakeCommand({"oid": 0, "armed": 0, "pin_value": 0})

    def create_oid(self):
        oid = self.oids
        self.oids += 1
        return oid

    def register_config_callback(self, cb):
        self.config_callbacks.append(cb)

    def add_config_cmd(self, cmd):
        self.config_cmds.append(cmd)

    def lookup_command(self, template):
        return self.query_cmd

    def lookup_query_command(self, template, response, oid=None):
        return self.state_cmd

    def seconds_to_clock(self, seconds):
        return int(seconds * 1_000_000)


class FakePrinter:
    def __init__(self):
        self.objects = {}

    def add_object(self, name, obj):
        self.objects[name] = obj

    def lookup_object(self, name, default=None):
        return self.objects.get(name, default)


def _pin_params(mcu, pin="PA8", invert=0, pullup=1):
    return {
        "chip": mcu,
        "chip_name": "mcu",
        "pin": pin,
        "invert": invert,
        "pullup": pullup,
    }


def _connected(mcu, endstop):
    for cb in mcu.config_callbacks:
        cb()
    return endstop


def test_config_cmd_emitted():
    mcu = FakeMcu()
    _connected(mcu, BridgeEndstop(_pin_params(mcu), 3))
    assert mcu.config_cmds == [
        "config_endstop oid=0 endstop_id=3 pin=PA8 pull_up=1 invert=0"
    ]


def test_is_triggered_applies_invert():
    mcu = FakeMcu()
    endstop = _connected(mcu, BridgeEndstop(_pin_params(mcu, invert=1), 3))
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 0}
    assert endstop.is_triggered() is True
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 1}
    assert endstop.is_triggered() is False


def test_arm_sends_rest_ticks():
    mcu = FakeMcu()
    endstop = _connected(mcu, BridgeEndstop(_pin_params(mcu), 3))
    endstop.arm(0.001)
    assert mcu.query_cmd.sent == [[0, 1000]]


def test_query_endstop_matches_is_triggered():
    mcu = FakeMcu()
    endstop = _connected(mcu, BridgeEndstop(_pin_params(mcu), 3))
    mcu.state_cmd.response = {"oid": 0, "armed": 0, "pin_value": 1}
    assert endstop.query_endstop(0.0) is True


def test_provider_ids_allocate_sequentially():
    printer = FakePrinter()
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST + 1
    assert allocate_provider_id(printer) == PROVIDER_ID_FIRST + 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run python -m pytest test/test_bridge_endstop.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'klippy.bridge_endstop'`

- [ ] **Step 3: Write the implementation**

Create `klippy/bridge_endstop.py`:

```python
PROVIDER_ID_FIRST = 3
ENDSTOP_ID_MAX = 255

_ALLOCATOR_OBJECT = "bridge_endstop_allocator"


class BridgeEndstop:
    def __init__(self, pin_params, endstop_id):
        self.mcu = pin_params["chip"]
        self.endstop_id = endstop_id
        self.pin = pin_params["pin"]
        self.pullup = pin_params["pullup"]
        self.invert = pin_params["invert"]
        self.oid = self.mcu.create_oid()
        self._query_cmd = None
        self._state_cmd = None
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.mcu.add_config_cmd(
            "config_endstop oid=%d endstop_id=%d pin=%s pull_up=%d invert=%d"
            % (self.oid, self.endstop_id, self.pin, self.pullup, self.invert)
        )
        self._query_cmd = self.mcu.lookup_command(
            "query_endstop oid=%c rest_ticks=%u"
        )
        self._state_cmd = self.mcu.lookup_query_command(
            "endstop_query_state oid=%c",
            "endstop_state oid=%c armed=%c pin_value=%c",
            oid=self.oid,
        )

    def is_triggered(self):
        params = self._state_cmd.send([self.oid])
        return bool(params["pin_value"] ^ self.invert)

    def arm(self, poll_period):
        rest_ticks = self.mcu.seconds_to_clock(poll_period)
        self._query_cmd.send([self.oid, rest_ticks])

    def query_endstop(self, print_time):
        return self.is_triggered()

    def bridge_mcu_handle(self):
        return getattr(self.mcu, "_bridge_handle", None)


class _ProviderIdAllocator:
    def __init__(self):
        self._next_id = PROVIDER_ID_FIRST

    def allocate(self):
        if self._next_id > ENDSTOP_ID_MAX:
            raise ValueError("out of bridge endstop ids")
        endstop_id = self._next_id
        self._next_id += 1
        return endstop_id


def allocate_provider_id(printer):
    allocator = printer.lookup_object(_ALLOCATOR_OBJECT, None)
    if allocator is None:
        allocator = _ProviderIdAllocator()
        printer.add_object(_ALLOCATOR_OBJECT, allocator)
    return allocator.allocate()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run python -m pytest test/test_bridge_endstop.py -q`
Expected: `6 passed`

- [ ] **Step 5: Commit**

```bash
git add klippy/bridge_endstop.py test/test_bridge_endstop.py
git commit -m "feat: shared BridgeEndstop wrapper + provider endstop-id allocator"
```

---

### Task 2: Refactor `homing.py` onto `BridgeEndstop` (no behavior change)

**Files:**
- Modify: `klippy/extras/homing.py` (full rewrite of the file)

The axis entries become dicts of `{"endstop": BridgeEndstop, "provider": None, "trigger_height": None}`. The `_BridgeEndstop` query wrapper class disappears (`BridgeEndstop.query_endstop` covers `query_endstops` directly). Virtual endstop pins are still skipped — that changes in Task 5.

- [ ] **Step 1: Replace the file**

Replace the entire contents of `klippy/extras/homing.py` with:

```python
import logging

from klippy.bridge_endstop import BridgeEndstop

HOMING_POLL_PERIOD = 0.001
HOMING_TIMEOUT = 30.0


class Homing:
    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object("pins")

        self._axes = {}
        for axis_index, axis_name in enumerate("xyz"):
            section = "stepper_" + axis_name
            if not config.has_section(section):
                continue
            endstop_pin = config.getsection(section).get("endstop_pin", None)
            if endstop_pin is None or "virtual_endstop" in endstop_pin:
                continue
            pin_params = ppins.parse_pin(
                endstop_pin, can_invert=True, can_pullup=True
            )
            self._axes[axis_index] = {
                "endstop": BridgeEndstop(pin_params, axis_index),
                "provider": None,
                "trigger_height": None,
            }

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("G28", self.cmd_G28, desc="Home")
        gcode.register_command(
            "_HOME_TEST",
            self.cmd_HOME_TEST,
            desc="Bench only: home one axis with override SPEED/MAX_TRAVEL",
        )

        query_endstops = self.printer.load_object(config, "query_endstops")
        for axis_index in sorted(self._axes):
            query_endstops.register_endstop(
                self._axes[axis_index]["endstop"], "xyz"[axis_index]
            )

    def cmd_G28(self, gcmd):
        requested = [
            i for i, a in enumerate("XYZ") if gcmd.get(a, None) is not None
        ]
        if not requested:
            requested = sorted(self._axes.keys())
        toolhead = self.printer.lookup_object("toolhead")
        bridge = self.printer.lookup_object("motion_bridge")
        kin = toolhead.get_kinematics()
        for axis in requested:
            entry = self._axes.get(axis)
            if entry is None:
                raise gcmd.error("G28: axis %s has no endstop" % ("XYZ"[axis],))
            self._home_axis(gcmd, toolhead, bridge, kin, axis, entry)

    def cmd_HOME_TEST(self, gcmd):
        axis_name = gcmd.get("AXIS").upper()
        if axis_name not in ("X", "Y", "Z"):
            raise gcmd.error("_HOME_TEST: AXIS must be X, Y, or Z")
        axis = "XYZ".index(axis_name)
        entry = self._axes.get(axis)
        if entry is None:
            raise gcmd.error("_HOME_TEST: axis %s has no endstop" % axis_name)
        speed = gcmd.get_float("SPEED", None, above=0.0)
        max_travel = gcmd.get_float("MAX_TRAVEL", None, above=0.0)
        toolhead = self.printer.lookup_object("toolhead")
        bridge = self.printer.lookup_object("motion_bridge")
        kin = toolhead.get_kinematics()
        self._home_axis(
            gcmd, toolhead, bridge, kin, axis, entry, speed, max_travel
        )

    def _home_axis(
        self,
        gcmd,
        toolhead,
        bridge,
        kin,
        axis,
        entry,
        speed_override=None,
        max_travel_override=None,
    ):
        rail = kin._axis_rails().get(axis)
        if rail is None:
            raise gcmd.error("G28: no rail for axis %s" % ("XYZ"[axis],))
        hi = rail.get_homing_info()
        pos_min, pos_max = rail.get_range()
        endstop = entry["endstop"]
        endstop_mcu = endstop.bridge_mcu_handle()
        if endstop_mcu is None:
            raise gcmd.error(
                "G28: endstop MCU for axis %s is not attached to the bridge"
                % ("XYZ"[axis],)
            )
        direction = 1.0 if hi.positive_dir else -1.0
        speed = speed_override if speed_override is not None else hi.speed
        max_travel = (
            max_travel_override
            if max_travel_override is not None
            else abs(pos_max - pos_min)
        )

        if endstop.is_triggered():
            raise gcmd.error(
                "G28 %s: endstop already triggered — move the axis off the "
                "switch before homing" % ("XYZ"[axis],)
            )

        toolhead.wait_moves()
        stepper_enable = self.printer.lookup_object("stepper_enable")
        for s in rail.get_steppers():
            stepper_enable.motor_debug_enable(s.get_name(), True)

        endstop.arm(HOMING_POLL_PERIOD)

        bridge.home_axis_start(
            axis, direction, speed, max_travel, endstop.endstop_id, endstop_mcu
        )
        reactor = self.printer.get_reactor()
        deadline = reactor.monotonic() + HOMING_TIMEOUT
        result = None
        while result is None:
            try:
                result = bridge.home_axis_poll()
            except Exception as e:
                bridge.home_abort()
                raise gcmd.error("G28 %s failed: %s" % ("XYZ"[axis], e))
            if result is not None:
                break
            if reactor.monotonic() > deadline:
                bridge.home_abort()
                raise gcmd.error(
                    "G28 %s: timed out waiting for endstop trip"
                    % ("XYZ"[axis],)
                )
            reactor.pause(reactor.monotonic() + 0.010)
        trip_pos, final_pos = result

        overshoot = final_pos[axis] - trip_pos[axis]
        newpos = list(toolhead.get_position())
        newpos[axis] = hi.position_endstop + overshoot
        toolhead.set_position(newpos, homing_axes=[axis])
        logging.info(
            "homing: %s switch=%.4f overshoot=%+.4f set %s=%.4f",
            "XYZ"[axis],
            hi.position_endstop,
            overshoot,
            "XYZ"[axis],
            newpos[axis],
        )


def load_config(config):
    return Homing(config)
```

- [ ] **Step 2: Verify imports and the suite stay green**

Run: `uv run python -m pytest test/test_imports.py test/test_bridge_endstop.py -q`
Expected: all pass (import_test imports `klippy.extras.homing`, which now imports `klippy.bridge_endstop`)

Run: `uv run python -m pytest -q 2>&1 | tail -3`
Expected: no new failures versus a pre-change run (the `.test` items skip without the native cdylib)

- [ ] **Step 3: Commit**

```bash
git add klippy/extras/homing.py
git commit -m "refactor(homing): axis entries use shared BridgeEndstop"
```

---

### Task 3: Extract `trip_move`, computed deadline, no-movement check, post-home retract

**Files:**
- Modify: `klippy/extras/homing.py`

`trip_move` is the public descend-until-trip primitive the probe will call in Task 4. Behavior changes versus Task 2: deadline computed from travel/speed instead of flat 30 s; trigger-before-movement is an error; G28 retracts by `homing_retract_dist` after setting position.

- [ ] **Step 1: Replace the constants**

In `klippy/extras/homing.py`, replace:

```python
HOMING_POLL_PERIOD = 0.001
HOMING_TIMEOUT = 30.0
```

with:

```python
HOMING_POLL_PERIOD = 0.001
TRIP_DEADLINE_MARGIN = 5.0
NO_MOVEMENT_EPSILON = 0.005
```

- [ ] **Step 2: Replace `_home_axis` and add `trip_move`**

Replace the entire `_home_axis` method with the following two methods:

```python
    def _home_axis(
        self,
        gcmd,
        toolhead,
        bridge,
        kin,
        axis,
        entry,
        speed_override=None,
        max_travel_override=None,
    ):
        rail = kin._axis_rails().get(axis)
        if rail is None:
            raise gcmd.error("G28: no rail for axis %s" % ("XYZ"[axis],))
        hi = rail.get_homing_info()
        pos_min, pos_max = rail.get_range()
        trigger_height = entry["trigger_height"]
        if trigger_height is None:
            trigger_height = hi.position_endstop
        direction = 1.0 if hi.positive_dir else -1.0
        speed = speed_override if speed_override is not None else hi.speed
        max_travel = (
            max_travel_override
            if max_travel_override is not None
            else abs(pos_max - pos_min)
        )

        stepper_enable = self.printer.lookup_object("stepper_enable")
        for s in rail.get_steppers():
            stepper_enable.motor_debug_enable(s.get_name(), True)

        trip_pos, final_pos = self.trip_move(
            gcmd, toolhead, bridge, axis, direction, speed, max_travel, entry
        )

        overshoot = final_pos[axis] - trip_pos[axis]
        newpos = list(toolhead.get_position())
        newpos[axis] = trigger_height + overshoot
        toolhead.set_position(newpos, homing_axes=[axis])
        logging.info(
            "homing: %s trigger=%.4f overshoot=%+.4f set %s=%.4f",
            "XYZ"[axis],
            trigger_height,
            overshoot,
            "XYZ"[axis],
            newpos[axis],
        )
        if hi.retract_dist:
            retractpos = list(toolhead.get_position())
            retractpos[axis] -= direction * hi.retract_dist
            toolhead.move(retractpos, hi.retract_speed)
            toolhead.wait_moves()

    def trip_move(
        self, gcmd, toolhead, bridge, axis, direction, speed, max_travel, entry
    ):
        endstop = entry["endstop"]
        endstop_mcu = endstop.bridge_mcu_handle()
        if endstop_mcu is None:
            raise gcmd.error(
                "trip_move: endstop MCU for axis %s is not attached to the"
                " bridge" % ("XYZ"[axis],)
            )
        if endstop.is_triggered():
            raise gcmd.error(
                "%s endstop already triggered — move off the trigger before"
                " homing or probing" % ("XYZ"[axis],)
            )
        toolhead.wait_moves()
        start_axis_pos = toolhead.get_position()[axis]
        provider = entry["provider"]
        if provider is not None and hasattr(provider, "trip_move_begin"):
            provider.trip_move_begin(entry)
        try:
            endstop.arm(HOMING_POLL_PERIOD)
            bridge.home_axis_start(
                axis,
                direction,
                speed,
                max_travel,
                endstop.endstop_id,
                endstop_mcu,
            )
            reactor = self.printer.get_reactor()
            deadline = (
                reactor.monotonic()
                + max_travel / speed
                + TRIP_DEADLINE_MARGIN
            )
            while True:
                try:
                    result = bridge.home_axis_poll()
                except Exception as e:
                    bridge.home_abort()
                    raise gcmd.error(
                        "%s trip move failed: %s" % ("XYZ"[axis], e)
                    )
                if result is not None:
                    break
                if reactor.monotonic() > deadline:
                    bridge.home_abort()
                    raise gcmd.error(
                        "%s endstop did not trigger within %.1fmm of travel"
                        % ("XYZ"[axis], max_travel)
                    )
                reactor.pause(reactor.monotonic() + 0.010)
        finally:
            if provider is not None and hasattr(provider, "trip_move_end"):
                provider.trip_move_end(entry)
        trip_pos, final_pos = result
        if abs(trip_pos[axis] - start_axis_pos) < NO_MOVEMENT_EPSILON:
            raise gcmd.error(
                "%s endstop triggered prior to movement — trigger is stuck"
                " or miswired" % ("XYZ"[axis],)
            )
        return trip_pos, final_pos
```

- [ ] **Step 3: Verify the suite stays green**

Run: `uv run python -m pytest test/test_imports.py test/test_bridge_endstop.py -q`
Expected: all pass

- [ ] **Step 4: Commit**

```bash
git add klippy/extras/homing.py
git commit -m "feat(homing): trip_move primitive, computed deadline, no-movement check, post-home retract"
```

---

### Task 4: Rewrite `klippy/extras/probe.py`

**Files:**
- Modify: `klippy/extras/probe.py` (full replacement — the legacy 1011-line module is deleted)
- Test: `test/test_probe_logic.py`

The ten extras that `from . import probe` keep importing fine (no import-time symbol references — verified), but break loudly at runtime if configured. That is the agreed follow-up.

- [ ] **Step 1: Write the failing test**

Create `test/test_probe_logic.py`:

```python
import pytest

from klippy import pins
from klippy.extras.probe import (
    calc_probe_z_result,
    validate_virtual_endstop_request,
)


def _pin_params(pin="z_virtual_endstop", invert=0, pullup=0):
    return {
        "chip": object(),
        "chip_name": "probe",
        "pin": pin,
        "invert": invert,
        "pullup": pullup,
    }


def test_average():
    assert calc_probe_z_result([1.0, 2.0, 6.0], "average") == pytest.approx(
        3.0
    )


def test_median_odd():
    assert calc_probe_z_result([5.0, 1.0, 2.0], "median") == 2.0


def test_median_even_averages_middle_pair():
    assert calc_probe_z_result([4.0, 1.0, 2.0, 3.0], "median") == pytest.approx(
        2.5
    )


def test_unknown_method_raises():
    with pytest.raises(ValueError):
        calc_probe_z_result([1.0], "mode")


def test_valid_virtual_endstop_request_passes():
    validate_virtual_endstop_request(_pin_params(), 2)


def test_wrong_pin_name_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(pin="virtual_endstop"), 2)


def test_modifiers_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(pullup=1), 2)
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(invert=1), 2)


def test_non_z_axis_rejected():
    with pytest.raises(pins.error):
        validate_virtual_endstop_request(_pin_params(), 0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run python -m pytest test/test_probe_logic.py -q`
Expected: FAIL — `ImportError: cannot import name 'calc_probe_z_result'`

- [ ] **Step 3: Replace `klippy/extras/probe.py`**

Replace the entire contents with:

```python
from klippy import pins
from klippy.bridge_endstop import BridgeEndstop, allocate_provider_id

Z_AXIS = 2
ACCURACY_DEFAULT_SAMPLES = 10


def calc_probe_z_result(values, method):
    if method == "median":
        ordered = sorted(values)
        middle = len(ordered) // 2
        if len(ordered) % 2:
            return ordered[middle]
        return (ordered[middle - 1] + ordered[middle]) / 2.0
    if method != "average":
        raise ValueError("unknown samples_result '%s'" % (method,))
    return sum(values) / len(values)


def validate_virtual_endstop_request(pin_params, axis):
    if pin_params["pin"] != "z_virtual_endstop":
        raise pins.error(
            "probe only provides the virtual pin 'z_virtual_endstop',"
            " not '%s'" % (pin_params["pin"],)
        )
    if pin_params["invert"] or pin_params["pullup"]:
        raise pins.error("Can not pullup/invert probe virtual endstop")
    if axis != Z_AXIS:
        raise pins.error(
            "probe:z_virtual_endstop is only usable as the Z endstop"
        )


class PrinterProbe:
    cmd_PROBE_help = "Probe Z-height at the current XY position"
    cmd_QUERY_PROBE_help = "Return the current probe state"
    cmd_PROBE_ACCURACY_help = "Probe Z-height repeatedly and report statistics"

    def __init__(self, config):
        self.printer = config.get_printer()
        ppins = self.printer.lookup_object("pins")
        pin_desc = config.get("pin")
        pin_params = ppins.lookup_pin(
            pin_desc, can_invert=True, can_pullup=True
        )
        if not hasattr(pin_params["chip"], "create_oid"):
            raise config.error(
                "[probe] pin must be a GPIO pin on an MCU, not '%s'"
                % (pin_desc,)
            )
        self._endstop = BridgeEndstop(
            pin_params, allocate_provider_id(self.printer)
        )

        self.z_offset = config.getfloat("z_offset")
        self.x_offset = config.getfloat("x_offset", 0.0)
        self.y_offset = config.getfloat("y_offset", 0.0)
        self.speed = config.getfloat("speed", 5.0, above=0.0)
        self.lift_speed = config.getfloat("lift_speed", self.speed, above=0.0)
        self.samples = config.getint("samples", 1, minval=1)
        self.sample_retract_dist = config.getfloat(
            "sample_retract_dist", 2.0, above=0.0
        )
        self.samples_result = config.getchoice(
            "samples_result", ["median", "average"], "average"
        )
        self.samples_tolerance = config.getfloat(
            "samples_tolerance", 0.100, minval=0.0
        )
        self.samples_retries = config.getint(
            "samples_tolerance_retries", 0, minval=0
        )

        self.last_query = False
        self.last_z_result = 0.0

        ppins.register_chip("probe", self)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command(
            "PROBE", self.cmd_PROBE, desc=self.cmd_PROBE_help
        )
        gcode.register_command(
            "QUERY_PROBE", self.cmd_QUERY_PROBE, desc=self.cmd_QUERY_PROBE_help
        )
        gcode.register_command(
            "PROBE_ACCURACY",
            self.cmd_PROBE_ACCURACY,
            desc=self.cmd_PROBE_ACCURACY_help,
        )
        query_endstops = self.printer.load_object(config, "query_endstops")
        query_endstops.register_endstop(self._endstop, "probe")

    def setup_bridge_endstop(self, pin_params, axis):
        validate_virtual_endstop_request(pin_params, axis)
        return self._endstop

    def get_position_endstop(self):
        return self.z_offset

    def get_offsets(self):
        return self.x_offset, self.y_offset, self.z_offset

    def get_status(self, eventtime):
        return {
            "name": "probe",
            "last_query": self.last_query,
            "last_z_result": self.last_z_result,
        }

    def _check_homed(self, gcmd, toolhead):
        curtime = self.printer.get_reactor().monotonic()
        kin_status = toolhead.get_kinematics().get_status(curtime)
        if "z" not in kin_status["homed_axes"]:
            raise gcmd.error("Must home before probe")

    def _probe_once(self, gcmd, toolhead, homing_obj, bridge, speed):
        kin = toolhead.get_kinematics()
        rail = kin._axis_rails().get(Z_AXIS)
        if rail is None:
            raise gcmd.error("PROBE: no Z rail configured")
        pos_min = rail.get_range()[0]
        current_z = toolhead.get_position()[Z_AXIS]
        max_travel = current_z - pos_min
        if max_travel <= 0.0:
            raise gcmd.error("PROBE: toolhead already at or below position_min")
        trip_pos, final_pos = homing_obj.trip_move(
            gcmd,
            toolhead,
            bridge,
            Z_AXIS,
            -1.0,
            speed,
            max_travel,
            {"endstop": self._endstop, "provider": None},
        )
        newpos = list(toolhead.get_position())
        newpos[Z_AXIS] = final_pos[Z_AXIS]
        toolhead.set_position(newpos)
        return trip_pos[Z_AXIS]

    def _retract(self, toolhead, target_z, lift_speed):
        newpos = list(toolhead.get_position())
        newpos[Z_AXIS] = target_z
        toolhead.move(newpos, lift_speed)
        toolhead.wait_moves()

    def run_probe(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        homing_obj = self.printer.lookup_object("homing")
        bridge = self.printer.lookup_object("motion_bridge")
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        sample_count = gcmd.get_int("SAMPLES", self.samples, minval=1)
        retract = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        tolerance = gcmd.get_float(
            "SAMPLES_TOLERANCE", self.samples_tolerance, minval=0.0
        )
        max_retries = gcmd.get_int(
            "SAMPLES_TOLERANCE_RETRIES", self.samples_retries, minval=0
        )
        method = gcmd.get("SAMPLES_RESULT", self.samples_result)
        if method not in ("median", "average"):
            raise gcmd.error("SAMPLES_RESULT must be median or average")
        self._check_homed(gcmd, toolhead)
        retries = 0
        measured = []
        while True:
            z = self._probe_once(gcmd, toolhead, homing_obj, bridge, speed)
            measured.append(z)
            if max(measured) - min(measured) > tolerance:
                if retries >= max_retries:
                    raise gcmd.error("Probe samples exceed samples_tolerance")
                gcmd.respond_info("Probe samples exceed tolerance. Retrying...")
                retries += 1
                measured = []
            if len(measured) >= sample_count:
                break
            self._retract(toolhead, z + retract, lift_speed)
        return calc_probe_z_result(measured, method)

    def cmd_PROBE(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        pos = toolhead.get_position()
        z_result = self.run_probe(gcmd)
        gcmd.respond_info(
            "probe at %.3f,%.3f is z=%.6f" % (pos[0], pos[1], z_result)
        )
        self.last_z_result = z_result

    def cmd_QUERY_PROBE(self, gcmd):
        triggered = self._endstop.is_triggered()
        self.last_query = triggered
        gcmd.respond_info("probe: %s" % ("TRIGGERED" if triggered else "open"))

    def cmd_PROBE_ACCURACY(self, gcmd):
        toolhead = self.printer.lookup_object("toolhead")
        homing_obj = self.printer.lookup_object("homing")
        bridge = self.printer.lookup_object("motion_bridge")
        speed = gcmd.get_float("PROBE_SPEED", self.speed, above=0.0)
        lift_speed = gcmd.get_float("LIFT_SPEED", self.lift_speed, above=0.0)
        sample_count = gcmd.get_int(
            "SAMPLES", ACCURACY_DEFAULT_SAMPLES, minval=1
        )
        retract = gcmd.get_float(
            "SAMPLE_RETRACT_DIST", self.sample_retract_dist, above=0.0
        )
        self._check_homed(gcmd, toolhead)
        pos = toolhead.get_position()
        gcmd.respond_info(
            "PROBE_ACCURACY at X:%.3f Y:%.3f Z:%.3f"
            " (samples=%d retract=%.3f speed=%.1f lift_speed=%.1f)"
            % (pos[0], pos[1], pos[2], sample_count, retract, speed, lift_speed)
        )
        measured = []
        for _ in range(sample_count):
            z = self._probe_once(gcmd, toolhead, homing_obj, bridge, speed)
            measured.append(z)
            self._retract(toolhead, z + retract, lift_speed)
        average = calc_probe_z_result(measured, "average")
        median = calc_probe_z_result(measured, "median")
        sigma = (
            sum((v - average) ** 2 for v in measured) / len(measured)
        ) ** 0.5
        gcmd.respond_info(
            "probe accuracy results: maximum %.6f, minimum %.6f,"
            " range %.6f, average %.6f, median %.6f, standard deviation %.6f"
            % (
                max(measured),
                min(measured),
                max(measured) - min(measured),
                average,
                median,
                sigma,
            )
        )


def load_config(config):
    return PrinterProbe(config)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run python -m pytest test/test_probe_logic.py test/test_imports.py -q`
Expected: all pass (`import_test` re-imports every extras module against the new probe.py)

- [ ] **Step 5: Commit**

```bash
git add klippy/extras/probe.py test/test_probe_logic.py
git commit -m "feat(probe): bridge-native [probe] rewrite (PROBE, QUERY_PROBE, PROBE_ACCURACY)"
```

---

### Task 5: Virtual endstop resolution in `homing.py`

**Files:**
- Modify: `klippy/extras/homing.py`

- [ ] **Step 1: Replace the endstop-parsing loop**

In `Homing.__init__`, replace:

```python
            endstop_pin = config.getsection(section).get("endstop_pin", None)
            if endstop_pin is None or "virtual_endstop" in endstop_pin:
                continue
            pin_params = ppins.parse_pin(
                endstop_pin, can_invert=True, can_pullup=True
            )
            self._axes[axis_index] = {
                "endstop": BridgeEndstop(pin_params, axis_index),
                "provider": None,
                "trigger_height": None,
            }
```

with:

```python
            stepper_config = config.getsection(section)
            endstop_pin = stepper_config.get("endstop_pin", None)
            if endstop_pin is None:
                continue
            pin_params = ppins.parse_pin(
                endstop_pin, can_invert=True, can_pullup=True
            )
            chip = pin_params["chip"]
            if hasattr(chip, "setup_bridge_endstop"):
                entry = self._provider_entry(
                    stepper_config, axis_index, chip, pin_params
                )
            elif hasattr(chip, "create_oid"):
                entry = {
                    "endstop": BridgeEndstop(pin_params, axis_index),
                    "provider": None,
                    "trigger_height": None,
                }
            else:
                raise config.error(
                    "endstop_pin '%s' in [%s]: chip '%s' is neither an MCU"
                    " nor a virtual endstop provider"
                    % (endstop_pin, section, pin_params["chip_name"])
                )
            self._axes[axis_index] = entry
```

- [ ] **Step 2: Add `_provider_entry`**

Add this method to `Homing`, directly after `__init__`:

```python
    def _provider_entry(self, stepper_config, axis_index, chip, pin_params):
        endstop = chip.setup_bridge_endstop(pin_params, axis_index)
        trigger_height = None
        if hasattr(chip, "get_position_endstop"):
            trigger_height = chip.get_position_endstop()
            if stepper_config.get("position_endstop", None) is not None:
                raise stepper_config.error(
                    "[%s] must not set position_endstop: its virtual endstop"
                    " '%s' supplies the trigger height"
                    % (stepper_config.get_name(), pin_params["chip_name"])
                )
        return {
            "endstop": endstop,
            "provider": chip,
            "trigger_height": trigger_height,
        }
```

Notes for the implementer:
- A `probe:z_virtual_endstop` pin with no `[probe]` section fails inside `ppins.parse_pin` with `Unknown pin chip name 'probe'` — pins.py raises this itself, no extra handling needed.
- `^probe:z_virtual_endstop` parses fine (pullup=1) and is then rejected by `validate_virtual_endstop_request` inside `setup_bridge_endstop`.

- [ ] **Step 3: Verify**

Run: `uv run python -m pytest test/test_imports.py test/test_probe_logic.py test/test_bridge_endstop.py -q`
Expected: all pass

- [ ] **Step 4: Commit**

```bash
git add klippy/extras/homing.py
git commit -m "feat(homing): resolve virtual endstop providers via the pins registry"
```

---

### Task 6: Full gates

- [ ] **Step 1: Run the lint gate**

Run: `./scripts/ci.sh ruff`
Expected: exit 0. Fix any formatting/lint findings in the new files and amend the relevant commit.

- [ ] **Step 2: Run the full Python suite**

Run: `uv run python -m pytest -n auto -q 2>&1 | tail -5`
Expected: no failures (skips for cdylib-gated tests are normal)

- [ ] **Step 3: Run the Rust suite (sanity — nothing should have changed)**

Run: `cd rust && cargo nextest run 2>&1 | tail -3`
Expected: all pass

- [ ] **Step 4: Commit any fixes**

Only if Steps 1–3 required changes.

---

### Task 7: Simulator validation

**Files:**
- Possibly create: a probe-enabled sim config + test under `tools/sim_klippy/` (exact shape depends on the harness — see Step 1)

- [ ] **Step 1: Learn the harness**

Invoke the `kalico-sim` skill. Study `tools/sim_klippy/test_home_x.py` (sends `G28 X` to a simulated printer and asserts success) and the configs under `tools/sim_klippy/configs/`.

- [ ] **Step 2: Cover the matrix**

Using the harness patterns from Step 1, validate:

1. Boot with `[probe]` + `[stepper_z] endstop_pin: probe:z_virtual_endstop` → klippy reaches ready.
2. `G28 Z` via probe → completes; reported Z ≈ `z_offset` + retract.
3. `PROBE` then `PROBE_ACCURACY SAMPLES=3` → complete, report plausible z.
4. `QUERY_PROBE` → reports `open`.
5. Boot with `[probe]` + GPIO Z endstop → ready; `PROBE` works.
6. Failure cases (expect hard errors, not hangs): `probe:z_virtual_endstop` without `[probe]`; `PROBE` while unhomed; `position_endstop` set in `[stepper_z]` alongside the virtual endstop; `^probe:z_virtual_endstop`.

If the sim cannot inject a trigger on the probe pin for descent moves, validate cases 1, 5, and 6 in the sim (boot/config-level — these need no motion) and defer 2–4 to the bench task; say so explicitly in the task report.

- [ ] **Step 3: Commit any sim tests added**

```bash
git add tools/sim_klippy/
git commit -m "test(sim): probe virtual-endstop coverage"
```

---

### Task 8: Bench validation on the Neptune (requires the user)

This task is interactive — coordinate with the user; never send motion G-code without their per-command go-ahead.

- [ ] **Step 1: Deploy host code to the Pi**

The change is host-Python only (no firmware rebuild — `config_endstop` is already in the flashed F401 firmware; the new probe is just a fourth instance of it). Confirm with the user which branch the Neptune should run (the Pi is on `sota-motion`; this work is on `homing-probe`), merge/push accordingly, then:

```bash
ssh dderg@ethercatpi5.local 'cd ~/kalico && git pull && sudo systemctl restart klipper'
```

- [ ] **Step 2: Confirm clean boot**

Poll moonraker `printer/info` until state `ready`. If it errors, fetch the log per the fetch-logs-to-tmp convention and debug before any motion.

- [ ] **Step 3: With per-command user approval**

In order: `QUERY_PROBE` (expect `open`; have the user press the probe and re-query for `TRIGGERED`), `QUERY_ENDSTOPS`, `G28` (X, Y, then Z via probe — user watching, hand near power), `PROBE`, `PROBE_ACCURACY`. Verify the homed Z makes physical sense against `z_offset` and that the post-home retract lifts the nozzle.
