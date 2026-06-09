# [safe_z_home] Support Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make `[safe_z_home]` work on the bridge-native homing stack by removing `Homing`'s config-load-order sensitivity, with regression coverage at two levels: a fast klippy-boot pytest and a kalico-sim end-to-end variant pinning the exact Neptune failure.

**Architecture:** `Homing.__init__` stops parsing endstop pins at construction; a new `resolve_endstops()` builds the axis entries and is invoked by `BridgeKinematics` at toolhead load — deterministically after all config sections, so virtual-endstop providers like `[probe]` are always registered first. The legacy `klippy/extras/safe_z_home.py` stays byte-for-byte unchanged (full API-compat trace in the spec). Verification: a subprocess pytest that boots klippy through the config phase with the failing section order, plus a `--probe-test safe-z` variant in the kalico-sim runner.

**Tech Stack:** Python (klippy host), pytest (repo `tests/`, auto-collected via `testpaths`), kalico-sim Docker harness (`tools/kalico-sim/runner.py`).

Spec: `docs/superpowers/specs/2026-06-09-safe-z-home-design.md`

**Context for the implementer:**
- The bug: klippy config sections load in file order. `[safe_z_home]` calls `printer.load_object(config, "homing")` at its own config time (`klippy/extras/safe_z_home.py:24`); the fork's `Homing.__init__` (`klippy/extras/homing.py:24`) immediately parses every `stepper_*` `endstop_pin`. When `stepper_z` uses `probe:z_virtual_endstop` and `[probe]` appears later in the file, the `probe` pin chip isn't registered yet → `klippy.pins.error: Unknown pin chip name 'probe'` → klippy never starts.
- `homing_override.py:18` has the same early `load_object("homing")`; the fix covers it automatically.
- MCU OID allocation (`BridgeEndstop.__init__` → `mcu.create_oid()` + `register_config_callback`) must stay inside the config phase. Toolhead load is inside the config phase (klippy's `_read_config` loads all sections, then calls `add_printer_objects` for the toolhead), so resolving there is safe.
- Fast-test marker: `MotionToolhead.__init__` logs `MotionToolhead: Phase 1 skeleton initialized` (`klippy/motion_toolhead.py:303`) *after* `BridgeKinematics` (and therefore endstop resolution) succeeds, but *before* any MCU connect. Its presence in klippy.log proves the config phase passed; `logging.exception("Config error")` (`klippy/printer.py:425`) marks failure. No MCU, firmware, or Docker needed.
- The kalico-sim probe tests run inside Docker: build the image, then run the entrypoint with `--probe-test <variant>`. Known harness limitation (pre-existing, out of scope): full-motion G28 may fault with `PieceStartInPast` in Docker, so motion-dependent checks are judged **relative to the `virtual` variant baseline**, not absolutely.

---

### Task 1: Fast failing regression test (klippy boot, no Docker)

**Files:**
- Create: `tests/klippy_host/__init__.py` (empty)
- Create: `tests/klippy_host/test_homing_load_order.py`

- [ ] **Step 1: Write the failing test**

Create `tests/klippy_host/__init__.py` empty, and `tests/klippy_host/test_homing_load_order.py`:

```python
import pathlib
import subprocess
import sys
import tempfile
import time

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]

CONFIG_PHASE_OK = "MotionToolhead: Phase 1 skeleton initialized"
CONFIG_PHASE_FAILED = "Config error"

NEPTUNE_SHAPED_CONFIG = """
[mcu]
serial: /tmp/kalico-test-no-such-serial

[printer]
kinematics: cartesian
max_velocity: 100
max_accel: 1000
max_z_velocity: 10
max_z_accel: 30

[stepper_x]
step_pin: PC12
dir_pin: PB3
enable_pin: !PD2
microsteps: 16
rotation_distance: 40
endstop_pin: PA13
position_endstop: 0
position_max: 235
homing_speed: 50

[stepper_y]
step_pin: PC11
dir_pin: PA15
enable_pin: !PC10
microsteps: 16
rotation_distance: 40
endstop_pin: PB8
position_endstop: 0
position_max: 234
homing_speed: 50

[stepper_z]
step_pin: PC7
dir_pin: PC9
enable_pin: !PC8
microsteps: 16
rotation_distance: 8
endstop_pin: probe:z_virtual_endstop
position_min: -5
position_max: 283
homing_speed: 10

[safe_z_home]
home_xy_position: 117.5, 117.5
z_hop: 10

[probe]
pin: ^PA8
speed: 5
x_offset: -28
y_offset: 20
z_offset: 3
"""


def _boot_through_config_phase(config_text):
    with tempfile.TemporaryDirectory(prefix="kalico_cfg_") as tmpdir:
        tmp = pathlib.Path(tmpdir)
        cfg = tmp / "printer.cfg"
        cfg.write_text(config_text)
        log = tmp / "klippy.log"
        proc = subprocess.Popen(
            [
                sys.executable,
                str(REPO_ROOT / "klippy" / "klippy.py"),
                str(cfg),
                "-l",
                str(log),
            ],
            cwd=str(REPO_ROOT),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        try:
            deadline = time.monotonic() + 60.0
            while time.monotonic() < deadline:
                text = log.read_text(errors="replace") if log.exists() else ""
                if CONFIG_PHASE_OK in text or CONFIG_PHASE_FAILED in text:
                    return text
                if proc.poll() is not None:
                    time.sleep(0.5)
                    return (
                        log.read_text(errors="replace")
                        if log.exists()
                        else ""
                    )
                time.sleep(0.2)
            raise AssertionError(
                "klippy reached neither config-phase marker; log:\n"
                + (log.read_text(errors="replace") if log.exists() else "")
            )
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()


def test_safe_z_home_section_before_probe_section_parses():
    log_text = _boot_through_config_phase(NEPTUNE_SHAPED_CONFIG)
    assert "Unknown pin chip name" not in log_text, log_text[-3000:]
    assert CONFIG_PHASE_FAILED not in log_text, log_text[-3000:]
    assert CONFIG_PHASE_OK in log_text
```

- [ ] **Step 2: Run the test to verify it fails on the bug**

```bash
python3 -m pytest tests/klippy_host/test_homing_load_order.py -v
```

Expected: FAIL with `Unknown pin chip name 'probe'` visible in the asserted log tail. If instead klippy fails for an environment reason (e.g. chelper C build unavailable on this machine), stop and report — the test must be red *for the right reason* before proceeding. Do not commit yet.

---

### Task 2: Deferred endstop resolution in `Homing` + call site

**Files:**
- Modify: `klippy/extras/homing.py:10-60` (`Homing.__init__`, new `resolve_endstops`)
- Modify: `klippy/motion_toolhead.py:102` (`BridgeKinematics.__init__` call site)
- Test: `tests/klippy_host/test_homing_load_order.py` (from Task 1)

- [ ] **Step 1: Split `Homing.__init__`**

In `klippy/extras/homing.py`, replace the current `__init__` (everything from `def __init__` through the `query_endstops` registration loop) with:

```python
    def __init__(self, config):
        self.printer = config.get_printer()
        self._config = config
        self._axes = None

        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("G28", self.cmd_G28, desc="Home")
        gcode.register_command(
            "_HOME_TEST",
            self.cmd_HOME_TEST,
            desc="Bench only: home one axis with override SPEED/MAX_TRAVEL",
        )

    def resolve_endstops(self):
        if self._axes is not None:
            raise self.printer.config_error(
                "homing: resolve_endstops called twice"
            )
        config, self._config = self._config, None
        ppins = self.printer.lookup_object("pins")

        self._axes = {}
        for axis_index, axis_name in enumerate("xyz"):
            section = "stepper_" + axis_name
            if not config.has_section(section):
                continue
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
                    "endstop": BridgeEndstop(
                        pin_params, AXIS_ENDSTOP_IDS[axis_index]
                    ),
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

        query_endstops = self.printer.load_object(config, "query_endstops")
        for axis_index in sorted(self._axes):
            query_endstops.register_endstop(
                self._axes[axis_index]["endstop"], "xyz"[axis_index]
            )
```

The endstop-resolution body is the existing code moved verbatim; only the split and the double-call guard are new (`printer.config_error` is `configfile.error`, set at `klippy/printer.py:166`). `_provider_entry`, `cmd_G28`, `cmd_HOME_TEST`, `_home_axis`, `trip_move` stay as they are.

- [ ] **Step 2: Guard command entry points against unresolved state**

`cmd_G28` and `cmd_HOME_TEST` index `self._axes`; if resolution never ran they would die with a bare `TypeError`. Fail loudly with a clear message instead — at the top of `cmd_G28`:

```python
    def cmd_G28(self, gcmd):
        if self._axes is None:
            raise gcmd.error("G28: homing endstops were never resolved")
```

and at the top of `cmd_HOME_TEST`:

```python
    def cmd_HOME_TEST(self, gcmd):
        if self._axes is None:
            raise gcmd.error("_HOME_TEST: homing endstops were never resolved")
```

- [ ] **Step 3: Invoke resolution from `BridgeKinematics`**

In `klippy/motion_toolhead.py`, change line 102:

```python
        self._printer.load_object(config, "homing")
```

to:

```python
        self._printer.load_object(config, "homing").resolve_endstops()
```

`load_object` returns the existing instance when `[safe_z_home]`/`[homing_override]` already loaded homing earlier, so resolution always runs exactly once, here, after all config sections.

- [ ] **Step 4: Run the Task 1 test — verify green**

```bash
python3 -m pytest tests/klippy_host/test_homing_load_order.py -v
```

Expected: PASS (klippy.log reaches `MotionToolhead: Phase 1 skeleton initialized`, no `Unknown pin chip name`, no `Config error`).

- [ ] **Step 5: Check the other klippy host suites for direct `Homing` construction fallout**

```bash
python3 -m pytest tests/ -q 2>&1 | tail -15
```

Expected: same pass/skip set as before the change (the `tests/motion_bridge` suite self-skips without the native cdylib; `tools/` is deliberately outside testpaths). If any test constructs `Homing` directly and now fails because endstops are unresolved, fix the test setup to call `resolve_endstops()` after construction — do not weaken the double-call guard.

- [ ] **Step 6: Commit**

```bash
git add klippy/extras/homing.py klippy/motion_toolhead.py tests/klippy_host/
git commit -m "fix(homing): defer endstop resolution to toolhead load

[safe_z_home] (and homing_override) force-load homing at config time,
before later sections like [probe] have registered their pin chips;
Homing parsed endstop pins in __init__, so a Neptune-style config
([safe_z_home] above [probe], probe:z_virtual_endstop on Z) died with
\"Unknown pin chip name 'probe'\" on every start. Homing now registers
G28 at load but resolves endstops via resolve_endstops(), called once
by BridgeKinematics after all config sections. Regression test boots
klippy through the config phase with the failing section order."
```

---

### Task 3: kalico-sim `safe-z` end-to-end variant

**Files:**
- Modify: `tools/kalico-sim/runner.py` (variants tuple ~line 1113, `_generate_probe_config` ~line 1128, `run_probe_test` checks ~lines 1391–1406)

- [ ] **Step 1: Capture the baseline — run the existing `virtual` variant**

From the repo root:

```bash
docker build -t kalico-sim -f tools/kalico-sim/Dockerfile .
docker run --rm kalico-sim --probe-test virtual
```

Record which checks PASS/FAIL. `boot-ready`, `query-probe-open`, and `probe-before-home-rejected` must PASS. The G28/PROBE motion checks may FAIL with `PieceStartInPast` (known Docker limitation) — note their status; the new variant is held to the same standard.

- [ ] **Step 2: Add the `safe-z` variant to the variants tuple**

In `tools/kalico-sim/runner.py`, change:

```python
PROBE_TEST_VARIANTS = (
    "virtual",
    "gpio-z",
    "no-probe",
    "conflict",
    "pullup",
)
```

to:

```python
PROBE_TEST_VARIANTS = (
    "virtual",
    "safe-z",
    "gpio-z",
    "no-probe",
    "conflict",
    "pullup",
)
```

`PROBE_TEST_BOOT_ERRORS` gets no entry for `safe-z` — this variant must boot cleanly.

- [ ] **Step 3: Generate the Neptune-shaped config for `safe-z`**

In `_generate_probe_config`, the `else` branch already produces `endstop_pin: probe:z_virtual_endstop` + `probe_pin = "gpiochip0/gpio202"`, which is correct for `safe-z`. Add a safe_z_home section emitted **before** `[probe]` — the ordering is the whole point of the regression. Change the section-assembly part of the function:

```python
    probe_section = ""
    if variant != "no-probe":
        probe_section = f"""
[probe]
pin: {probe_pin}
z_offset: 1.5
speed: 5
x_offset: 24.0
y_offset: 5.0
"""

    safe_z_section = ""
    if variant == "safe-z":
        safe_z_section = """
[safe_z_home]
home_xy_position: 125, 125
z_hop: 10
z_hop_speed: 15
"""
```

and in the returned f-string template, place `{safe_z_section}` between the `[stepper_z]` block and `{probe_section}`:

```python
{z_endstop}
position_min: -5
position_max: 250
homing_speed: 5
{safe_z_section}{probe_section}
[input_shaper]
```

(Current template has `{probe_section}` directly after `homing_speed: 5`.)

- [ ] **Step 4: Extend the runtime checks for `safe-z`**

In `run_probe_test`:

a. The Z-trigger-height check is currently gated on `variant == "virtual"`. Change:

```python
                if variant == "virtual":
```

to:

```python
                if variant in ("virtual", "safe-z"):
```

b. The post-home Z expectation: after G28-Z the head sits at trigger 1.5 + retract 5.0 = 6.5; safe_z_home then lifts to `z_hop` 10.0. Change:

```python
                z = _query_toolhead_z(api_socket)
                expected_z = 6.5 if variant == "virtual" else 5.0
```

to:

```python
                z = _query_toolhead_z(api_socket)
                if variant == "safe-z":
                    expected_z = 10.0
                elif variant == "virtual":
                    expected_z = 6.5
                else:
                    expected_z = 5.0
```

c. After that check, add a safe-z-only XY assertion (G28 ends at `home_xy_position` since `move_to_previous` defaults to False):

```python
                if variant == "safe-z":
                    status = query_status(api_socket)
                    pos = (
                        status.get("result", {})
                        .get("status", {})
                        .get("toolhead", {})
                        .get("position")
                    )
                    xy_ok = (
                        pos is not None
                        and abs(pos[0] - 125.0) < 0.5
                        and abs(pos[1] - 125.0) < 0.5
                    )
                    check(
                        "g28-at-safe-xy",
                        xy_ok,
                        "position=%s expected x,y~125,125" % (pos,),
                    )
```

The existing PROBE / PROBE_ACCURACY / QUERY_PROBE checks after this point apply to `safe-z` unchanged (`expected_probe_z = 1.5` branch already covers it).

- [ ] **Step 5: Run the new variant**

```bash
docker build -t kalico-sim -f tools/kalico-sim/Dockerfile .
docker run --rm kalico-sim --probe-test safe-z
```

Expected: `boot-ready` PASS (this is the check that fails with `Unknown pin chip name 'probe'` if Task 2 were reverted), `query-probe-open` PASS, `probe-before-home-rejected` PASS; `g28-z-trigger-height` / `post-home-retract-z` / `g28-at-safe-xy` / `probe` / `probe-accuracy` PASS **or** failing only in the same `PieceStartInPast` way the `virtual` baseline from Step 1 does.

- [ ] **Step 6: Regression-run the existing probe variants**

```bash
for v in virtual gpio-z no-probe conflict pullup; do
  docker run --rm kalico-sim --probe-test $v || echo "VARIANT $v FAILED"
done
```

Expected: identical results to the Step 1 baseline. Note `no-probe` still expects `Unknown pin chip name 'probe'` — now raised from `resolve_endstops()` at toolhead load instead of homing load; same message, still a startup config error, so the `boot-error[no-probe]` check stays green.

- [ ] **Step 7: Commit**

```bash
git add tools/kalico-sim/runner.py
git commit -m "test(sim): safe-z probe variant pins [safe_z_home]-before-[probe] boot order

Boots the Neptune-shaped config (safe_z_home section above probe,
probe:z_virtual_endstop on Z) and asserts clean startup plus
G28-through-safe_z_home behavior: z-hop to 10, XY parked at
home_xy_position, Z homed at the probe trigger height."
```

---

### Task 4: Bench verification on the Neptune

**Files:** none (operational task)

- [ ] **Step 1: Push the branch**

```bash
git push origin safe-z-home
```

- [ ] **Step 2: Flash the Neptune from this branch**

Use the user-blessed bench script (pulls, builds, holds klippy down, power-cycles, flashes via ST-Link, restores):

```bash
.claude/skills/neptune-bench/scripts/flash-neptune.sh safe-z-home
```

Expected: script ends with `DONE — F401 flashed and klippy is ready.` — on the pre-fix branch this fails at the final readiness poll because the config error keeps klippy from reaching ready.

- [ ] **Step 3: Confirm clean boot + FIRMWARE_RESTART**

```bash
ssh dderg@ethercatpi5.local 'curl -s http://127.0.0.1:7125/printer/info'
```

Expected: JSON with `"state": "ready"` (not a 404).

Then issue FIRMWARE_RESTART via moonraker and re-poll:

```bash
ssh dderg@ethercatpi5.local 'curl -s -X POST http://127.0.0.1:7125/printer/firmware_restart; sleep 20; curl -s http://127.0.0.1:7125/printer/info'
```

Expected: state returns to `ready`. If it does not, capture `~/printer_data/logs/klippy.log` to `/tmp/` and stop — that is the separate possibly-real MCU boot issue from the spec's caveat; report findings, do not improvise fixes.

- [ ] **Step 4: STOP — hardware motion requires explicit user permission**

Do **not** send G28 or any motion command. Report bench status to the user and let them drive the live homing test (G28 through safe_z_home on the real machine: expect z-hop to 10, X+Y home, travel to 117.5/117.5, Z probe-home). Per standing rule, every motion command needs per-command user approval.
