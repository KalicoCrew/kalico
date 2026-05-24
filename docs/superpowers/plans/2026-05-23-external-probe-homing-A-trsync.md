# Plan A: Re-enable trsync/TriggerDispatch for Non-Bridge MCUs

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Parallel plan.** This plan runs independently of Plan B (firmware/Rust). Both must complete before Plan C (integration) can begin.

**Goal:** Make `MCU_trsync` and `TriggerDispatch` work for non-bridge probe MCUs (Beacon, Eddy, load cell) while keeping them disabled for bridge-driven MCUs (H7, F446).

**Architecture:** Add `_bridge_drives_steppers` flag to MCU objects, set during kinematics config parsing. `MCU_trsync._build_config` conditionally creates `_trdispatch_mcu` (mainline FFI for non-bridge, None for bridge-driven). `TriggerDispatch.__init__` always allocates `_trdispatch`; `start()`/`stop()` runtime-gate on the primary MCU flag.

**Tech Stack:** Python (klippy)

**Spec:** `docs/kalico-rewrite/external-probe-homing.md`, Piece A (lines 88-212)

---

## File Map

| File | Change |
|------|--------|
| `klippy/mcu.py` | `_bridge_drives_steppers` attribute; conditional `MCU_trsync._build_config`/`start()`/`stop()`; `TriggerDispatch.__init__`/`start()`/`stop()`/`wait_end()` |
| `klippy/motion_toolhead.py` | Set flag in `_register_axis`; fix stale comment at line 115 |

---

### Task 1: Add `_bridge_drives_steppers` flag

**Files:**
- Modify: `klippy/mcu.py:970` (MCU.__init__)
- Modify: `klippy/motion_toolhead.py:139` (_register_axis)
- Modify: `klippy/motion_toolhead.py:115` (stale comment)

- [ ] **Step 1:** In `MCU.__init__` (mcu.py, around line 970 near `self._motion_bridge`), add:
```python
self._bridge_drives_steppers = False
```

- [ ] **Step 2:** In `BridgeKinematics._register_axis` (motion_toolhead.py:139), after the `for mcu_stepper in rail.get_steppers():` loop that calls `setup_itersolve` and `set_trapq`, add inside the same loop:
```python
mcu_stepper.get_mcu()._bridge_drives_steppers = True
```

- [ ] **Step 3:** Replace the stale comment at motion_toolhead.py:115-117:
```python
# corexy bridge does not drive Z, but a stable Klipper printer.cfg may
# still declare [stepper_z]/[stepper_z1..3]. Consume them as passthrough
# rails so option validation passes; runtime ignores them.
```
with:
```python
# Z is kinematically independent in corexy (no A/B mixing), but the
# bridge dispatches Z curves to the Z MCU normally. Register Z rails
# so printer.cfg validation passes and homing/move-checking works.
```

- [ ] **Step 4:** Verify ordering is correct: `_register_axis` runs during `BridgeKinematics.__init__` (config parsing), which happens before `klippy:mcu_identify` and `klippy:connect`. The flag is set before `MCU_trsync._build_config` runs. Confirm by searching for the event handler registration:
```
grep -n "klippy:connect.*_connect\|klippy:connect.*_init_planner" klippy/mcu.py klippy/motion_toolhead.py
```
Expected: MCU `_connect` and MotionToolhead `_init_planner` both registered for `klippy:connect`, both AFTER config parsing where `_register_axis` runs.

- [ ] **Step 5:** Commit: `feat: add _bridge_drives_steppers flag for MCU dispatch detection`

---

### Task 2: Conditional MCU_trsync

**Files:**
- Modify: `klippy/mcu.py:275` (_build_config)
- Modify: `klippy/mcu.py:330` (start)
- Modify: `klippy/mcu.py:370` (stop)

- [ ] **Step 1:** In `MCU_trsync._build_config` (mcu.py:275), replace the unconditional `self._trdispatch_mcu = None` at line 306 with a conditional block. The `config_trsync` oid allocation and command lookups (`trsync_start_cmd`, `trsync_set_timeout_cmd`, `trsync_trigger_cmd`, `trsync_query_cmd`, `stepper_stop_cmd`) stay unconditional — both bridge and non-bridge MCUs support these firmware commands.

After the command lookups, add:
```python
if self._mcu._bridge_drives_steppers:
    # Bridge owns step generation; no C-level trdispatch needed.
    # The firmware trsync stays idle (config_trsync allocated but
    # never started). See external-probe-homing.md Piece A.
    self._trdispatch_mcu = None
else:
    # Non-bridge MCU (Beacon, Eddy, load cell): restore mainline
    # trdispatch_mcu for C-level serial interception.
    set_timeout_tag = mcu.lookup_command(
        "trsync_set_timeout oid=%c clock=%u"
    ).get_command_tag()
    trigger_cmd = mcu.lookup_command(
        "trsync_trigger oid=%c reason=%c"
    )
    trigger_tag = trigger_cmd.get_command_tag()
    state_cmd = mcu.lookup_command(
        "trsync_state oid=%c can_trigger=%c trigger_reason=%c clock=%u"
    )
    state_tag = state_cmd.get_command_tag()
    ffi_main, ffi_lib = chelper.get_ffi()
    self._trdispatch_mcu = ffi_main.gc(
        ffi_lib.trdispatch_mcu_alloc(
            self._trdispatch,
            mcu._serial.get_serialqueue(),
            self._cmd_queue,
            self._oid,
            set_timeout_tag,
            trigger_tag,
            state_tag,
        ),
        ffi_lib.free,
    )
```

- [ ] **Step 2:** In `MCU_trsync.start()` (mcu.py:330), replace the unconditional raise with:
```python
def start(self, print_time, report_offset, trigger_completion, expire_timeout):
    self._trigger_completion = trigger_completion
    if self._mcu._bridge_drives_steppers:
        # Bridge-driven MCU: no-op. Firmware trsync was never started,
        # so no timeout can fire. The actual motion stop comes from
        # bridge software_trip, not from trsync.
        return
    # Non-bridge MCU: full mainline path
    self._home_end_clock = None
    clock = self._mcu.print_time_to_clock(print_time)
    expire_ticks = self._mcu.seconds_to_clock(expire_timeout)
    expire_clock = clock + expire_ticks
    report_ticks = self._mcu.seconds_to_clock(expire_timeout * 0.3)
    report_clock = clock + int(report_ticks * report_offset + 0.5)
    min_extend_ticks = int(report_ticks * 0.8 + 0.5)
    ffi_main, ffi_lib = chelper.get_ffi()
    ffi_lib.trdispatch_mcu_setup(
        self._trdispatch_mcu,
        clock,
        expire_clock,
        expire_ticks,
        min_extend_ticks,
    )
    self._mcu.register_response(
        self._handle_trsync_state, "trsync_state", self._oid
    )
    self._trsync_start_cmd.send(
        [self._oid, report_clock, report_ticks, self.REASON_COMMS_TIMEOUT],
        reqclock=clock,
    )
    for s in self._steppers:
        self._stepper_stop_cmd.send([s.get_oid(), self._oid])
    self._trsync_set_timeout_cmd.send(
        [self._oid, expire_clock], reqclock=clock
    )
```

- [ ] **Step 3:** In `MCU_trsync.stop()` (mcu.py:370), replace the unconditional raise with:
```python
def stop(self):
    if self._mcu._bridge_drives_steppers:
        # Bridge-driven MCU: no-op. Return REASON_ENDSTOP_HIT —
        # safe under both Beacon's and TriggerDispatch's aggregation
        # rules (primary trsync result dominates; secondary is only
        # scanned for COMMS_TIMEOUT).
        self._trigger_completion = None
        return self.REASON_ENDSTOP_HIT
    # Non-bridge MCU: full mainline path
    self._mcu.register_response(None, "trsync_state", self._oid)
    self._trigger_completion = None
    if self._mcu.is_fileoutput():
        return self.REASON_ENDSTOP_HIT
    params = self._trsync_query_cmd.send(
        [self._oid, self.REASON_HOST_REQUEST]
    )
    for s in self._steppers:
        s.note_homing_end()
    return params["trigger_reason"]
```

- [ ] **Step 4:** Verify by booting klippy on the printer with Beacon connected. Check klippy.log for:
  - No `MCU_trsync.start() not yet supported` error
  - Successful Beacon MCU identification
  - `config_trsync` allocated on both Beacon MCU (with trdispatch_mcu) and F446 (without)

```
# On trident.local:
sudo systemctl restart klipper
grep -i "trsync\|beacon\|MCU_trsync" /tmp/klippy.log | tail -20
```

- [ ] **Step 5:** Commit: `feat: conditional MCU_trsync — mainline path for non-bridge MCUs`

---

### Task 3: Conditional TriggerDispatch

**Files:**
- Modify: `klippy/mcu.py:388-445` (TriggerDispatch class)

- [ ] **Step 1:** In `TriggerDispatch.__init__` (mcu.py:388), always allocate `_trdispatch` via FFI. Replace:
```python
self._trdispatch = None
```
with:
```python
ffi_main, ffi_lib = chelper.get_ffi()
self._trdispatch = ffi_main.gc(ffi_lib.trdispatch_alloc(), ffi_lib.free)
```

- [ ] **Step 2:** In `TriggerDispatch.start()` (mcu.py:424), replace the unconditional raise with runtime-gated mainline:
```python
def start(self, print_time):
    if self._mcu._bridge_drives_steppers:
        raise error(
            "TriggerDispatch.start(): probe on bridge-driven MCU "
            "'%s' is not supported — probe must be on a separate "
            "board" % (self._mcu._name,)
        )
    reactor = self._mcu.get_printer().get_reactor()
    self._trigger_completion = reactor.completion()
    expire_timeout = get_danger_options().multi_mcu_trsync_timeout
    if len(self._trsyncs) == 1:
        expire_timeout = get_danger_options().single_mcu_trsync_timeout
    for i, trsync in enumerate(self._trsyncs):
        report_offset = float(i) / len(self._trsyncs)
        trsync.start(
            print_time,
            report_offset,
            self._trigger_completion,
            expire_timeout,
        )
    etrsync = self._trsyncs[0]
    ffi_main, ffi_lib = chelper.get_ffi()
    ffi_lib.trdispatch_start(
        self._trdispatch, etrsync.REASON_HOST_REQUEST
    )
    return self._trigger_completion
```

- [ ] **Step 3:** Replace `wait_end()` raise with mainline:
```python
def wait_end(self, end_time):
    etrsync = self._trsyncs[0]
    etrsync.set_home_end_time(end_time)
    if self._mcu.is_fileoutput():
        self._trigger_completion.complete(True)
    self._trigger_completion.wait()
```

- [ ] **Step 4:** Replace `stop()` raise with runtime-gated mainline:
```python
def stop(self):
    if self._mcu._bridge_drives_steppers:
        raise error(
            "TriggerDispatch.stop(): probe on bridge-driven MCU "
            "'%s' is not supported" % (self._mcu._name,)
        )
    ffi_main, ffi_lib = chelper.get_ffi()
    ffi_lib.trdispatch_stop(self._trdispatch)
    res = [trsync.stop() for trsync in self._trsyncs]
    err_res = [r for r in res if r >= MCU_trsync.REASON_COMMS_TIMEOUT]
    if err_res:
        return err_res[0]
    return res[0]
```

- [ ] **Step 5:** Verify `probe_eddy_current.py` and `load_cell_probe.py` paths wouldn't crash by checking that their probe MCUs are non-bridge:
```
grep -n "TriggerDispatch" klippy/extras/probe_eddy_current.py klippy/extras/load_cell/load_cell_probe.py
```
These construct `TriggerDispatch(self._mcu)` where `self._mcu` is the probe's own MCU (Eddy board, load cell board) — not a bridge-driven MCU. The runtime gate in `start()` would only fire if someone wired a probe directly to H7/F446.

- [ ] **Step 6:** Commit: `feat: conditional TriggerDispatch — mainline path for non-bridge probe MCUs`

---

## Verification Checklist

After all three tasks:

- [ ] klippy boots with Beacon connected — no crash, no `not yet supported` errors
- [ ] Beacon MCU identifies correctly (check klippy.log for serial connect + config)
- [ ] F446 and H7 MCUs still work (sensorless X/Y homing unchanged)
- [ ] `config_trsync` oid allocated on F446 (harmless idle allocation)
- [ ] No regressions in existing homing (G28 X, G28 Y)

Z homing with Beacon will NOT work yet — that requires Plan B (firmware commands) and Plan C (credit loop integration).
