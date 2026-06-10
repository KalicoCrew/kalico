# Bridge-Compatible External Probe Homing

> **SUPERSEDED (2026-06-10)** by
> [`beacon-fork-survey.md`](beacon-fork-survey.md). Written before the
> homing rewrite (PR #34); its premises — mainline `drip_move`/`home_wait`
> flow, unmodified beacon.py, Piece A's `_bridge_drives_steppers` flag —
> no longer hold (Piece A was implemented and demolished in 45f0fe737).
> Piece B (credit-window deadman) and Piece C (retained-curve trigger-time
> evaluation) carry forward into the new sub-project specs.

## Problem

External probes (Beacon, Cartographer, Eddy) follow mainline Klipper's homing
contract:

1. `home_start()` returns a reactor completion
2. `drip_move()` produces motion interruptible by that completion
3. `home_wait()` returns trigger time (0.0 if no trigger)

The bridge broke this contract in two places:

- **`MCU_trsync` disabled for all MCUs.** `mcu.py:306` sets
  `_trdispatch_mcu = None` unconditionally. Beacon's own MCU (which runs stock
  Klipper-compatible firmware with real trsync support) gets caught by this
  blanket disable. `MCU_trsync.start()` raises
  `"MCU_trsync.start() not yet supported under the new motion path (Phase 4)"`.

- **`drip_move` ignores the completion.** When no bridge `arm_ids` are
  registered in `active_homing_arms`, `drip_move` falls back to a regular
  `self.move()` — an uninterruptible full-travel move with no endstop abort
  capability.

The result: Z homing with Beacon crashes klippy with an unhandled `mcu.error`
before any Z motion begins.

## Design Principles

- **Zero changes to Beacon.** `BeaconEndstopWrapper`, `BeaconEndstopShared`,
  and `BeaconContactEndstopWrapper` stay unmodified.
- **Mainline safety parity.** The MCU never has authority to move more than
  50ms of travel without host permission, matching mainline's
  `DRIP_SEGMENT_TIME = 0.050`.
- **Unified code path.** External probe trips flow through the same bridge
  homing state machine as GPIO trips, with a different trigger source.
- **External probes on non-bridge MCUs work.** Any probe whose primary MCU is
  a separate board (Beacon, Cartographer, Eddy, load cell) works unmodified.
  Probes wired directly to a bridge-driven MCU (H7/F446) remain unsupported —
  they would need a bridge-local probe path, which is out of scope.

## Reference: Mainline Homing Flow

### Multi-MCU homing (Beacon + stepper MCU)

Beacon is a separate USB board. The Z stepper is on a different MCU (F446).
Mainline coordinates the stop across MCUs:

1. **Beacon MCU** detects trigger (eddy current threshold or nozzle contact).
   Fires its local `trsync`.
2. **Host C code** (`trdispatch_mcu`) intercepts the `trsync_state` message on
   the serial receive queue — no Python GIL involved. Immediately relays
   `trsync_trigger` to the F446's trsync.
3. **F446** receives `trsync_trigger`, executes `stepper_stop_on_trigger`,
   halts its step queue.

Response time: ~1-2ms (two USB hops). At 20 mm/s, that's 0.02-0.04mm of
overshoot.

### drip_move safety model

Mainline's `drip_move` feeds step pulses to the MCU in 50ms chunks
(`DRIP_SEGMENT_TIME = 0.050`). Between chunks, it checks
`drip_completion.test()`. The MCU never has more than 50ms of motion buffered.

If the host dies (crash, USB disconnect), the MCU runs through its ~50ms buffer
and stops. At 20 mm/s, that's 1mm of worst-case travel before the MCU runs dry.

Two mechanisms, two response times:
- **Normal operation:** C relay stops steps in ~1-2ms
- **Host death:** MCU runs out of buffered steps in ~50ms

### Position after trigger

`homing_move()` records two positions per stepper:

- `trig_pos = stepper.get_past_mcu_position(trigger_time)` — position at
  trigger instant
- `halt_pos = stepper.get_mcu_position()` — position when motion stopped

Beacon's `_handle_home_rails_end` then overrides `trig_pos` with a post-homing
measurement (`_sample()`) for eddy current homing.

## Design

Three independent pieces.

### Piece A: Re-enable trsync and TriggerDispatch for non-bridge MCUs

**What changed from mainline and why it's wrong:**

Two layers are broken:

1. **`TriggerDispatch`** (mcu.py:388): `__init__` sets `self._trdispatch = None`
   instead of calling `ffi_lib.trdispatch_alloc()`. `start()` and `stop()` raise
   unconditionally. This is the entry point for probes that construct their own
   dispatch: `probe_eddy_current.py:225` and `load_cell_probe.py:981` both call
   `mcu.TriggerDispatch(self._mcu)` directly. These probes never touch
   `MCU_endstop` or `BridgeTriggerDispatch`.

2. **`MCU_trsync`** (mcu.py:306): `_build_config` sets `_trdispatch_mcu = None`
   unconditionally. `start()` and `stop()` raise when `_trdispatch_mcu` is None.
   This is the lower layer that `TriggerDispatch` and Beacon's
   `BeaconEndstopShared` both depend on.

Both are correct for H7 and F446 (the bridge generates their steps via curve
evaluation), but wrong for non-bridge probe MCUs (Beacon, Eddy, load cell —
independent USB boards with stock Klipper-compatible firmware and real trsync).

**Fix:**

Add a flag `mcu._bridge_drives_steppers`, default `False`. **Meaning:** "the
bridge runtime dispatches motion curves to this MCU and owns its step
generation." This is stronger than "a configured rail references this MCU" —
it means the bridge's dispatch.rs has this MCU in its `McuAxisConfig` list
and sends `SegmentPushParams` to it when axes move.

Set to `True` during config parsing in `BridgeKinematics._register_axis`,
which iterates stepper configs and knows each stepper's MCU. This is correct
because the dispatch config (dispatch.rs:463-469) mirrors kinematics axis
registration — every axis registered in `_register_axis` gets a corresponding
`McuAxisConfig` entry with `axes: vec![AXIS_Z]` (or `AXIS_X, AXIS_Y`). The
"passthrough" comment at motion_toolhead.py:115 refers to the kinematic
transform (Z isn't mixed with X/Y in CoreXY), NOT to dispatch — the bridge
dispatches Z curves to F446 when Z is moving (dispatch.rs:240-258 only skips
when ALL axes are trivially constant).

This happens before `klippy:mcu_identify` and `klippy:connect`, so the flag is
available when `MCU_trsync._build_config` runs (fired during `_send_config` in
`klippy:connect`). H7 and F446 get `True`. Beacon / Eddy / load cell MCUs (no
motion axes) stay `False`.

**Ordering guarantee:** Config parsing (`__init__`) → `mcu_identify` (serial
connect, `claim_mcu`) → `connect` (`_send_config` → `_build_config`). The flag
is set in the first phase; `_build_config` reads it in the third.

Conditional behavior at both layers:

**`TriggerDispatch` (mcu.py:388):**

Constructor-time gating is unreliable: `TriggerDispatch.__init__` runs during
pin setup, which may happen before `_register_axis` sets the flag. A probe on a
bridge MCU could get `trdispatch_alloc()` if the flag is still `False`. To
prevent silent fallthrough into the unsupported legacy path:

- **`__init__`:** Always allocate `_trdispatch` via `trdispatch_alloc()`. This
  is safe — C trdispatch is a host-side data structure, not a firmware command.
  Allocating it for a bridge MCU wastes a few bytes but causes no harm.
- **`start()` and `stop()`:** Runtime-gate on `self._mcu._bridge_drives_steppers`
  — the PRIMARY probe MCU, not each secondary trsync's MCU. If `True`, raise
  (probe is on a bridge MCU — unsupported). This gate does NOT affect secondary
  `MCU_trsync` instances on bridge-driven stepper MCUs; those are handled by
  `MCU_trsync.start()` being a no-op for bridge-driven MCUs (see below). The
  flag is guaranteed to be set by the time `start()` is called (homing happens
  after `klippy:ready`, long after config parsing completes).

  Concrete example: Eddy probe on its own MCU (non-bridge) with Z stepper on
  F446 (bridge-driven). `TriggerDispatch.start()` checks `eddy_mcu` — not
  bridge-driven, proceeds. Iterates trsyncs: eddy trsync runs mainline path,
  F446 trsync's `start()` is a no-op. No conflict.

**`MCU_trsync._build_config`:**

| MCU type | `_bridge_drives_steppers` | `_trdispatch_mcu` | `start()` | `stop()` |
|----------|--------------------------|-------------------|-----------|----------|
| Non-bridge (Beacon, Eddy) | `False` | Created via FFI (mainline) | Full mainline path | Full mainline path |
| Bridge-driven (F446) | `True` | `None` | No-op (see below) | No-op, returns `REASON_ENDSTOP_HIT` |

**F446 trsync lifecycle — full no-op:**

Beacon's `BeaconEndstopShared.add_stepper()` creates a secondary `MCU_trsync`
on the Z stepper's MCU (F446). We can't prevent this without modifying Beacon.

The F446 trsync is entirely ceremonial:

- **`_build_config`:** `config_trsync` allocates the oid on F446 firmware. The
  firmware trsync starts idle (no timers, no callbacks — verified in
  `src/trsync.c:71`, `command_config_trsync` only allocates and sets function
  pointers). **Firmware requirement:** bridge-driven MCUs must continue to
  support `config_trsync` as a harmless idle allocation. This is already true
  of the current kalico firmware and stock Klipper firmware.
- **`start()`:** Complete no-op. Does NOT send `trsync_start`,
  `stepper_stop_on_trigger`, or `trsync_set_timeout` to F446. Does NOT call
  FFI `trdispatch_mcu_setup`. The firmware trsync stays idle — no timeout can
  fire because it was never started.
- **`stop()`:** Complete no-op. The firmware trsync was never started, so there
  is nothing to retire. Returns `REASON_ENDSTOP_HIT` unconditionally.
  **Aggregation rule this relies on:** Beacon's `trsync_stop` (beacon.py)
  uses `res[0]` (the primary Beacon MCU trsync) for the trigger/no-trigger
  decision, and scans ALL results only for `REASON_COMMS_TIMEOUT`. The F446
  result at `res[1]` is only checked against `REASON_COMMS_TIMEOUT`.
  Returning `REASON_ENDSTOP_HIT` is safe under this rule — any value except
  `REASON_COMMS_TIMEOUT` would also work. Document this dependency so a
  future refactor doesn't treat the ceremonial result as a real signal.

**F446 trsync is NOT part of the completion chain.** Beacon passes
`_trigger_completion` to all `trsync.start()` calls, but F446's `start()` is a
no-op — it never calls `trdispatch_mcu_setup`, so the C trdispatch layer has no
knowledge of the F446 trsync. Only the Beacon MCU trsync is connected to the
completion. When Beacon triggers, the Beacon-side trsync fires the completion.
F446 motion is stopped by the bridge's `software_trip` (Piece B), not by trsync.

**Why F446 doesn't need C-level interception:** In mainline, the C
`trdispatch_mcu` on F446 intercepts trsync messages and executes
`stepper_stop_on_trigger` at serial-interrupt speed. In bridge mode,
`stepper_stop_on_trigger` on F446 is meaningless — there are no queued step
pulses to stop. The curve evaluator is a separate step generation path that
doesn't listen to the trsync mechanism. The actual stop comes from the bridge's
`software_trip` command (Piece B). So the C relay would be sending commands to
an inert trsync — fast but pointless.

**Files:** `klippy/mcu.py`

### Piece B: Credit-windowed homing with software trip

**Mainline equivalent:** `drip_move` feeds 50ms chunks and checks
`drip_completion.test()` between chunks. The C relay stops steps in ~1-2ms
when the probe triggers.

**Bridge equivalent:** Two mechanisms, matching mainline's two-tier model:

1. **Normal operation (~2-3ms response):** The host waits on Beacon's reactor
   completion. When it fires, `software_trip` freezes the curve evaluator.
2. **Host death (50ms safety net):** Credit deadline on the curve evaluator.
   MCU freezes if the host stops extending.

#### MCU firmware changes

**Soft deadline on curve evaluator during homing.** When a homing segment has
source kind `Software` (no GPIO):

- Evaluator has `deadline_clock`, initially `evaluation_start + 50ms` of ticks
  (set when the MCU begins evaluating the segment, NOT at arm/submission time —
  this prevents a slow host-side dispatch path from consuming the first grant
  before the credit loop begins)
- **Pre-start extensions are silently ignored.** If the host sends
  `runtime_extend_homing_deadline` before the segment begins evaluating, the
  MCU discards it (no error, no latch). The initial 50ms grant at evaluation
  start covers the window. The host's 25ms extension interval ensures the
  first effective extension lands well within the initial grant.
- Each tick: if `current_clock >= deadline_clock`, freeze (same as reaching
  end of travel — snapshot positions, report as `REASON_DEADLINE_EXPIRED`)
- **Deadline expiry is a terminal safety fault**, not a resumable pause. If the
  host is alive and extending credit normally, deadlines never expire. If the
  host dies, the MCU stops permanently after 50ms. If a momentary host stall
  causes an unexpected expiry, homing fails (same as mainline's trsync timeout
  behavior — recoverable by re-issuing G28)

**New MCU commands:**

- `runtime_extend_homing_deadline arm_id=%u` — extends the deadline for the
  specified arm. The arm_id parameter addresses a specific active homing move
  (prevents extending the wrong one in future multi-arm scenarios). Grant size
  is fixed: MCU sets `deadline_clock = current_clock + FIXED_GRANT_TICKS`
  where `FIXED_GRANT_TICKS` corresponds to 50ms. The host cannot control the
  grant size — the MCU always grants exactly 50ms from now. This prevents a
  host bug from silently handing the MCU unlimited travel authority.
- `runtime_software_trip arm_id=%u` — immediate freeze, same as GPIO trip.
  Snapshots stepper positions, sends `kalico_endstop_tripped` event with
  step counts and trip clock.

#### Host-side flow in drip_move

**Path selection in drip_move.** `drip_completion` is ALWAYS passed by
homing.py — even for bridge-native GPIO homing. The discriminator is
`active_homing_arms`, not `drip_completion`:

```
arm_ids = list(self.active_homing_arms)
if arm_ids:
    # Bridge-native GPIO/sensorless path (existing, unchanged).
    # Bridge firmware handles trip detection; drip_completion is
    # ignored (bridge fires it via BridgeTriggerDispatch).
    bridge.submit_homing_move(pos3, speed, arm_ids)
    bridge.wait_moves()
elif drip_completion is not None and not drip_completion.test():
    # External probe software-trip path (new, described below).
else:
    # No endstop armed — regular move fallback.
    self.move(newpos, speed)
```

The paths are mutually exclusive by construction: bridge-native endstops
register arm_ids via `BridgeTriggerDispatch.start()`; external probes don't.
Mixed endstop sets (both bridge arm AND external probe for the same homing
move) cannot arise — a rail has one endstop.

**Detection invariant:** "no arm_ids + completion present = external probe"
relies on ALL `MCU_endstop` instances using `BridgeTriggerDispatch`, which is
enforced unconditionally by the constructor at `mcu.py:476`. Only custom
endstop wrappers (Beacon, Eddy, load cell) that bypass `MCU_endstop` produce
completions without registering arm_ids. If a future endstop type bypasses
both `MCU_endstop` and arm registration, it would fall into the software-trip
path — the virtual arm's stepper registration (from kinematics, not from the
endstop) would still be correct.

When `drip_move` enters the external-probe path (`active_homing_arms` empty,
`drip_completion` present and not triggered):

1. Create a virtual `BridgeTriggerDispatch` with source kind `Software`
2. Register moving steppers with the virtual dispatch. The virtual arm has no
   real `MCU_endstop` wrapper to call `add_stepper`, so `drip_move` registers
   them explicitly. Stepper selection uses `motion_kinematics.motor_deltas()`
   to map axis deltas to per-motor-slot deltas, then selects steppers from
   slots with nonzero deltas. For CoreXY with a Z-only probe move this yields
   slot 2 (stepper_z). For a hypothetical X-axis external probe move on
   CoreXY, `motor_deltas` would yield nonzero A and B (slots 0 and 1),
   correctly selecting both stepper_x and stepper_y. This matches the same
   kinematic mapping the bridge uses for dispatch.
3. **Resolve the arm's MCU and queue.** `endstop_arm()` requires a bridge MCU
   handle and command queue (motion_bridge.py:328). For bridge-native endstops,
   these come from `MCU_endstop`'s MCU. For the virtual arm, they come from
   the first moving stepper's MCU: `stepper.get_mcu()._bridge_handle` provides
   the handle, and `bridge.alloc_command_queue(handle)` provides the queue.
   All Z steppers are on the same MCU (F446) in the current hardware. If
   moving steppers span multiple bridge MCUs, `drip_move` raises
   `command_error("External probe homing across multiple bridge MCUs is not
   supported")`. This is an explicit unsupported config error, not a silent
   degradation — one virtual arm per MCU would require coordinated deadline
   extension and software trip across MCUs.
4. Register the arm's `arm_id` with `active_homing_arms`
5. `bridge.endstop_arm(mcu_handle, queue, arm_id, arm_clock, software_source,
   stepper_oids)` — MCU accepts, no GPIO polling
6. `bridge.submit_homing_move_async(pos3, speed, [arm_id])` — new non-blocking
   variant. The current `submit_homing_move` returns `PyResult<()>` and the
   caller immediately blocks on `wait_moves()` (bridge.rs:2592,
   motion_toolhead.py:483). The new `_async` variant submits the segment and
   returns immediately, exposing a `segment_completion` reactor completion that
   fires when the homing segment retires (natural end-of-travel, deadline
   expiry, or software trip). The existing blocking `submit_homing_move` +
   `wait_moves` path is unchanged for GPIO homing.
7. Credit-extension loop with two exit conditions, wrapped in try/finally
   for unconditional cleanup (deadline expiry raises, software_trip USB errors,
   etc. must not leak the virtual arm in `active_homing_arms` or bridge
   dispatch maps):
   ```
   bridge_lmt_before = bridge.get_last_move_time()
   try:
       while True:
           drip_completion.wait(waketime=reactor.monotonic() + 0.025)
           if drip_completion.test():
               bridge.software_trip(arm_id)
               break
           if segment_completion.test():
               reason = bridge.get_homing_segment_reason()
               if reason == REASON_DEADLINE_EXPIRED:
                   raise command_error("Homing deadline expired: ...")
               break  # natural no-trigger completion
           bridge.extend_homing_deadline(arm_id)
       bridge.wait_moves()
       # Update toolhead print-time projection so get_last_move_time()
       # returns the correct move_end_print_time for home_wait()
       # (homing.py:151). Mirrors the existing GPIO path at
       # motion_toolhead.py:482-487.
       bridge_lmt_after = bridge.get_last_move_time()
       duration = bridge_lmt_after - bridge_lmt_before
       self._bump_pending_end_time(duration)
   finally:
       active_homing_arms.discard(arm_id)
       bridge.unregister_homing_dispatch(arm_id)
       bridge.endstop_disarm(mcu_handle, queue, arm_id)  # no-op if tripped
   ```

**Two exit conditions:** The loop exits on either probe trigger
(`drip_completion`) or segment retirement (`segment_completion`). If the probe
never triggers, the segment eventually reaches end-of-travel, the MCU retires
it, `segment_completion` fires, and the loop exits. `home_wait()` then returns
0.0 and homing.py raises "No trigger after full movement." If deadline expires
(host stall), same path — MCU terminates the segment, `segment_completion`
fires.

**Completion semantics:** `completion.wait(waketime)` blocks until either the
completion fires or the waketime is reached. `completion.test()` returns `True`
if the completion has fired, regardless of the payload value. The loop uses
`test()` to detect trigger — NOT the return value of `wait()`.

In Klipper's reactor, `MCU_trsync` completes with `False` on normal trigger and
`True` on comms error. `multi_complete` fires on either case. Using `test()`
correctly handles both: normal triggers cause `software_trip`, and comms errors
also cause `software_trip` (stopping motion is always the right response). The
error is then detected later in `home_wait()` when the trsync stop reason is
checked.

**Normal response time from trigger to freeze:**

- Beacon MCU → USB → host: ~1ms
- Completion fires → `wait()` returns → `test()`: <1ms
- `software_trip` → USB → F446: ~1ms
- **Total: ~2-3ms**

At 20 mm/s: 0.06mm overshoot. At 1 mm/s contact probe: 0.003mm.

**Three terminal states, three distinct outcomes:**

1. **Probe triggers** (`drip_completion` fires): `software_trip` freezes motion.
   `home_wait()` returns trigger time. Normal homing success.

2. **Travel completes without trigger** (`segment_completion` fires with
   `REASON_PAST_END_TIME`): Segment retires naturally. `home_wait()` returns
   0.0. `homing.py` raises "No trigger after full movement." Same as mainline.

3. **Deadline expires** (`segment_completion` fires with
   `REASON_DEADLINE_EXPIRED`): Terminal safety fault — MCU freezes permanently.
   The bridge raises a distinct `command_error`:
   `"Homing deadline expired: host failed to extend credit within 50ms"`.
   This is NOT the same as "No trigger" — it indicates a transient host issue
   (GIL stall, load spike, USB latency), not a probe/config problem. User
   re-issues G28. Same class of failure as mainline's trsync comms timeout.

`REASON_DEADLINE_EXPIRED` is a bridge-private reason code defined in
`motion_bridge.py`, NOT in the `MCU_trsync` reason namespace. Klipper's trsync
convention treats reasons >= `REASON_COMMS_TIMEOUT` (4) as failures, and probe
modules already use values 5+ for sensor-specific errors. The bridge reason
lives in a separate enum (`BridgeHomingReason`) and never crosses into legacy
trsync code paths. The credit loop checks for it explicitly:

```
if segment_completion.test():
    reason = bridge.get_homing_segment_reason()
    if reason == REASON_DEADLINE_EXPIRED:
        raise command_error("Homing deadline expired: ...")
    break  # natural no-trigger completion
```

**Host death:** Credit deadline expires after 50ms. MCU freezes permanently.
At 20 mm/s: 1mm worst-case travel. No host to report the error — printer is
already in shutdown. Matches mainline's drip-feed safety model.

**Normal operation margin:** The 25ms extension interval with 50ms deadline
gives 25ms of slack — ample for normal host operation.

**Files:** `klippy/motion_toolhead.py`, `klippy/motion_bridge.py`,
`rust/motion-bridge/src/bridge.rs`, `rust/motion-bridge/src/homing.rs`,
MCU firmware C files

### Piece C: Trigger-time position from curve evaluation

**Mainline:** After a trip, `homing_move` records two positions per stepper:

- `trig_pos = stepper.get_past_mcu_position(trigger_time)` — position at
  trigger instant, looked up from the chelper C step history
- `halt_pos = stepper.get_mcu_position()` — position when motion stopped

The difference (`halt_pos - trig_pos`) is the overshoot. Probes that rely on
the returned trigger position (contact probes, Z switches on a separate MCU)
need `trig_pos` to be accurate at trigger time, not at halt time.

**Problem with naive software trip:** `runtime_software_trip` freezes the curve
evaluator ~2-3ms after the actual trigger (USB relay latency). Snapshotting
position at freeze time gives `halt_pos` but not `trig_pos`. At 20 mm/s, 3ms =
0.06mm error. Beacon eddy-current masks this with its post-homing sample, but
any probe that relies on returned trigger position gets a behavior regression.

**Fix — host-side curve evaluation:** The bridge retains the homing segment's
curve equation on the host (it submitted the curve to the MCU). Given the
trigger time, it evaluates the curve analytically — exact position, no step
history needed. This is more precise than mainline's step-counting (which
rounds to step boundaries).

**Timeline correctness:** Because deadline expiry is terminal (Piece B), the
curve evaluator never pauses and resumes during a homing segment. The
position-vs-time mapping of the curve is never disrupted. Host-side evaluation
at T_trigger is always correct: the curve ran continuously from segment start
until either software_trip, deadline expiry, or natural completion.

**Timebase transform — three clock domains:**

The trigger time crosses three clock domains:

1. **Beacon MCU clock** (ticks) — Beacon's trsync records the trigger. Beacon's
   `home_wait()` converts to Klippy print_time via
   `beacon_mcu.clock_to_print_time()`.
2. **Klippy print_time** (seconds) — shared across MCUs via clock sync. This is
   what `get_past_mcu_position(trigger_time)` receives.
3. **Bridge planner time** (seconds since planner start) — the curve is
   parameterized in this domain. `motion_toolhead._bump_pending_end_time()`
   projects bridge durations onto print_time, but does not retain per-segment
   epoch mappings.

The timebase conversion is **internal to the bridge** — not exported to Python.
`get_homing_step_count_at_time(stepper_name, print_time)` is a self-contained
operation. The bridge retains the following per homing segment:

- **Curve equation** (Bézier control points, parameterized in planner-local
  seconds)
- **Segment planner-local start time** (`t_start` in bridge seconds — assigned
  by the planner when the segment is created)
- **MCU clock sync state** (freq, offset, last_clock from `set_clock_est`) —
  already maintained by the bridge for dispatch

The conversion chain: `print_time` → MCU clock (via clock sync: `mcu_clock =
freq * (print_time + offset)`) → planner-local time (via the dispatch-time
mapping between MCU clock and planner epoch, which the bridge already maintains
for segment `t_start`/`t_end` clock conversion in dispatch.rs:266-267) → curve
parameter → position evaluation.

`set_clock_est` alone is not sufficient — the planner epoch (the relationship
between planner-local seconds and MCU clock at segment dispatch time) is also
needed. The bridge already computes this for normal dispatch (`t_start_clock`,
`t_end_clock` in dispatch.rs) and retains it alongside the homing segment.

No epoch mapping is exposed to or assembled by the Python layer. This avoids
the fragile epoch problem: `toolhead.get_last_move_time()` returns a synthetic
floor (`est + BUFFER_TIME_START`) when no work is pending, and
`estimated_print_time(monotonic())` is wall-clock, not scheduled time. Neither
is a reliable segment-start epoch.

**Flow:**

1. Beacon triggers at T_trigger (Beacon MCU clock → print_time via clock sync)
2. `software_trip` freezes F446 → trip event reports halt step count at T_stop
3. `get_homing_step_count_at_time(stepper_name, T_trigger)` — bridge-internal:
   converts T_trigger to curve parameter via its own clock sync + planner epoch,
   evaluates curve → toolhead position → kinematics forward transform →
   per-stepper motor position → `_mcu_position_offset` + `_step_dist` conversion
   → `trig_step_count`
4. Both `trig_pos` (from curve, step 3) and `halt_pos` (from MCU, step 2) are
   available

**Integration with homing.py:** `StepperPosition.note_home_end(trigger_time)`
calls `stepper.get_past_mcu_position(trigger_time)`. The current bridge
implementation (stepper.py:183) ignores `print_time` entirely — it returns
`_bridge_last_trip_step_count`, the snapshot from the most recent
`kalico_endstop_tripped` event. This is correct for GPIO bridge homing where
the MCU trip clock IS the trigger time (MCU detects GPIO → snapshots instantly).

For software-trip homing, this is a **behavioral replacement**, not a small
extension: `get_past_mcu_position(trigger_time)` must dispatch on whether the
current homing move used software-trip (not merely whether a retained curve
exists — a stale curve from a prior software-trip homing could persist until
replaced, and a subsequent GPIO homing move should use the MCU snapshot, not
the stale curve).

**Discriminator:** The bridge maintains a flag `_software_trip_active`, set to
`True` by the software-trip `drip_move` path, cleared by the GPIO `drip_move`
path and by `set_position` / planner reset. `get_past_mcu_position` checks
this flag:

- **`_software_trip_active` is `True`**: call
  `bridge.get_homing_step_count_at_time(stepper_name, print_time)`. The bridge
  evaluates the retained curve at `print_time` → toolhead position `[x,y,z]` →
  applies kinematics forward transform to get this stepper's motor-space
  position → applies `_mcu_position_offset` and `_step_dist` conversion
  matching `get_mcu_position()` semantics (stepper.py:170-177). For CoreXY,
  the kinematic transform (A=X+Y, B=X-Y) is load-bearing — raw toolhead Z is
  correct only for trivial single-axis steppers.
- **`_software_trip_active` is `False`** (GPIO bridge homing or no homing):
  return `_bridge_last_trip_step_count` as today. The MCU snapshot is
  authoritative because the MCU detected the trip and snapshotted in the same
  interrupt.

**Curve retention lifetime:** The bridge retains the homing curve from
`submit_homing_move_async` until explicitly discarded. In homing.py, the
lifecycle is: `drip_move` (line 146) → `home_wait` for all endstops (lines
152-162) → `flush_step_generation` (line 164) → `note_home_end` which calls
`get_past_mcu_position` for EACH stepper in a loop (lines 165-167). The curve
must survive all per-stepper evaluations. `get_past_mcu_position` evaluates
the curve on each call but does NOT mark it consumed — multiple steppers on the
same rail (e.g., Z, Z1, Z2, Z3) all need access.

**Cleanup cannot piggyback on `flush_step_generation`.** homing.py calls
`flush_step_generation()` at line 164 BEFORE `note_home_end()` at lines
165-167. A "discard on next flush" implementation would drop the curve before
`get_past_mcu_position()` reads it.

**Cleanup cannot happen in `drip_move`.** `drip_move` returns BEFORE
`home_wait` and `note_home_end` run (homing_move sequence: `drip_move` line
146 → `home_wait` lines 152-162 → `flush` line 164 → `note_home_end` lines
165-167).

**Retained curve persists until replaced.** The bridge holds the retained
homing curve indefinitely. It is replaced (not appended) when the next
`submit_homing_move_async` call stores a new curve, or cleared when the
planner is reset via `set_position` / `kalico_stream_open`. No explicit
discard API is needed. The curve is a single optional slot — at most one
homing curve is ever retained. This is safe because homing moves are
serialized (one `homing_move` completes fully before the next begins).

**Beacon post-homing:** `BeaconEndstopWrapper._handle_home_rails_end` does a
post-homing `_sample()` and calls `homing_state.set_homed_position()` to
override the raw stepper position with measured distance. The curve-evaluated
trigger position provides the correct `trig_pos` for the homing state machine;
Beacon's measurement then refines the final Z position. Same two-step process
as mainline.

**Files:** `rust/motion-bridge/src/bridge.rs` (retain + evaluate homing curve),
`klippy/motion_bridge.py` (expose `get_homing_step_count_at_time`),
`klippy/mcu.py` or stepper module (bridge-mode `get_past_mcu_position` override)

## Change Summary

| File | Change |
|------|--------|
| `klippy/mcu.py` | `TriggerDispatch.__init__` always allocates `_trdispatch`; `start()`/`stop()` runtime-gate on primary MCU `_bridge_drives_steppers`; conditional `_trdispatch_mcu` in `MCU_trsync._build_config`; no-op `start()`/`stop()` for bridge-driven MCU_trsync; `_bridge_drives_steppers` flag set by `_register_axis` |
| `klippy/stepper.py` | `get_past_mcu_position` dispatches: retained curve → curve eval, no curve → existing MCU snapshot |
| `klippy/motion_toolhead.py` | `drip_move` credit-extension loop with `drip_completion.wait()` + `test()` for external probes; stepper registration on virtual dispatch |
| `klippy/motion_bridge.py` | Virtual `BridgeTriggerDispatch` with `Software` source kind; `software_trip()` and `extend_homing_deadline()` wrappers; `BridgeHomingReason` enum (private, separate from trsync reasons) |
| `rust/motion-bridge/src/bridge.rs` | `submit_homing_move_async()` (non-blocking, returns segment completion); `software_trip()`, `extend_homing_deadline()` FFI surface; retain homing curve, `get_homing_step_count_at_time` evaluation |
| `rust/motion-bridge/src/homing.rs` | Software source kind in homing state machine; deadline-expired terminal state |
| MCU firmware (C) | `runtime_software_trip` command, `runtime_extend_homing_deadline arm_id=%u` command (fixed 50ms grant), deadline check in curve evaluator tick |

## Unchanged

- `klippy/extras/homing.py`
- `klippy/extras/safe_z_home.py`
- `beacon.py` (third-party, unmodified)
- Bridge-native homing (sensorless X/Y GPIO path)
