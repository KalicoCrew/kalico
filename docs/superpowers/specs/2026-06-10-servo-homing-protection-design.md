# Servo homing protection — drive-side following-error / torque limits

## Problem

The A6-EC ships with no usable runaway protection: `6065h` (excessive
position deviation threshold) defaults to 3,145,728 counts — ~960 mm of
travel at the Neptune's 3276.8 counts/mm — and `6072h` (max torque)
defaults to 3000 (300% of rated). If the endstop fails during homing, the
axis rams the frame at full torque (~150 N of belt tension from the 100 W
motor through a 20T GT2 pulley) until something gives.

Wanted: homing-scoped limits, configured in `[servo_*]`, independent of
any printing-time limits — and when a limit trips, homing stops with a
G28 error naming the cause, not a whole-session shutdown.

Manual facts (A6-EC):

- `6065h` excessive position deviation threshold, RW U32, position units
  (encoder counts), default 3,145,728.
- `6066h` following error timeout, RW U16 (ms), default 0 = fault as soon
  as the window is exceeded.
- `6072h` max torque, RW U16, units of 0.1% of rated, range 0–4000,
  default 3000. (`60E0h`/`60E1h` directional limits exist; not used here.)
- The deviation fault is `Er47.0` "Excessive position deviation",
  **resettable** — the existing CiA 402 enable ladder already pulses fault
  reset, so a faulted drive recovers on the next enable.

## Config (`[servo_*]`)

```ini
homing_following_error: 2.5   # mm; default 2.5
homing_max_torque: 50         # % of rated; default 50
#following_error: ...         # mm, optional session-wide (printing); unset = drive default
#max_torque: ...              # % rated, optional session-wide; unset = drive default
```

Conversions: mm → counts via the rail's `counts_per_mm`; % → 0.1% units.
Defaults make homing protection always-on for servo axes. The session-wide
pair is optional and independent — printing limits are never coupled to
homing limits.

## Design

### Endpoint: SDO limit writes + two new commands

- **Bringup** (`bench/libecrt.c`, beside the existing CSP/DC SDO setup):
  SDO-read `6065h`/`6066h`/`6072h`, log the drive's values, write
  `6066h = 0`. If session-wide values were passed (new CLI args, plumbed
  like `--counts-per-mm`), write them to `6065h`/`6072h`. Whatever is in
  effect after bringup is remembered as the **run values**. A NAKed SDO
  write fails bringup loudly (named claim error, like every other bringup
  failure).
- **`SetDriveLimits { following_error_counts: u32, max_torque_tenth_pct: u16 }`**
  → two SDO writes, correlated response with result. Sent by the host on
  homing entry.
- **`RestoreDriveLimits {}`** → endpoint rewrites its remembered run
  values. Sent by the host on homing exit (success or failure). No value
  plumbing back to the host; the endpoint owns the memory of what "run"
  means.
- The stub implements both commands with simulated state, plus a
  `--drive-fault-after-pieces N` switch that simulates `Er47.0` after N
  sampled pieces (test hook for the fault flow below).

### Drive fault becomes a reported state, not process death

Today any drive fault (`err != 0` in telemetry) makes the endpoint disable
and `exit(1)`; the bridge's supervision path (`EXIT_ON_FAULT`) then ends
the whole session. That is the wrong shape for a protection limit doing
its job during homing. New policy, split by fault class:

- **Drive faults** (`ec_rt_get_error_code() != 0`, e.g. `Er47.0`): the
  drive has already de-energized itself. The endpoint discards the ring,
  parks the torque gate, latches the drive's error code into the
  `StatusHeartbeat.fault_code` field (today always 0), and **keeps
  serving**. The session stays alive.
- **Walker faults** (`PieceStartInPast` — host-bug class) keep the
  existing latch-heartbeat-and-exit behavior, unchanged.

### Bridge: route the fault by context

The bridge's EtherCAT heartbeat handler gains fault_code awareness:

- Heartbeat with `fault_code != 0` **while a homing run is active on that
  MCU** → send `Err` into the homing run's notify channel:
  `"drive fault 0x%04x during homing — following-error/torque limit
  exceeded (endstop failure?)"`. `homing.py`'s poll loop already turns
  channel errors into a G28 error + `home_abort`; the printer stays up.
- Heartbeat with `fault_code != 0` with **no homing run active** → the
  existing fatal path (a drive fault mid-print is a real emergency;
  session shutdown stays correct).

Recovery after a homing fault: the next servo enable (any move or G28)
runs the CiA 402 ladder, which already pulses fault reset — no
FIRMWARE_RESTART needed.

### Host flow (`klippy/extras/homing.py` + bridge glue)

For a servo rail, `_home_axis` wraps the trip move:

1. send `SetDriveLimits(homing values)` (FIFO socket ordering puts it
   ahead of the homing pieces; SDO writes are valid in any drive state);
2. run the existing trip move;
3. in a `finally`: send `RestoreDriveLimits`.

Stepper rails are untouched. The bridge exposes the two commands to
klippy mirroring `set_torque` (handle-addressed, error on non-EtherCAT
MCUs).

## Out of scope

- `60E0h`/`60E1h` directional torque limits.
- Writing drive EEPROM (`1010h`) — all writes are session-volatile by
  design.
- Endpoint-side per-cycle following-error watchdog (the drive-side window
  is the autonomous layer; duplicate only if `Er47.0` proves too cryptic).
- Any change to stepper-axis homing or the MCU fault paths.

## Testing

- **Wire**: round-trip decode/encode tests for both new commands.
- **Stub integration**: SetDriveLimits → RestoreDriveLimits round trip;
  `--drive-fault-after-pieces` → heartbeat carries the fault code, stub
  parks and keeps serving (no exit), subsequent enable succeeds
  (simulated fault reset).
- **Bridge unit**: fault_code routing — homing-active → notify channel
  Err; idle → fatal path. (Closure-parameterized like `broadcast_stop`.)
- **klippy**: servo `_home_axis` sends limits/restore around the trip
  move including the failure path; config parsing/conversion tests
  (mm→counts, %→0.1%).
- **Hardware (Neptune, user-gated)**: normal G28 X with limits in place;
  then a deliberate no-endstop homing (small `_HOME_TEST MAX_TRAVEL`
  into open travel) — axis must stop within ~homing_following_error of
  contact at ≤homing_max_torque, G28 errors naming `Er47.0`, next G28
  recovers without restart.
