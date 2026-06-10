# Servo axis homing — EtherCAT endpoint joins the G28 flow

## Problem

The EtherCAT servo axis cannot home, and its presence breaks homing for
every other axis:

- `klippy/extras/homing.py` builds endstop entries only from
  `[stepper_<axis>]` sections, so a `[servo_<axis>]` axis has no endstop
  entry — `G28` on it is rejected with "axis has no endstop", and there is
  no config surface to even name the endstop pin.
- `handle_endstop_trip` (`rust/motion-bridge/src/bridge.rs`) broadcasts a
  `Stop` to every MCU in the homing run's `all_axis_keys` via `host_io` —
  EtherCAT nodes have no `host_io`, so any trip with a servo configured
  fails with "Stop: no host_io for mcu N". The physical stop on serial MCUs
  still happens, but G28 errors and never sets position. With a servo in
  the config, **no** axis can home.
- The endpoint wire protocol has no `Stop` command at all (only Identify /
  QueryRuntimeCaps / ClaimHandshake / SetTorque / PushPieces), so even with
  routing fixed there is nothing to call: no piece discard, no
  `discard_clock` for final-position reconstruction.
- `ServoRail.get_homing_info()` is a zeros stub (speed 0, endstop 0), and
  the pre-home enable step iterates `rail.get_steppers()` — empty for a
  servo — so the drive would still be parked when the first drip piece
  arrives and the endpoint faults `piece-while-parked`.

This is the "servo homing (Part A boundary)" item the 2026-06-06 lazy-enable
spec left out of scope.

Bench target: Neptune 3 Pro, servo on **X** (endstop `PA13` on the F401,
`position_endstop: -6.0` = `position_min`, so negative homing direction),
steppers on Y/Z.

What is already transport-agnostic and needs no change:

- Homing trajectory recording for trip reconstruction happens at dispatch
  (`bridge.rs` dispatch callback), keyed by `AxisKey` — EtherCAT axes are
  recorded like any other.
- Drip-cohort metering feeds off per-axis `retired_counts`, which the
  endpoint's `StatusHeartbeat` already carries.
- Clock mapping for the endpoint's nanosecond domain is registered at
  `init_planner` (`set_clock_est_from_sample`, freq 1e9), so
  `reconstruct_axis_position` handles a trip clock from the F401 and a
  discard clock from the endpoint without modification.

## Design

### 1. Endpoint: `Stop` command (`rust/kalico-ethercat-rt`)

The endpoint implements the existing `kalico-protocol`
`Stop` / `StopResponse { result: i32, discard_clock: u64 }` pair —
the same contract serial MCUs answer:

- discard all queued/unsampled pieces;
- hold position at the last sampled target (enabled) or keep
  target-tracks-actual (parked);
- reply `StopResponse { result: 0, discard_clock: now_ns }` where `now_ns`
  is the endpoint clock (`CLOCK_MONOTONIC_RAW` ns — the PushPieces time
  domain).

A `Stop` while parked or idle is valid and trivially succeeds (empty
ring). That property is what restores `G28` for the *other* axes while a
servo is configured: the trip broadcast reaches every node and every node
can answer.

The stub binary implements the same command with simulated state.

### 2. Bridge: per-transport Stop broadcast (`rust/motion-bridge`)

`handle_endstop_trip`'s stop loop gains the Serial/EtherCAT split the pump
already uses (`McuTransport`): serial MCUs keep `host_io.kalico_call`,
EtherCAT nodes send the same `Stop` over their `UnixNativeConn`
request-reply (the `query_ethercat_runtime_caps` call pattern, bounded
timeout). A node with neither transport is a loud error, as today.
`discard_clock` handling is unchanged — it is already keyed off
`run.axis_key.mcu_id`, whichever transport produced it.

### 3. klippy: servo axes in the homing flow (`klippy/extras/homing.py`)

- The endstop-entry loop reads `[servo_<axis>]` sections alongside
  `[stepper_<axis>]`, picking up `endstop_pin` (the pin lives on a serial
  MCU; `parse_pin` resolves the chip as usual).
- The pre-home enable step enables the rail's registered motor name when
  the rail has no steppers — `stepper_enable.motor_debug_enable("servo_x")`
  — which runs the SetTorque ladder synchronously before the first drip
  piece (registration exists from the lazy-enable work). Stepper rails
  keep the existing per-stepper loop.

### 4. klippy: real homing config (`klippy/extras/servo_axis.py`)

`[servo_<axis>]` gains:

- `endstop_pin` (consumed by homing.py, as for steppers);
- `position_endstop` (float, required when `endstop_pin` is set);
- `homing_speed` (float, default 5.0, above 0).

Homing direction is inferred the stock way: endstop at `position_min` →
negative, at `position_max` → positive, anything else is a config error
(no `homing_positive_dir` override until something needs it).
`get_homing_info()` returns these values; the bridge homing flow consumes
only `speed`, `position_endstop`, `positive_dir`, and the position range —
retract/second-pass fields stay at their inert defaults.

## Out of scope

- Torque limiting during homing (drive-side `0x6072` cap is the bench
  protection; homing-scoped SDO writes are future work if ever wanted).
- Sensorless/probe homing for servo axes; retract + second-pass homing
  (the bridge homing flow has none for steppers either).
- Multi-slave EtherCAT.

## Testing

- **Endpoint (stub) integration** (`rust/kalico-ethercat-rt/tests/`):
  Stop mid-stream discards queued pieces and returns a sane
  `discard_clock`; Stop while parked succeeds; piece pushed after Stop
  re-anchors cleanly (existing CountMap behavior).
- **Bridge unit tests**: trip broadcast routes Stop per transport; an
  EtherCAT node without a conn is a loud error; mixed serial+EtherCAT run
  completes and picks the discard clock from the moving axis.
- **klippy tests**: servo homing config surface parses (endstop entry
  created from `[servo_x]`, direction inference, error on
  `position_endstop` not at a range edge); pre-home enable fires for a
  steppers-empty rail.
- **Hardware (Neptune)**: flash latest branch via the flash script; with
  servo on X — `G28 Y` works with the servo present (the regression case),
  then supervised `G28 X` into `PA13`: torque on, drip, trip, stop,
  position set from trip reconstruction.
