# Virtual Endstops and Bridge-Native `[probe]`

## Problem

The homing rework (`klippy/extras/homing.py` + the bridge trip-run machinery)
only supports GPIO endstops declared directly in `[stepper_*]` sections. Any
`endstop_pin` containing `virtual_endstop` is silently skipped, so the axis
ends up with no endstop and `G28` hard-fails. Concretely, the Neptune bench
configures `[stepper_z] endstop_pin: probe:z_virtual_endstop`, and the stock
Kalico `klippy/extras/probe.py` that loads for `[probe]` is built entirely on
the deleted legacy homing contract (`MCU_endstop`, trsync, `probing_move`), so
it cannot work under the bridge.

"Virtual endstop" in mainline never meant "not a GPIO." It means: some other
config section owns the trigger pin and gets a say in the homing move. The
probe owns its switch pin and supplies the trigger height (`z_offset`); a TMC
driver owns its diag pin and flips StallGuard registers around the move. Both
map directly onto the bridge's existing generic endstop machinery.

## What the bridge already provides

- Firmware `src/endstop.c`: `config_endstop oid endstop_id pin pull_up invert`
  is fully generic — any pin, any `endstop_id` byte. Trip emits
  `kalico_endstop_tripped {endstop_id, trip_clock}`.
- `bridge.rs::home_axis_start(axis, direction, speed, max_travel, endstop_id,
  endstop_mcu)` plans a drip run and matches the trip on
  `(endstop_id, endstop_mcu)` as opaque tokens (`bridge.rs:3001`).
- Position reconstruction evaluates the planned curve at `trip_clock`, giving
  the exact trigger position plus the stop overshoot (`final_pos - trip_pos`).

**No Rust or firmware changes are required.** Both deliverables are host-side
Python.

## Scope

In scope:
- Deliverable 1: generic virtual-endstop resolution in `homing.py`, plus
  post-home retract and shared trip-move primitive.
- Deliverable 2: full rewrite of `klippy/extras/probe.py` as the first
  virtual-endstop provider, with `PROBE`, `QUERY_PROBE`, `PROBE_ACCURACY`.

Out of scope (follow-ups):
- bed_mesh / z_tilt / quad_gantry_level probe sessions and the ten extras that
  `from . import probe` (they fail loudly at import if configured).
- TMC sensorless provider (StallGuard hooks, `min_home_dist` semantics).
- BLTouch / dockable / smart_effector (`activate_gcode`, stow state).
- `PROBE_CALIBRATE`, `Z_OFFSET_APPLY_PROBE` (manual-probe helper).
- External probes on non-bridge MCUs (Beacon et al.) — separate design,
  `external-probe-homing.md`.

## Deliverable 1: virtual endstop support in `homing.py`

### Resolution

For each `[stepper_*]` `endstop_pin`, `homing.py` parses the pin through the
pins registry (`ppins.parse_pin`). Two cases on the resolved chip:

- **Bridge MCU** → today's GPIO path, unchanged: build a `config_endstop`
  entry directly.
- **Virtual endstop provider** → the chip object must implement
  `setup_bridge_endstop(pin_params, axis)`, returning its `BridgeEndstop`.
  The provider validates its own pin-name string (e.g. probe accepts only
  `z_virtual_endstop` on the Z axis) and rejects `^`/`!` modifiers on the
  virtual pin — those belong on the provider's own pin option.

A chip that is neither a bridge MCU nor a provider, or a provider chip whose
section is missing, is a hard config error. Ordering is safe: all config
sections (and thus provider chip registration and entry building) load
before the toolhead modules that load `homing`.

Provider-backed axes register with `query_endstops` under their axis name,
same as GPIO axes — `QUERY_ENDSTOPS` on the Neptune shows `x`, `y`, `z`
(the probe pin) and `probe` (registered by the probe itself).

Provider interface, duck-typed like mainline's `setup_pin`:

- `setup_bridge_endstop(pin_params, axis)` — required. Validates the request and
  returns the provider's already-built `BridgeEndstop`; it does not create it.
  The endstop must exist independently of homing, because a provider may be
  configured without backing any axis (e.g. `[probe]` alongside a GPIO Z
  endstop, probe used only for `PROBE`/`PROBE_ACCURACY`).
- `get_position_endstop()` — optional trigger-height override. Probe returns
  `z_offset`. When an override exists and the stepper section *also* sets
  `position_endstop`, that is a hard config error (mainline silently ignores
  it; current `stepper.py` silently defaults it to `position_min` — both are
  traps).
- `trip_move_begin(entry)` / `trip_move_end(entry)` — optional hooks invoked
  around the trip run. Probe implements neither; this is the seam for TMC
  StallGuard setup and, later, probe `activate_gcode`.

### Endstop-id allocation

Ids 0–2 stay statically reserved for the axes (as today — keeps trip logs
readable). Providers draw from a shared allocator starting at 3. The
allocator lives with the shared entry-builder helper extracted from
`homing.py` — not inside `Homing` — because providers build their entries at
their own config-load time, before `homing` loads. The bridge treats ids as
opaque, so nothing changes below Python.

### Shared trip-move primitive

The arm/start/poll/abort loop inside `_home_axis` is extracted into a public
method on `Homing` that both `G28` and the probe commands call:

1. Pre-checks: endstop not already triggered (raw `endstop_query_state`
   read), `toolhead.wait_moves()`, motors enabled.
2. Provider `trip_move_begin` hook (if any).
3. Arm `query_endstop` polling, `bridge.home_axis_start(...)`.
4. Poll loop with a **computed deadline**: `max_travel / speed + margin`
   instead of the current flat 30 s. On travel exhaustion without a trip,
   nothing arrives on the result channel, so the deadline is the only
   backstop — the error message names the real failure ("failed to trigger
   after full travel").
5. Provider `trip_move_end` hook (always, including error paths).
6. **No-movement check**: if `trip_pos ≈ start_pos` (within epsilon), raise
   "endstop/probe triggered prior to movement" — catches a stuck trigger that
   raced past the pre-check.
7. Return `(trip_pos, final_pos)`.

Callers differ only in what they do with the result:
- `G28`: `newpos[axis] = trigger_height + overshoot`,
  `toolhead.set_position(newpos, homing_axes=[axis])`, where
  `trigger_height` is the provider override if present, else
  `hi.position_endstop`.
- `PROBE`: resync toolhead to `final_pos` (frame already established, no
  `homing_axes`), record `trip_pos[2]` as the measurement.

### Post-home retract

After setting position, `G28` moves the axis off the endstop by
`homing_retract_dist` at `homing_retract_speed` (both already parsed by
`stepper.py`; default 5.0 mm). Applies to **all** axes — GPIO and virtual
alike; `homing_retract_dist: 0` keeps park-on-switch behavior. This restores
mainline ergonomics (nozzle not left touching the bed after a probe home,
switches released after XY home). The legacy second slow homing pass is
deliberately **not** restored: its rationale is trigger-timing slop, and the
bridge reconstructs the trigger position analytically from `trip_clock`.

## Deliverable 2: `klippy/extras/probe.py` rewrite

Full replacement of the 1011-line legacy module with a bridge-native
`PrinterProbe` (~250–300 lines), in the style of the new `homing.py`.

### Config surface

`pin` (required), `z_offset` (required), `x_offset`/`y_offset` (default 0),
`speed` (default 5), `lift_speed` (default = `speed`), `samples` (default 1),
`sample_retract_dist` (default 2), `samples_result` (`average`/`median`,
default `average`), `samples_tolerance` (default 0.1),
`samples_tolerance_retries` (default 0). Nothing else — klippy's
unused-option check rejects legacy options (`activate_gcode`,
`deactivate_on_each_sample`, …) loudly at boot.

### Registration

- Parses `pin` (invert/pullup allowed here), requires the pin's MCU to be
  bridge-attached, builds its endstop entry via the shared entry-building
  helper extracted from `homing.py`.
- Registers as pins chip `probe` so `endstop_pin: probe:z_virtual_endstop`
  resolves through the registry — the same indirection that lets `[bltouch]`
  and friends answer to the chip name `probe` later.
- `setup_bridge_endstop` accepts only pin name `z_virtual_endstop`; usable
  only on the Z axis (anything else is a config error).
- Registers with `query_endstops` as `probe`.

### Commands

- `QUERY_PROBE` — raw pin read, reports `probe: open` / `probe: TRIGGERED`,
  stores `last_query`.
- `PROBE` — multi-sample descent loop, reports `probe at X,Y is z=…`, stores
  `last_z_result`.
- `PROBE_ACCURACY` — N samples at the current position (`SAMPLES` default
  10, matching mainline), reports max/min/range/average/median/stddev.
- All accept the standard per-call overrides (`PROBE_SPEED`, `SAMPLES`,
  `SAMPLE_RETRACT_DIST`, `SAMPLES_TOLERANCE`, `SAMPLES_TOLERANCE_RETRIES`,
  `SAMPLES_RESULT`, `LIFT_SPEED`).

### One probe sample

1. Pre-checks: Z homed (else "Must home before probe"), probe not triggered.
2. Trip move down via the shared primitive:
   `max_travel = current_z − position_min`.
3. Resync toolhead Z to reconstructed `final_pos`; measurement is
   `trip_pos[2]`.
4. Retract to `trip_z + sample_retract_dist` at `lift_speed` (plain toolhead
   move).

Multi-sample: repeat; if `max − min > samples_tolerance`, retry the whole
batch up to `samples_tolerance_retries` times, then hard error. Result is
average or median per `samples_result`.

### Status

`get_status` exposes `name`, `last_query`, `last_z_result`. `get_offsets()`
and `get_position_endstop()` exposed for `homing.py` now and bed_mesh later.

## Error handling (all hard errors, no recovery)

- Endstop/probe already triggered before a trip move.
- Trigger prior to movement (stuck/miswired trigger).
- No trigger within full travel (computed deadline).
- `samples_tolerance` exceeded after retries.
- `[probe]` pin on a non-bridge MCU.
- Virtual endstop chip missing, not a provider, wrong pin name, or carrying
  `^`/`!` modifiers.
- `position_endstop` set alongside a provider trigger-height override.
- `probe:z_virtual_endstop` on a non-Z axis.
- `PROBE` while Z unhomed.

## Known divergences from mainline (accepted)

- **No trigger debounce.** Legacy firmware requires 4 consecutive active
  samples 15 µs apart; `src/endstop.c` trips on a single read of a 1 ms poll.
  XY homing already runs this way on the bench without false trips. Revisit
  in firmware only if noise shows up. Position quantization from the 1 ms
  poll is ~5 µm at 5 mm/s probing speed.
- **Single-pass homing** (no retract + slow second pass) — see Deliverable 1;
  trigger position is reconstructed analytically.
- **Host-visible stop latency does not affect accuracy.** The trip halts step
  execution MCU-side (same path XY homing uses); any physical overtravel is
  measured as `final_pos − trip_pos` overshoot and accounted for in the
  position set afterward.

## Testing

- Unit tests (separate files, per repo convention) for the pure logic:
  sample aggregation (average/median), tolerance/retry behavior, virtual
  endstop string validation, allocator.
- **kalico-sim**: Neptune-shaped config (`[probe]` +
  `probe:z_virtual_endstop`) running `G28`, `PROBE`, `PROBE_ACCURACY`,
  `QUERY_PROBE`; a `[probe]`-with-GPIO-Z-endstop config (probe used for
  `PROBE` only, no virtual endstop); failure cases: missing `[probe]`,
  `PROBE` while unhomed, `position_endstop` conflict, retract behavior on
  all axes.
- **Bench**: flash the Neptune, confirm clean boot, then (per-command
  user go-ahead) `G28`, `QUERY_PROBE`, `PROBE_ACCURACY`.
