# [safe_z_home] support on the bridge-native homing stack

Date: 2026-06-09
Branch: `safe-z-home`

## Problem

The Neptune bench config carries `[safe_z_home]` + `[probe]` with
`stepper_z` homed via `probe:z_virtual_endstop`. Klippy fails at config
parse on every start:

```
klippy.pins.error: Unknown pin chip name 'probe'
  safe_z_home.py:24 load_object("homing")
  -> homing.py:24 ppins.parse_pin(endstop_pin)
```

Klippy loads config sections in file order. `[safe_z_home]` appears
before `[probe]` and force-loads `homing` at its own config time. The
rewritten `Homing.__init__` (`klippy/extras/homing.py`) parses every
`endstop_pin` at construction, but the `probe` pin chip only registers
when `[probe]` loads — later in the file. Net effect: the bench never
boots, FIRMWARE_RESTART restarts into the same error, and moonraker
404s `/printer/info`.

Stock Klipper never had this failure: its `PrinterHoming` constructor
touches no pins; endstops resolve when the toolhead/kinematics load,
after all extras. Our rewrite made `Homing` sensitive to load order.
`homing_override.py:18` carries the identical landmine.

## Decision summary

1. **Homing defers endstop resolution to toolhead-load time**
   (Approach A below). Fixes the bug for every early loader, keeps
   fail-loudly-at-startup, keeps MCU OID allocation inside the config
   phase.
2. **Legacy `klippy/extras/safe_z_home.py` is kept unchanged.** Every
   API it touches exists on `MotionToolhead`/`BridgeKinematics` with
   matching semantics. If sim verification surfaces an incompatibility,
   rewrite it then — not preemptively.
3. **Regression coverage via a kalico-sim variant** that boots the
   exact failing config shape ([safe_z_home] before [probe]).

Servo-Z axes are explicitly out of scope for this work.

## Approach A: deferred endstop resolution in Homing

`Homing.__init__` keeps:

- G28 / `_HOME_TEST` registration (so safe_z_home's wrap chain —
  re-register G28, call previous handler — keeps working regardless of
  when homing constructs),
- a stashed reference to its config object.

It stops parsing endstop pins. A new `resolve_endstops()` method builds
`_axes` (pin parse, provider lookup via `setup_bridge_endstop`,
`BridgeEndstop` construction for direct GPIO pins, `query_endstops`
registration). `BridgeKinematics.__init__` — which constructs at
toolhead load, deterministically after all config sections — calls
`load_object(config, "homing")` followed by `resolve_endstops()`.

Properties preserved:

- **Fail loudly at startup.** Resolution still runs during the config
  phase (toolhead load is part of `_read_config`), so a bad
  `endstop_pin` is still a config error that aborts startup, not a
  deferred runtime surprise.
- **OID allocation stays in the config phase.** `BridgeEndstop`
  calls `mcu.create_oid()` + `register_config_callback`; both must
  happen before MCU connect. Toolhead load satisfies that.
- **G28 ordering contract.** `safe_z_home` (and `homing_override`)
  call `load_object("homing")` before wrapping G28, guaranteeing the
  chain order independent of section order.

Guard: calling G28/`_HOME_TEST`/`trip_move` before `resolve_endstops()`
has run cannot happen in practice (gcode runs post-ready), but
`resolve_endstops()` has exactly one caller (`BridgeKinematics`); a
second call raises, per the fail-loudly default.

### Rejected alternatives

- **Lazy chip loading in the pins registry** (unknown chip → try
  `load_object` of a same-named section): magic, and wrong for chips
  whose pin-chip name differs from their section name
  (`tmc2209_stepper_x:virtual_endstop` vs `[tmc2209 stepper_x]`).
- **Patch safe_z_home / homing_override to defer their homing load**:
  leaves the trap armed for the next extra that loads homing at config
  time.

## safe_z_home compatibility trace (why the legacy file stays)

| Legacy call | New-stack implementation |
|---|---|
| `toolhead.manual_move([x, y, z], speed)` | base `ToolHead.manual_move` → `MotionToolhead.move` (bridge submit) |
| `toolhead.set_position(pos, homing_axes=[2])` | `BridgeKinematics.set_position` — int-axis iterable, sets `limits` from rail range |
| `kin.get_status(t)["homed_axes"]` | `BridgeKinematics.get_status` returns the string |
| `kin.clear_homing_state((2,))` | `BridgeKinematics.clear_homing_state` — int-axis iterable |
| G28 re-register + chain to previous handler | new `Homing` registers G28 the standard way; chain works |
| `lookup_object("dockable_probe")` | optional, absent → `None` path |
| `config.getsection("stepper_z")` | present on the in-scope (stepper-Z) configs |

`z_calibration.py` reads `safe_z_home.home_x_pos` / `home_y_pos` —
preserved by keeping the legacy class.

## Verification

1. **Sim regression (the bug):** new `tools/kalico-sim/runner.py`
   variant (alongside `--probe-test`) boots a Neptune-shaped config —
   `endstop_pin: probe:z_virtual_endstop`, `[safe_z_home]` ordered
   *before* `[probe]` — and asserts clean klippy startup. This config
   fails to parse on current main.
2. **Sim behavior:** within the harness's known limits (full-motion
   G28 currently faults with `PieceStartInPast` in Docker —
   pre-existing), assert what boot/config/query coverage allows:
   G28 command accepted, safe_z_home object present with correct
   attributes, homing/probe endstop registration intact
   (`QUERY_ENDSTOPS` / `QUERY_PROBE`).
3. **Existing suites stay green:** `cargo nextest run` (rust/),
   sim_klippy pytest suite, existing `--probe-test` variants.
4. **Bench:** flash the Neptune on this branch, confirm clean boot and
   FIRMWARE_RESTART, then — with explicit per-command permission —
   G28 through safe_z_home on real hardware.

## Out of scope

- Servo-Z (`[servo_z]`) + safe_z_home: legacy file reads
  `config.getsection("stepper_z")`; revisit when servo-Z printers need
  safe_z_home.
- `homing_override` end-to-end support: Approach A removes its
  load-order landmine as a side effect, but porting/validating that
  module is its own task.
- The Docker harness `PieceStartInPast` full-motion fault:
  pre-existing, tracked separately.
