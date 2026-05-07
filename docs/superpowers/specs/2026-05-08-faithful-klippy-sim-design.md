# Faithful klippy-in-loop simulator — design

**Status:** approved 2026-05-08
**Replaces:** the synthetic single-MCU sim under `tools/sim_klippy/` (kept; this design extends it)
**Implements:** "catch unknown-unknown regressions before they brick the printer"

## Goal

Today's `tools/sim_klippy/` runs one Linux `klipper.elf` with a synthetic
3-axis printer.cfg. It catches motion-bridge / runtime regressions, but
nothing involving the user's actual printer.cfg, third-party plugins, or
multi-MCU topology. Two recent failures (the `clear_homing_state("z")`
regression from beacon's compat layer, the `is_active()` guard going
stale in bridge mode and breaking FIRMWARE_RESTART) sailed past every
existing test and got caught only by bricking the printer.

The faithful sim's target: run the same printer.cfg, the same set of
plugins, the same MCU topology, and the same gcode flow as the user's
Trident, end-to-end — boot to ready, G28 X/Y/Z, plus a small print —
all in `tools/sim_klippy/`. If the sim passes, the bricking-class bugs
of the kind we just hit don't reach hardware.

## Success bar

A test passes if the sim runs the following sequence cleanly:

1. **Boot** — both MCUs connect, beacon connects, all plugins (`[beacon]`,
   `[autotune_tmc]`, `[motors_sync]`, `[z_tilt_ng]`, `[chopper_tune]`,
   `[bed_mesh]`, KAMP macros, …) register without error, klippy reaches
   "Printer is ready". No tracebacks. No `MCU '*' shutdown`.
2. **G28** — `G28 X`, `G28 Y`, `G28 Z`, `G28` (all axes). Each axis
   homes via sensorless StallGuard (X/Y) or beacon-proximity (Z),
   `homed_axes` reflects after each, no MCU shutdowns. `M84` clears
   homed state.
3. **Small print** — a 30-line slicer chunk: heat bed/hotend, contact
   beacon calibration, bed-mesh adaptive (KAMP), 5 perimeter G1s, 5
   infill G1s, retract, M104 S0, M140 S0, M84. Final toolhead position
   matches expected; mesh has 9 points; heater feedback held within
   ±2°C of target.

## Non-goals

- Real-time deadline fidelity (sim isn't deadline-faithful)
- Charge-pump UV / USB enumeration / mechanical resonance — anything
  below the chip-protocol boundary
- CI integration (Mac Docker only for now)
- Auto-refresh of the vendored printer snapshot — manual rsync if needed

## Architecture overview

```
┌────────── Mac host (Docker) ──────────────────────────────────────┐
│                                                                   │
│  ┌─── orchestrator (Python, pytest) ───┐                          │
│  │  launch_mcus.py                     │                          │
│  │  tmc5160_emulator.py × 4 chips      │  on SPI bus to mcu_main  │
│  │  tmc2209_emulator.py × 3 chips      │  on UART bus to mcu_btm  │
│  │  beacon_serial_stub.py              │  on /tmp/klipper_sim_bcn │
│  │  adc_stub.py (heaters/thermistors)  │  via runtime_sim_adc_set │
│  │  sensorless_trigger.py              │  reads step counts,      │
│  │                                     │  drives DIAG via         │
│  │                                     │  runtime_sim_endstop_set │
│  └──────────────┬──────────────────────┘                          │
│                 │ stdin/stdout pipes + sim sockets                │
│                 │                                                 │
│  ┌──────────────▼─────────┐  ┌──────────────────────┐             │
│  │ klipper-h7-sim.elf      │  │ klipper-f4-sim.elf  │             │
│  │ (MACH_LINUX +           │  │ (MACH_LINUX, no     │             │
│  │  CONFIG_KALICO_RUNTIME) │  │  KALICO_RUNTIME)    │             │
│  │ /tmp/klipper_sim_h7     │  │ /tmp/klipper_sim_f4 │             │
│  └──────────────┬──────────┘  └──────────┬──────────┘             │
│                 │                        │                        │
│                 └─── motion bridge ──────┘                        │
│                            │                                      │
│  ┌────────────────────────▼────────────────────────────┐          │
│  │ klippy (Python)                                     │          │
│  │ • printer.cfg from printer_real/config/             │          │
│  │ • extras/ — ours + printer_real/third_party_repos/  │          │
│  │ • motion_bridge attach_serial to both MCUs          │          │
│  └─────────────────────────────────────────────────────┘          │
│                                                                   │
└───────────────────────────────────────────────────────────────────┘
```

## Components

### 1. Vendored snapshot (`tools/sim_klippy/printer_real/`)

Already in place. Real `printer.cfg` + all `.cfg` includes, plus
`third_party_repos/` with beacon, motors-sync, KAMP, mainsail-config,
moonraker-timelapse, chopper-resonance-tuner. Provenance + refresh
procedure in `printer_real/README.md`.

### 2. Multi-MCU launcher (`orchestrator/launch_mcus.py`)

Spawns two Linux `klipper.elf` processes:

- `out/klipper-h7-sim.elf` (MACH_LINUX + KALICO_RUNTIME=y) listening on
  `/tmp/klipper_sim_h7` — drives motion, runs the kalico runtime.
- `out/klipper-f4-sim.elf` (MACH_LINUX, KALICO_RUNTIME unset) listening
  on `/tmp/klipper_sim_f4` — hosts the F446's command surface (TMC UART,
  thermistors, fans, heaters).

Both binaries are produced by the existing `make` flow with two
`.config` snapshots committed under `tools/sim_klippy/configs/`. The
launcher waits for both PTY symlinks before returning. Per-MCU logs
under `.local-logs/h7.log` / `.local-logs/f4.log`.

### 3. TMC chip emulators

Pure-Python, run inside the orchestrator process. Build on PyTrinamic
(MIT, pip-installable) for register addresses + bitfield definitions +
power-on-reset values.

**Wire routing.** Linux `klipper.elf`'s SPI shim opens `/dev/spidev*`;
UART shim opens character devices. We extend the existing
`runtime_sim_*` command surface with two new commands so the firmware
forwards SPI/UART transfers to the orchestrator instead of opening real
device files:

- `runtime_sim_route_spi bus=%c chip=%s` — bind a virtual SPI bus to a
  Python emulator instance; firmware forwards subsequent
  `spi_transfer` payloads through `runtime_sim_chip_io` events.
- `runtime_sim_route_tmcuart oid=%c chip=%s` — same idea for tmcuart.

The orchestrator parses each `runtime_sim_chip_io` event, dispatches
to the right `TMC5160Emulator` or `TMC2209Emulator` instance, and
sends back the response bytes through the same channel.

**TMC5160 emulator behavior** (`orchestrator/tmc5160_emulator.py`,
~300 lines):

- Per-chip `dict[u8 reg, u32 value]`, init from PyTrinamic POR table
- 5-byte SPI datagram framer (datasheet §5.1): byte 0 = R/W bit + reg
  addr, bytes 1–4 = data; reply = previous status byte + previous-read
  data
- Side-effect registers:
  - `GSTAT` (0x01) — clear-on-read; after read, mask to 0
  - `DRV_STATUS` (0x6F) — derived from sensorless model (§6); exposes
    `SG_RESULT`, `s2gA`/`s2gB` flags
  - `IOIN` (0x04) — echoes firmware-driven pin levels
  - `RAMP_STAT`, `TSTEP` — derived from motion model
  - `GLOBALSCALER` (0x0B) — clamp writes to [32, 255]
  - `IHOLD_IRUN` — clamp each field to 0–31
  - `CHOPCONF` — write-mask reserved bits
- StallGuard injection: when virtual position hits configured wall,
  drop `SG_RESULT` below `SGTHRS` and assert DIAG via
  `runtime_sim_endstop_set_pin`

**TMC2209 emulator** (`orchestrator/tmc2209_emulator.py`, ~250 lines):

- Same dict-per-chip model
- UART frame parser: 4-byte read request (sync+slave+reg+CRC8),
  8-byte read response (sync+slave+reg+data+CRC8), 8-byte write
- CRC8 polynomial 0x07, init 0
- Same `GSTAT` clear-on-read; smaller register surface

**Drift detection**: any host write to a register the emulator doesn't
recognize logs a `TMC_UNKNOWN_REG` warning; tests assert zero such
warnings. Surfaces new register accesses we haven't modeled before
they brick something on hardware.

### 4. Beacon serial stub (`orchestrator/beacon_serial_stub.py`, ~250 lines)

Beacon talks its own binary protocol over USB-CDC. The stub re-uses
the wire-format parser from beacon.py (under
`printer_real/third_party_repos/beacon_klipper/beacon.py`) — inverted:
where beacon.py parses incoming bytes, the stub formats outgoing bytes.

Behavior:
- Init handshake: respond with consistent firmware version, NVM
  factory data (model coefficients, serial number)
- Continuous samples at ~1.6 kHz: `z_reading = z_target + sin_noise(t,
  ±5µm)` driven by the orchestrator's modeled toolhead Z position
- Probe events: when `z_reading <= configured_threshold`, fire the
  beacon-protocol probe event

Wire surface: a third sim socket `/tmp/klipper_sim_beacon`. Klippy's
`[beacon] serial: /dev/serial/by-id/usb-Beacon_*` gets path-translated
to `/tmp/klipper_sim_beacon` by the pin/serial override layer (§7).

### 5. ADC / heater stubs (`orchestrator/adc_stub.py`, ~150 lines)

Linux `klipper.elf` already has an ADC stub; we extend the existing
`runtime_sim_*` shim with a setter that the orchestrator drives:

- `runtime_sim_adc_set adc_pin=%c value=%u` — set the simulated ADC
  reading for a pin
- Default thermistor model:
  - Bed: 25°C ambient, ramps toward `M140 S<temp>` target at ~0.5°C/s
  - Hotend: 25°C ambient, ramps at ~3°C/s
  - MCU temp / motor temps: constant 35°C
  - Beacon coil: constant 25°C
- Heater PWM writes go to the ADC feedback loop so target-tracking
  stabilizes within tolerance

`[output_pin caselight]`, `[fan]`, `[filament_switch_sensor]`,
`[filament_motion_sensor]`: pure GPIO writes, accept silently;
filament-present hooks expose programmatic test toggles.

### 6. Sensorless homing trigger (`orchestrator/sensorless_trigger.py`, ~200 lines)

Models: X/Y motors stall against virtual walls; TMC StallGuard
load-measurement crosses threshold; DIAG asserts; klippy's virtual
endstop trips.

- **Position tracker**: orchestrator polls the firmware's exposed
  step-count-per-stepper (already in `runtime_handle_step_count`),
  converts to mm using `rotation_distance` from `printer.cfg`
- **Wall config**: per-axis virtual stop position in
  `tools/sim_klippy/printer_real/sim_geometry.toml` (defaults from
  `position_min` / `position_max` of each rail)
- **DIAG trigger**: as virtual position approaches wall,
  `SG_RESULT = max(0, distance_to_wall_mm * 50)`; when below `SGTHRS`,
  emulator asserts DIAG via `runtime_sim_endstop_set_pin`

Means tests never have to script "set DIAG at time T" — homing just
works because the model fires when the motor virtually hits the wall.
Tests can override wall positions per-test if needed.

### 7. Pin / serial translation

Real config has STM32 pin names (`PG4`, `PC7`, `spi1`, …) and USB
serial paths (`/dev/serial/by-id/usb-Klipper_stm32h723xx_*`). Linux
`klipper.elf` accepts `gpiochip0/gpioN` for GPIOs and `/dev/spidevX.Y`
for SPI. We translate via an overlay layer.

`tools/sim_klippy/printer_real/pin-overrides.toml`:

```toml
[mcu_main.gpio]
PG4 = "gpiochip0/gpio9"        # stepper_x.step_pin
PC7 = "gpiochip0/gpio10"       # tmc5160 stepper_x.cs_pin
# … one mapping per GPIO referenced in printer.cfg

[mcu_main.spi]
spi1 = "spidev0.0"             # SPI bus, intercepted by orchestrator

[mcu_bottom.gpio]
PD11 = "gpiochip0/gpio30"      # tmcuart oid=0
# …

[serial]
"usb-Klipper_stm32h723xx_*" = "/tmp/klipper_sim_h7"
"usb-Klipper_stm32f446xx_*" = "/tmp/klipper_sim_f4"
"usb-Beacon_*" = "/tmp/klipper_sim_beacon"
```

The override layer wraps klippy's pin-resolution and serial-path
lookup. The vendored `printer.cfg` is read verbatim — no edits.
Generation of `pin-overrides.toml` is a one-time scan over the
vendored config; emit in tree as a committed artifact.

### 8. Test harness (`tools/sim_klippy/tests/`)

Three pytest modules, runnable via `make sim-test` or
`pytest tools/sim_klippy/tests/`:

| Test | Asserts | Run time |
|---|---|---|
| `test_boot.py` | klippy reaches "ready"; both MCUs connected; no tracebacks; no MCU shutdowns; zero `TMC_UNKNOWN_REG` warnings | ~5 sec |
| `test_g28_full.py` | each axis homes via the right method; `homed_axes` reflects; toolhead position matches `position_endstop`; M84 clears homed | ~20 sec |
| `test_small_print.py` | 30-line slicer chunk completes; bed mesh has 9 points; heater feedback within ±2°C; final toolhead position matches expected; no MCU shutdowns mid-print | ~60 sec |

Test fixtures: `tools/sim_klippy/fixtures/small_print.gcode` (30 lines).

### 9. Failure-mode rules

Test fails if any of:
- klippy traceback during run
- `MCU '*' shutdown` during run
- `bridge_call: transport closed` / `transport timed out`
- Any `TMC_UNKNOWN_REG` warning (drift detection — extend the stub
  before deploying)
- Test-specific assertion failures

Logged but doesn't fail:
- Heater PID overshoot >2°C
- Beacon Z-noise spikes
- Status-frame cadence variations

Per-test artifacts under `.local-logs/<test_name>/`: full klippy.log,
both `klipper.elf` logs, orchestrator log, TMC stub register-touch
trace, beacon-stub frame trace.

## What this catches

- Every `klippy/extras/*.py` × `BridgeKinematics` / `MotionToolhead` API
  mismatch (today's `clear_homing_state("z")` class)
- Every "host-side timing assumption breaks under bridge mode" (today's
  `is_active()` queries_pending growth)
- Every "third plugin's print_time / homing_axes call expects something
  that motion-bridge doesn't provide"
- Every msgproto wire-format change that breaks the real config
- Every config-time validation never exercised in the synthetic sim cfg
- Every TMC register access we haven't modeled (via `TMC_UNKNOWN_REG`
  drift detection)

## What this deliberately doesn't catch

- Real chip-side bugs (charge-pump UV, USB enumeration, USB-CDC kernel
  cooked-mode, real timing)
- Real-time deadline misses (sim isn't deadline-faithful)
- Mechanical / thermal / vibrational issues

## Estimated scope

| Component | Effort |
|---|---|
| Vendored snapshot | done |
| Multi-MCU launcher | small |
| TMC5160 emulator | medium (PyTrinamic shrinks register-table work) |
| TMC2209 emulator | small (smaller register surface) |
| Beacon serial stub | medium (binary protocol parse-and-invert) |
| ADC/heater stubs | small |
| Sensorless trigger | medium (position model + DIAG dance) |
| Pin/serial override layer | small |
| `runtime_sim_route_spi` / `runtime_sim_route_tmcuart` firmware shims | small (mirrors existing `runtime_sim_endstop_set_pin`) |
| 3 pytest tests + fixtures | small |

Total: roughly two solid days end-to-end. The two TMC stubs and the
beacon stub are the only non-trivial pieces; everything else is
plumbing on existing infrastructure.

## Open questions

None — every architectural decision is captured above. Implementation
plan to follow.
