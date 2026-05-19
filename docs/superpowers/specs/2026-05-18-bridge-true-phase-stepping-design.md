# Bridge wiring for true TMC5160 phase stepping

**Status:** Draft, awaiting user review.
**Date:** 2026-05-18.
**Predecessor:** `docs/superpowers/specs/2026-05-18-phase-stepping-sim-design.md` (MCU-side modulator + Renode sim; shipped, bench-flashed).

## 1. Problem

After the predecessor spec landed, the bench printer accepts `phase_stepping: True`
in `printer.cfg` and moves. But the firmware path is still **hybrid stepping**
(Modulated step-pulse emission via TIM5 ISR), not **true phase stepping** (XDIRECT
register writes at 40 kHz with the chip's internal commutator bypassed).

The two reasons it is hybrid and not true:

1. `rust/motion-bridge/src/bridge.rs::configure_axes` only emits the 25-byte
   ConfigureAxes body. The 33-byte format (bytes 25–32 = per-motor
   `spi_bus_id`/`cs_pin_id`) is unimplemented. Consequence:
   `SharedState.phase_config[i] = NONE_SENTINEL` for every motor, and
   `runtime_modulated_tick` falls through to `StepPulseModulator` rather than
   `PhaseDirectModulator`.

2. `runtime_register_phase_bus` (the MCU command that lets the firmware
   `spi_setup` the SPI3 peripheral and `gpio_out_setup` the per-axis CS pin) is
   defined at `src/runtime_commands.c:534` but has **no klippy-side caller**.
   Even if `phase_config` were installed, `phase_stepping_write_xdirect` would
   silently early-exit on `!configured`.

3. No klippy-side code writes `GCONF.direct_mode = 1` (bit 16) on the TMC5160
   driver for phase-stepped motors. Without that, the chip ignores XDIRECT.

Verification path: `DUMP_TMC STEPPER=stepper_x` currently reports `direct_mode=0`.

## 2. Goal

For every stepper declared `phase_stepping: True` on an MCU advertising the
`PHASE_STEPPING_CAPABLE` identify-time capability bit, the bring-up sequence
shall:

- Configure the TMC5160 with `GCONF.direct_mode = 1` and `IHOLDIRUN.IHOLD = f(run_current)`
  before any motion.
- Register the SPI bus + CS pin with the MCU's phase-stepping subsystem so
  `phase_stepping_write_xdirect` is functional.
- Install the per-motor `(spi_bus_id, cs_pin_id)` pair into the runtime's
  `SharedState.phase_config[i]` via the 33-byte ConfigureAxes body, so
  `runtime_modulated_tick` takes the `PhaseDirectModulator` arm.

Verification: `DUMP_TMC STEPPER=stepper_x` reports `direct_mode=1`; sim wire
captures show 33-byte ConfigureAxes body; sim Tmc5160 stub records GCONF write
with bit 16 set followed by XDIRECT (reg 0x2D) writes during segment push; on
the bench, motor motion exhibits the phase-stepping audio signature relative to
the same motion with `phase_stepping: False`.

## 3. Non-goals

- Skip detection / closed-loop feedback (future Step 11).
- Phase stepping on TMC2240 or non-5160 chip families.
- Per-toolchange phase-stepping mode changes (single-tool, one-shot at print start).
- LUT calibration on the host (LUT is mocked / identity for v1; future work).

## 4. Architecture

### 4.1 Bring-up sequence at klippy `connect`

```
              klippy connect
                    │
                    ▼
   1.  TMC5160.__init__   (klippy/extras/tmc5160.py)
       ─ reads sister [stepper_*] section via printer.lookup_object
       ─ if phase_stepping=True:
            set_config_field("direct_mode", True)        # queued for connect-time write
            validate stealthchop_threshold absent
            validate microsteps==256
            swap CurrentHelper to map run_current → IHOLDIRUN.IHOLD (not IRUN)
            stash (bus_id, cs_pin_id) on the TMC instance for later retrieval
                    │
                    ▼
   2.  TMC config-time SPI burst                      ── SPI3
       ─ Klipper's _init_registers (klippy/extras/tmc.py:322-326)
         walks fields.registers and emits one spi_send per non-default field
         including GCONF.direct_mode=1, IHOLDIRUN.IHOLD
       ─ Runs to completion before any modulation
                    │
                    ▼
   3.  motion_toolhead._configure_axes_per_mcu        ── kalico-native wire
       ─ for each i with step_modes[i] == 0:
            tmc = printer.lookup_object("tmc5160 " + primary_stepper_name)
            phase_configs[i] = tmc.get_phase_config()  # (bus_id, cs_pin_id)
       ─ For each unique bus_id across phase_configs:
            bridge.register_phase_bus(bus_id, cs_pin_id_anchor, rate=2_000_000)
            (→ runtime_register_phase_bus wire cmd
             → spi_setup + gpio_out_setup
             → phase_stepping_register_bus on the MCU)
       ─ bridge.configure_axes(..., phase_configs=phase_configs)
            (→ 33-byte ConfigureAxes body
             → runtime_configure_axes_blob
             → SharedState.phase_config[i] = Some(cfg))
                    │
                    ▼
   4.  First push_segment → TIM5 ISR starts modulating
       ─ runtime_modulated_tick reads phase_config[i] → Some
       ─ PhaseDirectModulator computes coil_A, coil_B from LUT
       ─ phase_stepping_write_xdirect(bus_id, cs_pin, ...) → real XDIRECT writes
```

The order of operations matters:

- `phase_stepping_register_bus` MUST be called before any
  `phase_stepping_write_xdirect`. The latter checks an internal `configured`
  flag and silently drops frames if unset.
- `runtime_register_phase_bus` MUST be called before
  `runtime_configure_axes_blob`, because the blob path validates that each
  per-motor `(bus_id, cs_pin_id)` references a registered bus.
- `GCONF.direct_mode = 1` SHOULD be written before TIM5 ISR begins
  modulating. Klippy's `_init_registers` runs at `connect`, which is well
  before motion; this is satisfied by the existing TMC field-collection path.

### 4.2 SPI bus contention: ISR XDIRECT vs Klipper TMC SPI

SPI3 is shared between two writers:

- The TIM5 ISR's `phase_stepping_write_xdirect`, firing at 40 kHz, ~25 µs per
  5-byte SPI write.
- Klipper's regular TMC SPI driver, used during config (one burst at connect),
  during homing (3–5 writes per `handle_homing_move_begin`/`_end`), and
  periodically (`TMCErrorCheck._do_periodic_check` polls `DRV_STATUS` at 1 Hz
  during prints; see `klippy/extras/tmc.py:213, 240-244`).

The 1 Hz DRV_STATUS poll is the hot path — it is Klipper's
overtemp/short-detection mechanism and cannot be disabled during prints
without losing fault detection on the very things phase stepping stresses
(coils running hotter due to direct current control).

**v1 stance — cooperative busy-flag, ISR-priority:**

1. `src/stm32/phase_stepping_spi.c` defines an atomic `phase_spi_busy` flag
   plus `phase_spi_try_acquire()` / `phase_spi_release()` primitives.
2. The TIM5 ISR's `phase_stepping_write_xdirect` consults the flag. If a
   regular SPI transfer is in flight, it SKIPS this modulation cycle and
   bumps a `phase_spi_skip_count` telemetry counter. At 40 kHz, one skip is
   25 µs of held coil current — well below the audible / mechanical noise
   floor. The next ISR fire will retry.
3. The MCU-side SPI driver's blocking `spi_transfer` (or equivalent for SPI3)
   acquires `phase_spi_busy` before its transfer, executes (~25–40 µs for
   5–8 bytes at 2 MHz), releases.
4. Collision math: each Klipper 5–8 byte transfer at 2 MHz holds the bus
   for ~25–40 µs. The TIM5 ISR fires every 25 µs (40 kHz period), so a
   single Klipper poll forces the ISR to skip 1–2 modulation cycles.
   At 1 Hz Klipper polling that is ≤2 skips/second, each lasting 25 µs —
   inaudible and mechanically invisible. Skip-count telemetry is the
   canary: bench tests assert skip_count stays below 100/s during the
   modulation-active idle test (which is itself a wide safety margin —
   if we see anywhere near 100/s, something else is wrong).
5. Both writers acquire the flag — both `phase_stepping_write_xdirect`
   AND Klipper's `spi_transfer` for SPI3. If only the ISR consults the
   flag, a regular SPI transfer can be preempted mid-byte by a TIM5
   fire, the ISR will succeed-acquire (flag was unset), and bus
   corruption results. The §5.7 hook MUST be bidirectional.
5. Homing moves' 3–5 writes per call cause a short burst of skips, also
   negligible (homing is at print start/end).

This v1 stance keeps Klipper's DRV_STATUS polling fully functional during
prints and gives us a skip-counted budget that can be tightened in a future
revision by:

- Migrating the regular SPI path to interrupt-driven DMA (out of v1 scope).
- Routing Klipper SPI through a "scheduled-during-ISR-gap" proxy in
  `phase_stepping_spi.c` (out of v1 scope).

### 4.3 IHOLD vs IRUN in direct mode

The TMC5160 has two current scaling registers, IRUN and IHOLD, that work like
a two-speed gearbox switched by step pulses: a step pulse engages IRUN, then
the chip decays toward IHOLD over `IHOLDDELAY` clock periods if no further
steps arrive. In phase stepping, no step pulses are ever asserted — coil
currents are driven directly via XDIRECT — so the gearbox stays in IHOLD
indefinitely by construction. This is why `run_current` from Klipper config
must be mapped to `IHOLDIRUN.IHOLD` rather than `IHOLDIRUN.IRUN` for
phase-stepped axes.

The mapping uses the same formula `TMC5160CurrentHelper` already applies to
`IRUN`, just stored in a different bit field. Plain-English: same A→register
math, different register slot.

## 5. Component-level design

### 5.1 `klippy/extras/tmc5160.py`

Three additions, gated on `phase_stepping=True` in the sister stepper section:

1. Already-present GCONF mirror at line 167 maps `direct_mode` → `0x01 << 16`.
   No code change to the Fields dict.
2. In `TMC5160.__init__`, after the existing `set_config_field` block, add:

   ```python
   stepper_name = " ".join(config.get_name().split()[1:])  # tmc.py convention
   try:
       stepper_section = config.getsection(stepper_name)
       phase_stepping = stepper_section.getboolean("phase_stepping", False)
   except Exception:
       phase_stepping = False
   if phase_stepping:
       _enable_direct_mode(config, stepper_section, self.fields, self.mcu_tmc)
       self._phase_stepping = True
       self._phase_bus_id, self._phase_cs_pin_id = (
           self.mcu_tmc.tmc_spi.get_bus_and_cs_ids()
       )
   else:
       self._phase_stepping = False
   ```

3. New helper `_enable_direct_mode(config, stepper_section, fields, mcu_tmc)`:
   - `fields.set_field("direct_mode", 1)` to queue the GCONF bit.
   - Validate `stepper_section.getfloat("stealthchop_threshold", 0.) == 0.0`;
     raise `config.error` if non-zero with explanatory message.
   - Validate microsteps==256 — read the TMC's own `microsteps` config
     value via `config.getint("microsteps")`; raise if != 256.
   - Validate the driver is `tmc5160` (implicit from the class), not
     applicable inside `TMC5160.__init__` but documented for the dispatcher.
4. New public method `TMC5160.get_phase_config()` returning
   `(self._phase_bus_id, self._phase_cs_pin_id)` or raising if not
   phase-stepping. Called from `motion_toolhead._configure_axes_per_mcu`.

### 5.2 `klippy/extras/tmc5160.py::TMC5160CurrentHelper`

Direct-mode-aware mapping: if `phase_stepping=True` for this stepper, the
helper's `set_current` method writes BOTH `IHOLDIRUN.IHOLD = computed_value`
AND `IHOLDIRUN.IRUN = computed_value`. They are set equal so the chip is
robust against any unexpected step-pulse event that would transiently engage
IRUN scaling — with both equal, the effective current ceiling is the same
either way. For non-phase-stepped steppers, behaviour is unchanged
(`IRUN` from `run_current`, `IHOLD` from `hold_current` ratio).

This requires `TMC5160CurrentHelper` to learn about phase stepping. Cleanest
plug point: the same `phase_stepping` lookup added in §5.1 stores the flag on
the helper at construction time via an extra constructor arg.

### 5.3 `klippy/extras/tmc2130.py::MCU_TMC_SPI_chain`

One new accessor method: `get_bus_and_cs_ids() -> (int, int)` returning the
numeric bus ID and CS pin ID matching what the firmware's `spi_setup` and
`gpio_out_setup` expect. These values are needed in the same form the
firmware uses internally (port*16+pin for GPIO; SPI peripheral enum index
for the bus).

Implementation note: Klipper's `MCU_TMC_SPI_chain.__init__` already runs
the config strings through pin/bus resolvers when constructing the
`config_spi oid=%c bus=%u pin=%u mode=%u rate=%u` MCU command — that
command's `bus` and `pin` parameters are exactly the integers we want.
The accessor either stashes those resolved values at construction time, or
re-runs the same resolver (`mcu.lookup_pin(cs_pin_str)['pin']` style).
Implementer's choice; the contract is "returns the two ints the firmware
side sees, not the string names from config".

### 5.4 `klippy/motion_toolhead.py::_configure_axes_per_mcu`

After the existing `step_modes[i] = 0` assignment block, before the
`mcu_caps` capability check, add:

```python
phase_configs = [(0xFF, 0xFF)] * 4   # sentinel = absent
for i, slot in enumerate(slot_steppers):
    if step_modes[i] != 0 or not slot:
        continue
    primary_name = slot[0][0]
    try:
        tmc = self.printer.lookup_object("tmc5160 " + primary_name)
    except self.printer.config_error:
        raise self.printer.config_error(
            "phase_stepping requires a [tmc5160 %s] section" % primary_name
        )
    phase_configs[i] = tmc.get_phase_config()

# Register each unique bus once before configure_axes
seen_buses = set()
for i, (bus_id, cs_pin_id) in enumerate(phase_configs):
    if bus_id == 0xFF:
        continue
    if bus_id in seen_buses:
        continue
    seen_buses.add(bus_id)
    self.bridge.register_phase_bus(
        mcu_handle, bus_id, cs_pin_id, rate=2_000_000,
    )
```

Then thread `phase_configs` into the existing `configure_axes` call. When no
slot has phase stepping, `phase_configs` stays all-sentinel and the bridge
emits the 25-byte body unchanged.

### 5.5 `klippy/motion_bridge.py`

Two additions:

1. New `register_phase_bus(mcu_handle, bus_id, cs_pin_id, rate, timeout_s=5.0)`
   method delegating to the same-named PyO3 method on `self._bridge`.
2. Extend `configure_axes(...)` signature with
   `phase_configs=None` kwarg, passed through.

### 5.6 `rust/motion-bridge/src/bridge.rs`

1. Extend `configure_axes` PyO3 signature with
   `phase_configs: Option<Vec<(u8, u8)>>`. Validate length == 4 when present.
2. When `phase_configs` is present, emit the 33-byte body:
   - bytes 0..20: unchanged (kinematics, masks, steps_per_mm)
   - byte 20: phase_capable flag (existing)
   - bytes 21..24: step_modes (existing)
   - bytes 25..32: 4 × (bus_id u8, cs_pin_id u8) interleaved
3. When `phase_configs` is None: existing 25-byte / 20-byte behaviour
   preserved (no caller regressions).
4. New `register_phase_bus(mcu_handle, bus_id, cs_pin_id, rate, timeout_s)`
   PyO3 method. Sends the existing `runtime_register_phase_bus bus_id=%c
   cs_pin_id=%c rate=%u` wire command via `bridge_call`, waits for the
   `kalico_register_phase_bus_response result=%i` reply, returns Ok(()) on
   result==0 else raises PyRuntimeError.

### 5.7 `src/stm32/phase_stepping_spi.c` (and SPI driver hook point)

1. New `atomic_int phase_spi_busy` (use the CMSIS atomic primitives the rest
   of the runtime already uses).
2. `bool phase_spi_try_acquire(void)` — atomic CAS, returns true if
   acquired. `void phase_spi_release(void)` — atomic store 0.
3. `phase_stepping_write_xdirect` flow:
   - `if (!phase_spi_try_acquire()) { runtime_phase_spi_skip_count++; return; }`
   - existing SPI write
   - `phase_spi_release()`
4. The MCU-side blocking `spi_transfer` path (or H7-specific equivalent) for
   SPI3 acquires `phase_spi_busy` before its transfer and releases after.
   This is one acquire/release pair around the existing transfer. The hook
   point lives in `src/stm32/spi.c` or wherever `spi_transfer` is
   implemented for the H723 platform; gated on whether the active bus is
   the one registered with phase stepping.
5. `runtime_phase_spi_skip_count` is a `uint32_t` exposed via the existing
   status / telemetry surface (added to whatever struct gets serialised in
   `runtime_query_status` responses).

### 5.8 `rust/runtime` telemetry

Surface `phase_spi_skip_count` on the existing status struct so sim and bench
tests can assert it stays below thresholds. ~10 lines: one field, one bump
in the FFI conversion.

## 6. Error handling (klippy-side, raised at connect)

- `phase_stepping: True` on stepper with no `[tmc5160 <name>]` block:
  config_error "phase stepping requires a TMC5160 driver section for
  stepper `<name>`".
- `phase_stepping: True` on stepper with `[tmc2209 <name>]` or other
  non-5160 driver: config_error "phase stepping is currently only
  implemented for TMC5160 drivers (stepper `<name>` uses `<driver_type>`)".
- `phase_stepping: True` + `stealthchop_threshold > 0`: config_error
  "StealthChop is bypassed in direct mode; remove `stealthchop_threshold`
  from stepper `<name>`".
- `phase_stepping: True` + `microsteps != 256` (read from the `[tmc5160
  <name>]` section): config_error "phase stepping requires
  `microsteps: 256`; stepper `<name>` has `microsteps: <N>`".
- `phase_stepping: True` on a stepper whose MCU lacks PHASE_STEPPING_CAPABLE:
  already covered by `motion_toolhead.py:734`. No change.

## 7. Testing

### 7.1 Rust unit tests

`rust/motion-bridge/tests/`:

- `configure_axes_33_byte_body` — invoke `configure_axes` with
  `phase_configs=Some(vec![(3,5),(3,6),(255,255),(255,255)])`, capture the
  emitted wire body, assert length == 33 and bytes[25..33] ==
  [3,5,3,6,255,255,255,255].
- `configure_axes_25_byte_unchanged` — invoke without `phase_configs`,
  assert body is exactly 25 bytes (no regression on current callers).
- `register_phase_bus_round_trip` — fixture serial transport, emit
  `runtime_register_phase_bus`, fake-respond with
  `kalico_register_phase_bus_response result=0`, assert Ok(()).
- `register_phase_bus_error` — respond with `result=-88`, assert
  PyRuntimeError with the right detail.

### 7.2 Renode sim integration test

Extend `tools/test_sim_phase_stepping.py`:

- Build sim firmware with `phase_stepping: True` config baked in for X+Y.
- Wire-capture (via the existing Renode hooks) confirms:
  - ConfigureAxes body is 33 bytes.
  - bytes[25..33] match expected (bus_id=3 for SPI3, cs_pin_id from
    PA5/PA6 encoding).
  - `runtime_register_phase_bus` is emitted before
    `runtime_configure_axes_blob`.
- Tmc5160 sim stub (`tools/sim/renode_peripherals/Tmc5160.cs`)
  recording confirms:
  - At least one GCONF write with bit 16 set, before the first XDIRECT.
  - At least one IHOLDIRUN write with IHOLD field populated.
  - During segment push, ≥1 XDIRECT (reg 0x2D) write per phase-stepped
    motor, with coil_A/coil_B values within the expected LUT range.
- Assert `phase_spi_skip_count` reported by the runtime status frame
  stays at 0 during the test (sim has no concurrent klippy TMC SPI
  traffic).

### 7.3 Bench tests (Trident, H723 + F446)

- Pre-flash diff: confirm `make clean && cargo clean` between H7 and F4
  builds per `feedback_cargo_clean_between_mcus.md`.
- `DUMP_TMC STEPPER=stepper_x` after `connect`:
  - `GCONF.direct_mode = 1`.
  - `IHOLDIRUN.IHOLD` reflects `run_current` from config.
- `DUMP_TMC STEPPER=stepper_y`: same checks.
- `DUMP_TMC STEPPER=stepper_z` (non-phase-stepped):
  - `GCONF.direct_mode = 0`. Negative-control check.
- Issue a G1 jog on stepper_x — motor moves smoothly, audio signature
  compared subjectively to a control run with `phase_stepping: False`.
- "Modulation-active idle" test: issue a 60-second G1 hold (very slow
  motion, e.g. G1 X0.001 F1 repeated) so the TIM5 ISR is actively
  writing XDIRECT throughout. DRV_STATUS polling runs at 1 Hz in
  parallel. After 60 s, read `phase_spi_skip_count` from the runtime
  status frame; assert it grew by ≤ 120 over the interval (allows up
  to 2 skips per 1 Hz Klipper poll plus headroom).
- Negative test: temporarily set `phase_stepping: True` on `stepper_z`
  (tmc2209-driven). klippy should refuse to start with the
  "phase stepping is currently only implemented for TMC5160 drivers"
  config error.

## 8. Open concerns / future work

- Skip-counted SPI3 contention is a coarse first answer. If
  `phase_spi_skip_count` runs higher than expected on the bench, the
  fix is either (a) DMA-driven SPI3, (b) routing Klipper TMC SPI
  through a scheduled-between-ISR-fires proxy. Both are out of v1
  scope.
- Multi-MCU phase stepping (X on MCU A, Y on MCU B): the design
  already handles this — `_configure_axes_per_mcu` already iterates
  per-MCU, and `register_phase_bus` is per-bus on the local MCU. No
  cross-MCU coordination required for SPI buses.
- Phase stepping on a TMC2240 (newer hardware with the same XDIRECT
  protocol but different register layout): straightforward extension,
  another `_enable_direct_mode` shim in `tmc2240.py`. Out of v1.

## 9. Acceptance gate

This spec ships when all of the following pass:

1. Rust unit tests for the 33-byte body and register_phase_bus round-trip.
2. Renode sim integration test (`tools/test_sim_phase_stepping.py`
   extension) green, including Tmc5160 stub GCONF + XDIRECT capture.
3. Bench `DUMP_TMC STEPPER=stepper_x` reports `direct_mode=1` after
   connect with `phase_stepping: True`.
4. Bench `phase_spi_skip_count` < 100/s sustained during a steady-state
   modulation hold (no motion, no homing).
5. Negative test: misconfigured phase_stepping triggers the expected
   klippy config_error at startup.

Items 1–2 are blocking; 3–5 are bench-confirmed and reported back from
hardware before declaring done.
