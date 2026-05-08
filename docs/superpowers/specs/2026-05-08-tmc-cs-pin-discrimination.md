# TMC CS-pin discrimination on shared sim SPI bus — design

**Status:** approved 2026-05-08
**Parent spec:** `docs/superpowers/specs/2026-05-08-faithful-klippy-sim-design.md`
**Replaces:** the bus-level "one TMC5160 emulator stands in for all chips on sim_spi0" approach in `tools/sim_klippy/conftest.py:216`

## Goal

The user's H7 SPI bus `spi1` carries five chips: four TMC5160 stepper
drivers (X/Y/X1/Y1, CS pins PC7/PC6/PD11/PC4) and one MAX31865 RTD
amplifier (CS pin PF8) used for the extruder thermistor. Today the sim
collapses all five into a single TMC5160Emulator on the bus socket, so
MAX31865's 2-byte SPI transfers crash the TMC handler with
`ValueError: TMC5160 expects 5-byte datagram, got 2`.

Per CLAUDE.md's signal-hiding rule, the answer is not to silently
accept arbitrary lengths — it's to route each transfer to the correct
chip emulator by CS pin, then have each emulator strictly validate its
own protocol. That's what real hardware does.

## Architecture

```
┌──── firmware (klipper-h7-sim.elf) ────────┐
│                                           │
│  spi_transfer(config, len, data)          │
│  ├─ if sim_spi: prepend (dev:1, len:1)   │
│  │  to outgoing payload                   │
│  └─ sim_chip_socket_xfer reads framed     │
│     reply (len:1, payload:len)            │
└────────────────┬──────────────────────────┘
                 │ Unix socket
┌────────────────▼──────────────────────────┐
│  ChipSocketServer (framed mode)           │
│  ├─ read 1B cs, 1B len, len B payload    │
│  ├─ dispatch by cs to handler dict        │
│  ├─ write 1B len, len B reply             │
│  └─ TMC5160Emulator × 4 (one per CS)      │
│      MAX31865Emulator × 1 (RTD)           │
└───────────────────────────────────────────┘
```

## Components

### 1. Firmware wire-protocol extension

`src/linux/sim_chip_socket.{h,c}`:

- New API: `int sim_chip_socket_xfer_framed(int fd, uint8_t cs, const
  uint8_t *tx, size_t tx_len, uint8_t *rx)` — writes `[cs][tx_len][tx
  payload]`, reads `[rx_len][rx payload]`. The reply length comes
  back from the server; caller passes a buffer of `tx_len` size (SPI
  transfers are symmetric).
- Old `sim_chip_socket_xfer` retained for tmcuart's existing path
  (which has its own framing via start/stop bits — out of scope here).

`src/linux/spidev.c::spi_transfer`:
- Compute `dev_id = SPIBUS_TO_DEV(bus)` at setup, store in
  `sim_spi_route` alongside fd. (Today only `bus` is stored; `dev` is
  thrown away.)
- Replace the `sim_chip_socket_xfer(...)` call with
  `sim_chip_socket_xfer_framed(fd, dev_id, ...)`.

The `sim_spi_route` struct grows a `uint8_t dev` field. `spi_setup`
records it from `SPIBUS_TO_DEV(bus)`.

### 2. ChipSocketServer framed mode

`tools/sim_klippy/orchestrator/chip_socket_server.py`:

- New constructor option: `framed: bool = False`. When False, current
  fixed-`chunk` behavior is preserved (tmcuart path keeps working).
- When True, ignore `chunk`. The handler signature changes to
  `Callable[[int, bytes], bytes]` — receives `(cs, payload)`, returns
  reply payload. The server frames the reply with a 1-byte length.
- The `_serve` loop reads `[cs:1][len:1][payload:len]` and writes
  `[len:1][reply:len]`. EOF / partial reads handled cleanly.

### 3. CS dispatcher

New helper `tools/sim_klippy/orchestrator/spi_router.py`:

```python
class SpiRouter:
    """Dispatches (cs, payload) to the chip emulator registered on cs."""
    def __init__(self): self._chips = {}
    def attach(self, cs: int, handler: Callable[[bytes], bytes]) -> None: ...
    def __call__(self, cs: int, payload: bytes) -> bytes:
        h = self._chips.get(cs)
        if h is None:
            raise KeyError(f"sim spi: no chip on CS {cs}")
        return h(payload)
```

The orchestrator builds a `SpiRouter`, attaches a `TMC5160Emulator`
per CS for each stepper, attaches a `MAX31865Emulator` for the RTD
CS, and passes `SpiRouter` instance as the framed-mode handler.

### 4. MAX31865 minimal emulator

`tools/sim_klippy/orchestrator/max31865_emulator.py`: ~100 lines.
- 8-register state model per datasheet
- 1-byte address transfer: `addr & 0x7F` for read, `addr | 0x80` for
  write; data byte follows
- Default config register reads back as 0xC0 (auto-fault-detection
  off, bias on, V_BIAS on)
- RTD-MSB / RTD-LSB returns a value corresponding to a fixed 25 °C
  reading (0x4DAB or whatever the rtd_nominal_r=1000 / rtd_reference_r=
  4300 calibration produces). Constant value is fine — we're not
  testing thermistor accuracy.

### 5. CS-pin → physical-name map

The firmware has only the dev_id (the 4-bit `dev` field of the bus
encoding). On real hardware, klippy maps GPIO pin names like `PC7` to
GPIO chip lines via the pin-overrides layer. For sim SPI, the
firmware's `spi_setup` is called with `bus_id` derived from the GPIO
chardev pin number — the mapping `PC7 → gpiochip0/gpio10` is already
done in `pin-overrides.toml`, and the chardev pin number becomes the
`dev` field of the bus encoding.

So `dev_id` is the chardev gpio number. We snapshot the mapping in
`tools/sim_klippy/sim_geometry.toml` (already exists for sensorless
walls) so the orchestrator knows which CS number is which stepper:

```toml
[h7_spi0_chips]
# Maps gpiochip0 line number → chip name. Source of truth:
# pin-overrides.toml [mcu_main.gpio]
10 = "stepper_x"     # PC7
9  = "stepper_y"     # PC6  (verify against actual mapping)
30 = "stepper_x1"    # PD11
8  = "stepper_y1"    # PC4
…  = "extruder_rtd"  # PF8 (MAX31865)
```

The exact gpio numbers come from `pin-overrides.toml`. Conftest reads
both files and builds the SpiRouter wiring.

### 6. Conftest integration

`tools/sim_klippy/conftest.py:216` rewrite — instead of one shared
TMC5160Emulator, build the SpiRouter and attach four TMC5160s and one
MAX31865 to the appropriate CS values.

## Test plan

### A. Unit: framed wire protocol

`tests/test_chip_socket_server.py`:
- `test_framed_dispatches_by_cs_byte`: send `[5, 5, 0x80, 0, 0, 0, 1]`
  and `[3, 2, 0xC0, 0]`; assert handler called with `(5, ...)` and
  `(3, ...)` respectively.
- `test_framed_reply_length_byte`: handler returns 5 bytes → wire has
  `[5][reply]`.
- `test_partial_read_recovery`: split frame across two recv calls,
  assert correct dispatch.

### B. Unit: SPI router

`tests/test_spi_router.py`:
- `test_attach_dispatches_by_cs`: attach two emulators on different
  CS; route `(cs1, x)` and `(cs2, y)`; assert each reached its
  emulator.
- `test_unknown_cs_raises`: route to unattached CS → KeyError.

### C. Unit: MAX31865 emulator

`tests/test_max31865_emulator.py`:
- `test_config_register_default`: read addr 0x00 → 0xC0.
- `test_rtd_register_constant`: read addr 0x01-0x02 → constant 25 °C
  value.

### D. Integration: test_boot

Re-run `tools/sim_klippy/tests/test_boot.py`:
- Klippy connects, both H7 + F4 + beacon configure cleanly.
- TMC drivers' GCONF/IHOLD_IRUN init writes succeed (no MAX31865-on-
  TMC5160 emulator crash).
- klippy reaches "ready".

The success bar is `state == "ready"` from the api socket. Downstream
(`test_g28_full`, `test_small_print`) covered by future commits.

## Out of scope

- Per-stepper StallGuard SG_RESULT modeling (today all four TMCs
  share emulator state; with separate emulators they get independent
  state, which the next sensorless-trigger work will leverage). The
  `SensorlessTrigger` integration to drive per-axis SG_RESULT lands
  in a follow-up commit.
- F4 SPI bus chip discrimination (no shared bus on F4 in the user's
  config; not applicable).
- tmcuart framing changes (its existing chunked-fixed-length flow
  works fine; framed mode is opt-in).

## Implementation order

1. Firmware: `sim_chip_socket_xfer_framed` + `spi_setup` records `dev`
   + `spi_transfer` calls framed variant.
2. Firmware: rebuild `klipper-h7-sim.elf`, smoke-test with
   `tools/sim_klippy/run_local.sh`-style invocation.
3. Python: `ChipSocketServer` framed-mode option (TDD with unit
   tests).
4. Python: `SpiRouter` (TDD).
5. Python: `MAX31865Emulator` (TDD).
6. Conftest: replace single-TMC5160 with router + 4×TMC5160 + 1×MAX.
7. Run `test_boot.py`; iterate on chardev-line-to-CS mapping until
   each CS dispatches to the right emulator.
8. Commit.
