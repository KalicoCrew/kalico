# Phase-stepping per-motor CS — design

## Problem

`phase_stepping_write_xdirect(bus_id, cs_pin, …)` ignores its `cs_pin`
argument and uses a single CS GPIO cached per `bus_id`. The host
(`klippy/motion_toolhead.py`) dedupes `phase_configs` by `bus_id` before
registering, so only the first motor's `cs_pin_id` per bus is ever set
up. On a real Octopus Pro with two TMC5160s on one SPI bus (e.g.
`b_y` + `b_y2` on SPI3), every XDIRECT write pulls the same CS line and
only one driver listens. The other driver's coils are never updated.

Renode passes because its harness checks SPI bytes leaving the
peripheral, not which physical CS GPIO toggled to address a specific
driver among several on one MISO/MOSI/SCK.

Files involved:
- `klippy/motion_toolhead.py:830-911` — registration loop (dedup bug)
- `rust/motion-bridge/src/bridge.rs:1160-1231` — Python-facing register_phase_bus
- `src/runtime_commands.c:569-608` — wire command
- `src/stm32/phase_stepping_spi.{h,c}` — per-bus state + write_xdirect
- `rust/runtime/src/engine.rs:216-234,3340-3390` — Rust FFI shim + call site
- `rust/runtime/src/test_xdirect_capture.rs` — host-test sink

## Fix shape

**Separate the SPI bus config (shared, per-bus) from the CS handle
(distinct, per-motor).**

### C side (`src/stm32/phase_stepping_spi.{h,c}`)

```c
#define MAX_PHASE_BUSES  4
#define MAX_PHASE_MOTORS 16   /* matches Rust MAX_STEPPER_OIDS */

struct phase_bus_state {
    struct spi_config cfg;
    uint8_t configured;
};
static struct phase_bus_state phase_buses[MAX_PHASE_BUSES];

struct phase_motor_state {
    struct gpio_out cs;
    uint8_t bus_id;
    uint8_t configured;
};
static struct phase_motor_state phase_motors[MAX_PHASE_MOTORS];

void phase_stepping_register_bus(uint8_t bus_id, struct spi_config cfg);
void phase_stepping_register_motor(uint8_t motor_idx,
                                   uint8_t bus_id,
                                   uint8_t cs_pin_id);
void phase_stepping_write_xdirect(uint8_t motor_idx,
                                  int16_t coil_a,
                                  int16_t coil_b);
```

`phase_stepping_write_xdirect(motor_idx, …)` looks up the CS handle by
`motor_idx` (single deref) and the SPI cfg by
`phase_motors[motor_idx].bus_id` (second deref). No more `(void)cs_pin`
lie in the body. The function no-ops if either side is unconfigured.

### Wire protocol (`src/runtime_commands.c`)

Two cohesive commands:
- `runtime_register_phase_bus bus_id=%c rate=%u` — installs SPI cfg (one
  per unique bus_id).
- `runtime_register_phase_motor motor_idx=%c bus_id=%c cs_pin_id=%c` —
  installs per-motor CS (one per phase-stepped motor).

Sent in order: all `register_phase_bus` first, then all
`register_phase_motor`, both before `runtime_configure_axes_blob`.

### Rust runtime (`rust/runtime/src/engine.rs`)

```rust
fn write_xdirect(motor_idx: u8, coil_a: i16, coil_b: i16) { … }
```

Call site (`modulated_tick`, ~line 3372) becomes
`write_xdirect(motor_idx as u8, r.i_a, r.i_b)` — the per-motor
`PhaseConfig` (still keyed by `motor_idx` in shared.phase_config) is only
needed for the round-robin scheduling decision, not for the C call.

Host-test capture (`test_xdirect_capture.rs`) records `motor_idx`
instead of `(bus_id, cs_pin)`. This means host tests can now assert
that two motors on the same bus produce **distinct** XDirect records
keyed by motor — the regression test the original API made impossible.

### Bridge (`rust/motion-bridge/src/bridge.rs`)

- `register_phase_bus(mcu_handle, bus_id, rate, timeout_s)` — drops
  `cs_pin_id`.
- New `register_phase_motor(mcu_handle, motor_idx, bus_id, cs_pin_id,
  timeout_s)` mirrors the wire command.
- Both gate on `kalico_native_supported` like today.

### Klippy host (`klippy/motion_toolhead.py`)

Existing dedup loop (lines 858-867) splits in two:

```python
seen_buses = set()
for (bus_id, cs_pin_id, _slot_idx) in phase_configs:
    if bus_id == 0xFF: continue
    if bus_id in seen_buses: continue
    seen_buses.add(bus_id)
    self.bridge.register_phase_bus(mcu_handle, bus_id, rate=2_000_000)

for (motor_idx, sname, oid, inv) in bind_list:  # or whichever has motor_idx
    # Cross-reference to find the (bus_id, cs_pin_id) for this motor_idx
    …
    self.bridge.register_phase_motor(
        mcu_handle, motor_idx, bus_id, cs_pin_id,
    )
```

`phase_configs` is currently `(bus_id, cs_pin_id, slot_idx)`. The
`slot_idx` is the motor_idx already (verify in implementation). If so,
the per-motor loop iterates `phase_configs` directly.

## Why this design over alternatives

**Per-motor CS table (chosen) vs. per-(bus, cs) linear-scan lookup.**
Per-motor is O(1) in the ISR, matches the existing motor-indexed shape
(`shared.phase_config[motor_idx]`), and makes the `cs_pin` argument
load-bearing — which is the structural fix for "the next maintainer
re-introduces a per-bus dedup." Linear scan would cost ~16 ops per ISR
tick at 40 kHz on the H7, tolerable but unnecessarily ugly.

**Two wire commands vs. one combined.** Splitting matches the C-side
split (one bus cfg, multiple motors per bus). One combined command
would either need a rate-conflict check (every motor on a bus must
agree on rate) or first-wins semantics, both worse than the explicit
two-command form.

**Drop `(void)cs_pin` rather than keep a vestigial argument.** Per the
no-throwaway-code rule, vestigial arguments rot. The new signature is
the right shape.

## Boundary rules (MCU C/Rust)

- B1 (entry points): no new logical seam — these are methods on the
  existing phase-stepping helper API.
- B2 (shared state C-owned): `phase_buses`, `phase_motors` are C `.bss`,
  unchanged ownership model.
- B3 (no Rust types cross ABI): only `uint8_t` / `int16_t` cross. ✓
- B4 (`extern "C"` + `#[repr(C)]`): preserved. ✓
- B5 (memory model): single-writer (foreground registration) /
  single-reader (ISR) — same as before, just per-motor instead of
  per-bus. ✓

## Tests

- **Rust unit** (`rust/runtime/tests/modulator_integration.rs`): update
  existing round-robin test to assert `motor_idx`-keyed XDirect records;
  add new test where two phase motors share `spi_bus_id=0` with
  distinct `cs_pin_id` and verify both motors appear in the capture
  stream with their own `motor_idx`.
- **Bridge unit** (`rust/motion-bridge/src/bridge.rs` test module): add
  wire-format assertions for the new `register_phase_motor` command.
- **Bench (Step 7-D scope, manual)**: on the H723 Octopus Pro, configure
  two phase-stepped motors on one SPI bus and verify per-driver
  XDIRECT receipt via DRV_STATUS readback or scope on each CS line.
  Tracked in the bench-test backlog; not gating this merge.

## Out of scope

- Renode multi-CS GPIO observation: Renode's TMC5160 model doesn't
  distinguish CS lines on a shared bus, so a Renode-level test of "the
  right CS toggles" is not buildable today. The host-side test asserts
  the API/dispatch shape; physical correctness is bench-gated.
- Changing the rate per-motor: rate is still per-bus.
