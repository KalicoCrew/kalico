# Plan changes log

Running log of build-order/spec/constraint changes. Format per entry:
date, what changed, why, evidence link.

---

## 2026-05-19 — Phase-stepping: per-motor CS dispatch

**What changed.** The phase-stepping SPI write path went from a single
`(bus_id, cs_pin)` API where `cs_pin` was ignored, to a per-motor table
keyed by `motor_idx`. The wire protocol split into `runtime_register_phase_bus`
(SPI bus cfg) + `runtime_register_phase_motor` (per-motor CS GPIO). The
host's klippy registration loop now sends one bus message per unique
`bus_id` and one motor message per phase-stepped motor.

**Why.** The previous C-side `phase_stepping_write_xdirect(bus_id, cs_pin, ...)`
did `(void)cs_pin;` and pulled a single CS handle cached per `bus_id` in
`phase_buses[bus_id].cs`. The host (`klippy/motion_toolhead.py`) dedup'd
`phase_configs` by `bus_id`, so only the first motor's CS per bus was
ever registered. On a real Octopus Pro with two TMC5160s on one SPI bus
(e.g. dual-Y `b_y` + `b_y2` on SPI3), every XDIRECT write hit the same
physical CS line — only one driver listened, the other never received
coil current updates. Renode passed because its TMC5160 model doesn't
distinguish CS lines on a shared bus.

**Evidence.**
- Bug analysis: `src/stm32/phase_stepping_spi.c:84-90` (original
  `(void)cs_pin;` + single-CS dereference), `klippy/motion_toolhead.py`
  `seen_buses` dedup in the original commit.
- Design: `docs/superpowers/specs/2026-05-19-phase-stepping-per-motor-cs-design.md`.
- Regression test: `rust/runtime/tests/modulator_integration.rs::
  two_motors_on_same_bus_have_distinct_motor_idx`. Passes alongside the
  existing 3 phase-stepping integration tests (4/4 green).
- Files touched: `src/stm32/phase_stepping_spi.{h,c}`,
  `src/runtime_commands.c`, `rust/runtime/src/engine.rs`,
  `rust/runtime/src/test_xdirect_capture.rs`,
  `rust/runtime/tests/modulator_integration.rs`,
  `rust/motion-bridge/src/bridge.rs`, `klippy/motion_bridge.py`,
  `klippy/motion_toolhead.py`, `tools/test_sim_phase_stepping.py`.

**Bench verification (pending — Step 7-D scope).** Host-side and Renode
tests assert the API/dispatch shape. Confirming the *physical* CS line
goes low only for the addressed driver on a shared-bus dual-motor
config (b_y / b_y2 on SPI3) needs either a scope on each CS pin or a
DRV_STATUS readback round-trip per motor on the H723. Tracked in the
Step 7-D bench backlog; non-gating for this merge since the existing
single-TMC5160-per-bus bench config (X on SPI1, Y on SPI3) is
unaffected.
