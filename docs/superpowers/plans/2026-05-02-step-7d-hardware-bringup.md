# Step 7-D Hardware Bring-Up Implementation Plan

**Goal:** Physical hardware validation from first flash through first print.

**Spec:** `docs/superpowers/specs/2026-05-02-step-7d-hardware-bringup-design.md`

**Status:** Phase 2a in progress.

---

## Phase 2a — First hardware contact

### Pre-flight

- [ ] Build production firmware: `./tools/build_production_firmware.sh`
- [ ] Verify: `out/klipper.bin` present, ~61 KB ROM
- [ ] Octopus Pro in DFU mode: BOOT0 jumper + power-cycle
- [ ] Flash: `dfu-util -d 0483:df11 -a 0 -s 0x8020000:leave -D out/klipper.bin`
- [ ] USB-CDC device appears (on Linux: `/dev/ttyACM0`; set `--port` accordingly)

### Gate A — First light

- [ ] Run: `python3 tools/test_h723_first_light.py --port /dev/ttyACM0 --clock-freq 520000000`
- [ ] Observe: `PASS`
- [ ] If FAULT: check `last_err` code; re-flash if state-machine is corrupt

### Gate B — Cycle count

- [ ] Run: `python3 tools/test_h723_cycle_count.py --port /dev/ttyACM0 --clock-freq 180000000 --samples 512 --p99-budget-us 15.0`
- [ ] Record Pass A and Pass B results in `docs/research/step5-h723-cycle-budget.md`
- [ ] Run M2 extended soak: add `--m2-rounds 977 --m2-stir-protocol` (~4 min)
- [ ] Record M2 WORST_ISR_CYCLES and WORST_ISR_US in same doc

### Gate C — M1 host-stall soak (on Pi 5)

- [ ] Transfer kalico repo to Pi 5 (or pull latest branch)
- [ ] Run: `python3 tools/measure_m1_host_stall.py --port /dev/ttyACM0 --hours 0.5 --report /tmp/m1.json`
- [ ] Note Pi 5 workload during soak (Mainsail tab open, journald active)
- [ ] Record p99_us, p9999_us, max_us in `docs/research/step6-buffer-budget-measurements.md`

---

## Phase 2b — First real motion

### Pre-flight

- [ ] Edit `config/kalico-trident-production.cfg`:
  - [ ] Set `serial:` to actual USB-CDC path (`ls /dev/serial/by-id/` to find it)
  - [ ] Verify `dir_pin` polarity for stepper_x and stepper_y (may need `!` flip)
  - [ ] Verify `run_current` against motor spec (check stepper datasheet)
  - [ ] Verify `sense_resistor` (measure or check board silk: 0.075 Ω is typical for Octopus Pro 5160 socket)
  - [ ] If dual-motor-per-belt: uncomment Driver2/Driver3 blocks
- [ ] Start klippy: `python3 klippy/klippy.py config/kalico-trident-production.cfg`
- [ ] Verify: no Python exception at boot, bridge `init_planner` called

### Motion test

- [ ] In Mainsail / console:
  - [ ] `G28 X Y` — home XY, verify endstop triggers
  - [ ] `G1 X50 F6000` — X move, listen for smooth motion, no skipping
  - [ ] `G1 Y50 F6000` — Y move
  - [ ] `G1 X100 Y100 F6000` — diagonal
- [ ] Check: step counts via `DUMP_TMC STEPPER=stepper_x` register inspection, or listen/observe

---

## Phase 3 — F4x Z integration

- [ ] Build F446 firmware via standard Klipper `make menuconfig` (STM32F446, USB)
- [ ] Flash to F446 board via DFU
- [ ] Uncomment `[mcu bottom]` block in `config/kalico-trident-production.cfg`
- [ ] Update F446 serial path
- [ ] Uncomment stepper_z/z1/z2 blocks, verify pin assignments
- [ ] Start klippy with both MCUs
- [ ] Run M3 clock-sync soak: `python3 tools/measure_m3_clock_sync.py --port-h723 /dev/ttyACM0 --port-f4x /dev/ttyACM1 --hours 24`
- [ ] `G28 Z` — home Z, verify all three Z motors respond, no tilt
- [ ] `G1 Z5 F300` — verify Z lift

---

## Phase 4 — Calibration and first print

- [ ] Uncomment extruder, heater_bed, fan sections in printer.cfg
- [ ] Remove `de != 0` extrusion guard in `rust/motion-bridge/src/bridge.rs::submit_move`
- [ ] Rebuild and reinstall `libmotion_bridge.so`
- [ ] Run `PID_CALIBRATE HEATER=extruder TARGET=220`
- [ ] Run `PID_CALIBRATE HEATER=heater_bed TARGET=60`
- [ ] Run `PROBE_CALIBRATE` (Z-offset)
- [ ] Run `BED_MESH_CALIBRATE`
- [ ] Run `SHAPER_CALIBRATE AXIS=X` and `AXIS=Y` (ADXL345)
- [ ] Normalize test G-code through Step-13 compat (when available)
- [ ] Print 20mm calibration cube
- [ ] Document result in `docs/superpowers/plan-changes-log.md`
