# Step 7-D Phase 2b — First real motion on H723 (plan)

**Date:** 2026-05-02
**Predecessor:** Phase 2a complete (Gates A/B/C PASS — see `docs/superpowers/notes/2026-05-02-step-7d-phase-2a-session.md`)
**Parent plan:** `docs/superpowers/plans/2026-05-02-step-7d-hardware-bringup.md`
**Goal:** Drive the first XY moves through the kalico bridge on the physical H723 (Octopus Pro on Trident), one gate at a time so failure modes don't compound.

The parent plan's Phase 2b section collapses motion-test into a single block. This file expands it into five gated sub-steps. Each sub-step has independent pass criteria; resume by finding the first unchecked box.

---

## 2b-1 — Config parses & klippy boots clean (no motion)

**Intent:** klippy comes up against the real H723, motion_bridge initializes, no Python exception, no MCU shutdown. No motors moving yet.

- [ ] Find actual USB-CDC path on Pi: `ls /dev/serial/by-id/` (looking for `usb-Klipper_stm32h723xx_490017000851323235363233-if00`)
- [ ] Edit `config/kalico-trident-production.cfg`:
  - [ ] Replace `serial:` placeholder with real path
  - [ ] Verify `dir_pin` polarity for `stepper_x` / `stepper_y` matches Trident wiring (may need leading `!`)
  - [ ] Verify `run_current` against motor spec
  - [ ] Verify `sense_resistor` (0.075 Ω is typical for Octopus Pro 5160 socket — check board silk)
  - [ ] If dual-motor-per-belt: uncomment `stepper_x1` / `stepper_y1` blocks
- [ ] On Pi: `sudo systemctl stop klipper`
- [ ] `python3 klippy/klippy.py config/kalico-trident-production.cfg`
- [ ] Watch `klippy.log`. Pass criteria:
  - [ ] No Python exception at boot
  - [ ] `motion_bridge: init_planner` logs success (or equivalent — see `klippy/motion_toolhead.py:355`)
  - [ ] MCU identify completes; `Stats` lines start flowing
  - [ ] No `Shutdown due to ...` in log
- [ ] Likely failure surfaces to be ready for:
  - trapq allocation in `motion_toolhead.py` (already added pre-2b-1; should be fine)
  - kinematics load on the bridge-owned toolhead
  - TMC5160 SPI init reaching H723
  - msgproto-dict handover into the bridge (`mcu.py:1311`)

## 2b-2 — TMC5160 init & motor enable

**Intent:** confirm SPI, current set, and motor-enable logic *before* trying to spin a motor.

- [ ] `STATUS`
- [ ] `DUMP_TMC STEPPER=stepper_x` — verify GCONF / CHOPCONF / DRV_STATUS readback; no SPI errors
- [ ] `DUMP_TMC STEPPER=stepper_y`
- [ ] `SET_TMC_CURRENT STEPPER=stepper_x CURRENT=1.4` — listen for audible coil engage / feel motor lock
- [ ] Same for `stepper_y`
- [ ] Catches: SPI pin mux, sense_resistor mismatch, enable polarity inverted

## 2b-3 — Endstop visibility

**Intent:** confirm endstops are wired and visible to the bridge homing path *before* driving a carriage at one.

- [ ] Park each carriage manually away from its endstop
- [ ] `QUERY_ENDSTOPS` — should report `open` for x and y
- [ ] Manually trigger X endstop by hand → `QUERY_ENDSTOPS` → expect `TRIGGERED` for x
- [ ] Same for Y
- [ ] Catches: endstop pin assignment, pull-up / inversion, endstop wiring through bridge MCU-side

## 2b-4 — Single-axis homing

**Intent:** first real motion through bridge → H723 stream lifecycle on physical hardware. Homing is the first end-to-end exercise of bridge `submit_move` + endstop-watch.

- [ ] `G28 X`
  - [ ] Smooth approach at `homing_speed=80`
  - [ ] Endstop triggers
  - [ ] Retract `homing_retract_dist=5`
  - [ ] Slow re-approach at `second_homing_speed=20`
  - [ ] Final position registered (check via `GET_POSITION`)
- [ ] `G28 Y` — same checklist
- [ ] If skipping or stalls: drop `homing_speed`, recheck `dir_pin` polarity, recheck TMC current

## 2b-5 — Open-loop G1 moves

**Intent:** travel moves at increasing F-rates to surface throughput / stream-arming issues before Phase 3 brings Z online.

- [ ] After successful `G28 X Y`:
  - [ ] `G1 X50 F6000` — listen for skipping; verify with `GET_POSITION` + ruler
  - [ ] `G1 Y50 F6000`
  - [ ] `G1 X100 Y100 F6000` — diagonal (CoreXY both motors active)
  - [ ] `G1 X10 Y10 F6000` — return; checks reverse direction
- [ ] Push F-rate up toward the configured `max_velocity=300` mm/s (i.e. `F18000`):
  - [ ] `G1 X300 F12000`
  - [ ] `G1 X10 F18000`
- [ ] No skipped steps, no `Timer too close`, no bridge stall, no `klippy.log` errors
- [ ] If issues: capture `klippy.log` slice + recent `Stats` lines for analysis

---

## Pi-side run reminders

- Pi: `dderg@trident.local`, repo at `~/klipper`, branch `sota-motion`
- Workflow: edit + commit + **push from Mac**, then **pull on Pi**. Mac is source of truth; if Pi diverged: `git reset --hard origin/sota-motion`
- Stop klippy systemd service before manual launches: `sudo systemctl stop klipper`
- USB id (Trident H723, MCU1, AB + extruder driver): `usb-Klipper_stm32h723xx_490017000851323235363233-if00`

## ⚠️ Hardware safety — DO NOT TOUCH WITHOUT EXPLICIT PERMISSION

- **F446 (`usb-Klipper_stm32f446xx_…`, MCU2, Z + heaters) has heaters physically connected.** Any firmware/config interaction with the F446 risks driving heaters open-loop. Phase 3 only, and only with the user's explicit go-ahead.
- The H723 has heaters **disconnected**, so it's the safe MCU for Phase 2b tinkering.
- Beacon (`usb-Beacon_…`) — leave alone.

## Bridge .so build prerequisite

`klippy/motion_bridge.so` is a build artifact. It's **platform-specific** (Mach-O arm64 on Mac, ELF aarch64 on Pi) — never commit either. The Pi must rebuild it locally before klippy can load the bridge:

```bash
# On Pi, from ~/klipper:
make -f Makefile.kalico motion-bridge
file klippy/motion_bridge.so      # expect ELF 64-bit LSB shared object, ARM aarch64
```

If `klippy/motion_bridge.so` is currently tracked, `.gitignore` it before commit/push, otherwise the Mac build will shadow the Pi build after every pull.

## When 2b-5 passes

- Update `docs/superpowers/plans/2026-05-02-step-7d-hardware-bringup.md` Phase 2b section with PASS marker
- Add session-end note under `docs/superpowers/notes/` covering anything surprising (mirror the Phase 2a note's structure: gate-by-gate results + "findings worth carrying forward")
- Phase 3 (F4x Z integration) becomes the next pickup
