# Step 7-D — Hardware bring-up and first print

**Scope:** Surface-C cycle-budget actuals on H723, klippy booting with the production kalico bridge on real hardware, XY travel moves through the shaped planner, F4x Z integration, M1/M2/M3 soaks, calibration, and physical first print.

**Precondition:** 7-C-bridge Phase 2 complete — Renode gate passed 2026-05-02. Production H723 firmware config and build script committed (c15a3e04e).

---

## Phase breakdown

| Phase | Scope | Gate |
|-------|-------|------|
| 1 (done) | Pre-hardware: firmware config, build script compiles | `out/klipper.bin` builds, 61 KB ROM ✓ |
| 2a | First hardware contact: flash H723, klippy boots, first_light + cycle_count + M1 soak | cycle_count p99 ≤ 15 µs; M1 30-min no-FAULT |
| 2b | First real motion: XY travel moves through bridge on physical H723, step-count validation | X and Y step counts match `dist × steps_per_mm` ± 1% |
| 3 | F4x Z integration: dual-MCU klippy boot, Z moves, M3 24h clock-sync soak | Z moves correct; M3 residual ≤ 100 µs throughout |
| 4 | Calibration + first print: homing, bed leveling, extrusion unlock, print | Physical object printed |

---

## Phase 2a — First hardware contact

### 2a.1 Pre-flight checklist

Before power-on:

- [ ] arm-none-eabi-gcc installed (`~/.local/arm-gcc/xpack-arm-none-eabi-gcc-*/bin`)
- [ ] `./tools/build_production_firmware.sh` runs to completion (`out/klipper.bin` present)
- [ ] dfu-util installed (`brew install dfu-util` on macOS dev host, or `apt install dfu-util` on Pi)
- [ ] Octopus Pro H723 powered and in DFU mode (boot0 jumper + USB to dev host)
- [ ] USB-CDC serial device appears (`/dev/ttyACM0` on Linux, `/dev/cu.usbmodem*` on macOS)

### 2a.2 Flash procedure

```bash
# Build (if not already done)
./tools/build_production_firmware.sh

# Put Octopus Pro into DFU mode:
#   1. Hold BOOT0 button (or install boot0 jumper)
#   2. Press and release RESET (or power-cycle)
#   3. Release BOOT0

# Flash (DFU address for 128 KiB bootloader offset)
dfu-util -d 0483:df11 -a 0 -s 0x8020000:leave -D out/klipper.bin

# Verify: after 'leave', the MCU reboots into the application.
# USB-CDC device reappears within ~2 seconds.
```

**Serial URL for tests:** `--port /dev/ttyACM0` (Linux) or `--port /dev/cu.usbmodem*` (macOS — use `ls /dev/cu.*` to find it).

### 2a.3 Gate A — First light

Verifies the IDLE→RUNNING state-machine transition on real silicon.

```bash
python3 tools/test_h723_first_light.py \
    --port /dev/ttyACM0 \
    --clock-freq 520000000
```

Expected output: `PASS`

### 2a.4 Gate B — Cycle-count benchmark

Measures ISR tick latency. Fills the table in `docs/research/step5-h723-cycle-budget.md`.

```bash
# Standard single-round (Pass A + Pass B)
python3 tools/test_h723_cycle_count.py \
    --port /dev/ttyACM0 \
    --clock-freq 180000000 \
    --samples 512 \
    --p99-budget-us 15.0

# M2 extended soak (~1M ticks, ~4 min) — run after Gate B passes
python3 tools/test_h723_cycle_count.py \
    --port /dev/ttyACM0 \
    --clock-freq 180000000 \
    --samples 1024 \
    --m2-rounds 977 \
    --m2-stir-protocol \
    --p99-budget-us 15.0
```

Record results in `docs/research/step5-h723-cycle-budget.md`.

### 2a.5 Gate C — M1 host-stall soak (30 min)

Run on the Pi 5 (not the dev host) once the H723 is accessible over USB-CDC from the Pi.

```bash
# On the Pi 5:
python3 tools/measure_m1_host_stall.py \
    --port /dev/ttyACM0 \
    --hours 0.5 \
    --report /tmp/m1-$(date +%Y%m%d).json
```

Document `p99_us`, `p9999_us`, and `max_us` in `docs/research/step6-buffer-budget-measurements.md`.

---

## Phase 2b — First real motion

### 2b.1 Pre-flight

- [ ] Production `printer.cfg` in place (`config/kalico-trident-production.cfg`)
- [ ] TMC5160 SPI wiring correct (SCK/MISO/MOSI shared bus, CS per driver)
- [ ] klippy starts without Python exception: `python3 klippy/klippy.py config/kalico-trident-production.cfg`
- [ ] `SET_INPUT_SHAPER SHAPER_TYPE_X=smooth_mzv SHAPER_FREQ_X=180 SHAPER_TYPE_Y=smooth_mzv SHAPER_FREQ_Y=120` accepted

### 2b.2 Manual move test

```
# In Mainsail / klippy console:
G28 X Y          ; home only — verify endstop triggers, motors move correctly
G1 X50 F6000     ; shaped 50mm X move
G1 Y50 F6000     ; shaped 50mm Y move
G1 X100 Y100 F6000  ; diagonal
```

Assertions:
- No error/FAULT from bridge
- Measured move distances (belt-tooth counting / stepper count check) match expected within ± 1%
- No skipping sounds (reduce accel to 2000 if skipping occurs)

### 2b.3 Shaper validation (optional, qualitative)

Attach ADXL345 accelerometer to toolhead. Run `SHAPER_CALIBRATE AXIS=X` and `AXIS=Y`. Verify the measured resonance peaks are close to the configured 180/120 Hz. This is a confirmation, not a calibration — the shaper frequencies are baked into the trajectory at planning time.

---

## Phase 3 — F4x Z integration

### 3.1 F4x firmware

F446 Octopus uses a separate Klipper firmware. Build via standard Klipper `make menuconfig` (STM32F446, 32 KiB bootloader, USB).

The F4x MCU config section in `config/kalico-trident-production.cfg` uses `[mcu bottom]` with its own serial path.

### 3.2 Dual-MCU boot validation

```bash
python3 klippy/klippy.py config/kalico-trident-production.cfg
```

Both MCUs must enumerate and respond to `identify` within the Klipper boot sequence. Bridge init must call `init_planner` after both MCUs are claimed.

### 3.3 M3 clock-sync soak (24h, post-dual-MCU boot)

```bash
# On Pi 5:
python3 tools/measure_m3_clock_sync.py \
    --port-h723 /dev/ttyACM0 \
    --port-f4x  /dev/ttyACM1 \
    --hours 24 \
    --report /tmp/m3-$(date +%Y%m%d).json
```

Gate: `residual_max_in_window ≤ 100 µs` throughout; no arming gate trips.

### 3.4 Z moves

```
G28 Z         ; home Z via 3 endstops (probe or Z-endstop)
G1 Z5 F300    ; 5mm Z lift — verify all three Z motors step, no skew
```

---

## Phase 4 — Calibration and first print

### 4.1 Calibration sequence

1. Nozzle PID tuning: `PID_CALIBRATE HEATER=extruder TARGET=220`
2. Bed PID: `PID_CALIBRATE HEATER=heater_bed TARGET=60`
3. Z-offset via `PROBE_CALIBRATE` (Voron standard tap/klicky procedure)
4. Bed mesh: `BED_MESH_CALIBRATE`
5. Input shaper confirmation: `SHAPER_CALIBRATE` with ADXL345

### 4.2 Extrusion unlock

Bridge currently hard-rejects `de != 0` (Phase 2 design §1). To enable extrusion for first print: unlock by removing the `de != 0` guard in `rust/motion-bridge/src/bridge.rs::submit_move`. This is a deliberate Phase 2→4 transition gate, not a bug.

Coordinate with Step 9 (tanh PA) scope — first print uses E-follows-XY with zero PA (acceptable for a validation print; blob/zit artifact expected at corners until Step 9 lands).

### 4.3 First print

Test G-code: 20mm calibration cube from OrcaSlicer, processed through Step-13 compat normalizer (G1→G5 conversion), then fed to kalico live pipeline.

```bash
# Normalize G-code (Step 13 offline tool):
kalico-compat normalize input.gcode -o input_g5.gcode

# Send to Mainsail for print
```

Expected result: a physically printed object. Quality of corners is not the gate; completion without FAULT is.

---

## Surface-C measurement log

Results go into `docs/research/step5-h723-cycle-budget.md` (cycle count) and `docs/research/step6-buffer-budget-measurements.md` (M1/M3 stall + clock-sync soaks).

---

## Definition of done (7-D)

- [ ] H723 first-light PASS on real hardware
- [ ] Cycle-count table filled (p99 ≤ 15 µs both passes)
- [ ] M1 30-min soak PASS on Pi 5
- [ ] klippy boots with production Trident config (both MCUs)
- [ ] XY travel moves produce correct step counts on H723
- [ ] Z moves produce correct step counts on F4x
- [ ] M3 24h clock-sync soak PASS
- [ ] Calibration complete (Z-offset, bed mesh, PID, shaper confirmed)
- [ ] Physical first print completes without FAULT
