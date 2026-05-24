---
name: kalico-sim
description: Use when asked to test firmware or host-side changes end-to-end without a physical printer, predict print time for a G-code file, reproduce motion/homing bugs in simulation, validate a branch before merging, run G-code against real firmware, or compare branch behavior (e.g. main vs feature branch). Also use when setting up, debugging, or extending the Docker-based simulator.
---

# Kalico Simulator

Full-stack Klipper/Kalico simulator that runs MACH_LINUX firmware + klippy in Docker. Two modes: **full** (real firmware, catches bugs) and **batch** (motion planner only, 300-1200x speedup, predicts print time to seconds).

## Quick Start

```bash
# From the simulator worktree or any branch that has tools/kalico-sim/:

# Self-test (generates a test G-code, runs full pipeline):
docker run --rm kalico-sim

# Predict print time for a G-code file (~300-1200x faster than real time):
docker run --rm -v /path/to/file.gcode:/gcode/print.gcode:ro \
    kalico-sim --mode batch --gcode /gcode/print.gcode

# Test a specific branch (builds firmware from that branch):
bash tools/kalico-sim/run.sh --branch sota-motion

# Full mode with a G-code file (runs through virtual SD card):
docker run --rm -v /path/to/file.gcode:/gcode/print.gcode:ro \
    kalico-sim --mode full --gcode /gcode/print.gcode --timeout 120
```

## Architecture

```
┌──────────────────── Docker container ────────────────────┐
│                                                          │
│  FULL MODE:                                              │
│  ┌──────────┐  PTY  ┌────────┐  PTY  ┌──────────┐      │
│  │ MCU H7   │◄─────►│ klippy │◄─────►│ MCU F4   │      │
│  │ (klipper │       │(Python)│       │ (klipper │      │
│  │  .elf)   │       │        │       │  .elf)   │      │
│  └──────────┘       └────────┘       └──────────┘      │
│  LD_PRELOAD:         no shim          LD_PRELOAD:       │
│  libsim_intercept    (real time)      libsim_intercept  │
│  (GPIO/SPI/PWM)                       (GPIO/SPI/PWM)    │
│                                                          │
│  BATCH MODE:                                             │
│  ┌────────────────────────────────┐                      │
│  │ klippy --debuginput --debugout │  No MCU firmware     │
│  │ Full motion planner at CPU     │  300-1200x speedup   │
│  │ speed. Exact print time.       │  Reports seconds.    │
│  └────────────────────────────────┘                      │
└──────────────────────────────────────────────────────────┘
```

## Modes

### Batch Mode (`--mode batch`)

Runs Klipper's motion planner offline at CPU speed. No MCU firmware involved. Produces exact print time by running the real acceleration/jerk/corner-velocity pipeline.

- **Speed**: 300-1200x real time (a 23-minute print predicts in ~4 seconds)
- **Output**: Print time in seconds, pass/fail
- **Use for**: Print time estimation, motion planning validation, slicer comparison
- **Handles**: Real slicer G-code (OrcaSlicer, PrusaSlicer, etc.) — auto-strips PRINT_START, EXCLUDE_OBJECT, temperature/fan commands via built-in preprocessor

### Full Mode (`--mode full`, default)

Runs real MACH_LINUX firmware + klippy with GPIO/SPI LD_PRELOAD shim. Catches firmware bugs, protocol errors, state machine issues.

- **Speed**: ~1x real time (limited by MCU step execution in Docker VM)
- **Output**: Print time, pass/fail, error details
- **Use for**: Firmware bug detection, protocol validation, branch comparison
- **Multi-MCU**: H7 + F4 both spawn and connect via PTY
- **Endstops**: Auto-triggered via step counting in the GPIO shim (after N step pulses, linked endstop GPIO triggers)

## Building the Docker Image

```bash
# Build for current branch:
docker build -t kalico-sim -f tools/kalico-sim/Dockerfile .

# Build for a specific branch (via run.sh which prepares the build context):
bash tools/kalico-sim/run.sh --branch <branch-name>
```

For branches with `CONFIG_KALICO_RUNTIME=y` (like sota-motion), the Dockerfile:
1. Installs Rust toolchain
2. Patches missing Linux stubs (`fix_linux_build.sh`)
3. Builds the Rust staticlib + motion-bridge PyO3 module
4. Links everything into `klipper-h7-sim.elf` / `klipper-f4-sim.elf`

## Files

| File | Purpose |
|------|---------|
| `tools/kalico-sim/Dockerfile` | Docker image — Ubuntu + gcc + Rust + firmware build |
| `tools/kalico-sim/run.sh` | Convenience launcher: archives branch, overlays sim tools, builds, runs |
| `tools/kalico-sim/runner.py` | Python orchestrator: spawns MCUs, klippy, monitors, reports |
| `tools/kalico-sim/preprocess_gcode.py` | Strips slicer macros for batch mode |
| `tools/kalico-sim/libvtime/libsim_intercept.c` | GPIO/SPI/PWM/IIO LD_PRELOAD shim with auto-endstop |
| `tools/kalico-sim/libvtime/libvtime.c` | Virtual time shim (shared-memory clock) |
| `tools/kalico-sim/emulators/beacon_mcu.py` | Full Beacon eddy-current probe MCU emulator |
| `tools/kalico-sim/emulators/beacon_identify_dict.py` | Beacon firmware identify dictionary |
| `tools/kalico-sim/configs/h7-sim.config` | MACH_LINUX build config for H7-flavored MCU |
| `tools/kalico-sim/configs/f4-sim.config` | MACH_LINUX build config for F4-flavored MCU |
| `tools/kalico-sim/patches/fix_linux_build.sh` | Patches sota-motion for MACH_LINUX link errors |

## Beacon MCU Emulator

The simulator includes a full Beacon eddy-current probe emulator (`emulators/beacon_mcu.py`) that speaks Klipper's msgproto wire protocol over a PTY. It emulates everything the real Beacon firmware does:

**Implemented features:**
- Full msgproto wire protocol (identify, config, finalize, clock sync)
- Delta-compressed frequency sample streaming (`beacon_data` at 1600 Hz)
- Thermal telemetry (`beacon_status` at 10 Hz — MCU temp, supply voltage, coil temp)
- Z-aware frequency model: `freq = base + coeff / (z + offset)` — frequency varies realistically with distance to bed
- NVM reads (65536-byte image with calibration sentinels)
- Proximity homing trigger: watches frequency vs threshold, fires trsync
- Contact homing trigger: fires trsync after configurable delay
- Contact query state tracking
- Accelerometer streaming (`beacon_accel_data` at 6 kSps)
- trsync protocol (config, start, trigger, set_timeout, stepper_stop_on_trigger)

**Usage:** The emulator starts automatically in full mode when klippy's config references a Beacon probe. The runner creates the Beacon PTY and passes it to klippy's config via the serial override system.

**Adjusting Z position:**
```python
beacon_stub.set_z(5.0)  # 5mm above bed — affects frequency samples
```

## GPIO Shim — How It Works

`libsim_intercept.so` intercepts `open`, `ioctl`, `read`, `write`, `close` for:
- `/dev/gpiochip*` → simulated GPIO lines (step, dir, enable, endstop)
- `/dev/spidev*` → routed to chip emulator sockets (TMC5160, MAX31865)
- `/sys/class/pwm/*` → simulated PWM (heaters, fans)
- `/sys/bus/iio/*` → simulated ADC (thermistors)

**Auto-endstop**: The shim counts rising edges on step pins. After N steps (default 50), it sets the linked endstop GPIO to triggered. After the endstop triggers, it clears after 10 retract steps. This simulates physical endstop contact during homing.

**Control socket**: Each MCU gets a `sim_control` Unix socket at `$KALICO_SIM_SOCK_DIR/sim_control` for runtime GPIO/ADC injection:
```
set_gpio_input chip=0 line=10 value=1   # trigger endstop
set_adc channel=0 value=3900            # set ADC reading
get_gpio_output chip=0 line=0           # read step pin
```

## Printer Config for Sim

The simulator generates a minimal config when none is provided. Key constraints:

- **Pin format**: `gpiochip0/gpioN` (MACH_LINUX, not STM32 `PA3`)
- **Homing speed**: ≤10 mm/s (Docker VM jitter causes "Stepper too far in past" at higher rates)
- **`[force_move]` enabled**: Allows `SET_KINEMATIC_POSITION` as homing fallback
- **`[input_shaper]` with `smooth_mzv`**: Required on sota-motion (Kalico motion bridge rejects freq=0)
- **`[virtual_sdcard]` path**: Must match the directory where G-code files are placed

## Adding a New Test

1. Create a G-code file in `tools/kalico-sim/tests/`
2. For batch mode: `docker run --rm -v /path/to/test.gcode:/gcode/t.gcode:ro kalico-sim --mode batch --gcode /gcode/t.gcode`
3. For full mode: same but `--mode full --timeout 120`
4. For branch comparison: run the same G-code against two Docker images built from different branches

## Validated Results

| Test | Status | Print Time | Wall Time | Speedup |
|------|--------|-----------|-----------|---------|
| Main: full mode (21-move SD card print) | PASS | 26.9s | 28.0s | 1.0x |
| Main: batch (80-move pattern) | PASS | 383.5s | 0.3s | 1108x |
| Main: batch (160K-line real slicer) | PASS | 1389.3s | 4.4s | 318x |
| sota-motion: full mode | FAIL | — | 1.3s | Catches timing bug |

## Common Issues

| Issue | Fix |
|-------|-----|
| "Stepper too far in past" in full mode | Docker VM jitter — use `--privileged` or reduce homing speed. Batch mode unaffected. |
| "Unknown pin chip name 'probe'" | Config references Beacon probe. Use minimal config (no `--config` flag). |
| "shaper frequency must be finite" | Add `[input_shaper]` with `shaper_freq_x/y: 50` and `shaper_type: smooth_mzv` |
| Rust linking errors on sota-motion | `fix_linux_build.sh` should run automatically. Check Dockerfile for the `RUN bash ... fix_linux_build.sh` step. |
| SIGSEGV with both LD_PRELOAD shims | Order matters: `libvtime.so:libsim_intercept.so` (vtime FIRST). |
| G-code uses PRINT_START macro | Batch mode auto-preprocesses. Full mode uses `SET_KINEMATIC_POSITION` instead. |
| "klippy exited with code 255" | Config error — check klippy log. Usually missing extruder or wrong pin names. |

## Parallel Instances

Each `docker run` gets its own isolated environment (PTYs, sockets, /dev/shm). Run as many as your CPU supports:

```bash
# Run 4 batch predictions in parallel:
for f in a.gcode b.gcode c.gcode d.gcode; do
    docker run --rm -v /path/$f:/gcode/f.gcode:ro \
        kalico-sim --mode batch --gcode /gcode/f.gcode &
done
wait
```
