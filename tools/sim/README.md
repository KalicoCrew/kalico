# Renode H7 Simulator for kalico runtime

Lets you boot the kalico firmware in [Renode](https://renode.io/) on the dev
host and talk to it over a TCP socket — no DFU cycles, no risk to real
hardware. Useful for state-machine bring-up, FFI symbol checks, and quick
iteration on small commands.

The legacy msgproto streaming surface this harness originally validated has
been retired (streaming now goes over the kalico native transport); the
current driver is `tools/test_renode_phase2_gate.py` via
`tools/sim/run_phase2_gate.sh`. Two Renode platform quirks (FPU disabled;
CYCCNT frozen) are fixed in the .resc/sim build — see
[Renode sim quirks](#renode-sim-quirks-fixed-documented-so-nobody-chases-them-again).

## Prerequisites

```bash
# Renode itself.
brew install renode

# arm-gcc with newlib. Brew's arm-none-eabi-gcc formula ships without
# headers; the cask gcc-arm-embedded needs sudo. Cleanest path is
# xpack-dev-tools (no sudo, extracts under $HOME):
curl -sL https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v14.2.1-1.1/xpack-arm-none-eabi-gcc-14.2.1-1.1-darwin-arm64.tar.gz \
  -o /tmp/arm-gcc.tar.gz
mkdir -p ~/.local/arm-gcc && tar xzf /tmp/arm-gcc.tar.gz -C ~/.local/arm-gcc

# Rust + the workspace toolchain pin.
rustup target add thumbv7em-none-eabi
```

## Workflow

```bash
# 1. Build the sim-flavor firmware (USART2 instead of USB-CDC; watchdog off).
bash tools/sim/build_sim_firmware.sh

# 2. Launch Renode. UART2 is bridged to tcp://localhost:3334.
bash tools/sim/run_sim.sh &

# 3. Talk to the sim (kalico_host_io accepts pyserial URL syntax, e.g.
#    socket://localhost:3334), or run the phase-2 gate:
bash tools/sim/run_phase2_gate.sh
```

## What the sim is and isn't for

**Sim is good for**

- Confirming the firmware boots, runtime initializes, and kalico symbols are
  reachable.
- Round-tripping commands without %*s buffer arguments (the producer
  protocol, status queries, etc.).
- Iterating on Rust state-machine logic that's already covered by
  `cargo test -p runtime` but you want to see end-to-end on the wire.
- Verifying the data dictionary stays consistent (identify handshake exercises
  every DECL_COMMAND).

**Sim is NOT a substitute for silicon when**

- Cycle-count benchmarks matter. Renode's `DWT->CYCCNT` reads return 0 in the
  H743 .repl; the kalico runtime widens that to a u64 that never advances,
  so segment durations don't elapse meaningfully.
- USB-CDC enumeration is part of what you're testing. We use USART2 in sim.
- IWDG behavior matters. We disable IWDG in sim builds (CONFIG_KALICO_SIM=y).
- You need to validate Surface-C bench numbers, real-time deadline
  guarantees, or anything that depends on actual cycle pacing.

## Renode sim quirks (fixed; documented so nobody chases them again)

1. **FPU silently disabled in Renode's H7 model** (was: "load_curve hangs").
   Renode's stm32h743.repl uses `cpuType: "cortex-m7"` without an FPU flag,
   and the model silently drops writes to `SCB->CPACR` (CP10/CP11 enable
   bits). Klipper's `SystemInit()` writes those bits but they don't stick,
   so any FPU instruction in the firmware (`vpush`/`vldr`/`vcmp.f32`)
   raises a UsageFault that lands in `DefaultHandler`'s infinite loop.
   GDB-attach diagnosed `CFSR.UFSR.NOCP=1` with stacked PC inside
   `runtime::engine::tick_with_current` (the FPU-register-saving function
   prologue). **Fix:** the .resc now runs `cpu FpuEnabled true` after
   `LoadPlatformDescription` to put Renode's model into an FPU-enabled
   state. Requires Renode 1.16+ (`FpuEnabled` is exposed there;
   focaltech_ft9001_zephyr.resc uses it).

2. **DWT->CYCCNT freeze** (was: "engine widening loop never advances").
   Renode tags `DWT->CYCCNT` as opaque memory; reads return 0. C-side fork
   in `src/stm32/runtime_tick_h7.c::runtime_cyccnt_read()` returns a
   software counter (`runtime_sim_cyccnt` in `src/stm32/runtime_sim_clock.c`)
   bumped from the TIM5 ISR by `kalico_clock_freq / 40000` cycles per fire.
   Production builds (CONFIG_KALICO_SIM=n) read `DWT->CYCCNT` directly.

(The `runtime_load_fixture_curve` escape hatch that once backed up the FPU
fix has been removed along with the rest of the legacy msgproto streaming
surface.) NEVER flash a `CONFIG_KALICO_SIM=y` image to silicon —
IWDG-disable + sim CYCCNT is a debugging build only.

## Known limitations

1. **Renode's IWDG model misbehaves.** We work around by skipping
   `watchdog_init` / kicks via CONFIG_KALICO_SIM=y. Never flash an image
   built this way to real silicon — IWDG is the only thing that catches a
   hung MCU mid-print, and disabling it is unsafe.

2. **H723 platform model is approximated by H743.** Same Cortex-M7 core,
   same TIM5/USART/NVIC layout, but H723 has fewer peripherals and tighter
   timing. Renode hasn't shipped an H723-specific .repl as of v1.16.1.

3. **Cycle-count benchmarks are meaningless.** Both the software CYCCNT
   path under CONFIG_KALICO_SIM and Renode's virtual-time CPU model produce
   numbers that don't map to silicon timing. Run cycle benches against real
   hardware.

4. **Renode runs slower than wall-clock** (typically 0.05x–0.5x of real
   time depending on activity). Expect tests that have `time.sleep(N)`
   on the host side to need longer timeouts when pointed at the sim.

## Files

- `h723_sim.resc` — Renode setup script. Loads H743 platform model, tags
  the H7-specific PWR/RCC registers Klipper polls during boot, bridges
  USART2 to TCP localhost:3334, loads `out/klipper.elf`.
- `run_sim.sh` — Launcher. Pass `--gui` to keep the Renode monitor window.
- `build_sim_firmware.sh` — One-shot builder for the sim-flavor firmware.
- `sim.config` — Saved Klipper `.config` for the sim build (USART2,
  CONFIG_KALICO_SIM=y, no USB).

## Other future improvements

- Replace the H743 .repl with a derived H723 variant if Renode upstreams one.
- A `make sim` target in `src/Makefile` would be a nice convenience now
  that the path works end-to-end.
