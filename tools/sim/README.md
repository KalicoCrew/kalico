# Renode H7 Simulator for kalico runtime

Lets you boot the kalico firmware in [Renode](https://renode.io/) on the dev
host and talk to it over a TCP socket — no DFU cycles, no risk to real
hardware. Useful for state-machine bring-up, FFI symbol checks, and quick
iteration on small commands.

**Status: v0.1, Phase-0 Gate A passing.** `identify`,
`kalico_query_status`, `kalico_load_curve`, and `kalico_push_segment` all
work end-to-end. Engine state advances, segments retire, trace samples flow.
The two Step-5 known-broken paths (load_curve hang; CYCCNT freeze) closed in
Step-6 Phase 0 — see [Phase-0 fixes](#phase-0-fixes-step-6).

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

# 3. Talk to the sim. host_io accepts pyserial URL syntax:
python3 tools/test_h723_first_light.py --port socket://localhost:3334
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

## Phase-0 fixes (Step 6)

Two Step-5-leftover sim issues closed in Phase 0 of the
[Step-6 plan](../../docs/superpowers/plans/2026-04-28-step6-comm-protocol-and-sim-fixes.md).
Both are documented here so future debugging knows what to NOT chase again.

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
   in `src/stm32/kalico_h7_timer.c::kalico_h7_read_cyccnt()` returns a
   software counter (`kalico_sim_cyccnt` in `src/stm32/kalico_sim_clock.c`)
   bumped from the TIM5 ISR by `kalico_clock_freq / 40000` cycles per fire.
   Production builds (CONFIG_KALICO_SIM=n) read `DWT->CYCCNT` directly.

A third Phase-0 deliverable, the `kalico_load_fixture_curve` escape hatch
(spec §3.2), is wired through but not strictly required: with the FPU fix
above, the regular `kalico_load_curve` path works in sim. The escape hatch
remains as a backup if Renode regresses — gated on the `kalico-sim` Cargo
feature, which is gated on `CONFIG_KALICO_SIM=y`. NEVER flash a
`CONFIG_KALICO_SIM=y` image to silicon — IWDG-disable + sim CYCCNT +
kalico-sim FFI surface is a debugging build only.

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
   hardware. The `kalico_bench_*` commands are out of scope for sim
   validation per spec §3.

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

## Next steps if continuing this work

- Replace the H743 .repl with a derived H723 variant if Renode upstreams one.
- Wire Gate B (Step-6 spec §3.3) once §7/§8/§9 of Step 6 land — exercises
  status-frame, underrun fault, trace-overflow fault.
- A `make sim` target in `src/Makefile` would be a nice convenience now
  that the path works end-to-end.
