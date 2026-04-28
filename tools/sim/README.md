# Renode H7 Simulator for kalico runtime

Lets you boot the kalico firmware in [Renode](https://renode.io/) on the dev
host and talk to it over a TCP socket — no DFU cycles, no risk to real
hardware. Useful for state-machine bring-up, FFI symbol checks, and quick
iteration on small commands.

**Status: v0, partial fidelity.** Works for `identify`,
`kalico_query_status`, and a single `kalico_push_segment` (which does
return success). After the first push, TIM5 is enabled and the ISR
starts firing, which appears to wedge the simulated firmware — every
subsequent command times out. `kalico_load_curve` independently hangs
the firmware regardless of TIM5 state. Useful for FFI-symbol checks,
identify-handshake regressions, and the runtime-init path; **not yet
usable for full integration testing**. See
[Known limitations](#known-limitations) before relying on it.

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

## Known limitations

1. **`kalico_load_curve` hangs the firmware in sim.** Sending the
   `straight_line_x` fixture (msgblock 59 bytes, well under MESSAGE_MAX=64)
   causes the foreground command-dispatch path to never return. After the
   hang, all subsequent commands time out. Watchdog has been disabled, so
   the sim doesn't reset — it just sits there. Hypothesis: the %*s buffer
   handling reads memory the H743 .repl doesn't fully model, hitting a
   hard-fault that the DefaultHandler swallows in an infinite loop. Needs
   GDB-attached investigation (Renode supports it via `machine
   StartGdbServer`).

   **Update during bring-up:** even with TIM5 disabled, the load_curve
   handler stops responding after sending. Issue is independent of the
   ISR path.

2. **DWT/CYCCNT reads return 0.** Engine state-machine transitions that
   depend on widened cycle time won't progress; segment-end Drained transitions
   don't fire. `kalico_push_segment` returns success, but enabling TIM5 in
   the producer-protocol post-push path appears to wedge the firmware
   (subsequent commands time out). Likely related: the TIM5 ISR calls
   `kalico_h7_read_cyccnt()` which always returns 0, so the engine widening
   loop ingests garbage time and may hit an unexpected code path. Renode-side
   fix would be a small DWT model peripheral that returns
   `cpu.ExecutedInstructions` mod 2^32; firmware-side fix would be a
   software CYCCNT incremented from the TIM5 handler under CONFIG_KALICO_SIM.

3. **Renode's IWDG model misbehaves.** We work around by skipping
   `watchdog_init` / kicks via CONFIG_KALICO_SIM=y. Never flash an image
   built this way to real silicon — IWDG is the only thing that catches a
   hung MCU mid-print, and disabling it is unsafe.

4. **H723 platform model is approximated by H743.** Same Cortex-M7 core,
   same TIM5/USART/NVIC layout, but H723 has fewer peripherals and tighter
   timing. Renode hasn't shipped an H723-specific .repl as of v1.16.1.

5. **Renode runs slower than wall-clock** (typically 0.05x–0.5x of real
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

- Track down the load_curve hang. Connect Renode's GDB server (already
  exposed at port 3333 if you uncomment `machine StartGdbServer` in the
  .resc), break in, see whether the CPU is in DefaultHandler. If it's a
  hard-fault from a Renode-unmodeled peripheral access, the analyzer will
  show which address triggered it.
- Replace the H743 .repl with a derived H723 variant if Renode upstreams one.
- Consider a Renode peripheral model for DWT->CYCCNT that increments based
  on `cpu.ExecutedInstructions`, so the engine widening loop sees forward
  progress.
- A `make sim` target in `src/Makefile` would be a nice convenience but is
  blocked on the load_curve issue — no point streamlining a path that
  doesn't fully work.
