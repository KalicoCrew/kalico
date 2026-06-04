# STM32G0B1 (BTT EBB SB2209) build target — design record

> 2026-05-31. Adds a third MCU build target: STM32G0B1 (Cortex-M0+, `thumbv6m-none-eabi`),
> the chip on the BTT EBB SB2209 toolhead board, flashed and connected over **USB**.
> Goal of this milestone: the firmware builds, links the Rust motion engine, flashes,
> enumerates over USB, completes the Klipper MCU handshake, **and ticks the motion engine
> at a low rate** — sized tiny so it fits and runs. It is a bring-up target, not a
> performance target.

## Why this is not "just add a Kconfig"

The C side of Klipper already supports STM32G0B1 (Kconfig entry, USB-FS, `cortex-m0plus`
CFLAGS). f32 soft-float already covers the no-FPU case (`libm` on every MCU). Memory
placement already falls through to the F4 `.bss` path (no AXI SRAM on G0). So the *build
wiring* is small. The cost is that **Cortex-M0+ is ARMv6-M, a different ISA than the
ARMv7-M (`thumbv7em`) used by H7/F4**, and three engine assumptions are ARMv7-M-specific:

1. **No atomic read-modify-write.** ARMv6-M has no `LDREX`/`STREX`, so
   `core::sync::atomic`'s `fetch_add` / `compare_exchange` / `swap` do not exist on
   `thumbv6m`. The MCU-compiled crates (`runtime`, `kalico-c-api`) use these in ~15 sites.
2. **No TIM5.** The motion ISR timer is hardcoded to TIM5 in `runtime_tick_{h7,f4}.c`.
   G0B1 has no TIM5.
3. **No DWT/CYCCNT.** ARMv6-M has no DWT cycle counter. The engine's internal time-base
   (`clock.rs`) is the *CYCCNT-widened* `now: u64`, so a missing cycle source is not just a
   loss of profiling — it freezes engine time and motion never advances.

This matches the `docs/kalico-rewrite/mcu-c-rust-boundary.md` "When to revisit" trigger:
*"An MCU target with a different C compiler / linker model lands … Boundary assumptions
about ELF sections and ARMv7-M atomics need re-checking."* This is that re-check.

## Decisions

### Atomics → `portable_atomic` + single-core cfg
Route the RMW sites in `runtime` and `kalico-c-api` through `portable_atomic` (already a
dependency of `runtime`; `state.rs` uses `portable_atomic::AtomicU64`). Build the
`thumbv6m` target with `--cfg portable_atomic_unsafe_assume_single_core`, which lowers RMW
to brief interrupt-disable critical sections. This is **sound** — the MCU is single-core
and the engine's atomics are for ISR↔foreground coordination, exactly what interrupt
masking serializes. The cfg is scoped to `[target.thumbv6m-none-eabi]` only, so **H7/F4
are unaffected**: `portable_atomic` compiles to native `LDREX`/`STREX` on `thumbv7em`, and
the 64-bit `AtomicU64` path is already `portable_atomic` today. Ordering semantics (B5 of
the boundary doc) are preserved — `portable_atomic` honors the same `Ordering` arguments.

`nurbs` needs no change (its only `swap` is a slice element swap, not atomic).

The C-side segment queue (`kalico_segment_queue.c`) uses atomic load/store for head/tail
(fine on ARMv6-M) and `atomic_fetch_add` only on two **single-writer** diagnostic counters
(`enqueue_total` is producer-only, `dequeue_total` is consumer-only). Those become plain
`volatile` increments — correct on all targets, no libatomic dependency on `thumbv6m`.

### Motion timer → configurable alias, default TIM7 on G0
Introduce a per-architecture **timer alias** so the physical timer for the motion ISR is
chosen per board rather than hardcoded. A Kconfig choice `KALICO_MOTION_TIMER` selects it
(default TIM5 on H7/F4 — unchanged; default **TIM7** on G0), and a header
(`src/stm32/runtime_tick_timer.h`) maps the choice to the concrete
`{TIM instance, IRQn, IRQ-handler name, RCC-enable}` per MCU family.

**Why TIM7 on G0B1:** TIM2 is Klipper's own 32-bit scheduler clock (`stm32f0_timer.c`).
TIM3/TIM4 share one IRQ vector and are PWM-capable (contention risk). TIM6/TIM7 are *basic*
timers with no output channels, so `hard_pwm` can never claim them by construction. TIM7's
only shared-vector neighbor is LPTIM2, which a toolhead board never uses. TIM7 is on APB1
at the full 64 MHz. (16-bit ARR is fine: at 64 MHz a low sample rate fits without a
prescaler down to ~977 Hz; below that the init sets a prescaler.)

### CYCCNT → software counter driven by the motion ISR
On G0, `runtime_cyccnt_read()` returns a software counter that the motion ISR advances by
`runtime_clock_freq / CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ` cycles per fire. This is exactly
the mechanism the existing `CONFIG_KALICO_SIM` path uses (Renode also returns 0 for DWT).
It gives the widening clock a monotonic source at real cadence, so segment timing advances
correctly. No DWT register is touched on G0. When the timer is disabled (idle), the clock
freezes — identical to existing F4 behavior; re-enable reseeds via the existing
`WidenState::reinit`/`seed` path.

### Sizing → tiny
G0 defaults to `RUNTIME_TARGET_SMALL` with a small curve pool, small `rt_storage`, and a
low sample rate (a few kHz). The 64 MHz M0+ with soft-float cannot run the evaluator fast;
the milestone goal is "fits and runs," not throughput. Exact values are set in the
`.config.g0b1` snapshot and Kconfig G0 defaults.

## Build wiring
- `rust/rust-toolchain.toml`: add `thumbv6m-none-eabi`.
- `rust/.cargo/config.toml`: add `[target.thumbv6m-none-eabi]` — `target-cpu=cortex-m0plus`,
  `link-arg=--nmagic`, `--cfg portable_atomic_unsafe_assume_single_core`.
- `nurbs` / `runtime` / `kalico-c-api` `Cargo.toml`: add `mcu-g0` feature (mirrors `mcu-f4`,
  pulls `libm`). Extend the "exactly one of host/mcu-*" `compile_error!` gates to include it.
- `src/Makefile`: add a `CONFIG_MACH_STM32G0` branch (`KALICO_RUST_FEATURES := mcu-g0,…`,
  `KALICO_CARGO_TARGET_DIR := target-g0`) and parameterize the rustc target triple
  (`thumbv6m-none-eabi` for G0, `thumbv7em-none-eabi` otherwise) in the `KALICO_LIB` path,
  the `cargo --target` flag, and the cargo-clean list.
- `src/stm32/Makefile`: select `runtime_tick_g0.c` for `CONFIG_MACH_STM32G0`.
- `src/Kconfig`: `KALICO_MOTION_TIMER` choice; allow `KALICO_MOTION_SAMPLE_RATE_HZ` on
  `MACH_STM32G0` with a low default; default G0 to the small runtime profile.

## New / changed C files
- `src/stm32/runtime_tick_timer.h` — timer alias mapping (new).
- `src/stm32/runtime_tick_g0.c` — G0 motion-timer init + ISR, software CYCCNT, no DWT (new;
  templated from `runtime_tick_f4.c`).
- `src/kalico_segment_queue.c` — single-writer counters to `volatile` increments.

## Out of scope for this milestone
Phase stepping / current synthesis on G0, performance tuning, CAN transport (USB only here),
hardware profiling/bench (no DWT). These are deliberately deferred.

## Verification
1. Local: `cargo build -p kalico-c-api --no-default-features --features mcu-g0,header-nurbs,header-runtime --target thumbv6m-none-eabi --release` succeeds (validates atomics + features off-bench).
2. Pi: full firmware build for the G0B1 config (arm-none-eabi-gcc + cross-compiled staticlib).
3. Bench: flash over USB via katapult, confirm USB enumeration + Klipper `identify`/config
   handshake, then confirm the motion ISR ticks (engine time advances; no fault on zero motion).
