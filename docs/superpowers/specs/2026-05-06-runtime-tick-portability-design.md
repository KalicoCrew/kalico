# Runtime-Tick Portability — Design

**Status:** draft, awaiting user review
**Author:** brainstormed 2026-05-06
**Scope:** refactor the runtime-tick subsystem so the per-MCU family backend lives behind an explicit abstract interface (Klipper-style), split the 1082-line `src/runtime_tick.c` into focused TUs, fix Rust placement-policy logic, and complete the in-flight `kalico_*` prefix phase-out within this subsystem. **No F446 backend implementation in this work** — the refactor creates the seam; the F446 backend lands in a follow-up plan.

## 1. Problem

The runtime-tick subsystem (the 40 kHz periodic call into the Rust kalico runtime, plus the C-side glue exposing it to Klipper's command system) has accreted several tangles:

1. **`src/runtime_tick.c` is 1082 lines** doing too much: command surface, Rust FFI glue, bench command, sim shims, drain task scheduling, and direct `extern` reaches into H7 timer symbols.

2. **No real abstraction.** `runtime_tick.c` calls `kalico_h7_timer_init()` / `kalico_h7_disable_tim5()` / `kalico_h7_enable_tim5()` / `kalico_h7_read_cyccnt()` by H7-specific name. The Linux host-sim "implements the abstraction" by reusing those exact same names. The abstraction is implicit in shared function names — there is no header in `src/generic/`, no contract documentation, no boundary check. New backends (e.g. F446) cannot be added without renaming.

3. **Bench tangled with platform.** Bench buffer storage lives in `src/stm32/kalico_h7_timer.c`. Bench command logic lives in `src/runtime_tick.c`. Both reach into each other via `extern`. Bench is a feature (cycle profiling), not a platform concern; it should not be coupled to one MCU family's source file.

4. **`.axi_bss` placement decision in Rust.** `RT_CELL` (in `rust/kalico-c-api/src/runtime_ffi.rs`) is tagged `link_section(".axi_bss")` for any `target_arch = "arm"`, but `.axi_bss` is only defined by the linker on H7. The placement policy is driven by the wrong signal.

5. **Stale `kalico_*` prefix.** Per the prior brainstorm, new code drops the `kalico_` prefix. The runtime-tick subsystem is the natural place to do the rename — every symbol in this subsystem moves anyway, so retaining the prefix would mean two rename passes.

6. **Build-system gate too narrow.** `src/Makefile:42-64` builds the thumbv7em Rust staticlib only under `CONFIG_MACH_STM32H7=y`. Other thumbv7em-none-eabihf targets that share the same Rust target triple (e.g. F446) cannot get the staticlib built.

## 2. Goals

1. One explicit interface header in `src/generic/runtime_tick.h` defining the per-family backend contract.
2. Per-family backend files: H7 (renamed from existing) and host-sim (renamed from existing). No F446 backend.
3. Bench logic and storage extracted to a single portable TU (`src/generic/runtime_bench.c`) behind a Kconfig switch and a weak-symbol ISR hook.
4. `src/runtime_tick.c` slimmed to the runtime lifecycle (init / drain). Klipper command surface and sim-only commands move out into focused TUs.
5. Rust `RT_CELL` placement driven by an explicit Cargo feature, not by `target_arch`.
6. `kalico_*` prefix removed from every symbol introduced or moved by this refactor — including the Rust-FFI surface symbols that this subsystem owns.
7. Build-system rule for the thumbv7em Rust staticlib widened to `MACH_STM32H7 || MACH_STM32F4` (explicit allowlist, not a blanket thumbv7em check).
8. After the refactor, the H7 firmware and the Linux host-sim both build cleanly and behave identically. **F446 firmware does NOT build in this work** — the seam exists, but no F4 backend file does.

## 3. Non-goals

1. F446 backend implementation. Out of scope; will be a follow-up plan.
2. Renaming `kalico_*` symbols outside the runtime-tick subsystem — `kalico_dispatch.c`, `kalico_demux.c`, `kalico-c-api`, `kalico-protocol`, `kalico-host-rt`, `KALICO_RUNTIME` Kconfig, and friends keep their names. A wider fork-rename pass is separate.
3. Changing the 40 kHz tick rate, the producer/consumer protocol shape, or any runtime crate behavior.
4. Restructuring the Klipper command system itself.
5. Changing Klipper's existing per-family `src/<arch>/Makefile` selection model — we follow it, not redesign it.

## 4. Architecture

### 4.1 Abstract interface (`src/generic/runtime_tick.h`)

The per-family backend contract is exactly four functions:

```c
// src/generic/runtime_tick.h
//
// Per-family runtime-tick backend interface. Implementations live in
// src/<arch>/runtime_tick_<family>.c and are selected at build time by the
// architecture-specific Makefile. The host-process simulator implementation
// lives in src/linux/runtime_tick_host.c.
//
// Lifecycle: runtime_tick_init() configures the backend (peripherals, IRQ,
// counter sources) but does NOT start ticking. runtime_tick_enable() arms
// the 40 kHz tick — the Rust producer protocol calls this on the first
// segment push. runtime_tick_disable() stops ticking and is safe to call
// from foreground at any time.
//
// runtime_cyccnt_read() returns a free-running cycle counter that wraps
// modulo 2^32. Backends MUST guarantee that consecutive calls observe
// monotone non-decreasing values modulo wrap; the runtime widens to u64
// host-side. On Cortex-M parts this is DWT->CYCCNT; the host-sim uses a
// monotonic-clock-derived counter.
//
// runtime_tick_enable() may have side effects beyond starting the tick —
// in particular, the host-sim backend seeds Klipper's stats_send_time_high
// frame from the host clock during enable(). Backend authors implementing
// for new families MUST audit their host-clock-frame seeding requirements.

#ifndef RUNTIME_TICK_H
#define RUNTIME_TICK_H

#include <stdint.h>

void runtime_tick_init(void);
void runtime_tick_enable(void);
void runtime_tick_disable(void);
uint32_t runtime_cyccnt_read(void);

#endif // RUNTIME_TICK_H
```

The four-function shape was validated by an architect-reviewer pass (2026-05-06): IRQ priority and bench hookup are deliberately NOT part of the interface. IRQ priority is per-family-internal (each family selects priorities relative to its peripheral set). Bench hookup is via weak symbol — see §4.3.

### 4.2 Per-family backend files

Per-family implementations are renamed/relocated:

- **H7**: `src/stm32/kalico_h7_timer.c` → `src/stm32/runtime_tick_h7.c`. Symbol renames: `kalico_h7_timer_init` → `runtime_tick_init`, `kalico_h7_enable_tim5` → `runtime_tick_enable`, `kalico_h7_disable_tim5` → `runtime_tick_disable`, `kalico_h7_read_cyccnt` → `runtime_cyccnt_read`. The corresponding `src/stm32/kalico_h7_timer.h` is deleted; its content (the bench buffer extern + `KALICO_BENCH_MAX_SAMPLES` macro) relocates to `src/generic/runtime_bench.h` per §4.3 (renamed `RUNTIME_BENCH_MAX_SAMPLES`).

- **Host-sim**: `src/linux/kalico_host_tick.c` → `src/linux/runtime_tick_host.c`. Same symbol renames.

After this rename, `runtime_tick.c` includes only `generic/runtime_tick.h`, never reaches into a family-specific header.

### 4.3 Bench module (`src/generic/runtime_bench.c`)

Bench moves to a single portable TU under `src/generic/`. Storage and command logic colocate; the per-family ISR provides the cycle samples via a weak-symbol hook.

```c
// src/generic/runtime_bench.c
//
// Cycle-count benchmarking for the runtime tick. The per-family runtime-tick
// ISR calls `runtime_bench_capture(uint32_t cycles_delta)` once per tick.
// This TU is selected by CONFIG_RUNTIME_BENCH; without it, the weak-symbol
// fallback `runtime_bench_capture` (a no-op in src/runtime_tick_weak.c, see
// §4.4) is linked instead.
//
// SWSR invariant: a single ISR is the only writer; foreground reads only
// after observing `count == target`. Adding a second writer or a polling
// reader breaks the invariant — touch with care.

#include <stdint.h>
#include "command.h"   // sendf, DECL_COMMAND
// ...
void runtime_bench_capture(uint32_t cycles_delta);
// (full bench command + storage moves here from runtime_tick.c + runtime_tick_h7.c)
```

New Kconfig:
```kconfig
config RUNTIME_BENCH
    bool "Cycle-count benchmark for runtime tick"
    depends on KALICO_RUNTIME && MACH_STM32H7
    default y if MACH_STM32H7
    help
      Captures per-tick cycle counts for performance regression tracking.
      Enabled by default on H7. F4 / host targets do not currently provide
      a cycle counter suitable for bench; left unselected.
```

The dependency on `MACH_STM32H7` reflects today's reality (only H7 has the cycle-counter implementation wired). Lifting it to `MACH_STM32F4` is a one-line Kconfig change once the F446 backend lands and decides whether to provide bench.

### 4.4 Optional ISR hooks via weak symbols (`src/runtime_tick_weak.c`)

The per-family ISR has two optional hooks: `runtime_bench_capture()` (when `CONFIG_RUNTIME_BENCH=y`) and `runtime_sim_isr_wake_drain()` (when `CONFIG_KALICO_SIM=y`). Production firmware should call neither.

The per-family ISR calls both unconditionally (no `#ifdef`). A small always-linked TU defines weak no-op fallbacks; the bench / sim TUs, when their Kconfig is enabled, provide strong overrides that the linker selects.

```c
// src/runtime_tick_weak.c — always linked.
//
// Weak no-op fallbacks for optional runtime-tick ISR hooks. When the
// matching CONFIG_* is on, a strong override (in src/generic/runtime_bench.c
// or src/runtime_sim_commands.c) is linked instead. Per-family ISRs call
// these unconditionally — link-time selection picks strong-when-present.

#include <stdint.h>

__attribute__((weak)) void
runtime_bench_capture(uint32_t cycles_delta)
{
    (void)cycles_delta;
}

__attribute__((weak)) void
runtime_sim_isr_wake_drain(void)
{
}
```

The H7 ISR call site:

```c
// In src/stm32/runtime_tick_h7.c TIM5_IRQHandler:
runtime_sim_isr_wake_drain();   // weak no-op unless CONFIG_KALICO_SIM=y

uint32_t before = runtime_cyccnt_read();
if (runtime_handle) runtime_handle_tick(runtime_handle, before);
uint32_t after = runtime_cyccnt_read();

runtime_bench_capture(after - before);   // weak no-op unless CONFIG_RUNTIME_BENCH=y
```

Zero runtime cost on the disabled path (the call goes to a function that returns immediately; modern Cortex-M branch prediction handles it without measurable overhead at 40 kHz). No `#ifdef` in any per-family ISR, so adding a third backend (e.g. F446) is one new file with no awareness of which optional features happen to be enabled.

**Build-system feasibility caveat.** Klipper compiles with `-fwhole-program -flto`. Under that combination, GCC can sometimes fold a weak no-op into a caller before the strong override is observed by the linker, depending on link partitioning. The pattern *does* work in practice — Klipper itself uses `__attribute__((weak))` in `src/avr/main.c:30` (`alt_stack_save`), and weak/strong override is standard in the Linux kernel and libc. But it's not deployed in this fork's build today. Implementation step 1.5 (§5) is a 30-line trial commit that proves the override resolves correctly on H7 before the rest of the refactor relies on it. If the trial fails (weak no-op linked in production despite a strong override existing), the fallback is `#ifdef`-gating at the call site — a one-commit reversal, no other spec changes needed.

### 4.5 `runtime_tick.c` slimming + sibling TU split

After bench, sim shims, and command surface move out, `src/runtime_tick.c` keeps only the runtime-lifecycle layer:

- `DECL_INIT(runtime_init)` — configures Rust runtime, calls `runtime_tick_init()`.
- `DECL_TASK(runtime_drain)` — main-loop drain pump.
- `DECL_TASK(runtime_status_drain)`, `runtime_endstop_drain` — sibling drain pumps.
- `runtime_handle` global (renamed from `kalico_rt_handle`).
- `runtime_clock_freq` const (renamed from `kalico_clock_freq`) — the Rust-extern symbol exposing the 40 kHz tick rate.

Result: ~300 lines, single responsibility (runtime lifecycle).

The other concerns split out into:

- **`src/runtime_commands.c`** (NEW) — every `DECL_COMMAND` belonging to the runtime: `query_status`, `set_homed`, `configure_axes`, `stream_open` / `arm` / `terminal` / `flush`, `clock_sync`, `query_pool_state`, `arm_endstop` / `disarm_endstop`. ~250 lines. Includes `generic/runtime_tick.h` only when commands need to manipulate the tick (they don't directly — the producer protocol does — so includes may be minimal).

- **`src/runtime_sim_commands.c`** (NEW, gated `CONFIG_KALICO_SIM`) — every `command_kalico_sim_*` plus the load-fixture shim plus the strong definition of `runtime_sim_isr_wake_drain` (per §4.4). ~150 lines.

- **`src/generic/runtime_bench.c`** — bench storage + command + strong `runtime_bench_capture` definition, per §4.3.

- **`src/runtime_tick_weak.c`** — weak no-op fallbacks for `runtime_bench_capture` and `runtime_sim_isr_wake_drain`, per §4.4. Always linked; ~25 lines total.

The endstop sampler currently in `runtime_tick.c` — `endstop_pin_table[]` storage + `kalico_endstop_sample_pins()` (called from the per-family ISR) + `command_kalico_arm_endstop` / `command_kalico_disarm_endstop` — moves to `src/runtime_commands.c`. Both the table population (commands) and the per-tick sampler (called from the ISR via `extern`) colocate. The H7 ISR's existing `extern void kalico_endstop_sample_pins(void)` reference renames in lockstep with the rest of the table in §4.7.

The shared C globals (`runtime_handle`, `runtime_clock_freq`) stay as `extern void *` / `extern const uint32_t` references shared across TUs. Per architect feedback: putting them behind accessors adds a function call to every command for zero readability gain (single-writer-at-init pattern).

### 4.6 Rust `RT_CELL` placement via Cargo feature

Replace the current `target_arch`-driven placement attribute with a Cargo feature.

In `rust/kalico-c-api/Cargo.toml`:

```toml
[features]
# ... existing ...
axi-bss-placement = []
mcu-h7 = ["nurbs/mcu-h7", "runtime/mcu-h7", "axi-bss-placement"]
mcu-f4 = ["nurbs/mcu-f4", "runtime/mcu-f4"]   # no axi-bss-placement
```

In `rust/kalico-c-api/src/runtime_ffi.rs`:

```rust
#[cfg_attr(feature = "axi-bss-placement", unsafe(link_section = ".axi_bss"))]
pub(super) static RT_CELL: RuntimeCell = ...;
```

Now the Klipper Makefile selecting `KALICO_RUST_FEATURES = mcu-h7,...` automatically pulls `axi-bss-placement`. Selecting `mcu-f4` does not — and on F4 the `RT_CELL` static lands in regular bss, where the linker has been updated (per peer spec already in flight) to define `.axi_bss` in regular RAM as a fallback. (The Cargo-feature approach makes that linker fallback redundant for the Rust side, but the linker fallback is still useful for any C-side `__attribute__((section(".axi_bss")))` symbol, e.g. the demux buffer.)

**Linux host build** selects `host,header-nurbs,header-runtime,kalico-sim` features — neither `mcu-h7` nor `mcu-f4`, so `axi-bss-placement` is off. `RT_CELL` lands in default bss. This matches today's behavior (the current `cfg_attr(target_arch = "arm", ...)` attribute is also off on Linux because `target_arch` is `x86_64` / `aarch64` there). Net change on Linux: none.

Architect-reviewer rationale: Cargo features are visible to `cargo metadata`, IDEs, doc-builds, and downstream cargo consumers; bare `--cfg` flags are invisible. Features compose with the existing `KALICO_RUST_FEATURES` Makefile pattern as a one-line addition; `--cfg` would be a parallel mechanism.

### 4.7 Klipper command + Rust FFI symbol renames

The runtime-tick subsystem owns these symbols today; this refactor renames them as part of the prefix phase-out. Every symbol moved by §4.2–4.5 is also renamed.

**This table is illustrative, not exhaustive.** Implementation step 1 (§5) produces the canonical superset by grepping `kalico_*` across `src/`, `rust/`, `klippy/`, `tools/sim_klippy/`, and the test corpora. Known additions the grep pass must capture: `kalico_irq_save` / `kalico_irq_restore` (host-tick critical-section helpers), `kalico_host_now_us` / `kalico_host_widened_clock_now`, `kalico_endstop_sample_pins` (per-tick endstop sampler called from each backend's ISR), `kalico_aligned_cps` / `kalico_aligned_knots` (Rust-side scratch buffers exposed to C), `kalico_sim_drain_calls` / `kalico_sim_drain_counter`, `kalico_liveness_ok`, and every `command_kalico_*` C function body that backs a `DECL_COMMAND` text identifier.

**Not renamed (out of subsystem scope):** the staticlib filename `libkalico_c_api.a` does NOT change — that's the Cargo crate name, deliberately untouched per §3 non-goals. The Makefile path / build artifact filenames stay as-is.

| Old name (in code) | New name |
|---|---|
| `kalico_rt_handle` | `runtime_handle` |
| `kalico_clock_freq` | `runtime_clock_freq` |
| `kalico_h7_timer_init` | `runtime_tick_init` |
| `kalico_h7_enable_tim5` | `runtime_tick_enable` |
| `kalico_h7_disable_tim5` | `runtime_tick_disable` |
| `kalico_h7_read_cyccnt` | `runtime_cyccnt_read` |
| `kalico_bench_samples_buf` | `runtime_bench_samples_buf` |
| `kalico_bench_count` | `runtime_bench_count` |
| `kalico_bench_target` | `runtime_bench_target` |
| `kalico_bench_isolate` | `runtime_bench_isolate` |
| `kalico_sim_isr_wake_drain` | `runtime_sim_isr_wake_drain` |
| `kalico_sim_cyccnt` | `runtime_sim_cyccnt` |
| `kalico_runtime_tick` (Rust FFI from C ISR into Rust) | `runtime_handle_tick` |
| `kalico_runtime_init` (Rust FFI to allocate the runtime) | `runtime_handle_create` |
| `kalico_runtime_seed_widen` (Rust FFI) | `runtime_handle_seed_widen` |

The Rust-FFI symbols use the `runtime_handle_*` prefix to disambiguate from C-side DECL_INIT / DECL_TASK function bodies that are already named `runtime_init`, `runtime_drain`, etc. — those are local C lifecycle functions, not symbols crossing the FFI boundary. The FFI surface uniformly names "operations on the runtime handle".

Klipper command-name strings (the `DECL_COMMAND` text identifiers, e.g., `"kalico_status"`, `"kalico_runtime_init"`) **also rename** — they are part of the wire surface between C and the host. Host-side dispatch paths in `klippy/extras/`, `klippy/motion_toolhead.py`, `klippy/motion_bridge.py`, and `rust/motion-bridge/` must update in lockstep. The schema-hash for these renames is naturally captured because the command-name strings are inputs to schema generation.

NOT renamed (still own their existing prefix):

- `kalico_dispatch.c`, `kalico_demux.c`, `kalico_protocol_schema.h` — these are the kalico-native protocol layer, separate subsystem, separate rename pass.
- `KALICO_RUNTIME`, `KALICO_SIM`, `KALICO_BENCH_MAX_SAMPLES` Kconfig symbols — `KALICO_RUNTIME` is the master switch; renaming it cascades into every gated #if. Out of this scope.
- The `kalico-c-api` / `kalico-protocol` / `kalico-host-rt` / `kalico-native-transport` / `kalico-runtime` Cargo crate names. Crate renames are a separate workspace-level coordination.
- `kalico_endstop_*` — endstop subsystem, separate.

### 4.8 Build-system rule widening

`src/Makefile:42-64` builds the thumbv7em Rust staticlib. Change:

Before:
```make
ifeq ($(CONFIG_MACH_STM32H7),y)
    cd rust && PATH=... cargo build -p kalico-c-api ...
endif
```

After:
```make
ifneq (,$(filter y,$(CONFIG_MACH_STM32H7) $(CONFIG_MACH_STM32F4)))
    cd rust && PATH=... cargo build -p kalico-c-api ...
endif
```

The KALICO_RUST_FEATURES selection becomes Kconfig-driven:
```make
ifeq ($(CONFIG_MACH_STM32H7),y)
    KALICO_RUST_FEATURES := mcu-h7,header-nurbs,header-runtime
else ifeq ($(CONFIG_MACH_STM32F4),y)
    KALICO_RUST_FEATURES := mcu-f4,header-nurbs,header-runtime
else ifeq ($(CONFIG_MACH_LINUX),y)
    KALICO_RUST_FEATURES := host,header-nurbs,header-runtime,kalico-sim
endif
```

The `MACH_LINUX` branch matches today's host-sim build flags exactly — preserves existing behavior unchanged.

Per architect-reviewer feedback: this is an explicit allowlist (H7 || F4), NOT a blanket thumbv7em check, until per-family Cargo features cover other thumbv7em boards (G4, F7, etc.).

After this refactor lands, the **F446 backend follow-up plan** will (a) add `src/stm32/runtime_tick_f4.c`, (b) make `src/stm32/Makefile` select it on `MACH_STM32F4`, and (c) add the F446 to `printer.cfg` integration on the test bench. The Makefile rule is already widened for it; only the C backend is missing.

## 5. Implementation ordering

Each step is a self-contained commit. Steps are sequential — earlier ones unblock later ones.

1. **Symbol catalog (no code changes).** Grep every `kalico_*` symbol name appearing in C `extern`s, Rust `extern "C"` blocks, Klipper command-name string literals, schema files, test fixtures, klipper-sim corpora, captured-log fixtures, and host-side dispatch tables. The output is the canonical superset that subsequent rename steps must cover; §4.7's table is illustrative. Audit paths: `src/`, `rust/`, `klippy/`, `tools/sim_klippy/`, `tests/`, `scripts/`, `docs/superpowers/handoff/`, plus the local `~/Developer/klipper-sim/` corpus tree if reachable.

1.5. **Weak-symbol feasibility trial (no functional change).** Add a 30-line proof-of-concept commit: a single weak `__attribute__((weak)) void runtime_weak_probe(void) {}` in a new always-linked TU + an unconditional caller in the H7 ISR + a Kconfig-gated strong override in a separate TU. Build H7 with the override TU enabled and verify the strong definition is linked (`arm-none-eabi-objdump -d out/klipper.elf | grep -A1 runtime_weak_probe` shows the strong body, not a `bx lr`-only stub). Build with override TU disabled and verify the no-op resolves. If either check fails, **abandon the weak-symbol approach** and fall back to `#ifdef`-gating per the rejected alternative in §4.4 — single-commit reversal, no other spec changes. The probe itself is removed in the same commit as step 2 (replaced by the real bench / sim hooks).

2. **Extract bench to `src/generic/runtime_bench.c`** (and `runtime_bench.h`) + add `CONFIG_RUNTIME_BENCH` Kconfig + add `src/runtime_tick_weak.c` with the weak `runtime_bench_capture` no-op + add the unconditional `runtime_bench_capture(after - before)` call to the H7 ISR. Bench symbols renamed in this step. The H7 ISR computes `after - before` and passes the delta; the bench module owns count/target state. With `CONFIG_RUNTIME_BENCH=y`, the bench module's strong override is linked; with `=n`, the weak no-op resolves. H7 builds; bench still works.

3. **Add `src/generic/runtime_tick.h`** (the four-function interface). No file moves yet; existing H7 and host backends still expose old names but `runtime_tick.c` includes the new header alongside the existing extern decls. Compiles unchanged.

4. **Rename H7 backend internal symbols only.** `src/stm32/kalico_h7_timer.c` → `src/stm32/runtime_tick_h7.c`. The four backend-interface functions rename: `kalico_h7_timer_init` → `runtime_tick_init`, `kalico_h7_enable_tim5` → `runtime_tick_enable`, `kalico_h7_disable_tim5` → `runtime_tick_disable`, `kalico_h7_read_cyccnt` → `runtime_cyccnt_read`. References to `kalico_rt_handle`, `kalico_clock_freq`, `kalico_runtime_tick`, `kalico_sim_isr_wake_drain`, `kalico_endstop_sample_pins` inside the renamed file STAY at their old names — they rename in step 6 (cross-language) and step 7 (endstop sampler relocation). `runtime_tick.c`'s extern decl block updates to the new four-function names. **No Rust extern blocks change in this step** — Rust does not call any of the four backend-interface functions; only `runtime_tick.c` (C) does. Update `src/stm32/Makefile` to select the renamed file. H7 still builds.

5. **Rename host-sim backend internal symbols.** `src/linux/kalico_host_tick.c` → `src/linux/runtime_tick_host.c` + same four-function renames. Same scope rule as step 4 — references to FFI-surface symbols stay until step 6. Host-sim still builds.

6. **Rename runtime FFI surface symbols (cross-language atomic commit).** `kalico_rt_handle` → `runtime_handle`; `kalico_clock_freq` → `runtime_clock_freq`; `kalico_runtime_tick` → `runtime_handle_tick`; `kalico_runtime_init` → `runtime_handle_create`; `kalico_runtime_seed_widen` → `runtime_handle_seed_widen`; plus any additional FFI-crossing symbols surfaced by step 1's catalog (e.g. `kalico_irq_save` / `kalico_irq_restore` / `kalico_host_now_us` / `kalico_aligned_cps` / `kalico_aligned_knots` / `kalico_liveness_ok` if they cross). Both the Rust definition/extern-block and every C `extern` reference land in one commit. The staticlib filename `libkalico_c_api.a` does NOT change (crate name unchanged per §3 non-goal).

7. **Extract command surface** to `src/runtime_commands.c` (includes the `endstop_pin_table` storage + `runtime_endstop_sample_pins` per-tick sampler; the H7 ISR's `extern` reference renames to match). Move every `DECL_COMMAND` not directly related to runtime lifecycle. Klipper command-name strings rename in lockstep with host-side dispatch (klippy + motion-bridge + tests + klipper-sim corpora flagged in step 1). Schema-hash regenerates; tests pinning the hash literal update inline.

8. **Extract sim commands** to `src/runtime_sim_commands.c` (gated `CONFIG_KALICO_SIM`). Includes the strong definition of `runtime_sim_isr_wake_drain` per §4.4. The H7 ISR's `extern` reference renames to match.

9. **Final slim** `src/runtime_tick.c` to lifecycle-only (~300 lines).

10. **Rust `RT_CELL` placement** to Cargo feature `axi-bss-placement` (selected by `mcu-h7`).

11. **Widen `src/Makefile` thumbv7em rule** to `MACH_STM32H7 || MACH_STM32F4`. (No F4 backend file yet — Makefile rule is preparatory.)

After step 11: H7 firmware and host-sim both build clean, behave identically. F446 firmware does NOT build (no `runtime_tick_f4.c`); follow-up plan adds it.

## 6. Testing

1. **Per-step H7 cross-build smoke.** After each implementation step, `make clean && make -j4` for the H7 .config produces a working `klipper.bin` with unchanged `axi_ram` size.

2. **Per-step host-sim regression.** After each step, `tools/sim_klippy/run_local.sh "G1 X10 F1000"` produces step counts matching the pre-refactor baseline.

3. **Renode soak after step 9.** Existing Renode harness exercises the H7 firmware under simulated TIM5 ISR; runs to completion.

4. **Rust unit tests.** `cargo test -p runtime` and `cargo test -p kalico-host-rt` pass throughout. `cargo test -p motion-bridge --lib` passes (modulo pre-existing pyo3 cdylib issue noted in peer plan).

5. **Schema-hash regeneration.** Step 7 changes Klipper command-name strings, which feed the schema hash. Tests that pin a hash literal update inline; this is expected.

## 7. Risks and mitigations

1. **Symbol-rename Rust-side breakage** (architect-flagged). Mitigated by Step 1: explicit grep-then-rename catalog; Step 6 lands C and Rust changes atomically.

2. **Klipper command-name string renames** propagate to host-side dispatchers AND to any test fixture / log corpus / replay tool that pins the literal strings. Step 1's catalog audit covers `tools/sim_klippy/`, `tests/`, `scripts/`, `docs/superpowers/handoff/` capture corpora, and the external `~/Developer/klipper-sim/` corpus tree. Anything that pins literals (e.g. `"kalico_query_status"`, `"kalico_arm_endstop"`, `"kalico_stream_*"`) breaks silently if missed. Step 7 lands C + klippy + motion-bridge + the catalog's fixture updates in a single commit. Test: `tools/sim_klippy/run_local.sh` validates the host↔C surface end-to-end. Klipper's auto-discovered `data_dictionary` (consumed by Moonraker / Mainsail) is opaque and self-adapting, so external host tooling is not affected.

3. **Build-rule widening on currently-unsupported MCU families.** Mitigated by §4.8's explicit allowlist (H7 || F4). The widened rule does not fire on F7/G4/G7 even if a user has `CONFIG_MACH_STM32G4=y` somewhere.

4. **`.axi_bss` ld-script triple-branch in the future.** Architect-flagged: today the ld-script has H7/non-H7. If a future MCU has its own AXI-equivalent (e.g. STM32H7B0 with different AXI sizing), the script needs a third branch. Documented in §4.6 inline comment; no code change today.

5. **SWSR bench-buffer invariant after split.** Mitigated by the SWSR comment at top of `src/generic/runtime_bench.c` (architect-flagged).

6. **`runtime_tick_enable()` host-clock-frame seeding side effect.** Documented in `src/generic/runtime_tick.h` so backend authors writing for new families know to audit their host-clock-frame seeding.

7. **Refactor in flight while peer plan (per-MCU sizing) is mid-implementation.** Peer plan touched `runtime/build.rs` and `curve_pool.rs`. This refactor doesn't touch those files; the two specs are orthogonal. If both land in the same period, they merge cleanly.

## 8. Out-of-scope follow-ups

These items are deliberately deferred and noted here for visibility:

1. **F446 backend** (`src/stm32/runtime_tick_f4.c`). Will land as a follow-up plan once this refactor merges. Implements the four §4.1 functions for F446's TIM5 + DWT, plus IRQ vector wiring.

2. **F446 hardware bring-up** (Phase 5 of the peer per-MCU sizing plan). Blocked on (1).

3. **Wider fork-rename pass.** The remaining `kalico_*` symbols in dispatch, demux, and the kalico-protocol crate names are deliberately untouched here. Separate scope.

4. **`KALICO_RUNTIME` master Kconfig rename.** Cascades into every `#if CONFIG_KALICO_RUNTIME` site. Worth doing as part of the wider rename, not here.

5. **Future MCU families** (RP2040, ATSAM, G4, F7). Each gets a new backend file under the appropriate `src/<arch>/`. Build-system rule extended explicitly per family.

## 9. References

- `src/runtime_tick.c` — current god file.
- `src/stm32/kalico_h7_timer.c` — current H7 backend.
- `src/linux/kalico_host_tick.c` — current host-sim backend.
- `rust/kalico-c-api/src/runtime_ffi.rs:62` — `RT_CELL` placement attribute.
- `src/Makefile:42-64` — thumbv7em build rule.
- `docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md` — peer spec (sizing constants, orthogonal).
- Architect-reviewer transcript 2026-05-06 (this session) — validated the 9 adjustments captured above.
