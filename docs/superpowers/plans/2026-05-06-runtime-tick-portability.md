# Runtime-Tick Portability Refactor — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the runtime-tick subsystem so per-MCU-family backends sit behind an explicit `src/generic/runtime_tick.h` interface; split the 1082-line `src/runtime_tick.c` into focused TUs; introduce weak-symbol ISR hooks for optional bench / sim drainings; move `RT_CELL` placement to a Cargo feature; complete the `kalico_*` prefix phase-out within this subsystem. **No F446 backend implementation in this work** — the seam is created here, the backend lands separately.

**Architecture:** Keep H7 firmware and host-sim building cleanly throughout. Each step is a self-contained commit that compiles and passes existing tests. Step 1.5 is a 30-line feasibility trial that gates the weak-symbol approach — if it fails, single-commit reversal to `#ifdef`-gating per spec §4.4.

**Tech Stack:** C99 (firmware), Klipper Kconfig, GNU Make, Rust 1.85 (`kalico-c-api` staticlib), arm-none-eabi-gcc 14.2 cross-toolchain.

**Spec:** `docs/superpowers/specs/2026-05-06-runtime-tick-portability-design.md`

**Branch:** `sota-motion`. Local cross-toolchain at `~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin`. Print test-bench at `dderg@trident.local` (currently powered down — hardware verification deferred).

---

## File Map

**New files:**
- `src/generic/runtime_tick.h` — abstract per-family backend interface (4 functions).
- `src/generic/runtime_bench.c` — bench storage + command + strong `runtime_bench_capture`.
- `src/generic/runtime_bench.h` — bench public surface (replaces `src/stm32/kalico_h7_timer.h`).
- `src/runtime_tick_weak.c` — weak no-op fallbacks for `runtime_bench_capture` and `runtime_sim_isr_wake_drain`.
- `src/runtime_commands.c` — every `DECL_COMMAND` not part of runtime lifecycle (incl. endstop arm/disarm + per-tick sampler).
- `src/runtime_sim_commands.c` — `command_kalico_sim_*` commands + `runtime_sim_isr_wake_drain` strong def, gated `CONFIG_KALICO_SIM`.

**Renamed files:**
- `src/stm32/kalico_h7_timer.c` → `src/stm32/runtime_tick_h7.c`
- `src/stm32/kalico_h7_timer.h` → deleted (content moved to `src/generic/runtime_bench.h`)
- `src/linux/kalico_host_tick.c` → `src/linux/runtime_tick_host.c`
- `src/linux/kalico_host_tick.h` → `src/linux/runtime_tick_host.h`

**Modified files:**
- `src/runtime_tick.c` — slimmed from 1082 → ~300 lines (lifecycle only).
- `src/Kconfig` — adds `CONFIG_RUNTIME_BENCH`.
- `src/Makefile` — Kconfig-driven `KALICO_RUST_FEATURES`; widens thumbv7em build to `MACH_STM32H7 || MACH_STM32F4 || MACH_LINUX`.
- `src/stm32/Makefile` — selects renamed `runtime_tick_h7.c`.
- `src/linux/Makefile` (if present) — selects renamed `runtime_tick_host.c`.
- `rust/kalico-c-api/Cargo.toml` — adds `axi-bss-placement` feature, `mcu-h7` selects it.
- `rust/kalico-c-api/src/runtime_ffi.rs` — `RT_CELL` placement attribute switches to `feature = "axi-bss-placement"`; FFI symbol exports renamed (Step 6).
- All `klippy/`, `rust/motion-bridge/`, `tools/sim_klippy/`, test-corpus files that reference renamed Klipper command-name strings (Step 7's catalog audit identifies them).

---

## Task 1: Symbol catalog (no code changes)

**Purpose:** Produce the canonical superset of `kalico_*` symbols that subsequent rename steps must cover. The §4.7 spec table is illustrative; this step makes it complete.

**Files:**
- Create: `docs/superpowers/handoff/2026-05-06-runtime-tick-rename-catalog.md` (working document; not user-facing, will be deleted at end of refactor)

- [ ] **Step 1: Run the audit grep across all relevant trees**

```bash
cd /Users/daniladergachev/Developer/kalico
mkdir -p docs/superpowers/handoff
{
  echo "# Runtime-tick rename catalog (Task 1)"
  echo
  echo "## C symbols defined or extern-declared (src/)"
  echo '```'
  grep -rn 'kalico_[a-z_]*' src/ --include='*.c' --include='*.h' \
    | grep -vE '^src/(kalico_dispatch|kalico_demux|kalico_protocol)' \
    | grep -vE '#include "kalico_(dispatch|demux|protocol)' \
    | sort -u
  echo '```'
  echo
  echo "## Rust symbols (rust/)"
  echo '```'
  grep -rn 'kalico_[a-z_]*' rust/ --include='*.rs' \
    | grep -v 'kalico-protocol\|kalico-dispatch\|kalico-host-rt\|kalico-c-api\|kalico-native-transport\|KalicoHostIo\|KalicoRuntime\|kalico-runtime\|kalico-sim' \
    | sort -u
  echo '```'
  echo
  echo "## DECL_COMMAND text strings (src/runtime_tick.c)"
  echo '```'
  grep -nE 'DECL_(COMMAND|INIT|TASK).*"kalico' src/runtime_tick.c
  echo '```'
  echo
  echo "## Klippy host-side references"
  echo '```'
  grep -rn 'kalico_[a-z_]*' klippy/ tools/sim_klippy/ tests/ scripts/ 2>/dev/null \
    | grep -vE 'kalico-(protocol|dispatch|host-rt|native-transport|sim)' \
    | sort -u
  echo '```'
  echo
  echo "## External klipper-sim corpus (if reachable)"
  echo '```'
  if [ -d ~/Developer/klipper-sim ]; then
    grep -rn 'kalico_[a-z_]*' ~/Developer/klipper-sim/ --include='*.py' --include='*.cfg' --include='*.json' 2>/dev/null \
      | head -50
  else
    echo "(klipper-sim corpus not present at ~/Developer/klipper-sim; skip)"
  fi
  echo '```'
} > docs/superpowers/handoff/2026-05-06-runtime-tick-rename-catalog.md
wc -l docs/superpowers/handoff/2026-05-06-runtime-tick-rename-catalog.md
```

Expected: catalog file with several hundred lines. Each subsequent rename step references this file when scoping its commit.

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/handoff/2026-05-06-runtime-tick-rename-catalog.md
git commit -m "runtime-tick refactor: capture rename catalog (Task 1)"
```

---

## Task 1.5: Weak-symbol feasibility trial

**Purpose:** Spec §4.4 + §5 require an empirical proof that `__attribute__((weak))` strong-override-from-other-TU resolves correctly under Klipper's `-fwhole-program -flto` build. Without this proof, the bench / sim hooks could silently link the no-op even when overrides exist.

**Files:**
- Create: `src/runtime_tick_weak_probe.c` (temporary; deleted in Task 2)
- Create: `src/runtime_tick_weak_probe_strong.c` (temporary; deleted in Task 2)
- Modify: `src/runtime_tick.c` — add a one-line call to the probe (temporary; reverted in Task 2)
- Modify: `src/Kconfig` — add `CONFIG_RUNTIME_WEAK_PROBE` (temporary; reverted in Task 2)
- Modify: `src/Makefile` — conditional inclusion (temporary; reverted in Task 2)

- [ ] **Step 1: Add the weak no-op fallback TU**

Create `src/runtime_tick_weak_probe.c`:

```c
// Temporary: weak-symbol feasibility probe for the runtime-tick refactor.
// Spec §4.4 + plan Task 1.5. Removed in plan Task 2.
//
// This TU defines a weak no-op symbol. A second TU
// (runtime_tick_weak_probe_strong.c) provides a strong override when
// CONFIG_RUNTIME_WEAK_PROBE=y. The H7 ISR calls runtime_weak_probe()
// unconditionally; we verify post-link that the strong body wins.

#include <stdint.h>

__attribute__((weak)) void
runtime_weak_probe(uint32_t v)
{
    (void)v;  // weak no-op
}
```

- [ ] **Step 2: Add the strong override TU**

Create `src/runtime_tick_weak_probe_strong.c`:

```c
// Temporary: strong override for runtime_weak_probe.
// Plan Task 1.5; removed in Task 2.

#include <stdint.h>

// Volatile sink so LTO cannot eliminate the side effect that proves the
// strong body executed.
volatile uint32_t runtime_weak_probe_sink = 0;

void
runtime_weak_probe(uint32_t v)
{
    runtime_weak_probe_sink = v;
}
```

- [ ] **Step 3: Add a temporary unconditional caller in the H7 ISR**

In `src/stm32/kalico_h7_timer.c`'s `TIM5_IRQHandler`, after the existing `TIM5->SR = ~TIM_SR_UIF;` ack and before any other work, add:

```c
    extern void runtime_weak_probe(uint32_t);
    runtime_weak_probe(0xDEADBEEF);   // Task 1.5 trial — removed in Task 2
```

- [ ] **Step 4: Add the Kconfig option**

In `src/Kconfig`, append (anywhere under the existing `KALICO_RUNTIME` block is fine):

```kconfig
config RUNTIME_WEAK_PROBE
    bool "Weak-symbol feasibility probe (temporary, Task 1.5)"
    depends on KALICO_RUNTIME
    default n
    help
      Adds a strong override for runtime_weak_probe so we can verify the
      weak-fallback / strong-override pattern resolves correctly under
      -fwhole-program -flto. Removed in plan Task 2.
```

- [ ] **Step 5: Wire the probe into the Makefile**

In `src/Makefile`, find the `src-y +=` lines that build platform-agnostic sources (around the top), and add:

```make
src-y += runtime_tick_weak_probe.c
src-$(CONFIG_RUNTIME_WEAK_PROBE) += runtime_tick_weak_probe_strong.c
```

- [ ] **Step 6: Build with probe disabled, verify weak no-op resolves**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -5
arm-none-eabi-objdump -d out/klipper.elf | grep -A4 '<runtime_weak_probe>:' | head -8
```

Expected: build succeeds. Disassembly shows `runtime_weak_probe`'s body is the no-op — typically a single `bx lr` (return) instruction or an empty body.

- [ ] **Step 7: Build with probe enabled, verify strong override wins**

```bash
# Enable in .config without invoking menuconfig:
sed -i.bak '/CONFIG_KALICO_RUNTIME=y/a CONFIG_RUNTIME_WEAK_PROBE=y' .config
make olddefconfig 2>&1 | tail -2
grep RUNTIME_WEAK_PROBE out/autoconf.h

make clean && make -j4 2>&1 | tail -5
arm-none-eabi-objdump -d out/klipper.elf | grep -A6 '<runtime_weak_probe>:' | head -10
```

Expected: build succeeds. `CONFIG_RUNTIME_WEAK_PROBE 1` in autoconf.h. Disassembly shows `runtime_weak_probe`'s body now stores into `runtime_weak_probe_sink` (look for `str` instruction + a literal-pool reference). The strong body is the linked one, not the weak no-op.

- [ ] **Step 8: Restore .config and decide**

```bash
mv .config.bak .config
make olddefconfig 2>&1 | tail -2
grep -c RUNTIME_WEAK_PROBE out/autoconf.h   # should be 0 or unset
```

**If Step 7 disassembly shows the strong body wins:** weak symbols work in this codebase; proceed to Task 2 as written.

**If Step 7 shows the weak no-op even with `CONFIG_RUNTIME_WEAK_PROBE=y`:** **STOP.** The weak-symbol approach fails under our LTO config. Report BLOCKED to controller; spec §4.4's fallback is `#ifdef`-gating at call sites, single-commit pivot.

- [ ] **Step 9: Commit (only if Step 7 passed)**

```bash
git add src/runtime_tick_weak_probe.c src/runtime_tick_weak_probe_strong.c \
        src/stm32/kalico_h7_timer.c src/Kconfig src/Makefile
git commit -m "runtime-tick refactor: weak-symbol feasibility trial (Task 1.5)"
```

---

## Task 2: Extract bench module + weak-fallback TU

**Purpose:** Move bench storage and command logic to a single portable TU; introduce the always-linked weak-fallback TU; rename bench symbols to drop the `kalico_` prefix; add `CONFIG_RUNTIME_BENCH` Kconfig.

**Files:**
- Create: `src/generic/runtime_bench.c` — bench storage + command + strong `runtime_bench_capture`.
- Create: `src/generic/runtime_bench.h` — `RUNTIME_BENCH_MAX_SAMPLES` macro + extern declarations.
- Create: `src/runtime_tick_weak.c` — permanent weak fallbacks (replaces probe TUs from Task 1.5).
- Delete: `src/runtime_tick_weak_probe.c`, `src/runtime_tick_weak_probe_strong.c`.
- Modify: `src/runtime_tick.c` — remove the temporary probe call from Task 1.5 and the bench command body (~150 lines).
- Modify: `src/stm32/kalico_h7_timer.c` — remove bench buffer storage + remove the temporary probe call; add an unconditional `runtime_bench_capture(after - before)` call in the ISR.
- Modify: `src/stm32/kalico_h7_timer.h` — remove the bench-buffer extern declarations + macro.
- Modify: `src/Kconfig` — remove `CONFIG_RUNTIME_WEAK_PROBE`; add `CONFIG_RUNTIME_BENCH`.
- Modify: `src/Makefile` — remove probe TU lines; add `src-y += runtime_tick_weak.c` and `src-$(CONFIG_RUNTIME_BENCH) += generic/runtime_bench.c`.

- [ ] **Step 1: Create the bench header**

Create `src/generic/runtime_bench.h`:

```c
// src/generic/runtime_bench.h
//
// Cycle-count benchmarking for the runtime tick. Storage and command logic
// live in src/generic/runtime_bench.c (selected by CONFIG_RUNTIME_BENCH).
// The per-family ISR calls runtime_bench_capture(cycles_delta) on every
// tick; without CONFIG_RUNTIME_BENCH, the weak no-op in
// src/runtime_tick_weak.c resolves and the call is effectively free.
//
// SWSR invariant: a single ISR is the only writer; foreground reads only
// after observing `runtime_bench_count == runtime_bench_target`. Adding a
// second writer or a polling reader breaks the invariant — touch with care.

#ifndef RUNTIME_BENCH_H
#define RUNTIME_BENCH_H

#include <stdint.h>

#define RUNTIME_BENCH_MAX_SAMPLES 256

extern volatile uint32_t runtime_bench_samples_buf[RUNTIME_BENCH_MAX_SAMPLES];
extern volatile uint16_t runtime_bench_count;
extern volatile uint16_t runtime_bench_target;
extern volatile uint8_t  runtime_bench_isolate;

// Per-family ISR call site. Strong def in runtime_bench.c when
// CONFIG_RUNTIME_BENCH=y; weak no-op in runtime_tick_weak.c otherwise.
void runtime_bench_capture(uint32_t cycles_delta);

#endif // RUNTIME_BENCH_H
```

- [ ] **Step 2: Create the bench TU**

Create `src/generic/runtime_bench.c` by lifting bench command logic from `src/runtime_tick.c` (currently lines ~937-1082) and the bench buffer storage from `src/stm32/kalico_h7_timer.c` (lines 104-107). All `kalico_bench_*` symbols rename to `runtime_bench_*`.

```c
// src/generic/runtime_bench.c
//
// Bench storage + command logic. Selected by CONFIG_RUNTIME_BENCH.
// SWSR invariant per runtime_bench.h.

#include <stdint.h>
#include "command.h"             // sendf, DECL_COMMAND
#include "generic/runtime_bench.h"

// On H7 the bench buffer is placed in AXI SRAM so the 1 KB does not eat
// into the 128 KB DTCM. Other targets land in regular bss.
#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss")))
#endif
volatile uint32_t runtime_bench_samples_buf[RUNTIME_BENCH_MAX_SAMPLES];
volatile uint16_t runtime_bench_count = 0;
volatile uint16_t runtime_bench_target = 0;
volatile uint8_t  runtime_bench_isolate = 0;

void
runtime_bench_capture(uint32_t cycles_delta)
{
    if (runtime_bench_count < runtime_bench_target) {
        runtime_bench_samples_buf[runtime_bench_count] = cycles_delta;
        runtime_bench_count++;
    }
}

// (paste in here the body of command_kalico_bench_run from
// src/runtime_tick.c, renaming `kalico_bench_*` → `runtime_bench_*`,
// the command-name string `"kalico_bench_run ..."` → `"runtime_bench_run ..."`,
// and the response strings `"kalico_bench_done ..."` / `"kalico_bench_sample ..."`
// → `"runtime_bench_done ..."` / `"runtime_bench_sample ..."`.)
//
// The function name `command_kalico_bench_run` renames to
// `command_runtime_bench_run`. Constants KALICO_BENCH_OK /
// KALICO_BENCH_ERR_* rename to RUNTIME_BENCH_OK / RUNTIME_BENCH_ERR_*.
```

When pasting bench command body, refer to `src/runtime_tick.c` lines ~960-1082 in full (the `command_kalico_bench_run` function plus the error-code defines above it). Substitute every `kalico_bench_` → `runtime_bench_`. The bench command also references `kalico_liveness_ok` (defined in `src/stm32/watchdog.c`) — that symbol stays at its old name in this task; renamed in Task 6 if the catalog includes it.

- [ ] **Step 3: Create the always-linked weak fallback TU**

Create `src/runtime_tick_weak.c`:

```c
// src/runtime_tick_weak.c
//
// Always-linked weak no-op fallbacks for optional runtime-tick ISR hooks.
// When the matching CONFIG_* enables a sibling TU that provides a strong
// override (runtime_bench.c, runtime_sim_commands.c), the linker selects
// that override; otherwise these no-ops resolve. Per-family ISRs call the
// hooks unconditionally — no #ifdef in any backend.
//
// Spec §4.4. Empirical link-time selection verified by Task 1.5.

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

- [ ] **Step 4: Add `CONFIG_RUNTIME_BENCH` Kconfig; remove `CONFIG_RUNTIME_WEAK_PROBE`**

In `src/Kconfig`, replace the `RUNTIME_WEAK_PROBE` block (added in Task 1.5) with:

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

- [ ] **Step 5: Update Makefile selection**

In `src/Makefile`, replace the `RUNTIME_WEAK_PROBE` lines added in Task 1.5 with permanent selection:

```make
src-y += runtime_tick_weak.c
src-$(CONFIG_RUNTIME_BENCH) += generic/runtime_bench.c
```

Delete `src/runtime_tick_weak_probe.c` and `src/runtime_tick_weak_probe_strong.c`.

- [ ] **Step 6: Slim runtime_tick.c — remove bench command body and probe call**

In `src/runtime_tick.c`:
- Remove the temporary probe `extern void runtime_weak_probe(uint32_t)` + call added in Task 1.5 step 3.
- Remove lines ~937-1082 (the bench command body + error code defines + the bench buffer extern at lines 949-956). They moved to `src/generic/runtime_bench.c`.
- The `kalico_runtime_tick_counter` extern declaration used by the bench command stays in runtime_tick.c (it's the runtime FFI surface, renamed in Task 6).

The bench module needs access to `kalico_runtime_tick_counter()` and `kalico_liveness_ok` for its liveness gate — add `extern` decls at the top of `src/generic/runtime_bench.c` for both.

- [ ] **Step 7: Slim H7 backend — remove bench buffer storage + probe; add bench hook**

In `src/stm32/kalico_h7_timer.c`:
- Remove the temporary probe call added in Task 1.5 step 3.
- Remove lines 101-107 (`kalico_bench_samples_buf` storage + `kalico_bench_count` / `target` / `isolate` definitions). They moved to `src/generic/runtime_bench.c`.
- In `TIM5_IRQHandler`, replace the existing bench-capture block (lines 151-155):
  ```c
  // Bench capture (Task 27). Wraps subtract correctly modulo 2^32.
  if (kalico_bench_count < kalico_bench_target) {
      kalico_bench_samples_buf[kalico_bench_count] = after - before;
      kalico_bench_count++;
  }
  ```
  with a single unconditional hook call:
  ```c
  // Bench capture: weak no-op unless CONFIG_RUNTIME_BENCH=y.
  runtime_bench_capture(after - before);
  ```
  Add `extern void runtime_bench_capture(uint32_t cycles_delta);` near the top of the file (or `#include "generic/runtime_bench.h"`).

- [ ] **Step 8: Slim H7 timer header — remove bench declarations**

In `src/stm32/kalico_h7_timer.h`, delete lines 12-17 (the `KALICO_BENCH_MAX_SAMPLES` macro and bench-buffer extern declarations). The header now contains only the four function prototypes.

- [ ] **Step 9: Cross-build for H7, verify bench command renames work**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -10
```

Expected: build succeeds. `axi_ram` line shows ~285 KB (unchanged from before refactor — bench buffer was 1 KB on H7, moved location but same size). `arm-none-eabi-nm out/klipper.elf | grep -E 'runtime_bench_(capture|samples_buf|count)'` shows the renamed symbols.

- [ ] **Step 10: Commit**

```bash
git rm src/runtime_tick_weak_probe.c src/runtime_tick_weak_probe_strong.c
git add src/generic/runtime_bench.h src/generic/runtime_bench.c \
        src/runtime_tick_weak.c \
        src/runtime_tick.c src/stm32/kalico_h7_timer.c src/stm32/kalico_h7_timer.h \
        src/Kconfig src/Makefile
git commit -m "runtime-tick refactor: extract bench to src/generic; add weak-fallback TU"
```

---

## Task 3: Add abstract interface header

**Purpose:** Introduce `src/generic/runtime_tick.h` as the contract document. No symbol moves yet — this step only adds the header and its consumers' include directive. Each backend still exposes its current names.

**Files:**
- Create: `src/generic/runtime_tick.h` — 4-function abstract interface.
- Modify: `src/runtime_tick.c` — add `#include "generic/runtime_tick.h"`.

- [ ] **Step 1: Create the header**

Create `src/generic/runtime_tick.h`:

```c
// src/generic/runtime_tick.h
//
// Per-family runtime-tick backend interface. Implementations live in
// src/<arch>/runtime_tick_<family>.c and are selected at build time by the
// architecture-specific Makefile. The host-process simulator implementation
// lives in src/linux/runtime_tick_host.c.
//
// Lifecycle:
//   runtime_tick_init()    configures peripheral / IRQ / counter source.
//                          Does NOT start ticking. Called once at boot.
//   runtime_tick_enable()  arms the 40 kHz tick. Called by the producer
//                          protocol on first segment push. May have side
//                          effects beyond starting the tick — in particular
//                          the host-sim seeds Klipper's stats_send_time_high
//                          frame from the host clock here. New backends MUST
//                          audit their host-clock-frame seeding requirements.
//   runtime_tick_disable() stops ticking. Safe from foreground at any time.
//   runtime_cyccnt_read()  free-running cycle counter, wraps modulo 2^32.
//                          Consecutive calls observe monotone non-decreasing
//                          values modulo wrap; runtime widens to u64 host-side.
//                          DWT->CYCCNT on Cortex-M; monotonic-clock-derived
//                          on Linux.

#ifndef RUNTIME_TICK_H
#define RUNTIME_TICK_H

#include <stdint.h>

void runtime_tick_init(void);
void runtime_tick_enable(void);
void runtime_tick_disable(void);
uint32_t runtime_cyccnt_read(void);

#endif // RUNTIME_TICK_H
```

- [ ] **Step 2: Add include in runtime_tick.c**

In `src/runtime_tick.c`, near the existing platform-specific includes (around lines 19-26 currently `#include "stm32/kalico_h7_timer.h"` and `#include "linux/kalico_host_tick.h"`), add:

```c
#include "generic/runtime_tick.h"   // backend interface (consumer view)
```

Leave the existing platform-specific includes — they're removed in later steps once their symbols are renamed to match the abstract interface.

- [ ] **Step 3: Cross-build to confirm no regression**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make -j4 2>&1 | tail -3
```

Expected: build succeeds; nothing new to verify (header only declares functions; no callers yet).

- [ ] **Step 4: Commit**

```bash
git add src/generic/runtime_tick.h src/runtime_tick.c
git commit -m "runtime-tick refactor: add src/generic/runtime_tick.h interface"
```

---

## Task 4: Rename H7 backend

**Purpose:** Rename the H7 backend's four exported symbols to the abstract interface names; rename the file. **References to FFI-surface symbols (`kalico_rt_handle`, `kalico_clock_freq`, `kalico_runtime_tick`, `kalico_sim_isr_wake_drain`, `kalico_endstop_sample_pins`) inside the renamed file STAY at their old names.** They rename later (Tasks 6 / 7 / 8). No Rust extern blocks change in this task — Rust does not call any of the four backend-interface functions directly; only `runtime_tick.c` (C) does.

**Files:**
- Rename: `src/stm32/kalico_h7_timer.c` → `src/stm32/runtime_tick_h7.c`.
- Modify: `src/stm32/kalico_h7_timer.h` → keep at this filename for now (only renamed if Task 1's catalog says no cross-tree references remain that we'd rather not break in this commit). Update its content per below.
- Modify: `src/runtime_tick.c` — replace `kalico_h7_*` extern decls with the new names; remove the now-redundant `#include "stm32/kalico_h7_timer.h"`.
- Modify: `src/stm32/Makefile` — select renamed file.

- [ ] **Step 1: Rename H7 backend file**

```bash
git mv src/stm32/kalico_h7_timer.c src/stm32/runtime_tick_h7.c
```

- [ ] **Step 2: Rename four backend-interface symbols inside the renamed file**

In `src/stm32/runtime_tick_h7.c`, apply these substitutions:
- `kalico_h7_timer_init` → `runtime_tick_init`
- `kalico_h7_enable_tim5` → `runtime_tick_enable`
- `kalico_h7_disable_tim5` → `runtime_tick_disable`
- `kalico_h7_read_cyccnt` → `runtime_cyccnt_read`

Use targeted sed (the file has ~150 lines, all four symbols appear ~2-4 times each):

```bash
sed -i.bak \
    -e 's/kalico_h7_timer_init/runtime_tick_init/g' \
    -e 's/kalico_h7_enable_tim5/runtime_tick_enable/g' \
    -e 's/kalico_h7_disable_tim5/runtime_tick_disable/g' \
    -e 's/kalico_h7_read_cyccnt/runtime_cyccnt_read/g' \
    src/stm32/runtime_tick_h7.c
rm src/stm32/runtime_tick_h7.c.bak
```

Then update the `#include` inside the renamed file:

```c
// Replace:
#include "kalico_h7_timer.h"
// With:
#include "generic/runtime_tick.h"   // interface contract
```

(The H7 backend includes the generic header so the compiler verifies the four function signatures match the contract.)

The file's first comment line `// src/stm32/kalico_h7_timer.c` updates to `// src/stm32/runtime_tick_h7.c`.

The file-level guard at the bottom `#endif // CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32H7` and the `#if CONFIG_KALICO_RUNTIME && CONFIG_MACH_STM32H7` near the top stay as-is.

References inside the file to `kalico_rt_handle`, `kalico_runtime_tick`, `kalico_clock_freq`, `kalico_sim_isr_wake_drain`, `kalico_endstop_sample_pins` STAY at their old names. Don't touch them. They rename in later tasks.

- [ ] **Step 3: Update H7 timer header**

The remaining content in `src/stm32/kalico_h7_timer.h` is the four-function prototype block. Since those four functions now match `src/generic/runtime_tick.h`, the H7 header is redundant. Delete it:

```bash
git rm src/stm32/kalico_h7_timer.h
```

- [ ] **Step 4: Update runtime_tick.c — replace extern decls + remove old include**

In `src/runtime_tick.c`:
- Remove `#include "stm32/kalico_h7_timer.h"` (line ~20).
- Remove `#include "linux/kalico_host_tick.h"` IF the file uses ONLY the four backend-interface functions through it (check first; the host-sim header may declare other symbols renamed in Task 5 — leave the include in place for now if so).
- Anywhere `kalico_h7_timer_init` / `kalico_h7_enable_tim5` / `kalico_h7_disable_tim5` / `kalico_h7_read_cyccnt` appear (search with `grep -nE 'kalico_h7_(timer_init|enable_tim5|disable_tim5|read_cyccnt)' src/runtime_tick.c`), replace with the runtime-interface names.

Specifically the `extern void kalico_h7_timer_init(void);` declarations at lines 181-182 become single calls through `generic/runtime_tick.h`:

```c
runtime_tick_init();
```

The line at 341 `kalico_h7_disable_tim5();` becomes `runtime_tick_disable();`. Etc.

- [ ] **Step 5: Update src/stm32/Makefile**

In `src/stm32/Makefile`, find the line that selects the H7 timer file (`src-$(CONFIG_MACH_STM32H7) += stm32/kalico_h7_timer.c` or similar; grep for `kalico_h7_timer`). Change it to:

```make
src-$(CONFIG_MACH_STM32H7) += stm32/runtime_tick_h7.c
```

If the Makefile lists `kalico_h7_timer.c` more than once, replace every occurrence.

- [ ] **Step 6: Cross-build for H7, verify nothing broke**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -8
```

Expected: build succeeds. axi_ram unchanged. `arm-none-eabi-nm out/klipper.elf | grep -E '(runtime_tick_(init|enable|disable)|runtime_cyccnt_read)'` shows the four renamed symbols. The same `nm` should NOT show `kalico_h7_*` symbols (other than transitively in compile-time-request data — that's harmless).

- [ ] **Step 7: Commit**

```bash
git add src/stm32/runtime_tick_h7.c src/runtime_tick.c src/stm32/Makefile
git rm src/stm32/kalico_h7_timer.h
git commit -m "runtime-tick refactor: rename H7 backend to runtime_tick_h7"
```

---

## Task 5: Rename host-sim backend

**Purpose:** Same rename pass for the Linux host-sim backend. Symbol scope same as Task 4 (only the four backend-interface symbols; FFI-surface references stay).

**Files:**
- Rename: `src/linux/kalico_host_tick.c` → `src/linux/runtime_tick_host.c`.
- Rename: `src/linux/kalico_host_tick.h` → delete (its content is the same four-function prototypes, now redundant with `src/generic/runtime_tick.h`).
- Modify: `src/runtime_tick.c` — remove the now-redundant `#include "linux/kalico_host_tick.h"` if not already removed in Task 4.
- Modify: `src/linux/Makefile` — update the file selection.

- [ ] **Step 1: Rename host-sim file**

```bash
git mv src/linux/kalico_host_tick.c src/linux/runtime_tick_host.c
```

- [ ] **Step 2: Apply the four-symbol rename inside the renamed file**

```bash
sed -i.bak \
    -e 's/kalico_h7_timer_init/runtime_tick_init/g' \
    -e 's/kalico_h7_enable_tim5/runtime_tick_enable/g' \
    -e 's/kalico_h7_disable_tim5/runtime_tick_disable/g' \
    -e 's/kalico_h7_read_cyccnt/runtime_cyccnt_read/g' \
    src/linux/runtime_tick_host.c
rm src/linux/runtime_tick_host.c.bak
```

Update the file's `#include` block:

```c
// Replace:
#include "kalico_host_tick.h"
// With:
#include "generic/runtime_tick.h"
```

The file's first comment line `// src/linux/kalico_host_tick.c` updates to `// src/linux/runtime_tick_host.c`.

References inside the file to `kalico_rt_handle`, `kalico_runtime_tick`, `kalico_clock_freq`, `stats_send_time_high` (Klipper's host-clock-frame) stay at their old names. Task 6 renames the FFI-surface ones; `stats_send_time_high` is Klipper-internal and not renamed.

- [ ] **Step 3: Delete the host-sim header**

```bash
git rm src/linux/kalico_host_tick.h
```

- [ ] **Step 4: Update runtime_tick.c**

In `src/runtime_tick.c`, remove `#include "linux/kalico_host_tick.h"` if still present from Task 4.

- [ ] **Step 5: Update src/linux/Makefile**

In `src/linux/Makefile`, find the line that selects the host-tick file (grep `kalico_host_tick`). Change to:

```make
src-y += linux/runtime_tick_host.c
```

(The host-tick file is unconditionally part of `MACH_LINUX` builds; the existing selection scope stays.)

- [ ] **Step 6: Build host-sim to verify**

```bash
make clean
cp .config .config.h7.bak
cp .config.linux .config 2>/dev/null || cat > .config <<'EOF'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_LINUX=y
CONFIG_LINUX_SELECT=y
CONFIG_KALICO_RUNTIME=y
CONFIG_KALICO_SIM=y
CONFIG_BOARD_DIRECTORY="linux"
CONFIG_CLOCK_FREQ=50000000
CONFIG_USB=n
CONFIG_USBSERIAL=n
CONFIG_SERIAL=n
CONFIG_INLINE_STEPPER_HACK=y
EOF
make olddefconfig 2>&1 | tail -2
make -j4 2>&1 | tail -5
cp .config.h7.bak .config
make olddefconfig 2>&1 | tail -2
```

Expected: host-sim build succeeds; `out/klipper.elf` produced.

- [ ] **Step 7: Cross-build H7 to confirm no regression**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -5
```

Expected: H7 build succeeds.

- [ ] **Step 8: Commit**

```bash
git add src/linux/runtime_tick_host.c src/runtime_tick.c src/linux/Makefile
git rm src/linux/kalico_host_tick.h
git commit -m "runtime-tick refactor: rename host-sim backend to runtime_tick_host"
```

---

## Task 6: Rename runtime FFI surface symbols (cross-language atomic)

**Purpose:** Rename every `kalico_*` symbol that crosses the C↔Rust boundary or is shared across multiple TUs as a runtime global. Both Rust definition / extern blocks and every C `extern` reference land in one commit.

**Symbol list (canonical via Task 1's catalog; this list is what spec §4.7 + the catalog converge on):**

C globals:
- `kalico_rt_handle` → `runtime_handle`
- `kalico_clock_freq` → `runtime_clock_freq`
- `kalico_liveness_ok` → `runtime_liveness_ok` (if defined in `src/stm32/watchdog.c` — verify via Task 1 catalog)
- `kalico_irq_save` → `runtime_irq_save`
- `kalico_irq_restore` → `runtime_irq_restore`
- `kalico_host_now_us` → `runtime_host_now_us`
- `kalico_host_widened_clock_now` → `runtime_host_widened_clock_now`

Rust FFI exports (defined in `rust/kalico-c-api/src/runtime_ffi.rs`, called from C):
- `kalico_runtime_init` → `runtime_handle_create`
- `kalico_runtime_tick` → `runtime_handle_tick`
- `kalico_runtime_push_segment` → `runtime_handle_push_segment`
- `kalico_runtime_load_curve` → `runtime_handle_load_curve`
- `kalico_runtime_check_blob_version` → `runtime_handle_check_blob_version`
- `kalico_runtime_query_pool_state` → `runtime_handle_query_pool_state`
- `kalico_runtime_drain_trace` → `runtime_handle_drain_trace`
- `kalico_runtime_status` → `runtime_handle_status`
- `kalico_runtime_last_error` → `runtime_handle_last_error`
- `kalico_runtime_tick_counter` → `runtime_handle_tick_counter`
- `kalico_runtime_widened_now` → `runtime_handle_widened_now`
- `kalico_runtime_credit_epoch` → `runtime_handle_credit_epoch`
- `kalico_runtime_accepted_segment_id` → `runtime_handle_accepted_segment_id`
- `kalico_runtime_retired_through_segment_id` → `runtime_handle_retired_through_segment_id`
- `kalico_runtime_current_segment_id` → `runtime_handle_current_segment_id`
- `kalico_runtime_queue_depth` → `runtime_handle_queue_depth`
- `kalico_runtime_fault_detail` → `runtime_handle_fault_detail`
- `kalico_runtime_get_axis_steps_per_mm` → `runtime_handle_get_axis_steps_per_mm`
- `kalico_runtime_seed_widen` → `runtime_handle_seed_widen` (if present)

Rust externs FROM Rust to C:
- `kalico_clock_freq` (Rust extern static) → `runtime_clock_freq`

C-side scratch buffers (referenced from Rust via FFI):
- `kalico_aligned_cps` → `runtime_aligned_cps`
- `kalico_aligned_knots` → `runtime_aligned_knots`

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` — every `kalico_runtime_*` `pub extern "C" fn` and `extern "C" { static kalico_clock_freq }` block.
- Modify: `src/runtime_tick.c` — every reference to renamed symbols.
- Modify: `src/stm32/runtime_tick_h7.c` — `kalico_rt_handle`, `kalico_runtime_tick`, `kalico_clock_freq`, `kalico_liveness_ok`, `kalico_aligned_*`.
- Modify: `src/linux/runtime_tick_host.c` — `kalico_rt_handle`, `kalico_runtime_tick`, `kalico_clock_freq`.
- Modify: `src/generic/runtime_bench.c` — `kalico_runtime_tick_counter` (used by liveness gate), `kalico_liveness_ok`.
- Modify: `src/kalico_dispatch.c` — calls into the runtime via the renamed FFI exports (Task 1 catalog confirms scope).
- Modify: `src/runtime_tick_weak.c` — no changes (the two weak symbols `runtime_bench_capture` / `runtime_sim_isr_wake_drain` already use the new names).

- [ ] **Step 1: Apply the rename across C files**

Use one big sed pass over the runtime-tick consumer files (the catalog from Task 1 lists them):

```bash
FILES=$(cat <<'EOF'
src/runtime_tick.c
src/stm32/runtime_tick_h7.c
src/linux/runtime_tick_host.c
src/generic/runtime_bench.c
src/kalico_dispatch.c
EOF
)
for f in $FILES; do
  sed -i.bak \
    -e 's/\bkalico_rt_handle\b/runtime_handle/g' \
    -e 's/\bkalico_clock_freq\b/runtime_clock_freq/g' \
    -e 's/\bkalico_liveness_ok\b/runtime_liveness_ok/g' \
    -e 's/\bkalico_irq_save\b/runtime_irq_save/g' \
    -e 's/\bkalico_irq_restore\b/runtime_irq_restore/g' \
    -e 's/\bkalico_host_now_us\b/runtime_host_now_us/g' \
    -e 's/\bkalico_host_widened_clock_now\b/runtime_host_widened_clock_now/g' \
    -e 's/\bkalico_runtime_init\b/runtime_handle_create/g' \
    -e 's/\bkalico_runtime_tick_counter\b/runtime_handle_tick_counter/g' \
    -e 's/\bkalico_runtime_tick\b/runtime_handle_tick/g' \
    -e 's/\bkalico_runtime_push_segment\b/runtime_handle_push_segment/g' \
    -e 's/\bkalico_runtime_load_curve\b/runtime_handle_load_curve/g' \
    -e 's/\bkalico_runtime_check_blob_version\b/runtime_handle_check_blob_version/g' \
    -e 's/\bkalico_runtime_query_pool_state\b/runtime_handle_query_pool_state/g' \
    -e 's/\bkalico_runtime_drain_trace\b/runtime_handle_drain_trace/g' \
    -e 's/\bkalico_runtime_status\b/runtime_handle_status/g' \
    -e 's/\bkalico_runtime_last_error\b/runtime_handle_last_error/g' \
    -e 's/\bkalico_runtime_widened_now\b/runtime_handle_widened_now/g' \
    -e 's/\bkalico_runtime_credit_epoch\b/runtime_handle_credit_epoch/g' \
    -e 's/\bkalico_runtime_accepted_segment_id\b/runtime_handle_accepted_segment_id/g' \
    -e 's/\bkalico_runtime_retired_through_segment_id\b/runtime_handle_retired_through_segment_id/g' \
    -e 's/\bkalico_runtime_current_segment_id\b/runtime_handle_current_segment_id/g' \
    -e 's/\bkalico_runtime_queue_depth\b/runtime_handle_queue_depth/g' \
    -e 's/\bkalico_runtime_fault_detail\b/runtime_handle_fault_detail/g' \
    -e 's/\bkalico_runtime_get_axis_steps_per_mm\b/runtime_handle_get_axis_steps_per_mm/g' \
    -e 's/\bkalico_runtime_seed_widen\b/runtime_handle_seed_widen/g' \
    -e 's/\bkalico_aligned_cps\b/runtime_aligned_cps/g' \
    -e 's/\bkalico_aligned_knots\b/runtime_aligned_knots/g' \
    "$f"
  rm -f "$f.bak"
done
```

The `\b` word-boundary anchors prevent partial matches (e.g., `kalico_rt_handle_secondary` would not match — though that symbol doesn't exist; the anchors are defensive).

- [ ] **Step 2: Apply the rename in Rust**

In `rust/kalico-c-api/src/runtime_ffi.rs`:

```bash
sed -i.bak \
    -e 's/\bkalico_clock_freq\b/runtime_clock_freq/g' \
    -e 's/\bkalico_runtime_init\b/runtime_handle_create/g' \
    -e 's/\bkalico_runtime_tick_counter\b/runtime_handle_tick_counter/g' \
    -e 's/\bkalico_runtime_tick\b/runtime_handle_tick/g' \
    -e 's/\bkalico_runtime_push_segment\b/runtime_handle_push_segment/g' \
    -e 's/\bkalico_runtime_load_curve\b/runtime_handle_load_curve/g' \
    -e 's/\bkalico_runtime_check_blob_version\b/runtime_handle_check_blob_version/g' \
    -e 's/\bkalico_runtime_query_pool_state\b/runtime_handle_query_pool_state/g' \
    -e 's/\bkalico_runtime_drain_trace\b/runtime_handle_drain_trace/g' \
    -e 's/\bkalico_runtime_status\b/runtime_handle_status/g' \
    -e 's/\bkalico_runtime_last_error\b/runtime_handle_last_error/g' \
    -e 's/\bkalico_runtime_widened_now\b/runtime_handle_widened_now/g' \
    -e 's/\bkalico_runtime_credit_epoch\b/runtime_handle_credit_epoch/g' \
    -e 's/\bkalico_runtime_accepted_segment_id\b/runtime_handle_accepted_segment_id/g' \
    -e 's/\bkalico_runtime_retired_through_segment_id\b/runtime_handle_retired_through_segment_id/g' \
    -e 's/\bkalico_runtime_current_segment_id\b/runtime_handle_current_segment_id/g' \
    -e 's/\bkalico_runtime_queue_depth\b/runtime_handle_queue_depth/g' \
    -e 's/\bkalico_runtime_fault_detail\b/runtime_handle_fault_detail/g' \
    -e 's/\bkalico_runtime_get_axis_steps_per_mm\b/runtime_handle_get_axis_steps_per_mm/g' \
    -e 's/\bkalico_runtime_seed_widen\b/runtime_handle_seed_widen/g' \
    -e 's/\bkalico_h7_enable_tim5\b/runtime_tick_enable/g' \
    -e 's/\bkalico_h7_disable_tim5\b/runtime_tick_disable/g' \
    -e 's/\bkalico_h7_read_cyccnt\b/runtime_cyccnt_read/g' \
    rust/kalico-c-api/src/runtime_ffi.rs
rm rust/kalico-c-api/src/runtime_ffi.rs.bak
```

The `kalico_h7_*` substitutions catch the Rust extern blocks at lines 87-91 of `runtime_ffi.rs` (the H7 timer-control helpers) — those Rust externs DO call into the renamed C functions from Task 4, so they update here in lockstep with the cross-language rename.

- [ ] **Step 3: Search for residual references to renamed names**

```bash
grep -rn 'kalico_runtime_\|kalico_rt_handle\|kalico_clock_freq\|kalico_irq_\|kalico_aligned_\|kalico_host_now_us\|kalico_host_widened\|kalico_liveness' src/ rust/kalico-c-api/ 2>&1 | head -20
```

Expected: no matches (other than possibly in comments — those are fine to leave for a follow-up pass).

If `grep` finds anything: the catalog from Task 1 missed a site. Update the symbol list above and re-run sed for that file.

- [ ] **Step 4: Cross-build H7**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -10
```

Expected: build succeeds. Linker resolves all renamed symbols.

- [ ] **Step 5: Build host-sim**

```bash
make clean
cp .config .config.h7.bak
cp .config.linux .config
make olddefconfig 2>&1 | tail -2
make -j4 2>&1 | tail -3
cp .config.h7.bak .config
make olddefconfig 2>&1 | tail -2
```

Expected: host-sim builds.

- [ ] **Step 6: Run Rust unit tests**

```bash
cd rust && cargo test -p runtime --quiet 2>&1 | tail -5
cargo test -p kalico-host-rt --quiet 2>&1 | tail -5
cd ..
```

Expected: all tests pass.

- [ ] **Step 7: Commit**

```bash
git add src/runtime_tick.c src/stm32/runtime_tick_h7.c src/linux/runtime_tick_host.c \
        src/generic/runtime_bench.c src/kalico_dispatch.c \
        rust/kalico-c-api/src/runtime_ffi.rs
git commit -m "runtime-tick refactor: rename FFI surface symbols (atomic cross-language)"
```

---

## Task 7: Extract command surface to `src/runtime_commands.c`

**Purpose:** Move every `DECL_COMMAND` not part of the runtime lifecycle to a focused TU. Includes the endstop sampler hot-path (`endstop_pin_table` + `kalico_endstop_sample_pins` per-tick callback). Klipper command-name strings rename in lockstep with host-side dispatchers — Task 1's catalog identifies every host-side site.

**Files:**
- Create: `src/runtime_commands.c` — lifted commands; ~250 lines.
- Modify: `src/runtime_tick.c` — remove the lifted commands (keep only `runtime_init` DECL_INIT, `runtime_drain` DECL_TASK, sibling drains, and shared globals).
- Modify: `src/Makefile` — add `src-y += runtime_commands.c`.
- Modify: every klippy / motion-bridge / sim-tool / corpus file referenced by Task 1's catalog that pins a Klipper command-name string.

**Klipper command-name string renames** (per spec §4.7 + Task 1's catalog enumeration):

C function name | Old wire string | New wire string
---|---|---
command_kalico_query_status | `kalico_query_status` | `runtime_query_status`
command_kalico_set_homed | `kalico_set_homed` | `runtime_set_homed`
command_kalico_set_homed_state | `kalico_set_homed_state homed=%c` | `runtime_set_homed_state homed=%c`
command_kalico_disarm_endstop | `kalico_disarm_endstop arm_id=%u` | `runtime_disarm_endstop arm_id=%u`
command_kalico_arm_endstop | `kalico_arm_endstop ...` | `runtime_arm_endstop ...`
command_kalico_configure_axes | `kalico_configure_axes kinematics=%c` | `runtime_configure_axes kinematics=%c`
command_kalico_stream_open | `kalico_stream_open stream_id=%u` | `runtime_stream_open stream_id=%u`
command_kalico_stream_arm | `kalico_stream_arm ...` | `runtime_stream_arm ...`
command_kalico_stream_terminal | `kalico_stream_terminal ...` | `runtime_stream_terminal ...`
command_kalico_stream_flush | `kalico_stream_flush` | `runtime_stream_flush`
command_kalico_clock_sync | `kalico_clock_sync ...` | `runtime_clock_sync ...`
command_kalico_query_pool_state | `kalico_query_pool_state` | `runtime_query_pool_state`

(Confirm the full list against Task 1's catalog. Bench commands already moved in Task 2.)

C function name renames apply too: `command_kalico_query_status` → `command_runtime_query_status`, etc.

- [ ] **Step 1: Lift commands into the new TU**

Create `src/runtime_commands.c`. Its skeleton:

```c
// src/runtime_commands.c
//
// Klipper command surface for the kalico runtime. Every DECL_COMMAND that
// is not part of the lifecycle (runtime_init / runtime_drain / sibling
// drains, which stay in src/runtime_tick.c) lives here. Also hosts the
// endstop arm/disarm commands and the per-tick endstop sampler called from
// each backend's ISR.

#include <stdint.h>
#include "command.h"             // DECL_COMMAND, sendf
#include "sched.h"               // DECL_INIT, sched_wake_task
#include "kalico_runtime.h"      // FFI export prototypes
#include "kalico_dispatch.h"     // kalico_native_emit_*
#include "generic/runtime_bench.h"  // (if any command interacts with bench)

extern void *runtime_handle;     // defined in src/runtime_tick.c

// (paste each DECL_COMMAND function lifted from src/runtime_tick.c here,
// renaming both the C function name and the wire string per the table above.)
```

For each command function in `src/runtime_tick.c` between (approximately) lines 444 and 800 — `command_kalico_query_status`, `command_kalico_set_homed`, `command_kalico_set_homed_state`, `command_kalico_arm_endstop`, `command_kalico_disarm_endstop`, `command_kalico_configure_axes`, `command_kalico_stream_open`, `command_kalico_stream_arm`, `command_kalico_stream_terminal`, `command_kalico_stream_flush`, `command_kalico_clock_sync`, `command_kalico_query_pool_state`, plus the endstop sampler `kalico_endstop_sample_pins` and the `endstop_pin_table` storage — cut and paste into `src/runtime_commands.c`.

Apply the renames inside the new file:

```bash
sed -i.bak \
    -e 's/\bcommand_kalico_/command_runtime_/g' \
    -e 's/\bkalico_endstop_sample_pins\b/runtime_endstop_sample_pins/g' \
    -e 's/"kalico_query_status"/"runtime_query_status"/g' \
    -e 's/"kalico_set_homed"/"runtime_set_homed"/g' \
    -e 's/"kalico_set_homed_state homed=%c"/"runtime_set_homed_state homed=%c"/g' \
    -e 's/"kalico_arm_endstop /"runtime_arm_endstop /g' \
    -e 's/"kalico_disarm_endstop arm_id=%u"/"runtime_disarm_endstop arm_id=%u"/g' \
    -e 's/"kalico_configure_axes /"runtime_configure_axes /g' \
    -e 's/"kalico_stream_open /"runtime_stream_open /g' \
    -e 's/"kalico_stream_arm /"runtime_stream_arm /g' \
    -e 's/"kalico_stream_terminal /"runtime_stream_terminal /g' \
    -e 's/"kalico_stream_flush"/"runtime_stream_flush"/g' \
    -e 's/"kalico_clock_sync /"runtime_clock_sync /g' \
    -e 's/"kalico_query_pool_state"/"runtime_query_pool_state"/g' \
    src/runtime_commands.c
rm src/runtime_commands.c.bak
```

Confirm via Task 1's catalog that every wire-string entry matches. If any are missing, add them to the sed script.

- [ ] **Step 2: Remove the lifted commands from runtime_tick.c**

In `src/runtime_tick.c`, delete every function definition (and its `DECL_COMMAND`) lifted in Step 1. Also delete the `endstop_pin_table` storage and `kalico_endstop_sample_pins` definition (they moved). After this step, `runtime_tick.c` shrinks substantially — to roughly 400-450 lines.

- [ ] **Step 3: Update H7 ISR's extern reference to endstop sampler**

In `src/stm32/runtime_tick_h7.c`, find the line `extern void kalico_endstop_sample_pins(void);` and update to `extern void runtime_endstop_sample_pins(void);`. Update the call site too.

In `src/linux/runtime_tick_host.c`, same update if it calls the endstop sampler (verify via Task 1 catalog).

- [ ] **Step 4: Add to Makefile**

In `src/Makefile`, find the `src-y +=` block near the existing `src-y += runtime_tick.c` line and add:

```make
src-$(CONFIG_KALICO_RUNTIME) += runtime_commands.c
```

- [ ] **Step 5: Update host-side Klipper command consumers**

For every match in Task 1's catalog under "Klippy host-side references" / "External klipper-sim corpus":
- klippy/motion_bridge.py and klippy/motion_toolhead.py: rename Python literals matching the wire strings (`'kalico_query_status'` → `'runtime_query_status'`, etc.)
- rust/motion-bridge/src/*.rs: rename any `MessageKind` lookups by name string
- tools/sim_klippy/*.py: rename test assertions
- ~/Developer/klipper-sim/* if reachable

A targeted grep + sed:

```bash
HOST_FILES=$(grep -rl 'kalico_\(query_status\|set_homed\|arm_endstop\|disarm_endstop\|configure_axes\|stream_open\|stream_arm\|stream_terminal\|stream_flush\|clock_sync\|query_pool_state\)' klippy/ tools/ rust/motion-bridge/ 2>/dev/null)
for f in $HOST_FILES; do
  sed -i.bak \
    -e "s/\\bkalico_query_status\\b/runtime_query_status/g" \
    -e "s/\\bkalico_set_homed\\b/runtime_set_homed/g" \
    -e "s/\\bkalico_set_homed_state\\b/runtime_set_homed_state/g" \
    -e "s/\\bkalico_arm_endstop\\b/runtime_arm_endstop/g" \
    -e "s/\\bkalico_disarm_endstop\\b/runtime_disarm_endstop/g" \
    -e "s/\\bkalico_configure_axes\\b/runtime_configure_axes/g" \
    -e "s/\\bkalico_stream_open\\b/runtime_stream_open/g" \
    -e "s/\\bkalico_stream_arm\\b/runtime_stream_arm/g" \
    -e "s/\\bkalico_stream_terminal\\b/runtime_stream_terminal/g" \
    -e "s/\\bkalico_stream_flush\\b/runtime_stream_flush/g" \
    -e "s/\\bkalico_clock_sync\\b/runtime_clock_sync/g" \
    -e "s/\\bkalico_query_pool_state\\b/runtime_query_pool_state/g" \
    "$f"
  rm -f "$f.bak"
done
```

The `\\b` (escaped for shell-inside-quotes) prevents matches inside other identifiers.

- [ ] **Step 6: Cross-build H7 + run host-sim**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -5

cp .config .config.h7.bak
cp .config.linux .config 2>/dev/null && make olddefconfig 2>&1 | tail -2 && make -j4 2>&1 | tail -3
cp .config.h7.bak .config && make olddefconfig 2>&1 | tail -2
```

Expected: both builds succeed.

- [ ] **Step 7: Run host-sim G1 X10 smoke**

```bash
./tools/sim_klippy/run_local.sh "G1 X10 F1000" 2>&1 | tail -20
```

Expected: step-count output with non-zero step counts. Compare against pre-refactor baseline (preserve a snapshot of the output before this step starts; should be identical).

- [ ] **Step 8: Commit**

```bash
git add src/runtime_commands.c src/runtime_tick.c src/Makefile \
        src/stm32/runtime_tick_h7.c src/linux/runtime_tick_host.c \
        klippy/ tools/sim_klippy/ rust/motion-bridge/
git commit -m "runtime-tick refactor: extract command surface to runtime_commands.c"
```

---

## Task 8: Extract sim commands to `src/runtime_sim_commands.c`

**Purpose:** Move every `command_kalico_sim_*` (gated `CONFIG_KALICO_SIM`) plus the `runtime_sim_isr_wake_drain` strong definition to a dedicated TU.

**Files:**
- Create: `src/runtime_sim_commands.c` — sim-only commands + sim drain wake.
- Modify: `src/runtime_tick.c` — remove the lifted sim commands and the sim drain wake function.
- Modify: `src/Makefile` — `src-$(CONFIG_KALICO_SIM) += runtime_sim_commands.c`.
- Modify: `src/stm32/runtime_tick_h7.c` — `extern void kalico_sim_isr_wake_drain(void);` rename to `runtime_sim_isr_wake_drain` (already done in Task 6 sed pass — verify; otherwise rename here).

**Klipper sim command-name string renames**:

C function | Old wire string | New wire string
---|---|---
command_kalico_sim_diag | `kalico_sim_diag` | `runtime_sim_diag`
command_kalico_sim_engine_tick_start | (look up exact wire string in source) | replace `kalico_` with `runtime_`
command_kalico_sim_endstop_set_pin | (look up exact wire string in source) | replace `kalico_` with `runtime_`
command_kalico_sim_load_fixture | (look up exact wire string in source) | replace `kalico_` with `runtime_`
command_kalico_sim_stepper_count_query | (look up exact wire string in source) | replace `kalico_` with `runtime_`

Confirm by `grep -nE 'DECL_COMMAND.*"kalico_sim_' src/runtime_tick.c` for the canonical list.

- [ ] **Step 1: Lift sim commands into new TU**

Create `src/runtime_sim_commands.c` and move every `command_kalico_sim_*` function (their function bodies plus the `DECL_COMMAND` macros) from `src/runtime_tick.c`. Also move the `runtime_sim_isr_wake_drain` function definition (and the `kalico_sim_drain_counter` static state it uses + the `KALICO_SIM_DRAIN_PERIOD_TICKS` constant if it's defined locally — search `src/runtime_tick.c` for them).

Apply the same rename pattern inside the new file:

```bash
sed -i.bak \
    -e 's/\bcommand_kalico_sim_/command_runtime_sim_/g' \
    -e 's/\bkalico_sim_isr_wake_drain\b/runtime_sim_isr_wake_drain/g' \
    -e 's/\bkalico_sim_drain_counter\b/runtime_sim_drain_counter/g' \
    -e 's/\bkalico_sim_drain_calls\b/runtime_sim_drain_calls/g' \
    -e 's/\bkalico_sim_cyccnt\b/runtime_sim_cyccnt/g' \
    -e 's/"kalico_sim_/"runtime_sim_/g' \
    -e 's/\bKALICO_SIM_DRAIN_PERIOD_TICKS\b/RUNTIME_SIM_DRAIN_PERIOD_TICKS/g' \
    src/runtime_sim_commands.c
rm src/runtime_sim_commands.c.bak
```

- [ ] **Step 2: Remove lifted code from runtime_tick.c**

Delete every `command_kalico_sim_*` function and the `runtime_sim_isr_wake_drain` definition + supporting state from `src/runtime_tick.c`.

- [ ] **Step 3: Update H7 ISR if needed**

Verify `src/stm32/runtime_tick_h7.c` calls `runtime_sim_isr_wake_drain` (post-Task-6 rename). The Task 6 sed pass should have caught it; if not:

```bash
grep -n 'kalico_sim_isr_wake_drain' src/stm32/runtime_tick_h7.c src/linux/runtime_tick_host.c src/generic/runtime_bench.c
```

If found, sed-rename to `runtime_sim_isr_wake_drain`.

- [ ] **Step 4: Update Makefile**

In `src/Makefile`:

```make
src-$(CONFIG_KALICO_SIM) += runtime_sim_commands.c
```

- [ ] **Step 5: Update host-side consumers**

Repeat the sed pass over `klippy/`, `tools/`, `rust/motion-bridge/`, `tests/`, and (if reachable) `~/Developer/klipper-sim/` for sim command names:

```bash
HOST_FILES=$(grep -rl 'kalico_sim_\(diag\|engine_tick_start\|endstop_set_pin\|load_fixture\|stepper_count_query\)' klippy/ tools/ rust/motion-bridge/ tests/ 2>/dev/null)
for f in $HOST_FILES; do
  sed -i.bak 's/\bkalico_sim_/runtime_sim_/g' "$f"
  rm -f "$f.bak"
done
```

(The catalog from Task 1 lists exact sites.)

- [ ] **Step 6: Build host-sim (KALICO_SIM=y) + cross-build H7 (KALICO_SIM=n)**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -3   # H7

cp .config .config.h7.bak
cp .config.linux .config && make olddefconfig 2>&1 | tail -2 && make -j4 2>&1 | tail -3   # host-sim
cp .config.h7.bak .config && make olddefconfig 2>&1 | tail -2
```

Expected: both build.

- [ ] **Step 7: Re-run host-sim smoke**

```bash
./tools/sim_klippy/run_local.sh "G1 X10 F1000" 2>&1 | tail -20
```

Expected: step counts unchanged from Task 7 baseline.

- [ ] **Step 8: Commit**

```bash
git add src/runtime_sim_commands.c src/runtime_tick.c src/Makefile \
        src/stm32/runtime_tick_h7.c src/linux/runtime_tick_host.c \
        klippy/ tools/ rust/motion-bridge/ tests/
git commit -m "runtime-tick refactor: extract sim commands to runtime_sim_commands.c"
```

---

## Task 9: Final slim of `runtime_tick.c`

**Purpose:** After Tasks 7 + 8, `runtime_tick.c` should be down to lifecycle: `runtime_init` (DECL_INIT body), `runtime_drain` / `runtime_status_drain` / `runtime_endstop_drain` (DECL_TASK bodies), and the shared globals (`runtime_handle`, `runtime_clock_freq`, `runtime_irq_save` / `runtime_irq_restore`, `runtime_host_now_us`, `runtime_aligned_cps` / `runtime_aligned_knots`). This task verifies size and removes dead extern declarations.

**Files:**
- Modify: `src/runtime_tick.c` — final pass: delete unused extern declarations; reorder for readability; trim dead comments.

- [ ] **Step 1: Verify file size**

```bash
wc -l src/runtime_tick.c
```

Expected: ~300-450 lines. If >500, the prior tasks left more behind than expected — investigate before continuing.

- [ ] **Step 2: Audit and remove dead extern decls**

After the lifts in Tasks 2 / 7 / 8, some `extern` declarations at the top of `runtime_tick.c` may now reference symbols defined in the lifted TUs and re-imported only for the lifted code. Audit:

```bash
grep -nE '^extern' src/runtime_tick.c
```

For each `extern` line, confirm the symbol is referenced in the file (`grep <symbol> src/runtime_tick.c`). If unreferenced, delete the extern line.

- [ ] **Step 3: Update top-of-file documentation comment**

The first ~30 lines of `src/runtime_tick.c` likely have a comment describing the file's responsibilities pre-refactor. Update to reflect the new lifecycle-only scope:

```c
// src/runtime_tick.c
//
// Klipper-side lifecycle for the kalico runtime: DECL_INIT brings up the
// Rust runtime + the per-family tick backend; DECL_TASK pumps drain the
// Rust → Klipper response queue. Shared globals (runtime_handle,
// runtime_clock_freq, runtime_aligned_*) live here as the single
// definition site.
//
// Klipper command surface is in src/runtime_commands.c.
// Sim-only commands are in src/runtime_sim_commands.c (gated CONFIG_KALICO_SIM).
// Bench is in src/generic/runtime_bench.c (gated CONFIG_RUNTIME_BENCH).
// Per-family backends:
//   src/stm32/runtime_tick_h7.c   (H7 TIM5 ISR)
//   src/linux/runtime_tick_host.c (pthread tick for host-sim)
// Backend interface contract: src/generic/runtime_tick.h.
```

- [ ] **Step 4: Cross-build H7 + host-sim once more**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -3
cp .config .config.h7.bak
cp .config.linux .config && make olddefconfig 2>&1 | tail -2 && make -j4 2>&1 | tail -3
cp .config.h7.bak .config && make olddefconfig 2>&1 | tail -2
```

Expected: both succeed.

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c
git commit -m "runtime-tick refactor: final slim — runtime_tick.c is lifecycle-only"
```

---

## Task 10: Move `RT_CELL` placement to Cargo feature

**Purpose:** Replace `cfg_attr(target_arch = "arm", link_section = ".axi_bss")` with a Cargo-feature-gated attribute. Feature `axi-bss-placement` is selected by `mcu-h7`, ensuring `RT_CELL` lands in AXI SRAM only on builds that actually have AXI SRAM mapped.

**Files:**
- Modify: `rust/kalico-c-api/Cargo.toml` — add the feature; `mcu-h7` selects it.
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` — switch the `cfg_attr` predicate.

- [ ] **Step 1: Edit Cargo.toml**

In `rust/kalico-c-api/Cargo.toml`, find the `[features]` section. Add:

```toml
# Place RT_CELL in `.axi_bss` (mapped to AXI SRAM on H7). Selected only on
# targets that have AXI SRAM available; on others (e.g., F4 future, host-sim)
# RT_CELL lands in default bss.
axi-bss-placement = []
```

Modify the existing `mcu-h7` line to pull `axi-bss-placement`:

```toml
mcu-h7 = ["nurbs/mcu-h7", "runtime/mcu-h7", "axi-bss-placement"]
```

Leave `mcu-f4` and `host` features alone — they don't get `axi-bss-placement`.

- [ ] **Step 2: Edit runtime_ffi.rs**

In `rust/kalico-c-api/src/runtime_ffi.rs`, find the `RT_CELL` definition (currently around line 67):

```rust
#[cfg_attr(target_arch = "arm", unsafe(link_section = ".axi_bss"))]
pub(super) static RT_CELL: RuntimeCell = RuntimeCell(UnsafeCell::new(MaybeUninit::uninit()));
```

Change the predicate to feature-based:

```rust
#[cfg_attr(feature = "axi-bss-placement", unsafe(link_section = ".axi_bss"))]
pub(super) static RT_CELL: RuntimeCell = RuntimeCell(UnsafeCell::new(MaybeUninit::uninit()));
```

- [ ] **Step 3: Cross-build H7 — verify `RT_CELL` still in `.axi_bss`**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -3
arm-none-eabi-nm out/klipper.elf | grep -E 'RT_CELL$' | awk '{print $1, $2}'
```

Expected: `RT_CELL` address in the AXI SRAM range (`0x24xxxxxx`) — same as pre-refactor (~`0x24000000`-ish, matching the layout commits).

- [ ] **Step 4: Build host-sim — verify `RT_CELL` in default bss**

```bash
cp .config .config.h7.bak
cp .config.linux .config && make olddefconfig 2>&1 | tail -2 && make -j4 2>&1 | tail -3
nm out/klipper.elf 2>/dev/null | grep -E 'RT_CELL$' | head -3
cp .config.h7.bak .config && make olddefconfig 2>&1 | tail -2
```

Expected: host-sim builds; `RT_CELL` in regular bss (no `0x24xxxxxx` placement).

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-c-api/Cargo.toml rust/kalico-c-api/src/runtime_ffi.rs
git commit -m "kalico-c-api: gate RT_CELL placement on Cargo feature axi-bss-placement"
```

---

## Task 11: Widen Makefile thumbv7em rule

**Purpose:** Make the Klipper Makefile build the thumbv7em Rust staticlib for any of `MACH_STM32H7 || MACH_STM32F4 || MACH_LINUX`-with-host (the third branch builds with the `host` feature, not thumbv7em — so the gate only widens on the thumbv7em block). The F4 branch is preparatory: there's no F4 backend file yet, so an F4 build would fail to link a `runtime_tick_f4.c` — but the cargo build itself should succeed.

**Files:**
- Modify: `src/Makefile` — replace `ifeq ($(CONFIG_MACH_STM32H7),y)` guard with allowlist; add Kconfig-driven `KALICO_RUST_FEATURES`.

- [ ] **Step 1: Edit src/Makefile**

In `src/Makefile`, find the existing block (around lines 42-64):

```make
KALICO_RUST_FEATURES := mcu-h7,header-nurbs,header-runtime
ifeq ($(CONFIG_MACH_STM32H7),y)
	cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" \
		KALICO_RUNTIME_MAX_CONTROL_POINTS=$(CONFIG_RUNTIME_MAX_CONTROL_POINTS) \
		...
		cargo build -p kalico-c-api ...
endif
```

Replace with Kconfig-driven feature selection + widened guard:

```make
ifeq ($(CONFIG_MACH_STM32H7),y)
    KALICO_RUST_FEATURES := mcu-h7,header-nurbs,header-runtime
else ifeq ($(CONFIG_MACH_STM32F4),y)
    KALICO_RUST_FEATURES := mcu-f4,header-nurbs,header-runtime
else ifeq ($(CONFIG_MACH_LINUX),y)
    KALICO_RUST_FEATURES := host,header-nurbs,header-runtime,kalico-sim
endif

ifneq (,$(filter y,$(CONFIG_MACH_STM32H7) $(CONFIG_MACH_STM32F4)))
	# (cargo build with thumbv7em-none-eabihf target as before)
	cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" \
		KALICO_RUNTIME_MAX_CONTROL_POINTS=$(CONFIG_RUNTIME_MAX_CONTROL_POINTS) \
		...
		cargo build -p kalico-c-api ...
endif
```

(Preserve every `KALICO_RUNTIME_*` env var line and the `cargo build` invocation as-is. Only the outer `ifeq` widens, and the `KALICO_RUST_FEATURES` selection becomes Kconfig-driven instead of hardcoded.)

The Linux branch's cargo invocation is in a separate `ifeq ($(CONFIG_MACH_LINUX),y)` block elsewhere in the Makefile — leave that block untouched.

- [ ] **Step 2: Cross-build H7 to confirm no regression**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
make clean && make -j4 2>&1 | tail -5
```

Expected: build succeeds with `mcu-h7,header-nurbs,header-runtime` features.

- [ ] **Step 3: Try a hypothetical F4 build — will fail at link, but cargo build should succeed**

```bash
cp .config .config.h7.bak
cat > .config <<'EOF'
CONFIG_LOW_LEVEL_OPTIONS=y
CONFIG_MACH_STM32=y
CONFIG_BOARD_DIRECTORY="stm32"
CONFIG_MCU="stm32f446xx"
CONFIG_CLOCK_FREQ=180000000
CONFIG_USBSERIAL=y
CONFIG_FLASH_SIZE=0x80000
CONFIG_FLASH_BOOT_ADDRESS=0x8000000
CONFIG_RAM_START=0x20000000
CONFIG_RAM_SIZE=0x20000
CONFIG_STACK_SIZE=512
CONFIG_FLASH_APPLICATION_ADDRESS=0x8008000
CONFIG_STM32_SELECT=y
CONFIG_MACH_STM32F446=y
CONFIG_MACH_STM32F4=y
CONFIG_HAVE_STM32_USBOTG=y
CONFIG_KALICO_RUNTIME=y
EOF
make olddefconfig 2>&1 | tail -2
make clean
make 2>&1 | tail -20 || true
cp .config.h7.bak .config && make olddefconfig 2>&1 | tail -2
```

Expected: cargo builds the staticlib with `mcu-f4` features (no `axi-bss-placement`). Final `klipper.elf` link likely **fails** because there's no `runtime_tick_f4.c` and the H7 ISR symbols are unavailable. **That failure is expected** — the F446 backend file is the follow-up plan's responsibility. The cargo-side build of the Rust staticlib is what we're verifying here.

If cargo fails (not just the link): the F4 feature path in `runtime/Cargo.toml` or downstream is broken. Investigate before committing.

If cargo succeeds and only the link fails: success — commit.

- [ ] **Step 4: Build host-sim**

```bash
cp .config .config.h7.bak
cp .config.linux .config && make olddefconfig 2>&1 | tail -2 && make -j4 2>&1 | tail -3
cp .config.h7.bak .config && make olddefconfig 2>&1 | tail -2
```

Expected: builds.

- [ ] **Step 5: Commit**

```bash
git add src/Makefile
git commit -m "make: widen Rust staticlib build to MACH_STM32H7 || MACH_STM32F4; Kconfig-driven features"
```

---

## Task 12: Cleanup and final verification

**Purpose:** Remove the working catalog file (kept it as a per-task reference; no longer needed); run the complete test matrix one final time.

**Files:**
- Delete: `docs/superpowers/handoff/2026-05-06-runtime-tick-rename-catalog.md`.

- [ ] **Step 1: Delete the catalog**

```bash
git rm docs/superpowers/handoff/2026-05-06-runtime-tick-rename-catalog.md
```

- [ ] **Step 2: Final cross-build matrix**

```bash
export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH

# H7 production
make clean && make -j4 2>&1 | tail -5

# Host-sim
cp .config .config.h7.bak
cp .config.linux .config && make olddefconfig 2>&1 | tail -2 && make -j4 2>&1 | tail -3
cp .config.h7.bak .config && make olddefconfig 2>&1 | tail -2

# H7 with KALICO_SIM=y
sed -i.bak 's/^# CONFIG_KALICO_SIM is not set$/CONFIG_KALICO_SIM=y/' .config
make olddefconfig 2>&1 | tail -2 && make -j4 2>&1 | tail -3
mv .config.bak .config && make olddefconfig 2>&1 | tail -2
```

Expected: all three configurations build.

- [ ] **Step 3: Rust unit tests**

```bash
cd rust
cargo test -p runtime --quiet 2>&1 | tail -5
cargo test -p kalico-host-rt --quiet 2>&1 | tail -5
cargo test -p kalico-c-api --quiet 2>&1 | tail -5
cd ..
```

Expected: all pass.

- [ ] **Step 4: Host-sim smoke**

```bash
./tools/sim_klippy/run_local.sh "G1 X10 F1000" 2>&1 | tail -20
./tools/sim_klippy/run_local.sh "G1 Z5 F600" 2>&1 | tail -20
```

Expected: step counts match pre-refactor baseline for both moves.

- [ ] **Step 5: Final commit**

```bash
git rm docs/superpowers/handoff/2026-05-06-runtime-tick-rename-catalog.md
git commit -m "runtime-tick refactor: cleanup — remove rename catalog working doc"
```

---

## Self-review

**Spec coverage:**
- §1 Problem (tangling, bench coupling, .axi_bss policy, prefix, build gate): all addressed across Tasks 2-11. ✓
- §2 Goals (interface, per-family files, bench module, slim runtime_tick.c, Cargo feature, prefix removal, allowlist, no F446 backend): every goal traced to a task. ✓
- §3 Non-goals (no F446 backend file, no Kconfig master rename, no crate-name rename, no command-surface redesign, no per-family Makefile redesign): respected — no task touches them. ✓
- §4.1 Abstract interface header: Task 3. ✓
- §4.2 Per-family file renames: Tasks 4, 5. ✓
- §4.3 Bench module: Task 2. ✓
- §4.4 Weak-symbol pattern + feasibility trial: Tasks 1.5, 2. ✓
- §4.5 runtime_tick.c slimming + sibling TUs: Tasks 2, 7, 8, 9. ✓
- §4.6 Cargo feature for `RT_CELL`: Task 10. ✓
- §4.7 Symbol rename table: Task 1 catalog + Task 6 atomic FFI rename + per-task local renames. ✓
- §4.8 Build-system widening: Task 11. ✓
- §5 Implementation ordering: 11 steps map directly. ✓
- §6 Testing: cross-build + host-sim smoke per task; final matrix in Task 12. ✓
- §7 Risks: catalog (Task 1) covers symbol-rename + corpus audit; Task 1.5 covers weak-symbol feasibility; per-task cross-builds catch regressions. ✓
- §8 Out-of-scope follow-ups (F446 backend, etc.): documented; Task 11 leaves the Makefile rule preparatory. ✓

**Placeholder scan:** No "TBD" / "TODO" / "implement later" / "similar to Task N" patterns. Each step has exact paths, exact commands, exact code (or, where wholesale-lift-and-rename is the operation, exact sed scripts that produce the deterministic result).

**Type / signature consistency:**
- The four-function abstract interface (`runtime_tick_init`, `runtime_tick_enable`, `runtime_tick_disable`, `runtime_cyccnt_read`) is consistent across Tasks 3, 4, 5.
- `runtime_bench_capture(uint32_t)` consistent across Tasks 2 (def), 4 (call site), 9 (verification).
- `runtime_handle` (the renamed `kalico_rt_handle`) consistent across Tasks 6, 7, 8, 9 (all reference sites).
- Symbol-rename table in Task 6 is the canonical source; Tasks 7 and 8 only rename additional Klipper-command-name strings and locally-scoped helpers.

**Known not-fully-specified item:** Step 5 of Task 7 and Step 5 of Task 8 say "Repeat the sed pass over [host-side trees]" with the catalog as authority. The exhaustive list of per-file edits in those trees depends on Task 1's catalog output, which doesn't exist at plan-write time. The implementer must read the catalog and apply the named regex per file.

This is a deliberate gap — the alternative is enumerating every line in advance, which (a) bloats the plan with mechanical content and (b) goes stale if the codebase changes between plan-write and plan-execute. The plan instead provides the regex pattern and the authoritative source of file lists. Implementer judgment is required for the per-host-file scope, bounded by the catalog.
