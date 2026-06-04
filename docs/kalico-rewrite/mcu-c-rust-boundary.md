# MCU C/Rust Boundary

> AI generated. Architectural invariant — flag drift, don't drift silently.

## The rule

On the MCU, **Klipper's existing C code stays C** (boot, pins, GPIO, ADC, SPI, UART, thermistor / heater control, USB-CDC framing, command-table dispatch, the scheduler timer, the watchdog). **The new motion engine is Rust** (NURBS evaluator, per-sample trajectory, kinematics, phase-stepping current synthesis when it lands). The boundary between them is **narrow, one-directional in spirit, and language-agnostic by discipline**:

- C calls into Rust through a small, named set of `extern "C"` entry points. Today the central one is `runtime_tick` (called from the scheduler timer).
- Any state shared across the boundary lives in **C structs, in C-owned linker sections**, with a `#[repr(C)]` Rust mirror that is declared `extern "C"` and never owns the storage.
- **No Rust-typed structure crosses the ABI.** No `heapless::spsc::Producer<...>` parameter, no `&'a mut` borrow, no slice-with-lifetime. If Rust needs a queue between two of its own modules that the C side can also see, the queue is defined in C and Rust accesses it through `extern "C"` getters or volatile pointer reads on agreed offsets.

This is the formal version of the one-liner already in `CLAUDE.md`: *"Rust for the engine, C where the engine's primitives need to be trivially debuggable."* This doc spells out which primitives, why, and how to keep them debuggable as the surface grows.

## Why mixed, not all-of-one

**Why not all-Rust on the MCU?** The C side of Klipper has heater control, runaway detection, thermistor sanity, pin-overlap protection, command-table dispatch, USB framing, and the timer scheduler — code that has accumulated years of field hardening across thousands of printers, and where a regression is a fire risk in the literal sense. Rewriting it from scratch in Rust trades a known-good substrate for a re-implementation that has to re-earn that trust. The borrow checker doesn't help with "this thermistor table is wrong" or "this heater pin can't be the same as the bed-heater pin" — those are domain invariants that already exist correctly in C.

**Why not all-C on the MCU?** Per-sample NURBS evaluation, kinematics, phase-stepping current synthesis, and the post-shape trajectory representation are exactly the kind of algorithmically dense code that benefits from Rust's type and ownership discipline. A C implementation of this work would be a real correctness regression on the hot path. The Layer 0 / Layer 4 boundary is also designed around the f64-host / f32-MCU "single source compiled twice" property — that property is a Rust property.

So: each language goes where it's better, and the engineering work is to make the seam between them sturdy.

## The boundary discipline

These rules apply to every new piece of shared state added to the MCU. If you find yourself wanting to violate one, the answer is almost always "redefine the structure on the C side" or "narrow the surface to a single entry point."

### B1. Entry points are explicit; logical entry-point count is small

The C side calls Rust through a small, named list of *logical* entry points. The count of *physical* `extern "C"` symbols is larger (~82 today) because of the opaque-handle API pattern: many functions sharing one handle constitute one cohesive seam. New seams require justification; new methods on an existing handle do not. Current inventory:

- **`runtime_tick`** — scheduled by the C timer. The per-sample motion path hangs off this one call. Reentrant-safe by being non-reentrant (the C scheduler serializes calls).
- **`runtime_handle_create` + the `KalicoRuntime` opaque-handle API.** One init-once function plus the family of accessor / operation functions taking `*mut KalicoRuntime` (see `rust/kalico-c-api/src/runtime_ffi.rs` for the full inventory — currently 82 entries). Counted as one logical seam for the discipline this rule is enforcing.
- **`runtime_diag_progress`** — read by the diagnostics dump path; reports motion engine state into `.persistent_diag`.
- (Add new entries above this line, with a one-sentence justification. A new accessor on the existing handle is not a new entry; a fundamentally new seam is.)

Anything else — heater readback, pin events, USB byte arrival, command-table dispatch — stays inside C and is **not** routed through Rust. The motion engine is a tenant on the MCU, not the MCU's main loop.

### B2. Shared state lives in C, in C-owned sections

Anything visible to both languages — the segment SPSC queue, `.persistent_diag`, `RT_CELL`, future telemetry rings, future step-output queues — is **defined in a `.c` file, placed via the C linker script, with a sized symbol the Rust side imports via `extern "C"` and a `#[repr(C)]` mirror.**

Operational consequences:

- Linker sections (`.axi_bss`, `.persistent_diag`, custom regions) are *named in the linker script*, with start/end symbols. Rust never `#[link_section]`s into them to introduce a new section. This eliminates "Rust struct moved" / "section silently grew" failure modes.
- Section *ownership* (who allocates) and section *layout* (who decides offsets) live in one place: the C side. The H7 vs F4 RT_CELL section confusion was a Rust-side cache problem precisely because Rust was deciding the placement; the immediate resolution converged on cargo-clean discipline, but the deeper resolution is "don't have Rust decide the placement in the first place."
- The `dynmem` / `.persistent_diag` overlap (2026-05-12) had the same shape — two allocators in two languages each thinking they owned the same bytes. Single-language ownership of "where does this section start" fixes the class of bug, not the instance.

### B3. No Rust-typed structures cross the ABI

If Rust code on one side of an `extern "C"` boundary and Rust code on the other side want to share a queue, **they share it through a C struct.** The 2026-05-18 SPSC bench (`qlen_sd=6 qlen_ps=1` for the same `heapless::spsc::Consumer` instance observed from two call sites, with the producer's enqueues visible from one site but not the other) is the load-bearing evidence: even pure Rust-to-Rust shared state, when it crosses indirection LLVM treats as suspicious, can miscompile in ways that are not detectable from the source. A C struct accessed with explicit volatile reads is *greppable*, *debugger-friendly*, and not at the mercy of LLVM's aliasing model.

This rule is uncomfortable because it costs duplication: a queue type is declared in `motion_queue.h` and mirrored as `#[repr(C)]` in Rust. The duplication is the point — both sides agree on the contract, in the literal layout sense, and neither side can drift without the other noticing at compile time (offset asserts on the Rust side, sizeof asserts on the C side).

### B4. `extern "C"` + `#[repr(C)]` everywhere across the boundary

Any function visible across the boundary is `extern "C"` and listed in a header. Any struct visible across the boundary is `#[repr(C)]` on the Rust side and defined in a C header on the C side. No `#[repr(Rust)]` types, no `enum`-with-payloads, no `Option<&T>` in signatures, no zero-sized types. Slices cross as pointer + length.

`bool` is permitted. Rust's `bool` and C99 `_Bool` are layout-compatible (both 1 byte, both 0/1); the C side must `#include <stdbool.h>` to consume it. The audit pass on 2026-05-19 (A2 finding) ratified this against existing `runtime_handle_*` accessor sites that already return `bool` (`runtime_ffi.rs:2145, 2234, 2400, 2985`).

### B5. Atomicity and memory ordering follow the C model

The MCU side already has a memory model (ARMv7-M with explicit `__DMB` / `__DSB`). Shared-state access from Rust uses `core::sync::atomic` with explicit `Ordering` arguments that match what the C side expects. Where the C side reads or writes a shared word without atomics (single-producer / single-consumer with section-pinned alignment), **the Rust side does the same** (`read_volatile` / `write_volatile`, not atomic), so the two sides agree on the abstraction layer rather than one side "upgrading" silently.

## What lives where on the MCU

| Concern | Language | Notes |
|---|---|---|
| Boot, vector table, clock tree init, CPACR setup | C | F4 FPU-enable fix (2026-05-12) lives here. Rust assumes the CPU is configured before it runs. |
| Pins, GPIO, ADC, SPI, UART, USB-CDC framing | C | Klipper as-is. |
| Thermistor / heater control, runaway detection | C | Safety-critical. **Do not port to Rust without a separate review and test plan.** |
| Command-table dispatch (msgid → handler) | C | Klipper as-is. The bridge enters through the existing command path. |
| Scheduler timer | C | Calls `runtime_tick`. |
| Watchdog (IWDG) | C | Petted from the scheduler; pacing actuals are Step 7-D scope. |
| Motion engine — per-sample NURBS eval, kinematics, step output | Rust | The reason the boundary exists. |
| Segment queue (host → MCU) | C struct in regular `.bss` (DTCM on H7), Rust producer + Rust consumer | The 2026-05-18 SPSC fix. DTCM placement on H7 is deliberate: non-cached, eliminates cache-coherency concerns. NOT in `.axi_bss`. |
| `.persistent_diag` region | C struct, C-allocated, both sides read/write | 2026-05-12 fix anchors the placement. |
| `.axi_bss` occupants (H7-only) | All C-declared today: `kalico_buf` (`src/kalico_demux.c`), `receive_buf` (`src/generic/serial_irq.c`). Each tagged `__attribute__((section(".axi_bss")))` under `#if CONFIG_MACH_STM32H7`. | These rely on AXI SRAM at 0x24000000 because DTCM (128 KB) is saturated by the rest of Klipper. |
| Runtime context backing (`rt_storage`) | C-declared `uint8_t` buffer in `src/runtime_storage.c`. H7: `.axi_bss` (AXI SRAM at 0x24000000) via cfg-gated section attribute. F4: regular `.bss` via default rule. Rust imports via `extern "C" { static rt_storage: UnsafeCell<[u8; RT_STORAGE_SIZE]>; }` and casts to `*mut RuntimeContext`. | Migrated 2026-05-19 from the `RT_CELL` Rust static with `#[link_section]`. `RT_STORAGE_SIZE` flows from Kconfig (`RUNTIME_STORAGE_SIZE_LARGE` = 284 KB on H7, `_SMALL` = 64 KB on F4) through both `src/runtime_storage.h` and `rust/runtime/build.rs`. The cargo-clean operational tripwire (`feedback_cargo_clean_between_mcus.md`) is now optional rather than safety-critical. |
| Phase-stepping current synthesis (Step 10) | Rust | Algorithmic; lives on the Rust side of the line. |
| Telemetry transport (Step 11) | C framing, Rust producers | The producers write into a C-owned ring; C ships it over the existing transport. |

## Case studies (what the rule is reacting to)

These are the failures the boundary discipline is engineered against. They are *not* "Rust and C don't compose" failures — they are all about shared state and section ownership, which is hard between any two compilation units in any language. The rule treats the Rust ↔ C boundary the same as any C ↔ C boundary that crosses a section: with explicit ownership.

- **2026-05-18 — SPSC `Consumer` miscompile.** A `heapless::spsc::Consumer` instance reported `qlen_sd=6` from one call site and `qlen_ps=1` from another in the same execution, with producer enqueues visible to one site but not the other. Pure Rust on both ends; the mix only exposed the issue because the bench setup required cross-module visibility. **Resolution:** segment queue is now a C struct in regular `.bss` — DTCM on H7, normal SRAM on F4 — accessed by Rust via `extern "C"` (B2 + B3). DTCM was chosen over `.axi_bss` to eliminate any cache-coherency contribution; the relevant fix was avoiding the Rust borrow-projection, not memory placement. **Rule reinforced:** B3 — even Rust-only shared state should not cross indirection LLVM is allowed to optimize past.
- **2026-05-12 — `dynmem` / `.persistent_diag` overlap.** `dynmem_start()` returned `&_bss_end`, but `.persistent_diag` started at `_bss_end` in the linker map, so `alloc_chunk` overwrote `runtime_diag_progress` writes. Result: corrupted `oids[N].type`, "Invalid oid type" shutdowns mid-session. **Resolution:** `dynmem_start()` returns `&_persistent_diag_end`. **Rule reinforced:** B2 — two allocators in two languages each thinking they own the same bytes is the class of bug; one-language ownership of section placement is the class of fix.
- **2026-05-12 — F446 `configure_axes` crash.** F4 FPU disabled at boot (`SystemInit` skips CPACR when `__FPU_USED == 0`); Rust soft-float occasionally lowered reg-reg moves to `vmov`, which UNDEFINSTRs on M4F. **Resolution:** enable `CPACR.CP10/11` in `armcm_main` when `CONFIG_KALICO_RUNTIME=y && __FPU_PRESENT == 1`. **Rule reinforced:** B1 — boot / CPU-state setup is C's job; Rust assumes the CPU is configured before it runs.
- **`RT_CELL` H7 vs F4 section confusion (resolved 2026-05-19).** H7's `RT_CELL` belonged in `.axi_bss` (0x24000000); F4's in `.bss` (0x20000000). Cargo cache silently kept the wrong `.a` across `make clean`, leaking H7 placement into F4 builds and vice versa. **Initial resolution:** mandatory `cargo clean` between MCU targets; verify via `objdump`. **Structural resolution (2026-05-19, this refactor):** migrated to C-declared `rt_storage` in `src/runtime_storage.c` with cfg-gated `__attribute__((section(".axi_bss")))` (H7 only). Rust no longer makes placement decisions; cargo cache cannot leak across MCU targets. **Rule reinforced:** B2 — single-language ownership of section placement is the structural fix.
- **2026-05-19 — `RT_CELL` → `rt_storage` migration.** The Rust static with `#[link_section = ".axi_bss"]` was replaced by a C-declared `uint8_t rt_storage[RT_STORAGE_SIZE]` buffer. Section placement now decided exclusively by the C linker script (cfg-gated attribute: `.axi_bss` on H7, default `.bss` on F4). Rust imports via `extern "C"` with an `UnsafeCell` wrapper for interior-mutability rights; soundness verified under stacked/tree borrows. Closes the cargo-clean operational tripwire (`feedback_cargo_clean_between_mcus.md`). Two adversarial reviewers (codex + opus kalico-plan-reviewer) had flagged the naïve `extern "C" { static rt_storage: [u8; N]; }` (no `UnsafeCell`) as unsound in spec v1 — the v2 `UnsafeCell<[u8; N]>` mechanism is what landed. **Rule reinforced:** B2 — single-language ownership of section placement is the structural fix.

## Open migrations

State currently on the Rust side of the boundary that B2/B3 say should move to the C side:

- (none open as of 2026-05-19 — `RT_CELL` → `rt_storage` migration completed; see case study above.)

## Tradeoffs we accept

- **Duplicated type declarations.** Queue / shared-state types are declared in C and mirrored as `#[repr(C)]` in Rust. A `bindgen`-style auto-generation could remove the manual mirroring, but the cost is hidden behind generated code and the win is small for the handful of shared structs we have. Manual mirror + a comment cross-referencing both sides is currently fine; revisit if the shared surface grows past ~5–10 structs.
- **One-directional spirit, but practically bidirectional.** Rust can call into C (e.g., GPIO output helpers, `runtime_diag_progress` writing into `.persistent_diag`). The rule is "C is the host, Rust is the tenant," not "Rust never calls C." Rust calls C through the same `extern "C"` discipline; the asymmetry is in *who owns memory and lifetime*, not in who can call whom.
- **More plumbing for new shared state.** Adding a telemetry ring or a step-output queue is "write the C struct, write the linker symbol, write the Rust mirror" rather than "slap `#[link_section]` on a Rust static." The plumbing is the cost we pay for failure modes that are greppable when something goes wrong on the bench at 2 AM.

## When to revisit

This invariant is worth reopening if any of the following becomes true:

- **Klipper-C-side heater / thermistor / pin-overlap code starts blocking motion-engine work in a structural way.** Then the conversation is "do we port the safety-critical C to Rust with a tight test corpus and external review," not "do we keep mixing."
- **The shared-struct surface grows past ~5–10 types.** Manual mirroring stops scaling; consider `bindgen`-generated headers or a single source-of-truth IDL.
- **An MCU target with a different C compiler / linker model lands** (e.g., a non-ARM target, a hard-FreeRTOS port, an EtherCAT subordinate). Boundary assumptions about ELF sections and ARMv7-M atomics need re-checking.

Until then: motion in Rust, everything else in C, shared state owned by C, narrow `extern "C"` seam between them.
