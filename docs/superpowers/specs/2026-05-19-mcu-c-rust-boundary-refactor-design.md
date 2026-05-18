# MCU C/Rust Boundary Refactor — Design

> Date: 2026-05-19
> Status: brainstormed, ready for plan-writing
> Scope owner: claude+user. Implementation: subagent-driven via `rust-engineer` (per saved feedback memory; any Rust-touching subagent task uses `subagent_type: "rust-engineer"`).
> Companion to: [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](../../kalico-rewrite/mcu-c-rust-boundary.md)

## Motivation

The architectural invariant in [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](../../kalico-rewrite/mcu-c-rust-boundary.md) (rules B1–B5) is true of most of the MCU code today but has one prominent code-side violation and one prominent doc-side mismatch:

- **Code violation (B2):** `RT_CELL` (`rust/kalico-c-api/src/runtime_ffi.rs:68`) is a Rust static that uses `#[link_section = ".axi_bss"]` to dictate its own placement. Per B2 ("Linker sections are named in the linker script ... Rust never `#[link_section]`s into them to introduce a new section. Section ownership and section layout live in one place: the C side."), this storage should be declared on the C side. The operational discipline that currently keeps this working (mandatory `cargo clean` between H7 and F4 builds, per `feedback_cargo_clean_between_mcus.md`) is a tripwire, not a fix.
- **Doc/code mismatch (B1):** B1 says C calls Rust through a "small, named list" of entry points. In practice the Rust → C surface is ~70 `extern "C"` functions, almost all of which are accessors over a single `*mut KalicoRuntime` opaque handle. The pattern is correct; the wording is wrong.

Recent failures that motivate hardening the boundary, all spelled out in `docs/kalico-rewrite/mcu-c-rust-boundary.md` § Case studies:

- 2026-05-18 SPSC `Consumer` miscompile (resolved by moving the queue to a C struct in `.axi_bss`).
- 2026-05-12 `dynmem` / `.persistent_diag` overlap (resolved by single-language ownership of section placement).
- 2026-05-12 F446 `configure_axes` crash (resolved by C-side CPACR setup at boot).
- `RT_CELL` H7 vs F4 section confusion (resolved operationally; structural fix is this refactor).

None of these were "Rust and C don't compose" failures — they were all "two compilation units disagreeing about who owns this memory," which is what B2 is designed to prevent. This refactor closes the last code-side instance of that class.

## Goals

1. Move `RT_CELL`'s storage to a C-declared `uint8_t` buffer, with section placement decided exclusively by the C linker script (B2).
2. Delete the `#[link_section]` attribute and the `axi-bss-placement` Cargo feature flag (residue of the prior Rust-owned placement scheme).
3. Sweep the rest of the C ↔ Rust boundary for other B3 / B4 / B5 leaks; fix what's found.
4. Update `docs/kalico-rewrite/mcu-c-rust-boundary.md` to (a) reword B1's "small list" wording to match the opaque-handle API reality and (b) retire the "Open migrations: RT_CELL" line.
5. **Everything works on both H7 and F446 at the end.** This is the load-bearing acceptance criterion the user named explicitly. Bench-flash + motion-works on H7, bench-flash + steady-state on F4.

## Non-goals

- **No consolidation of the ~70 extern "C" function surface.** The opaque-handle API pattern is correct; we update the doc to reflect that, not the code.
- **No port of safety-critical Klipper C (heater control, thermistor, pin overlap) to Rust.** Out of scope per the boundary doc's "Why mixed, not all-of-one" section.
- **No introduction of a `bindgen`-generated header.** Manual mirroring at the current scale is fine; revisit when the shared-struct surface exceeds ~5–10 types.
- **No changes to the `.persistent_diag` design.** Already correctly C-owned; touch nothing.
- **No changes to the segment SPSC queue.** Already correctly C-owned in `.axi_bss` (the 2026-05-18 fix); touch nothing.

## Architecture

### What changes

- `rt_storage` becomes a C-declared `uint8_t` buffer in a new file `src/runtime_storage.c`, placed via `__attribute__((section(...)))`. The linker script decides where the section lands per MCU target.
- `rust/runtime/build.rs` is extended to emit a small C header (`runtime_storage.h`) containing `#define RT_STORAGE_SIZE …` based on `size_of::<RuntimeContext>()` for the active build profile. Same flow that already emits `sizing.rs` for Rust; the constant flows from one source of truth (Rust's `size_of`, gated by Kconfig) to two consumers (the Rust crate via `include!(concat!(env!("OUT_DIR"), "/sizing.rs"))`, the C compilation via include path).
- `rust/kalico-c-api/src/runtime_ffi.rs`: delete `RuntimeCell`, `RT_CELL`, the `unsafe(link_section = ".axi_bss")` attribute, the `axi-bss-placement` Cargo feature, and the `Sync` impl on the wrapper. Replace with `unsafe extern "C" { static rt_storage: [u8; RT_STORAGE_SIZE]; }` and rebase `runtime_handle_create` on `rt_storage.as_ptr() as *mut RuntimeContext`.
- `Cargo.toml` (kalico-c-api): remove the `axi-bss-placement` feature.
- `Makefile`: drop any feature-flag plumbing for `axi-bss-placement`; add `OUT_DIR/runtime_storage.h` to the C include path (or copy to a stable build-tree location and add that).

### What stays unchanged

- The half-split `FgState` / `IsrState` raw-pointer projection pattern (`rust/kalico-c-api/src/runtime_ffi.rs:1–17`). Only the origin of the pointer changes — from `RT_CELL.0.get()` to `rt_storage.as_ptr().cast()`. The `addr_of!` / `UnsafeCell::raw_get` projection chain is identical and remains sound under stacked-borrows / tree-borrows.
- `RuntimeContext::init`'s raw-pointer write protocol (`rust/runtime/src/state.rs:551`). It already writes through raw-pointer projections without ever materializing `&mut RuntimeContext`, so it works identically against a C-declared backing buffer.
- The linker script's `.axi_bss` declaration (`src/generic/armcm_link.lds.S:114-120`, currently gated `#if CONFIG_MACH_STM32H7`). On H7 this stays as-is. On F4, we either gate the `__attribute__((section(".axi_bss")))` out of the C declaration, or we add a one-line linker-script fallback that places `.axi_bss`-tagged sections into `> ram` on non-H7 targets. The latter keeps the C declaration uniform; the choice resolves at implementation time.
- All other `.axi_bss` C occupants (`src/kalico_demux.c:51`, `src/generic/runtime_bench.c:23`, `src/generic/serial_irq.c:29`, the segment SPSC queue in `src/kalico_segment_queue.c`). These already follow B2.

## Detailed design

### Section 1 — RT_CELL migration

C side (new file `src/runtime_storage.c`):

```c
#include "runtime_storage.h"  // from build.rs OUT_DIR, defines RT_STORAGE_SIZE

_Alignas(16) uint8_t rt_storage[RT_STORAGE_SIZE]
    __attribute__((section(".axi_bss"), used, externally_visible));

_Static_assert(RT_STORAGE_SIZE >= 1024,
               "RT_STORAGE_SIZE absurdly small — build.rs did not emit");
```

Header `src/runtime_storage.h`:

```c
#ifndef KALICO_RUNTIME_STORAGE_H
#define KALICO_RUNTIME_STORAGE_H
#include "../../out/generated/runtime_storage.h"  // or wherever build.rs emits
extern uint8_t rt_storage[RT_STORAGE_SIZE];
#endif
```

Rust side (`rust/kalico-c-api/src/runtime_ffi.rs`):

```rust
unsafe extern "C" {
    static rt_storage: [u8; RT_STORAGE_SIZE];
}

const _: () = assert!(
    core::mem::size_of::<RuntimeContext>() <= RT_STORAGE_SIZE,
    "RuntimeContext outgrew RT_STORAGE_SIZE — bump the build.rs ceiling"
);
```

(`RT_STORAGE_SIZE` is imported from `runtime` crate's `sizing.rs` — see Section 2.)

`runtime_handle_create` cast change (line ~119):

```rust
// BEFORE:
let rt_ptr: *mut RuntimeContext = (*RT_CELL.0.get()).as_mut_ptr();
// AFTER:
let rt_ptr: *mut RuntimeContext = rt_storage.as_ptr() as *mut RuntimeContext;
debug_assert_eq!(
    (rt_ptr as usize) % core::mem::align_of::<RuntimeContext>(),
    0,
    "rt_storage alignment mismatch"
);
```

Deletions:
- `RuntimeCell` struct + `Sync` impl (runtime_ffi.rs:55–61).
- `RT_CELL` static (runtime_ffi.rs:68–69).
- `#[cfg_attr(feature = "axi-bss-placement", unsafe(link_section = ".axi_bss"))]` attribute (runtime_ffi.rs:68).
- `axi-bss-placement` from `rust/kalico-c-api/Cargo.toml` `[features]`.

The `unsafe impl Sync for RuntimeContext` in `rust/runtime/src/state.rs:536` stays — it's still required for the `INIT_DONE` synchronization pattern.

### Section 2 — Size tracking via build.rs

`rust/runtime/build.rs` already emits `sizing.rs` to `OUT_DIR` from Kconfig values (`CONFIG_RUNTIME_CURVE_POOL_N`, etc.). Extend it to also emit:

- A `pub const RT_STORAGE_SIZE: usize = …;` line into `sizing.rs` (consumed by Rust as today).
- A small C header (`runtime_storage.h`) with `#define RT_STORAGE_SIZE …` to a path the C build can find.

Picking the constant:

```rust
// In build.rs (pseudocode):
let actual = core::mem::size_of::<RuntimeContext>();  // computed by including a sizing-only Rust file
let rounded = round_up_pow2(actual + 16 * 1024);      // round up + 16 KB headroom
emit("pub const RT_STORAGE_SIZE: usize = {rounded};");
emit_c_header("#define RT_STORAGE_SIZE {rounded}");
```

(Caveat: `size_of` in `build.rs` requires a host-build of the type. The exact mechanism — separate `cargo metadata --message-format json` query, or a build-script-internal `rustc --print type-sizes` parse, or a hand-tracked-then-asserted constant — is an implementation detail; the Rust-side `const _: () = assert!(...)` enforces the contract regardless of how the constant was derived.)

C-side upper-bound assert in the linker script-aware translation unit:

```c
// In some .c file that has visibility into AXI_SRAM_SIZE and the other .axi_bss occupants:
#if CONFIG_MACH_STM32H7
_Static_assert(
    RT_STORAGE_SIZE
        + KALICO_SEGMENT_QUEUE_AXI_BYTES
        + DEMUX_AXI_BYTES
        + SERIAL_IRQ_AXI_BYTES
        + BENCH_AXI_BYTES
        <= AXI_SRAM_SIZE,
    "AXI SRAM overflow — RuntimeContext too large for AXI region"
);
#endif
```

If exact byte accounting for the other occupants is fiddly, an `objdump`-driven CI check is an acceptable substitute (Section 4).

### Section 3 — Per-MCU placement

H7 (LARGE profile, CURVE_POOL_N=16): `rt_storage` lands in `.axi_bss` (linker rule `> axi_ram`, AXI SRAM at 0x24000000, 320 KB total).

F4 (SMALL profile, CURVE_POOL_N=4): no AXI SRAM exists; `.axi_bss` is not declared in the linker script today. The C declaration's `__attribute__((section(".axi_bss")))` needs to either be cfg-gated out on F4 (option A) or accommodated by a F4 linker-script fallback (option B):

- **Option A (cfg-gated attribute):**
  ```c
  #if CONFIG_MACH_STM32H7
  __attribute__((section(".axi_bss"), used, externally_visible))
  #else
  __attribute__((used, externally_visible))
  #endif
  _Alignas(16) uint8_t rt_storage[RT_STORAGE_SIZE];
  ```
  Pro: explicit per-target. Con: C source diverges per MCU.
- **Option B (linker fallback):** Move the `*(.axi_bss .axi_bss.*) > ram` placement out of the `#if CONFIG_MACH_STM32H7` block into a default-rule that fires when AXI SRAM is unavailable. Pro: uniform C declaration; per-target behavior lives where per-target placement decisions belong (the linker script). Con: linker-script edit.

**Recommendation: option B.** Aligns with B2 ("section ownership lives in the linker script"). Implementation detail; either is acceptable.

## Audit pass

Items to sweep after the RT_CELL migration lands.

### Inventory items (cheap)

- **`#[link_section]` uses across `rust/`:** confirm RT_CELL was the only one; if any new ones appeared, fix per B2.
- **`.axi_bss` C occupants on H7:** enumerate (`rt_storage` + `kalico_segment_queue` + `kalico_demux.c:51` + `serial_irq.c:29` + `runtime_bench.c:23`) and add the inventory to `docs/kalico-rewrite/mcu-c-rust-boundary.md` § "What lives where on the MCU" for grep-ability.
- **`extern "C"` signatures on the Rust side (~70 functions in `rust/kalico-c-api/src/runtime_ffi.rs`):** grep for signature patterns that cross the ABI with non-`#[repr(C)]` types — slice (`&[T]` / `&mut [T]`), tuple, `Option<&T>` where `T: !Sized`, Rust enum without `#[repr(C)]`, raw `bool` (use `u8`). Most signatures are simple accessors (`fn foo(rt: *mut KalicoRuntime) -> u32`) and will pass cleanly; the ones to actually look at carefully are `kalico_configure_axes`, `kalico_runtime_configure_axes_blob`, `runtime_handle_push_segment`, `runtime_handle_load_curve`, and the endstop functions.

### Specific risks flagged by the rust-engineer review

These came out of the RT_CELL design subagent and are scoped into this refactor's audit pass:

- **`engine.rs:22-34` `static mut kalico_producer_current_present: u8` import.** Imported from `src/kalico_segment_queue.c:138` as a bare `static mut`. The accessor-function path (`engine.rs:44`) is the safe read. If any code path reads the `static mut` directly rather than through the accessor, it carries the same LLVM-miscompilation risk that motivated the 2026-05-18 SPSC queue C-side move. Verify; if any direct reads exist, route them through accessor functions.
- **`portable_atomic::AtomicU64` `AcqRel` in ISR hot path.** `SharedState` uses `AcqRel` on `fetch_add` operations (e.g., `state.rs:298`). On thumbv7em-none-eabihf, `portable_atomic`'s `AtomicU64` fallback uses a critical section (disables interrupts) for the duration of the operation. In the TIM5 ISR hot path this is per-sample. Audit: confirm whether the cost is acceptable, or migrate the affected counters to a pair of `AtomicU32`s (split-and-recombine pattern) with `Relaxed` ordering where the cross-half barrier is provided elsewhere.
- **`runtime_irq_save` / `runtime_irq_restore` declarations** (`state.rs:103-106`, `#[allow(dead_code)]`). If unused in the current build, delete them to shrink the FFI surface. If still used by the Phase 7 §8.5 flush path, leave them and remove the `dead_code` allow.

### Atomics audit notes (B5)

Cross-language atomic-ordering discipline (B5: "Rust does the same as C — atomics with explicit `Ordering` matching C's `__atomic_*`, or `read_volatile`/`write_volatile` matching C's plain volatile, never one side 'upgrading' silently"). Spot-check (not exhaustive):

- `runtime_clock_freq: u32` imported from C — non-atomic, set once at init, read many times. C side writes it before TIM5 is armed; Rust reads it with plain load. Acceptable: write-once-before-publish pattern.
- `kalico_producer_current_present` — covered above.
- Other C-side counters / status words imported into Rust — enumerate during the audit; classify each as atomic-on-both-sides or volatile-on-both-sides; fix any mismatch.

## Doc reconciliation

Updates to `docs/kalico-rewrite/mcu-c-rust-boundary.md`:

1. **B1 wording.** Rewrite from "small, named list of entry points" to acknowledge the opaque-handle API surface. Logical entry-point inventory:
   - `runtime_tick` (timer-driven, per-sample).
   - `KalicoRuntime` opaque-handle API (one logical surface; ~70 functions in `rust/kalico-c-api/src/runtime_ffi.rs` — give a pointer to the file rather than enumerating).
   - `runtime_diag_progress` (diagnostics readback).
   - `runtime_handle_create` (init-once).
   - Any future additions list themselves here.
2. **"Open migrations: RT_CELL" line.** Delete once the migration lands; replace with a one-line "Migration complete YYYY-MM-DD, commit `<sha>`" note OR remove the bullet entirely. Either is acceptable; the case study captures the history.
3. **"What lives where on the MCU" table.** Update the `RT_CELL` row from "Rust today; should migrate" to "C-declared `rt_storage` byte buffer (`src/runtime_storage.c`); Rust types it as `*mut RuntimeContext` via `extern "C"` import." Add the `.axi_bss` inventory enumeration (see Audit § Inventory items).
4. **Case studies.** Add a new dated entry for the RT_CELL migration once it lands, citing the spec (this file) and the implementation commit.

## Verification

In order; do not skip:

1. **Build.** `make` succeeds on both H7 and F4 configs from a clean tree (clean both the C build via `make clean` AND the Rust build via `cargo clean` per `feedback_cargo_clean_between_mcus.md`). Verify `axi-bss-placement` feature gone from the build invocation (it should no longer appear in `cargo build --features ...`).
2. **`objdump` placement check.** On H7 build: `objdump -t out/klipper.elf | grep rt_storage` shows the symbol in `.axi_bss` at an address in `[0x24000000, 0x24050000)`. On F4 build: shows the symbol in `.bss` at an address in `[0x20000000, 0x20020000)`. The current memory entry `feedback_cargo_clean_between_mcus.md` documents this check today as a manual ritual; the refactor turns it into a CI-grade gate (script in `scripts/` or a Makefile target).
3. **Static_assert + const_assert build-time checks.** Both fire on size mismatch / AXI overflow; deliberately oversize `RuntimeContext` in a throwaway branch to confirm both backstops trigger.
4. **Existing test suite.** `cargo test` across the workspace passes — particularly the `kalico-c-api` tests (`tests/configure_axes_blob_step_modes.rs`, `tests/drain_trace_credit.rs`, and the A1–A7 deterministic test battery noted in CLAUDE.md Step 7-C-io). They exercise the FFI surface directly and would scream on a boundary regression.
5. **Bench: H7.** Flash via the standard `commit → push → pull on Pi → make → flash` flow (`feedback_bench_firmware_flow.md`). Run a representative jog (per user's bench-smoke sequence — execution requires per-command permission per `feedback_no_gcode_without_permission.md`). Motion works end-to-end.
6. **Bench: F4.** Flash. F4 doesn't run the Rust motion engine for Z today (Klipper's stepper path handles it), but the F4 build still instantiates `RuntimeContext`. Confirm: boots, no MCU-shutdown, steady-state for ≥1 minute.

If any gate fails: do not declare the refactor done. Diagnose, fix, re-run from the failed step.

## Open questions resolved at implementation time

These are deliberately deferred from this design to plan-writing / implementation; calling them out so the plan-writing skill picks them up:

- **`build.rs` ↔ C-header export mechanism.** Several equally-valid paths (copy to `out/generated/`, add to `KALICO_CFLAGS` include path, use `cargo metadata`-driven shim, etc.). Pick the path of least disruption to the existing Makefile.
- **`__attribute__((section))` on F4: option A (cfg-gated) vs. option B (linker fallback)?** Recommendation in Section 3 is option B; confirm during implementation by reading the existing `.axi_bss` C occupants and seeing how they handle the F4 build today.
- **Exact RT_STORAGE_SIZE value per profile.** Compute from `size_of::<RuntimeContext>()` under each profile, round up with headroom for incidental growth, fits inside the available RAM region after other occupants.
- **`portable_atomic::AtomicU64` audit outcome.** May result in a follow-up refactor sub-task (split-into-AtomicU32-pair); decision lives in the plan, not in this spec.

## References

- Boundary invariant doc: [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](../../kalico-rewrite/mcu-c-rust-boundary.md).
- Dependency graph (Layer 4 MCU runtime context): [`docs/kalico-rewrite/dependency-graph.md`](../../kalico-rewrite/dependency-graph.md).
- Memory entries motivating the refactor:
  - `feedback_cargo_clean_between_mcus.md` (the operational trip-wire this refactor structurally fixes).
  - `project_dynmem_persistent_diag_overlap.md` (precedent: section-overlap class of bug).
  - `project_f446_configure_axes_crash.md` (precedent: C owns boot / CPU-state setup).
- Existing per-MCU sizing pipeline: `rust/runtime/build.rs` (Kconfig → `OUT_DIR/sizing.rs`), `src/Kconfig:445` (`config RUNTIME_CURVE_POOL_N`, with `RUNTIME_TARGET_LARGE` / `RUNTIME_TARGET_SMALL` profile defaults), `rust/runtime/src/curve_pool.rs:1-30` (consumer-side comment). The narrative design doc that originally established this pipeline was removed in commit `b8d3315c7` ("remove stale docs"); the live source-of-truth is the code above.
- Code locations (this branch, `mcu-boundary-refactor` worktree):
  - `rust/kalico-c-api/src/runtime_ffi.rs:55-69` — current `RT_CELL` definition.
  - `rust/kalico-c-api/src/runtime_ffi.rs:1-17` — half-split aliasing discipline doc.
  - `rust/runtime/src/state.rs:508-536` — `RuntimeContext` definition.
  - `rust/runtime/src/state.rs:543-…` — `RuntimeContext::init`.
  - `rust/runtime/build.rs` — existing Kconfig-to-Rust constants flow to extend.
  - `src/generic/armcm_link.lds.S:109-132` — `.axi_bss` and `.bkp_bss` linker rules.
  - `src/kalico_segment_queue.c` — reference example of C-declared `.axi_bss` static + Rust import.
