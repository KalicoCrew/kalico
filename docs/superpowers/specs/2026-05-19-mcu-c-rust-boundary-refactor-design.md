# MCU C/Rust Boundary Refactor — Design

> Date: 2026-05-19 (v2 — revised after two adversarial reviews flagged blockers in v1)
> Status: brainstormed + reviewed, ready for plan-writing
> Scope owner: claude+user. Implementation: subagent-driven via `rust-engineer` (per saved feedback memory; any Rust-touching subagent task uses `subagent_type: "rust-engineer"`).
> Companion to: [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](../../kalico-rewrite/mcu-c-rust-boundary.md) — the architectural invariant this refactor implements.

## Revision log

- **v1 → v2 (2026-05-19, post-review).** Two adversarial reviews (codex + opus kalico-plan-reviewer) returned CHANGES REQUESTED with substantial agreement on four blockers and additional significant issues. Revisions in this v2: (1) aliasing mechanism specified concretely with `UnsafeCell` wrapper, no shared-reference path; (2) size-tracking architecture switched from build-time `size_of` (chicken-and-egg) to Kconfig per-profile ceiling + Rust `const_assert` backstop; (3) F4 placement recommendation switched from option B (linker fallback — both reviewers showed it's wrong) to option A (cfg-gated attribute); (4) `.axi_bss` inventory corrected (boundary doc + this spec both fixed); (5) audit pass expanded with panic-in-ISR, FPU/CPACR consistency, `bool`-FFI policy, DMA cacheability; (6) verification expanded with `cargo check --features mcu-h7/mcu-f4`, Renode soak, A1–A7 deterministic battery, section-name-aware `objdump` check, stale-header drift test; (7) AtomicU64 audit scope locked down (audit-only; splits as a follow-up); (8) build.rs ↔ C-header export mechanism specified (no longer "open question").

## Motivation

The architectural invariant in [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](../../kalico-rewrite/mcu-c-rust-boundary.md) (rules B1–B5) is true of most of the MCU code today but has one prominent code-side violation:

- **Code violation (B2):** `RT_CELL` (`rust/kalico-c-api/src/runtime_ffi.rs:68`) is a Rust static that uses `#[link_section = ".axi_bss"]` to dictate its own placement. Per B2 ("Linker sections are named in the linker script ... Rust never `#[link_section]`s into them to introduce a new section. Section ownership and section layout live in one place: the C side."), this storage should be declared on the C side. The operational discipline that currently keeps this working (mandatory `cargo clean` between H7 and F4 builds, per `feedback_cargo_clean_between_mcus.md`) is a tripwire, not a fix.

Recent failures that motivate hardening the boundary, all spelled out in `docs/kalico-rewrite/mcu-c-rust-boundary.md` § Case studies:

- 2026-05-18 SPSC `Consumer` miscompile (resolved by moving the queue to a C struct in regular `.bss` / DTCM on H7).
- 2026-05-12 `dynmem` / `.persistent_diag` overlap (resolved by single-language ownership of section placement).
- 2026-05-12 F446 `configure_axes` crash (resolved by C-side CPACR setup at boot).
- `RT_CELL` H7 vs F4 section confusion (resolved operationally; structural fix is this refactor).

None of these were "Rust and C don't compose" failures — they were all "two compilation units disagreeing about who owns this memory," which is what B2 is designed to prevent. This refactor closes the last code-side instance of that class.

## Goals

1. Move `RT_CELL`'s storage to a C-declared `uint8_t` buffer, with section placement decided exclusively by the C linker script (B2).
2. Delete the `#[link_section]` attribute and the `axi-bss-placement` Cargo feature flag (residue of the prior Rust-owned placement scheme).
3. Sweep the rest of the C ↔ Rust boundary for other B3 / B4 / B5 leaks; fix what's found.
4. Update [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](../../kalico-rewrite/mcu-c-rust-boundary.md) once the refactor lands: retire the "Open migrations: RT_CELL" line, add a new case-study entry, and refresh the "What lives where on the MCU" table.
5. **Everything works on both H7 and F446 at the end.** Load-bearing acceptance criterion the user named explicitly. Bench-flash + motion-works on H7, bench-flash + steady-state on F4.

## Non-goals

- **No consolidation of the 82-function `extern "C"` surface.** The opaque-handle API pattern is correct; the boundary doc's B1 wording has been corrected (in commit `e812d436b`) to acknowledge logical-entry-points vs. physical-symbol-count. No code consolidation required.
- **No port of safety-critical Klipper C (heater control, thermistor, pin overlap) to Rust.** Out of scope per the boundary doc's "Why mixed, not all-of-one" section.
- **No introduction of a `bindgen`-generated header.** Manual mirroring at the current scale is fine; revisit when the shared-struct surface exceeds ~5–10 types.
- **No changes to the `.persistent_diag` design.** Already correctly C-owned; touch nothing.
- **No changes to the segment SPSC queue placement.** Already correctly C-owned in regular `.bss` / DTCM-on-H7 (the 2026-05-18 fix); the DTCM placement is deliberate for cache-coherency reasons. Touch nothing.
- **No change to which `.axi_bss` regions are cached vs. non-cached on H7.** Cacheability and MPU config are out of scope for this refactor. The audit pass (§ Audit / DMA cacheability) confirms the status quo is correct; it does not modify it.
- **`AtomicU64` → AtomicU32-pair migration is NOT in scope.** The audit pass identifies whether the current `portable_atomic::AtomicU64` `AcqRel`-in-ISR cost is acceptable; if it is not, a follow-up sub-spec captures the split-into-pair work. This refactor only audits; it does not migrate.

## Architecture

### What changes

- `rt_storage` becomes a C-declared `uint8_t` buffer in a new file `src/runtime_storage.c`, placed via `__attribute__((section(...)))`. The C `.axi_bss` attribute is cfg-gated to `CONFIG_MACH_STM32H7` (option A — see § Detailed design § Section 3). The linker script is unchanged.
- `RT_STORAGE_SIZE` is a Kconfig-derived constant. Per-profile values (LARGE for H7, SMALL for F4) are picked as a ceiling above the maximum plausible `size_of::<RuntimeContext>()` under that profile. Both sides (Rust + C) read the constant from the existing Kconfig → autoconf.h → build.rs pipeline. **Rust enforces the contract at compile time** with `const _: () = assert!(size_of::<RuntimeContext>() <= RT_STORAGE_SIZE);`. **C enforces the upper bound** with `_Static_assert(RT_STORAGE_SIZE + axi_bss_other_users <= AXI_SRAM_SIZE, ...)` on H7.
- `rust/kalico-c-api/src/runtime_ffi.rs`: delete `RuntimeCell`, `RT_CELL`, the `#[link_section]` attribute, the `axi-bss-placement` Cargo feature, and the `Sync` impl on the wrapper. Replace with an `UnsafeCell`-wrapped `extern "C"` import (see § Detailed design § Section 1 for the exact declaration — this is load-bearing, not a placeholder).
- `Cargo.toml` (kalico-c-api): remove the `axi-bss-placement` feature.
- `Makefile`: drop any feature-flag plumbing for `axi-bss-placement`.

### What stays unchanged

- The half-split `FgState` / `IsrState` raw-pointer projection pattern (`rust/kalico-c-api/src/runtime_ffi.rs:1–17`). Only the origin of the pointer changes — from `RT_CELL.0.get()` to `rt_storage.get().cast()`. The `addr_of!` / `UnsafeCell::raw_get` projection chain inside `RuntimeContext` is identical and remains sound under stacked-borrows / tree-borrows. The aliasing-soundness argument for the *root* pointer is rewritten in § Section 1 to use the `UnsafeCell` wrapper, addressing the v1-blocker on this point.
- `RuntimeContext::init`'s raw-pointer write protocol (`rust/runtime/src/state.rs:551`). It already writes through raw-pointer projections without ever materializing `&mut RuntimeContext`, so it works identically against a C-declared backing buffer.
- The linker script's `.axi_bss` declaration (`src/generic/armcm_link.lds.S:114-120`, currently gated `#if CONFIG_MACH_STM32H7` and targeting `> axi_ram` — an H7-only MEMORY region). Not modified by this refactor.
- All other `.axi_bss` C occupants (`src/kalico_demux.c:51` → `kalico_buf`, `src/generic/runtime_bench.c:23` → `runtime_bench_samples_buf`, `src/generic/serial_irq.c:29` → `receive_buf`). They are all `#if CONFIG_MACH_STM32H7`-gated already and follow B2. `rt_storage` joins this inventory.
- The segment SPSC queue (`src/kalico_segment_queue.c`) — already in regular `.bss` (DTCM on H7), not `.axi_bss`. Touch nothing.

## Detailed design

### Section 1 — RT_CELL migration mechanism

**Aliasing-soundness foundation.** This v2 specifies the exact extern declaration. Two reviewers agreed v1's `extern "C" { static rt_storage: [u8; N]; }` was unsound because a shared-reference path through an immutable extern static does not grant write provenance to pointers derived from it. The fix: wrap in `UnsafeCell` on the Rust import side. `UnsafeCell<T>` has the same layout and ABI as `T` (guaranteed by the Rust reference), so the C declaration is a plain `uint8_t` array; only the Rust import expresses interior mutability.

C side (new file `src/runtime_storage.c`):

```c
#include "autoconf.h"           // CONFIG_MACH_STM32H7, CONFIG_RUNTIME_TARGET_LARGE/SMALL
#include "runtime_storage.h"    // declares RT_STORAGE_SIZE, exposes extern rt_storage

#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss"), used, externally_visible))
#else
__attribute__((used, externally_visible))
#endif
_Alignas(16) uint8_t rt_storage[RT_STORAGE_SIZE];

// Belt-and-suspenders sanity check. The contract enforcement is Rust-side
// (const_assert!(size_of::<RuntimeContext>() <= RT_STORAGE_SIZE)); this
// catches a Kconfig misconfiguration before runtime.
_Static_assert(RT_STORAGE_SIZE >= 1024,
               "RT_STORAGE_SIZE absurdly small — Kconfig profile broken");
```

Header `src/runtime_storage.h`:

```c
#ifndef KALICO_RUNTIME_STORAGE_H
#define KALICO_RUNTIME_STORAGE_H
#include "autoconf.h"

#if CONFIG_RUNTIME_TARGET_LARGE
#  define RT_STORAGE_SIZE CONFIG_RUNTIME_STORAGE_SIZE_LARGE
#elif CONFIG_RUNTIME_TARGET_SMALL
#  define RT_STORAGE_SIZE CONFIG_RUNTIME_STORAGE_SIZE_SMALL
#else
#  error "No RUNTIME_TARGET_* profile selected"
#endif

extern uint8_t rt_storage[RT_STORAGE_SIZE];
#endif
```

Rust side (`rust/kalico-c-api/src/runtime_ffi.rs`):

```rust
use core::cell::UnsafeCell;

// UnsafeCell<[u8; N]> is layout-compatible with [u8; N]; C side declares
// the plain uint8_t array. The UnsafeCell wrapper exists purely so the
// Rust aliasing model grants interior-mutability rights to pointers
// projected out via .get(), matching the v1-blocker fix from the
// 2026-05-19 review.
unsafe extern "C" {
    static rt_storage: UnsafeCell<[u8; RT_STORAGE_SIZE]>;
}

const _: () = assert!(
    core::mem::size_of::<RuntimeContext>() <= RT_STORAGE_SIZE,
    "RuntimeContext outgrew RT_STORAGE_SIZE — bump CONFIG_RUNTIME_STORAGE_SIZE_{LARGE,SMALL}"
);
const _: () = assert!(
    core::mem::align_of::<RuntimeContext>() <= 16,
    "RuntimeContext alignment > 16 — bump _Alignas in runtime_storage.c"
);
```

`runtime_handle_create` cast change (line ~119):

```rust
// BEFORE (v1):
// let rt_ptr: *mut RuntimeContext = (*RT_CELL.0.get()).as_mut_ptr();

// AFTER:
let rt_ptr: *mut RuntimeContext = rt_storage.get().cast::<RuntimeContext>();
debug_assert_eq!(
    (rt_ptr as usize) % core::mem::align_of::<RuntimeContext>(),
    0,
    "rt_storage alignment mismatch — linker placed it unaligned"
);
```

Note: `UnsafeCell::get()` returns `*mut [u8; N]`, which we `.cast::<RuntimeContext>()` to get `*mut RuntimeContext`. No shared reference (`&[u8; N]`) is ever formed from `rt_storage`, side-stepping the v1-blocker aliasing concern entirely. `RuntimeContext::init` then writes through this pointer using its existing `addr_of_mut!` projection chain (`state.rs:551`), and subsequent FFI calls receive the same pointer back as `rt: *mut KalicoRuntime`, cast back to `*mut RuntimeContext`, with the half-split `addr_of!` projection chain (`runtime_ffi.rs:172-176`) unchanged.

Deletions in `runtime_ffi.rs`:
- `RuntimeCell` struct + `Sync` impl (current lines 55–61).
- `RT_CELL` static (current lines 68–69).
- `#[cfg_attr(feature = "axi-bss-placement", unsafe(link_section = ".axi_bss"))]` attribute.

Deletions in `rust/kalico-c-api/Cargo.toml`:
- `axi-bss-placement` feature.

The `unsafe impl Sync for RuntimeContext` in `rust/runtime/src/state.rs:536` stays — still required for the `INIT_DONE` synchronization pattern. The half-split discipline doesn't change.

### Section 2 — Size tracking architecture

v1 proposed `build.rs` compute `size_of::<RuntimeContext>()` and emit it. Both reviewers flagged this as chicken-and-egg: `build.rs` runs *before* the runtime crate compiles, and `RuntimeContext` depends transitively on Kconfig-derived constants the build script emits (`CURVE_POOL_N`, etc.). The build script cannot evaluate `size_of` for a type it hasn't yet enabled the compilation of.

v2 architecture: **Kconfig provides per-profile ceilings; Rust `const_assert!` backstops correctness.**

Kconfig additions (`src/Kconfig`, adjacent to existing `RUNTIME_TARGET_LARGE` / `RUNTIME_TARGET_SMALL`):

```
config RUNTIME_STORAGE_SIZE_LARGE
    int "rt_storage byte ceiling (large profile, H7)"
    default 327680   # 320 KB; tune to leave headroom in AXI SRAM
    depends on KALICO_RUNTIME

config RUNTIME_STORAGE_SIZE_SMALL
    int "rt_storage byte ceiling (small profile, F4)"
    default 65536    # 64 KB; tune to leave headroom in main SRAM
    depends on KALICO_RUNTIME
```

(Exact defaults computed during implementation by building each profile, reading `size_of::<RuntimeContext>()` from a host build or the symbol size in the linked staticlib, and rounding up with comfortable headroom. The const_assert backstops correctness; the human-chosen ceiling provides growth room.)

C side: `autoconf.h` exposes `CONFIG_RUNTIME_STORAGE_SIZE_LARGE` and `CONFIG_RUNTIME_STORAGE_SIZE_SMALL`. `runtime_storage.h` (shown in § Section 1) selects between them based on the active profile.

Rust side: `rust/runtime/build.rs` already reads Klipper's autoconf and emits `OUT_DIR/sizing.rs` with profile-aware constants. Extend it to emit `pub const RT_STORAGE_SIZE: usize = …;` from the same Kconfig source. The current pipeline `include!`s `sizing.rs` directly into `curve_pool.rs` (exposing constants at `runtime::curve_pool::*`); the plan-writer picks whether to add a top-level `pub use ... RT_STORAGE_SIZE` in `runtime/src/lib.rs` or move sizing into a dedicated `runtime::sizing` module. `runtime_ffi.rs` then uses `RT_STORAGE_SIZE` via that path in the `const _: () = assert!(...)` check.

**No `build.rs` C-header generation is needed.** Both sides read from the same Kconfig source independently (C via `autoconf.h`, Rust via `build.rs` parsing the same file). The single source of truth is `src/Kconfig`. This resolves both the chicken-and-egg blocker and the v1 open question about `build.rs ↔ C-header export mechanism` — there is no header to export.

C-side AXI SRAM overflow assert. In `src/runtime_storage.c` (or wherever the AXI inventory is enumerated):

```c
#if CONFIG_MACH_STM32H7
// Other .axi_bss occupants on H7. Update this list when adding new .axi_bss statics.
#define AXI_BSS_KALICO_BUF_BYTES        2048   // kalico_demux.c
#define AXI_BSS_RUNTIME_BENCH_BYTES     4096   // runtime_bench.c (verify at impl time)
#define AXI_BSS_SERIAL_IRQ_RX_BYTES     2048   // serial_irq.c RX_BUFFER_SIZE
// (segment queue is NOT in .axi_bss — lives in DTCM/regular .bss)

#define AXI_BSS_OTHER_USERS \
    (AXI_BSS_KALICO_BUF_BYTES + AXI_BSS_RUNTIME_BENCH_BYTES + AXI_BSS_SERIAL_IRQ_RX_BYTES)
#define AXI_SRAM_SIZE (320 * 1024)

_Static_assert(RT_STORAGE_SIZE + AXI_BSS_OTHER_USERS + 16384 /* headroom */ <= AXI_SRAM_SIZE,
               "AXI SRAM overflow: RT_STORAGE_SIZE too large for AXI region");
#endif
```

Exact byte counts of other occupants are looked up during implementation. The headroom term (16 KB here) is a sanity margin; tune at implementation time.

### Section 3 — Per-MCU placement (option A: cfg-gated attribute)

v1 recommended option B (move `.axi_bss` linker rule outside the H7 `#if`). Both reviewers showed this is wrong:

- **Codex:** orphan output sections may be placed by the linker at addresses outside `[_bss_start, _bss_end]`, escaping the boot zeroing pass and leaving `rt_storage` uninitialized on F4 cold boot.
- **Opus:** the H7 `.axi_bss` rule targets `> axi_ram`, an H7-only MEMORY region; moving it out of `#if CONFIG_MACH_STM32H7` would fail to link on F4.

v2 adopts **option A: cfg-gated section attribute** (shown in § Section 1). On H7, `__attribute__((section(".axi_bss")))` places `rt_storage` via the existing linker rule. On F4, no section attribute is applied — `rt_storage` lands in regular `.bss` via the default rule, and is zeroed by `armcm_boot.c::boot_memset` (covered by the `[_bss_start, _bss_end]` zeroing pass).

The linker script (`src/generic/armcm_link.lds.S`) is unchanged.

## Audit pass

After the RT_CELL migration lands, sweep the rest of the boundary. Items below.

### A1. Inventory items (cheap)

- **`#[link_section]` uses across `rust/`.** Confirm `RT_CELL` was the only one; if any new ones appeared, fix per B2. Grep is `rg -n '#\[link_section\|link_section\b'`.
- **`.axi_bss` C occupants on H7.** Enumerate the inventory list with exact byte sizes (`rt_storage` + `kalico_buf` + `runtime_bench_samples_buf` + `receive_buf`); update `docs/kalico-rewrite/mcu-c-rust-boundary.md` § "What lives where on the MCU" table with the live inventory (this was partially done in commit `e812d436b`; the byte sizes get filled in after measurement).
- **`extern "C"` signatures on the Rust side (82 functions in `rust/kalico-c-api/src/runtime_ffi.rs`).** Grep for signature patterns that cross the ABI with non-`#[repr(C)]` types — slices (`&[T]` / `&mut [T]`), tuples, `Option<&T>` where `T: !Sized`, Rust enums without `#[repr(C)]`. Most signatures are simple accessors (`fn foo(rt: *mut KalicoRuntime) -> u32`); the ones to look at carefully are `kalico_configure_axes`, `kalico_runtime_configure_axes_blob`, `runtime_handle_push_segment`, `runtime_handle_load_curve`, and the endstop functions.

### A2. `bool`-FFI policy

Codex flagged: the boundary doc B4 says "no `bool` (use `uint8_t`)," but `runtime_ffi.rs:2145, 2234, 2400, 2985` already return `bool`. The audit must resolve this contradiction. Two options:

- **Accept `bool`.** Rust's `bool` is layout-compatible with C99 `_Bool` (both 1 byte, both 0/1). Document this in B4 ("`bool` is permitted; treat as `_Bool` on the C side"). Add a `<stdbool.h>` include where C reads it.
- **Migrate to `u8`.** Change the four return sites and their C-side consumers.

Recommendation: **accept `bool`**. Layout-compatibility is guaranteed by both standards; the boundary doc's "no bool" rule was overly conservative. Update B4 to permit `bool` with the `_Bool` contract documented.

### A3. `static mut` imports from C

Specifically: `rust/runtime/src/engine.rs:23` imports `kalico_producer_current_present` as `static mut`. The 2026-05-18 SPSC fix's analysis says this kind of cross-language `static mut` carries the same LLVM-miscompilation risk that motivated the queue C-side move. The accessor-function path (`engine.rs:44`) is the correct read.

**Audit action (structural fix, not investigation).** Grep all reads of `kalico_producer_current_present` from Rust. Any direct reads route through an accessor function. If the only reads are already through the accessor, delete the `static mut` import entirely.

### A4. `portable_atomic::AtomicU64` ordering audit (scope-limited)

`SharedState` uses `AcqRel` on `fetch_add` (`state.rs:298` and similar). On thumbv7em-none-eabihf, `portable_atomic`'s `AtomicU64` fallback uses a critical section (disables interrupts) for the duration of the op. In the TIM5 ISR hot path this is per-sample.

**Audit deliverable:** a short note (not a code change) documenting which `AtomicU64` operations are in the ISR hot path and whether the critical-section cost is acceptable. **If the audit decides a split-into-`AtomicU32`-pair is needed**, that work is captured in a **follow-up sub-spec** (its own design pass), not this refactor. This refactor's scope is the audit, not the migration. **Locked down per § Non-goals.**

### A5. Panic-in-ISR audit

Rust panic handler (`rust/kalico-c-api/src/lib.rs:29` or wherever the MCU panic handler lives) spins forever today. Rust is called from `TIM5_IRQHandler` and stepper-timer callbacks. A panic in those contexts locks inside an interrupt, preventing fault reporting and watchdog service.

**Audit deliverable:** map all panics reachable from C→Rust ISR call sites. Require panic-in-ISR to route through a C fault-latch / shutdown path (e.g., `fault_handler_report_task` or equivalent), not into a spin loop. This is structural-fix audit work, in scope.

### A6. FPU / CPACR consistency

The 2026-05-12 F446 crash was a CPACR / FPU-disabled issue. Rust runtime code uses `f32` heavily; C calls Rust from ISR contexts.

**Audit deliverable:** confirm at every C→Rust ISR call site that (a) the C build is hard-float ABI (`-mfloat-abi=hard -mfpu=fpv4-sp-d16` for F4, `-mfpu=fpv5-d16` for H7), (b) CPACR is enabled at boot per the 2026-05-12 fix, (c) FPU lazy stacking is configured consistently for ISRs, (d) the Rust staticlib was built with a matching float ABI. Compiler flags grep + boot-path read.

### A7. DMA cacheability for H7 `.axi_bss`

H7's AXI SRAM at 0x24000000 is normal-cacheable by default (unlike DTCM at 0x20000000, which is non-cached). The current `RT_CELL` Rust-static lands there and works; the question is whether anything in `.axi_bss` is DMA-touched, and if so, whether cache maintenance is correct.

**Audit deliverable:** confirm that (a) `RuntimeContext` is never DMA source/destination (it isn't today — the segment queue is in DTCM, the trace ring is internal to `RuntimeContext`), (b) the other `.axi_bss` occupants — `kalico_buf` (USB-CDC RX), `receive_buf` (serial IRQ RX), `runtime_bench_samples_buf` — either are not DMA-touched, or have correct cache maintenance, or rely on the H7 MPU marking `.axi_bss` non-cacheable. This is investigation-only audit work; no code changes expected. Per § Non-goals: "no change to which `.axi_bss` regions are cached vs. non-cached on H7."

### A8. Dead-code removal

- `runtime_irq_save` / `runtime_irq_restore` declarations (`state.rs:103-106`, currently `#[allow(dead_code)]`). Grep all callers across Rust, C, and the generated staticlib. If unused, delete the declarations. If still used (Phase 7 §8.5 flush path), remove the `dead_code` allow.

## Doc reconciliation

Updates to `docs/kalico-rewrite/mcu-c-rust-boundary.md` once the refactor lands:

1. **"Open migrations: RT_CELL" line.** Remove (or replace with a one-line "Migrated YYYY-MM-DD, commit `<sha>`" note).
2. **"What lives where on the MCU" table.** Update the `RT_CELL` row from "Rust today; should migrate" to "C-declared `rt_storage` byte buffer (`src/runtime_storage.c`); H7 in `.axi_bss`, F4 in regular `.bss`; Rust types it as `*mut RuntimeContext` via `extern "C"` + `UnsafeCell` import." Fill in measured byte sizes for the `.axi_bss` occupant inventory (the row added in `e812d436b`).
3. **Case studies.** Add a new dated entry for the RT_CELL migration once it lands.
4. **B4 wording.** Update to permit `bool` (per A2 audit recommendation) with the `_Bool` layout contract documented.

(Note: B1 wording and `.axi_bss` inventory errors in the boundary doc were fixed in commit `e812d436b`, prior to this refactor's implementation.)

## Verification

In order; do not skip:

### V1. Build gates

- **`cargo check --features mcu-h7 --no-default-features`** passes on the H7-feature profile (the staticlib variant linked into Klipper's H7 firmware build). Catches mcu-h7-specific feature-flag breakage that workspace-level `cargo test` does not exercise.
- **`cargo check --features mcu-f4 --no-default-features`** passes on the F4-feature profile.
- **`make` succeeds on both H7 and F4 configs** from a clean tree (clean both via `make clean` AND `cargo clean` per `feedback_cargo_clean_between_mcus.md`). Verify `axi-bss-placement` feature gone from the build invocation.

### V2. Compile-time contract gates

- **Rust-side `const_assert`** (`size_of::<RuntimeContext>() <= RT_STORAGE_SIZE`) fires when deliberately oversized; verifies the lower-bound contract is wired.
- **Rust-side `const_assert`** (`align_of::<RuntimeContext>() <= 16`) fires when deliberately misaligned; verifies the alignment contract.
- **C-side `_Static_assert`** (`RT_STORAGE_SIZE + AXI_BSS_OTHER_USERS + headroom <= AXI_SRAM_SIZE`) fires when RT_STORAGE_SIZE is bumped past the AXI overflow point; verifies the upper-bound contract.

Run these as deliberately-failing builds on a throwaway commit; revert before continuing.

### V3. Section-placement check (objdump, name-aware)

Codex flagged that an address check alone is insufficient because an orphan section on F4 can land at a valid RAM address but outside the zeroed BSS span. The check must verify section *name*, not just address.

- **H7 build:**
  ```
  objdump -t out/klipper.elf | grep ' rt_storage$'
  ```
  Symbol must appear in section `.axi_bss` (not a heuristic-placed orphan), at an address in `[0x24000000, 0x24050000)`.
- **F4 build:** symbol must appear in section `.bss` (or `.bss.*`), at an address in `[0x20000000, 0x20020000)`, inside the `[_bss_start, _bss_end]` span (verifiable via the same `objdump -t` cross-referencing the boot-zeroing symbols).

Codify as a `scripts/check_rt_storage_placement.sh` or equivalent Makefile target so it's not a manual ritual.

### V4. Stale-header drift gate

A C compile test asserts `sizeof(rt_storage) == RT_STORAGE_SIZE` (verifying C and Rust agree on the size at link time, not just compile time). Land this as a new test file in the existing test harness; pair with the existing cbindgen drift gate (`tests/headers_no_drift.rs` or equivalent).

### V5. Existing test suite

`cargo test` across the workspace passes — particularly the `kalico-c-api` tests (`tests/configure_axes_blob_step_modes.rs`, `tests/drain_trace_credit.rs`, and the **A1–A7 deterministic test battery** noted in CLAUDE.md Step 7-C-io). They exercise the FFI surface directly and would scream on a boundary regression. The A1–A7 battery is the closest analog to "does the FFI surface still behave correctly after pointer-provenance changes" — opus reviewer specifically flagged its absence from v1's verification.

### V6. Renode soak

The `INIT_DONE` workaround at `runtime_ffi.rs:105-113` (avoiding `compare_exchange` because Renode's H7 model silently drops STREXB) is preserved by this refactor, but the `runtime_handle_create` flow is modified. **Run the existing Renode sim soak (per Step 7-C-io infrastructure) on the migrated code** to confirm no regression in the init-once protocol under the Renode CPU model. This is a CI-gradable check, not a bench-only ritual.

### V7. Bench: H7

Flash via `commit → push → pull on Pi → make → flash` flow (`feedback_bench_firmware_flow.md`). Run the user's bench-smoke sequence (per-command permission required per `feedback_no_gcode_without_permission.md`). Motion works end-to-end.

### V8. Bench: F4

Flash. F4 doesn't run the Rust motion engine for Z today (Klipper's stepper path), but the F4 build instantiates `RuntimeContext`. Confirm: boots, no MCU-shutdown, steady-state for ≥1 minute.

If any gate fails: do not declare the refactor done. Diagnose, fix, re-run from the failed step.

## Open questions resolved at implementation time

These are deliberate deferrals to plan-writing / implementation, not architectural gaps:

- **Exact RT_STORAGE_SIZE values per profile.** Measure `size_of::<RuntimeContext>()` under each profile from a clean build; pick a ceiling above the measured value with a headroom margin; set in Kconfig defaults. Re-tune if `RuntimeContext` grows.
- **Exact byte counts of other `.axi_bss` occupants** for the AXI overflow `_Static_assert` formula. Look up at implementation time from `objdump` / source.
- **A2 `bool`-FFI policy ratification.** Recommended: accept `bool` with `_Bool` contract. Confirm during implementation.
- **A4 `AtomicU64` audit outcome.** Audit only; any split-to-`AtomicU32`-pair lands in a follow-up sub-spec.
- **A5 panic-in-ISR routing target.** Identify the right C fault-latch entry point (likely `fault_handler_report_task` or similar) during the audit.

## References

- Boundary invariant doc: [`docs/kalico-rewrite/mcu-c-rust-boundary.md`](../../kalico-rewrite/mcu-c-rust-boundary.md). Inventory + B1-wording fixes in commit `e812d436b`.
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
  - `src/kalico_segment_queue.c:31-40` — example of cross-boundary shared static (regular `.bss` / DTCM, deliberately not `.axi_bss`).
  - `src/kalico_demux.c`, `src/generic/runtime_bench.c`, `src/generic/serial_irq.c` — current `.axi_bss` C occupants.
- Adversarial reviews informing this v2: codex review (returned 7 issues, 3 blockers + 4 significant) and kalico-plan-reviewer review (returned 8 substantive issues + 6 advisory), both 2026-05-19. Substantial agreement on aliasing-soundness, size-tracking chicken-and-egg, F4 linker-fallback wrong, and `.axi_bss` inventory wrong.
