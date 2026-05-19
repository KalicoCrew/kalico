# MCU C/Rust Boundary Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking. All Rust-touching subagent tasks must use `subagent_type: "rust-engineer"` per saved feedback memory.

**Goal:** Move `RT_CELL`'s storage from a Rust-owned static with `#[link_section]` to a C-declared `rt_storage` buffer, eliminating the cargo-clean operational tripwire and bringing the MCU C/Rust boundary into compliance with rule B2 of `docs/kalico-rewrite/mcu-c-rust-boundary.md`.

**Architecture:** C declares `uint8_t rt_storage[RT_STORAGE_SIZE]` with a cfg-gated section attribute (`.axi_bss` on H7 only; default `.bss` on F4). Rust imports via `extern "C" { static rt_storage: UnsafeCell<[u8; N]>; }` and casts via `.get()` to `*mut RuntimeContext` — no shared-reference path, sound under stacked/tree borrows. Per-profile size ceilings flow from Kconfig (LARGE/SMALL profiles) through both Klipper's `autoconf.h` and the runtime crate's `build.rs`; `const_assert` on Rust and `_Static_assert` on C catch contract violations at compile time.

**Tech Stack:** Rust 2024 edition (no_std on MCU thumbv7em, std on host), C99 (Klipper firmware), Kconfig, GNU Make, GNU ld linker scripts, ARM Cortex-M7 (STM32H723) + Cortex-M4F (STM32F446).

**Spec:** [`docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md`](../specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md) v2.

---

## File structure

**Files to create:**

| Path | Responsibility |
|---|---|
| `src/runtime_storage.c` | C declaration of `rt_storage` buffer; cfg-gated section attribute; AXI overflow `_Static_assert` |
| `src/runtime_storage.h` | Declares `RT_STORAGE_SIZE` via Kconfig; exposes `extern rt_storage` |
| `scripts/check_rt_storage_placement.sh` | V3 section-placement gate (objdump-driven, name-aware) |
| `rust/kalico-c-api/tests/rt_storage_drift.rs` | V4 stale-header drift test — Rust assertion mirroring C `sizeof(rt_storage)` |
| `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md` | A1-A8 audit findings + decisions, single doc |

**Files to modify:**

| Path | Change |
|---|---|
| `src/Kconfig` | Add `RUNTIME_STORAGE_SIZE_LARGE` and `RUNTIME_STORAGE_SIZE_SMALL` |
| `src/Makefile` | Add `runtime_storage.c` to `CONFIG_KALICO_RUNTIME` src; add `KALICO_RUNTIME_STORAGE_SIZE` env passthrough |
| `rust/runtime/build.rs` | Read `KALICO_RUNTIME_STORAGE_SIZE` env, emit `RT_STORAGE_SIZE` constant |
| `rust/runtime/src/lib.rs` | Re-export `RT_STORAGE_SIZE` at crate root |
| `rust/kalico-c-api/Cargo.toml` | Remove `axi-bss-placement` from `mcu-h7` feature; delete `axi-bss-placement = []` |
| `rust/kalico-c-api/src/runtime_ffi.rs` | Replace `RT_CELL` block with `extern "C"` UnsafeCell import; update cast; delete RuntimeCell + Sync impl + cfg_attr; add const_asserts |
| `rust/runtime/src/engine.rs` | (A3) Route `kalico_producer_current_present` reads through C accessor only; remove bare `static mut` import if direct reads found |
| `rust/runtime/src/state.rs` | (A8) Delete `runtime_irq_save`/`runtime_irq_restore` declarations if unused |
| `docs/kalico-rewrite/mcu-c-rust-boundary.md` | (post-refactor) Update RT_CELL row; add case study; update B4 to permit `bool` |

---

## Phase 1 — Sizing foundation

### Task 1: Add Kconfig entries for RT_STORAGE_SIZE

**Files:**
- Modify: `src/Kconfig` (in the existing `RUNTIME_*` block, after `RUNTIME_CURVE_POOL_N`)

- [ ] **Step 1: Locate the existing RUNTIME_CURVE_POOL_N block**

```bash
grep -n 'config RUNTIME_CURVE_POOL_N' src/Kconfig
# Expected: a line number around 445
```

- [ ] **Step 2: Add the two new Kconfig entries after RUNTIME_CURVE_POOL_N**

Append to `src/Kconfig` immediately after the `RUNTIME_CURVE_POOL_N` block:

```kconfig
config RUNTIME_STORAGE_SIZE_LARGE
    int "rt_storage byte ceiling (large profile, H7)"
    default 327680   # 320 KB; tune to fit AXI SRAM with headroom for other .axi_bss occupants
    range 65536 524288
    depends on KALICO_RUNTIME

config RUNTIME_STORAGE_SIZE_SMALL
    int "rt_storage byte ceiling (small profile, F4)"
    default 65536    # 64 KB; tune to fit F4 main SRAM with headroom
    range 32768 131072
    depends on KALICO_RUNTIME
```

- [ ] **Step 3: Verify Kconfig parses by running menuconfig (headless check)**

Run: `make menuconfig` (or `make olddefconfig` for non-interactive verification).
Expected: no Kconfig parse errors. The two new entries appear under the `KALICO_RUNTIME` block.

If running headless, alternative verification:

```bash
make olddefconfig 2>&1 | grep -i error
# Expected: no output (no errors)
```

- [ ] **Step 4: Commit**

```bash
git add src/Kconfig
git commit -m "kconfig(runtime): add RUNTIME_STORAGE_SIZE_LARGE/_SMALL ceilings

Per-profile byte ceilings for rt_storage (the C-declared backing buffer
that replaces RT_CELL's Rust-side #[link_section] placement). Defaults
sized to fit H7's 320 KB AXI SRAM (LARGE) and F4's 128 KB main SRAM
(SMALL) with headroom. Const_assert on Rust + _Static_assert on C
(landing in subsequent commits) catch contract violations at compile
time.

Part of the MCU C/Rust boundary refactor — see
docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md."
```

---

### Task 2: Extend Makefile to pass RT_STORAGE_SIZE to build.rs

**Files:**
- Modify: `src/Makefile` (lines 79-82, the existing `KALICO_RUNTIME_*` env-var block)

- [ ] **Step 1: Identify which size constant to pass based on active profile**

Both `RUNTIME_TARGET_LARGE` and `RUNTIME_TARGET_SMALL` profiles select one of the two Kconfig values. Rather than picking on the Make side, pass both and let build.rs select; or pick on the Make side. **Pick on Make side (simpler):**

- [ ] **Step 2: Add the env-var passthrough**

In `src/Makefile`, locate the existing block (around lines 79-82):

```makefile
			KALICO_RUNTIME_MAX_CONTROL_POINTS=$(CONFIG_RUNTIME_MAX_CONTROL_POINTS) \
			KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN=$(CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN) \
			KALICO_RUNTIME_MAX_DEGREE=$(CONFIG_RUNTIME_MAX_DEGREE) \
			KALICO_RUNTIME_CURVE_POOL_N=$(CONFIG_RUNTIME_CURVE_POOL_N) \
```

Insert a new line immediately after `KALICO_RUNTIME_CURVE_POOL_N`:

```makefile
			KALICO_RUNTIME_STORAGE_SIZE=$(if $(CONFIG_RUNTIME_TARGET_LARGE),$(CONFIG_RUNTIME_STORAGE_SIZE_LARGE),$(CONFIG_RUNTIME_STORAGE_SIZE_SMALL)) \
```

- [ ] **Step 3: Verify the Make conditional resolves correctly**

```bash
make olddefconfig
make -n V=1 2>&1 | grep KALICO_RUNTIME_STORAGE_SIZE
# Expected: shows the env var resolving to either CONFIG_RUNTIME_STORAGE_SIZE_LARGE
# or _SMALL based on which RUNTIME_TARGET_* is active.
```

- [ ] **Step 4: Commit**

```bash
git add src/Makefile
git commit -m "make(runtime): pass KALICO_RUNTIME_STORAGE_SIZE env to build.rs

Selects between RUNTIME_STORAGE_SIZE_LARGE (H7) and _SMALL (F4) based
on the active RUNTIME_TARGET_* profile, exporting the chosen value
under the existing env-var passthrough pattern. The runtime crate's
build.rs (next commit) consumes this to emit a Rust-side RT_STORAGE_SIZE
constant matching the C-side ceiling."
```

---

### Task 3: Extend build.rs to emit RT_STORAGE_SIZE

**Files:**
- Modify: `rust/runtime/build.rs`

- [ ] **Step 1: Read the current build.rs**

The current file reads four env vars via `lookup()` and emits four `pub const`s. Pattern is established; we add a fifth.

- [ ] **Step 2: Add the new env var read and emit**

In `rust/runtime/build.rs`, modify the `main()` function. Find this block:

```rust
    let mcp = lookup("KALICO_RUNTIME_MAX_CONTROL_POINTS", "1830");
    let mkv = lookup("KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN", "1850");
    let mdg = lookup("KALICO_RUNTIME_MAX_DEGREE", "10");
    let cpn = lookup("KALICO_RUNTIME_CURVE_POOL_N", "16");
```

Add a fifth line:

```rust
    let mcp = lookup("KALICO_RUNTIME_MAX_CONTROL_POINTS", "1830");
    let mkv = lookup("KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN", "1850");
    let mdg = lookup("KALICO_RUNTIME_MAX_DEGREE", "10");
    let cpn = lookup("KALICO_RUNTIME_CURVE_POOL_N", "16");
    let rss = lookup("KALICO_RUNTIME_STORAGE_SIZE", "327680");  // default = LARGE
```

And modify the body formatting:

```rust
    let body = format!(
        "// Auto-generated by runtime/build.rs — do not edit.\n\
         pub const MAX_CONTROL_POINTS: usize = {mcp};\n\
         pub const MAX_KNOT_VECTOR_LEN: usize = {mkv};\n\
         pub const MAX_DEGREE: u8 = {mdg};\n\
         pub const CURVE_POOL_N: usize = {cpn};\n\
         pub const RT_STORAGE_SIZE: usize = {rss};\n"
    );
```

- [ ] **Step 3: Verify build.rs emits the constant**

```bash
cd rust && cargo build -p runtime --target x86_64-apple-darwin 2>&1 | tail -3
# (Or appropriate host target — `rustc -vV` shows the host triple.)
# Expected: build succeeds.
cat target/*/release/build/runtime-*/out/sizing.rs | grep RT_STORAGE_SIZE
# Expected: `pub const RT_STORAGE_SIZE: usize = 327680;` (default) or whatever Make set
```

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/build.rs
git commit -m "build(runtime): emit RT_STORAGE_SIZE from KALICO_RUNTIME_STORAGE_SIZE env

Reads the new env var (passed by src/Makefile in the prior commit) and
emits pub const RT_STORAGE_SIZE: usize into sizing.rs alongside the
existing per-profile constants. Default of 327680 (LARGE profile) keeps
host-only / sim builds working without going through Klipper's Make."
```

---

### Task 4: Re-export RT_STORAGE_SIZE from runtime crate root

**Files:**
- Modify: `rust/runtime/src/lib.rs`

- [ ] **Step 1: Find where curve_pool's constants are exposed**

```bash
grep -n 'CURVE_POOL_N\|pub use\|pub mod curve_pool' rust/runtime/src/lib.rs
# Expected: shows `pub mod curve_pool;` and possibly `pub use curve_pool::*;`
```

- [ ] **Step 2: Add a `pub use` at the runtime crate root**

In `rust/runtime/src/lib.rs`, locate the existing `pub use curve_pool::...` line (or `pub mod curve_pool;` if not re-exported). Add immediately after:

```rust
pub use curve_pool::{CURVE_POOL_N, MAX_CONTROL_POINTS, MAX_KNOT_VECTOR_LEN, MAX_DEGREE, RT_STORAGE_SIZE};
```

(The exact set may already be partially re-exported — check what's there and add only `RT_STORAGE_SIZE` to whatever exists, OR add a fresh `pub use` if none of these constants are re-exported today.)

- [ ] **Step 3: Verify the re-export compiles**

```bash
cd rust && cargo check -p runtime
# Expected: clean compile.
```

- [ ] **Step 4: Verify the constant is importable from kalico-c-api's crate path**

```bash
cd rust && cat > /tmp/import_test.rs <<'EOF'
use runtime::RT_STORAGE_SIZE;
fn main() { println!("RT_STORAGE_SIZE = {}", RT_STORAGE_SIZE); }
EOF
# Compile-time only check — actual usage lands in Task 7.
```

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/lib.rs
git commit -m "runtime: re-export RT_STORAGE_SIZE at crate root

Makes the constant importable as runtime::RT_STORAGE_SIZE from
kalico-c-api (where the RT_CELL replacement lives). Mirrors the existing
re-export pattern for CURVE_POOL_N et al."
```

---

## Phase 2 — C side

### Task 5: Create runtime_storage.h

**Files:**
- Create: `src/runtime_storage.h`

- [ ] **Step 1: Write the header**

Create `src/runtime_storage.h`:

```c
// Backing storage for the Kalico runtime engine (RuntimeContext).
// Replaces the Rust-side RT_CELL static with #[link_section] —
// per docs/kalico-rewrite/mcu-c-rust-boundary.md rule B2, C owns
// linker-section placement on the MCU.
//
// Storage size flows from Kconfig (RUNTIME_STORAGE_SIZE_LARGE on H7,
// _SMALL on F4) so both this header and Rust's RT_STORAGE_SIZE (emitted
// by rust/runtime/build.rs from the same env var) agree at compile time.
//
// Rust-side const_assert backstops the lower bound:
//   const _: () = assert!(size_of::<RuntimeContext>() <= RT_STORAGE_SIZE);
// C-side _Static_assert in runtime_storage.c backstops AXI overflow.
//
// Spec: docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md.

#ifndef KALICO_RUNTIME_STORAGE_H
#define KALICO_RUNTIME_STORAGE_H

#include "autoconf.h"
#include <stdint.h>

#if CONFIG_RUNTIME_TARGET_LARGE
#  define RT_STORAGE_SIZE CONFIG_RUNTIME_STORAGE_SIZE_LARGE
#elif CONFIG_RUNTIME_TARGET_SMALL
#  define RT_STORAGE_SIZE CONFIG_RUNTIME_STORAGE_SIZE_SMALL
#else
#  error "No CONFIG_RUNTIME_TARGET_* profile selected — pick LARGE or SMALL"
#endif

extern uint8_t rt_storage[RT_STORAGE_SIZE];

#endif // KALICO_RUNTIME_STORAGE_H
```

- [ ] **Step 2: Commit**

```bash
git add src/runtime_storage.h
git commit -m "c(runtime): declare runtime_storage.h header

Selects RT_STORAGE_SIZE from Kconfig profile (LARGE for H7, SMALL for F4)
and exposes the rt_storage extern. Companion .c file lands in next commit."
```

---

### Task 6: Create runtime_storage.c with cfg-gated section attribute

**Files:**
- Create: `src/runtime_storage.c`

- [ ] **Step 1: Write the .c file**

Create `src/runtime_storage.c`:

```c
// rt_storage — backing buffer for the Kalico runtime engine.
//
// Section placement:
//   H7: __attribute__((section(".axi_bss"))) — AXI SRAM at 0x24000000,
//       picked up by the linker rule in src/generic/armcm_link.lds.S
//       (gated #if CONFIG_MACH_STM32H7, targets > axi_ram region).
//   F4: no section attribute — lands in default .bss via the linker's
//       default rule, zeroed by armcm_boot.c::boot_memset.
//
// Alignment: _Alignas(16) is conservative. RuntimeContext contains
// AtomicU64 (8-byte aligned on ARMv7-M) and f32 arrays inside CurvePool;
// 16-byte alignment over-aligns for safety. The Rust side compile-asserts
// align_of::<RuntimeContext>() <= 16 so any future field that requires
// >16-byte alignment fails the build, prompting a bump here.
//
// Size contract: RT_STORAGE_SIZE comes from Kconfig (see
// runtime_storage.h). Rust-side const_assert ensures
// size_of::<RuntimeContext>() <= RT_STORAGE_SIZE; C-side _Static_assert
// below ensures the H7 AXI SRAM doesn't overflow when all .axi_bss
// occupants are summed.
//
// Spec: docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md.

#include "runtime_storage.h"

#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss"), used, externally_visible))
#else
__attribute__((used, externally_visible))
#endif
_Alignas(16) uint8_t rt_storage[RT_STORAGE_SIZE];

// Belt-and-suspenders sanity check: catches a Kconfig misconfiguration
// (e.g., RT_STORAGE_SIZE accidentally set to 0 or 100) before runtime.
// The real lower-bound enforcement is the Rust-side const_assert.
_Static_assert(RT_STORAGE_SIZE >= 1024,
               "RT_STORAGE_SIZE absurdly small — Kconfig profile broken");

// H7-only AXI SRAM overflow assert. Sum the bytes claimed by all .axi_bss
// occupants and verify the total fits with headroom in the 320 KB AXI
// region. Update this list when adding new .axi_bss statics.
//
// Other .axi_bss occupants on H7 today:
//   - kalico_buf (src/kalico_demux.c, ~2 KB)
//   - runtime_bench_samples_buf (src/generic/runtime_bench.c, ~4 KB — verify at impl time)
//   - receive_buf (src/generic/serial_irq.c, RX_BUFFER_SIZE = 2 KB)
//
// (Segment SPSC queue is NOT in .axi_bss — lives in DTCM/regular .bss
// deliberately, per kalico_segment_queue.c:31-40.)
#if CONFIG_MACH_STM32H7
#define AXI_BSS_KALICO_BUF_BYTES        2048
#define AXI_BSS_RUNTIME_BENCH_BYTES     4096   // TODO measure exact value
#define AXI_BSS_SERIAL_IRQ_RX_BYTES     2048   // RX_BUFFER_SIZE in serial_irq.c
#define AXI_BSS_HEADROOM                16384  // 16 KB margin
#define AXI_SRAM_SIZE                   (320 * 1024)

_Static_assert(
    RT_STORAGE_SIZE
        + AXI_BSS_KALICO_BUF_BYTES
        + AXI_BSS_RUNTIME_BENCH_BYTES
        + AXI_BSS_SERIAL_IRQ_RX_BYTES
        + AXI_BSS_HEADROOM
        <= AXI_SRAM_SIZE,
    "AXI SRAM overflow: RT_STORAGE_SIZE too large for AXI region "
    "(after summing other .axi_bss occupants + headroom)"
);
#endif // CONFIG_MACH_STM32H7
```

- [ ] **Step 2: Resolve the AXI_BSS_RUNTIME_BENCH_BYTES TODO**

```bash
grep -B2 -A2 'runtime_bench_samples_buf' src/generic/runtime_bench.c
# Look for sizeof / array dimension; replace the TODO comment and value.
```

If exact byte count differs from 4096, update the `#define` and remove the TODO comment.

- [ ] **Step 3: Commit**

```bash
git add src/runtime_storage.c
git commit -m "c(runtime): declare rt_storage with cfg-gated section attribute

H7 places rt_storage in .axi_bss (AXI SRAM at 0x24000000) via the
existing linker rule; F4 falls through to default .bss. _Alignas(16)
over-aligns conservatively for AtomicU64 + f32 fields inside
RuntimeContext.

Two _Static_asserts: (1) sanity-check RT_STORAGE_SIZE is non-absurd;
(2) H7-only AXI SRAM overflow gate summing all .axi_bss occupants
with 16 KB headroom against the 320 KB region. The Rust-side
const_assert (landing in the kalico-c-api migration commit) backstops
the size_of::<RuntimeContext>() <= RT_STORAGE_SIZE lower bound."
```

---

### Task 7: Add runtime_storage.c to the Makefile

**Files:**
- Modify: `src/Makefile`

- [ ] **Step 1: Locate the CONFIG_KALICO_RUNTIME src-y block**

```bash
grep -n 'src-\$(CONFIG_KALICO_RUNTIME)' src/Makefile
# Expected: a line around 6-7.
```

- [ ] **Step 2: Add runtime_storage.c to the existing line**

Edit the line to append `runtime_storage.c`:

```makefile
src-$(CONFIG_KALICO_RUNTIME) += kalico_demux.c kalico_dispatch.c \
    kalico_segment_queue.c runtime_storage.c
```

- [ ] **Step 3: Verify the C compiles**

(Cannot fully verify until rust side is migrated — rt_storage has no users yet. But the C file should compile standalone.)

```bash
# H7 build:
make KCONFIG_CONFIG=.config.h7.last olddefconfig
make -j$(sysctl -n hw.ncpu) 2>&1 | tail -20
# Expected: build progresses past the runtime_storage.c compile step.
# Final link will fail because Rust still references RT_CELL — that's
# expected at this stage; the C side is independent.
```

If build fails at runtime_storage.c compile (rather than link), debug.

- [ ] **Step 4: Commit**

```bash
git add src/Makefile
git commit -m "make(runtime): wire runtime_storage.c into the H7+F4 build

Appends runtime_storage.c to the CONFIG_KALICO_RUNTIME src list alongside
kalico_segment_queue.c et al. The Rust-side RT_CELL replacement
(landing in subsequent commits) imports rt_storage via extern \"C\"."
```

---

## Phase 3 — Rust migration

### Task 8: Replace RT_CELL with extern UnsafeCell import

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (lines ~22-69)

- [ ] **Step 1: Read the current RT_CELL block**

```bash
sed -n '40,75p' rust/kalico-c-api/src/runtime_ffi.rs
```

Identify:
- `RuntimeCell` struct (lines 55-56 or thereabouts)
- `unsafe impl Sync for RuntimeCell` (line ~61)
- `RT_CELL` static with `#[cfg_attr(feature = "axi-bss-placement", ...)]` (lines 68-69)

- [ ] **Step 2: Replace the RT_CELL block with the extern UnsafeCell import + const_asserts**

In `rust/kalico-c-api/src/runtime_ffi.rs`, replace lines that today read:

```rust
    pub(super) struct RuntimeCell(UnsafeCell<MaybeUninit<RuntimeContext>>);
    // SAFETY: synchronization is done externally via `INIT_DONE` ...
    unsafe impl Sync for RuntimeCell {}

    // Placed in AXI SRAM on H7 via the linker script (.axi_bss section).
    // The 282 KB RuntimeContext doesn't fit in the H723's 128 KB DTCM
    // region, but the H7 has 320 KB of AXI SRAM at 0x24000000 that is
    // unused by the rest of Klipper. Other targets (host / linux / non-H7
    // MCUs) ignore the section name and the static lands in regular .bss.
    #[cfg_attr(feature = "axi-bss-placement", unsafe(link_section = ".axi_bss"))]
    pub(super) static RT_CELL: RuntimeCell = RuntimeCell(UnsafeCell::new(MaybeUninit::uninit()));
```

With:

```rust
    // rt_storage — backing buffer for RuntimeContext, declared on the C
    // side (src/runtime_storage.c) per docs/kalico-rewrite/mcu-c-rust-boundary.md
    // rule B2 (C owns linker-section placement on the MCU).
    //
    // The UnsafeCell wrapper is layout-compatible with the C-side
    // `uint8_t rt_storage[RT_STORAGE_SIZE]`; it exists purely to grant
    // interior-mutability rights to pointers derived from it via .get().
    // No shared `&` reference to rt_storage is ever formed by Rust —
    // the only access path is rt_storage.get().cast::<RuntimeContext>()
    // in runtime_handle_create, after which all writes flow through the
    // existing half-split addr_of_mut! projection chain in
    // RuntimeContext::init.
    //
    // RT_STORAGE_SIZE comes from runtime::RT_STORAGE_SIZE (emitted by
    // runtime/build.rs from KALICO_RUNTIME_STORAGE_SIZE env, set by
    // src/Makefile from Kconfig).
    use core::cell::UnsafeCell;
    use runtime::RT_STORAGE_SIZE;

    unsafe extern "C" {
        static rt_storage: UnsafeCell<[u8; RT_STORAGE_SIZE]>;
    }

    // Compile-time size contract: RuntimeContext must fit in rt_storage.
    // Bump CONFIG_RUNTIME_STORAGE_SIZE_LARGE/_SMALL in src/Kconfig if this
    // fails after a RuntimeContext field addition.
    const _: () = {
        assert!(
            core::mem::size_of::<RuntimeContext>() <= RT_STORAGE_SIZE,
            "RuntimeContext outgrew RT_STORAGE_SIZE — bump Kconfig storage size"
        );
    };

    // Compile-time alignment contract: rt_storage is _Alignas(16) on the
    // C side; RuntimeContext's alignment must not exceed that. If this
    // fails, bump the _Alignas value in src/runtime_storage.c.
    const _: () = {
        assert!(
            core::mem::align_of::<RuntimeContext>() <= 16,
            "RuntimeContext alignment > 16 — bump _Alignas in runtime_storage.c"
        );
    };
```

- [ ] **Step 3: Verify the file still parses**

```bash
cd rust && cargo check -p kalico-c-api 2>&1 | head -20
# Expected: errors about RT_CELL usage (next task fixes the cast site).
# Specifically, runtime_handle_create still references RT_CELL.
```

- [ ] **Step 4: Stage the change but DO NOT commit yet — Task 9 finishes the migration in the same commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs
# DO NOT commit. Task 9 changes runtime_handle_create's cast in the same commit.
```

---

### Task 9: Update runtime_handle_create cast

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (around line 119, inside `runtime_handle_create`)

- [ ] **Step 1: Find the current cast**

```bash
grep -n 'RT_CELL.0.get\|as_mut_ptr' rust/kalico-c-api/src/runtime_ffi.rs
# Expected: a line around 119.
```

- [ ] **Step 2: Replace the cast**

Find this code in `runtime_handle_create`:

```rust
        // SAFETY: single-threaded init; no other context can observe RT_CELL
        // until INIT_DONE is published below. RuntimeContext::init writes
        // through raw-pointer projections and never forms `&mut
        // RuntimeContext`, matching the §11.2 aliasing discipline.
        unsafe {
            let rt_ptr: *mut RuntimeContext = (*RT_CELL.0.get()).as_mut_ptr();
            RuntimeContext::init(rt_ptr);
```

Replace with:

```rust
        // SAFETY: single-threaded init; no other context can observe
        // rt_storage until INIT_DONE is published below. RuntimeContext::init
        // writes through raw-pointer projections and never forms `&mut
        // RuntimeContext`, matching the §11.2 aliasing discipline.
        //
        // rt_storage.get() returns *mut [u8; N] with provenance over the
        // full C-declared buffer; the cast to *mut RuntimeContext inherits
        // that provenance (the const_assert above ensures RuntimeContext
        // fits within the buffer).
        unsafe {
            let rt_ptr: *mut RuntimeContext = rt_storage.get().cast::<RuntimeContext>();
            debug_assert_eq!(
                (rt_ptr as usize) % core::mem::align_of::<RuntimeContext>(),
                0,
                "rt_storage alignment mismatch — linker placed it unaligned"
            );
            RuntimeContext::init(rt_ptr);
```

- [ ] **Step 3: Verify the file compiles (cargo check)**

```bash
cd rust && cargo check -p kalico-c-api 2>&1 | head -30
# Expected: cargo check passes for host target. For MCU target, the
# `axi-bss-placement` feature flag is still referenced in Cargo.toml —
# that's removed in Task 10.
```

If `cargo check` reports a missing import or type error, fix inline before continuing.

- [ ] **Step 4: Stage the change (still no commit; Task 10 cleans up Cargo.toml in the same commit)**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs
# Note: file is already staged from Task 8. This is an incremental update.
```

---

### Task 10: Remove axi-bss-placement Cargo feature

**Files:**
- Modify: `rust/kalico-c-api/Cargo.toml`

- [ ] **Step 1: Read the current features block**

```bash
grep -B1 -A2 'axi-bss-placement\|^mcu-h7' rust/kalico-c-api/Cargo.toml
```

Expected output shows:
- `mcu-h7 = ["nurbs/mcu-h7", "runtime/mcu-h7", "axi-bss-placement"]`
- `axi-bss-placement = []`
- Comments explaining the feature

- [ ] **Step 2: Remove the feature**

Modify `rust/kalico-c-api/Cargo.toml`:

1. Remove `"axi-bss-placement"` from the `mcu-h7` feature list:

```toml
# BEFORE:
mcu-h7 = ["nurbs/mcu-h7", "runtime/mcu-h7", "axi-bss-placement"]
# AFTER:
mcu-h7 = ["nurbs/mcu-h7", "runtime/mcu-h7"]
```

2. Delete the `axi-bss-placement = []` line and its preceding comment block. The comment likely reads:

```toml
# Place RT_CELL in `.axi_bss` (mapped to AXI SRAM on H7). Selected only on
# the H7 MCU build where the linker script declares `.axi_bss`. On other
# targets the section attribute is omitted via cfg_attr and RT_CELL lands
# in default bss.
axi-bss-placement = []
```

Delete all of those lines.

- [ ] **Step 3: Verify cargo recognizes the change**

```bash
cd rust && cargo check -p kalico-c-api --features mcu-h7 --no-default-features 2>&1 | head -10
# Expected: clean compile (no warning about unknown feature, no error).
cd rust && cargo check -p kalico-c-api --features mcu-f4 --no-default-features 2>&1 | head -10
# Expected: clean compile.
```

- [ ] **Step 4: Commit (the Phase 3 migration as one commit)**

```bash
git add rust/kalico-c-api/Cargo.toml rust/kalico-c-api/src/runtime_ffi.rs
git commit -m "runtime(mcu): migrate RT_CELL to C-declared rt_storage

Replaces the Rust-side RT_CELL static (with #[link_section(\".axi_bss\")])
with an extern \"C\" import of the C-declared rt_storage buffer
(src/runtime_storage.c, prior commit). Section placement is now decided
exclusively by the C linker script (rule B2 of mcu-c-rust-boundary.md).

Mechanism:
- C: uint8_t rt_storage[RT_STORAGE_SIZE] with cfg-gated section attribute.
- Rust: extern \"C\" { static rt_storage: UnsafeCell<[u8; N]>; }
  UnsafeCell is layout-compatible with the C array; the wrapper grants
  interior-mutability rights to pointers derived via .get(). No shared
  reference to rt_storage is ever formed by Rust.
- runtime_handle_create casts rt_storage.get() to *mut RuntimeContext;
  RuntimeContext::init's existing addr_of_mut! projection chain proceeds
  identically from the new pointer origin.
- const_asserts on Rust enforce size <= RT_STORAGE_SIZE and align <= 16.
- _Static_assert on C (in runtime_storage.c) enforces AXI overflow gate.

Deleted: RuntimeCell wrapper type, Sync impl on RuntimeCell, the
#[cfg_attr(feature = \"axi-bss-placement\", ...)] attribute, the
axi-bss-placement Cargo feature, the inclusion of that feature in mcu-h7.

The cargo-clean operational tripwire (feedback_cargo_clean_between_mcus.md)
becomes optional after this commit — section placement no longer depends
on Rust's compile artifacts. The unsafe impl Sync for RuntimeContext in
state.rs:536 stays unchanged (still required for the INIT_DONE
synchronization pattern).

Spec: docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md
v2, sections 1 + 3."
```

---

## Phase 4 — Verification scaffolding

### Task 11: Section-placement check script (V3)

**Files:**
- Create: `scripts/check_rt_storage_placement.sh`

- [ ] **Step 1: Write the script**

Create `scripts/check_rt_storage_placement.sh`:

```bash
#!/usr/bin/env bash
# V3 — section-placement gate for rt_storage.
#
# Usage: ./scripts/check_rt_storage_placement.sh out/klipper.elf
#
# On H7 builds: rt_storage must land in `.axi_bss` at an address in
# [0x24000000, 0x24050000).
# On F4 builds: rt_storage must land in `.bss` (or `.bss.*`) at an address
# in [0x20000000, 0x20020000) and must be inside the [_bss_start,_bss_end]
# span (verified via boot-zeroing coverage cross-check).
#
# Codifies the previously-manual `objdump | grep RT_CELL` ritual
# (feedback_cargo_clean_between_mcus.md).

set -euo pipefail

ELF="${1:-out/klipper.elf}"
if [ ! -f "$ELF" ]; then
    echo "ERROR: ELF not found: $ELF" >&2
    exit 1
fi

# Find rt_storage symbol with section name and address.
SYM_LINE="$(objdump -t "$ELF" | awk '/ rt_storage$/{print}')"
if [ -z "$SYM_LINE" ]; then
    echo "ERROR: rt_storage symbol not found in $ELF" >&2
    echo "  (expected to be a C-declared uint8_t array in src/runtime_storage.c)" >&2
    exit 2
fi

# objdump -t format: <addr> <flags> <section> <size> <name>
ADDR_HEX="$(echo "$SYM_LINE" | awk '{print $1}')"
SECTION="$(echo "$SYM_LINE" | awk '{print $(NF-2)}')"
ADDR=$((16#${ADDR_HEX#0x}))

# Detect MCU family from the ELF's section table or symbol presence.
# Heuristic: H7 builds have an _axi_bss_start symbol; F4 builds don't.
if objdump -t "$ELF" | grep -q '_axi_bss_start'; then
    MCU=H7
    EXPECTED_SECTION=".axi_bss"
    MIN_ADDR=$((0x24000000))
    MAX_ADDR=$((0x24050000))
else
    MCU=F4
    EXPECTED_SECTION=".bss"
    MIN_ADDR=$((0x20000000))
    MAX_ADDR=$((0x20020000))
fi

echo "rt_storage: section=$SECTION addr=$ADDR_HEX MCU=$MCU"

if [ "$SECTION" != "$EXPECTED_SECTION" ]; then
    # Allow .bss subsections on F4 (.bss.runtime_storage, etc.)
    if [ "$MCU" = "F4" ] && [[ "$SECTION" == .bss* ]]; then
        :
    else
        echo "ERROR: rt_storage is in section '$SECTION', expected '$EXPECTED_SECTION' on $MCU" >&2
        exit 3
    fi
fi

if [ "$ADDR" -lt "$MIN_ADDR" ] || [ "$ADDR" -ge "$MAX_ADDR" ]; then
    printf "ERROR: rt_storage address 0x%x outside expected range [0x%x, 0x%x) on %s\n" \
        "$ADDR" "$MIN_ADDR" "$MAX_ADDR" "$MCU" >&2
    exit 4
fi

# F4-only: verify rt_storage is inside the boot-zeroed BSS span.
if [ "$MCU" = "F4" ]; then
    BSS_START_HEX="$(objdump -t "$ELF" | awk '/_bss_start$/{print $1; exit}')"
    BSS_END_HEX="$(objdump -t "$ELF" | awk '/_bss_end$/{print $1; exit}')"
    if [ -z "$BSS_START_HEX" ] || [ -z "$BSS_END_HEX" ]; then
        echo "WARNING: could not locate _bss_start/_bss_end symbols; skipping boot-zero coverage check" >&2
    else
        BSS_START=$((16#${BSS_START_HEX#0x}))
        BSS_END=$((16#${BSS_END_HEX#0x}))
        if [ "$ADDR" -lt "$BSS_START" ] || [ "$ADDR" -ge "$BSS_END" ]; then
            printf "ERROR: rt_storage 0x%x outside boot-zeroed [_bss_start=0x%x, _bss_end=0x%x) — orphan section escaped zeroing\n" \
                "$ADDR" "$BSS_START" "$BSS_END" >&2
            exit 5
        fi
    fi
fi

echo "OK: rt_storage placed correctly on $MCU"
```

- [ ] **Step 2: Make it executable**

```bash
chmod +x scripts/check_rt_storage_placement.sh
```

- [ ] **Step 3: Test it against a build (any successful H7 or F4 build with rt_storage will do)**

Skip until after the first successful H7 or F4 build (Phase 6 covers this).

- [ ] **Step 4: Commit**

```bash
git add scripts/check_rt_storage_placement.sh
git commit -m "scripts: V3 section-placement gate for rt_storage

Codifies the previously-manual 'objdump -t | grep RT_CELL' ritual
(feedback_cargo_clean_between_mcus.md) into a script that verifies:
1. rt_storage symbol exists in the ELF.
2. Section name matches expected per MCU (.axi_bss on H7, .bss[.*] on F4).
3. Address is in the expected MCU SRAM range.
4. On F4, address is inside [_bss_start, _bss_end] (catches orphan-section
   escape from boot-zeroing — codex reviewer flagged this risk).

Wired into the build verification phase (Phase 6); intended as a CI gate."
```

---

### Task 12: Stale-header drift gate (V4)

**Files:**
- Create: `rust/kalico-c-api/tests/rt_storage_drift.rs`

- [ ] **Step 1: Write the drift test**

Create `rust/kalico-c-api/tests/rt_storage_drift.rs`:

```rust
//! V4 — stale-header drift gate.
//!
//! Verifies that the Rust-side RT_STORAGE_SIZE (compiled into the
//! staticlib via runtime/build.rs) matches the C-side rt_storage[]
//! declared size at link time. A mismatch means the build saw two
//! different KALICO_RUNTIME_STORAGE_SIZE values — Makefile / Kconfig /
//! cargo-cache drift.
//!
//! This test is host-only (`cfg(not(target_arch = "thumbv7em"))`) because
//! the MCU build doesn't link Rust tests. The test exercises the same
//! const_assert as the FFI shim's; if the staticlib's compile-time
//! constant disagreed with rt_storage[]'s declared size, the firmware
//! link would fail with "size mismatch." We verify the const here so
//! host CI catches the drift before MCU builds attempt.

#![cfg(feature = "host")]

use runtime::RT_STORAGE_SIZE;

#[test]
fn rt_storage_size_matches_runtime_context_bound() {
    // Lower bound: RuntimeContext must fit in RT_STORAGE_SIZE.
    // The kalico-c-api crate's runtime_ffi.rs already enforces this at
    // compile time via const_assert; this test verifies the runtime
    // crate's exposed constant is sane (non-zero, reasonable).
    assert!(
        RT_STORAGE_SIZE >= 32 * 1024,
        "RT_STORAGE_SIZE = {} bytes — implausibly small for RuntimeContext",
        RT_STORAGE_SIZE
    );
    assert!(
        RT_STORAGE_SIZE <= 1024 * 1024,
        "RT_STORAGE_SIZE = {} bytes — implausibly large; check Kconfig",
        RT_STORAGE_SIZE
    );
}

#[test]
fn rt_storage_size_consistent_with_runtime_context_size() {
    use runtime::state::RuntimeContext;
    assert!(
        core::mem::size_of::<RuntimeContext>() <= RT_STORAGE_SIZE,
        "RuntimeContext is {} bytes but RT_STORAGE_SIZE is only {} — \
         bump CONFIG_RUNTIME_STORAGE_SIZE_LARGE/_SMALL in src/Kconfig",
        core::mem::size_of::<RuntimeContext>(),
        RT_STORAGE_SIZE
    );
}
```

- [ ] **Step 2: Verify the test compiles**

```bash
cd rust && cargo test -p kalico-c-api --test rt_storage_drift 2>&1 | tail -10
# Expected: 2 tests pass.
```

If the test fails because `RuntimeContext` isn't accessible at `runtime::state::RuntimeContext` (visibility), either re-export it from the runtime crate or use a `pub` accessor. Fix inline.

- [ ] **Step 3: Commit**

```bash
git add rust/kalico-c-api/tests/rt_storage_drift.rs
git commit -m "test(rt_storage): V4 stale-header drift gate

Host-only test that verifies RT_STORAGE_SIZE is non-implausible and
that RuntimeContext fits within it. Catches Makefile / Kconfig /
cargo-cache drift before MCU build link time.

The const_assert in kalico-c-api/src/runtime_ffi.rs already enforces
the bound at compile time; this test fails earlier (in cargo test
output) for clearer diagnostics."
```

---

### Task 13: Verify const_asserts fire on deliberate failure

**Files:** (no file edits; verification-only)

- [ ] **Step 1: Deliberately oversize RuntimeContext to verify the Rust const_assert fires**

Create a throwaway branch from current HEAD:

```bash
git checkout -b throwaway/verify-const-assert
```

In `rust/runtime/src/state.rs`, add a giant array field to `RuntimeContext`:

```rust
pub struct RuntimeContext {
    // ... existing fields ...
    pub _verify_const_assert: [u8; 1024 * 1024],  // 1 MB — deliberately oversize
}
```

- [ ] **Step 2: Build and confirm const_assert fires**

```bash
cd rust && cargo check -p kalico-c-api --features mcu-h7 --no-default-features 2>&1 | grep -A2 'RuntimeContext outgrew'
# Expected: the const_assert message appears.
```

- [ ] **Step 3: Deliberately undersize RT_STORAGE_SIZE via env var**

```bash
cd rust && KALICO_RUNTIME_STORAGE_SIZE=100 cargo check -p kalico-c-api --features mcu-h7 --no-default-features 2>&1 | grep -A2 'RuntimeContext outgrew'
# Expected: const_assert fires.
```

- [ ] **Step 4: Deliberately bump CONFIG_RUNTIME_STORAGE_SIZE_LARGE past AXI capacity to verify C _Static_assert fires**

```bash
# Modify .config.h7.last (or a copy) to set CONFIG_RUNTIME_STORAGE_SIZE_LARGE=400000
sed -i.bak 's/CONFIG_RUNTIME_STORAGE_SIZE_LARGE=.*/CONFIG_RUNTIME_STORAGE_SIZE_LARGE=400000/' .config.h7.test
KCONFIG_CONFIG=.config.h7.test make 2>&1 | grep -A2 'AXI SRAM overflow'
# Expected: _Static_assert fires at runtime_storage.c compile.
```

- [ ] **Step 5: Discard the throwaway branch**

```bash
git checkout mcu-boundary-refactor
git branch -D throwaway/verify-const-assert
```

- [ ] **Step 6: Document verification outcome in the audit findings doc (Task 14 creates it; come back to add notes if doing strict sequencing)**

No commit for this task — it's verification only. Notes go into Phase 5's audit-findings doc (Task 14).

---

## Phase 5 — Audit pass

### Task 14: Create audit findings doc

**Files:**
- Create: `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md`

- [ ] **Step 1: Create the directory if needed**

```bash
mkdir -p docs/superpowers/audits
```

- [ ] **Step 2: Write the initial structure**

Create `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md`:

```markdown
# MCU C/Rust Boundary Audit Findings — 2026-05-19

> Findings doc for audit items A1–A8 from
> `docs/superpowers/specs/2026-05-19-mcu-c-rust-boundary-refactor-design.md`.
> One section per audit item. Each section records: discovery commands run,
> findings, decisions, and follow-up actions.

## A1 — Inventory items

### A1.1 — Rust `#[link_section]` uses

Command: `rg -n '#\[link_section\|link_section\b' rust/`
Expected: only RT_CELL appears (the migration target).
Result: TBD — run after Phase 3 migration completes.
Decision: TBD.

### A1.2 — `.axi_bss` C occupant inventory

Files inspected:
- `src/kalico_demux.c` — `kalico_buf` (size: TBD bytes, measured by `objdump`)
- `src/generic/runtime_bench.c` — `runtime_bench_samples_buf` (size: TBD)
- `src/generic/serial_irq.c` — `receive_buf` (size: 2048 = RX_BUFFER_SIZE)
- `src/runtime_storage.c` — `rt_storage` (size: RT_STORAGE_SIZE per profile)

Update planned: boundary doc § "What lives where on the MCU" `.axi_bss` row gets exact byte counts after measurement.

### A1.3 — `extern "C"` signature sweep

Targets:
- `rust/kalico-c-api/src/runtime_ffi.rs` (~82 functions)

Patterns to grep for:
- `&[` or `&mut [` → slices crossing the ABI (forbidden by B3/B4)
- `Option<&` → fat pointer cases
- `(.*,.*)` → tuple returns (forbidden)

Result: TBD.
Decision: TBD.

## A2 — `bool`-FFI policy ratification

Discovery:
- `runtime_ffi.rs:2145, 2234, 2400, 2985` return `bool` today.
- Boundary doc B4 says "no `bool` (use `uint8_t`)" — contradicts code.

Decision: **Accept `bool`.** Rust's `bool` is layout-compatible with C99 `_Bool` (both 1 byte, both 0/1). Update boundary doc B4 to permit it.

Action: B4 rewording lands in the doc-reconciliation phase (Task 23).

## A3 — `kalico_producer_current_present` static-mut import

Discovery commands:
```
rg -n 'kalico_producer_current_present' rust/
```

Look for: direct reads of the bare `static mut` (other than through accessor functions).

Result: TBD.
Decision: TBD. If direct reads exist, route through C accessor function. If none, delete the `static mut` import.

## A4 — `portable_atomic::AtomicU64` ordering in ISR hot path

Discovery commands:
```
rg -n 'AtomicU64.*fetch_add\|AtomicU64.*Ordering' rust/runtime/src/state.rs
rg -n 'AtomicU64' rust/runtime/src/
```

Identify: which `AtomicU64` operations are called from the TIM5 ISR path (engine.rs `tick`, `runtime_handle_tick`).

Critical-section cost: on thumbv7em-none-eabihf, `portable_atomic::AtomicU64` fallback uses interrupt-disable for the duration of the op. Quantify the typical operation count per ISR tick.

Result: TBD.
Decision: Audit-only this refactor. If cost is unacceptable, a follow-up sub-spec captures the split-into-AtomicU32-pair work. Locked per spec § Non-goals.

## A5 — Panic-in-ISR audit

Discovery:
- Find the MCU panic handler in `rust/kalico-c-api/src/` (likely `lib.rs` or a panic-handler-* crate).
- Identify reachable panics from `runtime_handle_tick` and related ISR-called FFI entry points.

C fault-latch entry point candidates:
- `fault_handler_report_task` (per recent commits about `diag: emit sched_bad_add state via fault_handler_report_task`)

Result: TBD.
Decision: TBD. If panics reachable in ISR context, route through C fault-latch rather than the current spin-forever handler.

## A6 — FPU / CPACR consistency

Discovery:
- Confirm C build flags include hard-float ABI:
  - F4: `-mfloat-abi=hard -mfpu=fpv4-sp-d16`
  - H7: `-mfloat-abi=hard -mfpu=fpv5-d16`
- Confirm CPACR enabled at boot per the 2026-05-12 fix (`armcm_main` when `__FPU_PRESENT == 1`).
- Confirm Rust staticlib build target is `thumbv7em-none-eabi` (NOT `thumbv7em-none-eabihf`, OR explicit `-C target-feature=+vfp4` flag).

Commands:
```
grep -rn '\-mfloat-abi\|\-mfpu\|FPU_PRESENT' src/stm32/Makefile src/generic/armcm_*.c
grep -rn 'target = "thumbv7em\|target-feature' rust/.cargo/config.toml rust/runtime/Cargo.toml
```

Result: TBD.
Decision: TBD.

## A7 — DMA cacheability for H7 `.axi_bss`

Investigation:
- Is `RuntimeContext` ever DMA source/destination? (Expected: no — DMA goes through dedicated USB/UART buffers.)
- For each `.axi_bss` C occupant (`kalico_buf`, `runtime_bench_samples_buf`, `receive_buf`), is it DMA-touched?
- If yes, is cache maintenance performed correctly today?
- Is `.axi_bss` MPU-marked non-cacheable on H7?

Commands:
```
rg -n 'DMA\|dma_' src/kalico_demux.c src/generic/serial_irq.c src/generic/runtime_bench.c
rg -n 'MPU\|mpu_protect' src/generic/mpu_protect.c src/stm32/
```

Result: TBD.
Decision: Per § Non-goals: no code changes; document findings only.

## A8 — Dead-code removal

### A8.1 — `runtime_irq_save` / `runtime_irq_restore`

Discovery commands:
```
rg -n 'runtime_irq_save\|runtime_irq_restore' rust/ src/
```

Result: TBD. If unused everywhere, delete the declarations.

### A8.2 — Other dead-code candidates

Discovery: `cargo +nightly udeps` (if available) or manual grep for `#[allow(dead_code)]` in the FFI surface.

Result: TBD.
```

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md
git commit -m "audit(boundary): scaffold A1-A8 findings doc

One section per audit item from the refactor spec. Discovery commands
pre-populated; results and decisions filled in as each audit task runs.
Acts as the single source of truth for what the audit found and what
we did about it."
```

---

### Task 15: A1 inventory — Rust link_section + .axi_bss occupants + signature sweep

**Files:**
- Modify: `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md` (fill in A1 results)

- [ ] **Step 1: Run the link_section grep**

```bash
rg -n '#\[link_section\|link_section\b' rust/
# Expected after Phase 3: no results except maybe in comments/docs.
```

If any active `#[link_section]` attribute remains, debug — Phase 3 didn't fully clean up. Fix and re-commit Phase 3 work.

- [ ] **Step 2: Measure .axi_bss occupant byte sizes**

After a successful H7 build:

```bash
objdump -t out/klipper.elf | grep ' \.axi_bss\| _axi_bss' | sort
# Look for: rt_storage, kalico_buf, runtime_bench_samples_buf, receive_buf
# Each symbol has a size column.
```

Record exact byte counts in A1.2 section of the audit findings doc.

- [ ] **Step 3: Run the signature sweep**

```bash
# Slices crossing the ABI:
rg -n 'extern "C".*&\[|extern "C".*&mut \[' rust/kalico-c-api/src/runtime_ffi.rs
# Tuple returns:
rg -n 'pub.*extern "C".*->.*\(.*,.*\)' rust/kalico-c-api/src/runtime_ffi.rs
# Option<&T>:
rg -n 'extern "C".*Option<&' rust/kalico-c-api/src/runtime_ffi.rs
```

Expected: no results for any of these (the opaque-handle API uses `*mut KalicoRuntime` and primitive types).

If any results appear, document in A1.3 and decide per-signature: refactor to pointer+length, or accept with documented exception.

- [ ] **Step 4: Update the audit findings doc**

Fill in A1.1, A1.2, A1.3 in `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md` with the actual command outputs and decisions.

- [ ] **Step 5: Commit**

```bash
git add docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md
git commit -m "audit(A1): inventory findings — link_section, .axi_bss occupants, signatures

Records Phase 3 migration left zero active #[link_section] attributes
in Rust, the four .axi_bss occupants with measured byte sizes, and the
extern \"C\" signature sweep results. Decisions: [fill in based on
actual findings during audit run]."
```

---

### Task 16: A3 — Route `kalico_producer_current_present` reads through accessor

**Files:**
- Modify: `rust/runtime/src/engine.rs` (around line 22-34, the `static mut` import)
- Modify: `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md` (fill in A3)

- [ ] **Step 1: Find all reads of `kalico_producer_current_present` in Rust**

```bash
rg -n 'kalico_producer_current_present' rust/
```

- [ ] **Step 2: Classify each reference**

For each match:
- Through a C accessor function (e.g., `kalico_producer_current_present_load()`): safe.
- Direct read of the `static mut` (e.g., `unsafe { kalico_producer_current_present }`): unsafe — must route through accessor.

- [ ] **Step 3: For each direct read, route through C accessor**

If the C side doesn't expose an accessor, add one in `src/kalico_segment_queue.c`:

```c
// Accessor — single read path for kalico_producer_current_present.
// Rust must NOT import the bare static; cross-language `static mut`
// imports carry the same LLVM-miscompilation risk that motivated the
// 2026-05-18 queue move.
uint8_t
kalico_producer_current_present_load(void)
{
    return atomic_load_explicit(&kalico_producer_current_present,
                                memory_order_acquire);
}
```

Header (`src/kalico_segment_queue.h`):

```c
uint8_t kalico_producer_current_present_load(void);
```

On the Rust side, replace direct `static mut` reads with:

```rust
unsafe extern "C" {
    fn kalico_producer_current_present_load() -> u8;
}
// Caller: let present = unsafe { kalico_producer_current_present_load() };
```

If no direct reads exist (only the bare `static mut` declaration with no users), delete the declaration.

- [ ] **Step 4: Run cargo test to verify nothing broke**

```bash
cd rust && cargo test -p runtime 2>&1 | tail -5
# Expected: tests pass.
```

- [ ] **Step 5: Update audit doc + commit**

Update A3 section with: number of direct reads found, the accessor change (or deletion), test outcome.

```bash
git add rust/runtime/src/engine.rs docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md
# Add src/kalico_segment_queue.{c,h} if accessor added
git commit -m "audit(A3): route producer_current_present through C accessor

[Or: 'audit(A3): delete unused producer_current_present static mut import'
 if no direct reads existed.]

Same class of fix as the 2026-05-18 SPSC queue migration: a Rust-side
`static mut` imported from C carries the LLVM-miscompilation risk that
bit the Consumer pattern. The accessor function path makes the read
non-projection-based and debugger-friendly."
```

---

### Task 17: A2 — Update boundary doc B4 to permit `bool`

**Files:**
- Modify: `docs/kalico-rewrite/mcu-c-rust-boundary.md` (B4 section)
- Modify: `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md` (A2 closed)

- [ ] **Step 1: Find the current B4 wording**

```bash
grep -n 'B4.*extern "C"\|no .bool.\|bool.*uint8_t' docs/kalico-rewrite/mcu-c-rust-boundary.md
```

- [ ] **Step 2: Update B4**

In `docs/kalico-rewrite/mcu-c-rust-boundary.md`, find B4 and replace the `bool` clause:

```markdown
### B4. `extern "C"` + `#[repr(C)]` everywhere across the boundary

Any function visible across the boundary is `extern "C"` and listed in a header. Any struct visible across the boundary is `#[repr(C)]` on the Rust side and defined in a C header on the C side. No `#[repr(Rust)]` types, no `enum`-with-payloads, no `Option<&T>` in signatures, no zero-sized types. Slices cross as pointer + length.

`bool` is permitted. Rust's `bool` and C99 `_Bool` are layout-compatible (both 1 byte, both 0/1); the C side must `#include <stdbool.h>` to consume it. The audit pass on 2026-05-19 (A2 finding) ratified this against existing `runtime_handle_*` accessor sites that already return `bool`.
```

- [ ] **Step 3: Update audit doc — mark A2 closed**

```bash
# In audit findings doc, change A2 from "Decision: Accept bool. Action: B4 rewording lands in the doc-reconciliation phase (Task 23)."
# to: "Action: B4 rewording landed in commit <will-fill>."
```

- [ ] **Step 4: Commit**

```bash
git add docs/kalico-rewrite/mcu-c-rust-boundary.md docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md
git commit -m "audit(A2): permit bool in FFI signatures (B4 doc update)

Rust bool and C99 _Bool are layout-compatible (1 byte, 0/1).
runtime_handle_*_bool accessor sites (runtime_ffi.rs:2145, 2234, 2400,
2985) already use bool returns; updating B4 to ratify this rather than
the inverse migration. C consumers must #include <stdbool.h>."
```

---

### Task 18: A5 — Panic-in-ISR audit + fix if needed

**Files:**
- Modify: `rust/kalico-c-api/src/lib.rs` (or wherever the MCU panic handler is)
- Modify: `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md` (A5)

- [ ] **Step 1: Locate the MCU panic handler**

```bash
rg -n '#\[panic_handler\]' rust/
# Expected: one location, likely rust/kalico-c-api/src/lib.rs or a panic-handler module.
```

- [ ] **Step 2: Read it and identify the behavior**

```bash
# Read the function body. Look for `loop {}` (spin-forever), or a fault-latch call.
```

- [ ] **Step 3: Map panics reachable from ISR-called FFI entry points**

The ISR-called entry points are listed in `runtime_ffi.rs`: `runtime_handle_tick`, `kalico_runtime_modulated_tick`, and any kalico_endstop functions called from interrupt contexts.

```bash
rg -n 'panic!\|unwrap()\|expect("' rust/runtime/src/engine.rs rust/runtime/src/state.rs rust/runtime/src/curve_pool.rs | head -30
```

Each match is a potential panic site. Evaluate which are reachable from the ISR-called entry points (by call-graph reasoning).

- [ ] **Step 4: If panic sites are reachable from ISR, route the panic handler through fault_handler_report_task**

Modify the panic handler. Concrete example:

```rust
use core::panic::PanicInfo;

unsafe extern "C" {
    // From src/generic/fault_handler.c — latches a fault state and
    // schedules a shutdown report.
    fn fault_handler_report_task(reason: u32);
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Route through C fault-latch rather than spinning. The C side
    // handles the IWDG bite and the shutdown-report frame; spinning
    // here would prevent both.
    const RUST_PANIC_FAULT_CODE: u32 = 0xDEAD_C0DE;  // pick a sentinel
    unsafe { fault_handler_report_task(RUST_PANIC_FAULT_CODE); }
    // Unreachable but required for !-return.
    loop { core::hint::spin_loop(); }
}
```

(The exact fault-code sentinel and the fault_handler_report_task signature need to match what `src/generic/fault_handler.c` exports. Verify before committing.)

- [ ] **Step 5: Test by deliberately panicking in a host test and confirming the ISR path is not taken (host tests don't have ISR context, but the panic-handler change should still compile and not regress host behavior)**

```bash
cd rust && cargo test -p kalico-c-api 2>&1 | tail -10
# Expected: existing tests still pass.
```

- [ ] **Step 6: Update audit doc + commit**

```bash
git add rust/kalico-c-api/src/lib.rs docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md
git commit -m "audit(A5): route panic-in-ISR through fault_handler_report_task

[OR: 'audit(A5): no panic sites reachable from ISR-called FFI entries']

Previously the Rust panic handler spun forever. From the TIM5 ISR or
stepper-timer callback contexts this would lock inside an interrupt,
preventing IWDG service and shutdown-report frame emission. Routing
through the C fault-latch path closes that hole — C handles the bite
and the report; Rust just signals the fault code.

Reviewer (codex) flagged the spin-forever as a structural gap in v1
of the spec."
```

---

### Task 19: A4 + A6 + A7 — Audit notes (no code changes)

**Files:**
- Modify: `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md` (A4, A6, A7)

- [ ] **Step 1: A4 — AtomicU64 audit**

```bash
rg -n 'AtomicU64\|portable_atomic::AtomicU64' rust/runtime/src/state.rs rust/runtime/src/engine.rs
```

For each match, classify: ISR-path or foreground-only. Count fetch_add operations in `runtime_handle_tick` (and its transitive callees).

Document in A4 section: list of ISR-path AtomicU64 ops, cost estimate (critical-section-disable for ~10-50 cycles each on thumbv7em), decision: audit-only this refactor. If a follow-up sub-spec is warranted, name it.

- [ ] **Step 2: A6 — FPU/CPACR consistency**

```bash
# Check C build flags:
grep -rn '\-mfloat-abi\|\-mfpu' src/stm32/Makefile src/generic/Makefile 2>/dev/null | head
# Check Rust target:
cat rust/.cargo/config.toml 2>/dev/null | head -30
# Check CPACR-enable code:
grep -B2 -A10 'CPACR\|__FPU_PRESENT' src/generic/armcm_main.c src/generic/armcm_boot.c 2>/dev/null
```

Document in A6: actual C flags per MCU, actual Rust target triple + `-C target-feature`, presence of CPACR-enable code per `project_f446_configure_axes_crash.md` resolution. Verify all four agree.

- [ ] **Step 3: A7 — DMA cacheability**

```bash
rg -n 'DMA\|HAL_DMA\|dma_\|cache_clean\|cache_invalidate\|SCB_\|MPU_\|MPU->\|mpu_protect' src/kalico_demux.c src/generic/serial_irq.c src/generic/runtime_bench.c src/generic/mpu_protect.c 2>/dev/null | head -30
```

For each `.axi_bss` occupant, classify: DMA-touched (with cache maintenance details) or not.

For MPU: is `.axi_bss` MPU-marked non-cacheable on H7?

Document in A7. Per spec § Non-goals, no code changes — only document.

- [ ] **Step 4: Commit**

```bash
git add docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md
git commit -m "audit(A4+A6+A7): AtomicU64 / FPU-ABI / DMA-cacheability findings

A4: [N] AtomicU64 fetch_add ops on the ISR hot path, each uses
critical-section disable on portable_atomic's thumbv7em fallback.
Decision: audit-only; follow-up sub-spec if performance audit demands
splits. (Per spec § Non-goals, locked.)

A6: Confirmed hard-float ABI on both MCU profiles, Rust staticlib
built with [target setting], CPACR-enable in armcm_main present per
project_f446_configure_axes_crash.md resolution. [or report any drift]

A7: [N of M] .axi_bss occupants are DMA-touched. [Cache maintenance
status documented; no code changes per spec § Non-goals.]"
```

---

### Task 20: A8 — Dead-code removal

**Files:**
- Modify: `rust/runtime/src/state.rs` (lines 103-106, `runtime_irq_save`/`runtime_irq_restore` declarations)
- Modify: `docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md` (A8)

- [ ] **Step 1: Verify the functions are actually unused**

```bash
rg -n 'runtime_irq_save\|runtime_irq_restore' rust/ src/
```

Check each match: is it a declaration (`unsafe extern "C" fn ...`), a definition (`pub extern "C" fn ...`), a caller, or a comment/doc?

- [ ] **Step 2: If unused, delete the declarations**

In `rust/runtime/src/state.rs` (lines ~103-106 currently `#[allow(dead_code)]`):

```rust
// BEFORE:
#[allow(dead_code)]
unsafe extern "C" {
    fn runtime_irq_save() -> u32;
    fn runtime_irq_restore(flags: u32);
}

// AFTER: (delete the entire block)
```

Also check `rust/kalico-c-api/tests/*` for test-only definitions; those stay (they're test fixtures).

- [ ] **Step 3: Run cargo check**

```bash
cd rust && cargo check -p runtime 2>&1 | tail -5
# Expected: clean compile.
cd rust && cargo check -p kalico-c-api 2>&1 | tail -5
# Expected: clean compile.
```

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/state.rs docs/superpowers/audits/2026-05-19-mcu-boundary-audit-findings.md
git commit -m "audit(A8): remove dead runtime_irq_save/restore declarations

Grep confirmed no Rust caller (only the #[allow(dead_code)] stub in
state.rs and the test fixtures, which stay). The C side may still
define them for future use; dropping the Rust import shrinks the FFI
surface and removes the dead-code allowance."
```

(If the functions are still used in production code, this task becomes "remove the `#[allow(dead_code)]` attribute and document the caller path" instead. Decide based on grep output.)

---

## Phase 6 — Verification execution

### Task 21: V1 — Build gates on both MCU profiles

**Files:** (no edits; verification-only)

- [ ] **Step 1: Clean tree (per cargo-clean discipline)**

```bash
make clean
cd rust && cargo clean
cd ..
```

- [ ] **Step 2: H7 build**

```bash
cp .config.h7.last .config
make olddefconfig
make -j$(sysctl -n hw.ncpu) 2>&1 | tail -30
# Expected: build succeeds; rt_storage symbol appears in the ELF.
```

- [ ] **Step 3: Verify `axi-bss-placement` feature is GONE from the cargo invocation**

```bash
make -n 2>&1 | grep 'axi-bss-placement'
# Expected: no output (feature is no longer in the build).
```

- [ ] **Step 4: F4 build**

```bash
make clean
cd rust && cargo clean
cd ..
cp .config.f446.last .config
make olddefconfig
make -j$(sysctl -n hw.ncpu) 2>&1 | tail -30
# Expected: build succeeds.
```

- [ ] **Step 5: cargo check per MCU profile (host-side verification)**

```bash
cd rust && cargo check -p kalico-c-api --features mcu-h7 --no-default-features 2>&1 | tail -5
# Expected: clean.
cd rust && cargo check -p kalico-c-api --features mcu-f4 --no-default-features 2>&1 | tail -5
# Expected: clean.
```

- [ ] **Step 6: If any V1 gate fails, do not proceed. Diagnose and fix.**

(No commit; verification.)

---

### Task 22: V3 — Section-placement check on both ELFs

**Files:** (no edits; verification-only)

- [ ] **Step 1: Run on H7 ELF (assuming Task 21 produced one)**

```bash
./scripts/check_rt_storage_placement.sh out/klipper.elf
# Expected: "OK: rt_storage placed correctly on H7"
```

- [ ] **Step 2: Repeat for F4 ELF**

(Rebuild for F4 if Task 21 left the H7 ELF in place.)

```bash
./scripts/check_rt_storage_placement.sh out/klipper.elf
# Expected: "OK: rt_storage placed correctly on F4"
```

- [ ] **Step 3: Verify the script also catches deliberate misplacement**

Quick smoke test: pass a non-existent ELF or an ELF without rt_storage:

```bash
./scripts/check_rt_storage_placement.sh /tmp/empty.elf 2>&1
# Expected: ERROR, non-zero exit.
```

(No commit unless modifying the script.)

---

### Task 23: V5 + V6 — cargo test workspace + Renode soak

**Files:** (no edits; verification-only)

- [ ] **Step 1: cargo test workspace**

```bash
cd rust && cargo test --workspace 2>&1 | tail -20
# Expected: all tests pass, including the new rt_storage_drift tests.
```

- [ ] **Step 2: Run the A1–A7 deterministic battery**

```bash
cd rust && cargo test -p kalico-c-api 2>&1 | grep -E 'test a[1-7]_'
# Expected: A1 through A7 tests all listed as passing.
```

- [ ] **Step 3: Renode soak**

Per Step 7-C-io infrastructure. The exact invocation depends on existing Renode tooling — likely a `make renode-soak` target or a script under `scripts/`.

```bash
# Adjust per the actual Renode soak invocation:
make renode-soak 2>&1 | tail -10
# Expected: soak completes without MCU-shutdown or assertion failure.
```

If the Renode soak harness isn't wired into the Makefile yet, document the expected manual invocation in the audit doc and skip (Renode soak is a Step 7-D acceptance gate per CLAUDE.md).

- [ ] **Step 4: Document gate outcomes**

If gates pass: continue to Task 24.
If any gate fails: diagnose, fix, return to Task 21.

(No commit unless tests were modified.)

---

### Task 24: V7 + V8 — Bench on both MCUs

**Files:** (no edits; user-driven verification)

- [ ] **Step 1: Push the branch**

```bash
git push -u origin mcu-boundary-refactor
```

- [ ] **Step 2: H7 bench**

Per `feedback_bench_firmware_flow.md`: commit → push → pull on Pi → make → flash. Run the user's bench-smoke sequence (per-command permission required per `feedback_no_gcode_without_permission.md`).

- [ ] **Step 3: F4 bench**

Flash F4. Confirm: boots, no MCU-shutdown, steady-state for ≥1 minute.

- [ ] **Step 4: If any bench fails, do not declare done. Diagnose and fix.**

(No commit unless bug-fix needed.)

---

## Phase 7 — Doc reconciliation

### Task 25: Update boundary doc post-refactor

**Files:**
- Modify: `docs/kalico-rewrite/mcu-c-rust-boundary.md`

- [ ] **Step 1: Update the RT_CELL row in the "What lives where on the MCU" table**

Find the row currently reading:

```
| `RT_CELL` (motion runtime state, per-MCU-target placement) | Rust today; *should migrate to C-owned `rt_storage` byte buffer per B2.* | See "Open migrations." | ...
```

Replace with:

```
| Runtime context backing (`rt_storage`) | C-declared `uint8_t` buffer in `src/runtime_storage.c`. H7: `.axi_bss` (AXI SRAM at 0x24000000). F4: regular `.bss`. Rust imports via `extern "C" { static rt_storage: UnsafeCell<[u8; N]>; }` and casts to `*mut RuntimeContext`. | Migrated 2026-MM-DD from `RT_CELL` Rust static. Cargo-clean discipline (`feedback_cargo_clean_between_mcus.md`) is now optional rather than safety-critical. |
```

- [ ] **Step 2: Update the .axi_bss occupants row with measured byte counts**

Find the `.axi_bss` row added in commit `e812d436b`. Fill in exact byte counts from the A1 audit findings.

- [ ] **Step 3: Delete the "Open migrations" section's RT_CELL entry**

Find the section reading `## Open migrations` and the `RT_CELL` bullet. Either delete the bullet or, if it was the only item, delete the entire section.

- [ ] **Step 4: Add a new case study**

In the case-studies section, add an entry:

```markdown
- **2026-MM-DD — RT_CELL migrated to C-owned `rt_storage`.** The Rust static with `#[link_section = ".axi_bss"]` was replaced by a C-declared `uint8_t rt_storage[RT_STORAGE_SIZE]` buffer in `src/runtime_storage.c`. Section placement now decided exclusively by the C linker script (cfg-gated attribute: `.axi_bss` on H7, default `.bss` on F4). Rust imports via `extern "C"` with an `UnsafeCell` wrapper for interior-mutability rights; soundness verified under stacked/tree borrows. Closes the cargo-clean operational tripwire (`feedback_cargo_clean_between_mcus.md`) — the operational discipline becomes optional because Rust no longer makes a section-placement decision. **Rule reinforced:** B2 — single-language ownership of section placement is the structural fix; per-target compile artifacts no longer interact with placement.
```

(Replace `MM-DD` with the actual completion date.)

- [ ] **Step 5: Commit**

```bash
git add docs/kalico-rewrite/mcu-c-rust-boundary.md
git commit -m "docs(boundary): retire RT_CELL open-migration; add case study

Reflects the RT_CELL → rt_storage refactor (commits 7aab973cc..HEAD).
Updates the 'What lives where' table with the new architecture, fills
in .axi_bss occupant byte counts from the A1 audit, deletes the open-
migration entry, and adds the migration to the case-study list as the
fourth example of the boundary-discipline pattern."
```

---

### Task 26: Update memory entry: cargo-clean discipline now optional

**Files:**
- Modify: `/Users/daniladergachev/.claude/projects/-Users-daniladergachev-Developer-kalico/memory/feedback_cargo_clean_between_mcus.md`

- [ ] **Step 1: Update the memory entry**

Mark the cargo-clean discipline as historical / optional. The exact wording depends on the current file; goal is to flag that the operational ritual was an operational fix for a now-structurally-fixed problem.

- [ ] **Step 2: No commit (memory files are not in git).**

---

## Self-review

After completing all tasks, run the spec coverage check:

| Spec section | Task(s) |
|---|---|
| Goal 1 (RT_CELL → C-declared buffer) | 5, 6, 7, 8, 9, 10 |
| Goal 2 (delete #[link_section] + feature) | 8, 10 |
| Goal 3 (sweep boundary for other leaks) | 14-20 |
| Goal 4 (doc reconciliation) | 17, 25 |
| Goal 5 (everything works on both MCUs) | 21-24 |
| Spec § Architecture | 5-10 |
| Spec § Section 1 (mechanism) | 8, 9, 10 |
| Spec § Section 2 (size tracking) | 1, 2, 3, 4, 6 |
| Spec § Section 3 (per-MCU placement) | 6 |
| Spec § Audit A1 | 15 |
| Spec § Audit A2 | 17 |
| Spec § Audit A3 | 16 |
| Spec § Audit A4 | 19 |
| Spec § Audit A5 | 18 |
| Spec § Audit A6 | 19 |
| Spec § Audit A7 | 19 |
| Spec § Audit A8 | 20 |
| Spec § Verification V1 | 21 |
| Spec § Verification V2 | 13 |
| Spec § Verification V3 | 11, 22 |
| Spec § Verification V4 | 12 |
| Spec § Verification V5 | 23 |
| Spec § Verification V6 | 23 |
| Spec § Verification V7 | 24 |
| Spec § Verification V8 | 24 |
| Spec § Doc reconciliation | 17, 25 |

Every spec section maps to at least one task. No gaps identified.

**Critical-path note:** Tasks 1–10 are the structural refactor (about half the work). Tasks 11–13 are verification scaffolding that lands inline. Tasks 14–20 are the audit pass (discovery + small fixes); these may surface unexpected work but each is bounded. Tasks 21–24 are user-driven verification on actual hardware. Tasks 25–26 are doc cleanup that closes the loop.

**Subagent recommendation:** All Rust-touching tasks (3, 4, 8, 9, 10, 12, 16, 18, 20) dispatched as `subagent_type: "rust-engineer"` per saved feedback memory. C/Makefile/Kconfig tasks (1, 2, 5, 6, 7, 11) can go to `general-purpose` or run inline.

---

## Execution handoff

**Plan complete and saved to `docs/superpowers/plans/2026-05-19-mcu-c-rust-boundary-refactor.md`.**

Two execution options:

**1. Subagent-Driven (recommended)** — Dispatch a fresh `rust-engineer` subagent per Rust task (and general-purpose for C/Make/Kconfig tasks); review between tasks; fast iteration; bounded context per task.

**2. Inline Execution** — Execute tasks in this session using `superpowers:executing-plans`; batch execution with checkpoints for user review.

Which approach?
