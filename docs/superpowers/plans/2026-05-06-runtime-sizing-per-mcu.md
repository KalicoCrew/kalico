# Per-MCU Runtime Sizing — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make F446 fork-native firmware build, route per-MCU caps from firmware to host so the planner adapts curve sizes per destination, then flash F446 on `dderg@trident.local` so a multi-MCU `G28` (Z lift on F446 + X/Y home on H7) works against the new motion bridge end-to-end.

**Architecture:** Kconfig owns four `RUNTIME_*` integers; C reads them via `autoconf.h`, Rust reads them via env vars + a `runtime/build.rs` that emits a sizing module. A new `QueryRuntimeCaps` / `RuntimeCapsResponse` message pair (Identify is frozen-forever) carries the firmware's caps to the host. The motion-bridge stores caps per-MCU and splits any planned curve that would exceed the destination MCU's `max_control_points`.

**Tech Stack:** Klipper Kconfig, GNU Make, Rust 1.85 (`runtime`, `kalico-protocol`, `motion-bridge`, `kalico-c-api` crates), C99 (firmware), Renode + `tools/sim_klippy/run_local.sh` for simulator validation.

**Spec:** `docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md`

**Branch:** `sota-motion`. The local cross-build toolchain at `~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin` is the iterate loop; trident SSH is the deploy loop.

---

## File Map

**Kconfig and build:**
- Modify: `src/Kconfig` — add the profile choice + four int entries under `KALICO_RUNTIME`.
- Modify: `src/Makefile:49,55-58,65-71` — pass `KALICO_RUNTIME_*` env vars into cargo invocations.
- Modify: `src/runtime_tick.c:424-425` — replace hardcoded `[1830]` / `[1850]` with `[CONFIG_RUNTIME_MAX_CONTROL_POINTS]` / `[CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN]`.
- Modify: `src/kalico_dispatch.c:25-26,313-314` — extern declarations and bound checks read from same `CONFIG_RUNTIME_*` macros.
- Modify: `src/kalico_demux.h:30` — derive `KALICO_DEMUX_KALICO_BUF_SIZE` from `CONFIG_RUNTIME_MAX_CONTROL_POINTS + CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN`.

**Rust runtime:**
- Create: `rust/runtime/build.rs` — read env vars, emit `$OUT_DIR/sizing.rs`.
- Modify: `rust/runtime/Cargo.toml` — declare `build = "build.rs"`.
- Modify: `rust/runtime/src/curve_pool.rs:32-43` — replace hardcoded constants with `include!(concat!(env!("OUT_DIR"), "/sizing.rs"));`.

**Protocol:**
- Modify: `rust/kalico-protocol/src/messages.rs` — add `QueryRuntimeCaps = 0x0040` / `RuntimeCapsResponse = 0x0041` enum variants and structs.
- Modify: `rust/kalico-protocol/schema_def.rs` — register the two new message kinds for schema-hash inclusion.
- Modify: `src/kalico_dispatch.c` — add handler that responds with `CONFIG_RUNTIME_*` values.

**Host bridge:**
- Modify: `rust/motion-bridge/src/bridge.rs` — bootstrap step queries caps after Identify; stores `McuCaps` per MCU.
- Modify: `rust/motion-bridge/src/dispatch.rs` — `McuAxisConfig` gains `caps: McuCaps`; helper to check if a curve fits.
- New helper module: `rust/motion-bridge/src/cap_check.rs` — pure function `fits(caps: &McuCaps, curve: &ScalarNurbs<f64>) -> bool` and `split_segment(seg, caps_per_mcu) -> Vec<ShapedSegment>`.

**Firmware configs and tests:**
- Create on trident only: `~/klipper/.config.f446.bak` — F446 menuconfig snapshot.
- Modify: `rust/motion-bridge/tests/sim_motion.rs` — multi-MCU sim run with one low-cap MCU.

---

## Phase 1 — Kconfig + build pipeline (constants flow; H7 values unchanged)

### Task 1: Add Kconfig profile choice + four int entries

**Files:**
- Modify: `src/Kconfig` (insert after the existing `KALICO_RUNTIME` block; before `KALICO_SIM`)

- [ ] **Step 1: Read the current Kconfig section to know where to insert**

Run: `sed -n '380,410p' src/Kconfig`
Expected: Shows `config KALICO_RUNTIME` (lines 385-393) followed by `config KALICO_SIM` (lines 395-403).

- [ ] **Step 2: Insert profile choice and four entries between them**

Edit `src/Kconfig`. After the closing line of `KALICO_RUNTIME`'s help block (line ~393, the line ending with `simulator (pthread tick) for klippy-in-loop testing.`), and before the blank line preceding `config KALICO_SIM`, insert:

```kconfig

choice
    prompt "Runtime sizing profile"
    default RUNTIME_TARGET_LARGE if MACH_STM32H7
    default RUNTIME_TARGET_SMALL if MACH_STM32F4
    default RUNTIME_TARGET_LARGE
    depends on KALICO_RUNTIME
    help
      Selects per-MCU runtime sizing presets. `large` is the H7-class
      profile that places the curve pool in AXI SRAM. `small` is the
      F4-class profile that fits in 128 KB SRAM. `custom` lets you
      override individual values below.

    config RUNTIME_TARGET_LARGE
        bool "Large (H7-class — fits in AXI SRAM)"

    config RUNTIME_TARGET_SMALL
        bool "Small (F4-class — fits in 128 KB SRAM)"

    config RUNTIME_TARGET_CUSTOM
        bool "Custom (override individual values)"
endchoice

config RUNTIME_MAX_CONTROL_POINTS
    int "Max control points per curve slot"
    default 1830 if RUNTIME_TARGET_LARGE
    default 512 if RUNTIME_TARGET_SMALL
    default 512
    range 16 4096
    depends on KALICO_RUNTIME

config RUNTIME_MAX_KNOT_VECTOR_LEN
    int "Max knot-vector length per curve slot"
    default 1850 if RUNTIME_TARGET_LARGE
    default 524 if RUNTIME_TARGET_SMALL
    default 524
    range 27 4108
    depends on KALICO_RUNTIME
    help
      Must satisfy: RUNTIME_MAX_KNOT_VECTOR_LEN >= RUNTIME_MAX_CONTROL_POINTS
      + RUNTIME_MAX_DEGREE + 1. The Rust runtime asserts this at compile
      time; mismatched values fail the build.

config RUNTIME_MAX_DEGREE
    int "Max polynomial degree"
    default 10
    range 1 16
    depends on KALICO_RUNTIME

config RUNTIME_CURVE_POOL_N
    int "Curve-pool slot count (planner look-ahead depth)"
    default 16 if RUNTIME_TARGET_LARGE
    default 4 if RUNTIME_TARGET_SMALL
    default 4
    range 1 64
    depends on KALICO_RUNTIME
```

- [ ] **Step 3: Verify olddefconfig produces expected autoconf values**

Run: `make olddefconfig 2>&1 | tail -3 && grep RUNTIME_ out/autoconf.h`
Expected: Shows `#define CONFIG_RUNTIME_MAX_CONTROL_POINTS 1830`, `..._KNOT_VECTOR_LEN 1850`, `..._MAX_DEGREE 10`, `..._CURVE_POOL_N 16`, and `#define CONFIG_RUNTIME_TARGET_LARGE 1`. (The current `.config` is H7, so `large` profile is auto-selected.)

- [ ] **Step 4: Commit**

```bash
git add src/Kconfig
git commit -m "kconfig: add per-MCU runtime sizing profile + four int entries"
```

### Task 2: Replace hardcoded C-side scratch buffer sizes

**Files:**
- Modify: `src/runtime_tick.c:424-425`
- Modify: `src/kalico_dispatch.c:25-26,313-314`

- [ ] **Step 1: Read the current scratch buffer declarations**

Run: `sed -n '422,427p' src/runtime_tick.c && echo --- && sed -n '23,28p' src/kalico_dispatch.c`
Expected: Shows `float kalico_aligned_cps[1830];` and `float kalico_aligned_knots[1850];` in runtime_tick.c, and `extern float kalico_aligned_cps[1830]; extern float kalico_aligned_knots[1850];` in kalico_dispatch.c.

- [ ] **Step 2: Make runtime_tick.c read from autoconf**

Edit `src/runtime_tick.c`. Replace:

```c
float kalico_aligned_cps[1830];    // MAX_CONTROL_POINTS
float kalico_aligned_knots[1850];  // MAX_KNOT_VECTOR_LEN
```

with:

```c
float kalico_aligned_cps[CONFIG_RUNTIME_MAX_CONTROL_POINTS];
float kalico_aligned_knots[CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN];
```

(The file already includes `autoconf.h` via the standard Klipper include chain; confirm with `grep -n autoconf src/runtime_tick.c | head -1` — should show line 1-20 region.)

- [ ] **Step 3: Make kalico_dispatch.c match the new sizes**

Edit `src/kalico_dispatch.c`. Replace lines 25-26:

```c
extern float kalico_aligned_cps[1830];
extern float kalico_aligned_knots[1850];
```

with:

```c
extern float kalico_aligned_cps[CONFIG_RUNTIME_MAX_CONTROL_POINTS];
extern float kalico_aligned_knots[CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN];
```

The bound checks at lines 313-314 already use `sizeof(kalico_aligned_cps)` / `sizeof(kalico_aligned_knots)`, so they auto-adapt — no edit needed.

- [ ] **Step 4: Cross-build to verify nothing broke**

Run: `export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH && make -j4 2>&1 | tail -8`
Expected: `Creating hex file out/klipper.bin`. The `axi_ram` line should still show `284808 B / 320 KB`. (Same value because the constants haven't changed yet.)

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c src/kalico_dispatch.c
git commit -m "runtime: C scratch buffers read sizes from CONFIG_RUNTIME_*"
```

### Task 3: Derive demux buffer size from runtime constants

**Files:**
- Modify: `src/kalico_demux.h:21-30`

- [ ] **Step 1: Read the current sizing**

Run: `sed -n '20,32p' src/kalico_demux.h`
Expected: Shows `#define KALICO_DEMUX_KLIPPER_BUF_SIZE MESSAGE_MAX` and `#define KALICO_DEMUX_KALICO_BUF_SIZE 8192`.

- [ ] **Step 2: Replace the hardcoded 8192**

Edit `src/kalico_demux.h`. Replace:

```c
#define KALICO_DEMUX_KALICO_BUF_SIZE  8192
```

with:

```c
// Largest in-bound kalico frame is a LoadCurve carrying one slot's worth of
// control points + knots, plus a small per-frame header. Sizing: 4 bytes per
// f32 × (cps + knots) + 32 bytes for sync/len/channel/header/CRC and
// per-message envelope. Stays in lockstep with the Rust runtime's pool
// dimensions (CONFIG_RUNTIME_MAX_*) so the firmware never has to drop a
// curve upload that the Rust side would still accept.
#define KALICO_DEMUX_KALICO_BUF_SIZE \
    (4u * (CONFIG_RUNTIME_MAX_CONTROL_POINTS + CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN) + 32u)
```

Add `#include "autoconf.h"` at the top of `src/kalico_demux.h` if not already present (check first with `head -10 src/kalico_demux.h`).

- [ ] **Step 3: Cross-build and confirm RAM goes down**

Run: `export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH && make clean && make -j4 2>&1 | grep -E "ram|axi_ram"`
Expected: `axi_ram` still ~285 KB. `ram` line still 100% (we shrank demux from 8192 to (1830+1850)*4+32 = 14752 — bigger! Because H7 large profile uses bigger constants). Wait, that's a regression on H7 RAM.

Branch decision in this step: if H7 RAM overflows after the change, the demux derivation is too generous for H7. Cap it: `#define KALICO_DEMUX_KALICO_BUF_SIZE (4u * (CONFIG_RUNTIME_MAX_CONTROL_POINTS + CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN) + 32u)` works for the small profile (4192 bytes) but is worse than 8192 on large.

**Resolution:** the H7 demux buffer should also be relocated to AXI SRAM since it's runtime-related. Add `__attribute__((section(".axi_bss")))` to `kalico_buf` declaration in `src/kalico_demux.c:35` *only* on H7:

```c
#if CONFIG_MACH_STM32H7
__attribute__((section(".axi_bss")))
#endif
static uint8_t kalico_buf[KALICO_DEMUX_KALICO_BUF_SIZE];
```

- [ ] **Step 4: Cross-build and confirm**

Run: `export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH && make clean && make -j4 2>&1 | grep -E "ram|axi_ram"`
Expected: `ram` no longer at 100%; `axi_ram` slightly larger to absorb the demux buf. Both within their regions.

- [ ] **Step 5: Commit**

```bash
git add src/kalico_demux.h src/kalico_demux.c
git commit -m "demux: size kalico_buf from runtime constants; place in AXI on H7"
```

### Task 4: Add `runtime/build.rs` that emits sizing constants

**Files:**
- Create: `rust/runtime/build.rs`
- Modify: `rust/runtime/Cargo.toml`

- [ ] **Step 1: Add `build = "build.rs"` to runtime/Cargo.toml**

Edit `rust/runtime/Cargo.toml`. Find the `[package]` section (lines 1-10) and add `build = "build.rs"` directly after the `edition` line:

```toml
[package]
name = "runtime"
version = "0.1.0"
edition = "2024"
build = "build.rs"
```

(Confirm by `grep -n edition rust/runtime/Cargo.toml` first.)

- [ ] **Step 2: Create the build script**

Create `rust/runtime/build.rs` with content:

```rust
//! Emits sizing constants that vary per target MCU build.
//!
//! Reads four env vars exported by Klipper's Makefile (which sources them
//! from the matching `CONFIG_RUNTIME_*` Kconfig values). Defaults match the
//! H7 `large` profile so host-only / sim builds (which don't go through the
//! Klipper Makefile) still compile.
//!
//! Spec: docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md §4.3.

use std::env;
use std::fs;
use std::path::PathBuf;

fn lookup(name: &str, default: &str) -> String {
    println!("cargo:rerun-if-env-changed={name}");
    env::var(name).unwrap_or_else(|_| default.to_string())
}

fn main() {
    let mcp = lookup("KALICO_RUNTIME_MAX_CONTROL_POINTS", "1830");
    let mkv = lookup("KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN", "1850");
    let mdg = lookup("KALICO_RUNTIME_MAX_DEGREE", "10");
    let cpn = lookup("KALICO_RUNTIME_CURVE_POOL_N", "16");

    let out_dir = PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set by cargo"));
    let body = format!(
        "// Auto-generated by runtime/build.rs — do not edit.\n\
         pub const MAX_CONTROL_POINTS: usize = {mcp};\n\
         pub const MAX_KNOT_VECTOR_LEN: usize = {mkv};\n\
         pub const MAX_DEGREE: u8 = {mdg};\n\
         pub const CURVE_POOL_N: usize = {cpn};\n"
    );
    fs::write(out_dir.join("sizing.rs"), body).expect("write sizing.rs");
}
```

- [ ] **Step 3: Verify build.rs runs and emits sizing.rs**

Run: `cd rust && cargo build -p runtime 2>&1 | tail -3 && find target -name sizing.rs -exec cat {} \;`
Expected: build succeeds, `target/.../out/sizing.rs` shows the four `pub const`s with the H7-default values.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/build.rs rust/runtime/Cargo.toml
git commit -m "runtime: build.rs emits sizing constants from env vars"
```

### Task 5: Replace hardcoded constants in `curve_pool.rs` with the generated module

**Files:**
- Modify: `rust/runtime/src/curve_pool.rs:25-43`

- [ ] **Step 1: Read the current constants**

Run: `sed -n '25,45p' rust/runtime/src/curve_pool.rs`
Expected: Shows the four `pub const` declarations with H7 values.

- [ ] **Step 2: Replace with `include!`**

Edit `rust/runtime/src/curve_pool.rs`. Replace lines 25-43 (the doc comment block + four `pub const` lines + the `MAX_DIM` const block, but keep the knot-rule assert at line 38) with:

```rust
/// Build-time-configurable sizing constants. The four `pub const`s (see
/// `runtime/build.rs`) are emitted from Klipper's Kconfig values:
///   CONFIG_RUNTIME_MAX_CONTROL_POINTS
///   CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN
///   CONFIG_RUNTIME_MAX_DEGREE
///   CONFIG_RUNTIME_CURVE_POOL_N
/// Defaults (no Klipper Makefile in the loop) match the `large` profile per
/// `docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md`.
include!(concat!(env!("OUT_DIR"), "/sizing.rs"));

// Looser invariant: MAX_KNOT_VECTOR_LEN >= MAX_CONTROL_POINTS + MAX_DEGREE + 1
// (NURBS knot rule). Mismatched Kconfig values fail this assert at compile time.
const _: () = assert!(MAX_KNOT_VECTOR_LEN >= MAX_CONTROL_POINTS + MAX_DEGREE as usize + 1);

/// Deprecated — kept for `kalico-c-api` compilation until Task 8 updates
/// the FFI. The scalar architecture uses 1D control points, not 3D vectors.
#[deprecated(note = "scalar curve pool — use 1D control points; removed in Task 8")]
pub const MAX_DIM: usize = 1;
```

- [ ] **Step 3: Verify all existing tests still pass**

Run: `cd rust && cargo test -p runtime 2>&1 | tail -10`
Expected: All tests pass (constants are unchanged from H7 default).

- [ ] **Step 4: Cross-build firmware**

Run: `export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH && make -j4 2>&1 | tail -5`
Expected: builds, `axi_ram` size unchanged.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/curve_pool.rs
git commit -m "runtime: curve_pool reads sizing from build.rs-generated module"
```

### Task 6: Wire Klipper Makefile to pass env vars to cargo

**Files:**
- Modify: `src/Makefile:49-71` (the two cargo invocations)

- [ ] **Step 1: Read the current cargo block**

Run: `sed -n '40,75p' src/Makefile`
Expected: Shows `KALICO_RUST_FEATURES := mcu-h7,header-nurbs,header-runtime` and two `cd rust && PATH=... cargo build` blocks (one for production, one for sim).

- [ ] **Step 2: Inject env vars into both cargo invocations**

Edit `src/Makefile`. Find each `cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" cargo build -p kalico-c-api` line and prefix it with the env vars. Example replacement:

Before:
```make
	cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" cargo build -p kalico-c-api \
		--no-default-features \
		--features $(KALICO_RUST_FEATURES) \
		--release
```

After:
```make
	cd rust && PATH="$(HOME)/.cargo/bin:$$PATH" \
		KALICO_RUNTIME_MAX_CONTROL_POINTS=$(CONFIG_RUNTIME_MAX_CONTROL_POINTS) \
		KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN=$(CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN) \
		KALICO_RUNTIME_MAX_DEGREE=$(CONFIG_RUNTIME_MAX_DEGREE) \
		KALICO_RUNTIME_CURVE_POOL_N=$(CONFIG_RUNTIME_CURVE_POOL_N) \
		cargo build -p kalico-c-api \
		--no-default-features \
		--features $(KALICO_RUST_FEATURES) \
		--release
```

Apply the same prefix to **both** invocations (production and sim — there should be exactly two `cd rust && ... cargo build` lines in the file).

- [ ] **Step 3: Verify build.rs sees the env vars**

Run: `export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH && make clean && make -j4 2>&1 | grep -E "ram|axi_ram" | tail -2`
Expected: `axi_ram: 285 KB` (unchanged for H7).

To confirm env vars actually flowed through, set a sentinel value temporarily and re-run:
```bash
KALICO_RUNTIME_MAX_CONTROL_POINTS=1832 make 2>&1 | tail -3
find rust/target -name sizing.rs | head -1 | xargs cat
```
Expected: `pub const MAX_CONTROL_POINTS: usize = 1832;`. Then `make clean && make` to reset.

- [ ] **Step 4: Commit**

```bash
git add src/Makefile
git commit -m "make: pass CONFIG_RUNTIME_* into cargo via KALICO_RUNTIME_* env vars"
```

---

## Phase 2 — `QueryRuntimeCaps` / `RuntimeCapsResponse` message pair

### Task 7: Add MessageKind variants and structs

**Files:**
- Modify: `rust/kalico-protocol/src/messages.rs`

- [ ] **Step 1: Add the two new variants to the enum + decoder**

Edit `rust/kalico-protocol/src/messages.rs`. In the `MessageKind` enum (lines 18-30), add two variants in the unused 0x004x range:

```rust
pub enum MessageKind {
    Identify = 0x0001,
    IdentifyResponse = 0x0002,
    LoadCurve = 0x0010,
    LoadCurveResponse = 0x0011,
    PushSegment = 0x0020,
    PushSegmentResponse = 0x0021,
    ConfigureAxes = 0x0030,
    ConfigureAxesResponse = 0x0031,
    QueryRuntimeCaps = 0x0040,
    RuntimeCapsResponse = 0x0041,
    StatusEvent = 0x0080,
    CreditFreed = 0x0081,
    FaultEvent = 0x0082,
}
```

In `from_u16` (lines 32-48), add the two matches:
```rust
0x0040 => Self::QueryRuntimeCaps,
0x0041 => Self::RuntimeCapsResponse,
```

In the `MessageKind` `all()` test array (line 413+), add:
```rust
MessageKind::QueryRuntimeCaps,
MessageKind::RuntimeCapsResponse,
```

- [ ] **Step 2: Add the response struct + Encode/Decode impls**

Append to `rust/kalico-protocol/src/messages.rs` (after the existing `LoadCurveResponse` block, in the same style):

```rust
// QueryRuntimeCaps (0x0040) — request body: empty.
// RuntimeCapsResponse (0x0041) — body layout:
//   0..4   max_control_points  : u32_le
//   4..8   max_knot_vector_len : u32_le
//   8      max_degree          : u8
//   9..11  curve_pool_n        : u16_le
// Total: 11 bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeCapsResponse {
    pub max_control_points: u32,
    pub max_knot_vector_len: u32,
    pub max_degree: u8,
    pub curve_pool_n: u16,
}

pub const RUNTIME_CAPS_RESPONSE_BODY_LEN: usize = 11;

impl Encode for RuntimeCapsResponse {
    fn encode_body(&self, out: &mut Vec<u8>) {
        put_u32(out, self.max_control_points);
        put_u32(out, self.max_knot_vector_len);
        out.push(self.max_degree);
        put_u16(out, self.curve_pool_n);
    }
}

impl Decode for RuntimeCapsResponse {
    fn decode_body(c: &mut Cursor) -> Result<Self, DecodeError> {
        let max_control_points = get_u32(c)?;
        let max_knot_vector_len = get_u32(c)?;
        let max_degree = get_u8(c)?;
        let curve_pool_n = get_u16(c)?;
        Ok(Self { max_control_points, max_knot_vector_len, max_degree, curve_pool_n })
    }
}
```

(`Encode`, `Decode`, `Cursor`, `put_u32`, `put_u16`, `get_u32`, `get_u16`, `get_u8`, `DecodeError` are existing imports — confirm via the top of the file.)

- [ ] **Step 3: Add a roundtrip test**

Append to the `mod tests` block at the bottom of `rust/kalico-protocol/src/messages.rs`:

```rust
#[test]
fn runtime_caps_roundtrip() {
    let original = RuntimeCapsResponse {
        max_control_points: 512,
        max_knot_vector_len: 524,
        max_degree: 10,
        curve_pool_n: 4,
    };
    let mut buf = Vec::new();
    original.encode_body(&mut buf);
    assert_eq!(buf.len(), RUNTIME_CAPS_RESPONSE_BODY_LEN);
    let mut c = Cursor::new(&buf);
    let decoded = RuntimeCapsResponse::decode_body(&mut c).unwrap();
    assert_eq!(decoded, original);
}
```

- [ ] **Step 4: Run tests**

Run: `cd rust && cargo test -p kalico-protocol runtime_caps_roundtrip 2>&1 | tail -5`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-protocol/src/messages.rs
git commit -m "protocol: QueryRuntimeCaps + RuntimeCapsResponse messages"
```

### Task 8: Register new messages in `schema_def.rs`

**Files:**
- Modify: `rust/kalico-protocol/schema_def.rs`

- [ ] **Step 1: Read the current schema definition**

Run: `head -100 rust/kalico-protocol/schema_def.rs`
Expected: Pure-data array of `(MessageTag, FieldList)` entries. Locate the entry for `LoadCurveResponse` (a similarly-shaped fixed-size response) to use as a template.

- [ ] **Step 2: Add entries for QueryRuntimeCaps (empty body) and RuntimeCapsResponse (11-byte body)**

Edit `rust/kalico-protocol/schema_def.rs`. Locate the data array and add two entries in the same position as the new MessageKind discriminants (between ConfigureAxesResponse=0x0031 and the events block starting at 0x0080):

```rust
SchemaEntry {
    tag: 0x0040,
    name: "QueryRuntimeCaps",
    fields: &[],
},
SchemaEntry {
    tag: 0x0041,
    name: "RuntimeCapsResponse",
    fields: &[
        Field { name: "max_control_points", ty: FieldType::U32 },
        Field { name: "max_knot_vector_len", ty: FieldType::U32 },
        Field { name: "max_degree", ty: FieldType::U8 },
        Field { name: "curve_pool_n", ty: FieldType::U16 },
    ],
},
```

(Match the exact struct/field types used elsewhere in the file — adjust if the existing schema definitions use different names, e.g. `MessageSchema` instead of `SchemaEntry`.)

- [ ] **Step 3: Verify schema-hash regenerates and tests pass**

Run: `cd rust && cargo test -p kalico-protocol 2>&1 | tail -10`
Expected: All tests pass. The `schema_hash` test (if it asserts a fixed hash) will fail — update it inline to the new value reported in the failure output. (Schema-hash changes are expected for this kind of additive protocol change.)

- [ ] **Step 4: Commit**

```bash
git add rust/kalico-protocol/schema_def.rs rust/kalico-protocol/tests/
git commit -m "protocol: include runtime-caps messages in schema hash"
```

### Task 9: C-side `QueryRuntimeCaps` handler

**Files:**
- Modify: `src/kalico_dispatch.c`

- [ ] **Step 1: Read the existing identify handler as a template**

Run: `sed -n '155,200p' src/kalico_dispatch.c`
Expected: `handle_identify` shows the pattern: build response body, call `kalico_dispatch_send_response` (or whatever the existing helper is named).

- [ ] **Step 2: Add the new handler function**

Edit `src/kalico_dispatch.c`. After `handle_identify` (around line 200), add:

```c
// RuntimeCapsResponse body (§5.1 of the per-MCU sizing spec). Pulled from
// Kconfig at compile time via autoconf.h — same source of truth that sizes
// the Rust runtime's curve pool.
static void
handle_query_runtime_caps(uint32_t correlation_id, const uint8_t *body,
                          uint16_t body_len)
{
    (void)body;
    (void)body_len;  // request body is empty
    uint8_t response_body[11];
    // u32 max_control_points
    response_body[0] = (uint8_t)(CONFIG_RUNTIME_MAX_CONTROL_POINTS & 0xFF);
    response_body[1] = (uint8_t)((CONFIG_RUNTIME_MAX_CONTROL_POINTS >> 8) & 0xFF);
    response_body[2] = (uint8_t)((CONFIG_RUNTIME_MAX_CONTROL_POINTS >> 16) & 0xFF);
    response_body[3] = (uint8_t)((CONFIG_RUNTIME_MAX_CONTROL_POINTS >> 24) & 0xFF);
    // u32 max_knot_vector_len
    response_body[4] = (uint8_t)(CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN & 0xFF);
    response_body[5] = (uint8_t)((CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN >> 8) & 0xFF);
    response_body[6] = (uint8_t)((CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN >> 16) & 0xFF);
    response_body[7] = (uint8_t)((CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN >> 24) & 0xFF);
    // u8 max_degree
    response_body[8] = (uint8_t)CONFIG_RUNTIME_MAX_DEGREE;
    // u16 curve_pool_n
    response_body[9] = (uint8_t)(CONFIG_RUNTIME_CURVE_POOL_N & 0xFF);
    response_body[10] = (uint8_t)((CONFIG_RUNTIME_CURVE_POOL_N >> 8) & 0xFF);

    // Build full message: kind(2) + version(1) + correlation_id(4) + body(11)
    uint8_t full[7 + 11];
    encode_message_header(full, 0x0041, 0, correlation_id);
    memcpy(&full[7], response_body, 11);
    kalico_transport_send_frame(KALICO_CHANNEL_KALICO, full, sizeof(full));
}
```

- [ ] **Step 3: Wire the new handler into the dispatcher**

In `src/kalico_dispatch.c`, find the dispatcher function (look for the existing `case 0x0001:` for Identify; should be near line 215-230). Add a case for `0x0040`:

```c
case 0x0040:
    handle_query_runtime_caps(correlation_id, body, body_len);
    break;
```

(The exact case statement format matches the existing dispatch table — match style.)

- [ ] **Step 4: Cross-build to verify**

Run: `export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH && make -j4 2>&1 | tail -5`
Expected: builds clean.

- [ ] **Step 5: Commit**

```bash
git add src/kalico_dispatch.c
git commit -m "dispatch: handle QueryRuntimeCaps; respond with CONFIG_RUNTIME_* values"
```

---

## Phase 3 — Host bridge stores caps and splits oversized curves

### Task 10: Bridge bootstrap queries caps after Identify

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs`

- [ ] **Step 1: Find where Identify completes and the MCU is registered**

Run: `grep -nE "IdentifyResponse|register_mcu|mcu_id|set_clock_est" rust/motion-bridge/src/bridge.rs | head -15`
Expected: Shows the bootstrap path. Note the function/closure where Identify response is received and the MCU enters the `mcu_configs` map.

- [ ] **Step 2: Add a `query_caps` helper that sends QueryRuntimeCaps and waits for response**

In `rust/motion-bridge/src/bridge.rs`, add a private fn near the existing transport helpers:

```rust
async fn query_runtime_caps(io: &KalicoHostIo, mcu_id: u32, timeout: Duration)
    -> Result<RuntimeCapsResponse, String>
{
    use kalico_protocol::messages::{MessageKind, RuntimeCapsResponse};
    let correlation_id = io.alloc_correlation_id();
    io.send_command(mcu_id, MessageKind::QueryRuntimeCaps, correlation_id, &[])
        .map_err(|e| format!("send QueryRuntimeCaps: {e}"))?;
    let resp = io.recv_response(mcu_id, correlation_id, timeout)
        .map_err(|e| format!("recv RuntimeCapsResponse: {e}"))?;
    let mut c = kalico_protocol::messages::Cursor::new(&resp.body);
    RuntimeCapsResponse::decode_body(&mut c)
        .map_err(|e| format!("decode RuntimeCapsResponse: {e:?}"))
}
```

(Adjust the call signatures to match the actual host_io API — the helpers `alloc_correlation_id`, `send_command`, `recv_response` are illustrative; check `rust/kalico-host-rt/src/host_io/mod.rs` for the real method names and adapt.)

- [ ] **Step 3: Call `query_runtime_caps` after Identify in bootstrap**

In the bootstrap function (the one that runs Identify per MCU), after `IdentifyResponse` is decoded successfully, immediately call `query_runtime_caps` and store the result alongside the existing `mcu_configs` entry. If the call times out or errors, log a warning and use `RuntimeCapsResponse { max_control_points: 1830, max_knot_vector_len: 1850, max_degree: 10, curve_pool_n: 16 }` (large profile) as a fallback — preserves existing behavior for older firmware.

- [ ] **Step 4: Add a unit test using the existing mock_transport**

Add to `rust/motion-bridge/tests/sim_motion.rs`:

```rust
#[test]
fn query_runtime_caps_roundtrip_via_mock() {
    use kalico_protocol::messages::{MessageKind, RuntimeCapsResponse};
    let mock = MockTransport::new();
    mock.expect_command(MessageKind::QueryRuntimeCaps, 0)
        .respond_with(MessageKind::RuntimeCapsResponse, |corr| {
            let mut body = Vec::new();
            RuntimeCapsResponse {
                max_control_points: 512,
                max_knot_vector_len: 524,
                max_degree: 10,
                curve_pool_n: 4,
            }.encode_body(&mut body);
            (corr, body)
        });
    // ... drive query_runtime_caps via bridge bootstrap path and assert
    // the returned struct matches.
}
```

(Match the actual `MockTransport` API; see `rust/kalico-host-rt/tests/mock_transport.rs`.)

- [ ] **Step 5: Run the test**

Run: `cd rust && cargo test -p motion-bridge query_runtime_caps_roundtrip 2>&1 | tail -10`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs rust/motion-bridge/tests/sim_motion.rs
git commit -m "bridge: query RuntimeCaps after Identify; store per-MCU"
```

### Task 11: Bridge stores `McuCaps` in `McuAxisConfig`

**Files:**
- Modify: `rust/motion-bridge/src/dispatch.rs`
- Modify: `rust/motion-bridge/src/bridge.rs` (the call sites that build `McuAxisConfig`)

- [ ] **Step 1: Add `McuCaps` struct + field**

Edit `rust/motion-bridge/src/dispatch.rs`. After `pub struct McuAxisConfig` (lines 25-32), add:

```rust
/// Subset of `RuntimeCapsResponse` that the dispatcher needs to enforce
/// per-MCU sizing limits when planning a curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuCaps {
    pub max_control_points: u32,
    pub max_knot_vector_len: u32,
    pub max_degree: u8,
    pub curve_pool_n: u16,
}

impl Default for McuCaps {
    /// Large-profile defaults for backward compatibility with firmware
    /// that doesn't yet implement QueryRuntimeCaps.
    fn default() -> Self {
        Self {
            max_control_points: 1830,
            max_knot_vector_len: 1850,
            max_degree: 10,
            curve_pool_n: 16,
        }
    }
}
```

Then add `caps: McuCaps,` to `McuAxisConfig`:

```rust
pub struct McuAxisConfig {
    pub mcu_id: u32,
    pub axes: Vec<usize>,
    pub kinematics: u8,
    pub caps: McuCaps,
}
```

- [ ] **Step 2: Update all `McuAxisConfig {...}` constructors**

Run: `grep -rn "McuAxisConfig {" rust/motion-bridge/`
Expected: Lists every site where the struct is built. For each, add `caps: McuCaps::default()` (or thread the real caps from bootstrap if the construction site is downstream of Identify).

The primary construction site is in `rust/motion-bridge/src/bridge.rs` around line 960 (where `mcu_configs = vec![...]` is built). At that point the bootstrap has already received `RuntimeCapsResponse` per MCU; thread it through.

- [ ] **Step 3: cargo check**

Run: `cd rust && cargo check -p motion-bridge 2>&1 | tail -10`
Expected: clean compile.

- [ ] **Step 4: Run existing tests to ensure nothing regressed**

Run: `cd rust && cargo test -p motion-bridge 2>&1 | tail -10`
Expected: All existing tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/dispatch.rs rust/motion-bridge/src/bridge.rs
git commit -m "bridge: McuAxisConfig carries McuCaps from Identify"
```

### Task 12: Cap-check helper + test

**Files:**
- Create: `rust/motion-bridge/src/cap_check.rs`
- Modify: `rust/motion-bridge/src/lib.rs` (add the module)

- [ ] **Step 1: Write the failing test first**

Create `rust/motion-bridge/src/cap_check.rs` with just the test (compile error proves we have a TDD setup):

```rust
//! Per-MCU curve-size validation.
//!
//! Spec: docs/superpowers/specs/2026-05-06-runtime-sizing-per-mcu-design.md §5.3.

use crate::dispatch::McuCaps;
use nurbs::ScalarNurbs;

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dispatch::McuCaps;

    fn small_caps() -> McuCaps {
        McuCaps { max_control_points: 64, max_knot_vector_len: 76, max_degree: 10, curve_pool_n: 4 }
    }

    #[test]
    fn small_curve_fits_small_caps() {
        // 8-piece cubic: 8*3 + 1 = 25 cps.
        let cps = vec![0.0_f64; 25];
        let knots = vec![0.0_f64; 29];
        let curve = ScalarNurbs::new(3, &cps, &knots).unwrap();
        assert!(fits(&small_caps(), &curve));
    }

    #[test]
    fn oversize_curve_does_not_fit() {
        // 100-piece cubic: 100*3 + 1 = 301 cps — over 64 cap.
        let cps = vec![0.0_f64; 301];
        let knots = vec![0.0_f64; 305];
        let curve = ScalarNurbs::new(3, &cps, &knots).unwrap();
        assert!(!fits(&small_caps(), &curve));
    }
}
```

In `rust/motion-bridge/src/lib.rs`, add `pub mod cap_check;`.

- [ ] **Step 2: Run; verify failure**

Run: `cd rust && cargo test -p motion-bridge cap_check 2>&1 | tail -10`
Expected: FAIL with "cannot find function `fits` in this scope".

- [ ] **Step 3: Implement `fits`**

Append to `rust/motion-bridge/src/cap_check.rs`:

```rust
/// True if the curve fits within the caps reported by the destination MCU.
pub fn fits(caps: &McuCaps, curve: &ScalarNurbs<f64>) -> bool {
    curve.control_points().len() as u32 <= caps.max_control_points
        && curve.knots().len() as u32 <= caps.max_knot_vector_len
        && curve.degree() as u8 <= caps.max_degree
}
```

(Adjust `.degree()`, `.control_points()`, `.knots()` to whatever the actual `nurbs::ScalarNurbs` API offers — confirm via `grep -n "pub fn" rust/nurbs/src/lib.rs | head`.)

- [ ] **Step 4: Tests pass**

Run: `cd rust && cargo test -p motion-bridge cap_check 2>&1 | tail -10`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/cap_check.rs rust/motion-bridge/src/lib.rs
git commit -m "bridge: cap_check::fits validates curve against MCU caps"
```

### Task 13: Bridge splits oversized segments before dispatch

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (the dispatch closure, around line 1086)

- [ ] **Step 1: Locate the dispatch site and write a failing integration test first**

Add to `rust/motion-bridge/tests/sim_motion.rs`:

```rust
#[test]
fn dispatch_splits_curve_over_mcu_cap() {
    // Build a synthetic ShapedSegment whose Z curve has 200 control points.
    // Configure a single MCU with max_control_points=64.
    // After dispatch, expect at least 4 LoadCurve frames (200/64 ≈ 4 splits).
    // ... (test scaffolding — match the existing sim_motion.rs harness)
}
```

(Detailed test code: study `rust/motion-bridge/tests/sim_motion.rs` to see how segments are constructed and what assertions look like in that harness; mirror the style.)

- [ ] **Step 2: Run; verify failure**

Run: `cd rust && cargo test -p motion-bridge dispatch_splits 2>&1 | tail -10`
Expected: FAIL — current code dispatches one oversize curve.

- [ ] **Step 3: Add splitting in the dispatch closure**

In `rust/motion-bridge/src/bridge.rs` around line 1086 (the closure that builds `mcu_plans`), wrap the per-axis curve construction with a fits-check:

```rust
let mcu_plans = build_push_params(seg, &mcu_configs_for_cb, 0, 0);

for plan in &mcu_plans {
    let caps = mcu_configs_for_cb
        .iter()
        .find(|c| c.mcu_id == plan.mcu_id)
        .map(|c| c.caps)
        .unwrap_or_default();
    for (_axis, curve_params) in &plan.curves_to_load {
        // Reconstruct nurbs::ScalarNurbs from CurveLoadParams to check fit.
        // (helper: dispatch::curve_from_params)
        let curve = dispatch::curve_from_params(curve_params);
        if !cap_check::fits(&caps, &curve) {
            // Bisect this ShapedSegment along its time axis and recurse.
            // This is the policy point — implement bisect_segment in
            // src/cap_check.rs (returns Vec<ShapedSegment>) and dispatch
            // each half through the same closure.
            return dispatch_split(seg, &mcu_configs_for_cb);
        }
    }
}
// ... rest of original closure unchanged
```

The recursion is bounded by max_curve_size; in pathological cases (single piece doesn't fit) `dispatch_split` returns an `Err` that propagates as a planner error.

- [ ] **Step 4: Test passes**

Run: `cd rust && cargo test -p motion-bridge dispatch_splits 2>&1 | tail -10`
Expected: PASS.

- [ ] **Step 5: Run full motion-bridge test suite to confirm no regression**

Run: `cd rust && cargo test -p motion-bridge 2>&1 | tail -15`
Expected: all pass.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs rust/motion-bridge/src/cap_check.rs rust/motion-bridge/tests/sim_motion.rs
git commit -m "bridge: split logical segments whose curves exceed MCU caps"
```

---

## Phase 4 — F446 firmware build and simulator validation

### Task 14: Validate small-profile cross-build locally (forced .config)

**Files:**
- Create (locally, not committed): `.config.f446.test` (an F446 .config snapshot for local cross-builds)

- [ ] **Step 1: Stash the H7 .config and write an F446 one**

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
make olddefconfig 2>&1 | tail -3
grep -E "RUNTIME_TARGET|RUNTIME_MAX_CONTROL_POINTS|RUNTIME_CURVE_POOL_N" out/autoconf.h
```
Expected: `CONFIG_RUNTIME_TARGET_SMALL=1`, `MAX_CONTROL_POINTS 512`, `CURVE_POOL_N 4` — auto-selected by Task 1's `default ... if MACH_STM32F4`.

- [ ] **Step 2: Cross-build F446**

Run: `export PATH=~/opt/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH && make clean && make -j4 2>&1 | tail -8`
Expected: `Creating hex file out/klipper.bin`. Note the `ram` line — should be **well under 100%** (target ≤90% to leave stack headroom).

If RAM overflows: shrink `RUNTIME_CURVE_POOL_N` to 2 in the F446 .config (override the `small` default), `make olddefconfig`, retry. If still overflows, the C-side scratch buffers + Klipper baseline + runtime statics are larger than projected — measure with `arm-none-eabi-nm --size-sort -S out/klipper.elf | awk '$3=="b"||$3=="B"' | sort -k2 | tail -10` and report.

- [ ] **Step 3: Restore H7 .config**

Run: `cp .config.h7.bak .config && make olddefconfig 2>&1 | tail -2`
Expected: `large` profile re-active.

- [ ] **Step 4: No commit** — `.config.h7.bak` and `.config.f446.test` are local working files (already in `.gitignore`).

### Task 15: Run full sim-klippy harness with multi-MCU setup

**Files:**
- Modify: `tools/sim_klippy/printer.cfg` (if needed for multi-MCU sim) — or pass through `run_local.sh`'s arg flow.

- [ ] **Step 1: Inspect existing sim setup**

Run: `cat tools/sim_klippy/printer.cfg | head -40 && echo --- && cat tools/sim_klippy/run_local.sh | head -30`
Expected: Single-MCU sim. If true, the multi-MCU test below requires extending the sim to spawn two `klipper.elf` instances. Skip multi-MCU sim for this task and rely on the unit tests written in Phase 3.

- [ ] **Step 2: Run a single-MCU sim with a Z move to confirm no regression**

Run: `./tools/sim_klippy/run_local.sh "G1 Z5 F600" 2>&1 | tail -20`
Expected: Step counts reported for Z stepper; no errors.

- [ ] **Step 3: Run a long Z move that would exceed F446 cap (uses H7 sim, but exercises the splitting logic if the sim is configured with low caps)**

This step is optional unless step 2 surfaced a regression. Defer multi-MCU sim infrastructure to a follow-up plan.

- [ ] **Step 4: No commit (no source changes).**

---

## Phase 5 — Hardware bring-up on `dderg@trident.local`

> Phases 5 tasks are **interactive** — they require the user's printer powered on. Document the steps; the user runs them when the printer is available.

### Task 16: Create F446 .config on trident via menuconfig

**Files (on trident):**
- Create: `~/klipper/.config.f446.bak`

- [ ] **Step 1: Stop klipper service on trident**

```bash
ssh dderg@trident.local 'sudo systemctl stop klipper'
```

- [ ] **Step 2: Stash H7 .config and run menuconfig for F446**

```bash
ssh dderg@trident.local '
  cd ~/klipper
  cp .config .config.h7.bak
  make menuconfig
'
```

In menuconfig:
- "Micro-controller Architecture" → STMicroelectronics STM32
- "Processor model" → STM32F446
- "Bootloader offset" → 32KiB (matches typical Klipper F446 build)
- "Communication interface" → USB (on PA11/PA12)
- **NOT** any "USB on" or "DFU on initial pin" — F446 doesn't have the H7's CPAP-pin-on-boot quirk; user explicitly warned to verify F446 has no equivalent set
- "Enable kalico Rust runtime (Layer 4 motion planner)" → **Yes**
- "Runtime sizing profile" → **Small** (auto-selected by `default RUNTIME_TARGET_SMALL if MACH_STM32F4`, but verify)
- "Build for Renode simulator" → **No**

Save and exit.

- [ ] **Step 3: Snapshot the F446 config**

```bash
ssh dderg@trident.local 'cd ~/klipper && cp .config .config.f446.bak'
```

- [ ] **Step 4: No commit (host-local config files).**

### Task 17: Build F446 firmware on trident and flash

**Files:** none (deploy step)

- [ ] **Step 1: Build F446 firmware**

```bash
ssh dderg@trident.local '
  cd ~/klipper
  make clean
  make -j4 2>&1 | tail -10
'
```
Expected: `Creating hex file out/klipper.bin`. RAM line shows fits with margin.

- [ ] **Step 2: Find F446 device path**

```bash
ssh dderg@trident.local 'ls /dev/serial/by-id/ | grep stm32f446'
```
Expected: `usb-Klipper_stm32f446xx_2C0036000851313133353932-if00` (or similar — capture the exact name).

- [ ] **Step 3: Flash F446**

```bash
ssh dderg@trident.local '
  cd ~/klipper
  make flash FLASH_DEVICE=/dev/serial/by-id/usb-Klipper_stm32f446xx_<id>-if00 2>&1 | tail -20
'
```
Expected: `File downloaded successfully`. (A trailing `Error during download get_status` is harmless — same harmless dfu-util artifact as the H7 flash.)

- [ ] **Step 4: Restart klipper and check log**

```bash
ssh dderg@trident.local '
  sudo systemctl start klipper
  sleep 5
  tail -80 ~/printer_data/logs/klippy.log
'
```
Expected: both `mcu` (H7) and `bottom` (F446) connect; log shows fork-native bridge handshake on both; `[bridge-trace]` lines for endstops on both MCUs; `MotionToolhead: configure_axes mcu=bottom` log entry confirms F446 is now bridge-attached.

- [ ] **Step 5: Restore H7 .config on trident for the next H7 reflash**

```bash
ssh dderg@trident.local 'cd ~/klipper && cp .config.h7.bak .config && make olddefconfig'
```

### Task 18: Smoke-test G28

**Files:** none (interactive verification)

- [ ] **Step 1: Mark Z homed (so a bare Z move is allowed) and try a tiny Z lift**

In Mainsail console (or via `gcode shell command`):
```
SET_KINEMATIC_POSITION X=0 Y=0 Z=10
G1 Z15 F600
```
Expected: Z stepper rotates 5mm upward. No "MCU shutdown" or "missed steps" in log.

- [ ] **Step 2: Try `G28` (Z lift first, then X/Y home, then Z home)**

```
G28
```
Expected: full homing sequence executes. Z lifts ~5mm, X homes against PG6, Y homes against PG9, Z homes against its endstop (probe or microswitch per printer.cfg). Toolhead lands at home position.

- [ ] **Step 3: Capture klippy.log timestamp range and look for issues**

```bash
ssh dderg@trident.local 'tail -150 ~/printer_data/logs/klippy.log'
```
Expected: no "Lost communication" / "Stepper queue overflow" / "MCU shutdown" errors. Timing stats reasonable.

- [ ] **Step 4: Document outcome**

If G28 works end-to-end: append to `docs/superpowers/handoff/<date>-runtime-sizing-handoff.md` summarizing observed behavior + any quirks.

If G28 fails: capture full klippy.log, observed symptom (which step in homing failed, what error), and feed back into a follow-up debugging plan.

---

## Self-review

**Spec coverage:**
- §4.1 Kconfig profile: Task 1 ✓
- §4.2 C reads from autoconf: Tasks 2-3 ✓
- §4.3 Rust reads via env vars + build.rs: Tasks 4-6 ✓
- §4.4 Cargo features keep narrow purpose: no change needed (existing features remain) ✓
- §5.1 New QueryRuntimeCaps message: Tasks 7-9 ✓
- §5.2 Bridge stores caps: Tasks 10-11 ✓
- §5.3 Cap-check + split: Tasks 12-13 ✓
- §5.4 Capability lifecycle (immutable per connection): implicit in Task 10 (one-time query at bootstrap) ✓
- §6 Numerical defaults: Task 1 (in Kconfig defaults) ✓
- §7.1 H7 build flow: Task 6 verifies, Task 5 confirms unchanged behavior ✓
- §7.2 F446 build flow: Tasks 14, 16 ✓
- §8.1 Cross-build both targets: Tasks 5, 14 ✓
- §8.2 Renode regression: Task 15 (or skipped if multi-MCU sim infra absent) — flagged ✓
- §8.3 Identify-roundtrip test: Task 7 (encode/decode) + Task 10 (mock-transport flow) ✓
- §8.4 Cap-enforcement test: Tasks 12-13 ✓
- §8.5 Live hardware: Tasks 17-18 ✓
- §9 Naming convention: respected throughout — new symbols are `RUNTIME_*` / `KALICO_RUNTIME_*` env vars; existing `kalico_*` symbols untouched ✓
- §10 Open question on schema-hash: addressed in Task 8 — re-deriving the hash is expected, test inline-updated ✓
- §11 References: all spec-listed files are touched in matching tasks ✓

**Type consistency check:**
- `McuCaps` field names (`max_control_points`, `max_knot_vector_len`, `max_degree`, `curve_pool_n`) — same names used in `RuntimeCapsResponse` (Task 7) and `McuCaps` (Task 11) and the Kconfig defaults (Task 1). ✓
- Message kind discriminants `0x0040`, `0x0041` — used in messages.rs (Task 7), schema_def.rs (Task 8), and kalico_dispatch.c (Task 9). ✓
- `cap_check::fits` — defined in Task 12, called in Task 13. ✓

**Placeholders:**
- Task 10's `query_runtime_caps` references "alloc_correlation_id / send_command / recv_response" with a comment saying "adjust to the real host_io API". This is a partial placeholder — flagged as needing the implementer to confirm method names. Not a hard placeholder (the right pattern is shown), but worth a checkpoint.
- Task 13's test scaffolding refers to "study sim_motion.rs to see how segments are constructed" — also partial, since the harness API is non-trivial. Acceptable for a test; the engineer must read existing tests to mirror style.

Both flagged items are inherent to integrating with code I don't have full read of in this session; an executing-plans subagent or interactive engineer will resolve them by reading the referenced files. Not fixable inline without that reading.
