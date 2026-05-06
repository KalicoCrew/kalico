# Per-MCU Runtime Sizing — Design

**Status:** draft, awaiting user review
**Author:** brainstormed 2026-05-06
**Scope:** make the fork-native runtime's curve-pool / scratch-buffer constants build-time-configurable per target MCU, so a single source tree can produce both an H7 firmware (large pool, fits via AXI SRAM) and an F446 firmware (small pool, fits in 128 KB SRAM). Establish the host↔MCU capability handshake so the planner adapts curve sizes per-MCU.

## 1. Problem

The Rust runtime's `CurvePool` is sized for the H723 in `rust/runtime/src/curve_pool.rs`:

- `MAX_CONTROL_POINTS = 1830`
- `MAX_KNOT_VECTOR_LEN = 1850`
- `MAX_DEGREE = 10`
- `CURVE_POOL_N = 16`

The resulting `RuntimeContext` static (`RT_CELL` in `rust/kalico-c-api/src/runtime_ffi.rs`) is **282 KB**. On the H723 it lives in AXI SRAM (newly mapped in the same dev cycle). The F446 has only 128 KB of regular SRAM and no AXI SRAM region — the firmware does not link.

`curve_pool.rs:25` already flagged the work: *"F446 will get a separate build later in Phase D with smaller constants."* This spec is that work.

The Cargo features `mcu-h7` / `mcu-f4` exist in `rust/kalico-c-api/Cargo.toml` but currently differ only in `dep:libm`. They do not drive sizing.

## 2. Goals

1. One source tree, build-config-driven sizing — no fork of the runtime crate per MCU.
2. Klipper Kconfig is the single source of truth for build-time constants. Both Rust and C read from it.
3. Sizing values are configurable per build, not selected from a fixed two-bucket enum. Typical users pick a profile; advanced users override per-constant.
4. Host planner learns each MCU's caps at startup and adapts. No declarative duplication in `printer.cfg`.
5. New code uses unprefixed names (`CONFIG_RUNTIME_*`) — see §9.

## 3. Non-goals

1. Renaming existing `kalico_*` C/Rust symbols, crates, or files. Out of scope; tracked separately as a fork-rename pass.
2. Redesigning the curve-upload wire protocol. The Identify message gains capability fields; framing is unchanged.
3. Changing H7 sizing behavior. Existing values become the default for the `large` profile.
4. Supporting "legacy stepper" backends (queue_step over the old Klipper command stream to non-bridge MCUs). Decided against during brainstorm — F446 will run fork-native firmware; legacy backend would be throwaway scaffolding.

## 4. Architecture

### 4.1 Kconfig is the source

New Kconfig section under `CONFIG_KALICO_RUNTIME` in `src/Kconfig`:

```kconfig
choice
    prompt "Runtime sizing profile"
    default RUNTIME_TARGET_LARGE if MACH_STM32H7
    default RUNTIME_TARGET_SMALL if MACH_STM32F4
    default RUNTIME_TARGET_LARGE
    depends on KALICO_RUNTIME

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
    depends on KALICO_RUNTIME

config RUNTIME_MAX_KNOT_VECTOR_LEN
    int "Max knot-vector length per curve slot"
    default 1850 if RUNTIME_TARGET_LARGE
    default 524 if RUNTIME_TARGET_SMALL
    depends on KALICO_RUNTIME

config RUNTIME_MAX_DEGREE
    int "Max polynomial degree"
    default 10
    depends on KALICO_RUNTIME

config RUNTIME_CURVE_POOL_N
    int "Curve-pool slot count (planner look-ahead depth)"
    default 16 if RUNTIME_TARGET_LARGE
    default 4 if RUNTIME_TARGET_SMALL
    depends on KALICO_RUNTIME
```

Any of `LARGE` / `SMALL` / `CUSTOM` exposes the four `RUNTIME_*` int values; the profile choice just changes their defaults. This is Kconfig idiom for "preset that's tweakable."

### 4.2 C side reads `autoconf.h` directly

After `make olddefconfig`, `out/autoconf.h` will contain `#define CONFIG_RUNTIME_MAX_CONTROL_POINTS 512` etc. C statics that today hardcode 1830/1850 in `src/runtime_tick.c` change to:

```c
float kalico_aligned_cps[CONFIG_RUNTIME_MAX_CONTROL_POINTS];
float kalico_aligned_knots[CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN];
```

(Note: the symbol names retain the legacy `kalico_` prefix; only the *configuration values* use the new `RUNTIME_` namespace. Consistent with §9.)

`src/kalico_demux.c`'s `KALICO_DEMUX_KALICO_BUF_SIZE` becomes derived: `(CONFIG_RUNTIME_MAX_CONTROL_POINTS + CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN) * 4 + 32` (frame headers/CRC), with `8192` removed entirely. This keeps the demux buf right-sized for whatever profile is active.

### 4.3 Rust side reads via env vars + `build.rs`

Klipper's `src/Makefile` already invokes cargo with feature flags. Extend it to export the same Kconfig values as env vars:

```make
cd rust && \
    KALICO_RUNTIME_MAX_CONTROL_POINTS=$(CONFIG_RUNTIME_MAX_CONTROL_POINTS) \
    KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN=$(CONFIG_RUNTIME_MAX_KNOT_VECTOR_LEN) \
    KALICO_RUNTIME_MAX_DEGREE=$(CONFIG_RUNTIME_MAX_DEGREE) \
    KALICO_RUNTIME_CURVE_POOL_N=$(CONFIG_RUNTIME_CURVE_POOL_N) \
    cargo build -p kalico-c-api ...
```

A new `rust/runtime/build.rs` reads those env vars and emits a generated module:

```rust
// build.rs
fn main() {
    for var in ["KALICO_RUNTIME_MAX_CONTROL_POINTS",
                "KALICO_RUNTIME_MAX_KNOT_VECTOR_LEN",
                "KALICO_RUNTIME_MAX_DEGREE",
                "KALICO_RUNTIME_CURVE_POOL_N"] {
        println!("cargo:rerun-if-env-changed={var}");
    }
    let mcp = env_or_default("KALICO_RUNTIME_MAX_CONTROL_POINTS", "1830");
    // ... (same pattern for the other three)
    let out_dir = std::env::var("OUT_DIR").unwrap();
    std::fs::write(format!("{out_dir}/sizing.rs"), format!(
        "pub const MAX_CONTROL_POINTS: usize = {mcp};\n\
         pub const MAX_KNOT_VECTOR_LEN: usize = {mkv};\n\
         pub const MAX_DEGREE: u8 = {mdg};\n\
         pub const CURVE_POOL_N: usize = {cpn};\n"
    )).unwrap();
}
```

`curve_pool.rs` then drops its hardcoded constants and `include!`s the generated file:

```rust
include!(concat!(env!("OUT_DIR"), "/sizing.rs"));
```

The existing knot-rule assert (`MAX_KNOT_VECTOR_LEN >= MAX_CONTROL_POINTS + MAX_DEGREE + 1`) stays — `const` asserts at the use site catch invalid combinations at compile time.

Defaults inside `build.rs` match the H7 `large` profile so a host-only test build (no Klipper Makefile in the loop) keeps working.

### 4.4 Existing `mcu-h7` / `mcu-f4` Cargo features

These currently differ only in `dep:libm` (both pull it). After this change they remain — their narrow current purpose (target-specific math choices, future DSP intrinsics) is preserved. They no longer claim sizing responsibility. Comments in `rust/kalico-c-api/Cargo.toml` and `rust/runtime/Cargo.toml` get updated to reflect the split: features = target-arch concerns, env vars = sizing.

## 5. Host↔MCU capability handshake

### 5.1 Identify response gains capability fields

The fork-native Identify response in `rust/kalico-protocol/src/messages.rs` is extended with four new fields:

```rust
pub struct IdentifyResponse {
    // ... existing fields ...
    pub max_control_points: u32,
    pub max_knot_vector_len: u32,
    pub max_degree: u8,
    pub curve_pool_n: u16,
}
```

The MCU populates these from `CONFIG_RUNTIME_*` (via `autoconf.h` constants reachable in `src/kalico_dispatch.c::handle_identify`).

Wire-format addition: append the four fields to the existing Identify body. Trailing-field append is the protocol's evolution rule per `docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md`; a host built against the old layout simply stops reading after the original fields, while a host built against the new layout reads the trailing four. Confirm during implementation whether the schema-hash mechanism still considers this a breaking change requiring a version bump or accepts trailing-append as compatible.

### 5.2 Host stores caps per-MCU

`rust/motion-bridge/src/bridge.rs` already maintains a per-MCU `mcu_configs` map (used by `dispatch_ios.get(&plan.mcu_id)` at `bridge.rs:1089`). The Identify response is consumed in the bridge's bootstrap path; extend to record `McuCaps { max_control_points, max_knot_vector_len, max_degree, curve_pool_n }` per MCU.

### 5.3 Planner respects per-MCU caps

When `build_push_params` (`bridge.rs:1086`) constructs per-axis curve plans, it consults the destination MCU's caps. If a planned curve exceeds `max_control_points` (or violates the knot rule), the bridge splits the logical move into two or more shaped segments — each producing a curve that fits.

Splitting strategy: bisect the logical move along the time axis (or along arclength — equivalent for monotone parameterizations) and re-plan each half through the existing trajectory pipeline. The bridge already produces multiple `ShapedSegment`s for a single G-code move when adaptive grid demands it; the addition is a per-MCU cap check at the dispatch boundary.

Out-of-band cases: if a single piece (degree-`d` polynomial) cannot satisfy the cap, that's a configuration error (cap is below minimum viable size — `n_cps >= MAX_DEGREE + 1`); bridge logs an error and the plan fails fast at build time, before any TX. The Kconfig `int` ranges should enforce a floor (e.g., `range 16 4096` on `RUNTIME_MAX_CONTROL_POINTS`) to make the misconfiguration unreachable.

### 5.4 Capability lifecycle

Caps are read once at Identify time and immutable for the connection lifetime. If the MCU is reflashed with different sizing, klippy disconnects/reconnects → new Identify → new caps. No live re-negotiation needed.

## 6. Numerical profile defaults

Justification derived in the brainstorm via `kalico-verifier` subagent on 2026-05-06. Summary:

| Profile  | MAX_CONTROL_POINTS | MAX_KNOT_VECTOR_LEN | MAX_DEGREE | CURVE_POOL_N | Per-slot | Pool total |
|----------|-------------------:|--------------------:|-----------:|-------------:|---------:|-----------:|
| `large` (H7) |              1830 |                1850 |         10 |           16 |  14.7 KB |    232 KB |
| `small` (F4) |               512 |                 524 |         10 |            4 |   4.1 KB |     16 KB |

Worst-case-curve sizing rule: `n_cps_max = piece_count × refit_degree + 1`, where `piece_count = ceil(arclength / 0.5mm)` and the post-fit refit produces degree-4 pieces. The `small` profile covers any single uncapped move up to ~64 mm; longer moves (e.g., a 250 mm Z home) must be split upstream per §5.3 — already a policy item independent of this sizing work.

`MAX_DEGREE=10` stays in both profiles: pre-refit the planner emits degree-≤10 curves (composition of degree-2 `s(t)` with degree-≤5 `x(s)`), and the `split_without_refit` path retains them at full degree.

## 7. Build flow

### 7.1 H723 build

```
make menuconfig                        # KALICO_RUNTIME=y, RUNTIME_TARGET_LARGE auto-selected
make                                   # Cargo gets KALICO_RUNTIME_MAX_CONTROL_POINTS=1830 etc.
                                       # Rust runtime constants = large profile
                                       # C scratch sized to 1830/1850
                                       # RT_CELL placed in AXI SRAM (already wired)
```

### 7.2 F446 build

Separate `.config` file (since each MCU has its own). Menuconfig:

```
CONFIG_MACH_STM32F446=y                # selects MACH_STM32F4 family
CONFIG_KALICO_RUNTIME=y
                                       # RUNTIME_TARGET_SMALL auto-selected
                                       # USB CDC, no DFU pin gotchas (per user warning re: H7 boot pin)
```

`make` produces F446 firmware with small profile. RT_CELL is ~16 KB plus rest of RuntimeContext — fits in regular SRAM. `kalico_console_write_raw` shim in `src/generic/usb_cdc.c` (added in the same dev cycle) is gated `CONFIG_KALICO_RUNTIME` and works on both H7 and F446 unchanged.

### 7.3 Workflow on the test host

The user's `dderg@trident.local` build host keeps two `.config` snapshots: `.config` (currently H7) and a new `.config.f446.bak`. Reflashing alternates between them; the working `.config` carries whichever is being built at the moment.

## 8. Validation

1. **Cross-build both targets locally.** Mac toolchain + each `.config` produces an ELF that fits its target's RAM. `arm-none-eabi-size` confirms `.bss` totals.
2. **Renode sim regression.** Existing Renode harness is H7 sim. Continue running with `RUNTIME_TARGET_LARGE`. F446 sim coverage is out of scope (no F446 Renode platform today).
3. **Identify-roundtrip unit test.** Add a Rust integration test in `rust/kalico-host-rt/tests/` that decodes an Identify response with the new fields and verifies the host stores them per-MCU.
4. **Per-MCU cap-enforcement test.** Construct a synthetic plan whose curve exceeds a low (e.g., `max_control_points=64`) cap and assert the bridge splits it; assert no over-cap LoadCurve frame is emitted.
5. **Live hardware once F446 is flashed.** `G28` exercises Z (small move, well within `small` cap) and surfaces both the hardware-pipeline and the per-MCU dispatch code paths.

## 9. Naming convention

Brainstormed and decided 2026-05-06: new fork-native code uses **no project prefix**. Configuration and identifiers introduced by this spec carry the `RUNTIME_*` namespace (the subsystem name) rather than `KALICO_*` or any future fork brand:

- Kconfig: `CONFIG_RUNTIME_TARGET_LARGE`, `CONFIG_RUNTIME_MAX_CONTROL_POINTS`, etc.
- Rust env vars used by `runtime/build.rs`: `KALICO_RUNTIME_*` (env-var names retain the legacy `KALICO_` prefix purely to keep the cargo-build-input namespace distinct from anything the user shell or cargo itself might define; the constants the build script emits are unprefixed within the `runtime` crate: `MAX_CONTROL_POINTS`, `MAX_KNOT_VECTOR_LEN`, etc.).
- New C symbols (none introduced by this spec, but if any: `runtime_*` or just topic-named).

Existing `kalico_*` symbols (crate names, file names like `src/kalico_dispatch.c`, the `CONFIG_KALICO_RUNTIME` master switch, FFI prefix `kalico_`) are **not renamed by this spec.** They remain until a coordinated fork-rename pass. Mixing prefixes in the codebase during the transition is acceptable per the brainstorm.

## 10. Open questions

None blocking. Two minor items deferred to implementation:

1. **Identify schema-hash bump:** the kalico-native protocol's schema-hash mechanism may force a host-side guard against old firmware. Confirm whether trailing-field append truly compiles under the existing schema-hash policy or whether a version bump is required. Pin during implementation.
2. **Bridge-side split implementation detail:** the existing trajectory-layer pipeline already supports multi-segment plans for one logical G-code move; the per-MCU cap check needs to live at the bridge boundary, not inside the planner (the planner is host-CPU-bound and shouldn't know about MCU sizes). Plumbing point: `bridge.rs::dispatch` callback at line ~1086.

## 11. References

- `rust/runtime/src/curve_pool.rs:25-43` — current sizing constants, Phase-D comment.
- `rust/kalico-c-api/Cargo.toml` — existing `mcu-h7` / `mcu-f4` features.
- `rust/motion-bridge/src/bridge.rs:1077-1089` — per-MCU dispatch entry point.
- `docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md` — fork-native wire protocol (Identify, LoadCurve framing).
- `docs/superpowers/specs/2026-05-04-incremental-curve-upload-design.md` — chunked upload protocol.
- Brainstorm transcript 2026-05-06 (this session) — verifier-derived sizing math.
