# Kalico Rust Workspace

First-party Rust code for the kalico motion stack rewrite. See `docs/superpowers/specs/2026-04-26-nurbs-evaluation-library-design.md` for the design context.

## Layout

- `nurbs/` — Layer 0 mathematical foundations (NURBS eval, arc-length, algebra).
- `kalico-c-api/` — umbrella staticlib + cbindgen FFI surface for kalico's Rust crates. cbindgen-generated header at `kalico-c-api/include/kalico_nurbs.h` (checked in).

## Build

Host (default — for tests, linting, host-side use):

    cargo build
    cargo test

MCU (H723 = Cortex-M7 with double-precision FPU):

    cargo build --release --no-default-features --features mcu-h7 --target thumbv7em-none-eabi

The Klipper Make build picks up the resulting staticlib at `target/thumbv7em-none-eabi/release/libkalico_c_api.a` and the C header at `kalico-c-api/include/kalico_nurbs.h`.

## Toolchain

Pinned via `rust-toolchain.toml`. Update intentionally with regression testing — embedded codegen is sensitive to compiler version. FPU flag strings in `.cargo/config.toml` may need to track LLVM target-feature renames across toolchain versions; verify on bumps.

## C link contract

- C side `#include`s `kalico-c-api/include/kalico_nurbs.h` (committed; CI verifies regen is a no-op).
- C side links against `libkalico_c_api.a`.
- All C symbols are namespaced `kalico_nurbs_*`.
- Type ownership: C never frees Rust-allocated memory; constructors/destructors come in pairs across the FFI boundary. Pointer types are opaque to C.
