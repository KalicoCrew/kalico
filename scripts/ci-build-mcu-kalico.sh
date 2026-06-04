#!/bin/bash
# Full MCU firmware build (C compile + Rust staticlib + link) for every
# kalico-specific board config under test/configs/kalico-*.config.
#
# WHY THIS EXISTS
# ---------------
# Our other CI (ci-rust-runtime.yaml) only runs `cargo build` on the Rust
# staticlib (libkalico_c_api.a). It never *links* the C firmware against it.
# C<->Rust FFI drift therefore goes undetected: e.g. runtime_tick_g0.c kept
# calling kalico_runtime_count_modulated_steppers after that symbol was
# removed from the c-api — the staticlib built fine, but `make` (the C link)
# failed. Nobody noticed until a hardware flash, because no CI ran `make`
# per arch. This script does exactly that: a missing/renamed FFI symbol makes
# arm-none-eabi-ld fail, which fails the build, which fails CI.
#
# Mirrors mainline Klipper's scripts/ci-build.sh (cp config -> olddefconfig
# -> make), restricted to the configs that enable the kalico Rust runtime
# and adding the Rust-target + arm-none-eabi requirements.
#
# Requirements: arm-none-eabi-gcc, and the rustup targets thumbv7em-none-eabi
# (H7/F4) + thumbv6m-none-eabi (G0). See .github/workflows/ci-mcu-firmware.yaml.
#
# Usage: ./scripts/ci-build-mcu-kalico.sh

set -euo pipefail

MAIN_DIR="${PWD}"
DICTDIR="${DICTDIR:-${MAIN_DIR}/out/ci-mcu-kalico-dicts}"
mkdir -p "${DICTDIR}"

shopt -s nullglob
CONFIGS=(test/configs/kalico-*.config)
if [ ${#CONFIGS[@]} -eq 0 ]; then
    echo "ERROR: no test/configs/kalico-*.config found" >&2
    exit 1
fi

for TARGET in "${CONFIGS[@]}"; do
    NAME="$(basename "${TARGET}" .config)"

    # MACH_LINUX is a NATIVE build (gcc, host Rust target) — not an
    # arm-none-eabi cross-compile. It is built and link-checked by the separate
    # `linux-mcu` CI job. Skip it here so this cross-compile job stays ARM-only.
    if grep -qxF 'CONFIG_MACH_LINUX=y' "${TARGET}"; then
        echo "=============== skip ${NAME} (native MACH_LINUX — see linux-mcu job)"
        continue
    fi

    echo "::group::=============== build ${NAME}"

    make clean
    make distclean
    unset CC

    cp "${TARGET}" .config
    make olddefconfig

    # Full firmware build: C objects + cargo (libkalico_c_api.a) + link.
    # An undefined Rust FFI symbol fails the link here.
    make V=1 -j"$(nproc)"

    size out/klipper.elf
    # `make distclean` above wipes out/ (including DICTDIR, which lives under
    # out/), so recreate it each iteration before stashing the dict.
    mkdir -p "${DICTDIR}"
    cp out/klipper.dict "${DICTDIR}/${NAME}.dict"

    echo "=============== ok ${NAME}"
    echo "::endgroup::"
done

make clean
make distclean
echo "All kalico MCU firmware targets built + linked OK."
