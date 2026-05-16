#!/usr/bin/env bash
# Build a Renode-friendly firmware image at out/klipper.elf.
#
# Differences from the production silicon image:
#   - CONFIG_STM32_SERIAL_USART2=y instead of CONFIG_USBSERIAL=y (Renode's
#     OTG_HS isn't modeled; USART2 is).
#   - CONFIG_KALICO_SIM=y to skip watchdog_init / kicks (Renode's IWDG
#     model fires spurious resets).
#
# Both options are sim-only. Never flash an out/klipper.bin produced by
# this script to real hardware — leaving IWDG armed is the only thing
# that catches a hung MCU mid-print.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

# Locate arm-gcc. Prefer the user's xpack install since brew's
# arm-none-eabi-gcc formula ships without newlib (no <stdint.h>); the
# cask `gcc-arm-embedded` requires sudo to install. xpack-dev-tools
# ships a working bundle that extracts under $HOME with no privileges.
XPACK_DIR="$HOME/.local/arm-gcc"
XPACK_BIN="$(/bin/ls -d "$XPACK_DIR"/xpack-arm-none-eabi-gcc-*/bin 2>/dev/null | head -n1 || true)"

if [[ -n "${XPACK_BIN}" && -x "${XPACK_BIN}/arm-none-eabi-gcc" ]]; then
  export PATH="${XPACK_BIN}:$HOME/.cargo/bin:/opt/homebrew/bin:$PATH"
elif command -v arm-none-eabi-gcc >/dev/null 2>&1; then
  export PATH="$HOME/.cargo/bin:/opt/homebrew/bin:$PATH"
else
  cat >&2 <<EOF
error: arm-none-eabi-gcc not found.

Recommended (no sudo):
  curl -sL https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v14.2.1-1.1/xpack-arm-none-eabi-gcc-14.2.1-1.1-darwin-arm64.tar.gz \\
    -o /tmp/arm-gcc.tar.gz
  mkdir -p ~/.local/arm-gcc && tar xzf /tmp/arm-gcc.tar.gz -C ~/.local/arm-gcc

Or (requires sudo):
  brew install --cask gcc-arm-embedded
EOF
  exit 1
fi

# Apply the saved sim config and reconcile against current Kconfig.
cp tools/sim/sim.config .config
make olddefconfig >/dev/null

# CONFIG_STACK_SIZE is declared in Kconfig without a user prompt, so
# `make olddefconfig` clobbers any override from sim.config back to the
# 4096 default. Force the value back after olddefconfig, then touch
# .config and remove generated autoconf headers so the next make picks
# up the new value.
if grep -q '^CONFIG_STACK_SIZE=' tools/sim/sim.config; then
  STACK_OVERRIDE=$(grep '^CONFIG_STACK_SIZE=' tools/sim/sim.config | head -n1)
  sed -i.bak "s|^CONFIG_STACK_SIZE=.*|${STACK_OVERRIDE}|" .config
  rm -f .config.bak
  touch .config
  rm -f out/autoconf.h
fi

# Clean rebuild — switching SERIAL/USB orientation invalidates a lot of objects.
make clean >/dev/null
make -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"

echo
echo "Built sim firmware:"
ls -lh out/klipper.elf out/klipper.bin
