#!/usr/bin/env bash
# Build a production H723 firmware image at out/klipper.elf + out/klipper.bin.
#
# Differences from the Renode sim image (tools/sim/build_sim_firmware.sh):
#   - CONFIG_STM32_USB_PA11_PA12=y (USB-CDC over PA11/PA12)
#   - CONFIG_STM32_SERIAL_USART2 not set
#   - CONFIG_KALICO_SIM not set (IWDG armed; real watchdog kicks active)
#
# Flash the resulting out/klipper.bin via USB DFU or SD card per the
# Octopus Pro flashing guide.  The firmware expects a UF2/DFU bootloader
# at 0x8000000 and the application starting at 0x8020000.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$REPO_ROOT"

# Locate arm-gcc (same logic as build_sim_firmware.sh).
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
EOF
  exit 1
fi

# Apply the production config and reconcile against current Kconfig.
cp tools/h723_production.config .config
make olddefconfig >/dev/null

# Clean rebuild — switching SERIAL/USB orientation invalidates many objects.
make clean >/dev/null
make -j"$(sysctl -n hw.ncpu 2>/dev/null || echo 4)"

echo
echo "Built production firmware:"
ls -lh out/klipper.elf out/klipper.bin
echo
echo "Flash to Octopus Pro via DFU:"
echo "  dfu-util -d 0483:df11 -a 0 -s 0x8020000:leave -D out/klipper.bin"
