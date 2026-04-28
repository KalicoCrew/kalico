#!/usr/bin/env bash
# Launch Renode with the kalico-h723-sim machine.
#
# Prereqs:
#   - renode (brew install renode) on PATH
#   - A simulator-mode firmware build at out/klipper.elf with
#     CONFIG_STM32_SERIAL_USART2=y and CONFIG_KALICO_RUNTIME=y. See
#     tools/sim/sim.config and tools/sim/build_sim_firmware.sh.
#
# After launch, USART2 is available at tcp localhost:3334. Host tools talk
# to it via `socket://localhost:3334` — see tools/sim/test_first_light_sim.sh.
#
# Pass --gui to drop --disable-gui (useful for the Renode monitor window).
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

if [[ ! -f out/klipper.elf ]]; then
  echo "error: out/klipper.elf not found." >&2
  echo "       Build the simulator firmware first: tools/sim/build_sim_firmware.sh" >&2
  exit 1
fi

GUI_FLAGS=(--disable-gui)
if [[ "${1:-}" == "--gui" ]]; then
  GUI_FLAGS=()
fi

# --console reads stdin for the monitor; if the parent backgrounds us with
# stdin redirected, the monitor closes and Renode exits. Pipe a long-running
# /dev/zero into stdin so headless backgrounding keeps the sim alive.
exec renode --console "${GUI_FLAGS[@]}" \
  -e "include @${REPO_ROOT}/tools/sim/h723_sim.resc" \
  -e 'logLevel 3 sysbus' \
  -e 'logLevel 3 rcc' \
  -e 'logLevel 3 nvic' \
  -e 'logLevel 3 usart2' \
  -e 'start' \
  < /dev/zero
