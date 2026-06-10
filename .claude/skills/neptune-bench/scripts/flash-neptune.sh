#!/usr/bin/env bash
# flash-neptune.sh — pull, build, and flash the Neptune 3 Pro bench F401 from the Mac.
#
# Orchestrates the whole bench reflash:
#   pull the branch on the Pi -> build F401 firmware -> hold klippy down so PA13
#   stays SWDIO -> power-cycle (HomeKit Plug 2) for a fresh boot -> openocd flash over
#   ST-Link (NRST disconnected, software reset) -> restore auto-restart + bring klippy up.
#
# Runs from the Mac: SSH does the Pi work; `shortcuts` toggles the smart plug.
# Idempotent: always pulls and flashes; restart-suppression and udev disable handle
# being already-applied; a trap restores the bench even if a step fails.
#
# Usage: ./flash-neptune.sh <branch>
set -euo pipefail

BRANCH=${1:-}
[ -n "$BRANCH" ] || { echo "usage: $0 <branch>" >&2; exit 2; }

PI=dderg@ethercatpi5.local
REPO='$HOME/kalico'
PW=password
PLUG_OFF='Plug 2 OFF'
PLUG_ON='Plug 2 ON'
TTY=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
APP_ADDR=0x8008000

say() { printf '\n\033[1;36m[flash-neptune]\033[0m %s\n' "$*"; }
die() { printf '\n\033[1;31m[flash-neptune] FATAL:\033[0m %s\n' "$*" >&2; exit 1; }

# Run a command on the Pi. Stdin is forwarded so heredocs work.
pi() { ssh "$PI" "$@"; }
# Run a sudo command on the Pi, feeding the password to `sudo -S`.
pisudo() { ssh "$PI" "echo '$PW' | sudo -S -p '' bash -c '$1'"; }

restored=0
restore_bench() {
  [ "$restored" = 1 ] && return 0
  restored=1
  say "Restoring auto-restart + bringing klippy back up..."
  pisudo '
    if [ -e /etc/udev/rules.d/99-klipper-mcu-autorestart.rules.disabled ]; then
      mv /etc/udev/rules.d/99-klipper-mcu-autorestart.rules.disabled \
         /etc/udev/rules.d/99-klipper-mcu-autorestart.rules
    fi
    rm -f /etc/systemd/system/klipper.service.d/norestart.conf
    systemctl daemon-reload
    udevadm control --reload-rules
    systemctl start moonraker klipper
  ' || echo "[flash-neptune] WARN: restore step reported an error; check the bench."
}

# ---------------------------------------------------------------------------
# 0. Local reminder: the Pi pulls origin/$BRANCH, so surface anything on this
#    Mac worktree that won't make it onto the board (no push happens here).
# ---------------------------------------------------------------------------
if git rev-parse --abbrev-ref "$BRANCH" >/dev/null 2>&1; then
  ahead=$(git rev-list --count "origin/$BRANCH..$BRANCH" 2>/dev/null || echo '?')
  # grep -c exits 1 on a clean tree; without `|| true` that kills the script
  # under `set -e`/`pipefail` before anything prints.
  dirty=$(git status --porcelain 2>/dev/null | grep -c . || true)
  say "Reminder: $ahead unpushed commit(s) on '$BRANCH', and $dirty uncommitted file(s) locally."
  say "The Pi flashes origin/$BRANCH — push first if you want local work on the board."
fi

# ---------------------------------------------------------------------------
# 1. Pull + build on the Pi: F401 firmware, the host-side pyo3 cdylib, AND the
#    EtherCAT endpoint. klippy imports klippy/motion_bridge_native.so — a stale
#    cdylib against new klippy Python crashes at runtime (AttributeError on new
#    bridge methods). klippy also spawns rust/target/release/kalico-ethercat-rt —
#    a stale endpoint silently drops wire messages it predates (host-side
#    transport timeout instead of an error).
# ---------------------------------------------------------------------------
say "Pulling $BRANCH and building F401 firmware + motion-bridge cdylib + ethercat endpoint on the Pi..."
pi bash -se <<EOF || die "pull/build failed"
set -euo pipefail
cd $REPO
git fetch origin $BRANCH
git checkout $BRANCH
git pull --ff-only
grep -q '^CONFIG_MACH_STM32F401=y' .config || { echo "ERROR: .config is not an F401 build"; exit 1; }
make -j\$(nproc)
test -f out/klipper.bin || { echo "ERROR: out/klipper.bin missing after build"; exit 1; }
ls -l out/klipper.bin
make -f Makefile.kalico motion-bridge -j\$(nproc)
test -f klippy/motion_bridge_native.so || { echo "ERROR: motion_bridge_native.so missing after build"; exit 1; }
ls -l klippy/motion_bridge_native.so
make -f Makefile.kalico ethercat-endpoint-hw
test -f rust/target/release/kalico-ethercat-rt || { echo "ERROR: kalico-ethercat-rt missing after build"; exit 1; }
ls -l rust/target/release/kalico-ethercat-rt
EOF

say "Re-applying capabilities to the EtherCAT endpoint (raw sockets, RT prio)..."
pisudo "setcap cap_net_raw,cap_sys_nice,cap_ipc_lock+ep ~dderg/kalico/rust/target/release/kalico-ethercat-rt"

# ---------------------------------------------------------------------------
# 2. Hold klippy down (Restart=no drop-in + disable the CH340 udev autostart).
#    From here on, restore_bench MUST run no matter how we exit.
# ---------------------------------------------------------------------------
trap restore_bench EXIT
say "Stopping klippy/moonraker and suppressing auto-restart..."
pisudo '
  systemctl stop klipper moonraker
  printf "[Service]\nRestart=no\n" > /etc/systemd/system/klipper.service.d/norestart.conf
  systemctl daemon-reload
  if [ -e /etc/udev/rules.d/99-klipper-mcu-autorestart.rules ]; then
    mv /etc/udev/rules.d/99-klipper-mcu-autorestart.rules \
       /etc/udev/rules.d/99-klipper-mcu-autorestart.rules.disabled
  fi
  udevadm control --reload-rules
'

# ---------------------------------------------------------------------------
# 3. Power-cycle for a fresh boot — PA13 stays SWDIO while no host is attached.
# ---------------------------------------------------------------------------
say "Power-cycling the printer (HomeKit Plug 2)..."
shortcuts run "$PLUG_OFF" >/dev/null
sleep 3
shortcuts run "$PLUG_ON" >/dev/null

say "Waiting for the board to enumerate (CH340 tty)..."
booted=0
for _ in $(seq 1 30); do
  if pi "test -e $TTY" 2>/dev/null; then booted=1; break; fi
  sleep 1
done
[ "$booted" = 1 ] || die "board did not enumerate after power-on (klippy held down, bench will be restored)"
sleep 2   # let the post-boot window settle before grabbing SWD

# ---------------------------------------------------------------------------
# 4. Flash over ST-Link: software reset (NRST disconnected), reset halt to catch
#    the core at the reset vector, write+verify the app at 0x8008000.
# ---------------------------------------------------------------------------
say "Flashing firmware via openocd (ST-Link, software reset)..."
pi bash -se <<EOF || die "openocd flash failed (bench will be restored)"
set -euo pipefail
cd $REPO
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "reset_config none" -c "init" -c "reset halt" \
  -c "flash write_image erase out/klipper.bin $APP_ADDR" \
  -c "verify_image out/klipper.bin $APP_ADDR" \
  -c "reset run" -c "shutdown"
EOF
say "Flash + verify OK."

# ---------------------------------------------------------------------------
# 5. Restore the bench (trap also covers failure paths) and confirm klippy ready.
# ---------------------------------------------------------------------------
restore_bench
trap - EXIT

say "Waiting for klippy to reach 'ready'..."
state=
for _ in $(seq 1 30); do
  state=$(pi "curl -s http://127.0.0.1:7125/printer/info" 2>/dev/null \
    | python3 -c 'import sys,json;print(json.load(sys.stdin)["result"]["state"])' 2>/dev/null || true)
  [ "$state" = ready ] && break
  sleep 2
done
if [ "$state" = ready ]; then
  say "DONE — F401 flashed and klippy is ready."
else
  die "flashed OK but klippy state is '${state:-unknown}', not ready — check the bench."
fi
