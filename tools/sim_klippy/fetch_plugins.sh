#!/usr/bin/env bash
# Fetch third-party klippy plugins at pinned revs for the faithful sim.
#
# These plugins are NOT vendored in this repo. The sim needs them locally
# for parity with the real printer (printer.cfg references autotune_tmc /
# motor_constants / KAMP / beacon / etc.). Re-running is a no-op once the
# pinned revs are checked out.
#
# To bump a pin: edit the rev in the PLUGINS list below and re-run.
# If a plugin needs local modifications, fork upstream on github and point
# the URL at the fork (the rev pin still applies).

set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
DEST="${1:-$REPO_ROOT/tools/sim_klippy/printer_real/third_party_repos}"

# url | dir name | pinned rev
PLUGINS=(
  "https://github.com/beacon3d/beacon_klipper.git|beacon_klipper|ef987001b85e9cf18cb4029d89d8d1d97dec6cc9"
  "https://github.com/MRX8024/motors-sync.git|motors-sync|4372a220f45454f256974780f7b840bf407ceb44"
  "https://github.com/kyleisah/Klipper-Adaptive-Meshing-Purging.git|Klipper-Adaptive-Meshing-Purging|b0dad8ec9ee31cb644b94e39d4b8a8fb9d6c9ba0"
  "https://github.com/mainsail-crew/mainsail-config.git|mainsail-config|ff3869a621db17ce3ef660adbbd3fa321995ac42"
  "https://github.com/mainsail-crew/moonraker-timelapse.git|moonraker-timelapse|c7fff11e542b95e0e15b8bb1443cea8159ac0274"
  "https://github.com/MRX8024/chopper-resonance-tuner.git|chopper-resonance-tuner|1f98212ca9dbfdf15d516115dd4c26e97b914a8d"
  "https://github.com/andrewmcgr/klipper_tmc_autotune.git|klipper_tmc_autotune|f366d75fa44d177aa6fb002cdff50195e6952772"
)

mkdir -p "$DEST"

for entry in "${PLUGINS[@]}"; do
  IFS='|' read -r url name rev <<< "$entry"
  dir="$DEST/$name"

  if [ -d "$dir/.git" ]; then
    cur="$(git -C "$dir" rev-parse HEAD)"
    if [ "$cur" = "$rev" ]; then
      echo "OK    $name @ ${rev:0:7}"
      continue
    fi
    echo "RESET $name (${cur:0:7} -> ${rev:0:7})"
    git -C "$dir" fetch --quiet origin "$rev" 2>/dev/null \
      || git -C "$dir" fetch --quiet --tags
    git -C "$dir" checkout --quiet --detach "$rev"
  else
    if [ -e "$dir" ]; then
      # Dir exists without a .git — e.g. sim runtime artifacts left behind
      # after a re-fetch. This whole tree is owned by this script (see
      # README); wipe and re-clone.
      echo "WIPE  $name (non-git leftover)"
      rm -rf "$dir"
    fi
    echo "CLONE $name @ ${rev:0:7}"
    git clone --quiet "$url" "$dir"
    git -C "$dir" checkout --quiet --detach "$rev"
  fi
done

echo "done: $DEST"
