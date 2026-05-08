#!/usr/bin/env bash
# Build + run the klippy-in-loop sim entirely in Docker on macOS.
# Mounts the repo into the container, builds klipper.elf and the bridge
# .so, then runs the sim harness. No Pi or remote machine needed.
#
#   ./tools/sim_klippy/run_local.sh "G28 X"
#
# First run takes ~5 min (image build + cargo first-build). Cached after.
set -euo pipefail

REPO="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
IMG="kalico-sim:latest"
CONTAINER_HOME=/work
SCRIPT_ARGS="${*:-G28 X}"

docker build -q -t "$IMG" -f "$REPO/tools/sim_klippy/Dockerfile" "$REPO/tools/sim_klippy" >/dev/null

# Build firmware + bridge .so + run the harness in one container invocation.
# --tmpfs /tmp keeps the unix socket and PTY symlinks ephemeral.
docker run --rm -i \
    -v "$REPO":$CONTAINER_HOME \
    -w $CONTAINER_HOME \
    --tmpfs /tmp:exec \
    "$IMG" \
    bash -c "set -e
      cp .config.linux .config
      make olddefconfig >/dev/null
      make -j\$(nproc) 2>&1 | tail -5
      make -f Makefile.kalico motion-bridge 2>&1 | tail -3
      # macOS-host dev keeps a Mach-O c_helper.so in the tree; delete it
      # unconditionally so klippy/chelper rebuilds it for this container's
      # ELF environment. Cheap rebuild (~2s) and avoids architecture skew.
      rm -f klippy/chelper/c_helper.so klippy/chelper/c_helper.so.dSYM 2>/dev/null
      rm -rf klippy/chelper/c_helper.so.dSYM 2>/dev/null || true
      # Remove any stale misnamed motion_bridge.so that shadows motion_bridge.py.
      # The correct native module is always motion_bridge_native.so (built above).
      rm -f klippy/motion_bridge.so 2>/dev/null || true
      mkdir -p /work/tools/sim_klippy/.local-logs
      python3 tools/sim_klippy/run.py $SCRIPT_ARGS"
