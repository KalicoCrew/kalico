#!/bin/bash
# Run the homing-lag e2e test in Docker.
#
# First run: builds the image (~6 min for firmware + Rust).
# Subsequent runs with only Python/script changes: instant (volume-mounted).
#
# Usage:
#   bash tools/sim/run_homing_test.sh          # run test
#   bash tools/sim/run_homing_test.sh --build   # force image rebuild
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
IMAGE="kalico-homing-test"

if [[ "${1:-}" == "--build" ]] || ! docker image inspect "$IMAGE" &>/dev/null; then
    echo "Building Docker image (this takes ~6 min the first time)..."
    docker build -f "$REPO_ROOT/tools/sim/Dockerfile.homing-test" \
        -t "$IMAGE" "$REPO_ROOT"
fi

# Volume-mount Python sources + test scripts so changes don't need a
# rebuild. chelper is NOT mounted — the image has its own Linux build.
# motion_bridge_native.so is also baked into the image.
exec docker run --rm \
    -v "$REPO_ROOT/klippy/extras:/work/klippy/extras" \
    -v "$REPO_ROOT/klippy/kinematics:/work/klippy/kinematics" \
    -v "$REPO_ROOT/klippy/motion_toolhead.py:/work/klippy/motion_toolhead.py" \
    -v "$REPO_ROOT/klippy/motion_bridge.py:/work/klippy/motion_bridge.py" \
    -v "$REPO_ROOT/klippy/mcu.py:/work/klippy/mcu.py" \
    -v "$REPO_ROOT/klippy/serialhdl.py:/work/klippy/serialhdl.py" \
    -v "$REPO_ROOT/klippy/stepper.py:/work/klippy/stepper.py" \
    -v "$REPO_ROOT/klippy/pins.py:/work/klippy/pins.py" \
    -v "$REPO_ROOT/tools/sim/test_homing_lag.py:/work/tools/sim/test_homing_lag.py" \
    -v "$REPO_ROOT/tools/sim/docker_homing_test.sh:/work/tools/sim/docker_homing_test.sh" \
    -v "$REPO_ROOT/tools/sim/h723_sim_docker.resc:/work/tools/sim/h723_sim_docker.resc" \
    -v "$REPO_ROOT/tools/sim/dual_mcu_docker.resc:/work/tools/sim/dual_mcu_docker.resc" \
    "$IMAGE"
