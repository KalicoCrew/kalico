#!/bin/bash
# Kalico Simulator — build and run in Docker.
#
# Usage:
#   ./run.sh                          # Homing test on current branch
#   ./run.sh --branch main            # Test specific branch
#   ./run.sh --gcode benchy.gcode     # Print a G-code file
#   ./run.sh --branch sota-motion     # Test sota-motion (should catch bugs)
#
# Multiple instances run in parallel safely (each container has its
# own /dev/shm for the virtual clock).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

BRANCH="HEAD"
GCODE=""
EXTRA_ARGS=""
VERBOSE=""
TAG_SUFFIX=""

while [[ $# -gt 0 ]]; do
    case $1 in
        --branch|-b)
            BRANCH="$2"
            TAG_SUFFIX="-${2//\//-}"
            shift 2
            ;;
        --gcode|-g)
            GCODE="$2"
            shift 2
            ;;
        --verbose|-v)
            VERBOSE="--verbose"
            shift
            ;;
        *)
            EXTRA_ARGS="$EXTRA_ARGS $1"
            shift
            ;;
    esac
done

IMAGE_TAG="kalico-sim${TAG_SUFFIX}"

echo "=== Kalico Simulator ==="
echo "  Branch: $BRANCH"
echo "  Image:  $IMAGE_TAG"
echo "  G-code: ${GCODE:-none (homing test)}"
echo ""

# Build the Docker image
echo "Building Docker image..."
docker build \
    --build-arg "BRANCH=$BRANCH" \
    -t "$IMAGE_TAG" \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$REPO_ROOT"

# Run the simulation
DOCKER_ARGS="--rm"

# Mount G-code file if specified
if [[ -n "$GCODE" ]]; then
    GCODE_ABS="$(cd "$(dirname "$GCODE")" && pwd)/$(basename "$GCODE")"
    DOCKER_ARGS="$DOCKER_ARGS -v $GCODE_ABS:/gcode/$(basename "$GCODE"):ro"
    EXTRA_ARGS="$EXTRA_ARGS --gcode /gcode/$(basename "$GCODE")"
fi

echo "Running simulation..."
docker run $DOCKER_ARGS "$IMAGE_TAG" $VERBOSE $EXTRA_ARGS
