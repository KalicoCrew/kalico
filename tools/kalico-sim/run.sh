#!/bin/bash
# Kalico Simulator — build and run in Docker.
#
# Usage:
#   ./run.sh                          # Test current branch (main)
#   ./run.sh --branch sota-motion     # Test a specific branch
#   ./run.sh --gcode benchy.gcode     # Print a G-code file
#   ./run.sh --privileged             # Enable SCHED_FIFO for homing
#
# The script creates a clean build context from the target branch,
# overlays the simulator tools, builds the Docker image, and runs it.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Find the actual git directory (handle worktrees)
GIT_DIR="$(cd "$REPO_ROOT" && git rev-parse --git-common-dir 2>/dev/null || echo "$REPO_ROOT/.git")"
MAIN_REPO="$(cd "$GIT_DIR/.." 2>/dev/null && pwd || echo "$REPO_ROOT")"

BRANCH=""
GCODE=""
EXTRA_ARGS=""
DOCKER_ARGS="--rm"
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
        --privileged)
            DOCKER_ARGS="$DOCKER_ARGS --privileged"
            shift
            ;;
        --verbose|-v)
            EXTRA_ARGS="$EXTRA_ARGS --verbose"
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
echo "  Branch:    ${BRANCH:-HEAD}"
echo "  Image:     $IMAGE_TAG"
echo "  Main repo: $MAIN_REPO"
echo "  G-code:    ${GCODE:-none (basic test)}"
echo ""

# Create build context: archive target branch + overlay simulator tools
BUILD_CTX=$(mktemp -d)
trap "rm -rf $BUILD_CTX" EXIT

if [[ -n "$BRANCH" ]]; then
    echo "Extracting branch '$BRANCH'..."
    (cd "$MAIN_REPO" && git archive "$BRANCH") | tar -x -C "$BUILD_CTX"
else
    echo "Extracting HEAD..."
    (cd "$REPO_ROOT" && git archive HEAD) | tar -x -C "$BUILD_CTX"
fi

# Overlay simulator tools (from the worktree/current checkout)
echo "Adding simulator tools..."
mkdir -p "$BUILD_CTX/tools/kalico-sim"
cp -a "$SCRIPT_DIR"/* "$BUILD_CTX/tools/kalico-sim/"

# Build
echo "Building Docker image '$IMAGE_TAG'..."
docker build \
    -t "$IMAGE_TAG" \
    -f "$BUILD_CTX/tools/kalico-sim/Dockerfile" \
    "$BUILD_CTX"

# Run
if [[ -n "$GCODE" ]]; then
    GCODE_ABS="$(cd "$(dirname "$GCODE")" && pwd)/$(basename "$GCODE")"
    DOCKER_ARGS="$DOCKER_ARGS -v $GCODE_ABS:/gcode/$(basename "$GCODE"):ro"
    EXTRA_ARGS="$EXTRA_ARGS --gcode /gcode/$(basename "$GCODE")"
fi

echo "Running simulation..."
docker run $DOCKER_ARGS "$IMAGE_TAG" $EXTRA_ARGS
