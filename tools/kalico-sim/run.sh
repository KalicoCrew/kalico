#!/bin/bash
# Kalico Simulator — build and run in Docker.
#
# Usage:
#   ./run.sh                          # Test current working tree (HEAD)
#   ./run.sh --branch sota-motion     # Test a specific branch
#   ./run.sh --gcode benchy.gcode     # Print a G-code file
#   ./run.sh --privileged             # Enable SCHED_FIFO for homing
#   ./run.sh --no-cache               # Force a full rebuild

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"

# Handle worktrees: resolve the common git dir to find the main repo.
GIT_DIR="$(cd "$REPO_ROOT" && git rev-parse --git-common-dir 2>/dev/null || echo "$REPO_ROOT/.git")"
MAIN_REPO="$(cd "$GIT_DIR/.." 2>/dev/null && pwd || echo "$REPO_ROOT")"

BRANCH=""
GCODE=""
EXTRA_ARGS=""
DOCKER_ARGS="--rm"
DOCKER_BUILD_ARGS=""
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
        --no-cache)
            DOCKER_BUILD_ARGS="$DOCKER_BUILD_ARGS --no-cache"
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

if [[ -n "$BRANCH" ]]; then
    # For a named branch: extract into a stable, deterministic directory so
    # Docker can diff the context against its layer cache across runs.
    # A mktemp path defeats the cache because Docker hashes the context path
    # (or its contents) — same files at a different temp path = full rebuild.
    BUILD_CTX="$REPO_ROOT/.cache/kalico-sim/build-ctx-${BRANCH//\//-}"
    echo "Extracting branch '$BRANCH' to $BUILD_CTX ..."
    rm -rf "$BUILD_CTX"
    mkdir -p "$BUILD_CTX"
    (cd "$MAIN_REPO" && git archive "$BRANCH") | tar -x -C "$BUILD_CTX"
    # Overlay current simulator tools from the worktree so local edits to
    # run.sh / Dockerfile / configs are tested without committing.
    mkdir -p "$BUILD_CTX/tools/kalico-sim"
    cp -a "$SCRIPT_DIR"/. "$BUILD_CTX/tools/kalico-sim/"
    echo "Building Docker image '$IMAGE_TAG' from $BUILD_CTX ..."
    # shellcheck disable=SC2086
    docker build \
        $DOCKER_BUILD_ARGS \
        -t "$IMAGE_TAG" \
        -f "$BUILD_CTX/tools/kalico-sim/Dockerfile" \
        "$BUILD_CTX"
else
    # For HEAD: use the repo root directly as build context.
    # The .dockerignore at the repo root keeps the context clean.
    # Docker's layer cache works against file content, so unchanged
    # layers are reused across successive builds on the same tree.
    echo "Building Docker image '$IMAGE_TAG' from repo root ..."
    # shellcheck disable=SC2086
    docker build \
        $DOCKER_BUILD_ARGS \
        -t "$IMAGE_TAG" \
        -f "$SCRIPT_DIR/Dockerfile" \
        "$REPO_ROOT"
fi

if [[ -n "$GCODE" ]]; then
    GCODE_ABS="$(cd "$(dirname "$GCODE")" && pwd)/$(basename "$GCODE")"
    DOCKER_ARGS="$DOCKER_ARGS -v $GCODE_ABS:/gcode/$(basename "$GCODE"):ro"
    EXTRA_ARGS="$EXTRA_ARGS --gcode /gcode/$(basename "$GCODE")"
fi

echo "Running simulation..."
# shellcheck disable=SC2086
docker run $DOCKER_ARGS "$IMAGE_TAG" $EXTRA_ARGS
