#!/bin/bash

set -euo pipefail

KALICO_REPO="KalicoCrew/kalico"

if [[ $# != 1 ]]; then
  echo "Usage: $0 path/to/danger-klipper"
  exit 2
fi

ROOT_PATH="$(realpath "$1")"
cd "$ROOT_PATH"

if [[ ! -d "$ROOT_PATH/klippy/" ]]; then
  echo "$1 is not a Danger-Klipper path"
  exit 1
fi

if ! git rev-parse --git-dir >/dev/null 2>&1; then
  echo "$1 is not installed using git, unable to migrate"
  exit 1
fi

GIT_URL="$(git config --get remote.origin.url)"
if [[ "${GIT_URL,,}" != *"dangerklippers/danger-klipper"* ]]; then
  echo "$1 url $GIT_URL does not seem to be Danger-Klipper"
  exit 1
fi

GIT_BRANCH=$(git symbolic-ref --short -q HEAD)

if [[ "${GIT_URL,,}" == *"git@github.com"* ]]; then
  KALICO_ORIGIN="git@github.com:${KALICO_REPO}"
else
  KALICO_ORIGIN="https://github.com/${KALICO_REPO}"
fi

echo "Changing origin to '$KALICO_ORIGIN'"
git remote set-url origin "$KALICO_ORIGIN"

echo "Updating the main branch"
git fetch origin main:main
git branch --set-upstream-to origin/main main

if [[ "${GIT_BRANCH,,}" == "master" ]]; then
  git checkout main
fi

echo "Migrated ${ROOT_PATH}!"
