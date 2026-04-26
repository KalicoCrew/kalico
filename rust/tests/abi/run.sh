#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
make clean
make run
