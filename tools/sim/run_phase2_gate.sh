#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$REPO_ROOT"

LOG_DIR="${LOG_DIR:-/tmp/kalico-phase2-gate-logs}"
SIM_BOOT_DELAY_S="${SIM_BOOT_DELAY_S:-10}"
PORT="${PORT:-socket://localhost:3334}"

mkdir -p "${LOG_DIR}"
SIM_LOG="${LOG_DIR}/renode-$(date +%Y%m%d-%H%M%S).log"

if [[ ! -f out/klipper.elf ]]; then
  echo "error: out/klipper.elf not found. Run tools/sim/build_sim_firmware.sh first." >&2
  exit 2
fi
if [[ ! -f klippy/motion_bridge.so ]]; then
  echo "error: klippy/motion_bridge.so not found. Run 'make -f Makefile.kalico motion-bridge' first." >&2
  exit 2
fi
if ! command -v renode >/dev/null 2>&1; then
  echo "error: renode not on PATH (brew install renode)." >&2
  exit 2
fi

pkill -f renode 2>/dev/null || true
sleep 1

# Note: setsid is Linux-only; on macOS we rely on pkill -f renode for cleanup.
echo "[gate] launching Renode (log=${SIM_LOG}) ..."
set +e
bash "${REPO_ROOT}/tools/sim/run_sim.sh" >"${SIM_LOG}" 2>&1 &
SIM_PID=$!
set -e

cleanup() {
  echo "[gate] cleaning up sim (pid=${SIM_PID}) ..."
  if kill -0 "${SIM_PID}" 2>/dev/null; then
    kill -TERM "${SIM_PID}" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      if ! kill -0 "${SIM_PID}" 2>/dev/null; then break; fi
      sleep 1
    done
    kill -KILL "${SIM_PID}" 2>/dev/null || true
  fi
  pkill -f renode 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[gate] waiting up to ${SIM_BOOT_DELAY_S}s for ${PORT} ..."
deadline=$(( $(date +%s) + SIM_BOOT_DELAY_S ))
ready=0
while [[ $(date +%s) -lt ${deadline} ]]; do
  # /dev/tcp probe — bash builtin, no nc required.
  if (echo -n >/dev/tcp/localhost/3334) 2>/dev/null; then
    ready=1
    break
  fi
  sleep 0.5
done
if [[ ${ready} -ne 1 ]]; then
  echo "FAIL: Renode UART bridge did not come up within ${SIM_BOOT_DELAY_S}s" >&2
  echo "      see ${SIM_LOG} for Renode output" >&2
  exit 1
fi
echo "[gate] sim ready, running harness ..."

export PYTHONPATH="${REPO_ROOT}/klippy:${REPO_ROOT}/tools:${PYTHONPATH:-}"
python3 "${REPO_ROOT}/tools/test_renode_phase2_gate.py" --port "${PORT}"
RC=$?

if [[ ${RC} -eq 0 ]]; then
  echo "[gate] PASS"
else
  echo "[gate] FAIL (rc=${RC}); Renode log: ${SIM_LOG}"
fi
exit ${RC}
