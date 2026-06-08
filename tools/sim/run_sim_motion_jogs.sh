#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
cd "${REPO_ROOT}/rust"

case "${1:-all}" in
  smoke)        FILTER="sim_harness_boots_and_emits_status" ;;
  first|a)      FILTER="first_jog_after_stream_open_runs_on_sim" ;;
  alternating|b) FILTER="ten_alternating_jogs_run_on_sim" ;;
  rapid|c)      FILTER="rapid_short_jogs_burst_no_fault" ;;
  home|homing|e) FILTER="homing_x_trips_when_pa0_raised_via_monitor" ;;
  g28|g)        FILTER="g28_shaped_xy_two_pass_homing_via_renode_monitor" ;;
  phase|f|phase_stepping)
                FILTER="phase_stepping_rapid_g1_x25_after_set_position_no_crash" ;;
  tmc|step|g1x50)
                FILTER="g1_x50_emits_step_pulses_on_sim" ;;
  all|"")       FILTER="" ;;
  *)
    echo "Usage: $0 [smoke|first|alternating|rapid|home|g28|phase|tmc|all]" >&2
    exit 2 ;;
esac

# A stale renode still bound to *:3334 makes the new sim's bind fail
# silently; the sleep lets the port actually release before we relaunch.
pkill -9 -f renode 2>/dev/null || true
sleep 2

if [[ ! -f "${REPO_ROOT}/out/klipper.elf" ]]; then
  echo "error: out/klipper.elf not found. Build the sim firmware first:" >&2
  echo "       bash tools/sim/build_sim_firmware.sh" >&2
  exit 2
fi

export RUST_BACKTRACE=1
# Without this, a transport fault SIGABRTs the reactor thread and bypasses
# test cleanup before the assertion can report.
export KALICO_NO_EXIT_ON_FAULT=1

if [[ -n "${FILTER}" ]]; then
  exec cargo test -p motion-bridge --test sim_motion_jogs -- \
    --ignored --test-threads=1 --nocapture "${FILTER}"
else
  exec cargo test -p motion-bridge --test sim_motion_jogs -- \
    --ignored --test-threads=1 --nocapture
fi
