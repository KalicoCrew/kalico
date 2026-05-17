#!/usr/bin/env bash
# Wire-format sim-driven reproduction harness for the live-bench jog bugs.
#
# Runs the `rust/motion-bridge/tests/sim_motion_jogs.rs` test battery, which
# spawns a Renode H723 sim subprocess and drives the production
# `motion-bridge` (PlannerHandle + producer::load_curve / push_segment over a
# live KalicoHostIo reactor) against it. Three test cases exercise the live
# jog sequences the user reproduces on hardware:
#
#   Test A — first 25 mm pure-X jog at F=100 immediately after stream_open.
#            (live-bench symptom: ~50% of fresh-connect first jogs energize
#            motors but produce no actual motion).
#   Test B — 10 alternating ±25 mm jogs at ~1 s intervals (live-bench
#            symptom: "speed all over the place / huge delays").
#   Test C — rapid burst of 20 × 5 mm jogs (live-bench symptom: intermittent
#            no-motion / `KALICO_ERR_STEP_BURST_EXCEEDED`).
#
# Plus a smoke test (`sim_harness_boots_and_emits_status`) that exercises
# just sim boot + identify + clock-sync — useful to debug the harness
# infrastructure independently.
#
# Prereqs:
#   - renode installed (brew install renode)
#   - Sim firmware built: tools/sim/build_sim_firmware.sh
#
# Each test takes ~1–2 minutes wall clock (Renode quantum=1µs is ~5x slower
# than wall-clock). `--test-threads=1` is MANDATORY — all tests own the
# singleton TCP port 3334. Parallel execution would race for the sim.
#
# Usage:
#   bash tools/sim/run_sim_motion_jogs.sh             # all four tests
#   bash tools/sim/run_sim_motion_jogs.sh smoke       # smoke test only
#   bash tools/sim/run_sim_motion_jogs.sh first       # test A only
#   bash tools/sim/run_sim_motion_jogs.sh alternating # test B only
#   bash tools/sim/run_sim_motion_jogs.sh rapid       # test C only
#   bash tools/sim/run_sim_motion_jogs.sh phase       # test F only

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
  all|"")       FILTER="" ;;
  *)
    echo "Usage: $0 [smoke|first|alternating|rapid|home|g28|phase|all]" >&2
    exit 2 ;;
esac

# Pre-flight: ensure no stale sim is on port 3334. Renode binds *:3334 on
# `CreateServerSocketTerminal`; a leftover one (from a previously-killed
# `cargo test` run) makes the new sim's bind fail silently and our test
# panics with "Renode TCP port did not accept connections".
pkill -9 -f renode 2>/dev/null || true
sleep 2

# Pre-flight: ensure the sim firmware exists.
if [[ ! -f "${REPO_ROOT}/out/klipper.elf" ]]; then
  echo "error: out/klipper.elf not found. Build the sim firmware first:" >&2
  echo "       bash tools/sim/build_sim_firmware.sh" >&2
  exit 2
fi

# `--nocapture` so we see the harness's diag `eprintln!` lines as the tests
# run (which jog is being submitted, which segments dispatched, etc.).
# `RUST_BACKTRACE=1` makes any panic show file:line — useful when the test
# fails in a helper deep in the call stack.
export RUST_BACKTRACE=1

if [[ -n "${FILTER}" ]]; then
  exec cargo test -p motion-bridge --test sim_motion_jogs -- \
    --ignored --test-threads=1 --nocapture "${FILTER}"
else
  exec cargo test -p motion-bridge --test sim_motion_jogs -- \
    --ignored --test-threads=1 --nocapture
fi
