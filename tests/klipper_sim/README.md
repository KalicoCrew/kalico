# klipper-sim integration test

Cross-branch step-time comparison harness for the redesigned stepping engine.

## Purpose

Verify that step times produced by our stepping engine match mainline
Klipper's within **< 500 ns per step** at typical accel. The 500 ns
threshold is the local-linear-extrapolation tolerance from the stepping
spec — it captures the dispatcher jitter introduced by the mainline-style
one-pulse-per-fire consumer pattern we adopted. Drift above that
threshold means the redesign has measurably diverged from mainline timing,
not just rounded differently.

## Dependency

Requires `~/Developer/klipper-sim/` (an offline planner + shaper +
pressure-advance simulator that runs a real Klipper clone's planner against
a G-code file). Override the location with `KLIPPER_SIM_DIR=...`.

## How to run

```bash
./tests/klipper_sim/run_stepping_redesign.sh
```

Exit codes:

- `0` with `SKIP: klipper-sim not installed` — harness OK, klipper-sim missing.
- `0` with `PASS: max drift ... ns` — comparison succeeded under threshold.
- Non-zero — drift exceeded threshold, or the CLI bridge is not yet wired
  (currently the expected state — see below).

## Current status: stub

klipper-sim today emits **trajectory CSVs sampled at 100 µs** (position,
velocity, accel per axis), not per-step pulse events. Comparing step times
apples-to-apples requires either:

1. Extending klipper-sim with a `--emit-steps` flag that runs stepcompress
   and dumps step events, or
2. Wrapping klipper-sim output through Klipper's stepcompress logic locally
   inside this harness.

Until one of those exists, `run_sim()` raises `NotImplementedError` with a
pointer back to klipper-sim. The harness fails loudly rather than silently
passing — by design.
