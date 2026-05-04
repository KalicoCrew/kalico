# Bridge-mode Phase 4 — handoff to next session

**Date:** 2026-05-04 (rev 2 — Wave 1 root-cause fix landed)
**Branch:** `sota-motion`
**Last commit:** `3e6df1955` (fix: reject zero/non-finite shaper freq at config boundary)

## TL;DR

Phase 4's `algebra.rs:1065` panic is **fixed**. Root cause was not the trajectory/nurbs pipeline at all — it was `tools/sim_klippy/printer.cfg` setting `shaper_freq_x: 0` as a "disable shaping" workaround. That zero flowed unvalidated through `parse_required_shaper`, produced a NaN-laden bell kernel on `(−∞, +∞)` support, and propagated NaN through `pad_segment_axis` → `convolve` → `BezierPiece::Add`'s `u_start != rhs.u_start` check (which evaluates true on NaN per IEEE-754, triggering the spurious `SupportMismatch`). The four prior Codex rounds chased symptoms (NaN-safe sort, adaptive Hermite subdivision) without removing the NaN producer.

**Wave 1 fix (commit `3e6df1955`):** validate `freq > 0 && freq.is_finite()` at the config boundary in `parse_required_shaper`; sim cfg uses `shaper_freq_x: 50` to match the test-harness default. Diagnosis verified end-to-end via `kalico-verifier` (IEEE-754 + source trace).

**New blocker — wire-level, not algebra-level.** With NaN gone, dispatch reaches the producer transport and trips:

```
RuntimeError: dispatch error: load_curve mcu=0:
  producer transport: transport parse error:
  OutOfRange { value: 976, range: "buffer len 0..=255" }
```

The post-shape NURBS payload is 976 bytes (≈117 f32 control points + knots for one axis); the `kalico_load_curve` wire field is encoded with a u8 length prefix. The NaN panic was masking this because dispatch never ran. This is genuinely separate work — Wave 2 in the new plan.

The big-picture plan + RFC are still relevant:
- `docs/superpowers/specs/2026-05-04-bridge-mode-completion-rfc.md`
- `docs/superpowers/plans/2026-05-04-bridge-mode-path-a-completion.md`

## End goal

Get `G28 X` to complete on the user's H723-driven Trident with the kalico Rust runtime owning motion. Phase 4 (segment dispatch → step pulses on a basic move, sim only) is in progress. Phases 5–7 (endstop trip, peripheral validation, physical bring-up) sit on top of a fully-validated dispatch path.

## Phase 4 sub-status (rev 2)

| Sub-piece | State |
|---|---|
| Bridge identify handshake (Phase 1) | ✅ green |
| Config-stage round-trips (Phase 2) | ✅ green |
| Async event dispatch / kalico_status_v6 (Phase 3) | ✅ green |
| Outbound command wire routing (set_homed_state, push_segment FFI) | ✅ green |
| Per-MCU clock scheduling of shaped segments | ✅ green |
| Single-MCU sim homed gate at planner init | ✅ green |
| Trajectory C¹ Hermite adaptive subdivision | ✅ green |
| NaN-safe break-point sort in nurbs::algebra | ✅ green (algebra.rs:1044) |
| ~~Same-support accumulation in convolution~~ | ✅ **green — root cause was NaN kernel from freq=0; fixed at config boundary** |
| Trajectory pipeline produces valid shaped segment | ✅ green (sim run 2026-05-04 confirms `convolve` succeeds) |
| **`load_curve` wire-frame size limit** | 🔴 **next blocker — `OutOfRange 976 vs 255`** |
| Step pulses observable via KALICO_SIM_STEP_COUNT | ❌ blocked behind the above |

## How to reproduce the current blocker

**Important:** the previous handoff's repro command skipped the Rust rebuild and produced spurious `set_homed_state: Timeout` failures with a stale `motion_bridge_native.so`. Use `run_local.sh` (which calls `make -f Makefile.kalico motion-bridge` before the test) for the canonical path. Single command from the repo root on macOS (Docker Desktop running):

```bash
cd /Users/daniladergachev/Developer/kalico
bash tools/sim_klippy/run_local.sh "G1 X10 F1000"
```

That gets you a clean klippy boot through `MotionToolhead: marked single-MCU local sim homed` and `Welcome to Kalico` — but the move itself doesn't exercise the step-count gate.

For the actual Phase 4 gate test (which sends `G1 X10 F1000` + `M400` and queries step counters), **first** run `run_local.sh` once to ensure the binary is fresh, **then**:

```bash
docker run --rm -v "$(pwd)":/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  rm -f klippy/chelper/c_helper.so
  RUST_BACKTRACE=full python3 tools/sim_klippy/test_phase4_steps.py 2>&1 | tail -50
"
```

Expected output (current state — Wave 1 landed, OutOfRange exposed):

```
LOG: MotionToolhead: marked single-MCU local sim homed
LOG: SET_KINEMATIC_POSITION: set bridge homed on mcu
LOG: RuntimeError: dispatch error: load_curve mcu=0:
       producer transport: transport parse error:
       OutOfRange { value: 976, range: "buffer len 0..=255" }
[phase4] tearing down
```

If `kalico-sim:latest` doesn't exist yet (fresh shell):

```bash
docker build -q -t kalico-sim:latest -f tools/sim_klippy/Dockerfile tools/sim_klippy/
```

## What 976 bytes is

`encode_load_curve_scalar(degree, knots, cps)` at `rust/kalico-host-rt/src/wire.rs:70` packs a per-axis scalar NURBS as f32. Total bytes = 4·(N_knots + N_cps + small header). For N_cps ≈ 117 and corresponding knots, that lands at 976 bytes.

Why so many control points for a 10 mm move? The post-shape pipeline:
- Layer 1 cubic Bézier (degree 3, 4 cps per piece)
- Composition with degree-2 s(t) → degree 6
- C¹ Hermite refit → degree 4
- Convolution with degree-4 bell kernel → degree 4 + 4 + 1 = 9
- Each break-point in the convolution Minkowski-sum becomes a piece

A 10 mm move at 50 Hz shaping with multiple temporal break-points easily reaches ~10 pieces × ~10 cps/piece + knot vector overhead. So 976 bytes is *plausible* output, not a runaway.

The wire spec, however, treats the cps/knots blob as a single Klipper-protocol "buffer" field with a **u8 length prefix** (max 255 bytes). That's the mismatch.

## Likely fixes for the OutOfRange (Wave 2 candidate)

Three shapes, in increasing scope:

1. **Chunked `load_curve` over multiple frames.** Add `kalico_load_curve_chunk` (or extend existing with `chunk_index`/`total_chunks`/`payload_offset` fields). Host splits into ≤200-byte chunks; MCU re-assembles into the curve pool slot. Most Klipper-idiomatic — Klipper's existing `tmcuart_send` and large-data flows use chunking. Estimated 2–4 hours.

2. **Switch the buffer field to a u16-length prefix.** Requires a wire-spec bump (probably a new command name like `kalico_load_curve_v2`) and MCU-side `command_kalico_load_curve` accept a wider buffer. Cleaner protocol but invalidates any captured corpora and the canonical-H723-capture work blocked behind Step 7-C.

3. **Reduce curve complexity at the trajectory layer.** Spread one shaped segment across multiple `load_curve`s by splitting at temporal break-points. Doesn't change the wire protocol but adds dispatch-layer complexity and may produce per-axis discontinuities at split boundaries (need C¹ matching).

**Recommendation:** Option 1 (chunked transport). It's the most Klipper-idiomatic and preserves the canonical-corpus work. Option 2 is "right" for a green-field design but expensive. Option 3 is a workaround.

## Build + test commands (rev 2)

```bash
# Canonical path (rebuilds bridge .so, klipper.elf, then runs the harness)
bash tools/sim_klippy/run_local.sh "<gcode>"

# Phase 4 step-count gate (run AFTER run_local.sh once to ensure fresh binary)
docker run --rm -v "$(pwd)":/work -w /work --tmpfs /tmp:exec kalico-sim:latest \
  bash -c "rm -f klippy/chelper/c_helper.so; python3 tools/sim_klippy/test_phase4_steps.py 2>&1 | tail -30"

# Inspect logs
tail -50 tools/sim_klippy/.local-logs/klippy.log | grep -v kalico_status_v6
cat   tools/sim_klippy/.local-logs/klipper_elf.log

# Rust unit tests for trajectory + nurbs (run locally on macOS — no Docker needed)
cargo test -p trajectory -p nurbs 2>&1 | tail -20

# Workspace lint check
cargo check --workspace --all-targets 2>&1 | tail -10
```

**Stop using the bare-docker repro from rev 1 of this handoff** — it doesn't rebuild the `.so` and produces spurious `set_homed_state: Timeout` failures.

## Wave 2 plan (separate from this handoff scope, still tracked)

Two parallel tracks once the OutOfRange is unblocked:

1. **`Passthrough` variant on `RequiredShaper`** mirroring `AxisShaper`. Lets sim cfgs opt out of X/Y shaping cleanly without picking an arbitrary frequency. Plumb through `shape_axis` to skip `convolve` when kernel is None.

2. **Re-evaluate the Hermite-refit tolerance claim** in the original `printer.cfg:58-61` comment ("can't hit tolerance on synthetic moves"). The fact that `single_axis_x_move` passes at 50 Hz on the same move shape suggests it may have been speculative or stale. ~10 min check.

## Where to start tomorrow — concrete

**Immediate next step (Wave 2.0 — diagnose load_curve OutOfRange):**

1. Read `rust/kalico-host-rt/src/wire.rs::encode_load_curve_scalar` and trace what field type the producer assigns to `cps`/`knots` (likely `'s'` Klipper-buffer type, max 255 bytes).
2. Read MCU-side `command_kalico_load_curve` at `src/runtime_tick.c:426-477` to see what it expects.
3. Decide between Options 1/2/3 above. If chunked transport (recommended), spec the new command shape and update both ends.
4. Add a regression test in `rust/motion-bridge/tests/sim_motion.rs` exercising a curve >255 bytes (the existing `single_axis_x_move` may already do this — verify and assert dispatch succeeds).

## Key files to know about

### Test harness

- `tools/sim_klippy/run_local.sh` — **canonical** one-shot Docker runner (rebuilds bridge .so first)
- `tools/sim_klippy/run.py` — generic sim launcher
- `tools/sim_klippy/test_phase4_steps.py` — Phase 4 gate test (G1 X10 + M400 → step counts)
- `tools/sim_klippy/printer.cfg` — synthetic minimal corexy config (shaper at 50 Hz post-Wave-1)
- `tools/sim_klippy/Dockerfile` — bookworm-slim + libgpiod + rustup 1.85
- `tools/sim_klippy/.local-logs/{klippy.log,klipper_elf.log}` — runtime logs (gitignored)

### Bridge wire path (Path A)

- `klippy/serialhdl.py::SerialReader.connect_pipe` — bridge branch: `attach_serial` → identify → `_bridge_event_poller` reactor timer
- `klippy/motion_bridge.py::MotionBridgeWrapper` — Python wrapper over `motion_bridge_native` (PyO3)
- `rust/motion-bridge/src/bridge.rs::PyMotionBridge` — pyo3 entry points
- `rust/kalico-host-rt/src/host_io/` — KalicoHostIo reactor
- `rust/kalico-host-rt/src/producer.rs::load_curve` — the call that's tripping OutOfRange
- `rust/kalico-host-rt/src/wire.rs::encode_load_curve_scalar` — wire encoder
- `rust/kalico-host-rt/src/host_io/parser.rs:481` — origin of OutOfRange error

### Trajectory + nurbs pipeline (now confirmed working post-Wave-1)

- `rust/trajectory/src/fit.rs` — C¹ Hermite refit + adaptive subdivision
- `rust/trajectory/src/beta.rs` — β-medium TOPP-RA outer iteration
- `rust/temporal/` — TOPP-RA SOCP solver
- `rust/nurbs/src/algebra.rs::convolve` — produces the post-shape NURBS that's now too big for the wire
- `rust/nurbs/src/bezier.rs::BezierPiece` — same-support check at `Add` line 470

### Firmware (Linux host build)

- `src/runtime_tick.c::command_kalico_load_curve` (line 426) — MCU receiver for the curve payload
- `src/runtime_tick.c` — DECL_COMMAND for kalico_*; CONFIG_KALICO_SIM-gated diags
- `src/linux/kalico_host_tick.c` — pthread @ 40 kHz replacing TIM5_IRQHandler
- `.config.linux` — defconfig: `MACH_LINUX=y`, `KALICO_RUNTIME=y`, `KALICO_SIM=y`

## Constraints / gotchas to remember

- **Always use `run_local.sh` (or include `make -f Makefile.kalico motion-bridge` in your docker invocation).** The bare `docker run … python3 test_phase4_steps.py` form runs against a stale `.so` and produces nonsense failures (e.g. `set_homed_state: Timeout` from rev 1 of this handoff).
- The user's production printer.cfg at `~/printer_data/config/printer.cfg` is **off-limits**. The sim has its own config at `tools/sim_klippy/printer.cfg`.
- `motion_bridge.so` (PyO3 native module) is named `motion_bridge_native.so` to avoid shadowing the Python wrapper `motion_bridge.py`. See the RFC for why this used to silently disable bridge mode in production.
- `klippy/chelper/c_helper.so` is built per-platform — delete before each Docker run if it was last built on macOS.
- Rust workspace uses `host` features (f64) for the Linux build, `mcu-h7` (f32) for production firmware.
- Codex (sandboxed) cannot commit because `.git` is read-only — its work shows up as unstaged diff that needs `git commit` from the main session.
- The verifier flagged a *third* potential NaN-sensitive site (`bezier_pieces_to_nurbs`'s contiguity assert) that should now be unreachable; if a panic surfaces there post-Wave-2, that's a separate algebra bug.

## Quick commit log (this session vs prior)

```
3e6df1955 fix(7-D Phase 4): reject zero/non-finite shaper freq at config boundary  ← Wave 1
1bd18ace6 doc: bridge-mode Phase 4 handoff — repro, end goal, where to pick up    (rev 1, now superseded)
4fa3486fa fix(7-D Phase 4): NaN-safe break-point sort in nurbs algebra (Codex r4)  (symptom patch — root cause was freq=0, not the sort itself)
a0f53215a feat(7-D Phase 4): adaptive C¹ Hermite subdivision + downstream nurbs panic
c375eae55 wip(7-D Phase 4): sim flushes via M400, surfaces trajectory refit bug
0eaf8ff0e wip(7-D): Phase 4 segment scheduling fixes (Codex round 2)
```

The four prior Codex rounds (r2 through r4) each landed real fixes for real bugs, but none of them were the *blocking* bug. r4's NaN-safe sort prevented the sort itself from panicking on NaN, which moved the panic one site downstream to `BezierPiece::Add`. Future rounds should resist the temptation to symptom-patch; trace NaN/infinity to their producer.
