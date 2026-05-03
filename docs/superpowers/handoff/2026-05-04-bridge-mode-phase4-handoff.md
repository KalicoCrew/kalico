# Bridge-mode Phase 4 — handoff to next session

**Date:** 2026-05-04
**Branch:** `sota-motion`
**Last commit:** `4fa3486fa` (Codex round 4: NaN-safe break-point sort)

## TL;DR

The klippy-in-loop sim is plumbed end-to-end. Phases 1–3 are green. Phase 4 (segment dispatch produces step pulses) is blocked by a long tail of bugs in the **`rust/trajectory` + `rust/nurbs/algebra`** pipeline that surface only when an actual move is flushed through the planner. Four Codex rounds fixed the obvious wire / scheduling / NaN issues; the next panic is `algebra.rs:1065 .expect("same-support accumulation")` and there are likely more latent bugs behind it.

The big-picture plan + RFC + Path A architecture diagram are at:

- `docs/superpowers/specs/2026-05-04-bridge-mode-completion-rfc.md` — why bridge mode was silently disabled in production for months
- `docs/superpowers/plans/2026-05-04-bridge-mode-path-a-completion.md` — 7-phase implementation plan

## End goal

Get `G28 X` to complete on the user's H723-driven Trident, with the kalico Rust runtime actually owning motion (not the legacy klipper serialqueue). All the work between here and there is per the Path A plan:

- **Phase 4 (current):** segment dispatch → step pulses on a basic move (sim only)
- **Phase 5:** endstop trip path (sim + hardware)
- **Phase 6:** peripheral validation (TMC, heater, fan)
- **Phase 7:** physical bring-up on the H723

Once Phase 4 is green, Phase 5–7 are mostly wiring on top of an already-validated stack.

## Phase 4 sub-status

| Sub-piece | State |
|---|---|
| Bridge identify handshake (Phase 1) | ✅ green |
| Config-stage round-trips (Phase 2) | ✅ green |
| Async event dispatch / kalico_status_v6 (Phase 3) | ✅ green |
| Outbound command wire routing (set_homed_state, push_segment FFI) | ✅ green |
| Per-MCU clock scheduling of shaped segments | ✅ green |
| Single-MCU sim homed gate at planner init | ✅ green |
| Trajectory C¹ Hermite adaptive subdivision | ✅ green (fixes ToleranceNotReached) |
| NaN-safe break-point sort in nurbs::algebra | ✅ green (algebra.rs:1044 — `total_cmp` instead of `partial_cmp().unwrap()`) |
| **Same-support accumulation in convolution** | 🔴 **next panic — algebra.rs:1065** |
| Step pulses observable via KALICO_SIM_STEP_COUNT | ❌ blocked behind the above |

## How to reproduce the current panic

Single command from the repo root on macOS (Docker Desktop must be running):

```bash
cd /Users/daniladergachev/Developer/kalico
docker run --rm -v "$(pwd)":/work -w /work --tmpfs /tmp:exec kalico-sim:latest bash -c "
  rm -f klippy/chelper/c_helper.so
  RUST_BACKTRACE=1 python3 tools/sim_klippy/test_phase4_steps.py 2>&1 | tail -30
"
```

If `kalico-sim:latest` doesn't exist yet (fresh shell):

```bash
docker build -q -t kalico-sim:latest -f tools/sim_klippy/Dockerfile tools/sim_klippy/
```

**Expected output (current state):**

```
thread 'kalico-planner' panicked at /work/rust/nurbs/src/algebra.rs:1065:50:
called `Option::unwrap()` on a `None` value
note: run with `RUST_BACKTRACE=1` environment variable to display a backtrace
[phase4] cleaning up prior processes
[phase4] spawning klipper.elf
[phase4] spawning klippy
[phase4] fake-homing: SET_KINEMATIC_POSITION X=0 Y=0 Z=0
  response: {'id': 1, 'result': {}}
[phase4] sending G1 X10 F1000 then M400 (flush)
  response: {}
[phase4] querying step count for OID 0 (X stepper)
ConnectionRefusedError: [Errno 111] Connection refused
```

The `ConnectionRefusedError` is a downstream effect of the planner thread panicking — it kills the bridge, which kills the api connection.

The panic site:

```rust
// rust/nurbs/src/algebra.rs:1065
let contribution = integrate_product_piece(x_p, w_p, alpha, beta);
accum = (&accum + &contribution).expect("same-support accumulation");
```

The `.expect()` is on an `Option<BezierPiece>` returned by `BezierPiece::add`. `add` returns `None` when its two operands have mismatched `(u_start, u_end)` support intervals. So somewhere upstream a `contribution` piece is being built with a different alpha/beta than the `accum` window, or `integrate_product_piece` is producing a piece whose support doesn't match the surrounding window.

Likely root cause (matches the pattern of the prior 4 rounds): the new tiny-piece filter in `rust/trajectory/src/fit.rs` (commit `4fa3486fa`) skips composed pieces below `MIN_HERMITE_PIECE_DURATION` but doesn't update the downstream break-point set. That causes the convolution's per-piece windows to fall out of sync.

## Build + test commands

```bash
# One-shot full build + sim test (runs entirely in Docker, ~30s if cached)
bash tools/sim_klippy/run_local.sh "<gcode>"   # for ad-hoc gcode
docker run --rm -v "$(pwd)":/work -w /work --tmpfs /tmp:exec kalico-sim:latest \
  bash -c "rm -f klippy/chelper/c_helper.so; python3 tools/sim_klippy/test_phase4_steps.py 2>&1 | tail -15"

# Inspect logs
tail -50 tools/sim_klippy/.local-logs/klippy.log | grep -v kalico_status_v6
cat   tools/sim_klippy/.local-logs/klipper_elf.log

# Rust unit tests for trajectory + nurbs (run in container)
docker run --rm -v "$(pwd)":/work -w /work kalico-sim:latest bash -c "
  cd rust && cargo test -p trajectory -p nurbs 2>&1 | tail -20
"

# Workspace lint check
docker run --rm -v "$(pwd)":/work -w /work kalico-sim:latest bash -c "
  cd rust && cargo check --workspace 2>&1 | tail -10
"
```

## Where to start tomorrow — concrete options

**Option 1 (incremental, recommended):** Fix the `algebra.rs:1065` same-support panic. Likely root cause is in `rust/trajectory/src/fit.rs::fit_and_split` where the new tiny-piece filter skips pieces but doesn't update downstream break sets. Either (a) keep the original break set even when filtering, (b) propagate the filter to all consumers consistently, or (c) make `BezierPiece::add` tolerant of small support mismatches with an explicit re-windowing.

This continues the pattern of round-by-round Codex fixes. Estimated 1–3 more rounds before step pulses actually emit.

**Option 2 (holistic):** Add property-based fuzzing to `rust/trajectory` and `rust/nurbs/algebra` that generates synthetic moves at the boundary cases (zero-duration, single-point, near-zero velocity, max-velocity-from-rest) and asserts no panic. Run, collect every panic, fix as a batch. Higher up-front cost (1–2 days) but actually finishes the long tail.

**Option 3 (workaround for tonight only):** Switch `tools/sim_klippy/test_phase4_steps.py` to a longer faster move (`G1 X100 F12000`) that's less likely to hit degenerate sub-pieces. This **only validates the bridge wire end-to-end** — it leaves the trajectory pipeline's robustness gap open. Useful as a "the wire path works, the algorithmic gap is a separate workstream" data point.

I'd recommend **Option 1** for tomorrow's first session — finish what we started, get one move to emit step pulses, confirm the entire dispatch path works, then triage whether to proceed to Phase 5 immediately or take a holistic pass on trajectory robustness.

## Key files to know about

### Test harness

- `tools/sim_klippy/run_local.sh` — one-shot Docker runner (laptop-local, no Pi needed)
- `tools/sim_klippy/run.py` — generic sim launcher; takes a gcode string
- `tools/sim_klippy/test_phase4_steps.py` — Phase 4 gate test (G1 X10 + M400 → query step counts)
- `tools/sim_klippy/printer.cfg` — synthetic minimal corexy config (gpiochip0 unused-input pins, no TMC, no extruder, shaper freq=0)
- `tools/sim_klippy/Dockerfile` — bookworm-slim + libgpiod + rustup 1.85
- `tools/sim_klippy/.local-logs/{klippy.log,klipper_elf.log}` — runtime logs (gitignored)

### Bridge wire path (Path A)

- `klippy/serialhdl.py::SerialReader.connect_pipe` — bridge branch: `attach_serial` → identify → `_bridge_event_poller` reactor timer
- `klippy/motion_bridge.py::MotionBridgeWrapper` — Python wrapper over `motion_bridge_native` (PyO3)
- `rust/motion-bridge/src/bridge.rs::PyMotionBridge` — pyo3 entry points: `attach_serial`, `get_identify_data`, `bridge_call`, `bridge_send`, `endstop_arm`, `submit_move`, `init_planner`, `set_homed_state`
- `rust/kalico-host-rt/src/host_io/` — KalicoHostIo reactor: `open_pipe_with_config`, `call_typed`, `send_fire_and_forget`, `identify_handshake`

### Trajectory + nurbs pipeline (the current pain point)

- `rust/trajectory/src/fit.rs` — C¹ Hermite refit + adaptive subdivision (Codex r3); tiny-piece filter (Codex r4)
- `rust/trajectory/src/beta.rs` — β-medium TOPP-RA outer iteration
- `rust/temporal/` — TOPP-RA SOCP solver (degree-2 s(t) per piece)
- `rust/nurbs/src/algebra.rs` — convolution + composition; current panic site at line 1065
- `rust/nurbs/src/bezier.rs` — `BezierPiece::add` returns `None` on support mismatch (the `.expect` that panics)

### Firmware (Linux host build)

- `src/runtime_tick.c` — DECL_COMMAND for kalico_*; CONFIG_KALICO_SIM-gated diags
- `src/linux/kalico_host_tick.c` — pthread @ 40 kHz replacing TIM5_IRQHandler
- `src/linux/gpio.c` — under CONFIG_KALICO_SIM, all GPIO ops are memory-only no-ops (no /dev/gpiochip required)
- `.config.linux` — defconfig: `MACH_LINUX=y`, `KALICO_RUNTIME=y`, `KALICO_SIM=y`

## Constraints / gotchas to remember

- The user's production printer.cfg at `~/printer_data/config/printer.cfg` is **off-limits**. The sim has its own config at `tools/sim_klippy/printer.cfg`.
- `motion_bridge.so` (the PyO3 native module) was renamed to `motion_bridge_native.so` to stop it from shadowing the Python wrapper file `motion_bridge.py`. **This is the bug that hid bridge mode being silently disabled in production for months.** See the RFC.
- `klippy/chelper/c_helper.so` is built per-platform — delete before each Docker run if it was last built on macOS.
- Rust workspace uses `host` features (f64) for the Linux build, `mcu-h7` (f32) for production firmware. Single-source rule preserved.
- Codex (sandboxed) cannot commit because `.git` is read-only — its work shows up as unstaged diff that needs `git commit` from the main session.
- The 4 Codex rounds so far each surfaced a new panic. Don't be surprised if Option 1 (incremental fix) takes 1–3 more rounds.

## Quick commit log (what landed this session)

```
4fa3486fa fix(7-D Phase 4): NaN-safe break-point sort in nurbs algebra (Codex round 4)
a0f53215a feat(7-D Phase 4): adaptive C¹ Hermite subdivision + downstream nurbs panic
c375eae55 wip(7-D Phase 4): sim flushes via M400, surfaces trajectory refit bug
0eaf8ff0e wip(7-D): Phase 4 segment scheduling fixes (Codex round 2)
4562de12e fix(7-D): route stream/segment commands through KalicoHostIo onto the wire
209ce7833 fix(bridge): use s.params.shaper_type/freq accessors in _init_planner
1d64dc734 feat(7-D): Phase 2 bridge-mode config round-trip — MCU loads config cleanly
b8d90f798 fix(sim): klippy-in-loop bug surface
acaba545d feat(sim): local Docker harness — no Pi required
e232337f8 feat(Phase-1): bridge attach_serial + identify handshake + PTY open_pipe
30674e3bb plan: bridge-mode Path A completion — 7 sim-gated phases
6715f7204 rfc: bridge-mode end-to-end completion — discovery + path-forward
```
