# MVP global scalar jerk — design

**Date:** 2026-05-05
**Author:** Phase 4 homing-unblock brainstorm
**Status:** Design ready; implementation pending.
**Related:** `docs/research/stall-homing-move.md`, `rust/temporal/src/topp/constraints.rs:236-247` (maintainer warning), `docs/superpowers/specs/2026-04-27-layer-2-topp-prototype-design.md` §11.

## 1. Summary

Replace the per-axis configuration default `j_max = [max_accel*2, max_accel*2, max_z_accel*2]` in `rust/motion-bridge/src/config.rs::PlannerLimits::to_temporal_limits` with a **single global jerk scalar** `J` populating all three slots: `j_max = [J, J, J]`. Add `J` as a new `max_jerk: f64` stored field on `PlannerLimits`, defaulted from `max_accel * 2.0` at init time, with `Option<f64>` at the PyO3 boundary so klippy can express "use Rust default." `SET_VELOCITY_LIMIT` gains an optional `JERK=` parameter for runtime override. The `temporal::Limits.j_max: [f64; 3]` API is unchanged; the planner's two-stage SLP machinery (Stage 1 scalar path-jerk Lee 2024 cuts in block (h); Stage 2 per-axis Cartesian-jerk active-set SLP) is unchanged. Only the bridge config layer and the klippy g-code surface change.

This unblocks the Phase 4 homing stall (`StalledOnInfeasibleSegment` on G28 X with `j_max[Z] = 200` dominating `J_path = min(j_max)`) and explicitly defers per-axis Cartesian-jerk *configuration* to a later step, alongside the per-axis Cartesian-jerk *SOC relaxation* work that the maintainer warning at `constraints.rs:236-247` was already deferring.

## 2. Motivation

### 2.1 Observed failure

`tools/sim_klippy/test_home_x.py` reproduces:
```
G28 X result: shape pipeline error: temporal joining: StalledOnInfeasibleSegment { last_dirty_count: 1 }
```
The submitted move is a 300 mm pure-X collinear cubic at 50 mm/s. Stage 1 path-jerk SLP diverges at `last_max_ratio ≈ 1.019`; Stage 2 per-axis SLP short-circuits because of Stage 1 failure (`solver.rs:1288-1293`).

### 2.2 Root cause

`rust/temporal/src/topp/constraints.rs:248`:
```rust
let j_path = limits.j_max[0].min(limits.j_max[1]).min(limits.j_max[2]);
```
With the bridge config defaulting `j_max[Z] = max_z_accel * 2 = 200 mm/s³`, every move (including pure-X) is jerk-clamped at 200 even though Z is not moving. At J_path = 200, the 300 mm rest-to-rest move has no cruise (peak feasible v ≪ 50 mm/s requires ≫ 750 mm distance), forcing a single full-length S-curve ramp. Large `b''''(s)` exposes residual Stage 1 SLP gaps that the `1/√b` Lee 2024 cuts cannot close, and the divergence detector trips.

### 2.3 Why "fix it properly with per-axis Cartesian jerk" is not the MVP answer

The first-pass fix idea was to replace `J_path = min(j_max)` with a per-grid-point `J_path[i] = min over axes of j_max[axis] / |c'_axis(s_i)|`. Codex cross-check (transcript captured in this session, summarized in `docs/research/stall-homing-move.md` follow-up) refuted the claim that this is a valid outer approximation of full per-axis Cartesian jerk on curved paths. The cross-term `3·c″·ṡ·s̈` and curvature-derivative term `c‴·ṡ³` can carry sign-cancelling contributions; bounding only the path-jerk component `c′·s⃛` per-axis can produce false infeasibility on curved paths whose total per-axis jerk is genuinely within limits.

A correct per-axis Cartesian-jerk *relaxation* is the work Stage 2 SLP already does and is what the maintainer warning at `constraints.rs:236-247` defers. Doing it *and* coupling Stage 2 to run even on Stage 1's "Diverged" status (so Stage 2 can recover from Stage 1's relaxation imperfection on curved paths) is the proper full fix — it is out of scope for the first-print MVP.

The MVP question is: **do we need per-axis jerk distinction at all for first-print?** The answer is no — the per-axis distinction in our defaults is `2 × per-axis-accel`, a heuristic placeholder, not a measured per-axis physical limit. Collapsing it to a single value loses no real per-axis physics for this hardware.

## 3. What stays per-axis

- **`v_max` per-axis**: real per-axis kinematic limits (Z lead-screw mechanical max ≪ X/Y belt max). Unchanged.
- **`a_max` per-axis**: real per-axis dynamic limits (Z bed-flinging tolerance ≪ X/Y carriage tolerance). Unchanged.
- **`a_centripetal_max`**: scalar today, unchanged.
- **Input shaper kernels**: per-axis (X/Y configurable; Z passthrough by default). Resonance suppression is the smoother's job, not the jerk constraint's.

## 4. What collapses to scalar

- **`j_max[X] = j_max[Y] = j_max[Z] = J`** (single value, populated uniformly into the per-axis array).
- The `temporal::Limits.j_max: [f64; 3]` shape is preserved at the API level so the per-axis SLP machinery in Stage 2 has nothing to refactor when a future step re-introduces non-uniform jerk.

## 5. Configuration

### 5.1 Rust side — `PlannerLimits` storage and PyO3 signature

Add `max_jerk: f64` to `PlannerLimits` in `rust/motion-bridge/src/config.rs`. Default in `PlannerConfig::default()`: `max_accel * 2.0` (= 6000 mm/s³ for the current default `max_accel = 3000`). `PlannerLimits::to_temporal_limits` becomes:

```rust
Limits::new(
    [self.max_velocity, self.max_velocity, self.max_z_velocity],
    [self.max_accel,    self.max_accel,    self.max_z_accel],
    [self.max_jerk,     self.max_jerk,     self.max_jerk],
    self.square_corner_velocity.powi(2) / (self.max_accel * 0.5),
)
```

Drop the comment fragment "Jerk is set to 2× accel as a reasonable default" — it no longer matches the code; replace with a one-paragraph note pointing at this spec.

The PyO3 init boundary uses `Option<f64>` so the Python side can express "use Rust default" cleanly. `rust/motion-bridge/src/bridge.rs::init_planner` (around `bridge.rs:885`) extends its signature to accept `max_jerk: Option<f64>` after the existing `max_accel` argument; when `None`, the constructed `PlannerLimits` uses `max_accel * 2.0` (computed from the *passed-in* `max_accel`, not the `PlannerConfig::default()` constant — so the default tracks the actual init-time accel value).

### 5.2 Klippy side — `printer.cfg` surface

In the `[printer]` section, expose an optional `max_jerk` float key. Behavior:
- If present, parsed and passed through to the bridge as `PlannerLimits::max_jerk`.
- If absent, the Rust default (`init-time max_accel * 2.0`) applies.
- No per-axis `max_jerk_x` / `max_jerk_y` / `max_jerk_z` keys for MVP — explicitly deferred.

The klippy-side parsing site is `klippy/motion_toolhead.py:133-140` (`config.getfloat("max_velocity", ...)` etc. for `[printer]` keys) — add `self.max_jerk = config.getfloat("max_jerk", default=None, above=0.0)` alongside the existing keys. The bridge wire-up is `klippy/motion_bridge.py:179::MotionBridgeWrapper.init_planner`, which currently passes `max_velocity, max_accel, ...` into the Rust side; extend its signature to take `max_jerk` (Python-side `Optional[float]`) and forward it to PyO3 as `Option<f64>` (Python `None` ⇒ Rust `None` ⇒ Rust-side default).

### 5.3 Runtime limit updates — `SET_VELOCITY_LIMIT`

Today, `SET_VELOCITY_LIMIT VELOCITY=... ACCEL=...` in `klippy/motion_toolhead.py:469` forwards through `MotionBridgeWrapper.update_limits` (`klippy/motion_bridge.py:223`) into `rust/motion-bridge/src/bridge.rs::update_limits` (around `bridge.rs:1437`), which mutates `cfg.limits.max_velocity` and `cfg.limits.max_accel` and rebuilds the temporal limits via `to_temporal_limits`. Today, jerk is recomputed implicitly because `to_temporal_limits` derives it from `max_accel` inline.

Under this spec, `max_jerk` becomes a stored field. The runtime-update semantics must be specified explicitly; they are:

**`max_jerk` stays at its init-time value unless explicitly overridden.** Changing `max_accel` at runtime via `SET_VELOCITY_LIMIT ACCEL=...` does *not* recompute `max_jerk`. Reasoning: `max_jerk` is a real configuration knob (with a default derived from init-time accel) rather than a derived-from-current-accel quantity. Recomputing on every accel change would surprise users who set `max_jerk` deliberately; not recomputing is more predictable.

To override at runtime, the `SET_VELOCITY_LIMIT` g-code accepts a new optional `JERK=<float>` parameter. When present, it updates `cfg.limits.max_jerk`; when absent, jerk is preserved. Wire layer (in execution order):

1. `klippy/motion_toolhead.py:469::cmd_SET_VELOCITY_LIMIT` parses `JERK` via `gcmd.get_float("JERK", None, above=0.0)`.
2. `MotionBridgeWrapper.update_limits` signature extends to `(max_velocity, max_accel, max_jerk: Optional[float])`. Forwards through PyO3.
3. `rust/motion-bridge/src/bridge.rs::update_limits` accepts `max_jerk: Option<f64>`. When `Some(j)`, mutates `cfg.limits.max_jerk = j` before rebuilding temporal limits; when `None`, leaves `cfg.limits.max_jerk` untouched.

Effect: the existing two-arg `SET_VELOCITY_LIMIT VELOCITY=... ACCEL=...` continues to work unchanged (jerk preserved). Users who want to retune jerk live add `JERK=...`.

### 5.4 Effect on Z

Z's effective jerk goes from `2 × max_z_accel = 200` (today's hidden default) to `2 × max_accel = 6000` (the new global default). Per-axis `a_max[Z] = 100 mm/s²` continues to constrain Z's actual acceleration profile. The transient time to ramp Z accel from 0 to peak under the new jerk is `100/6000 ≈ 17 ms` — comfortably within Z screw-drive mechanical tolerance. Z motion is short and infrequent; no real-world print case is impacted.

### 5.5 Effect on β-medium loop

`rust/trajectory/src/beta.rs` is **not modified** by this change — the β code path is unchanged. However, β's converged trajectory will differ because it consumes `j_max` indirectly through `plan_batch` (`beta.rs:286-300` rebuilds temporal limits with `orig.limits.j_max` per iteration). With `j_max` going from `[6000, 6000, 200]` to `[6000, 6000, 6000]`, every β iteration solves a less-constrained TOPP-RA problem on Z-bearing moves. The β derate algorithm (mutate `planning_a_max` until shaped peak ≤ machine `a_max`) is unaffected; it just operates on a slightly different upstream profile. No β-medium test failures expected, but the converged trajectory's `total_time` may differ slightly (typically lower) on Z-bearing fixtures.

## 6. Test plan

### 6.1 Rust-layer regression — proves Stage 1 SLP convergence at the new default

Add `rust/trajectory/tests/homing_300mm_pure_x.rs`. Drives `trajectory::shape_batch` with the captured homing fixture inputs:
- segment: 300 mm pure-X collinear cubic (control points `[(-300,0,0), (-200,0,0), (-100,0,0), (0,0,0)]`)
- `feedrate_mm_s = 50.0`, `e_mode = Travel`
- `limits.j_max = [6000.0, 6000.0, 6000.0]` (the new MVP default for the sim's `max_accel = 3000`)
- `limits.v_max = [300, 300, 15]`, `limits.a_max = [3000, 3000, 100]` (unchanged from sim config)
- shaper: `SmoothMzv { frequency_hz: 50.0 }` on X/Y, `Passthrough` on Z (matches bridge default)
- `grid_strategy = Adaptive { min_n: 20, max_n: 200, target_grid_spacing_mm: 0.5 }` (matches bridge)

Asserts:
1. `result.is_ok()` (no `ShapeError`).
2. `output.temporal_status == JoiningStatus::Converged`.
3. `output.segments[0].profile.status` is one of `Solved | SolvedInexact | SolvedSlp` (no `Diverged*` / `MaxIter*`).
4. `output.beta_warning.is_none()`.

This test runs in `cargo test -p trajectory` and proves Stage 1 SLP convergence at the new uniform `j_max` *before* the sim integration test exercises the same code path. It is the architectural correctness gate; §6.2 below is the user-facing acceptance gate.

### 6.2 Sim integration test — `test_home_x.py`

`tools/sim_klippy/test_home_x.py` must succeed end-to-end. **Note: the existing test's pass condition (`test_home_x.py:96`) is non-discriminating against the failure mode it claims to test** — `r` is overwritten by the M114 response before the return-code computation, so the test currently exits 0 even when G28 produces `StalledOnInfeasibleSegment`. The implementation must fix this bug as part of the change.

Strengthened assertions:
1. The G28 response (captured *before* M114) must contain `result` and not `error`.
2. The M114 response must report `X = 0.0` (parse `pos["X"]` from the M114 result; assert `< 1e-6`).
3. `tools/sim_klippy/.local-logs/klippy.log` must not contain the substring `StalledOnInfeasibleSegment` after the test run.
4. The test exits 0 only when all three assertions pass.

### 6.3 Curved-fixture invariance

Existing `cargo test -p temporal -p trajectory` must remain green. Specifically called out:

- `rust/temporal/tests/conditioning.rs::rational_quadratic_arc_n200_solves_with_centripetal_cruise` — the curved-arc fixture that was breaking under the prior session's stencil-unification attempt. Verified during spec authoring: the fixture's `textbook_limits()` (lines 30-37) uses uniform `j_max = [100_000.0, 100_000.0, 100_000.0]`, so it is invariant under this change by construction.
- Full `cargo test -p temporal -p trajectory -p motion-bridge` — green.

### 6.4 Config layer unit tests

In `rust/motion-bridge/src/config.rs` tests (or a new `tests/config.rs`):
1. `PlannerLimits::to_temporal_limits` produces `[J, J, J]` for `j_max` given any `max_jerk = J`.
2. `PlannerConfig::default()` has `max_jerk = max_accel * 2.0`.
3. Round-trip a `PlannerLimits { max_jerk: 7500.0, .. }` and confirm `to_temporal_limits().j_max == [7500.0; 3]`.
4. **PyO3 init default**: `init_planner` called with `max_jerk: None` produces `cfg.limits.max_jerk = passed_in_max_accel * 2.0`. Called with `max_jerk: Some(j)` produces `cfg.limits.max_jerk = j`.
5. **Runtime update preservation**: `update_limits(v, a, max_jerk: None)` after init leaves `cfg.limits.max_jerk` unchanged. `update_limits(v, a, Some(j_new))` updates it to `j_new`.

### 6.5 Sim sanity print *(optional, not a release gate)*

After 6.2 passes, drive a manual sequence through `tools/sim_klippy/run.py`:
- `G28 X`
- `G1 X10 F1000`
- `G1 Z2 F300`
- `G1 X20 Y20 Z3 F1500`
- `M114`

Confirm no errors and reasonable step output. This is eyeball-level sanity, not an automated assertion.

## 7. Maintainer-doc updates

### 7.1 `constraints.rs:236-247`

The existing MAINTAINER WARNING about per-axis Cartesian-jerk SOC rows stays as-is — its concern is the SOCP relaxation layer, not the config layer. Append one paragraph noting that **the bridge config layer also collapses jerk to a single scalar for MVP** (with pointer to this spec), so today's `j_max = [J, J, J]` is uniform-by-construction and `J_path = min(j_max) = J` is the correct number for every move under this config. Future per-axis jerk distinction at the config layer must land alongside the SOCP-side per-axis Cartesian jerk work — the two are coupled.

### 7.2 Plan-changes-log entry

Add an entry to `docs/superpowers/plan-changes-log.md` (the running log referenced from `CLAUDE.md`):
- date: 2026-05-05
- what: bridge config layer adopts single scalar `max_jerk` for MVP; per-axis jerk *configuration* is deferred alongside the per-axis Cartesian-jerk SOCP work.
- why: Phase 4 homing unblock; Codex cross-check refuted the per-grid-projected-scalar `J_path[i]` proposal as a strict outer approximation on curved paths.
- evidence: this spec, `docs/research/stall-homing-move.md`.

### 7.3 No build-order step renumbering

The build-order in `CLAUDE.md` already has Step 8 (corner-blend + smooth-shaper expansion) and Step 10 (phase stepping); per-axis Cartesian jerk does not need its own numbered step today. When the future per-axis work is scoped, it gets its own brainstorm + spec + plan, and the plan-changes-log will record the build-order entry then.

## 8. Out of scope

- Per-axis jerk in `printer.cfg` (e.g. `max_jerk_x`, `max_jerk_y`, `max_jerk_z`).
- Per-axis Cartesian-jerk SOCP relaxation (the `constraints.rs:236-247` maintainer-warning territory).
- Decoupling Stage 2 from Stage 1's success status (the "Stage 2 runs even on Stage 1 Diverged" idea).
- Any change to Stage 1 / Stage 2 SLP, the verifier, or β-medium.
- Any change to extruder limits (`ELimits.v_max`, `ELimits.a_max`) — these are scalar today and stay scalar.
- Per-motor jerk limits (kinematics-aware projection — a future step well beyond per-axis).

## 9. Acceptance criteria

The change is complete when:
1. New Rust regression `rust/trajectory/tests/homing_300mm_pure_x.rs` exists and passes (Stage 1 SLP convergence at `j_max=[6000;3]` proven).
2. `tools/sim_klippy/test_home_x.py` exits 0 with M114-reported `X = 0.0` AND no `StalledOnInfeasibleSegment` substring in `klippy.log`. The existing test's pass-condition bug (variable shadowing of `r`) is fixed as part of the change.
3. `cargo test -p temporal -p trajectory -p motion-bridge` is green.
4. `rust/motion-bridge/src/config.rs` exposes `max_jerk: f64` with default `max_accel * 2.0` and produces uniform `j_max` from `to_temporal_limits`.
5. `init_planner` (Rust) accepts `max_jerk: Option<f64>`; PyO3 boundary maps Python `None` to Rust `None` to "use default".
6. `update_limits` (Rust + Python) accepts an optional `max_jerk`; absence preserves the stored value, presence overwrites.
7. Klippy-side `[printer]` config accepts an optional `max_jerk` key; absence falls back to the Rust default. `SET_VELOCITY_LIMIT` accepts an optional `JERK=` parameter with the same preserve-on-absence semantics.
8. `constraints.rs` MAINTAINER WARNING gains the one-paragraph note about config-layer collapse.
9. `docs/superpowers/plan-changes-log.md` records the change.

## 10. Future work (referenced, not scoped here)

When per-axis jerk distinction becomes physically motivated (e.g., a stiffer/lighter machine where Z genuinely tolerates less jerk than X/Y; or extruder-axis jerk physics distinct from carriage jerk), the proper fix per Codex's cross-check is:

1. **Replace Stage 1's scalar `J_path` SOC chain** with per-axis Cartesian-jerk SLP cuts directly (extending the existing `append_axis_jerk_cut_to_clarabel` machinery to be the only jerk gate).
2. **Decouple Stage 2 from Stage 1's success status** — Stage 2 runs on Stage 1's best-iterate-so-far even when Stage 1 marks Diverged, with Stage 2's per-axis Cartesian-jerk verifier as the final feasibility arbiter. This handles cases where Stage 1's scalar relaxation is too conservative on curved paths.
3. **Re-introduce per-axis `max_jerk_*` config keys** at the klippy and bridge layers, propagating into `PlannerLimits.j_max`.

Items 1–2 are the work the maintainer warning was already deferring; item 3 is the config-layer counterpart added by this spec. They land together as a single coherent change.
