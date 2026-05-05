# MVP global scalar jerk — design

**Date:** 2026-05-05
**Author:** Phase 4 homing-unblock brainstorm
**Status:** Design ready; implementation pending.
**Related:** `docs/research/stall-homing-move.md`, `rust/temporal/src/topp/constraints.rs:236-247` (maintainer warning), `docs/superpowers/specs/2026-04-27-layer-2-topp-prototype-design.md` §11.

## 1. Summary

Replace the per-axis configuration default `j_max = [max_accel*2, max_accel*2, max_z_accel*2]` in `rust/motion-bridge/src/config.rs::PlannerLimits::to_temporal_limits` with a **single global jerk scalar** `J` populating all three slots: `j_max = [J, J, J]`. Surface `J` as a new optional `max_jerk` field in `PlannerLimits`, defaulted from `max_accel * 2.0`. The `temporal::Limits.j_max: [f64; 3]` API is unchanged; the planner's two-stage SLP machinery (Stage 1 scalar path-jerk Lee 2024 cuts in block (h); Stage 2 per-axis Cartesian-jerk active-set SLP) is unchanged. Only the bridge config layer changes.

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

### 5.1 Rust side — `PlannerLimits`

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

### 5.2 Klippy side — `printer.cfg` surface

In the `[printer]` section, expose an optional `max_jerk` float key. Behavior:
- If present, parsed and passed through to the bridge as `PlannerLimits::max_jerk`.
- If absent, the Rust default (`max_accel * 2.0`) applies.
- No per-axis `max_jerk_x` / `max_jerk_y` / `max_jerk_z` keys for MVP — explicitly deferred.

The klippy-side parsing site is `klippy/motion_toolhead.py:133-140` (`config.getfloat("max_velocity", ...)` etc. for `[printer]` keys) — add `self.max_jerk = config.getfloat("max_jerk", default=None, above=0.0)` alongside the existing keys. The bridge wire-up is `klippy/motion_bridge.py:179::MotionBridgeWrapper.init_planner`, which currently passes `max_velocity, max_accel, ...` into the Rust side; extend its signature to take an optional `max_jerk` and forward it to PyO3 `PlannerLimits`. When `max_jerk is None` on the Python side, omit it from the PyO3 call so the Rust default (`max_accel * 2.0`) applies.

### 5.3 Effect on Z

Z's effective jerk goes from `2 × max_z_accel = 200` (today's hidden default) to `2 × max_accel = 6000` (the new global default). Per-axis `a_max[Z] = 100 mm/s²` continues to constrain Z's actual acceleration profile. The transient time to ramp Z accel from 0 to peak under the new jerk is `100/6000 ≈ 17 ms` — comfortably within Z screw-drive mechanical tolerance. Z motion is short and infrequent; no real-world print case is impacted.

## 6. Test plan

### 6.1 Homing regression — unblocks Phase 4

`tools/sim_klippy/test_home_x.py` must succeed end-to-end after the change:
1. G28 X submitted, planner runs `shape_batch` without error.
2. M114 reports `X = 0.0`.
3. No `StalledOnInfeasibleSegment` in `klippy.log`.

This is the binding test — the change ships only when this test passes.

### 6.2 Curved-fixture invariance

Existing `cargo test -p temporal -p trajectory` must remain green. Specifically called out:

- `rust/temporal/tests/conditioning.rs::rational_quadratic_arc_n200_solves_with_centripetal_cruise` — the curved-arc fixture that was breaking under the prior session's stencil-unification attempt. Today's fixture uses uniform `j_max = [100_000; 3]`, so it is invariant under this change by construction. We assert this explicitly: read the fixture, confirm uniform jerk, then run.
- `rust/trajectory/tests/stall_homing_move.rs` — the existing planner-side regression that already converges on `sota-motion`. Unchanged behavior expected.
- Full `cargo test -p temporal -p trajectory` — green.

### 6.3 Config layer unit tests

In `rust/motion-bridge/src/config.rs` tests (or a new `tests/config.rs`):
1. `PlannerLimits::to_temporal_limits` produces `[J, J, J]` for `j_max` given any `max_jerk = J`.
2. `PlannerConfig::default()` has `max_jerk = max_accel * 2.0`.
3. Round-trip a `PlannerLimits { max_jerk: 7500.0, .. }` and confirm `to_temporal_limits().j_max == [7500.0; 3]`.

### 6.4 Sim sanity print *(optional, not a release gate)*

After 6.1 passes, drive a manual sequence through `tools/sim_klippy/run.py`:
- `G28 X`
- `G1 X10 F1000`
- `G1 Z2 F300`
- `G1 X20 Y20 Z3 F1500`
- `M114`

Confirm no errors and reasonable step output. This is eyeball-level sanity, not an automated assertion.

## 7. Maintainer-doc updates

### 7.1 `constraints.rs:236-247`

The existing MAINTAINER WARNING about per-axis Cartesian-jerk SOC rows stays as-is — its concern is the SOCP relaxation layer, not the config layer. Append one paragraph noting that **the bridge config layer also collapses jerk to a single scalar for MVP** (with pointer to this spec), so today's `j_max = [J, J, J]` is uniform-by-construction and `J_path = min(j_max) = J` is the correct number for every move under this config. Future per-axis jerk distinction at the config layer must land alongside the SOCP-side per-axis Cartesian jerk work — the two are coupled.

### 7.2 `CLAUDE.md` plan-changes-log

Add an entry to `docs/superpowers/plan-changes-log.md`:
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
1. `tools/sim_klippy/test_home_x.py` exits 0 with M114 reporting `X = 0.0`.
2. `cargo test -p temporal -p trajectory -p motion-bridge` is green.
3. `rust/motion-bridge/src/config.rs` exposes `max_jerk: f64` with default `max_accel * 2.0` and produces uniform `j_max` from `to_temporal_limits`.
4. Klippy-side `[printer]` config accepts an optional `max_jerk` key; absence falls back to the Rust default.
5. `constraints.rs` MAINTAINER WARNING gains the one-paragraph note about config-layer collapse.
6. `docs/superpowers/plan-changes-log.md` records the change.

## 10. Future work (referenced, not scoped here)

When per-axis jerk distinction becomes physically motivated (e.g., a stiffer/lighter machine where Z genuinely tolerates less jerk than X/Y; or extruder-axis jerk physics distinct from carriage jerk), the proper fix per Codex's cross-check is:

1. **Replace Stage 1's scalar `J_path` SOC chain** with per-axis Cartesian-jerk SLP cuts directly (extending the existing `append_axis_jerk_cut_to_clarabel` machinery to be the only jerk gate).
2. **Decouple Stage 2 from Stage 1's success status** — Stage 2 runs on Stage 1's best-iterate-so-far even when Stage 1 marks Diverged, with Stage 2's per-axis Cartesian-jerk verifier as the final feasibility arbiter. This handles cases where Stage 1's scalar relaxation is too conservative on curved paths.
3. **Re-introduce per-axis `max_jerk_*` config keys** at the klippy and bridge layers, propagating into `PlannerLimits.j_max`.

Items 1–2 are the work the maintainer warning was already deferring; item 3 is the config-layer counterpart added by this spec. They land together as a single coherent change.
