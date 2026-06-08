# Faithful Position-at-Time Homing Seam — Design

**Goal:** Replace the fragile, mode-switched bridge-mode homing position machinery with a single pure primitive — "where was this motor at time *t*?" — evaluated against the planner's committed trajectory, with all kinematics owned solely by Rust. This deletes an entire class of recurring homing bugs and makes external probe modules (Beacon, `probe.py`, `z_tilt`, QGL) work through one honest seam.

**Architecture:** The Rust planner is the single source of truth for both the trajectory and the kinematic transform. The host holds *zero* kinematics and *zero* position state; it asks the engine. `get_mcu_position`/`get_past_mcu_position` become pure evaluations of a retained per-motor curve; every trip source (sensorless software trip, physical endstop, Beacon trsync) converges on the same `eval(slot, trip_clock)`.

**Tech stack:** Rust (`rust/motion-bridge`, `rust/geometry`/NURBS) compiled to the `motion_bridge_native` PyO3 cdylib; Python host (`klippy/stepper.py`, `klippy/mcu.py`, `klippy/motion_toolhead.py`, `klippy/extras/homing.py`). Cubic-Bézier-native trajectory.

**Scope:** Stage 1 (this spec) — discrete homing & probing for sensorless, physical, and Beacon-homing. Stage 2 (deferred) — continuous scanning, which widens retention to a rolling time-window and adds a batch query. The seam is designed so Stage 2 changes *only* retention, not the eval or anything above it.

---

## 1. Background: why this exists (the root cause)

Klipper's `homing.py` assumes a hardware step counter: `get_mcu_position()` is a *pure, stable* read, and `start_pos`/`halt_pos` are two reads of the same monotonic counter, so `(halt − start) × step_dist` is real distance (`homing.py:204-213`). The whole second-approach / `min_home_dist` decision (`homing.py:245-262, 364`) rides on that subtraction.

Our engine has no step counter — truth is the commanded trajectory. To satisfy `homing.py`, the fork made `get_mcu_position()` *lie*, returning different things depending on a global mutable flag (`_software_trip_active`/`_software_trip_clock`, set in `mcu.py:793-795`, cleared in three other files). `start_pos` and `halt_pos` became reads of *two different functions* whose correctness hinged on the flag's phase at one instant. Every homing bug to date — distance always 0, drain hang, trip-handler collision, `min_home_dist` ignored — is the **same** bug: the flag in the wrong phase when some reader touched it.

Compounding it, the toolhead↔motor kinematic transform is hand-written in **four** places that must agree forever:

| Location | Direction |
|---|---|
| `enqueue.rs:22-46` (drives the motors) | toolhead → motor |
| `stepper.py:214-232` `_calc_motor_position_from_xyz` | toolhead → motor |
| `motion_toolhead.py:252-268` `BridgeKinematics.set_position` | toolhead → motor |
| `motion_toolhead.py:184-203` `BridgeKinematics.calc_position` | motor → toolhead |

(plus a likely fourth, `motion_kinematics.py:7-15`). Adding any new kinematics (delta) means editing all of them correctly. This is the fragility engine.

## 2. The primitive

One pure function, owned by Rust:

```
eval_motor_position(mcu_handle, stepper_oid, print_time) -> f64   // motor-frame mm
```

It evaluates the **retained per-motor curve** for that stepper's slot at `print_time`. It is referentially transparent: same inputs → same output, no global flags, no hidden mode. Outside the retained curve's time range it clamps to the curve endpoint (the resting position is a degenerate constant curve — see §5).

Everything above it collapses onto this:

- `stepper.get_mcu_position()` → `eval` at "now" (latest committed time)
- `stepper.get_past_mcu_position(t)` → `eval` at `t`
- position-at-trip, for **every** trip source → `eval` at `clock_to_print_time(trip_clock)`

`start_pos` and `halt_pos` are again two evals of *one* continuous function — their difference is real distance, so `min_home_dist` is correct by construction, permanently.

## 3. Kinematics consolidated into Rust

A single Rust kinematics module owns both directions:

- **forward** `toolhead → motor[]` — used by `enqueue` (already exists as the CoreXY combination in `enqueue.rs`/`dispatch.rs::motor_frame_xy`) and by `set_position` grounding.
- **inverse** `motor[] → toolhead` — new, used by reporting (`calc_position`).

Both are pure functions, unit-tested directly, plus a **round-trip invariant test**: `inverse(forward(p)) ≈ p` for random `p`, which locks the two directions as true inverses and is the regression guard against a future kinematics drifting out of sync.

The host keeps **no** kinematics:

- `stepper.get_mcu_position`/`get_past_mcu_position` → motor-frame `eval` (no transform needed; motor frame comes straight out).
- `BridgeKinematics.calc_position(stepper_positions)` → collect per-stepper motor mm, call Rust **inverse**, return toolhead coords.
- `BridgeKinematics.set_position(newpos)` → tell Rust the toolhead position (already does, `bridge.set_position`), obtain per-slot motor positions from Rust **forward**, set the per-stepper scalar anchors.

Deleted outright: `_calc_motor_position_from_xyz` (`stepper.py:214-232`), the CoreXY literal in `set_position` (`motion_toolhead.py:252-268`), the CoreXY literal in `calc_position` (`motion_toolhead.py:184-203`), and `motion_kinematics.py`'s transform if delete-first proves it redundant. `_bridge_active_axes` **survives** — `is_active_axis` still needs axis membership for `z_tilt`/`quad_gantry_level`/homing; only its use as a kinematics input goes away.

Adding delta later = add a delta forward+inverse to the one Rust module. Homing position, probing, and Beacon are correct the same day, with zero host edits.

## 4. Trip-path unification

All position-at-trip derivation collapses to `eval`. Deleted:

- the global stash `_software_trip_active` / `_software_trip_clock` (`mcu.py:791-801`) and its clear-sites,
- the physical-trip step-count snapshot path: `bridge_set_position_from_step_count`, `_bridge_last_trip_step_count` (`stepper.py:234-238`), and the per-stepper snapshot application in `home_wait` (`mcu.py:803-822`).

`home_wait` keeps only: receive trip → read `trip_clock` → `return clock_to_print_time(trip_clock)` (`mcu.py:823-832`). `homing.py` then reads `get_past_mcu_position(trigger_time)` = `eval` at that time, identically for sensorless, physical, and Beacon. The MCU's reported step counts in physical-trip messages become unused (commanded-position-at-trigger is the model, exactly as mainline interpolates its trapq).

## 5. Retention & grounding

The engine always retains "the current trajectory" per `(mcu, slot)`:

- **During a homing/probing move** (an isolated `drip_move`): the move's motor-frame curves. `eval` within `[move_start, move_end]` is exact.
- **At rest / outside that range**: a degenerate constant curve at the grounded position. `eval` clamps to it. This is *not* a mode-switch — it is one pure `eval` over a trajectory that happens to be a point when idle.

Retention is replaced when the next move commits or on `set_position`. Memory is **O(slots × pieces-of-one-move)** — bounded and tiny, important for Pi-class hosts shipping to many users. **Stage 2** swaps "current move" for "rolling window of the last *W* seconds," evicted by horizon; nothing above `eval` changes.

**Grounding/anchor:** a per-stepper scalar `_mcu_position_offset` remains as the absolute zero-point anchor (`get_mcu_position = round((eval + offset) / step_dist)`). It cancels in distance subtractions (so `min_home_dist` never depends on it) and is re-established by `set_position`. The *time-varying* truth is `eval`; the anchor is a pure scalar. Exact grounding arithmetic is pinned in the implementation plan.

## 6. External compatibility contract (unchanged surface)

The `MCU_endstop` protocol stays signature-identical: `home_start`, `home_wait`, `query_endstop`, `add_stepper`, `get_steppers`, `get_mcu` (`mcu.py:632-840`). External modules see no change:

- **Beacon (not in-tree):** attaches its non-bridge MCU trsync via `add_classic_trsync` (`motion_bridge.py:575`); the bridge already relays it (`trip_dispatch_prepare`, `SourceSpec::Trsync`). Its trip yields a `trip_clock` like any other; `get_past_mcu_position(trigger_time)` is now an honest eval, so Beacon **homing** works unchanged. (Beacon **scanning** = Stage 2.)
- **`probe.py` / `PrinterProbe`:** `probing_move` → `HomingMove.homing_move(probe_pos=True)` → `get_past_mcu_position(trigger_time)` (`homing.py:474-489`, `homing.py:43`) → eval. Works unchanged.
- **`z_tilt` / `quad_gantry_level`:** rely on `is_active_axis` (preserved) and probing (above).

## 7. Execution method: delete-first, fail-loud

Per the repo's fail-loud rule and to make every consumer self-identify:

1. **Delete the host kinematics + trip-stash + snapshot code first.** Consumers then fail loudly — `AttributeError`/`NotImplementedError` at call time, never a silent wrong value.
2. **Enumerate consumers** by grep (static; Python won't surface them at compile time) **and** the test suite (dynamic). Ensure at least one test exercises the homing-position read so the hole lights up; config-specific paths (delta, idex) are found by grep since the suite may not run them.
3. **Build the Rust seam** (`eval`, forward+inverse kinematics, motor-frame retention) and repoint each failing host site at it.
4. **Nothing is run on hardware or shipped until the seam is whole.** We work in the `homing-rework` worktree; an intermediate non-importable tree is expected and fine.

## 8. Testing strategy

**Rust (pure, no hardware):**
- `eval(known Bézier, t)` → expected motor position at sampled `t`; clamp-before/after behavior.
- forward and inverse kinematics, per machine type; **round-trip `inverse(forward(p)) ≈ p`**.
- retention lifecycle: arm move → eval mid-move → replace → eval clamps to grounded point.
- `trip_clock → print_time → eval` produces the commanded position at that clock.

**Python:**
- homing distance from eval: `(halt − start)` yields the true traveled distance; `min_home_dist` shortfall triggers the second approach; an early/short trip fails loudly instead of pretending to home.
- `set_position` grounding: after grounding, `get_mcu_position` reads the set position; offset cancels in distance.
- trip routing per arm (existing `test/test_motion_bridge_trip_routing.py`) still green.

**Integration:** full `motion-bridge` suite; offline planner sim where applicable.

## 9. Alternatives considered

- **A — one kinematics copy per side (host inverse + Rust forward).** Deletes `_calc_motor_position_from_xyz` but keeps a consolidated host `calc_position`. Two locations, consistency-tested. Rejected: the host never sheds kinematics, so delta still needs a host edit, and "one source of truth" is not achieved. Chosen B per explicit direction: planner lives in one place.
- **C — keep step-count snapshots for physical/Beacon, eval only for sensorless.** Rejected: the snapshot path *cannot* serve Beacon (it never reports our bridge steppers' counts), so it is strictly less capable than eval — keeping it is the exact two-mechanisms trap being removed.
- **Re-ground the offset at each trip (varnished status quo).** Rejected: still "position is a constant except when special-cased"; does not generalize to scanning.

## 10. Open questions / risks

- **`motion_kinematics.py:7-15`** — confirm whether it is a live forward-transform on the move-feed path (a fourth copy that must also route to the one Rust source) or dead; delete-first resolves this.
- **`set_position` grounding arithmetic** — exact offset reconciliation between engine frame and mcu-step frame; pin in the plan with a test.
- **Mid-print `get_mcu_position` callers** — verify no non-homing consumer needs a live mid-stream value in Stage 1 (retention is scoped to the active homing/probing move + grounded rest); if one exists, note it for Stage 2 retention.
- **Delta/SCARA forward+inverse** are out of scope here; the architecture must not block them (it doesn't — they become one Rust module addition).
- **Physical-trip step counts** become unused — confirm no other consumer before deleting that message handling.
