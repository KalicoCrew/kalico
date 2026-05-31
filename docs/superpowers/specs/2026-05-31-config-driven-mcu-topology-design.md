# Config-driven MCU topology in `init_planner`

**Date:** 2026-05-31
**Status:** Approved (design)

## Problem

The planner's MCU topology is hardcoded across two layers instead of read from
the printer config:

- **Python** (`klippy/motion_toolhead.py:_init_planner`): locates exactly two
  MCUs by name (`octopus`/`f446`), passes their handles positionally, and falls
  back to `f446 = octopus` when only one MCU exists.
- **Rust** (`rust/motion-bridge/src/bridge.rs:init_planner`): takes two named
  `octopus_handle` / `f446_handle` parameters, fetches caps as a hand-unrolled
  `(octopus_caps, f446_caps)` tuple, and builds `mcu_configs` as a fixed
  two-element vec with literal axis lists and kinematics tags
  (octopus → `[X,Y]` kinematics `0` CoreXyAndE; f446 → `[Z]` kinematics `1`
  CartesianXyzAndE).

The host already knows the full topology: the global `kinematics` setting
(loaded mainline-style from `[printer] kinematics`) and the stepper→MCU
assignment (each stepper's pins resolve to an MCU via `stepper.get_mcu()`,
already trusted by `_configure_axes_per_mcu`). The hardcoding *reconstructs*
known facts as literals.

## Goal

Derive the per-MCU topology from existing config and pass it through, removing
all hardcoded MCU names, axis lists, and kinematics tags. Behavior for the
current two-MCU corexy bench must be preserved (modulo the deliberate E-axis
addition below). The change generalizes to 1-MCU, N-MCU, and cartesian
printers with no further edits.

## Non-goals

- E-axis curve dispatch (shaping E as its own NURBS curve) is **not**
  implemented here. This design only ensures E is placed on the correct MCU in
  the derived topology so that no topology rework is needed when E dispatch
  lands.
- No changes to the wire ABI, dispatch closure, `build_push_params`, the corexy
  transform, or `_configure_axes_per_mcu`.

## Design

### Principle

Stop reconstructing the topology with literals. Derive it from the global
`kinematics` setting plus the stepper→MCU assignment, then pass a per-MCU
descriptor list across the PyO3 boundary.

### Layer 1 — Python (`motion_toolhead.py:_init_planner`)

Replace the octopus/f446 name-matching and the `f446 = octopus` fallback with a
generic derivation:

1. **Axis→MCU grouping.** Using the existing slot map
   `[stepper_x→X(0), stepper_y→Y(1), stepper_z→Z(2), extruder→E(3)]`, for each
   kinematic stepper — **including the extruder** — read `stepper.get_mcu()` and
   its bridge handle, and group axis indices by MCU handle. This is the same
   `get_mcu()` truth `_configure_axes_per_mcu` already relies on.
2. **Per-MCU kinematics tag** (mainline-derived rule, no hardcode):

   ```
   tag = COREXY(0)    if global kinematics == "corexy" and {X, Y} ⊆ mcu_axes
       = CARTESIAN(1) otherwise
   ```

   This reproduces today's literals (the XY-carrying MCU → `0`, a Z-only MCU →
   `1` on a corexy printer) and generalizes: cartesian printers yield all-`1`;
   a single-MCU printer yields one descriptor carrying all its axes with tag
   `0` when corexy applies.
3. **Build the descriptor list** `[(handle, sorted_axes, tag), …]` and pass it
   to the bridge. The `f446 = octopus` fallback disappears — a one-MCU printer
   simply produces one descriptor.

**E is populated now.** It is inert downstream (see Layer 2's range-skip) until
E-curve shaping exists, at which point it dispatches to the extruder's MCU with
no topology change required.

### Layer 2 — Rust (`bridge.rs:init_planner`)

- Drop the `octopus_handle` / `f446_handle` parameters. Add a single
  `mcus: Vec<(u32, Vec<u8>, u8)>` parameter: `(handle, axes, kinematics_tag)`.
- Replace the hand-unrolled `(octopus_caps, f446_caps)` fetch and the fixed
  two-element `mcu_configs` vec with a loop over `mcus`: per entry, pull
  `runtime_caps` for that handle (same large-profile-default fallback as today),
  and build one `McuAxisConfig`. Everything downstream
  (`host_ios`, `kalico_native_for_plans`, the dispatch closure) already iterates
  `mcu_configs`, so no further Rust changes are needed.
- Add `pub const AXIS_E: usize = 3;` in `dispatch.rs` alongside `AXIS_X`/
  `AXIS_Y`/`AXIS_Z`, completing the axis vocabulary.
- Rewrite the `init_planner` doc comment from "two-MCU first-print MVP topology"
  to "N-MCU host-supplied topology."

### Layer 3 — Python wrapper (`motion_bridge.py:init_planner`)

Change the wrapper signature to forward the descriptor list instead of two
positional handles.

### Why E is inert today

The only consumer of `cfg.axes` is the `build_push_params` loop at
`dispatch.rs:371`, which guards every axis with
`if axis_idx >= shaped.axes.len() { continue; }` (`dispatch.rs:372`).
`shaped.axes` is `[X, Y, Z]` (3 entries) today, so an `AXIS_E = 3` entry is
silently skipped. The corexy check at `dispatch.rs:346-350` only inspects X/Y.
When E-curve shaping lands and `shaped.axes` grows a 4th entry, the existing
descriptor dispatches E to the extruder's MCU automatically.

## Behavior change (deliberate)

The derived topology includes E on the MCU carrying the extruder stepper —
e.g. the octopus descriptor becomes `[X, Y, E]` where the old literals had
`[X, Y]`. This is inert today (range-skip) and avoids revisiting topology
assembly when E shaping is implemented. No other behavior changes.

## Testing

- **Python derivation** unit test (`tools/` or alongside existing motion
  toolhead tests):
  - corexy, 2 MCUs, extruder on octopus → `[(h0,[X,Y,E],0), (h1,[Z],1)]`
  - cartesian, 1 MCU → `[(h0,[X,Y,Z,E],1)]`
  - corexy, 1 MCU → `[(h0,[X,Y,Z,E],0)]`
- **Rust** (`rust/motion-bridge/tests/bridge_to_runtime_step_chain.rs` and the
  `sim_motion_jogs` dispatch mirror): update call sites to the list form; assert
  `mcu_configs` built from `[(h0,[X,Y],0), (h1,[Z],1)]` matches the old literals,
  and that an added `AXIS_E` is skipped given 3-axis `shaped`.

## Files touched

- `klippy/motion_toolhead.py` (`_init_planner`)
- `klippy/motion_bridge.py` (`init_planner` wrapper)
- `rust/motion-bridge/src/bridge.rs` (`init_planner`)
- `rust/motion-bridge/src/dispatch.rs` (`AXIS_E` constant)
- `rust/motion-bridge/tests/bridge_to_runtime_step_chain.rs`
- `rust/motion-bridge/tests/sim_motion_jogs.rs`
- `tools/test_renode_phase2_gate.py` (`init_planner` call site)
