# Restore host→MCU position-seed delivery (CoreXY motor-frame)

**Date:** 2026-05-30
**Status:** Design — approved in brainstorming, pending spec review
**Branch:** `simple-mcu-contract`

## Problem

On the bench (CoreXY: Octopus H723 drives motor A = axis 0 and motor B = axis 1;
F446 drives Z), the first move after establishing position moves wrong and now
hard-faults. Concretely:

- A pure +X jog moved ~45° diagonal (silent, pre-fault).
- After the `StepsPerSampleExceeded` fault landed (see below), the same operation
  hard-faults: the bench log showed `MCU 'bottom' shutdown: kalico runtime fault`,
  `fault_code 65226` (= −310), `fault_detail 171072` = axis 2 (Z), 40000 steps in
  one sample.

Both are the same underlying defect.

## Root cause

The MCU is a dumb per-axis evaluator: for each axis it evaluates an **absolute
motor-frame position** `p_end` from its piece ring, computes
`target_step_count = round(p_end / microstep_distance)`, and emits
`signed_steps = target − last_step_count`. The `last_step_count` baseline is
therefore load-bearing: it must agree with the absolute positions the pieces
encode.

That baseline is established two ways:
1. **Connect-time runtime reset** zeroes every axis's `last_step_count`.
2. **`set_position`** (driven by `G28` homing, `G92`, `SET_KINEMATIC_POSITION`)
   must re-establish it, in **motor frame**, via the `runtime_seed_position`
   MCU command. For CoreXY that means A = X+Y, B = X−Y; Z and Cartesian axes pass
   through.

The Task-8 push-pieces rewrite replaced the old dispatch closure (which drained a
`pending_seed` and sent `runtime_seed_position` with the per-MCU CoreXY transform
— see `git show sota-motion:rust/motion-bridge/src/bridge.rs`, the
`if cfg.kinematics == KINEMATICS_COREXY { (x+y, x−y) }` block) with an
enqueue-only closure that **never drains `pending_seed` and never sends the seed**.
`bridge.set_position` (`bridge.rs:2709`) still stores the seed in `pending_seed`;
nothing consumes it.

Consequence: after homing/`SET_KINEMATIC_POSITION` to a non-origin position, every
axis whose absolute position differs from the reset baseline (0) overruns on its
first sample. With the bench homing to **(150, 150)**: motor A's baseline should be
X+Y = 300 but stays 0 → ~48000-step first sample; Z's baseline should be ~50 mm
(40000 steps) but stays 0. Whichever axis the runtime evaluates first wins the
race — on the bench it was Z (bottom MCU). From a true origin (0,0,0) the bug is
invisible because the reset baseline already matches.

The pieces themselves are correct — `enqueue.rs` applies the CoreXY transform to
the piece stream post-shaper (A = X+Y, B = X−Y) and feeds both rings. Only the
**seed** was dropped.

### Already landed (the safety net, not the fix)

`StepsPerSampleExceeded = −310` (commit `70d0104cf`): a per-sample step delta
beyond `MAX_STEPS_PER_SAMPLE` (16) now hard-faults like `PieceStartInPast` instead
of silently reverting and freezing the axis. `fault_detail` packs the axis index
(bits 16..24) and the saturated step count (low 16 bits). This converts the silent
corruption into a loud, decodable crash. It is the backstop; this spec is the
actual repair.

## Goals

- Restore per-MCU `runtime_seed_position` delivery so the MCU baseline matches the
  piece stream after every `set_position`.
- Apply the correct per-MCU motor-frame transform (CoreXY → (x+y, x−y, z);
  Cartesian → (x, y, z)).
- Collapse the `A=X+Y / B=X−Y` mapping into one shared helper so the seed path and
  the piece path (`enqueue.rs`) cannot drift.

## Non-goals / out of scope

- **In-flight piece ordering / flush mechanics.** The host is responsible for
  flushing before it seeds. The case where a prior move's pieces are still in
  flight when `set_position` fires (e.g. a retract during homing) is explicitly not
  addressed here. (This is the reason the old code deferred via `pending_seed`; we
  are dropping that concern.)
- **Reworking absolute-vs-relative piece encoding.** The absolute-position +
  one-shot-seed design is fragile by nature (a single dropped seed silently
  corrupts everything), but restoring the seed plus the `StepsPerSampleExceeded`
  backstop is the chosen pragmatic fix.
- **Extruder follower** (E axis) seeding — separate work.
- Pump changes. The seed is deliberately **not** routed through the pump.

## Design

### Decision 1 — the seed is a direct control command, not a pump message

`runtime_seed_position` is handled by the MCU command dispatcher
(`engine.seed_position`), not the piece ring. It does not share the pump's
piece-stream queue. It is sent the same way `configure_axes` and `runtime_reset`
go out — directly to each MCU's IO — and lives next to them, not in the pump.

### Decision 2 — sent from `set_position`, no extra argument

`set_position` *is* the operation "the machine is now at this position"; seeding
the MCU baseline is the MCU-side half of that same statement. The send is folded
into the existing `bridge.set_position`, inside the `if let Some(planner)` block
that already gates the stream re-anchor.

No `skip`/`seed` argument is needed. In any config that can move, `_init_planner`
runs on `klippy:connect` and the planner is up before any gcode/homing
`set_position`. The only no-planner case is a config with **no kalico-native
motion MCU** (`_init_planner` logs "skipping init_planner" when `octopus is None`,
e.g. a Beacon-only setup) — that config physically cannot drive steppers, so
seeding is moot. The seed therefore reuses the planner-present guard and is a
no-op exactly when the neighboring `kalico_stream_open` is.

### Decision 3 — one shared motor-frame helper

The `(x, y) → (a, b)` CoreXY mapping currently lives in `enqueue.rs` (piece
transform) and `motion_kinematics.motor_deltas` (Python, stepper enable), and
`motion_toolhead.set_position` computes it again for host stepper bookkeeping.
Add a single Rust helper (in `dispatch.rs`) that owns the per-MCU decision and the
scalar transform, and have both the seed path and `enqueue.rs` use it.

Sketch:
```rust
/// True when this MCU drives both CoreXY motors and should receive motor-frame
/// (A,B) values rather than Cartesian (X,Y).
pub fn cfg_is_corexy(cfg: &McuAxisConfig) -> bool {
    cfg.kinematics == KINEMATICS_COREXY
        && cfg.axes.contains(&AXIS_X)
        && cfg.axes.contains(&AXIS_Y)
}

/// Map a Cartesian (x, y) to this MCU's motor frame:
/// CoreXY → (x+y, x−y); otherwise passthrough (x, y).
pub fn motor_frame_xy(cfg: &McuAxisConfig, x: f64, y: f64) -> (f64, f64) {
    if cfg_is_corexy(cfg) { (x + y, x - y) } else { (x, y) }
}
```
`enqueue.rs` keeps its NURBS-curve transform but gates it through `cfg_is_corexy`;
the seed path uses `motor_frame_xy` on the scalar position. Z is always
passthrough.

### Component: `bridge.set_position`

Inside the existing `if let Some(planner)` block:

1. For each `McuAxisConfig` in the stored `mcu_axis_configs`:
   - `(a, b) = motor_frame_xy(cfg, x, y)`; `z` passthrough.
   - Q16-encode each (`mm * 65536`, round, clamp to `i32`).
   - Send `runtime_seed_position x_q16=… y_q16=… z_q16=…` to that MCU's IO
     (the same send mechanism `configure_axes` uses).
2. Retire the `pending_seed` field and `SeedPosition` struct — their sole purpose
   (deferring the send for the in-flight case) is now out of scope. Implementation
   must first confirm `pending_seed` has no other consumer (it currently has none
   beyond `set_position`'s store; the homing-active flag tracked nearby is a
   separate field and is unaffected).

The wire field names are confirmed against HEAD:
`runtime_seed_position x_q16=%i y_q16=%i z_q16=%i` (`src/runtime_commands.c:286`),
Q16.16 mm; the FFI seeds the engine's per-axis `last_step_count` directly in motor
frame (`runtime_ffi.rs:598`, `engine.seed_position`). The MCU stays dumb — the host
hands it motor-frame values; no transform is added on the MCU.

### Data flow

```
SET_KINEMATIC_POSITION / G28 / G92
  → motion_toolhead.set_position(newpos, homing_axes)
  → bridge.set_position(x, y, z)                         [Rust pyo3]
      if planner up:
        for cfg in mcu_axis_configs:
          (a,b) = motor_frame_xy(cfg, x, y); z passthrough
          send runtime_seed_position(enc(a), enc(b), enc(z)) → cfg's MCU
      else: skip (motion-less config)
  → MCU command dispatcher → engine.seed_position → last_step_count[axis] set
```

## Error handling

- **Planner up but an MCU config or IO handle is missing where it must exist →
  panic.** This is a broken invariant (`init_planner` guarantees configs + IO),
  and we want it loud, per the "always fail loudly" decision.
- **Planner absent → skip** (no panic). This is the motion-less config; it already
  logs a warning at `_init_planner` and cannot move, so there is nothing to seed.
  Consistent with the adjacent `kalico_stream_open` guard.
- **Backstop:** if a seed is ever missed despite this, `StepsPerSampleExceeded`
  (−310) hard-faults on the first oversized sample rather than producing corrupted
  motion.

## Testing

**Unit (Rust):**
- `cfg_is_corexy` / `motor_frame_xy`: CoreXY cfg → (x+y, x−y); Cartesian cfg →
  (x, y). Z passthrough.
- Q16 encoding round-trips a known value (e.g. 300.0 mm).
- `enqueue.rs` still produces identical motor-frame pieces after refactoring to use
  the shared predicate (existing `corexy_x_slot_is_x_plus_y` test must stay green).

**Integration (Rust, host side):**
- `bridge.set_position(150, 150, 50)` with the bench config sends, for the Octopus
  (CoreXY), `x_q16 = enc(300)`, `y_q16 = enc(0)`; for the F446, `z_q16 = enc(50)`
  passthrough. Assert one `runtime_seed_position` per configured motion MCU with the
  expected encoded values.
- Panic when the planner is initialized but an MCU's IO/config is missing.

**Bench (manual, after flashing both MCUs):**
- `SET_KINEMATIC_POSITION X=150 Y=150` (or home), then jog X → pure +X motion, no
  diagonal, **no** `StepsPerSampleExceeded` fault.
- Jog Z and a diagonal XY move → correct motion, no fault.
- Confirm a `runtime_seed_position` send appears in the bench log at
  `SET_KINEMATIC_POSITION` time with `x_q16/y_q16/z_q16` matching the transform.

## Files touched (anticipated)

- `rust/motion-bridge/src/dispatch.rs` — shared `cfg_is_corexy` / `motor_frame_xy`.
- `rust/motion-bridge/src/enqueue.rs` — use the shared predicate (behavior
  unchanged).
- `rust/motion-bridge/src/bridge.rs` — `set_position` sends the seed; retire
  `pending_seed` / `SeedPosition`.

No MCU/C changes (the seed command and engine path already exist and are correct).
No Python changes required (the existing `motion_toolhead.set_position →
bridge.set_position(x,y,z)` call is the trigger).
