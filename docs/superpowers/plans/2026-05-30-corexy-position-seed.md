# CoreXY Position-Seed Delivery — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restore host→MCU `runtime_seed_position` delivery so the MCU's motor-frame `last_step_count` baseline is re-established (with the per-MCU CoreXY transform) after every `set_position`, fixing the diagonal/over-step fault.

**Architecture:** The seed is a direct fire-and-forget MCU command (not a pump message), sent from `bridge.set_position` inside the existing planner-present guard, fanned out per configured MCU. The `A=X+Y / B=X−Y` decision lives in one shared helper used by both the seed path and `enqueue.rs`. The MCU/C side and Python side are unchanged.

**Tech Stack:** Rust (`motion-bridge`, `kalico-host-rt`), PyO3 bridge, `cargo` workspace at `rust/`.

**Spec:** `docs/superpowers/specs/2026-05-30-corexy-position-seed-design.md`

**Reference (what HEAD dropped):** `git show sota-motion:rust/motion-bridge/src/bridge.rs` — the dispatch-closure block that drained `pending_seed` and sent `runtime_seed_position` with `if corexy { (x+y, x−y) }`.

---

## Key facts (verified against HEAD)

- MCU command exists: `runtime_seed_position x_q16=%i y_q16=%i z_q16=%i` (Q16.16 mm), `src/runtime_commands.c:286`; FFI seeds per-axis `last_step_count` in motor frame (`engine.seed_position`). **No MCU/C change needed.**
- Typed fire-and-forget send: `KalicoHostIo::send_typed(name, &[(field, FieldValue)]) -> Result<(), TransportError>` (`rust/kalico-host-rt/src/host_io/mod.rs:702`). `%i` fields take `FieldValue::I32`.
- `FieldValue` import path in `bridge.rs`: `kalico_host_rt::host_io::parser::FieldValue` (the parser module is already imported at `bridge.rs:19`).
- Per-MCU configs: `self.mcu_axis_configs: Mutex<Vec<McuAxisConfig>>` (`bridge.rs:388`). `McuAxisConfig { mcu_id: u32, axes: Vec<usize>, kinematics: u8, caps }` (`dispatch.rs`).
- Per-MCU IO: `self.mcus.lock().get(&mcu_id)` → `McuConnection { host_io: Option<Arc<KalicoHostIo>>, kalico_native_supported: bool, .. }` (`bridge.rs:61`).
- `pending_seed`/`SeedPosition` have exactly one consumer: the store in `set_position` (`bridge.rs:43,406,675,2742`). No other reader — safe to retire.
- Existing constants in `dispatch.rs`: `KINEMATICS_COREXY` (=0), `AXIS_X` (0), `AXIS_Y` (1), `AXIS_Z` (2).
- `enqueue.rs` current CoreXY gate (`enqueue.rs:66-70`):
  ```rust
  let corexy = cfg.kinematics == KINEMATICS_COREXY
      && cfg.axes.contains(&AXIS_X)
      && cfg.axes.contains(&AXIS_Y)
      && AXIS_X < seg.axes.len()
      && AXIS_Y < seg.axes.len();
  ```

## File structure

- **Modify** `rust/motion-bridge/src/dispatch.rs` — add `cfg_is_corexy`, `motor_frame_xy`, `encode_q16`, `SeedSend`, `build_seed_sends` + unit tests. (One responsibility: per-MCU axis config + the motor-frame mapping derived from it.)
- **Modify** `rust/motion-bridge/src/enqueue.rs` — use `cfg_is_corexy` instead of the inline gate (behavior unchanged).
- **Modify** `rust/motion-bridge/src/bridge.rs` — `set_position` sends the seed; remove the `pending_seed` store; remove the `SeedPosition` struct + field + init.

All commands run from `rust/` unless noted.

---

## Task 1: Shared CoreXY predicate + scalar transform

**Files:**
- Modify: `rust/motion-bridge/src/dispatch.rs` (add functions + `#[cfg(test)] mod` if not present)

- [ ] **Step 1: Write the failing tests**

Add to `rust/motion-bridge/src/dispatch.rs` (append a test module at end of file; if a `#[cfg(test)] mod tests` already exists, add these into it):

```rust
#[cfg(test)]
mod seed_tests {
    use super::*;

    fn corexy_cfg() -> McuAxisConfig {
        McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X, AXIS_Y, AXIS_E],
            kinematics: KINEMATICS_COREXY, // 0
            caps: McuCaps { total_piece_memory: 62 * 1024 },
        }
    }
    fn cartesian_z_cfg() -> McuAxisConfig {
        McuAxisConfig {
            mcu_id: 2,
            axes: vec![AXIS_Z],
            kinematics: 1, // CartesianXyzAndE
            caps: McuCaps { total_piece_memory: 62 * 1024 },
        }
    }

    #[test]
    fn cfg_is_corexy_true_only_for_corexy_xy_mcu() {
        assert!(cfg_is_corexy(&corexy_cfg()));
        assert!(!cfg_is_corexy(&cartesian_z_cfg()));
    }

    #[test]
    fn motor_frame_xy_transforms_corexy_passes_through_cartesian() {
        // CoreXY: A = x+y, B = x-y.
        assert_eq!(motor_frame_xy(&corexy_cfg(), 150.0, 150.0), (300.0, 0.0));
        assert_eq!(motor_frame_xy(&corexy_cfg(), 10.0, 4.0), (14.0, 6.0));
        // Cartesian: passthrough.
        assert_eq!(motor_frame_xy(&cartesian_z_cfg(), 150.0, 150.0), (150.0, 150.0));
    }
}
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p motion-bridge cfg_is_corexy_true_only_for_corexy_xy_mcu motor_frame_xy_transforms_corexy_passes_through_cartesian`
Expected: FAIL to compile — `cfg_is_corexy`/`motor_frame_xy` not found.

- [ ] **Step 3: Implement the helpers**

Add to `rust/motion-bridge/src/dispatch.rs` (after the `AXIS_*`/`KINEMATICS_COREXY` consts, before `McuAxisConfig` or right after it — top-level `pub fn`s):

```rust
/// True when this MCU drives both CoreXY motors and must receive motor-frame
/// `(A, B)` values rather than Cartesian `(X, Y)`. Single source of truth for
/// the CoreXY decision, shared by the piece path (`enqueue.rs`) and the seed
/// path (`build_seed_sends`) so they cannot drift.
pub fn cfg_is_corexy(cfg: &McuAxisConfig) -> bool {
    cfg.kinematics == KINEMATICS_COREXY
        && cfg.axes.contains(&AXIS_X)
        && cfg.axes.contains(&AXIS_Y)
}

/// Map a Cartesian `(x, y)` into this MCU's motor frame:
/// CoreXY → `(x + y, x − y)`; otherwise passthrough `(x, y)`. Z is always
/// passthrough and handled by the caller.
pub fn motor_frame_xy(cfg: &McuAxisConfig, x: f64, y: f64) -> (f64, f64) {
    if cfg_is_corexy(cfg) {
        (x + y, x - y)
    } else {
        (x, y)
    }
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p motion-bridge cfg_is_corexy_true_only_for_corexy_xy_mcu motor_frame_xy_transforms_corexy_passes_through_cartesian`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/dispatch.rs
git commit -m "feat(motion-bridge): shared cfg_is_corexy + motor_frame_xy helpers"
```

---

## Task 2: q16 encoder + per-MCU seed builder

**Files:**
- Modify: `rust/motion-bridge/src/dispatch.rs`

- [ ] **Step 1: Write the failing tests**

Add to the `seed_tests` module in `rust/motion-bridge/src/dispatch.rs`:

```rust
    #[test]
    fn encode_q16_is_mm_times_65536_rounded() {
        assert_eq!(encode_q16(0.0), 0);
        assert_eq!(encode_q16(50.0), 3_276_800);     // 50 * 65536
        assert_eq!(encode_q16(150.0), 9_830_400);    // 150 * 65536
        assert_eq!(encode_q16(300.0), 19_660_800);   // 300 * 65536
    }

    #[test]
    fn build_seed_sends_applies_per_mcu_transform() {
        // Bench topology: Octopus CoreXY [X,Y,E] (mcu 1), F446 Cartesian [Z] (mcu 2).
        let configs = vec![corexy_cfg(), cartesian_z_cfg()];
        // Position (150, 150, 50): A=x+y=300, B=x-y=0; F446 passthrough (150,150,50).
        let sends = build_seed_sends(&configs, 150.0, 150.0, 50.0);
        assert_eq!(sends.len(), 2);

        let octo = sends.iter().find(|s| s.mcu_id == 1).expect("octopus seed");
        assert_eq!(octo.x_q16, encode_q16(300.0)); // motor-A = X+Y
        assert_eq!(octo.y_q16, encode_q16(0.0));   // motor-B = X-Y
        assert_eq!(octo.z_q16, encode_q16(50.0));  // Z passthrough

        let z = sends.iter().find(|s| s.mcu_id == 2).expect("f446 seed");
        assert_eq!(z.x_q16, encode_q16(150.0));    // cartesian passthrough
        assert_eq!(z.y_q16, encode_q16(150.0));
        assert_eq!(z.z_q16, encode_q16(50.0));
    }
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cargo test -p motion-bridge encode_q16_is_mm_times_65536_rounded build_seed_sends_applies_per_mcu_transform`
Expected: FAIL to compile — `encode_q16`/`build_seed_sends`/`SeedSend` not found.

- [ ] **Step 3: Implement the encoder, struct, and builder**

Add to `rust/motion-bridge/src/dispatch.rs` (top-level):

```rust
/// One MCU's motor-frame seed, already Q16.16-encoded for the
/// `runtime_seed_position` wire command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SeedSend {
    pub mcu_id: u32,
    pub x_q16: i32,
    pub y_q16: i32,
    pub z_q16: i32,
}

/// Encode millimetres as Q16.16 fixed point (the `runtime_seed_position` wire
/// format), rounding to nearest and clamping into `i32` range.
pub fn encode_q16(mm: f64) -> i32 {
    let raw = mm * 65536.0;
    raw.round().clamp(i32::MIN as f64, i32::MAX as f64) as i32
}

/// Build one [`SeedSend`] per configured MCU: apply the per-MCU motor-frame
/// transform to `(x, y)` (Z always passthrough) and Q16.16-encode. Pure — the
/// caller performs the actual `runtime_seed_position` send.
pub fn build_seed_sends(configs: &[McuAxisConfig], x: f64, y: f64, z: f64) -> Vec<SeedSend> {
    configs
        .iter()
        .map(|cfg| {
            let (mx, my) = motor_frame_xy(cfg, x, y);
            SeedSend {
                mcu_id: cfg.mcu_id,
                x_q16: encode_q16(mx),
                y_q16: encode_q16(my),
                z_q16: encode_q16(z),
            }
        })
        .collect()
}
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cargo test -p motion-bridge encode_q16_is_mm_times_65536_rounded build_seed_sends_applies_per_mcu_transform`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/dispatch.rs
git commit -m "feat(motion-bridge): build_seed_sends + encode_q16 for per-MCU seed"
```

---

## Task 3: Refactor `enqueue.rs` to the shared predicate (no behavior change)

**Files:**
- Modify: `rust/motion-bridge/src/enqueue.rs:66-70`

- [ ] **Step 1: Replace the inline gate**

In `rust/motion-bridge/src/enqueue.rs`, replace:

```rust
        let corexy = cfg.kinematics == KINEMATICS_COREXY
            && cfg.axes.contains(&AXIS_X)
            && cfg.axes.contains(&AXIS_Y)
            && AXIS_X < seg.axes.len()
            && AXIS_Y < seg.axes.len();
```

with:

```rust
        // Shared predicate (dispatch::cfg_is_corexy) owns the "is this MCU
        // CoreXY" decision so the piece path and the seed path cannot drift.
        // The segment-arity check stays here — it is specific to the curve path.
        let corexy = crate::dispatch::cfg_is_corexy(cfg)
            && AXIS_X < seg.axes.len()
            && AXIS_Y < seg.axes.len();
```

- [ ] **Step 2: Fix the now-unused import (if the compiler warns)**

`KINEMATICS_COREXY` may become unused in `enqueue.rs`. Update the `use` at `enqueue.rs:23`:

```rust
use crate::dispatch::{McuAxisConfig, AXIS_X, AXIS_Y};
```

(Keep `AXIS_X`/`AXIS_Y` — still used for slot matching and the arity check. Drop `KINEMATICS_COREXY` only if the build flags it unused.)

- [ ] **Step 3: Run the enqueue tests to verify unchanged behavior**

Run: `cargo test -p motion-bridge corexy_x_slot_is_x_plus_y cartesian_x_axis_yields_pieces_with_projected_start_time`
Expected: PASS (2 tests) — the existing CoreXY/Cartesian enqueue tests live in `enqueue.rs`'s own `#[cfg(test)] mod tests` and must stay green after the gate refactor.

- [ ] **Step 4: Build to confirm no warnings/errors**

Run: `cargo build -p motion-bridge 2>&1 | tail -20`
Expected: `Finished` with no `unused import` warning for `enqueue.rs`.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/enqueue.rs
git commit -m "refactor(motion-bridge): enqueue uses shared cfg_is_corexy gate"
```

---

## Task 4: Send the seed from `set_position`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (the `set_position` method, ~`bridge.rs:2709`)

This is the wiring task. The pure logic is already covered by Task 2's `build_seed_sends` tests; this step is verified by the build + the bench checklist (Task 6). No unit test is added here because the method is a PyO3 entry point that requires a live `KalicoHostIo`.

- [ ] **Step 1: Add the `FieldValue` import**

At the top of `rust/motion-bridge/src/bridge.rs`, the parser module is already imported at line 19:
```rust
use kalico_host_rt::host_io::parser::{DataDictionary, MsgProtoParser};
```
Extend it to include `FieldValue`:
```rust
use kalico_host_rt::host_io::parser::{DataDictionary, FieldValue, MsgProtoParser};
```

- [ ] **Step 2: Replace the `pending_seed` store with the seed send**

In `set_position` (`bridge.rs`), the body currently contains the `if let Some(planner)` block followed by the `pending_seed` store. Replace **both** the `pending_seed` comment block and its store, and fold the send into the planner guard. Specifically, change:

```rust
        if let Some(planner) = self.planner.get() {
            planner
                .kalico_stream_open([x, y, z, 0.0])
                .map_err(planner_err)?;
        }

        // Seed the MCU engine's prev_x/y/z so the first segment after
        // SET_KINEMATIC_POSITION computes its delta against the correct
        // origin rather than the boot-time (0, 0, 0). Without this the
        // delta for a move starting at e.g. Y=100 is computed as
        // (Y_end - 0) instead of (Y_end - 100), which exceeds
        // MAX_STEPS_PER_TICK_DEFAULT and raises FaultCode::StepBurstExceeded.
        //
        // We do NOT send `runtime_seed_position` here directly.  In-flight
        // segments from a previous move (e.g. a retract queued during homing)
        // may not have reached the MCU yet.  Firing the seed immediately would
        // overwrite the MCU's `prev_x/y/z` before the retract finishes,
        // corrupting its step-delta computation.
        //
        // Instead, store the seed as `pending_seed`.  The dispatch closure
        // (planner thread) drains it before sending the next segment, which
        // guarantees the seed arrives AFTER all previously-dispatched segments.
        *self.pending_seed.lock().unwrap_or_else(|p| p.into_inner()) =
            Some(SeedPosition { x, y, z });
```

to:

```rust
        if let Some(planner) = self.planner.get() {
            planner
                .kalico_stream_open([x, y, z, 0.0])
                .map_err(planner_err)?;

            // Re-establish each MCU's motor-frame `last_step_count` baseline so
            // the first move after homing / G92 / SET_KINEMATIC_POSITION computes
            // a correct step delta. The connect-time runtime reset zeroed every
            // baseline; without this the delta for a move starting at e.g.
            // motor-A = X+Y = 300 is computed against 0 and trips
            // StepsPerSampleExceeded. The CoreXY transform (A=X+Y, B=X-Y) lives
            // on the host (shared `dispatch::cfg_is_corexy`); the MCU stays dumb.
            //
            // This is a direct fire-and-forget command (NOT a pump message): the
            // MCU handles it via its command dispatcher, not the piece ring. The
            // host is responsible for flushing before re-seeding; ordering against
            // in-flight pieces is out of scope (see the design spec).
            let sends = {
                let configs = self
                    .mcu_axis_configs
                    .lock()
                    .unwrap_or_else(|p| p.into_inner());
                crate::dispatch::build_seed_sends(&configs, x, y, z)
            };
            let mcus = self.mcus.lock().unwrap_or_else(|p| p.into_inner());
            for s in sends {
                // Planner is up ⇒ init_planner guaranteed these configs come from
                // real, attached MCUs. A missing connection/io here is a broken
                // invariant — fail loudly.
                let conn = mcus.get(&s.mcu_id).unwrap_or_else(|| {
                    panic!(
                        "set_position seed: planner up but mcu_id {} absent \
                         (broken invariant)",
                        s.mcu_id
                    )
                });
                let io = conn.host_io.as_ref().unwrap_or_else(|| {
                    panic!(
                        "set_position seed: mcu_id {} has no host_io \
                         (broken invariant)",
                        s.mcu_id
                    )
                });
                io.send_typed(
                    "runtime_seed_position",
                    &[
                        ("x_q16", FieldValue::I32(s.x_q16)),
                        ("y_q16", FieldValue::I32(s.y_q16)),
                        ("z_q16", FieldValue::I32(s.z_q16)),
                    ],
                )
                .map_err(|e| {
                    PyRuntimeError::new_err(format!(
                        "set_position seed send to mcu_id {} failed: {e:?}",
                        s.mcu_id
                    ))
                })?;
            }
        }
```

(Leave the `retained_homing_curve` clear and the final `Ok(())` that follow it untouched.)

- [ ] **Step 3: Build to confirm it compiles**

Run: `cargo build -p motion-bridge 2>&1 | tail -25`
Expected: compile error(s) only about `SeedPosition` still being referenced by the now-dead struct/field (resolved in Task 5), OR clean if the struct is unused-but-defined. If the only errors are unused `SeedPosition`/`pending_seed`, proceed to Task 5 before re-building.

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "fix(motion-bridge): set_position sends per-MCU runtime_seed_position"
```

---

## Task 5: Retire `pending_seed` / `SeedPosition`

**Files:**
- Modify: `rust/motion-bridge/src/bridge.rs` (struct `SeedPosition` ~line 43; field ~line 406; init ~line 675)

- [ ] **Step 1: Confirm no remaining readers**

Run: `grep -n "pending_seed\|SeedPosition" rust/motion-bridge/src/bridge.rs`
Expected: only the struct definition (~43), the field declaration (~406), and the init (~675) — the `set_position` store is gone after Task 4. If any *other* reference appears, STOP and reassess (the design assumed a single consumer).

- [ ] **Step 2: Remove the struct definition**

Delete the `SeedPosition` struct (the `struct SeedPosition { ... }` block at ~`bridge.rs:43`, including its doc comment).

- [ ] **Step 3: Remove the field**

Delete the `pending_seed: Arc<Mutex<Option<SeedPosition>>>,` field declaration (~`bridge.rs:406`) and its doc comment.

- [ ] **Step 4: Remove the initializer**

Delete the `pending_seed: Arc::new(Mutex::new(None)),` line in the constructor (~`bridge.rs:675`).

- [ ] **Step 5: Build to confirm clean**

Run: `cargo build -p motion-bridge 2>&1 | tail -20`
Expected: `Finished`, no `SeedPosition`/`pending_seed` references, no unused-import warnings.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/bridge.rs
git commit -m "refactor(motion-bridge): retire dead pending_seed/SeedPosition"
```

---

## Task 6: Full verification

- [ ] **Step 1: Run the full motion-bridge + runtime suites**

Run: `cargo test -p motion-bridge -p runtime 2>&1 | tail -30`
Expected: all green, including the new `seed_tests` and the unchanged `corexy_x_slot_is_x_plus_y`, plus `runtime`'s `pulse_steps_per_sample_exceeded_hard_faults` backstop test.

- [ ] **Step 2: Workspace build**

Run: `cargo build 2>&1 | tail -10`
Expected: `Finished`.

- [ ] **Step 3: Commit (only if Step 1/2 produced incidental fmt/lint fixes; otherwise skip)**

```bash
git add -A && git commit -m "test: motion-bridge + runtime green for seed delivery"
```

- [ ] **Step 4: Bench verification (manual — requires flashing; do NOT automate)**

Flash both MCUs from this branch HEAD (use the `flashing-trident-mcus` skill: commit → push → pull → build host `.so` → build+flash H7 then F446). Then, with explicit per-command permission from the user (motion commands are gated):
  1. `SET_KINEMATIC_POSITION X=150 Y=150` (or home).
  2. Confirm a `bridge_send mcu=… msg=runtime_seed_position x_q16=… y_q16=… z_q16=…` appears in `klippy.log` with Octopus `x_q16=19660800` (300 mm), `y_q16=0`, and F446 `z_q16` matching Z.
  3. Jog X a small amount → toolhead moves **pure +X**, no diagonal, **no** `kalico runtime fault` / `StepsPerSampleExceeded`.
  4. Jog Y and a diagonal XY move → correct motion, no fault.

Expected: pure-axis jogs produce pure-axis motion; the −310 fault no longer fires for a normal post-`set_position` move.

---

## Self-review notes

- **Spec coverage:** Decision 1 (direct command, not pump) → Task 4 uses `send_typed`, no `PumpMsg`. Decision 2 (from `set_position`, no 4th arg, planner-guarded) → Task 4 folds into `if let Some(planner)`. Decision 3 (shared helper) → Tasks 1 & 3. Error handling (panic on planner-up-but-missing-io; skip when planner absent) → Task 4 `unwrap_or_else(panic!)` inside the guard. Retire `pending_seed` → Task 5. Tests (helper, encoding, build_seed_sends, bench) → Tasks 1, 2, 6.
- **Backstop unchanged:** the `StepsPerSampleExceeded` fault (commit `70d0104cf`) stays as the safety net; Task 6 Step 1 keeps its test green.
- **No MCU/C or Python changes** — confirmed: the seed command + FFI exist on HEAD, and `motion_toolhead.set_position → bridge.set_position(x,y,z)` already triggers the path.
