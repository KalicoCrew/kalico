# Idle Re-anchor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the `-308 PieceStartInPast` MCU fault on the second jog after an idle gap by rewinding the planner timeline to 0 when the toolhead comes to rest at quiescence, so the next move re-anchors to `host_now + LEAD` like a first move.

**Architecture:** When the planner's `T_COMMIT` quiescence timer fires (decel-to-zero already committed → toolhead stopped), call `state.reset(state.current_position())`. `reset()` zeros the planner-time cursors and reseeds the shaper history at the settled position; the existing `Anchor` backward-jump branch then re-anchors the next move. Requires one new read-only accessor (`current_position()`) and one call site in the planner run-loop. `anchor.rs` is unchanged.

**Tech Stack:** Rust workspace (`rust/`). Crates: `trajectory` (`ShaperState`), `motion-bridge` (planner run-loop). `cargo test`.

**Spec:** `docs/superpowers/specs/2026-05-30-idle-reanchor-design.md`

---

## File Structure

- `rust/trajectory/src/streaming/state.rs` — add `current_position()` (pub) + `axis_position_at()` (private helper). Mirrors the existing `read_path_speed_at` / `axis_velocity_at` pair but evaluates position instead of velocity.
- `rust/trajectory/src/streaming/tests.rs` — unit tests for the two new functions.
- `rust/motion-bridge/src/planner.rs` — add the `state.reset(state.current_position())` call in the `T_COMMIT` (`RecvTimeoutError::Timeout`) run-loop arm.
- `rust/motion-bridge/src/planner/tests.rs` — regression test that quiescence rewinds the timeline.

`rust/motion-bridge/src/anchor.rs` is intentionally **not** touched.

**Package names:** the crate at `rust/trajectory` is package `trajectory`; the crate at `rust/motion-bridge` is package `motion-bridge`. If a `cargo test -p <name>` invocation errors with "package not found", read the `[package] name` from the crate's `Cargo.toml` and use that. All `cargo` commands run from `rust/`.

---

## Task 1: `current_position()` accessor on `ShaperState`

**Files:**
- Modify: `rust/trajectory/src/streaming/state.rs` (add two methods inside `impl ShaperState`, next to `read_path_speed_at` near line 486)
- Test: `rust/trajectory/src/streaming/tests.rs`

**Background the implementer needs:**
- `self.axes[i].pieces` is a `VecDeque<BezierPiece<f64>>` of **unshaped** position polynomials in time order. `BezierPiece` has `u_start`, `u_end`, and `evaluate(u) -> f64` and `differentiate() -> Self` (`rust/nurbs/src/bezier.rs`).
- The existing `axis_velocity_at(axis_idx, t)` (same file, ~line 506) finds the piece covering `t` and returns `Some(p.differentiate().evaluate(t))`, with a terminal clamp: if `t` is at/just past the last piece's `u_end`, it evaluates at `u_end`. `current_position` is the same walk but uses `p.evaluate(t)` (no derivative).
- `TIME_LOOKUP_TOLERANCE` (const in the same file) is the boundary slack already used by `axis_velocity_at`.
- Shaped axes (`h > 0`, e.g. X/Y) always carry pieces covering `t_appended`. Passthrough/none axes (`h == 0`, e.g. Z/E) may have an **empty** queue → `axis_position_at` returns `None`. That fallback value is a **don't-care**: `reseed_axis_queue` discards the seed position for `h == 0` axes (an empty queue carries no position; the next move re-derives position from its own absolute geometry). So `unwrap_or(0.0)` is safe for those axes.

- [ ] **Step 1: Write the failing test (motion case)**

Add to the `tests` module in `rust/trajectory/src/streaming/tests.rs`:

```rust
#[test]
fn current_position_reads_settled_endpoint_after_motion() {
    let shapers = replan_shapers(); // X,Y = SmoothMzv (h>0); Z passthrough; E none
    let mut state = ShaperState::new([0.0; 4], &shapers);
    let ctx_replan = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    // Move X 0 -> 200; append + emit advances t_appended to the move's end
    // (including the decel-to-zero ramp), with the X curve settled at 200, v=0.
    let m1 = linear_x_segment(0.0, 200.0, 200.0);
    state.append_and_replan(m1, &ctx_replan).expect("append");
    let _ = state.emit_committed(&ctx_emit).expect("emit");
    assert!(state.t_appended > 0.0, "precondition: t_appended advanced");

    let pos = state.current_position();
    // Shaped X axis reads its settled endpoint from the unshaped curve.
    assert!(
        (pos[0] - 200.0).abs() < 1e-2,
        "X should settle at endpoint 200, got {}",
        pos[0]
    );
    // Y never moved: shaped Y seed is the home 0.0.
    assert!((pos[1] - 0.0).abs() < 1e-2, "Y stays at home 0, got {}", pos[1]);
}
```

- [ ] **Step 2: Write the failing test (fresh-state seed case)**

Add directly below the first test:

```rust
#[test]
fn current_position_on_fresh_shaped_state_reads_seed() {
    let shapers = replan_shapers();
    // Shaped axes (X,Y) seed a constant `home_pos` piece over [-2h, 0]; at
    // t_appended == 0 current_position reads that seed back exactly.
    let state = ShaperState::new([7.0, 9.0, 5.0, 3.0], &shapers);
    let pos = state.current_position();
    assert!((pos[0] - 7.0).abs() < 1e-12, "X seed, got {}", pos[0]);
    assert!((pos[1] - 9.0).abs() < 1e-12, "Y seed, got {}", pos[1]);
    // Z (passthrough) and E (none) have empty queues; their values are
    // don't-cares (reset discards them) so we do not assert on pos[2]/pos[3].
}
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cargo test -p trajectory current_position 2>&1 | tail -20`
Expected: FAIL — `no method named current_position found for struct ShaperState`.

- [ ] **Step 4: Implement `axis_position_at` + `current_position`**

In `rust/trajectory/src/streaming/state.rs`, inside `impl ShaperState`, add next to `read_path_speed_at`:

```rust
    /// The settled toolhead position: each axis's unshaped curve evaluated at
    /// the end of the appended timeline (`t_appended`). After a `T_COMMIT`
    /// decel-to-zero commit this is the rest position, suitable for feeding to
    /// [`Self::reset`] to rewind the planner clock without moving the toolhead.
    ///
    /// Shaped axes (`h > 0`) always carry pieces covering `t_appended`, so they
    /// read exactly. Passthrough / none axes (`h == 0`) may have an empty queue;
    /// their fallback (`0.0`) is a don't-care because [`reseed_axis_queue`]
    /// discards the seed position for `h == 0` axes — an empty queue carries no
    /// position, and the next move on such an axis re-derives position from its
    /// own absolute geometry.
    #[must_use]
    pub fn current_position(&self) -> [f64; 4] {
        std::array::from_fn(|i| self.axis_position_at(i, self.t_appended).unwrap_or(0.0))
    }

    /// Evaluate axis `axis_idx`'s unshaped position curve at time `t`. Mirrors
    /// [`Self::axis_velocity_at`] (same piece-walk and terminal clamp) but
    /// evaluates the piece itself rather than its derivative. `None` when the
    /// axis queue is empty or no piece covers `t`.
    fn axis_position_at(&self, axis_idx: usize, t: f64) -> Option<f64> {
        let pieces = &self.axes[axis_idx].pieces;
        if pieces.is_empty() {
            return None;
        }

        // Last-piece terminal: clamp `t` to `u_end` (the decel-to-zero ends at
        // the target position; evaluating at `u_end` returns it).
        let last = pieces.back().unwrap();
        if t >= last.u_end && t <= last.u_end + TIME_LOOKUP_TOLERANCE {
            return Some(last.evaluate(last.u_end));
        }

        for p in pieces {
            if p.u_start - TIME_LOOKUP_TOLERANCE <= t && t < p.u_end {
                return Some(p.evaluate(t));
            }
        }
        None
    }
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `cargo test -p trajectory current_position 2>&1 | tail -20`
Expected: PASS — both `current_position_*` tests pass.

- [ ] **Step 6: Run the surrounding test module to check nothing regressed**

Run: `cargo test -p trajectory streaming:: 2>&1 | tail -20`
Expected: PASS — existing streaming tests (incl. `reset_after_motion_clears_state_and_reseeds_at_home`) still pass.

- [ ] **Step 7: Commit**

```bash
git add rust/trajectory/src/streaming/state.rs rust/trajectory/src/streaming/tests.rs
git commit -m "feat(trajectory): add ShaperState::current_position (settled position read)"
```

---

## Task 2: Rewind the planner timeline on the `T_COMMIT` quiescence fire

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs` — the `Err(RecvTimeoutError::Timeout)` arm of the run-loop `match` (around lines 726–752)
- Test: `rust/motion-bridge/src/planner/tests.rs`

**Background the implementer needs:**
- The run-loop arms a 50 ms quiescence timer (`T_COMMIT`) after every `Move`. When it fires (no follow-on move), `run_commit_and_dispatch(...)` flushes the held-back decel-to-zero tail to the wire — the toolhead is now stopped — and the arm sets `last_append_time = None`.
- `state` is the `ShaperState` owned by the run-loop. `state.current_position()` (Task 1) and `state.reset(pos)` are both `pub`.
- The planner run-loop emits `ShapedSegment`s with planner-time `t_start` / `t_end` to the `dispatch` closure. After a rewind, the next move's segments restart near planner-time 0 instead of continuing from the previous move's `t_end` — that is the observable the regression test checks.
- The rewind goes **only** in the `T_COMMIT` arm, not in `Flush` (M400 / `wait_moves` is deferred per the spec's "Out of scope").

- [ ] **Step 1: Write the failing regression test**

Add to `rust/motion-bridge/src/planner/tests.rs` (add `use std::time::Duration;` and `use std::sync::Mutex;` at the top of the file if not already present):

```rust
/// Dispatch closure that records each dispatched segment's (t_start, t_end).
fn capturing_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    Arc<Mutex<Vec<(f64, f64)>>>,
) {
    let log = Arc::new(Mutex::new(Vec::new()));
    let l = Arc::clone(&log);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            l.lock().unwrap().push((seg.t_start, seg.t_end));
            Ok(())
        });
    (cb, log)
}

#[test]
fn quiescence_rewinds_timeline_so_next_move_restarts_near_zero() {
    let (dispatch, log) = capturing_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    // Move 1: X 0 -> 200. Let the 50 ms T_COMMIT quiescence fire so the
    // decel-to-zero tail is committed AND (with the fix) the timeline rewinds.
    h.submit_move(long_move()).unwrap();
    std::thread::sleep(Duration::from_millis(120));
    let m1_max_t_end = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(_, e)| e)
        .fold(0.0_f64, f64::max);
    assert!(m1_max_t_end > 0.0, "move 1 produced no dispatched segments");

    // Move 2: continue X 200 -> 400 (physically contiguous so append_and_replan
    // is well-formed). With the rewind, its first segment starts near planner
    // time 0; without it, segments continue from m1_max_t_end.
    log.lock().unwrap().clear();
    let m2 = classify_and_build([200.0, 0.0, 0.0], 400.0, 0.0, 0.0, 0.0, 200.0).unwrap();
    h.submit_move(m2).unwrap();
    std::thread::sleep(Duration::from_millis(120));
    let m2_min_t_start = log
        .lock()
        .unwrap()
        .iter()
        .map(|&(s, _)| s)
        .fold(f64::INFINITY, f64::min);
    assert!(
        m2_min_t_start.is_finite(),
        "move 2 produced no dispatched segments"
    );
    assert!(
        m2_min_t_start < m1_max_t_end,
        "timeline did not rewind: move 2 started at {m2_min_t_start}, move 1 ended at {m1_max_t_end}"
    );

    h.shutdown();
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cargo test -p motion-bridge quiescence_rewinds 2>&1 | tail -20`
Expected: FAIL — `m2_min_t_start` (≈ `m1_max_t_end`, since the timeline keeps accumulating) is **not** `< m1_max_t_end`, so the final assert fails.

- [ ] **Step 3: Add the rewind to the `T_COMMIT` arm**

In `rust/motion-bridge/src/planner.rs`, find the `Err(RecvTimeoutError::Timeout)` arm. Replace:

```rust
                let _ok = run_commit_and_dispatch(
                    &mut state,
                    &thread_state,
                    &dispatch,
                    &error,
                    &last_move_time_bits,
                    &commit_fire_count,
                );
                last_append_time = None;
                continue;
```

with:

```rust
                let _ok = run_commit_and_dispatch(
                    &mut state,
                    &thread_state,
                    &dispatch,
                    &error,
                    &last_move_time_bits,
                    &commit_fire_count,
                );
                // Idle re-anchor (spec 2026-05-30-idle-reanchor): the
                // quiescence commit has flushed the decel-to-zero tail — the
                // toolhead is stopped. Rewind the planner timeline to 0,
                // reseeding the shaper history at the settled position, so the
                // next move arrives as a backward jump and the bridge `Anchor`
                // re-anchors it to `host_now + LEAD`, exactly like a first move.
                // Without this, a move after an idle gap is stamped in the
                // MCU's past → -308 PieceStartInPast. Scope: T_COMMIT only;
                // Flush / M400 is handled separately.
                let settled = state.current_position();
                state.reset(settled);
                last_append_time = None;
                continue;
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cargo test -p motion-bridge quiescence_rewinds 2>&1 | tail -20`
Expected: PASS — `m2_min_t_start` is near 0, well below `m1_max_t_end`.

- [ ] **Step 5: Run the planner test module to check nothing regressed**

Run: `cargo test -p motion-bridge planner:: 2>&1 | tail -30`
Expected: PASS — existing planner tests (`submit_and_flush_dispatches_segments`, `submit_triggers_replan_per_move`, the `z_only_move_*` tests, etc.) still pass.

- [ ] **Step 6: Commit**

```bash
git add rust/motion-bridge/src/planner.rs rust/motion-bridge/src/planner/tests.rs
git commit -m "fix(planner): rewind timeline on quiescence so post-idle moves re-anchor"
```

---

## Task 3: Hardware verification (the original repro)

**Not a code task** — this is the on-hardware regression check for the fault that started this. It needs both MCUs flashed with the new firmware. Follow the bench firmware flow (commit → push → pull on the Pi → build → flash); do **not** cross-compile locally and scp.

- [ ] **Step 1: Confirm the host-side build is clean**

Run (from `rust/`): `cargo build -p motion-bridge 2>&1 | tail -5`
Expected: builds without error.

- [ ] **Step 2: Build + flash both MCUs on the Pi**

On `dderg@trident.local`, after pushing this branch and pulling on the Pi: build and flash the H723 (`mcu`) and F446 (`bottom`) per the bench flow (`make clean` between the two C builds; `make -j$(nproc)`; flash H7 from `.config.h7.bak`, F446 from `.config.f446.test`). Driving MCU motion commands requires explicit per-command user permission — do not issue them autonomously.

- [ ] **Step 3: Run the two-jog repro**

With the user's go-ahead for each motion command, in the console:
```
SET_KINEMATIC_POSITION X=150 Y=150 Z=50
_CLIENT_LINEAR_MOVE Y=-1 F=6000
   (pause a few seconds)
_CLIENT_LINEAR_MOVE Y=-25 F=6000
```
Expected: both jogs complete; **no** `MCU 'bottom' shutdown: kalico runtime fault` and **no** `-308 / fault_code 65228`. Repeat with a longer (>10 s) idle and a third jog to confirm the rewind holds across longer gaps.

- [ ] **Step 4: Confirm clean in the log**

Fetch `klippy.log` to a local snapshot (`/tmp/klippy-<ts>.log`) and grep for `kalico runtime fault` / `65228` over the repro window.
Expected: no fault entries.

---

## Notes / known limitations (from the spec)

- **M400 / `Flush` is out of scope.** A long idle that follows a `Flush` (M400 / `wait_moves`) disarms `T_COMMIT` and would re-open the same fault. The `Flush` path is suspected not to work well and is being designed separately. This plan deliberately rewinds only on `T_COMMIT`.
- `T_COMMIT` is 50 ms, so the timeline rewinds after any ≥50 ms submission gap — but that only happens after a commit-to-stop that already occurs under the existing design, so no new stops are introduced. Continuous prints (moves < 50 ms apart) never trip it.
- The Task 2 regression test uses real `T_COMMIT` timing (two ~120 ms sleeps). It is deterministic in outcome but adds ~0.25 s of wall-clock; keep it as a single focused test.
