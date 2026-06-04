# Monotonic Planner Clock Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make planner-time a monotonic clock (never reset on idle) so moves self-place via `max(t_appended, elapsed_since_sync)`, the decel-commit fires off a clock-derived deadline, and Flush completes on a clock wait — eliminating the `-308 PieceStartInPast` fault class on idle-then-jog and M400-then-move.

**Architecture:** `t0` is established once per stream by the unchanged `Anchor`; planner-time only ever advances. A genuinely-idle move inserts a host-side rest-hold (`ShaperState::advance_idle`) to bump `t_appended` to "now" before appending, keeping piece stamps monotone. The decel-to-zero safety commit fires when `elapsed_since_sync ≥ t_dispatched + LEAD − SAFETY_MARGIN`. Flush commits the tail, then the caller blocks until `t_appended + LEAD`. MCU retirement counts stay reserved for pump ring flow control.

**Tech Stack:** Rust, `std::time::Instant`, `crossbeam_channel`, the `motion-bridge` planner run-loop, the `trajectory` streaming `ShaperState`.

**Spec:** `docs/superpowers/specs/2026-05-30-monotonic-planner-clock-design.md`

---

## File Structure

- `rust/trajectory/src/streaming/state.rs` — add `pub fn advance_idle(&mut self, target_t: f64)`. Keep `current_position()` / `axis_position_at` (the rest-hold reuses the latter).
- `rust/trajectory/src/streaming/tests.rs` — unit tests for `advance_idle`, the commit-cursor invariant, and idle-gap monotonicity. Reuse the file's existing fixtures (e.g. the helpers used by `current_position_reads_settled_endpoint_after_motion` at `streaming/tests.rs:1299-1345`).
- `rust/motion-bridge/src/planner.rs` — add `LEAD` / `SAFETY_MARGIN` constants and a `sync_instant` run-loop local; replace the `T_COMMIT` timeout with the clock-derived decel-commit deadline; add the placement rule in the `Move` arm; rewrite the `Flush` arm to hand back a finish `Instant`; capture/clear `sync_instant`; remove the idle-reanchor block. Change `PlannerMsg::Flush` payload and `PlannerHandle::flush`.
- `rust/motion-bridge/src/planner/tests.rs` — integration tests for flush timing and flush-then-move.
- `rust/motion-bridge/src/anchor.rs` — remove only the temporary `[anchor]` diag log (commit 381e8f7eb). No logic change.
- `rust/motion-bridge/src/drain.rs` — **unchanged** (flow control + `set_position` barrier). Add a clarifying comment to its existing tests.

**Out of scope (do not touch):** the 60 s `DrainSync` timeout, `wait_moves` returning at dispatch. See spec § Out of scope.

---

## Task 1: `ShaperState::advance_idle` + unit tests

**Files:**
- Modify: `rust/trajectory/src/streaming/state.rs` (add method after `current_position`, ~line 567)
- Test: `rust/trajectory/src/streaming/tests.rs`

- [ ] **Step 1: Write the failing tests**

Add to `rust/trajectory/src/streaming/tests.rs`. Construct `ShaperState` and segments using the **same fixtures the existing tests in this file use** (read the existing `#[cfg(test)] mod tests` setup — e.g. the helpers feeding `current_position_reads_settled_endpoint_after_motion`). The behavioral assertions:

```rust
#[test]
fn advance_idle_is_noop_when_target_not_past_t_appended() {
    // Queued-ahead case: target <= t_appended → no change.
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    state
        .append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx_replan)
        .expect("append");
    let t_app_before = state.t_appended;
    let pieces_x_before = state.axes[0].pieces.len();

    state.advance_idle(state.t_appended * 0.5); // target < t_appended

    assert!((state.t_appended - t_app_before).abs() < 1e-12,
        "queued-ahead: t_appended must not change");
    assert_eq!(state.axes[0].pieces.len(), pieces_x_before,
        "queued-ahead: no piece inserted");
}

#[test]
fn advance_idle_when_drained_extends_to_target_preserving_position() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    state
        .append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx_replan)
        .expect("append");
    // Commit fully so t_dispatched == t_appended (the post-decel state).
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);
    let _ = state.emit_committed(&ctx_emit).expect("emit");
    let _ = state.commit_decel_to_zero(&ctx_emit).expect("commit");

    let t_app_before = state.t_appended;
    let pos_before = state.current_position(); // [f64; 4] at t_appended

    let target = t_app_before + 0.3;
    state.advance_idle(target);

    assert!((state.t_appended - target).abs() < 1e-12, "t_appended -> target");
    assert!((state.t_decel_start - target).abs() < 1e-12, "t_decel_start -> target");
    let pos_after = state.current_position(); // now read at t_appended == target
    for i in 0..4 {
        assert!((pos_after[i] - pos_before[i]).abs() < 1e-6,
            "axis {i} position must be continuous across the rest-hold");
    }
    // The hold piece on a shaped axis spans [t_app_before, target].
    let last_x = state.axes[0].pieces.back().unwrap();
    assert!((last_x.u_end - target).abs() < 1e-12);
    assert!((last_x.u_start - t_app_before).abs() < 1e-12);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p trajectory advance_idle`
Expected: FAIL — `no method named advance_idle found for struct ShaperState`.

- [ ] **Step 3: Implement `advance_idle`**

Add to `impl ShaperState` in `rust/trajectory/src/streaming/state.rs`:

```rust
/// Advance the planner timeline from the current `t_appended` to `target_t`
/// by inserting a "park at current position, v=0" rest segment on every
/// shaped axis. Host-side only — nothing is dispatched; the MCU is genuinely
/// at rest, so the shaper history window stays valid with no reseed.
///
/// No-op when `target_t <= t_appended` (caller's "queued-ahead" branch).
/// After this call: `t_appended == t_decel_start == target_t`;
/// `uncommitted_moves` / `planned_fitted` / `planned_meta` are cleared (the
/// rest-hold replaces any speculative decel tail); `t_dispatched` is
/// unchanged (the hold is held back until commit, like a normal decel tail).
pub fn advance_idle(&mut self, target_t: f64) {
    if target_t <= self.t_appended + 1e-12 {
        return;
    }
    let hold_start = self.t_appended;
    let hold_end = target_t;
    let end_pos: [f64; 4] =
        std::array::from_fn(|i| self.axis_position_at(i, hold_start).unwrap_or(0.0));

    for (i, axis) in self.axes.iter_mut().enumerate() {
        if axis.h > 0.0 {
            axis.pieces.push_back(nurbs::bezier::BezierPiece {
                u_start: hold_start,
                u_end: hold_end,
                coeffs: vec![end_pos[i]],
            });
        }
    }

    self.uncommitted_moves.clear();
    self.planned_fitted.clear();
    self.planned_meta.clear();
    self.t_appended = hold_end;
    self.t_decel_start = hold_end;
}
```

Note: confirm the exact field/type names against the file — `axes[i].pieces` is `VecDeque<BezierPiece<f64>>`, `axes[i].h` is the kernel half-support, and the constant-piece construction mirrors `build_axis_queue` (`state.rs:1144-1149`). If `planned_meta` does not exist, drop that line (clear only the fields that exist).

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p trajectory advance_idle`
Expected: PASS (both tests).

- [ ] **Step 5: Commit**

```bash
git add rust/trajectory/src/streaming/state.rs rust/trajectory/src/streaming/tests.rs
git commit -m "feat(trajectory): add ShaperState::advance_idle rest-hold primitive"
```

---

## Task 2: `LEAD` / `SAFETY_MARGIN` constants + `sync_instant` local

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs` (constants near `T_COMMIT`/`T_IDLE` ~line 53-60; local in `run_loop` ~line 690)

- [ ] **Step 1: Add the constants**

In `rust/motion-bridge/src/planner.rs`, after the `T_IDLE` constant (line 60):

```rust
/// Lead time (s) the Anchor inserts between planner time 0 and `host_now` at
/// first dispatch. Must equal `anchor::DEFAULT_LEAD_SECS`; duplicated here so
/// run_loop can compute clock-derived deadlines without depending on anchor's
/// private constant. Keep in sync manually (anchor.rs is unchanged).
const LEAD: f64 = 0.25;

/// Safety margin (s) for the decel-commit deadline: the commit must reach the
/// MCU at least this long before the on-wire buffer (`t_dispatched + LEAD`)
/// drains. Covers shaping + dispatch + pump + wire latency. Starting value
/// per spec §G; tune on hardware.
const SAFETY_MARGIN: f64 = 0.050;
```

- [ ] **Step 2: Add the `sync_instant` run-loop local**

In `run_loop`, after `let mut last_recv_time: Option<Instant> = None;` (line 690):

```rust
    // The `Instant` of the stream's first dispatch. `None` until the first
    // non-empty dispatch after a genuine reset; re-set to `None` on every
    // genuine reset (stream-open, homing, SET_KINEMATIC, Underrun, ForceIdle)
    // so it is re-captured at the next first dispatch. Same OS monotonic clock
    // as the projection's host-time input, so `elapsed_since_sync` carries no
    // drift (spec §"Why the clock is trustworthy").
    let mut sync_instant: Option<Instant> = None;
```

- [ ] **Step 3: Verify it compiles**

Run: `cargo build -p motion-bridge`
Expected: builds clean (unused `sync_instant`/consts warnings are acceptable at this task; they are consumed in Tasks 3-6).

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/planner.rs
git commit -m "chore(planner): add LEAD/SAFETY_MARGIN consts and sync_instant local"
```

---

## Task 3: Capture `sync_instant` on first dispatch; clear on genuine resets

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs` (`Move` arm after the dispatch loop ~line 847; the `KalicoStreamOpen`/`Homing`/`Underrun`/`ForceIdle` arms before their `state.reset(...)`)

- [ ] **Step 1: Capture on first dispatch**

In the `Move` arm, immediately after the dispatch `for s in &drained { ... }` loop (after line 847):

```rust
                // Capture the stream's sync origin at its first dispatch, in
                // the same sub-ms window the Anchor establishes `t0`. Residual
                // skew is absorbed by LEAD (spec §F).
                if sync_instant.is_none() && !drained.is_empty() {
                    sync_instant = Some(Instant::now());
                }
```

- [ ] **Step 2: Clear on every genuine reset**

In each arm that calls `state.reset(...)` (search for `state.reset(` in `planner.rs` — `KalicoStreamOpen`, `Homing`, `Underrun`, `ForceIdle`, and any reconnect path), add immediately before the `state.reset(...)` call:

```rust
                sync_instant = None; // re-captured at next first dispatch
```

- [ ] **Step 3: Verify it compiles and existing tests pass**

Run: `cargo test -p motion-bridge`
Expected: PASS (no behavior change yet to existing tests).

- [ ] **Step 4: Commit**

```bash
git add rust/motion-bridge/src/planner.rs
git commit -m "feat(planner): capture sync_instant at first dispatch, clear on reset"
```

---

## Task 4: Replace the `T_COMMIT` timer with the clock-derived decel-commit deadline

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs` (`next_timeout` block ~line 698-701; `Timeout` arm ~line 726-768)
- Test: `rust/trajectory/src/streaming/tests.rs` (commit-cursor invariant)

- [ ] **Step 1: Write the failing test (commit cursor invariant)**

In `rust/trajectory/src/streaming/tests.rs`, using the file's existing fixtures:

```rust
#[test]
fn commit_decel_to_zero_advances_t_dispatched_to_t_appended_and_is_idempotent() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx_replan = replan_context();
    state
        .append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx_replan)
        .expect("append");
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    let partial = state.emit_committed(&ctx_emit).expect("emit");
    assert!(!partial.is_empty());
    assert!(state.t_dispatched < state.t_appended, "tail held back before commit");

    let committed = state.commit_decel_to_zero(&ctx_emit).expect("commit");
    assert!(!committed.is_empty(), "commit emits the decel tail");
    assert!((state.t_dispatched - state.t_appended).abs() < 1e-12,
        "after commit t_dispatched == t_appended");

    let again = state.commit_decel_to_zero(&ctx_emit).expect("commit2");
    assert!(again.is_empty(), "second commit is a no-op");
    assert!((state.t_dispatched - state.t_appended).abs() < 1e-12);
}
```

- [ ] **Step 2: Run to verify it passes or fails meaningfully**

Run: `cargo test -p trajectory commit_decel_to_zero_advances`
Expected: PASS if `commit_decel_to_zero` already maintains the invariant (this test documents the property the new timer relies on). If it FAILS, stop — the deadline design assumes this invariant; investigate before proceeding.

- [ ] **Step 3: Replace the `next_timeout` block**

Replace `rust/motion-bridge/src/planner.rs` lines 698-701:

```rust
        // Clock-derived decel-commit deadline (spec §B). The MCU starts
        // executing planner-time 0 at elapsed_since_sync == LEAD and plays
        // forward 1:1, so the on-wire buffer (ending at t_dispatched) drains
        // at elapsed_since_sync == t_dispatched + LEAD. Commit SAFETY_MARGIN
        // before that. When there is no held-back tail (t_dispatched ≈
        // t_appended), sleep on the long sentinel until the next Move.
        let next_timeout = if state.t_dispatched < state.t_appended - 1e-12 {
            let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
            let remaining = (state.t_dispatched + LEAD - SAFETY_MARGIN) - esc;
            if remaining <= 0.0 { Duration::ZERO } else { Duration::from_secs_f64(remaining) }
        } else {
            T_IDLE
        };
```

- [ ] **Step 4: Replace the `Timeout` arm body**

Replace the `Err(RecvTimeoutError::Timeout) => { ... }` arm (lines 726-768) with:

```rust
            Err(RecvTimeoutError::Timeout) => {
                // Decel-commit deadline (spec §B): the on-wire buffer is about
                // to drain; commit the held-back decel-to-zero so the MCU
                // stops cleanly. DO NOT reset the timeline — the monotonic
                // clock keeps running; the next move self-places via
                // max(t_appended, elapsed_since_sync).
                if state.t_dispatched < state.t_appended - 1e-12 {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                    );
                }
                continue;
            }
```

This deletes `state.current_position()`, the `[idle-reanchor]` log, `state.reset(settled)`, and `last_append_time = None` from the old arm.

- [ ] **Step 5: Replace the obsolete rewind test with a monotonicity test**

`rust/motion-bridge/src/planner/tests.rs:473` has `quiescence_rewinds_timeline_so_next_move_restarts_near_zero`, which asserts the **old** rewind behavior (`m2_min_t_start < m1_max_t_end`) — the exact inverse of the new invariant. It will now fail. Replace it (keep the `wait_for_commits` helper and `capturing_dispatch` setup verbatim; flip the move-2 assertion):

```rust
#[test]
fn quiescence_keeps_timeline_monotone_next_move_does_not_rewind() {
    let (dispatch, log) = capturing_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);

    fn wait_for_commits(h: &PlannerHandle, target: u32) {
        let start = std::time::Instant::now();
        while h.commit_fire_count() < target {
            assert!(
                start.elapsed() < Duration::from_secs(5),
                "commit fired only {} of {target} times within 5s",
                h.commit_fire_count()
            );
            std::thread::sleep(Duration::from_millis(2));
        }
    }

    // Move 1: X 0 -> 200. Wait for the decel-commit (commit #1).
    h.submit_move(long_move()).unwrap();
    wait_for_commits(&h, 1);
    let m1_max_t_end = log
        .lock().unwrap().iter().map(|&(_, e)| e).fold(0.0_f64, f64::max);
    assert!(m1_max_t_end > 0.0, "move 1 produced no dispatched segments");

    // Move 2: X 200 -> 400, submitted after a real idle gap so the
    // monotonic clock has advanced past move 1's end.
    log.lock().unwrap().clear();
    std::thread::sleep(Duration::from_millis(400)); // > LEAD + move tail
    let m2 = classify_and_build([200.0, 0.0, 0.0], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap();
    h.submit_move(m2).unwrap();
    wait_for_commits(&h, 2);
    let m2_min_t_start = log
        .lock().unwrap().iter().map(|&(s, _)| s).fold(f64::INFINITY, f64::min);
    assert!(m2_min_t_start.is_finite(), "move 2 produced no dispatched segments");

    // Monotone clock: move 2 starts at or after move 1's end — NOT rewound.
    assert!(
        m2_min_t_start >= m1_max_t_end - 1e-3,
        "timeline rewound: move 2 started at {m2_min_t_start}, move 1 ended at {m1_max_t_end}"
    );

    h.shutdown();
}
```

- [ ] **Step 6: Run tests**

Run: `cargo test -p trajectory && cargo test -p motion-bridge`
Expected: PASS. The flipped test now guards monotonicity; the idle-reset behavior is gone.

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/planner.rs rust/trajectory/src/streaming/tests.rs rust/motion-bridge/src/planner/tests.rs
git commit -m "feat(planner): clock-derived decel-commit deadline, drop idle reset"
```

---

## Task 5: Placement rule in the `Move` arm

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs` (`Move` arm, before `append_and_replan` ~line 795)
- Test: `rust/trajectory/src/streaming/tests.rs` (idle-gap monotonicity)

- [ ] **Step 1: Write the failing test (monotonicity across idle gap)**

In `rust/trajectory/src/streaming/tests.rs`:

```rust
#[test]
fn piece_stamps_monotone_across_idle_gap() {
    let mut state = ShaperState::new([0.0; 4], &replan_shapers());
    let ctx = replan_context();
    let kernels = replan_kernels_piecewise();
    let halos: Vec<EHalo> = Vec::new();
    let ctx_emit = emit_context_default(&kernels, &halos);

    // Move 1, fully committed.
    state.append_and_replan(linear_x_segment(0.0, 200.0, 200.0), &ctx).expect("m1");
    let _ = state.emit_committed(&ctx_emit).expect("emit1");
    let _ = state.commit_decel_to_zero(&ctx_emit).expect("commit1");
    let t_after_m1 = state.t_appended;

    // Idle gap of 0.5 s, then move 2.
    state.advance_idle(t_after_m1 + 0.5);
    state.append_and_replan(linear_x_segment(200.0, 400.0, 200.0), &ctx).expect("m2");
    let _ = state.emit_committed(&ctx_emit).expect("emit2");

    let stamps: Vec<f64> = state.axes[0].pieces.iter().map(|p| p.u_start).collect();
    for w in stamps.windows(2) {
        assert!(w[1] >= w[0] - 1e-12, "u_start went backward: {} -> {}", w[0], w[1]);
    }
}
```

- [ ] **Step 2: Run to verify it passes**

Run: `cargo test -p trajectory piece_stamps_monotone`
Expected: PASS — `advance_idle` (Task 1) already guarantees this at the `ShaperState` level. This test locks the property the `Move` arm change relies on.

- [ ] **Step 3: Insert the placement rule in the `Move` arm**

In `rust/motion-bridge/src/planner.rs`, inside `PlannerMsg::Move(m) => { ... }`, after the variable captures (`prior_t_appended` etc.) and **before** `state.append_and_replan(...)` (line 795):

```rust
                // Placement rule (spec §A): if the clock has run past the plan
                // tail, the toolhead is genuinely idle. Commit any held-back
                // tail first (so the MCU gets the prior decel-to-zero), then
                // insert a rest-hold advancing t_appended to "now" so the new
                // move starts at elapsed_since_sync (== host_now + LEAD via the
                // Anchor) instead of overlapping the prior committed tail.
                let esc = sync_instant.map_or(0.0, |t| t.elapsed().as_secs_f64());
                if esc > state.t_appended + 1e-6 {
                    if state.t_dispatched < state.t_appended - 1e-12 {
                        let _ok = run_commit_and_dispatch(
                            &mut state,
                            &thread_state,
                            &dispatch,
                            &error,
                            &last_move_time_bits,
                            &commit_fire_count,
                        );
                    }
                    state.advance_idle(esc);
                }
```

- [ ] **Step 4: Run tests**

Run: `cargo test -p trajectory && cargo test -p motion-bridge`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/planner.rs rust/trajectory/src/streaming/tests.rs
git commit -m "feat(planner): rest-hold placement rule for genuinely-idle moves"
```

---

## Task 6: Time-based Flush (caller-side wait)

**Files:**
- Modify: `rust/motion-bridge/src/planner.rs` (`PlannerMsg::Flush` payload ~line 92-95; `PlannerHandle::flush` ~line 312-328; `Flush` arm ~line 875-913)
- Test: `rust/motion-bridge/src/planner/tests.rs`

**Design:** the planner commits the tail and computes the finish `Instant` (`sync_instant + t_appended + LEAD`), then sends it back. The already-blocking `PlannerHandle::flush` sleeps until that instant on the **caller** thread, keeping the run-loop responsive.

- [ ] **Step 1: Write the failing tests**

In `rust/motion-bridge/src/planner/tests.rs`, using the file's existing `PlannerHandle::spawn` fixtures:

```rust
#[test]
#[ignore] // slow (~250 ms): exercises the real clock wait
fn flush_blocks_until_motion_complete_by_clock() {
    let (dispatch, _counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);
    let t0 = std::time::Instant::now();
    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let elapsed = t0.elapsed().as_secs_f64();
    assert!(elapsed >= 0.25 * 0.9, "flush returned too early: {:.4}s", elapsed); // ~LEAD
    h.shutdown();
}

#[test]
fn flush_then_move_dispatches_without_error() {
    let (dispatch, counter) = counting_dispatch();
    let mut h = PlannerHandle::spawn(relaxed_config(), dispatch);
    h.submit_move(long_move()).unwrap();
    h.flush().unwrap();
    let before = counter.load(Ordering::Relaxed);
    let m2 = classify_and_build([200.0, 0.0, 0.0], 200.0, 0.0, 0.0, 0.0, 200.0).unwrap();
    h.submit_move(m2).unwrap();
    h.flush().unwrap();
    assert!(counter.load(Ordering::Relaxed) > before);
    h.shutdown();
}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cargo test -p motion-bridge flush_then_move_dispatches_without_error`
Expected: compile error or FAIL until the new Flush arm lands.

- [ ] **Step 3: Change the `Flush` payload**

In `rust/motion-bridge/src/planner.rs`, change the `PlannerMsg::Flush` variant (lines 92-95):

```rust
    Flush {
        /// Planner sends the wall-clock `Instant` at which all committed
        /// motion finishes executing (`sync_instant + t_appended + LEAD`), or
        /// `None` if nothing is in flight. The caller waits until then.
        notify: Sender<Option<Instant>>,
    },
```

- [ ] **Step 4: Rewrite the `Flush` arm**

Replace the `Flush` arm (lines 875-913):

```rust
            PlannerMsg::Flush { notify } => {
                // Commit the held-back decel-to-zero (spec §E); idempotent.
                if state.t_dispatched < state.t_appended - 1e-12 {
                    let _ok = run_commit_and_dispatch(
                        &mut state,
                        &thread_state,
                        &dispatch,
                        &error,
                        &last_move_time_bits,
                        &commit_fire_count,
                    );
                }
                // Compute the wall-clock finish: the last committed piece ends
                // at planner-time t_appended, executing at elapsed == t_appended
                // + LEAD. Hand the deadline back; the caller waits (keeps the
                // run-loop responsive). No timeline reset — the monotonic clock
                // carries the next move via the placement rule.
                let finish = sync_instant
                    .map(|t| t + Duration::from_secs_f64(state.t_appended + LEAD));
                let _ = notify.send(finish);
            }
```

- [ ] **Step 5: Update `PlannerHandle::flush` to wait on the caller thread**

Replace the body of `PlannerHandle::flush` (lines 312-328):

```rust
    pub fn flush(&self) -> Result<(), PlannerError> {
        self.check_error()?;
        let (tx, rx) = crossbeam_channel::bounded(1);
        self.sender
            .send(PlannerMsg::Flush { notify: tx })
            .map_err(|_| PlannerError::ChannelClosed)?;
        match rx.recv() {
            Ok(finish) => {
                if let Some(deadline) = finish {
                    let now = Instant::now();
                    if deadline > now {
                        std::thread::sleep(deadline - now);
                    }
                }
                self.check_error()
            }
            Err(_) => {
                self.check_error()?;
                Err(PlannerError::ChannelClosed)
            }
        }
    }
```

- [ ] **Step 6: Run tests**

Run: `cargo test -p motion-bridge && cargo test -p motion-bridge -- --ignored flush_blocks`
Expected: PASS (the `--ignored` run executes the slow clock-wait test).

- [ ] **Step 7: Commit**

```bash
git add rust/motion-bridge/src/planner.rs rust/motion-bridge/src/planner/tests.rs
git commit -m "feat(planner): time-based Flush completion via caller-side clock wait"
```

---

## Task 7: Remove temporary diag traces; document flow-control invariance

**Files:**
- Modify: `rust/motion-bridge/src/anchor.rs` (remove `[anchor]` log ~line 33-37)
- Modify: `rust/motion-bridge/src/drain.rs` (comment on existing tests)

- [ ] **Step 1: Remove the `[anchor]` diag log**

In `rust/motion-bridge/src/anchor.rs`, delete the `log::info!("[anchor] seg_t=...")` block (lines 33-36) added by commit 381e8f7eb. Leave the surrounding `fresh`/`t0` logic untouched.

- [ ] **Step 2: Document flow-control invariance**

In `rust/motion-bridge/src/drain.rs`, above the existing test module, add:

```rust
// These tests cover pump ring flow-control accounting (pushed/retired).
// This mechanism is UNCHANGED by the monotonic-clock design (spec §E):
// DrainSync.add_sent / set_retired remain the sole flow-control path and are
// NOT replaced by the clock-based Flush.
```

- [ ] **Step 3: Verify the `[idle-reanchor]` log is already gone**

Run: `grep -rn "idle-reanchor\|\[anchor\]" rust/motion-bridge/src/`
Expected: no matches (the `[idle-reanchor]` log was removed in Task 4; `[anchor]` in this task).

- [ ] **Step 4: Run the full suite**

Run: `cargo test -p trajectory && cargo test -p motion-bridge`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/motion-bridge/src/anchor.rs rust/motion-bridge/src/drain.rs
git commit -m "chore(planner): remove idle-reanchor diag traces; note flow-control invariance"
```

---

## Task 8: Hardware validation (not a code task)

- [ ] **Step 1: Build + flash both MCUs** per the bench flow (H7 from `.config.h7.bak`, F446 from `.config.f446.test`; `make clean` between builds; commit→push→pull→compile on Pi→flash). Rebuild `motion_bridge_native.so`.

- [ ] **Step 2: Idle-then-jog repro.** `SET_KINEMATIC_POSITION X=150 Y=150 Z=50`; jog `Y=-1 F=6000`; pause > 250 ms; jog `Y=-25 F=6000`. Expected: completes, **no `-308`**.

- [ ] **Step 3: Rapid same-axis jogs.** Reproduce the 114 ms-gap sequence from `klippy.log.2026-05-30_20-13-16` (five `X=±25 F=6000` jogs in quick succession). Expected: **no `-308`**; rapid jogs blend rather than fault.

- [ ] **Step 4: M400-then-move.** A move, then `M400`, then another move. Expected: completes, **no `-308`**.

- [ ] **Step 5: Capture journalctl** and confirm no `[idle-reanchor]`/`[anchor]` traces remain and no fault events. Fetch `klippy.log` to `/tmp` for analysis.

---

## Notes for the implementer

- **Field/type names:** the production snippets assume `ShaperState.axes[i].pieces: VecDeque<BezierPiece<f64>>`, `axes[i].h: f64`, and fields `t_appended`/`t_decel_start`/`t_dispatched`/`uncommitted_moves`/`planned_fitted`(/`planned_meta`). Verify against `state.rs` before pasting; drop `planned_meta` if it does not exist.
- **Test fixtures:** every test above must construct `ShaperState` / `PlannerHandle` / emit & replan contexts using the **existing helpers in the same test file** — read the neighboring tests (`current_position_*` in `streaming/tests.rs:1299-1345`; the flush/dispatch tests in `planner/tests.rs`) and mirror their setup. Do not invent fixtures.
- **`last_append_time`:** after Task 4 it is no longer the timeout trigger but remains the "held-back tail exists" proxy used by the `Flush`/`UpdateShaper`/`ClockSyncRearm` arms. Leave it; a follow-up may replace it with the `t_dispatched < t_appended` cursor check.
- **`LEAD` duplication:** `planner::LEAD` must equal `anchor::DEFAULT_LEAD_SECS`. If you prefer, make `anchor::DEFAULT_LEAD_SECS` `pub` and import it — but the spec marks anchor.rs unchanged, so the duplicated-constant-with-comment is the default.
