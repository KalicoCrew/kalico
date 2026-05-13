# Step emission architecture — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the per-segment step-schedule architecture with an append-only per-motor step ring driven by an event-triggered producer Klipper timer and per-stepper consumer Klipper timers. TIM5 is enabled only when at least one motor is in `Modulated` mode and is reduced to a single responsibility (today: polled-tick StepAccumulator for Modulated motors; future Step 10: phase current synthesis). For the MVP (all axes `StepTime`) TIM5 stays off.

**Architecture:** Two disjoint emission paths. (1) **StepTime**: producer timer Newton-fills a per-motor SPSC ring of `(abs_cycles, dir)` entries; per-stepper consumer timer fires one pulse per call at the entry's scheduled time. Producer wakes only on `push_segment`, consumer low-water, or self-reschedule. (2) **Modulated**: TIM5 ISR samples curves for Modulated motors and runs StepAccumulator. Curve pool retirement is per-consumer: a slot returns to the host only when every motor consuming it is done with it (TIM5 done = wall-clock past `t_end`; producer done = Newton returned `SegmentExhausted`).

**Tech Stack:** Rust (no_std for runtime, std for host tests), C (Klipper MCU build, gcc-arm-none-eabi), Python (klippy host), Klipper sched.h `struct timer`.

**Spec:** [`docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md`](../specs/2026-05-14-step-emission-architecture-design.md)

**Open-question resolutions** (from spec §7, committed for this plan):

1. **Direction storage:** plain `[i8; N]` parallel array (clarity > 1 KB/motor savings).
2. **Curve pull policy:** per-motor cursor into the segment queue. Each motor advances independently; a segment retires from the queue when all motors have consumed (or skipped via UNUSED handle) their respective axis curves.
3. **`producer_pending` protocol:** single `AtomicBool`. Kickers CAS-set `false → true`; if win, `sched_add_timer(now)`. Producer on entry CAS-clears `true → false` then runs. Self-reschedule (batch cap) uses `sched_add_timer(now)` directly, bypassing the flag. Worst case: producer runs twice in close succession — benign.
4. **Mixed-mode CoreXY / eval API:** unify. `scalar_eval` returns `(pos, vel, accel)` everywhere; the de Boor walk produces all three at essentially the same cost.
5. **`step_distance` precision:** `f64` in producer state and in `compute_next_step_time`.

**Test infrastructure:**
- Rust unit tests: `cargo test -p kalico-runtime --features std` (host build).
- Rust integration: `cargo test -p kalico-runtime --features std --test <name>` for files in `rust/runtime/tests/`.
- Klippy unit: `python3 -m pytest klippy/test/` (if present; otherwise lint-only).
- Bench: SSH `dderg@trident.local`. Build with `make`, flash H7 + F4 per the `flash_h7.md` reference. Bench validation tasks list the exact G-code; the **user** runs them (memory: `feedback_no_gcode_without_permission.md`).

**Conventions to honor:**
- **No `Co-Authored-By: Claude` trailer** in any commit (user memory).
- **`cargo clean` between H7 and F4 builds** (memory: `cargo_clean_between_mcus.md`). The H7 places `RT_CELL` in `.axi_bss` (0x24000000); F4 needs `.bss` (0x20000000). Cargo's build cache silently keeps the wrong `.a` across `make clean` alone.
- **`make clean` between H7 and F4** (memory: `always_make_clean.md`).
- **Never issue G-code from agent** (memory: `feedback_no_gcode_without_permission.md`). Bench tasks describe G-code to be run by the user.
- **No bench-state hacks**: this plan is the architectural fix; do not add band-aid rate caps, MAX_STEPS_PER_TICK raises, schedule-reset toggles, etc. If a test fails, fix the design or call it out.

---

## Working-tree precondition

The current working tree (commit ~`5ee62f513` plus 2 days of uncommitted bench iteration) has ~800 lines of band-aid hacks in `rust/runtime/src/engine.rs`, smaller hacks in `runtime_tick.c`, and diagnostic atomics in `state.rs` that this plan removes. **Before Task 1**, the user (or implementer) must decide:

- (recommended) `git stash` the working tree → fresh state from HEAD → implement plan → `git stash drop` at the end. The stash preserves the diagnostic instrumentation in case bench evidence is wanted while debugging.
- Alternative: commit working-tree changes to a side branch as a checkpoint, then `git reset --hard HEAD` and proceed.

This is a manual decision step; the plan's tasks assume a clean tree from HEAD. If the implementer skips this and proceeds with the current tree, every modify-file step below will diff against the modified working tree, not HEAD — outcomes will diverge from the plan's expected commits.

---

## File structure

**Rust runtime crate** (`rust/runtime/src/`)
- `step_ring.rs` *(new)* — `StepRing` SPSC data structure (replaces the existing untracked `step_schedule.rs`).
- `step_producer.rs` *(new)* — `ProducerState`, `producer_step()` function.
- `step_time.rs` — signature change `(f32) -> (f64, f64)` → `(f32) -> (f64, f64, f64)`; degenerate-velocity Newton seeding.
- `engine.rs` — keep only Modulated-path state + `runtime_modulated_tick`. Delete `Engine::tick`, `Engine::tick_with_current`, `precompute_step_schedules`, `refill_step_schedules`, `arm_step_timer`, boundary-loop infrastructure, `force_idle` ISR check.
- `engine_force_idle.rs` *(new)* — `runtime_force_idle()` foreground sync flush.
- `state.rs` — delete diagnostic atomics for the old architecture; add the minimal new diagnostic set per spec §6.
- `clock.rs` — delete `publish_widened_now`; add on-demand `widened_now_from_stats(stats_send_time, stats_send_time_high)` helper.
- `step.rs` — untouched (`StepMotorState` / `MAX_STEPS_PER_TICK_DEFAULT = 16`).
- `step_schedule.rs` — **deleted** in T14 (after replacement is fully in place).
- `lib.rs` — `pub mod step_ring; pub mod step_producer; pub mod engine_force_idle;` add; `pub mod step_schedule;` remove in T14.

**Nurbs eval** (`rust/nurbs/src/eval.rs`)
- Add `eval_polynomial_f32_with_pos_vel_accel_f64(cps, knots, degree, u) -> (f64, f64, f64)` — extends the existing f32→f64 helper to also return the second derivative.

**FFI** (`rust/kalico-c-api/`)
- `src/runtime_ffi.rs`:
  - Replace `kalico_runtime_step_schedule_peek`, `kalico_runtime_step_schedule_advance` with `kalico_runtime_step_ring_pop(motor_idx, out_t, out_dir) -> bool`.
  - Add `kalico_runtime_producer_step()` — called from the producer Klipper timer.
  - Replace `runtime_handle_widened_now` body with on-demand computation.
  - Add `kalico_runtime_force_idle()` — synchronous foreground flush entry point.
  - Add `kalico_runtime_kick_producer()` — schedule the producer timer (CAS dedupe via `producer_pending` atomic in SharedState).
  - Delete `kalico_runtime_arm_step_timer`, `kalico_runtime_compute_next_step_time`.
- `include/kalico_runtime.h`:
  - Reflect the symbol set above. Bump no version; this is an MCU-internal protocol surface.

**MCU C** (`src/`)
- `runtime_tick.c`:
  - Rewrite `step_time_event` per spec §3.5 (no rate cap, no seqlock retry, single pulse per fire).
  - Add `runtime_producer_timer` — one Klipper `struct timer` whose callback calls `kalico_runtime_producer_step` and self-reschedules iff the FFI signals "more work pending".
  - Add `arm_runtime_timers_at_init` — replaces `arm_step_time_steppers_after_push`; registers producer + per-stepper consumer timers once at runtime init.
  - Delete `MAX_STEP_BURST`, the 50 kHz rate cap, the per-tick step burst diagnostics.
- `stm32/runtime_tick_h7.c` and `stm32/runtime_tick_f4.c`:
  - `runtime_tick_enable` becomes conditional: if `count_modulated_steppers() == 0`, do not enable TIM5 (peripheral clock optional — leave on for register-write safety, but `CR1.CEN` stays 0 and NVIC stays disabled).
  - Delete the rate ping-pong (`count_modulated > 0 ? 40000U : 1000U`) — fixed 40 kHz (H7) / 10 kHz (F4) when enabled.
  - Delete the `widen_seed` print path that depends on TIM5 (the on-demand widening makes it irrelevant).

**Klippy host** (`klippy/`)
- `motion_toolhead.py`:
  - Remove the bench `_on_credit_freed` log spam (diff in working tree).
  - No other change — `step_modes` already plumbed.
- `motion_bridge.py` (and the underlying Rust `motion-bridge` crate where appropriate):
  - On every `push_segment` ACK from the MCU, no extra action needed — the MCU side's `runtime_handle_push_segment` already kicks the producer (via `kalico_runtime_kick_producer` inside the FFI).

**Tests** (`rust/runtime/tests/`)
- `step_ring_spsc.rs` *(new)* — SPSC correctness tests.
- `step_time_degenerate_velocity.rs` *(new)* — accel-from-rest, decel-to-rest, jerk-only-start tests.
- `step_producer.rs` *(new)* — fills ring across multiple synthetic curves.
- `engine_curve_retirement.rs` *(new)* — per-consumer retirement decoupling.
- `engine_force_idle.rs` — update existing (covers `runtime_force_idle` semantics).
- Existing tests: `engine_tick.rs`, `engine_underrun.rs`, `flush_basic.rs`, `flush_drains_queue.rs`, `flush_timeout.rs`, `force_idle_short_circuit.rs`, `step_time_newton.rs` — review each; some assert behaviors that change (boundary-loop semantics, schedule_peek) and either need rewrite or deletion.

---

## Task list

### Task 1: `StepRing` data structure

**Files:**
- Create: `rust/runtime/src/step_ring.rs`
- Modify: `rust/runtime/src/lib.rs` (add `pub mod step_ring;`)
- Test: same file (inline `#[cfg(test)] mod tests`)

- [ ] **Step 1: Write the failing tests**

```rust
// rust/runtime/src/step_ring.rs
#![cfg_attr(not(test), no_std)]

use core::sync::atomic::{AtomicU32, Ordering};

pub const STEP_RING_CAPACITY: usize = 1024;

#[derive(Debug)]
pub struct StepRing {
    pub cycles_abs_lo: [u32; STEP_RING_CAPACITY],
    pub dirs:          [i8;  STEP_RING_CAPACITY],
    pub head:          AtomicU32,
    pub cursor:        AtomicU32,
}

impl StepRing {
    pub const fn new() -> Self {
        Self {
            cycles_abs_lo: [0; STEP_RING_CAPACITY],
            dirs:          [0; STEP_RING_CAPACITY],
            head:          AtomicU32::new(0),
            cursor:        AtomicU32::new(0),
        }
    }

    /// Producer: number of free slots available to write.
    #[inline]
    pub fn space(&self) -> u32 {
        let head   = self.head.load(Ordering::Relaxed);
        let cursor = self.cursor.load(Ordering::Acquire);
        (STEP_RING_CAPACITY as u32).saturating_sub(head.wrapping_sub(cursor))
    }

    /// Producer: append one entry. Caller must have verified `space() > 0`.
    pub fn push(&mut self, cycles_abs_lo: u32, dir: i8) {
        let head = self.head.load(Ordering::Relaxed);
        let slot = (head as usize) % STEP_RING_CAPACITY;
        self.cycles_abs_lo[slot] = cycles_abs_lo;
        self.dirs[slot] = dir;
        self.head.store(head.wrapping_add(1), Ordering::Release);
    }

    /// Consumer: number of entries available to read.
    #[inline]
    pub fn available(&self) -> u32 {
        let head   = self.head.load(Ordering::Acquire);
        let cursor = self.cursor.load(Ordering::Relaxed);
        head.wrapping_sub(cursor)
    }

    /// Consumer: peek the entry at the cursor without advancing. Returns
    /// `None` if the ring is empty.
    pub fn peek_head(&self) -> Option<(u32, i8)> {
        let head   = self.head.load(Ordering::Acquire);
        let cursor = self.cursor.load(Ordering::Relaxed);
        if head == cursor {
            return None;
        }
        let slot = (cursor as usize) % STEP_RING_CAPACITY;
        Some((self.cycles_abs_lo[slot], self.dirs[slot]))
    }

    /// Consumer: peek the *second* entry (the one after the cursor's head).
    /// Returns `None` if fewer than two entries are available.
    pub fn peek_next(&self) -> Option<(u32, i8)> {
        let head   = self.head.load(Ordering::Acquire);
        let cursor = self.cursor.load(Ordering::Relaxed);
        if head.wrapping_sub(cursor) < 2 {
            return None;
        }
        let slot = (cursor.wrapping_add(1) as usize) % STEP_RING_CAPACITY;
        Some((self.cycles_abs_lo[slot], self.dirs[slot]))
    }

    /// Consumer: advance the cursor past `n` entries.
    pub fn advance(&self, n: u32) {
        let cursor = self.cursor.load(Ordering::Relaxed);
        self.cursor.store(cursor.wrapping_add(n), Ordering::Release);
    }

    /// Producer side: reset both counters to 0 and zero the buffer. Used
    /// by `runtime_force_idle` (foreground synchronous; no concurrent
    /// consumer at the moment of call).
    pub fn reset(&mut self) {
        self.head.store(0, Ordering::Release);
        self.cursor.store(0, Ordering::Release);
    }
}

impl Default for StepRing {
    fn default() -> Self { Self::new() }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_ring_has_full_space_and_no_entries() {
        let r = StepRing::new();
        assert_eq!(r.space(), STEP_RING_CAPACITY as u32);
        assert_eq!(r.available(), 0);
        assert!(r.peek_head().is_none());
        assert!(r.peek_next().is_none());
    }

    #[test]
    fn push_then_peek_returns_pushed_entry() {
        let mut r = StepRing::new();
        r.push(0xDEAD_BEEF, 1);
        assert_eq!(r.available(), 1);
        assert_eq!(r.peek_head(), Some((0xDEAD_BEEF, 1)));
        assert!(r.peek_next().is_none());
    }

    #[test]
    fn advance_consumes_entries() {
        let mut r = StepRing::new();
        r.push(100, 1);
        r.push(200, -1);
        r.push(300, 1);
        assert_eq!(r.available(), 3);
        r.advance(2);
        assert_eq!(r.available(), 1);
        assert_eq!(r.peek_head(), Some((300, 1)));
    }

    #[test]
    fn wrap_around_at_capacity_boundary() {
        let mut r = StepRing::new();
        // Fill capacity-1 entries, drain 100, push 200 more — head wraps.
        for i in 0..(STEP_RING_CAPACITY as u32 - 1) {
            r.push(i, if i % 2 == 0 { 1 } else { -1 });
        }
        r.advance(100);
        for i in 0..200 {
            r.push(i + 10000, 1);
        }
        assert_eq!(
            r.available(),
            STEP_RING_CAPACITY as u32 - 1 - 100 + 200,
        );
        // First entry after drain: original index 100, value 100, dir alternates.
        assert_eq!(r.peek_head(), Some((100, 1)));
    }

    #[test]
    fn space_correctly_tracks_head_cursor_delta() {
        let mut r = StepRing::new();
        for _ in 0..500 {
            r.push(0, 0);
        }
        assert_eq!(r.space(), STEP_RING_CAPACITY as u32 - 500);
        r.advance(300);
        assert_eq!(r.space(), STEP_RING_CAPACITY as u32 - 200);
    }

    #[test]
    fn reset_clears_head_and_cursor() {
        let mut r = StepRing::new();
        for _ in 0..500 { r.push(0, 0); }
        r.advance(250);
        r.reset();
        assert_eq!(r.available(), 0);
        assert_eq!(r.space(), STEP_RING_CAPACITY as u32);
    }
}
```

- [ ] **Step 2: Run tests to verify they fail (file doesn't exist yet)**

Run: `cargo test -p kalico-runtime --features std step_ring`
Expected: compile error (module not in `lib.rs`).

- [ ] **Step 3: Add module to `lib.rs`**

```rust
// rust/runtime/src/lib.rs — add:
pub mod step_ring;
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p kalico-runtime --features std step_ring`
Expected: all 6 tests pass.

- [ ] **Step 5: Commit**

```bash
git add rust/runtime/src/step_ring.rs rust/runtime/src/lib.rs
git commit -m "feat(step_ring): SPSC append-only step pulse ring"
```

---

### Task 2: Nurbs eval — return acceleration alongside position and velocity

**Files:**
- Modify: `rust/nurbs/src/eval.rs`
- Test: same file (inline `#[cfg(test)] mod tests`)

The de Boor recurrence already computes the first derivative for free. The second derivative requires one more parallel running term — about 10 lines of code, near-zero added cost on top of the existing `eval_polynomial_with_derivative_f32_to_f64`.

- [ ] **Step 1: Write the failing test**

```rust
// rust/nurbs/src/eval.rs — append to existing `#[cfg(test)] mod tests`:

#[test]
fn pos_vel_accel_on_quadratic_polynomial() {
    // f(u) = u² on u ∈ [0,1] as degree-2 Bézier with cps = [0, 0, 1]
    // (knots [0,0,0,1,1,1]).
    // Verify: f(0.5)=0.25, f'(0.5)=1.0, f''(0.5)=2.0.
    let cps   = vec![0.0_f32, 0.0, 1.0];
    let knots = vec![0.0_f32, 0.0, 0.0, 1.0, 1.0, 1.0];
    let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 2, 0.5);
    assert!((p - 0.25).abs() < 1e-9, "pos={}", p);
    assert!((v - 1.0_f64).abs() < 1e-9, "vel={}", v);
    assert!((a - 2.0_f64).abs() < 1e-9, "accel={}", a);
}

#[test]
fn pos_vel_accel_on_cubic_polynomial() {
    // f(u) = u³ on u ∈ [0,1] as degree-3 Bézier with cps = [0,0,0,1]
    // (knots [0,0,0,0,1,1,1,1]).
    // Verify at u=0.5: f=0.125, f'=0.75, f''=3.0.
    let cps   = vec![0.0_f32, 0.0, 0.0, 1.0];
    let knots = vec![0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 3, 0.5);
    assert!((p - 0.125).abs() < 1e-9, "pos={}", p);
    assert!((v - 0.75_f64).abs() < 1e-9, "vel={}", v);
    assert!((a - 3.0_f64).abs() < 1e-9, "accel={}", a);
}

#[test]
fn pos_vel_accel_on_linear_polynomial_returns_zero_accel() {
    // f(u) = u, degree-1 Bézier cps=[0,1], knots=[0,0,1,1].
    let cps   = vec![0.0_f32, 1.0];
    let knots = vec![0.0_f32, 0.0, 1.0, 1.0];
    let (p, v, a) = eval_polynomial_f32_with_pos_vel_accel_f64(&cps, &knots, 1, 0.3);
    assert!((p - 0.3).abs() < 1e-9);
    assert!((v - 1.0_f64).abs() < 1e-9);
    assert!(a.abs() < 1e-9, "linear curve must have zero second derivative; got {}", a);
}
```

- [ ] **Step 2: Run tests to verify they fail (function doesn't exist)**

Run: `cargo test -p nurbs eval::tests::pos_vel_accel`
Expected: compile error.

- [ ] **Step 3: Implement `eval_polynomial_f32_with_pos_vel_accel_f64`**

Append to `rust/nurbs/src/eval.rs`:

```rust
/// Same recurrence as `eval_polynomial_with_derivative_f32_to_f64`, but
/// also tracks the second derivative. The de Boor walk produces position
/// (`d`) and first derivative (`dd`); we add a parallel `ddd` array that
/// is the difference-of-`dd` recurrence — algebraically the second
/// derivative of the same polynomial.
///
/// Cost over the pos+vel variant: one extra triple of f64 ops per inner
/// iteration. Workspace stays bounded by `WORKSPACE_SIZE` (168 B each ×
/// 3 = 504 B stack).
#[inline]
pub fn eval_polynomial_f32_with_pos_vel_accel_f64(
    cps: &[f32],
    knots: &[f32],
    degree: u8,
    u: f32,
) -> (f64, f64, f64) {
    debug_assert!((degree as usize) <= MAX_DEGREE);
    debug_assert!(knots.len() == cps.len() + (degree as usize) + 1);

    let u_f64 = u as f64;
    let p = usize::from(degree);
    let n = cps.len();

    if degree == 0 {
        return (cps[0] as f64, 0.0, 0.0);
    }
    if degree == 1 {
        // Linear: dd is constant; ddd is zero everywhere.
        let (pos, vel) = eval_polynomial_with_derivative_f32_to_f64(cps, knots, 1, u);
        return (pos, vel, 0.0);
    }

    let k = find_knot_span_f32_with_f64_u(knots, p, n, u_f64);

    let mut d   = [0.0_f64; WORKSPACE_SIZE];
    let mut dd  = [0.0_f64; WORKSPACE_SIZE];
    let mut ddd = [0.0_f64; WORKSPACE_SIZE];
    for j in 0..=p {
        d[j] = cps[k - p + j] as f64;
    }

    for r in 1..=p {
        for j in (r..=p).rev() {
            let lo = knots[k - p + j] as f64;
            let hi = knots[k + 1 + j - r] as f64;
            let denom = hi - lo;
            let old_d_jm1   = d[j - 1];
            let old_d_j     = d[j];
            let old_dd_jm1  = dd[j - 1];
            let old_dd_j    = dd[j];
            let old_ddd_jm1 = ddd[j - 1];
            let old_ddd_j   = ddd[j];
            if denom > 0.0_f64 {
                let inv_denom = 1.0_f64 / denom;
                let alpha = (u_f64 - lo) * inv_denom;
                let one_minus_alpha = 1.0_f64 - alpha;
                ddd[j] = one_minus_alpha * old_ddd_jm1
                       + alpha * old_ddd_j
                       + 2.0 * (old_dd_j - old_dd_jm1) * inv_denom;
                dd[j]  = one_minus_alpha * old_dd_jm1
                       + alpha * old_dd_j
                       + (old_d_j - old_d_jm1) * inv_denom;
                d[j]   = (old_d_j - old_d_jm1) * alpha + old_d_jm1;
            } else {
                d[j]   = old_d_jm1;
                dd[j]  = old_dd_jm1;
                ddd[j] = old_ddd_jm1;
            }
        }
    }

    (d[p], dd[p], ddd[p])
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p nurbs eval::tests::pos_vel_accel`
Expected: 3 tests pass.

- [ ] **Step 5: Run full nurbs test suite (regression check)**

Run: `cargo test -p nurbs`
Expected: all pre-existing nurbs tests still pass.

- [ ] **Step 6: Commit**

```bash
git add rust/nurbs/src/eval.rs
git commit -m "feat(nurbs): eval returning pos+vel+accel from f32 cps/knots"
```

---

### Task 3: `compute_next_step_time` — degenerate-velocity Newton seed

**Files:**
- Modify: `rust/runtime/src/step_time.rs`
- Test: `rust/runtime/tests/step_time_degenerate_velocity.rs` *(new)*
- Modify: existing `rust/runtime/tests/step_time_newton.rs` (update signatures to match new eval shape)

The eval closure changes from `Fn(f32) -> (f64, f64)` to `Fn(f32) -> (f64, f64, f64)`. The Newton seed becomes degree-aware: velocity → accel → fall-back to small forward sample. Bail conditions become precise: Newton non-convergence after `MAX_NEWTON_ITERS`, or `t_try` outside `[t_curr, t_segment_end]`.

- [ ] **Step 1: Write the failing tests in a new file**

```rust
// rust/runtime/tests/step_time_degenerate_velocity.rs
use kalico_runtime::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

/// Accel from rest: x(u) = (a/2)u². At u=0, v=0 exactly. Verify Newton
/// uses the accel-based seed and finds the first step.
#[test]
fn accel_from_rest_first_step_under_quadratic_position() {
    // a = 200 mm/u², step_distance = 1 mm. First step at:
    //   1 = (200/2) u² → u = sqrt(2/200) = sqrt(0.01) = 0.1
    let eval = |u: f32| {
        let u64 = u as f64;
        let pos = 0.5 * 200.0 * u64 * u64;
        let vel = 200.0 * u64;
        let acc = 200.0_f64;
        (pos, vel, acc)
    };
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { t, dir } => {
            assert_eq!(dir, 1);
            assert!((t - 0.1).abs() < 1e-3, "expected t≈0.1, got {}", t);
        }
        StepTimeResult::SegmentExhausted => {
            panic!("Newton must not bail on v(0)=0 when accel is non-zero");
        }
    }
}

/// Pure jerk start: x(u) = (j/6)u³. At u=0, v=0 AND a=0. Verify Newton
/// falls back to the jerk-based seed.
#[test]
fn jerk_only_start_first_step_under_cubic_position() {
    // j = 6000 mm/u³, step_distance = 1 mm.
    //   1 = (6000/6) u³ → u = (6/6000)^(1/3) = 0.1
    let eval = |u: f32| {
        let u64 = u as f64;
        let pos = (6000.0 / 6.0) * u64 * u64 * u64;
        let vel = (6000.0 / 2.0) * u64 * u64;
        let acc = 6000.0 * u64;
        (pos, vel, acc)
    };
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { t, dir } => {
            assert_eq!(dir, 1);
            assert!((t - 0.1).abs() < 1e-2, "expected t≈0.1, got {}", t);
        }
        StepTimeResult::SegmentExhausted => panic!("Newton must not bail when jerk is non-zero"),
    }
}

/// Reverse motion from rest: x(u) = -(a/2)u². Verify dir is -1.
#[test]
fn reverse_accel_from_rest_negative_direction() {
    let eval = |u: f32| {
        let u64 = u as f64;
        let pos = -0.5 * 200.0 * u64 * u64;
        let vel = -200.0 * u64;
        let acc = -200.0_f64;
        (pos, vel, acc)
    };
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    match compute_next_step_time(&q) {
        StepTimeResult::NextAt { dir, .. } => assert_eq!(dir, -1),
        other => panic!("expected NextAt, got {:?}", other),
    }
}

/// Genuinely motionless curve (all derivatives zero) returns
/// SegmentExhausted.
#[test]
fn truly_motionless_curve_exhausts() {
    let eval = |_u: f32| (0.0_f64, 0.0_f64, 0.0_f64);
    let q = StepTimeQuery {
        eval: &eval,
        step_distance: 1.0,
        current_step: 0,
        t_curr: 0.0,
        t_segment_end: 1.0,
    };
    assert!(matches!(compute_next_step_time(&q), StepTimeResult::SegmentExhausted));
}

/// Decel ending at v=0: 200 steps over u∈[0,1] with x(u) = u(2-u).
/// Newton should fire every step until the last one bumps against
/// t_segment_end.
#[test]
fn decel_to_rest_fires_all_but_last_step() {
    // x(u) = 100·(u·(2-u)) = 200u - 100u². v(u) = 200 - 200u. a(u) = -200.
    // Total distance: 100. step_distance = 0.5 → ~200 steps.
    let eval = |u: f32| {
        let u64 = u as f64;
        let pos = 200.0 * u64 - 100.0 * u64 * u64;
        let vel = 200.0 - 200.0 * u64;
        let acc = -200.0_f64;
        (pos, vel, acc)
    };
    let mut t_curr = 0.0_f64;
    let mut count  = 0_i32;
    loop {
        let q = StepTimeQuery {
            eval: &eval,
            step_distance: 0.5,
            current_step: count,
            t_curr,
            t_segment_end: 1.0,
        };
        match compute_next_step_time(&q) {
            StepTimeResult::NextAt { t, dir } => {
                assert_eq!(dir, 1);
                assert!(t > t_curr);
                t_curr = t;
                count += 1;
            }
            StepTimeResult::SegmentExhausted => break,
        }
    }
    // x(1) = 100, so 200 steps at step_distance=0.5 cleanly. Allow one
    // off-by-one for the final step that bumps against t_segment_end.
    assert!(count >= 199 && count <= 200, "fired {} steps", count);
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p kalico-runtime --features std --test step_time_degenerate_velocity`
Expected: compile errors (signature of `Fn(f32) -> (f64, f64)` doesn't match the new `(f64, f64, f64)`).

- [ ] **Step 3: Update `step_time.rs` to the new signature and seeding policy**

Replace the body of `rust/runtime/src/step_time.rs`:

```rust
//! Step-time scheduling: compute the next step pulse time for a stepper
//! by Newton-iterating the position polynomial.
//!
//! ## Why `(pos, vel, accel)`
//!
//! The `eval` closure returns the position, first derivative, and second
//! derivative of the motor-frame position polynomial. Newton's initial
//! `dt` guess is taken from the highest-magnitude non-degenerate
//! derivative:
//!   - `|v| ≥ EPS_VELOCITY` → linear: `dt = step_distance / |v|`
//!   - else `|a| ≥ EPS_ACCEL` → quadratic: `dt = sqrt(2·step_distance/|a|)`
//!   - else fall back to a forward scan (rare; only on triple-degenerate
//!     curves which the planner does not emit in practice).
//!
//! Returning `SegmentExhausted` happens only when `t_try` exits
//! `[t_curr, t_segment_end]` for `MAX_NEWTON_ITERS` consecutive
//! iterations, OR when all three derivatives are below their thresholds
//! AND the forward scan finds no motion within the segment. Mid-segment
//! velocity collapse no longer bails — the accel-based seed handles
//! decel-to-rest correctly.

const NEWTON_TOL_FRACTION: f64 = 1e-6;
const MAX_NEWTON_ITERS: usize = 3;
const EPS_VELOCITY: f64 = 1e-12;
const EPS_ACCEL: f64 = 1e-9;
const FORWARD_SCAN_FRACTION: f64 = 1e-3;  // 0.1% of segment

pub struct StepTimeQuery<'a, F: Fn(f32) -> (f64, f64, f64)> {
    pub eval: &'a F,
    pub step_distance: f64,
    pub current_step: i32,
    pub t_curr: f64,
    pub t_segment_end: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StepTimeResult {
    NextAt { t: f64, dir: i8 },
    SegmentExhausted,
}

pub fn compute_next_step_time<F: Fn(f32) -> (f64, f64, f64)>(
    q: &StepTimeQuery<F>,
) -> StepTimeResult {
    let (_p0, v0, a0) = (q.eval)(q.t_curr as f32);

    // Direction is sign of the first non-degenerate derivative. If all
    // derivatives are degenerate at t_curr, scan forward for motion.
    let dir_i8: i8 = if v0.abs() >= EPS_VELOCITY {
        if v0 > 0.0 { 1 } else { -1 }
    } else if a0.abs() >= EPS_ACCEL {
        if a0 > 0.0 { 1 } else { -1 }
    } else {
        // Scan forward up to t_segment_end looking for motion.
        let span = q.t_segment_end - q.t_curr;
        if span <= 0.0 {
            return StepTimeResult::SegmentExhausted;
        }
        let probe_t = q.t_curr + (span * 0.5);
        let (_, v_probe, a_probe) = (q.eval)(probe_t as f32);
        if v_probe.abs() >= EPS_VELOCITY {
            if v_probe > 0.0 { 1 } else { -1 }
        } else if a_probe.abs() >= EPS_ACCEL {
            if a_probe > 0.0 { 1 } else { -1 }
        } else {
            return StepTimeResult::SegmentExhausted;
        }
    };
    let dir = f64::from(dir_i8);
    let target = (f64::from(q.current_step) + dir) * q.step_distance;

    // Initial guess: pick the cheapest analytic seed available.
    let mut dt = if v0.abs() >= EPS_VELOCITY {
        q.step_distance / v0.abs()
    } else if a0.abs() >= EPS_ACCEL {
        (2.0 * q.step_distance / a0.abs()).sqrt()
    } else {
        // Forward scan got us here — use a fraction of the segment as seed.
        (q.t_segment_end - q.t_curr) * FORWARD_SCAN_FRACTION
    };
    let tol = q.step_distance.abs() * NEWTON_TOL_FRACTION;

    for _ in 0..MAX_NEWTON_ITERS {
        let t_try = q.t_curr + dt;
        if t_try > q.t_segment_end || t_try < q.t_curr {
            return StepTimeResult::SegmentExhausted;
        }
        let (pos, vel, _acc) = (q.eval)(t_try as f32);
        let err = pos - target;
        if err.abs() < tol {
            return StepTimeResult::NextAt { t: t_try, dir: dir_i8 };
        }
        // Newton step uses velocity. If velocity is degenerate at t_try
        // we'd divide by zero — fall back to a fractional step instead
        // of bailing. (This is rare; happens only on degenerate accel
        // crossings inside the segment.)
        if vel.abs() < EPS_VELOCITY {
            dt += (q.t_segment_end - q.t_curr) * FORWARD_SCAN_FRACTION;
            continue;
        }
        dt -= err / vel;
    }

    // Out-of-iters: accept the last candidate within a looser 0.1% tolerance.
    let t_final = q.t_curr + dt;
    if t_final > q.t_segment_end || t_final < q.t_curr {
        return StepTimeResult::SegmentExhausted;
    }
    let (pos, _vel, _acc) = (q.eval)(t_final as f32);
    if (pos - target).abs() < q.step_distance.abs() * 1e-3 {
        StepTimeResult::NextAt { t: t_final, dir: dir_i8 }
    } else {
        StepTimeResult::SegmentExhausted
    }
}
```

- [ ] **Step 4: Update the existing `step_time_newton.rs` test file to the new eval signature**

Open `rust/runtime/tests/step_time_newton.rs` and replace all `|u: f32| -> (f32, f32) { ... }` closures with the `(f64, f64, f64)` shape. Pre-existing assertions on `t` values may shift slightly because Newton's seed quality improved; loosen tolerances if needed (but do not lower correctness expectations).

Example replacement pattern:

```rust
// Before:
let eval = |u: f32| (u, 1.0_f32);

// After:
let eval = |u: f32| (f64::from(u), 1.0_f64, 0.0_f64);
```

- [ ] **Step 5: Run the new and updated tests**

Run:
```
cargo test -p kalico-runtime --features std --test step_time_degenerate_velocity
cargo test -p kalico-runtime --features std --test step_time_newton
```
Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/step_time.rs \
        rust/runtime/tests/step_time_degenerate_velocity.rs \
        rust/runtime/tests/step_time_newton.rs
git commit -m "feat(step_time): degree-aware Newton seed; handle v(0)=0"
```

---

### Task 4: `ProducerState` and `producer_step`

**Files:**
- Create: `rust/runtime/src/step_producer.rs`
- Modify: `rust/runtime/src/lib.rs` (add `pub mod step_producer;`)
- Test: `rust/runtime/tests/step_producer.rs` *(new)*

`ProducerState` owns the per-motor "where am I in Newton-filling" state. `producer_step` is a pure function that takes (rings, producer states, curve queue, curve pool, an "any work done?" output flag) and runs the fill loop for all StepTime motors.

The function is `no_std`-compatible (called from MCU). The test harness calls it on host with mock curves and a fake queue/pool.

- [ ] **Step 1: Write the failing test**

```rust
// rust/runtime/tests/step_producer.rs
use kalico_runtime::step_producer::{producer_step, ProducerState, ProducerTickResult};
use kalico_runtime::step_ring::{StepRing, STEP_RING_CAPACITY};

/// Minimal mock for the test: a single motor, fed one curve at a time.
/// Production wiring (Task 5) uses the engine's segment queue + curve
/// pool; the function under test takes a closure to fetch the next
/// curve to keep this test isolated.

#[test]
fn producer_fills_ring_from_a_single_linear_curve() {
    // 100 steps over the curve, step_distance 0.1, x(u) = 10·u.
    let curve_eval = |u: f32| {
        let u64 = u as f64;
        (10.0 * u64, 10.0_f64, 0.0_f64)
    };
    let mut ring  = StepRing::new();
    let mut state = ProducerState::new(0.1_f64);

    // First call: producer should fully fill the ring (or finish the curve).
    let result = producer_step(
        &mut [&mut ring],
        &mut [&mut state],
        &mut [Some(&curve_eval)],
        &[0_u64],         // curve_t_start per motor (cycles)
        &[1_000_000_u64], // curve_duration per motor (cycles)
        16,               // batch cap
    );

    // Ring should have 16 entries (batch cap reached), more work pending.
    assert_eq!(ring.available(), 16);
    assert_eq!(result, ProducerTickResult::WorkPending);
}

#[test]
fn producer_completes_short_curve_in_one_call() {
    // 5 steps total, step_distance 2.0, x(u) = 10·u.
    let curve_eval = |u: f32| {
        let u64 = u as f64;
        (10.0 * u64, 10.0_f64, 0.0_f64)
    };
    let mut ring  = StepRing::new();
    let mut state = ProducerState::new(2.0_f64);

    let result = producer_step(
        &mut [&mut ring],
        &mut [&mut state],
        &mut [Some(&curve_eval)],
        &[0_u64],
        &[1_000_000_u64],
        32,
    );

    // 5 steps emitted; Newton's next call returned SegmentExhausted.
    assert_eq!(ring.available(), 5);
    assert!(state.is_idle(), "expected idle after curve completed");
    assert_eq!(result, ProducerTickResult::AllIdle);
}

#[test]
fn producer_respects_ring_space_backpressure() {
    let curve_eval = |u: f32| {
        let u64 = u as f64;
        (1000.0 * u64, 1000.0_f64, 0.0_f64)
    };
    let mut ring  = StepRing::new();
    let mut state = ProducerState::new(0.001_f64);  // 1,000,000 steps

    // Drain the ring to leave space=10 and try to fill.
    for _ in 0..(STEP_RING_CAPACITY - 10) {
        ring.push(0, 1);
    }
    let pre_head = ring.available();

    producer_step(
        &mut [&mut ring],
        &mut [&mut state],
        &mut [Some(&curve_eval)],
        &[0_u64],
        &[1_000_000_u64],
        100,
    );

    // Should fill exactly the 10 free slots; curve has more work pending.
    assert_eq!(ring.available(), pre_head + 10);
    assert!(!state.is_idle());
}
```

- [ ] **Step 2: Write the implementation skeleton (test will not yet pass)**

Create `rust/runtime/src/step_producer.rs`:

```rust
//! Per-motor step-time producer. Drains step times from the active curve
//! into the motor's `StepRing` via Newton iteration. Called from the
//! producer Klipper timer (event-driven; see runtime_tick.c).

use crate::step_ring::StepRing;
use crate::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

/// Per-motor producer state. Newton resume bookkeeping between batch
/// calls within a single curve.
#[derive(Debug)]
pub struct ProducerState {
    step_distance: f64,
    /// Resume point in normalized u-domain. `None` when no curve is
    /// currently being filled.
    t_resume: Option<f64>,
    /// Motor step counter at curve start (for absolute target math).
    step_at_curve_start: i32,
    /// How many steps have been pushed for the current curve so far.
    steps_pushed_this_curve: i32,
}

impl ProducerState {
    pub const fn new(step_distance: f64) -> Self {
        Self {
            step_distance,
            t_resume: None,
            step_at_curve_start: 0,
            steps_pushed_this_curve: 0,
        }
    }

    /// True iff no curve is currently being filled.
    pub const fn is_idle(&self) -> bool {
        self.t_resume.is_none()
    }

    /// Start a fresh curve for this motor. `step_at_start` is the motor's
    /// integer step counter at the curve's u=0; subsequent step targets
    /// are `(step_at_start + n) * step_distance`.
    pub fn start_curve(&mut self, step_at_start: i32) {
        self.t_resume = Some(0.0);
        self.step_at_curve_start = step_at_start;
        self.steps_pushed_this_curve = 0;
    }

    pub fn clear(&mut self) {
        self.t_resume = None;
        self.steps_pushed_this_curve = 0;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProducerTickResult {
    /// At least one motor has more work but ran out of batch budget or
    /// ring space. Producer should be rescheduled.
    WorkPending,
    /// All motors are idle (no current curve and the caller could not
    /// provide a next one). Producer should wait for a kick.
    AllIdle,
}

/// Fill rings for every motor that has a curve closure. The caller
/// provides per-motor closures returning `(pos, vel, accel)` in motor
/// frame; `None` means "no curve available for this motor right now"
/// (skip it). Returns whether more work is pending overall.
///
/// `curve_t_start` and `curve_duration` are MCU clock cycles. They are
/// constant for the duration of one curve.
pub fn producer_step<'a, F>(
    rings: &mut [&mut StepRing],
    states: &mut [&mut ProducerState],
    evals: &mut [Option<&F>],
    curve_t_start: &[u64],
    curve_duration: &[u64],
    batch_cap: u32,
) -> ProducerTickResult
where
    F: Fn(f32) -> (f64, f64, f64),
{
    debug_assert_eq!(rings.len(), states.len());
    debug_assert_eq!(rings.len(), evals.len());
    debug_assert_eq!(rings.len(), curve_t_start.len());
    debug_assert_eq!(rings.len(), curve_duration.len());

    let mut any_work_pending = false;

    for i in 0..rings.len() {
        let Some(eval) = evals[i].as_ref() else {
            continue;  // No curve for this motor.
        };
        let state = &mut *states[i];
        let ring  = &mut *rings[i];

        // If state is idle, the caller hasn't started the curve yet;
        // start at step counter 0 for now (caller can pre-set via
        // start_curve if it wants absolute counters).
        if state.is_idle() {
            state.start_curve(0);
        }

        let mut filled = 0_u32;
        let duration_f64 = curve_duration[i] as f64;
        let t_segment_end = 1.0_f64;

        while filled < batch_cap && ring.space() > 0 {
            let q = StepTimeQuery {
                eval: *eval,
                step_distance: state.step_distance,
                current_step: state.step_at_curve_start
                    .wrapping_add(state.steps_pushed_this_curve),
                t_curr: state.t_resume.unwrap_or(0.0),
                t_segment_end,
            };
            match compute_next_step_time(&q) {
                StepTimeResult::NextAt { t, dir } => {
                    let dt_cycles = (t * duration_f64) as u64;
                    let abs_cycles = curve_t_start[i].saturating_add(dt_cycles);
                    ring.push(abs_cycles as u32, dir);
                    state.t_resume = Some(t);
                    state.steps_pushed_this_curve =
                        state.steps_pushed_this_curve.saturating_add(i32::from(dir));
                    filled += 1;
                }
                StepTimeResult::SegmentExhausted => {
                    state.clear();
                    break;
                }
            }
        }
        if !state.is_idle() {
            any_work_pending = true;
        }
    }

    if any_work_pending { ProducerTickResult::WorkPending } else { ProducerTickResult::AllIdle }
}
```

Add to `rust/runtime/src/lib.rs`:

```rust
pub mod step_producer;
```

- [ ] **Step 3: Run tests to verify they pass**

Run: `cargo test -p kalico-runtime --features std --test step_producer`
Expected: all 3 tests pass.

- [ ] **Step 4: Commit**

```bash
git add rust/runtime/src/step_producer.rs rust/runtime/src/lib.rs \
        rust/runtime/tests/step_producer.rs
git commit -m "feat(step_producer): producer_step Newton-fills per-motor ring"
```

---

### Task 5: Engine integration — per-motor step rings, producer entry point, segment-queue per-motor cursor, curve retirement decoupling

This is the largest task. Combines (a) replacing `step_schedules` array with `step_rings` array, (b) introducing per-motor segment-queue cursors, (c) per-curve consumer-remaining tracking for retirement, (d) the engine's host-callable `producer_step` entry point that wires rings + states + closures + curve queue together.

**Files:**
- Modify: `rust/runtime/src/engine.rs` (substantial — see step 3)
- Modify: `rust/runtime/src/segment.rs` (consumer-remaining field)
- Modify: `rust/runtime/src/state.rs` (`producer_pending` atomic added)
- Test: `rust/runtime/tests/engine_producer_integration.rs` *(new)*
- Test: `rust/runtime/tests/engine_curve_retirement.rs` *(new)*

- [ ] **Step 1: Write the failing integration test for the producer path**

Create `rust/runtime/tests/engine_producer_integration.rs`:

```rust
//! End-to-end test for the producer path inside the engine. Pushes a
//! synthetic segment, calls `engine.producer_step`, verifies the per-
//! motor rings receive correct step times.

use kalico_runtime::engine::Engine;
use kalico_runtime::segment::{Segment, EMode, KinematicTag};
use kalico_runtime::curve_pool::CurvePool;
use kalico_runtime::state::SharedState;

// Tests below assume helpers like `make_linear_x_curve` exist in
// `rust/runtime/tests/fixtures/` — reuse / extend the existing fixtures
// directory.

#[test]
fn one_segment_one_motor_fills_ring() {
    let pool   = CurvePool::new();
    let shared = SharedState::new(/* args */);
    let mut engine = Engine::new(&pool, &shared, /* ... */);

    // Build a Cartesian X-only segment: X 0→10 mm over 100 ms.
    let seg = build_test_segment_cartesian_x(&pool, /* x_start */ 0.0, /* x_end */ 10.0,
                                             /* t_start_cycles */ 0,
                                             /* duration_cycles */ 18_000_000);
    engine.push_segment(seg).expect("push ok");

    engine.producer_step();

    // Motor 0 (X) should have ~1600 entries (at 160 spm); cursor stays at 0.
    let avail = engine.step_ring(0).available();
    assert!(avail >= 1500 && avail <= 1024,
            "motor 0 ring should be filled up to capacity for a long curve; got {}", avail);
}

#[test]
fn empty_queue_returns_all_idle() {
    let pool   = CurvePool::new();
    let shared = SharedState::new(/* args */);
    let mut engine = Engine::new(&pool, &shared, /* ... */);
    engine.producer_step();
    for m in 0..4 {
        assert_eq!(engine.step_ring(m).available(), 0);
    }
}

#[test]
fn push_segment_sets_producer_pending() {
    let pool   = CurvePool::new();
    let shared = SharedState::new(/* args */);
    let mut engine = Engine::new(&pool, &shared, /* ... */);
    let seg = build_test_segment_cartesian_x(&pool, 0.0, 10.0, 0, 18_000_000);
    engine.push_segment(seg).expect("push ok");
    assert!(shared.producer_pending.load(core::sync::atomic::Ordering::Acquire),
            "push_segment must kick the producer");
}
```

(If `Engine::new` / `Engine::push_segment` / `engine.step_ring` / `engine.producer_step` have different signatures in your tree, adjust accordingly. The fixture helpers — `build_test_segment_cartesian_x` etc. — must be added to `rust/runtime/tests/fixtures/` if absent.)

- [ ] **Step 2: Write the failing test for retirement decoupling**

Create `rust/runtime/tests/engine_curve_retirement.rs`:

```rust
//! Verify that a curve's pool slot is released only when every motor
//! consuming it has finished, and that the producer-completion path
//! triggers retirement independently of any wall-clock "current segment"
//! progress.

#[test]
fn slot_retires_when_producer_finishes_curve_with_no_modulated_consumers() {
    let pool   = CurvePool::new();
    let shared = SharedState::new(/* args */);
    let mut engine = Engine::new(&pool, &shared, /* ... */);

    // All motors StepTime by default. Push a short curve. Run producer
    // until ProducerTickResult::AllIdle. Slot should be released.
    let seg = build_test_segment_cartesian_x(&pool, 0.0, 0.5, 0, 1_000_000);
    let x_handle = seg.x_handle;
    engine.push_segment(seg).expect("push ok");
    while engine.producer_step() == kalico_runtime::step_producer::ProducerTickResult::WorkPending {}
    assert!(pool.is_slot_free(x_handle.slot_idx),
            "X slot should be retired after producer completes the curve");
}

#[test]
fn slot_does_not_retire_until_modulated_wallclock_also_past_t_end() {
    let pool   = CurvePool::new();
    let shared = SharedState::new(/* args */);
    let mut engine = Engine::new(&pool, &shared, /* ... */);
    // Set motor 0 to Modulated.
    shared.step_modes[0].store(kalico_runtime::state::StepMode::Modulated as u8,
                               core::sync::atomic::Ordering::Release);

    let seg = build_test_segment_cartesian_x(&pool, 0.0, 0.5, /* t_start */ 0,
                                             /* duration */ 1_000_000);
    let x_handle = seg.x_handle;
    engine.push_segment(seg).expect("push ok");

    // Run producer to completion — StepTime side of motor 0 (well, motor 0
    // is now Modulated so it doesn't have a StepTime producer; the engine
    // skips it). Other motors are idle. Curve is now waiting for
    // wall-clock retirement.
    while engine.producer_step() == kalico_runtime::step_producer::ProducerTickResult::WorkPending {}
    assert!(!pool.is_slot_free(x_handle.slot_idx),
            "X slot must not retire while Modulated consumer (motor 0) still has wall-clock to cross");

    // Now simulate wall-clock past t_end via the Modulated tick.
    engine.runtime_modulated_tick(/* now */ 2_000_000, &shared);
    assert!(pool.is_slot_free(x_handle.slot_idx),
            "X slot should retire once Modulated wall-clock has crossed t_end");
}
```

- [ ] **Step 3: Implement the engine changes**

This step rewrites several pieces of `engine.rs`. Below is the consolidated set of changes; apply them as a single coherent edit.

**3a. Replace `step_schedules: [StepSchedule; 4]` with `step_rings: [StepRing; 4]` and add `producer_states: [ProducerState; 4]`:**

```rust
// rust/runtime/src/engine.rs (top of Engine struct):
use crate::step_ring::StepRing;
use crate::step_producer::ProducerState;

pub struct Engine {
    // ... existing per-motor StepAccumulator state etc. ...
    pub step_rings: [StepRing; 4],
    pub producer_states: [ProducerState; 4],
    // ... existing prev_x / prev_y / prev_z / e_accumulator etc. ...

    // Per-motor cursor into the segment queue. Indexed in monotonic
    // counter form against `Segment::id`; the queue itself stays one
    // shared Producer/Consumer SPSC.
    pub motor_curve_cursor: [u32; 4],

    // Delete: `step_schedules`, `next_step_idx`, `schedule_seq`,
    // `current`, `tick_counter`, all boundary-loop fields.
}
```

**3b. `Segment` gains a per-axis "consumers-remaining" mask. Each bit represents a motor that still needs this curve:**

```rust
// rust/runtime/src/segment.rs (in Segment struct):
pub struct Segment {
    // ... existing fields ...
    /// Per-axis-curve consumer bitmask. Each curve handle (x/y/z/e) has
    /// a u4 mask in the low/mid/high nibbles. A bit set = "motor N still
    /// needs this curve". The Modulated retirement path clears its bit
    /// on wall-clock t_end cross; the producer clears its bit on Newton
    /// SegmentExhausted. When a curve's nibble reaches 0, the slot is
    /// retired.
    pub consumers_remaining: u16,
}
```

(Existing `Segment` already encodes axis presence via the UNUSED sentinel; the bitmask is a per-motor refinement layered on top. Specifically: a motor's bit is set initially iff that motor will consume the curve based on `kinematics` and the handle being non-UNUSED.)

**3c. New `Engine::producer_step` method — the entry point called from the producer Klipper timer:**

```rust
impl Engine {
    pub fn producer_step(&mut self, pool: &CurvePool, queue: &SegmentQueue,
                         shared: &SharedState) -> ProducerTickResult {
        // 1. Clear producer_pending. Kicks arriving after this point will
        //    re-set it and trigger another producer_step call.
        shared.producer_pending.store(false, Ordering::Release);

        // 2. For each motor that is StepTime mode AND whose producer state
        //    is idle, pull its next curve from the segment queue (using the
        //    motor's per-motor cursor) and start the producer state.
        for i in 0..4 {
            let mode = shared.step_modes[i].load(Ordering::Acquire);
            if mode != StepMode::StepTime as u8 { continue; }
            if !self.producer_states[i].is_idle() { continue; }

            // Look up next segment for this motor: scan the queue from
            // motor_curve_cursor[i] forward, find the first segment whose
            // axis-i curve is present (not UNUSED).
            if let Some((seg, _idx)) = peek_next_curve_for_motor(queue, self.motor_curve_cursor[i], i) {
                // Initialize producer state with the motor's current step
                // count (so step targets are absolute).
                let initial_step = shared.stepper_counts[i].load(Ordering::Acquire);
                self.producer_states[i].start_curve(initial_step);
                // (Bookkeeping for "which segment am I currently filling
                // for motor i" stored in a per-motor slot so we can clear
                // the consumer bit when Newton finishes.)
                self.motor_current_segment_id[i] = Some(seg.id);
            }
        }

        // 3. Call producer_step with per-motor closures, t_starts, durations.
        //    Each closure resolves the motor's current curve via the pool
        //    + kinematic transform (CoreXY mixes X/Y for motors 0/1).
        //    On WorkPending: more steps to compute later; AllIdle: idle.
        // 4. After producer_step returns: for any motor whose state is
        //    now idle (curve fully Newton-filled), advance
        //    motor_curve_cursor[i] past the segment it just finished, and
        //    clear that motor's bit in the segment's consumers_remaining.
        //    If consumers_remaining reaches 0 for the segment, retire its
        //    pool slots and emit kalico_credit_freed for the host.
        // 5. Return AllIdle / WorkPending based on aggregate result.

        // ... full implementation (omitted here for brevity; ~150 lines) ...
    }
}
```

Implementation details that must be filled in:
- `peek_next_curve_for_motor(queue, cursor, motor_idx)`: walks the queue from `cursor` forward, returns the first segment whose axis-`motor_idx`-effective curve handle is non-UNUSED. For CoreXY motors 0/1, axes are X and Y combined — "non-UNUSED" means at least one of X/Y is non-UNUSED.
- The per-motor eval closure builds on the existing `eval_position_and_du` from today's `precompute_step_schedules` but switches to `(pos, vel, accel)` via the new nurbs eval (Task 2).
- `motor_current_segment_id[i]` is a new `[Option<u32>; 4]` field on `Engine` so we know which segment to clear the consumer bit on when Newton completes.
- `kalico_credit_freed` event emission stays where it is today (in the trace path); the trigger moves from boundary-loop retirement to consumers_remaining-reaches-zero.

**3d. Delete from `Engine`:**
- `tick`, `tick_with_current`, `precompute_step_schedules`, `refill_step_schedules`, `arm_step_timer`, `force_idle` ISR-side check (moves to T13), `tick_counter` (replaced by `producer_runs_total` in SharedState).
- `current: Option<Segment>` — replaced by per-motor `motor_current_segment_id`.
- All boundary-loop infrastructure: `MAX_BOUNDARY_ITERS`, `debug_last_now`, `debug_last_tstart`, `debug_last_duration`, `injected_iter_start` (#[cfg(test)] field), `boundary_loop_skipped_segments` increment, `step_gen_activations` increment.

**3e. Add to `SharedState`:**

```rust
// rust/runtime/src/state.rs:
pub struct SharedState {
    // ... existing fields ...

    /// Producer-pending dedupe flag. Kickers (push_segment, low-water
    /// hook) CAS-set false→true; if win, schedule the producer timer.
    /// Producer clears at start of run.
    pub producer_pending: AtomicBool,

    /// Replacement for the old `tick_counter`: per-MCU producer run
    /// count. Surfaces via status drain.
    pub producer_runs_total: AtomicU64,

    /// Replacement for the per-stepper schedule diagnostics.
    pub consumer_pulses_total: [AtomicU64; 4],
    pub consumer_underrun_total: [AtomicU64; 4],
    pub ring_high_water: [AtomicU32; 4],
}
```

Delete from `SharedState` (mirror of spec §4.2):
`first_tick_segment_id`, `first_tick_delta_steps`, `step_gen_activations`, `boundary_loop_skipped_segments`, `catch_up_nonzero_emits`, `catch_up_total_pulses`, `max_boundary_lateness_cycles`, `peek_seq_odd_count`, `peek_torn_count`, `peek_cursor_at_total_count`, `peek_ok_count`, `peek_last_count_m0`, `peek_last_count_m1`.

- [ ] **Step 4: Run integration tests**

Run:
```
cargo test -p kalico-runtime --features std --test engine_producer_integration
cargo test -p kalico-runtime --features std --test engine_curve_retirement
```
Expected: all tests pass.

- [ ] **Step 5: Run the existing engine test suite to identify regressions**

Run: `cargo test -p kalico-runtime --features std --test engine_tick`

Expected outcome: this test file likely **fails** — its assertions encode the old `Engine::tick` boundary-loop semantics. Don't paper over with band-aids. For each failing assertion, decide: (a) the behavior under test is meaningful in the new architecture → rewrite the test to assert the new equivalent; (b) it was a property of the old architecture only → delete the test. Document the rewrites in commit messages.

Similarly run and triage: `engine_underrun.rs`, `flush_basic.rs`, `flush_drains_queue.rs`, `flush_timeout.rs`, `force_idle_short_circuit.rs`.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/engine.rs rust/runtime/src/segment.rs \
        rust/runtime/src/state.rs \
        rust/runtime/tests/engine_producer_integration.rs \
        rust/runtime/tests/engine_curve_retirement.rs \
        rust/runtime/tests/engine_tick.rs  # if rewritten/trimmed
git commit -m "feat(engine): per-motor step rings, decoupled curve retirement"
```

---

### Task 6: FFI surface — replace schedule peek/advance with ring pop; add producer_step / kick / force_idle entry points

**Files:**
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs`
- Modify: `rust/kalico-c-api/include/kalico_runtime.h`
- Test: `rust/kalico-c-api/tests/ffi_smoke.rs` *(if exists; otherwise add minimal coverage)*

- [ ] **Step 1: Define the new FFI surface in `runtime_ffi.rs`**

```rust
/// Pop one entry from motor `motor_idx`'s step ring. Returns `false` if
/// the ring is empty.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_step_ring_pop(
    rt: *mut KalicoRuntime,
    motor_idx: u8,
    out_cycles_abs_lo: *mut u32,
    out_dir: *mut i8,
) -> bool {
    if rt.is_null() || (motor_idx as usize) >= 4 || out_cycles_abs_lo.is_null() || out_dir.is_null() {
        return false;
    }
    if !INIT_DONE.load(Ordering::Acquire) { return false; }
    let ctx = unsafe { rt.cast::<RuntimeContext>().as_ref().unwrap() };
    let ring = ctx.engine.lock().step_ring(motor_idx as usize);
    let Some((t, dir)) = ring.peek_head() else { return false; };
    let now_low = read_timer_low();  // see helper in same file
    if (t.wrapping_sub(now_low) as i32) > 0 {
        // Future entry — don't pop, leave for consumer to wait.
        unsafe {
            *out_cycles_abs_lo = t;
            *out_dir = dir;
        }
        return false;  // No pop occurred.
    }
    unsafe {
        *out_cycles_abs_lo = t;
        *out_dir = dir;
    }
    ring.advance(1);
    true
}

/// Peek the *next* (second-from-cursor) entry. Used by the consumer to
/// reschedule itself. Returns false if no second entry exists.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_step_ring_peek_next(
    rt: *mut KalicoRuntime,
    motor_idx: u8,
    out_cycles_abs_lo: *mut u32,
) -> bool { /* ... */ }

/// Producer timer callback entry point. Runs one batch and returns
/// whether more work is pending.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_producer_step(rt: *mut KalicoRuntime) -> bool {
    // Returns true = work pending (caller should self-reschedule);
    // false = all idle (caller waits for a kick).
    // ...
}

/// Kick the producer (CAS-set producer_pending, return whether it was
/// the winning kicker — caller schedules the timer iff true).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_kick_producer(rt: *mut KalicoRuntime) -> bool {
    // ...
}

/// Foreground synchronous flush. Clears queue, rings, producer states;
/// releases all in-flight curve pool slots. Returns success.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_force_idle(rt: *mut KalicoRuntime) -> bool {
    // ...
}
```

Delete: `kalico_runtime_step_schedule_peek`, `kalico_runtime_step_schedule_advance`, `kalico_runtime_arm_step_timer`, `kalico_runtime_compute_next_step_time`.

- [ ] **Step 2: Mirror in `kalico_runtime.h`**

```c
bool kalico_runtime_step_ring_pop(KalicoRuntime *rt, uint8_t motor_idx,
                                  uint32_t *out_cycles_abs_lo,
                                  int8_t *out_dir);
bool kalico_runtime_step_ring_peek_next(KalicoRuntime *rt, uint8_t motor_idx,
                                        uint32_t *out_cycles_abs_lo);
bool kalico_runtime_producer_step(KalicoRuntime *rt);
bool kalico_runtime_kick_producer(KalicoRuntime *rt);
bool kalico_runtime_force_idle(KalicoRuntime *rt);
```

Delete the four deleted Rust exports' declarations.

- [ ] **Step 3: Update `runtime_handle_widened_now`**

Replace the body with on-demand computation:

```rust
#[unsafe(no_mangle)]
pub unsafe extern "C" fn runtime_handle_widened_now(rt: *mut KalicoRuntime) -> u64 {
    extern "C" {
        fn timer_read_time() -> u32;
        static stats_send_time: u32;
        static stats_send_time_high: u32;
    }
    unsafe {
        let low  = timer_read_time();
        let high = stats_send_time_high + ((low < stats_send_time) as u32);
        ((high as u64) << 32) | (low as u64)
    }
}
```

(The function signature stays the same; only the body changes. No more SharedState seqlock dependency.)

- [ ] **Step 4: Build to confirm FFI compiles**

Run: `cargo build -p kalico-c-api`
Expected: clean build.

- [ ] **Step 5: Commit**

```bash
git add rust/kalico-c-api/src/runtime_ffi.rs rust/kalico-c-api/include/kalico_runtime.h
git commit -m "feat(ffi): step_ring_pop / producer_step / kick / force_idle"
```

---

### Task 7: Rewrite the C-side consumer (`step_time_event`)

**Files:**
- Modify: `src/runtime_tick.c` (replace the `step_time_event` body from spec §3.5)

- [ ] **Step 1: Replace `step_time_event` body**

```c
// src/runtime_tick.c — replace existing step_time_event:
static uint_fast8_t
step_time_event(struct timer *t)
{
    step_time_event_fires++;
    struct step_timer_ctx *ctx =
        container_of(t, struct step_timer_ctx, timer);
    uint8_t motor = ctx->stepper_idx;

    uint32_t t_next;
    int8_t dir;
    bool popped = kalico_runtime_step_ring_pop(runtime_handle, motor,
                                               &t_next, &dir);

    if (!popped) {
        // Either the ring is empty (no work) or the head entry is in
        // the future. If a future entry was provided via out params,
        // schedule there; otherwise short poll.
        if (t_next != 0) {
            t->waketime = t_next;
        } else {
            t->waketime += runtime_clock_freq / 10000U;  // 100 µs
        }
        return SF_RESCHEDULE;
    }

    // A pulse fires.
    runtime_emit_step_pulses(motor, dir >= 0 ? 1 : -1);
    kalico_runtime_apply_step(runtime_handle, motor, dir >= 0 ? 1 : -1);
    runtime_endstop_sample_one(motor);

    // Kick producer if low-water — single CAS-set, dedupe in Rust.
    if (kalico_runtime_step_ring_available(runtime_handle, motor) < LOW_WATER) {
        if (kalico_runtime_kick_producer(runtime_handle))
            sched_add_timer(&runtime_producer_timer.timer);
    }

    // Reschedule for the next entry, or short poll if ring is empty.
    uint32_t t_next2;
    if (kalico_runtime_step_ring_peek_next(runtime_handle, motor, &t_next2)) {
        t->waketime = t_next2;
    } else {
        t->waketime = timer_read_time() + (runtime_clock_freq / 10000U);
    }
    return SF_RESCHEDULE;
}
```

Delete: `MAX_STEP_BURST`, all the rate-limit code from the 2026-05-14 hack, the seqlock-retry diagnostic emits.

`LOW_WATER` is a new constant near the top of the file: `#define LOW_WATER 256` (matches spec §3.4 — `N/4 = 256` for `N=1024`).

`kalico_runtime_step_ring_available` is an additional FFI export (one-line wrapper around `ring.available()`). Add it in Task 6 retroactively if missed; if so, include in this commit.

- [ ] **Step 2: Build the H7 firmware**

Per memory:
```bash
cd /Users/daniladergachev/Developer/kalico
cp .config.h7.bak .config        # H7 build config
cargo clean
make clean
make
```
Expected: clean build.

- [ ] **Step 3: Build the F4 firmware** (regression check — even if F4 not in this MVP scope, the code must compile)

```bash
cp .config.f446.test .config
cargo clean
make clean
make
```
Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add src/runtime_tick.c
git commit -m "feat(mcu): simplified step_time_event consumer"
```

---

### Task 8: C-side producer Klipper timer + kick mechanism

**Files:**
- Modify: `src/runtime_tick.c`
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (already from T6 — `kalico_runtime_kick_producer` returns whether the kicker won the CAS)

- [ ] **Step 1: Add producer timer struct and callback**

```c
// src/runtime_tick.c — near other runtime timers:

static struct {
    struct timer timer;
    uint8_t enabled;
} runtime_producer_timer;

static uint_fast8_t
runtime_producer_event(struct timer *t)
{
    bool work_pending = kalico_runtime_producer_step(runtime_handle);
    if (work_pending) {
        // Self-reschedule at now to drain the next batch.
        t->waketime = timer_read_time();
        return SF_RESCHEDULE;
    }
    // No more work — wait for a kick to wake us.
    runtime_producer_timer.enabled = 0;
    return SF_DONE;
}
```

(Note: `SF_DONE` removes the timer from Klipper's schedule. The kick mechanism in `kalico_runtime_kick_producer` re-adds it via `sched_add_timer`.)

- [ ] **Step 2: Register the producer timer once at init**

Replace `arm_step_time_steppers_after_push` with a single `arm_runtime_timers_at_init` that registers consumers and producer at runtime init (not on every push):

```c
void
arm_runtime_timers_at_init(void)
{
    if (!runtime_handle) return;

    for (uint8_t i = 0; i < MAX_STEPPER_OIDS_C; i++) {
        if (step_timers[i].enabled) continue;
        uint8_t mode = kalico_runtime_get_step_mode(runtime_handle, i);
        if (mode != 1 /* StepMode::StepTime */) continue;
        step_timers[i].timer.func = step_time_event;
        step_timers[i].stepper_idx = i;
        step_timers[i].enabled = 1;
        step_timers[i].timer.waketime = timer_read_time() + runtime_clock_freq / 10000U;
        sched_add_timer(&step_timers[i].timer);
    }

    // Producer timer — added on first push, not at init (no work yet).
    runtime_producer_timer.timer.func = runtime_producer_event;
    runtime_producer_timer.enabled = 0;
}
```

- [ ] **Step 3: Wire `push_segment` to kick the producer**

In the C-side `handle_push_segment` dispatch (`src/runtime_commands.c` or `src/kalico_dispatch.c` — `grep` for the function):

```c
// After kalico_runtime_handle_push_segment returns KALICO_OK:
if (kalico_runtime_kick_producer(runtime_handle)) {
    if (!runtime_producer_timer.enabled) {
        runtime_producer_timer.enabled = 1;
        runtime_producer_timer.timer.waketime = timer_read_time();
        sched_add_timer(&runtime_producer_timer.timer);
    }
    // If already enabled, the in-flight producer run will see the
    // CAS-set flag and either re-run or drain naturally.
}
```

- [ ] **Step 4: Build H7 + F4**

Same commands as Task 7 steps 2-3.

- [ ] **Step 5: Commit**

```bash
git add src/runtime_tick.c src/runtime_commands.c  # or whichever has handle_push_segment
git commit -m "feat(mcu): producer Klipper timer + push_segment kick"
```

---

### Task 9: TIM5 gating — enable iff `count_modulated > 0`

**Files:**
- Modify: `src/stm32/runtime_tick_h7.c`
- Modify: `src/stm32/runtime_tick_f4.c`

- [ ] **Step 1: H7 `runtime_tick_enable` becomes conditional**

```c
// src/stm32/runtime_tick_h7.c — replace runtime_tick_enable:
void
runtime_tick_enable(void)
{
    if (!runtime_handle) return;

    uint8_t n_modulated = kalico_runtime_count_modulated_steppers(runtime_handle);
    if (n_modulated == 0) {
        // No phase-stepping consumers — TIM5 stays disabled. The
        // StepTime path (producer timer + per-stepper consumer timers)
        // handles all motion.
        return;
    }

    // From here on: Modulated path is live. TIM5 at 40 kHz.
    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR = (runtime_clock_freq / 40000U) - 1U;
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0;
    TIM5->SR = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}
```

Delete the WidenState seed (`runtime_handle_seed_widen`) call — widening is now on-demand (T6 changed `runtime_handle_widened_now`). Delete the `widen_seed` `output` print.

- [ ] **Step 2: F4 same treatment**

```c
// src/stm32/runtime_tick_f4.c — same conditional:
void
runtime_tick_enable(void)
{
    if (!runtime_handle) return;
    if (kalico_runtime_count_modulated_steppers(runtime_handle) == 0) return;

    TIM5->CR1 &= ~TIM_CR1_CEN;
    TIM5->ARR = (runtime_clock_freq / 10000U) - 1U;  // F4 stays at 10 kHz when on
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR = 0;
    TIM5->SR = ~TIM_SR_UIF;
    TIM5->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM5_IRQn);
}
```

- [ ] **Step 3: Build H7 and verify TIM5 stays off when all StepTime**

```bash
cp .config.h7.bak .config
cargo clean && make clean && make
```

Expected: clean build. Bench validation deferred to Task 16 — for now, code-level review confirms TIM5 enable path is unreachable when `count_modulated == 0`.

- [ ] **Step 4: Commit**

```bash
git add src/stm32/runtime_tick_h7.c src/stm32/runtime_tick_f4.c
git commit -m "feat(mcu): TIM5 enabled only when count_modulated > 0"
```

---

### Task 10: `runtime_modulated_tick` — TIM5's only Rust callback

**Files:**
- Modify: `rust/runtime/src/engine.rs` (extract Modulated path into a new method)
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (FFI export `kalico_runtime_modulated_tick`)
- Modify: `src/stm32/runtime_tick_h7.c` and `_f4.c` (TIM5 ISR body calls the new FFI)

- [ ] **Step 1: Extract the polled-tick StepAccumulator loop from today's `tick_with_current` (engine.rs:1731-1773) into a new method**

```rust
impl Engine {
    /// Single TIM5 tick for Modulated motors. Called from TIM5 ISR.
    /// Iterates motors with mode == Modulated, samples their current
    /// curve, runs StepAccumulator, emits step pulses. Also checks
    /// whether any Modulated-consumed curve has crossed t_end and, if
    /// so, clears the curve's consumer bit and (if zero) retires the
    /// slot.
    pub fn runtime_modulated_tick(
        &mut self,
        now: u64,
        pool: &CurvePool,
        queue: &SegmentQueue,
        shared: &SharedState,
    ) {
        // (a) Pull/advance the wall-clock "currently playing segment for
        //     Modulated motors" cursor; this is independent of the
        //     producer cursor.
        // (b) For each motor with mode == Modulated: evaluate curve at
        //     u(t) = (now - t_start) / duration, call ss.update, emit.
        // (c) For each retiring-segment-this-tick: clear Modulated
        //     consumer bits, retire slots if consumers_remaining == 0.
        // ... ~80 lines, body modeled on today's engine.rs:1731-1773 +
        //     the segment-progress bookkeeping currently in tick_with_current.
    }
}
```

- [ ] **Step 2: FFI export**

```rust
// rust/kalico-c-api/src/runtime_ffi.rs:
#[unsafe(no_mangle)]
pub unsafe extern "C" fn kalico_runtime_modulated_tick(rt: *mut KalicoRuntime, raw_cyccnt: u32) {
    // Project to ISR-half engine. Compute `now` on the spot via
    // stats_send_time_high (same as runtime_handle_widened_now).
    // Call engine.runtime_modulated_tick(now, ...).
}
```

- [ ] **Step 3: H7 / F4 TIM5 ISR body**

```c
// src/stm32/runtime_tick_h7.c — TIM5_IRQHandler body:
void
TIM5_IRQHandler(void)
{
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR = ~TIM_SR_UIF;
        uint32_t raw = DWT->CYCCNT;  // or timer_read_time(), depending on widening source
        kalico_runtime_modulated_tick(runtime_handle, raw);
    }
}
```

Same for F4. The body shrinks dramatically — no widening dance, no force_idle check (handled foreground per T13).

- [ ] **Step 4: Test**

Run: `cargo test -p kalico-runtime --features std --test engine_curve_retirement`
Expected: the test from Task 5 step 2 covers Modulated retirement; it should still pass after the extraction.

- [ ] **Step 5: Build H7 and F4**

```bash
cp .config.h7.bak .config && cargo clean && make clean && make
cp .config.f446.test .config && cargo clean && make clean && make
```

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/engine.rs rust/kalico-c-api/src/runtime_ffi.rs \
        src/stm32/runtime_tick_h7.c src/stm32/runtime_tick_f4.c
git commit -m "feat(engine): runtime_modulated_tick is TIM5's only callback"
```

---

### Task 11: `runtime_force_idle` — foreground synchronous flush

**Files:**
- Create: `rust/runtime/src/engine_force_idle.rs`
- Modify: `rust/runtime/src/engine.rs` (delete the old ISR-side `force_idle` check at the top of `tick`)
- Modify: `rust/runtime/src/lib.rs` (add the new module)
- Modify: `rust/kalico-c-api/src/runtime_ffi.rs` (`kalico_runtime_force_idle` body from T6 step 1)
- Test: `rust/runtime/tests/engine_force_idle.rs` — review existing; new behavior is synchronous, not ISR-acknowledged.

- [ ] **Step 1: Replace test assertions for force-idle**

```rust
// rust/runtime/tests/engine_force_idle.rs — replace:
#[test]
fn force_idle_clears_queue_rings_and_producer_state() {
    let pool   = CurvePool::new();
    let shared = SharedState::new(/* args */);
    let mut engine = Engine::new(&pool, &shared, /* ... */);

    // Push a segment, run producer to populate rings.
    let seg = build_test_segment_cartesian_x(&pool, 0.0, 10.0, 0, 18_000_000);
    engine.push_segment(seg).unwrap();
    engine.producer_step();
    assert!(engine.step_ring(0).available() > 0);

    // Force idle.
    engine.runtime_force_idle();

    // Everything cleared.
    for m in 0..4 {
        assert_eq!(engine.step_ring(m).available(), 0);
        assert!(engine.producer_state(m).is_idle());
    }
    assert!(engine.segment_queue_is_empty());
}
```

- [ ] **Step 2: Implement `runtime_force_idle`**

```rust
// rust/runtime/src/engine_force_idle.rs
use crate::engine::Engine;
use crate::curve_pool::CurvePool;
use crate::state::SharedState;

impl Engine {
    /// Foreground synchronous flush. Caller must guarantee no concurrent
    /// `producer_step` or `runtime_modulated_tick` is in flight (this
    /// is the foreground equivalent of the old ISR force_idle flag —
    /// called from klippy's flush path).
    pub fn runtime_force_idle(&mut self) {
        // 1. Mark producer_pending false so kicks don't race.
        self.shared.producer_pending.store(false, Ordering::Release);
        // 2. Drain segment queue (caller's responsibility to provide the
        //    queue handle here — depends on Engine struct layout).
        // 3. Reset rings.
        for ring in &mut self.step_rings {
            ring.reset();
        }
        // 4. Clear producer states.
        for state in &mut self.producer_states {
            state.clear();
        }
        // 5. Reset per-motor segment cursors.
        for cur in &mut self.motor_curve_cursor {
            *cur = 0;
        }
        // 6. Retire any in-flight curve pool slots (release back to host).
        //    Walk every segment that was in the queue + the one in flight
        //    and call pool.confirm_retired on each handle.
    }
}
```

(Adjust signature based on actual Engine ownership of `shared` / queue — the code above is the conceptual shape.)

- [ ] **Step 3: Delete the ISR-side `force_idle` check in `Engine::tick`** — already gone if Task 5 step 3d was done.

- [ ] **Step 4: Wire klippy's flush path to call `kalico_runtime_force_idle`** via FFI

In `klippy/motion_toolhead.py` and/or `klippy/motion_bridge.py`, replace any path that previously set the `force_idle` atomic with a synchronous call into the bridge that calls `kalico_runtime_force_idle` over the wire. (The wire-level protocol may need a new command — depends on existing flush implementation. Verify via `grep`.)

- [ ] **Step 5: Run the test**

```
cargo test -p kalico-runtime --features std --test engine_force_idle
```
Expected: passes.

- [ ] **Step 6: Commit**

```bash
git add rust/runtime/src/engine_force_idle.rs rust/runtime/src/engine.rs \
        rust/runtime/src/lib.rs rust/runtime/tests/engine_force_idle.rs \
        rust/kalico-c-api/src/runtime_ffi.rs klippy/motion_toolhead.py
git commit -m "feat(engine): runtime_force_idle is a synchronous foreground flush"
```

---

### Task 12: Delete `step_schedule.rs` and dead diagnostic atomics

**Files:**
- Delete: `rust/runtime/src/step_schedule.rs`
- Modify: `rust/runtime/src/lib.rs` (remove `pub mod step_schedule;`)
- Modify: `rust/runtime/src/state.rs` (already partially done in T5 step 3e — confirm full deletion list)

- [ ] **Step 1: Confirm no remaining callers of `step_schedule` items**

Run:
```
git grep -n 'step_schedule\|StepSchedule\|ScheduleExitReason\|start_schedule_for_segment\|refill_schedule_chunk'
```
Expected: only matches in `rust/runtime/src/step_schedule.rs` itself.

- [ ] **Step 2: Delete the file**

```
git rm rust/runtime/src/step_schedule.rs
```

Remove `pub mod step_schedule;` from `lib.rs`.

- [ ] **Step 3: Confirm SharedState diagnostic atomics are gone**

Run:
```
git grep -n 'first_tick_segment_id\|first_tick_delta_steps\|step_gen_activations\|boundary_loop_skipped_segments\|catch_up_nonzero_emits\|catch_up_total_pulses\|max_boundary_lateness_cycles\|peek_seq_odd_count\|peek_torn_count\|peek_cursor_at_total_count\|peek_ok_count\|peek_last_count_m0\|peek_last_count_m1'
```
Expected: zero matches (all deleted via T5 step 3e).

- [ ] **Step 4: Run full Rust test suite to confirm nothing broke**

```
cargo test -p kalico-runtime --features std
cargo test -p nurbs
cargo test -p kalico-c-api
```
Expected: all green.

- [ ] **Step 5: Commit**

```bash
git add -u rust/runtime/src/lib.rs
git commit -m "chore(runtime): delete step_schedule.rs and old diagnostic atomics"
```

---

### Task 13: Bench validation — H7 jog smoke test

**Files:** none (manual bench task).

The user runs G-code; the implementer reads logs and confirms acceptance criteria from spec §9.

- [ ] **Step 1: Build H7 fresh and prepare flash**

```bash
cp .config.h7.bak .config
cargo clean && make clean && make
```

Inform user: "Ready to flash H7. Please flash to `dderg@trident.local` per the memory `reference_flash_h7.md` procedure."

- [ ] **Step 2: User flashes H7 and brings up klippy.**

Wait for user confirmation.

- [ ] **Step 3: Ask user to run jog test**

Tell user: "When ready, please run the following from the Mainsail/Fluidd console — one at a time — and report the audible/visual result for each:

1. `G28` — home
2. `G1 X10 F600` — slow 10 mm X jog
3. `G1 X0 F600` — return
4. `G1 X100 F6000` — long fast jog (tests no `SCHEDULE_OVERFLOW`)
5. `G1 X0 F6000` — return
6. Sustained-load test: `G1 X10 F6000` followed by `G1 X0 F6000` × 10 (script if convenient)

For each, I want to know: did the toolhead actually move the commanded distance? Was the motion audibly smooth (no clicks)?"

- [ ] **Step 4: Read klippy log via the `reading-klippy-log` skill**

After the user runs the test, dispatch the `reading-klippy-log` skill to fetch the relevant log slice. Look for:
- `consumer_pulses_total[0]` and `[1]` advancing by ~1600 per `G1 X10`.
- Zero `consumer_underrun_total`.
- Zero `KALICO_ERR_*` fault codes.
- `producer_runs_total` advancing on each push (not on a heartbeat).

- [ ] **Step 5: Verify TIM5 stayed off (all StepTime config)**

Add a one-shot diagnostic print to the firmware (or inspect via the `prior_diag` BKPSRAM dump) confirming `TIM5->CR1 & TIM_CR1_CEN == 0` after the test. Document.

- [ ] **Step 6: Commit any post-bench adjustments**

If the test passes cleanly: no commits needed.
If the test fails: STOP. Read the relevant systematic-debugging skill, root-cause the failure, return to the appropriate earlier task. Do NOT add band-aid commits.

---

## Self-review checklist (applied to this plan)

**Spec coverage:**
- §3.1 (two emission paths, disjoint) → T5 (`producer_step` for StepTime; T10 `runtime_modulated_tick` for Modulated), T9 (TIM5 gating).
- §3.2 (TIM5 lifecycle) → T9.
- §3.3 (StepRing) → T1.
- §3.4 (producer) → T4, T8.
- §3.5 (consumer) → T7.
- §3.6 (compute_next_step_time fix) → T2, T3.
- §3.7 (endstops) → preserved (T7 step_time_event keeps `runtime_endstop_sample_one`).
- §3.8 (curve retirement) → T5, T10.
- §3.9 (clock sync) → T6 (`runtime_handle_widened_now` body change).
- §3.10 (force_idle) → T11.
- §4.2 (delete list) → T5 step 3d/3e, T12.
- §5 (worked example) → T13 (bench validation).
- §6 (replacement diagnostics) → T5 step 3e (`producer_runs_total`, `consumer_pulses_total`, `consumer_underrun_total`, `ring_high_water`).
- §9 (acceptance criteria) → T13.

**Placeholder scan:** Task 5 step 3c and 3d describe extensive Engine surgery in narrative form rather than full code. This is intentional — the actual edit is several hundred lines and the abstract shape is what the implementer needs to know. The plan flags each deleted item by name (no "remove other dead code" hand-waves). Each test file lists explicit assertions.

**Type consistency:** `producer_step` returns `ProducerTickResult` (Task 4). T5 step 3c calls it and dispatches on `WorkPending` / `AllIdle` — matches. `runtime_force_idle` signature stays `fn(&mut self)` consistently across T6 and T11. `StepRing::push` and `peek_head` signatures consistent across T1 and the consumer in T7.

---

## Execution handoff

Plan complete and saved to `docs/superpowers/plans/2026-05-14-step-emission-architecture.md`.

Two execution options:

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration. Each subagent gets the spec, the plan, and a target task; returns when its task is committed and tests pass.

**2. Inline Execution** — Execute tasks in this session using `superpowers:executing-plans`, batch execution with checkpoints for review.

Which approach?
