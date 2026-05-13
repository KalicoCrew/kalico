# Step emission architecture: append-only ring + event-driven producer

**Status:** Design proposal, awaiting review.
**Author:** Brainstorm 2026-05-14 between Danila Dergachev and Claude (Opus 4.7).
**Implements:** Step 7-D unblocker. Replaces the per-segment step schedule design from `2026-05-12-step-time-scheduling-design.md`. Keeps that design's `StepMode` enum and config surface intact; rewrites the StepTime emission path beneath them.

## 1. Problem

After three days of bench iteration on the step-time scheduling design from `2026-05-12-step-time-scheduling-design.md`, jogs reproducibly fail with **audible step pulses but no toolhead motion** ("clicks instead of jogging"). Each commanded G1 produces a handful of step pulses then stops. Modulation-mode (`step_modes[i] = Modulated`, TIM5 polled-tick StepAccumulator) was the last known-good emission path — commit `7de488433`, before any of the step-time scheduling work landed.

The earlier design tried to fix this with a sequence of patches — single-step-per-ISR rate caps, MAX_STEPS_PER_TICK 65536, seqlock retry, per-segment "re-seed accumulator", boundary-loop catch-up emit, refill-disabled — each addressing a symptom of the same architectural defect.

## 2. Root cause

The 2026-05-12 design conflated two notions:

1. **Which curve is currently playing in wall-clock time.** Needed for the (future Step 10) phase-stepping current synthesis path, which samples `position(t)` at the modulation rate.
2. **Which step pulses are owed to the motor.** A causal stream of `(time, direction)` entries that, once decided, must fire on the wire regardless of where the engine's wall-clock state machine has progressed to.

Both notions were collapsed into a single per-motor `StepSchedule` keyed off the engine's "current segment." Consequence: when the engine's TIM5 ISR ran late (preempted by foreground, USB, or SysTick), its boundary loop retired the late segment **and overwrote the schedule** before the per-stepper consumer could drain the step pulses owed for it. The motor was commanded to emit a small fraction of the planned steps, then jumped to the next segment's first step — the planned motion between those points was silently discarded. Audibly: a few clicks per segment, barely any net motion.

The structural fix is to **separate the two notions in code**. Step pulses owed to the motor live in an **append-only per-motor ring** decoupled from any "engine current segment" concept. Whether the engine has retired a segment is independent of whether the consumer has fired the step pulses the engine derived from that segment.

This is also what stepcompress does in mainline Klipper, for the same reason.

## 3. Design overview

### 3.1 Two emission paths, cleanly disjoint

| Mode | Driver | Step pulses out | TIM5 involvement |
|---|---|---|---|
| `Modulated` (today: polled-tick StepAccumulator; future Step 10: phase current synthesis) | TIM5 ISR at modulation rate (40 kHz on H7, 10 kHz on F4) | yes (today, via `StepAccumulator::update`); future: no, just phase currents | yes |
| `StepTime` | Producer Klipper timer fills per-motor ring; per-stepper consumer Klipper timer fires pulses at ring entry times | yes | none |

A motor is in exactly one mode at a time. The two paths share only:

- The segment queue (one queue, X/Y/Z/E curve handles per segment).
- The curve pool (shared storage; per-motor accessors).
- The retirement criterion per curve (§5.3): a curve's pool slot returns to the host only when **all** motors that consume it are done with it.

Nothing else.

### 3.2 TIM5 lifecycle

- TIM5 is enabled iff `count_modulated_steppers() > 0` on this MCU. If zero, TIM5 is not started — no ISR fires, no engine state machine ticks, no widened-now seqlock is published.
- When enabled, TIM5's callback does **only** Modulated-axis work: poll position from each Modulated motor's currently-playing curve, call `StepAccumulator::update`, emit step pulses for that motor. Future Step 10 replaces "emit step pulses" with phase-current synthesis; the surrounding scaffolding is the same.
- TIM5 **does not**: dequeue segments, manage StepTime motor schedules, publish widened clock for clock-sync, retire curves consumed by StepTime motors, run a boundary loop.

For the MVP (all axes StepTime, the bench setup), TIM5 stays disabled on both H7 and F4. The engine's `tick` function as it exists today goes away.

### 3.3 StepTime path: append-only ring

Per StepTime motor:

```rust
struct StepRing {
    cycles_abs_lo: [u32; N],   // when to fire — low 32 bits of MCU clock
    dirs:           [i8;  N],   // +1 or -1, fixed at the time the entry was produced
    head:           AtomicU32,  // producer monotonic counter
    cursor:         AtomicU32,  // consumer monotonic counter
}

// SPSC ring invariants: head - cursor < N. Indices use modulo N.
```

- `head` advances only by the producer.
- `cursor` advances only by the consumer.
- **Neither ever decreases. Neither resets on segment retire.** A ring entry, once committed, is guaranteed to fire on the wire — its time is fixed and the consumer reads it independent of any "current segment" state.

Plain SPSC. No seqlock. The reader reads `cursor` Relaxed (its own counter), reads `head` Acquire, reads `cycles_abs_lo[cursor % N]` and `dirs[cursor % N]`, fires. The writer reads `cursor` Acquire to check space, writes the slots, then publishes via `head` Release.

`N` is sized per MCU's available RAM. Working assumption: `N = 1024` per motor (same as today). At 4 motors × 1024 entries × 5 bytes = 20 KB. Fits axi_ram on H7; fits BSS on F4 (Z-only deployment uses one ring).

### 3.4 StepTime path: producer

One shared Klipper `struct timer` for all StepTime motors. Body:

```rust
fn producer_step() {
    for motor in step_time_motors() {
        let ring = &mut rings[motor];
        let mut state = &mut producer_state[motor];
        let mut filled = 0;
        while ring.has_space() && filled < REFILL_BATCH {
            if state.current_curve.is_none() {
                let Some(next) = pull_next_curve_for(motor, &curve_queue) else {
                    break;  // out of work for this motor
                };
                state.start(next);
            }
            match compute_next_step_time(&state.into_query()) {
                NextAt { t, dir } => {
                    ring.push(state.curve_t_start + (t * state.curve_duration) as u64
                                                                              as u32,
                              dir);
                    state.advance(t);
                    filled += 1;
                }
                SegmentExhausted => {
                    notify_curve_done(motor, state.current_curve.unwrap());
                    state.clear_current();
                }
            }
        }
    }
    maybe_self_reschedule();
}
```

Wake sources:

1. **`push_segment` kick.** Foreground accepts a segment and schedules the producer at `now`.
2. **Consumer low-water kick.** When a consumer's `available = head - cursor` falls below `LOW_WATER` (say `N/4 = 256`), it schedules the producer at `now`.
3. **Self-reschedule** on `REFILL_BATCH` cap (more work remains, but we bounded ISR duration).

The producer **does not** wake on a heartbeat. If neither (1) nor (2) ever fire, the system is genuinely idle — the producer has no work to do.

Kick dedupe: a single `producer_pending: AtomicBool`. Kickers `compare_exchange(false, true)`; if they win, `sched_add_timer`. Producer at start CAS-clears.

`REFILL_BATCH` sizing: current estimate ≤ 32 entries per call. At Newton cost ~30 cycles/step on H7, that's ~960 cycles ≈ 1.8 µs per motor; 4 motors = 7.4 µs per producer call. To be revisited with measurement.

### 3.5 StepTime path: consumer

Per-stepper Klipper `struct timer`, callback:

```c
static uint_fast8_t step_time_event(struct timer *t) {
    struct step_timer_ctx *ctx = container_of(t, struct step_timer_ctx, timer);
    uint8_t motor = ctx->stepper_idx;

    uint32_t available, head;
    if (ring_peek(motor, &available, &head) == 0 || available == 0) {
        // No entries; back off and wait for a producer kick to land.
        t->waketime += POLL_INTERVAL_CYCLES;
        return SF_RESCHEDULE;
    }

    uint32_t t_next;
    int8_t dir;
    ring_read_head(motor, &t_next, &dir);
    uint32_t now = timer_read_time();
    if ((int32_t)(t_next - now) > 0) {
        t->waketime = t_next;
        return SF_RESCHEDULE;
    }

    runtime_emit_step_pulses(motor, dir >= 0 ? 1 : -1);
    ring_advance(motor, 1);
    runtime_endstop_sample_one(motor);

    // Kick producer if we've drained below low-water.
    if (ring_available_after_advance(motor) < LOW_WATER)
        kick_producer();

    // Reschedule for next entry, or back off if none.
    if (ring_peek_next(motor, &t_next))
        t->waketime = t_next;
    else
        t->waketime = now + POLL_INTERVAL_CYCLES;
    return SF_RESCHEDULE;
}
```

Notes:

- No `MAX_STEP_BURST`. No 50 kHz rate cap. No multi-step batch loop. One pulse per fire.
- No catch-up rate limiter. Producer-computed step times are physical step times — if two consecutive entries are 20 µs apart, that's the planner's intent and the consumer fires them 20 µs apart. The planner-side limits (max velocity, max step rate) enforce the physical constraint, not the consumer.
- `POLL_INTERVAL_CYCLES` corresponds to ~100 µs at the MCU clock rate. Only used when the ring is empty AND the producer hasn't yet kicked us to a real next entry. With (1) `push_segment` kick reaching the producer immediately and (2) producer self-rescheduling on batch cap, the empty-ring poll path is rare and short-lived.

### 3.6 `compute_next_step_time` — degenerate-velocity fix

Today the function bails as `SegmentExhausted` whenever `|v(t_curr)| < EPS_VELOCITY`. This is wrong at the **start** of an accel-from-rest segment, where `v(0) = 0` exactly. Fix:

```rust
pub fn compute_next_step_time<F>(q: &StepTimeQuery<F>) -> StepTimeResult
where F: Fn(f32) -> (f64, f64, f64) {  // (pos, vel, accel) — see below
    let (pos0, v0, a0) = (q.eval)(q.t_curr as f32);

    let dir_i8 = derive_direction(v0, a0, /* jerk fallback if needed */);
    if dir_i8 == 0 {
        return SegmentExhausted;
    }
    let dir = f64::from(dir_i8);
    let target = (f64::from(q.current_step) + dir) * q.step_distance;

    // Initial guess: prefer linear extrapolation when v is non-degenerate;
    // fall back to quadratic (from accel) when v ≈ 0; cubic (from jerk)
    // when both are degenerate.
    let mut dt = if v0.abs() >= EPS_VELOCITY {
        q.step_distance / v0.abs()
    } else if a0.abs() >= EPS_ACCEL {
        (2.0 * q.step_distance / a0.abs()).sqrt()
    } else {
        // Sample one more derivative or scan-forward; see implementation note.
        scan_forward_initial_dt(q)
    };

    // Newton refinement as today, but the bail condition becomes
    // "t_try outside [t_curr, t_end] for MAX_ITERS iterations" — never
    // "instantaneous velocity dipped below eps mid-iteration." A velocity
    // sign flip mid-segment is a planner invariant violation; for now
    // surface it as a Newton non-convergence (currently silent
    // SegmentExhausted; we can promote to a fault in a follow-up).
    // ...
}
```

The `eval` closure changes signature to return `(pos, vel, accel)` instead of `(pos, vel)`. For cubic Bézier (our universal curve representation) the accel is the second derivative — cheap to extract from the same de Boor walk.

If both `v0` and `a0` are below their thresholds, the curve is effectively a (j/6)t³ jerk-only ramp at `t_curr`. We can either:

- (a) compute the third derivative `j0` and seed `dt = (6·step_distance / |j0|)^(1/3)`, or
- (b) scan forward a small step (`t_curr += duration · 1e-3`) and re-evaluate.

(a) is analytic and cheap; (b) is robust to extra-degenerate cases. The implementation can do (a) and treat persistent `j0 ≈ 0` as `SegmentExhausted` (the curve is truly motionless at this point).

### 3.7 Endstops

Already sampled per-step in the consumer (`runtime_endstop_sample_one(motor)`). No change. Sample granularity is the natural step rate — finer than TIM5 was giving us.

### 3.8 Curve pool retirement

A curve is retired (slot returned to host via `kalico_credit_freed`) when **every motor that consumes it** is done with it:

- **Modulated consumer (TIM5)**: done when wall-clock crosses the curve's `t_end`. TIM5 ISR detects this on the tick at which `now ≥ t_end` and calls `pool.confirm_retired(handle)`.
- **StepTime consumer (producer)**: done when Newton returns `SegmentExhausted` (the curve has produced all its step times, which are already in the ring). The producer calls `pool.confirm_retired(handle)` at that moment.

A curve consumed by both kinds of motors (mixed-mode CoreXY would be unusual but possible) retires when **both** criteria are met. Each curve carries a small "consumers remaining" counter (one bit per consuming motor, decremented as each finishes). When zero, retirement fires.

The host-facing protocol (`kalico_credit_freed` event with `retired_through_segment_id`) is unchanged. Retirement happens earlier in wall-clock terms than today (the producer often finishes Newton ahead of wall-clock), which is strictly better for slot economy.

### 3.9 Clock sync

Today the engine's TIM5 ISR publishes widened-now into a seqlock-protected `SharedState` field, read by `clock_sync_respond`. With TIM5 potentially off (MVP), this needs to move.

Replacement: `runtime_handle_widened_now` (foreground accessor) reads `timer_read_time()` and `stats_send_time + stats_send_time_high` on the spot and returns the freshly computed widened value:

```c
uint64_t runtime_handle_widened_now(KalicoRuntime *rt) {
    uint32_t low = timer_read_time();
    uint32_t high = stats_send_time_high + (low < stats_send_time);
    return ((uint64_t)high << 32) | low;
}
```

Same widening identity Klipper uses for its own `command_get_uptime`. No seqlock needed because there's no concurrent writer.

### 3.10 `force_idle` / homing flush

Today: `force_idle` is an atomic flag checked at the top of every TIM5 tick. With TIM5 off, the flag has no consumer.

Replacement: foreground call `runtime_force_idle()` that synchronously:

1. Sets `producer_pending = false` so no in-flight kicks land mid-flush.
2. Clears the curve queue.
3. For each StepTime motor: clears the producer state (forgets the in-flight curve) and resets `head = cursor` (drops any pending unfired step times).
4. For each Modulated motor: clears its StepAccumulator's segment-bound state.
5. Calls `pool.confirm_retired` on all in-flight curves (releases slots back to the host).

This is called from the foreground homing/abort path. Synchronous → host knows the engine is quiescent when the call returns. No TIM5 needed.

## 4. Concrete file/module changes

### 4.1 New / renamed

- `rust/runtime/src/step_ring.rs` (new) — `StepRing`, producer state, kick atomic. Subsumes today's `step_schedule.rs`.
- `rust/runtime/src/step_producer.rs` (new) — `producer_step()` function called from the producer Klipper timer's callback. Contains the curve-pulling logic and Newton fill loop.

### 4.2 Deleted

- `rust/runtime/src/step_schedule.rs` — replaced by `step_ring.rs`. Functions removed: `start_schedule_for_segment`, `refill_schedule_chunk`. `StepSchedule`, `ScheduleExitReason`.
- `Engine::tick` and `Engine::tick_with_current` — gutted. The state machine is gone. Replaced by:
  - `runtime_modulated_tick` (in `engine.rs` or a new `engine_modulated.rs`) — called from TIM5 ISR, runs only when at least one motor is Modulated. Contains the polled-tick StepAccumulator loop (today's `engine.rs:1731-1773`) plus per-Modulated-motor segment activation/retirement bookkeeping.
  - `producer_step` — called from the producer timer, runs always.
- `Engine::precompute_step_schedules`, `Engine::refill_step_schedules`, `Engine::arm_step_timer` — removed.
- `Engine::current` / `Engine::tick_counter` / `Engine::WidenState` / `publish_widened_now` — the per-motor producer state and per-Modulated-motor "currently playing" state replace `Engine::current`. Widening moves to `runtime_handle_widened_now` as in §3.9.
- All `schedule_seq` seqlock infrastructure on the engine side.
- `SharedState::first_tick_segment_id`, `first_tick_delta_steps`, `step_gen_activations`, `boundary_loop_skipped_segments`, `catch_up_nonzero_emits`, `catch_up_total_pulses`, `max_boundary_lateness_cycles`, `peek_seq_odd_count`, `peek_torn_count`, `peek_cursor_at_total_count`, `peek_ok_count`, `peek_last_count_m0`, `peek_last_count_m1` — diagnostics for an architecture that's gone. New diagnostics (much smaller set) in §6.
- `kalico_runtime_step_schedule_peek`, `kalico_runtime_step_schedule_advance` FFI — replaced by `kalico_runtime_step_ring_pop` (atomic-read head + cursor; the consumer drains in C, the FFI is read-only).
- `kalico_runtime_arm_step_timer` FFI — no caller in the new design (the producer doesn't need an "arm" entry point; it's just a timer callback that runs).
- `src/runtime_tick.c::arm_step_time_steppers_after_push` — replaced by a one-shot init that adds the per-stepper consumer timers and the producer timer to the Klipper scheduler. After init they reschedule themselves.
- `src/runtime_tick.c::step_time_event` rate-limit hack (the 2026-05-14 "1 step per ISR, 50 kHz cap" block, lines ~1199-1254) — gone, replaced by §3.5.
- `MAX_STEP_BURST` macro and surrounding rationale — gone.
- The TIM5 rate ping-pong in `src/stm32/runtime_tick_h7.c` and `_f4.c` (`count_modulated == 0 ? 1000U : 40000U` etc.) — replaced by "TIM5 enabled iff `count_modulated > 0`; rate is always 40 kHz (or per-MCU-target rate). If `count_modulated == 0`, the timer is never started."
- The 65 536-cap restoration commit's `MAX_STEPS_PER_TICK_DEFAULT` text — already restored to 16 in the working tree; that's correct as-is.

### 4.3 Touched but preserved

- `rust/runtime/src/step.rs` — `StepMotorState` / `StepAccumulator` untouched. The Modulated path keeps using it exactly as `7de488433` had it.
- `rust/runtime/src/step_time.rs` — signature change (`(f32) -> (f64, f64)` → `(f32) -> (f64, f64, f64)`) and the §3.6 robustness fix. Otherwise structurally the same.
- `rust/runtime/src/state.rs` — `step_modes` array stays; `SharedState` becomes much smaller; `IsrState` no longer owns the producer state (producer runs from a separate Klipper timer, owns its own state).
- `rust/runtime/src/segment.rs` — segment queue stays. Producer reads from this queue; Modulated TIM5 also reads from this queue. No change in segment representation.
- `klippy/motion_toolhead.py` config plumbing — `phase_stepping: 1` per stepper still flips `step_modes[i] = Modulated`. No change visible to the user.

## 5. Worked example: pure-X jog on the bench (MVP, all StepTime)

User issues `G1 X10 F600` from the printer's MMI. Bridge plans the move into ~3 segments (accel, cruise, decel), pushes them to the runtime via `push_segment`.

1. Foreground accepts segment 1 (accel). Calls `runtime_handle_push_segment(seg1)`. Pushes onto curve queue. **Kicks producer.**
2. Producer wakes. Pulls segment 1's X curve for motor 0, Y curve for motor 1 (both CoreXY). Starts Newton:
   - Motor 0: `eval = X+Y curve at u`. At `u=0`, X(0) = current X, Y(0) = current Y, `v0_motor0 = dx/du(0) + dy/du(0) = 0 + 0 = 0`. **Cold-start, but the §3.6 fix handles this**: read `a0` from the curve; cubic Bézier accel at u=0 is non-zero for an accel ramp. Newton seed `dt = sqrt(2·step_distance/|a0|)`. Iterate. NextAt returned. Producer pushes `(t_abs, +1)` to motor 0's ring.
   - Motor 1: same eval (X-Y instead of X+Y for CoreXY B). Same handling.
3. Newton continues until `SegmentExhausted` or `REFILL_BATCH` cap. Both motors get their accel step times appended to their rings. Curve slots for X and Y retire when both motors return `SegmentExhausted` on their respective Newton-fill of that curve.
4. Foreground accepts segments 2 and 3 (cruise, decel), kicks producer again. Producer naturally continues into them.
5. Per-stepper consumer (motor 0): timer fires at the first ring entry's time. Emits `+1` step pulse. Reschedules to ring entry 1's time. And so on.
6. Consumer drains 256-deep ring slowly behind producer — natural step rate is on the order of kHz, producer fills 32 entries per call in microseconds, so the ring always has 700-1000 entries of headroom.
7. At natural step rate, consumer fires evenly spaced pulses. **No clicks. No silent motion loss. Toolhead moves 10 mm.**

If foreground stalls for 50 ms mid-jog (USB pump preempts at SysTick priority): producer doesn't run for 50 ms. Ring drains by ~400 entries (assuming 8 kHz step rate). Ring still has ~600 entries — consumer keeps firing on time. When foreground unblocks, producer's next call fills the drained portion. No motion is lost.

If foreground stalls for 130 ms (longer than the ring at peak step rate): consumer drains the ring entirely. Ring empty → consumer goes to 100 µs poll cadence (no pulses fire). When producer next runs, it refills. Consumer's next poll sees entries, fires them. **Motion pauses for the duration of the stall; no pulses are lost — they fire late, in order, all of them.** That's the correct behavior under genuine overload, and represents the "ideally it shouldn't ever be" failure mode from CLAUDE.md's non-negotiable performance constraints.

## 6. Replacement diagnostics

Drop the existing pile (§4.2 list). Replace with a minimal, architecturally meaningful set:

- `producer_runs_total: AtomicU64` — bumped each time the producer timer's callback enters. Heartbeat indicator (host can plot rate).
- `producer_curves_completed_total: AtomicU64` — bumped each time Newton returns `SegmentExhausted` for a curve.
- `consumer_pulses_total[4]: [AtomicU64; 4]` — per-motor pulse count. Matches `stepper_counts` cumulatively.
- `consumer_underrun_total[4]: [AtomicU64; 4]` — bumped when the consumer fires while the ring is empty (the poll-cadence path runs and finds no work). High value = producer is falling behind for this motor.
- `ring_high_water[4]: [AtomicU32; 4]` — peak `available` ever observed in each ring. Indicates how close we are to ring-full backpressure.

Surfaced via the existing `runtime_status_drain` mechanism; format adjustments TBD with the host-side telemetry consumer.

## 7. Open questions

1. **Direction storage in the ring.** Plain `[i8; N]` parallel array (1 KB per motor at N=1024) vs. packed into the high bit of `cycles_abs_lo` (31-bit time, 1-bit direction; 4 s wrap at 520 MHz which is comfortably above any inter-step gap). I lean plain array for clarity; revisit if RAM tightens.

2. **Curve pull policy for the producer.** A segment carries up to 4 curves (X/Y/Z/E). When the producer is filling motor `i`'s ring, does it pull the next segment from the queue (consuming the segment for *all* motors at once) or peek at queued segments and pull only `i`'s curve? The latter is necessary if motors finish curves at different rates — e.g., E motor's curve has way more steps than X's, so E's producer state lags X's by several segments. **Recommendation**: per-motor cursor into the segment queue; the queue is read-only-via-cursor until all motor cursors have advanced past a segment, at which point the segment slot can be reused. Concrete data structure TBD in the implementation plan.

3. **`producer_pending` race with self-reschedule.** If the producer hits batch cap and self-reschedules at `now`, while simultaneously a consumer kick is in flight, the kick's CAS will see `pending = true` (the producer cleared it on entry, then either it or the kicker will set it again). Need to verify the CAS ordering is right so we don't lose a kick. Sketch: producer enters → CAS-clears `pending → true → false`. Producer body. At exit, if work remains → `sched_add_timer(now)` (self-reschedule, no flag involved). If no work → returns; flag stays clean. Concurrent kicker landing at any point CAS-sets the flag and `sched_add_timer`s if it won the CAS. Worst case = producer runs twice in close succession — benign.

4. **Klipper scheduler priority of the producer timer.** Klipper dispatches `struct timer` callbacks from the scheduler context. All struct timers (producer + 4 consumers + every other timer in Klipper) share the same dispatch. If the dispatch is starved by foreground, all timers are starved together — including the consumers. In that case the user-perceived behavior is "motion pauses, then resumes" (the underrun-then-recover path, §5). Acceptable, but we should measure foreground-induced timer starvation duration on the bench to confirm it stays below the ring's buffer depth at peak step rates.

5. **Mixed-mode CoreXY.** The §3.1 table assumes a motor is in one mode. On CoreXY, motors A and B both depend on the X and Y curves; they're independent motors with independent modes in principle. If a user sets A=Modulated but B=StepTime (no practical reason to today), the X and Y curves have one Modulated consumer and one StepTime consumer. Retirement (§3.8) handles this correctly. The §3.6 cold-start eval signature change (returning `(pos, vel, accel)`) also has to be threaded through TIM5's modulated path so it doesn't need separate eval logic. **Decision needed:** unify the eval API to always return `(pos, vel, accel)` (cost: trivial — the de Boor walk produces all three anyway), or keep two flavors. I lean unify.

6. **`step_distance` precision.** Today it's `f32` derived from `1.0 / steps_per_mm`. For motors at 160 spm that's `0.00625` exactly representable in f32; for 80 spm, `0.0125` exact. For exotic spm values (rotational, configured-by-user fractional) we may want `f64` to avoid step targets accumulating error over thousands of steps in a single segment. **Recommendation**: store `step_distance` as `f64` in the producer state and pass `f64` into `compute_next_step_time`. Trivial change.

## 8. Out of scope

- The actual Step 10 phase-stepping current synthesis kernel. The Modulated path stays as today's polled-tick StepAccumulator. Step 10 lands by replacing that loop's body without restructuring the producer/consumer/TIM5 split this design pins down.
- Skip detection / sensorless homing (Step 11) — orthogonal.
- Bridge-side changes — `push_segment` keeps its current contract.
- Compatibility-layer / G-code reduction (Step 13) — unaffected.

## 9. Acceptance criteria (for the eventual implementation plan)

This design is correct if, after implementation, on the H7+F4 bench with all axes StepTime:

1. `G1 X10 F600` produces 10 mm of toolhead motion, audibly smooth (no clicks), with `consumer_pulses_total[0]` and `[1]` each advancing by ~1600 (X+10 mm at 160 spm on CoreXY).
2. A `G1 X100 F6000` long-distance jog completes without `KALICO_ERR_SCHEDULE_OVERFLOW` (the error code itself goes away; ring is streaming).
3. Sustained 5-minute back-to-back jog pattern produces zero `consumer_underrun_total` events on any motor.
4. `producer_runs_total` advances on `push_segment` and on consumer low-water — not on a heartbeat.
5. `count_modulated_steppers() == 0` → TIM5 register `CR1.CEN` reads 0 throughout the test (TIM5 actually off, not just no-oping).
6. After flipping one motor to `Modulated` via `runtime_set_step_mode`: TIM5 enables; that motor's pulses come from the StepAccumulator path; the other motors continue on the producer/consumer path; no regression.
