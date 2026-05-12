# Step-time scheduling for non-phase-stepped axes

**Status:** Design approved, ready for implementation plan.
**Author:** Brainstorm 2026-05-12 between Danila Dergachev and Claude.
**Implements:** Build-order Step 7-D unblocker for F446 Z motion; sets the architectural foundation that future Step 10 (phase stepping) will slot into.

## 1. Problem

Running the kalico runtime on STM32F446 with a Z curve loaded saturates the chip at 100% CPU and trips the 511 ms IWDG within seconds of motion start.

Captured forensics (post-`.persistent_diag`-enablement on F4, 2026-05-12):

```
prior_diag_summary boot 2 tim5_n 15619 tim5_max_cyc 10420 tim5_total_lo 142106488
prior_diag_summary_eval n 15619 max 5274 total_lo 67222989
prior_diag_summary_curve x_deg 0 ... z_deg 3 z_cps 64 z_knots 68
prior_diag_tasks out_n 1362 out_max_gap 191022599 in_n 4196 in_max_gap 126225422
prior_diag_drops klipper 65 ring_overflow 2970
```

Decoded on the F446 (180 MHz):
- TIM5 average per fire: 50.5 µs (`142106488 / 15619 / 180`)
- TIM5 worst-case per fire: 57.9 µs (`10420 / 180`)
- TIM5 tick interval at 40 kHz: 25 µs

Each TIM5 IRQ already takes more than 2× its own tick interval. TIM5 tail-chains continuously, foreground starves for 1.06 s (`out_max_gap 191022599 / 180e6`), IWDG fires.

The H7 (520 MHz) handles the same engine workload at ~2.6 % TIM5 CPU. The F446 at 1/3 the clock cannot.

## 2. Why not just lower the F4 tick rate

A one-line band-aid (`TIM5->ARR = (runtime_clock_freq / 10000U) - 1U`) would unblock the bench. But it would also:

- Calcify the assumption that every regular-stepped axis must use modulation polling.
- Leave step pulses with up-to-one-tick jitter (100 µs at 10 kHz) when we can have ~zero.
- Block the cleaner future architecture in which any non-phase-stepped axis (including Z on H7) can use event-driven scheduling — needed eventually for sensorless homing on phase-stepped axes (see §10).

The project's "no throwaway code beyond 1-2 lines" preference plus the "no queue-based offload" constraint from CLAUDE.md both point toward fixing this properly.

## 3. Design overview

Every stepper has a `StepMode`:

| StepMode | Driven by | Step pulse jitter | Use case |
|----------|-----------|-------------------|----------|
| `Modulated` | TIM5 ISR (per-MCU 40 kHz tick) | Up to one tick interval | Phase-stepped axes (today: nothing actually phase-steps yet; this is current "polled-tick + StepAccumulator" behavior. Future: grows to include sin/cos commutation per Step 10.) |
| `StepTime` | Per-stepper Klipper `struct timer` rearmed by the engine | ~NVIC interrupt latency (sub-µs) | Every non-phase-stepped axis (default) |

The two paths coexist on the same MCU. F4 has zero `Modulated` steppers, so TIM5 never enables on F4. H7 with `phase_stepping: 1` on X/Y and default Z: TIM5 runs for X/Y, Z gets a `struct timer`.

`StepMode` is per-stepper and **runtime-mutable** (§10).

## 4. Configuration

New per-stepper config key in `printer.cfg`:

```ini
[stepper_x]
phase_stepping: 1     # opt-in; default 0

[stepper_z]
# no phase_stepping → default StepTime
```

- Default: `0` → `StepMode::StepTime`.
- `1` → `StepMode::Modulated`. The MCU capability bitmap (`caps.capabilities & PHASE_STEPPING_BIT` per `rust/kalico-protocol/src/bootstrap.rs:55`) is a hard ceiling: if a stepper is on an MCU that doesn't advertise the bit, klippy rejects `phase_stepping: 1` at config time with a clear error.

The user-facing knob is named `phase_stepping` because that's the eventual feature the user is opting into. Today the path runs the polled-tick + StepAccumulator implementation we already have; future Step 10 work adds sin/cos commutation inside the same ISR without changing the config name.

## 5. Engine API

```rust
pub enum StepMode {
    Modulated,   // TIM5-driven polling — current behavior
    StepTime,    // event-driven step pulse — new
}

// Set at configure_axes time; flippable at runtime (§10).
pub fn runtime_set_step_mode(stepper_idx: u8, mode: StepMode) -> Result<(), Error>;

// Driven by the TIM5 ISR (caller restricts to Modulated steppers internally).
// Unchanged behavior — current code path.
pub fn runtime_handle_tick(now: u64);

// For StepTime steppers. Returns the MCU clock at which the next step pulse
// should fire, or None if the active segment has no more steps in the current
// direction. Computed via Newton on the cubic position polynomial.
pub fn runtime_compute_next_step_time(stepper_idx: u8, now: u64) -> Option<u64>;

// Called when a new segment loads or after a mode flip: gives the timer ISR
// its initial waketime. Equivalent to compute_next_step_time but anchored to
// "wherever the engine considers t=now for this stepper".
pub fn runtime_arm_step_timer(stepper_idx: u8) -> Option<u64>;
```

Two engine entry points (`tick` and `compute_next_step_time`) rather than one unified `next_event` — clearer caller intent, each entry point only needs the data its mode actually uses.

## 6. MCU integration

### 6.1 Configure-axes time

At `configure_axes_blob`:
- Each stepper has both a `struct timer` slot and a place in the Modulated polling loop allocated. Cost: ~32 bytes per stepper for the `struct timer`. Both exist in cold state so runtime mutation (§10) is a single flag flip.
- The initial `StepMode` is whatever the host requested in the configure blob.

### 6.2 Per-stepper StepTime timer

```c
static uint_fast8_t
step_time_event(struct timer *t)
{
    struct step_timer_ctx *ctx = container_of(t, struct step_timer_ctx, timer);

    // 1. Fire the step pulse (HIGH → LOW chain handled per the existing
    //    stepper.c pulse-width discipline; minimum pulse width applies).
    gpio_out_toggle_noirq(ctx->step_pin);

    // 2. Sample armed endstops for this stepper's axis (§7).
    runtime_endstop_sample_one(ctx->stepper_idx);

    // 3. Ask the engine for the next step time.
    uint64_t next;
    if (!runtime_compute_next_step_time(ctx->stepper_idx, t->waketime, &next)) {
        return SF_DONE;  // segment exhausted; re-armed on next push_segment
    }
    t->waketime = (uint32_t)next;  // klipper timer is u32 cycles
    return SF_RESCHEDULE;
}
```

The Klipper scheduler's `struct timer` is FIFO-of-by-waketime; `sched_add_timer` inserts in waketime order. The existing `runtime_drain_timer` in `src/runtime_tick.c:217` follows this exact pattern.

### 6.3 TIM5 lifecycle change

Today: `runtime_tick_enable()` (in `runtime_tick_h7.c` / `runtime_tick_f4.c`) fires unconditionally on first segment push.

New: TIM5 only enables if at least one stepper on this MCU has `StepMode::Modulated`. On F4 (zero `Modulated` steppers via the capability check), the body of `runtime_tick_enable()` becomes a no-op. TIM5 IRQ never fires on F4. Foreground starvation impossible.

Edge case: a runtime mode flip (§10) from the only-`Modulated`-stepper-on-this-MCU to `StepTime`. The TIM5 ISR can be left running with no work — it's a per-tick CPU cost we pay until config reload — or we can `runtime_tick_disable()` when the count of `Modulated` steppers drops to zero. We do the latter; symmetric with the enable path.

## 7. Endstop sampling

Currently `runtime_endstop_sample_pins()` is called inside the TIM5 ISR (every 25 µs at 40 kHz). The F4 with no `Modulated` steppers has no TIM5 → no sampling.

New behavior: each stepper's step-time ISR samples its own axis's armed endstops. Frequency = step rate (≥1 kHz during any motion). No motion → no need to sample (nothing can change). Klipper's existing endstop ARM/disarm protocol unchanged.

For `Modulated` MCUs, the existing TIM5-side sampling continues — no regression on H7-with-phase-X/Y.

Plain English: instead of a constant 40 kHz heartbeat watching the endstop pin, the check rides along with each step pulse — naturally fast enough during motion (when crashes happen), naturally idle when nothing's moving.

## 8. Step-time computation

The curve is `position(t)` — a cubic Bezier polynomial in `t` (in MCU clock cycles). `velocity(t) = position'(t)` is a quadratic. We solve for the smallest `dt > 0` such that
`position(t_curr + dt) = (current_step + sign(velocity)) · step_distance`.

That's a cubic root-find. Method: Newton-Raphson from a velocity-based initial guess.

```
target = (current_step + sign(v(t_curr))) · step_distance
Δt₀ = step_distance / |v(t_curr)|              # constant-velocity guess
for i in 0..3:
    Δt_{i+1} = Δt_i - (position(t_curr + Δt_i) - target) / velocity(t_curr + Δt_i)
    if |position(t_curr + Δt_{i+1}) - target| < step_distance · 1e-6:
        break
return t_curr + Δt_final
```

- Cost: ~150-200 cycles on F446 (3× cubic eval + 3× quadratic eval + 3 divisions). Compare to current per-tick eval cost of ~4300 cycles. The new path is ~25× cheaper per step than the old path is per tick.
- Convergence: quadratic; 2 iterations to FP precision in the well-conditioned case.
- Cardano closed-form was considered and rejected: `cbrt` + `arccos` cost ~1500-3000 cycles on Cortex-M4 with software float, plus discriminant-branch corner cases.

### Edge case: segment exhaustion

If `sign(v(t_curr)) · (position(segment_end) - position(t_curr)) < step_distance`, the segment can't produce another step in the current direction of motion before it ends. Return `None`. The next `runtime_arm_step_timer` call (on the next pushed segment) re-arms.

That's the only edge case the implementation needs. Velocity-zero crossings naturally fall under segment exhaustion (the curve at zero-velocity points produces no steps over any practical interval); no bisection fallback needed.

## 9. Segment lifecycle for StepTime steppers

1. **First segment after `configure_axes`:** host pushes segment → `runtime_handle_push_segment` calls `runtime_arm_step_timer(stepper_idx)` for each `StepTime` stepper that has a curve on this axis. Returns the first step time. `sched_add_timer()` registers the timer at that waketime.

2. **Mid-segment update:** new segment loads into a different curve-pool slot while the active one keeps running. Engine atomically advances slots on `segment_id` retirement (existing mechanism, unchanged). `compute_next_step_time` reads "whichever slot the engine considers current" — same primitives as the existing `Modulated` path uses.

3. **Segment exhaustion:** timer returns `SF_DONE`. Re-arming happens on the next pushed segment.

Concurrency: same primitives as the existing engine code (no new locks; `StepAccumulator` already runs in the TIM5 ISR today).

## 10. Runtime mutability (justified by TMC StallGuard constraint)

`StepMode` is an `AtomicU8` per stepper, flippable at any time via `runtime_set_step_mode`.

### Why mutable

TMC drivers in direct/XDIRECT mode (where the host writes coil currents per tick — the eventual phase-stepping implementation) do **not** generate StallGuard signals. StallGuard requires the driver's internal step sequencer to be active, which the direct-mode register write bypasses.

**Confirmed** via:
- Prusa Buddy firmware production code:
  - `lib/Marlin/Marlin/src/feature/phase_stepping/phase_stepping.hpp` defines `EnsureSuitableForHoming` as a RAII guard that calls `StateRestorer(false)` — i.e., it disables phase stepping for the duration.
  - `lib/Marlin/Marlin/src/gcode/calibrate/G28.cpp:92` includes the header with the comment *"for disabling phase stepping during homing"*.
  - `lib/Marlin/Marlin/src/gcode/calibrate/G28.cpp:655` instantiates the guard at G28 entry.
- Local research: `docs/research/tmc5160-open-loop-phase-stepping.md` describes XDIRECT register (0x2D), GCONF.direct_mode (bit 16), and Prusa's REFRESH_FREQ = 40000 (the equivalent of our TIM5 rate).

So to sensorless-home an X/Y axis that normally phase-steps, we will need to (a) write the TMC register to switch the driver out of direct mode, and (b) flip the engine's `StepMode` for that stepper from `Modulated` to `StepTime` so the host can fire step pulses on edges StallGuard can hear. Both swaps for the duration of the homing move, then restore.

### What's in scope of *this* refactor

- `runtime_set_step_mode(stepper_idx, mode) -> Result` is implemented and tested with mid-segment flips.
- The capability ceiling (§4) applies: cannot flip a stepper on a non-phase-capable MCU into `Modulated`.

### What's out of scope

- klippy-side homing.py integration. No production caller flips at runtime in this PR.
- The TMC driver-side direct-mode register write. That's a TMC concern, not engine, and lands when phase stepping itself lands (Step 10).

## 11. Testing

### Unit (rust/runtime crate)

- `compute_next_step_time` returns correct `t_next` for a known cubic (synthetic curve, hand-verified answer).
- Convergence in ≤3 Newton iterations across a stress matrix of (v, a, j) values.
- `None` at segment exhaustion (forward direction, reverse direction, both).
- Direction flip mid-segment — verify next call returns the boundary in the new direction.
- Mode flip mid-segment via `runtime_set_step_mode`: starting in `Modulated`, flip to `StepTime`, verify the next `compute_next_step_time` is consistent with the engine's recorded position.

### Sim integration (Linux build, `CONFIG_KALICO_SIM=y`)

- F4-config equivalent: build with all steppers default → `StepMode::StepTime`. Push a Z curve. Assert step pulses fire at the computed times (within ±1 cycle of `compute_next_step_time`'s output).
- Run a 1000-step segment, verify total elapsed time matches integral of `|v(t)|` between segment endpoints to <1 % error.
- H7-config equivalent: build with `phase_stepping: 1` on X/Y, default Z. Verify TIM5 enables (X/Y), Z timer scheduling works alongside, no interference.

### Bench (Trident)

- **Pre-deploy baseline:** F4 prior_diag from current state (captured 2026-05-12).
- **Acceptance criterion 1:** F4 post-deploy prior_diag shows `tim5_n 0` (TIM5 never enabled) across 10 sequential Z jogs.
- **Acceptance criterion 2:** F4 `out_max_gap < 50 ms` across the same 10 jogs.
- **Acceptance criterion 3:** F4 IWDG never fires across a full homing cycle (G28) plus 5 Z hops.
- **Acceptance criterion 4:** H7 prior_diag unchanged (no regression on the Modulated path).

## 12. Files touched

| Area | File | Change |
|------|------|--------|
| Config parser | `klippy/extras/stepper.py` | Parse `phase_stepping: 1` per stepper section |
| Capability check | `klippy/motion_bridge.py` (or wherever `configure_axes_blob` is assembled) | Reject `phase_stepping: 1` if MCU caps lack `PHASE_STEPPING_BIT` |
| Engine state | `rust/runtime/src/state.rs` | Per-stepper `step_mode: AtomicU8` field |
| Engine API | `rust/runtime/src/lib.rs` + new `rust/runtime/src/step_time.rs` | `compute_next_step_time`, `arm_step_timer`, `set_step_mode` |
| C FFI | `rust/kalico-c-api/src/runtime_ffi.rs` | Export the three new functions |
| MCU runtime | `src/runtime_tick.c` | Per-stepper `struct timer` allocation; `step_time_event` ISR; register/cancel on segment load/exhaust |
| TIM5 lifecycle | `src/stm32/runtime_tick_h7.c`, `src/stm32/runtime_tick_f4.c` | `runtime_tick_enable` no-op when zero Modulated steppers; symmetric `runtime_tick_disable` on mode flip |
| Endstop | `src/runtime_endstop.c` (or wherever `runtime_endstop_sample_pins` lives) | Add `runtime_endstop_sample_one(stepper_idx)` for use from step-time ISR; existing TIM5 path stays |
| Tests | `rust/runtime/tests/step_time_*`, `rust/runtime/tests/sim_steptime_*` | New unit + sim tests per §11 |

`StepAccumulator` (`rust/runtime/src/step.rs`) is untouched — still used by `Modulated` and by any future flip back from `StepTime` to `Modulated`.

## 13. Out of scope (explicit YAGNI)

- Host-side step-time scheduling. Host stays curve-level; per-step compute is MCU-side.
- New wire protocol bits. `phase_stepping` is host-side config; the MCU just receives `StepMode` per stepper in the `configure_axes_blob` it already gets.
- Removal of `StepAccumulator`. Still needed for `Modulated` and for any future `StepTime → Modulated` flip.
- Reworking the endstop ARM/disarm protocol. We add a new sample site; we don't change the existing protocol.
- Cardano closed-form solution path. Newton is cheaper, simpler, sufficient.
- TMC direct-mode register write. TMC concern, lands with phase stepping (Step 10).
- klippy homing.py changes to call `runtime_set_step_mode`. Future Step 10 sub-task.

## 14. References

- F4 wedge forensics that motivated this work: klippy.log captures from 2026-05-12 bench session, decoded above in §1.
- `docs/research/tmc5160-open-loop-phase-stepping.md` — XDIRECT / direct_mode background.
- `docs/research/open-loop-phase-stepping-prior-art.md` — architecture comparison.
- Prusa Buddy firmware:
  - `lib/Marlin/Marlin/src/feature/phase_stepping/phase_stepping.hpp` — `EnsureSuitableForHoming` RAII guard.
  - `lib/Marlin/Marlin/src/gcode/calibrate/G28.cpp:92, 655` — production usage during G28.
- Kalico:
  - `rust/kalico-protocol/src/bootstrap.rs:55` — `phase_stepping=0x1` capability bit.
  - `rust/runtime/src/step.rs` — current `StepAccumulator` (unchanged by this refactor).
  - `src/runtime_tick.c:217-291` — `struct timer` + `sched_add_timer` template for `runtime_drain_timer`.
  - `src/stm32/runtime_tick_f4.c` — F4 TIM5 init (modified to conditional enable).
  - `src/generic/armcm_link.lds.S:82-88` — `.persistent_diag` section (already in place; not modified).
- CLAUDE.md constraints honored:
  - "Print throughput is non-negotiable" — step-time scheduling preserves trajectory; only changes how step pulses are timed, not what they are.
  - "Real time communication with MCUs, no queue-based offload" — step times are computed on-the-fly via Newton, not queued in advance.
  - "Phase stepping with open loop steppers with BTT Octopus pro and similar (H723 chip)" — Modulated path preserved, ready for Step 10 commutation.
  - "Regular stepping for non-phase-capable drivers (e.g. 2209 on Z)" — StepTime is the implementation of "regular stepping".
