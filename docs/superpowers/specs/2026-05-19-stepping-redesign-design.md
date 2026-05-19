# Stepping engine redesign — fixed-rate sampling with velocity-extrapolated step times

Status: design, awaiting implementation plan
Author: brainstorming session 2026-05-19

## Why this exists

The current StepTime (regular stepping) path runs a per-step Newton root solver
on cubic Bezier pieces, pre-computes step times into a 1024-entry per-motor
ring, and dispatches each step via a Klipper timer. Two weeks of bench testing
have produced crashes, audible clunking, brief snippets of motion, and at no
point continuous smooth motion. The path was never validated end-to-end.

The architecture has two structural problems:

1. **Per-step Newton solve is the wrong CPU budget.** At our architectural
   ceiling (320 kHz/motor × 4 motors = 1.28 MHz aggregate step events × ~200-500
   cycles per Newton solve = 48-123% of one H7 core), it cannot keep up.
   Realistic prints stay well below this rate, but the path has no headroom and
   gives back precision the motor cannot use.
2. **`SF_RESCHEDULE_FLOOR = 100 µs` hard-caps step rate at 10 kHz/motor**, and
   `EMPTY_POLL_CYCLES = 100 ms` introduces multi-hundred-millisecond stalls on
   ring underflow. The first means we cannot hit mainline-equivalent step
   rates; the second means a single missed producer fill produces stacked-late
   pulses (the "clunking").

Mainline Klipper, Prunt, and Marlin all use some form of locally-linear
approximation rather than exact per-step root finding. Mainline runs
`(interval, count, add)` packets walked inside a SysTick-dispatched ISR. Prunt
samples position at 20 kHz and lets the hardware track. Marlin uses integer
Bresenham. The exactness of our current path is wasted precision relative to
motor mechanical bandwidth (~hundreds of Hz). State-of-the-art means smoothest
perceived output, not bit-exactness against an analytic Bezier.

## What this redesign does

Replace the per-step Newton ring with a **fixed-rate sample-and-extrapolate
architecture**: TIM5 ISR fires at a configurable sample rate (40 kHz on H7,
20 kHz on F446), evaluates each axis's cubic Bezier curve once per sample via
monomial-form Horner, computes integer-step deltas, and queues velocity-
extrapolated sub-sample step times into a small per-axis SPSC queue. A per-axis
Klipper timer drains the queue and toggles GPIO via the existing
`runtime_emit_step_pulses` fan-out.

This unifies regular stepping and phase stepping into one architecture. The
only difference between them is the output stage: Pulse mode pushes to the step
queue, Phase mode pushes to an SPI write queue carrying TMC coil currents.

## Hard constraints

These cannot be relaxed without re-doing the design.

- **F446 stays first-class.** It must boot, sample at ≥ 20 kHz, and drive
  steppers without exceeding ~80% CPU budget.
- **No per-axis per-step compute.** Hot path math is per-sample-per-axis, not
  per-step.
- **One emission architecture for all motors.** Single TIM5 ISR, single math
  model, per-axis dispatch on a `StepMode` flag. No "legacy" path retained.
- **Velocity-extrapolated sub-sample step times.** Naive uniform distribution
  produces audible 40 kHz beat artifacts at constant velocity when step rate
  doesn't divide sample rate. Local-linear extrapolation eliminates them.
- **Mainline-aligned where possible.** Per-axis Klipper timers ride on
  SysTick + `sched.c`, identical pattern to mainline's `stepper_event_edge`,
  just per-axis instead of per-stepper.

## What it gives us

- ~3% CPU on H7 for all motion math at 40 kHz × 4 axes (vs current ~50%+ Newton
  load at architectural ceiling).
- No `SF_RESCHEDULE_FLOOR` step-rate cap; Klipper scheduler dispatch latency
  (~3-5 µs on H7) is the only floor.
- No producer underflow risk — TIM5 fires reliably at the sample rate, queues
  hold ≤ 2 samples of headroom.
- Unified phase stepping: the same TIM5 ISR drives both, with mode dispatch
  per axis.
- F446 sample rate of 20 kHz fits inside its FPU budget with comfortable
  headroom.
- Eliminates the entire StepTime/Modulated split. Newton solver, 1024-entry
  step ring, per-stepper Klipper timer, and the "permanent producer timer"
  race-avoidance comment block all get deleted.

---

## Architecture

### Glossary

- **Motor axis** (or just *axis*): A, B, Z, E. The four motion-output axes the
  engine evaluates per sample. On CoreXY, A = X+Y, B = X-Y combined in the
  curve-load path (`engine.rs:1854-1882`). Host-facing config still uses X/Y
  naming (matches mainline); spec uses A/B for the post-kinematics motor axes
  to avoid confusion.
- **Stepper**: a physical TMC driver / motor. One axis has 1-N steppers bound
  (e.g., Voron 2.4 X-axis: stepper_x + stepper_x1 → both bound to motor axis A).
  Up to 9 steppers total on H7.
- **Sample**: one TIM5 ISR fire. Duration = 1 / sample_rate_hz.
- **Piece**: one cubic Bezier curve segment in monomial form. A motion segment
  is composed of 1+ pieces per axis.

### Sample-rate config

Kconfig parameter `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ`.

Defaults:
- H7: 40000 (40 kHz, 25 µs sample period)
- F446: 20000 (20 kHz, 50 µs sample period)

Overridable per build via the standard Kconfig path. Validation at boot: must
produce a TIM5 period in (0, ARR_max) cycles; misconfiguration → boot-time
fault, no motion starts.

Sample rate determines velocity-extrapolation error bound: `½·a·dt² / v`. At
20 kHz with `a = 50000 mm/s²`, error per step is ~250 nm position, below
motor positioning resolution.

### TIM5 ISR — the unified evaluator

**Per-sample state caches.** Carried across ISR fires; the `*_this`
fields are populated within a single ISR pass for cross-axis use within
the same sample:

```rust
struct TickCaches {
    // Per-axis: position/velocity sampled at the end of the PREVIOUS
    // sample, which becomes "start" for THIS sample's velocity
    // extrapolation. Read at sample entry, overwritten at sample exit.
    P_prev: [f32; N_AXES],
    v_prev: [f32; N_AXES],

    // Cartesian XY arc-length velocity from end of PREVIOUS sample.
    // Persists across ISR fires.
    v_xy_prev: f32,

    // Accumulated Cartesian XY arc length since segment start. Reset on
    // segment retire; running sum across samples within a segment.
    ds_xy_segment: f32,

    // Computed once per sample after A and B are evaluated, BEFORE E
    // is evaluated. Lives only for one ISR pass (overwritten next fire).
    v_xy_this: f32,                  // |v_xy(t)| at this sample's end
    vdot_xy_accelerating: bool,      // sign of (v_xy_this - v_xy_prev)
}
```

```
TIM5 ISR fires every (1 / sample_rate) µs:

  # Phase 1: evaluate motion axes A, B, Z (NOT E yet — E needs XY-derived
  # quantities computed after A/B finish).
  P_end_axis = [0.0; N_AXES]; v_end_axis = [0.0; N_AXES]

  for axis in [A, B, Z]:
    P_sample_start = caches.P_prev[axis]
    v_sample_start = caches.v_prev[axis]

    # (1) Advance piece if sample straddles boundary
    while t_local_for_axis(t_sample_end) > axis.piece.duration:
      advance to next piece (or break if segment retiring)

    if axis.piece is None:
      P_end_axis[axis] = P_sample_start    # axis idle: position unchanged
      v_end_axis[axis] = 0
      continue

    # (2) Per-axis polynomial eval (one per axis per sample)
    (P_end, v_end) = monomial_horner_eval(axis.piece, t_local_for_axis)
    if !P_end.is_finite() || !v_end.is_finite():
      fault(MathNonFinite, axis); continue

    # (3) Endstop sample (existing hook, cheap when no arm)
    kalico_endstop_tick_step_time(handle, now)

    P_end_axis[axis] = P_end
    v_end_axis[axis] = v_end

    # (4) Per-axis dispatch on stepping mode — see "Per-axis dispatch" below
    dispatch_axis(axis, P_end, v_end, P_sample_start, v_sample_start)

  # Phase 2: XY-derived quantities (Cartesian arc length + acceleration sign).
  # See K_xy notes below: 1.0 cartesian, 1/sqrt(2) CoreXY.
  if axis A or axis B had an active piece this sample:
    v_motor_sq = v_end_axis[A]² + v_end_axis[B]²
    caches.v_xy_this = sqrt(v_motor_sq) · K_xy                # |v_xy(t)|
    caches.vdot_xy_accelerating = caches.v_xy_this >= caches.v_xy_prev
    caches.ds_xy_segment += caches.v_xy_this · sample_period  # Cartesian arc len
    caches.v_xy_prev = caches.v_xy_this                       # for next sample
  else:
    caches.v_xy_this = 0
    caches.vdot_xy_accelerating = false  # no motion; PA term zeroes anyway

  # Phase 3: evaluate E with full XY context.
  # CLAUDE.md formula:
  #   E_target = extrusion_per_xy_mm · ds_xy_segment
  #            + advance · ratio_per_xy_mm · |v_xy(t)|
  # where `advance` is K_accel or K_decel depending on sign(v̇_xy)
  # (asymmetric PA from bleeding-edge kalico Step 9). For the spec
  # `ratio_per_xy_mm == extrusion_per_xy_mm` (both are the XY-arc-length-to-
  # filament-length conversion).
  axis = E
  P_sample_start = caches.P_prev[axis]
  v_sample_start = caches.v_prev[axis]
  # ...piece advance + polynomial eval as in phase 1...
  (P_end_intrinsic, v_end) = monomial_horner_eval(axis.piece, t_local_for_axis)

  pa_K = if caches.vdot_xy_accelerating then advance_accel else advance_decel
  P_end = P_end_intrinsic
        + extrusion_per_xy_mm · caches.ds_xy_segment        # baseline follow
        + pa_K · extrusion_per_xy_mm · caches.v_xy_this      # PA, instantaneous

  P_end_axis[E] = P_end; v_end_axis[E] = v_end
  dispatch_axis(E, P_end, v_end, P_sample_start, v_sample_start)

  # Phase 4: per-sample bookkeeping
  for axis in [A, B, Z, E]:
    caches.P_prev[axis] = P_end_axis[axis]
    caches.v_prev[axis] = v_end_axis[axis]

  # Phase 5: segment retirement check
  if all participating axes' cursors have reached segment.duration:
    retire segment (host sync via existing retired_through_segment_id)
    caches.ds_xy_segment = 0      # reset XY arc length for next segment
    advance to next segment


# Per-axis dispatch subroutine — called from phase 1 (A,B,Z) and phase 3 (E)
fn dispatch_axis(axis, P_end, v_end, P_sample_start, v_sample_start):
  match axis.mode.load(Acquire):
    Pulse:
      prev_step_count = axis.last_step_count
      target_step_count = round(P_end / axis.microstep_distance)
      n_steps = target_step_count - prev_step_count
      axis.last_step_count = target_step_count

      if |n_steps| > 0:
        # Sub-sample step times via SECANT-SLOPE local-linear interpolation
        # through (0, P_sample_start) and (sample_period, P_end). This is
        # the canonical linear approximation of the cubic Bezier over this
        # sample. By construction, t_local ∈ [0, sample_period] for every
        # step_pos_k between P_sample_start and P_end — guaranteed in-sample.
        #
        # We do NOT use (v_sample_start + v_end) / 2 as the slope, even
        # though it's an obvious-looking estimate. That trapezoidal average
        # is only exact for linear v(t) (quadratic P); our cubic P has
        # quadratic v, where trapezoidal disagrees with the true time-average
        # by Simpson-rule curvature. The disagreement can push the formula's
        # t_k outside [0, sample_period], scheduling steps into next sample's
        # territory and double-counting at sample boundaries.
        #
        # cycle_abs is u32 = lower 32 bits of cycle counter. Wraps every
        # ~8.3 s on H7, ~4 min on F446. Use wrapping_add for absolute time
        # computation. Consumer side uses timer_is_before (signed-delta).
        displacement = P_end - P_sample_start
        if |displacement| > DISPLACEMENT_THRESHOLD:   # default 1 microstep
          for k in 0..|n_steps|:
            step_pos_k = (prev_step_count + (k+1) · sign(n_steps))
                         · axis.microstep_distance
            # Secant-slope linear interpolation:
            t_local_sec = (step_pos_k - P_sample_start) · sample_period
                          / displacement
            # debug_assert: t_local_sec ∈ [0, sample_period] by construction
            dt_cycles = (t_local_sec · cycles_per_second) as u32
            cycle_abs = sample_start_cycles.wrapping_add(dt_cycles)
            push (cycle_abs, sign(n_steps)) to step_queues[axis]
        else:
          # Sample produced n_steps but P barely changed (e.g., curve
          # reversing direction at v=0 — n_steps integer counts of a back-
          # and-forth that net to near zero). Fall back to uniform within
          # sample; consumer fires them spread evenly.
          for k in 0..|n_steps|:
            dt_cycles = (sample_period_cycles · (k+1)) / (|n_steps|+1)
            cycle_abs = sample_start_cycles.wrapping_add(dt_cycles)
            push (cycle_abs, sign(n_steps)) to step_queues[axis]

        for stepper in axis.steppers:
          stepper.position_count.checked_add(n_steps)
            .or_else(|| fault(PositionCountOverflow, stepper))

    Phase:
      # Per-stepper SPI dispatch (each TMC has its own CS).
      # TMC5160 electrical cycle = 1024 microsteps (10-bit MSCNT) =
      # 4 full steps. Coil-current LUT is 1024 entries spanning one
      # electrical cycle.
      target_microsteps_axis = round(P_end / axis.microstep_distance)
      axis.last_step_count = target_microsteps_axis    # kept in sync even in Phase

      # Per-stepper target = axis position + this stepper's phase offset.
      # Tracked per-stepper because phase_offset can change between samples
      # (motors-sync / Z tilt) and the change-vs-axis-motion distinction
      # matters for position_count accounting.
      for stepper in axis.steppers:
        target_stepper = target_microsteps_axis
                       + stepper.phase_offset_microsteps.load(Acquire)
        prev_stepper = stepper.last_phase_target.load(Acquire)
        delta_stepper = target_stepper - prev_stepper
        stepper.last_phase_target.store(target_stepper, Release)

        phase = target_stepper & 0x3FF                 # 10-bit, 1024 entries
        (coil_A, coil_B) = phase_lut[phase]
        spi_queue.push(stepper.tmc_cs, XDIRECT_REG, pack(coil_A, coil_B))
        stepper.last_coil_A.store(coil_A)
        stepper.last_coil_B.store(coil_B)

        # position_count tracks the stepper's actual commanded position,
        # which combines axis motion and offset changes. delta_stepper
        # naturally captures both because it's computed from the per-stepper
        # target (which includes offset).
        stepper.position_count.checked_add(delta_stepper)
          .or_else(|| fault(PositionCountOverflow, stepper))
```

### Position counters: invariants and update rules

Three counters per axis/stepper, each with a specific job:

- **`axis.last_step_count: i32`** — axis-level quantized position in
  microsteps, ignoring per-stepper offsets. Updated every sample in BOTH
  Pulse and Phase modes from `round(P_end / microstep_distance)`. Mode-
  agnostic — Pulse reads it to compute axis-level step deltas; Phase reads
  it as the base for per-stepper target computation. **No mode-switch
  resync needed in the engine** because both modes maintain it.

- **`stepper.last_phase_target: AtomicI32`** — per-stepper Phase-mode target
  position (= `axis.last_step_count + phase_offset_microsteps` at the
  last sample). Used only in Phase mode to compute per-stepper delta and
  thus update `position_count` correctly when offsets change between
  samples. Initialized at `kalico_set_axis_mode(Phase)` time to
  `axis.last_step_count + phase_offset_microsteps`.

- **`stepper.position_count: AtomicI32`** — the physical stepper's
  commanded position. Updated by:
  - Pulse mode: `axis_delta` per sample (lockstep with other paired steppers
    unless a single-stepper-jog segment masks this stepper out)
  - Phase mode: `delta_stepper` per sample (combines axis motion + any
    offset change picked up this sample)

### `phase_offset_microsteps` update semantics

`kalico_set_stepper_offset(stepper_idx, delta, max_microsteps_per_sample)`
in **Phase mode** atomically updates `phase_offset_microsteps` by `delta`.
The next TIM5 sample's Phase dispatch reads the new value, recomputes
`target_stepper`, and the resulting `delta_stepper` for that sample
includes the offset change. The motor physically slews to the new position
within that one sample (or several, if rate-limited).

**Rate limiting:** to honor `max_microsteps_per_sample`, the firmware
clamps how much `phase_offset_microsteps` may change per sample. If
the host requests a large delta in one call, the firmware ramps it
across multiple samples internally. (Engine-side ramping rather than
host-iterative offset writes — avoids host/firmware latency in the
loop and keeps the rate-limit guarantee firmly enforced.)

Specifically, each TIM5 sample, Phase-mode steppers run a ramp step
before the dispatch loop:

```
for stepper in all_steppers (Phase-mode axes):
  if stepper.phase_offset_target != stepper.phase_offset_microsteps:
    step = sign(target - current) · min(|target - current|, max_per_sample)
    stepper.phase_offset_microsteps += step
    # Next iteration of dispatch uses the new value automatically
```

This adds a small additional per-stepper field (`phase_offset_target`)
distinct from the current value. Total per-stepper Phase-mode state:
`phase_offset_microsteps`, `phase_offset_target`, `last_phase_target`,
`position_count`, `last_coil_A`, `last_coil_B`.

### Mode-switch counter resync

A mode switch can produce a one-shot burst on the first post-switch sample
if the dispatch logic computes a delta against a stale counter. The
`axis.last_step_count` field is updated in **both** Pulse and Phase samples
(per the dispatch pseudocode), so a Phase→Pulse transition has no stale
counter. The Pulse→Phase direction does: `stepper.last_phase_target` is
only written in Phase samples, so on Pulse→Phase the first Phase sample
would compute `delta_stepper = target − stale_last_phase_target` and burst.

**Resolution:** the `kalico_set_axis_mode` engine sequence performs an
explicit counter resync between flushing the queues and storing the new
mode value. See the full sequence under
[`kalico_set_axis_mode`](#kalico_set_axis_modeaxis_idx-u8-mode-stepmode---kalicoresult)
in the commands section — that's the authoritative spec.

### Per-axis Klipper timer (consumer, Pulse mode)

One permanent `struct timer` per motor axis (A, B, Z, E), riding on the
existing Klipper SysTick scheduler. The `struct timer` itself is C-allocated
(Klipper-owned), and its `func` pointer points to a **Rust `extern "C"`
function** that performs the drain. This is the "Option A storage + Rust
consumer logic" hybrid: queue storage stays C-owned per B2/B3, while the
drain logic gets Rust type safety on `StepEntry` decode and direct access
to the engine's atomic fault flags.

Body matches mainline's `stepper_event_edge` pattern: **one entry per
fire, fire at-or-after `cycle_abs`, never before.**

```
fn per_axis_step_event(t: &mut struct timer) -> u_fast8_t {
  let now = timer_read_time();
  let floor_time  = now.wrapping_add(DISPATCHER_FLOOR_CYCLES);
  let next_sample = now.wrapping_add(sample_period_cycles);

  // All u32 cycle-counter comparisons use Klipper's `timer_is_before(a, b)`,
  // which evaluates `(int32_t)(a - b) < 0` — wrap-aware on the 32-bit cycle
  // counter. NEVER compare cycle counters with plain `<`, `>`, or `max` —
  // they wrap every ~8.3 seconds on H7 (520 MHz / 2³² cycles) and ~4 minutes
  // on F446 (180 MHz / 2³² cycles), within a single print.

  // Pop exactly one entry if its scheduled time has arrived (cycle_abs <= now).
  // Mainline invariant: pulses fire at-or-after cycle_abs, never before.
  // If the head entry is still in the future, leave it for the next dispatch.
  if let Some(entry) = axis.step_queue.peek_head() {
    if !timer_is_before(now, entry.cycle_abs) {       // entry.cycle_abs <= now
      axis.step_queue.pop();
      runtime_emit_step_pulses(axis_idx, sign=entry.dir);
    }
  }

  match axis.step_queue.peek_head() {
    Some(next) => {
      // Pick whichever is later (wrap-aware): the entry's scheduled time,
      // or the dispatcher floor. timer_is_before(a, b) ⇒ a is earlier.
      t.waketime = if timer_is_before(next.cycle_abs, floor_time) {
        floor_time            // entry would race the dispatcher; clamp forward
      } else {
        next.cycle_abs
      };
    },
    None => t.waketime = next_sample,                  // re-check next TIM5
  }
  return SF_RESCHEDULE;
}
```

**Timing contract.** Pulses fire at `cycle_abs + dispatcher_jitter`, where
`dispatcher_jitter >= 0` (i.e., never early; potentially late by the time
between the cycle_abs and the next dispatcher dispatch). This matches
mainline Klipper's invariant. The < 500 ns klipper-sim test threshold
(Section 6) is the spec of acceptable dispatcher jitter, measured on the
target MCU.

**`DISPATCHER_FLOOR_CYCLES`** is the only constant here: minimum reschedule
gap, set to the *measured* Klipper scheduler dispatch overhead on the
target MCU. Profiled during bring-up Stage 1. Expected ~1-2 µs on H7
(~700 cycles), ~3-4 µs on F446 (~600 cycles).

**Per-MCU step-rate ceilings (measured during bring-up):**

| MCU | Sample rate | Max sustained step rate per axis | Limit |
|-----|-------------|----------------------------------|-------|
| H7 @ 520 MHz | 40 kHz | ~500 kHz | per-step body cost |
| F446 @ 180 MHz | 20 kHz | ~250 kHz | dispatcher floor |

`configure_axis` rejects configurations exceeding the per-MCU ceiling with a
`StepRateExceedsMcuCeiling` fault, computed from max-velocity × steps/mm.

For Phase-mode axes the step queue is unused; the per-axis timer fires but
its body sees an empty queue and reschedules. ~0.5% CPU per idle axis on H7.

### Queue granularity: per-edge entries, not bursts

The queue stores **one entry per step pulse** (`(cycle_abs, dir)`), not one
entry per burst (`(cycle_abs, n_steps, dir)`). This is deliberate.

A burst-style entry would carry `n_steps` pulses to emit back-to-back
starting at one `cycle_abs`. That would reduce queue pressure (1 entry per
sample instead of up to 13) and consumer fire count — `runtime_emit_step_pulses`
already takes `n_steps` and could be called once per burst — but it would
discard the per-step timing the velocity extrapolation produces. All N
pulses in a burst would fire essentially at the same moment (~100 ns
back-to-back), then nothing until the next sample. At constant high
velocity, the motor would see a 40 kHz burst-then-silence pattern, which
produces audible torque-ripple artifacts and frame resonance at exactly the
sample rate. This is the same back-to-back-at-sample-start failure mode
the velocity-extrapolated timing was designed to avoid.

The consumer fires **one entry per dispatch**, mainline-style, matching
the timing contract above (fire at-or-after `cycle_abs`, never before).
This preserves the producer's velocity-extrapolated precision through the
emission side, at the cost of more dispatcher fires per second at high
step rates than a burst-based design would need.

In music-score terms: per-edge entries are a score with each note's exact
time written, and the consumer plays each note at its written time. Burst
entries would be "play 8 notes starting at t=8.3 µs" — a 40-times-per-second
machine-gun cadence with no within-burst timing. We chose the former because
the motor doesn't tolerate the burst pattern (40 kHz beat artifacts) and
the cost of per-edge dispatching is acceptable on our hardware.

**If high-rate dispatcher cost becomes a profiling concern** during
bench bring-up (e.g., F446 hitting >50% CPU on dispatcher alone at peak
step rate), revisit by optimizing the consumer body (inline-style hot
path, fewer atomic ops, reduce volatile reads) rather than re-introducing
batching. Batching is a last resort, not the first optimization, because
it costs motion quality. The current design priority is mainline-quality
per-step timing.

Producer cost stays at ~4% CPU on H7 at peak (52 entries/sample × 4 axes ×
40 kHz × ~10 cycles/push), well inside budget.

### SPI write queue (Phase mode)

Per SPI bus, a small SPSC queue of `(cs_pin, register, value)` writes pushed
by TIM5 ISR, drained by a foreground task or DMA-driven SPI master.

Queue depth: 2 × (number of TMCs on bus), minimum 8.

Real-world bandwidth: roughly 2-3 TMC5160s per SPI bus at 40 kHz sample rate
before saturation (per mass3d fork experience). Octopus Pro: single physical
SPI bus serves all stepper slots, two other buses available for splitting.

Overflow → `SpiQueueOverflow(bus_idx)` fault → controlled shutdown.

Per-bus utilization telemetry: rolling fraction of sample windows that hit
≥90% bus occupancy, exposed via existing diag channels.

---

## State

### Per-stepper

```rust
pub struct StepperRef {
    pub step_pin: GpioPin,
    pub dir_pin: GpioPin,
    pub dir_invert: bool,                 // wiring polarity

    pub position_count: AtomicI32,        // signed; checked_add fault on overflow

    // Phase mode only:
    pub tmc_cs: Option<GpioPin>,
    pub last_coil_A: AtomicI16,           // re-write on motor re-energize
    pub last_coil_B: AtomicI16,
    pub phase_offset_microsteps: AtomicI32,  // CURRENT offset (ramped toward target)
    pub phase_offset_target: AtomicI32,      // TARGET offset (set by host)
    pub last_phase_target: AtomicI32,        // axis.last_step_count + phase_offset
                                              // at last Phase-mode sample
}
```

`position_count` is i32 — covers all realistic axis travel × microstepping
configurations with order-of-magnitude headroom on X/Y/Z. Extruder on
multi-kg prints at high microstepping can approach overflow; we accept this
trade for simplicity and detect it with `checked_add`. Migration path to
i64 (lo/hi split with sequence counter) documented and ready if needed.

`phase_offset_microsteps` is used by Phase mode coil-current calculation to
support motors-sync-style alignment. For Pulse mode, the equivalent
preservation happens automatically via TMC MSCNT — no firmware field needed.

### Per-axis (Rust engine state)

```rust
pub struct AxisConfig {
    pub mode: AtomicU8,                   // StepMode::Pulse=0, StepMode::Phase=1; switchable
    pub steppers: ArrayVec<StepperRef, 4>,
    pub piece: Option<BezierPieceMonomial>,
    pub piece_start_time: u64,            // cycles
    pub last_step_count: i32,             // axis-level, engine-private (not shared)
    pub microstep_distance: f32,          // mm per microstep, uniform across axis
    pub spi_bus: Option<&SpiBusRef>,      // bound for Phase mode
    // Step queue is NOT a field here — see "Per-axis step queue (C-owned shared state)"
    // below. AxisConfig accesses it by axis index through the C-defined extern.
}

pub struct BezierPieceMonomial {
    pub coeffs: [f32; 4],                 // c0 + c1·t + c2·t² + c3·t³
    pub vel_coeffs: [f32; 3],             // pre-baked derivative coefficients
    pub duration: f32,                    // seconds in this piece
}
```

### Per-axis step queue (C-owned shared state)

Per the architectural invariant in `docs/kalico-rewrite/mcu-c-rust-boundary.md`
(rules **B2** and **B3**), shared state that crosses the C/Rust boundary — or
even pure-Rust shared state that crosses indirection LLVM is allowed to
optimize past — **must be defined in C, with a `#[repr(C)]` Rust mirror that
does not own storage.** The 2026-05-18 case study (heapless::spsc::Consumer
miscompile, pure Rust on both ends) is the load-bearing precedent: the rule
applies even when both producer and consumer are Rust ISR contexts.

**Storage:** 4 instances of `StepQueue`, declared in `src/step_queue.c`,
placed in DTCM on H7 (non-cached, eliminates cache-coherency concerns) and
default `.bss` on F4. Linker section name TBD against the existing H7 linker
script — see open question Q-LINKER below.

**C definitions (`src/step_queue.h`):**

Depth sized for the stated per-MCU step-rate ceilings:
- H7 peak: 500 kHz / 40 kHz sample = ⌈13⌉ entries per sample peak production
- F446 peak: 250 kHz / 20 kHz sample = ⌈13⌉ entries per sample peak production
- Choose **32** (power of 2): holds 2.5 samples of headroom against
  consumer preemption (USB ISR, status drain, etc.). Power-of-2 mask
  (`& 0x1F`) replaces software modulo on the hot path.
- SPSC convention uses absolute u16 head/tail counters with wrapping
  subtraction for length; all 32 slots are usable (no empty-slot
  reservation needed).

```c
#define STEP_QUEUE_DEPTH      32
#define STEP_QUEUE_DEPTH_MASK 0x1F   // depth - 1; depth must be power of 2

typedef struct {
    uint32_t cycle_abs;   // lower 32 bits of DWT CYCCNT; wrap-aware compare only
    int8_t   dir;         // +1 / -1
    uint8_t  _pad[3];     // explicit padding, matches Rust #[repr(C)] padding
} StepEntry;              // sizeof == 8, 4-byte aligned

typedef struct {
    volatile uint16_t tail;   // producer-owned (TIM5 ISR writes)
    volatile uint16_t head;   // consumer-owned (per-axis timer writes)
    uint8_t _pad[4];          // align buf to 8 bytes
    StepEntry buf[STEP_QUEUE_DEPTH];
} StepQueue;                  // sizeof == 8 + 8*32 == 264, 8-byte aligned

extern StepQueue step_queues[4];   // one per motor axis (A, B, Z, E)
```

```c
// src/step_queue.c
#if CONFIG_MACH_STM32H7
__attribute__((section(".dtcm_bss")))    // exact section name pending Q-LINKER
#endif
StepQueue step_queues[4];

_Static_assert(sizeof(StepEntry)  == 8,   "StepEntry layout drift");
_Static_assert(sizeof(StepQueue)  == 264, "StepQueue layout drift");
_Static_assert(offsetof(StepQueue, buf) == 8, "StepQueue.buf offset drift");
_Static_assert((STEP_QUEUE_DEPTH & STEP_QUEUE_DEPTH_MASK) == 0,
               "STEP_QUEUE_DEPTH must be power of 2");
```

Storage: 4 × 264 = 1056 B in DTCM (H7) / `.bss` (F4). Negligible.

**Rust mirror (`rust/runtime/src/step_queue.rs`):**

```rust
use core::cell::UnsafeCell;

#[repr(C)]
pub struct StepEntry {
    pub cycle_abs: u32,
    pub dir: i8,
    _pad: [u8; 3],
}

pub const STEP_QUEUE_DEPTH: usize = 32;
pub const STEP_QUEUE_DEPTH_MASK: u16 = (STEP_QUEUE_DEPTH as u16) - 1;

#[repr(C)]
pub struct StepQueue {
    pub tail: u16,   // accessed via ptr::{read,write}_volatile from Rust
    pub head: u16,
    _pad: [u8; 4],
    pub buf: [StepEntry; STEP_QUEUE_DEPTH],
}

const _: () = {
    assert!(core::mem::size_of::<StepEntry>() == 8);
    assert!(core::mem::size_of::<StepQueue>() == 264);
    assert!(core::mem::offset_of!(StepQueue, buf) == 8);
    assert!(STEP_QUEUE_DEPTH.is_power_of_two());
};

extern "C" {
    // C owns storage; UnsafeCell carries interior-mutability rights for ISR
    // access from both producer (TIM5) and consumer (per-axis timer) sides.
    pub static step_queues: UnsafeCell<[StepQueue; 4]>;
}
```

**SPSC access pattern (matches B5):**

Producer (TIM5 ISR, Rust):
1. Check `tail - head < STEP_QUEUE_DEPTH` (wrapping u16 subtract); overflow → fault.
2. `ptr::write_volatile` the entry at `buf[(tail & STEP_QUEUE_DEPTH_MASK) as usize]`
   (equivalent to `tail % STEP_QUEUE_DEPTH`; mask form is the hot-path version).
3. `core::sync::atomic::fence(Ordering::Release)` — lowers to `DMB` on ARMv7-M.
4. `ptr::write_volatile` to `tail = tail + 1`.

Consumer (per-axis timer, Rust `extern "C"` function — see below):
1. `ptr::read_volatile` `tail` and `head`.
2. If `tail == head`: queue empty, return without popping.
3. `core::sync::atomic::fence(Ordering::Acquire)` before reading entry data.
4. Read entry at `buf[(head & STEP_QUEUE_DEPTH_MASK) as usize]` via `read_volatile`.
5. `core::sync::atomic::fence(Ordering::Release)` before advancing head.
6. `write_volatile` to `head = head + 1`.

Memory cost: 4 queues × 264 B = 1056 B (matches the value stated above at
`StepQueue` declaration). Trivial in DTCM (H7) or `.bss` (F4).

`AxisConfig::mode` uniform across all steppers on the axis (Section 4
constraint). Mixed-mode-per-axis is rejected at `configure_axes` time with a
clear error.

`BezierPieceMonomial` is pre-baked at piece load. Bernstein control points
stay in host-facing storage (matches CLAUDE.md, planner, wire format).
Conversion happens at piece-load time, once per piece — not per sample.

Monomial form gives ~17 cycles per (position, velocity) pair via Horner
unrolled, vs ~80 cycles for de Boor on the same cubic. At 4 axes × 40 kHz =
160 kHz eval rate, that's the difference between 3% and 12% CPU on H7.

### Shared (host-visible)

Existing `shared.SharedRuntime` extended with:

```rust
pub queue_high_water: [AtomicU32; N_AXES],         // per-axis peak queue depth
pub queue_overflow_count: [AtomicU32; N_AXES],
pub spi_saturated_samples: [AtomicU32; N_SPI_BUSES],
pub sample_isr_peak_cycles: AtomicU32,
pub per_axis_consumer_peak_cycles: [AtomicU32; N_AXES],
pub fault: AtomicU32,                              // existing
```

Fault encoding: low 16 bits = `FaultCode` enum; high 16 bits = axis or
stepper index.

---

## New commands

### `kalico_set_axis_mode(axis_idx: u8, mode: StepMode) -> KalicoResult`

Switch an axis between Pulse and Phase. **Idle-only**: the engine refuses
the command if any motion segment is in flight on any axis. The host is
responsible for quiescing motion before calling.

Returns:
- `Ok` — mode switch applied successfully.
- `Err(MotionInProgress)` — at least one axis has an active segment; caller
  must wait and retry. Not a fault: a recoverable busy signal.

Engine sequence on accepting the command (authoritative — the "Mode-switch
counter resync" subsection above is supporting commentary; this sequence is
what the implementation follows):

1. Verify no segment is mid-execution on any axis (atomic check on
   `producer_current`). If any active, return `Err(MotionInProgress)`.
2. Flush `axis.step_queue` (drain any queued entries — there should be none
   since motion is idle, but defensive).
3. Flush SPI write queue entries targeting this axis's TMCs.
4. **Counter resync** before the mode store, to avoid one-shot bursts on
   the first sample after switch:
   - Entering **Phase** mode (from Pulse): for each stepper on the axis,
     `last_phase_target.store(axis.last_step_count + phase_offset_microsteps,
     Release)`. The first Phase-mode sample then computes
     `delta_stepper = 0` (modulo any in-flight offset ramp), no burst.
   - Entering **Pulse** mode (from Phase): no resync needed.
     `axis.last_step_count` was being updated during Phase samples (per the
     dispatch pseudocode), so the first Pulse sample's
     `n_steps = round(P_end / microstep_distance) - axis.last_step_count`
     reflects only motion since the last sample — no burst.
5. Atomic store on `axis.mode` (Release ordering, picked up by next TIM5 ISR).
6. Return `Ok`.

The engine **does not** write TMC configuration registers. That's the host's
responsibility, sequenced **after** `set_axis_mode` returns Ok:

```
# Host-side flow (in Klippy homing extra or equivalent):
toolhead.wait_moves()                            # drain all in-flight motion
result = mcu.send(kalico_set_axis_mode, axis=X, mode=Pulse)
if result == MotionInProgress:
    raise GcodeException("motion not idle, cannot switch axis mode")
# Engine now in Pulse mode for X, no SPI XDIRECT writes happening.
# Safe to reconfigure TMC chip:
tmc_x.set_chopconf(mres=0, ...)                  # restore microstep table
tmc_x.set_coolconf(sgt=2, ...)                   # enable stallguard
tmc_x.set_tcoolthrs(...)
# Now safe to queue motion in the new mode:
toolhead.move(...)
```

This ordering avoids the race where the engine is still writing XDIRECT
while the host reconfigures the TMC into normal microstep mode (or vice
versa). Engine flushes first; host configures second; motion resumes third.

The `ModeSwitchWhileMoving` fault is removed — the idle-only check returns
a recoverable `Err`, not an unrecoverable fault.

Use case: sensorless homing on a normally-phase-stepped axis. Klippy homing
extra reads per-axis `primary_mode` and `homing_mode` from printer.cfg,
waits for motion idle, calls `set_axis_mode(homing_mode)`, then reconfigures
TMC, then performs the homing maneuver. After homing, the symmetric sequence
restores `primary_mode`.

### `kalico_set_stepper_offset(stepper_idx: u8, delta_microsteps: i32, max_microsteps_per_sample: u16) -> KalicoResult`

Apply a per-stepper microstep offset. Physical motion: motor moves by
`delta_microsteps`. Other steppers on the same axis stay put.

`max_microsteps_per_sample`: rate limit per TIM5 sample. Default 8 — safe
under skip threshold for any realistic motor / microstep config. Host can
pass higher (or 0 to mean axis-default) at its own risk.

Behavior:
- `|delta| ≤ max_per_sample`: apply in one sample. Pulse mode → emit
  step pulse burst within sample; Phase mode → write new coil currents.
- `|delta| > max_per_sample`: spread across `ceil(|delta| / max_per_sample)`
  samples.
- Out-of-range parameters → `JogParametersInvalid` fault.

Position_count of target stepper updates by `delta_microsteps`. Other steppers
on the axis remain unchanged.

Motors-sync-style use case: host iterates small calls (≤3 microsteps default),
runs accelerometer test between calls, converges to target phase. Total
algorithm runs at macro-level, no per-iteration firmware state needed.

Persistence model:
- **Coil de-energize (motor sleeps, driver stays powered):** Pulse mode — TMC
  MSCNT preserved by driver, motor returns to last microstep on re-energize.
  Phase mode — `last_coil_A/B` re-written from shared state on re-energize
  command, motor returns to last position.
- **Full board power cycle:** all MCU state lost. Host saves
  `phase_offset_microsteps` via Klipper's save_variables, replays via
  `kalico_set_stepper_offset` at boot.

### `kalico_configure_axis(axis_idx, mode, microstep_distance, steppers)`

Replaces the current per-stepper `config_runtime_stepper` command. Configures
an entire axis in one call, including its bound steppers, the uniform stepping
mode, and microstep distance. Called once per axis at startup.

Per-stepper detail (step_pin, dir_pin, dir_invert, tmc_cs) carried in a
sub-message per stepper in the axis config.

Per-axis uniform-mode constraint enforced here: rejected if `steppers` carries
steppers with incompatible drivers for the requested mode.

### `kalico_configure_kinematics(k_xy: f32)`

Sets the Cartesian-arc-length kinematic factor K_xy. Called once at startup,
before any motion. Values:
- Cartesian: 1.0
- CoreXY: 1.0 / sqrt(2) ≈ 0.7071068

Used by the TIM5 ISR's E-follows-XY integration to convert motor-space
velocity into Cartesian XY arc length: `|v_xy| = sqrt(vA² + vB²) · K_xy`.
Without this, CoreXY moves over-extrude by ~41%.

---

## What gets deleted

- `engine.rs:2780-2900` (`producer_step` Newton-fill loop)
- `engine.rs:107` `PRODUCER_BATCH_CAP = 32`
- `compute_next_step_time` and `solve_monotone_cubic_root`
- `step_ring.rs` 1024-entry per-motor `StepRing` (replaced by 16-entry per-axis
  `SpscQueue` with same pattern)
- `runtime_tick.c:1735-1838` (`step_time_event` per-stepper consumer)
- `runtime_tick.c:1840-1891` (`runtime_producer_event` and the permanent
  producer timer with its race-avoidance comment block)
- `runtime_tick.c:83,96` `SF_RESCHEDULE_FLOOR=100µs` and `EMPTY_POLL_CYCLES=100ms`
- `runtime_tick.c:1900-1955` `init_step_time_timers` (replaced with per-axis init)
- `stepper.c:601-602` `config_runtime_stepper` signature (replaced)
- The current Modulated TIM5 ISR body (replaced by the unified evaluator)

What stays:
- `runtime_emit_step_pulses` GPIO toggle + direction logic
- Endstop sampling hooks (called from new TIM5 ISR at sample rate)
- Fault propagation, watchdog, shutdown plumbing
- Curve pool / segment pool storage
- Segment push / retire host-sync via `retired_through_segment_id`
- All upstream Bezier infrastructure (host planner, wire format, compat crate,
  gcode parser)

One-shot replacement, not parallel paths. No legacy fallback flag.

---

## Faults

New fault codes added:

- `StepQueueOverflow(axis_idx)` — per-axis Klipper timer fell behind by > 2
  samples. Indicates ISR preemption storm or scheduler corruption.
- `SpiQueueOverflow(bus_idx)` — SPI bus bandwidth exceeded. Indicates too many
  Phase-mode steppers on one bus at configured sample rate.
- `MathNonFinite(axis_idx)` — Bezier evaluator produced NaN/Inf.
- `PieceAdvanceUnderflow(axis_idx)` — sample straddled > 4 pieces, suggests a
  pathological segment.
- `SampleRateMisconfigured` — boot-time validation failure.
- `PositionCountOverflow(stepper_idx)` — i32 `position_count` exceeded range.
- `JogParametersInvalid` — `kalico_set_stepper_offset` parameters out of bounds.
- `StepRateExceedsMcuCeiling(axis_idx)` — config-time rejection when an axis's
  max sustained step rate (max_velocity · steps_per_mm) exceeds the per-MCU
  ceiling stated in the consumer section. Not a fault during motion; raised
  at `configure_axis` time so misconfigured machines refuse to start motion.

All faults route through existing `shared.fault` mechanism → foreground
reactor → `kalico_runtime_shutdown_engine` → Klipper shutdown.

---

## Testing

### Pre-bench (offline)

**Rust unit tests on math kernel:**
- `monomial_horner_eval` vs de Boor on randomized cubic Beziers (f32 ulp match)
- Bernstein→monomial conversion round-trip
- Velocity-extrapolation formula against analytical step times for constant-v
  and constant-a curves (< 100 ns timing error)
- `DISPLACEMENT_THRESHOLD` fallback triggers when per-sample displacement
  is below ~1 microstep (near-stationary samples, direction reversal at v=0)
- Piece-boundary advancement (sample straddling pieces)

**Property tests on SPSC step queue:**
- Producer push / consumer pop random ordering, no torn reads
- Overflow detection fires exactly when capacity exhausted

**klipper-sim integration:**
- Same G-code through mainline klipper-sim and this fork's engine
- Per-axis step times match within local-linear-extrapolation tolerance
  (< 500 ns per step at typical accel)
- Per-axis cumulative step counts match exactly at every segment boundary

### Renode (narrow use only)

Per past experience ("looked fine in sim, broken on hardware"), Renode is used
only for structural checks:
- Boot reaches main loop
- TIM5 ISR fires at all
- Memory placement (`.axi_bss`, DTCM)
- GPIO toggles when commanded
Not used for motion correctness or sustained timing.

### Cycle-cost profiling

Targets:

| Path | H7 @ 520 MHz | F446 @ 180 MHz |
|------|--------------|----------------|
| TIM5 ISR (4 axes Pulse, idle queues) | ≤ 800 cycles | ≤ 1500 cycles |
| TIM5 ISR (peak, 2 axes × 8 steps each) | ≤ 2500 cycles | ≤ 5000 cycles |
| Per-axis consumer (pop + emit) | ≤ 100 cycles | ≤ 200 cycles |
| SPI write enqueue | ≤ 50 cycles | ≤ 100 cycles |

`AtomicI64` is not available on `thumbv7em-none-eabihf`; `AtomicI32` is used
for `position_count`. Verification: compile target before bench bring-up.

### Bench bring-up stages

**Stage 1 — Boot + idle.** Flash, verify telemetry-only:
- TIM5 ISR fires at configured rate
- All step queues depth 0
- No faults
- Idle ISR cycle cost matches target

**Stage 2 — Single-stepper offset (calibration primitive).**
`kalico_set_stepper_offset(0, 10, 8)`. Verify telemetry:
- Target stepper position_count += 10
- Other steppers unchanged
- TIM5 sampled ~400 times during a 10 ms jog window
- No faults
Then issue a 100-microstep version and physically verify motor moves.

**Stage 3 — Pure-X G1, Pulse mode (CoreXY: both A and B step in lockstep).**
G1 X10 F600.
- Both A and B position_count increment by same amount, same direction
- Queue drain rate matches step rate
- klipper-sim agrees

**Stage 3b — Pure-Y G1 (CoreXY: A and B opposite directions).**
G1 Y10 F600 then G1 Y0 F600.
- A and B step opposite directions, same magnitude
- Reversal works (sign of n_steps)

**Stage 4 — Multi-motor stress.** G1 X50 Y-50 F12000 (drives both A and B at
full rate). Sustained square pattern. No overflows, no faults.

**Stage 5 — Phase mode bring-up on X.** Switch X to Phase, repeat stages 3-4.
- SPI bus utilization < 80%
- TMC5160 XDIRECT receives expected coil sequence (logic analyzer)
- Position tracking matches Pulse mode

**Stage 6 — Sensorless homing via mode switch.** X primary=Phase, homing=Pulse.
G28 X.
- Mode switch fires at segment boundary
- TMC config re-written for stallguard
- Home completes via stallguard
- Mode returns to Phase

**Stage 7 — Long-print soak.** Real CoreXY print, ≥ 1 hour.
- No faults, no peak-cycle drift, position_count returns to expected total
  motion at end.

---

## Open items deferred

These are recorded for future work; not blocking this redesign.

- Hardware capture-compare GPIO toggling — requires board pin allocation we
  don't have. **Not planned**; we don't have plans for in-house controllers.
  Removed entirely from "future destinations."
- DMA-driven GPIO — same reasoning, deferred indefinitely.
- Closed-loop encoder integration — architecturally supported via existing
  `position_count` + `set_stepper_offset` primitive; control loop is host-side
  and out of scope.
- Pellet / multi-day extruder configs that exceed i32 `position_count` range
  — migration to lo/hi-split i64 documented, ready if needed.
- Local-quadratic velocity extrapolation — local-linear is sufficient for
  motor mechanical bandwidth; upgrade documented but not implemented.
- Per-axis sample rate — global config sufficient; per-axis is a future change
  if some axis demands different time resolution.
- Step pulse width emitter for non-DEDGE drivers — out of scope; all target
  hardware (TMC2209/2240/5160) supports DEDGE.

### Open questions to resolve before implementation

- **Q-LINKER:** The `StepQueue` storage section name. The boundary doc names
  H7's `.axi_bss` section, but DTCM placement is preferred for the step
  queues (non-cached, eliminates cache-coherency concerns between TIM5 ISR
  and SysTick consumer). Confirm via inspection of the existing H7 linker
  script whether there's a DTCM-mapped `.bss` region (matching the segment
  queue's placement). If only `.axi_bss` is available, evaluate whether AXI
  SRAM's cache behavior is acceptable for SPSC use. Resolve before writing
  the implementation plan.

---

## Glossary recap

- **Sample**: one TIM5 ISR fire.
- **Sample rate**: configurable. 40 kHz default H7, 20 kHz default F446.
- **Motor axis (A, B, Z, E)**: post-kinematics output axis; what the engine
  evaluates per sample.
- **Stepper**: physical TMC driver bound to one motor axis.
- **Piece**: one cubic Bezier curve in monomial form.
- **StepMode**: per-axis enum {Pulse, Phase}.
- **Pulse mode**: STEP pin driven, TMC internal microstep table.
- **Phase mode**: TMC DIRECT mode, coil currents driven via SPI.
- **Step queue**: per-axis SPSC queue of `(cycle_abs, dir)` for Pulse mode.
- **SPI write queue**: per-bus SPSC queue of TMC register writes for Phase mode.
- **Velocity extrapolation**: secant-slope linear interpolation through
  `(0, P_sample_start)` and `(sample_period, P_end)`, evaluated at each
  integer step boundary `step_pos_k`. Yields `t_k ∈ [0, sample_period]`
  by construction. Replaces trapezoidal-average estimate that could push
  step times outside the sample window for cubic position curves.
