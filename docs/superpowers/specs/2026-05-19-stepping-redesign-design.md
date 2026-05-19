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

```
TIM5 ISR fires every (1 / sample_rate) µs:

  P_sample_start_cache = P_sample_end_cache_from_last_fire
  v_sample_start_cache = v_sample_end_cache_from_last_fire

  for axis in [A, B, Z, E]:
    # (1) Advance piece if sample straddles boundary
    while t_local_for_axis(t_sample_end) > axis.piece.duration:
      advance to next piece (or break if segment retiring)

    if axis.piece is None:
      skip (axis idle this sample)

    # (2) Per-axis polynomial eval (one per axis per sample)
    (P_end, v_end) = monomial_horner_eval(axis.piece, t_local_for_axis)
    if !P_end.is_finite() || !v_end.is_finite():
      fault(MathNonFinite, axis)
      continue

    # (3) Endstop sample (existing hook, cheap when no arm)
    kalico_endstop_tick_step_time(handle, now)

    # (4) E-follows-XY arc-length integration (CLAUDE.md §extruder).
    # IMPORTANT: arc length must be Cartesian XY, not motor-space A/B.
    # For CoreXY: |v_xy| = sqrt(vA² + vB²) / sqrt(2); for cartesian: = sqrt(vA² + vB²).
    # Host pushes the kinematic factor K_xy at configure_axes time:
    #   K_xy = 1.0           for cartesian (A=X, B=Y trivially)
    #   K_xy = 1.0/sqrt(2)   for CoreXY
    # (Delta and other non-orthogonal kinematics need K computed per pose, out
    # of scope here; CoreXY + Cartesian cover the bench and all CLAUDE.md
    # target machines.)
    # Computed once per sample after A and B have both been evaluated.
    if axis == B:    # last of the XY pair; A already evaluated this sample
      v_motor_sq = v_A_cached² + v_B_cached²
      v_xy = sqrt(v_motor_sq) · K_xy
      ds_xy += v_xy · sample_period
      v_xy_delta = v_xy - v_xy_prev_sample
    if axis == E:
      P_end += extrusion_per_xy_mm · ds_xy
              + pressure_advance(sign(v_xy_delta)) · v_xy_delta

    # (5) Per-axis dispatch on stepping mode
    match axis.mode.load(Acquire):
      Pulse:
        prev_step_count = axis.last_step_count
        target_step_count = round(P_end / axis.microstep_distance)
        n_steps = target_step_count - prev_step_count
        axis.last_step_count = target_step_count

        if |n_steps| > 0:
          v_avg = (v_sample_start + v_end) / 2
          if |v_avg| > V_EXTRAPOLATION_THRESHOLD:    # default 1 mm/s
            # Velocity-extrapolated sub-sample times
            for k in 0..|n_steps|:
              step_pos_k = (prev_step_count + (k+1) · sign(n_steps))
                           · axis.microstep_distance
              t_local = (step_pos_k - P_sample_start) / v_avg
              cycle_abs = sample_start_cycles + t_local · cycles_per_second
              push to axis.step_queue
          else:
            # Near-zero velocity fallback: uniform within sample
            for k in 0..|n_steps|:
              t_local = sample_period · (k+1) / (|n_steps|+1)
              push to axis.step_queue

          for stepper in axis.steppers:
            stepper.position_count.checked_add(n_steps)
              .or_else(|| fault(PositionCountOverflow, stepper))

      Phase:
        # Per-stepper SPI dispatch (each TMC has its own CS).
        # TMC5160 electrical cycle = 1024 microsteps (10-bit MSCNT) =
        # 4 full steps. Coil-current LUT is 1024 entries spanning one
        # electrical cycle.
        for stepper in axis.steppers:
          target_microsteps = round(P_end / axis.microstep_distance)
                            + stepper.phase_offset_microsteps
          phase = target_microsteps & 0x3FF              # 10-bit, 1024 entries
          (coil_A, coil_B) = phase_lut[phase]
          spi_queue.push(stepper.tmc_cs, XDIRECT_REG, pack(coil_A, coil_B))
          stepper.last_coil_A.store(coil_A)
          stepper.last_coil_B.store(coil_B)
          stepper.position_count.checked_add(target_microsteps - prev_target)
            .or_else(|| fault(PositionCountOverflow, stepper))

    # (6) Segment retirement check
    if all participating axes' cursors have reached segment.duration:
      retire segment (host sync via existing retired_through_segment_id)
      advance to next segment

  P_sample_end_cache = P_end
  v_sample_end_cache = v_end
```

### Per-axis Klipper timer (consumer, Pulse mode)

One permanent `struct timer` per motor axis (A, B, Z, E), riding on the
existing Klipper SysTick scheduler. Body identical pattern to mainline's
`stepper_event_edge`:

```
fn per_axis_step_event(t: &mut struct timer) -> u_fast8_t {
  if entry = axis.step_queue.try_pop():
    runtime_emit_step_pulses(axis_idx, sign=entry.dir)
      # toggles all step pins of steppers bound to axis (existing fan-out)
  if next = axis.step_queue.peek_head():
    t.waketime = max(next.cycle_abs, now + SF_RESCHEDULE_FLOOR_NEW)
  else:
    t.waketime = now + sample_period_cycles  # re-check after next TIM5 fire
  return SF_RESCHEDULE
}
```

`SF_RESCHEDULE_FLOOR_NEW`: ~5 µs (1500 cycles on H7, 900 cycles on F446) —
matches Klipper scheduler dispatch overhead, not the previous arbitrary
100 µs. No artificial step-rate ceiling.

For Phase-mode axes the step queue is unused; the per-axis timer fires but
its body sees an empty queue and reschedules. ~0.5% CPU per idle axis on H7.

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
    pub phase_offset_microsteps: AtomicI32, // for motors-sync-style alignment
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

### Per-axis

```rust
pub struct AxisConfig {
    pub mode: AtomicU8,                   // StepMode::Pulse=0, StepMode::Phase=1; switchable
    pub steppers: ArrayVec<StepperRef, 4>,
    pub piece: Option<BezierPieceMonomial>,
    pub piece_start_time: u64,            // cycles
    pub last_step_count: i32,             // axis-level, engine-private (not shared)
    pub step_queue: SpscQueue<StepEntry, 16>,
    pub microstep_distance: f32,          // mm per microstep, uniform across axis
    pub spi_bus: Option<&SpiBusRef>,      // bound for Phase mode
}

pub struct BezierPieceMonomial {
    pub coeffs: [f32; 4],                 // c0 + c1·t + c2·t² + c3·t³
    pub vel_coeffs: [f32; 3],             // pre-baked derivative coefficients
    pub duration: f32,                    // seconds in this piece
}

pub struct StepEntry {
    pub cycle_abs: u32,                   // lower 32 bits of cycle counter
    pub dir: i8,                          // +1 / -1
}
```

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

Switch an axis between Pulse and Phase. Synchronous — waits for current
segment to retire before applying. Returns `MotionInProgress` if a non-jog
segment is mid-execution.

Sequence:
1. Engine accepts the request, blocks new push_segment until the switch completes
2. Wait for current segment retire (typical: ms; bounded: segment.duration)
3. Flush axis.step_queue and SPI writes for that axis's TMCs
4. Host writes TMC config registers for the new mode (mode change of CHOPCONF,
   stallguard enable/disable, etc.) — this is a separate Klippy step preceding
   `kalico_set_axis_mode`
5. Engine updates `axis.mode` (Release store, picked up by next TIM5 ISR)
6. Engine unblocks push_segment

Use case: sensorless homing on a normally-phase-stepped axis. Klippy homing
extra reads per-axis `primary_mode` and `homing_mode` from printer.cfg,
issues `set_axis_mode(homing_mode)` before the homing maneuver and restores
`set_axis_mode(primary_mode)` after.

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
- `ModeSwitchWhileMoving(axis_idx)` — `kalico_set_axis_mode` called mid-segment.

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
- `V_EXTRAPOLATION_THRESHOLD` fallback triggers near v=0
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
- **Velocity extrapolation**: `t_k = (step_pos_k - P_start) / v_avg` over each
  sample interval.
