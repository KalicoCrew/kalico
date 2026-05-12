# Phase-stepping math kernel (pure functions, no engine integration)

**Status:** Design approved 2026-05-12, ready for implementation plan.
**Author:** Brainstorm 2026-05-12 between Danila Dergachev and Claude.
**Implements:** Math half of `StepMode::Modulated`'s future evolution (build-order Step 10 — phase stepping). This spec lands the pure-math kernel only. Engine integration, TMC5160 XDIRECT driver, and klippy plumbing are out of scope and addressed when full Step 10 lands.

## 1. Problem

Build-order Step 10 (phase stepping) replaces discrete STEP/DIR pulses with continuous coil-current commutation: instead of telling the TMC driver "take one microstep now," the engine writes `(I_A, I_B) = (cos θ, sin θ)` directly to the driver's coil-current setpoints at 40 kHz, where θ is the rotor's commanded electrical angle. The driver does no microstep interpolation; the rotor follows the magnetic field smoothly.

Today the `Modulated` path (see `docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md` §3) emits discrete step pulses via the `StepAccumulator`. Step 10 augments that path with cos/sin commutation. Until then, no phase-current math exists in the codebase.

**Plain English:** A stepper motor has two coils, A and B. If we set their currents to `(cos θ, sin θ)`, the rotor sits at angle θ — *any* θ, smoothly. Today we tell the driver "next microstep" and let it interpolate. Phase stepping skips that, computes the smooth waveform ourselves, and ships values directly. Smoother motion, no microstep ridges. This spec adds the math that produces `(cos θ, sin θ)` from a motor position — nothing else.

## 2. Scope

### 2.1 In scope

- A new leaf module `rust/runtime/src/phase.rs` with three pure functions and their unit tests.
- libm-based correct-by-construction implementation. Optimized variants (LUT, polynomial) added later, in the same file, as additional pure functions, when an MCU performance budget mandates them.

### 2.2 Out of scope (explicit YAGNI)

- Engine ISR integration. `phase.rs` is dead code (importable but unused) until Step 10 wires it into `runtime_handle_tick` for `StepMode::Modulated` steppers.
- Per-stepper mode dispatch. Already exists — see `StepMode` in `rust/runtime/src/state.rs`. This spec adds no new dispatch.
- C-side coil-current emit (`runtime_emit_phase_currents` or equivalent). Lands with Step 10.
- TMC5160 XDIRECT driver (GCONF.direct_mode write, XDIRECT register stream). Lands with Step 10.
- Wire-protocol changes. The eventual phase-stepping config flows through the existing `phase_stepping: 1` knob and `StepMode::Modulated` state already plumbed by the step-time-scheduling work.
- klippy homing.py runtime mode-flip integration (sensorless-homing-on-phase-stepped-axes workflow). Lands with Step 10.
- LUT-based or polynomial cos/sin approximations. Future PRs add them as additional pure functions; not required for the math kernel to be correct or testable.

## 3. Module surface

`rust/runtime/src/phase.rs`. Sibling of `rust/runtime/src/step.rs` (discrete-step accumulator) and `rust/runtime/src/step_time.rs` (event-driven step scheduling). Does not import from any other runtime module. Only `pub mod phase;` is added to `lib.rs`.

Three pure functions:

```rust
/// Electrical angle of the rotor, in radians, given its mechanical position.
///
/// A two-phase hybrid stepper has 4 full mechanical steps per electrical
/// revolution (the rotor's magnetic field repeats every 4 full steps). So
/// `electrical_angle = 2π × (motor_position_mm / mechanical_wavelength_mm)`,
/// where `mechanical_wavelength_mm = 4 / full_steps_per_mm`.
///
/// `full_steps_per_mm` is the *full-step* (not microstep) density:
/// `full_steps_per_rev × gearing / rotation_distance`. The
/// `steps_per_mm` field carried over the wire (`MotorConfig::steps_per_mm`)
/// is microsteps/mm; callers must divide by the microstep multiplier
/// before calling this function.
pub fn electrical_angle_rad(motor_position_mm: f32, full_steps_per_mm: f32) -> f32;

/// Coil currents (I_A, I_B), each ∈ [-1.0, +1.0], that position the rotor
/// at `motor_position_mm`. `I_A = cos(θ)`, `I_B = sin(θ)`, where `θ` is
/// the electrical angle from `electrical_angle_rad`.
///
/// Outputs are normalized to "full-scale current"; the eventual on-MCU
/// emitter is responsible for scaling to the TMC driver's register units.
pub fn phase_currents(motor_position_mm: f32, full_steps_per_mm: f32) -> (f32, f32);

/// Same as `phase_currents` but honors a per-stepper direction inversion
/// flag (the equivalent of klippy's `dir_pin: !PIN` polarity). When
/// `invert_dir` is `true`, the electrical angle is negated before
/// computing cos/sin — i.e., the rotor's "positive direction" reverses.
pub fn phase_currents_with_invert(
    motor_position_mm: f32,
    full_steps_per_mm: f32,
    invert_dir: bool,
) -> (f32, f32);
```

That is the entire surface. No state. No constructors. No traits. Three functions; the latter two compose with the first.

### 3.1 Implementation notes

- Internally uses `libm::cosf` and `libm::sinf` (already a transitive dependency via the `nurbs` crate, which uses `libm::fmaf` / `sqrtf` / `fabsf` in `rust/nurbs/src/float.rs`).
- All inputs and outputs are `f32`. The MCU runtime is single-source-compiled `f64` host / `f32` MCU (CLAUDE.md), and phase commutation is squarely in the MCU side of that split.
- No allocation, no I/O, no panics on valid inputs. `f32::NAN` / `f32::INFINITY` inputs are pass-through (libm returns NaN; callers must not feed those values — guarded upstream when integration lands).

## 4. Math behind the kernel

A two-phase hybrid stepper has `full_steps_per_rev` full mechanical steps per shaft revolution (typically 200 for a 1.8°/step motor). Each electrical revolution covers 4 full mechanical steps — the rotor's magnetic field repeats every 4 full steps, regardless of how the driver microsteps within them. So:

- Mechanical wavelength: `λ_mech = 4 / full_steps_per_mm` (mm per electrical revolution).
- Electrical angle: `θ = 2π × motor_position_mm / λ_mech = (π/2) × full_steps_per_mm × motor_position_mm`.
- Coil currents: `I_A = cos θ`, `I_B = sin θ`. Unit-circle invariant `I_A² + I_B² = 1` holds at every sample.
- Direction inversion: `θ_inverted = −θ`, equivalently `(I_A, I_B) → (I_A, −I_B)` (cos is even, sin is odd).

The `(π/2) × full_steps_per_mm` term is the only computation specific to this kernel beyond the trig. Everything else is libm.

**Plain English:** Multiply the motor position by a constant — `π/2 × full_steps_per_mm` — to get the electrical angle. Take cos and sin of that angle. Done. The constant is "how many electrical radians per millimeter of mechanical motion," which depends on the motor (1.8° vs 0.9° stepper, gearing, leadscrew pitch) but is fixed for a given configuration.

## 5. Test plan

All tests live in `rust/runtime/src/phase.rs` under `#[cfg(test)] mod tests`. Host-only; no MCU build matters at this stage.

| Test | What it asserts |
|------|-----------------|
| Closed-form correctness sweep | For a sweep of `(motor_position_mm, full_steps_per_mm)` covering ±1000 mm × {40, 80, 200, 400} steps/mm, each component of `(I_A, I_B)` matches the `f64::cos`/`f64::sin` reference within absolute epsilon ≤ 1e-6 (outputs are bounded in [-1, +1], so absolute is the meaningful bound; libm's `cosf`/`sinf` are documented at ≤ 1 ULP ≈ 1.2e-7). |
| Periodicity | For 100 random base positions, `phase_currents(p, s)` and `phase_currents(p + 4.0/s, s)` agree on each component within absolute epsilon ≤ 1e-6 (one electrical revolution = `4 / full_steps_per_mm` mm). |
| Unit-circle invariant | For 1000 random `(position, full_steps_per_mm)` pairs, `I_A² + I_B² ∈ [1.0 − ε, 1.0 + ε]` with `ε = 1e-5` (f32 trig identity, loose enough for f32 squaring + summation). |
| Direction inversion equivalence | `phase_currents_with_invert(p, s, true)` equals `phase_currents_with_invert(-p, s, false)` for arbitrary `p`, `s`, on each component within absolute epsilon ≤ 1e-6. |
| Continuity bound | For 100 random base positions and δ = 1 µm, `‖(I_A, I_B)|_{p+δ} − (I_A, I_B)|_p‖₂ ≤ (π/2) × full_steps_per_mm × δ × (1 + 1e-3)`. (Lipschitz bound from `d(cos θ, sin θ)/dp = (π/2) × full_steps_per_mm` — see §4; the 1e-3 slack covers second-order curvature over a finite δ. Catches accidental discontinuities introduced by future LUT/polynomial variants when those are added.) |
| Zero motion | `phase_currents(0.0, s) == (1.0, 0.0)` exactly for any `s` (rotor at electrical-zero → full current in coil A, none in coil B). |
| Negative `full_steps_per_mm` rejected by upstream callers | Not enforced by the kernel — `full_steps_per_mm` is configuration-derived and assumed positive. Documented in the function comment; no runtime check. |

`proptest` is already a dev-dependency in `runtime/Cargo.toml`, so the randomized sweeps can use it for compact test code.

## 6. Future integration path (informational, not in scope here)

When Step 10 fully lands, the integration looks roughly like:

1. The TIM5 ISR's per-tick path inside `runtime_handle_tick` (currently `step_state.update()` → `runtime_emit_step_pulses`) gains a branch for `StepMode::Modulated` steppers that opted into phase stepping:
   ```rust
   if step_mode == StepMode::Modulated {
       let (i_a, i_b) = phase::phase_currents_with_invert(
           motor_position_mm, full_steps_per_mm, invert_dir,
       );
       runtime_emit_phase_currents(motor_idx, i_a, i_b);   // C-side SPI write
   }
   // StepAccumulator continues running for bookkeeping (klippy's
   // get_mcu_position reads its counter). The pulse-emit side is
   // skipped for Modulated-with-phase steppers; the StepTime path
   // already short-circuits the TIM5 path entirely per the
   // step-time-scheduling spec §3.
   ```
2. `runtime_emit_phase_currents(motor_idx, i_a, i_b)` is a new C-side stub that scales `(i_a, i_b)` to TMC5160 XDIRECT register units and pushes them out via SPI.
3. The TMC5160 driver sets `GCONF.direct_mode = 1` at boot (or per-stepper at homing-mode-flip time), so the chip honors XDIRECT writes instead of running its internal microstep sequencer.
4. klippy's homing.py calls `runtime_set_step_mode(stepper_idx, StepMode::StepTime)` for the duration of sensorless homing on a phase-stepped axis (StallGuard requires the chip's internal sequencer to be active — see the step-time-scheduling spec §10 for the Prusa Buddy prior art).

None of those four pieces lands here. This spec adds only the math that step 1 above will call.

**Plain English:** This is the "smooth-waveform calculator" — one self-contained piece. The rest of phase stepping is plumbing (where to send the values, how to talk to the chip, when to switch modes for homing). Plumbing comes later. The math has to exist first so it can be unit-tested in isolation; once we trust it, integration is a straight wiring exercise.

## 7. Files touched

| File | Change |
|------|--------|
| `rust/runtime/src/phase.rs` | New. Contains the three pure functions + unit tests. |
| `rust/runtime/src/lib.rs` | One line: `pub mod phase;`. |

No other file is modified.

## 8. Non-goals

This spec deliberately does **not**:

- Choose a LUT or polynomial trig approximation strategy. That decision waits until an MCU profiling pass on real workload tells us libm cos/sin is or isn't fast enough on the H7 inside a 40 kHz ISR.
- Pre-allocate fields in `MotorConfig`, `SharedState`, or the wire protocol. Future PRs add `full_steps_per_mm` (or microsteps + the existing `steps_per_mm`, computed on the MCU) when integration needs it.
- Touch `engine.rs`, `state.rs`, `step.rs`, `step_time.rs`, `endstop.rs`, `runtime_tick.c`, `stepper.c`, klippy, or the wire protocol.
- Constrain the engine's eventual dispatch design between StepAccumulator-only vs phase-currents-only vs both-parallel for `StepMode::Modulated` steppers. The math doesn't care.

## 9. References

- `docs/superpowers/specs/2026-05-12-step-time-scheduling-design.md` — sibling spec that landed earlier today; introduces `StepMode::Modulated` / `StepMode::StepTime` and the `phase_stepping: 1` klippy knob. This spec's eventual integration site is inside that spec's `Modulated` path.
- `docs/kalico-rewrite/dependency-graph.md:154` — Layer-4 design note: *"Build a 'dumb' version that takes pre-computed step times and does phase modulation, validate the phase-stepping firmware on its own, then integrate with the trajectory evaluator. De-risks two complex things developing in parallel."* This spec is the "math on its own" half.
- `docs/research/tmc5160-open-loop-phase-stepping.md` — XDIRECT register (0x2D), GCONF.direct_mode bit, Prusa REFRESH_FREQ = 40 000 (matches our TIM5 rate).
- `rust/runtime/src/state.rs:42` — `StepMode` enum.
- `rust/nurbs/src/float.rs` — existing `libm::fmaf` / `sqrtf` / `fabsf` usage in the runtime tree; same pattern applies here.
- CLAUDE.md constraints honored:
  - *"Phase stepping with open loop steppers with BTT Octopus pro and similar (H723 chip)"* — this spec implements the math half of that.
  - *"Single source compiled f64 host / f32 MCU"* — kernel is `f32`, host tests compare against `f64::cos/sin` as ground truth.
  - *"No throwaway code beyond 1-2 lines"* — pure functions in a leaf module; integration path documented but explicitly deferred; zero scaffolding to delete later.
