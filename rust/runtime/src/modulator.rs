//! Phase-stepping output-stage state and computation, decoupled from the
//! SPI/trace plumbing for unit testability.
//!
//! The hot-path `runtime_modulated_tick` (engine.rs) gates per-motor on
//! whether a `PhaseDirectModulator` is configured for that motor. The
//! modulator computes the TMC5160 `mscount`, the `(coil_A, coil_B)`
//! current pair via the identity LUT, and a per-tick step delta used to
//! advance `SharedState::stepper_counts` so host position queries and
//! homing snapshots continue to work for phase-stepped axes.

use crate::phase_lut::{self, MOTOR_PERIOD};
use crate::step::MAX_STEPS_PER_TICK_DEFAULT;

/// Per-tick output from `PhaseDirectModulator::compute`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PhaseTickResult {
    /// Electrical-cycle position, 0..MOTOR_PERIOD-1.
    pub mscount: u16,
    /// Coil-A current setpoint, signed 9-bit (-256..255 representable).
    pub i_a: i16,
    /// Coil-B current setpoint, signed 9-bit (-256..255 representable).
    pub i_b: i16,
    /// Direction sense for this tick, +1 / 0 / -1. Sticky across sub-
    /// microstep ticks (see field docs on `last_direction`).
    pub direction: i8,
    /// Integer microstep delta vs the previous tick. Used by the engine to
    /// advance `SharedState::stepper_counts`. May be negative.
    pub steps_delta: i32,
}

/// Per-motor phase-stepping state. Hot-path callers create one per phase-
/// stepped motor at `configure_axes` time and call `compute(motor_pos_mm)`
/// every tick.
#[derive(Debug, Clone, Copy)]
pub struct PhaseDirectModulator {
    steps_per_mm: f32,
    /// f64 accumulator of motor position in microstep units, advanced each
    /// tick by `steps_delta`. The fractional residual stays in the
    /// accumulator the same way `StepMotorState::step_accumulator` does.
    step_accumulator: f64,
    /// Last reported direction sense. Held across ticks where the magnitude
    /// of the per-tick advance is below the direction-update threshold —
    /// prevents flicker at sub-microstep velocities.
    last_direction: i8,
    /// True after the first `compute` call. The first call seeds the
    /// accumulator without reporting a delta (matches `StepMotorState::seed`
    /// semantics — no spurious burst from physical zero).
    seeded: bool,
    /// Max integer microsteps per tick before the engine should raise
    /// `STEP_BURST_EXCEEDED`. Mirrors `StepMotorState::max_steps_per_tick`.
    pub max_steps_per_tick: i32,
}

/// Minimum |advance|, in microsteps per tick, that updates `last_direction`.
/// Below this, the previously-latched direction sticks. 0.5 microsteps is
/// half a microstep — the smallest delta that's unambiguously directional
/// once rounded.
const DIRECTION_UPDATE_THRESHOLD: f64 = 0.5;

impl PhaseDirectModulator {
    pub fn new(steps_per_mm: f32) -> Self {
        Self {
            steps_per_mm,
            step_accumulator: 0.0,
            last_direction: 0,
            seeded: false,
            // Single source of truth — same cap as `StepMotorState`. 192
            // tripped immediately on the same cross-segment planner
            // discontinuities that forced step.rs's 2026-05-13 bump.
            max_steps_per_tick: MAX_STEPS_PER_TICK_DEFAULT,
        }
    }

    /// First-tick seed without reporting a step delta. Matches
    /// `StepMotorState::seed` — called once after configure / homing-snap.
    pub fn seed(&mut self, motor_position_mm: f32) {
        self.step_accumulator =
            f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
        self.seeded = true;
        self.last_direction = 0;
    }

    /// Per-tick computation. Returns the mscount, current setpoints,
    /// direction, and steps delta for the engine to apply to
    /// `stepper_counts`.
    ///
    /// Returns `Err(())` if the burst cap (`max_steps_per_tick`) would be
    /// exceeded — caller fault-handles (raise `STEP_BURST_EXCEEDED`).
    /// Matches `StepMotorState::update` semantics: on `Err`, the
    /// accumulator is NOT advanced, so retrying after the caller resets
    /// the cap is safe.
    pub fn compute(
        &mut self,
        motor_position_mm: f32,
    ) -> Result<PhaseTickResult, ()> {
        let new_pos_steps =
            f64::from(motor_position_mm) * f64::from(self.steps_per_mm);

        if !self.seeded {
            self.step_accumulator = new_pos_steps;
            self.seeded = true;
            // Seed: report zero delta and direction from rest.
            let mscount = wrap_mscount(new_pos_steps);
            let (i_a, i_b) = phase_lut::lookup(mscount, 0);
            return Ok(PhaseTickResult {
                mscount,
                i_a,
                i_b,
                direction: 0,
                steps_delta: 0,
            });
        }

        let delta = new_pos_steps - self.step_accumulator;

        // Integer steps delta: truncate toward zero, residual stays in the
        // accumulator. Same semantics as `StepMotorState::update`.
        let steps_delta = delta as i32;

        // Burst cap: bail BEFORE advancing the accumulator or latching a
        // new direction, so the caller can fault-handle and retry once
        // the cap is reset. Same Err(()) shape as `StepMotorState::update`.
        if steps_delta.abs() > self.max_steps_per_tick {
            return Err(());
        }

        // Direction: update only when the per-tick advance is clearly
        // directional. Otherwise the previous direction sticks. This
        // matches the architectural reviewer's "phase-advance accumulator"
        // recommendation — prevents `sign(0)`-driven flicker at sub-
        // microstep velocities.
        if delta.abs() >= DIRECTION_UPDATE_THRESHOLD {
            self.last_direction = if delta > 0.0 { 1 } else { -1 };
        }

        self.step_accumulator += f64::from(steps_delta);

        // mscount comes from the *accumulator* (the rounded electrical-
        // cycle position), not from raw motor_position_mm. This ensures
        // mscount and stepper_counts stay phase-coherent across the
        // fractional residual.
        let mscount = wrap_mscount(self.step_accumulator);
        let (i_a, i_b) = phase_lut::lookup(mscount, self.last_direction);

        Ok(PhaseTickResult {
            mscount,
            i_a,
            i_b,
            direction: self.last_direction,
            steps_delta,
        })
    }

    /// Reset the fractional residual without dropping `steps_per_mm`.
    /// Mirrors `StepMotorState::reset_accumulator` — used by
    /// `runtime_force_idle` after a flush.
    pub fn reset_accumulator(&mut self) {
        self.step_accumulator = 0.0;
        self.last_direction = 0;
        self.seeded = false;
    }
}

#[inline]
fn wrap_mscount(accumulator_steps: f64) -> u16 {
    let rounded = libm::round(accumulator_steps) as i64;
    let modulus = MOTOR_PERIOD as i64;
    let wrapped = ((rounded % modulus) + modulus) % modulus;
    wrapped as u16
}
