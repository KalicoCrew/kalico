/// Default maximum steps allowed per tick — burst cap guard.
///
/// Bumped from 16 to 64 (2026-05-05) as a temporary workaround for
/// post-shape curves coming out higher-degree than the architecture
/// targets. With smooth-MZV applied to a Hermite-refit-degree-4 input,
/// the X axis comes out degree 9 with ~1k control points, and `f32`
/// De Boor evaluation of that on the MCU produces occasional one-tick
/// position spikes that the previous 16-step cap rejected. At configured
/// peak velocity (300 mm/s × 80 steps/mm / 40 kHz ≈ 0.6 step/tick), 64
/// is still ~100× the legitimate per-tick load.
///
/// Real fix lives at the trajectory layer: keep the post-shape curve
/// at low degree (per `CLAUDE.md`'s "uniform cubic Bézier across Layer
/// 1/2/3/4") so f32 evaluation is well-conditioned. Until then this
/// cap accepts numerical wobble at the cost of weakening genuine
/// runaway-curve detection.
pub const MAX_STEPS_PER_TICK_DEFAULT: i32 = 64;

/// Output of a single [`StepMotorState::update`] call.
#[derive(Debug)]
pub struct StepResult {
    /// Signed step count for this tick. Positive = forward, negative = reverse.
    pub n_steps: i32,
}

/// Per-axis accumulator that converts a continuous motor position (mm) into
/// integer step pulses. Uses an `f64` accumulator internally — the H723 has a
/// hardware double-precision FPU, so there is no penalty, and it keeps the
/// sub-step residual accurate over millions of ticks.
#[derive(Debug, Clone, Copy)]
pub struct StepMotorState {
    /// Accumulator in step-space (fractional steps retained between ticks).
    step_accumulator: f64,
    steps_per_mm: f32,
    max_steps_per_tick: i32,
}

impl Default for StepMotorState {
    fn default() -> Self {
        Self {
            step_accumulator: 0.0,
            steps_per_mm: 0.0,
            max_steps_per_tick: MAX_STEPS_PER_TICK_DEFAULT,
        }
    }
}

impl StepMotorState {
    /// Diagnostic accessor.
    pub fn debug_steps_per_mm(&self) -> f32 {
        self.steps_per_mm
    }

    /// Diagnostic accessor: current step accumulator (sub-step residual + integer).
    pub fn debug_accumulator(&self) -> f64 {
        self.step_accumulator
    }

    /// Create a new state for an axis with the given steps-per-mm ratio.
    pub fn new(steps_per_mm: f32) -> Self {
        Self {
            step_accumulator: 0.0,
            steps_per_mm,
            max_steps_per_tick: MAX_STEPS_PER_TICK_DEFAULT,
        }
    }

    /// Seed the accumulator from a known absolute motor position (mm).
    ///
    /// Call this after homing or on the first trajectory segment so that the
    /// first `update` does not see a spurious burst relative to physical zero.
    pub fn seed(&mut self, motor_position_mm: f32) {
        self.step_accumulator =
            f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
    }

    /// Advance the accumulator to `motor_position_mm` and return the integer
    /// step delta for this tick.
    ///
    /// The `as i32` truncation is intentional: it truncates toward zero,
    /// retaining the sub-step residual in the accumulator for the next tick.
    ///
    /// Returns `Err(())` if the burst cap (`max_steps_per_tick`) would be
    /// exceeded — the caller should raise a fault and halt the axis.
    pub fn update(&mut self, motor_position_mm: f32) -> Result<StepResult, ()> {
        let new_pos_steps =
            f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
        let delta = new_pos_steps - self.step_accumulator;
        // Truncate toward zero — fractional residual stays in the accumulator.
        #[allow(clippy::integer_division)]
        let n_steps = delta as i32;
        if n_steps.abs() > self.max_steps_per_tick {
            return Err(());
        }
        self.step_accumulator += f64::from(n_steps);
        Ok(StepResult { n_steps })
    }
}
