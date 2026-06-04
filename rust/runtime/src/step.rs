pub const MAX_STEPS_PER_TICK_DEFAULT: i32 = 65536;

#[derive(Debug)]
pub struct StepResult {
    pub n_steps: i32,
}

/// Per-axis accumulator that converts a continuous motor position (mm) into
/// integer step pulses. Uses an `f64` accumulator internally — the H723 has a
/// hardware double-precision FPU, so there is no penalty, and it keeps the
/// sub-step residual accurate over millions of ticks.
#[derive(Debug, Clone, Copy)]
pub struct StepMotorState {
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
    pub fn debug_steps_per_mm(&self) -> f32 {
        self.steps_per_mm
    }

    pub fn debug_accumulator(&self) -> f64 {
        self.step_accumulator
    }

    pub fn new(steps_per_mm: f32) -> Self {
        Self {
            step_accumulator: 0.0,
            steps_per_mm,
            max_steps_per_tick: MAX_STEPS_PER_TICK_DEFAULT,
        }
    }

    pub fn seed(&mut self, motor_position_mm: f32) {
        self.step_accumulator = f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
    }

    /// Drop the sub-step residual without zeroing `steps_per_mm`. Used by
    /// `runtime_force_idle`: the motor's position is re-anchored by the host on
    /// the next segment push, so the cross-segment accumulator is meaningless;
    /// `Default::default()` must NOT be used here because it also zeros
    /// `steps_per_mm`, and the host doesn't re-call `configure()` after a flush.
    pub fn reset_accumulator(&mut self) {
        self.step_accumulator = 0.0;
    }

    /// Advance the accumulator to `motor_position_mm` and return the integer
    /// step delta for this tick. The `as i32` truncation retains the sub-step
    /// residual in the accumulator for the next tick.
    ///
    /// Returns `Err(())` if the burst cap (`max_steps_per_tick`) would be
    /// exceeded — the caller should raise a fault and halt the axis.
    #[allow(clippy::result_unit_err)]
    pub fn update(&mut self, motor_position_mm: f32) -> Result<StepResult, ()> {
        let new_pos_steps = f64::from(motor_position_mm) * f64::from(self.steps_per_mm);
        let delta = new_pos_steps - self.step_accumulator;
        #[allow(clippy::integer_division)]
        let n_steps = delta as i32;
        if n_steps.abs() > self.max_steps_per_tick {
            return Err(());
        }
        self.step_accumulator += f64::from(n_steps);
        Ok(StepResult { n_steps })
    }
}
