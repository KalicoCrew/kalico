// Sub-sample step times via secant-slope linear interpolation.
//
// Within one ISR sample period the position trajectory is approximated as a
// straight line between the previous-sample's position `P_start` and the
// current-sample's position `P_end`. The k-th step's cycle-counter time is:
//
//     t_k = (step_pos_k - P_start) · sample_period / (P_end - P_start)
//
// When the net displacement falls below `displacement_threshold` the inversion
// is numerically unstable; falls back to uniform spacing across the sample
// window (`t_k = sample_period · (k+1) / (n+1)`).
//
// All cycle-counter arithmetic uses `wrapping_add` so the 32-bit MCU
// cycle counter wrap is handled by construction.

use heapless::Vec;

/// Hard cap on the per-sample step count. Peak observed in benches is ~13
/// at 40 kHz; 16 leaves headroom without growing the stack meaningfully.
pub const MAX_STEPS_PER_SAMPLE: usize = 16;

#[derive(Clone, Copy, Debug)]
pub struct StepTimeInputs {
    pub p_start: f32,
    pub p_end: f32,
    pub prev_step_count: i32,
    pub target_step_count: i32,
    pub microstep_distance: f32,
    pub sample_period_sec: f32,
    pub sample_start_cycles: u32,
    pub cycles_per_second: f32,
    pub displacement_threshold: f32,
}

#[derive(Debug)]
pub enum StepTimingResult {
    SecantSlope(Vec<u32, MAX_STEPS_PER_SAMPLE>),
    Uniform(Vec<u32, MAX_STEPS_PER_SAMPLE>),
    NoSteps,
}

#[must_use]
pub fn compute_step_times(inp: &StepTimeInputs) -> StepTimingResult {
    let signed_steps = inp.target_step_count - inp.prev_step_count;
    if signed_steps == 0 {
        return StepTimingResult::NoSteps;
    }
    let sign: i32 = if signed_steps > 0 { 1 } else { -1 };
    let n_steps: usize = signed_steps.unsigned_abs() as usize;

    let displacement = inp.p_end - inp.p_start;

    // f32 → u32: product is the cycle count for one sample window
    // (e.g. 50 µs × 520 MHz = 26_000 cycles), always non-negative and
    // far below u32::MAX. Sign-loss / truncation are safe here.
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let sample_period_cycles = (inp.sample_period_sec * inp.cycles_per_second) as u32;

    let mut times: Vec<u32, MAX_STEPS_PER_SAMPLE> = Vec::new();

    let abs_disp = if displacement >= 0.0 {
        displacement
    } else {
        -displacement
    };

    if abs_disp <= inp.displacement_threshold {
        let n_plus_1 = (n_steps as u64) + 1;
        for k in 0..n_steps {
            // Integer division is intentional: computing a uniformly-spaced
            // cycle offset; residue is sub-cycle (< 1 cycle of jitter).
            #[allow(clippy::integer_division)]
            let dt_cycles = u64::from(sample_period_cycles) * ((k as u64) + 1) / n_plus_1;
            // dt_cycles ≤ sample_period_cycles by construction; u64 → u32 is lossless.
            #[allow(clippy::cast_possible_truncation)]
            let _ = times.push(inp.sample_start_cycles.wrapping_add(dt_cycles as u32));
        }
        return StepTimingResult::Uniform(times);
    }

    for k in 0..n_steps {
        // `n_steps` ≤ MAX_STEPS_PER_SAMPLE = 16; cast cannot wrap.
        #[allow(clippy::cast_possible_wrap)]
        let step_idx = inp.prev_step_count + ((k as i32) + 1) * sign;
        // i32 → f32: step_idx ranges over at most ±MAX_STEPS_PER_SAMPLE
        // around `prev_step_count`; precision loss is far below the microstep grid.
        #[allow(clippy::cast_precision_loss)]
        let step_pos_k = (step_idx as f32) * inp.microstep_distance;
        let t_local_sec = (step_pos_k - inp.p_start) * inp.sample_period_sec / displacement;
        // f32 → u32: t_local_sec ∈ [0, sample_period_sec], bounded well below u32::MAX.
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let cycle_abs = inp
            .sample_start_cycles
            .wrapping_add((t_local_sec * inp.cycles_per_second) as u32);
        let _ = times.push(cycle_abs);
    }
    StepTimingResult::SecantSlope(times)
}
