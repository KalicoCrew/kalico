//! Sub-sample step times via secant-slope linear interpolation.
//!
//! Within one ISR sample period the position trajectory is approximated as a
//! straight line between the previous-sample's position `P_start` and the
//! current-sample's position `P_end`. The k-th step's cycle-counter time is
//! then the inverse-linear solution:
//!
//! ```text
//!     t_k = (step_pos_k - P_start) · sample_period / (P_end - P_start)
//! ```
//!
//! When the sample's net displacement falls below
//! `displacement_threshold` the inversion is numerically unstable; the
//! function falls back to uniform spacing across the sample window
//! (`t_k = sample_period · (k+1) / (n+1)`).
//!
//! All cycle-counter arithmetic uses `wrapping_add` so the 32-bit MCU
//! cycle counter wrap is handled by construction.

use heapless::Vec;

/// Hard cap on the per-sample step count. At 40 kHz sample rate the peak
/// observed in benches is ~13; 16 leaves headroom without growing the
/// stack frame meaningfully (16 × 4 B = 64 B).
pub const MAX_STEPS_PER_SAMPLE: usize = 16;

/// Inputs to `compute_step_times`.
///
/// All positions are in microsteps' physical units (mm or rad) and must
/// share the same scale as `microstep_distance`.
#[derive(Clone, Copy, Debug)]
pub struct StepTimeInputs {
    /// Position at the start of the sample window.
    pub p_start: f32,
    /// Position at the end of the sample window.
    pub p_end: f32,
    /// Signed microstep counter at the start of the sample window.
    pub prev_step_count: i32,
    /// Signed microstep counter at the end of the sample window.
    pub target_step_count: i32,
    /// Physical distance per microstep (mm or rad).
    pub microstep_distance: f32,
    /// Sample period in seconds.
    pub sample_period_sec: f32,
    /// Cycle counter value at the start of the sample window.
    pub sample_start_cycles: u32,
    /// MCU core clock in Hz (e.g. `520_000_000` on the H723).
    pub cycles_per_second: f32,
    /// `|P_end - P_start|` below this triggers the uniform fallback.
    pub displacement_threshold: f32,
}

/// Result of `compute_step_times`.
#[derive(Debug)]
pub enum StepTimingResult {
    /// Secant-slope interpolation produced these absolute cycle-counter
    /// times (already offset by `sample_start_cycles`).
    SecantSlope(Vec<u32, MAX_STEPS_PER_SAMPLE>),
    /// Net displacement below threshold → uniform fallback was used.
    Uniform(Vec<u32, MAX_STEPS_PER_SAMPLE>),
    /// `prev_step_count == target_step_count` — no steps fire this sample.
    NoSteps,
}

/// Compute per-step cycle-counter times within one sample window.
///
/// See module docs for the formula. The output vector length equals
/// `|target_step_count - prev_step_count|`, which the caller has already
/// bounded by `MAX_STEPS_PER_SAMPLE` upstream.
#[must_use]
pub fn compute_step_times(inp: &StepTimeInputs) -> StepTimingResult {
    let signed_steps = inp.target_step_count - inp.prev_step_count;
    if signed_steps == 0 {
        return StepTimingResult::NoSteps;
    }
    let sign: i32 = if signed_steps > 0 { 1 } else { -1 };
    let n_steps: usize = signed_steps.unsigned_abs() as usize;

    let displacement = inp.p_end - inp.p_start;

    // f32 → u32: the product is the cycle count for one sample window
    // (e.g. 50 µs × 520 MHz = 26_000 cycles), always non-negative and
    // far below u32::MAX in any sane configuration. Sign-loss / truncation
    // are safe here.
    #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
    let sample_period_cycles =
        (inp.sample_period_sec * inp.cycles_per_second) as u32;

    let mut times: Vec<u32, MAX_STEPS_PER_SAMPLE> = Vec::new();

    let abs_disp = if displacement >= 0.0 {
        displacement
    } else {
        -displacement
    };

    if abs_disp <= inp.displacement_threshold {
        // Uniform fallback. u64 intermediate to avoid overflow on the
        // (sample_period_cycles · (k+1)) product.
        let n_plus_1 = (n_steps as u64) + 1;
        for k in 0..n_steps {
            // Integer division here is intentional: we're computing a
            // uniformly-spaced cycle offset and the residue is sub-cycle
            // (< 1 cycle of jitter), well below the timing precision we
            // care about.
            #[allow(clippy::integer_division)]
            let dt_cycles =
                u64::from(sample_period_cycles) * ((k as u64) + 1) / n_plus_1;
            // dt_cycles ≤ sample_period_cycles by construction, so the
            // u64 → u32 truncation is lossless.
            #[allow(clippy::cast_possible_truncation)]
            let _ = times.push(
                inp.sample_start_cycles.wrapping_add(dt_cycles as u32),
            );
        }
        return StepTimingResult::Uniform(times);
    }

    // Secant-slope branch.
    for k in 0..n_steps {
        // `n_steps` is bounded by `MAX_STEPS_PER_SAMPLE = 16`, so the
        // `k as i32` cast cannot wrap on any supported target.
        #[allow(clippy::cast_possible_wrap)]
        let step_idx =
            inp.prev_step_count + ((k as i32) + 1) * sign;
        // i32 → f32: step_idx ranges over at most ±MAX_STEPS_PER_SAMPLE
        // around `prev_step_count`; precision loss for the f32 multiply is
        // far below the microstep grid.
        #[allow(clippy::cast_precision_loss)]
        let step_pos_k = (step_idx as f32) * inp.microstep_distance;
        let t_local_sec =
            (step_pos_k - inp.p_start) * inp.sample_period_sec / displacement;
        // f32 → u32: t_local_sec is in [0, sample_period_sec], so the
        // product with cycles_per_second is in [0, sample_period_cycles],
        // bounded well below u32::MAX.
        #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
        let cycle_abs = inp
            .sample_start_cycles
            .wrapping_add((t_local_sec * inp.cycles_per_second) as u32);
        let _ = times.push(cycle_abs);
    }
    StepTimingResult::SecantSlope(times)
}
