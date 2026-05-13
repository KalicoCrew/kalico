//! Per-motor step-time producer. Drains step times from the active curve
//! into the motor's `StepRing` via Newton iteration. Called from the
//! producer Klipper timer in the MCU runtime (event-driven; see
//! `runtime_tick.c`).
//!
//! `producer_step` is a pure function — it takes rings, per-motor state,
//! per-motor curve closures, and produces ring entries. The engine (Task 5)
//! wraps this with the curve-queue / pool / kinematics integration.
//!
//! Spec: docs/superpowers/specs/2026-05-14-step-emission-architecture-design.md §3.4

use crate::step_ring::StepRing;
use crate::step_time::{compute_next_step_time, StepTimeQuery, StepTimeResult};

/// Per-motor producer state. Newton resume bookkeeping between batch
/// calls within a single curve.
#[derive(Debug)]
pub struct ProducerState {
    step_distance: f64,
    /// Resume point in normalized u-domain. `None` when no curve is
    /// currently being filled.
    t_resume: Option<f64>,
    /// Motor step counter at curve start (for absolute target math).
    step_at_curve_start: i32,
    /// How many steps have been pushed for the current curve so far
    /// (signed: cumulative direction-aware count).
    steps_pushed_this_curve: i32,
}

impl ProducerState {
    pub const fn new(step_distance: f64) -> Self {
        Self {
            step_distance,
            t_resume: None,
            step_at_curve_start: 0,
            steps_pushed_this_curve: 0,
        }
    }

    /// True iff no curve is currently being filled.
    pub const fn is_idle(&self) -> bool {
        self.t_resume.is_none()
    }

    /// Start a fresh curve for this motor. `step_at_start` is the motor's
    /// integer step counter at the curve's u=0; subsequent step targets
    /// are `(step_at_start + n_pushed_so_far) * step_distance`.
    pub fn start_curve(&mut self, step_at_start: i32) {
        self.t_resume = Some(0.0);
        self.step_at_curve_start = step_at_start;
        self.steps_pushed_this_curve = 0;
    }

    pub fn clear(&mut self) {
        self.t_resume = None;
        self.steps_pushed_this_curve = 0;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProducerTickResult {
    /// At least one motor has more work but ran out of batch budget or
    /// ring space. Producer should be rescheduled.
    WorkPending,
    /// All motors are idle (no current curve and the caller could not
    /// provide a next one). Producer should wait for a kick.
    AllIdle,
}

/// Fill rings for every motor that has a curve closure.
///
/// The caller provides per-motor closures returning `(pos, vel, accel)`
/// in motor frame; `None` means "no curve available for this motor right
/// now" (skip it). Returns whether more work is pending overall.
///
/// `curve_t_start` and `curve_duration` are absolute MCU clock cycles
/// and clock cycles respectively. They are constant for the duration of
/// one curve fill (the engine constructs them when it starts a motor's
/// curve).
///
/// **Auto-start semantics**: if a motor's `ProducerState` is `idle` AND
/// it has a curve closure provided, this function calls `start_curve(0)`
/// before filling. The engine (Task 5) will replace this with the
/// motor's true current step count.
pub fn producer_step<F>(
    rings: &mut [&mut StepRing],
    states: &mut [&mut ProducerState],
    evals: &mut [Option<&F>],
    curve_t_start: &[u64],
    curve_duration: &[u64],
    batch_cap: u32,
) -> ProducerTickResult
where
    F: Fn(f32) -> (f64, f64, f64),
{
    debug_assert_eq!(rings.len(), states.len());
    debug_assert_eq!(rings.len(), evals.len());
    debug_assert_eq!(rings.len(), curve_t_start.len());
    debug_assert_eq!(rings.len(), curve_duration.len());

    let mut any_work_pending = false;

    // Zip-based iteration keeps the four parallel slices in lockstep without
    // bare `[i]` indexing — the deny(clippy::indexing_slicing) lint at the
    // crate root rejects raw indexing in lib code.
    let motors = rings
        .iter_mut()
        .zip(states.iter_mut())
        .zip(evals.iter_mut())
        .zip(curve_t_start.iter())
        .zip(curve_duration.iter());

    for ((((ring, state), eval_slot), &t_start), &duration) in motors {
        let Some(eval) = eval_slot.as_ref() else { continue; };
        let state: &mut ProducerState = &mut **state;
        let ring: &mut StepRing = &mut **ring;

        if state.is_idle() {
            state.start_curve(0);
        }

        let mut filled = 0_u32;
        let duration_f64 = duration as f64;

        while filled < batch_cap && ring.space() > 0 {
            let q = StepTimeQuery {
                eval: *eval,
                step_distance: state.step_distance,
                current_step: state
                    .step_at_curve_start
                    .wrapping_add(state.steps_pushed_this_curve),
                t_curr: state.t_resume.unwrap_or(0.0),
                t_segment_end: 1.0,
            };
            match compute_next_step_time(&q) {
                StepTimeResult::NextAt { t, dir } => {
                    let dt_cycles = (t * duration_f64) as u64;
                    let abs_cycles = t_start.saturating_add(dt_cycles);
                    ring.push(abs_cycles as u32, dir);
                    state.t_resume = Some(t);
                    state.steps_pushed_this_curve = state
                        .steps_pushed_this_curve
                        .saturating_add(i32::from(dir));
                    filled += 1;
                }
                StepTimeResult::SegmentExhausted => {
                    state.clear();
                    break;
                }
            }
        }
        if !state.is_idle() {
            any_work_pending = true;
        }
    }

    if any_work_pending {
        ProducerTickResult::WorkPending
    } else {
        ProducerTickResult::AllIdle
    }
}
