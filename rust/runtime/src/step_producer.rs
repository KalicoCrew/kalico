//! Per-motor producer state (step-distance, curve resume point, accumulator)
//! and the tick-result enum returned by `Engine::producer_step`.
//!
//! Was previously a full module containing a standalone Newton-fill function
//! used by an early test scaffold; the production path inlined its variant
//! inside `Engine::producer_step` (per the T5 implementer's note), making
//! the standalone function dead code. The standalone function was retired
//! in commit `f88dea94c`; only the types — which the rest of the runtime
//! depends on — survive in this module.

/// Per-motor state carried across producer ticks. Initialised on segment
/// activation, mutated as Newton-fills produce step times for that segment.
#[derive(Debug)]
pub struct ProducerState {
    /// Step distance in mm (1 / steps_per_mm). Set once at motor configure.
    step_distance: f64,
    /// Resume point in normalized segment u-domain, ∈ [0, 1]. Advances
    /// past each pushed step's u-coordinate. `None` when no segment is
    /// being actively filled (motor is idle on the producer side).
    t_resume: Option<f64>,
    /// Integer-step value at curve(0) — anchors the absolute step target
    /// to the curve's coordinate frame. Seeded from `eval(0) / step_distance`
    /// when a curve activates (engine.rs).
    step_at_curve_start: i32,
    /// Steps pushed into the ring so far for the current curve. `+dir` per
    /// step. Resets when a new curve activates.
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

    /// Activate a curve for this motor. Seeds the step counter baseline
    /// from `eval(0) / step_distance` (per the curve's coordinate frame),
    /// which the caller computes and passes in.
    pub fn start_curve(&mut self, step_at_curve_start: i32) {
        self.t_resume = Some(0.0);
        self.step_at_curve_start = step_at_curve_start;
        self.steps_pushed_this_curve = 0;
    }

    pub fn clear(&mut self) {
        self.t_resume = None;
        self.steps_pushed_this_curve = 0;
    }

    // ─── Resume-state accessors used by `Engine::producer_step` ──────────
    //
    // Crate-private: external crates (kalico-c-api, motion-bridge, …)
    // must not reach into per-motor Newton resume state. The engine is
    // the sole authorised caller.

    #[inline]
    pub(crate) fn step_distance(&self) -> f64 {
        self.step_distance
    }

    #[inline]
    pub(crate) fn t_resume(&self) -> Option<f64> {
        self.t_resume
    }

    #[inline]
    pub(crate) fn set_t_resume(&mut self, v: Option<f64>) {
        self.t_resume = v;
    }

    #[inline]
    pub(crate) fn step_at_curve_start(&self) -> i32 {
        self.step_at_curve_start
    }

    #[inline]
    pub(crate) fn steps_pushed_this_curve(&self) -> i32 {
        self.steps_pushed_this_curve
    }

    #[inline]
    pub(crate) fn bump_steps_pushed(&mut self, by: i32) {
        self.steps_pushed_this_curve = self.steps_pushed_this_curve.saturating_add(by);
    }
}

/// Outcome of one producer tick over all motors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProducerTickResult {
    /// At least one motor made progress (filled ≥1 ring entry OR finished
    /// a curve). The producer Klipper timer should self-reschedule for
    /// another batch ASAP.
    WorkPending,
    /// No motor made progress this call (all rings full, all curves done,
    /// or no segments queued). Producer should wait for an external kick
    /// (push_segment or consumer low-water hook).
    AllIdle,
}
