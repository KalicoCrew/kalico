//! `Engine` — per-axis evaluator + ISR state machine. Spec §3.1 / §4.2.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use crate::clock::{one_tick_cycles, TickCounter, WidenState};
use crate::curve_pool::{CurvePool, CurveView};
use crate::error::RuntimeError;
use crate::kinematics::{cartesian_xyz_with_e, corexy_with_e};
use crate::queue::SegmentQueue;
use crate::segment::{KinematicTag, Segment};
use crate::slot::{IsSlot, PaSlot};
use crate::state::TickState;
use crate::trace::{
    TraceRing, TraceSample, TRACE_FLAG_FAULT_MARKER, TRACE_FLAG_SEGMENT_END,
};

/// Bounded sub-tick boundary-loop iteration count.
/// Matches `Q_N` (queue capacity = 8) so a single tick can at most carry
/// across one full queue's worth of zero-duration segments before faulting.
const MAX_BOUNDARY_ITERS: u32 = 8;

/// `TraceRing` capacity used by the `Engine` ISR. Spec §3.1.
const TRACE_RING_N: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum RuntimeStatus {
    Idle = 0,
    Running = 1,
    Drained = 2,
    Fault = 3,
}

impl RuntimeStatus {
    #[inline]
    fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::Running,
            2 => Self::Drained,
            _ => Self::Fault,
        }
    }
}

#[allow(missing_debug_implementations)] // P, I are open trait bounds; ISR-internal struct.
pub struct Engine<P: PaSlot, I: IsSlot> {
    current: Option<Segment>,
    last_motors: [f32; 3],     // last-known-good motor positions (used in FAULT marker)
    pa_slot: P,
    is_slot: I,
    one_tick_cycles_value: u64,
    pub(crate) widen_state: WidenState,
    pub(crate) status: AtomicU8,
    pub(crate) last_error: AtomicI32,
    pub(crate) tick_counter: TickCounter,
}

impl<P: PaSlot + Default, I: IsSlot + Default> Engine<P, I> {
    pub fn new(clock_freq: u32) -> Self {
        Self {
            current: None,
            last_motors: [0.0; 3],
            pa_slot: P::default(),
            is_slot: I::default(),
            one_tick_cycles_value: u64::from(one_tick_cycles(clock_freq)),
            widen_state: WidenState::default(),
            status: AtomicU8::new(RuntimeStatus::Idle as u8),
            last_error: AtomicI32::new(0),
            tick_counter: TickCounter::new(),
        }
    }
}

// Engine::Default impl for tests where slot types implement Default.
// Production callers must use ::new(clock_freq) — Default hardcodes 520 MHz.
#[cfg(test)]
impl<P: PaSlot + Default, I: IsSlot + Default> Default for Engine<P, I> {
    fn default() -> Self {
        // H723 Klipper Kconfig default is 520 MHz (src/stm32/Kconfig). Tests using
        // Default get this; tests requiring a specific value should call ::new() directly.
        Self::new(520_000_000)
    }
}

impl<P: PaSlot, I: IsSlot> Engine<P, I> {
    pub fn status(&self) -> RuntimeStatus {
        RuntimeStatus::from_u8(self.status.load(Ordering::Acquire))
    }

    pub fn last_error(&self) -> i32 {
        self.last_error.load(Ordering::Acquire)
    }

    pub fn tick_counter(&self) -> u32 {
        self.tick_counter.snapshot()
    }

    /// Foreground-callable: read the most recent widened `now: u64` so the
    /// producer-protocol can preserve epoch across a TIM5 disable→enable cycle.
    /// SAFETY: ISR is the sole writer of `WidenState`; foreground reads it only
    /// while ISR is disabled (between `kalico_h7_disable_tim5` and re-enable).
    pub fn last_widened_now(&self) -> u64 {
        // Reconstruct from the ISR-private static. In the real implementation,
        // this returns `widen_state.high | (widen_state.last_low as u64)` —
        // accessed under the spec §4.7 SAFETY invariant that ISR is paused.
        self.widen_state.high | u64::from(self.widen_state.last_low)
    }

    /// Latch FAULT and emit one fault marker sample (last-known-good motors,
    /// not zero, so host plots show the fault in context). ISR self-disables
    /// the timer in the C wrapper after this returns.
    /// `segment_id` is passed explicitly by the call site (decoupled from
    /// `self.current`). Pass `0` only if no segment was active — producer-side
    /// segment ids start at 1, so `segment_id == 0` ⇒ fault before any segment
    /// was active.
    fn latch_fault(
        &mut self,
        code: RuntimeError,
        segment_id: u32,
        now: u64,
        trace: &mut TraceRing<TRACE_RING_N>,
    ) {
        self.last_error.store(i32::from(code), Ordering::Release);
        self.status.store(RuntimeStatus::Fault as u8, Ordering::Release);
        let _ = trace.try_emit(TraceSample {
            tick: now,
            motor_a: self.last_motors[0],
            motor_b: self.last_motors[1],
            motor_e: self.last_motors[2],
            segment_id,
            flags: TRACE_FLAG_FAULT_MARKER,
            _pad: [0; 7],
        });
    }

    /// Single 40 kHz tick. Spec §4.2 step ordering — must remain stable.
    ///
    /// `now` is the widened u64 cycle count — caller (FFI shim or test)
    /// is responsible for widening via `WidenState`.
    pub fn tick(
        &mut self,
        now: u64,
        queue: &mut SegmentQueue,
        pool: &CurvePool,
        trace: &mut TraceRing<TRACE_RING_N>,
    ) -> Result<(), RuntimeError> {
        if self.status() == RuntimeStatus::Fault {
            return Err(RuntimeError::FaultLatched);
        }

        // Step 1 + 2: queue + idle check, segment activation. See spec §4.2.
        // Idle path with §4.4 ISR-disable protocol.
        // (Caller observes status == Idle and clears CR1.CEN.)
        let Some(current) = self.current.take() else {
            self.status.store(RuntimeStatus::Idle as u8, Ordering::Release);
            // Re-check queue with Acquire — race against producer's enqueue.
            if !queue.is_empty() {
                self.current = queue.try_pop();
                if let Some(seg) = self.current {
                    self.status.store(RuntimeStatus::Running as u8, Ordering::Release);
                    // Fall through with the freshly dequeued segment.
                    return self.tick_with_current(seg, now, queue, pool, trace);
                }
            }
            return Ok(());
        };

        self.tick_with_current(current, now, queue, pool, trace)
    }

    fn tick_with_current(
        &mut self,
        mut current: Segment,
        now: u64,
        queue: &mut SegmentQueue,
        pool: &CurvePool,
        trace: &mut TraceRing<TRACE_RING_N>,
    ) -> Result<(), RuntimeError> {
        // Step 3: sub-tick boundary loop. Spec §4.2 step 3 — bounded by queue depth.
        let mut iters = 0u32;
        let mut t_segment = now.saturating_sub(current.t_start);
        while t_segment >= current.duration() {
            iters += 1;
            if iters > MAX_BOUNDARY_ITERS {
                let seg_id = current.id;
                self.current = Some(current);
                self.latch_fault(RuntimeError::BoundaryLoopExhausted, seg_id, now, trace);
                return Err(RuntimeError::BoundaryLoopExhausted);
            }
            let delta_t = t_segment - current.duration();
            // Drop current; advance to next.
            let Some(next) = queue.try_pop() else {
                // No next segment — drained. Set status; return.
                self.current = None;
                self.status.store(RuntimeStatus::Drained as u8, Ordering::Release);
                return Ok(());
            };
            current = next;
            current.t_start = now.saturating_sub(delta_t);
            t_segment = delta_t;
        }
        // Step 4: curve evaluation. Spec invariant: segments are time-parameterized.
        let Some(curve_view) = pool.resolve(current.curve) else {
            self.latch_fault(RuntimeError::InvalidHandle, current.id, now, trace);
            return Err(RuntimeError::InvalidHandle);
        };
        let duration = current.duration().max(1) as f32;  // saturating_sub avoids 0
        let u = (t_segment as f32 / duration).clamp(0.0, 1.0);
        let Ok(xyz_e) = nurbs_eval_3d(&curve_view, u) else {
            self.latch_fault(RuntimeError::InvalidCurve, current.id, now, trace);
            return Err(RuntimeError::InvalidCurve);
        };

        // Step 5: NaN/Inf check. Spec §5.4 — necessary even with producer-side
        // validation (NaN can arise from finite inputs).
        if !xyz_e.iter().all(|x: &f32| x.is_finite()) {
            self.latch_fault(RuntimeError::NaNOrInfFromEval, current.id, now, trace);
            return Err(RuntimeError::NaNOrInfFromEval);
        }

        // Step 6: kinematic transform. Pipeline order: kinematics BEFORE PA/IS.
        let motors = match current.kinematics {
            KinematicTag::CoreXyAndE => corexy_with_e(xyz_e),
            KinematicTag::CartesianXyzAndE => cartesian_xyz_with_e(xyz_e),
        };

        // Step 7: slot pipeline. Noop ZSTs at Step 5.
        let dt = 1.0 / (crate::clock::TICK_RATE_HZ as f32);
        let mut state = TickState { dt, xyz_e, motors };
        self.pa_slot.apply(&mut state);
        self.is_slot.apply(&mut state);

        // Step 8: trace emit.
        let next_t_segment = t_segment.saturating_add(self.one_tick_cycles_value);
        let segment_end_flag = if next_t_segment >= current.duration() {
            TRACE_FLAG_SEGMENT_END
        } else {
            0
        };
        let _ = trace.try_emit(TraceSample {
            tick: now,
            motor_a: state.motors[0],
            motor_b: state.motors[1],
            motor_e: state.motors[2],
            segment_id: current.id,
            flags: segment_end_flag,
            _pad: [0; 7],
        });
        self.last_motors = state.motors;
        self.current = Some(current);

        // Step 9: tick counter heartbeat.
        self.tick_counter.increment();

        // Step 10: status update.
        self.status.store(RuntimeStatus::Running as u8, Ordering::Release);
        Ok(())
    }
}

/// Wrapper around `nurbs::eval::vector_eval` for f32 3D rational NURBS.
///
/// Uses `nurbs::VectorNurbsRef` (the borrowed view type) per the actual
/// Layer-0 API at `rust/nurbs/src/vector.rs` (verified during plan review).
fn nurbs_eval_3d(curve: &CurveView<'_>, u: f32) -> Result<[f32; 3], ()> {
    use nurbs::VectorNurbsRef;

    // Actual API: try_new(degree: u8, knots: &[T], control_points: &[[T; N]],
    //                     weights: Option<&[T]>) -> Result<Self, ConstructError>.
    // Returns owning struct over the borrowed slices.
    let view = VectorNurbsRef::<f32, 3>::try_new(
        curve.degree,
        curve.knots,
        curve.control_points,
        Some(curve.weights),
    )
    .map_err(|_| ())?;
    // vector_eval returns [T; N] directly — no Result wrapper.
    Ok(nurbs::eval::vector_eval(&view, u))
}
