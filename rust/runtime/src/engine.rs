// Per-axis piece-ring walker engine.
//
// Ring layout: each axis carries a `RingDescriptor` — a borrow-free set of
// bookkeeping integers — and all mutation goes through `&mut [PieceEntry]`
// passed explicitly into every operation. `PieceRing<'a>` is kept for host
// unit tests; `RingDescriptor` is used exclusively by the engine.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use crate::clock::TickCounter;
use crate::error::{KALICO_ERR_INVALID_ARG, KALICO_ERR_RING_FULL, KALICO_OK};
use crate::fault_sink::FaultSink;
use crate::piece_ring::PieceEntry;
use crate::state::{MAX_STEPPER_OIDS, SharedState};
use crate::step::StepMotorState;
use crate::stepping_state::{AxisState, MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

pub use crate::stepping_state::N_AXES;

pub(crate) struct SharedFaultSink<'a> {
    pub shared: &'a SharedState,
}

impl FaultSink for SharedFaultSink<'_> {
    #[inline]
    fn piece_start_in_past(&self, axis_idx: usize, deficit_us: u32) {
        crate::fault_helpers::raise_piece_start_in_past(self.shared, axis_idx, deficit_us);
    }
}

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
    pub fn from_u8(v: u8) -> Self {
        match v {
            0 => Self::Idle,
            1 => Self::Running,
            2 => Self::Drained,
            _ => Self::Fault,
        }
    }
}

#[allow(missing_debug_implementations)]
pub struct Engine {
    pub(crate) status: AtomicU8,
    pub(crate) last_error: AtomicI32,
    pub(crate) tick_counter: TickCounter,
    pub sample_period_cycles: u32,
    pub cycles_per_second: f32,
    pub stepping_axes: [Option<AxisState>; MAX_AXES],
    pub num_axes: u8,
    ring_alloc_cursor: usize,
    pub(crate) step_state: [StepMotorState; MAX_AXES],
    pub(crate) last_motors: [f32; MAX_AXES],
    pub tick_caches: crate::stepping_state::TickCaches,
    #[cfg(any(test, feature = "host"))]
    test_queue_ptrs: [*mut crate::step_queue::StepQueue; MAX_AXES],
}

impl Engine {
    pub fn new(clock_freq: u32, sample_rate_hz: u32) -> Self {
        let (_, sample_period_cycles) = Self::compute_sample_period(clock_freq, sample_rate_hz);
        Self {
            status: AtomicU8::new(RuntimeStatus::Idle as u8),
            last_error: AtomicI32::new(0),
            tick_counter: TickCounter::new(),
            sample_period_cycles,
            cycles_per_second: clock_freq as f32,
            stepping_axes: [const { None }; MAX_AXES],
            num_axes: 0,
            ring_alloc_cursor: 0,
            step_state: [StepMotorState::default(); MAX_AXES],
            last_motors: [0.0; MAX_AXES],
            tick_caches: crate::stepping_state::TickCaches::new(),
            #[cfg(any(test, feature = "host"))]
            test_queue_ptrs: [core::ptr::null_mut(); MAX_AXES],
        }
    }

    pub fn new_production(clock_freq: u32, sample_rate_hz: u32) -> Self {
        Self::new(clock_freq, sample_rate_hz)
    }

    #[inline]
    fn compute_sample_period(clock_freq: u32, sample_rate_hz: u32) -> (f32, u32) {
        if sample_rate_hz == 0 {
            return (0.0, 0);
        }
        let sec = 1.0_f32 / (sample_rate_hz as f32);
        #[allow(clippy::integer_division)]
        let cycles = (clock_freq + sample_rate_hz / 2) / sample_rate_hz;
        (sec, cycles)
    }

    /// # Safety
    /// `ptr` must be valid for writes of `size_of::<Engine>()` bytes and must
    /// not be aliased for the duration of this call.
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place(ptr: *mut Self, clock_freq: u32, sample_rate_hz: u32) {
        use core::ptr::addr_of_mut;
        let (_, sample_period_cycles) = Self::compute_sample_period(clock_freq, sample_rate_hz);
        unsafe {
            addr_of_mut!((*ptr).status).write(AtomicU8::new(RuntimeStatus::Idle as u8));
            addr_of_mut!((*ptr).last_error).write(AtomicI32::new(0));
            addr_of_mut!((*ptr).tick_counter).write(TickCounter::new());
            addr_of_mut!((*ptr).sample_period_cycles).write(sample_period_cycles);
            addr_of_mut!((*ptr).cycles_per_second).write(clock_freq as f32);
            addr_of_mut!((*ptr).stepping_axes).write([const { None }; MAX_AXES]);
            addr_of_mut!((*ptr).num_axes).write(0);
            addr_of_mut!((*ptr).ring_alloc_cursor).write(0);
            addr_of_mut!((*ptr).step_state).write([StepMotorState::default(); MAX_AXES]);
            addr_of_mut!((*ptr).last_motors).write([0.0; MAX_AXES]);
            addr_of_mut!((*ptr).tick_caches).write(crate::stepping_state::TickCaches::new());
            #[cfg(any(test, feature = "host"))]
            addr_of_mut!((*ptr).test_queue_ptrs).write([core::ptr::null_mut(); MAX_AXES]);
        }
    }

    /// # Safety
    /// See [`init_in_place`].
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place_production(ptr: *mut Self, clock_freq: u32, sample_rate_hz: u32) {
        unsafe { Self::init_in_place(ptr, clock_freq, sample_rate_hz) }
    }
}

impl Engine {
    pub fn status(&self) -> RuntimeStatus {
        RuntimeStatus::from_u8(self.status.load(Ordering::Acquire))
    }

    pub fn last_error(&self) -> i32 {
        self.last_error.load(Ordering::Acquire)
    }

    pub fn tick_counter(&self) -> u32 {
        self.tick_counter.snapshot()
    }

    pub fn configure_axis(
        &mut self,
        axis_idx: u8,
        mode: StepMode,
        microstep_distance: f32,
        ring_depth: usize,
        bindings: &[StepperBindingRust],
        total_ring_pieces: usize,
    ) -> i32 {
        if (axis_idx as usize) >= MAX_AXES {
            return KALICO_ERR_INVALID_ARG;
        }
        if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
            return KALICO_ERR_INVALID_ARG;
        }
        if self.ring_alloc_cursor + ring_depth > total_ring_pieces {
            return KALICO_ERR_RING_FULL;
        }

        let offset = self.ring_alloc_cursor;
        self.ring_alloc_cursor += ring_depth;

        let idx = axis_idx as usize;
        // SAFETY: `idx < MAX_AXES` is guaranteed by the bounds check above.
        // `stepping_axes` has exactly `MAX_AXES` elements.
        #[allow(clippy::indexing_slicing)]
        let axis = self.stepping_axes[idx].get_or_insert_with(AxisState::new_unconfigured);

        axis.microstep_distance = microstep_distance;
        axis.ring = crate::piece_ring::RingDescriptor::new(offset, ring_depth);
        axis.reset_isr_cache();
        axis.steppers.clear();
        for b in bindings {
            let tmc_cs_oid = if b.tmc_cs_oid == TMC_CS_OID_NONE {
                None
            } else {
                Some(b.tmc_cs_oid)
            };
            let stepper = crate::stepping_state::StepperRef::new(b.stepper_oid, tmc_cs_oid);
            let _ = axis.steppers.push(stepper);
        }
        axis.mode.store(mode as u8, Ordering::Release);

        if idx + 1 > self.num_axes as usize {
            #[allow(clippy::cast_possible_truncation)]
            {
                self.num_axes = (idx + 1) as u8;
            }
        }

        KALICO_OK
    }

    /// Reset the engine to a clean, just-initialized motion state.
    ///
    /// Preserves `sample_period_cycles`, `cycles_per_second`, and the running
    /// `tick_counter` — resetting those would desync the ISR time base.
    ///
    /// The per-axis C step queues are cleared separately by the FFI caller
    /// (`kalico_runtime_reset`).
    pub fn reset(&mut self) {
        self.ring_alloc_cursor = 0;
        self.stepping_axes = [const { None }; MAX_AXES];
        self.num_axes = 0;
        self.step_state = [StepMotorState::default(); MAX_AXES];
        self.last_motors = [0.0; MAX_AXES];
        self.tick_caches = crate::stepping_state::TickCaches::new();
        self.status
            .store(RuntimeStatus::Idle as u8, Ordering::Release);
        self.last_error.store(0, Ordering::Release);
    }

    pub fn push_pieces(
        &mut self,
        axis_idx: u8,
        pieces: &[PieceEntry],
        storage: &mut [PieceEntry],
    ) -> i32 {
        let Some(axis) = self
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return KALICO_ERR_INVALID_ARG;
        };
        for &piece in pieces {
            if axis.ring.push(storage, piece).is_err() {
                return KALICO_ERR_RING_FULL;
            }
        }
        KALICO_OK
    }

    pub fn tick(&mut self, now: u64, shared: &SharedState, storage: &mut [PieceEntry]) -> bool {
        #[cfg(feature = "motion-module-stepper")]
        use crate::dispatch_stepper::dispatch_axis;

        #[cfg(feature = "motion-module-stepper")]
        #[cfg(any(test, feature = "host"))]
        let get_queue = |i: usize| {
            self.test_queue_ptrs
                .get(i)
                .copied()
                .unwrap_or(core::ptr::null_mut())
        };
        #[cfg(feature = "motion-module-stepper")]
        #[cfg(not(any(test, feature = "host")))]
        let get_queue = |i: usize| crate::step_queue::queue_for_axis(i);

        #[cfg(feature = "motion-module-stepper")]
        let sample_period_sec = if self.sample_period_cycles == 0 || self.cycles_per_second == 0.0 {
            0.0_f32
        } else {
            self.sample_period_cycles as f32 / self.cycles_per_second
        };

        #[cfg(feature = "motion-module-stepper")]
        #[allow(clippy::cast_possible_truncation)]
        let now_lo = now as u32;

        let mut active = false;
        let mut v_per_axis_q16 = [0u32; 3];

        for i in 0..(self.num_axes as usize) {
            let (p_end, v_end, p_sample_start) = {
                let Some(axis) = self.stepping_axes.get_mut(i).and_then(|s| s.as_mut()) else {
                    continue;
                };
                let cps = self.cycles_per_second;
                let fault = SharedFaultSink { shared };
                let Some((p_end, v_end)) = crate::motion_core::get_position_and_velocity(
                    &mut axis.armed,
                    &mut axis.ring,
                    storage,
                    now,
                    self.sample_period_cycles,
                    cps,
                    i,
                    &fault,
                ) else {
                    continue;
                };
                active = true;
                let p_sample_start = axis.p_prev;
                axis.p_prev = p_end;
                axis.v_prev = v_end;
                (p_end, v_end, p_sample_start)
            };

            // Accumulate per-axis Q16 velocity for the first three axes (X/Y/Z).
            // v_end is mm/s (f32). Q16 = v * 65536, clamped to u32::MAX.
            #[allow(clippy::cast_sign_loss, clippy::cast_possible_truncation)]
            if i < 3 {
                let v_q16_f = v_end.abs() * 65536.0_f32;
                let v_q16 = if v_q16_f >= u32::MAX as f32 {
                    u32::MAX
                } else {
                    v_q16_f as u32
                };
                // SAFETY: i < 3 == v_per_axis_q16.len()
                #[allow(clippy::indexing_slicing)]
                {
                    v_per_axis_q16[i] = v_q16;
                }
            }

            #[cfg(feature = "motion-module-stepper")]
            {
                let Some(axis) = self.stepping_axes.get_mut(i).and_then(|s| s.as_mut()) else {
                    continue;
                };
                let queue_ptr = get_queue(i);
                dispatch_axis(
                    i,
                    axis,
                    queue_ptr,
                    shared,
                    p_end,
                    v_end,
                    p_sample_start,
                    sample_period_sec,
                    now_lo,
                    self.cycles_per_second,
                );
            }

            #[cfg(not(feature = "motion-module-stepper"))]
            {
                let _ = (p_end, v_end, p_sample_start);
            }
        }

        // Evaluate armed GPIO/StallGuard endstop sources now that pin levels
        // are fresh (C sampler ran before this tick) and step counts are current
        // (dispatch updated position_count above). Build an OID-indexed count
        // array: publish_snapshot accesses stepper_counts[oid].
        let mut stepper_counts = [0i32; MAX_STEPPER_OIDS];
        for axis_opt in &self.stepping_axes {
            let Some(axis) = axis_opt.as_ref() else {
                continue;
            };
            for stepper in &axis.steppers {
                let oid = usize::from(stepper.stepper_oid);
                if let Some(slot) = stepper_counts.get_mut(oid) {
                    *slot = stepper.position_count.load(Ordering::Acquire);
                }
            }
        }
        crate::endstop::tick(now, v_per_axis_q16, &stepper_counts);

        active
    }

    pub fn retired_counts(&self) -> [u32; MAX_AXES] {
        let mut out = [0u32; MAX_AXES];
        for (slot, entry) in out.iter_mut().zip(self.stepping_axes.iter()) {
            if let Some(axis) = entry {
                *slot = axis.ring.retired_count();
            }
        }
        out
    }

    pub fn configure_kinematics(&mut self, k_xy: f32) -> i32 {
        if !k_xy.is_finite() || k_xy <= 0.0 {
            return -1;
        }
        0
    }

    pub fn configure_pressure_advance(&mut self, advance_accel: f32, advance_decel: f32) -> i32 {
        if !advance_accel.is_finite() || !advance_decel.is_finite() {
            return -1;
        }
        if advance_accel < 0.0 || advance_decel < 0.0 {
            return -1;
        }
        0
    }

    /// Legacy `configure_axis` overload without `ring_depth` — used by
    /// `kalico_runtime_configure_axis` FFI which does not yet carry a
    /// ring_depth field on the wire. Allocates a default region of 64
    /// pieces per axis.
    pub fn configure_axis_legacy(
        &mut self,
        axis_idx: u8,
        mode: StepMode,
        microstep_distance: f32,
        bindings: &[StepperBindingRust],
        total_ring_pieces: usize,
    ) -> i32 {
        let remaining = total_ring_pieces.saturating_sub(self.ring_alloc_cursor);
        let default_depth = remaining.min(64).max(1);
        self.configure_axis(
            axis_idx,
            mode,
            microstep_distance,
            default_depth,
            bindings,
            total_ring_pieces,
        )
    }

    pub fn set_axis_mode(&mut self, axis_idx: u8, new_mode_byte: u8) -> i32 {
        if (axis_idx as usize) >= MAX_AXES {
            return -1;
        }
        let new_mode = match new_mode_byte {
            0 => StepMode::Pulse,
            1 => StepMode::Phase,
            _ => return -1,
        };
        let motion_active = self
            .stepping_axes
            .iter()
            .any(|a| a.as_ref().map_or(false, |ax| ax.armed.is_some()));
        if motion_active {
            return -2;
        }
        #[cfg(not(any(test, feature = "host")))]
        {
            #[allow(unsafe_code)]
            {
                use crate::step_queue::{StepQueue, step_queues};
                unsafe {
                    let q = step_queues.get().cast::<StepQueue>().add(axis_idx as usize);
                    core::ptr::write_volatile(&mut (*q).head, 0);
                    core::ptr::write_volatile(&mut (*q).tail, 0);
                }
            }
        }
        let Some(axis) = self
            .stepping_axes
            .get_mut(axis_idx as usize)
            .and_then(|s| s.as_mut())
        else {
            return -1;
        };
        match new_mode {
            StepMode::Phase => {
                use core::sync::atomic::Ordering;
                for stepper in &axis.steppers {
                    let offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
                    let target = axis.last_step_count.wrapping_add(offset);
                    stepper.last_phase_target.store(target, Ordering::Release);
                }
            }
            StepMode::Pulse => {}
        }
        axis.mode.store(new_mode as u8, Ordering::Release);
        0
    }

    pub fn set_stepper_offset(
        &mut self,
        shared: &SharedState,
        stepper_idx: u8,
        delta_microsteps: i32,
        max_microsteps_per_sample: u16,
    ) -> i32 {
        use core::sync::atomic::Ordering;
        if delta_microsteps == 0 {
            return 0;
        }
        if max_microsteps_per_sample == 0 || max_microsteps_per_sample > 256 {
            crate::fault_helpers::raise_jog_parameters_invalid(shared);
            return -1;
        }
        let mut remaining = stepper_idx as usize;
        for axis_opt in &mut self.stepping_axes {
            let Some(axis) = axis_opt.as_mut() else {
                continue;
            };
            if remaining < axis.steppers.len() {
                #[allow(clippy::indexing_slicing)]
                let stepper = &axis.steppers[remaining];
                let new_target = stepper
                    .phase_offset_target
                    .load(Ordering::Acquire)
                    .wrapping_add(delta_microsteps);
                stepper
                    .phase_offset_target
                    .store(new_target, Ordering::Release);
                shared
                    .max_phase_offset_ramp_per_sample
                    .store(max_microsteps_per_sample, Ordering::Release);
                return 0;
            }
            remaining -= axis.steppers.len();
        }
        crate::fault_helpers::raise_jog_parameters_invalid(shared);
        -1
    }

    pub fn seed_position(&mut self, xyz: [f32; 3]) {
        use core::sync::atomic::Ordering;
        let motor_positions = [xyz[0], xyz[1], xyz[2], 0.0_f32, 0.0, 0.0, 0.0, 0.0];
        for (ss, &pos) in self.step_state.iter_mut().zip(motor_positions.iter()) {
            ss.seed(pos);
        }
        for ((lm, pp), &pos) in self
            .last_motors
            .iter_mut()
            .zip(self.tick_caches.p_prev.iter_mut())
            .zip(motor_positions.iter())
        {
            *lm = pos;
            *pp = pos;
        }
        for vp in self.tick_caches.v_prev.iter_mut() {
            *vp = 0.0;
        }

        for (i, axis_opt) in self.stepping_axes.iter_mut().enumerate() {
            let Some(axis) = axis_opt.as_mut() else {
                continue;
            };
            let axis_pos_mm = motor_positions.get(i).copied().unwrap_or(0.0);
            let microstep_distance = axis.microstep_distance;
            if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
                continue;
            }
            #[allow(clippy::cast_possible_truncation)]
            let seed_steps = libm::roundf(axis_pos_mm / microstep_distance) as i32;
            axis.last_step_count = seed_steps;
            axis.p_prev = axis_pos_mm;
            axis.v_prev = 0.0;
            for stepper in &axis.steppers {
                stepper.position_count.store(seed_steps, Ordering::Release);
                stepper
                    .last_phase_target
                    .store(seed_steps, Ordering::Release);
            }
        }
    }

    pub fn debug_steps_per_mm(&self, i: usize) -> f32 {
        self.step_state
            .get(i)
            .map(|s| s.debug_steps_per_mm())
            .unwrap_or(0.0)
    }

    pub fn debug_accumulator(&self, i: usize) -> f64 {
        self.step_state
            .get(i)
            .map(|s| s.debug_accumulator())
            .unwrap_or(0.0)
    }

    pub fn debug_last_motor(&self, i: usize) -> f32 {
        self.last_motors.get(i).copied().unwrap_or(0.0)
    }

    pub fn debug_last_timing(&self) -> (u64, u64, u64) {
        (0, 0, 0)
    }

    pub fn runtime_force_idle(&mut self, shared: &SharedState) {
        for ss in &mut self.step_state {
            ss.reset_accumulator();
        }
        for axis_opt in &mut self.stepping_axes {
            if let Some(axis) = axis_opt.as_mut() {
                axis.reset_isr_cache();
                axis.ring.drain();
            }
        }
        self.last_motors = [0.0; MAX_AXES];
        if self.status() != RuntimeStatus::Fault {
            self.status
                .store(RuntimeStatus::Idle as u8, Ordering::Release);
        }
        shared.acked_force_idle.store(true, Ordering::Release);
    }

    #[cfg(any(test, feature = "host"))]
    pub fn test_set_sample_period(&mut self, sample_rate_hz: u32) {
        let cycles = if sample_rate_hz == 0 || self.cycles_per_second == 0.0 {
            0
        } else {
            (self.cycles_per_second / (sample_rate_hz as f32)).round() as u32
        };
        self.sample_period_cycles = cycles;
    }

    #[cfg(any(test, feature = "host"))]
    pub fn test_install_step_queues(
        &mut self,
        queues: [*mut crate::step_queue::StepQueue; MAX_AXES],
    ) {
        self.test_queue_ptrs = queues;
    }

    #[cfg(any(test, feature = "host"))]
    pub fn test_queue_ptr(&self, axis_idx: usize) -> *mut crate::step_queue::StepQueue {
        self.test_queue_ptrs
            .get(axis_idx)
            .copied()
            .unwrap_or(core::ptr::null_mut())
    }

    #[cfg(any(test, feature = "host"))]
    pub fn debug_current_is_some(&self) -> bool {
        self.stepping_axes
            .iter()
            .any(|a| a.as_ref().map_or(false, |ax| ax.armed.is_some()))
    }
}

#[cfg(test)]
impl Default for Engine {
    fn default() -> Self {
        Self::new(520_000_000, crate::clock::TEST_ONLY_TICK_RATE_HZ)
    }
}
