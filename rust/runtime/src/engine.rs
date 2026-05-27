//! `Engine` — stub for Task 5. Task 6 rewrites this properly.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use heapless::spsc::Producer;

use crate::state::SharedState;
use crate::trace::{TRACE_RING_N, TraceSample};

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

/// Placeholder engine — Task 6 rewrites this properly.
///
/// Retains the fields required for compilation by `state.rs` and `kalico-c-api`.
#[allow(missing_debug_implementations)]
pub struct Engine<P, I> {
    pub(crate) current: Option<crate::segment::Segment>,
    pub(crate) status: AtomicU8,
    pub(crate) last_error: AtomicI32,
    pub(crate) tick_counter: crate::clock::TickCounter,
    pub stepping_axes: [crate::stepping_state::AxisConfig; crate::stepping_state::N_AXES],
    pub sample_period_sec: f32,
    pub sample_period_cycles: u32,
    pub cycles_per_second: f32,
    pub tick_caches: crate::stepping_state::TickCaches,
    #[cfg(any(test, feature = "host"))]
    test_queue_ptrs: [*mut crate::step_queue::StepQueue; crate::stepping_state::N_AXES],
    _marker: core::marker::PhantomData<(P, I)>,
    // Legacy fields retained for kalico-c-api compatibility until Task 6.
    step_state: [crate::step::StepMotorState; 4],
    last_motors: [f32; 4],
    phase_modulators:
        [Option<crate::modulator::PhaseDirectModulator>; crate::state::MAX_STEPPER_OIDS],
    phase_tick_counter: u32,
    mcu_config: Option<crate::config::McuAxisConfig>,
}

impl<P: Default, I: Default> Engine<P, I> {
    pub fn new(clock_freq: u32, sample_rate_hz: u32) -> Self {
        let (sample_period_sec, sample_period_cycles) =
            Self::compute_sample_period(clock_freq, sample_rate_hz);
        Self {
            current: None,
            status: AtomicU8::new(RuntimeStatus::Idle as u8),
            last_error: AtomicI32::new(0),
            tick_counter: crate::clock::TickCounter::new(),
            stepping_axes: [
                crate::stepping_state::AxisConfig::new_unconfigured(),
                crate::stepping_state::AxisConfig::new_unconfigured(),
                crate::stepping_state::AxisConfig::new_unconfigured(),
                crate::stepping_state::AxisConfig::new_unconfigured(),
            ],
            sample_period_sec,
            sample_period_cycles,
            cycles_per_second: clock_freq as f32,
            tick_caches: crate::stepping_state::TickCaches::new(),
            #[cfg(any(test, feature = "host"))]
            test_queue_ptrs: [core::ptr::null_mut(); crate::stepping_state::N_AXES],
            _marker: core::marker::PhantomData,
            step_state: [crate::step::StepMotorState::default(); 4],
            last_motors: [0.0; 4],
            phase_modulators: [const { None }; crate::state::MAX_STEPPER_OIDS],
            phase_tick_counter: 0,
            mcu_config: None,
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

    /// In-place initialization — avoids materializing the struct on the stack.
    ///
    /// # Safety
    /// `ptr` must be valid for writes of `size_of::<Engine<P, I>>()` bytes.
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place(ptr: *mut Self, clock_freq: u32, sample_rate_hz: u32) {
        use core::ptr::addr_of_mut;
        let (sample_period_sec, sample_period_cycles) =
            Self::compute_sample_period(clock_freq, sample_rate_hz);
        unsafe {
            addr_of_mut!((*ptr).current).write(None);
            addr_of_mut!((*ptr).status).write(AtomicU8::new(RuntimeStatus::Idle as u8));
            addr_of_mut!((*ptr).last_error).write(AtomicI32::new(0));
            addr_of_mut!((*ptr).tick_counter).write(crate::clock::TickCounter::new());
            let axes_ptr =
                addr_of_mut!((*ptr).stepping_axes).cast::<crate::stepping_state::AxisConfig>();
            for i in 0..crate::stepping_state::N_AXES {
                axes_ptr
                    .add(i)
                    .write(crate::stepping_state::AxisConfig::new_unconfigured());
            }
            addr_of_mut!((*ptr).sample_period_sec).write(sample_period_sec);
            addr_of_mut!((*ptr).sample_period_cycles).write(sample_period_cycles);
            addr_of_mut!((*ptr).cycles_per_second).write(clock_freq as f32);
            addr_of_mut!((*ptr).tick_caches).write(crate::stepping_state::TickCaches::new());
            #[cfg(any(test, feature = "host"))]
            addr_of_mut!((*ptr).test_queue_ptrs)
                .write([core::ptr::null_mut(); crate::stepping_state::N_AXES]);
            addr_of_mut!((*ptr)._marker).write(core::marker::PhantomData);
            addr_of_mut!((*ptr).step_state).write([crate::step::StepMotorState::default(); 4]);
            addr_of_mut!((*ptr).last_motors).write([0.0; 4]);
            addr_of_mut!((*ptr).phase_modulators)
                .write([const { None }; crate::state::MAX_STEPPER_OIDS]);
            addr_of_mut!((*ptr).phase_tick_counter).write(0);
            addr_of_mut!((*ptr).mcu_config).write(None);
        }
    }

    /// # Safety
    /// See [`init_in_place`].
    #[allow(unsafe_code)]
    pub unsafe fn init_in_place_production(ptr: *mut Self, clock_freq: u32, sample_rate_hz: u32) {
        unsafe { Self::init_in_place(ptr, clock_freq, sample_rate_hz) }
    }
}

impl<P, I> Engine<P, I> {
    pub fn status(&self) -> RuntimeStatus {
        RuntimeStatus::from_u8(self.status.load(Ordering::Acquire))
    }

    pub fn last_error(&self) -> i32 {
        self.last_error.load(Ordering::Acquire)
    }

    pub fn tick_counter(&self) -> u32 {
        self.tick_counter.snapshot()
    }

    /// No-op stub retained for FFI ABI compatibility.
    pub fn configure_kinematics(&mut self, k_xy: f32) -> i32 {
        if !k_xy.is_finite() || k_xy <= 0.0 {
            return -1;
        }
        0
    }

    /// No-op stub retained for FFI ABI compatibility.
    pub fn configure_pressure_advance(&mut self, advance_accel: f32, advance_decel: f32) -> i32 {
        if !advance_accel.is_finite() || !advance_decel.is_finite() {
            return -1;
        }
        if advance_accel < 0.0 || advance_decel < 0.0 {
            return -1;
        }
        0
    }

    pub fn configure_axis(
        &mut self,
        axis_idx: u8,
        mode: crate::stepping_state::StepMode,
        microstep_distance: f32,
        bindings: &[crate::stepping_state::StepperBindingRust],
    ) -> i32 {
        use crate::error::{KALICO_ERR_INVALID_ARG, KALICO_ERR_MOTION_IN_PROGRESS, KALICO_OK};

        if (axis_idx as usize) >= crate::stepping_state::N_AXES {
            return KALICO_ERR_INVALID_ARG;
        }
        if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
            return KALICO_ERR_INVALID_ARG;
        }
        if self.current.is_some() {
            return KALICO_ERR_MOTION_IN_PROGRESS;
        }

        #[allow(clippy::indexing_slicing)]
        let axis = &mut self.stepping_axes[axis_idx as usize];
        axis.microstep_distance = microstep_distance;
        axis.piece = None;
        axis.piece_start_time_cycles = 0;
        axis.last_step_count = 0;
        axis.steppers.clear();
        for b in bindings {
            let tmc_cs_oid = if b.tmc_cs_oid == crate::stepping_state::TMC_CS_OID_NONE {
                None
            } else {
                Some(b.tmc_cs_oid)
            };
            let stepper = crate::stepping_state::StepperRef::new(b.stepper_oid, tmc_cs_oid);
            let _ = axis.steppers.push(stepper);
        }
        axis.mode
            .store(mode as u8, core::sync::atomic::Ordering::Release);
        KALICO_OK
    }

    pub fn set_axis_mode(&mut self, axis_idx: u8, new_mode_byte: u8) -> i32 {
        if (axis_idx as usize) >= crate::stepping_state::N_AXES {
            return -1;
        }
        let new_mode = match new_mode_byte {
            0 => crate::stepping_state::StepMode::Pulse,
            1 => crate::stepping_state::StepMode::Phase,
            _ => return -1,
        };
        let motion_active = self.stepping_axes.iter().any(|a| a.piece.is_some());
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
        #[allow(clippy::indexing_slicing)]
        let axis = &mut self.stepping_axes[axis_idx as usize];
        match new_mode {
            crate::stepping_state::StepMode::Phase => {
                use core::sync::atomic::Ordering;
                for stepper in &axis.steppers {
                    let offset = stepper.phase_offset_microsteps.load(Ordering::Acquire);
                    let target = axis.last_step_count.wrapping_add(offset);
                    stepper.last_phase_target.store(target, Ordering::Release);
                }
            }
            crate::stepping_state::StepMode::Pulse => {}
        }
        axis.mode
            .store(new_mode as u8, core::sync::atomic::Ordering::Release);
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
        for axis in &mut self.stepping_axes {
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
        let motor_positions = [xyz[0], xyz[1], xyz[2], 0.0_f32];
        for i in 0..4 {
            if let Some(ss) = self.step_state.get_mut(i) {
                ss.seed(motor_positions[i]);
            }
        }
        self.last_motors = motor_positions;
        self.tick_caches.p_prev = motor_positions;
        self.tick_caches.v_prev = [0.0; 4];

        for (axis, &axis_pos_mm) in self.stepping_axes.iter_mut().zip(motor_positions.iter()) {
            let microstep_distance = axis.microstep_distance;
            if !microstep_distance.is_finite() || microstep_distance <= 0.0 {
                continue;
            }
            #[allow(clippy::cast_possible_truncation)]
            let seed_steps = libm::roundf(axis_pos_mm / microstep_distance) as i32;
            axis.last_step_count = seed_steps;
            for stepper in &axis.steppers {
                stepper.position_count.store(seed_steps, Ordering::Release);
                stepper
                    .last_phase_target
                    .store(seed_steps, Ordering::Release);
            }
        }
    }

    pub fn configure(&mut self, config: crate::config::McuAxisConfig) {
        for (i, motor_opt) in config.motors.iter().enumerate() {
            if let Some(motor) = motor_opt {
                if let Some(ss) = self.step_state.get_mut(i) {
                    *ss = crate::step::StepMotorState::new(motor.steps_per_mm);
                }
            }
        }
        self.mcu_config = Some(config);
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

    pub fn runtime_force_idle(
        &mut self,
        _pool: &crate::curve_pool::CurvePool,
        queue: &mut crate::c_segment_queue::Consumer<crate::segment::Segment>,
        shared: &SharedState,
    ) {
        shared.producer_pending.store(false, Ordering::Release);
        while queue.dequeue().is_some() {}
        for ss in &mut self.step_state {
            ss.reset_accumulator();
        }
        for slot in &mut self.phase_modulators {
            *slot = None;
        }
        self.phase_tick_counter = 0;
        self.clear_current();
        self.last_motors = [0.0; 4];
        if self.status() != RuntimeStatus::Fault {
            self.status
                .store(RuntimeStatus::Idle as u8, Ordering::Release);
        }
        shared.acked_force_idle.store(true, Ordering::Release);
    }

    pub(crate) fn clear_current(&mut self) {
        self.current = None;
    }

    /// Stub arm_segment_with_diag — no-op until Task 6.
    pub fn arm_segment_with_diag(
        &mut self,
        seg: crate::segment::Segment,
        _curve_pool: &crate::curve_pool::CurvePool,
        _shared: &SharedState,
    ) {
        self.current = Some(seg);
    }

    /// Stub tick_sample — no-op until Task 6.
    pub fn tick_sample(
        &mut self,
        _shared: &SharedState,
        _curve_pool: &crate::curve_pool::CurvePool,
        _trace: &mut Producer<'_, TraceSample, TRACE_RING_N>,
    ) {
    }

    /// Test-only: set sample period.
    #[cfg(any(test, feature = "host"))]
    pub fn test_set_sample_period(&mut self, sample_rate_hz: u32) {
        let sec = 1.0_f32 / (sample_rate_hz as f32);
        let cycles = (self.cycles_per_second / (sample_rate_hz as f32)).round() as u32;
        self.sample_period_sec = sec;
        self.sample_period_cycles = cycles;
    }

    /// Test-only queue installer.
    #[cfg(any(test, feature = "host"))]
    pub fn test_install_step_queues(
        &mut self,
        queues: [*mut crate::step_queue::StepQueue; crate::stepping_state::N_AXES],
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
        self.current.is_some()
    }

    #[cfg(any(test, feature = "host"))]
    pub fn debug_current_segment_id(&self) -> Option<u32> {
        self.current.as_ref().map(|s| s.id)
    }
}

#[cfg(test)]
impl<P: Default, I: Default> Default for Engine<P, I> {
    fn default() -> Self {
        Self::new(520_000_000, crate::clock::TICK_RATE_HZ)
    }
}
