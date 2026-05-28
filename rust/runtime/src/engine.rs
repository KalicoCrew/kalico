//! Per-axis piece-ring walker engine — Task 6 rewrite.
//!
//! The `Engine` struct replaces the old curve-pool + segment architecture with
//! a simple per-axis polynomial playback engine:
//!
//! - Up to [`MAX_AXES`] axes, each backed by a contiguous region of the shared
//!   `piece_storage` array in `RuntimeContext`.
//! - The ISR (`tick`) iterates over configured axes, calls
//!   `get_piece_for_time`, evaluates the Horner polynomial, and dispatches
//!   stepping via the unchanged `dispatch_axis` backend in `tick.rs`.
//! - No curve pool, no segments, no kinematic transforms, no E-follower on
//!   the MCU.
//!
//! Ring layout rationale: splitting one flat `[PieceEntry; N]` array into N
//! independent `&mut PieceRing` borrows fights the borrow checker (the entire
//! array would need to be behind a single `&mut`).  Instead, each axis carries
//! a `RingDescriptor` — a borrow-free set of bookkeeping integers — and all
//! mutation goes through `&mut [PieceEntry]` passed explicitly into every
//! operation.  `PieceRing<'a>` is kept for host unit tests; `RingDescriptor`
//! is used exclusively by the engine.

use core::sync::atomic::{AtomicI32, AtomicU8, Ordering};

use crate::clock::TickCounter;
use crate::error::{KALICO_ERR_INVALID_ARG, KALICO_ERR_RING_FULL, KALICO_OK};
use crate::fault_helpers::raise_piece_start_in_past;
use crate::piece_ring::PieceEntry;
use crate::state::SharedState;
use crate::step::StepMotorState;
use crate::stepping_state::{AxisState, MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};

pub use crate::stepping_state::N_AXES;

/// Engine status.
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

/// The per-axis piece-ring walker engine.
///
/// Generic parameters and pressure-advance / input-shaping slot types have
/// been dropped: the host pre-bakes those transforms before uploading pieces.
///
/// Storage model:
/// - `RuntimeContext::piece_storage` holds a flat `[PieceEntry; TOTAL_RING_PIECES]`.
/// - Each configured axis owns a contiguous sub-slice described by
///   `AxisState::ring` (`RingDescriptor` — offset + depth + head + tail + count +
///   consumed).
/// - The engine bump-allocates axis regions from a running `ring_alloc_cursor`
///   during `configure_axis`.
#[allow(missing_debug_implementations)]
pub struct Engine {
    pub(crate) status: AtomicU8,
    pub(crate) last_error: AtomicI32,
    pub(crate) tick_counter: TickCounter,
    pub sample_period_cycles: u32,
    pub cycles_per_second: f32,
    /// Per-axis state.  `Option` allows the array to be const-initialized;
    /// `Some` once `configure_axis` runs for that slot.
    pub stepping_axes: [Option<AxisState>; MAX_AXES],
    pub num_axes: u8,
    /// Bump-allocate cursor: next unused index in piece_storage.
    ring_alloc_cursor: usize,
    // Per-axis `StepMotorState` for the accumulator-based step path.
    // Indexed 0..MAX_AXES; entries beyond num_axes are unused.
    pub(crate) step_state: [StepMotorState; MAX_AXES],
    pub(crate) last_motors: [f32; MAX_AXES],
    // Tick caches kept for `seed_position` compatibility.
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

    /// In-place initialization — avoids materializing the struct on the stack.
    ///
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

    /// Register an axis and allocate its ring region from the shared storage.
    ///
    /// `ring_depth` is the number of piece entries to allocate for this axis.
    /// The engine bump-allocates from `ring_alloc_cursor`; returns
    /// `KALICO_ERR_RING_FULL` if the remaining space is insufficient.
    ///
    /// Returns `KALICO_OK` (0) on success.
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
        // Check that we have enough room left.
        if self.ring_alloc_cursor + ring_depth > total_ring_pieces {
            return KALICO_ERR_RING_FULL;
        }

        let offset = self.ring_alloc_cursor;
        self.ring_alloc_cursor += ring_depth;

        let idx = axis_idx as usize;
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

        // Track num_axes as the high-water mark of configured indices.
        if idx + 1 > self.num_axes as usize {
            #[allow(clippy::cast_possible_truncation)]
            {
                self.num_axes = (idx + 1) as u8;
            }
        }

        KALICO_OK
    }

    /// Append pieces into an axis's ring region.
    ///
    /// `storage` is the shared piece_storage array.
    /// Returns `KALICO_OK` on success, `KALICO_ERR_RING_FULL` if the ring is
    /// full, `KALICO_ERR_INVALID_ARG` if the axis is not configured.
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

    /// Per-sample ISR body.
    ///
    /// For each configured axis: advance to the correct piece for `now`,
    /// evaluate the Horner polynomial, and call `dispatch_axis`.
    ///
    /// `storage` is projected from `RuntimeContext::piece_storage` by the
    /// caller.
    pub fn tick(
        &mut self,
        now: u64,
        shared: &SharedState,
        storage: &mut [PieceEntry],
    ) {
        use crate::tick::dispatch_axis;
        #[cfg(any(test, feature = "host"))]
        let get_queue = |i: usize| {
            self.test_queue_ptrs
                .get(i)
                .copied()
                .unwrap_or(core::ptr::null_mut())
        };
        #[cfg(not(any(test, feature = "host")))]
        let get_queue = |i: usize| crate::step_queue::queue_for_axis(i);

        let sample_period_sec = if self.sample_period_cycles == 0 || self.cycles_per_second == 0.0 {
            0.0_f32
        } else {
            self.sample_period_cycles as f32 / self.cycles_per_second
        };

        #[allow(clippy::cast_possible_truncation)]
        let now_lo = now as u32;

        for i in 0..(self.num_axes as usize) {
            // Pull out the axis + storage borrow in a single block to avoid
            // aliasing the mutable reference.
            let (p_end, v_end, p_sample_start) = {
                let Some(axis) = self.stepping_axes.get_mut(i).and_then(|s| s.as_mut()) else {
                    continue;
                };
                let cps = self.cycles_per_second;
                let Some((p_end, v_end)) = get_position_and_velocity(
                    axis,
                    now,
                    self.sample_period_cycles,
                    cps,
                    shared,
                    storage,
                    i,
                ) else {
                    continue;
                };
                let p_sample_start = axis.p_prev;
                axis.p_prev = p_end;
                axis.v_prev = v_end;
                (p_end, v_end, p_sample_start)
            };

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
    }

    /// Returns per-axis consumed piece counts for the heartbeat.
    pub fn consumed_counts(&self) -> [u32; MAX_AXES] {
        let mut out = [0u32; MAX_AXES];
        for i in 0..MAX_AXES {
            if let Some(Some(axis)) = self.stepping_axes.get(i) {
                out[i] = axis.ring.consumed_count();
            }
        }
        out
    }

    // ── Legacy stubs retained for FFI call sites ──────────────────────────

    /// No-op stub retained for FFI ABI compatibility (kinematics now on host).
    pub fn configure_kinematics(&mut self, k_xy: f32) -> i32 {
        if !k_xy.is_finite() || k_xy <= 0.0 {
            return -1;
        }
        0
    }

    /// No-op stub retained for FFI ABI compatibility (PA now on host).
    pub fn configure_pressure_advance(&mut self, advance_accel: f32, advance_decel: f32) -> i32 {
        if !advance_accel.is_finite() || !advance_decel.is_finite() {
            return -1;
        }
        if advance_accel < 0.0 || advance_decel < 0.0 {
            return -1;
        }
        0
    }

    /// Legacy `configure_axis` overload without `ring_depth` — used by the
    /// existing `kalico_runtime_configure_axis` FFI which does not yet carry
    /// a ring_depth field on the wire.  Allocates a default region of 64
    /// pieces per axis.
    ///
    /// TODO(task7): update the wire protocol and FFI to pass ring_depth.
    pub fn configure_axis_legacy(
        &mut self,
        axis_idx: u8,
        mode: StepMode,
        microstep_distance: f32,
        bindings: &[StepperBindingRust],
        total_ring_pieces: usize,
    ) -> i32 {
        // Default per-axis depth: divide remaining space equally among unconfigured
        // axes, but floor at 64 and cap at remaining.
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
            .any(|a| a.as_ref().map_or(false, |ax| ax.has_piece));
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
        // Map logical xyz to motor positions (no kinematics on MCU).
        let motor_positions = [xyz[0], xyz[1], xyz[2], 0.0_f32, 0.0, 0.0, 0.0, 0.0];
        for i in 0..MAX_AXES {
            if let Some(ss) = self.step_state.get_mut(i) {
                ss.seed(motor_positions[i]);
            }
        }
        for i in 0..MAX_AXES {
            self.last_motors[i] = motor_positions[i];
            self.tick_caches.p_prev[i] = motor_positions[i];
            self.tick_caches.v_prev[i] = 0.0;
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

    /// Stub `runtime_force_idle` — drains pending state.
    pub fn runtime_force_idle(&mut self, shared: &SharedState) {
        for ss in &mut self.step_state {
            ss.reset_accumulator();
        }
        for axis_opt in &mut self.stepping_axes {
            if let Some(axis) = axis_opt.as_mut() {
                axis.reset_isr_cache();
            }
        }
        self.last_motors = [0.0; MAX_AXES];
        if self.status() != RuntimeStatus::Fault {
            self.status
                .store(RuntimeStatus::Idle as u8, Ordering::Release);
        }
        shared.acked_force_idle.store(true, Ordering::Release);
    }

    // ── Test helpers ──────────────────────────────────────────────────────

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
            .any(|a| a.as_ref().map_or(false, |ax| ax.has_piece))
    }
}

// ── Default for tests ─────────────────────────────────────────────────────────

#[cfg(test)]
impl Default for Engine {
    fn default() -> Self {
        Self::new(520_000_000, crate::clock::TICK_RATE_HZ)
    }
}

// ── get_piece_for_time (per spec §4.2) ────────────────────────────────────────

/// Advance the axis to the correct piece for `now`, returning
/// `(position, velocity)` if an active piece exists.
///
/// Implements the spec §4.2 slot-freeing invariant:
/// 1. Cache velocity coefficients and assign the new current piece FIRST.
/// 2. THEN free (pop) the previous slot.
///
/// Returns `None` if the axis should idle this tick (ring empty or gap
/// between pieces).  Raises a hard fault if the next piece's start_time is
/// more than `2 * sample_period_cycles` in the past.
fn get_position_and_velocity(
    axis: &mut AxisState,
    now: u64,
    sample_period_cycles: u32,
    cycles_per_second: f32,
    shared: &SharedState,
    storage: &mut [PieceEntry],
    axis_idx: usize,
) -> Option<(f32, f32)> {
    // Fast path: still within the current piece.
    if axis.has_piece && now < axis.piece_end_cycles {
        return Some(eval_horner(
            &axis.mono_coeffs,
            &axis.vel_coeffs,
            axis.piece_start_cycles,
            now,
            cycles_per_second,
        ));
    }

    // Current piece expired (or no piece yet). Peek the next one.
    // We need to do this without holding a live reference to storage while
    // also mutating axis, so copy the candidate entry by value.
    let next_entry: PieceEntry = *axis.ring.peek(storage)?;

    // Gap: next piece hasn't started yet.
    if now < next_entry.start_time {
        // Axis idles — not a fault.
        axis.has_piece = false;
        return None;
    }

    // Fault check: piece start_time is too far in the past.
    let fault_tolerance = u64::from(sample_period_cycles) * 2;
    if now.saturating_sub(next_entry.start_time) > fault_tolerance {
        raise_piece_start_in_past(shared, axis_idx);
        axis.has_piece = false;
        return None;
    }

    // Arm the new piece: cache coefficients BEFORE popping the slot.
    // Spec §4.2: arm new piece first, then free the slot it occupied.
    let (mono, vel) = next_entry.to_monomial();
    axis.mono_coeffs = mono;
    axis.vel_coeffs = vel;
    axis.piece_start_cycles = next_entry.start_time;
    axis.piece_end_cycles = next_entry.end_time(cycles_per_second);
    axis.has_piece = true;
    // Now free the ring slot (the entry is fully cached above).
    axis.ring.pop();

    Some(eval_horner(
        &axis.mono_coeffs,
        &axis.vel_coeffs,
        axis.piece_start_cycles,
        now,
        cycles_per_second,
    ))
}

/// Evaluate position and velocity via Horner using the axis's cached
/// coefficients.  Returns `(p_end, v_end)` in mm and mm/s.
#[inline]
fn eval_horner(
    mono: &[f32; 4],
    vel: &[f32; 3],
    piece_start_cycles: u64,
    now: u64,
    cycles_per_second: f32,
) -> (f32, f32) {
    let elapsed_cycles = now.saturating_sub(piece_start_cycles);
    let t = if cycles_per_second > 0.0 {
        elapsed_cycles as f32 / cycles_per_second
    } else {
        0.0_f32
    };
    let p = mono[0] + t * (mono[1] + t * (mono[2] + t * mono[3]));
    let v = vel[0] + t * (vel[1] + t * vel[2]);
    (p, v)
}
