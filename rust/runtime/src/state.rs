#![allow(unsafe_code)]

use core::cell::UnsafeCell;
use portable_atomic::{AtomicBool, AtomicI32, AtomicU8, AtomicU16, AtomicU32, AtomicU64};

use crate::clock::WidenState;
use crate::engine::Engine;
use crate::piece_ring::PieceEntry;

pub use crate::sizing::RT_STORAGE_SIZE;

pub use crate::sizing::TOTAL_RING_PIECES;

pub use crate::sizing::PIECE_RING_SIZE;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum StepMode {
    Modulated = 0,
    StepTime = 1,
}

impl StepMode {
    pub fn from_u8(raw: u8) -> Option<StepMode> {
        match raw {
            0 => Some(StepMode::Modulated),
            1 => Some(StepMode::StepTime),
            _ => None,
        }
    }
}

pub const MAX_STEPPER_OIDS: usize = 16;

#[derive(Debug, Clone, Copy)]
pub struct TickState {
    pub dt: f32,
    pub positions: [f32; 4], // [x, y, z, e] logical
    pub motors: [f32; 4],    // [a, b, z, e] or [x, y, z, e] post-kinematic
}

pub type EngineImpl = Engine;

// `force_idle` handshake calls Klipper's `irq_save`/`irq_restore` to gate
// the queue-drain step. We import via thin C wrappers `runtime_irq_save` /
// `runtime_irq_restore` defined in `src/runtime_tick.c`, not the bare
// functions directly: Klipper's MCU build uses `-flto=auto -fwhole-program`,
// which lets the LTO inliner DCE the standalone `irq_save` / `irq_restore`
// symbols — leaving the kalico staticlib's references unresolvable at the
// final link. The wrappers are marked `used + externally_visible` on the C
// side so LTO keeps them.
unsafe extern "C" {
    pub fn runtime_irq_save() -> u32;
    pub fn runtime_irq_restore(flags: u32);
}

#[derive(Debug)]
pub struct FgState {
    pub current_stream_id: Option<u32>,
    pub armed_t_start_t0: Option<u64>,
    pub flush_start_tick: Option<u64>,
}

/// ISR-half state. Written exclusively by the TIM5 ISR
/// (`kalico_runtime_tick_sample`). Mutual exclusion is enforced by
/// Cortex-M NVIC priority arbitration.
#[allow(missing_debug_implementations)]
#[repr(C)]
pub struct IsrState {
    pub engine: EngineImpl,
    /// ISR is the sole writer of `widen_state`; foreground must not call
    /// `widen()` directly.
    pub widen_state: WidenState,
    pub last_tick_now: Option<u64>,
}

impl IsrState {
    pub fn raw_ref_from_ctx(ctx: &RuntimeContext) -> *const Self {
        core::cell::UnsafeCell::raw_get(core::ptr::addr_of!(ctx.isr))
    }
}

#[derive(Debug)]
#[repr(C)]
pub struct SharedState {
    pub last_error: AtomicI32,
    pub runtime_status: AtomicU8,
    pub stream_open: AtomicBool,
    /// Foreground sets `force_idle` first, ack-waits, then clears
    /// `stream_open` (flush handshake decision A).
    pub force_idle: AtomicBool,
    pub acked_force_idle: AtomicBool,
    /// §11.4 widened-clock seqlock — foreground reads, ISR writes.
    pub widened_now_lo: AtomicU32,
    pub widened_now_hi: AtomicU32,
    pub widened_now_seq: AtomicU32,
    pub sample_drop_pending: AtomicBool,
    /// fault_detail 0xFB: low 32 bits of `now - seg.t_start` inside
    /// `runtime_modulated_tick`. If stuck at 0, the clock is behind
    /// `seg.t_start` and the `elapsed >= duration` retirement check can't fire.
    pub last_modulated_elapsed_lo: AtomicU32,
    /// fault_detail 0xFC: companion to `last_modulated_elapsed_lo`.
    /// `0xFB >= 0xFC` means retirement should fire on the next tick.
    pub last_modulated_duration_lo: AtomicU32,
    pub modulated_retire_attempts: AtomicU32,
    pub modulated_retire_successes: AtomicU32,
    pub last_retire_consumers_after_clear: AtomicU32,
    /// §9.2: last latched fault's encoded `fault_detail` payload.
    /// Set in lockstep with `last_error`. `0` when no fault has latched.
    pub fault_detail: AtomicU32,
    /// Most-recently-dispatched scheduler timer func at `-311` fault time.
    /// `0` until `-311`.
    pub tick_blocker_func: AtomicU32,
    /// Stacked exception-frame PC at TIM5 handler entry, captured on the
    /// `-311` path. Primary `addr2line` target. `0` until `-311`.
    pub tick_blocker_pc: AtomicU32,
    /// Stacked xPSR exception number at `-311` time (`xPSR & 0x1FF`).
    /// `0` = foreground was interrupted; nonzero = that IRQ overran into TIM5.
    pub tick_blocker_exc: AtomicU32,
    pub stepper_counts: [AtomicI32; MAX_STEPPER_OIDS],
    pub step_modes: [AtomicU8; MAX_STEPPER_OIDS],
    /// Per-motor phase-stepping SPI config. Packed (`spi_bus_id << 8 |
    /// cs_pin_id`). `0xFFFF` means "no phase config — use the StepPulse
    /// output path."
    pub phase_config: [AtomicU16; MAX_STEPPER_OIDS],
    /// `phase_slot_idx[motor_idx]` is the kinematic slot whose commanded
    /// `motors[slot_idx]` position drives motor `motor_idx`'s XDIRECT output.
    /// Unused entries hold `0xFF`.
    pub phase_slot_idx: [AtomicU8; MAX_STEPPER_OIDS],
    /// ISR loops `0..phase_motor_count` rather than scanning the full array.
    /// `0` disables phase stepping on this MCU.
    pub phase_motor_count: AtomicU8,

    pub producer_pending: AtomicBool,
    pub producer_runs_total: AtomicU64,
    pub consumer_pulses_total: [AtomicU64; 4],
    pub consumer_underrun_total: [AtomicU64; 4],
    pub ring_high_water: [AtomicU32; 4],
    pub producer_steps_pushed_total: AtomicU64,
    pub producer_motor_finished_curve_total: AtomicU64,
    pub producer_segment_retired_total: AtomicU64,
    /// Per-stage running MAX cycle counts for `runtime::tick::isr_sample_tick`.
    /// Read via status_drain rotation as tags 0xE6/0xE7/0xE8.
    /// If any stage approaches the TIM5 period (~13000 cycles at 40 kHz on H7),
    /// that stage is starving the foreground.
    pub isr_widen_cycles_max: AtomicU32,
    pub isr_arm_cycles_max: AtomicU32,
    pub isr_eval_cycles_max: AtomicU32,
    /// Increments per ISR that exceeds 30000 cycles (~58 µs) of total body
    /// time — circuit-breaker signal.
    pub isr_overrun_count: AtomicU32,
    pub isr_deq_some_count: AtomicU32,
    pub isr_deq_none_count: AtomicU32,
    pub isr_parked_count: AtomicU32,
    pub isr_armed_count: AtomicU32,
    /// Comparands at the most-recent park/arm decision. If
    /// `isr_parked_count > 0`, `isr_last_t_start_lo > isr_last_widened_lo`
    /// is the actual park reason.
    pub isr_last_t_start_lo: AtomicU32,
    pub isr_last_widened_lo: AtomicU32,
    /// High 32 bits of `seg.t_start` and `widened_now` at the most-recent
    /// park/arm decision. Distinguishes wrong-epoch t_start from narrowed-to-u32-on-wire
    /// t_start (`isr_last_t_start_hi == 0` while `isr_last_widened_hi != 0`).
    pub isr_last_t_start_hi: AtomicU32,
    pub isr_last_widened_hi: AtomicU32,
    /// Low / high 32 bits of `now.saturating_sub(seg.t_start)` at the
    /// same park/arm decision. If ≈ uptime×clock_freq, t_start was 0 or
    /// in the wrong epoch.
    pub isr_arm_delta_lo: AtomicU32,
    pub isr_arm_delta_hi: AtomicU32,
    pub isr_last_p_end_bits: AtomicU32,
    pub isr_last_microstep_bits: AtomicU32,
    pub isr_last_c0_bits: AtomicU32,
    /// f32-bits of `t_local` (sec since piece start). If huge, the
    /// time-domain mapping between seg.t_start (cycles) and widened_now
    /// (cycles) is broken or duration is wrongly tiny.
    pub isr_last_t_local_bits: AtomicU32,
    pub isr_step_push_count: AtomicU32,
    pub isr_last_signed_steps: AtomicU32,
    pub isr_pulse_call_count: AtomicU32,
    pub isr_pulse_zero_step_count: AtomicU32,
    pub isr_pulse_bad_mstep_count: AtomicU32,
    pub isr_phase_call_count: AtomicU32,
    /// Packed `(axis_idx << 16) | raw_mode_byte` from the most recent
    /// `dispatch_axis` call.
    pub isr_last_axis_mode_packed: AtomicU32,
    /// Packed `(target_step_count low16 << 16) | prev_step_count low16`
    /// from the most recent Pulse dispatch.
    pub isr_last_step_counts_packed: AtomicU32,
    /// Raw packed `seg.x_handle` of the most recently armed segment.
    /// `0xFFFE_FFFE` = UNUSED_SENTINEL.
    pub isr_last_arm_x_handle: AtomicU32,
    /// Outcome of arm for X axis: 0 = never armed, 1 = UNUSED handle,
    /// 2 = `lookup_active` returned None (slot/gen mismatch),
    /// 3 = curve loaded but `piece_count == 0`, 4 = OK.
    pub isr_last_arm_x_outcome: AtomicU32,
    pub isr_last_arm_x_piece_count: AtomicU32,
    /// `participating_mask` snapshot at end of arm.
    /// Bit 0 = A/X, 1 = B/Y, 2 = Z, 3 = E. If 0, no axes participated
    /// → instant retire ("ghost retire" symptom).
    pub isr_last_arm_participating: AtomicU32,
    /// f32-bits of `pieces[0].duration` for the X curve at arm. If 0
    /// (= 0.0 s), `bernstein_to_monomial_with_duration` divides by 0
    /// → inf/NaN → signed_steps=0.
    pub isr_last_arm_x_piece0_duration_bits: AtomicU32,
    /// Incremented in `producer_step` every time `producer_current.is_none()`
    /// is entered. Cross-check with `producer_segment_dequeued_total`:
    ///   `observed_none == dequeued` → ISR not clearing producer_current after retire.
    ///   `observed_none >> dequeued` → `queue.dequeue()` returns None despite entries (SPSC bug).
    pub producer_observed_none_total: AtomicU64,
    pub producer_step_last_len_snapshot: AtomicU32,
    pub producer_step_current_is_some_snapshot: AtomicU8,
    pub producer_current_present: AtomicBool,
    pub producer_fetch_attempts_total: AtomicU64,
    pub producer_enqueue_success_total: AtomicU64,
    pub last_push_segment_result: AtomicI32,
    pub producer_primary_resolved_total: AtomicU64,
    pub producer_primary_stale_total: AtomicU64,
    pub producer_primary_unused_total: AtomicU64,
    pub push_segment_all_unused_total: AtomicU64,
    pub last_push_x_handle_packed: AtomicU32,
    pub last_push_y_handle_packed: AtomicU32,
    pub last_push_consumers_remaining: AtomicU32,
    /// `cps[0]` (start CP, mm) of the last resolved primary X curve, raw f32 bits.
    pub last_resolved_primary_cps_0: AtomicU32,
    /// `cps[3]` (end CP, mm) of the last resolved primary X curve, raw f32
    /// bits. Matching `cps_0` means zero displacement — indicates a planner-side bug.
    pub last_resolved_primary_cps_3: AtomicU32,
    /// CoreXY-combined `cps[0]` after `kine.combine` for motor A (A = X + Y),
    /// f32 bits. Compare with `last_resolved_primary_cps_0` to detect kinematic-mixing bugs.
    pub last_combined_motor_a_cps_0: AtomicU32,
    pub last_combined_motor_a_cps_3: AtomicU32,

    pub queue_high_water: [AtomicU32; 4],
    pub queue_overflow_count: [AtomicU32; 4],
    pub spi_saturated_samples: AtomicU32,
    pub sample_isr_peak_cycles: AtomicU32,
    pub per_axis_consumer_peak_cycles: [AtomicU32; 4],

    pub dispatcher_floor_cycles: AtomicU32,
    pub sample_period_cycles: AtomicU32,

    pub max_phase_offset_ramp_per_sample: AtomicU16,
}

impl SharedState {
    pub const fn new() -> Self {
        Self {
            last_error: AtomicI32::new(0),
            runtime_status: AtomicU8::new(crate::engine::RuntimeStatus::Idle as u8),
            stream_open: AtomicBool::new(false),
            force_idle: AtomicBool::new(false),
            acked_force_idle: AtomicBool::new(false),
            widened_now_lo: AtomicU32::new(0),
            widened_now_hi: AtomicU32::new(0),
            widened_now_seq: AtomicU32::new(0),
            sample_drop_pending: AtomicBool::new(false),
            last_modulated_elapsed_lo: AtomicU32::new(0),
            last_modulated_duration_lo: AtomicU32::new(0),
            modulated_retire_attempts: AtomicU32::new(0),
            modulated_retire_successes: AtomicU32::new(0),
            last_retire_consumers_after_clear: AtomicU32::new(0),
            fault_detail: AtomicU32::new(0),
            tick_blocker_func: AtomicU32::new(0),
            tick_blocker_pc: AtomicU32::new(0),
            tick_blocker_exc: AtomicU32::new(0),
            stepper_counts: [
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
                AtomicI32::new(0),
            ],
            step_modes: [
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
                AtomicU8::new(StepMode::StepTime as u8),
            ],
            phase_config: [
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
                AtomicU16::new(crate::phase_config::NONE_SENTINEL),
            ],
            phase_slot_idx: [
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
                AtomicU8::new(0xFF),
            ],
            phase_motor_count: AtomicU8::new(0),
            producer_pending: AtomicBool::new(false),
            producer_runs_total: AtomicU64::new(0),
            consumer_pulses_total: [
                AtomicU64::new(0),
                AtomicU64::new(0),
                AtomicU64::new(0),
                AtomicU64::new(0),
            ],
            consumer_underrun_total: [
                AtomicU64::new(0),
                AtomicU64::new(0),
                AtomicU64::new(0),
                AtomicU64::new(0),
            ],
            ring_high_water: [
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
                AtomicU32::new(0),
            ],
            producer_steps_pushed_total: AtomicU64::new(0),
            producer_motor_finished_curve_total: AtomicU64::new(0),
            producer_segment_retired_total: AtomicU64::new(0),
            isr_widen_cycles_max: AtomicU32::new(0),
            isr_arm_cycles_max: AtomicU32::new(0),
            isr_eval_cycles_max: AtomicU32::new(0),
            isr_overrun_count: AtomicU32::new(0),
            isr_deq_some_count: AtomicU32::new(0),
            isr_deq_none_count: AtomicU32::new(0),
            isr_parked_count: AtomicU32::new(0),
            isr_armed_count: AtomicU32::new(0),
            isr_last_t_start_lo: AtomicU32::new(0),
            isr_last_widened_lo: AtomicU32::new(0),
            isr_last_t_start_hi: AtomicU32::new(0),
            isr_last_widened_hi: AtomicU32::new(0),
            isr_arm_delta_lo: AtomicU32::new(0),
            isr_arm_delta_hi: AtomicU32::new(0),
            isr_last_p_end_bits: AtomicU32::new(0),
            isr_last_microstep_bits: AtomicU32::new(0),
            isr_last_c0_bits: AtomicU32::new(0),
            isr_last_t_local_bits: AtomicU32::new(0),
            isr_step_push_count: AtomicU32::new(0),
            isr_last_signed_steps: AtomicU32::new(0),
            isr_pulse_call_count: AtomicU32::new(0),
            isr_pulse_zero_step_count: AtomicU32::new(0),
            isr_pulse_bad_mstep_count: AtomicU32::new(0),
            isr_phase_call_count: AtomicU32::new(0),
            isr_last_axis_mode_packed: AtomicU32::new(0),
            isr_last_step_counts_packed: AtomicU32::new(0),
            isr_last_arm_x_handle: AtomicU32::new(0),
            isr_last_arm_x_outcome: AtomicU32::new(0),
            isr_last_arm_x_piece_count: AtomicU32::new(0),
            isr_last_arm_participating: AtomicU32::new(0),
            isr_last_arm_x_piece0_duration_bits: AtomicU32::new(0),
            producer_observed_none_total: AtomicU64::new(0),
            producer_step_last_len_snapshot: AtomicU32::new(0),
            producer_step_current_is_some_snapshot: AtomicU8::new(0),
            producer_current_present: AtomicBool::new(false),
            producer_fetch_attempts_total: AtomicU64::new(0),
            producer_enqueue_success_total: AtomicU64::new(0),
            last_push_segment_result: AtomicI32::new(0),
            producer_primary_resolved_total: AtomicU64::new(0),
            producer_primary_stale_total: AtomicU64::new(0),
            producer_primary_unused_total: AtomicU64::new(0),
            push_segment_all_unused_total: AtomicU64::new(0),
            last_push_x_handle_packed: AtomicU32::new(0),
            last_push_y_handle_packed: AtomicU32::new(0),
            last_push_consumers_remaining: AtomicU32::new(0),
            last_resolved_primary_cps_0: AtomicU32::new(0),
            last_resolved_primary_cps_3: AtomicU32::new(0),
            last_combined_motor_a_cps_0: AtomicU32::new(0),
            last_combined_motor_a_cps_3: AtomicU32::new(0),

            queue_high_water: [const { AtomicU32::new(0) }; 4],
            queue_overflow_count: [const { AtomicU32::new(0) }; 4],
            spi_saturated_samples: AtomicU32::new(0),
            sample_isr_peak_cycles: AtomicU32::new(0),
            per_axis_consumer_peak_cycles: [const { AtomicU32::new(0) }; 4],

            dispatcher_floor_cycles: AtomicU32::new(0),
            sample_period_cycles: AtomicU32::new(0),

            max_phase_offset_ramp_per_sample: AtomicU16::new(0),
        }
    }
}

impl Default for SharedState {
    fn default() -> Self {
        Self::new()
    }
}

/// Half-split runtime context. `fg` and `isr` are disjoint memory regions
/// behind `UnsafeCell` so the FFI shim can project to either half via
/// `core::ptr::addr_of!` + `UnsafeCell::raw_get` without materializing
/// `&mut RuntimeContext`. At most ONE `&mut FgState` OR `&mut IsrState`
/// (disjoint memory) exists at a time. `shared` uses no `UnsafeCell` — all
/// writes go through atomics.
///
/// C/Rust boundary: `piece_storage` lives inside the C-declared `rt_storage`
/// buffer; C owns linker-section placement on the MCU
/// (docs/kalico-rewrite/mcu-c-rust-boundary.md rule B2). No additional
/// `#[link_section]` is needed here.
#[allow(missing_debug_implementations)]
pub struct RuntimeContext {
    pub fg: UnsafeCell<FgState>,
    pub isr: UnsafeCell<IsrState>,
    pub shared: SharedState,
    pub piece_storage: UnsafeCell<[PieceEntry; TOTAL_RING_PIECES]>,
}

// SAFETY: `UnsafeCell` is `!Sync` by default — we provide `Sync` for
// `RuntimeContext` because the FFI shim only ever forms shared `&` and
// projects to disjoint `&mut FgState` / `&mut IsrState` via raw pointers,
// never `&mut RuntimeContext`.
unsafe impl Sync for RuntimeContext {}

unsafe extern "C" {
    static runtime_clock_freq: u32;
    /// TIM5 ISR fire rate (Hz). Defined in `src/runtime_tick.c` as
    /// `CONFIG_KALICO_MOTION_SAMPLE_RATE_HZ` (defaults: 40000 on H7,
    /// 20000 on F4, 10000 on Linux sim).
    static runtime_sample_rate_hz: u32;
}

impl RuntimeContext {
    /// # Safety
    ///
    /// Caller must guarantee single-threaded init before any FFI call hits the
    /// runtime. Writes through raw-pointer projections; never materializes
    /// `&mut RuntimeContext`; sound to call against `MaybeUninit::as_mut_ptr()`.
    pub unsafe fn init(rt_ptr: *mut RuntimeContext) {
        // SAFETY: caller guarantees `rt_ptr` is valid for writes and
        // exclusively-owned for the duration of init. We only form raw
        // pointers to fields and never a `&mut RuntimeContext`.
        unsafe {
            let shared_ptr = core::ptr::addr_of_mut!((*rt_ptr).shared);
            shared_ptr.write(SharedState::new());

            // `UnsafeCell` is `#[repr(transparent)]`; writing zeroes is safe
            // because `PieceEntry` is `#[repr(C)]` with no padding and
            // all-zero is a valid bit pattern.
            let ps_ptr = core::ptr::addr_of_mut!((*rt_ptr).piece_storage);
            core::ptr::write_bytes(
                ps_ptr.cast::<u8>(),
                0u8,
                core::mem::size_of::<UnsafeCell<[crate::piece_ring::PieceEntry; TOTAL_RING_PIECES]>>(
                ),
            );

            let freq = core::ptr::read_volatile(core::ptr::addr_of!(runtime_clock_freq));
            let sample_rate_hz =
                core::ptr::read_volatile(core::ptr::addr_of!(runtime_sample_rate_hz));

            // Integer division is intentional: round-to-nearest cycles.
            #[allow(clippy::integer_division)]
            let sample_period_cycles_init: u32 = if sample_rate_hz == 0 {
                0
            } else {
                (freq + sample_rate_hz / 2) / sample_rate_hz
            };
            #[allow(clippy::integer_division)]
            let dispatcher_floor_cycles_init: u32 = freq / 1_000_000;
            let shared_ref: *const SharedState = core::ptr::addr_of!((*rt_ptr).shared);
            (*shared_ref).sample_period_cycles.store(
                sample_period_cycles_init,
                core::sync::atomic::Ordering::Release,
            );
            (*shared_ref).dispatcher_floor_cycles.store(
                dispatcher_floor_cycles_init,
                core::sync::atomic::Ordering::Release,
            );

            let fg_ptr = core::ptr::addr_of_mut!((*rt_ptr).fg);
            fg_ptr.write(UnsafeCell::new(FgState {
                current_stream_id: None,
                armed_t_start_t0: None,
                flush_start_tick: None,
            }));

            // Initialize IsrState in-place to avoid materializing the
            // multi-KB `EngineImpl` on the 8 KB MCU stack.
            let isr_ptr = core::ptr::addr_of_mut!((*rt_ptr).isr);
            let inner_ptr: *mut IsrState = UnsafeCell::raw_get(isr_ptr);
            EngineImpl::init_in_place_production(
                core::ptr::addr_of_mut!((*inner_ptr).engine),
                freq,
                sample_rate_hz,
            );
            core::ptr::addr_of_mut!((*inner_ptr).widen_state).write(WidenState::default());
            core::ptr::addr_of_mut!((*inner_ptr).last_tick_now).write(None);
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SetStepModeError {
    CapabilityMissing,
    OutOfRange,
}

/// `Release` ordering on the store pairs with `Acquire` loads in the
/// engine ISR and any foreground reader of `step_modes`.
pub fn set_step_mode(
    shared: &SharedState,
    stepper_idx: u8,
    mode: StepMode,
    mcu_supports_phase: bool,
) -> Result<(), SetStepModeError> {
    if (stepper_idx as usize) >= MAX_STEPPER_OIDS {
        return Err(SetStepModeError::OutOfRange);
    }
    if mode == StepMode::Modulated && !mcu_supports_phase {
        return Err(SetStepModeError::CapabilityMissing);
    }
    // SAFETY: bounds checked above: stepper_idx < MAX_STEPPER_OIDS == step_modes.len()
    #[allow(clippy::indexing_slicing)]
    shared.step_modes[stepper_idx as usize]
        .store(mode as u8, core::sync::atomic::Ordering::Release);
    Ok(())
}

#[cfg(test)]
mod size_task18;

#[cfg(test)]
mod tests;
