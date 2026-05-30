//! Per-axis state for the piece-ring walker engine.
//!
//! `AxisState` replaces the old `AxisConfig` and carries:
//!   - stepper bindings (unchanged)
//!   - a `RingDescriptor` for the axis's region of the shared piece_storage
//!   - ISR working cache: current piece coefficients and timing
//!   - sub-sample carry-over: p_prev / v_prev

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8};
use heapless::Vec;

use crate::piece_ring::RingDescriptor;

/// Maximum configured axes.
pub const MAX_AXES: usize = 8;

/// Legacy alias kept for FFI / tick.rs call sites that reference N_AXES.
/// The engine itself uses MAX_AXES; the tick dispatch constants still use the
/// alias for readability.
pub const N_AXES: usize = MAX_AXES;

pub const MAX_STEPPERS_PER_AXIS: usize = 4;

/// Per-stepper output mode.
///
/// `Pulse` drives the classic STEP/DIR GPIO path.
/// `Phase` drives the TMC5160 SPI coil-current path for true phase stepping.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepMode {
    Pulse = 0,
    Phase = 1,
}

/// Per-stepper Rust-side state.
#[allow(non_snake_case)]
#[derive(Debug)]
pub struct StepperRef {
    pub stepper_oid: u8,
    pub position_count: AtomicI32,
    /// OID of `command_config_spi` for this stepper's TMC driver.
    /// `None` means Pulse-only (no SPI traffic for this stepper).
    pub tmc_cs_oid: Option<u8>,
    pub last_coil_A: AtomicI16,
    pub last_coil_B: AtomicI16,
    pub phase_offset_microsteps: AtomicI32,
    pub phase_offset_target: AtomicI32,
    pub last_phase_target: AtomicI32,
}

impl StepperRef {
    pub fn new(stepper_oid: u8, tmc_cs_oid: Option<u8>) -> Self {
        Self {
            stepper_oid,
            position_count: AtomicI32::new(0),
            tmc_cs_oid,
            last_coil_A: AtomicI16::new(0),
            last_coil_B: AtomicI16::new(0),
            phase_offset_microsteps: AtomicI32::new(0),
            phase_offset_target: AtomicI32::new(0),
            last_phase_target: AtomicI32::new(0),
        }
    }
}

/// FFI ABI: per-stepper binding payload passed from C to Rust.
/// Sentinel: `tmc_cs_oid == 0xFF` means "no TMC driver" (Pulse-only stepper).
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct StepperBindingRust {
    pub stepper_oid: u8,
    pub tmc_cs_oid: u8,
    #[allow(clippy::pub_underscore_fields)]
    pub _pad: [u8; 2],
}
const _: () = assert!(core::mem::size_of::<StepperBindingRust>() == 4);

pub const TMC_CS_OID_NONE: u8 = 0xFF;

/// Per-logical-axis state for the piece-ring walker engine.
///
/// Holds:
/// - Stepper bindings (which physical steppers this axis drives).
/// - A `RingDescriptor` pointing into `RuntimeContext::piece_storage`.
/// - ISR working cache for the current piece (mono/vel coefficients,
///   start/end timestamps). Recomputed once per piece transition.
/// - Sub-sample position/velocity carry-over (`p_prev`, `v_prev`) for
///   the secant-slope step-timing path.
///
/// `mode` is atomic so the host can flip between Pulse and Phase without
/// a stop-the-world handshake (e.g. sensorless homing temporarily reverts
/// a phase-stepped axis to Pulse mode).
#[derive(Debug)]
pub struct AxisState {
    pub mode: AtomicU8,
    pub steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS>,
    pub microstep_distance: f32,
    // ── Ring bookkeeping (logical descriptor into shared piece_storage) ──
    pub ring: RingDescriptor,
    // ── ISR working cache for the current piece ──
    /// `true` when a piece has been armed and `piece_end_cycles` is valid.
    pub has_piece: bool,
    /// Position monomial coefficients (c0, c1, c2, c3) for the current piece.
    pub mono_coeffs: [f32; 4],
    /// Velocity coefficients (vc0, vc1, vc2) for the current piece.
    pub vel_coeffs: [f32; 3],
    /// MCU clock cycle at which the current piece starts.
    pub piece_start_cycles: u64,
    /// MCU clock cycle at which the current piece ends (cached on arm).
    pub piece_end_cycles: u64,
    pub last_step_count: i32,
    // ── Sub-sample timing carry ──
    pub p_prev: f32,
    pub v_prev: f32,
}

impl AxisState {
    /// Construct a default (unconfigured) `AxisState`. `mode` defaults to
    /// `StepMode::Pulse`; no ring is allocated; no piece is active.
    pub const fn new_unconfigured() -> Self {
        Self {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            microstep_distance: 0.0,
            ring: RingDescriptor::new_unconfigured(),
            has_piece: false,
            mono_coeffs: [0.0; 4],
            vel_coeffs: [0.0; 3],
            piece_start_cycles: 0,
            piece_end_cycles: 0,
            last_step_count: 0,
            p_prev: 0.0,
            v_prev: 0.0,
        }
    }

    /// Reset ISR working state (called by `configure_axis`).
    pub fn reset_isr_cache(&mut self) {
        self.has_piece = false;
        self.mono_coeffs = [0.0; 4];
        self.vel_coeffs = [0.0; 3];
        self.piece_start_cycles = 0;
        self.piece_end_cycles = 0;
        self.last_step_count = 0;
        self.p_prev = 0.0;
        self.v_prev = 0.0;
    }
}

/// Backward-compat alias for call sites in tick.rs that still reference the
/// old name.  Task 7 will update those sites; for now the alias keeps them
/// compiling without changes to the dispatch signatures.
pub type AxisConfig = AxisState;

/// ISR-local scratch state carried across consecutive sample ticks.
///
/// Kept for compatibility with `seed_position` and legacy callers that still
/// reference `TickCaches`.  For the new engine the per-axis `p_prev`/`v_prev`
/// fields on `AxisState` supersede this; `TickCaches` is retained as an alias
/// to avoid churning tick.rs call sites in this task.
#[derive(Debug)]
pub struct TickCaches {
    pub p_prev: [f32; MAX_AXES],
    pub v_prev: [f32; MAX_AXES],
}

impl TickCaches {
    pub const fn new() -> Self {
        Self {
            p_prev: [0.0; MAX_AXES],
            v_prev: [0.0; MAX_AXES],
        }
    }
}

impl Default for TickCaches {
    fn default() -> Self {
        Self::new()
    }
}
