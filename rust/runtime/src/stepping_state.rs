use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8};
use heapless::Vec;

use crate::motion_core::ArmedPiece;
use crate::piece_ring::RingDescriptor;

pub const MAX_AXES: usize = 8;

/// Legacy alias kept for FFI / tick.rs call sites that reference N_AXES.
pub const N_AXES: usize = MAX_AXES;

pub const MAX_STEPPERS_PER_AXIS: usize = 4;

#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepMode {
    Pulse = 0,
    Phase = 1,
}

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
/// `mode` is atomic so the host can flip between Pulse and Phase without
/// a stop-the-world handshake.
#[derive(Debug)]
pub struct AxisState {
    pub mode: AtomicU8,
    pub steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS>,
    pub microstep_distance: f32,
    pub ring: RingDescriptor,
    pub armed: Option<ArmedPiece>,
    pub last_step_count: i32,
    pub p_prev: f32,
    pub v_prev: f32,
}

impl AxisState {
    pub const fn new_unconfigured() -> Self {
        Self {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            microstep_distance: 0.0,
            ring: RingDescriptor::new_unconfigured(),
            armed: None,
            last_step_count: 0,
            p_prev: 0.0,
            v_prev: 0.0,
        }
    }

    pub fn reset_isr_cache(&mut self) {
        self.armed = None;
        self.last_step_count = 0;
        self.p_prev = 0.0;
        self.v_prev = 0.0;
    }
}

/// Backward-compat alias for call sites in tick.rs that still reference the old name.
pub type AxisConfig = AxisState;

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
