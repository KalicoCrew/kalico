//! State shapes for the unified stepping architecture.
//! See docs/superpowers/specs/2026-05-19-stepping-redesign-design.md
//! "State" section for the design rationale.

use core::sync::atomic::{AtomicI16, AtomicI32, AtomicU8};
use heapless::Vec;

use crate::monomial::BezierPieceMonomial;

pub const N_AXES: usize = 4;
pub const MAX_STEPPERS_PER_AXIS: usize = 4;

/// Per-stepper output mode for the unified stepping engine.
///
/// `Pulse` drives the classic STEP/DIR GPIO path (e.g. TMC2209 on Z, or a
/// non-phase-capable driver on any axis). `Phase` drives the TMC5160 SPI
/// coil-current path used for true phase stepping. The discriminant is
/// fixed via `#[repr(u8)]` so it can be stored in an `AtomicU8` and
/// reloaded with a plain `from`-style match on the ISR hot path.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StepMode {
    Pulse = 0,
    Phase = 1,
}

/// Per-stepper Rust-side state. GPIO + direction-inversion live in C
/// (`runtime_motor_steppers[][]`), so this struct only holds atomic state
/// the ISR reads/writes.
///
/// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §4.2.
#[derive(Debug)]
pub struct StepperRef {
    pub position_count: AtomicI32,
    /// OID of `command_config_spi` for this stepper's TMC driver. `None`
    /// means Pulse-only (no SPI traffic for this stepper).
    pub tmc_cs_oid: Option<u8>,
    #[allow(non_snake_case)]
    pub last_coil_A: AtomicI16,
    #[allow(non_snake_case)]
    pub last_coil_B: AtomicI16,
    pub phase_offset_microsteps: AtomicI32,
    pub phase_offset_target: AtomicI32,
    pub last_phase_target: AtomicI32,
}

impl StepperRef {
    pub fn new(tmc_cs_oid: Option<u8>) -> Self {
        Self {
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

/// FFI ABI: per-stepper binding payload, passed from C to Rust by
/// `kalico_runtime_configure_axis`. Sentinel: `tmc_cs_oid == 0xFF` means
/// "no TMC driver" (Pulse-only stepper). OID 0 is a legal SPI OID and
/// must not be conflated with "absent."
///
/// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §5.2.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct StepperBindingRust {
    pub tmc_cs_oid: u8,
    pub _pad: [u8; 3],
}
const _: () = assert!(core::mem::size_of::<StepperBindingRust>() == 4);

pub const TMC_CS_OID_NONE: u8 = 0xFF;

/// Per-logical-axis configuration: the steppers physically yoked to this
/// axis, the active Bezier piece being tracked, and the cached scalars
/// the sample ISR needs every tick.
///
/// `mode` is atomic so the host can flip an axis between `Pulse` and
/// `Phase` without a stop-the-world handshake (e.g. sensorless homing on
/// a normally-phase-stepped axis temporarily reverts to `Pulse`).
#[derive(Debug)]
pub struct AxisConfig {
    pub mode: AtomicU8,
    pub steppers: Vec<StepperRef, MAX_STEPPERS_PER_AXIS>,
    /// Active curve handle. `None` when no segment is armed or the curve
    /// is exhausted.
    pub curve_handle: Option<crate::curve_pool::CurveHandle>,
    /// Index into the loaded curve's `pieces` array. Advanced by
    /// `advance_piece_if_needed` (Task 9).
    pub piece_cursor: u16,
    /// Cached active piece (= curve.pieces[piece_cursor]). Refreshed
    /// only on piece-boundary advancement.
    pub piece: Option<BezierPieceMonomial>,
    pub piece_start_time_cycles: u64,
    pub last_step_count: i32,
    pub microstep_distance: f32,
}

impl AxisConfig {
    /// Construct a default (unconfigured) `AxisConfig`. `mode` defaults to
    /// `StepMode::Pulse`, the stepper-bindings list is empty, and no
    /// Bezier piece is active. All scalar fields are zero — the unified
    /// tick treats `microstep_distance == 0.0` as "axis is not yet
    /// configured" and skips step generation for that axis.
    ///
    /// `const fn` so it can be used in array literals during static /
    /// non-static struct construction (`Engine::new`).
    pub const fn new_unconfigured() -> Self {
        Self {
            mode: AtomicU8::new(StepMode::Pulse as u8),
            steppers: Vec::new(),
            curve_handle: None,
            piece_cursor: 0,
            piece: None,
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.0,
        }
    }
}

/// ISR-local scratch state carried across consecutive sample ticks.
///
/// All fields are tick-private — never observed by anything outside the
/// sample ISR — so plain values suffice (no atomics, no locks). Used by
/// the secant-slope sub-sample timing path to recover per-axis velocity
/// without re-evaluating the cubic at intermediate points.
#[derive(Debug)]
pub struct TickCaches {
    pub p_prev: [f32; N_AXES],
    pub v_prev: [f32; N_AXES],
    pub v_xy_prev: f32,
    pub v_xy_this: f32,
    pub vdot_xy_accelerating: bool,
}

impl TickCaches {
    pub const fn new() -> Self {
        Self {
            p_prev: [0.0; N_AXES],
            v_prev: [0.0; N_AXES],
            v_xy_prev: 0.0,
            v_xy_this: 0.0,
            vdot_xy_accelerating: false,
        }
    }
}

impl Default for TickCaches {
    fn default() -> Self {
        Self::new()
    }
}
