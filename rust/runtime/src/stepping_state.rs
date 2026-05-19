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

/// Per-physical-stepper static configuration + cross-half atomic state.
///
/// One instance per stepper that the unified engine drives. Fields split
/// into:
///
/// - Static config (`step_pin`, `dir_pin`, `dir_invert`, `tmc_cs`): set at
///   `configure_axes` time, never mutated by the ISR.
/// - Cross-half atomics (`position_count`, the coil/phase fields): written
///   by the ISR, read by the foreground / telemetry without taking a
///   shared reference to `AxisConfig`. Atomics give us cross-half access
///   without aliasing issues.
///
/// The `last_coil_A` / `last_coil_B` names use uppercase A/B to mirror the
/// TMC5160 `XDIRECT` register's `COIL_A` / `COIL_B` field names verbatim —
/// the mapping is load-bearing for debugging, so the snake_case lint is
/// suppressed at the field level.
#[derive(Debug)]
#[allow(non_snake_case)]
pub struct StepperRef {
    pub step_pin: u32,
    pub dir_pin: u32,
    pub dir_invert: bool,

    pub position_count: AtomicI32,

    pub tmc_cs: Option<u32>,
    pub last_coil_A: AtomicI16,
    pub last_coil_B: AtomicI16,
    pub phase_offset_microsteps: AtomicI32,
    pub phase_offset_target: AtomicI32,
    pub last_phase_target: AtomicI32,
}

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
    pub piece: Option<BezierPieceMonomial>,
    pub piece_start_time_cycles: u64,
    pub last_step_count: i32,
    pub microstep_distance: f32,
    pub extrusion_per_xy_mm: f32,
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
            piece: None,
            piece_start_time_cycles: 0,
            last_step_count: 0,
            microstep_distance: 0.0,
            extrusion_per_xy_mm: 0.0,
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
    pub ds_xy_segment: f32,
    pub v_xy_this: f32,
    pub vdot_xy_accelerating: bool,
}

impl TickCaches {
    pub const fn new() -> Self {
        Self {
            p_prev: [0.0; N_AXES],
            v_prev: [0.0; N_AXES],
            v_xy_prev: 0.0,
            ds_xy_segment: 0.0,
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
