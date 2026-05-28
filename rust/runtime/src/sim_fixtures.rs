//! Pre-baked fixtures for the sim escape hatch and step-time engine tests.
//!
//! Compiled only when the `kalico-sim` Cargo feature is on (which is gated on
//! `CONFIG_KALICO_SIM=y` via `src/Makefile`). NEVER include in production
//! firmware — the production loading path validates caller-supplied data.

#![cfg(feature = "kalico-sim")]

/// Output buffer sizes for fixture geometry (legacy NURBS fixture helpers).
/// MAX_CONTROL_POINTS = 8, MAX_DIM = 3, MAX_KNOT_VECTOR_LEN = 12.
pub const FIXTURE_CPS_MAX: usize = 8 * 3;
pub const FIXTURE_KNOTS_MAX: usize = 12;
pub const FIXTURE_WEIGHTS_MAX: usize = 8;

/// Look up a fixture by `id`. Fills the caller-provided buffers and returns
/// `(degree, n_cp, n_knots, n_weights)` on success, `None` for unknown ids.
///
/// Fixtures:
///   0 = straight_line_x  (degree-1, 2 CP from (0,0,0) to (10,0,0))
///   1 = quarter_arc_xy   (degree-2 rational, 3 CP, R=20mm quarter)
///   2 = cubic_bezier_xy  (degree-3, 4 CP)
pub fn lookup(
    fixture_id: u16,
    cps_out: &mut [f32; FIXTURE_CPS_MAX],
    knots_out: &mut [f32; FIXTURE_KNOTS_MAX],
    weights_out: &mut [f32; FIXTURE_WEIGHTS_MAX],
) -> Option<(u8, usize, usize, usize)> {
    match fixture_id {
        0 => Some(straight_line_x(cps_out, knots_out, weights_out)),
        1 => Some(quarter_arc_xy(cps_out, knots_out, weights_out)),
        2 => Some(cubic_bezier_xy(cps_out, knots_out, weights_out)),
        _ => None,
    }
}

fn straight_line_x(
    cps: &mut [f32; FIXTURE_CPS_MAX],
    knots: &mut [f32; FIXTURE_KNOTS_MAX],
    weights: &mut [f32; FIXTURE_WEIGHTS_MAX],
) -> (u8, usize, usize, usize) {
    cps[0..3].copy_from_slice(&[0.0, 0.0, 0.0]);
    cps[3..6].copy_from_slice(&[10.0, 0.0, 0.0]);
    knots[..4].copy_from_slice(&[0.0, 0.0, 1.0, 1.0]);
    weights[..2].copy_from_slice(&[1.0, 1.0]);
    (1, 2, 4, 2)
}

fn quarter_arc_xy(
    cps: &mut [f32; FIXTURE_CPS_MAX],
    knots: &mut [f32; FIXTURE_KNOTS_MAX],
    weights: &mut [f32; FIXTURE_WEIGHTS_MAX],
) -> (u8, usize, usize, usize) {
    let r: f32 = 20.0;
    cps[0..3].copy_from_slice(&[r, 0.0, 0.0]);
    cps[3..6].copy_from_slice(&[r, r, 0.0]);
    cps[6..9].copy_from_slice(&[0.0, r, 0.0]);
    knots[..6].copy_from_slice(&[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    let cos_pi4 = core::f32::consts::FRAC_1_SQRT_2;
    weights[..3].copy_from_slice(&[1.0, cos_pi4, 1.0]);
    (2, 3, 6, 3)
}

fn cubic_bezier_xy(
    cps: &mut [f32; FIXTURE_CPS_MAX],
    knots: &mut [f32; FIXTURE_KNOTS_MAX],
    weights: &mut [f32; FIXTURE_WEIGHTS_MAX],
) -> (u8, usize, usize, usize) {
    cps[0..3].copy_from_slice(&[0.0, 0.0, 0.0]);
    cps[3..6].copy_from_slice(&[3.0, 5.0, 0.0]);
    cps[6..9].copy_from_slice(&[7.0, 5.0, 0.0]);
    cps[9..12].copy_from_slice(&[10.0, 0.0, 0.0]);
    knots[..8].copy_from_slice(&[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    weights[..4].copy_from_slice(&[1.0, 1.0, 1.0, 1.0]);
    (3, 4, 8, 4)
}

// ─── Integration-test helpers (engine + RuntimeContext) ────────────────────
//
// These functions are compiled as part of the `kalico-sim` feature so the
// `step_time_engine` integration test can use them. They require std/alloc
// for `Box::leak` and only compile on hosted environments.
#[cfg(not(target_os = "none"))]
mod init_test_runtime_impl {
    pub use ::alloc::boxed::Box;
}

#[cfg(not(target_os = "none"))]
use self::init_test_runtime_impl::Box;

/// Clock frequency used by `init_test_runtime`.
pub const TEST_CLOCK_FREQ: u32 = 180_000_000;

/// Z-axis step resolution used by `push_test_segment_linear_z`.
pub const TEST_Z_STEPS_PER_MM: f32 = 400.0;

/// Initialize a `RuntimeContext` suitable for the step-time engine tests.
#[cfg(not(target_os = "none"))]
#[allow(unsafe_code)]
pub fn init_test_runtime() -> Box<crate::state::RuntimeContext> {
    use core::cell::UnsafeCell;
    use heapless::spsc::Queue;

    use crate::clock::WidenState;
    use crate::config::{McuAxisConfig, MotorConfig};
    use crate::reclaim::RetirementTable;
    use crate::segment::KinematicTag;
    use crate::state::{EngineImpl, FgState, IsrState, RuntimeContext, SharedState};
    use crate::stream::FgStreamState;
    use crate::trace::{TRACE_RING_N, TraceSample};

    let q_producer = crate::c_segment_queue::Producer::new();
    let q_consumer = crate::c_segment_queue::Consumer::new();

    let trace_queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (t_producer, t_consumer) = trace_queue.split();

    let mut engine = EngineImpl::new(TEST_CLOCK_FREQ, 40_000);
    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig {
                steps_per_mm: 80.0,
                is_awd: false,
                invert_dir: false,
            }),
            Some(MotorConfig {
                steps_per_mm: 80.0,
                is_awd: false,
                invert_dir: false,
            }),
            Some(MotorConfig {
                steps_per_mm: TEST_Z_STEPS_PER_MM,
                is_awd: false,
                invert_dir: false,
            }),
            Some(MotorConfig {
                steps_per_mm: 80.0,
                is_awd: false,
                invert_dir: false,
            }),
        ],
        kinematics: KinematicTag::CartesianXyzAndE,
    });

    use crate::sizing::TOTAL_RING_PIECES;
    Box::new(RuntimeContext {
        fg: UnsafeCell::new(FgState {
            queue_producer: q_producer,
            trace_consumer: t_consumer,
            stream_state_machine: FgStreamState::Idle,
            current_stream_id: None,
            armed_t_start_t0: None,
            first_priming_segment_t_start: None,
            terminal_segment_id: None,
            flush_start_tick: None,
            retirement_table: RetirementTable::new(),
        }),
        isr: UnsafeCell::new(IsrState {
            queue_consumer: q_consumer,
            trace_producer: t_producer,
            engine,
            widen_state: WidenState::default(),
            pending_segment: None,
        }),
        shared: SharedState::new(),
        // Backing storage not used — we split from the leaked queues above.
        queue_storage: UnsafeCell::new(Queue::new()),
        trace_storage: UnsafeCell::new(Queue::new()),
        piece_storage: UnsafeCell::new(
            [crate::piece_ring::PieceEntry {
                start_time: 0,
                coeffs: [0.0; 4],
                duration: 0.0,
                _reserved: 0,
            }; TOTAL_RING_PIECES],
        ),
    })
}

/// Push a Z-only linear segment into the engine's piece ring, starting at
/// `t_start`.
///
/// Constructs a single cubic Bézier Z piece (collinear control points →
/// linear position) and loads it into the Z ring via `engine.push_pieces`.
///
/// - `t_start`: absolute MCU cycle at which the segment begins
/// - `velocity_mm_s`: Z velocity in mm/s (must be > 0)
/// - `duration_s`: segment duration in seconds
#[allow(unsafe_code)]
pub fn push_test_segment_linear_z_at(
    ctx: &mut crate::state::RuntimeContext,
    t_start: u64,
    velocity_mm_s: f32,
    duration_s: f32,
) {
    use crate::piece_ring::PieceEntry;

    let z_end_mm = velocity_mm_s * duration_s;

    // Single cubic Bernstein piece: collinear CPs give linear position(u).
    let piece = PieceEntry {
        start_time: t_start,
        coeffs: [0.0, z_end_mm / 3.0, z_end_mm * 2.0 / 3.0, z_end_mm],
        duration: duration_s,
        _reserved: 0,
    };

    // SAFETY: we hold &mut RuntimeContext so no concurrent ISR access exists.
    let isr = unsafe { &mut *ctx.isr.get() };
    let storage = unsafe { &mut *ctx.piece_storage.get() };
    let storage_slice: &mut [PieceEntry] = storage;

    // axis_idx=2 is Z. push_pieces allocates from the Z ring descriptor.
    let rc = isr.engine.push_pieces(2, &[piece], storage_slice);
    assert_eq!(rc, 0, "push_pieces for Z failed (ring not configured?)");
}

/// Push a Z-only linear segment into the engine's piece ring, starting at
/// cycle 0.
pub fn push_test_segment_linear_z(
    ctx: &mut crate::state::RuntimeContext,
    velocity_mm_s: f32,
    duration_s: f32,
) {
    push_test_segment_linear_z_at(ctx, 0, velocity_mm_s, duration_s);
}
