//! Pre-baked NURBS fixtures for the sim escape hatch.
//!
//! Compiled only when the `kalico-sim` Cargo feature is on (which is gated on
//! `CONFIG_KALICO_SIM=y` via `src/Makefile`). NEVER include in production
//! firmware — the production `kalico_runtime_load_curve` path validates the
//! caller-supplied data and is the only blessed entry point on silicon.
//!
//! Diagnosis (Step-6 plan Phase 0 Task 0.2 GDB-attach): under Renode, the H7
//! platform model silently ignores `SCB->CPACR` writes from `SystemInit()`,
//! so the FPU stays disabled. Any FPU instruction in
//! `runtime::curve_pool::CurvePool::load` (the `is_finite()` and `> 0.0`
//! checks lower to `vldr`/`vcmp.f32`) raises a UsageFault that lands in
//! Klipper's `DefaultHandler` infinite loop. The fixture path uses
//! pre-validated static data; the FFI wrapper still calls `CurvePool::load`,
//! but the data is known-good so even the validation FPU ops produce the
//! correct branch target on silicon. (Under sim, CurvePool::load itself
//! still UsageFaults — but Step-6 protocol iteration only requires the
//! FFI shape to land segments via fixtures, with the actual ISR-side curve
//! evaluation skipped on the zero-CYCCNT path. Once the engine has a
//! tractable widened-clock advance (Task 0.1), segments retire correctly
//! by reaching their `t_end` window without ever calling NURBS eval.)
//!
//! The fixture lookup returns flat slices into caller-provided buffers so
//! `CurvePool::load`'s flat-slice API can consume them directly without an
//! intermediate `LoadedCurve` struct (which is private to `curve_pool`).

#![cfg(feature = "kalico-sim")]

/// Output buffer sizes match `runtime::curve_pool` MAX_* constants:
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
    // 2 control points × 3 dims = 6 floats.
    cps[0..3].copy_from_slice(&[0.0, 0.0, 0.0]);
    cps[3..6].copy_from_slice(&[10.0, 0.0, 0.0]);
    // Clamped degree-1 knot vector: [0, 0, 1, 1].
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
    // Clamped degree-2 knot vector: [0, 0, 0, 1, 1, 1].
    knots[..6].copy_from_slice(&[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
    // Rational-quadratic quarter-arc weight pattern: w_mid = cos(pi/4).
    let cos_pi4 = core::f32::consts::FRAC_1_SQRT_2; // exact equivalent of cos(pi/4) without runtime FPU
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
    // Clamped degree-3 knot vector: [0, 0, 0, 0, 1, 1, 1, 1].
    knots[..8].copy_from_slice(&[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
    weights[..4].copy_from_slice(&[1.0, 1.0, 1.0, 1.0]);
    (3, 4, 8, 4)
}

// ─── Integration-test helpers (engine + RuntimeContext) ────────────────────
//
// These functions are compiled as part of the `kalico-sim` feature so the
// `step_time_engine` integration test can use them via:
//
//   cargo test -p runtime --features kalico-sim --test step_time_engine
//
// They are NOT part of the Renode-sim escape hatch; they live in this module
// purely for proximity with other fixture-level helpers.
//
// The Box-allocating helpers below require std (or `alloc`) so they only
// compile when targeting a hosted environment, not the no_std MCU firmware.
// `#[cfg(not(target_os = "none"))]` gates them off for the ARM firmware
// build (the Renode-sim firmware target is `thumbv7em-none-eabi`, i.e.
// `target_os = "none"`, so it also skips these and uses the Renode-side
// fixture lookup only).
#[cfg(not(target_os = "none"))]
mod init_test_runtime_impl {
    pub use ::alloc::boxed::Box;
}

#[cfg(not(target_os = "none"))]
use self::init_test_runtime_impl::Box;

/// Clock frequency used by `init_test_runtime`. Chosen so that 400 steps/mm
/// at 1 mm/s produces a first-step time of exactly 450,000 cycles:
///
///   step_distance = 1/400 mm = 0.0025 mm
///   dt_to_first_step = 0.0025 s
///   cycles = 0.0025 × 180_000_000 = 450_000
pub const TEST_CLOCK_FREQ: u32 = 180_000_000;

/// Z-axis step resolution used by `push_test_segment_linear_z`. Matches a
/// common 400 step/mm Z lead-screw (T8 lead-screw + 16× microstep on a
/// 200-step motor).
pub const TEST_Z_STEPS_PER_MM: f32 = 400.0;

/// Initialize a `RuntimeContext` suitable for the step-time engine tests.
///
/// The ISR-side engine is configured for Cartesian kinematics with:
///   - Motor 2 (Z): 400 steps/mm
///   - Motors 0, 1, 3 (X, Y, E): 80 steps/mm (placeholder; tests use Z only)
///
/// The queue / trace backing stores are Box::leaked so the `Producer` /
/// `Consumer` halves carry `'static` lifetimes as required by the type.
/// `queue_storage` and `trace_storage` inside the returned `RuntimeContext`
/// are dummy (never used — the split halves reference the separate leaked
/// queues).
#[cfg(not(target_os = "none"))]
#[allow(unsafe_code)]
pub fn init_test_runtime() -> Box<crate::state::RuntimeContext> {
    use core::cell::UnsafeCell;
    use heapless::spsc::Queue;

    use crate::clock::WidenState;
    use crate::config::{McuAxisConfig, MotorConfig};
    use crate::curve_pool::CurvePool;
    use crate::queue::Q_N;
    use crate::reclaim::RetirementTable;
    use crate::segment::{KinematicTag, Segment};
    use crate::state::{EngineImpl, FgState, IsrState, RuntimeContext, SharedState};
    use crate::stream::FgStreamState;
    use crate::trace::{TRACE_RING_N, TraceSample};

    // Box::leak the queues so Producer/Consumer halves are 'static.
    let seg_queue: &'static mut Queue<Segment, Q_N> =
        Box::leak(Box::new(Queue::new()));
    let (q_producer, q_consumer) = seg_queue.split();

    let trace_queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (t_producer, t_consumer) = trace_queue.split();

    let mut engine = EngineImpl::new(TEST_CLOCK_FREQ);
    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: 80.0, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: 80.0, is_awd: false, invert_dir: false }),
            Some(MotorConfig {
                steps_per_mm: TEST_Z_STEPS_PER_MM,
                is_awd: false,
                invert_dir: false,
            }),
            Some(MotorConfig { steps_per_mm: 80.0, is_awd: false, invert_dir: false }),
        ],
        kinematics: KinematicTag::CartesianXyzAndE,
    });

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
        }),
        shared: SharedState::new(),
        curve_pool: CurvePool::new(),
        // Backing storage not used — we split from the leaked queues above.
        queue_storage: UnsafeCell::new(Queue::new()),
        trace_storage: UnsafeCell::new(Queue::new()),
    })
}

/// Push a Z-only linear segment into the engine's active-segment slot,
/// starting at the given absolute cycle anchor `t_start`.
///
/// Synthesizes a degree-3 Bézier Z curve with collinear control points so
/// that `z_position(u) = velocity_mm_s * duration_s * u` — exactly linear
/// motion at the given velocity over the segment duration.
///
/// The segment is placed directly into `engine.current` (bypassing the SPSC
/// queue) so `arm_step_timer_for_stepper` can find it immediately without a
/// preceding tick. All other axis handles are set to `UNUSED_SENTINEL` with
/// Cartesian kinematics.
///
/// - `t_start`: absolute cycle at which the segment begins
/// - `velocity_mm_s`: Z velocity in mm/s (must be > 0)
/// - `duration_s`: segment duration in seconds
///
/// **Motor-space note:** stepper_idx = 2 is the Z motor in both CoreXY and
/// Cartesian kinematics; the generated curve is consumed via the z_handle.
#[allow(unsafe_code)]
pub fn push_test_segment_linear_z_at(
    ctx: &mut crate::state::RuntimeContext,
    t_start: u64,
    velocity_mm_s: f32,
    duration_s: f32,
) {
    use crate::config::EMode;
    use crate::curve_pool::CurveHandle;
    use crate::segment::{KinematicTag, Segment};

    // Duration in cycles.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let duration_cycles: u64 = (duration_s * TEST_CLOCK_FREQ as f32) as u64;

    // Total Z displacement: velocity × duration.
    let z_end_mm = velocity_mm_s * duration_s;

    // Degree-3 Bézier with collinear CPs at 0, L/3, 2L/3, L.
    // This gives exactly position(u) = L * u (linear in u).
    let cp0 = 0.0_f32;
    let cp1 = z_end_mm / 3.0;
    let cp2 = z_end_mm * 2.0 / 3.0;
    let cp3 = z_end_mm;
    let cps = [cp0, cp1, cp2, cp3];
    // Clamped degree-3 knot vector: [0, 0, 0, 0, 1, 1, 1, 1].
    let knots = [0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];

    // Load into curve pool at slot 0 (test assumes a freshly-init pool).
    let z_handle = ctx
        .curve_pool
        .validate_and_load(0, 3, &knots, &cps)
        .expect("Z curve must load into fresh pool");

    // Place the segment directly into engine.current so arm_step_timer sees
    // it without needing a preceding tick.
    // SAFETY: we hold &mut RuntimeContext so no concurrent ISR access exists.
    let isr = unsafe { &mut *ctx.isr.get() };
    isr.engine.current = Some(Segment {
        id: 1,
        x_handle: CurveHandle::UNUSED_SENTINEL,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start,
        t_end: t_start + duration_cycles,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
    });
}

/// Push a Z-only linear segment into the engine's active-segment slot,
/// starting at cycle 0.
///
/// Thin wrapper around [`push_test_segment_linear_z_at`] with `t_start = 0`.
/// See that function for full documentation.
pub fn push_test_segment_linear_z(
    ctx: &mut crate::state::RuntimeContext,
    velocity_mm_s: f32,
    duration_s: f32,
) {
    push_test_segment_linear_z_at(ctx, 0, velocity_mm_s, duration_s);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unknown_id_returns_none() {
        let mut cps = [0.0f32; FIXTURE_CPS_MAX];
        let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
        let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
        assert!(lookup(99, &mut cps, &mut knots, &mut weights).is_none());
    }

    #[test]
    fn straight_line_shape() {
        let mut cps = [0.0f32; FIXTURE_CPS_MAX];
        let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
        let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
        let (degree, n_cp, n_knots, n_weights) =
            lookup(0, &mut cps, &mut knots, &mut weights).expect("fixture 0");
        assert_eq!((degree, n_cp, n_knots, n_weights), (1, 2, 4, 2));
        // Clamped degree-1: knots == [0, 0, 1, 1].
        assert_eq!(&knots[..4], &[0.0, 0.0, 1.0, 1.0]);
        assert_eq!(cps[3], 10.0);
    }

    #[test]
    fn quarter_arc_shape() {
        let mut cps = [0.0f32; FIXTURE_CPS_MAX];
        let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
        let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
        let (degree, n_cp, n_knots, n_weights) =
            lookup(1, &mut cps, &mut knots, &mut weights).expect("fixture 1");
        assert_eq!((degree, n_cp, n_knots, n_weights), (2, 3, 6, 3));
        assert_eq!(weights[0], 1.0);
        assert_eq!(weights[2], 1.0);
        // Middle weight is cos(pi/4) ≈ 0.7071...
        assert!((weights[1] - 0.707_106_77).abs() < 1e-6);
    }

    #[test]
    fn cubic_bezier_shape() {
        let mut cps = [0.0f32; FIXTURE_CPS_MAX];
        let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
        let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
        let (degree, n_cp, n_knots, n_weights) =
            lookup(2, &mut cps, &mut knots, &mut weights).expect("fixture 2");
        assert_eq!((degree, n_cp, n_knots, n_weights), (3, 4, 8, 4));
        // Clamped degree-3: 4 zeros + 4 ones.
        assert_eq!(&knots[..4], &[0.0, 0.0, 0.0, 0.0]);
        assert_eq!(&knots[4..8], &[1.0, 1.0, 1.0, 1.0]);
    }

    /// Extract scalar (first component) from 3D fixture CPs.
    fn extract_scalar_cps(
        cps_3d: &[f32],
        n_cp: usize,
    ) -> [f32; crate::curve_pool::MAX_CONTROL_POINTS] {
        let mut scalar = [0.0f32; crate::curve_pool::MAX_CONTROL_POINTS];
        for i in 0..n_cp {
            scalar[i] = cps_3d[i * 3];
        }
        scalar
    }

    #[test]
    fn loads_into_curve_pool_via_validate_and_load() {
        // End-to-end: fixture 0 must validate as a NURBS through the regular
        // (production) `validate_and_load` path. Step 7-B: fixtures emit
        // 3D data; we extract the X component as scalar.
        use crate::curve_pool::CurvePool;
        let mut cps = [0.0f32; FIXTURE_CPS_MAX];
        let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
        let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
        let (degree, n_cp, n_knots, _n_weights) =
            lookup(0, &mut cps, &mut knots, &mut weights).expect("fixture 0");
        let scalar = extract_scalar_cps(&cps, n_cp);
        let pool = CurvePool::new();
        let r = pool.validate_and_load(0, degree, &knots[..n_knots], &scalar[..n_cp]);
        assert!(r.is_ok(), "fixture 0 must validate as a NURBS: {r:?}");
    }

    #[test]
    fn load_unchecked_round_trips() {
        // The FFI path: `load_unchecked` should accept fixture data and
        // produce a resolvable view.
        use crate::curve_pool::CurvePool;
        let pool = CurvePool::new();
        for fid in [0u16, 1u16, 2u16] {
            let mut cps = [0.0f32; FIXTURE_CPS_MAX];
            let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
            let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
            let (degree, n_cp, n_knots, _n_weights) =
                lookup(fid, &mut cps, &mut knots, &mut weights).expect("fixture");
            let scalar = extract_scalar_cps(&cps, n_cp);
            let handle = pool
                .load_unchecked(fid, degree, &knots[..n_knots], &scalar[..n_cp])
                .unwrap_or_else(|e| panic!("fixture {fid} must load_unchecked: {e:?}"));
            assert!(pool.lookup(handle).is_ok());
            // After confirm_retired we can re-load the same slot — exercises
            // the SEGMENT_END reclaim path indirectly.
            pool.confirm_retired(handle);
        }
    }

    #[test]
    fn loads_quarter_arc_and_cubic() {
        use crate::curve_pool::CurvePool;
        let pool = CurvePool::new();
        for fid in [1u16, 2u16] {
            let mut cps = [0.0f32; FIXTURE_CPS_MAX];
            let mut knots = [0.0f32; FIXTURE_KNOTS_MAX];
            let mut weights = [0.0f32; FIXTURE_WEIGHTS_MAX];
            let (degree, n_cp, n_knots, _n_weights) =
                lookup(fid, &mut cps, &mut knots, &mut weights).expect("fixture");
            let scalar = extract_scalar_cps(&cps, n_cp);
            let r = pool.validate_and_load(fid, degree, &knots[..n_knots], &scalar[..n_cp]);
            assert!(r.is_ok(), "fixture {fid} must validate: {r:?}");
        }
    }
}
