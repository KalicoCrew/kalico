//! Failing-test harness for the bench bug: "full fork firmware, klippy
//! connects, jog issued, no motor motion."
//!
//! ## What this test covers (vs. the existing jog_repro.rs)
//!
//! `runtime/tests/jog_repro.rs` drives `isr_sample_tick` with a
//! **manually-constructed** `WirePiece` (control points picked by hand) and
//! a **manually-constructed** `Segment`. That test passes, proving the
//! runtime ISR layer in isolation is correct when the clock starts at 0.
//!
//! This test exercises the **bridge encoding path + realistic MCU clock base**:
//!
//!  1. `classify_and_build` (G1 X+10 F600 → CubicSegment + NURBS)
//!  2. A `ShapedSegment` is synthesised from the bridge's collinear-cubic
//!     NURBS (matching what `build_push_params` expects as input).
//!  3. `build_push_params` → `McuPushPlan` with `CurveLoadParams`.
//!  4. `CurveLoadParams` (Bernstein control points) → `PieceEntry` array with
//!     absolute MCU start times derived from `t_start_clock`.
//!  5. `Engine::push_pieces` loads the entries into the per-axis ring.
//!  6. `isr_sample_tick` loop (advancing mock clock from `clock_base`).
//!  7. Assert `position_count > 0` on the X-axis stepper.
//!
//! ## The bench bug (now fixed in 5be894004)
//!
//! The root cause was f32 catastrophic cancellation in `t_local`:
//!
//!   `t_local = (now_cycles as f32)/Hz - (piece_start_cycles as f32)/Hz`
//!
//! At ~9 s of MCU uptime, `now_cycles ≈ 5×10⁹`. The f32 mantissa has only
//! 24 bits (~7 decimal digits); a 13 000-cycle per-sample increment has a
//! relative ratio of 2.6×10⁻⁶ < f32 epsilon at this magnitude. Both
//! operands round to the same f32 value; their difference is always 0.
//!
//! The piece-ring architecture fixes this by computing:
//!   `elapsed_cycles = now_u64 - piece_start_u64`  (exact u64 subtract)
//! and only then converting the small result to f32. This test verifies the
//! fix survives by setting `PieceEntry::start_time = CLOCK_BASE_CYCLES` and
//! driving ticks from that base.
//!
//! ## Regression test purpose
//!
//! `bridge_encoded_x_jog_at_realistic_mcu_uptime_drives_steps` sets the
//! initial clock base to 9 s of H7 cycles (4.68×10⁹) to ensure the fix
//! is tested under conditions that would have triggered the cancellation
//! bug before the piece-ring architecture.
//!
//! ## Stepper parameters
//!
//! 0.0125 mm/microstep (80 steps/mm) — matches jog_repro.rs and sim tests.
//! For a 10 mm move: expected ≥ 700 microsteps (nominal 800).

#![allow(unsafe_code)]
#![allow(clippy::unwrap_used)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_precision_loss)]

use core::ptr::addr_of_mut;
use core::sync::atomic::Ordering;
use std::sync::Mutex;

use heapless::spsc::Queue;
use runtime::clock::WidenState;
use runtime::engine::Engine;
use runtime::piece_ring::PieceEntry;
use runtime::state::{IsrState, SharedState, TOTAL_RING_PIECES};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{MAX_AXES, StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::trace::{TRACE_RING_N, TraceSample};

use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::dispatch::{AXIS_X, AXIS_Y, McuAxisConfig, McuCaps, build_push_params};

// Match the H7 bench: 520 MHz clock, 40 kHz modulation rate.
const H7_CLOCK_HZ: u32 = 520_000_000;
const SAMPLE_RATE_HZ: u32 = 40_000;
// 0.0125 mm/microstep (80 steps/mm) — matches jog_repro.rs and sim tests.
const MICROSTEP_DISTANCE_MM: f32 = 0.0125;
// 10 mm X-only travel move, 10 mm/s. Duration = 1 s.
const JOG_DIST_MM: f64 = 10.0;
const JOG_FEEDRATE_MM_S: f64 = 10.0;

// ── Realistic MCU uptime base ─────────────────────────────────────────────
//
// On the bench, the H7 has been running for ~9 s before klippy sends the
// first jog segment (boot + identify + clock-sync + configure_axes).
// At 520 MHz that's 4.68×10⁹ cycles — well past f32's ability to represent
// a 13 000-cycle per-sample increment (f32 epsilon is ~6×10⁻⁸ at 5×10⁹
// → relative resolution 300 cycles; 13 000 cycles needs <1 ULP at 5e9 or
// absolute error ≤ 13000, which requires the u64-subtract trick).
//
// Setting CLOCK_BASE_CYCLES to this value puts the test in the regime that
// triggered the f32-cancellation bench bug in the old Bezier-eval path.
// The piece-ring architecture avoids the cancellation by computing
// `elapsed_cycles = now_u64 - piece_start_u64` before converting to f32.
const MCU_UPTIME_S: u64 = 9;
const CLOCK_BASE_CYCLES: u64 = MCU_UPTIME_S * H7_CLOCK_HZ as u64;

// Ring depth per axis — large enough for all pieces of a 1 s jog.
const RING_DEPTH: usize = 64;

// ── Helper: convert CurveLoadParams → PieceEntry slice ──────────────────────
//
// `CurveLoadParams::bp_per_piece` holds Bernstein control points; these map
// directly to `PieceEntry::coeffs`.  `start_time` for piece `i` is
// `t_start_clock + sum(duration[0..i] * clock_hz)`.

fn curve_load_params_to_piece_entries(
    params: &kalico_host_rt::producer::CurveLoadParams,
    t_start_clock: u64,
    clock_hz: u32,
) -> Vec<PieceEntry> {
    assert_eq!(
        params.bp_per_piece.len(),
        params.duration_per_piece.len(),
        "bp_per_piece and duration_per_piece must have the same length"
    );
    let mut entries = Vec::with_capacity(params.bp_per_piece.len());
    let mut start = t_start_clock;
    for (bp, &dur) in params
        .bp_per_piece
        .iter()
        .zip(params.duration_per_piece.iter())
    {
        entries.push(PieceEntry {
            start_time: start,
            coeffs: *bp,
            duration: dur,
            _reserved: 0,
        });
        let dur_cycles = (dur * clock_hz as f32) as u64;
        start += dur_cycles;
    }
    entries
}

// ── Helper: build a configured engine ─────────────────────────────────────

fn configured_engine() -> Engine {
    let mut e = Engine::new(H7_CLOCK_HZ, SAMPLE_RATE_HZ);
    let binding = StepperBindingRust {
        stepper_oid: 0,
        tmc_cs_oid: TMC_CS_OID_NONE,
        _pad: [0; 2],
    };
    assert_eq!(
        e.configure_axis(
            0,
            StepMode::Pulse,
            MICROSTEP_DISTANCE_MM,
            RING_DEPTH,
            &[binding],
            TOTAL_RING_PIECES,
        ),
        0
    );
    e
}

// ── Helper: build a ShapedSegment from a linear X-only NURBS ──────────────
//
// `classify_and_build` produces a CubicSegment whose `xyz` NURBS is on the
// parametric domain [0, 1]. The trajectory shaper would reparameterize it
// into absolute time [0, T] before handing it to the bridge. We replicate
// that: build a ScalarNurbs for each axis on knot domain [0, T_duration_s]
// so `extract_bezier_pieces` produces a piece with `u_end - u_start ≈ T`.

fn make_shaped_segment_for_x_jog(
    start_x: f64,
    dist_mm: f64,
    feedrate_mm_s: f64,
) -> trajectory::ShapedSegment {
    let m = classify_and_build([start_x, 0.0, 0.0], dist_mm, 0.0, 0.0, 0.0, feedrate_mm_s)
        .expect("classify_and_build must succeed for a valid X jog");

    let t_duration_s = dist_mm / feedrate_mm_s;

    // Reparameterize into absolute time domain [0, T].
    let xyz = &m.segment.xyz;
    let degree: u8 = 3;
    let knots: Vec<f64> = vec![
        0.0,
        0.0,
        0.0,
        0.0,
        t_duration_s,
        t_duration_s,
        t_duration_s,
        t_duration_s,
    ];
    let cps_vec = xyz.control_points();
    assert_eq!(
        cps_vec.len(),
        4,
        "collinear cubic must have 4 control points"
    );
    let x_cps: Vec<f64> = cps_vec.iter().map(|cp| cp[0]).collect();
    let y_cps: Vec<f64> = cps_vec.iter().map(|cp| cp[1]).collect();
    let z_cps: Vec<f64> = cps_vec.iter().map(|cp| cp[2]).collect();

    let x_nurbs = nurbs::ScalarNurbs::<f64>::try_new(degree, knots.clone(), x_cps, None)
        .expect("X ScalarNurbs construction");
    let y_nurbs = nurbs::ScalarNurbs::<f64>::try_new(degree, knots.clone(), y_cps, None)
        .expect("Y ScalarNurbs construction");
    let z_nurbs = nurbs::ScalarNurbs::<f64>::try_new(degree, knots, z_cps, None)
        .expect("Z ScalarNurbs construction");

    trajectory::ShapedSegment {
        axes: [x_nurbs, y_nurbs, z_nurbs],
        e_mode: geometry::segment::EMode::Travel,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start: 0.0,
        t_end: t_duration_s,
    }
}

// ── Global test mutex ─────────────────────────────────────────────────────
//
// Two tests that both call push_pieces + isr_loop will race if they
// run concurrently. Serialize them via this mutex.
static TEST_MUTEX: Mutex<()> = Mutex::new(());

// ── Shared test body ────────────────────────────────────────────────────────
//
// Both tests call this with different `clock_base` values:
//   - 0 (trivial case, passes even with f32 cancellation)
//   - CLOCK_BASE_CYCLES (realistic MCU uptime, fails pre-piece-ring fix)

fn run_bridge_to_isr_chain(clock_base: u64, test_label: &str) {
    let _guard = TEST_MUTEX.lock().unwrap_or_else(|p| p.into_inner());
    // Step 1: build the shaped segment.
    let shaped = make_shaped_segment_for_x_jog(0.0, JOG_DIST_MM, JOG_FEEDRATE_MM_S);
    assert!(shaped.t_end > 0.0);

    // Step 2: bridge → McuPushPlan.
    let mcu_configs = vec![McuAxisConfig {
        mcu_id: 0,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: runtime::segment::KinematicTag::CartesianXyzAndE as u8,
        caps: McuCaps::default(),
    }];
    let t_start_clock: u64 = clock_base;
    let t_end_clock: u64 = clock_base + (shaped.t_end * f64::from(H7_CLOCK_HZ)) as u64;

    let plans = build_push_params(&shaped, &mcu_configs, t_start_clock, t_end_clock);

    assert!(
        !plans.is_empty(),
        "[{test_label}] build_push_params produced no plans — bridge silently skips segment"
    );
    let plan = plans.into_iter().next().unwrap();
    assert!(
        !plan.curves_to_load.is_empty(),
        "[{test_label}] McuPushPlan has no curves — X axis NURBS skipped"
    );

    // Step 3: CurveLoadParams → PieceEntry → Engine::push_pieces.
    let mut engine = configured_engine();
    let mut storage = vec![
        PieceEntry {
            start_time: 0,
            coeffs: [0.0; 4],
            duration: 0.0,
            _reserved: 0,
        };
        TOTAL_RING_PIECES
    ];

    let mut x_loaded = false;

    for (axis_idx, curve_params) in &plan.curves_to_load {
        let entries = curve_load_params_to_piece_entries(curve_params, t_start_clock, H7_CLOCK_HZ);

        // Piece duration must be positive — zero means t_local never
        // advances, p_end stays constant, signed_steps stays 0.
        for (k, entry) in entries.iter().enumerate() {
            assert!(
                entry.duration > 0.0,
                "[{test_label}] PieceEntry[{k}] for axis {axis_idx}: duration={} ≤ 0. \
                 CurveLoadParams produced a zero-duration piece. \
                 t_local will never advance past 0 → no steps.",
                entry.duration
            );
        }

        // X span must be meaningful.
        if *axis_idx == AXIS_X {
            let bp0 = entries[0].coeffs[0];
            let bp3_last = entries.last().unwrap().coeffs[3];
            let span = (bp3_last - bp0).abs();
            assert!(
                span > 0.1,
                "[{test_label}] X axis bezier span={span:.6} mm (bp0={bp0:.6}, bp3={bp3_last:.6}). \
                 Expected ~{JOG_DIST_MM} mm. A near-zero span produces ~0 steps."
            );
            eprintln!(
                "[{test_label}] X curve: span={span:.6} mm, n_pieces={}, dur={:?}",
                entries.len(),
                &curve_params.duration_per_piece,
            );
        }

        let axis_u8 = *axis_idx as u8;
        let rc = engine.push_pieces(axis_u8, &entries, &mut storage);
        assert_eq!(
            rc, 0,
            "[{test_label}] engine.push_pieces failed for axis {axis_idx} (rc={rc})"
        );

        if *axis_idx == AXIS_X {
            x_loaded = true;
        }
    }

    assert!(
        x_loaded,
        "[{test_label}] X axis pieces were not loaded — curve missing from plan."
    );

    // Step 4: ISR setup + tick loop.
    let mut step_queues: [StepQueue; MAX_AXES] = std::array::from_fn(|_| StepQueue::new());
    let queue_ptrs: [*mut StepQueue; MAX_AXES] =
        std::array::from_fn(|i| addr_of_mut!(step_queues[i]));
    engine.test_install_step_queues(queue_ptrs);

    let trace_queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (trace_producer, _trace_consumer) = trace_queue.split();

    let mut widen_state = WidenState::default();
    // Seed the WidenState with the clock_base so the first `widen()` inside
    // `isr_sample_tick` correctly extends the u32 CYCCNT into the u64 domain
    // starting at `clock_base`. Without seeding, WidenState would start at 0
    // and would need to roll over `clock_base / 2^32` times before the
    // piece's start_time became reachable — far more samples than the budget.
    widen_state.seed(clock_base);

    // `IsrState` still carries `queue_consumer` and `pending_segment` for
    // the C-segment-queue path (used by the MCU-side legacy bridge), but
    // `isr_sample_tick` no longer reads from them in the piece-ring model —
    // the engine drives itself from its per-axis rings.
    let queue_consumer = runtime::c_segment_queue::Consumer::<runtime::segment::Segment>::new();
    let mut isr = IsrState {
        queue_consumer,
        trace_producer,
        engine,
        widen_state,
        pending_segment: None,
    };
    let shared = SharedState::new();

    // Drive samples. Each tick advances raw_cyccnt by cycles_per_sample.
    let cycles_per_sample = H7_CLOCK_HZ / SAMPLE_RATE_HZ;
    let total_samples = SAMPLE_RATE_HZ as u64 + 200; // 1 s + 5 ms at 40 kHz
    let mut raw_cyccnt: u32 = clock_base as u32;

    for _ in 0..total_samples {
        raw_cyccnt = raw_cyccnt.wrapping_add(cycles_per_sample);
        runtime::tick::isr_sample_tick(&mut isr, &shared, &mut storage, raw_cyccnt);
        unsafe { while runtime::step_queue::pop(queue_ptrs[0]).is_some() {} }
    }

    let isr_step_pushes = shared.isr_step_push_count.load(Ordering::Acquire);
    let position_count = isr.engine.stepping_axes[0].as_ref().unwrap().steppers[0]
        .position_count
        .load(Ordering::Acquire);

    eprintln!(
        "[{test_label}] clock_base={clock_base} \
         step_pushes={isr_step_pushes} position_count={position_count}",
    );

    // ASSERTION A: ISR must have pushed some step entries.
    //
    // If clock_base is large (9 s uptime) and the engine incorrectly computes
    // t_local as  f32(now)/Hz - f32(piece_start)/Hz  (the old cancellation
    // bug), t_local stays 0 every sample → p_end constant → no steps.
    // The piece-ring engine uses  elapsed = now_u64 - piece_start_u64
    // (exact u64 subtract), so this assertion must pass at all clock bases.
    assert!(
        isr_step_pushes > 0,
        "[{test_label}] ISR pushed 0 steps after {total_samples} ticks. \
         Likely cause: t_local frozen at 0 due to f32 catastrophic cancellation \
         (clock_base={clock_base} ≈ {:.1} s of uptime). \
         This is the bench bug: steppers energized but no motion.",
        clock_base as f64 / f64::from(H7_CLOCK_HZ)
    );

    // ASSERTION B (primary): stepper must reflect ~800 microsteps.
    assert!(
        position_count.abs() >= 700,
        "[{test_label}] position_count={position_count} after 10 mm jog \
         (expected |count| >= 700). \
         isr_step_pushes={isr_step_pushes}, clock_base={clock_base}.",
    );
}

// ──────────────────────────────────────────────────────────────────────────
// Test A: trivial clock base (0)
// ──────────────────────────────────────────────────────────────────────────
//
// Baseline: passes even with the f32-cancellation bug because clock_base=0
// makes elapsed = now_cycles - 0 = small positive number with no cancellation.
// If this test fails, the bug is in the bridge encoding or runtime ISR
// plumbing, NOT in the cancellation fix.

#[test]
fn bridge_encoded_x_jog_clock_base_zero_drives_steps() {
    run_bridge_to_isr_chain(0, "clock_base=0");
}

// ──────────────────────────────────────────────────────────────────────────
// Test B: realistic MCU uptime base (~9 s)
// ──────────────────────────────────────────────────────────────────────────
//
// Regression test for the f32-cancellation bench bug.
//
// Before the piece-ring fix: t_local computed as
//   f32(now)/Hz - f32(piece_start)/Hz.
//   At clock_base = 4.68×10⁹ cycles, both operands round to the same f32
//   value; their difference is always 0; p_end never changes; signed_steps
//   is always 0; ASSERTION A fails.
//
// After the fix: elapsed_cycles = now_u64 - piece_start_u64 (exact u64
//   subtract); the small result converts to f32 accurately; p_end advances
//   per sample; steps fire; test passes.

#[test]
fn bridge_encoded_x_jog_at_realistic_mcu_uptime_drives_steps() {
    run_bridge_to_isr_chain(CLOCK_BASE_CYCLES, "clock_base=9s_uptime");
}
