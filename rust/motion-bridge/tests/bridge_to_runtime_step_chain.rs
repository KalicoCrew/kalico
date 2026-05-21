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
//!  4. `CurveLoadParams` → `WirePiece` array (the bit-pattern the bridge
//!     encodes onto the wire and the MCU decodes via `populate_from_wire`).
//!  5. `CurvePool::try_alloc_and_load` with those wire pieces.
//!  6. `Segment` constructed from pool handles + t_start/t_end at a
//!     **realistic MCU clock base** (~9 s of uptime = 4.68×10⁹ cycles).
//!  7. `c_segment_queue::Producer::enqueue`.
//!  8. `isr_sample_tick` loop (advancing mock clock from the same base).
//!  9. Assert `position_count > 0` on the X-axis stepper.
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
//! Bezier eval at frozen `t_local` produces constant `p_end`; `signed_steps`
//! stays 0 every sample; no steps ever fire; motors are silent.
//!
//! Fix (`5be894004`): compute `t_local_cycles = now_u64 - piece_start_u64`
//! (exact u64 subtract, no precision loss), THEN convert the small result
//! to f32.
//!
//! ## Regression test purpose
//!
//! `bridge_encoded_x_jog_at_realistic_mcu_uptime_drives_steps` sets the
//! initial clock base to 9 s of H7 cycles (4.68×10⁹) to ensure the fix
//! is tested under conditions that would have triggered the cancellation
//! bug before `5be894004`. If the fix ever regresses, this test fails at
//! the `position_count >= 700` assertion.
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
use runtime::c_segment_queue;
use runtime::clock::WidenState;
use runtime::config::EMode;
use runtime::cubic_curve::WirePiece;
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::{IsrState, SharedState};
use runtime::step_queue::StepQueue;
use runtime::stepping_state::{StepMode, StepperBindingRust, TMC_CS_OID_NONE};
use runtime::trace::{TraceSample, TRACE_RING_N};

use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::dispatch::{build_push_params, McuAxisConfig, McuCaps, AXIS_X, AXIS_Y};

type EngineImpl = Engine<NoopPa, NoopIs>;

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
// absolute error ≤ 13000, which requires f64 or the u64-subtract trick).
//
// Setting CLOCK_BASE_CYCLES to this value puts the test in the regime that
// triggered the f32-cancellation bench bug pre-5be894004.
const MCU_UPTIME_S: u64 = 9;
const CLOCK_BASE_CYCLES: u64 = MCU_UPTIME_S * H7_CLOCK_HZ as u64;

// ── Helper: convert CurveLoadParams → WirePiece slice ──────────────────────
//
// Replicates the encoding that `kalico_host_rt::producer::load_curve` sends
// over the wire, which the MCU's `runtime_handle_load_curve_cubic` decodes
// via `populate_from_wire`. Using this here lets us inject the bridge-produced
// curve into a host-side `CurvePool` without a live serial connection.

fn curve_load_params_to_wire_pieces(
    params: &kalico_host_rt::producer::CurveLoadParams,
) -> Vec<WirePiece> {
    assert_eq!(
        params.bp_per_piece.len(),
        params.duration_per_piece.len(),
        "bp_per_piece and duration_per_piece must have the same length"
    );
    params
        .bp_per_piece
        .iter()
        .zip(params.duration_per_piece.iter())
        .map(|(bp, &dur)| WirePiece {
            bp0_bits: bp[0].to_bits(),
            bp1_bits: bp[1].to_bits(),
            bp2_bits: bp[2].to_bits(),
            bp3_bits: bp[3].to_bits(),
            duration_bits: dur.to_bits(),
        })
        .collect()
}

// ── Helper: build a configured engine ─────────────────────────────────────

fn configured_engine() -> EngineImpl {
    let mut e = EngineImpl::new(H7_CLOCK_HZ, SAMPLE_RATE_HZ);
    let binding = StepperBindingRust { tmc_cs_oid: TMC_CS_OID_NONE, _pad: [0; 3] };
    assert_eq!(e.configure_axis(0, StepMode::Pulse, MICROSTEP_DISTANCE_MM, &[binding]), 0);
    assert_eq!(e.configure_kinematics(1.0), 0);
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
        0.0, 0.0, 0.0, 0.0,
        t_duration_s, t_duration_s, t_duration_s, t_duration_s,
    ];
    let cps_vec = xyz.control_points();
    assert_eq!(cps_vec.len(), 4, "collinear cubic must have 4 control points");
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
// `c_segment_queue` is a process-global singleton (OnceLock<Mutex<VecDeque>>).
// Two tests that both call reset() + enqueue() + isr_loop will race if they
// run concurrently. Serialize them via this mutex.
static TEST_MUTEX: Mutex<()> = Mutex::new(());

// ── Shared test body ────────────────────────────────────────────────────────
//
// Both tests call this with different `clock_base` values:
//   - 0 (trivial case, passes even with f32 cancellation)
//   - CLOCK_BASE_CYCLES (realistic MCU uptime, fails pre-5be894004)

fn run_bridge_to_isr_chain(clock_base: u64, test_label: &str) {
    let _guard = TEST_MUTEX.lock().unwrap_or_else(|p| p.into_inner());
    // Step 1: build the shaped segment.
    let shaped = make_shaped_segment_for_x_jog(0.0, JOG_DIST_MM, JOG_FEEDRATE_MM_S);
    assert!(shaped.t_end > 0.0);

    // Step 2: bridge → McuPushPlan.
    let mcu_configs = vec![McuAxisConfig {
        mcu_id: 0,
        axes: vec![AXIS_X, AXIS_Y],
        kinematics: KinematicTag::CartesianXyzAndE as u8,
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

    // Step 3: CurveLoadParams → WirePiece → CurvePool.
    let pool = CurvePool::new();
    let mut x_handle = CurveHandle::UNUSED_SENTINEL;
    let mut y_handle = CurveHandle::UNUSED_SENTINEL;

    for (axis_idx, curve_params) in &plan.curves_to_load {
        let wire_pieces = curve_load_params_to_wire_pieces(curve_params);

        // Piece duration must be positive — zero here means t_local never
        // advances, p_end stays constant, signed_steps stays 0.
        for (k, wp) in wire_pieces.iter().enumerate() {
            let dur = f32::from_bits(wp.duration_bits);
            assert!(
                dur > 0.0,
                "[{test_label}] WirePiece[{k}] for axis {axis_idx}: duration={dur} ≤ 0. \
                 CurveLoadParams::from_scalar_nurbs_normalized produced a zero-duration piece. \
                 t_local will never advance past 0 → no steps."
            );
        }

        // X span must be meaningful.
        if *axis_idx == AXIS_X {
            let bp0 = f32::from_bits(wire_pieces[0].bp0_bits);
            let bp3_last = f32::from_bits(wire_pieces.last().unwrap().bp3_bits);
            let span = (bp3_last - bp0).abs();
            assert!(
                span > 0.1,
                "[{test_label}] X axis bezier span={span:.6} mm (bp0={bp0:.6}, bp3={bp3_last:.6}). \
                 Expected ~{JOG_DIST_MM} mm. A near-zero span produces ~0 steps."
            );
            eprintln!(
                "[{test_label}] X curve: span={span:.6} mm, n_pieces={}, dur={:?}",
                wire_pieces.len(),
                &curve_params.duration_per_piece,
            );
        }

        let slot_idx = if *axis_idx == AXIS_X { 0 } else { 1 };
        let handle = pool
            .try_alloc_and_load(slot_idx, &wire_pieces)
            .unwrap_or_else(|| {
                panic!(
                    "[{test_label}] CurvePool::try_alloc_and_load failed for slot {slot_idx} \
                     (axis {axis_idx}) — populate_from_wire rejected the WirePiece."
                )
            });

        if *axis_idx == AXIS_X {
            x_handle = handle;
        } else if *axis_idx == AXIS_Y {
            y_handle = handle;
        }
    }

    assert!(
        !x_handle.is_unused_sentinel(),
        "[{test_label}] X handle is UNUSED_SENTINEL — X curve was not loaded into pool."
    );

    // Step 4: build Segment + enqueue.
    let mut seg = Segment {
        id: 1,
        x_handle,
        y_handle,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start: t_start_clock,
        t_end: t_end_clock,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
        flags: 0,
        _pad: [0; 1],
        extrusion_ratio: 0.0,
        consumers_remaining: 0,
    };
    seg.consumers_remaining = Segment::compute_consumers_remaining(
        seg.kinematics,
        seg.x_handle,
        seg.y_handle,
        seg.z_handle,
        seg.e_handle,
    );
    assert_ne!(
        seg.consumers_remaining, 0,
        "[{test_label}] consumers_remaining=0 — ISR would retire segment immediately with 0 steps."
    );

    c_segment_queue::reset();
    let mut queue_producer = c_segment_queue::Producer::<Segment>::new();
    let queue_consumer = c_segment_queue::Consumer::<Segment>::new();
    queue_producer.enqueue(seg).expect("segment enqueue");
    assert!(
        !queue_consumer.is_empty(),
        "[{test_label}] queue is empty immediately after enqueue."
    );

    // Step 5: ISR setup + tick loop.
    let mut engine = configured_engine();
    let mut step_queues = [
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
        StepQueue::new(),
    ];
    let queue_ptrs = [
        addr_of_mut!(step_queues[0]),
        addr_of_mut!(step_queues[1]),
        addr_of_mut!(step_queues[2]),
        addr_of_mut!(step_queues[3]),
    ];
    engine.test_install_step_queues(queue_ptrs);

    let trace_queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
        Box::leak(Box::new(Queue::new()));
    let (trace_producer, _trace_consumer) = trace_queue.split();

    let mut widen_state = WidenState::default();
    // Seed the WidenState with the clock_base so the first `widen()` inside
    // `isr_sample_tick` correctly extends the u32 CYCCNT into the u64 domain
    // starting at `clock_base`. Without seeding, WidenState would start at 0
    // and would need to roll over `clock_base / 2^32` times to reach the
    // segment's t_start — far more samples than the test budget.
    //
    // `WidenState::seed(baseline)` sets:
    //   self.high = baseline & !0xFFFF_FFFFu64  (upper bits)
    //   self.last_low = baseline as u32          (lower 32 bits)
    // The next `widen(raw)` then measures advance from this reference point.
    widen_state.seed(clock_base);

    let mut isr = IsrState {
        queue_consumer,
        trace_producer,
        engine,
        widen_state,
        pending_segment: None,
    };
    let shared = SharedState::new();

    // Drive samples. Each tick advances raw_cyccnt by cycles_per_sample.
    // Starting from (clock_base as u32) + cycles_per_sample, the widened
    // result is clock_base + cycles_per_sample (no rollover on the first tick
    // because seed set last_low = clock_base as u32 and raw is higher).
    let cycles_per_sample = H7_CLOCK_HZ / SAMPLE_RATE_HZ;
    let total_samples = SAMPLE_RATE_HZ as u64 + 200; // 1 s + 5 ms at 40 kHz
    let mut raw_cyccnt: u32 = clock_base as u32;

    for _ in 0..total_samples {
        raw_cyccnt = raw_cyccnt.wrapping_add(cycles_per_sample);
        runtime::tick::isr_sample_tick(&mut isr, &shared, &pool, raw_cyccnt);
        unsafe {
            while runtime::step_queue::pop(queue_ptrs[0]).is_some() {}
        }
    }

    let isr_deq = shared.isr_deq_some_count.load(Ordering::Acquire);
    let isr_armed = shared.isr_armed_count.load(Ordering::Acquire);
    let isr_step_pushes = shared.isr_step_push_count.load(Ordering::Acquire);
    let position_count = isr.engine.stepping_axes[0].steppers[0]
        .position_count
        .load(Ordering::Acquire);
    let t_local_bits = shared.isr_last_t_local_bits.load(Ordering::Relaxed);
    let t_local_last = f32::from_bits(t_local_bits);

    eprintln!(
        "[{test_label}] clock_base={clock_base} isr_deq={isr_deq} isr_armed={isr_armed} \
         step_pushes={isr_step_pushes} position_count={position_count} \
         t_local_last={t_local_last:.6}s",
    );

    // ASSERTION A: ISR must dequeue the segment.
    assert!(
        isr_deq >= 1,
        "[{test_label}] isr_sample_tick never dequeued the segment \
         ({total_samples} ticks, isr_deq_some=0). Segment stuck in queue."
    );

    // ASSERTION B: ISR must arm the segment (t_start must be <= widened_now).
    //
    // On the MCU the clock starts somewhere in the u32 wrap range and the
    // WidenState extends it. The test's initial raw_cyccnt is `clock_base as
    // u32`. The segment's t_start is also `clock_base`. On the first tick
    // after the base, raw_cyccnt > t_start (in u64 space) so t_start <= now
    // and the arm fires.
    //
    // If this assertion fails, the segment is perpetually parked — t_start is
    // in the future relative to the widened clock — which would mean the
    // WidenState epoch seeding is wrong.
    assert!(
        isr_armed >= 1,
        "[{test_label}] ISR dequeued (isr_deq={isr_deq}) but never armed \
         (isr_armed=0). seg.t_start={t_start_clock} not <= widened_now. \
         WidenState not seeded correctly from clock_base={clock_base}."
    );

    // ASSERTION C: ISR must have pushed some step entries.
    //
    // Pre-5be894004: at large clock_base (9 s uptime) the f32 cancellation
    // made t_local always 0 → p_end always 0 → signed_steps always 0 →
    // this assertion fails with isr_step_pushes=0.
    // Post-5be894004: u64 subtract keeps t_local accurate → p_end advances
    // correctly → steps fire.
    assert!(
        isr_step_pushes > 0,
        "[{test_label}] ISR armed segment (isr_armed={isr_armed}) but pushed 0 steps. \
         dispatch_pulse early-returned at signed_steps==0 every sample.\n\
         Likely cause: t_local frozen at 0 due to f32 catastrophic cancellation \
         (clock_base={clock_base} ≈ {:.1} s of uptime). \
         Last t_local sampled: {t_local_last:.9} s — if this is 0.0 or repeating, \
         the u64-subtract fix in 5be894004 has regressed.\n\
         This is the bench bug: steppers energized but no motion.",
        clock_base as f64 / f64::from(H7_CLOCK_HZ)
    );

    // ASSERTION D (primary): stepper must reflect ~800 microsteps.
    assert!(
        position_count.abs() >= 700,
        "[{test_label}] position_count={position_count} after 10 mm jog \
         (expected |count| >= 700). \
         isr_step_pushes={isr_step_pushes}, clock_base={clock_base}.\n\
         If isr_step_pushes>0 but position_count~0, the step-queue consumer \
         is not calling commit_position_count, or the wrong stepper index \
         is being read."
    );
}

// ──────────────────────────────────────────────────────────────────────────
// Test A: trivial clock base (0)
// ──────────────────────────────────────────────────────────────────────────
//
// Baseline: passes even with the f32-cancellation bug because clock_base=0
// makes t_local = now_cycles/Hz - 0 = small positive number with no
// cancellation. If this test fails, the bug is in the bridge encoding or
// runtime ISR plumbing, NOT in the t_local precision fix.

#[test]
fn bridge_encoded_x_jog_clock_base_zero_drives_steps() {
    run_bridge_to_isr_chain(0, "clock_base=0");
}

// ──────────────────────────────────────────────────────────────────────────
// Test B: realistic MCU uptime base (~9 s)
// ──────────────────────────────────────────────────────────────────────────
//
// Regression test for the f32-cancellation bench bug (commit 5be894004).
//
// Before 5be894004: t_local computed as f32(now)/Hz - f32(piece_start)/Hz.
//   At clock_base = 4.68×10⁹ cycles, both operands round to the same f32
//   value; their difference is always 0; p_end never changes; signed_steps
//   is always 0; isr_step_push_count stays 0; ASSERTION C fails.
//
// After 5be894004: t_local_cycles = now_u64 - piece_start_u64 (exact u64
//   subtract); the small result converts to f32 accurately; p_end advances
//   per sample; steps fire; test passes.

#[test]
fn bridge_encoded_x_jog_at_realistic_mcu_uptime_drives_steps() {
    run_bridge_to_isr_chain(CLOCK_BASE_CYCLES, "clock_base=9s_uptime");
}
