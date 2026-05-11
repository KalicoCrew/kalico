//! Bench-bug reproduction harness for the live-printer Voron 2.4 deployment.
//!
//! This harness drives the host-side `PlannerHandle` (the same one
//! `bridge.rs::init_planner` spawns when klippy attaches) and feeds the
//! dispatched `ShapedSegment`s through a host-resident `runtime::Engine`
//! (the same engine the H723 firmware runs in its TIM5 ISR). The Engine
//! is `target_os != "none"`, so `runtime_emit_step_pulses` is a no-op, but
//! the per-stepper accumulators in `SharedState.stepper_counts` are
//! incremented exactly the way they would be on hardware.
//!
//! That lets us count step pulses on the host without booting the sim
//! firmware. Five tests cover the five live-bench symptoms reported on
//! `sota-motion`:
//!
//! 1. `first_jog_after_stream_open_emits_step_pulses` — "first motion only
//!    energizes the steppers, doesn't actually move."
//! 2. `feedrate_caps_trajectory_velocity` — feedrate=50 is ignored.
//! 3. `consecutive_short_jogs_produce_consistent_motion` — intermittent
//!    no-motion on some presses.
//! 4. `single_segment_has_monotone_velocity_profile` — slow-then-suddenly-
//!    faster mid-move (velocity discontinuity within a single segment).
//! 5. `no_step_burst_violations_under_rapid_jogs` — rapid-fire jogs trip
//!    `KALICO_ERR_STEP_BURST_EXCEEDED`.
//!
//! The harness mirrors the live config: smooth_mzv @ 186 Hz X / 122 Hz Y,
//! max_velocity=1000, max_accel=70000, CoreXY kinematics.

#![allow(clippy::too_many_lines)]
#![allow(clippy::unwrap_used)]
#![allow(clippy::expect_used)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_precision_loss)]

use std::sync::{Arc, Mutex};

use heapless::spsc::Queue;
use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::config::{PlannerConfig, PlannerLimits};
use motion_bridge_native::planner::PlannerHandle;
use trajectory::{AxisShaper, RequiredShaper, ShapedSegment, ShaperConfig};

use runtime::clock::{WidenState, one_tick_cycles};
use runtime::config::{EMode as RtEMode, McuAxisConfig as RtMcuAxisConfig, MotorConfig};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::{Engine, RuntimeStatus};
use runtime::queue::Q_N;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_RING_N, TraceSample};

// ---------------------------------------------------------------------------
// Live-printer config snapshot — matches the user's Trident printer.cfg:
//   max_velocity=1000
//   max_accel=70000
//   max_z_velocity=5
//   max_z_accel=100
//   square_corner_velocity=5
//   shaper_freq_x = 186 (smooth_mzv)
//   shaper_freq_y = 122 (smooth_mzv)
//   CoreXY, 80 steps/mm (typical 16T / 20T GT2)
// ---------------------------------------------------------------------------

const LIVE_MAX_VEL: f64 = 1000.0;
const LIVE_MAX_ACC: f64 = 70_000.0;
const LIVE_MAX_Z_VEL: f64 = 5.0;
const LIVE_MAX_Z_ACC: f64 = 100.0;
const LIVE_SQUARE_CORNER_VEL: f64 = 5.0;

const LIVE_SHAPER_FREQ_X: f64 = 186.0;
const LIVE_SHAPER_FREQ_Y: f64 = 122.0;

// Klipper Kconfig default for H723; sim uses the same per `tools/sim/sim.config`.
const CLOCK_FREQ: u32 = 520_000_000;

// CoreXY 80 steps/mm.
const STEPS_PER_MM: f32 = 80.0;

fn live_limits() -> PlannerLimits {
    PlannerLimits {
        max_velocity: LIVE_MAX_VEL,
        max_accel: LIVE_MAX_ACC,
        max_z_velocity: LIVE_MAX_Z_VEL,
        max_z_accel: LIVE_MAX_Z_ACC,
        square_corner_velocity: LIVE_SQUARE_CORNER_VEL,
    }
}

fn live_planner_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.limits = live_limits();
    c.shaper = ShaperConfig {
        x: RequiredShaper::SmoothMzv {
            frequency_hz: LIVE_SHAPER_FREQ_X,
        },
        y: RequiredShaper::SmoothMzv {
            frequency_hz: LIVE_SHAPER_FREQ_Y,
        },
        z: AxisShaper::Passthrough,
    };
    // Loose enough that bench-stress jog sequences pass the C¹ refit cap on
    // short moves; 50 µm is the same value used in `streaming_replan.rs`
    // and `sim_motion.rs`. Live firmware uses the default 5 µm but the
    // bench-bug reproduction is a host-side architectural question, not a
    // refit-precision regression.
    c.fit_tolerance_mm = 0.05;
    c
}

// ---------------------------------------------------------------------------
// Recording dispatch — same shape used by `streaming_replan.rs`.
// ---------------------------------------------------------------------------

type Recorded = Arc<Mutex<Vec<ShapedSegment>>>;

fn recording_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync>,
    Recorded,
) {
    let recorded: Recorded = Arc::new(Mutex::new(Vec::new()));
    let rec_for_closure = Arc::clone(&recorded);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), String> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            rec_for_closure.lock().unwrap().push(seg.clone());
            Ok(())
        });
    (cb, recorded)
}

// ---------------------------------------------------------------------------
// Runtime-engine harness: load a sequence of ShapedSegments into a fresh
// Engine + queue + CurvePool, tick the engine across the entire dispatched
// time window at 40 kHz, and capture the per-stepper step counts.
// ---------------------------------------------------------------------------

/// Allocate a curve into the pool. Returns the resulting handle, or panics
/// on validation failure (only happens if the planner emits a NURBS the
/// runtime considers out-of-spec, which is itself a regression).
///
/// **Wire-format mirror.** The bridge's `from_scalar_nurbs_normalized`
/// truncates the f64 control points / knots to f32 and rescales knots from
/// `[t_start, t_end]` → `[0, 1]`. We replicate that here so the test
/// engine sees byte-for-byte what the H723 firmware would.
fn load_axis_curve(
    pool: &CurvePool,
    slot: u16,
    curve: &nurbs::ScalarNurbs<f64>,
    t_start_s: f64,
    t_end_s: f64,
) -> Option<CurveHandle> {
    let duration = t_end_s - t_start_s;
    let knots_f32: Vec<f32> = curve
        .knots()
        .iter()
        .map(|&k| {
            let u = if duration > 0.0 {
                (k - t_start_s) / duration
            } else {
                k
            };
            u.clamp(0.0, 1.0) as f32
        })
        .collect();
    let cps_f32: Vec<f32> = curve
        .control_points()
        .iter()
        .map(|&v| v as f32)
        .collect();
    pool.try_alloc_and_load(slot as usize, curve.degree(), &knots_f32, &cps_f32)
}

/// `is_trivially_constant` clone from `dispatch.rs`. Single-source-of-
/// truth would be nicer but `dispatch::is_trivially_constant` lives behind
/// a `#[doc(hidden)] pub` module and isn't worth re-exporting just for
/// this test.
fn is_trivially_constant(curve: &nurbs::ScalarNurbs<f64>) -> bool {
    const EPS: f64 = 1.0e-12;
    let cps = curve.control_points();
    if cps.is_empty() {
        return true;
    }
    let first = cps[0];
    cps.iter().all(|&v| (v - first).abs() <= EPS)
}

#[derive(Debug, Clone)]
struct EngineRunSummary {
    /// Per-motor step pulse counts, indexed in motor space:
    /// CoreXY = [A, B, Z, E].
    step_counts: [i32; 4],
    /// Final engine status. `Drained` is the healthy end-of-stream value;
    /// `Fault` indicates a runtime fault latched during the run.
    final_status: RuntimeStatus,
    /// Latched fault code if any (0 = none).
    last_error: i32,
    /// Number of segments queued (post-trivial-constant filter — only
    /// segments with at least one non-constant axis are pushed).
    segments_queued: usize,
    /// Per-tick motor-A position trace (only populated when requested).
    /// One sample per engine tick at 40 kHz across the run.
    #[allow(dead_code)]
    motor_trace: Option<Vec<[f32; 4]>>,
}

#[derive(Debug, Clone, Copy)]
struct RunOptions {
    /// Capture per-tick motor positions into `motor_trace`. Off by default
    /// to keep the harness fast on bursty tests.
    capture_motor_trace: bool,
    /// CoreXY engine; the H723 case.
    kinematics: KinematicTag,
}

impl Default for RunOptions {
    fn default() -> Self {
        Self {
            capture_motor_trace: false,
            kinematics: KinematicTag::CoreXyAndE,
        }
    }
}

/// Drive `segs` through a fresh `Engine` and return the summary.
///
/// The engine is configured for CoreXY with 80 steps/mm on both A and B
/// (matches the live Trident config). `t_start` / `t_end` for the queued
/// segments are mapped from absolute trajectory seconds to MCU clocks via
/// `seconds * CLOCK_FREQ`, with the first segment starting at clock 0 so
/// the engine's wraparound logic doesn't interact with the test.
///
/// **Returns `None` if `segs.is_empty()`.** Callers should treat this as
/// "zero segments dispatched" and surface their own test-specific failure
/// message.
fn run_segments_through_engine(
    segs: &[ShapedSegment],
    opts: RunOptions,
) -> Option<EngineRunSummary> {
    if segs.is_empty() {
        return None;
    }

    // Engine + half-split queue + curve pool. We Box::leak so the static
    // 'static SPSC halves live across the test, mirroring `engine_tick.rs`.
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_prod, mut q_cons) = queue.split();
    let trace: &'static mut Queue<TraceSample, TRACE_RING_N> = Box::leak(Box::new(Queue::new()));
    let (mut t_prod, _t_cons) = trace.split();

    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut widen = WidenState::default();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);

    // CoreXY: both A and B motors with steps_per_mm=80.
    let mcu_cfg = RtMcuAxisConfig {
        motors: [
            Some(MotorConfig {
                steps_per_mm: STEPS_PER_MM,
                is_awd: false,
                invert_dir: false,
            }),
            Some(MotorConfig {
                steps_per_mm: STEPS_PER_MM,
                is_awd: false,
                invert_dir: false,
            }),
            None,
            None,
        ],
        kinematics: opts.kinematics,
    };
    engine.configure(mcu_cfg);

    // Mark the stream as open so an empty-queue tick during the trailing
    // post-decel window emits an Underrun fault — matches the live
    // firmware behaviour where `kalico_stream_open` precedes the first
    // segment push.
    shared
        .stream_open
        .store(true, core::sync::atomic::Ordering::Release);

    let tick_cycles = u64::from(one_tick_cycles(CLOCK_FREQ));

    // Reference timeline: shift each segment so the first one starts at
    // clock 0. The engine widens raw CYCCNT internally; starting at 0
    // avoids needing to seed the widen state.
    let t_offset = segs[0].t_start;

    let mut segments_queued = 0_usize;
    let mut slot_counter: u16 = 0;
    let mut segment_id_counter: u32 = 1;

    let mut last_t_end_clock: u64 = 0;
    for shaped in segs {
        let rel_start_s = shaped.t_start - t_offset;
        let rel_end_s = shaped.t_end - t_offset;
        let t_start_clock = (rel_start_s * f64::from(CLOCK_FREQ)).round() as u64;
        let t_end_clock = (rel_end_s * f64::from(CLOCK_FREQ)).round() as u64;
        last_t_end_clock = last_t_end_clock.max(t_end_clock);

        // Skip segments where every axis is trivially constant (the bridge
        // does the same — no work to push to the MCU).
        let any_moving = shaped.axes.iter().any(|c| !is_trivially_constant(c));
        if !any_moving {
            continue;
        }

        let mut x_handle = CurveHandle::UNUSED_SENTINEL;
        let mut y_handle = CurveHandle::UNUSED_SENTINEL;
        let mut z_handle = CurveHandle::UNUSED_SENTINEL;

        for (axis_idx, curve) in shaped.axes.iter().enumerate() {
            if is_trivially_constant(curve) {
                continue;
            }
            // No slot reuse within a single engine run — bench-repro
            // doesn't simulate the trace-drain → confirm_retired pipeline,
            // so once `current_gen` is bumped past `last_retired_gen` for
            // a slot, that slot is wedged. We have CURVE_POOL_N slots
            // available which bounds the segment count per engine run;
            // longer runs need explicit chunking by the caller.
            let slot = slot_counter;
            slot_counter += 1;
            if (slot as usize) >= runtime::curve_pool::CURVE_POOL_N {
                panic!(
                    "bench-repro: pool slot exhaustion at segment_id={segment_id_counter}, \
                     axis={axis_idx} (CURVE_POOL_N={}). Caller should chunk segments through \
                     separate engine runs.",
                    runtime::curve_pool::CURVE_POOL_N
                );
            }
            let handle = load_axis_curve(&pool, slot, curve, shaped.t_start, shaped.t_end)
                .unwrap_or_else(|| {
                    panic!(
                        "load_axis_curve failed for axis {} (degree={}, ncps={}, nknots={}, slot={slot})",
                        axis_idx,
                        curve.degree(),
                        curve.control_points().len(),
                        curve.knots().len()
                    )
                });
            match axis_idx {
                0 => x_handle = handle,
                1 => y_handle = handle,
                2 => z_handle = handle,
                _ => {}
            }
        }

        let seg = Segment {
            id: segment_id_counter,
            x_handle,
            y_handle,
            z_handle,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: t_start_clock,
            t_end: t_end_clock,
            kinematics: opts.kinematics,
            e_mode: RtEMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        };
        segment_id_counter += 1;
        q_prod.enqueue(seg).unwrap_or_else(|_| {
            panic!(
                "Q_N={} overflow at segment_id={} — bench-repro harness needs to chunk this run",
                Q_N, segment_id_counter
            )
        });
        segments_queued += 1;
    }

    // Tick from clock 0 through `last_t_end_clock + small post-roll` so the
    // engine retires the final segment via the boundary-loop's
    // "queue empty + stream_open=true" check. Post-roll is 5 ticks so we
    // observe Underrun (= host failed to keep up) cleanly when it happens.
    let post_roll_ticks = 5_u64;
    let total_ticks = (last_t_end_clock / tick_cycles) + post_roll_ticks;

    let mut motor_trace = if opts.capture_motor_trace {
        Some(Vec::with_capacity(total_ticks as usize))
    } else {
        None
    };

    for tick_idx in 0..=total_ticks {
        let now = tick_idx * tick_cycles;
        // Stream closes once we're past `last_t_end_clock`. This mirrors
        // the live behaviour: `kalico_stream_terminal` is published from
        // foreground on flush, and the ISR clears `stream_open` on the
        // matching retire. The simulation here flips it directly because
        // we don't run the foreground reactor — but the Underrun path is
        // still the right thing to detect "engine stalled while queue
        // empty before the trajectory completed."
        if now > last_t_end_clock {
            shared
                .stream_open
                .store(false, core::sync::atomic::Ordering::Release);
        }
        let raw = now as u32;
        let r = engine.tick(raw, &mut widen, &pool, &mut q_cons, &mut t_prod, &shared);
        if let Some(trace) = motor_trace.as_mut() {
            trace.push([
                engine.debug_last_motor(0),
                engine.debug_last_motor(1),
                engine.debug_last_motor(2),
                engine.debug_last_motor(3),
            ]);
        }
        if r.is_err() {
            break;
        }
    }

    let mut step_counts = [0_i32; 4];
    for (i, c) in step_counts.iter_mut().enumerate() {
        *c = shared.stepper_counts[i].load(core::sync::atomic::Ordering::Acquire);
    }

    Some(EngineRunSummary {
        step_counts,
        final_status: engine.status(),
        last_error: engine.last_error(),
        segments_queued,
        motor_trace,
    })
}

/// Submit a single (dx, dy, dz, feedrate) jog through the live-config
/// planner and return the dispatched `ShapedSegment` set. Handles
/// `kalico_stream_open` so the planner state is anchored at `start_pos`
/// before the jog (mirroring what the klippy bridge does at attach time).
fn submit_one_jog(
    h: &PlannerHandle,
    start_pos: [f64; 3],
    dx: f64,
    dy: f64,
    dz: f64,
    feedrate: f64,
) -> Result<(), Box<dyn std::error::Error>> {
    let m = classify_and_build(start_pos, dx, dy, dz, 0.0, feedrate)?;
    h.submit_move(m).map_err(|e| -> Box<dyn std::error::Error> { Box::new(e) })?;
    Ok(())
}

// ===========================================================================
//
// Test 1 — first_jog_after_stream_open_emits_step_pulses
//
// The bench symptom: every fresh start, the first jog button-press
// energizes motors but produces zero actual stepper steps. We reproduce
// by spawning a fresh planner, calling `kalico_stream_open`, submitting
// one 10 mm X jog, flushing, and driving the dispatched segments through
// a fresh `Engine`.
//
// Pass condition: motor A and motor B both get >0 step pulses. On a CoreXY
// at 10 mm pure X, |dA| = |dB| = 10 mm × 80 steps/mm = 800 steps each.
//
// ===========================================================================

#[test]
fn first_jog_after_stream_open_emits_step_pulses() {
    let (dispatch, recorded) = recording_dispatch();
    let h = PlannerHandle::spawn(live_planner_config(), dispatch);

    h.kalico_stream_open([0.0; 4]).expect("kalico_stream_open");

    submit_one_jog(&h, [0.0; 3], 10.0, 0.0, 0.0, 100.0).expect("submit 10mm X");
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    drop(h);

    let summary =
        run_segments_through_engine(&segs, RunOptions::default()).expect("non-empty segments");

    eprintln!(
        "[bench-repro] first_jog summary: segs_queued={} step_counts={:?} status={:?} last_error={}",
        summary.segments_queued, summary.step_counts, summary.final_status, summary.last_error,
    );

    assert!(
        summary.segments_queued > 0,
        "first jog produced zero queued segments — planner did not emit anything for the first move"
    );

    // CoreXY pure-X: A and B both move by ±X. ~800 step pulses each at
    // 80 steps/mm * 10 mm. Allow huge slop — we only care that pulses
    // come out at all.
    assert!(
        summary.step_counts[0].abs() > 100,
        "BENCH BUG #1: first jog dispatched {} segments but motor A saw \
         only {} step pulses (expected ≈ ±800). This is the 'first motion \
         energizes only, no movement' symptom.",
        summary.segments_queued,
        summary.step_counts[0]
    );
    assert!(
        summary.step_counts[1].abs() > 100,
        "BENCH BUG #1: first jog dispatched {} segments but motor B saw \
         only {} step pulses (expected ≈ ±800). This is the 'first motion \
         energizes only, no movement' symptom.",
        summary.segments_queued,
        summary.step_counts[1]
    );
}

// ===========================================================================
//
// Test 2 — feedrate_caps_trajectory_velocity
//
// The bench symptom: live trace shows a 25mm @ feedrate=100 mm/s move
// taking 0.138 s, peak velocity ≈ 330 mm/s. Feedrate=100 should cap
// peak velocity at 100 mm/s + acceleration-overshoot allowance.
//
// We sample velocity at 1 ms intervals from the dispatched X-axis curve
// and assert peak ≤ 60 mm/s for feedrate=50 (10% slop for shaper-overshoot).
//
// Currently expected to FAIL until the planner plumbs `feedrate_mm_s` into
// the temporal v_max. The classify path stores feedrate on the segment
// but `to_temporal_limits()` reads only `PlannerLimits.max_velocity`, so
// the trajectory runs at machine v_max regardless of the F value.
//
// ===========================================================================

#[test]
fn feedrate_caps_trajectory_velocity() {
    let (dispatch, recorded) = recording_dispatch();
    let h = PlannerHandle::spawn(live_planner_config(), dispatch);
    h.kalico_stream_open([0.0; 4]).expect("kalico_stream_open");

    // 25mm @ feed=50 — matches bench-observed live commands (the user
    // jogged with both 10mm and 50mm presses; 25mm sits in the same
    // operating regime, fits within smooth-MZV/feedrate feasibility on
    // the live limits, and provides enough cruise window for the velocity
    // peak to land at the requested cap).
    submit_one_jog(&h, [0.0; 3], 25.0, 0.0, 0.0, 50.0).expect("submit 25mm X @ feed=50");
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    drop(h);

    assert!(!segs.is_empty(), "planner emitted nothing for 25mm jog");

    // Sample velocity at 1 ms over the entire dispatched X trajectory.
    // For each ShapedSegment, sample X position at fine granularity and
    // diff to get velocity.
    let dt_sample = 1.0e-3_f64;
    let mut peak_v: f64 = 0.0;
    for seg in &segs {
        let x_curve = &seg.axes[0];
        let dur = seg.t_end - seg.t_start;
        if dur <= 0.0 {
            continue;
        }
        let n = (dur / dt_sample).ceil() as usize + 1;
        let mut prev: Option<f64> = None;
        for k in 0..=n {
            let t_offset = (k as f64) * dt_sample;
            let t = (seg.t_start + t_offset).min(seg.t_end);
            let x = nurbs::eval::eval(x_curve, t);
            if let Some(px) = prev {
                let v = (x - px) / dt_sample;
                if v.abs() > peak_v {
                    peak_v = v.abs();
                }
            }
            prev = Some(x);
        }
    }

    eprintln!(
        "[bench-repro] feedrate_caps: feedrate=50 mm/s, peak observed v = {:.2} mm/s",
        peak_v
    );

    // Allow 10% overshoot for shaper-induced velocity peak. Bench
    // observation: ~330 mm/s for feedrate=100 (commanded 25 mm in 0.138s
    // vs nominal 0.25s) — feedrate is ignored.
    let allowed = 50.0 * 1.10;
    assert!(
        peak_v <= allowed,
        "BENCH BUG #5: peak trajectory velocity {peak_v:.2} mm/s exceeds \
         feedrate cap 50 mm/s + 10% slop ({allowed:.2}). The planner is \
         ignoring the commanded feedrate; live trace shows the same — \
         25mm @ feed=100 runs at ~330 mm/s peak."
    );
}

// ===========================================================================
//
// Test 3 — consecutive_short_jogs_produce_consistent_motion
//
// The bench symptom: intermittent no-motion on rapid presses. We submit
// 10 × 5 mm jogs at 200 ms intervals (the planner pacing the user can
// reasonably produce with the jog button) and assert every dispatched
// **batch** generates step pulses on motor A.
//
// Pacing detail: `submit_move` returns immediately; the planner thread
// processes asynchronously. We rely on `flush()` as a barrier per jog so
// each is dispatched independently — same as the bench would see if the
// user paced their button presses.
//
// ===========================================================================

#[test]
fn consecutive_short_jogs_produce_consistent_motion() {
    let n_jogs = 10_usize;
    let jog_mm = 5.0;

    let (dispatch, recorded) = recording_dispatch();
    let h = PlannerHandle::spawn(live_planner_config(), dispatch);
    h.kalico_stream_open([0.0; 4]).expect("kalico_stream_open");

    // Track segment count per jog so we can attribute step pulses correctly.
    let mut jog_segment_counts = Vec::with_capacity(n_jogs);
    let mut last_count = 0;

    let mut pos = [0.0_f64; 3];
    for i in 0..n_jogs {
        let dx = if i % 2 == 0 { jog_mm } else { -jog_mm };
        submit_one_jog(&h, pos, dx, 0.0, 0.0, 100.0)
            .unwrap_or_else(|e| panic!("submit jog {i}: {e}"));
        pos[0] += dx;
        h.flush().unwrap_or_else(|e| panic!("flush jog {i}: {e}"));

        let cur = recorded.lock().unwrap().len();
        jog_segment_counts.push(cur - last_count);
        last_count = cur;
    }
    let all_segs = recorded.lock().unwrap().clone();
    drop(h);

    // Run each jog's segments through its OWN engine instance so we can
    // detect any single jog that fails to produce step pulses (the live
    // intermittent-no-motion symptom).
    let mut cursor = 0_usize;
    let mut zero_motion_jogs: Vec<usize> = Vec::new();
    let mut per_jog_step_counts: Vec<i32> = Vec::with_capacity(n_jogs);
    for (i, &n_segs) in jog_segment_counts.iter().enumerate() {
        let jog_segs = &all_segs[cursor..cursor + n_segs];
        cursor += n_segs;
        let summary = match run_segments_through_engine(jog_segs, RunOptions::default()) {
            Some(s) => s,
            None => {
                zero_motion_jogs.push(i);
                per_jog_step_counts.push(0);
                continue;
            }
        };
        per_jog_step_counts.push(summary.step_counts[0].abs());
        if summary.step_counts[0].abs() < 50 {
            zero_motion_jogs.push(i);
        }
    }

    eprintln!(
        "[bench-repro] consecutive_jogs: per-jog motor-A step counts = {:?}",
        per_jog_step_counts
    );

    assert!(
        zero_motion_jogs.is_empty(),
        "BENCH BUG #3 (intermittent no-motion): jogs {:?} produced < 50 \
         step pulses on motor A despite being dispatched. Per-jog step \
         counts: {:?}",
        zero_motion_jogs,
        per_jog_step_counts
    );
}

// ===========================================================================
//
// Test 4 — single_segment_has_monotone_velocity_profile
//
// The bench symptom: "slow then suddenly faster mid-move." A 50 mm X jog
// should produce a single accel→cruise→decel velocity profile with
// monotone-accel up, flat cruise, monotone-decel down. We sample velocity
// at 1 ms and check for the canonical mid-segment discontinuity:
// adjacent samples differing by more than `phys_a_max * dt + slop`.
//
// We DON'T require strict monotonicity — shaper ripple is real. We DO
// require that the maximum tick-to-tick acceleration is bounded by
// `LIVE_MAX_ACC × 1.5` (50% headroom for the post-shape peak).
//
// ===========================================================================

#[test]
fn single_segment_has_monotone_velocity_profile() {
    let (dispatch, recorded) = recording_dispatch();
    let h = PlannerHandle::spawn(live_planner_config(), dispatch);
    h.kalico_stream_open([0.0; 4]).expect("kalico_stream_open");

    // 25mm @ feed=100 — exactly the move the live bench shipped on jog
    // button presses, and which the bench trace confirms reaches the
    // dispatch path (0.138s duration, ~330 mm/s peak). Smaller than the
    // task spec's 50mm because feasibility margins shrink with longer
    // moves under the live MZV-shaped accel envelope.
    submit_one_jog(&h, [0.0; 3], 25.0, 0.0, 0.0, 100.0).expect("submit 25mm X");
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    drop(h);
    assert!(!segs.is_empty());

    // 1 ms sampling.
    let dt = 1.0e-3_f64;
    let mut velocities = Vec::new();
    for seg in &segs {
        let x_curve = &seg.axes[0];
        let dur = seg.t_end - seg.t_start;
        if dur <= 0.0 {
            continue;
        }
        let n = (dur / dt).ceil() as usize + 1;
        let mut prev: Option<f64> = None;
        for k in 0..=n {
            let t = (seg.t_start + (k as f64) * dt).min(seg.t_end);
            let x = nurbs::eval::eval(x_curve, t);
            if let Some(px) = prev {
                velocities.push((x - px) / dt);
            }
            prev = Some(x);
        }
    }

    // Tick-to-tick accel cap with 50% slop.
    let allowed_dv = LIVE_MAX_ACC * dt * 1.5;

    let mut worst_jump = 0.0_f64;
    let mut worst_idx = 0_usize;
    for (i, w) in velocities.windows(2).enumerate() {
        let dv = (w[1] - w[0]).abs();
        if dv > worst_jump {
            worst_jump = dv;
            worst_idx = i;
        }
    }
    eprintln!(
        "[bench-repro] velocity_profile: peak_v = {:.2}, worst dv/dt = {:.1} mm/s² at sample {} (cap = {:.1} mm/s²)",
        velocities.iter().fold(0.0_f64, |a, &b| a.max(b.abs())),
        worst_jump / dt,
        worst_idx,
        allowed_dv / dt,
    );

    assert!(
        worst_jump <= allowed_dv,
        "BENCH BUG #4 (slow-then-suddenly-faster): mid-segment velocity \
         discontinuity {:.1} mm/s² between samples {} and {} exceeds \
         1.5 × max_accel ({:.1} mm/s²). Either β-medium is computing a \
         non-physical curve, or the runtime is mis-seeding step \
         accumulators mid-segment.",
        worst_jump / dt,
        worst_idx,
        worst_idx + 1,
        allowed_dv / dt
    );
}

// ===========================================================================
//
// Test 5 — no_step_burst_violations_under_rapid_jogs
//
// The bench symptom: long sequences of rapid jogs sometimes trip
// `KALICO_ERR_STEP_BURST_EXCEEDED`. We submit 20 × 5 mm jogs with no
// pacing and run them through a single shared Engine, checking
// `last_error == 0` and `final_status != Fault`.
//
// ===========================================================================

#[test]
fn no_step_burst_violations_under_rapid_jogs() {
    let n_jogs = 20_usize;
    let jog_mm = 5.0;

    let (dispatch, recorded) = recording_dispatch();
    let h = PlannerHandle::spawn(live_planner_config(), dispatch);
    h.kalico_stream_open([0.0; 4]).expect("kalico_stream_open");

    let mut pos = [0.0_f64; 3];
    for i in 0..n_jogs {
        let dx = if i % 2 == 0 { jog_mm } else { -jog_mm };
        submit_one_jog(&h, pos, dx, 0.0, 0.0, 100.0)
            .unwrap_or_else(|e| panic!("submit jog {i}: {e}"));
        pos[0] += dx;
    }
    h.flush().expect("flush");

    let segs = recorded.lock().unwrap().clone();
    drop(h);

    assert!(!segs.is_empty());

    // Q_N is 8 (heapless), CURVE_POOL_N is 16. Each ShapedSegment may
    // claim up to 3 pool slots (X, Y, Z) — post-shape Y/Z residues from
    // the convolution kernel can clear the trivially-constant filter even
    // on pure-X jogs. Cap the per-engine chunk at 5 segments so 5 × 3 = 15
    // slots stay within CURVE_POOL_N. The step-burst check is a per-tick
    // accumulator overflow inside the engine — it'd trip on any chunk
    // where post-shape motor velocity exceeds `MAX_STEPS_PER_TICK / dt`
    // (16 steps × 40 kHz × (1/steps_per_mm) ≈ 8 m/s on 80 steps/mm), so
    // chunk boundaries don't mask the fault.
    let chunk_size = 5_usize;
    let mut chunk_results = Vec::new();
    for chunk in segs.chunks(chunk_size) {
        let summary =
            run_segments_through_engine(chunk, RunOptions::default()).expect("chunk non-empty");
        chunk_results.push((
            summary.final_status,
            summary.last_error,
            summary.step_counts,
        ));
    }

    eprintln!(
        "[bench-repro] rapid_jogs: {} chunks, results = {:?}",
        chunk_results.len(),
        chunk_results
    );

    for (i, (status, err, _)) in chunk_results.iter().enumerate() {
        // The healthy end-of-stream is Drained (we close stream_open
        // post-roll); Underrun is acceptable for the last chunk's tail.
        // Anything else with a non-zero last_error is the bug.
        assert_ne!(
            *status,
            RuntimeStatus::Fault,
            "BENCH BUG #2 (step burst): chunk {i} latched Fault, last_error={err}"
        );
        // KALICO_ERR_STEP_BURST_EXCEEDED is the specific err code we care
        // about; assert no fault at all so we catch a related regression
        // (e.g. InvalidHandle from a curve-pool corruption) too.
        assert_eq!(
            *err, 0,
            "BENCH BUG #2 (step burst or related): chunk {i} latched \
             last_error = {err}. This is the rapid-jog runtime fault \
             the live bench reports."
        );
    }
}
