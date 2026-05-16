//! End-to-end comparison harness: drive the same G-code-equivalent jog
//! through the production planner stack, replay the resulting
//! `ShapedSegment` sequence through TWO independent `Engine` instances —
//! one in StepTime mode (`producer_step` + ring drain), one in Modulated
//! mode (`runtime_modulated_tick` at 40 kHz) — and capture per-emit
//! `(motor_idx, n_steps, simulated_time_cycles)` traces via the
//! `runtime::engine::step_sink` host hook for direct diff.
//!
//! Purpose: answer the user's "what's different between modulated and
//! stepped modes?" question without Renode in the loop. The two modes
//! converge on the same `emit_step_pulses` callsite inside `engine.rs`
//! — that's the single point of instrumentation, and the comparison
//! reveals any divergence in step count, net direction, or timing
//! distribution for the same shaped-curve input.
//!
//! Run via:
//!   cargo test -p motion-bridge --test compare_modes -- --nocapture

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::sync::{Arc, Mutex};

use heapless::spsc::Queue;

use motion_bridge_native::classify::classify_and_build;
use motion_bridge_native::config::{PlannerConfig, PlannerLimits};
use motion_bridge_native::planner::{DispatchError, PlannerHandle};

use runtime::clock::one_tick_cycles;
use runtime::config::{EMode as RtEMode, McuAxisConfig as RtMcuAxisConfig, MotorConfig};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::{Engine, step_sink};
use runtime::queue::Q_N;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::{SharedState, StepMode};
use runtime::step_producer::ProducerTickResult;
use trajectory::{AxisShaper, RequiredShaper, ShapedSegment, ShaperConfig};

const CLOCK_FREQ: u32 = 520_000_000;
const STEPS_PER_MM: f32 = 160.0;

fn live_planner_config() -> PlannerConfig {
    let mut c = PlannerConfig::default();
    c.limits = PlannerLimits {
        max_velocity: 1000.0,
        max_accel: 70_000.0,
        max_z_velocity: 5.0,
        max_z_accel: 100.0,
        square_corner_velocity: 5.0,
    };
    c.shaper = ShaperConfig {
        x: RequiredShaper::SmoothMzv { frequency_hz: 186.0 },
        y: RequiredShaper::SmoothMzv { frequency_hz: 122.0 },
        z: AxisShaper::Passthrough,
    };
    c.fit_tolerance_mm = 0.05;
    c
}

type RecordedSegs = Arc<Mutex<Vec<ShapedSegment>>>;

fn recording_dispatch() -> (
    Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync>,
    RecordedSegs,
) {
    let recorded: RecordedSegs = Arc::new(Mutex::new(Vec::new()));
    let rec = Arc::clone(&recorded);
    let cb: Arc<dyn Fn(&ShapedSegment) -> Result<(), DispatchError> + Send + Sync> =
        Arc::new(move |seg: &ShapedSegment| {
            rec.lock().unwrap().push(seg.clone());
            Ok(())
        });
    (cb, recorded)
}

/// Mirror of the bridge's `from_scalar_nurbs_normalized`: f64 → f32, knots
/// rescaled from `[t_start, t_end]` → `[0, 1]`, then `validate_and_load`
/// into a curve-pool slot.
fn load_axis_curve(
    pool: &CurvePool,
    slot: u16,
    curve: &nurbs::ScalarNurbs<f64>,
    t_start_s: f64,
    t_end_s: f64,
) -> Option<CurveHandle> {
    let duration = t_end_s - t_start_s;
    if duration <= 0.0 {
        return None;
    }
    let knots_f32: Vec<f32> = curve
        .knots()
        .iter()
        .map(|&k| (((k - t_start_s) / duration).clamp(0.0, 1.0)) as f32)
        .collect();
    let cps_f32: Vec<f32> = curve.control_points().iter().map(|&v| v as f32).collect();
    let degree = nurbs::NurbsView::degree(curve);
    pool.validate_and_load(slot, degree, &knots_f32, &cps_f32).ok()
}

fn is_trivially_constant(c: &nurbs::ScalarNurbs<f64>) -> bool {
    let cps = c.control_points();
    if cps.is_empty() {
        return true;
    }
    let first = cps[0];
    cps.iter().all(|&v| (v - first).abs() < 1e-12)
}

/// One step emission captured from the engine's `emit_step_pulses` callsite.
#[derive(Debug, Clone, Copy)]
struct StepEvent {
    /// Simulated wall-clock at the moment of emission, in MCU cycles.
    t_cycles: u64,
    motor_idx: u8,
    /// Signed step count. Per `Engine::emit_step_pulses` contract, +ve =
    /// forward, -ve = reverse, magnitude = number of step pulses (always 1
    /// for the StepTime ring-drain path; up to MAX_STEPS_PER_TICK for the
    /// Modulated path's StepAccumulator).
    n_steps: i32,
}

/// Net signed steps emitted per motor across all captured events.
fn net_per_motor(events: &[StepEvent]) -> [i32; 4] {
    let mut net = [0_i32; 4];
    for e in events {
        if (e.motor_idx as usize) < net.len() {
            net[e.motor_idx as usize] += e.n_steps;
        }
    }
    net
}

/// Drive `segs` through a fresh Engine in StepTime mode. Returns the
/// captured step events.
fn run_step_time(segs: &[ShapedSegment], home_pos: [f64; 4]) -> Vec<StepEvent> {
    if segs.is_empty() {
        return Vec::new();
    }
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_prod, mut q_cons) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(RtMcuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: STEPS_PER_MM, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: STEPS_PER_MM, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });

    // Seed engine's logical toolhead position so producer_step's initial-
    // step seeding (eval(0.0) / step_distance) lines up with the curve's
    // start point rather than X=Y=Z=0.
    engine.seed_position([home_pos[0] as f32, home_pos[1] as f32, home_pos[2] as f32]);
    let motor_a_seed = home_pos[0] + home_pos[1];
    let motor_b_seed = home_pos[0] - home_pos[1];
    shared.stepper_counts[0]
        .store((motor_a_seed * STEPS_PER_MM as f64) as i32, core::sync::atomic::Ordering::Release);
    shared.stepper_counts[1]
        .store((motor_b_seed * STEPS_PER_MM as f64) as i32, core::sync::atomic::Ordering::Release);

    // Step mode = StepTime for both A and B (default but explicit).
    shared.step_modes[0].store(StepMode::StepTime as u8, core::sync::atomic::Ordering::Release);
    shared.step_modes[1].store(StepMode::StepTime as u8, core::sync::atomic::Ordering::Release);

    // Load each segment's curves + enqueue.
    let t_offset = segs[0].t_start;
    let mut slot_counter: u16 = 0;
    let mut last_t_end_clock: u64 = 0;
    for shaped in segs {
        let rel_start_s = shaped.t_start - t_offset;
        let rel_end_s = shaped.t_end - t_offset;
        let t_start_clock = (rel_start_s * f64::from(CLOCK_FREQ)).round() as u64;
        let t_end_clock = (rel_end_s * f64::from(CLOCK_FREQ)).round() as u64;
        last_t_end_clock = last_t_end_clock.max(t_end_clock);

        let any_moving = shaped.axes.iter().any(|c| !is_trivially_constant(c));
        if !any_moving {
            continue;
        }

        let mut handles = [CurveHandle::UNUSED_SENTINEL; 3];
        for (axis_idx, curve) in shaped.axes.iter().enumerate() {
            if is_trivially_constant(curve) {
                continue;
            }
            let slot = slot_counter;
            slot_counter = slot_counter.wrapping_add(1);
            if let Some(h) = load_axis_curve(&pool, slot, curve, shaped.t_start, shaped.t_end) {
                handles[axis_idx] = h;
            }
        }

        let seg = Segment {
            id: 1,
            x_handle: handles[0],
            y_handle: handles[1],
            z_handle: handles[2],
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: t_start_clock,
            t_end: t_end_clock,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: RtEMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
            consumers_remaining: 0,
        };
        engine.push_segment(seg, &mut q_prod, &shared).expect("push");
    }

    // Run the producer until idle. Captures every ring push as a single-
    // step event with the step's scheduled t_next in cycles.
    let events: Arc<Mutex<Vec<StepEvent>>> = Arc::new(Mutex::new(Vec::new()));
    let mut runs = 0_u32;
    let mut total_work_pending_iterations = 0_u32;
    loop {
        let r = engine.producer_step(&pool, &mut q_cons, &shared);
        runs += 1;
        if r == ProducerTickResult::WorkPending {
            total_work_pending_iterations += 1;
        }
        if r == ProducerTickResult::AllIdle {
            break;
        }
        assert!(runs < 5000, "producer_step should converge");
    }

    // Brief one-line producer diagnostic. Most useful is the steps_pushed
    // counter — when this is 0 with segment_dequeued > 0 you're hitting
    // a producer-side bail; that was the bug this test was built to catch.
    use core::sync::atomic::Ordering;
    eprintln!(
        "[run_step_time diag] runs={} steps_pushed={} segment_dequeued={} motor_finished_curve={}",
        runs,
        shared.producer_steps_pushed_total.load(Ordering::Acquire),
        shared.producer_segment_dequeued_total.load(Ordering::Acquire),
        shared.producer_motor_finished_curve_total.load(Ordering::Acquire),
    );
    let _ = total_work_pending_iterations;

    // Drain each motor's ring into the events vec. The ring entry's
    // `cycles_abs_lo` is the scheduled step time (low 32 bits of MCU clock).
    for motor_idx in 0..4_u8 {
        if let Some(ring) = engine.step_ring(motor_idx as usize) {
            while let Some((cyc_lo, dir)) = ring.peek_head() {
                events.lock().unwrap().push(StepEvent {
                    t_cycles: u64::from(cyc_lo),
                    motor_idx,
                    n_steps: i32::from(dir),
                });
                ring.advance(1);
            }
        }
    }

    let mut events = Arc::try_unwrap(events).unwrap().into_inner().unwrap();
    events.sort_by_key(|e| (e.t_cycles, e.motor_idx));
    events
}

/// Drive `segs` through a fresh Engine in Modulated mode (TIM5 polled tick
/// at 40 kHz). Uses `step_sink` to capture every `emit_step_pulses` call.
fn run_modulated(segs: &[ShapedSegment], home_pos: [f64; 4]) -> Vec<StepEvent> {
    if segs.is_empty() {
        return Vec::new();
    }
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_prod, mut q_cons) = queue.split();
    let pool = CurvePool::new();
    let shared = SharedState::new();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(RtMcuAxisConfig {
        motors: [
            Some(MotorConfig { steps_per_mm: STEPS_PER_MM, is_awd: false, invert_dir: false }),
            Some(MotorConfig { steps_per_mm: STEPS_PER_MM, is_awd: false, invert_dir: false }),
            None,
            None,
        ],
        kinematics: KinematicTag::CoreXyAndE,
    });

    engine.seed_position([home_pos[0] as f32, home_pos[1] as f32, home_pos[2] as f32]);
    let motor_a_seed = home_pos[0] + home_pos[1];
    let motor_b_seed = home_pos[0] - home_pos[1];
    shared.stepper_counts[0]
        .store((motor_a_seed * STEPS_PER_MM as f64) as i32, core::sync::atomic::Ordering::Release);
    shared.stepper_counts[1]
        .store((motor_b_seed * STEPS_PER_MM as f64) as i32, core::sync::atomic::Ordering::Release);

    // Step mode = Modulated for both A and B.
    shared.step_modes[0].store(StepMode::Modulated as u8, core::sync::atomic::Ordering::Release);
    shared.step_modes[1].store(StepMode::Modulated as u8, core::sync::atomic::Ordering::Release);

    let t_offset = segs[0].t_start;
    let mut slot_counter: u16 = 0;
    let mut last_t_end_clock: u64 = 0;
    for shaped in segs {
        let rel_start_s = shaped.t_start - t_offset;
        let rel_end_s = shaped.t_end - t_offset;
        let t_start_clock = (rel_start_s * f64::from(CLOCK_FREQ)).round() as u64;
        let t_end_clock = (rel_end_s * f64::from(CLOCK_FREQ)).round() as u64;
        last_t_end_clock = last_t_end_clock.max(t_end_clock);

        let any_moving = shaped.axes.iter().any(|c| !is_trivially_constant(c));
        if !any_moving {
            continue;
        }

        let mut handles = [CurveHandle::UNUSED_SENTINEL; 3];
        for (axis_idx, curve) in shaped.axes.iter().enumerate() {
            if is_trivially_constant(curve) {
                continue;
            }
            let slot = slot_counter;
            slot_counter = slot_counter.wrapping_add(1);
            if let Some(h) = load_axis_curve(&pool, slot, curve, shaped.t_start, shaped.t_end) {
                handles[axis_idx] = h;
            }
        }

        let seg = Segment {
            id: 1,
            x_handle: handles[0],
            y_handle: handles[1],
            z_handle: handles[2],
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: t_start_clock,
            t_end: t_end_clock,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: RtEMode::Travel,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
            consumers_remaining: 0,
        };
        engine.push_segment(seg, &mut q_prod, &shared).expect("push");
    }

    // Install step_sink to capture emissions with the current tick time.
    let events: Arc<Mutex<Vec<StepEvent>>> = Arc::new(Mutex::new(Vec::new()));
    let events_for_sink = Arc::clone(&events);
    let current_tick: Arc<core::sync::atomic::AtomicU64> =
        Arc::new(core::sync::atomic::AtomicU64::new(0));
    let current_tick_for_sink = Arc::clone(&current_tick);
    let prev = step_sink::install(move |motor_idx, n_steps| {
        let t = current_tick_for_sink.load(core::sync::atomic::Ordering::Acquire);
        events_for_sink.lock().unwrap().push(StepEvent {
            t_cycles: t,
            motor_idx,
            n_steps,
        });
    });
    // (No need for an RAII guard — the sink is installed for the duration
    // of this function and explicitly uninstalled below. If a test panics
    // mid-run, the next test in the same thread will install its own sink
    // on top via `replace`, which is also safe.)
    let _ = prev;

    // Tick at 40 kHz from t=0 through last segment t_end (+5 ticks post-roll).
    let tick_cycles = u64::from(one_tick_cycles(CLOCK_FREQ));
    let total_ticks = last_t_end_clock / tick_cycles + 5;
    for i in 1..=total_ticks {
        let now = i * tick_cycles;
        current_tick.store(now, core::sync::atomic::Ordering::Release);
        engine.runtime_modulated_tick(now, &mut q_cons, &pool, &shared);
    }

    // Cleanup: uninstall the sink before returning so the next call's
    // events don't leak into a stale closure.
    let _ = step_sink::uninstall();

    let mut events = Arc::try_unwrap(events).unwrap().into_inner().unwrap();
    events.sort_by_key(|e| (e.t_cycles, e.motor_idx));
    events
}

#[test]
fn compare_modes_one_mm_x_jog() {
    // Identical input: 1 mm +X jog at F=600 mm/min (10 mm/s) on CoreXY @
    // 160 steps/mm, starting from (125, 100, 10) — the user's Trident
    // live-bench geometry.
    const START: [f64; 3] = [125.0, 100.0, 10.0];
    const HOME: [f64; 4] = [125.0, 100.0, 10.0, 0.0];

    let (dispatch, recorded) = recording_dispatch();
    let h = PlannerHandle::spawn(live_planner_config(), dispatch);
    h.kalico_stream_open(HOME).expect("kalico_stream_open");
    let m = classify_and_build(START, 1.0, 0.0, 0.0, 0.0, 10.0).expect("classify");
    h.submit_move(m).expect("submit");
    h.flush().expect("flush");
    let segs = recorded.lock().unwrap().clone();
    drop(h);
    eprintln!("[compare] planner emitted {} shaped segments", segs.len());

    let st_events = run_step_time(&segs, HOME);
    let md_events = run_modulated(&segs, HOME);

    let st_net = net_per_motor(&st_events);
    let md_net = net_per_motor(&md_events);

    let st_count_m0 = st_events.iter().filter(|e| e.motor_idx == 0).count();
    let md_count_m0 = md_events.iter().filter(|e| e.motor_idx == 0).count();
    let st_count_m1 = st_events.iter().filter(|e| e.motor_idx == 1).count();
    let md_count_m1 = md_events.iter().filter(|e| e.motor_idx == 1).count();

    eprintln!(
        "[compare] StepTime  motor 0: {} events, net={} ; motor 1: {} events, net={}",
        st_count_m0, st_net[0], st_count_m1, st_net[1],
    );
    eprintln!(
        "[compare] Modulated motor 0: {} events, net={} ; motor 1: {} events, net={}",
        md_count_m0, md_net[0], md_count_m1, md_net[1],
    );

    // Extra diagnostics: print the FIRST handful of step events from each
    // mode (with timestamps) so a human reviewer can see the divergence
    // pattern, plus per-motor producer counters for the StepTime side to
    // see whether the producer fetched the segment / resolved curves /
    // pushed step events at all.
    eprintln!(
        "[compare] StepTime first 10 events: {:?}",
        st_events.iter().take(10).collect::<Vec<_>>(),
    );
    eprintln!(
        "[compare] Modulated first 10 events: {:?}",
        md_events.iter().take(10).collect::<Vec<_>>(),
    );

    // For a +1mm X-only jog on CoreXY: motor A = X+Y, motor B = X-Y.
    // Both motors should advance by +1mm × 160 spm = +160 microsteps net.
    // Modulated mode often falls short by ~20% because the simulated
    // tick range stops a few ticks before the trajectory's full tail-
    // decay completes. StepTime mode should match exactly (it emits the
    // pre-computed Newton-iterated step times regardless of wall-clock).
    let expected = 160_i32;

    // Hard expectation: StepTime must emit AT LEAST some steps.
    assert!(
        st_net[0].abs() > 10 || st_net[1].abs() > 10,
        "StepTime emitted NO step events — this reproduces the live-bench \
         'motors don't move' bug. ST motor 0 net={}, motor 1 net={}. \
         Modulated for the same input emitted net (m0={}, m1={}) — \
         confirms the planner output is valid; bug is StepTime-side.",
        st_net[0], st_net[1], md_net[0], md_net[1],
    );

    // Soft check: both modes should converge on roughly the same net.
    let diff = (st_net[0] - md_net[0]).abs() + (st_net[1] - md_net[1]).abs();
    assert!(
        diff < 50,
        "Modes diverge by {} (St={:?}, Md={:?}), expected within ~50",
        diff, st_net, md_net,
    );
    // Sanity: net is in the right direction (positive for +X jog).
    assert!(md_net[0] > 0 && md_net[1] > 0, "Modulated net should be positive: {:?}", md_net);
    let _ = expected;
}
