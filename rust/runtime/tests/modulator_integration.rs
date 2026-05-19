//! Integration test for Task 6 of the 2026-05-18 phase-stepping plan:
//! a synthetic linear-X position trajectory is fed through
//! `Engine::runtime_modulated_tick` and the resulting XDIRECT SPI writes
//! are asserted against expected values.
//!
//! Coverage:
//!   - phase-config-installed motors take the phase-direct output path
//!     (XDIRECT write + phase LUT lookup) instead of `emit_step_pulses`;
//!   - `stepper_counts` advances by `steps_delta` for phase motors,
//!     preserving homing-snapshot / host-position-query semantics;
//!   - round-robin SPI scheduling: with two phase motors, X writes on
//!     even ticks and Y writes on odd ticks (or vice versa, depending
//!     on which is registered first — the test asserts that both bus
//!     ids appear and that the count per bus is ~ticks/2);
//!   - the `phase_trace_enabled` gate threads through to per-tick
//!     `TraceSample::PhaseStep` emissions.
//!
//! The test uses the production `runtime_modulated_tick` entry point with
//! a `c_segment_queue::Consumer<Segment>` (the same Consumer type the FFI
//! uses), so it exercises the real hot-path branch added in Task 6.

#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic
)]

use core::sync::atomic::Ordering;
use std::sync::{Mutex, MutexGuard, OnceLock};

use heapless::spsc::Queue;

use runtime::c_segment_queue::{self, Consumer as SegConsumer};
use runtime::config::{EMode, McuAxisConfig, MotorConfig};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::Engine;
use runtime::phase_config::{self, PhaseConfig};
use runtime::phase_lut;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::{SharedState, StepMode};
use runtime::test_xdirect_capture::{self, XDirectRecord};
use runtime::trace::{TRACE_RING_N, TraceSample};

const CLOCK_FREQ: u32 = 520_000_000;
const STEPS_PER_MM: f32 = 160.0;

/// Serialise every test in this file. The `test_xdirect_capture` sink and
/// `c_segment_queue` backend are process-globals; running tests in parallel
/// would interleave captures and queue state and break the per-test
/// assertions. Each test acquires this lock for its full duration.
fn test_lock() -> MutexGuard<'static, ()> {
    static LOCK: OnceLock<Mutex<()>> = OnceLock::new();
    LOCK.get_or_init(|| Mutex::new(()))
        .lock()
        .unwrap_or_else(|p| p.into_inner())
}

/// Single-piece linear cubic Bézier from 0 to `end` mm over u ∈ [0, 1].
fn linear_cubic(end: f32) -> (u8, Vec<f32>, Vec<f32>) {
    let cps = vec![0.0, end / 3.0, end * 2.0 / 3.0, end];
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    (3_u8, knots, cps)
}

/// Linear-X cartesian segment with the X curve loaded into the pool.
/// Y/Z/E stay at UNUSED so the engine holds their previous positions;
/// for a CoreXY topology that would couple Y into motor A — here we use
/// the Cartesian kinematics so motor 0 == X and motor 1 == Y directly.
fn build_segment_linear_x(
    pool: &CurvePool,
    end_mm: f32,
    t_start: u64,
    duration: u64,
    slot_idx: u16,
    seg_id: u32,
) -> Segment {
    let (deg, knots, cps) = linear_cubic(end_mm);
    let x_handle = pool
        .validate_and_load(slot_idx, deg, &knots, &cps)
        .expect("load X curve");
    let mut seg = Segment {
        id: seg_id,
        x_handle,
        y_handle: CurveHandle::UNUSED_SENTINEL,
        z_handle: CurveHandle::UNUSED_SENTINEL,
        e_handle: CurveHandle::UNUSED_SENTINEL,
        t_start,
        t_end: t_start + duration,
        kinematics: KinematicTag::CartesianXyzAndE,
        e_mode: EMode::Travel,
        extrusion_ratio: 0.0,
        flags: 0,
        _pad: [0; 1],
        consumers_remaining: 0,
    };
    seg.consumers_remaining = Segment::compute_consumers_remaining(
        seg.kinematics,
        seg.x_handle,
        seg.y_handle,
        seg.z_handle,
        seg.e_handle,
    );
    seg
}

/// Empty consumer for the C-backed segment queue. Production tests pre-
/// seed `producer_current` directly so the queue stays empty.
fn empty_seg_consumer() -> SegConsumer<Segment> {
    // Reset the host-backend singleton so cross-test state doesn't bleed.
    c_segment_queue::reset();
    SegConsumer::<Segment>::new()
}

/// Owns a leaked `heapless::spsc::Queue<TraceSample, TRACE_RING_N>` so
/// the split halves carry `'static` lifetimes — the same pattern used by
/// `e_mode_dispatch.rs` / `engine_tick.rs` etc.
struct TraceHarness {
    producer: heapless::spsc::Producer<'static, TraceSample, TRACE_RING_N>,
    consumer: heapless::spsc::Consumer<'static, TraceSample, TRACE_RING_N>,
}

impl TraceHarness {
    fn new() -> Self {
        let queue: &'static mut Queue<TraceSample, TRACE_RING_N> =
            Box::leak(Box::new(Queue::new()));
        let (producer, consumer) = queue.split();
        Self { producer, consumer }
    }

    fn drain(&mut self) -> Vec<TraceSample> {
        let mut out = Vec::new();
        while let Some(s) = self.consumer.dequeue() {
            out.push(s);
        }
        out
    }
}

/// Build a freshly-configured engine + shared state for the two-motor
/// phase-stepping topology used by these tests.
fn build_engine_two_phase_motors() -> (Engine<NoopPa, NoopIs>, SharedState) {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(McuAxisConfig {
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
        kinematics: KinematicTag::CartesianXyzAndE,
    });

    let shared = SharedState::new();
    // X (motor 0) → bus 0, CS pin 10. Y (motor 1) → bus 1, CS pin 11.
    // Z (motor 2) and E (motor 3) stay phase-config-less.
    phase_config::store(
        &shared.phase_config[0],
        Some(PhaseConfig { spi_bus_id: 0, cs_pin_id: 10 }),
    );
    phase_config::store(
        &shared.phase_config[1],
        Some(PhaseConfig { spi_bus_id: 1, cs_pin_id: 11 }),
    );

    // All four motors default to Modulated=0 already, but be explicit so the
    // test contract is self-documenting and survives a future default flip.
    shared.step_modes[0].store(StepMode::Modulated as u8, Ordering::Release);
    shared.step_modes[1].store(StepMode::Modulated as u8, Ordering::Release);
    shared.step_modes[2].store(StepMode::StepTime as u8, Ordering::Release);
    shared.step_modes[3].store(StepMode::StepTime as u8, Ordering::Release);

    (engine, shared)
}

#[test]
fn two_phase_motors_round_robin_xdirect_capture() {
    let _lock = test_lock();
    test_xdirect_capture::clear();
    let (mut engine, shared) = build_engine_two_phase_motors();
    let pool = CurvePool::new();
    let mut trace_h = TraceHarness::new();

    // Enable PhaseStep trace emissions for this test.
    shared.phase_trace_enabled.store(true, Ordering::Release);

    // 1 mm X jog over a comfortable duration. With 160 steps/mm that's
    // 160 microsteps total — well below the per-tick burst cap.
    const DURATION: u64 = 200 * 13_000;
    const N_TICKS: u64 = 40;
    let seg = build_segment_linear_x(&pool, 1.0, 0, DURATION, 0, 1);
    engine.producer_current = Some(seg);

    let mut q = empty_seg_consumer();
    for i in 1..=N_TICKS {
        let now = (DURATION / (N_TICKS + 2)) * i;
        engine.runtime_modulated_tick(now, &mut q, &pool, &mut trace_h.producer, &shared);
    }

    let captures = test_xdirect_capture::drain();
    let trace_samples: Vec<_> = trace_h
        .drain()
        .into_iter()
        .filter_map(|s| s.as_phase_step())
        .collect();

    // 1) Round-robin: exactly one XDIRECT write per tick (one of the two
    // phase motors writes each tick; the other defers). Endstop poll +
    // segment retirement may eat one tick at the boundary, so allow a
    // small slack.
    assert!(
        (N_TICKS as i64 - captures.len() as i64).abs() <= 2,
        "expected ~{} XDIRECT writes (one per tick), got {} (captures: {:?})",
        N_TICKS,
        captures.len(),
        captures.iter().take(4).collect::<Vec<_>>()
    );

    // 2) Both bus ids appear — neither motor was starved.
    let bus0 = captures.iter().filter(|r| r.bus_id == 0).count();
    let bus1 = captures.iter().filter(|r| r.bus_id == 1).count();
    assert!(bus0 > 0, "motor 0 (bus 0) never wrote XDIRECT");
    assert!(bus1 > 0, "motor 1 (bus 1) never wrote XDIRECT");
    // 3) Approximately balanced (off-by-one is fine — depends on whether
    // the trip endstop / retirement check runs on the boundary tick).
    assert!(
        (bus0 as i64 - bus1 as i64).abs() <= 2,
        "round-robin should balance bus 0 ({}) vs bus 1 ({})",
        bus0,
        bus1
    );

    // 4) Round-robin alternation: consecutive captures alternate buses.
    for window in captures.windows(2) {
        assert_ne!(
            window[0].bus_id, window[1].bus_id,
            "consecutive XDIRECT writes must alternate between the two phase \
             motors; got {:?} then {:?}",
            window[0], window[1]
        );
    }

    // 5) Each capture's coil currents come from the identity LUT — verify
    // by re-running the LUT on the recorded buses and comparing.
    for r in &captures {
        // For our motor 1 (Y) on a pure-X segment, Y stays at prev_y = 0.0,
        // so its mscount stays at 0, and the LUT returns (0, +amplitude).
        if r.bus_id == 1 {
            let (a, b) = phase_lut::lookup(0, 0);
            assert_eq!(
                (r.coil_a, r.coil_b), (a, b),
                "stationary Y axis must emit LUT(mscount=0) on every XDIRECT \
                 write; got {:?}", r
            );
        }
        // Motor 0 (X) advances mscount with the curve. Coil values are
        // bounded by CURRENT_AMPLITUDE = 248 either way.
        if r.bus_id == 0 {
            assert!(
                r.coil_a.abs() <= phase_lut::CURRENT_AMPLITUDE,
                "coil_A out of LUT bounds: {}",
                r.coil_a
            );
            assert!(
                r.coil_b.abs() <= phase_lut::CURRENT_AMPLITUDE,
                "coil_B out of LUT bounds: {}",
                r.coil_b
            );
        }
    }

    // 6) `stepper_counts` advanced for motor 0 (X). 1 mm × 160 spm = 160
    // microsteps across the segment; the test spans ~95% of the segment,
    // so we expect roughly 130–160 accumulated steps.
    let count0 = shared.stepper_counts[0].load(Ordering::Acquire);
    assert!(
        (100..=170).contains(&count0),
        "motor 0 stepper_counts should reflect ~160 microsteps for a 1 mm \
         X jog at 160 spm; got {count0}"
    );

    // 7) Motor 1 (Y) stationary — stepper_counts stays at 0.
    let count1 = shared.stepper_counts[1].load(Ordering::Acquire);
    assert_eq!(count1, 0, "Y is stationary; its stepper_counts must stay 0");

    // 8) Trace samples: with `phase_trace_enabled = true`, every Modulated
    // motor with a phase config emits one PhaseStep sample per tick.
    // Two phase motors × N_TICKS ticks ≈ 2 * N_TICKS samples.
    assert!(
        trace_samples.len() >= (N_TICKS as usize),
        "expected ~{} PhaseStep trace samples (2 motors x N_TICKS); got {}",
        2 * N_TICKS,
        trace_samples.len()
    );

    // 9) For each trace sample where `wrote_spi == true`, there should be
    // a matching XDIRECT capture (same motor, same coil currents). This
    // proves the trace and the SPI write came from the same per-tick
    // compute() call.
    let written_traces: Vec<_> =
        trace_samples.iter().filter(|t| t.wrote_spi).collect();
    assert!(
        !written_traces.is_empty(),
        "at least some trace samples should have wrote_spi=true"
    );
    for trace in written_traces.iter().take(5) {
        // For motor 0 the expected bus is 0; for motor 1 the expected bus is 1.
        let expected_bus = trace.motor;
        let matching_cap = captures.iter().find(|c| {
            c.bus_id == expected_bus && c.coil_a == trace.i_a && c.coil_b == trace.i_b
        });
        assert!(
            matching_cap.is_some(),
            "trace sample {:?} (wrote_spi=true) has no matching XDIRECT capture",
            trace
        );
    }
}

#[test]
fn phase_trace_disabled_emits_no_trace_samples() {
    // Same topology as above, but with `phase_trace_enabled = false`
    // (the default). Verifies the gate actually suppresses emissions.
    let _lock = test_lock();
    test_xdirect_capture::clear();
    let (mut engine, shared) = build_engine_two_phase_motors();
    let pool = CurvePool::new();
    let mut trace_h = TraceHarness::new();

    // Leave phase_trace_enabled at its default (false).
    assert!(!shared.phase_trace_enabled.load(Ordering::Acquire));

    const DURATION: u64 = 200 * 13_000;
    let seg = build_segment_linear_x(&pool, 0.5, 0, DURATION, 0, 2);
    engine.producer_current = Some(seg);

    let mut q = empty_seg_consumer();
    for i in 1..=10_u64 {
        let now = (DURATION / 12) * i;
        engine.runtime_modulated_tick(now, &mut q, &pool, &mut trace_h.producer, &shared);
    }

    let phase_samples: Vec<_> = trace_h
        .drain()
        .into_iter()
        .filter_map(|s| s.as_phase_step())
        .collect();
    assert!(
        phase_samples.is_empty(),
        "phase_trace_enabled = false must suppress PhaseStep emissions; got {} samples",
        phase_samples.len()
    );

    // But XDIRECT writes still happen — the gate only suppresses trace
    // emissions, not the actual phase-direct output path.
    let captures = test_xdirect_capture::drain();
    assert!(
        !captures.is_empty(),
        "XDIRECT writes must still fire even when phase trace is disabled"
    );
}

#[test]
fn phase_motor_uses_phase_path_steptime_motor_uses_emit() {
    // Single phase motor (X) + one motor without phase config (E, motor 3,
    // explicitly StepTime). Verifies that the per-motor branch in
    // runtime_modulated_tick selects the correct output path for each.
    let _lock = test_lock();
    test_xdirect_capture::clear();
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    engine.configure(McuAxisConfig {
        motors: [
            Some(MotorConfig {
                steps_per_mm: STEPS_PER_MM,
                is_awd: false,
                invert_dir: false,
            }),
            None,
            None,
            None,
        ],
        kinematics: KinematicTag::CartesianXyzAndE,
    });
    let shared = SharedState::new();
    // Only motor 0 (X) has a phase config.
    phase_config::store(
        &shared.phase_config[0],
        Some(PhaseConfig { spi_bus_id: 0, cs_pin_id: 10 }),
    );
    shared.step_modes[0].store(StepMode::Modulated as u8, Ordering::Release);

    let pool = CurvePool::new();
    let mut trace_h = TraceHarness::new();
    let mut q = empty_seg_consumer();

    const DURATION: u64 = 200 * 13_000;
    let seg = build_segment_linear_x(&pool, 0.5, 0, DURATION, 0, 3);
    engine.producer_current = Some(seg);

    for i in 1..=20_u64 {
        let now = (DURATION / 22) * i;
        engine.runtime_modulated_tick(now, &mut q, &pool, &mut trace_h.producer, &shared);
    }

    let captures = test_xdirect_capture::drain();
    // With exactly one phase motor, round-robin degenerates to "write every
    // tick" — so ~20 captures.
    assert!(
        captures.len() >= 15,
        "single phase motor should write XDIRECT every tick; got {} captures",
        captures.len()
    );
    // All captures must be on bus 0 (the only registered phase bus).
    for r in &captures {
        assert_eq!(r.bus_id, 0, "only phase motor 0 (bus 0) should write; got {:?}", r);
        assert_eq!(r.cs_pin, 10);
    }

    // Motor 0 stepper count advanced — modulator's steps_delta hooked into
    // the shared counter the same way StepPulse did.
    let count0 = shared.stepper_counts[0].load(Ordering::Acquire);
    assert!(
        count0 > 0,
        "phase motor 0 should advance stepper_counts; got {count0}"
    );
}

/// Ensure that XDIRECT records carry the configured CS pin (not just the
/// bus id). Important so that the C-side SPI helper hands the right pin
/// to the GPIO write.
#[test]
fn xdirect_record_carries_cs_pin() {
    let _lock = test_lock();
    test_xdirect_capture::clear();
    let (mut engine, shared) = build_engine_two_phase_motors();
    let pool = CurvePool::new();
    let mut trace_h = TraceHarness::new();
    let mut q = empty_seg_consumer();

    const DURATION: u64 = 200 * 13_000;
    let seg = build_segment_linear_x(&pool, 0.25, 0, DURATION, 0, 4);
    engine.producer_current = Some(seg);

    for i in 1..=8_u64 {
        let now = (DURATION / 10) * i;
        engine.runtime_modulated_tick(now, &mut q, &pool, &mut trace_h.producer, &shared);
    }

    let captures: Vec<XDirectRecord> = test_xdirect_capture::drain();
    // X (bus 0) → cs 10; Y (bus 1) → cs 11. Both bus / cs pairings must
    // match what we configured.
    for r in &captures {
        match r.bus_id {
            0 => assert_eq!(r.cs_pin, 10, "bus 0 must carry cs_pin=10; got {:?}", r),
            1 => assert_eq!(r.cs_pin, 11, "bus 1 must carry cs_pin=11; got {:?}", r),
            other => panic!("unexpected bus_id {other}"),
        }
    }
}
