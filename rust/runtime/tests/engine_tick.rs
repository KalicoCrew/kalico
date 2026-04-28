//! Integration tests for `Engine::tick`. Spec §4.2.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic,
    clippy::items_after_statements
)]

use runtime::clock::one_tick_cycles;
use runtime::curve_pool::CurvePool;
use runtime::engine::{Engine, RuntimeStatus};
use runtime::queue::SegmentQueue;
use runtime::segment::{CurveHandle, KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::trace::{TraceRing, TraceSample, TRACE_FLAG_SEGMENT_END};

// Default H723 Klipper Kconfig clock is 520 MHz (src/stm32/Kconfig). Keeping
// tests parametric here so a future bump to 550 MHz (or different alternate
// kconfig) doesn't invalidate the fixture math.
const CLOCK_FREQ: u32 = 520_000_000;

mod fixtures;  // see Task 17a — shared step5_segments.json parser

/// Load fixture-by-name into the curve pool slot. Single source of truth for
/// "which curves the Step-5 tests use" — mirrored by Surface C's host script.
fn load_fixture(pool: &mut CurvePool, handle: u16, name: &str) {
    let set = fixtures::load();
    let f = set.fixtures.iter().find(|f| f.name == name)
        .unwrap_or_else(|| panic!("fixture {name} missing from step5_segments.json"));
    let cps_flat: Vec<f32> = f.control_points.iter().flat_map(|p| p.iter().copied()).collect();
    pool.load(CurveHandle(handle), &cps_flat, &f.knots, &f.weights, f.degree).unwrap();
}

#[test]
fn tick_on_empty_queue_returns_idle() {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let r = engine.tick(0, &mut SegmentQueue::new(), &CurvePool::new(),
                       &mut TraceRing::<1024>::new());
    assert!(r.is_ok());
    assert_eq!(engine.status(), RuntimeStatus::Idle);
}

#[test]
fn tick_processes_one_segment_to_completion() {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut queue = SegmentQueue::new();
    let mut pool = CurvePool::new();
    let mut trace = TraceRing::<1024>::new();

    load_fixture(&mut pool, 0, "straight_line_x");

    let tick_cycles = u64::from(one_tick_cycles(CLOCK_FREQ));
    let n_ticks = 4u64;
    queue.try_push(Segment {
        id: 1,
        curve: CurveHandle(0),
        t_start: 0,
        t_end: n_ticks * tick_cycles,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();

    // Tick repeatedly through the segment.
    for tick_idx in 0..=n_ticks {
        let now = tick_idx * tick_cycles;
        engine.tick(now, &mut queue, &pool, &mut trace)
            .expect("tick should succeed in healthy run");
    }

    // Drain trace and verify samples emitted along the line.
    let mut out = [TraceSample::default(); 16];
    let n = trace.drain_into(&mut out);
    assert!(n >= 4, "expected at least 4 samples along the line, got {n}");

    // Last sample at u≈1 → motors at endpoint, segment-end flag set.
    let last = &out[n - 1];
    assert_eq!(last.flags & TRACE_FLAG_SEGMENT_END, TRACE_FLAG_SEGMENT_END);
}

#[test]
fn sub_tick_boundary_carries_partial_into_next_segment() {
    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut queue = SegmentQueue::new();
    let mut pool = CurvePool::new();
    let mut trace = TraceRing::<1024>::new();

    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));
    // Two distinct fixtures back-to-back — exercise sub-tick boundary carry.
    // straight_line_x ends at (10,0,0); rational_quadratic_arc starts at
    // (10,0,0) so the boundary is geometrically continuous and motor_a
    // increases monotonically across the seam.
    load_fixture(&mut pool, 0, "straight_line_x");
    load_fixture(&mut pool, 1, "rational_quadratic_arc");

    // Sized so that tick 1 lands near u≈1 of seg1 and tick 2 (post-boundary)
    // lands near u≈0 of seg2 — the only configuration that satisfies the
    // 0.05 mm seam tolerance with the 4-tick test loop. (D1 = tc + 1 cycle
    // → tick 1 at u = tc/(tc+1) ≈ 1; D2 = 1000·tc → tick 2 at u ≈ 0.001.)
    let d1 = tc + 1;
    let d2 = 1000 * tc;
    queue.try_push(Segment {
        id: 1, curve: CurveHandle(0), t_start: 0, t_end: d1,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();
    queue.try_push(Segment {
        id: 2, curve: CurveHandle(1), t_start: d1, t_end: d1 + d2,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();

    // Tick at t = 0, tc, 2tc, 3tc — third tick straddles the seg1→seg2 boundary.
    for tick_idx in 0..=3u64 {
        engine.tick(tick_idx * tc, &mut queue, &pool, &mut trace)
            .expect("tick should succeed in healthy run");
    }

    let mut out = [TraceSample::default(); 16];
    let n = trace.drain_into(&mut out);

    // Boundary correctness check: the LAST sample of segment 1 and the FIRST
    // sample of segment 2 must agree on (motor_a, motor_b, motor_e) to within
    // the sub-tick boundary tolerance. straight_line_x ends at (10, 0, 0);
    // rational_quadratic_arc starts at (10, 0, 0) — both yield motor = (10, 10, 0)
    // at the seam. Per-sample monotonicity over the whole trace is NOT asserted
    // (the arc's motor_a rises to ~14.14 mid-arc and falls back to 10 at u=1
    // because motor_a = X+Y and the arc's path through (10,10,0) increases X+Y).
    let mut last_seg1: Option<&TraceSample> = None;
    let mut first_seg2: Option<&TraceSample> = None;
    for s in out.iter().take(n) {
        if s.segment_id == 1 { last_seg1 = Some(s); }
        if s.segment_id == 2 && first_seg2.is_none() { first_seg2 = Some(s); }
    }
    let last1 = last_seg1.expect("expected at least one sample from segment 1");
    let first2 = first_seg2.expect("expected at least one sample from segment 2");

    // Seam tolerance: 25 µm × 2 (start + end of tick) = 50 µm = 0.05 mm.
    const SEAM_TOL_MM: f32 = 0.05;
    assert!((first2.motor_a - last1.motor_a).abs() < SEAM_TOL_MM,
        "motor_a discontinuous at seam: {} → {}", last1.motor_a, first2.motor_a);
    assert!((first2.motor_b - last1.motor_b).abs() < SEAM_TOL_MM,
        "motor_b discontinuous at seam: {} → {}", last1.motor_b, first2.motor_b);
    assert!((first2.motor_e - last1.motor_e).abs() < SEAM_TOL_MM,
        "motor_e discontinuous at seam: {} → {}", last1.motor_e, first2.motor_e);
}
