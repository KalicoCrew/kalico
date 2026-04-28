//! Wrap-arithmetic tests. Spec §5.8.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use runtime::clock::{one_tick_cycles, WidenState};
use runtime::curve_pool::CurvePool;
use runtime::engine::{Engine, RuntimeStatus};
use runtime::queue::SegmentQueue;
use runtime::segment::{CurveHandle, KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::trace::TraceRing;

// Default H723 Klipper Kconfig clock is 520 MHz (src/stm32/Kconfig). Keeping
// tests parametric here so a future bump to 550 MHz (or different alternate
// kconfig) doesn't invalidate the fixture math.
const CLOCK_FREQ: u32 = 520_000_000;

#[test]
fn widen_handles_max_minus_one_to_zero() {
    let mut state = WidenState::default();
    state.reinit(0xFFFF_FFF0, 0);
    let now1 = state.widen(0xFFFF_FFFE);
    assert_eq!(now1, 0xFFFF_FFFE);
    // Now wrap.
    let now2 = state.widen(0x0000_0010);
    assert_eq!(now2, (1u64 << 32) | 0x10);
    assert!(now2 > now1, "monotonicity broken");
}

#[test]
fn boundary_loop_works_near_u64_max() {
    let tc = u64::from(one_tick_cycles(CLOCK_FREQ));

    let mut engine = Engine::<NoopPa, NoopIs>::new(CLOCK_FREQ);
    let mut queue = SegmentQueue::new();
    let mut pool = CurvePool::new();
    let mut trace = TraceRing::<1024>::new();

    // Construct a segment whose t_start, t_end are near u64::MAX.
    let near_max = u64::MAX - tc * 100;
    let cps = [0.0f32, 0.0, 0.0, 1.0, 0.0, 0.0];
    let knots = [0.0f32, 0.0, 1.0, 1.0];
    let weights = [1.0f32, 1.0];
    pool.load(CurveHandle(0), &cps, &knots, &weights, 1).unwrap();
    queue.try_push(Segment {
        id: 1,
        curve: CurveHandle(0),
        t_start: near_max,
        t_end: near_max + tc * 4,
        kinematics: KinematicTag::CoreXyAndE,
    }).unwrap();

    let r = engine.tick(near_max + tc, &mut queue, &pool, &mut trace);
    assert!(r.is_ok(), "tick near u64::MAX should not panic or fault");
    assert_ne!(engine.status(), RuntimeStatus::Fault);
}
