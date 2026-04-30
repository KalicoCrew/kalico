//! Wrap-arithmetic tests. Spec §5.8.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use heapless::spsc::Queue;

use runtime::clock::{WidenState, one_tick_cycles};
use runtime::curve_pool::{CurveHandle, CurvePool};
use runtime::engine::{Engine, RuntimeStatus};
use runtime::queue::Q_N;
use runtime::config::EMode;
use runtime::segment::{KinematicTag, Segment};
use runtime::slot::{NoopIs, NoopPa};
use runtime::state::SharedState;
use runtime::trace::{TRACE_RING_N, TraceSample};

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

    // Heap-leaked SPSC backing for `'static` Producer/Consumer halves —
    // matches the half-split shape used by the real `RuntimeContext`.
    let queue: &'static mut Queue<Segment, Q_N> = Box::leak(Box::new(Queue::new()));
    let (mut q_producer, mut q_consumer) = queue.split();
    let trace: &'static mut Queue<TraceSample, TRACE_RING_N> = Box::leak(Box::new(Queue::new()));
    let (mut t_producer, _t_consumer) = trace.split();

    let pool = CurvePool::new();
    let shared = SharedState::new();
    // Step 7-B: homed gate — set homed=true so the tick reaches the evaluator.
    shared.homed.store(true, core::sync::atomic::Ordering::Release);

    // Pre-seed widen_state to a high-water mark close to u64::MAX so the
    // engine's first widen() doesn't reset to 0.
    let mut widen = WidenState::default();
    let near_max = u64::MAX - tc * 100;
    #[allow(clippy::cast_possible_truncation)]
    let near_max_low = near_max as u32;
    widen.reinit(near_max_low, near_max);

    let cps = [0.0f32, 1.0];
    let knots = [0.0f32, 0.0, 1.0, 1.0];
    let handle = pool
        .validate_and_load(0, 1, &knots, &cps)
        .unwrap();
    let _ = handle; // kept alive in pool; segment carries a copy of the value
    q_producer
        .enqueue(Segment {
            id: 1,
            x_handle: handle,
            y_handle: CurveHandle::UNUSED_SENTINEL,
            z_handle: CurveHandle::UNUSED_SENTINEL,
            e_handle: CurveHandle::UNUSED_SENTINEL,
            t_start: near_max,
            t_end: near_max + tc * 4,
            kinematics: KinematicTag::CoreXyAndE,
            e_mode: EMode::CoupledToXy,
            extrusion_ratio: 0.0,
            flags: 0,
            _pad: [0; 1],
        })
        .unwrap();

    // Engine widens internally; we feed the raw u32 corresponding to
    // (near_max + tc).
    #[allow(clippy::cast_possible_truncation)]
    let raw = (near_max + tc) as u32;
    let r = engine.tick(
        raw,
        &mut widen,
        &pool,
        &mut q_consumer,
        &mut t_producer,
        &shared,
    );
    assert!(r.is_ok(), "tick near u64::MAX should not panic or fault");
    assert_ne!(engine.status(), RuntimeStatus::Fault);
}
