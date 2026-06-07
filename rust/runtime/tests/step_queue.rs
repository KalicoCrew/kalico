#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]
#![allow(unsafe_code)]

use std::cell::UnsafeCell;

use runtime::step_queue::{STEP_QUEUE_DEPTH, StepEntry, StepQueue, StepQueueFull, len, pop, push};

fn entry(cycle_abs: u32, dir: i8) -> StepEntry {
    StepEntry {
        cycle_abs,
        dir,
        _pad: [0; 3],
    }
}

#[test]
fn fifo_order_under_random_push_pop() {
    let q = UnsafeCell::new(StepQueue::new());
    let qp = q.get();

    for i in 0..30u32 {
        let r = unsafe { push(qp, entry(1000 + i, if i % 2 == 0 { 1 } else { -1 })) };
        assert!(r.is_ok(), "push {i} should succeed");
    }
    assert_eq!(unsafe { len(qp) }, 30);

    for i in 0..30u32 {
        let got = unsafe { pop(qp) }.expect("pop should yield entry");
        assert_eq!(got.cycle_abs, 1000 + i, "FIFO order broken at {i}");
        assert_eq!(got.dir, if i % 2 == 0 { 1 } else { -1 });
    }
    assert_eq!(unsafe { len(qp) }, 0);
    assert!(unsafe { pop(qp) }.is_none());
}

#[test]
fn overflow_detected_at_full_capacity() {
    let q = UnsafeCell::new(StepQueue::new());
    let qp = q.get();

    for i in 0..STEP_QUEUE_DEPTH as u32 {
        let r = unsafe { push(qp, entry(i, 1)) };
        assert!(r.is_ok(), "push {i} (within capacity) should succeed");
    }
    assert_eq!(unsafe { len(qp) }, STEP_QUEUE_DEPTH as u16);

    let overflow = unsafe { push(qp, entry(9999, 1)) };
    assert_eq!(overflow, Err(StepQueueFull));
    assert_eq!(unsafe { len(qp) }, STEP_QUEUE_DEPTH as u16);

    for i in 0..STEP_QUEUE_DEPTH as u32 {
        let got = unsafe { pop(qp) }.expect("pop should yield entry");
        assert_eq!(
            got.cycle_abs, i,
            "FIFO contents corrupted by overflow at {i}"
        );
    }
}

#[test]
fn wraparound_u16_counters_correct() {
    let q = UnsafeCell::new(StepQueue::new());
    let qp = q.get();

    for round in 0..3u32 {
        for i in 0..25u32 {
            let v = round * 25 + i;
            let r = unsafe { push(qp, entry(v, 1)) };
            assert!(r.is_ok(), "push round={round} i={i} should succeed");
        }
        assert_eq!(unsafe { len(qp) }, 25);

        for i in 0..25u32 {
            let v = round * 25 + i;
            let got = unsafe { pop(qp) }.expect("pop should yield entry");
            assert_eq!(
                got.cycle_abs, v,
                "wraparound corrupted ordering at round={round} i={i}"
            );
        }
        assert_eq!(unsafe { len(qp) }, 0);
    }

    assert_eq!(unsafe { len(qp) }, 0);
}

#[test]
fn empty_pop_returns_none() {
    let q = UnsafeCell::new(StepQueue::new());
    let qp = q.get();
    assert!(unsafe { pop(qp) }.is_none());
    assert_eq!(unsafe { len(qp) }, 0);
}
