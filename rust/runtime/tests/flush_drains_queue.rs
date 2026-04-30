//! Step-6 §8.5 flush() queue-drain test (Phase 7 Task 7.2 step 4).
//!
//! Verifies that flush()'s IRQ-disable + queue-drain step removes any
//! enqueued segments from the ISR-side `IsrState.queue_consumer`. Pre-loads
//! the queue with several segments via the foreground `FgState.queue_producer`,
//! then runs flush() and asserts the queue is empty afterward.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::doc_markdown,
    unsafe_code
)]

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::{AtomicU64, Ordering};

use runtime::curve_pool::CurveHandle;
use runtime::error::KALICO_OK;
use runtime::config::EMode;
use runtime::segment::{KinematicTag, Segment};
use runtime::state::{FgState, IsrState, RuntimeContext};
use runtime::stream;

#[unsafe(no_mangle)]
pub static kalico_clock_freq: u32 = 520_000_000;

static HOST_NOW_US: AtomicU64 = AtomicU64::new(0);

#[unsafe(no_mangle)]
pub extern "C" fn kalico_host_now_us() -> u64 {
    HOST_NOW_US.fetch_add(1, Ordering::Relaxed)
}

#[unsafe(no_mangle)]
pub extern "C" fn kalico_irq_save() -> u32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn kalico_irq_restore(_flags: u32) {}

fn make_runtime_context() -> *mut RuntimeContext {
    let storage: Box<UnsafeCell<MaybeUninit<RuntimeContext>>> =
        Box::new(UnsafeCell::new(MaybeUninit::uninit()));
    let raw = Box::into_raw(storage);
    unsafe {
        let rt_ptr: *mut RuntimeContext = (*(*raw).get()).as_mut_ptr();
        RuntimeContext::init(rt_ptr);
        rt_ptr
    }
}

#[test]
fn flush_drains_pending_segments() {
    let rt = make_runtime_context();
    let shared = unsafe { &*core::ptr::addr_of!((*rt).shared) };
    // SAFETY: caller is single-threaded test; we form the FgState borrow,
    // enqueue, drop it before calling flush.
    {
        let fg: &mut FgState = unsafe { &mut *UnsafeCell::raw_get(core::ptr::addr_of!((*rt).fg)) };
        for i in 1..=4 {
            fg.queue_producer
                .enqueue(Segment {
                    id: i,
                    x_handle: CurveHandle::new(0, 1),
                    y_handle: CurveHandle::UNUSED_SENTINEL,
                    z_handle: CurveHandle::UNUSED_SENTINEL,
                    e_handle: CurveHandle::UNUSED_SENTINEL,
                    t_start: 0,
                    t_end: 1_000_000,
                    kinematics: KinematicTag::CoreXyAndE,
                    e_mode: EMode::CoupledToXy,
                    extrusion_ratio: 0.0,
                    flags: 0,
                    _pad: [0; 1],
                })
                .unwrap();
        }
    }

    // Pre-ack so flush proceeds.
    shared.acked_force_idle.store(true, Ordering::Release);
    let mut out_epoch: u32 = 0;
    let r = unsafe { stream::flush(rt, &raw mut out_epoch) };
    assert_eq!(r, KALICO_OK);

    // Verify queue is empty post-flush.
    // SAFETY: same single-threaded test invariant; flush returned, so we
    // can transiently project to IsrState.
    let drained: bool = unsafe {
        let isr: &mut IsrState = &mut *UnsafeCell::raw_get(core::ptr::addr_of!((*rt).isr));
        !isr.queue_consumer.ready()
    };
    assert!(drained, "queue should be empty after flush");
}
