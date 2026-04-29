//! Step-6 §8.5 flush() timeout path test (Phase 7 Task 7.2).
//!
//! With a stuck "ISR" (no `acked_force_idle` ever set), flush() must spin
//! at most 1 ms wall-clock and return `KALICO_ERR_LIVENESS_STALLED`. We
//! drive the host-clock counter manually so the deadline check fires in
//! deterministic time.
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

use runtime::engine::RuntimeStatus;
use runtime::error::{FaultCode, KALICO_ERR_LIVENESS_STALLED};
use runtime::state::RuntimeContext;
use runtime::stream;

#[unsafe(no_mangle)]
pub static kalico_clock_freq: u32 = 520_000_000;

// Each call to `kalico_host_now_us` advances the counter by 100 µs. With
// the 1 ms (1000 µs) deadline, the loop fires the timeout after ~10 calls.
static HOST_NOW_US: AtomicU64 = AtomicU64::new(0);
const HOST_TICK_INCREMENT_US: u64 = 100;

#[unsafe(no_mangle)]
pub extern "C" fn kalico_host_now_us() -> u64 {
    HOST_NOW_US.fetch_add(HOST_TICK_INCREMENT_US, Ordering::Relaxed)
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
fn flush_timeout_yields_liveness_stalled() {
    let rt = make_runtime_context();
    let shared = unsafe { &*core::ptr::addr_of!((*rt).shared) };

    // No ISR ack ever — flush()'s spin loop must time out at the 1 ms
    // boundary set against `kalico_host_now_us`.
    HOST_NOW_US.store(0, Ordering::Relaxed);

    let mut out_epoch: u32 = 0;
    let r = unsafe { stream::flush(rt, &mut out_epoch) };

    assert_eq!(r, KALICO_ERR_LIVENESS_STALLED);
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::LivenessStalled.as_i32()
    );
    assert_eq!(
        shared.runtime_status.load(Ordering::Acquire),
        RuntimeStatus::Fault as u8
    );
    // force_idle cleared (best-effort cleanup so a stuck ISR isn't pinned
    // permanently in the short-circuit path).
    assert!(!shared.force_idle.load(Ordering::Acquire));
}
