//! Step-6 §8.5 flush() basic-path tests (Phase 7 Task 7.2).
//!
//! Exercises the foreground flush sequence on a hand-built `RuntimeContext`.
//! The test pre-acks `force_idle` from the test thread before flush() spins,
//! simulating the ISR's §8.5 step-2 short-circuit having fired. flush()
//! then proceeds through queue-drain, slot-reset, and credit_epoch bump.
//!
//! flush_timeout.rs covers the LIVENESS_STALLED path where ack never comes.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::doc_markdown,
    unsafe_code
)]

use core::cell::UnsafeCell;
use core::mem::MaybeUninit;
use core::sync::atomic::Ordering;

use runtime::error::{KALICO_ERR_NULL_PTR, KALICO_OK};
use runtime::state::RuntimeContext;
use runtime::stream;

// flush() imports `kalico_host_now_us` and `irq_save`/`irq_restore` from C.
// On host we provide no-op stubs so the linker resolves.
#[unsafe(no_mangle)]
pub static kalico_clock_freq: u32 = 520_000_000;

// Strictly monotone host-clock counter so the deadline math in flush()
// always advances. The flush_basic tests don't exercise the timeout path,
// but a strict monotone counter is harmless and useful for any future
// "ack arrives just before deadline" scenario.
static HOST_NOW_US: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

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

/// Build a fully initialized `RuntimeContext` on the heap and return a raw
/// pointer the test can pass to `stream::flush`. The Box leaks for the test
/// process lifetime — fine for unit tests.
fn make_runtime_context() -> *mut RuntimeContext {
    let storage: Box<UnsafeCell<MaybeUninit<RuntimeContext>>> =
        Box::new(UnsafeCell::new(MaybeUninit::uninit()));
    let raw = Box::into_raw(storage);
    // SAFETY: caller (test) is single-threaded; init writes through raw
    // pointer projections.
    unsafe {
        let rt_ptr: *mut RuntimeContext = (*(*raw).get()).as_mut_ptr();
        RuntimeContext::init(rt_ptr);
        rt_ptr
    }
}

#[test]
fn flush_null_handle_returns_null_ptr() {
    let mut credit_epoch: u32 = 0;
    let r = unsafe { stream::flush(core::ptr::null_mut(), &raw mut credit_epoch) };
    assert_eq!(r, KALICO_ERR_NULL_PTR);
}

#[test]
fn flush_pre_acked_returns_ok_and_bumps_epoch() {
    let rt = make_runtime_context();
    let shared = unsafe { &*core::ptr::addr_of!((*rt).shared) };

    // Pre-condition: stream_open=true so flush has work to do; pre-ack
    // force_idle from the "ISR" so the spin loop returns immediately.
    shared.stream_open.store(true, Ordering::Release);
    shared.acked_force_idle.store(true, Ordering::Release);

    let epoch_before = shared.credit_epoch.load(Ordering::Acquire);

    let mut out_epoch: u32 = 0;
    let r = unsafe { stream::flush(rt, &raw mut out_epoch) };
    assert_eq!(r, KALICO_OK);

    let epoch_after = shared.credit_epoch.load(Ordering::Acquire);
    assert_eq!(epoch_after, epoch_before.wrapping_add(1));
    assert_eq!(out_epoch, epoch_after);

    // stream_open cleared.
    assert!(!shared.stream_open.load(Ordering::Acquire));
    // force_idle + acked_force_idle cleared (ISR resumes on next tick).
    assert!(!shared.force_idle.load(Ordering::Acquire));
    assert!(!shared.acked_force_idle.load(Ordering::Acquire));
}

#[test]
fn flush_clears_terminal_segment_state() {
    let rt = make_runtime_context();
    let shared = unsafe { &*core::ptr::addr_of!((*rt).shared) };

    shared
        .terminal_segment_id_set
        .store(true, Ordering::Release);
    shared
        .terminal_segment_id_value
        .store(42, Ordering::Release);
    shared
        .accepted_segment_id_seen
        .store(true, Ordering::Release);
    shared.accepted_segment_id.store(99, Ordering::Release);

    shared.acked_force_idle.store(true, Ordering::Release);
    let mut out_epoch: u32 = 0;
    let r = unsafe { stream::flush(rt, &raw mut out_epoch) };
    assert_eq!(r, KALICO_OK);

    assert!(!shared.terminal_segment_id_set.load(Ordering::Acquire));
    assert_eq!(shared.terminal_segment_id_value.load(Ordering::Acquire), 0);
    assert!(!shared.accepted_segment_id_seen.load(Ordering::Acquire));
    assert_eq!(shared.accepted_segment_id.load(Ordering::Acquire), 0);
}

#[test]
fn flush_resets_pool_slots_to_current() {
    let rt = make_runtime_context();
    let shared = unsafe { &*core::ptr::addr_of!((*rt).shared) };
    let pool = unsafe { &*core::ptr::addr_of!((*rt).curve_pool) };

    // Load a couple of curves so current_gen ≠ last_retired_gen.
    let cps = [0.0_f32, 0.0, 0.0, 10.0, 0.0, 0.0];
    let knots = [0.0_f32, 0.0, 1.0, 1.0];
    let weights = [1.0_f32, 1.0];
    let h0 = pool
        .validate_and_load(0, &cps, &knots, &weights, 1)
        .expect("slot 0");
    let h1 = pool
        .validate_and_load(1, &cps, &knots, &weights, 1)
        .expect("slot 1");
    assert_eq!(h0.generation, 1);
    assert_eq!(h1.generation, 1);

    // Pre-flush state: current_gen=1, last_retired_gen=0 for both.
    assert_eq!(pool.slots[0].current_gen.load(Ordering::Acquire), 1);
    assert_eq!(pool.slots[0].last_retired_gen.load(Ordering::Acquire), 0);
    assert_eq!(pool.slots[1].current_gen.load(Ordering::Acquire), 1);

    shared.acked_force_idle.store(true, Ordering::Release);
    let mut out_epoch: u32 = 0;
    let r = unsafe { stream::flush(rt, &raw mut out_epoch) };
    assert_eq!(r, KALICO_OK);

    // Post-flush: every slot's last_retired_gen == current_gen.
    for slot in &pool.slots {
        let cur = slot.current_gen.load(Ordering::Acquire);
        let last = slot.last_retired_gen.load(Ordering::Acquire);
        assert_eq!(cur, last, "slot retired_gen must match current_gen");
    }
}
