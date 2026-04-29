//! Step-6 §13.1 trace-overflow → `KALICO_FAULT_TRACE_OVERFLOW` latch tests.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::indexing_slicing)]

use core::sync::atomic::Ordering;

use runtime::engine::RuntimeStatus;
use runtime::error::{FaultCode, KALICO_ERR_TRACE_OVERFLOW};
use runtime::reclaim::check_trace_overflow_and_fault;
use runtime::state::SharedState;

#[test]
fn no_pending_drop_does_not_latch() {
    let shared = SharedState::new();
    assert!(!check_trace_overflow_and_fault(&shared));
    assert_eq!(shared.last_error.load(Ordering::Acquire), 0);
    assert_eq!(
        shared.runtime_status.load(Ordering::Acquire),
        RuntimeStatus::Idle as u8
    );
}

#[test]
fn pending_drop_latches_fault_once() {
    let shared = SharedState::new();
    shared.sample_drop_pending.store(true, Ordering::Release);
    assert!(check_trace_overflow_and_fault(&shared));
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        KALICO_ERR_TRACE_OVERFLOW
    );
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::TraceOverflow.as_i32()
    );
    assert_eq!(
        shared.runtime_status.load(Ordering::Acquire),
        RuntimeStatus::Fault as u8
    );
    // Calling again with the flag still set should NOT re-latch (last_error
    // is already non-zero); the helper is idempotent on repeated calls.
    assert!(!check_trace_overflow_and_fault(&shared));
}

#[test]
fn pending_drop_does_not_clobber_earlier_fault() {
    let shared = SharedState::new();
    // Pre-existing fault from another path.
    shared
        .last_error
        .store(FaultCode::Underrun.as_i32(), Ordering::Release);
    shared
        .runtime_status
        .store(RuntimeStatus::Fault as u8, Ordering::Release);

    shared.sample_drop_pending.store(true, Ordering::Release);
    // check_trace_overflow_and_fault returns false because last_error was
    // already non-zero; the original fault wins.
    assert!(!check_trace_overflow_and_fault(&shared));
    assert_eq!(
        shared.last_error.load(Ordering::Acquire),
        FaultCode::Underrun.as_i32()
    );
}
