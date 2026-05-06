//! Closure-review regression test for finding #1 (HIGH, SHIP-BLOCKER):
//! `runtime_handle_drain_trace` must signal whether it consumed any
//! `TRACE_FLAG_SEGMENT_END` samples, so the C-side `runtime_drain` can
//! emit `kalico_credit_freed` even when the trace leg consumes the
//! `SEGMENT_END` before the reclaim leg sees it.
//!
//! Bug shape: prior to this fix, the trace-drain leg silently consumed
//! `SEGMENT_END` (calling `pool.confirm_retired` correctly, but reporting
//! nothing back), and the reclaim leg's saw-segment-end bit gated the
//! credit emission. Under steady-state push the trace leg always wins,
//! so the host's credit counter drains to zero and flow control
//! deadlocks.

#![allow(
    unsafe_code,
    non_upper_case_globals,
    clippy::borrow_as_ptr,
    clippy::ref_as_ptr
)]

use runtime::trace::{TRACE_FLAG_SEGMENT_END, TraceSample};

#[unsafe(no_mangle)]
pub static runtime_clock_freq: u32 = 520_000_000;

#[unsafe(no_mangle)]
pub extern "C" fn runtime_tick_enable() {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_tick_disable() {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_cyccnt_read() -> u32 {
    0
}

/// End-to-end smoke: init the runtime, push a `SEGMENT_END` trace sample
/// into the ISR-side trace producer (via the published `RT_CELL` pointer),
/// drain through `runtime_handle_drain_trace` with a non-null
/// `out_saw_segment_end`, and assert the bit is set + the sample is
/// reported.
///
/// SAFETY notes inline. The test runs as its own integration-test
/// binary so it has exclusive ownership of the runtime singleton; no
/// other test in this binary calls `runtime_handle_create`.
#[test]
fn drain_trace_reports_segment_end_to_caller() {
    use core::cell::UnsafeCell;
    use runtime::curve_pool::CurveHandle;
    use runtime::state::{IsrState, RuntimeContext};

    let rt = kalico_c_api::runtime_handle_create();
    assert!(!rt.is_null(), "runtime_handle_create returned null");

    // Inject a SEGMENT_END sample directly into the ISR trace producer.
    // This mirrors what the engine does on segment retirement, but
    // bypasses the engine entirely — we're testing the drain shim's
    // bookkeeping, not the producer.
    //
    // SAFETY: the test owns the runtime singleton and there is no
    // concurrent ISR on the host; we project to IsrState's trace
    // producer the same way the FFI shim does in tick().
    let handle = CurveHandle::new(0, 1);
    let sample = TraceSample {
        tick: 1234,
        motor_a: 0.0,
        motor_b: 0.0,
        motor_z: 0.0,
        motor_e: 0.0,
        segment_id: 7,
        curve_handle: handle,
        flags: TRACE_FLAG_SEGMENT_END,
        _pad: [0; 7],
    };
    unsafe {
        let ctx = rt.cast::<RuntimeContext>();
        let isr_ptr: *mut IsrState = UnsafeCell::raw_get(core::ptr::addr_of!((*ctx).isr));
        let isr: &mut IsrState = &mut *isr_ptr;
        isr.trace_producer
            .enqueue(sample)
            .expect("trace producer enqueue");
    }

    // Drain with the new out-param — pre-fix, the Rust drain quietly
    // consumed SEGMENT_END and reported only the count, leaving the C
    // caller without a way to know that a credit event should fire.
    let mut out_buf = [TraceSample {
        tick: 0,
        motor_a: 0.0,
        motor_b: 0.0,
        motor_z: 0.0,
        motor_e: 0.0,
        segment_id: 0,
        curve_handle: CurveHandle::new(0, 0),
        flags: 0,
        _pad: [0; 7],
    }; 4];
    let mut saw_segment_end: u8 = 0;
    let n = unsafe {
        kalico_c_api::runtime_handle_drain_trace(
            rt,
            out_buf.as_mut_ptr(),
            4,
            &mut saw_segment_end as *mut u8,
        )
    };
    assert_eq!(n, 1, "drain returned wrong count");
    assert_eq!(
        saw_segment_end, 1,
        "drain_trace MUST set saw_segment_end=1 when a SEGMENT_END sample \
         was consumed (closure-review fix for credit-emission deadlock)"
    );
    assert_eq!(out_buf[0].segment_id, 7);
    assert_eq!(out_buf[0].flags, TRACE_FLAG_SEGMENT_END);

    // Empty-drain afterwards: bit must clear.
    saw_segment_end = 0xFF;
    let n2 = unsafe {
        kalico_c_api::runtime_handle_drain_trace(
            rt,
            out_buf.as_mut_ptr(),
            4,
            &mut saw_segment_end as *mut u8,
        )
    };
    assert_eq!(n2, 0);
    assert_eq!(
        saw_segment_end, 0,
        "drain_trace must clear saw_segment_end on empty drain"
    );

    // Null out-param is permitted (caller may not care about the bit).
    let n3 = unsafe {
        kalico_c_api::runtime_handle_drain_trace(rt, out_buf.as_mut_ptr(), 4, core::ptr::null_mut())
    };
    assert_eq!(n3, 0);
}
