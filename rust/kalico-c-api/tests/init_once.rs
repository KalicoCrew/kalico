//! Init-once invariant tests. Spec §3.2.
//!
//! Both tests share the global `INIT_STATE` static defined in `runtime_ffi`,
//! so order across `#[test]`s is observable. The tests are designed to be
//! order-independent: `null_handle_returns_null_ptr_error` short-circuits on
//! the null-pointer check before touching `INIT_STATE`, so it does not rely
//! on `kalico_runtime_init` having (or not having) run first.

// Test crate has to expose `#[no_mangle]` C-side symbols so the runtime FFI
// links cleanly on host; workspace lints deny `unsafe_code` outside the FFI
// crate, but this is the FFI crate's test harness — opt out at file scope.
#![allow(unsafe_code, non_upper_case_globals)]

// Host-side stubs for the `extern "C"` symbols `runtime_ffi` declares.
// In the MCU build these come from `src/runtime_tick.c` and the H7 timer
// driver; on host we provide them here so the linker resolves cleanly. Only
// `kalico_runtime_init` actually reads `kalico_clock_freq`; the timer hooks
// are not exercised by these two tests but must still link.
#[unsafe(no_mangle)]
pub static kalico_clock_freq: u32 = 520_000_000;

#[unsafe(no_mangle)]
pub extern "C" fn kalico_h7_enable_tim5() {}

#[unsafe(no_mangle)]
pub extern "C" fn kalico_h7_disable_tim5() {}

#[unsafe(no_mangle)]
pub extern "C" fn kalico_h7_read_cyccnt() -> u32 {
    0
}

#[test]
fn second_init_returns_null() {
    // Step-6 Phase 1: kalico_runtime_init is now `extern "C"` (not `unsafe`)
    // — the init guard short-circuits on the second call by returning null,
    // so there's no precondition the caller has to uphold.
    let h1 = kalico_c_api::kalico_runtime_init();
    assert!(!h1.is_null());
    let h2 = kalico_c_api::kalico_runtime_init();
    assert!(h2.is_null(), "second init must return null");
}

#[test]
fn null_handle_returns_null_ptr_error() {
    // Phase 3.3: push_segment now takes (curve_handle_packed: u32, t_start, t_end,
    // kinematics, *mut accepted_segment_id, *mut credit_epoch). Pass nulls for the
    // out-params (they're optional).
    let r = unsafe {
        kalico_c_api::kalico_runtime_push_segment(
            std::ptr::null_mut(),
            0,
            0,
            0,
            100,
            0,
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    };
    assert_eq!(r, kalico_c_api::KALICO_ERR_NULL_PTR);
}
