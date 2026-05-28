//! Init-once invariant tests. Spec §3.2.
//!
//! Both tests share the global `INIT_STATE` static defined in `runtime_ffi`,
//! so order across `#[test]`s is observable. The tests are designed to be
//! order-independent: `null_handle_returns_null_ptr_error` short-circuits on
//! the null-pointer check before touching `INIT_STATE`, so it does not rely
//! on `runtime_handle_create` having (or not having) run first.

// Test crate has to expose `#[no_mangle]` C-side symbols so the runtime FFI
// links cleanly on host; workspace lints deny `unsafe_code` outside the FFI
// crate, but this is the FFI crate's test harness — opt out at file scope.
#![allow(unsafe_code, non_upper_case_globals)]

// Host-side stubs for the `extern "C"` symbols `runtime_ffi` declares.
// In the MCU build these come from `src/runtime_tick.c` and the H7 timer
// driver; on host we provide them here so the linker resolves cleanly. Only
// `runtime_handle_create` actually reads `runtime_clock_freq`; the timer hooks
// are not exercised by these two tests but must still link.
#[unsafe(no_mangle)]
pub static runtime_clock_freq: u32 = 520_000_000;

#[unsafe(no_mangle)]
pub static runtime_sample_rate_hz: u32 = 40_000;

#[unsafe(no_mangle)]
pub extern "C" fn runtime_tick_enable() {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_tick_disable() {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_cyccnt_read() -> u32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_reset_stepper_bindings() {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_diag_progress(_tag: u32, _stage: u32, _value: u32) {}

#[test]
fn second_init_returns_null() {
    // Step-6 Phase 1: runtime_handle_create is now `extern "C"` (not `unsafe`)
    // — the init guard short-circuits on the second call by returning null,
    // so there's no precondition the caller has to uphold.
    let h1 = kalico_c_api::runtime_handle_create();
    assert!(!h1.is_null());
    let h2 = kalico_c_api::runtime_handle_create();
    assert!(h2.is_null(), "second init must return null");
}

#[test]
fn null_handle_returns_null_ptr_error() {
    // Step 7-B: push_segment now takes 4 per-axis handles (x, y, z, e),
    // e_mode, extrusion_ratio_bits, plus t_start, t_end, kinematics.
    // Pass nulls for the out-params (they're optional).
    let r = unsafe {
        kalico_c_api::runtime_handle_push_segment(
            std::ptr::null_mut(),
            0,   // id
            0,   // x_handle_packed
            0,   // y_handle_packed
            0,   // z_handle_packed
            0,   // e_handle_packed
            0,   // t_start
            100, // t_end
            0,   // kinematics
            0,   // e_mode
            0,   // extrusion_ratio_bits
            std::ptr::null_mut(),
            std::ptr::null_mut(),
        )
    };
    assert_eq!(r, kalico_c_api::KALICO_ERR_NULL_PTR);
}
