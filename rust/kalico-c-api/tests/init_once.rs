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
pub extern "C" fn runtime_cyccnt_read() -> u32 {
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_diag_progress(_tag: u32, _stage: u32, _value: u32) {}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_widened_host_clock() -> u64 {
    0
}
#[unsafe(no_mangle)]
pub extern "C" fn runtime_host_now_us() -> u64 {
    0
}
#[unsafe(no_mangle)]
pub extern "C" fn runtime_irq_save() -> u32 {
    0
}
#[unsafe(no_mangle)]
pub extern "C" fn runtime_irq_restore(_flags: u32) {}
#[unsafe(no_mangle)]
pub static stats_send_time: u32 = 0;
#[unsafe(no_mangle)]
pub static stats_send_time_high: u32 = 0;
#[unsafe(no_mangle)]
pub extern "C" fn timer_read_time() -> u32 {
    0
}
#[unsafe(no_mangle)]
pub extern "C" fn timer_is_before(_a: u32, _b: u32) -> u8 {
    0
}
#[unsafe(no_mangle)]
pub extern "C" fn runtime_emit_step_pulses(_axis_idx: u8, _n_steps: i32) {}

#[test]
fn second_init_returns_null() {
    let h1 = kalico_c_api::runtime_handle_create();
    assert!(!h1.is_null());
    let h2 = kalico_c_api::runtime_handle_create();
    assert!(h2.is_null(), "second init must return null");
}

#[test]
fn null_handle_returns_null_ptr_error() {
    let piece = [0u8; 32];
    let r = unsafe {
        kalico_c_api::kalico_runtime_write_piece(
            std::ptr::null_mut(),
            0,
            0,
            0,
            piece.as_ptr(),
        )
    };
    assert_eq!(r, kalico_c_api::KALICO_ERR_NULL_PTR);
}
