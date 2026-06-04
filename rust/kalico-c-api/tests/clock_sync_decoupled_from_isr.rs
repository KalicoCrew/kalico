//! Regression test for the 2026-05-11 silent-retire bug.
//!
//! The bench reproduction: two sequential jogs ~1.5 s apart. First moves;
//! second silently dropped, `engine_status=Drained`, `current_segment_id`
//! advanced, no fault raised, no step pulses fired. Root cause: on the H7
//! we disable TIM5 on Drained / Fault (`src/runtime_tick.c:326-329`), which
//! stops `Engine::tick` from running `publish_widened_now`, freezing the
//! §11.4 widened-now seqlock at whatever value the last ISR tick wrote.
//! The old `clock_sync_respond` read that seqlock and returned the frozen
//! value, so the bridge's clock-sync regression flatlined and the next
//! jog's `t_start_clock` landed in the MCU's real past — boundary loop
//! retired without producing pulses.
//!
//! Fix: clock_sync_respond reads `runtime_widened_host_clock` (defined in
//! `src/runtime_tick.c`), which widens DWT using Klipper's
//! `stats_send_time_high` lookback — independent of TIM5 state.
//!
//! This test pins the architectural invariant: the value returned to the
//! host for clock-sync comes from the C-side `runtime_widened_host_clock`
//! symbol, NOT from the ISR-published seqlock atomics. We stub the symbol
//! to a sentinel value and verify that the FFI surfaces it unchanged.

#![allow(unsafe_code, non_upper_case_globals)]

use std::sync::atomic::{AtomicU64, Ordering};

// Host-side stubs. Same set as init_once.rs plus the new one we're testing.
#[unsafe(no_mangle)]
pub static runtime_clock_freq: u32 = 520_000_000;

#[unsafe(no_mangle)]
pub static runtime_sample_rate_hz: u32 = 40_000;

#[unsafe(no_mangle)]
pub extern "C" fn runtime_cyccnt_read() -> u32 {
    0
}

/// Test-controlled value the new `runtime_widened_host_clock` shim returns.
/// Distinct from any value the §11.4 seqlock would produce: a 32-bit raw
/// CYCCNT widens to at most `0x0000_0000_FFFF_FFFF` on the first wrap, so
/// `0xDEAD_BEEF_CAFE_BABE` cannot have come from the seqlock path.
static STUB_MCU_CLOCK: AtomicU64 = AtomicU64::new(0xDEAD_BEEF_CAFE_BABE);

#[unsafe(no_mangle)]
pub extern "C" fn runtime_widened_host_clock() -> u64 {
    STUB_MCU_CLOCK.load(Ordering::Relaxed)
}

#[unsafe(no_mangle)]
pub extern "C" fn runtime_diag_progress(_tag: u32, _stage: u32, _value: u32) {}

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
fn clock_sync_returns_widened_host_clock_not_seqlock() {
    // Spin up the runtime. `runtime_handle_create` runs the half-split
    // init; clock-sync is part of the foreground surface, no segment
    // dispatch needed.
    let rt = kalico_c_api::runtime_handle_create();
    assert!(!rt.is_null());

    // Issue a clock-sync request. The returned `mcu_clock` must be the
    // inline-widened value (timer_read_time + stats_send_time_high), NOT
    // the §11.4 seqlock (which is zero at this point because no ISR tick
    // has run on this host build). With all stubs returning 0, the
    // widened clock is 0.
    let mut mcu_clock: u64 = 0;
    let r = unsafe {
        kalico_c_api::kalico_runtime_clock_sync_request(
            rt,
            42, // request_id (echoed; we don't care here)
            0,  // host_send_time_lo (foreground unused)
            0,  // host_send_time_hi (foreground unused)
            &mut mcu_clock,
        )
    };
    assert_eq!(r, 0, "clock_sync_request returned non-OK: {r}");
    assert_eq!(
        mcu_clock, 0,
        "clock_sync returned unexpected clock value — expected the inline \
         timer_read_time + stats_send_time_high widening (0 with host stubs)."
    );

    // Second call — still returns the inline-widened value (0 with stubs).
    let r2 =
        unsafe { kalico_c_api::kalico_runtime_clock_sync_request(rt, 43, 0, 0, &mut mcu_clock) };
    assert_eq!(r2, 0);
    assert_eq!(mcu_clock, 0);
}
