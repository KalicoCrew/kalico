//! FFI tests for `kalico_runtime_reset`.
//!
//! Each integration-test file is its own binary, so `INIT_DONE` (a
//! process-global) is fresh here. The shared handle + TEST_LOCK pattern mirrors
//! `write_piece.rs`.

#![allow(unsafe_code, non_upper_case_globals)]

use std::sync::{Mutex, OnceLock};

// --- Host-side linker stubs (each test binary links independently) ----------
#[unsafe(no_mangle)]
pub static runtime_clock_freq: u32 = 520_000_000;
#[unsafe(no_mangle)]
pub static runtime_sample_rate_hz: u32 = 40_000;
#[unsafe(no_mangle)]
pub extern "C" fn runtime_cyccnt_read() -> u32 { 0 }
#[unsafe(no_mangle)]
pub extern "C" fn runtime_diag_progress(_tag: u32, _stage: u32, _value: u32) {}

// --- Handle setup -----------------------------------------------------------
static TEST_LOCK: Mutex<()> = Mutex::new(());

struct RtHandle(*mut kalico_c_api::KalicoRuntime);
// SAFETY: all FFI calls are serialised by TEST_LOCK; the FFI's own INIT_DONE +
// null-ptr guards add a second layer. See write_piece.rs for the full rationale.
unsafe impl Send for RtHandle {}
unsafe impl Sync for RtHandle {}

static RUNTIME: OnceLock<RtHandle> = OnceLock::new();

fn rt() -> *mut kalico_c_api::KalicoRuntime {
    RUNTIME
        .get_or_init(|| {
            let handle = kalico_c_api::runtime_handle_create();
            assert!(!handle.is_null(), "runtime_handle_create returned null");
            RtHandle(handle)
        })
        .0
}

// --- Tests ------------------------------------------------------------------

#[test]
fn reset_reclaims_allocation_across_many_configures() {
    let _g = TEST_LOCK.lock().unwrap();
    let handle = rt();
    // Without a working reset, repeated allocation exhausts the bump allocator
    // (total pool is ~1984 pieces). With reset before each configure, all 128
    // iterations (128*64 = 8192 pieces of demand) succeed.
    for i in 0..128 {
        unsafe {
            let rc = kalico_c_api::kalico_runtime_reset(handle);
            assert_eq!(rc, kalico_c_api::KALICO_OK, "reset failed at iter {i}");
            let rc = kalico_c_api::kalico_runtime_configure_axis(
                handle,
                0,                               // axis_idx
                0,                               // mode = Pulse
                (1.0_f32 / 160.0_f32).to_bits(), // microstep_distance bits
                64,                              // ring_depth
                core::ptr::null(),               // bindings_ptr
                0,                               // stepper_count
            );
            assert_eq!(rc, kalico_c_api::KALICO_OK, "configure failed at iter {i}");
        }
    }
}

#[test]
fn reset_null_rt_is_null_ptr_error() {
    let _g = TEST_LOCK.lock().unwrap();
    unsafe {
        let rc = kalico_c_api::kalico_runtime_reset(core::ptr::null_mut());
        assert_eq!(rc, kalico_c_api::KALICO_ERR_NULL_PTR);
    }
}
