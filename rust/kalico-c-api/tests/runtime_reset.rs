#![allow(unsafe_code, non_upper_case_globals)]

use std::sync::{Mutex, OnceLock};

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

#[test]
fn reset_reclaims_allocation_across_many_configures() {
    let _g = TEST_LOCK.lock().unwrap();
    let handle = rt();
    for i in 0..128 {
        unsafe {
            let rc = kalico_c_api::kalico_runtime_reset(handle);
            assert_eq!(rc, kalico_c_api::KALICO_OK, "reset failed at iter {i}");
            let rc = kalico_c_api::kalico_runtime_configure_axis(
                handle,
                0,
                0,
                (1.0_f32 / 160.0_f32).to_bits(),
                64,
                core::ptr::null(),
                0,
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
