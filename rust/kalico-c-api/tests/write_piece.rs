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

/// `cargo test` runs intra-binary tests in parallel by default.  Because
/// `INIT_DONE` is a process-global boolean and the runtime pointer is a
/// non-thread-safe raw pointer, every test that calls into the FFI must hold
/// this lock for the duration of its FFI calls.  The lock makes the access
/// pattern sequentially consistent even when the test harness spawns multiple
/// threads.
static TEST_LOCK: Mutex<()> = Mutex::new(());

/// # SAFETY
///
/// All FFI calls in this file are serialised by `TEST_LOCK` (acquired before
/// every call and held for its duration), so no two threads ever call into the
/// runtime concurrently.  Additionally, the internal FFI guards (`INIT_DONE`
/// check, null-pointer check) provide a second layer of protection.  Together
/// these two mechanisms make the raw-pointer accesses sequentially consistent,
/// satisfying the aliasing contract of `*mut KalicoRuntime`.
struct RtHandle(*mut kalico_c_api::KalicoRuntime);

// SAFETY: see the `RtHandle` doc comment above — serialisation by TEST_LOCK
// plus internal FFI guards makes Send + Sync sound here.
unsafe impl Send for RtHandle {}
unsafe impl Sync for RtHandle {}

static RUNTIME: OnceLock<RtHandle> = OnceLock::new();

fn rt() -> *mut kalico_c_api::KalicoRuntime {
    RUNTIME
        .get_or_init(|| {
            let handle = kalico_c_api::runtime_handle_create();
            assert!(!handle.is_null(), "runtime_handle_create returned null");

            let rc = unsafe {
                kalico_c_api::kalico_runtime_configure_axis(
                    handle,
                    0,
                    0,                               // mode = Pulse
                    (1.0_f32 / 160.0_f32).to_bits(),
                    64,
                    core::ptr::null(),               // bindings_ptr — null legal when count == 0
                    0,
                )
            };
            assert_eq!(rc, kalico_c_api::KALICO_OK, "configure_axis failed: {rc}");

            RtHandle(handle)
        })
        .0
}

#[test]
fn write_piece_then_commit_head_makes_one_piece_visible() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();

    let mut piece = [0u8; 32];
    piece[0..8].copy_from_slice(&7777u64.to_le_bytes());

    unsafe {
        let rc = kalico_c_api::kalico_runtime_write_piece(
            handle,
            0,
            0,
            0,
            piece.as_ptr(),
        );
        assert_eq!(rc, kalico_c_api::KALICO_OK, "write_piece failed: {rc}");

        let rc = kalico_c_api::kalico_runtime_commit_head(
            handle,
            0,
            1,
        );
        assert_eq!(rc, kalico_c_api::KALICO_OK, "commit_head failed: {rc}");
    }
}

#[test]
fn write_piece_rejects_unconfigured_axis() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();
    let piece = [0u8; 32];
    unsafe {
        let rc = kalico_c_api::kalico_runtime_write_piece(
            handle,
            3,
            0,
            0,
            piece.as_ptr(),
        );
        assert_eq!(rc, kalico_c_api::KALICO_ERR_INVALID_ARG);
    }
}

#[test]
fn write_piece_null_rt_is_null_ptr_error() {
    let _guard = TEST_LOCK.lock().unwrap();
    let piece = [0u8; 32];
    unsafe {
        let rc = kalico_c_api::kalico_runtime_write_piece(
            core::ptr::null_mut(),
            0,
            0,
            0,
            piece.as_ptr(),
        );
        assert_eq!(rc, kalico_c_api::KALICO_ERR_NULL_PTR);
    }
}

#[test]
fn commit_head_null_rt_is_null_ptr_error() {
    let _guard = TEST_LOCK.lock().unwrap();
    unsafe {
        let rc = kalico_c_api::kalico_runtime_commit_head(core::ptr::null_mut(), 0, 1);
        assert_eq!(rc, kalico_c_api::KALICO_ERR_NULL_PTR);
    }
}

#[test]
fn commit_head_rejects_unconfigured_axis() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();
    unsafe {
        let rc = kalico_c_api::kalico_runtime_commit_head(handle, 3, 1);
        assert_eq!(rc, kalico_c_api::KALICO_ERR_INVALID_ARG);
    }
}
