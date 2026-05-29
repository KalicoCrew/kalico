//! FFI tests for `kalico_runtime_write_piece` and `kalico_runtime_commit_head`.
//!
//! These replace the old `kalico_runtime_push_pieces` tests at the FFI seam
//! (Task 4).
//!
//! All tests in this file share one `*mut KalicoRuntime` through a
//! `OnceLock`-guarded helper, because `INIT_DONE` is a process-global boolean
//! and `runtime_handle_create` returns null on any subsequent call. The tests
//! that need the handle call `rt()` which initialises on first use; the
//! null-pointer tests pass `core::ptr::null_mut()` explicitly and never touch
//! `INIT_DONE`.

#![allow(unsafe_code, non_upper_case_globals)]

use std::sync::{Mutex, OnceLock};

// ---------------------------------------------------------------------------
// Host-side linker stubs — same as init_once.rs. Each test binary is
// independently linked, so these must appear here too.
// ---------------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
// Handle setup
// ---------------------------------------------------------------------------

/// Serialises all test bodies that touch the shared runtime.
///
/// `cargo test` runs intra-binary tests in parallel by default.  Because
/// `INIT_DONE` is a process-global boolean and the runtime pointer is a
/// non-thread-safe raw pointer, every test that calls into the FFI must hold
/// this lock for the duration of its FFI calls.  The lock makes the access
/// pattern sequentially consistent even when the test harness spawns multiple
/// threads.
static TEST_LOCK: Mutex<()> = Mutex::new(());

/// Wraps `*mut KalicoRuntime` so it can live in a `OnceLock`.
///
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

/// Returns the process-global runtime handle, initialising it on first call.
/// Also calls `kalico_runtime_configure_axis` for axis 0 (ring_depth 64,
/// Pulse mode, no stepper bindings) on the same first call so every test that
/// needs a configured axis sees it without repeating setup.
fn rt() -> *mut kalico_c_api::KalicoRuntime {
    RUNTIME
        .get_or_init(|| {
            let handle = kalico_c_api::runtime_handle_create();
            assert!(!handle.is_null(), "runtime_handle_create returned null");

            // Configure axis 0 once for all tests that need a configured axis.
            let rc = unsafe {
                kalico_c_api::kalico_runtime_configure_axis(
                    handle,
                    0,                                // axis_idx
                    0,                                // mode = Pulse
                    (1.0_f32 / 160.0_f32).to_bits(), // microstep_distance_f32_bits
                    64,                               // ring_depth
                    core::ptr::null(),                // bindings_ptr — null legal when count == 0
                    0,                                // stepper_count
                )
            };
            assert_eq!(rc, kalico_c_api::KALICO_OK, "configure_axis failed: {rc}");

            RtHandle(handle)
        })
        .0
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[test]
fn write_piece_then_commit_head_makes_one_piece_visible() {
    // Structural ring-state visibility (head advance) is verified at the
    // runtime-ring layer in Task 3 unit tests.  No ring-state query FFI exists
    // at this boundary, so this test exercises only the FFI return-path:
    // both calls must return KALICO_OK with valid inputs.
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();

    // Build a 32-byte PieceEntry with a recognisable start_time sentinel.
    let mut piece = [0u8; 32];
    piece[0..8].copy_from_slice(&7777u64.to_le_bytes());

    unsafe {
        let rc = kalico_c_api::kalico_runtime_write_piece(
            handle,
            0,              // axis_idx
            0,              // start_slot
            0,              // index
            piece.as_ptr(),
        );
        assert_eq!(rc, kalico_c_api::KALICO_OK, "write_piece failed: {rc}");

        let rc = kalico_c_api::kalico_runtime_commit_head(
            handle,
            0,  // axis_idx
            1,  // new_head — advance from 0 to 1
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
            3,              // axis_idx 3 — never configured
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
