//! The homing Stop gate: after `kalico_runtime_gate_pieces`, head commits are
//! refused (`KALICO_ERR_STREAM_HALTED`) so a piece frame racing the Stop
//! discard can never publish into the ring and execute from the halted
//! position (bench 2026-06-10: -310, 740 steps — one drip window at 100mm/s).
//! `kalico_runtime_ungate_pieces` lifts the gate; lifting an open gate is a
//! host sequencing bug and fails loudly.
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

/// # SAFETY
///
/// All FFI calls in this file are serialised by `TEST_LOCK`; see
/// `write_piece.rs` for the full aliasing argument.
struct RtHandle(*mut kalico_c_api::KalicoRuntime);

// SAFETY: serialisation by TEST_LOCK plus internal FFI guards.
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
                    0,
                    (1.0_f32 / 160.0_f32).to_bits(),
                    64,
                    core::ptr::null(),
                    0,
                )
            };
            assert_eq!(rc, kalico_c_api::KALICO_OK, "configure_axis failed: {rc}");
            RtHandle(handle)
        })
        .0
}

fn write_one_piece(handle: *mut kalico_c_api::KalicoRuntime, start_slot: u16) {
    let mut piece = [0u8; 32];
    piece[0..8].copy_from_slice(&7777u64.to_le_bytes());
    let rc = unsafe {
        kalico_c_api::kalico_runtime_write_piece(handle, 0, start_slot, 0, piece.as_ptr())
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "write_piece failed: {rc}");
}

#[test]
fn gate_refuses_commit_then_ungate_allows_it() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();
    unsafe {
        let rc = kalico_c_api::kalico_runtime_gate_pieces(handle);
        assert_eq!(rc, kalico_c_api::KALICO_OK, "gate_pieces failed: {rc}");

        write_one_piece(handle, 0);
        let rc = kalico_c_api::kalico_runtime_commit_head(handle, 0, 1);
        assert_eq!(
            rc,
            kalico_c_api::KALICO_ERR_STREAM_HALTED,
            "commit while gated must be refused"
        );

        let rc = kalico_c_api::kalico_runtime_ungate_pieces(handle);
        assert_eq!(rc, kalico_c_api::KALICO_OK, "ungate_pieces failed: {rc}");

        write_one_piece(handle, 1);
        let rc = kalico_c_api::kalico_runtime_commit_head(handle, 0, 2);
        assert_eq!(rc, kalico_c_api::KALICO_OK, "commit after ungate: {rc}");
    }
}

#[test]
fn ungate_without_gate_is_a_state_violation() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();
    unsafe {
        let rc = kalico_c_api::kalico_runtime_ungate_pieces(handle);
        assert_eq!(rc, kalico_c_api::KALICO_ERR_STREAM_STATE_VIOLATION);
    }
}

#[test]
fn gate_is_idempotent_like_a_repeated_stop() {
    let _guard = TEST_LOCK.lock().unwrap();
    let handle = rt();
    unsafe {
        assert_eq!(
            kalico_c_api::kalico_runtime_gate_pieces(handle),
            kalico_c_api::KALICO_OK
        );
        assert_eq!(
            kalico_c_api::kalico_runtime_gate_pieces(handle),
            kalico_c_api::KALICO_OK
        );
        assert_eq!(
            kalico_c_api::kalico_runtime_ungate_pieces(handle),
            kalico_c_api::KALICO_OK
        );
    }
}

#[test]
fn null_rt_is_null_ptr_error() {
    let _guard = TEST_LOCK.lock().unwrap();
    unsafe {
        assert_eq!(
            kalico_c_api::kalico_runtime_gate_pieces(core::ptr::null_mut()),
            kalico_c_api::KALICO_ERR_NULL_PTR
        );
        assert_eq!(
            kalico_c_api::kalico_runtime_ungate_pieces(core::ptr::null_mut()),
            kalico_c_api::KALICO_ERR_NULL_PTR
        );
    }
}
