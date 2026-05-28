//! Integration tests for the extended `configure_axes_blob` format (spec §4 C1).
//!
//! Covers four paths:
//!  1. Legacy 20-byte blob → `SharedState::step_modes` is not modified; existing
//!     values are preserved (no step_mode bytes means "don't change").
//!  2. Extended 25-byte blob with explicit modes (including `Modulated` on a
//!     phase-capable MCU) → `step_modes` populated correctly.
//!  3. Extended 25-byte blob requesting `Modulated` on a non-phase-capable MCU
//!     (mcu_caps bit 0 = 0) → `KALICO_ERR_CAPABILITY_MISSING` returned.
//!  4. Any blob length other than 20 or 25 → `KALICO_ERR_INVALID_KINEMATICS`.
//!
//! All tests run in one binary that calls `runtime_handle_create` once
//! (singleton precondition). `kalico_runtime_configure_axes_blob` is
//! idempotent once initialised, so multiple calls in the same binary are safe.
//!
//! Tests are written to be order-independent: each test that checks step_modes
//! first sends an explicit extended blob to establish a known state.

#![allow(unsafe_code, non_upper_case_globals)]

use core::sync::atomic::Ordering;
use runtime::state::{RuntimeContext, SharedState, StepMode};

// ---- Host-side stubs required by runtime_ffi's extern "C" declarations ------

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

// ---- Helpers ----------------------------------------------------------------

/// Build a minimal valid 20-byte blob (legacy format).
///
/// Uses CoreXY kinematics (tag=0), motors A and B present (present_mask=0x03),
/// no AWD, no invert, 80 steps/mm for both.
fn legacy_blob() -> [u8; 20] {
    let mut blob = [0u8; 20];
    blob[0] = 0; // kinematics = CoreXyAndE
    blob[1] = 0x03; // present_mask: A (bit0) + B (bit1)
    blob[2] = 0x00; // awd_mask: none
    blob[3] = 0x00; // invert_mask: none
    // steps_per_mm = 80.0f32 for motors 0 and 1; 0.0 for 2 and 3.
    let eighty: [u8; 4] = 80.0f32.to_le_bytes();
    blob[4..8].copy_from_slice(&eighty);
    blob[8..12].copy_from_slice(&eighty);
    blob
}

/// Build a 25-byte extended blob.
///
/// `mcu_caps`: bit 0 = mcu_supports_phase_stepping.
/// `step_modes`: four u8 values for motors 0..4.
fn extended_blob(mcu_caps: u8, step_modes: [u8; 4]) -> [u8; 25] {
    let mut blob = [0u8; 25];
    blob[..20].copy_from_slice(&legacy_blob());
    blob[20] = mcu_caps;
    blob[21] = step_modes[0];
    blob[22] = step_modes[1];
    blob[23] = step_modes[2];
    blob[24] = step_modes[3];
    blob
}

/// Read step_mode for stepper `i` from the runtime's SharedState.
///
/// SAFETY: the runtime pointer must be valid and `i < MAX_STEPPER_OIDS`.
unsafe fn read_step_mode(ctx: *mut RuntimeContext, i: usize) -> StepMode {
    let shared: &SharedState = unsafe { &*core::ptr::addr_of!((*ctx).shared) };
    let raw = shared.step_modes[i].load(Ordering::Acquire);
    StepMode::from_u8(raw).expect("valid StepMode discriminant in shared state")
}

/// Write step_mode directly into SharedState (for test setup without going
/// through the blob path).
unsafe fn write_step_mode(ctx: *mut RuntimeContext, i: usize, mode: StepMode) {
    let shared: &SharedState = unsafe { &*core::ptr::addr_of!((*ctx).shared) };
    shared.step_modes[i].store(mode as u8, Ordering::Release);
}

// ---- Singleton runtime init -------------------------------------------------

/// Thin wrapper that makes `*mut KalicoRuntime` shareable across test threads.
///
/// SAFETY invariant: the pointer is stable (backed by the `RT_CELL` static)
/// and all concurrent accesses go through the `AtomicBool INIT_DONE` guard and
/// the half-split ownership discipline described in `runtime_ffi`.  The tests
/// in this binary are all foreground-only (no ISR, no tick thread) so there
/// is no actual concurrent mutable access.
struct RuntimePtr(*mut kalico_c_api::KalicoRuntime);
// SAFETY: see above.
unsafe impl Send for RuntimePtr {}
unsafe impl Sync for RuntimePtr {}

fn get_runtime() -> *mut kalico_c_api::KalicoRuntime {
    static RT: std::sync::OnceLock<RuntimePtr> = std::sync::OnceLock::new();
    RT.get_or_init(|| {
        let rt = kalico_c_api::runtime_handle_create();
        assert!(!rt.is_null(), "runtime_handle_create failed");
        RuntimePtr(rt)
    })
    .0
}

// ---- Tests ------------------------------------------------------------------

/// Legacy blob does not modify step_modes; pre-existing values survive.
#[test]
fn legacy_blob_preserves_existing_step_modes() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();

    // Set a known state: flip motor 0 to Modulated via direct atomic write.
    unsafe { write_step_mode(ctx, 0, StepMode::Modulated) };
    unsafe { write_step_mode(ctx, 1, StepMode::StepTime) };

    // Send a legacy 20-byte blob. It should NOT reset step_modes.
    let blob = legacy_blob();
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "legacy blob should be accepted"
    );

    // Motor 0 must still be Modulated (legacy path doesn't touch step_modes).
    let m0 = unsafe { read_step_mode(ctx, 0) };
    assert_eq!(
        m0,
        StepMode::Modulated,
        "legacy blob must preserve pre-existing Modulated on motor 0",
    );
    let m1 = unsafe { read_step_mode(ctx, 1) };
    assert_eq!(m1, StepMode::StepTime, "motor 1 still StepTime");
}

/// Extended blob with explicit step_modes populates SharedState correctly.
#[test]
fn extended_blob_populates_step_modes() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();

    // Phase-capable MCU (bit 0 set). Request: motor 0 = Modulated, rest = StepTime.
    let blob = extended_blob(
        0x01, // mcu_caps: phase-stepping capable
        [
            StepMode::Modulated as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "extended blob (phase-capable) should be accepted"
    );

    let expected = [
        StepMode::Modulated,
        StepMode::StepTime,
        StepMode::StepTime,
        StepMode::StepTime,
    ];
    for (i, &want) in expected.iter().enumerate() {
        let got = unsafe { read_step_mode(ctx, i) };
        assert_eq!(got, want, "motor {i}: expected {want:?}, got {got:?}");
    }
    // Slots 4..8 are not covered by the blob's 4-entry array; they should not
    // have been modified by the blob path (they are initialised to StepTime and
    // the blob loop stops at i=3).
    for i in 4..runtime::state::MAX_STEPPER_OIDS {
        // We only assert they are valid StepMode values, not their exact value,
        // since other tests may have modified them. Just confirm no panic.
        let _ = unsafe { read_step_mode(ctx, i) };
    }
}

/// Extended blob with all-StepTime and no phase capability is accepted.
#[test]
fn extended_blob_all_step_time_non_phase_mcu_accepted() {
    let rt = get_runtime();
    let blob = extended_blob(
        0x00, // mcu_caps: no phase stepping
        [
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_OK,
        "all-StepTime extended blob on non-phase MCU must be accepted",
    );
    let ctx = rt.cast::<RuntimeContext>();
    for i in 0..4 {
        let mode = unsafe { read_step_mode(ctx, i) };
        assert_eq!(mode, StepMode::StepTime, "motor {i} should be StepTime");
    }
}

/// Extended blob requesting Modulated on a non-phase MCU returns CAPABILITY_MISSING.
/// Defense-in-depth check — the host (Task E1) should prevent this case.
#[test]
fn extended_blob_modulated_without_capability_rejected() {
    let rt = get_runtime();
    let blob = extended_blob(
        0x00, // mcu_caps: no phase stepping
        [
            StepMode::Modulated as u8, // motor 0 requests Modulated → rejected
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_CAPABILITY_MISSING,
        "Modulated on non-phase MCU must return KALICO_ERR_CAPABILITY_MISSING, got {rc}",
    );
}

/// Any blob length not in {20, 25, 26 + 3·N for 0 ≤ N ≤ MAX_STEPPER_OIDS}
/// is rejected. (26 itself is a valid phase-length with N=0; 27 / 28 are not
/// since they don't satisfy `(len - 26) % 3 == 0`.)
#[test]
fn invalid_blob_lengths_rejected() {
    let rt = get_runtime();
    for bad_len in [0u32, 1, 19, 21, 24, 27, 28, 100] {
        let buf = vec![0u8; bad_len as usize];
        let rc =
            unsafe { kalico_c_api::kalico_runtime_configure_axes_blob(rt, buf.as_ptr(), bad_len) };
        assert_eq!(
            rc,
            kalico_c_api::KALICO_ERR_INVALID_KINEMATICS,
            "blob_len={bad_len} must be rejected with KALICO_ERR_INVALID_KINEMATICS, got {rc}",
        );
    }
}
