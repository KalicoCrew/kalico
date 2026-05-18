//! 33-byte configure_axes blob: phase-stepping per-motor SPI config parsing,
//! validation, and FFI introspection (spec §4.1, §3.2).
//!
//! Covers four paths:
//!  1. Three motors carrying phase config → `KALICO_ERR_INVALID_PHASE_AXIS_COUNT`
//!     (audible-band protection per spec §3.2).
//!  2. Phase config installed on a `StepTime` motor → `KALICO_ERR_INVALID_KINEMATICS`.
//!  3. Two phase motors (X+Y Modulated) plus Z+E StepTime → `KALICO_OK`,
//!     `kalico_runtime_query_phase_config` returns the packed config.
//!  4. Legacy 25-byte blob still accepted → no phase config on any motor.
//!
//! Test-harness FFI shims mirror `configure_axes_blob_step_modes.rs`. Runtime
//! singleton is shared with that test binary (each tests/ file compiles to its
//! own binary, so the singleton is per-binary).

#![allow(unsafe_code, non_upper_case_globals)]

use core::sync::atomic::Ordering;
use runtime::phase_config::PhaseConfig;
use runtime::state::{RuntimeContext, SharedState, StepMode};

// ---- Host-side stubs required by runtime_ffi's extern "C" declarations ------

#[unsafe(no_mangle)]
pub static runtime_clock_freq: u32 = 520_000_000;

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

/// Build a 33-byte configure_axes blob.
///
/// - `kinematics`: kinematics tag byte (0 = CoreXyAndE, 1 = CartesianXyzAndE).
/// - `present_mask`: bit i set → motor i is present.
/// - `step_modes`: per-motor StepMode raw bytes (0 = Modulated, 1 = StepTime).
/// - `phase_configs`: `Some((bus, cs))` to install phase config; `None` to leave
///   the slot as "no phase config" (sentinel `0xFF` on bus/cs bytes).
fn build_33_byte_blob(
    kinematics: u8,
    present_mask: u8,
    step_modes: [u8; 4],
    phase_configs: [Option<(u8, u8)>; 4],
) -> [u8; 33] {
    let mut blob = [0u8; 33];
    blob[0] = kinematics;
    blob[1] = present_mask;
    blob[2] = 0; // awd_mask
    blob[3] = 0; // invert_mask
    let eighty = 80.0f32.to_le_bytes();
    for i in 0..4 {
        let off = 4 + i * 4;
        blob[off..off + 4].copy_from_slice(&eighty);
    }
    blob[20] = 0x01; // mcu_caps: PHASE_STEPPING_CAPABLE
    blob[21] = step_modes[0];
    blob[22] = step_modes[1];
    blob[23] = step_modes[2];
    blob[24] = step_modes[3];
    for i in 0..4 {
        let (bus, cs) = phase_configs[i].unwrap_or((0xFF, 0xFF));
        blob[25 + i * 2] = bus;
        blob[26 + i * 2] = cs;
    }
    blob
}

/// Build a legacy 25-byte extended blob (no phase config bytes).
fn build_25_byte_blob(present_mask: u8, step_modes: [u8; 4]) -> [u8; 25] {
    let mut blob = [0u8; 25];
    blob[0] = 1; // CartesianXyzAndE
    blob[1] = present_mask;
    let eighty = 80.0f32.to_le_bytes();
    for i in 0..4 {
        let off = 4 + i * 4;
        blob[off..off + 4].copy_from_slice(&eighty);
    }
    blob[20] = 0x01; // mcu_caps: PHASE_STEPPING_CAPABLE
    blob[21] = step_modes[0];
    blob[22] = step_modes[1];
    blob[23] = step_modes[2];
    blob[24] = step_modes[3];
    blob
}

/// Wipe SharedState::phase_config back to "no config on any motor" so each
/// test starts from a known-clean slate (the singleton runtime persists
/// across tests in the same binary).
unsafe fn clear_all_phase_configs(ctx: *mut RuntimeContext) {
    let shared: &SharedState = unsafe { &*core::ptr::addr_of!((*ctx).shared) };
    for slot in shared.phase_config.iter() {
        slot.store(0xFFFF, Ordering::Release);
    }
}

// ---- Singleton runtime init -------------------------------------------------

struct RuntimePtr(*mut kalico_c_api::KalicoRuntime);
// SAFETY: pointer is stable (RT_CELL static); tests are foreground-only.
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

#[test]
fn rejects_blob_with_three_phase_motors() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_configs(ctx) };

    // Three motors with phase config — exceeds the spec §3.2 ≤2 limit.
    let blob = build_33_byte_blob(
        1,           // CartesianXyzAndE
        0b0000_1111, // all 4 present
        [
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::StepTime as u8,
        ],
        [
            Some((0, 0x05)),
            Some((0, 0x06)),
            Some((0, 0x07)),
            None,
        ],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_INVALID_PHASE_AXIS_COUNT,
        "three phase motors must be rejected with KALICO_ERR_INVALID_PHASE_AXIS_COUNT, got {rc}",
    );
}

#[test]
fn rejects_phase_config_on_steptime_motor() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_configs(ctx) };

    // Phase config installed on motor 0 but motor 0 is StepTime — reject.
    let blob = build_33_byte_blob(
        1,
        0b0000_1111,
        [
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
        [Some((0, 0x05)), None, None, None],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_INVALID_KINEMATICS,
        "phase config on StepTime motor must return KALICO_ERR_INVALID_KINEMATICS, got {rc}",
    );
}

#[test]
fn accepts_two_phase_motors() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_configs(ctx) };

    // X+Y Modulated with SPI config (bus=0, cs=0x05/0x06). Z+E StepTime, no
    // phase config. Spec §3.2 audible-band rule: exactly 2 phase motors.
    let blob = build_33_byte_blob(
        1,
        0b0000_1111,
        [
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
        [Some((0, 0x05)), Some((0, 0x06)), None, None],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "X+Y phase blob must be accepted, got {rc}");

    // Post-condition: per-motor phase_config reflects the blob.
    let raw0 = unsafe { kalico_c_api::kalico_runtime_query_phase_config(rt, 0) };
    let raw1 = unsafe { kalico_c_api::kalico_runtime_query_phase_config(rt, 1) };
    let raw2 = unsafe { kalico_c_api::kalico_runtime_query_phase_config(rt, 2) };
    let raw3 = unsafe { kalico_c_api::kalico_runtime_query_phase_config(rt, 3) };
    assert_eq!(
        PhaseConfig::unpack(raw0),
        Some(PhaseConfig {
            spi_bus_id: 0,
            cs_pin_id: 0x05
        }),
    );
    assert_eq!(
        PhaseConfig::unpack(raw1),
        Some(PhaseConfig {
            spi_bus_id: 0,
            cs_pin_id: 0x06
        }),
    );
    assert_eq!(PhaseConfig::unpack(raw2), None, "motor 2 must have no phase config");
    assert_eq!(PhaseConfig::unpack(raw3), None, "motor 3 must have no phase config");
}

#[test]
fn legacy_25_byte_blob_still_accepted() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_configs(ctx) };

    // 25-byte all-StepTime blob — must still parse cleanly with no phase
    // config installed on any motor. Guards against regression on the
    // Gate-A / Gate-B paths.
    let blob = build_25_byte_blob(
        0b0000_1111,
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
    assert_eq!(rc, kalico_c_api::KALICO_OK, "legacy 25-byte blob must remain valid, got {rc}");

    for i in 0..4u8 {
        let raw = unsafe { kalico_c_api::kalico_runtime_query_phase_config(rt, i) };
        assert_eq!(
            PhaseConfig::unpack(raw),
            None,
            "motor {i}: legacy blob must not install phase config",
        );
    }
}
