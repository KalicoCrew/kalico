//! Variable-length configure_axes blob: per-motor phase-stepping SPI config
//! parsing, validation, and FFI introspection (topology-agnostic;
//! supports N motors per kinematic slot up to MAX_STEPPER_OIDS=16 total).
//!
//! Covers:
//!  1. Two phase motors (X+Y Modulated) → `KALICO_OK`, per-motor lookups
//!     return the packed config.
//!  2. CoreXY+AWD (4 motors → 2 slots, slot_idx layout [0,0,1,1]) →
//!     `KALICO_OK`, all 4 motors carry phase config in their own slots.
//!  3. Phase config with `slot_idx >= 4` → `KALICO_ERR_INVALID_KINEMATICS`.
//!  4. Phase config whose `step_modes[slot_idx]` is StepTime →
//!     `KALICO_ERR_INVALID_KINEMATICS`.
//!  5. Bad blob length (mid-entry truncation; N exceeds cap) →
//!     `KALICO_ERR_INVALID_KINEMATICS`.
//!  6. 25-byte blob still accepted → no phase config on any motor.
//!  7. 25-byte after a prior phase-config install clears all slots
//!     (variable-length parser publishes phase_motor_count = 0 when
//!     blob_len == 25; prior config goes away).
//!
//! Test-harness FFI shims mirror `configure_axes_blob_step_modes.rs`. Runtime
//! singleton is shared with that test binary (each tests/ file compiles to its
//! own binary, so the singleton is per-binary).

#![allow(unsafe_code, non_upper_case_globals)]

use core::sync::atomic::Ordering;
use runtime::phase_config::PhaseConfig;
use runtime::state::{RuntimeContext, SharedState, StepMode, MAX_STEPPER_OIDS};

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

/// Build a variable-length configure_axes blob: 26 + 3·N bytes.
///
/// - `kinematics`: kinematics tag byte (0 = CoreXyAndE, 1 = CartesianXyzAndE).
/// - `present_mask`: bit i set → motor slot i is present.
/// - `step_modes`: per-slot StepMode raw bytes (0 = Modulated, 1 = StepTime).
/// - `phase_entries`: dense list of (bus_id, cs_pin_id, slot_idx) triples.
fn build_phase_blob(
    kinematics: u8,
    present_mask: u8,
    step_modes: [u8; 4],
    phase_entries: &[(u8, u8, u8)],
) -> Vec<u8> {
    let n = phase_entries.len();
    let mut blob = vec![0u8; 26 + 3 * n];
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
    blob[25] = n as u8;
    for (i, &(bus, cs, slot)) in phase_entries.iter().enumerate() {
        blob[26 + i * 3] = bus;
        blob[26 + i * 3 + 1] = cs;
        blob[26 + i * 3 + 2] = slot;
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

/// Wipe SharedState's per-motor phase tables back to "no config anywhere".
unsafe fn clear_all_phase_state(ctx: *mut RuntimeContext) {
    let shared: &SharedState = unsafe { &*core::ptr::addr_of!((*ctx).shared) };
    for slot in shared.phase_config.iter() {
        slot.store(0xFFFF, Ordering::Release);
    }
    for slot in shared.phase_slot_idx.iter() {
        slot.store(0xFF, Ordering::Release);
    }
    shared.phase_motor_count.store(0, Ordering::Release);
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

fn shared_of(ctx: *mut RuntimeContext) -> &'static SharedState {
    // SAFETY: ctx is the singleton; SharedState lives for the test process.
    unsafe { &*core::ptr::addr_of!((*ctx).shared) }
}

// ---- Tests ------------------------------------------------------------------

#[test]
fn accepts_two_phase_motors_on_distinct_slots() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_state(ctx) };

    // X+Y Modulated, Z+E StepTime. One TMC per slot — slot_idx 0 and 1.
    let blob = build_phase_blob(
        1,
        0b0000_1111,
        [
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
        &[(0, 0x05, 0), (0, 0x06, 1)],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "two-motor blob must accept, got {rc}");

    let shared = shared_of(ctx);
    assert_eq!(shared.phase_motor_count.load(Ordering::Acquire), 2);
    assert_eq!(
        PhaseConfig::unpack(shared.phase_config[0].load(Ordering::Acquire)),
        Some(PhaseConfig {
            spi_bus_id: 0,
            cs_pin_id: 0x05,
        }),
    );
    assert_eq!(shared.phase_slot_idx[0].load(Ordering::Acquire), 0);
    assert_eq!(
        PhaseConfig::unpack(shared.phase_config[1].load(Ordering::Acquire)),
        Some(PhaseConfig {
            spi_bus_id: 0,
            cs_pin_id: 0x06,
        }),
    );
    assert_eq!(shared.phase_slot_idx[1].load(Ordering::Acquire), 1);
    // Past-count entries stay cleared.
    for i in 2..MAX_STEPPER_OIDS {
        assert_eq!(
            shared.phase_config[i].load(Ordering::Acquire),
            0xFFFF,
            "motor {i} must be clear past count",
        );
        assert_eq!(shared.phase_slot_idx[i].load(Ordering::Acquire), 0xFF);
    }
}

#[test]
fn accepts_corexy_awd_four_motors_two_slots() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_state(ctx) };

    // CoreXY+AWD: stepper_x + stepper_x1 share slot 0; stepper_y +
    // stepper_y1 share slot 1. slot_idx layout [0,0,1,1] — exactly the
    // shape that was broken in the previous 4-slot wire format.
    let blob = build_phase_blob(
        0,
        0b0000_1111,
        [
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
        &[(3, 5, 0), (3, 6, 0), (3, 7, 1), (3, 8, 1)],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK, "CoreXY+AWD must accept, got {rc}");

    let shared = shared_of(ctx);
    assert_eq!(shared.phase_motor_count.load(Ordering::Acquire), 4);
    let expected = [
        (PhaseConfig { spi_bus_id: 3, cs_pin_id: 5 }, 0u8),
        (PhaseConfig { spi_bus_id: 3, cs_pin_id: 6 }, 0u8),
        (PhaseConfig { spi_bus_id: 3, cs_pin_id: 7 }, 1u8),
        (PhaseConfig { spi_bus_id: 3, cs_pin_id: 8 }, 1u8),
    ];
    for (i, (cfg, slot)) in expected.iter().enumerate() {
        assert_eq!(
            PhaseConfig::unpack(shared.phase_config[i].load(Ordering::Acquire)),
            Some(*cfg),
            "motor {i}: phase_config",
        );
        assert_eq!(
            shared.phase_slot_idx[i].load(Ordering::Acquire),
            *slot,
            "motor {i}: slot_idx",
        );
    }
}

#[test]
fn rejects_slot_idx_out_of_range() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_state(ctx) };

    let blob = build_phase_blob(
        1,
        0b0000_1111,
        [
            StepMode::Modulated as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
        &[(0, 0x05, 7)], // slot_idx >= 4 is invalid
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_INVALID_KINEMATICS,
        "slot_idx=7 must reject, got {rc}",
    );
}

#[test]
fn rejects_phase_config_on_steptime_slot() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_state(ctx) };

    // Phase config installed on slot 0 but step_modes[0] is StepTime → reject.
    let blob = build_phase_blob(
        1,
        0b0000_1111,
        [
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
        &[(0, 0x05, 0)],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_INVALID_KINEMATICS,
        "phase config on StepTime slot must reject, got {rc}",
    );
}

#[test]
fn rejects_bad_blob_length_mid_entry_truncation() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_state(ctx) };

    // Count says 2, but the body length corresponds to N=1.5 (one full
    // entry plus one truncated byte). 26 + 3*2 = 32 expected vs 30 actual.
    let mut blob = build_phase_blob(
        1,
        0b0000_1111,
        [
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
        &[(0, 0x05, 0), (0, 0x06, 1)],
    );
    // Trim 2 bytes off the tail to create a mid-entry truncation.
    blob.truncate(blob.len() - 2);
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_INVALID_KINEMATICS,
        "truncated blob must reject, got {rc}",
    );
}

#[test]
fn rejects_count_above_max_stepper_oids() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_state(ctx) };

    // Build a blob with count = MAX_STEPPER_OIDS + 1 = 17. The byte count
    // is consistent (26 + 3*17 = 77) so only the count-cap check trips.
    let mut entries: Vec<(u8, u8, u8)> = (0..17u8)
        .map(|i| (3, 0x10 + i, i % 4))
        .collect();
    let blob = build_phase_blob(
        1,
        0b0000_1111,
        [
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
        ],
        &entries,
    );
    let _ = &mut entries; // keep lint happy
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(rt, blob.as_ptr(), blob.len() as u32)
    };
    assert_eq!(
        rc,
        kalico_c_api::KALICO_ERR_INVALID_KINEMATICS,
        "count above MAX_STEPPER_OIDS must reject, got {rc}",
    );
}

#[test]
fn legacy_25_byte_blob_clears_prior_phase_config() {
    let rt = get_runtime();
    let ctx = rt.cast::<RuntimeContext>();
    unsafe { clear_all_phase_state(ctx) };

    // Step 1: install a 2-motor phase config.
    let installed = build_phase_blob(
        1,
        0b0000_1111,
        [
            StepMode::Modulated as u8,
            StepMode::Modulated as u8,
            StepMode::StepTime as u8,
            StepMode::StepTime as u8,
        ],
        &[(0, 0x05, 0), (0, 0x06, 1)],
    );
    let rc = unsafe {
        kalico_c_api::kalico_runtime_configure_axes_blob(
            rt,
            installed.as_ptr(),
            installed.len() as u32,
        )
    };
    assert_eq!(rc, kalico_c_api::KALICO_OK);
    assert_eq!(shared_of(ctx).phase_motor_count.load(Ordering::Acquire), 2);

    // Step 2: send a 25-byte blob with all-StepTime — must clear all
    // prior phase config and zero the count.
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
    assert_eq!(rc, kalico_c_api::KALICO_OK, "25-byte blob must accept, got {rc}");

    let shared = shared_of(ctx);
    assert_eq!(shared.phase_motor_count.load(Ordering::Acquire), 0);
    for i in 0..MAX_STEPPER_OIDS {
        assert_eq!(
            shared.phase_config[i].load(Ordering::Acquire),
            0xFFFF,
            "motor {i}: 25-byte blob must clear phase_config",
        );
        assert_eq!(
            shared.phase_slot_idx[i].load(Ordering::Acquire),
            0xFF,
            "motor {i}: 25-byte blob must clear phase_slot_idx",
        );
    }
}
