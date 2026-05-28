#![allow(clippy::integer_division)] // LUT sizes are powers of 2; integer division for quarter/half indices is exact
#![allow(clippy::indexing_slicing)] // LUT index expressions guaranteed in-bounds by construction

use super::{COIL_AMPLITUDE, PHASE_LUT, PHASE_LUT_SIZE};

/// Plan-canonical anchor check: the `(cos, sin)`-ordered LUT must
/// have its four quadrant points exactly at the amplitude axes.
#[test]
fn anchors_match_expectation() {
    assert_eq!(PHASE_LUT[0], (COIL_AMPLITUDE, 0));
    assert_eq!(PHASE_LUT[PHASE_LUT_SIZE / 4], (0, COIL_AMPLITUDE));
    assert_eq!(PHASE_LUT[PHASE_LUT_SIZE / 2], (-COIL_AMPLITUDE, 0));
    assert_eq!(PHASE_LUT[3 * PHASE_LUT_SIZE / 4], (0, -COIL_AMPLITUDE));
}

/// Every entry must be inside the i16 amplitude box.
#[test]
fn all_entries_within_amplitude() {
    for (i, (a, b)) in PHASE_LUT.iter().enumerate() {
        assert!(
            a.abs() <= COIL_AMPLITUDE,
            "PHASE_LUT[{i}].0 = {a} out of range"
        );
        assert!(
            b.abs() <= COIL_AMPLITUDE,
            "PHASE_LUT[{i}].1 = {b} out of range"
        );
    }
}

/// Sanity check on the legacy `(sin, cos)`-ordered table.
#[test]
fn legacy_lut_entries_anchors() {
    use super::{CURRENT_AMPLITUDE, LUT_ENTRIES, MOTOR_PERIOD};
    // LUT_ENTRIES[i] = (sin, cos)
    assert_eq!(LUT_ENTRIES[0], (0, CURRENT_AMPLITUDE));
    assert_eq!(LUT_ENTRIES[MOTOR_PERIOD / 4], (CURRENT_AMPLITUDE, 0));
}
