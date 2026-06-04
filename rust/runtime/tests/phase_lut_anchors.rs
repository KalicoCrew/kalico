#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Smoke test for the build.rs-generated `PHASE_LUT`.
//!
//! Lives in `tests/` rather than as a `#[cfg(test)] mod tests` inside
//! `src/phase_lut.rs` so it can be exercised even when the broader
//! library test build is broken (e.g. by an unrelated engine.rs type
//! drift). The src-level unit tests cover the same anchors with finer
//! granularity once the lib-test path compiles again.

use runtime::phase_lut::{COIL_AMPLITUDE, PHASE_LUT, PHASE_LUT_SIZE};

#[test]
fn phase_lut_anchors_match_plan() {
    assert_eq!(PHASE_LUT[0], (COIL_AMPLITUDE, 0));
    assert_eq!(PHASE_LUT[PHASE_LUT_SIZE / 4], (0, COIL_AMPLITUDE));
    assert_eq!(PHASE_LUT[PHASE_LUT_SIZE / 2], (-COIL_AMPLITUDE, 0));
    assert_eq!(PHASE_LUT[3 * PHASE_LUT_SIZE / 4], (0, -COIL_AMPLITUDE));
}

#[test]
fn phase_lut_has_expected_size() {
    assert_eq!(PHASE_LUT_SIZE, 1024);
    assert_eq!(PHASE_LUT.len(), PHASE_LUT_SIZE);
}

#[test]
fn phase_lut_all_entries_within_amplitude_box() {
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
