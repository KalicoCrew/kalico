#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Identity sinusoid LUT contract: amplitude anchors and symmetry.

use runtime::phase_lut::{self, CURRENT_AMPLITUDE, MOTOR_PERIOD};

#[test]
#[allow(clippy::integer_division)] // power-of-2 LUT size; integer quarter/half indices are exact
fn lut_quarter_cycle_anchors() {
    // angle 0   -> (0, +A)        sin(0)=0, cos(0)=1
    let (i_a, i_b) = phase_lut::lookup(0, 1);
    assert_eq!(i_a, 0);
    assert_eq!(i_b, CURRENT_AMPLITUDE);

    // angle pi/2 (mscount = MOTOR_PERIOD/4) -> (+A, 0)
    let (i_a, i_b) = phase_lut::lookup((MOTOR_PERIOD / 4) as u16, 1);
    assert_eq!(i_a, CURRENT_AMPLITUDE);
    assert!(i_b.abs() <= 1, "cos(pi/2) ~ 0, got {i_b}");

    // angle pi (mscount = MOTOR_PERIOD/2) -> (0, -A)
    let (i_a, i_b) = phase_lut::lookup((MOTOR_PERIOD / 2) as u16, 1);
    assert!(i_a.abs() <= 1, "sin(pi) ~ 0, got {i_a}");
    assert_eq!(i_b, -CURRENT_AMPLITUDE);

    // angle 3pi/2 (mscount = 3*MOTOR_PERIOD/4) -> (-A, 0)
    let (i_a, i_b) = phase_lut::lookup((3 * MOTOR_PERIOD / 4) as u16, 1);
    assert_eq!(i_a, -CURRENT_AMPLITUDE);
    assert!(i_b.abs() <= 1, "cos(3pi/2) ~ 0, got {i_b}");
}

#[test]
fn lut_direction_ignored_for_identity() {
    // For the identity sinusoid, forward and reverse must produce the same
    // currents. Calibration LUTs (silicon follow-up) introduce asymmetry.
    for &m in &[0u16, 137, 511, 768, 1023] {
        assert_eq!(phase_lut::lookup(m, 1), phase_lut::lookup(m, -1));
        assert_eq!(phase_lut::lookup(m, 1), phase_lut::lookup(m, 0));
    }
}

#[test]
fn lut_amplitude_bounded() {
    for m in 0u16..MOTOR_PERIOD as u16 {
        let (i_a, i_b) = phase_lut::lookup(m, 1);
        assert!(i_a.abs() <= CURRENT_AMPLITUDE);
        assert!(i_b.abs() <= CURRENT_AMPLITUDE);
    }
}

#[test]
fn lut_mscount_wraps() {
    // Caller mistakenly passes mscount >= MOTOR_PERIOD; lookup must wrap.
    assert_eq!(
        phase_lut::lookup(MOTOR_PERIOD as u16, 1),
        phase_lut::lookup(0, 1),
    );
    assert_eq!(
        phase_lut::lookup((MOTOR_PERIOD as u16) + 7, 1),
        phase_lut::lookup(7, 1),
    );
}
