#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

use runtime::stepping_state::{
    AxisConfig, MAX_STEPPERS_PER_AXIS, N_AXES, StepMode, StepperRef, TickCaches,
};

#[test]
#[allow(clippy::float_cmp)] // testing zero initialization; exact equality is the contract here
fn tick_caches_constructs() {
    let c = TickCaches::new();
    assert_eq!(c.p_prev[0], 0.0);
    assert_eq!(c.v_prev[N_AXES - 1], 0.0);
}

#[test]
fn step_mode_discriminants_are_stable() {
    // Phase-stepping ISR stores `StepMode as u8` into an AtomicU8; the
    // numeric values are load-bearing across the C/Rust boundary.
    assert_eq!(StepMode::Pulse as u8, 0);
    assert_eq!(StepMode::Phase as u8, 1);
}

#[test]
fn constants_match_spec() {
    assert_eq!(N_AXES, 8);
    assert_eq!(MAX_STEPPERS_PER_AXIS, 4);
    let _ = core::mem::size_of::<StepperRef>();
    let _ = core::mem::size_of::<AxisConfig>();
}
