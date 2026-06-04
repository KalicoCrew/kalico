#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]

//! Smoke test for `stepping_state` — verifies the module compiles in
//! isolation. The broader `runtime` lib-test build is currently blocked
//! by pre-existing `Consumer<Segment>` mismatches in `engine.rs` (see
//! 2026-05-19 plan, Task 6 verification path (d)); this integration
//! test sidesteps those by linking only what `stepping_state` actually
//! depends on.

use runtime::stepping_state::{
    AxisConfig, MAX_STEPPERS_PER_AXIS, N_AXES, StepMode, StepperRef, TickCaches,
};

#[test]
#[allow(clippy::float_cmp)] // testing zero initialization; exact equality is the contract here
fn tick_caches_constructs() {
    let c = TickCaches::new();
    assert_eq!(c.p_prev[0], 0.0);
    assert_eq!(c.v_prev[N_AXES - 1], 0.0);
    // TickCaches holds only p_prev and v_prev since the E-follows-XY
    // arc-length fields (v_xy_prev, v_xy_this, vdot_xy_accelerating)
    // were removed — all four axes are now evaluated uniformly.
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
    // Sanity: TypeIds we expect to construct exist and are nameable.
    let _ = core::mem::size_of::<StepperRef>();
    let _ = core::mem::size_of::<AxisConfig>();
}
