use super::*;
use crate::state::TickState;
use core::mem::size_of;

#[test]
fn noop_slots_are_zsts() {
    assert_eq!(size_of::<NoopPa>(), 0);
    assert_eq!(size_of::<NoopIs>(), 0);
}

#[test]
fn noop_apply_does_not_mutate_state() {
    let mut pa = NoopPa;
    let mut is = NoopIs;
    let mut state = TickState {
        dt: 1.0 / 40_000.0,
        positions: [10.0, 20.0, 0.0, 5.0],
        motors: [30.0, -10.0, 0.0, 5.0],
    };
    let original = state;
    pa.apply(&mut state);
    is.apply(&mut state);
    // Noop must not touch any bits — compare as u32 to satisfy clippy::float_cmp.
    let bits = |a: [f32; 4]| a.map(f32::to_bits);
    assert_eq!(bits(state.positions), bits(original.positions));
    assert_eq!(bits(state.motors), bits(original.motors));
}
