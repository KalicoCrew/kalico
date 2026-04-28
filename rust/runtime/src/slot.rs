//! `PaSlot` / `IsSlot` traits — runtime-evaluation slots. Spec §3.1.
//!
//! Step 5 ships only `Noop` impls (ZST + `#[inline(always)]` → optimizer
//! removes the call). Step 9 adds `TanhPa`; Step 8 adds `SmoothShaper`.
//!
//! Slot signature is intentionally `apply(&mut self, &mut TickState)` — `&mut self`
//! lets future impls maintain per-slot state (e.g., `TanhPa`'s previous-tick history)
//! without widening `TickState`. Spec §3.1 forward note.

use crate::state::TickState;

pub trait PaSlot {
    #[inline(always)]
    fn apply(&mut self, _state: &mut TickState) {}
}

pub trait IsSlot {
    #[inline(always)]
    fn apply(&mut self, _state: &mut TickState) {}
}

#[derive(Debug, Clone, Copy)]
pub struct NoopPa;

impl PaSlot for NoopPa {}

#[derive(Debug, Clone, Copy)]
pub struct NoopIs;

impl IsSlot for NoopIs {}

#[cfg(test)]
mod tests {
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
            xyz_e: [10.0, 20.0, 5.0],
            motors: [30.0, -10.0, 5.0],
        };
        let original = state;
        pa.apply(&mut state);
        is.apply(&mut state);
        // Noop must not touch any bits — compare as u32 to satisfy clippy::float_cmp.
        let bits = |a: [f32; 3]| a.map(f32::to_bits);
        assert_eq!(bits(state.xyz_e), bits(original.xyz_e));
        assert_eq!(bits(state.motors), bits(original.motors));
    }
}
