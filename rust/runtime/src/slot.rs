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

#[derive(Debug, Default, Clone, Copy)]
pub struct NoopPa;

impl PaSlot for NoopPa {}

#[derive(Debug, Default, Clone, Copy)]
pub struct NoopIs;

impl IsSlot for NoopIs {}

#[cfg(test)]
mod tests;
