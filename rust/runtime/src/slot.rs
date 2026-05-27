//! PA/IS slot trait stubs — Task 5 placeholder.
//!
//! The full slot trait hierarchy has been removed. These stubs retain the
//! minimum needed for `Engine<P, I>` to compile until Task 6 rewrites it.

/// Pressure-advance slot trait (stub).
pub trait PaSlot {}

/// Input-shaping slot trait (stub).
pub trait IsSlot {}

/// Zero-cost no-op implementations for the production `EngineImpl` typedef.
#[derive(Debug, Default, Clone, Copy)]
pub struct NoopPa;

#[derive(Debug, Default, Clone, Copy)]
pub struct NoopIs;

impl PaSlot for NoopPa {}
impl IsSlot for NoopIs {}
