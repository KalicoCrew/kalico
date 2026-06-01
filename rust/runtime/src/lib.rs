//! Layer 4 MCU runtime — 40 kHz hard-real-time ISR with stub NURBS evaluator.
//! See `docs/superpowers/specs/2026-04-28-layer-4-mcu-framework-stub-design.md`.

#![cfg_attr(not(feature = "host"), no_std)]
#![deny(
    clippy::panic,
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::indexing_slicing,
    clippy::panic_in_result_fn,
    clippy::todo,
    clippy::unimplemented,
    clippy::unreachable,
    clippy::integer_division,
    unsafe_op_in_unsafe_fn
)]

// `alloc` is needed by the `sim_fixtures::init_test_runtime` helper on hosted
// environments. Gate to match the helper's `#[cfg(not(target_os = "none"))]`
// so the embedded sim build stays allocator-free.
#[cfg(all(feature = "kalico-sim", not(target_os = "none")))]
extern crate alloc;

pub mod bezier_root;
pub mod segment;
pub mod sizing;
pub use sizing::RT_STORAGE_SIZE;
pub mod clock;
pub mod endstop;
pub mod engine;
pub mod error;
pub mod fault_helpers;
pub(crate) mod isr_phase;
pub mod monomial;
pub mod per_axis_timer;
pub mod phase_config;
pub mod phase_lut;
pub mod piece_ring;
#[cfg(feature = "kalico-sim")]
pub mod sim_fixtures;
pub mod spi_queue;
pub mod state;
pub use state::{SetStepModeError, StepMode, set_step_mode};
pub mod step;
pub mod step_queue;
pub mod stepping_state;
pub mod stream;
pub mod sub_sample_timing;
pub mod test_xdirect_capture;
pub mod tick;
pub mod wire;

#[cfg(test)]
mod tests;
