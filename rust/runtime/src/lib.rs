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

pub mod clock;
pub mod config;
pub mod curve_pool;
pub mod endstop;
pub mod engine;
pub mod error;
pub mod kinematics;
pub mod queue;
pub mod reclaim;
pub mod segment;
#[cfg(feature = "kalico-sim")]
pub mod sim_fixtures;
pub mod slot;
pub mod state;
pub use state::StepMode;
pub mod step;
pub mod step_time;
pub mod stream;
pub mod trace;
pub mod wire;

#[cfg(test)]
mod tests {
    #[test]
    fn smoke() {
        // Crate compiles; module tree intact.
    }
}
