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

// `alloc` is needed by the `sim_fixtures::init_test_runtime` helper, which
// `Box::leak`s static queues so the split `Producer`/`Consumer` halves carry
// `'static` lifetimes. Pull it in unconditionally when `kalico-sim` is on so
// the helper compiles under both the `host` (std) and `no_std`-with-alloc
// builds. `target_os = "none"` MCU firmware skips the helper via the
// `#[cfg(not(target_os = "none"))]` gate in `sim_fixtures.rs`, so this
// `extern crate alloc;` adds no firmware footprint there either.
#[cfg(feature = "kalico-sim")]
extern crate alloc;

pub mod bezier_root;
pub mod clock;
pub mod config;
pub mod curve_pool;
pub mod endstop;
pub mod engine;
pub use engine::arm_step_timer_for_stepper;
pub mod error;
pub mod kinematics;
pub mod queue;
pub mod reclaim;
pub mod segment;
#[cfg(feature = "kalico-sim")]
pub mod sim_fixtures;
pub mod slot;
pub mod state;
pub use state::{set_step_mode, SetStepModeError, StepMode};
pub mod step;
pub mod step_producer;
pub mod step_ring;
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
