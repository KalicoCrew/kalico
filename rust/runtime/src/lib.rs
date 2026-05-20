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
// `'static` lifetimes. The helper itself is gated `#[cfg(not(target_os =
// "none"))]` in `sim_fixtures.rs`, so the MCU sim firmware build (target
// `thumbv7em-none-eabi`, `target_os = "none"`) doesn't compile it — and
// pulling in `alloc` on that target without supplying a `#[global_allocator]`
// fails the link with "no global memory allocator found." Gate this `extern
// crate alloc;` to match the helper's gate so the embedded sim build is
// allocator-free.
#[cfg(all(feature = "kalico-sim", not(target_os = "none")))]
extern crate alloc;

pub mod bezier_root;
pub mod c_segment_queue;
pub mod clock;
pub mod config;
pub mod cubic_curve;
pub mod curve_pool;
pub use curve_pool::RT_STORAGE_SIZE;
pub mod endstop;
pub mod engine;
pub mod error;
pub mod fault_helpers;
pub mod kinematics;
pub mod modulator;
pub mod monomial;
pub mod per_axis_timer;
pub mod phase_config;
pub mod phase_lut;
pub mod queue;
pub mod reclaim;
pub mod segment;
#[cfg(feature = "kalico-sim")]
pub mod sim_fixtures;
pub mod slot;
pub mod spi_queue;
pub mod state;
pub use state::{set_step_mode, SetStepModeError, StepMode};
pub mod step;
pub mod step_queue;
pub mod stepping_state;
pub mod stream;
pub mod sub_sample_timing;
pub mod test_xdirect_capture;
pub mod tick;
pub mod trace;
pub mod wire;

#[cfg(test)]
mod tests {
    #[test]
    fn smoke() {
        // Crate compiles; module tree intact.
    }
}
