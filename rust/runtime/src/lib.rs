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

#[cfg(all(
    not(feature = "host"),
    not(test),
    not(feature = "motion-module-stepper")
))]
compile_error!(
    "a bare-metal motion node must enable a dispatch module \
     (e.g. motion-module-stepper); none is active"
);

pub mod bezier_root;
pub mod segment;
pub mod sizing;
pub use sizing::RT_STORAGE_SIZE;
pub mod clock;
#[cfg(feature = "motion-module-stepper")]
pub mod dispatch_stepper;
pub mod engine;
pub mod error;
pub mod fault_helpers;
pub mod fault_sink;
pub(crate) mod isr_phase;
pub mod log_codes;
pub mod monomial;
pub mod motion_core;
pub mod per_axis_timer;
pub mod phase_lut;
pub mod piece_ring;
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
