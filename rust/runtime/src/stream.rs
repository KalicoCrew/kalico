//! Stream lifecycle state machine (host + MCU side). Spec §8.
//!
//! Phase 1 lands the bare enum so `FgState::stream_state_machine` has a
//! type to point at; Phase 6 fleshes out the transition rules and the
//! handlers for `kalico_stream_open` / `kalico_stream_arm` / `kalico_stream_terminal`.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FgStreamState {
    Idle = 0,
    StreamOpening = 1,
    StreamOpenPriming = 2,
    Arming = 3,
    Armed = 4,
    Running = 5,
    Draining = 6,
    Drained = 7,
    Fault = 8,
}
