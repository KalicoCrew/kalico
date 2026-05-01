//! Passthrough queue — Rust port of `serialqueue.c` core data structures.

mod command_queue;
mod config_stage;
mod entry;
mod mcu_state;
mod notify;
mod receive_window;
mod router;
mod stats;

pub use command_queue::CommandQueue;
pub use config_stage::{ConfigStage, ConfigStagePhase};
pub use entry::{NotifyId, PassthroughEntry, BACKGROUND_PRIORITY_CLOCK};
pub use mcu_state::{CommandQueueId, McuState, PushError};
pub use notify::{NotifyCallback, NotifyResponse, NotifyTable};
pub use receive_window::ReceiveWindow;
pub use router::{McuHandle, PassthroughRouter, RouterError};
pub use stats::PassthroughStats;
