#![forbid(unsafe_code)]

pub mod bootstrap;
pub mod codec;
pub mod messages;

pub use bootstrap::{Identify, IdentifyResponse};
pub use codec::{Decode, DecodeError, Encode};
pub use messages::{
    ClaimHandshakeReply, FaultEvent, McuLog, MessageKind, PushPieces, PushPiecesResponse,
    RuntimeCapsResponse, SlaveState, SlaveStatus, StatusHeartbeat,
};

include!(concat!(env!("OUT_DIR"), "/schema_hash.rs"));

pub const PROTO_VERSION: u8 = 0x01;

// Channel discriminators mirror KALICO_CHANNEL_* in src/kalico_dispatch.c — keep in sync.
pub const KALICO_CHANNEL_CONTROL: u8 = 0x00;
pub const KALICO_CHANNEL_EVENTS: u8 = 0x01;
pub const KALICO_CHANNEL_PIECES: u8 = 0x02;

// result_codes mirror KALICO_ERR_* in src/kalico_dispatch.c — keep in sync.
// Canonical numeric values are defined in rust/runtime/src/error.rs.
pub mod result_codes {
    pub const OK: i32 = 0;
    pub const RING_FULL: i32 = -309;
    pub const INVALID_ARG: i32 = -26;
}

pub const PER_MESSAGE_HEADER_LEN: usize = 7;

#[cfg(test)]
mod tests;
