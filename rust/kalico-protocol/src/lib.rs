#![forbid(unsafe_code)]

pub mod bootstrap;
pub mod codec;
pub mod messages;

pub use bootstrap::{Identify, IdentifyResponse};
pub use codec::{Decode, DecodeError, Encode};
pub use messages::{
    FaultEvent, McuLog, MessageKind, PushPieces, PushPiecesResponse, RuntimeCapsResponse,
    StatusHeartbeat,
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
mod tests {
    use super::result_codes;

    #[test]
    fn result_codes_are_stable() {
        assert_eq!(result_codes::OK, 0);
        assert_eq!(result_codes::RING_FULL, -309); // KALICO_ERR_RING_FULL (error.rs:126)
        assert_eq!(result_codes::INVALID_ARG, -26); // KALICO_ERR_INVALID_ARG (error.rs:92)
        assert_ne!(result_codes::RING_FULL, result_codes::INVALID_ARG);
    }
}
