//! Kalico-native transport schema (Layer 4).
//!
//! Pure schema + encoding. No I/O, no framing (the framing layer lives in
//! the `kalico-native-transport` crate, per spec §12). No dependencies on
//! other kalico crates — this crate is foundational.
//!
//! See `docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md`
//! for the wire contract. Sections referenced inline.

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

// Generated at build time. Provides:
//   pub const SCHEMA_HASH: [u8; 32];
//   pub const SCHEMA_HASH_HEX: &str;
//   pub const SCHEMA_CANONICAL: &str;
include!(concat!(env!("OUT_DIR"), "/schema_hash.rs"));

/// Wire-level protocol version, per spec §5. Frozen forever at the
/// bootstrap layer; cross-version migration would be a new spec.
pub const PROTO_VERSION: u8 = 0x01;

/// Channel discriminators (frame header byte 3), per spec §6.
///
/// Mirrors the `KALICO_CHANNEL_*` #defines in `src/kalico_dispatch.c`.
/// Keep the two in sync — a renumber on one side without the other silently
/// desyncs the wire contract.
pub const KALICO_CHANNEL_CONTROL: u8 = 0x00;
pub const KALICO_CHANNEL_EVENTS: u8 = 0x01;
pub const KALICO_CHANNEL_PIECES: u8 = 0x02;

/// `PushPiecesResponse.result` codes, shared host <-> MCU. These mirror the
/// `KALICO_ERR_*` values the C side returns from `handle_push_pieces`
/// (`src/kalico_dispatch.c`). Keep the two in sync — a renumber on one side
/// without the other silently desyncs the wire contract.
///
/// Canonical numeric values are defined in `rust/runtime/src/error.rs`.
pub mod result_codes {
    /// Success.
    pub const OK: i32 = 0;
    /// Ring is full — `PushPieces` rejected because the axis ring has no space.
    /// Mirrors `KALICO_ERR_RING_FULL` (`-309`).
    pub const RING_FULL: i32 = -309;
    /// Bad argument — returned for two distinct rejection reasons:
    /// (1) `axis_idx` is out of range or the axis is not yet configured;
    /// (2) `pieces_len != piece_count * 32` (framing mismatch — the byte
    /// buffer length does not match the declared piece count).
    /// Mirrors `KALICO_ERR_INVALID_ARG` (`-26`). See `runtime_ffi.rs:1466`.
    pub const INVALID_ARG: i32 = -26;
}

/// Per-message header size in bytes (`type:u16 + version:u8 + correlation_id:u32`),
/// per spec §7.2. The header is the transport crate's responsibility; this
/// constant is exposed here so encoders/decoders on the transport side can
/// reserve the right amount of space.
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
