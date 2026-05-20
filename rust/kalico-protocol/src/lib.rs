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
    CreditFreed, FaultEvent, LoadCurveCubic, LoadCurveResponse, MessageKind, PushSegment,
    PushSegmentResponse, StatusEvent,
};

// Generated at build time. Provides:
//   pub const SCHEMA_HASH: [u8; 32];
//   pub const SCHEMA_HASH_HEX: &str;
//   pub const SCHEMA_CANONICAL: &str;
include!(concat!(env!("OUT_DIR"), "/schema_hash.rs"));

/// Wire-level protocol version, per spec §5. Frozen forever at the
/// bootstrap layer; cross-version migration would be a new spec.
pub const PROTO_VERSION: u8 = 0x01;

/// Per-message header size in bytes (`type:u16 + version:u8 + correlation_id:u32`),
/// per spec §7.2. The header is the transport crate's responsibility; this
/// constant is exposed here so encoders/decoders on the transport side can
/// reserve the right amount of space.
pub const PER_MESSAGE_HEADER_LEN: usize = 7;
