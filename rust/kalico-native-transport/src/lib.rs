//! Kalico-native transport (host side).
//!
//! Implements Phase A of the kalico-native transport design
//! (`docs/superpowers/specs/2026-05-04-kalico-native-transport-design.md`):
//!
//! * Layer-1 frame envelope encode / decode (§4).
//! * Stream-level demux state machine (§6) — splits a single byte stream into
//!   complete Klipper frames and complete kalico frames.
//! * Bootstrap-ABI Identify / `IdentifyResponse` hand codecs (§5).
//! * `Transport` trait + `KalicoNativeTransport<C: Connection>` impl with
//!   reset-epoch state machine (§9).
//! * Connection abstraction (Layer 0 stub) so tests can drive the transport
//!   over an in-memory pipe.
//!
//! Schema-level types (`MessageKind`, `PROTO_VERSION`, `SCHEMA_HASH`,
//! `Identify`/`IdentifyResponse`) are re-exported from `kalico-protocol`.
//! Wire-level helpers (per-message header encode/decode, `StatusEvent`
//! field accessors used during demux) live in [`wire_helpers`].

pub mod bootstrap;
pub mod connection;
pub mod demux;
pub mod frame;
pub mod frame_source;
pub mod transport;
pub mod wire_helpers;

pub use bootstrap::{
    BOOTSTRAP_IDENTIFY_LEN, BOOTSTRAP_IDENTIFY_RESPONSE_LEN, IdentifyResponse,
    decode_identify_response, encode_identify,
};
pub use connection::{Connection, MockConnection};
pub use demux::{Demuxer, Frame, KlipperFrame, PollOutcome, StreamError};
pub use frame::{
    CHANNEL_CONTROL, CHANNEL_EVENTS, FRAME_SYNC, FrameError, decode_frame, encode_frame,
};
pub use frame_source::{FrameSource, FrameSourceError};
pub use kalico_protocol::{MessageKind, PROTO_VERSION, SCHEMA_HASH};
pub use transport::{
    ConnectionState, EpochChange, EventMessage, KalicoNativeTransport, Transport, TransportError,
};
