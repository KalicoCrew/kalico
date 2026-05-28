//! Kalico host runtime — Step-6 substrate. Spec §2.1 component layout.
//!
//! # Step-6 scope: identify-only, NOT a working command channel
//!
//! Plan-decision C (Round-3-corrected) constrains Step 6 to the bare
//! minimum required to validate the transport plumbing. The
//! [`host_io::KalicoHostIo`] shim runs the identify handshake and tracks
//! the wire seq, but the msgproto JSON dictionary is **NOT** parsed:
//! [`host_io::KalicoHostIo::send`] **always returns `Err`** after
//! identify because [`host_io`] cannot encode any command without the
//! dictionary. Step-7 MVP wires the full parser.
//!
//! Until Step 7:
//!   * Production wire I/O is **not** functional. Anything beyond the
//!     identify handshake fails fast with a `TransportError::Parse`.
//!   * Unit tests for [`producer`], [`stream`], etc. MUST drive the
//!     [`transport::Transport`] trait via `MockTransport` (see
//!     `tests/mock_transport.rs`).
//!
//! Plan-decision C also defers (Step-7 MVP work):
//!   * NAK-driven retransmit
//!   * Async event-dispatch thread
//!   * Identify-during-reconnect race recovery
//!
//! The new modules consume the [`transport::Transport`] trait so they
//! can run on top of either the Step-6 minimal [`host_io::KalicoHostIo`]
//! (post-Step-7) or a test harness today.

pub mod clock;
pub mod clock_sync;
pub mod endstop;
pub mod fault;
pub mod host_io;
pub mod passthrough_queue;
pub mod producer;
pub mod stream;
pub mod transport;
pub mod wire;
