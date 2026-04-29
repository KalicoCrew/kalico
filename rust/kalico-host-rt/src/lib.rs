//! Kalico host runtime — Step-6 substrate. Spec §2.1 component layout.
//!
//! Plan-decision C (Round-3-corrected): Step 6 ships a minimal [`host_io`]
//! shim (connect/identify/send/recv-with-timeout) implementing
//! [`transport::Transport`]. Production-grade hardening (NAK retransmit,
//! async event dispatch thread, reconnect race recovery) is Step 7 MVP
//! work.
//!
//! The new modules consume the [`transport::Transport`] trait so they can
//! run on top of either the Step-6 minimal [`host_io::KalicoHostIo`] or a
//! future test harness backed by `tools/kalico_host_io.py`.

pub mod clock_sync;
pub mod credit;
pub mod fault;
pub mod host_io;
pub mod producer;
pub mod stream;
pub mod transport;
pub mod wire;
