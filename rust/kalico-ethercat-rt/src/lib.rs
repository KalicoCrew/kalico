//! Host-side EtherCAT motion-node endpoint: decodes the kalico-native piece
//! stream and streams CSP position to an A6-EC servo over EtherCAT/DC.
pub mod clock;
pub mod curves;
/// Raw EtherCAT FFI — only compiled with the `hw` feature (needs libecrt/SOEM).
#[cfg(feature = "hw")]
pub mod ffi;
pub mod scale;
pub mod server;
pub mod wire;
