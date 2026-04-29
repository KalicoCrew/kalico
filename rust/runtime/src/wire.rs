//! Versioned blob payload format. Per spec §4.2.
//!
//! Every kalico-native blob payload carried inside Klipper msgproto's `%*s`
//! is prefixed with a 1-byte format-version field, followed by the binary
//! struct in little-endian. Future schema evolution: bump version, MCU
//! rejects unknown.

use crate::error::FaultCode;

/// First (and only) protocol version. Step-6 v1.
pub const FORMAT_VERSION_V1: u8 = 0x01;

/// Minimum blob length: 1 byte (the version field).
pub const MIN_BLOB_HEADER_LEN: usize = 1;

/// Validate the leading version byte. Returns
/// `Err(FaultCode::ProtocolVersionUnsupported)` for an empty blob or an
/// unknown version code.
pub fn check_version(blob: &[u8]) -> Result<(), FaultCode> {
    if blob.len() < MIN_BLOB_HEADER_LEN {
        return Err(FaultCode::ProtocolVersionUnsupported);
    }
    match blob.first().copied() {
        Some(FORMAT_VERSION_V1) => Ok(()),
        _ => Err(FaultCode::ProtocolVersionUnsupported),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn version_check_v1_passes() {
        let payload = [0x01_u8, 0x02, 0x03];
        assert!(check_version(&payload).is_ok());
    }

    #[test]
    fn version_check_unknown_rejects() {
        let payload = [0xFF_u8, 0x02, 0x03];
        assert_eq!(
            check_version(&payload),
            Err(FaultCode::ProtocolVersionUnsupported)
        );
    }

    #[test]
    fn version_check_empty_rejects() {
        let payload: [u8; 0] = [];
        assert_eq!(
            check_version(&payload),
            Err(FaultCode::ProtocolVersionUnsupported)
        );
    }

    #[test]
    fn version_v1_bare_minimum_one_byte() {
        let payload = [FORMAT_VERSION_V1];
        assert!(check_version(&payload).is_ok());
    }
}
