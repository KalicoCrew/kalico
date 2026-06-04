// WIRE-STABLE: FORMAT_VERSION_V1 == 0x01 is the only supported version.
// MCU rejects any unknown version code (ProtocolVersionUnsupported).
use crate::error::FaultCode;

pub const FORMAT_VERSION_V1: u8 = 0x01;
pub const MIN_BLOB_HEADER_LEN: usize = 1;

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
mod tests;
