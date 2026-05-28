//! Bootstrap ABI — `Identify` and `IdentifyResponse` (spec §5).
//!
//! **These byte layouts are frozen forever at protocol version 1.** They are
//! intentionally NOT routed through the [`Encode`]/[`Decode`] traits used for
//! schema-validated messages — those traits are an implementation detail of
//! the schema, and the bootstrap exists precisely so the host and MCU can
//! agree on `schema_hash` before trusting the schema. Any change to these
//! offsets or field types is a protocol-incompatibility break.
//!
//! Both messages still ride the framing layer (sync + len + channel + crc)
//! and carry the per-message header (`type` + `version` + `correlation_id`)
//! emitted by the framing layer. This module encodes/decodes the
//! **fixed-layout body only**, exactly as specified in §5.
//!
//! Identify body: 1 byte.
//! IdentifyResponse body: 81 bytes.

/// Spec §5: `proto_version: u8`. Single field, single byte.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Identify {
    pub proto_version: u8,
}

/// Body length in bytes. Fixed forever.
pub const IDENTIFY_BODY_LEN: usize = 1;

impl Identify {
    pub fn encode_body(&self, out: &mut Vec<u8>) {
        out.push(self.proto_version);
    }

    pub fn encode_body_to_array(&self) -> [u8; IDENTIFY_BODY_LEN] {
        [self.proto_version]
    }

    pub fn decode_body(buf: &[u8]) -> Result<Self, BootstrapDecodeError> {
        if buf.len() != IDENTIFY_BODY_LEN {
            return Err(BootstrapDecodeError::WrongLength {
                expected: IDENTIFY_BODY_LEN,
                got: buf.len(),
            });
        }
        Ok(Self {
            proto_version: buf[0],
        })
    }
}

/// Spec §5 byte layout (frozen forever):
///
/// ```text
///  0..1   proto_version : u8
///  1..5   firmware_ver  : u32_le
///  5..25  build_hash    : [u8; 20]    git commit SHA-1 (informational)
/// 25..57  schema_hash   : [u8; 32]    SHA-256 over canonicalized schema
/// 57..61  reset_epoch   : u32_le      nonzero, unique per MCU boot
/// 61..69  capabilities  : u64_le      bitmap (phase_stepping=0x1, ...)
/// 69..81  mcu_serial    : [u8; 12]    chip serial (informational)
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IdentifyResponse {
    pub proto_version: u8,
    pub firmware_ver: u32,
    pub build_hash: [u8; 20],
    pub schema_hash: [u8; 32],
    pub reset_epoch: u32,
    pub capabilities: u64,
    pub mcu_serial: [u8; 12],
}

/// Body length in bytes. Fixed forever (`81 = 1 + 4 + 20 + 32 + 4 + 8 + 12`).
pub const IDENTIFY_RESPONSE_BODY_LEN: usize = 81;

// Field offsets, frozen forever. Exposed for the C side and for tests.
pub const IDR_OFF_PROTO_VERSION: usize = 0;
pub const IDR_OFF_FIRMWARE_VER: usize = 1;
pub const IDR_OFF_BUILD_HASH: usize = 5;
pub const IDR_OFF_SCHEMA_HASH: usize = 25;
pub const IDR_OFF_RESET_EPOCH: usize = 57;
pub const IDR_OFF_CAPABILITIES: usize = 61;
pub const IDR_OFF_MCU_SERIAL: usize = 69;

impl IdentifyResponse {
    pub fn encode_body(&self, out: &mut Vec<u8>) {
        let arr = self.encode_body_to_array();
        out.extend_from_slice(&arr);
    }

    pub fn encode_body_to_array(&self) -> [u8; IDENTIFY_RESPONSE_BODY_LEN] {
        let mut b = [0u8; IDENTIFY_RESPONSE_BODY_LEN];
        b[IDR_OFF_PROTO_VERSION] = self.proto_version;
        b[IDR_OFF_FIRMWARE_VER..IDR_OFF_FIRMWARE_VER + 4]
            .copy_from_slice(&self.firmware_ver.to_le_bytes());
        b[IDR_OFF_BUILD_HASH..IDR_OFF_BUILD_HASH + 20].copy_from_slice(&self.build_hash);
        b[IDR_OFF_SCHEMA_HASH..IDR_OFF_SCHEMA_HASH + 32].copy_from_slice(&self.schema_hash);
        b[IDR_OFF_RESET_EPOCH..IDR_OFF_RESET_EPOCH + 4]
            .copy_from_slice(&self.reset_epoch.to_le_bytes());
        b[IDR_OFF_CAPABILITIES..IDR_OFF_CAPABILITIES + 8]
            .copy_from_slice(&self.capabilities.to_le_bytes());
        b[IDR_OFF_MCU_SERIAL..IDR_OFF_MCU_SERIAL + 12].copy_from_slice(&self.mcu_serial);
        b
    }

    pub fn decode_body(buf: &[u8]) -> Result<Self, BootstrapDecodeError> {
        if buf.len() != IDENTIFY_RESPONSE_BODY_LEN {
            return Err(BootstrapDecodeError::WrongLength {
                expected: IDENTIFY_RESPONSE_BODY_LEN,
                got: buf.len(),
            });
        }
        let proto_version = buf[IDR_OFF_PROTO_VERSION];
        let firmware_ver = u32::from_le_bytes(
            buf[IDR_OFF_FIRMWARE_VER..IDR_OFF_FIRMWARE_VER + 4]
                .try_into()
                .expect("range checked above"),
        );
        let mut build_hash = [0u8; 20];
        build_hash.copy_from_slice(&buf[IDR_OFF_BUILD_HASH..IDR_OFF_BUILD_HASH + 20]);
        let mut schema_hash = [0u8; 32];
        schema_hash.copy_from_slice(&buf[IDR_OFF_SCHEMA_HASH..IDR_OFF_SCHEMA_HASH + 32]);
        let reset_epoch = u32::from_le_bytes(
            buf[IDR_OFF_RESET_EPOCH..IDR_OFF_RESET_EPOCH + 4]
                .try_into()
                .expect("range checked above"),
        );
        let capabilities = u64::from_le_bytes(
            buf[IDR_OFF_CAPABILITIES..IDR_OFF_CAPABILITIES + 8]
                .try_into()
                .expect("range checked above"),
        );
        let mut mcu_serial = [0u8; 12];
        mcu_serial.copy_from_slice(&buf[IDR_OFF_MCU_SERIAL..IDR_OFF_MCU_SERIAL + 12]);
        Ok(Self {
            proto_version,
            firmware_ver,
            build_hash,
            schema_hash,
            reset_epoch,
            capabilities,
            mcu_serial,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BootstrapDecodeError {
    WrongLength { expected: usize, got: usize },
}

impl core::fmt::Display for BootstrapDecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::WrongLength { expected, got } => write!(
                f,
                "bootstrap message wrong length: expected {expected} bytes, got {got}"
            ),
        }
    }
}

impl std::error::Error for BootstrapDecodeError {}

#[cfg(test)]
mod tests;
