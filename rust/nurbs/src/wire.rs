//! Wire-format constants and shared deserialization helpers.
//! See spec §Substrate / Wire format.

pub const FORMAT_VERSION_V1: u8 = 0x01;

/// Header byte counts for each format. Each is 8 bytes total to land subsequent
/// `T[..]` regions naturally aligned for f32 and f64.
pub const SCALAR_HEADER_BYTES: usize = 8;
pub const VECTOR_HEADER_BYTES: usize = 8;
pub const ARC_LENGTH_HEADER_BYTES: usize = 8;
