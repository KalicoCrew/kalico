pub const FORMAT_VERSION_V1: u8 = 0x01;

// 8-byte headers align subsequent T[..] for both f32 and f64.
pub const SCALAR_HEADER_BYTES: usize = 8;
pub const VECTOR_HEADER_BYTES: usize = 8;
pub const ARC_LENGTH_HEADER_BYTES: usize = 8;
