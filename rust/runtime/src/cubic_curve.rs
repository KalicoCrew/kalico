//! Cubic curve wire-format stub — Task 5 placeholder.
//!
//! The real `cubic_curve` module (Bernstein evaluation, piece loading,
//! `LoadedCubicCurve`) has been removed together with `curve_pool`. This stub
//! retains only `WirePiece` so `kalico-c-api`'s `runtime_handle_load_curve_cubic`
//! FFI shim compiles until Task 6 replaces the entire curve-load path.

/// One piece of a cubic Bézier curve as encoded on the wire.
///
/// Five `u32` words (20 bytes) in little-endian order:
/// - `bp0_bits`–`bp3_bits`: the four Bernstein control points, each an `f32` bit pattern.
/// - `duration_bits`: piece duration in seconds, `f32` bit pattern.
///
/// `#[repr(C)]` and `Copy` so the C-side blob decoder can alias the same
/// memory layout without reinterpretation.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WirePiece {
    pub bp0_bits: u32,
    pub bp1_bits: u32,
    pub bp2_bits: u32,
    pub bp3_bits: u32,
    pub duration_bits: u32,
}
