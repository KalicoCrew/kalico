//! Versioned-blob v1 wire encoder. Spec §3.2 + §4.2.
//!
//! Every kalico-versioned blob carries a 1-byte format-version prefix
//! (Phase 3.1). Step-6 only defines V1; future schema bumps add new
//! constants and decoders here.

/// Format-version magic for the V1 schema. Spec §3.2.
pub const FORMAT_VERSION_V1: u8 = 0x01;

/// Encode a `kalico_load_curve` blob payload (V1).
///
/// Wire layout:
///
/// ```text
/// [u8 format_version=0x01]
/// [u8 degree]
/// [u8 num_cps]
/// [u8 num_knots]
/// [u8 num_weights]
/// [num_cps × (3 × f32_le)]   // (x, y, z) control points
/// [num_knots × f32_le]       // knot vector
/// [num_weights × f32_le]     // per-cp weights
/// ```
///
/// Counts are u8 because spec §10.1 caps each curve at 256 entries
/// (matches `Q_N_MAX`); host callers must enforce the cap before
/// invoking this encoder.
pub fn encode_load_curve_v1(
    degree: u8,
    cps: &[[f32; 3]],
    knots: &[f32],
    weights: &[f32],
) -> Vec<u8> {
    debug_assert!(u8::try_from(cps.len()).is_ok());
    debug_assert!(u8::try_from(knots.len()).is_ok());
    debug_assert!(u8::try_from(weights.len()).is_ok());
    let mut out = Vec::with_capacity(5 + cps.len() * 12 + knots.len() * 4 + weights.len() * 4);
    out.push(FORMAT_VERSION_V1);
    out.push(degree);
    out.push(cps.len() as u8);
    out.push(knots.len() as u8);
    out.push(weights.len() as u8);
    for cp in cps {
        for &v in cp {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    for &k in knots {
        out.extend_from_slice(&k.to_le_bytes());
    }
    for &w in weights {
        out.extend_from_slice(&w.to_le_bytes());
    }
    out
}

/// Encode a `kalico_load_curve` blob for per-axis scalar curves (Step 7-B+).
///
/// Wire layout (V1 format, scalar variant):
///
/// ```text
/// [u8 format_version=0x01]
/// [u8 degree]
/// [u8 num_cps]
/// [u8 num_knots]
/// [u8 num_weights=0]
/// [num_cps × f32_le]   // scalar control points
/// [num_knots × f32_le] // knot vector
/// ```
pub fn encode_load_curve_scalar(degree: u8, knots: &[f32], cps: &[f32]) -> Vec<u8> {
    debug_assert!(u8::try_from(cps.len()).is_ok());
    debug_assert!(u8::try_from(knots.len()).is_ok());
    let mut out = Vec::with_capacity(5 + cps.len() * 4 + knots.len() * 4);
    out.push(FORMAT_VERSION_V1);
    out.push(degree);
    out.push(cps.len() as u8);
    out.push(knots.len() as u8);
    out.push(0); // num_weights — always 0 for polynomial scalar curves
    for &v in cps {
        out.extend_from_slice(&v.to_le_bytes());
    }
    for &k in knots {
        out.extend_from_slice(&k.to_le_bytes());
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_encoder_header_and_length() {
        let knots = [0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let cps = [0.0_f32, 3.33, 6.67, 10.0];
        let blob = encode_load_curve_scalar(3, &knots, &cps);
        assert_eq!(blob[0], FORMAT_VERSION_V1);
        assert_eq!(blob[1], 3, "degree");
        assert_eq!(blob[2], 4, "num_cps");
        assert_eq!(blob[3], 8, "num_knots");
        assert_eq!(blob[4], 0, "num_weights (always 0 for scalar)");
        assert_eq!(blob.len(), 53);
    }

    #[test]
    fn scalar_encoder_values_are_le() {
        let knots = [0.0_f32, 1.0];
        let cps = [1.5_f32];
        let blob = encode_load_curve_scalar(0, &knots, &cps);
        let cp_bytes: [u8; 4] = blob[5..9].try_into().unwrap();
        assert_eq!(f32::from_le_bytes(cp_bytes), 1.5);
        let k0_bytes: [u8; 4] = blob[9..13].try_into().unwrap();
        assert_eq!(f32::from_le_bytes(k0_bytes), 0.0);
    }

    #[test]
    fn header_and_length_are_correct() {
        let cps: [[f32; 3]; 2] = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]];
        let knots = [0.0_f32, 0.0, 1.0, 1.0];
        let weights = [1.0_f32, 1.0];
        let blob = encode_load_curve_v1(1, &cps, &knots, &weights);
        assert_eq!(blob[0], FORMAT_VERSION_V1);
        assert_eq!(blob[1], 1, "degree");
        assert_eq!(blob[2], 2, "num_cps");
        assert_eq!(blob[3], 4, "num_knots");
        assert_eq!(blob[4], 2, "num_weights");
        // 5-byte header + 24 cp-bytes + 16 knot-bytes + 8 weight-bytes.
        assert_eq!(blob.len(), 5 + 24 + 16 + 8);
    }

    #[test]
    fn encodes_floats_little_endian() {
        let cps = [[1.5_f32, 0.0, 0.0]];
        let knots = [0.0_f32];
        let weights = [1.0_f32];
        let blob = encode_load_curve_v1(0, &cps, &knots, &weights);
        // 1.5_f32 little-endian = [0x00, 0x00, 0xC0, 0x3F].
        assert_eq!(&blob[5..9], &[0x00, 0x00, 0xC0, 0x3F]);
    }
}
