//! Versioned-blob v1 wire encoder. Spec §3.2 + §4.2.
//!
//! Every kalico-versioned blob carries a 1-byte format-version prefix
//! (Phase 3.1). Step-6 only defines V1; future schema bumps add new
//! constants and decoders here.

use thiserror::Error;

/// Format-version magic for the V1 schema. Spec §3.2.
pub const FORMAT_VERSION_V1: u8 = 0x01;

/// Errors returned by the versioned-blob wire encoders.
///
/// Spec §10.1 caps each curve at 256 entries (matches `Q_N_MAX`); the
/// header counts (`num_cps`, `num_knots`, `num_weights`) are u8 on the
/// wire. If a caller passes more than 255 of any element the encoder
/// returns an error rather than silently truncating the header.
#[derive(Debug, Error)]
pub enum WireError {
    #[error("{field} has {len} entries, exceeds u8 wire limit of 255")]
    CountOverflow { field: &'static str, len: usize },
}

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
/// Returns [`WireError::CountOverflow`] if any of the variable-length
/// sections exceeds the u8 header limit (255 entries) defined by spec
/// §10.1. Prior to this change the encoder relied on a `debug_assert!`
/// that was stripped in release builds; an over-cap input silently
/// truncated the header byte and produced a malformed blob.
pub fn encode_load_curve_v1(
    degree: u8,
    cps: &[[f32; 3]],
    knots: &[f32],
    weights: &[f32],
) -> Result<Vec<u8>, WireError> {
    let num_cps = u8::try_from(cps.len()).map_err(|_| WireError::CountOverflow {
        field: "num_cps",
        len: cps.len(),
    })?;
    let num_knots = u8::try_from(knots.len()).map_err(|_| WireError::CountOverflow {
        field: "num_knots",
        len: knots.len(),
    })?;
    let num_weights = u8::try_from(weights.len()).map_err(|_| WireError::CountOverflow {
        field: "num_weights",
        len: weights.len(),
    })?;
    let mut out = Vec::with_capacity(5 + cps.len() * 12 + knots.len() * 4 + weights.len() * 4);
    out.push(FORMAT_VERSION_V1);
    out.push(degree);
    out.push(num_cps);
    out.push(num_knots);
    out.push(num_weights);
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
    Ok(out)
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
///
/// See [`encode_load_curve_v1`] for the rationale behind the fallible
/// signature (silent header truncation was possible in release builds).
pub fn encode_load_curve_scalar(
    degree: u8,
    knots: &[f32],
    cps: &[f32],
) -> Result<Vec<u8>, WireError> {
    let num_cps = u8::try_from(cps.len()).map_err(|_| WireError::CountOverflow {
        field: "num_cps",
        len: cps.len(),
    })?;
    let num_knots = u8::try_from(knots.len()).map_err(|_| WireError::CountOverflow {
        field: "num_knots",
        len: knots.len(),
    })?;
    let mut out = Vec::with_capacity(5 + cps.len() * 4 + knots.len() * 4);
    out.push(FORMAT_VERSION_V1);
    out.push(degree);
    out.push(num_cps);
    out.push(num_knots);
    out.push(0); // num_weights — always 0 for polynomial scalar curves
    for &v in cps {
        out.extend_from_slice(&v.to_le_bytes());
    }
    for &k in knots {
        out.extend_from_slice(&k.to_le_bytes());
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn scalar_encoder_header_and_length() {
        let knots = [0.0_f32, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        let cps = [0.0_f32, 3.33, 6.67, 10.0];
        let blob = encode_load_curve_scalar(3, &knots, &cps).unwrap();
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
        let blob = encode_load_curve_scalar(0, &knots, &cps).unwrap();
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
        let blob = encode_load_curve_v1(1, &cps, &knots, &weights).unwrap();
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
        let blob = encode_load_curve_v1(0, &cps, &knots, &weights).unwrap();
        // 1.5_f32 little-endian = [0x00, 0x00, 0xC0, 0x3F].
        assert_eq!(&blob[5..9], &[0x00, 0x00, 0xC0, 0x3F]);
    }

    #[test]
    fn count_overflow_returns_error_in_release() {
        // 256 entries trips the u8 header limit. This used to silently
        // truncate the header byte (debug_assert stripped in release).
        let cps = vec![[0.0_f32; 3]; 256];
        let knots = [0.0_f32, 1.0];
        let weights = [1.0_f32];
        let err = encode_load_curve_v1(0, &cps, &knots, &weights).unwrap_err();
        assert!(matches!(
            err,
            WireError::CountOverflow {
                field: "num_cps",
                len: 256
            }
        ));

        let cps_scalar = vec![0.0_f32; 256];
        let err = encode_load_curve_scalar(0, &knots, &cps_scalar).unwrap_err();
        assert!(matches!(
            err,
            WireError::CountOverflow {
                field: "num_cps",
                len: 256
            }
        ));
    }
}
