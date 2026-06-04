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
/// [u8 num_weights=0]           // always 0; rational weights removed
/// [num_cps × (3 × f32_le)]   // (x, y, z) control points
/// [num_knots × f32_le]       // knot vector
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
) -> Result<Vec<u8>, WireError> {
    let num_cps = u8::try_from(cps.len()).map_err(|_| WireError::CountOverflow {
        field: "num_cps",
        len: cps.len(),
    })?;
    let num_knots = u8::try_from(knots.len()).map_err(|_| WireError::CountOverflow {
        field: "num_knots",
        len: knots.len(),
    })?;
    let mut out = Vec::with_capacity(5 + cps.len() * 12 + knots.len() * 4);
    out.push(FORMAT_VERSION_V1);
    out.push(degree);
    out.push(num_cps);
    out.push(num_knots);
    out.push(0); // num_weights — always 0 for polynomial curves
    for cp in cps {
        for &v in cp {
            out.extend_from_slice(&v.to_le_bytes());
        }
    }
    for &k in knots {
        out.extend_from_slice(&k.to_le_bytes());
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
mod tests;
