use thiserror::Error;

pub const FORMAT_VERSION_V1: u8 = 0x01;

#[derive(Debug, Error)]
pub enum WireError {
    #[error("{field} has {len} entries, exceeds u8 wire limit of 255")]
    CountOverflow { field: &'static str, len: usize },
}

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
