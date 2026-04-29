//! G5 line emission helpers for the compat layer output.

use std::fmt;
use std::io::{self, Write};

/// A fully-resolved G5 move ready to be written to the output file.
#[derive(Debug, Clone)]
pub struct G5Line {
    /// Target X coordinate (absolute, mm).
    pub x: f64,
    /// Target Y coordinate (absolute, mm).
    pub y: f64,
    /// Target Z coordinate (absolute, mm).
    pub z: f64,
    /// Control-point 1 X offset from current position (I word).
    pub i: f64,
    /// Control-point 1 Y offset from current position (J word).
    pub j: f64,
    /// Control-point 2 X offset from target position (P word).
    pub p: f64,
    /// Control-point 2 Y offset from target position (Q word).
    pub q: f64,
    /// Output-side E position (absolute, mm).
    pub e: f64,
    /// Feed rate in mm/min, emitted only when it changes.
    pub f: Option<f64>,
}

impl fmt::Display for G5Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "G5 X{:.3} Y{:.3} Z{:.3} I{:.3} J{:.3} P{:.3} Q{:.3} E{:.5}",
            self.x, self.y, self.z, self.i, self.j, self.p, self.q, self.e
        )?;
        if let Some(feed) = self.f {
            write!(f, " F{feed}")?;
        }
        Ok(())
    }
}

/// Write the standard preamble to the output writer.
///
/// The preamble includes:
/// - A header comment identifying the file as compat-layer output.
/// - `G90` (absolute XYZ coordinates).
/// - `M82` (absolute E coordinates).
/// - `G17` (XY active plane).
pub fn write_preamble(
    w: &mut dyn Write,
    input_name: &str,
    tolerance_um: f64,
) -> io::Result<()> {
    writeln!(w, "; kalico-compat output")?;
    writeln!(w, "; source: {input_name}")?;
    writeln!(w, "; arc-to-bezier tolerance: {tolerance_um:.1} um")?;
    writeln!(w, "G90   ; absolute XYZ")?;
    writeln!(w, "M82   ; absolute E")?;
    writeln!(w, "G17   ; XY plane")?;
    Ok(())
}
