use std::fmt;
use std::io::{self, Write};

#[derive(Debug, Clone)]
pub struct G5Line {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub i: f64,
    pub j: f64,
    pub p: f64,
    pub q: f64,
    pub e: f64,
    pub f: Option<f64>,
}

impl fmt::Display for G5Line {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "G5 X{:.4} Y{:.4} Z{:.4} I{:.4} J{:.4} P{:.4} Q{:.4} E{:.5}",
            self.x, self.y, self.z, self.i, self.j, self.p, self.q, self.e
        )?;
        if let Some(feed) = self.f {
            write!(f, " F{feed}")?;
        }
        Ok(())
    }
}

pub fn write_preamble(w: &mut dyn Write, input_name: &str, tolerance_um: f64) -> io::Result<()> {
    writeln!(w, "; kalico-compat output")?;
    writeln!(w, "; source: {input_name}")?;
    writeln!(w, "; arc-to-bezier tolerance: {tolerance_um:.1} um")?;
    writeln!(w, "G90   ; absolute XYZ")?;
    writeln!(w, "M82   ; absolute E")?;
    writeln!(w, "G17   ; XY plane")?;
    Ok(())
}
