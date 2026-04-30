//! `kalico-compat` — offline G-code compatibility layer.
//!
//! Reads legacy G-code (G0/G1/G2/G3/G5.1) and writes G5-only output
//! consumable by the kalico live pipeline.

use std::fs::File;
use std::io::{self, BufWriter, Read, Write};
use std::path::PathBuf;
use std::process::ExitCode;

use clap::Parser;

/// Offline G-code compatibility layer: convert legacy G0/G1/G2/G3/G5.1 to G5-only output.
#[derive(Debug, Parser)]
#[command(name = "kalico-compat", version)]
struct Cli {
    /// Input G-code file (use `-` for stdin).
    input: String,

    /// Output file path (default: stdout).
    #[arg(short = 'o', long = "output", value_name = "PATH")]
    output: Option<PathBuf>,

    /// Arc-to-Bézier approximation tolerance in micrometres.
    #[arg(long = "tolerance", default_value_t = 5.0, value_name = "UM")]
    tolerance: f64,
}

fn main() -> ExitCode {
    let cli = Cli::parse();

    // Read input.
    let input_name = cli.input.clone();
    let input_text = match read_input(&cli.input) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("kalico-compat: error reading {}: {e}", cli.input);
            return ExitCode::FAILURE;
        }
    };

    // Convert first, write output only on success — avoids truncating an
    // existing output file before discovering a fatal input error.
    let output_text = match compat::converter::convert(&input_text, &input_name, cli.tolerance) {
        Ok(text) => text,
        Err(e) => {
            eprintln!("kalico-compat: {e}");
            return ExitCode::from(2);
        }
    };

    let mut out: Box<dyn Write> = match open_output(cli.output.as_deref()) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("kalico-compat: cannot open output: {e}");
            return ExitCode::FAILURE;
        }
    };
    out.write_all(output_text.as_bytes()).expect("write failed");
    ExitCode::SUCCESS
}

/// Read the entire input into a `String`.
fn read_input(path: &str) -> io::Result<String> {
    if path == "-" {
        let mut s = String::new();
        io::stdin().read_to_string(&mut s)?;
        Ok(s)
    } else {
        std::fs::read_to_string(path)
    }
}

/// Open an output writer — either a file or stdout.
fn open_output(path: Option<&std::path::Path>) -> io::Result<Box<dyn Write>> {
    match path {
        Some(p) => {
            let f = File::create(p)?;
            Ok(Box::new(BufWriter::new(f)))
        }
        None => Ok(Box::new(BufWriter::new(io::stdout()))),
    }
}
