//! Token types: `Command`, `Comment`, `Marker` plus the `Params` slot vector.

use crate::marker::MarkerKind;

/// Parameter words for a single G-code line, indexed by uppercase ASCII letter.
/// `Params::get(b'X')` returns `Some(value)` if the line had `X<value>`.
///
/// Stored as `[Option<f64>; 26]` for O(1) access and zero allocations.
/// `Option<f64>` is 16 bytes (no niche on f64), so this array is 416 bytes;
/// tokens stream through and don't accumulate.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Params {
    words: [Option<f64>; 26],
}

impl Params {
    /// Look up a parameter by its uppercase letter byte.
    /// Returns `None` for non-letter bytes or unset parameters.
    #[must_use]
    pub fn get(&self, letter: u8) -> Option<f64> {
        if letter.is_ascii_uppercase() {
            self.words[(letter - b'A') as usize]
        } else {
            None
        }
    }

    /// Set a parameter by its uppercase letter byte. No-op for non-letter bytes.
    pub fn set(&mut self, letter: u8, value: f64) {
        if letter.is_ascii_uppercase() {
            self.words[(letter - b'A') as usize] = Some(value);
        }
    }

    #[must_use]
    pub fn x(&self) -> Option<f64> {
        self.get(b'X')
    }
    #[must_use]
    pub fn y(&self) -> Option<f64> {
        self.get(b'Y')
    }
    #[must_use]
    pub fn z(&self) -> Option<f64> {
        self.get(b'Z')
    }
    #[must_use]
    pub fn e(&self) -> Option<f64> {
        self.get(b'E')
    }
    #[must_use]
    pub fn f(&self) -> Option<f64> {
        self.get(b'F')
    }
    #[must_use]
    pub fn i(&self) -> Option<f64> {
        self.get(b'I')
    }
    #[must_use]
    pub fn j(&self) -> Option<f64> {
        self.get(b'J')
    }
    #[must_use]
    pub fn r(&self) -> Option<f64> {
        self.get(b'R')
    }
    #[must_use]
    pub fn p(&self) -> Option<f64> {
        self.get(b'P')
    }
    #[must_use]
    pub fn q(&self) -> Option<f64> {
        self.get(b'Q')
    }
}

/// A single tokenized G-code line.
///
/// `Command` covers G/M/T words with optional decimal (e.g. G5.1 → minor=Some(1)).
/// `Comment` carries verbatim text for unrecognized comments.
/// `Marker` carries slicer-dialect-recognized comment markers (layer changes, etc.).
#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
#[allow(clippy::large_enum_variant)]
pub enum Token {
    Command {
        letter: u8,
        major: u32,
        minor: Option<u32>,
        params: Params,
        line_no: u32,
    },
    Comment {
        text: Box<str>,
        line_no: u32,
    },
    Marker {
        kind: MarkerKind,
        line_no: u32,
    },
}

#[cfg(test)]
mod tests;
