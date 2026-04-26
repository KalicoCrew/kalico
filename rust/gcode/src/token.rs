//! Token types: `Command`, `Comment`, `Marker` plus the `Params` slot vector.

use crate::marker::MarkerKind;

/// Parameter words for a single G-code line, indexed by uppercase ASCII letter.
/// `Params::get(b'X')` returns `Some(value)` if the line had `X<value>`.
///
/// Stored as `[Option<f64>; 26]` for O(1) access and zero allocations. 208 bytes
/// per `Params`; tokens stream through and don't accumulate.
#[derive(Debug, Clone, PartialEq)]
pub struct Params {
    words: [Option<f64>; 26],
}

#[allow(clippy::derivable_impls)]
impl Default for Params {
    fn default() -> Self {
        Self { words: [None; 26] }
    }
}

impl Params {
    /// Look up a parameter by its uppercase letter byte.
    /// Returns `None` for non-letter bytes or unset parameters.
    #[must_use]
    #[allow(clippy::manual_is_ascii_check)]
    pub fn get(&self, letter: u8) -> Option<f64> {
        if (b'A'..=b'Z').contains(&letter) {
            self.words[(letter - b'A') as usize]
        } else {
            None
        }
    }

    /// Set a parameter by its uppercase letter byte. No-op for non-letter bytes.
    #[allow(clippy::manual_is_ascii_check)]
    pub fn set(&mut self, letter: u8, value: f64) {
        if (b'A'..=b'Z').contains(&letter) {
            self.words[(letter - b'A') as usize] = Some(value);
        }
    }

    #[must_use]
    pub fn x(&self) -> Option<f64> { self.get(b'X') }
    #[must_use]
    pub fn y(&self) -> Option<f64> { self.get(b'Y') }
    #[must_use]
    pub fn z(&self) -> Option<f64> { self.get(b'Z') }
    #[must_use]
    pub fn e(&self) -> Option<f64> { self.get(b'E') }
    #[must_use]
    pub fn f(&self) -> Option<f64> { self.get(b'F') }
    #[must_use]
    pub fn i(&self) -> Option<f64> { self.get(b'I') }
    #[must_use]
    pub fn j(&self) -> Option<f64> { self.get(b'J') }
    #[must_use]
    pub fn r(&self) -> Option<f64> { self.get(b'R') }
    #[must_use]
    pub fn p(&self) -> Option<f64> { self.get(b'P') }
    #[must_use]
    pub fn q(&self) -> Option<f64> { self.get(b'Q') }
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
mod tests {
    use super::*;

    #[test]
    fn params_indexed_by_letter() {
        let mut p = Params::default();
        p.set(b'X', 1.5);
        p.set(b'Y', -2.0);
        assert_eq!(p.get(b'X'), Some(1.5));
        assert_eq!(p.get(b'Y'), Some(-2.0));
        assert_eq!(p.get(b'Z'), None);
    }

    #[test]
    fn token_command_round_trip() {
        let mut params = Params::default();
        params.set(b'X', 10.0);
        let t = Token::Command {
            letter: b'G',
            major: 1,
            minor: None,
            params,
            line_no: 42,
        };
        match t {
            Token::Command { letter, major, line_no, .. } => {
                assert_eq!(letter, b'G');
                assert_eq!(major, 1);
                assert_eq!(line_no, 42);
            }
            _ => panic!("expected Command"),
        }
    }

    #[test]
    fn marker_kind_layer_change() {
        let m = crate::marker::MarkerKind::LayerChange { layer: 5 };
        match m {
            crate::marker::MarkerKind::LayerChange { layer } => assert_eq!(layer, 5),
            _ => panic!("expected LayerChange"),
        }
    }
}
