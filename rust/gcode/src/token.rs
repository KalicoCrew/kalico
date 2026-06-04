use crate::marker::MarkerKind;

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Params {
    words: [Option<f64>; 26],
}

impl Params {
    #[must_use]
    pub fn get(&self, letter: u8) -> Option<f64> {
        if letter.is_ascii_uppercase() {
            self.words[(letter - b'A') as usize]
        } else {
            None
        }
    }

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
