//! `ParseError`: errors that can arise during tokenization.
//!
//! These are returned from the lexer's iterator items as `Err(ParseError)`.
//! `geometry::reduce` translates persistent parse errors into
//! `Recovery::MalformedParams` events. Most lexer errors are localizable to a
//! single line and don't terminate iteration.

use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ParseError {
    #[error("line {line_no}: malformed number in `{text}`")]
    MalformedNumber { line_no: u32, text: Box<str> },

    #[error("line {line_no}: unrecognized head `{head}`")]
    UnrecognizedHead { line_no: u32, head: Box<str> },

    #[error("line {line_no}: empty command (no head letter)")]
    EmptyCommand { line_no: u32 },

    #[error("line {line_no}: parameter `{letter}` appears more than once")]
    DuplicateParam { line_no: u32, letter: char },
}

#[cfg(test)]
mod tests;
