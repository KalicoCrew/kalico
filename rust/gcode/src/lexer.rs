//! Lexer entry point: `lex(&str) -> impl Iterator<Item = Result<Token, ParseError>>`.

use crate::{ParseError, Token};

/// Tokenize a complete G-code buffer. Returns an iterator over per-line
/// tokenization results. Empty lines and pure-whitespace lines yield no tokens.
/// Comments yield `Token::Comment` (Task 8 will promote slicer-recognized
/// comments to `Token::Marker`).
pub fn lex(text: &str) -> Lexer<'_> {
    Lexer {
        lines: text.lines().enumerate(),
    }
}

#[derive(Debug)]
pub struct Lexer<'a> {
    lines: std::iter::Enumerate<std::str::Lines<'a>>,
}

impl Iterator for Lexer<'_> {
    type Item = Result<Token, ParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let (idx, raw) = self.lines.next()?;
            let line_no = (idx as u32).checked_add(1).expect("line count overflow");
            // Strip inline comment but capture standalone comments.
            let trimmed = raw.trim();
            if trimmed.is_empty() {
                continue;
            }
            if let Some(stripped) = trimmed.strip_prefix(';') {
                // Pure comment line.
                return Some(Ok(Token::Comment {
                    text: stripped.trim().to_string().into_boxed_str(),
                    line_no,
                }));
            }
            // Task 5/6 will handle command/parameter tokenization.
            // For now, treat any non-comment non-empty line as unrecognized so
            // we have a return path while building up the lexer in pieces.
            return Some(Err(ParseError::UnrecognizedHead {
                line_no,
                head: trimmed.to_string().into_boxed_str(),
            }));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn collect(text: &str) -> Vec<Result<Token, ParseError>> {
        lex(text).collect()
    }

    #[test]
    fn empty_input_yields_nothing() {
        assert!(collect("").is_empty());
    }

    #[test]
    fn whitespace_only_yields_nothing() {
        assert!(collect("   \n\t  \n").is_empty());
    }

    #[test]
    fn pure_comment_yields_comment_token() {
        let toks = collect("; just a comment\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Comment { text, line_no }) => {
                assert_eq!(text.as_ref(), "just a comment");
                assert_eq!(*line_no, 1);
            }
            other => panic!("expected Comment, got {other:?}"),
        }
    }

    #[test]
    fn line_numbers_are_one_indexed() {
        let toks = collect("\n\n; third line\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Comment { line_no, .. }) => assert_eq!(*line_no, 3),
            other => panic!("expected Comment, got {other:?}"),
        }
    }
}
