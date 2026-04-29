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

/// Strip an inline `;`-comment from a line, returning only the command portion.
fn strip_inline_comment(line: &str) -> &str {
    match line.find(';') {
        Some(idx) => &line[..idx],
        None => line,
    }
}

/// Parse a `(major, minor)` head number like `1` → `(1, None)` or `5.1` →
/// `(5, Some(1))`.
fn parse_head_number(s: &str) -> Option<(u32, Option<u32>)> {
    if let Some((maj, min)) = s.split_once('.') {
        let major = maj.parse::<u32>().ok()?;
        let minor = min.parse::<u32>().ok()?;
        Some((major, Some(minor)))
    } else {
        Some((s.parse::<u32>().ok()?, None))
    }
}

/// Tokenize a single non-comment, non-empty trimmed line into a `Token::Command`.
fn tokenize_command_line(line: &str, line_no: u32) -> Result<Token, ParseError> {
    let mut chars = line.char_indices();

    // Read the head letter.
    let Some((_, head_char)) = chars.next() else {
        return Err(ParseError::EmptyCommand { line_no });
    };
    if !head_char.is_ascii_uppercase() {
        return Err(ParseError::UnrecognizedHead {
            line_no,
            head: line
                .split_whitespace()
                .next()
                .unwrap_or(line)
                .to_string()
                .into_boxed_str(),
        });
    }
    let head_byte = head_char as u8;

    // Find start of the remainder after the head letter.
    let after_letter_idx = chars.next().map_or(line.len(), |(i, _)| i);
    let after_letter = &line[after_letter_idx..];

    // Head number runs up to the first whitespace.
    let head_number_str = after_letter.split_whitespace().next().unwrap_or("");
    let (major, minor) =
        parse_head_number(head_number_str).ok_or_else(|| ParseError::UnrecognizedHead {
            line_no,
            head: format!("{head_char}{head_number_str}").into_boxed_str(),
        })?;

    // Parse remaining whitespace-separated tokens as `<letter><number>`.
    let mut params = crate::Params::default();
    let mut seen = [false; 26];
    let after_head_idx = after_letter_idx + head_number_str.len();

    for tok in line[after_head_idx..].split_whitespace() {
        let mut tc = tok.chars();
        let Some(letter_ch) = tc.next() else { continue };
        // Reject non-ASCII-uppercase letters consistently with the head check above.
        // Slicer output is always uppercase; tolerating lowercase silently can mask bugs.
        if !letter_ch.is_ascii_uppercase() {
            return Err(ParseError::MalformedNumber {
                line_no,
                text: tok.to_string().into_boxed_str(),
            });
        }
        let letter = letter_ch as u8;
        let num_str = &tok[letter_ch.len_utf8()..];
        let value: f64 = num_str.parse().map_err(|_| ParseError::MalformedNumber {
            line_no,
            text: tok.to_string().into_boxed_str(),
        })?;
        // Reject non-finite literals (NaN, +/-inf, infinity). Rust's
        // f64::FromStr accepts all of these. Without this guard they
        // silently propagate through reduce → pipeline into planner-visible
        // segments; worse, NaN-poisoned XY classifies as ZeroMotion (because
        // `NaN > 1e-6` is false) so the entire move is silently dropped with
        // no telemetry, and modal state.position becomes NaN-poisoned for
        // every subsequent G5. (Round-5 review fix: contain at the lexer
        // boundary — single point, applies uniformly to all parameter letters.)
        if !value.is_finite() {
            return Err(ParseError::MalformedNumber {
                line_no,
                text: tok.to_string().into_boxed_str(),
            });
        }
        let idx = (letter - b'A') as usize;
        if seen[idx] {
            return Err(ParseError::DuplicateParam {
                line_no,
                letter: letter as char,
            });
        }
        seen[idx] = true;
        params.set(letter, value);
    }

    Ok(Token::Command {
        letter: head_byte,
        major,
        minor,
        params,
        line_no,
    })
}

impl Iterator for Lexer<'_> {
    type Item = Result<Token, ParseError>;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let (idx, raw) = self.lines.next()?;
            let line_no = (idx as u32).checked_add(1).expect("line count overflow");
            let trimmed_full = raw.trim();
            if trimmed_full.is_empty() {
                continue;
            }
            if trimmed_full.starts_with(';') {
                if let Some(kind) = crate::marker::match_comment(trimmed_full) {
                    return Some(Ok(Token::Marker { kind, line_no }));
                }
                let stripped = trimmed_full.trim_start_matches(';').trim();
                return Some(Ok(Token::Comment {
                    text: stripped.to_string().into_boxed_str(),
                    line_no,
                }));
            }
            let no_inline = strip_inline_comment(trimmed_full).trim();
            if no_inline.is_empty() {
                continue;
            }
            return Some(tokenize_command_line(no_inline, line_no));
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

    #[test]
    fn parses_g1_with_xy() {
        let toks = collect("G1 X10 Y-5\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Command {
                letter,
                major,
                minor,
                params,
                line_no,
            }) => {
                assert_eq!(*letter, b'G');
                assert_eq!(*major, 1);
                assert_eq!(*minor, None);
                assert_eq!(params.x(), Some(10.0));
                assert_eq!(params.y(), Some(-5.0));
                assert_eq!(params.z(), None);
                assert_eq!(*line_no, 1);
            }
            other => panic!("expected Command, got {other:?}"),
        }
    }

    #[test]
    fn parses_g1_with_decimal_params() {
        let toks = collect("G1 X1.234 Y5.678 E0.123 F1500\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Command { params, .. }) => {
                assert_eq!(params.x(), Some(1.234));
                assert_eq!(params.y(), Some(5.678));
                assert_eq!(params.e(), Some(0.123));
                assert_eq!(params.f(), Some(1500.0));
            }
            other => panic!("expected Command, got {other:?}"),
        }
    }

    #[test]
    fn malformed_number_returns_error() {
        let toks = collect("G1 X1.2.3\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Err(ParseError::MalformedNumber { line_no: 1, .. }) => {}
            other => panic!("expected MalformedNumber, got {other:?}"),
        }
    }

    #[test]
    fn duplicate_param_returns_error() {
        let toks = collect("G1 X1 X2\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Err(ParseError::DuplicateParam {
                line_no: 1,
                letter: 'X',
            }) => {}
            other => panic!("expected DuplicateParam, got {other:?}"),
        }
    }

    #[test]
    fn inline_comment_is_stripped() {
        let toks = collect("G1 X1.0 Y2.0 ; trailing comment\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Command { params, .. }) => {
                assert_eq!(params.x(), Some(1.0));
                assert_eq!(params.y(), Some(2.0));
            }
            other => panic!("expected Command, got {other:?}"),
        }
    }

    #[test]
    fn missing_param_value_returns_error() {
        // "X" with no number after it — different branch from "X1.2.3"
        let toks = collect("G1 X\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Err(ParseError::MalformedNumber { line_no: 1, .. }) => {}
            other => panic!("expected MalformedNumber for missing value, got {other:?}"),
        }
    }

    #[test]
    fn numeric_token_without_letter_returns_error() {
        // "1.0" as a parameter — first character isn't a letter
        let toks = collect("G1 1.0\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Err(ParseError::MalformedNumber { line_no: 1, .. }) => {}
            other => panic!("expected MalformedNumber for digit-leading param, got {other:?}"),
        }
    }

    #[test]
    fn lowercase_param_letter_returns_error() {
        // After Fix 1: lowercase param letters reject consistently with lowercase heads.
        let toks = collect("G1 x10\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Err(ParseError::MalformedNumber { line_no: 1, .. }) => {}
            other => panic!("expected MalformedNumber for lowercase param letter, got {other:?}"),
        }
    }

    #[test]
    fn head_with_no_number_returns_error() {
        // "G" alone — head letter without a number portion.
        let toks = collect("G\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Err(ParseError::UnrecognizedHead { line_no: 1, .. }) => {}
            other => panic!("expected UnrecognizedHead for bare head letter, got {other:?}"),
        }
    }

    #[test]
    fn parses_g5_1() {
        let toks = collect("G5.1 X10 Y20 I1 J2\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Command {
                letter,
                major,
                minor,
                ..
            }) => {
                assert_eq!(*letter, b'G');
                assert_eq!(*major, 5);
                assert_eq!(*minor, Some(1));
            }
            other => panic!("expected Command, got {other:?}"),
        }
    }

    #[test]
    fn parses_m104() {
        let toks = collect("M104 S210\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Command {
                letter,
                major,
                params,
                ..
            }) => {
                assert_eq!(*letter, b'M');
                assert_eq!(*major, 104);
                assert_eq!(params.get(b'S'), Some(210.0));
            }
            other => panic!("expected Command, got {other:?}"),
        }
    }

    #[test]
    fn parses_t0() {
        let toks = collect("T0\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Command { letter, major, .. }) => {
                assert_eq!(*letter, b'T');
                assert_eq!(*major, 0);
            }
            other => panic!("expected Command, got {other:?}"),
        }
    }

    #[test]
    fn layer_change_comment_is_marker_token() {
        let toks = collect(";LAYER:5\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Marker { kind, line_no }) => {
                assert_eq!(
                    *kind,
                    crate::marker::MarkerKind::LayerChange { layer: Some(5) }
                );
                assert_eq!(*line_no, 1);
            }
            other => panic!("expected Marker, got {other:?}"),
        }
    }
}
