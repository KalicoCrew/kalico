use crate::{ParseError, Token};

pub fn lex(text: &str) -> Lexer<'_> {
    Lexer {
        lines: text.lines().enumerate(),
    }
}

#[derive(Debug)]
pub struct Lexer<'a> {
    lines: std::iter::Enumerate<std::str::Lines<'a>>,
}

fn strip_inline_comment(line: &str) -> &str {
    match line.find(';') {
        Some(idx) => &line[..idx],
        None => line,
    }
}

fn parse_head_number(s: &str) -> Option<(u32, Option<u32>)> {
    if let Some((maj, min)) = s.split_once('.') {
        let major = maj.parse::<u32>().ok()?;
        let minor = min.parse::<u32>().ok()?;
        Some((major, Some(minor)))
    } else {
        Some((s.parse::<u32>().ok()?, None))
    }
}

fn tokenize_command_line(line: &str, line_no: u32) -> Result<Token, ParseError> {
    let mut chars = line.char_indices();

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

    let after_letter_idx = chars.next().map_or(line.len(), |(i, _)| i);
    let after_letter = &line[after_letter_idx..];

    let head_number_str = after_letter
        .find(char::is_whitespace)
        .map_or(after_letter, |ws_idx| &after_letter[..ws_idx]);
    let (major, minor) =
        parse_head_number(head_number_str).ok_or_else(|| ParseError::UnrecognizedHead {
            line_no,
            head: format!("{head_char}{head_number_str}").into_boxed_str(),
        })?;

    let mut params = crate::Params::default();
    let mut seen = [false; 26];
    let after_head_idx = after_letter_idx + head_number_str.len();

    for tok in line[after_head_idx..].split_whitespace() {
        let mut tc = tok.chars();
        let Some(letter_ch) = tc.next() else { continue };
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
mod tests;
