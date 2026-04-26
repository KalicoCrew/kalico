# Layer 1 Phase 1 — Foundations Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Layer 1 Phase 1 (build-order step 6): complete `rust/gcode/` parser plus minimal `rust/geometry/` pipeline that emits a degree-1 NURBS for each G1 segment, a 3D rational-quadratic NURBS for each G2/G3, and a `JunctionDeviation` at every G1 vertex. Drives the kalico MVP first-print.

**Architecture:** Per `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`. Two new workspace members. `gcode/` is a pure tokenizer with no NURBS dep. `geometry/` minimal: pipeline + segment + reduce + params + error + telemetry. **No classifier, no fitter, no corner-blend slot construction** — those land in Phase 2. Public API of `geometry/` is identical across phases; Phase 1 produces a subset of segment kinds, consumers handle the full enum from day one.

**Tech Stack:** Rust 1.85, `nurbs` Layer 0 substrate (existing), `thiserror` 2.x for error enums, `proptest` 1.x for property tests, `cargo-fuzz` for parser fuzzing.

---

## Phase 1 Scope Reminder

Phase 1 emits these `Item` variants:
- `Item::Segment(Segment::Fitted(_))` — degree-1 NURBS for each G1 line segment.
- `Item::Segment(Segment::Arc(_))` — 3D rational quadratic for each G2/G3.
- `Item::Segment(Segment::Junction(_))` — at every G1-G1 vertex transition.
- `Item::Recovered(_, Recovery::UnrecognizedCommand | MalformedParams)` — parser fallbacks.
- `Item::Fatal(_)` — invariant violations (rare).

Phase 1 emits these `TelemetryEvent` variants:
- `LayerChange`, `ToolChange`, `Retraction`, `Recovery(_)`.

Phase 1 does NOT emit (defined but never produced):
- `Item::Segment(Segment::CornerBlend(_))` — Phase 2.
- `Recovery::WindowCapHit | DegenerateSlotFallback | ToleranceExceeded | LspiaNotConverged` — Phase 2.
- `TelemetryEvent::WindowFlush | FitObservation` — Phase 2.

The full taxonomy of types is defined from Phase 1 so the public API is stable across the phase boundary.

---

## File Structure

`rust/gcode/`:
- `Cargo.toml`
- `src/lib.rs` — re-exports
- `src/lexer.rs` — `&str` → `Iterator<Item = Result<Token, ParseError>>`
- `src/token.rs` — `Token`, `MarkerKind`, `Params`
- `src/marker.rs` — slicer-dialect comment-pattern matchers
- `src/error.rs` — `ParseError`
- `fuzz/Cargo.toml` + `fuzz/fuzz_targets/lex.rs`
- `tests/golden_corpus_lex.rs`
- `tests/property_lex.rs`

`rust/geometry/`:
- `Cargo.toml`
- `src/lib.rs` — re-exports
- `src/pipeline.rs` — `GeometryPipeline`, `Segments`, `Item`
- `src/reduce.rs` — `Token` → `ReduceEvent` (pub(crate))
- `src/segment.rs` — `Segment`, `FittedSegment`, `ArcSegment`, `CornerBlendSlot`, `JunctionDeviation`, `BlendFamily`, `SourceRange`
- `src/telemetry.rs` — `TelemetryEvent`
- `src/params.rs` — `FitterParams` + `Default`
- `src/error.rs` — `Recovery`, `SlotDegeneracy`, `Fatal`, `InternalKind`, `InternalDetails`
- `tests/helical_arc_3d.rs`
- `tests/degenerate_inputs.rs`
- `tests/integration_orca.rs`

(Phase 2 will add: `src/classify.rs`, `src/fit.rs`, `src/corner_blend.rs`, `tests/vase_mode_smoke.rs`, `tests/cross_check_python.rs`.)

`SmallString` representation: **`Box<str>`**. No extra dep, small stack footprint, good enough for non-hot allocations (per spec §11). Optimize via `compact_str` if profiling later flags allocator pressure.

---

## Task 1: Workspace setup + `gcode/` scaffold

**Files:**
- Modify: `rust/Cargo.toml`
- Create: `rust/gcode/Cargo.toml`
- Create: `rust/gcode/src/lib.rs`

- [ ] **Step 1: Add `gcode` to workspace members and add `thiserror` to workspace deps**

Edit `rust/Cargo.toml`. Replace:
```toml
[workspace]
members = ["nurbs", "nurbs-c-api"]
resolver = "2"

[workspace.dependencies]
# Shared deps versioned here from day one.
```
with:
```toml
[workspace]
members = ["nurbs", "nurbs-c-api", "gcode"]
resolver = "2"

[workspace.dependencies]
# Shared deps versioned here from day one.
thiserror = "2"
```

- [ ] **Step 2: Create the `gcode` crate manifest**

Create `rust/gcode/Cargo.toml`:
```toml
[package]
name = "gcode"
version = "0.1.0"
edition = "2021"
rust-version = "1.85"
publish = false
description = "G-code lexer for the kalico motion planner. Lexically G-code-aware, motion-semantics-agnostic. See docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md."

[dependencies]
thiserror = { workspace = true }

[dev-dependencies]
proptest = "1.5"

[lints]
workspace = true
```

- [ ] **Step 3: Create the empty lib root**

Create `rust/gcode/src/lib.rs`:
```rust
//! G-code lexer. Pure text → typed tokens. No motion semantics. No NURBS.
//! See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`.

#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod error;
pub mod lexer;
pub mod marker;
pub mod token;

pub use error::ParseError;
pub use lexer::lex;
pub use marker::MarkerKind;
pub use token::{Params, Token};
```

- [ ] **Step 4: Create empty stub modules so the crate compiles**

Create `rust/gcode/src/error.rs`:
```rust
//! `ParseError`: errors that can arise during tokenization.

use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ParseError {}
```

Create `rust/gcode/src/token.rs`:
```rust
//! Token types: `Command`, `Comment`, `Marker` plus the `Params` slot vector.
```

Create `rust/gcode/src/marker.rs`:
```rust
//! Slicer-dialect comment-pattern matchers. Covers OrcaSlicer / PrusaSlicer
//! / BambuStudio.
```

Create `rust/gcode/src/lexer.rs`:
```rust
//! Lexer entry point: `lex(&str) -> impl Iterator<Item = Result<Token, ParseError>>`.
```

- [ ] **Step 5: Verify workspace compiles**

Run: `cargo build -p gcode --manifest-path rust/Cargo.toml`
Expected: succeeds with warnings (empty modules); no errors.

- [ ] **Step 6: Commit**

```bash
git add rust/Cargo.toml rust/gcode/
git commit -m "gcode: scaffold crate with empty modules"
```

---

## Task 2: Token, Params, and MarkerKind types

**Files:**
- Modify: `rust/gcode/src/token.rs`
- Modify: `rust/gcode/src/marker.rs`

- [ ] **Step 1: Write the failing tests**

Append to `rust/gcode/src/token.rs`:
```rust
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml`
Expected: FAIL — `Token`, `Params`, `MarkerKind` not yet defined.

- [ ] **Step 3: Implement `Params` and `Token`**

Replace `rust/gcode/src/token.rs` body (above the `#[cfg(test)]` block) with:
```rust
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

impl Default for Params {
    fn default() -> Self {
        Self { words: [None; 26] }
    }
}

impl Params {
    /// Look up a parameter by its uppercase letter byte.
    /// Returns `None` for non-letter bytes or unset parameters.
    #[must_use]
    pub fn get(&self, letter: u8) -> Option<f64> {
        if (b'A'..=b'Z').contains(&letter) {
            self.words[(letter - b'A') as usize]
        } else {
            None
        }
    }

    /// Set a parameter by its uppercase letter byte. No-op for non-letter bytes.
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
```

- [ ] **Step 4: Implement `MarkerKind`**

Replace `rust/gcode/src/marker.rs` body (above any test block) with:
```rust
//! Slicer-dialect comment-pattern matchers. Covers OrcaSlicer / PrusaSlicer
//! / BambuStudio. The matcher itself lands in Task 8; this module just defines
//! the type today.

#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum MarkerKind {
    /// `;LAYER:5` and equivalents.
    LayerChange { layer: u32 },
    /// `;TYPE:WALL-OUTER`, `;TYPE:INFILL`, etc.
    LayerType { name: Box<str> },
    /// `;END_OF_PRINT` and equivalents.
    EndOfPrint,
}
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml`
Expected: PASS — all three tests.

- [ ] **Step 6: Commit**

```bash
git add rust/gcode/src/token.rs rust/gcode/src/marker.rs
git commit -m "gcode: define Token, Params, and MarkerKind types"
```

---

## Task 3: ParseError taxonomy

**Files:**
- Modify: `rust/gcode/src/error.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/gcode/src/error.rs`:
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_error_displays_line_no() {
        let e = ParseError::MalformedNumber {
            line_no: 7,
            text: "G1 X1.2.3".into(),
        };
        let s = format!("{e}");
        assert!(s.contains("line 7"));
        assert!(s.contains("malformed number"));
    }

    #[test]
    fn parse_error_unrecognized_head() {
        let e = ParseError::UnrecognizedHead {
            line_no: 12,
            head: "X1".into(),
        };
        let s = format!("{e}");
        assert!(s.contains("line 12"));
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml --test '*' parse_error`
Expected: FAIL — `MalformedNumber` and `UnrecognizedHead` variants not defined.

- [ ] **Step 3: Implement the variants**

Replace `rust/gcode/src/error.rs` body (above `#[cfg(test)]`) with:
```rust
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/gcode/src/error.rs
git commit -m "gcode: define ParseError taxonomy"
```

---

## Task 4: Lexer skeleton — comments and whitespace

**Files:**
- Modify: `rust/gcode/src/lexer.rs`

- [ ] **Step 1: Write the failing tests**

Append to `rust/gcode/src/lexer.rs`:
```rust
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml lexer::tests`
Expected: FAIL — `lex` not yet defined.

- [ ] **Step 3: Implement `lex` skeleton**

Replace `rust/gcode/src/lexer.rs` body (above `#[cfg(test)]`) with:
```rust
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

pub struct Lexer<'a> {
    lines: std::iter::Enumerate<std::str::Lines<'a>>,
}

impl<'a> Iterator for Lexer<'a> {
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml lexer::tests`
Expected: PASS — all four tests.

- [ ] **Step 5: Commit**

```bash
git add rust/gcode/src/lexer.rs
git commit -m "gcode/lexer: comment and whitespace handling"
```

---

## Task 5: Lexer parameter extraction

**Files:**
- Modify: `rust/gcode/src/lexer.rs`

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `rust/gcode/src/lexer.rs`:
```rust
    #[test]
    fn parses_g1_with_xy() {
        let toks = collect("G1 X10 Y-5\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Command { letter, major, minor, params, line_no }) => {
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
            Err(ParseError::DuplicateParam { line_no: 1, letter: 'X' }) => {}
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml lexer::tests`
Expected: FAIL — current lexer returns `UnrecognizedHead` for non-comment lines.

- [ ] **Step 3: Implement parameter extraction**

In `rust/gcode/src/lexer.rs`, add helper functions and update `Lexer::next`:

Add after the `Lexer` struct, before `impl Iterator`:
```rust
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
    let head_byte = head_char as u32;
    if !((b'A' as u32..=b'Z' as u32).contains(&head_byte)) {
        return Err(ParseError::UnrecognizedHead {
            line_no,
            head: line.split_whitespace().next().unwrap_or(line).to_string().into_boxed_str(),
        });
    }
    // Read the head number up to whitespace.
    let rest_after_letter_idx = chars.next().map_or(line.len(), |(i, _)| i);
    let after_letter = &line[rest_after_letter_idx..];
    let head_number_str = after_letter
        .split_whitespace()
        .next()
        .unwrap_or("");
    let (major, minor) = parse_head_number(head_number_str).ok_or_else(|| {
        ParseError::UnrecognizedHead {
            line_no,
            head: format!("{head_char}{head_number_str}").into_boxed_str(),
        }
    })?;
    // Parse remaining whitespace-separated tokens as `<letter><number>`.
    let mut params = crate::Params::default();
    let mut seen = [false; 26];
    let after_head_index = rest_after_letter_idx + head_number_str.len();
    for tok in line[after_head_index..].split_whitespace() {
        let mut tc = tok.chars();
        let Some(letter_ch) = tc.next() else { continue };
        let letter = letter_ch.to_ascii_uppercase() as u8;
        if !(b'A'..=b'Z').contains(&letter) {
            return Err(ParseError::MalformedNumber {
                line_no,
                text: tok.to_string().into_boxed_str(),
            });
        }
        let num_str = &tok[letter_ch.len_utf8()..];
        let value: f64 = num_str.parse().map_err(|_| ParseError::MalformedNumber {
            line_no,
            text: tok.to_string().into_boxed_str(),
        })?;
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
        letter: head_byte as u8,
        major,
        minor,
        params,
        line_no,
    })
}
```

Now replace the body of `Lexer::next` (the `loop { ... }` block) with:
```rust
        loop {
            let (idx, raw) = self.lines.next()?;
            let line_no = (idx as u32).checked_add(1).expect("line count overflow");
            let trimmed_full = raw.trim();
            if trimmed_full.is_empty() {
                continue;
            }
            if let Some(stripped) = trimmed_full.strip_prefix(';') {
                return Some(Ok(Token::Comment {
                    text: stripped.trim().to_string().into_boxed_str(),
                    line_no,
                }));
            }
            let no_inline = strip_inline_comment(trimmed_full).trim();
            if no_inline.is_empty() {
                continue;
            }
            return Some(tokenize_command_line(no_inline, line_no));
        }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml`
Expected: PASS — all lexer tests including the new parameter ones.

- [ ] **Step 5: Commit**

```bash
git add rust/gcode/src/lexer.rs
git commit -m "gcode/lexer: parameter extraction and command tokenization"
```

---

## Task 6: Lexer — decimal heads (G5.1) and M/T-words

**Files:**
- Modify: `rust/gcode/src/lexer.rs`

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `rust/gcode/src/lexer.rs`:
```rust
    #[test]
    fn parses_g5_1() {
        let toks = collect("G5.1 X10 Y20 I1 J2\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Command { letter, major, minor, .. }) => {
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
            Ok(Token::Command { letter, major, params, .. }) => {
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
```

- [ ] **Step 2: Run tests to verify they pass (decimal heads + M/T already work via Task 5's general implementation)**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml lexer::tests`
Expected: PASS — Task 5's implementation already handles these via `parse_head_number` and the generic letter-loop.

If any test fails, the implementation has a bug. Debug, fix, re-run until PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/gcode/src/lexer.rs
git commit -m "gcode/lexer: regression tests for decimal heads, M-codes, T-codes"
```

---

## Task 7: Slicer-dialect marker matching

**Files:**
- Modify: `rust/gcode/src/marker.rs`
- Modify: `rust/gcode/src/lexer.rs`

- [ ] **Step 1: Write the failing tests**

Append a `tests` module to `rust/gcode/src/marker.rs`:
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn matches_orca_layer_change() {
        assert_eq!(match_comment(";LAYER:5"), Some(MarkerKind::LayerChange { layer: 5 }));
        assert_eq!(match_comment(";LAYER:0"), Some(MarkerKind::LayerChange { layer: 0 }));
    }

    #[test]
    fn matches_prusa_layer() {
        assert_eq!(match_comment(";LAYER_CHANGE"), None); // tag-only; layer number in next ;Z line
        // PrusaSlicer also emits ;LAYER:N; treat both.
        assert_eq!(match_comment(";LAYER:12"), Some(MarkerKind::LayerChange { layer: 12 }));
    }

    #[test]
    fn matches_type() {
        assert_eq!(
            match_comment(";TYPE:WALL-OUTER"),
            Some(MarkerKind::LayerType { name: "WALL-OUTER".to_string().into_boxed_str() })
        );
    }

    #[test]
    fn unknown_comment_returns_none() {
        assert_eq!(match_comment("; just a comment"), None);
        assert_eq!(match_comment(";generated by Slic3r"), None);
    }
}
```

Also add a test to `lexer::tests`:
```rust
    #[test]
    fn layer_change_comment_is_marker_token() {
        let toks = collect(";LAYER:5\n");
        assert_eq!(toks.len(), 1);
        match &toks[0] {
            Ok(Token::Marker { kind, line_no }) => {
                assert_eq!(*kind, crate::marker::MarkerKind::LayerChange { layer: 5 });
                assert_eq!(*line_no, 1);
            }
            other => panic!("expected Marker, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml`
Expected: FAIL — `match_comment` not defined; lexer doesn't promote comments to markers.

- [ ] **Step 3: Implement `match_comment`**

In `rust/gcode/src/marker.rs`, add (above `#[cfg(test)]`):
```rust
/// Match a comment line (with leading `;` already verified) against known
/// slicer-dialect patterns. Returns `Some(MarkerKind)` if recognized, `None`
/// otherwise. Caller passes the raw comment line (including the leading `;`).
#[must_use]
pub fn match_comment(comment_line: &str) -> Option<MarkerKind> {
    // Strip leading ';' and surrounding whitespace within the comment.
    let body = comment_line.strip_prefix(';')?.trim();

    // ;LAYER:N (Orca, Prusa, Bambu)
    if let Some(rest) = body.strip_prefix("LAYER:") {
        if let Ok(n) = rest.trim().parse::<u32>() {
            return Some(MarkerKind::LayerChange { layer: n });
        }
    }

    // ;TYPE:NAME (Orca, Prusa, Bambu)
    if let Some(rest) = body.strip_prefix("TYPE:") {
        return Some(MarkerKind::LayerType {
            name: rest.trim().to_string().into_boxed_str(),
        });
    }

    // ;END_OF_PRINT and similar
    let upper = body.to_ascii_uppercase();
    if upper == "END_OF_PRINT" || upper == "END OF PRINT" || upper.starts_with("END_GCODE") {
        return Some(MarkerKind::EndOfPrint);
    }

    None
}
```

- [ ] **Step 4: Wire `match_comment` into the lexer**

In `rust/gcode/src/lexer.rs`, replace the comment branch in `Lexer::next` (the `if let Some(stripped) = trimmed_full.strip_prefix(';')` block) with:
```rust
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
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add rust/gcode/src/marker.rs rust/gcode/src/lexer.rs
git commit -m "gcode/marker: slicer-dialect comment matchers"
```

---

## Task 8: Property-test the lexer never panics

**Files:**
- Create: `rust/gcode/tests/property_lex.rs`

- [ ] **Step 1: Write the property test**

Create `rust/gcode/tests/property_lex.rs`:
```rust
//! The lexer must never panic on arbitrary input. It must always terminate
//! (yielding either a `Token` or a `ParseError` per non-empty line, and
//! eventually returning `None`).

use gcode::lex;
use proptest::prelude::*;

proptest! {
    #![proptest_config(ProptestConfig {
        cases: 1024,
        ..Default::default()
    })]

    #[test]
    fn lexer_never_panics_on_arbitrary_text(s in ".{0,4096}") {
        let _: Vec<_> = lex(&s).collect();
    }

    #[test]
    fn lexer_never_panics_on_arbitrary_lines(
        lines in proptest::collection::vec(".{0,128}", 0..64)
    ) {
        let s = lines.join("\n");
        let _: Vec<_> = lex(&s).collect();
    }

    #[test]
    fn lexer_terminates_on_long_input(s in ".{0,16384}") {
        let count = lex(&s).count();
        // Per-line tokens at most; must terminate.
        prop_assert!(count <= s.lines().count() + 1);
    }
}
```

- [ ] **Step 2: Run the property tests**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml --test property_lex`
Expected: PASS — 1024 random inputs per property, no panics.

If any case panics, the lexer has a bug on a specific input shape. Reduce the failing case (proptest reports it), fix the lexer, re-run.

- [ ] **Step 3: Commit**

```bash
git add rust/gcode/tests/property_lex.rs
git commit -m "gcode: property tests — lexer never panics, always terminates"
```

---

## Task 9: cargo-fuzz target setup

**Files:**
- Create: `rust/gcode/fuzz/Cargo.toml`
- Create: `rust/gcode/fuzz/fuzz_targets/lex.rs`
- Create: `rust/gcode/fuzz/.gitignore`

- [ ] **Step 1: Initialize the fuzz target**

Create `rust/gcode/fuzz/Cargo.toml`:
```toml
[package]
name = "gcode-fuzz"
version = "0.0.0"
publish = false
edition = "2021"

[package.metadata]
cargo-fuzz = true

[dependencies]
libfuzzer-sys = "0.4"
gcode = { path = ".." }

[[bin]]
name = "lex"
path = "fuzz_targets/lex.rs"
test = false
doc = false
bench = false
```

Create `rust/gcode/fuzz/fuzz_targets/lex.rs`:
```rust
#![no_main]
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    if let Ok(s) = std::str::from_utf8(data) {
        let _: Vec<_> = gcode::lex(s).collect();
    }
});
```

Create `rust/gcode/fuzz/.gitignore`:
```
target
corpus
artifacts
```

- [ ] **Step 2: Verify the fuzz target builds**

Run: `cd rust/gcode/fuzz && cargo +nightly fuzz build lex 2>&1 | tail -5`
Expected: builds successfully (requires `cargo-fuzz` installed; if not present, document as a CI requirement and skip the run).

If `cargo-fuzz` is not available locally, that's fine — verify only that the manifest is syntactically valid:
Run: `cd rust/gcode/fuzz && cargo metadata --manifest-path Cargo.toml --no-deps > /dev/null`
Expected: succeeds with no error.

- [ ] **Step 3: Commit**

```bash
git add rust/gcode/fuzz/
git commit -m "gcode: cargo-fuzz target on lex(&[u8])"
```

---

## Task 10: Golden corpus snapshot test

**Files:**
- Create: `rust/gcode/tests/golden_corpus_lex.rs`

- [ ] **Step 1: Write the corpus smoke test**

Create `rust/gcode/tests/golden_corpus_lex.rs`:
```rust
//! Tokenize the OrcaSlicer corpus end-to-end. Asserts:
//!  - No panics.
//!  - Token counts match expected order-of-magnitude.
//!  - At least one LayerChange marker is recognized.
//!  - At least 100k Command tokens for G/M/T heads.

use gcode::{lex, Token};
use std::path::Path;

const CORPUS_DIR: &str = "../../scripts/fitter_prototype/corpus";

fn read_corpus_file(name: &str) -> Option<String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(CORPUS_DIR).join(name);
    std::fs::read_to_string(&path).ok()
}

#[test]
fn arc_fitted_corpus_lexes_without_panic() {
    let Some(text) = read_corpus_file("voron_cube_arc_fitted.gcode") else {
        eprintln!("skipping: corpus file not present");
        return;
    };

    let mut commands = 0u64;
    let mut comments = 0u64;
    let mut markers = 0u64;
    let mut errors = 0u64;
    let mut layer_changes = 0u64;

    for item in lex(&text) {
        match item {
            Ok(Token::Command { .. }) => commands += 1,
            Ok(Token::Comment { .. }) => comments += 1,
            Ok(Token::Marker { kind, .. }) => {
                markers += 1;
                if matches!(kind, gcode::MarkerKind::LayerChange { .. }) {
                    layer_changes += 1;
                }
            }
            Err(_) => errors += 1,
        }
    }

    eprintln!(
        "arc_fitted: commands={commands} comments={comments} markers={markers} \
         errors={errors} layer_changes={layer_changes}"
    );

    assert!(commands > 100_000, "expected > 100k Command tokens, got {commands}");
    assert!(layer_changes >= 1, "expected at least one LayerChange marker");
    assert!(
        errors < commands / 100,
        "more than 1% of commands errored: {errors} errors vs {commands} commands"
    );
}

#[test]
fn straight_line_corpus_lexes_without_panic() {
    let Some(text) = read_corpus_file("voron_cube_straight_line.gcode") else {
        eprintln!("skipping: corpus file not present");
        return;
    };
    let mut commands = 0u64;
    for item in lex(&text) {
        if let Ok(Token::Command { .. }) = item {
            commands += 1;
        }
    }
    assert!(commands > 150_000, "expected > 150k Command tokens, got {commands}");
}
```

- [ ] **Step 2: Run the corpus tests**

Run: `cargo test -p gcode --manifest-path rust/Cargo.toml --test golden_corpus_lex -- --nocapture`
Expected: PASS — both tests run on the committed corpus files. Token counts in the eprintln output should be sensible (commands in the 100k-200k range).

- [ ] **Step 3: Commit**

```bash
git add rust/gcode/tests/golden_corpus_lex.rs
git commit -m "gcode: golden corpus smoke test on OrcaSlicer files"
```

---

## Task 11: `geometry/` crate scaffold

**Files:**
- Modify: `rust/Cargo.toml`
- Create: `rust/geometry/Cargo.toml`
- Create: `rust/geometry/src/lib.rs`
- Create: `rust/geometry/src/segment.rs`
- Create: `rust/geometry/src/params.rs`
- Create: `rust/geometry/src/error.rs`
- Create: `rust/geometry/src/telemetry.rs`
- Create: `rust/geometry/src/reduce.rs`
- Create: `rust/geometry/src/pipeline.rs`

- [ ] **Step 1: Add `geometry` to workspace members**

Edit `rust/Cargo.toml`. Replace the `members = [...]` line with:
```toml
members = ["nurbs", "nurbs-c-api", "gcode", "geometry"]
```

- [ ] **Step 2: Create the `geometry` crate manifest**

Create `rust/geometry/Cargo.toml`:
```toml
[package]
name = "geometry"
version = "0.1.0"
edition = "2021"
rust-version = "1.85"
publish = false
description = "Layer 1 geometry pipeline for the kalico motion planner. Token stream → typed segments. See docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md."

[dependencies]
nurbs = { path = "../nurbs" }
gcode = { path = "../gcode" }
thiserror = { workspace = true }

[lints]
workspace = true
```

- [ ] **Step 3: Create empty stub modules**

Create `rust/geometry/src/lib.rs`:
```rust
//! Layer 1 geometry pipeline. Token stream → typed segments.
//! See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`.

#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod error;
pub mod params;
pub mod pipeline;
pub(crate) mod reduce;
pub mod segment;
pub mod telemetry;

pub use error::{Fatal, InternalDetails, InternalKind, Recovery, SlotDegeneracy};
pub use params::FitterParams;
pub use pipeline::{GeometryPipeline, Item, Segments};
pub use segment::{
    ArcSegment, BlendFamily, CornerBlendSlot, FittedSegment, JunctionDeviation,
    Segment, SourceRange,
};
pub use telemetry::TelemetryEvent;
```

Create stub bodies in each module file, sufficient to satisfy the `pub use`s. Each will be filled out in subsequent tasks.

`rust/geometry/src/segment.rs`:
```rust
//! Segment types — the product of the iterator. Layer 2 reads these.

use nurbs::{ScalarNurbs, VectorNurbs};

#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {}

#[derive(Debug, Clone, PartialEq)]
pub struct FittedSegment {
    pub xyz: VectorNurbs<f64, 3>,
    pub e: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub degree: u8,
    pub max_residual_mm: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct ArcSegment {
    pub xyz: VectorNurbs<f64, 3>,
    pub e: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct CornerBlendSlot {
    pub position: [f64; 3],
    pub t_in: [f64; 3],
    pub t_out: [f64; 3],
    pub seg_len_in: f64,
    pub seg_len_out: f64,
    pub tolerance_budget_mm: f64,
    pub default_family: BlendFamily,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JunctionDeviation {
    pub position: [f64; 3],
    pub angle_deg: f64,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlendFamily {
    CubicBezier,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceRange {
    pub start_line: u32,
    pub end_line: u32,
}
```

`rust/geometry/src/params.rs`:
```rust
//! `FitterParams`: tunable knobs for classifier, fitter, lookahead, and corner blend.

#[derive(Copy, Clone, Debug, PartialEq)]
pub struct FitterParams {
    pub theta_smooth_deg: f64,
    pub theta_hard_deg: f64,
    pub seg_len_collapse_mm: f64,
    pub degree: u8,
    pub n_init_interior: u32,
    pub eps_chord_mm: f64,
    pub eps_iter_mm: f64,
    pub max_lspia_iter: u32,
    pub max_refine_iter: u32,
    pub n_chord_samples: u32,
    pub max_window_vertices: u32,
    pub blend_tolerance_mm: f64,
}

impl Default for FitterParams {
    fn default() -> Self {
        Self {
            theta_smooth_deg: 15.0,
            theta_hard_deg: 60.0,
            seg_len_collapse_mm: 0.05,
            degree: 3,
            n_init_interior: 4,
            eps_chord_mm: 0.025,
            eps_iter_mm: 1e-9,
            max_lspia_iter: 100,
            max_refine_iter: 20,
            n_chord_samples: 50,
            max_window_vertices: 64,
            blend_tolerance_mm: 0.050,
        }
    }
}
```

`rust/geometry/src/error.rs`:
```rust
//! Error model. `Recovery` for anomalies (#[non_exhaustive]), `Fatal` for
//! invariant violations (closed; consumers must handle every variant).

use crate::SourceRange;

#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum Recovery {
    UnrecognizedCommand { line_no: u32, head: String },
    MalformedParams { line_no: u32, raw: String },
    WindowCapHit { source: SourceRange, run_vertex_count: u32 },
    DegenerateSlotFallback { line_no: u32, reason: SlotDegeneracy },
    ToleranceExceeded { source: SourceRange, actual_mm: f64, budget_mm: f64 },
    LspiaNotConverged { source: SourceRange, last_update_mm: f64 },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SlotDegeneracy {
    BacktrackingCorner,
    ZeroIncidentLength,
    ColinearTangents,
}

#[derive(Debug)]
pub enum Fatal {
    Internal(Box<InternalDetails>),
}

#[derive(Debug)]
pub struct InternalDetails {
    pub kind: InternalKind,
    pub context: String,
    pub backtrace: std::backtrace::Backtrace,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InternalKind {
    NonMonotoneKnotVector,
    NaNDetected,
    KnotInsertionFailed,
    BasisMatrixSingular,
    DegreeOutOfBounds,
}
```

`rust/geometry/src/telemetry.rs`:
```rust
//! Observability events emitted to the consumer-supplied closure sink.

use crate::Recovery;

#[derive(Debug, Clone, PartialEq)]
#[non_exhaustive]
pub enum TelemetryEvent {
    LayerChange { layer: u32, line_no: u32 },
    ToolChange { tool: u32, line_no: u32 },
    Retraction { e_delta_mm: f64, line_no: u32 },
    WindowFlush { run_vertex_count: u32, line_no: u32 },
    Recovery(Recovery),
    FitObservation {
        residual_mm: f64,
        tolerance_mm: f64,
        run_vertex_count: u32,
        piece_count: u32,
        degree: u8,
    },
}
```

`rust/geometry/src/reduce.rs`:
```rust
//! Reduce: token stream → internal `ReduceEvent` stream. Pub(crate); tests
//! import via `#[cfg(test)] pub use`.
//! Phase 1 implementation is filled in across Tasks 13-17.
```

`rust/geometry/src/pipeline.rs`:
```rust
//! `GeometryPipeline`, `Segments`, `Item`. Phase 1 implementation is filled in
//! across Tasks 18-23.

use crate::{Fatal, FitterParams, Recovery, Segment};

pub struct GeometryPipeline {
    #[allow(dead_code)]
    params: FitterParams,
}

impl GeometryPipeline {
    #[must_use]
    pub fn new(params: FitterParams) -> Self {
        Self { params }
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum Item {
    Segment(Segment),
    Recovered(Segment, Recovery),
    Fatal(Fatal),
}

pub struct Segments<'a> {
    _lifetime: std::marker::PhantomData<&'a ()>,
}

impl<'a> Iterator for Segments<'a> {
    type Item = Item;
    fn next(&mut self) -> Option<Item> {
        None
    }
}
```

- [ ] **Step 4: Verify the workspace compiles**

Run: `cargo build -p geometry --manifest-path rust/Cargo.toml`
Expected: succeeds with warnings about empty `Segment` enum / unused fields; no errors.

The empty `Segment` enum will become populated in Task 12.

- [ ] **Step 5: Commit**

```bash
git add rust/Cargo.toml rust/geometry/
git commit -m "geometry: scaffold crate with all public types as skeletons"
```

---

## Task 12: Populate the `Segment` enum

**Files:**
- Modify: `rust/geometry/src/segment.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/geometry/src/segment.rs`:
```rust
#[cfg(test)]
mod tests {
    use super::*;
    use nurbs::VectorNurbs;

    #[test]
    fn segment_variants_construct() {
        let xyz = VectorNurbs::<f64, 3>::try_new(
            1,
            vec![0.0, 0.0, 1.0, 1.0],
            vec![[0.0, 0.0, 0.0], [1.0, 1.0, 0.0]],
            None,
        )
        .expect("valid degree-1 NURBS");
        let f = FittedSegment {
            xyz: xyz.clone(),
            e: None,
            feedrate_mm_s: 100.0,
            degree: 1,
            max_residual_mm: 0.0,
            source: SourceRange { start_line: 1, end_line: 2 },
        };
        let _seg_fitted: Segment = Segment::Fitted(f);

        let arc = ArcSegment {
            xyz,
            e: None,
            feedrate_mm_s: 100.0,
            source: SourceRange { start_line: 3, end_line: 3 },
        };
        let _seg_arc: Segment = Segment::Arc(arc);

        let slot = CornerBlendSlot {
            position: [0.0; 3],
            t_in: [1.0, 0.0, 0.0],
            t_out: [0.0, 1.0, 0.0],
            seg_len_in: 1.0,
            seg_len_out: 1.0,
            tolerance_budget_mm: 0.05,
            default_family: BlendFamily::CubicBezier,
            feedrate_mm_s: 100.0,
            source: SourceRange { start_line: 5, end_line: 5 },
        };
        let _seg_slot: Segment = Segment::CornerBlend(slot);

        let jd = JunctionDeviation {
            position: [0.0; 3],
            angle_deg: 90.0,
            feedrate_mm_s: 100.0,
            source: SourceRange { start_line: 7, end_line: 7 },
        };
        let _seg_jd: Segment = Segment::Junction(jd);
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml segment::tests`
Expected: FAIL — `Segment::Fitted`, `Segment::Arc`, `Segment::CornerBlend`, `Segment::Junction` do not exist.

- [ ] **Step 3: Populate the `Segment` enum**

In `rust/geometry/src/segment.rs`, replace the enum definition `pub enum Segment {}` with:
```rust
#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {
    Fitted(FittedSegment),
    Arc(ArcSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml segment::tests`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/segment.rs
git commit -m "geometry/segment: populate Segment enum with four variants"
```

---

## Task 13: ReduceEvent type and modal-state struct

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/geometry/src/reduce.rs`:
```rust
#[cfg(test)]
pub use tests::*;  // expose internal types to integration tests if needed

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modal_state_initializes_at_origin() {
        let st = ModalState::new();
        assert_eq!(st.position, [0.0, 0.0, 0.0]);
        assert_eq!(st.feedrate_mm_s, None);
        assert_eq!(st.tool, 0);
    }

    #[test]
    fn reduce_event_variants_construct() {
        let _e1 = ReduceEvent::G1Move {
            from: [0.0, 0.0, 0.0],
            to: [1.0, 0.0, 0.0],
            e_delta: Some(0.05),
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
        let _e2 = ReduceEvent::Arc {
            start: [0.0, 0.0, 0.0],
            end: [1.0, 0.0, 0.0],
            center: [0.5, -0.5, 0.0],
            clockwise: true,
            z_delta: 0.0,
            e_delta: Some(0.05),
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
        let _e3 = ReduceEvent::Marker {
            kind: MotionMarkerKind::ZOnly,
            line_no: 5,
            tool: None,
            e_delta_mm: None,
        };
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: FAIL — `ModalState`, `ReduceEvent`, `MotionMarkerKind` not defined.

- [ ] **Step 3: Implement the types**

Replace the body of `rust/geometry/src/reduce.rs` (above `#[cfg(test)]`) with:
```rust
//! Reduce: token stream → internal `ReduceEvent` stream. Pub(crate); tests
//! import via `#[cfg(test)] pub use`.

/// Modal state machine — accumulates the current position, feedrate, and tool
/// across the gcode stream, applying G1's modal "params absent → unchanged"
/// semantics.
#[derive(Debug, Clone)]
pub(crate) struct ModalState {
    pub position: [f64; 3],
    pub e: f64,
    pub feedrate_mm_s: Option<f64>,
    pub tool: u32,
}

impl ModalState {
    pub fn new() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            e: 0.0,
            feedrate_mm_s: None,
            tool: 0,
        }
    }
}

/// Internal reduce-output events. `pipeline` consumes these.
#[derive(Debug, Clone, PartialEq)]
pub(crate) enum ReduceEvent {
    G1Move {
        from: [f64; 3],
        to: [f64; 3],
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    },
    Arc {
        start: [f64; 3],
        end: [f64; 3],
        center: [f64; 3],
        clockwise: bool,
        z_delta: f64,
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    },
    Marker {
        kind: MotionMarkerKind,
        line_no: u32,
        /// For T-codes, the tool number from the command's `major` field.
        tool: Option<u32>,
        /// For E-only G1 markers, the signed E delta (mm).
        e_delta_mm: Option<f64>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum MotionMarkerKind {
    /// G0 (rapid travel)
    G0,
    /// G1 with no X/Y change (Z-only or E-only move)
    ZOnly,
    /// G1 with E delta but no XY motion (retract / unretract)
    EOnly,
    /// G92 (set position)
    G92,
    /// M-code
    M,
    /// T-code (tool change)
    T,
    /// End of input
    EndOfFile,
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: ReduceEvent type and ModalState scaffold"
```

---

## Task 14: Reduce — G1 modal-state machine

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

- [ ] **Step 1: Write the failing tests**

Add to the `tests` module in `rust/geometry/src/reduce.rs`:
```rust
    use gcode::{Params, Token};

    fn cmd(letter: u8, major: u32, line_no: u32, params: Params) -> Token {
        Token::Command { letter, major, minor: None, params, line_no }
    }

    fn p(setters: &[(u8, f64)]) -> Params {
        let mut p = Params::default();
        for (l, v) in setters { p.set(*l, *v); }
        p
    }

    #[test]
    fn g1_xy_emits_g1move() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 2.0), (b'F', 1500.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::G1Move { from, to, feedrate_mm_s, .. } => {
                assert_eq!(*from, [0.0, 0.0, 0.0]);
                assert_eq!(*to, [1.0, 2.0, 0.0]);
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9, "F1500 → 25 mm/s");
            }
            other => panic!("expected G1Move, got {other:?}"),
        }
    }

    #[test]
    fn g1_z_only_emits_zonly_marker() {
        let toks = vec![cmd(b'G', 1, 1, p(&[(b'Z', 0.2), (b'F', 1500.0)]))];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Marker { kind: MotionMarkerKind::ZOnly, line_no: 1, .. } => {}
            other => panic!("expected ZOnly Marker, got {other:?}"),
        }
    }

    #[test]
    fn g1_e_only_emits_eonly_marker() {
        let toks = vec![cmd(b'G', 1, 1, p(&[(b'E', -1.5), (b'F', 3000.0)]))];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Marker { kind: MotionMarkerKind::EOnly, line_no: 1, e_delta_mm: Some(d), .. } => {
                assert!((d - (-1.5)).abs() < 1e-12);
            }
            other => panic!("expected EOnly Marker, got {other:?}"),
        }
    }

    #[test]
    fn g0_emits_g0_marker() {
        let toks = vec![cmd(b'G', 0, 1, p(&[(b'X', 5.0), (b'Y', 5.0)]))];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Marker { kind: MotionMarkerKind::G0, line_no: 1, .. } => {}
            other => panic!("expected G0 Marker, got {other:?}"),
        }
    }

    #[test]
    fn t_marker_carries_tool_number() {
        let toks = vec![cmd(b'T', 2, 1, Params::default())];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Marker { kind: MotionMarkerKind::T, tool: Some(2), .. } => {}
            other => panic!("expected T Marker with tool=2, got {other:?}"),
        }
    }

    #[test]
    fn modal_position_persists_across_g1s() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 1, 2, p(&[(b'X', 2.0)])),  // Y not given, should persist as 0.0
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::G1Move { from, to, .. } => {
                assert_eq!(*from, [1.0, 0.0, 0.0]);
                assert_eq!(*to, [2.0, 0.0, 0.0]);
            }
            other => panic!("expected G1Move, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: FAIL — `reduce` function not defined.

- [ ] **Step 3: Implement `reduce`**

Add to `rust/geometry/src/reduce.rs` (above `#[cfg(test)]`):
```rust
use gcode::{ParseError, Token};

/// Convert F-word (mm/min) to mm/s.
fn f_to_mm_s(f: f64) -> f64 {
    f / 60.0
}

/// Walk a token iterator, maintain modal state, and emit `ReduceEvent`s.
///
/// Phase 1 handles: G0 (marker), G1 (move or marker), G2/G3 (Arc — Task 15),
/// G92 (marker — Task 16), M-codes (marker), T-codes (marker), and forwards
/// recognized comment markers to `MotionMarkerKind`-bearing telemetry events.
/// Parse errors are skipped here; the pipeline layer translates them to
/// `Recovery` items.
pub(crate) fn reduce<I>(tokens: I) -> impl Iterator<Item = ReduceEvent>
where
    I: IntoIterator<Item = Result<Token, ParseError>>,
{
    ReduceIter {
        tokens: tokens.into_iter(),
        state: ModalState::new(),
    }
}

struct ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    tokens: I,
    state: ModalState,
}

impl<I> Iterator for ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    type Item = ReduceEvent;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let tok = self.tokens.next()?;
            let Ok(tok) = tok else { continue }; // parse errors handled at pipeline layer
            match tok {
                Token::Command { letter: b'G', major: 0, params, line_no, .. } => {
                    // G0 — update position state, emit G0 marker.
                    self.update_position(&params);
                    if let Some(f) = params.f() {
                        self.state.feedrate_mm_s = Some(f_to_mm_s(f));
                    }
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::G0, line_no,
                        tool: None, e_delta_mm: None,
                    });
                }
                Token::Command { letter: b'G', major: 1, params, line_no, .. } => {
                    let from = self.state.position;
                    let xy_changed = params.x().is_some() || params.y().is_some();
                    let z_changed = params.z().is_some();
                    let e_present = params.e().is_some();
                    if let Some(f) = params.f() {
                        self.state.feedrate_mm_s = Some(f_to_mm_s(f));
                    }
                    if !xy_changed && z_changed && !e_present {
                        // Z-only move: marker, but update position.
                        self.update_position(&params);
                        return Some(ReduceEvent::Marker {
                            kind: MotionMarkerKind::ZOnly, line_no,
                            tool: None, e_delta_mm: None,
                        });
                    }
                    if !xy_changed && !z_changed && e_present {
                        // E-only (retract / unretract).
                        let new_e = params.e().unwrap();
                        let delta = new_e - self.state.e;
                        self.state.e = new_e;
                        return Some(ReduceEvent::Marker {
                            kind: MotionMarkerKind::EOnly, line_no,
                            tool: None, e_delta_mm: Some(delta),
                        });
                    }
                    if !xy_changed && !z_changed && !e_present {
                        // G1 with only F — no motion, treated as no-op.
                        continue;
                    }
                    // Real move: update position and E, emit G1Move.
                    self.update_position(&params);
                    let e_delta = params.e().map(|new_e| {
                        let d = new_e - self.state.e;
                        self.state.e = new_e;
                        d
                    });
                    let to = self.state.position;
                    let feedrate_mm_s = self.state.feedrate_mm_s.unwrap_or(0.0);
                    return Some(ReduceEvent::G1Move {
                        from, to, e_delta, feedrate_mm_s, line_no,
                    });
                }
                Token::Command { letter: b'G', major: 92, line_no, .. } => {
                    // G92: position reset. Treated as marker break.
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::G92, line_no,
                        tool: None, e_delta_mm: None,
                    });
                }
                Token::Command { letter: b'M', line_no, .. } => {
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::M, line_no,
                        tool: None, e_delta_mm: None,
                    });
                }
                Token::Command { letter: b'T', major, line_no, .. } => {
                    self.state.tool = major;
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::T, line_no,
                        tool: Some(major), e_delta_mm: None,
                    });
                }
                // Arc + comment forwarding handled in Tasks 15 / 17.
                _ => continue,
            }
        }
    }
}

impl<I> ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    fn update_position(&mut self, params: &gcode::Params) {
        if let Some(x) = params.x() { self.state.position[0] = x; }
        if let Some(y) = params.y() { self.state.position[1] = y; }
        if let Some(z) = params.z() { self.state.position[2] = z; }
    }
}
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: G0/G1/G92/M/T modal-state machine"
```

---

## Task 15: Reduce — G2/G3 arc handling (helical)

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

- [ ] **Step 1: Write the failing tests**

Add to `tests` in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    fn g2_emits_arc_clockwise() {
        // Set position to (1, 0, 0), then arc to (0, 1, 0) around (0, 0).
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 2, 2, p(&[(b'X', 0.0), (b'Y', 1.0), (b'I', -1.0), (b'J', 0.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::Arc { start, end, center, clockwise, z_delta, .. } => {
                assert_eq!(*start, [1.0, 0.0, 0.0]);
                assert_eq!(*end, [0.0, 1.0, 0.0]);
                assert_eq!(*center, [0.0, 0.0, 0.0]);
                assert!(*clockwise);
                assert_eq!(*z_delta, 0.0);
            }
            other => panic!("expected Arc, got {other:?}"),
        }
    }

    #[test]
    fn g3_emits_arc_counter_clockwise() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'F', 1500.0)])),
            cmd(b'G', 3, 2, p(&[(b'X', 0.0), (b'Y', 1.0), (b'I', -1.0), (b'J', 0.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[1] {
            ReduceEvent::Arc { clockwise: false, .. } => {}
            other => panic!("expected counter-clockwise Arc, got {other:?}"),
        }
    }

    #[test]
    fn g2_with_z_delta_yields_z_delta_field() {
        // Helical arc: end Z differs from start Z.
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Z', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 2, 2, p(&[
                (b'X', 0.0), (b'Y', 1.0), (b'Z', 0.5),
                (b'I', -1.0), (b'J', 0.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[1] {
            ReduceEvent::Arc { z_delta, end, .. } => {
                assert!((z_delta - 0.5).abs() < 1e-12);
                assert_eq!(end[2], 0.5);
            }
            other => panic!("expected helical Arc, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: FAIL — G2/G3 currently fall through to `_ => continue`.

- [ ] **Step 3: Implement arc handling**

In `rust/geometry/src/reduce.rs`, in the `match tok` block of `ReduceIter::next`, add **before** the `_ => continue,` arm:
```rust
                Token::Command {
                    letter: b'G', major: 2 | 3, params, line_no, ..
                } => {
                    let start = self.state.position;
                    let i = params.i().unwrap_or(0.0);
                    let j = params.j().unwrap_or(0.0);
                    let center = [start[0] + i, start[1] + j, start[2]];
                    let new_x = params.x().unwrap_or(start[0]);
                    let new_y = params.y().unwrap_or(start[1]);
                    let new_z = params.z().unwrap_or(start[2]);
                    let end = [new_x, new_y, new_z];
                    let z_delta = new_z - start[2];
                    let clockwise = matches!(tok, Token::Command { major: 2, .. });
                    if let Some(f) = params.f() {
                        self.state.feedrate_mm_s = Some(f_to_mm_s(f));
                    }
                    let e_delta = params.e().map(|new_e| {
                        let d = new_e - self.state.e;
                        self.state.e = new_e;
                        d
                    });
                    self.state.position = end;
                    let feedrate_mm_s = self.state.feedrate_mm_s.unwrap_or(0.0);
                    return Some(ReduceEvent::Arc {
                        start, end, center, clockwise, z_delta, e_delta,
                        feedrate_mm_s, line_no,
                    });
                }
```

Note: the `matches!(tok, ...)` requires `tok` to still be in scope; the existing `match tok { ... }` consumes `tok`. Refactor by binding the major in the pattern. Replace the entire match arm above with:
```rust
                Token::Command {
                    letter: b'G', major: g, params, line_no, ..
                } if g == 2 || g == 3 => {
                    let start = self.state.position;
                    let i = params.i().unwrap_or(0.0);
                    let j = params.j().unwrap_or(0.0);
                    let center = [start[0] + i, start[1] + j, start[2]];
                    let new_x = params.x().unwrap_or(start[0]);
                    let new_y = params.y().unwrap_or(start[1]);
                    let new_z = params.z().unwrap_or(start[2]);
                    let end = [new_x, new_y, new_z];
                    let z_delta = new_z - start[2];
                    let clockwise = g == 2;
                    if let Some(f) = params.f() {
                        self.state.feedrate_mm_s = Some(f_to_mm_s(f));
                    }
                    let e_delta = params.e().map(|new_e| {
                        let d = new_e - self.state.e;
                        self.state.e = new_e;
                        d
                    });
                    self.state.position = end;
                    let feedrate_mm_s = self.state.feedrate_mm_s.unwrap_or(0.0);
                    return Some(ReduceEvent::Arc {
                        start, end, center, clockwise, z_delta, e_delta,
                        feedrate_mm_s, line_no,
                    });
                }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: G2/G3 helical arc handling"
```

---

## Task 16: Reduce — comment-marker forwarding (telemetry hooks)

**Files:**
- Modify: `rust/geometry/src/reduce.rs`

The reduce layer cannot emit telemetry events itself (no sink in scope). It surfaces marker events to the pipeline via a new `ReduceEvent::CommentMarker` variant; the pipeline then translates these to `TelemetryEvent::LayerChange` etc.

- [ ] **Step 1: Write the failing test**

Add to `tests` in `rust/geometry/src/reduce.rs`:
```rust
    #[test]
    fn comment_marker_layer_change_is_forwarded() {
        let toks = vec![
            Token::Marker {
                kind: gcode::MarkerKind::LayerChange { layer: 7 },
                line_no: 42,
            },
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::CommentMarker { kind, line_no: 42 } => {
                match kind {
                    gcode::MarkerKind::LayerChange { layer } => assert_eq!(*layer, 7),
                    _ => panic!("expected LayerChange"),
                }
            }
            other => panic!("expected CommentMarker, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: FAIL — `ReduceEvent::CommentMarker` not defined; `Token::Marker` falls through.

- [ ] **Step 3: Add the variant and handle it**

In `rust/geometry/src/reduce.rs`, add to the `ReduceEvent` enum:
```rust
    CommentMarker {
        kind: gcode::MarkerKind,
        line_no: u32,
    },
```

In `ReduceIter::next`, add a match arm above `_ => continue,`:
```rust
                Token::Marker { kind, line_no } => {
                    return Some(ReduceEvent::CommentMarker { kind, line_no });
                }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml reduce::tests`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/reduce.rs
git commit -m "geometry/reduce: forward comment markers to pipeline"
```

---

## Task 17: Pipeline — `GeometryPipeline::process` skeleton + `Item::Fatal` terminal contract

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

- [ ] **Step 1: Write the failing test**

Append to `rust/geometry/src/pipeline.rs`:
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn empty_input_yields_no_items() {
        let mut p = GeometryPipeline::new(FitterParams::default());
        let mut sink = |_e: crate::TelemetryEvent| {};
        let items: Vec<_> = p.process("", &mut sink).collect();
        assert!(items.is_empty());
    }

    #[test]
    fn whitespace_input_yields_no_items() {
        let mut p = GeometryPipeline::new(FitterParams::default());
        let mut sink = |_e: crate::TelemetryEvent| {};
        let items: Vec<_> = p.process("\n\n   \n", &mut sink).collect();
        assert!(items.is_empty());
    }
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests`
Expected: FAIL — `process` not defined.

- [ ] **Step 3: Implement `process` skeleton**

Replace the body of `rust/geometry/src/pipeline.rs` (above `#[cfg(test)]`) with:
```rust
//! `GeometryPipeline`, `Segments`, `Item`. Drives reduce events into typed
//! segments. Phase 1 emits degree-1 NURBS for G1, 3D rational quadratic for
//! G2/G3, and `JunctionDeviation` at every G1-G1 transition.

use crate::{
    reduce::{reduce, MotionMarkerKind, ReduceEvent},
    Fatal, FitterParams, Recovery, Segment, TelemetryEvent,
};
use gcode::lex;
use std::collections::VecDeque;

pub struct GeometryPipeline {
    params: FitterParams,
}

impl GeometryPipeline {
    #[must_use]
    pub fn new(params: FitterParams) -> Self {
        debug_assert!(params.degree >= 1 && params.degree <= 5,
            "degree must be in [1, 5], got {}", params.degree);
        debug_assert!(params.theta_smooth_deg > 0.0
            && params.theta_smooth_deg < params.theta_hard_deg
            && params.theta_hard_deg < 180.0);
        debug_assert!(params.eps_chord_mm > 0.0);
        debug_assert!(params.max_window_vertices >= u32::from(params.degree) + 2);
        Self { params }
    }

    /// Process a complete G-code buffer. Returns a borrowing iterator over
    /// the segment stream. Sink receives observability events synchronously
    /// during processing.
    ///
    /// One-shot per file by convention.
    pub fn process<'a>(
        &'a mut self,
        text: &'a str,
        sink: &'a mut dyn FnMut(TelemetryEvent),
    ) -> Segments<'a> {
        Segments {
            #[allow(dead_code)]
            params: &self.params,
            events: Box::new(reduce(lex(text))),
            queue: VecDeque::new(),
            sink,
            terminal: false,
            prev_g1_end: None,
            prev_g1_feedrate: None,
            prev_g1_dir: None,
        }
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum Item {
    Segment(Segment),
    Recovered(Segment, Recovery),
    Fatal(Fatal),
}

pub struct Segments<'a> {
    params: &'a FitterParams,
    events: Box<dyn Iterator<Item = ReduceEvent> + 'a>,
    queue: VecDeque<Item>,
    sink: &'a mut dyn FnMut(TelemetryEvent),
    terminal: bool,
    /// End-position of the previous emitted G1 segment, for junction-deviation construction.
    prev_g1_end: Option<[f64; 3]>,
    /// Feedrate of the previous emitted G1, for junction-deviation construction.
    prev_g1_feedrate: Option<f64>,
    /// 3D unit direction of the previous emitted G1 segment, used to compute
    /// the junction angle when the next G1 arrives. Cleared at any marker break.
    prev_g1_dir: Option<[f64; 3]>,
}

const QUEUE_HARD_BOUND: usize = 8;

impl<'a> Iterator for Segments<'a> {
    type Item = Item;

    fn next(&mut self) -> Option<Item> {
        if self.terminal {
            return None;
        }
        loop {
            if let Some(item) = self.queue.pop_front() {
                if matches!(item, Item::Fatal(_)) {
                    self.terminal = true;
                }
                return Some(item);
            }
            // Drive the reduce iterator forward until something queues an item.
            let Some(event) = self.events.next() else {
                return None;
            };
            self.handle_event(event);
            debug_assert!(self.queue.len() <= QUEUE_HARD_BOUND,
                "queue grew beyond bound: {}", self.queue.len());
        }
    }
}

impl<'a> Segments<'a> {
    fn handle_event(&mut self, _event: ReduceEvent) {
        // Filled in across Tasks 18-22.
    }
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/pipeline.rs
git commit -m "geometry/pipeline: process skeleton, queue discipline, debug_asserts"
```

---

## Task 18: Pipeline — degree-1 NURBS construction for G1 moves

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

- [ ] **Step 1: Write the failing test**

Add to `tests` in `rust/geometry/src/pipeline.rs`:
```rust
    use crate::{Item, Segment, FittedSegment};

    fn collect(text: &str) -> Vec<Item> {
        let mut p = GeometryPipeline::new(FitterParams::default());
        let mut sink = |_: crate::TelemetryEvent| {};
        p.process(text, &mut sink).collect()
    }

    #[test]
    fn single_g1_emits_degree_1_fitted() {
        let items = collect("G1 X10 Y0 F1500\n");
        // First G1 from origin to (10,0): 1 FittedSegment (no preceding G1, so no junction).
        assert_eq!(items.len(), 1, "expected 1 item, got {items:#?}");
        match &items[0] {
            Item::Segment(Segment::Fitted(FittedSegment { xyz, degree, feedrate_mm_s, .. })) => {
                assert_eq!(*degree, 1);
                assert!((*feedrate_mm_s - 25.0).abs() < 1e-9);
                assert_eq!(xyz.degree(), 1);
                assert_eq!(xyz.control_points().len(), 2);
                assert_eq!(xyz.control_points()[0], [0.0, 0.0, 0.0]);
                assert_eq!(xyz.control_points()[1], [10.0, 0.0, 0.0]);
            }
            other => panic!("expected Fitted, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::single_g1_emits_degree_1_fitted`
Expected: FAIL — `handle_event` is empty.

- [ ] **Step 3: Implement degree-1 fitting in `handle_event`**

In `rust/geometry/src/pipeline.rs`, replace the `handle_event` body with:
```rust
    fn handle_event(&mut self, event: ReduceEvent) {
        match event {
            ReduceEvent::G1Move { from, to, e_delta: _, feedrate_mm_s, line_no } => {
                let xyz = degree_1_nurbs(from, to);
                let seg = FittedSegment {
                    xyz,
                    e: None,  // Phase 1: E carried as marker-break or per-segment scalar; full E NURBS is Phase 2.
                    feedrate_mm_s,
                    degree: 1,
                    max_residual_mm: 0.0,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Fitted(seg)));
                self.prev_g1_end = Some(to);
                self.prev_g1_feedrate = Some(feedrate_mm_s);
            }
            _ => {
                // Other event kinds handled in subsequent tasks.
            }
        }
    }
}

fn degree_1_nurbs(from: [f64; 3], to: [f64; 3]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![from, to],
        None,
    )
    .expect("degree-1 NURBS with 2 CPs is always valid")
}
```

Add to the imports at the top of the file:
```rust
use crate::{
    reduce::{reduce, MotionMarkerKind, ReduceEvent},
    Fatal, FittedSegment, FitterParams, Recovery, Segment, SourceRange,
    TelemetryEvent,
};
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::single_g1_emits_degree_1_fitted`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/pipeline.rs
git commit -m "geometry/pipeline: degree-1 NURBS for G1 moves"
```

---

## Task 19: Pipeline — JunctionDeviation between consecutive G1s

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

- [ ] **Step 1: Write the failing test**

Add to `tests` in `rust/geometry/src/pipeline.rs`:
```rust
    use crate::JunctionDeviation;

    #[test]
    fn two_g1s_emit_fitted_junction_fitted() {
        let items = collect("G1 X10 F1500\nG1 X10 Y10\n");
        // First G1: Fitted only (no prev).
        // Second G1: Junction (between prev_g1_end and current from), then Fitted.
        assert_eq!(items.len(), 3, "expected 3 items, got {items:#?}");
        match &items[0] {
            Item::Segment(Segment::Fitted(_)) => {}
            other => panic!("[0] expected Fitted, got {other:?}"),
        }
        match &items[1] {
            Item::Segment(Segment::Junction(JunctionDeviation { position, angle_deg, feedrate_mm_s, .. })) => {
                assert_eq!(*position, [10.0, 0.0, 0.0]);
                // First leg goes (0,0)→(10,0), second leg (10,0)→(10,10): 90° turn.
                assert!((angle_deg - 90.0).abs() < 1e-6, "expected ~90°, got {angle_deg}");
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9);
            }
            other => panic!("[1] expected Junction, got {other:?}"),
        }
        match &items[2] {
            Item::Segment(Segment::Fitted(_)) => {}
            other => panic!("[2] expected Fitted, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::two_g1s_emit_fitted_junction_fitted`
Expected: FAIL — no junction emitted.

- [ ] **Step 3: Emit junctions before each G1 except the first**

In `rust/geometry/src/pipeline.rs`, modify the `G1Move` arm of `handle_event` to emit a junction **before** the new fitted segment:
```rust
            ReduceEvent::G1Move { from, to, e_delta: _, feedrate_mm_s, line_no } => {
                if let Some(prev_end) = self.prev_g1_end {
                    if let Some(prev_f) = self.prev_g1_feedrate {
                        // Compute angle between (prev_end - prev_start) and (to - from).
                        // The previous leg's direction we don't have stored; we have
                        // prev_end (= start of this leg). Reconstruct prev incoming as
                        // (from - prev_end), but this is only nonzero if there was an
                        // intervening transform; in straight gcode `from == prev_end`.
                        // The actual incoming direction is the previous emitted segment's
                        // direction, which we need to track. For Task 19 minimum: store
                        // the previous direction and use it.
                        if let Some(prev_dir) = self.prev_g1_dir {
                            let cur_dir = unit([
                                to[0] - from[0], to[1] - from[1], to[2] - from[2],
                            ]);
                            let angle_deg = angle_between_deg(prev_dir, cur_dir);
                            let jd = JunctionDeviation {
                                position: from,
                                angle_deg,
                                feedrate_mm_s: prev_f.min(feedrate_mm_s),
                                source: SourceRange { start_line: line_no, end_line: line_no },
                            };
                            self.queue.push_back(Item::Segment(Segment::Junction(jd)));
                        }
                    }
                }
                let xyz = degree_1_nurbs(from, to);
                let seg = FittedSegment {
                    xyz,
                    e: None,
                    feedrate_mm_s,
                    degree: 1,
                    max_residual_mm: 0.0,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Fitted(seg)));
                self.prev_g1_end = Some(to);
                self.prev_g1_feedrate = Some(feedrate_mm_s);
                self.prev_g1_dir = Some(unit([to[0]-from[0], to[1]-from[1], to[2]-from[2]]));
            }
```

Add to the imports `use crate::JunctionDeviation;` at top. The `prev_g1_dir` field is already declared on `Segments` (Task 17) and initialized to `None` in `process`.

Add helpers at the bottom of the file:
```rust
fn unit(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
    if n < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0]/n, v[1]/n, v[2]/n]
    }
}

fn angle_between_deg(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dot = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]).clamp(-1.0, 1.0);
    dot.acos().to_degrees()
}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::two_g1s_emit_fitted_junction_fitted`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/pipeline.rs
git commit -m "geometry/pipeline: emit JunctionDeviation between consecutive G1s"
```

---

## Task 20: Pipeline — ArcSegment emission (3D rational quadratic)

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

- [ ] **Step 1: Write the failing test**

Add to `tests` in `rust/geometry/src/pipeline.rs`:
```rust
    use crate::ArcSegment;

    #[test]
    fn g2_emits_arc_segment_with_3d_control_points() {
        // Quarter-circle from (1, 0, 0) to (0, 1, 0), center (0, 0, 0), CW (G2).
        let items = collect("G1 X1 F1500\nG2 X0 Y1 I-1 J0\n");
        // Expect: Fitted (G1) + ArcSegment.
        assert!(items.len() >= 2);
        let arc_seg = items.iter().find_map(|it| match it {
            Item::Segment(Segment::Arc(a)) => Some(a),
            _ => None,
        });
        let arc = arc_seg.expect("expected an ArcSegment");
        assert_eq!(arc.xyz.degree(), 2);
        // Rational quadratic uses 3 control points; weighted middle CP.
        assert_eq!(arc.xyz.control_points().len(), 3);
        assert!(arc.xyz.weights().is_some(), "rational arc must have weights");
        // For a 90° arc, the corner control point is at the corner of the
        // tangent extension — for arc center (0,0) start (1,0) end (0,1)
        // tangents extend to (1,1).
        let cps = arc.xyz.control_points();
        let approx_eq = |a: f64, b: f64| (a - b).abs() < 1e-9;
        assert!(approx_eq(cps[0][0], 1.0) && approx_eq(cps[0][1], 0.0));
        assert!(approx_eq(cps[1][0], 1.0) && approx_eq(cps[1][1], 1.0));
        assert!(approx_eq(cps[2][0], 0.0) && approx_eq(cps[2][1], 1.0));
        // Z constant.
        for cp in cps { assert_eq!(cp[2], 0.0); }
    }

    #[test]
    fn g2_helical_yields_z_linear_control_points() {
        let items = collect("G1 X1 Z0 F1500\nG2 X0 Y1 Z0.5 I-1 J0\n");
        let arc = items.iter().find_map(|it| match it {
            Item::Segment(Segment::Arc(a)) => Some(a),
            _ => None,
        }).expect("ArcSegment expected");
        let cps = arc.xyz.control_points();
        // Z linear across CPs: 0.0, 0.25, 0.5
        let approx_eq = |a: f64, b: f64| (a - b).abs() < 1e-9;
        assert!(approx_eq(cps[0][2], 0.0));
        assert!(approx_eq(cps[1][2], 0.25));
        assert!(approx_eq(cps[2][2], 0.5));
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::g2`
Expected: FAIL — no `Arc` arm in `handle_event`.

- [ ] **Step 3: Implement arc construction**

In `rust/geometry/src/pipeline.rs`, add an `Arc` arm to `handle_event` (above the `_ => {}` arm):
```rust
            ReduceEvent::Arc {
                start, end, center, clockwise, z_delta: _, e_delta: _,
                feedrate_mm_s, line_no,
            } => {
                let xyz = build_arc_nurbs(start, end, center, clockwise);
                let seg = ArcSegment {
                    xyz,
                    e: None,
                    feedrate_mm_s,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Arc(seg)));
                // Arcs break the G1-junction chain; clear prev state so the
                // next G1 doesn't generate a junction against an arc endpoint.
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
```

Add the helper at the bottom of the file:
```rust
/// Build a 3D rational-quadratic NURBS arc from a center-form description.
///
/// For arcs spanning ≤ 90°, a single rational quadratic Bezier suffices.
/// For larger arcs, we split into ≤ 90° sub-arcs and concatenate via knot
/// insertion. Phase 1 supports any sweep using a multi-piece NURBS.
///
/// The 2D Bezier construction follows Piegl & Tiller §7.2 (control points
/// at start, tangent intersection P, end with weight cos(half-angle)).
/// Z is interpolated linearly across control points to support helical arcs.
fn build_arc_nurbs(
    start: [f64; 3],
    end: [f64; 3],
    center: [f64; 3],
    clockwise: bool,
) -> nurbs::VectorNurbs<f64, 3> {
    let r_start = [start[0] - center[0], start[1] - center[1]];
    let r_end = [end[0] - center[0], end[1] - center[1]];
    let radius = (r_start[0]*r_start[0] + r_start[1]*r_start[1]).sqrt();
    let start_angle = r_start[1].atan2(r_start[0]);
    let mut end_angle = r_end[1].atan2(r_end[0]);
    // Normalize sweep direction.
    let sweep = if clockwise {
        let mut s = start_angle - end_angle;
        if s < 0.0 { s += 2.0 * std::f64::consts::PI; }
        -s  // negative for CW
    } else {
        let mut s = end_angle - start_angle;
        if s < 0.0 { s += 2.0 * std::f64::consts::PI; }
        s
    };
    // For a single rational quadratic, half-sweep ≤ 90° (i.e. |sweep| ≤ π).
    // Phase 1 uses a single piece for sweeps up to 180° by setting the
    // tangent-intersection control point and weight cos(sweep/2). For very
    // wide arcs (> 180°), behavior is approximate; full multi-piece arc
    // support is Phase 2 polish.
    let half = sweep / 2.0;
    let cos_half = half.cos();
    // Mid control point at tangent intersection, in 2D first.
    let mid_x = center[0] + radius * (start_angle + half).cos() / cos_half;
    let mid_y = center[1] + radius * (start_angle + half).sin() / cos_half;
    // Z linear across 3 CPs.
    let z0 = start[2];
    let z2 = end[2];
    let z1 = (z0 + z2) / 2.0;
    let cps = vec![start, [mid_x, mid_y, z1], end];
    let _ = end_angle; // suppress unused-mut warning
    nurbs::VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        cps,
        Some(vec![1.0, cos_half, 1.0]),
    )
    .expect("rational quadratic arc construction is always valid")
}
```

Add to imports: `use crate::ArcSegment;`

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::g2`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/pipeline.rs
git commit -m "geometry/pipeline: ArcSegment emission with 3D helical control points"
```

---

## Task 21: Pipeline — telemetry routing (LayerChange, ToolChange, Retraction)

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`

- [ ] **Step 1: Write the failing tests**

Add to `tests` in `rust/geometry/src/pipeline.rs`:
```rust
    use crate::TelemetryEvent;

    #[test]
    fn layer_change_marker_fires_telemetry() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let _items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process(";LAYER:5\n", &mut sink).collect()
        };
        assert!(matches!(
            events.as_slice(),
            [TelemetryEvent::LayerChange { layer: 5, line_no: 1 }]
        ));
    }

    #[test]
    fn tool_change_fires_telemetry() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let _items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("T1\n", &mut sink).collect()
        };
        assert!(matches!(
            events.as_slice(),
            [TelemetryEvent::ToolChange { tool: 1, line_no: 1 }]
        ));
    }

    #[test]
    fn retraction_fires_telemetry() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let _items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("G1 E-1.5 F3000\n", &mut sink).collect()
        };
        assert_eq!(events.len(), 1);
        match &events[0] {
            TelemetryEvent::Retraction { e_delta_mm, line_no: 1 } => {
                assert!((e_delta_mm - (-1.5)).abs() < 1e-12);
            }
            other => panic!("expected Retraction, got {other:?}"),
        }
    }
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests`
Expected: FAIL — no telemetry emission yet.

- [ ] **Step 3: Wire telemetry routing**

In `rust/geometry/src/pipeline.rs`, in `Segments::handle_event`, add two arms **before** the `_ => {}` arm.

For comment-derived markers (forwarded by reduce from `gcode::Token::Marker`):
```rust
            ReduceEvent::CommentMarker { kind, line_no } => {
                match kind {
                    gcode::MarkerKind::LayerChange { layer } => {
                        (self.sink)(TelemetryEvent::LayerChange { layer, line_no });
                    }
                    gcode::MarkerKind::LayerType { .. } | gcode::MarkerKind::EndOfPrint => {
                        // No telemetry mapping in Phase 1; reduce as a no-op marker boundary.
                    }
                    _ => {}
                }
                // Marker terminates G1 chain.
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
```

For motion-semantic markers (G0, ZOnly, EOnly, T, M, G92), the fields `tool` and `e_delta_mm` defined on `ReduceEvent::Marker` in Task 13 carry the telemetry payload directly:
```rust
            ReduceEvent::Marker { kind, line_no, tool, e_delta_mm } => {
                match kind {
                    MotionMarkerKind::T => {
                        if let Some(tool) = tool {
                            (self.sink)(TelemetryEvent::ToolChange { tool, line_no });
                        }
                    }
                    MotionMarkerKind::EOnly => {
                        if let Some(e_delta_mm) = e_delta_mm {
                            (self.sink)(TelemetryEvent::Retraction { e_delta_mm, line_no });
                        }
                    }
                    _ => {}
                }
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml`
Expected: PASS — all pipeline + reduce tests, including the new telemetry ones.

- [ ] **Step 5: Commit**

```bash
git add rust/geometry/src/pipeline.rs rust/geometry/src/reduce.rs
git commit -m "geometry/pipeline: telemetry routing for LayerChange/ToolChange/Retraction"
```

---

## Task 22: Pipeline — Recovery dual-emit on parse errors

**Files:**
- Modify: `rust/geometry/src/pipeline.rs`
- Modify: `rust/geometry/src/reduce.rs`

Currently `reduce` swallows `Err(ParseError)` from the lexer (`let Ok(tok) = tok else { continue }`). For Phase 1, we want parse errors to surface as `Item::Recovered(Segment::Junction(_), Recovery::MalformedParams { ... })` so the consumer sees them and the sink fires a Recovery event.

For Phase 1, the simplest safe behavior: when reduce encounters a parse error, forward it through to pipeline as a new `ReduceEvent::ParseError`, and the pipeline emits `Item::Recovered` paired with a synthetic placeholder Junction at the previous position (or skips emitting any segment if no position is known).

- [ ] **Step 1: Write the failing test**

Add to `tests` in `rust/geometry/src/pipeline.rs`:
```rust
    #[test]
    fn parse_error_yields_recovered() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("G1 X1.2.3\n", &mut sink).collect()
        };
        assert_eq!(items.len(), 1);
        match &items[0] {
            Item::Recovered(_, Recovery::MalformedParams { line_no: 1, .. }) => {}
            other => panic!("expected Recovered, got {other:?}"),
        }
        // Sink should also see Recovery (dual-emit).
        assert!(matches!(
            events.as_slice(),
            [TelemetryEvent::Recovery(Recovery::MalformedParams { line_no: 1, .. })]
        ));
    }
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml pipeline::tests::parse_error_yields_recovered`
Expected: FAIL — parse errors currently dropped.

- [ ] **Step 3: Add `ReduceEvent::ParseError` and forward it**

In `rust/geometry/src/reduce.rs`, add to the `ReduceEvent` enum:
```rust
    ParseError {
        line_no: u32,
        kind: ParseErrorKind,
        text: String,
    },
```
And:
```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ParseErrorKind {
    MalformedNumber,
    UnrecognizedHead,
    EmptyCommand,
    DuplicateParam,
}
```

Replace `let Ok(tok) = tok else { continue };` with:
```rust
            let tok = match tok {
                Ok(t) => t,
                Err(e) => {
                    let (kind, line_no, text) = match e {
                        gcode::ParseError::MalformedNumber { line_no, text } =>
                            (ParseErrorKind::MalformedNumber, line_no, text.into_string()),
                        gcode::ParseError::UnrecognizedHead { line_no, head } =>
                            (ParseErrorKind::UnrecognizedHead, line_no, head.into_string()),
                        gcode::ParseError::EmptyCommand { line_no } =>
                            (ParseErrorKind::EmptyCommand, line_no, String::new()),
                        gcode::ParseError::DuplicateParam { line_no, letter } =>
                            (ParseErrorKind::DuplicateParam, line_no, letter.to_string()),
                    };
                    return Some(ReduceEvent::ParseError { line_no, kind, text });
                }
            };
```

(Note: `Box<str>::into_string()` is stable. If the compiler complains, use `text.to_string()` or `String::from(text)`.)

- [ ] **Step 4: Handle `ParseError` in pipeline**

In `rust/geometry/src/pipeline.rs`, add a `ParseError` arm to `handle_event` (above the `_ => {}` arm):
```rust
            ReduceEvent::ParseError { line_no, kind, text } => {
                let recovery = match kind {
                    crate::reduce::ParseErrorKind::MalformedNumber
                    | crate::reduce::ParseErrorKind::DuplicateParam
                    | crate::reduce::ParseErrorKind::EmptyCommand => {
                        Recovery::MalformedParams { line_no, raw: text }
                    }
                    crate::reduce::ParseErrorKind::UnrecognizedHead => {
                        Recovery::UnrecognizedCommand { line_no, head: text }
                    }
                };
                // Dual-emit: sink first per §5.1 ordering contract.
                (self.sink)(TelemetryEvent::Recovery(recovery.clone()));
                // For Phase 1, the "recovered segment" is a synthetic
                // zero-length junction at the previous position (or origin
                // if none). This carries the line through the consumer's
                // segment stream without losing it; consumer pattern-matchers
                // see Item::Recovered.
                let pos = self.prev_g1_end.unwrap_or([0.0, 0.0, 0.0]);
                let jd = JunctionDeviation {
                    position: pos,
                    angle_deg: 0.0,
                    feedrate_mm_s: self.prev_g1_feedrate.unwrap_or(0.0),
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Recovered(Segment::Junction(jd), recovery));
            }
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml`
Expected: PASS — all tests including the new Recovery one.

- [ ] **Step 6: Commit**

```bash
git add rust/geometry/src/pipeline.rs rust/geometry/src/reduce.rs
git commit -m "geometry/pipeline: parse errors → Item::Recovered + sink Recovery dual-emit"
```

---

## Task 23: Anchor test — helical_arc_3d

**Files:**
- Create: `rust/geometry/tests/helical_arc_3d.rs`

- [ ] **Step 1: Write the anchor test**

Create `rust/geometry/tests/helical_arc_3d.rs`:
```rust
//! Helical G2/G3 → 3D rational quadratic ArcSegment with linear-Z control points.
//! Locks the full-3D commitment as a tested invariant.

use geometry::{FitterParams, GeometryPipeline, Item, Segment, TelemetryEvent};

fn run(text: &str) -> Vec<Item> {
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |_: TelemetryEvent| {};
    p.process(text, &mut sink).collect()
}

#[test]
fn helical_g2_quarter_with_z_progression() {
    // Quarter-circle in XY from (1,0,0) to (0,1,0), Z progresses 0 → 0.5.
    let items = run("G1 X1 Z0 F1500\nG2 X0 Y1 Z0.5 I-1 J0\n");
    let arc = items.iter().find_map(|it| match it {
        Item::Segment(Segment::Arc(a)) => Some(a),
        _ => None,
    }).expect("expected ArcSegment");
    // Rational quadratic with 3 control points.
    assert_eq!(arc.xyz.degree(), 2);
    let cps = arc.xyz.control_points();
    assert_eq!(cps.len(), 3);
    // Weights: [1, cos(45°), 1] for a 90° arc. cos(π/4) ≈ 0.7071.
    let weights = arc.xyz.weights().expect("rational arc has weights");
    assert!((weights[0] - 1.0).abs() < 1e-12);
    assert!((weights[1] - (std::f64::consts::FRAC_1_SQRT_2)).abs() < 1e-9);
    assert!((weights[2] - 1.0).abs() < 1e-12);
    // Z linear: 0.0, 0.25, 0.5
    assert!((cps[0][2] - 0.0).abs() < 1e-12);
    assert!((cps[1][2] - 0.25).abs() < 1e-12);
    assert!((cps[2][2] - 0.5).abs() < 1e-12);
}
```

- [ ] **Step 2: Run the anchor test**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml --test helical_arc_3d`
Expected: PASS.

- [ ] **Step 3: Commit**

```bash
git add rust/geometry/tests/helical_arc_3d.rs
git commit -m "geometry: anchor test for helical 3D ArcSegment"
```

---

## Task 24: Anchor test — degenerate inputs

**Files:**
- Create: `rust/geometry/tests/degenerate_inputs.rs`

- [ ] **Step 1: Write the anchor test**

Create `rust/geometry/tests/degenerate_inputs.rs`:
```rust
//! Degenerate input handling: empty file, single G1, comment-only file,
//! malformed line. Each should surface through Item or Recovery without
//! panicking.

use geometry::{FitterParams, GeometryPipeline, Item, Recovery, TelemetryEvent};

fn run(text: &str) -> (Vec<Item>, Vec<TelemetryEvent>) {
    let mut events = vec![];
    let mut p = GeometryPipeline::new(FitterParams::default());
    let items: Vec<_> = {
        let mut sink = |e: TelemetryEvent| events.push(e);
        p.process(text, &mut sink).collect()
    };
    (items, events)
}

#[test]
fn empty_input() {
    let (items, events) = run("");
    assert!(items.is_empty());
    assert!(events.is_empty());
}

#[test]
fn comment_only_file_with_layer() {
    let (items, events) = run(";LAYER:0\n; just a comment\n");
    assert!(items.is_empty(), "comments alone produce no segments");
    assert_eq!(events.len(), 1);
    assert!(matches!(events[0], TelemetryEvent::LayerChange { layer: 0, .. }));
}

#[test]
fn single_g1_no_junction_emitted() {
    let (items, _) = run("G1 X10 F1500\n");
    assert_eq!(items.len(), 1);
    // First G1 has no preceding G1, so no junction.
}

#[test]
fn malformed_line_yields_recovered() {
    let (items, _) = run("G1 X1.2.3\n");
    assert_eq!(items.len(), 1);
    assert!(matches!(items[0], Item::Recovered(_, Recovery::MalformedParams { .. })));
}

#[test]
fn unknown_command_yields_recovered() {
    let (items, _) = run("Z1 Y2\n");
    assert_eq!(items.len(), 1);
    assert!(matches!(items[0], Item::Recovered(_, Recovery::UnrecognizedCommand { .. })));
}
```

- [ ] **Step 2: Run the anchor tests**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml --test degenerate_inputs`
Expected: PASS — all five tests.

- [ ] **Step 3: Commit**

```bash
git add rust/geometry/tests/degenerate_inputs.rs
git commit -m "geometry: anchor tests for degenerate inputs"
```

---

## Task 25: Integration test — OrcaSlicer corpus end-to-end

**Files:**
- Create: `rust/geometry/tests/integration_orca.rs`

- [ ] **Step 1: Write the integration test**

Create `rust/geometry/tests/integration_orca.rs`:
```rust
//! End-to-end smoke test on the OrcaSlicer corpus. Phase 1 emits one
//! FittedSegment per G1, plus JunctionDeviation between consecutive G1s,
//! plus ArcSegments for G2/G3. Test:
//!  - Pipeline runs to completion without panic.
//!  - Segment counts are within sane order-of-magnitude.
//!  - Telemetry sees expected events.

use geometry::{
    FitterParams, GeometryPipeline, Item, Recovery, Segment, TelemetryEvent,
};
use std::path::Path;

const CORPUS_DIR: &str = "../../scripts/fitter_prototype/corpus";

fn read_corpus_file(name: &str) -> Option<String> {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join(CORPUS_DIR).join(name);
    std::fs::read_to_string(&path).ok()
}

#[derive(Default)]
struct Counts {
    fitted: u64,
    arc: u64,
    junction: u64,
    corner_blend: u64,
    recovered: u64,
    fatal: u64,
    layer_changes: u64,
    tool_changes: u64,
    retractions: u64,
}

fn run_corpus(name: &str) -> Option<Counts> {
    let text = read_corpus_file(name)?;
    let mut counts = Counts::default();
    let mut p = GeometryPipeline::new(FitterParams::default());
    let mut sink = |e: TelemetryEvent| match e {
        TelemetryEvent::LayerChange { .. } => counts.layer_changes += 1,
        TelemetryEvent::ToolChange { .. } => counts.tool_changes += 1,
        TelemetryEvent::Retraction { .. } => counts.retractions += 1,
        _ => {}
    };
    for item in p.process(&text, &mut sink) {
        match item {
            Item::Segment(Segment::Fitted(_)) => counts.fitted += 1,
            Item::Segment(Segment::Arc(_)) => counts.arc += 1,
            Item::Segment(Segment::Junction(_)) => counts.junction += 1,
            Item::Segment(Segment::CornerBlend(_)) => counts.corner_blend += 1,
            Item::Recovered(_, _) => counts.recovered += 1,
            Item::Fatal(_) => {
                counts.fatal += 1;
                break;
            }
        }
    }
    Some(counts)
}

#[test]
fn arc_fitted_corpus_runs_end_to_end() {
    let Some(c) = run_corpus("voron_cube_arc_fitted.gcode") else {
        eprintln!("skipping: corpus file not present");
        return;
    };
    eprintln!(
        "arc_fitted: fitted={} arc={} junction={} cornerblend={} recovered={} \
         fatal={} layers={} tools={} retracts={}",
        c.fitted, c.arc, c.junction, c.corner_blend, c.recovered,
        c.fatal, c.layer_changes, c.tool_changes, c.retractions,
    );
    assert_eq!(c.fatal, 0, "Phase 1 should not fatal on legitimate input");
    assert_eq!(c.corner_blend, 0, "Phase 1 emits no CornerBlendSlot");
    assert!(c.fitted > 100_000, "expected > 100k FittedSegments");
    assert!(c.arc > 5_000, "expected > 5k ArcSegments (corpus has ~9710 G2/G3)");
    assert!(c.layer_changes >= 1, "expected at least one LayerChange");
}

#[test]
fn straight_line_corpus_runs_end_to_end() {
    let Some(c) = run_corpus("voron_cube_straight_line.gcode") else {
        eprintln!("skipping: corpus file not present");
        return;
    };
    assert_eq!(c.fatal, 0);
    assert_eq!(c.arc, 0, "straight-line corpus has no G2/G3");
    assert!(c.fitted > 150_000);
}
```

- [ ] **Step 2: Run the integration tests**

Run: `cargo test -p geometry --manifest-path rust/Cargo.toml --test integration_orca -- --nocapture`
Expected: PASS — both tests run on the committed corpus, no fatals, expected counts hit.

- [ ] **Step 3: Commit**

```bash
git add rust/geometry/tests/integration_orca.rs
git commit -m "geometry: integration test on OrcaSlicer corpus"
```

---

## Task 26: Verify the full Phase 1 build

- [ ] **Step 1: Run the full test suite**

Run: `cargo test --manifest-path rust/Cargo.toml --all`
Expected: PASS — all unit tests, all integration tests, all property tests, both crates.

- [ ] **Step 2: Run clippy with workspace lints**

Run: `cargo clippy --manifest-path rust/Cargo.toml --all-targets -- -D warnings`
Expected: zero warnings. Fix any that appear; common ones are unused imports, missing docs on public items, or `must_use` candidates.

- [ ] **Step 3: Run `cargo build --release` to verify release-mode compiles**

Run: `cargo build --manifest-path rust/Cargo.toml --release`
Expected: succeeds with no errors and no warnings.

- [ ] **Step 4: Spot-check release-mode behavior on the corpus**

Run: `cargo test --manifest-path rust/Cargo.toml --release --test integration_orca -- --nocapture`
Expected: PASS, with similar timings to debug (Phase 1 is not a perf target).

- [ ] **Step 5: Commit any final lint/cleanup fixes if applied**

```bash
# Only if step 2 required edits.
git status
git add -p
git commit -m "geometry/gcode: clippy cleanup for Phase 1 ship"
```

---

## Task 27: Phase 1 ship — README + crate docs

**Files:**
- Create: `rust/gcode/README.md`
- Create: `rust/geometry/README.md`

- [ ] **Step 1: Write `rust/gcode/README.md`**

```markdown
# `gcode`

G-code lexer for the kalico motion planner. Pure text → typed tokens. No
NURBS dependency; no motion semantics. Reusable outside the planner for
offline analysis tools, replay tooling, or fuzz-target use.

See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`
for the architecture.

## Public surface

- `gcode::lex(&str)` returns `impl Iterator<Item = Result<Token, ParseError>>`.
- `Token` variants: `Command { letter, major, minor, params, line_no }`,
  `Comment { text, line_no }`, `Marker { kind, line_no }`.
- `MarkerKind` variants (slicer dialect): `LayerChange`, `LayerType`,
  `EndOfPrint`.
- `ParseError` variants: `MalformedNumber`, `UnrecognizedHead`,
  `EmptyCommand`, `DuplicateParam`.

## Fuzzing

`cargo +nightly fuzz run lex` from `rust/gcode/fuzz/`. Treat any panic or
hang as P0.

## What this crate does NOT do

- Interpret motion semantics (G2 = arc, G92 = set position, etc.).
- Track modal state.
- Construct NURBS or geometric segments.

Those are `geometry/`'s job.
```

- [ ] **Step 2: Write `rust/geometry/README.md`**

```markdown
# `geometry`

Layer 1 geometry pipeline for the kalico motion planner. Token stream →
typed segments. Phase 1 (current): degree-1 NURBS for G1, 3D rational
quadratic for G2/G3, JunctionDeviation between consecutive G1s. Phase 2
adds the LSPIA fitter, classifier, and corner-blend slot construction.

See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`.

## Public surface

```rust
let mut pipeline = GeometryPipeline::new(FitterParams::default());
let mut sink = |event: TelemetryEvent| { /* observability */ };
for item in pipeline.process(&gcode_text, &mut sink) {
    match item {
        Item::Segment(s) => { /* normal */ }
        Item::Recovered(s, recovery) => { /* anomaly + segment */ }
        Item::Fatal(f) => { /* terminal */ break; }
    }
}
```

## Phase 1 vs Phase 2

Public API is identical across phases. Phase 1 produces a subset of segment
kinds:

- Emitted in Phase 1: `Segment::Fitted` (degree 1 only), `Segment::Arc`,
  `Segment::Junction`.
- Defined but never produced in Phase 1: `Segment::CornerBlend`,
  `Recovery::WindowCapHit`, `Recovery::DegenerateSlotFallback`,
  `Recovery::ToleranceExceeded`, `Recovery::LspiaNotConverged`,
  `TelemetryEvent::WindowFlush`, `TelemetryEvent::FitObservation`.

Consumers handle the full enum from day one — no API churn at the phase
boundary.
```

- [ ] **Step 3: Verify the docs build cleanly**

Run: `cargo doc --manifest-path rust/Cargo.toml -p gcode -p geometry --no-deps`
Expected: succeeds, both crates' rustdoc renders without warnings.

- [ ] **Step 4: Commit**

```bash
git add rust/gcode/README.md rust/geometry/README.md
git commit -m "docs: README for gcode and geometry crates (Phase 1)"
```

---

## Phase 1 complete

At this point:

- `rust/gcode/` is feature-complete: lexer, marker matchers, ParseError taxonomy, fuzz target, golden corpus snapshot, property tests.
- `rust/geometry/` Phase 1 is feature-complete: pipeline emits FittedSegment (degree-1), ArcSegment (3D helical), JunctionDeviation between G1 transitions, Recovery dual-emit on parse errors, telemetry routing for LayerChange/ToolChange/Retraction.
- Public API surface stable across the Phase 1/Phase 2 boundary; consumers handle the full enum from day one.
- All anchor tests + integration tests pass; corpus runs end-to-end without fatals.
- `cargo clippy -- -D warnings` clean.

Phase 2 (classifier + LSPIA fitter + corner-blend slot construction) gets its own plan after Phase 1 lands.
