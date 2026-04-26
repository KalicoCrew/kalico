//! G-code lexer. Pure text → typed tokens. No motion semantics. No NURBS.
//! See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`.

#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod error;
pub mod lexer;
pub mod marker;
pub mod token;

pub use marker::MarkerKind;
pub use token::{Params, Token};

// TODO(task 3/4): restore plan re-exports once error.rs / lexer.rs are populated:
//   pub use error::ParseError;
//   pub use lexer::lex;
