//! G-code lexer. Pure text → typed tokens. No motion semantics. No NURBS.
//! See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`.

#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod error;
pub mod lexer;
pub mod marker;
pub mod token;

// TODO(task 2/4): restore plan re-exports once lexer.rs / token.rs / marker.rs are populated:
//   pub use error::ParseError;
//   pub use lexer::lex;
//   pub use marker::MarkerKind;
//   pub use token::{Params, Token};
