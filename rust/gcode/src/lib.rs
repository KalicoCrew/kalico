//! G-code lexer. Pure text → typed tokens. No motion semantics. No NURBS.
//! See `docs/superpowers/specs/2026-04-26-layer-1-rust-architecture-design.md`.

#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod error;
pub mod lexer;
pub mod marker;
pub mod token;

pub use error::ParseError;
pub use marker::MarkerKind;
pub use token::{Params, Token};

// TODO(task 4): lexer::lex function body—streaming lexer over bytes, emitting Token or ParseError
