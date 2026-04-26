//! `ParseError`: errors that can arise during tokenization.

use thiserror::Error;

#[derive(Debug, Error)]
#[non_exhaustive]
pub enum ParseError {}
