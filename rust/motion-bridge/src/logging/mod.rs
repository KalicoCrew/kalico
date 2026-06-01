//! Structured logging for the Rust host (Stage 2 of the observability pipeline).
//! Emits the same NDJSON schema as the Stage 1 Python host into
//! `<events_dir>/host-rust.jsonl`.

pub mod context;
pub mod schema;
pub mod writer;
