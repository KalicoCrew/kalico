#![allow(
    clippy::ref_as_ptr,
    clippy::float_cmp,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::too_many_lines,
    clippy::uninlined_format_args,
    clippy::doc_markdown
)]
#![cfg(feature = "loom")]

#[cfg(feature = "loom")]
mod loom_tests {
    // TODO Step-6: wire `loom::sync::atomic::*` through the existing
    // `TraceRing` and `Engine` atomics under `cfg(loom)` and write the
    // first interleaving model — e.g. producer try_emit / consumer
    // drain_into on `overflow_pending`. The crate currently uses
    // `core::sync::atomic` unconditionally; swapping to a `cfg(loom)`-
    // gated re-export is a prerequisite for any actual loom test body.
}
