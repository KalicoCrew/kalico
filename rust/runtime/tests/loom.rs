//! Loom feature gate scaffold. Spec §6.2 / §6.8.
//!
//! Loom can't model cortex-m IRQs, but it exhaustively explores Acquire/
//! Release interleavings on the trace `overflow_pending` bool and other
//! atomic-only synchronization points. Full loom coverage is deferred to
//! Step 6 — when the live producer task lands, the actual atomic-ordering
//! model becomes load-bearing and the loom suite earns its keep.
//!
//! For Step 5, the gate is in place so Step 6's first task is wiring the
//! atomic-ordering model itself, not setting up the `cfg(feature = "loom")`
//! infrastructure. See `rust/runtime/Cargo.toml` for the `loom` feature.

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
