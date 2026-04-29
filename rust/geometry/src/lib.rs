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
    ArcSegment, BlendFamily, CornerBlendSlot, EMode, FittedSegment, JunctionDeviation, Segment,
    SourceRange, SplitInfo,
};
pub use telemetry::TelemetryEvent;
