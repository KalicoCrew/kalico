#![cfg_attr(not(test), forbid(unsafe_code))]

pub mod error;
pub mod params;
pub mod pipeline;
pub(crate) mod reduce;
pub mod segment;
pub mod splitter;
pub mod telemetry;

pub use error::{Fatal, GeometryError, InternalDetails, InternalKind, Recovery, SlotDegeneracy};
pub use params::FitterParams;
pub use pipeline::{GeometryPipeline, Item, Segments, degree_elevate_2_to_3};
pub use segment::{
    BlendFamily, CornerBlendSlot, CubicSegment, EMode, JunctionDeviation, Segment, SourceRange,
    SplitInfo,
};
pub use splitter::{SplitError, split_segment_to_cap};

pub use telemetry::TelemetryEvent;
