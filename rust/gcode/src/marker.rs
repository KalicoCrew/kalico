//! Slicer-dialect comment-pattern matchers. Covers `OrcaSlicer` / `PrusaSlicer`
//! / `BambuStudio`. The matcher itself lands in Task 8; this module just defines
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