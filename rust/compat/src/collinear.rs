//! G1 → G5 reduction: collinear cubic Bézier.
//!
//! A G1 linear move is converted to a single-piece cubic Bézier (G5) with
//! collinear control points at 1/3 and 2/3 lerp along the segment. This is
//! an exact degree-elevation — zero fit error.
//!
//! Implementation is in Task 2.
