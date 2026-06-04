//! SOCP numerical-conditioning regression tests.
//!
//! Spec §11 (numerical conditioning). The rational-quadratic arc fixture was
//! removed when rational NURBS support was eliminated; the underlying
//! conditioning fix (block-(c) RHS cap) remains in the solver.

// No tests remain after rational NURBS removal.
// Polynomial curved-arc regression tests live in `path/tests.rs`.
