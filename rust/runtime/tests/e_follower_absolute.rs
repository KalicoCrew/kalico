//! E-follower absolute-position tests — removed.
//!
//! The `evaluate_e_axis` function and the entire Phase-3 E-follows-XY
//! evaluator were removed in the E-unification refactor. E is now a regular
//! Bezier axis pre-computed by the host; the MCU evaluates all four axes
//! (A/B/Z/E) uniformly with no XY arc-length integration, no PA correction,
//! and no `engine_segment_base_e` / `ds_xy_segment` state.
//!
//! The per-axis uniform evaluation is covered by `tick_integration.rs`'s
//! `e_axis_evaluated_uniformly_like_other_axes` test.

// This module intentionally contains no tests.
