// Phase 1 — Streaming-shaper module: skeletal types only.
//
// This module is the new home of the per-axis stateful trajectory queue that
// will eventually replace the per-batch pad-and-trim shaping driven by
// `shape_batch`. Phase 1 introduces *only* the data structures and a thin
// `append_batch` shim that delegates to the existing pad → shape → refit
// pipeline; subsequent phases progressively replace the shim with
// history-aware behaviour. See:
//
// - Spec: `docs/superpowers/specs/2026-05-10-streaming-shaper-design.md` §3.1
// - Plan: `docs/superpowers/plans/2026-05-10-streaming-shaper.md` Phase 1
//
// **Behaviour invariant (Phase 1):** for any single-segment input, the output
// of `append_batch` followed by `drain_committed` is byte-identical to a
// direct call sequence of `pad::pad_segment_axis` → `shaper::shape_axis` →
// `refit::refit_to_cubic` (with passthrough Z falling through to the fitted
// axis exactly as `beta::run_one_iteration` does it). The unit tests below
// pin that invariant.

use std::collections::VecDeque;

use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;

use crate::fit::FittedSegment;
use crate::AxisShaper;
use crate::ShapedSegment;

/// Per-axis unshaped trajectory queue + kernel + half-support.
///
/// `pieces` accumulates the unshaped polynomial pieces that the convolution
/// must see (history, current, lookahead). Phase 1 keeps `pieces` populated
/// only with the `home_pos` rest-extension seed; phases 2/3 fill it from
/// real `append_batch` input and trim it against the dispatched cursor.
#[derive(Debug, Clone)]
pub struct AxisShaperQueue {
    /// Unshaped polynomial pieces, in time order. See module docs.
    pub pieces: VecDeque<BezierPiece<f64>>,
    /// Smooth-shaper kernel for this axis. `None` for passthrough.
    pub kernel: Option<PiecewisePolynomialKernel<f64>>,
    /// Kernel half-support (seconds). Equal to `T_sm / 2` for active shapers,
    /// `0.0` for passthrough.
    pub h: f64,
}

/// Stateful streaming-shaper planner state, sharing one absolute time line
/// across all axes (every append is multi-axis).
///
/// Phase 1 only uses `axes` and `pending_dispatch`; the cursors are seeded
/// to zero / left untouched by `append_batch`. Phase 3 (`append_and_replan`
/// / `emit_committed`) drives the cursors (`t_appended`, `t_decel_start`,
/// `t_shaped`, `t_dispatched`).
///
/// **v5 field set.** The v4-era fields `t_tentative`, `rest_tentative`, and
/// `generation` were removed when v5's design eliminated the
/// tentative-rest extension model — the streaming planner now appends each
/// move's terminal decel-to-zero outright and tracks where that decel begins
/// in `t_decel_start`. See spec §3.1 ("State invariants").
#[derive(Debug)]
pub struct ShaperState {
    /// Per-axis queues (X, Y, Z, E). Z is typically passthrough; E is unused
    /// in Phase 1 (extruder is followed off the shaped XY arc-length and is
    /// not a shaped axis in CLAUDE.md's MVP scope).
    pub axes: [AxisShaperQueue; 4],

    /// Latest absolute time for which a real `append_batch` has been received.
    pub t_appended: f64,
    /// Absolute time at which the most-recently-submitted move's terminal
    /// decel-to-zero begins. Phase 3's `append_and_replan` populates this
    /// from the planner's velocity profile so the next `submit_move` can
    /// rewind to it and re-plan the un-committed tail. Initialized to
    /// `0.0` at construction; equal to `t_appended` when the queue is empty.
    pub t_decel_start: f64,
    /// Latest absolute time for which a shaped sample has been computed.
    pub t_shaped: f64,
    /// Latest absolute time for which a shaped sample has been *dispatched*
    /// to the wire.
    pub t_dispatched: f64,

    /// Shaped output computed but not yet drained / dispatched. Populated
    /// transiently by Phase 3's `emit_committed` and by Phase 1's
    /// `append_batch` shim; drained via `drain_committed`.
    pub pending_dispatch: Vec<ShapedSegment>,
}

impl ShaperState {
    /// Construct a fresh streaming-shaper state at `home_pos` for each axis,
    /// with the per-axis kernels in `shapers`. Each axis queue is seeded with
    /// a `(home_pos[i], v=0)` rest extension covering `[-( h + δ_safety ), 0]`.
    ///
    /// `δ_safety` is set to `h` (so the initial seed spans `2 * h` of past)
    /// per open-question 2 in the spec. For a passthrough axis (`h = 0`) the
    /// seed has zero duration and is omitted.
    #[must_use]
    pub fn new(home_pos: [f64; 4], shapers: &[Option<AxisShaper>; 4]) -> Self {
        let axes: [AxisShaperQueue; 4] =
            std::array::from_fn(|i| build_axis_queue(home_pos[i], shapers[i]));

        Self {
            axes,
            t_appended: 0.0,
            t_decel_start: 0.0,
            t_shaped: 0.0,
            t_dispatched: 0.0,
            pending_dispatch: Vec::new(),
        }
    }

    /// **Phase 1 shim.** Run the existing per-segment pad → shape → refit
    /// pipeline on `fitted` and stage the resulting `ShapedSegment` into
    /// `pending_dispatch`. The internal queue state (`axes`, `t_appended`,
    /// etc.) is intentionally left untouched here — Phase 2 replaces this
    /// shim with real history-aware logic.
    ///
    /// Returns `Ok(())` on success. Errors from the algebra pipeline are
    /// surfaced via `nurbs::AlgebraError`; refit and other failures are
    /// flattened into `AlgebraError::DegreeMismatch` placeholders so the
    /// shim's signature stays narrow until Phase 2 widens it.
    pub fn append_batch(&mut self, fitted: &FittedSegment) -> Result<(), nurbs::AlgebraError> {
        let shaped = shape_single_segment(fitted, &self.axes)?;
        self.pending_dispatch.push(shaped);
        Ok(())
    }

    /// Drain `pending_dispatch`, returning all shaped segments that are
    /// ready for the wire. Clears the field.
    pub fn drain_committed(&mut self) -> Vec<ShapedSegment> {
        std::mem::take(&mut self.pending_dispatch)
    }
}

// ---------------------------------------------------------------------------
// Construction helpers
// ---------------------------------------------------------------------------

fn build_axis_queue(home_pos: f64, shaper: Option<AxisShaper>) -> AxisShaperQueue {
    let kernel = shaper.and_then(|s| s.to_kernel());
    let h = match shaper {
        Some(AxisShaper::SmoothZv { frequency_hz }) => 0.8025 / frequency_hz / 2.0,
        Some(AxisShaper::SmoothMzv { frequency_hz }) => 0.95625 / frequency_hz / 2.0,
        Some(AxisShaper::Passthrough) | None => 0.0,
    };

    let mut pieces = VecDeque::new();

    // Seed with a `(home_pos, v=0)` rest extension over `[-(h + δ_safety), 0]`.
    // `δ_safety = h` per spec open-question 2. For passthrough axes (`h = 0`)
    // the seed would be a zero-duration piece, which is degenerate; skip it.
    if h > 0.0 {
        let delta_safety = h;
        let total = h + delta_safety;
        pieces.push_back(BezierPiece {
            u_start: -total,
            u_end: 0.0,
            // Pascal-shifted monomial basis: a constant `home_pos` is just
            // `coeffs = [home_pos]`.
            coeffs: vec![home_pos],
        });
    }

    AxisShaperQueue { pieces, kernel, h }
}

// ---------------------------------------------------------------------------
// Phase 1 shim: delegate to the existing per-segment pipeline
// ---------------------------------------------------------------------------

/// Apply the existing pad → shape → refit pipeline to a single fitted
/// segment, mirroring exactly what `beta::run_one_iteration` does for a
/// stand-alone segment with no E gaps and no neighbours. The output is
/// byte-identical to that path; the streaming module's
/// `append_batch` is just a struct-shaped re-entry.
fn shape_single_segment(
    fitted: &FittedSegment,
    axes: &[AxisShaperQueue; 4],
) -> Result<ShapedSegment, nurbs::AlgebraError> {
    use crate::pad::pad_segment_axis;
    use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
    use crate::shaper::shape_axis;

    let t_start = fitted.t_start;
    let t_end = fitted.t_end;

    // Single-segment slice for the existing pad implementation.
    let fitted_slice = std::slice::from_ref(fitted);

    let mut shaped_axes: [Option<nurbs::ScalarNurbs<f64>>; 3] = [None, None, None];

    for axis in 0..3 {
        let q = &axes[axis];
        let axis_shaped = if let Some(kernel) = q.kernel.as_ref() {
            let padded = pad_segment_axis(0, axis, fitted_slice, &[], q.h, t_start, t_end);
            shape_axis(&padded, kernel, t_start, t_end)?
        } else {
            // Passthrough — use the fitted axis directly. Mirrors the
            // `kernels.z = None` branch in `beta::run_one_iteration`.
            fitted.axes[axis].clone()
        };

        // Match `beta::run_one_iteration`: refit *every* axis (including the
        // passthrough Z) to cubic Bézier. Without this the streaming shim's
        // output would diverge byte-for-byte from the existing pipeline.
        //
        // `refit_to_cubic` returns `nurbs::algebra::FitError`; surface it
        // through `AlgebraError::NotImplemented` until Phase 2 widens the
        // error type. Refit failures should not happen on the existing
        // production input (the shim's caller has already exercised this
        // path through `shape_batch`), so this is purely a defensive map.
        let refit = refit_to_cubic(&axis_shaped, REFIT_TOLERANCE_MM).map_err(|_| {
            nurbs::AlgebraError::NotImplemented(
                "streaming::append_batch: refit_to_cubic failed (Phase 1 shim)",
            )
        })?;
        shaped_axes[axis] = Some(refit);
    }

    Ok(ShapedSegment {
        axes: [
            shaped_axes[0].take().unwrap(),
            shaped_axes[1].take().unwrap(),
            shaped_axes[2].take().unwrap(),
        ],
        // Phase 1 has no E plumbing — match `beta::assemble_with_e_gaps`'s
        // default for `EMode::CoupledToXy`-with-zero-ratio; the planner-side
        // wiring (Task 1.2) will overwrite with the real input metadata.
        e_mode: geometry::segment::EMode::CoupledToXy,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start,
        t_end,
    })
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::build_smooth_zv_kernel;
    use crate::pad::pad_segment_axis;
    use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
    use crate::shaper::shape_axis;
    use crate::{AxisShaper, RequiredShaper};
    use nurbs::bezier::{bezier_pieces_to_nurbs, BezierPiece};
    use nurbs::ScalarNurbs;

    /// Build a simple linear-move `FittedSegment`: X linear from 0 → 10,
    /// Y and Z constant at 0, on `t ∈ [0, 1]`.
    fn linear_segment() -> FittedSegment {
        let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 10.0],
        }]);
        let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0],
        }]);
        let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0],
        }]);
        FittedSegment {
            axes: [x_nurbs, y_nurbs, z_nurbs],
            t_start: 0.0,
            t_end: 1.0,
        }
    }

    /// Byte-equivalent NURBS comparator: same degree, same knots, same
    /// control points, same weight presence. We compare with `==` on `f64`
    /// (NaN-free in this pipeline) to make "byte-equivalent" literal.
    fn assert_nurbs_byte_equal(a: &ScalarNurbs<f64>, b: &ScalarNurbs<f64>, label: &str) {
        assert_eq!(a.degree(), b.degree(), "{label}: degree differs");
        assert_eq!(a.knots(), b.knots(), "{label}: knots differ");
        assert_eq!(
            a.control_points(),
            b.control_points(),
            "{label}: control points differ"
        );
        assert_eq!(
            a.weights().is_some(),
            b.weights().is_some(),
            "{label}: weight presence differs"
        );
        if let (Some(wa), Some(wb)) = (a.weights(), b.weights()) {
            assert_eq!(wa, wb, "{label}: weights differ");
        }
    }

    #[test]
    #[allow(clippy::float_cmp)] // Time bounds and cursor zeros are exact-by-construction.
    fn shim_matches_direct_pipeline_for_single_linear_move() {
        let fitted = linear_segment();
        let freq = 60.0;
        let h = 0.8025 / freq / 2.0;
        let kernel = build_smooth_zv_kernel(0.8025 / freq);

        // ---- Method A: streaming shim ----
        let shapers: [Option<AxisShaper>; 4] = [
            Some(AxisShaper::SmoothZv { frequency_hz: freq }),
            Some(AxisShaper::SmoothZv { frequency_hz: freq }),
            Some(AxisShaper::Passthrough),
            Some(AxisShaper::Passthrough),
        ];
        let mut state = ShaperState::new([0.0, 0.0, 0.0, 0.0], &shapers);
        state.append_batch(&fitted).expect("shim should succeed");
        let shim_out = state.drain_committed();
        assert_eq!(shim_out.len(), 1, "shim should emit exactly one segment");
        let shim_seg = &shim_out[0];

        // After draining, `pending_dispatch` is empty.
        assert!(state.pending_dispatch.is_empty());
        // Re-draining yields nothing.
        assert!(state.drain_committed().is_empty());

        // ---- Method B: direct call sequence (mirrors `beta::run_one_iteration`) ----
        let fitted_slice = std::slice::from_ref(&fitted);

        // X: shaped + refit.
        let x_padded = pad_segment_axis(0, 0, fitted_slice, &[], h, 0.0, 1.0);
        let x_shaped = shape_axis(&x_padded, &kernel, 0.0, 1.0).unwrap();
        let x_refit = refit_to_cubic(&x_shaped, REFIT_TOLERANCE_MM).unwrap();

        // Y: shaped + refit (Y also SmoothZv at the same freq → same kernel).
        let y_padded = pad_segment_axis(0, 1, fitted_slice, &[], h, 0.0, 1.0);
        let y_shaped = shape_axis(&y_padded, &kernel, 0.0, 1.0).unwrap();
        let y_refit = refit_to_cubic(&y_shaped, REFIT_TOLERANCE_MM).unwrap();

        // Z: passthrough → still refit.
        let z_passthrough = fitted.axes[2].clone();
        let z_refit = refit_to_cubic(&z_passthrough, REFIT_TOLERANCE_MM).unwrap();

        // ---- Compare byte-for-byte ----
        assert_nurbs_byte_equal(&shim_seg.axes[0], &x_refit, "X");
        assert_nurbs_byte_equal(&shim_seg.axes[1], &y_refit, "Y");
        assert_nurbs_byte_equal(&shim_seg.axes[2], &z_refit, "Z");

        // Time bounds match the input.
        assert_eq!(shim_seg.t_start, 0.0);
        assert_eq!(shim_seg.t_end, 1.0);
    }

    #[test]
    #[allow(clippy::float_cmp)] // Cursor zeros and h=0 for passthrough are exact-by-construction.
    fn new_seeds_axis_queues_with_rest_extension() {
        let shapers: [Option<AxisShaper>; 4] = [
            Some(AxisShaper::SmoothZv {
                frequency_hz: 100.0,
            }),
            Some(AxisShaper::SmoothMzv {
                frequency_hz: 80.0,
            }),
            Some(AxisShaper::Passthrough),
            None,
        ];
        let state = ShaperState::new([1.0, 2.0, 3.0, 4.0], &shapers);

        // Active axes get a single seed piece spanning `2h` of the past
        // (`δ_safety = h`).
        let h_x = 0.8025 / 100.0 / 2.0;
        assert_eq!(state.axes[0].pieces.len(), 1);
        let seed_x = &state.axes[0].pieces[0];
        assert!((seed_x.u_start - (-2.0 * h_x)).abs() < 1e-15);
        assert_eq!(seed_x.u_end, 0.0);
        assert_eq!(seed_x.coeffs, vec![1.0]);
        assert!((state.axes[0].h - h_x).abs() < 1e-15);
        assert!(state.axes[0].kernel.is_some());

        let h_y = 0.95625 / 80.0 / 2.0;
        assert_eq!(state.axes[1].pieces.len(), 1);
        let seed_y = &state.axes[1].pieces[0];
        assert!((seed_y.u_start - (-2.0 * h_y)).abs() < 1e-15);
        assert_eq!(seed_y.coeffs, vec![2.0]);

        // Passthrough — h = 0, no seed piece, no kernel.
        assert!(state.axes[2].pieces.is_empty());
        assert_eq!(state.axes[2].h, 0.0);
        assert!(state.axes[2].kernel.is_none());

        // None — same as Passthrough for the seed/kernel; recorded for E.
        assert!(state.axes[3].pieces.is_empty());
        assert_eq!(state.axes[3].h, 0.0);
        assert!(state.axes[3].kernel.is_none());

        // Cursors start at zero.
        assert_eq!(state.t_appended, 0.0);
        assert_eq!(state.t_decel_start, 0.0);
        assert_eq!(state.t_shaped, 0.0);
        assert_eq!(state.t_dispatched, 0.0);
        assert!(state.pending_dispatch.is_empty());
    }

    #[test]
    fn required_shaper_h_matches_axis_shaper_h() {
        // Sanity: the half-support computation matches `RequiredShaper::to_kernel`'s
        // own conversion (`0.8025 / freq` → support `[-h, h]`).
        let shapers: [Option<AxisShaper>; 4] = [
            Some(AxisShaper::SmoothZv {
                frequency_hz: 186.0,
            }),
            Some(AxisShaper::SmoothMzv {
                frequency_hz: 122.0,
            }),
            Some(AxisShaper::Passthrough),
            None,
        ];
        let state = ShaperState::new([0.0; 4], &shapers);

        let kernel_x = RequiredShaper::SmoothZv {
            frequency_hz: 186.0,
        }
        .to_kernel();
        let (lo_x, hi_x) = kernel_x.support();
        let expected_h_x = (hi_x - lo_x) / 2.0;
        assert!((state.axes[0].h - expected_h_x).abs() < 1e-15);

        let kernel_y = RequiredShaper::SmoothMzv {
            frequency_hz: 122.0,
        }
        .to_kernel();
        let (lo_y, hi_y) = kernel_y.support();
        let expected_h_y = (hi_y - lo_y) / 2.0;
        assert!((state.axes[1].h - expected_h_y).abs() < 1e-15);
    }
}
