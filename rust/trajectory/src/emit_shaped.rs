//! Phase-2 Task-2.2: shaping half of the streaming-shaper split.
//!
//! `emit_shaped` consumes the time-domain β-converged [`FittedSegment`]s
//! produced by [`crate::plan_velocity`] (Task 2.1) and runs the
//! pad → convolve → trim → C¹ refit pipeline that the existing `shape_batch`
//! does inline. It returns one [`ShapedSegment`] per input segment, ready
//! for the wire (E-gap insertion is the caller's job; see
//! [`crate::shape_batch`]).
//!
//! The streaming planner (`ShaperState::emit_committed` in Phase 3) calls
//! this with a non-empty [`PerAxisHistory`] supplying real prior planned
//! pieces as the left-pad source — the convolution sees a continuous time
//! line across `submit_move` boundaries instead of the constant-extension
//! seam that v5 of the spec exists to eliminate.
//!
//! The right-pad still uses constant-extension at `batch_t_end` (current
//! `pad_segment_axis` semantics). Phase 3 swaps the right-pad for held-back
//! planned-tail content; that lives in `ShaperState::emit_committed`'s
//! caller, not here.
//!
//! See `docs/superpowers/specs/2026-05-10-streaming-shaper-design.md` §3.3.

use nurbs::algebra::PiecewisePolynomialKernel;
use nurbs::bezier::BezierPiece;
use nurbs::ScalarNurbs;

use crate::beta::kernel_half_support;
use crate::fit::FittedSegment;
use crate::pad::{pad_segment_axis_with_history, EHalo};
use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
use crate::shaper::shape_axis;
use crate::{ShapeError, ShapedSegment};

/// Per-axis history of prior planned `BezierPiece`s, in absolute time order,
/// immediately preceding `batch_t_start`.
///
/// Slot ordering matches [`crate::plan_velocity::PlanInput::kernels`]:
/// `[X, Y, Z, E]`. The E slot is unused today (the extruder is not a
/// shaped axis); it is kept for forward-compatibility with
/// [`crate::streaming::ShaperState`]'s 4-axis layout.
///
/// Empty per-axis slices fall back to constant-extension at `batch_t_start`,
/// reproducing `pad_segment_axis`'s pre-streaming behaviour byte-for-byte.
#[derive(Debug, Clone, Copy)]
pub struct PerAxisHistory<'a> {
    /// Prior planned pieces per axis, in time order. Each axis is
    /// independent — the history may be present on some axes and empty on
    /// others (e.g., during bring-up where Z is passthrough and the E queue
    /// has no shaping context).
    pub axes: [&'a [BezierPiece<f64>]; 4],
}

impl PerAxisHistory<'_> {
    /// Construct an empty history (all four axes are empty slices).
    /// `emit_shaped` with an empty history reproduces `shape_batch`'s
    /// pre-streaming output byte-for-byte.
    #[must_use]
    pub const fn empty() -> Self {
        Self {
            axes: [&[], &[], &[], &[]],
        }
    }
}

impl Default for PerAxisHistory<'_> {
    fn default() -> Self {
        Self::empty()
    }
}

/// Per-segment metadata that survives [`emit_shaped`]'s shape-and-refit step
/// onto the resulting [`ShapedSegment`].
///
/// `e_independent` and `feedrate_mm_s` are intentionally absent — those
/// belong to E-gap segments which `emit_shaped` does not produce. Callers
/// that need to interleave E gaps (i.e., `shape_batch`) handle that
/// post-`emit_shaped`.
#[derive(Debug, Clone, Copy)]
pub struct EmitSegmentMeta {
    /// E-axis mode classification, forwarded onto the output [`ShapedSegment`].
    pub e_mode: geometry::segment::EMode,
    /// Extrusion ratio (mm E per mm XY arc length); forwarded onto output.
    pub extrusion_per_xy_mm: f64,
}

/// Run the shaping half of the shaper pipeline.
///
/// For each segment in `planned`:
///
/// 1. **Pad** each of axes 0/1/2 with neighbour data + history (left) and
///    constant-extension at `batch_t_end` (right).
/// 2. **Convolve** the padded curve with that axis's kernel; or pass the
///    fitted axis through unchanged when the kernel is `None`.
/// 3. **Refit** each post-shape axis to a chain of cubic Bézier pieces with
///    C¹ continuity (CLAUDE.md's "uniform cubic Bézier across Layer 1/2/3/4"
///    invariant at the post-shape boundary).
/// 4. Assemble a [`ShapedSegment`] with `meta[i]`'s e-mode metadata.
///
/// `kernels` mirrors [`crate::plan_velocity::PlanInput::kernels`] in slot
/// ordering: `[X, Y, Z, E]`. Only the first three slots are consulted; the
/// E slot is reserved for forward compatibility. `Some(kernel)` triggers
/// pad+convolve+trim; `None` is passthrough (used for Z when `AxisShaper`
/// is configured `Passthrough`).
///
/// `e_halos` is the same `BatchPartition`-derived halo list `shape_batch`
/// uses to insert constant-position pieces over E-gap intervals during the
/// neighbour scan. Streaming callers supply an empty slice — no E gaps
/// exist in the look-ahead replan window.
///
/// # Errors
///
/// Forwards [`ShapeError::Algebra`] (convolution failure) and
/// [`ShapeError::FitFailure`] (post-shape refit failure). The `index` field
/// in those errors is the index into `planned`.
pub fn emit_shaped(
    planned: &[FittedSegment],
    meta: &[EmitSegmentMeta],
    kernels: &[Option<PiecewisePolynomialKernel<f64>>; 4],
    e_halos: &[EHalo],
    history: &PerAxisHistory<'_>,
    batch_t_start: f64,
    batch_t_end: f64,
) -> Result<Vec<ShapedSegment>, ShapeError> {
    debug_assert_eq!(
        planned.len(),
        meta.len(),
        "emit_shaped: planned and meta lengths must match",
    );

    let half_supports = [
        kernels[0].as_ref().map_or(0.0, kernel_half_support),
        kernels[1].as_ref().map_or(0.0, kernel_half_support),
        kernels[2].as_ref().map_or(0.0, kernel_half_support),
    ];

    let mut output: Vec<ShapedSegment> = Vec::with_capacity(planned.len());

    for (seg_idx, fitted) in planned.iter().enumerate() {
        let t_start = fitted.t_start;
        let t_end = fitted.t_end;

        let mut shaped_axes: [Option<ScalarNurbs<f64>>; 3] = [None, None, None];

        for axis in 0..3 {
            let cps = fitted.axes[axis].control_points();
            let axis_is_constant = if let Some(&first) = cps.first() {
                cps.iter().all(|c| (*c - first).abs() < 1e-12)
            } else {
                true
            };

            let mut axis_shaped = if axis_is_constant {
                fitted.axes[axis].clone()
            } else if let Some(kernel) = kernels[axis].as_ref() {
                let padded = pad_segment_axis_with_history(
                    seg_idx,
                    axis,
                    planned,
                    e_halos,
                    history.axes[axis],
                    half_supports[axis],
                    batch_t_start,
                    batch_t_end,
                );
                shape_axis(&padded, kernel, t_start, t_end).map_err(|detail| {
                    ShapeError::Algebra {
                        index: seg_idx,
                        detail,
                    }
                })?
            } else {
                fitted.axes[axis].clone()
            };

            if !axis_is_constant {
                axis_shaped =
                    refit_to_cubic(&axis_shaped, REFIT_TOLERANCE_MM).map_err(|detail| {
                        ShapeError::FitFailure {
                            index: seg_idx,
                            detail,
                        }
                    })?;
            }

            shaped_axes[axis] = Some(axis_shaped);
        }

        let m = meta[seg_idx];
        output.push(ShapedSegment {
            axes: [
                shaped_axes[0].take().unwrap(),
                shaped_axes[1].take().unwrap(),
                shaped_axes[2].take().unwrap(),
            ],
            e_mode: m.e_mode,
            extrusion_per_xy_mm: m.extrusion_per_xy_mm,
            // E-gap segments (which carry the independent-E NURBS) are
            // inserted by `shape_batch` outside this function.
            e_independent: None,
            t_start,
            t_end,
        });
    }

    Ok(output)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fit::FittedSegment;
    use crate::{
        plan_velocity, AxisShaper, ELimits, PlanInput, PlanSegment, PlanShaper, RequiredShaper,
        SafetyMode, ShapeBatchInput, ShapeSegmentInput, ShaperConfig,
    };
    use geometry::segment::EMode;
    use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces};
    use nurbs::VectorNurbs;

    fn straight_linear(start: [f64; 3], end: [f64; 3]) -> VectorNurbs<f64, 3> {
        VectorNurbs::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![start, end], None).unwrap()
    }

    fn default_limits() -> temporal::Limits {
        temporal::Limits::new(
            [500.0, 500.0, 500.0],
            [5_000.0, 5_000.0, 5_000.0],
            [100_000.0, 100_000.0, 100_000.0],
            2_500.0,
        )
    }

    fn default_e_limits() -> ELimits {
        ELimits {
            v_max: 100.0,
            a_max: 5_000.0,
        }
    }

    fn default_shaper_config() -> ShaperConfig {
        ShaperConfig {
            x: RequiredShaper::SmoothZv {
                frequency_hz: 180.0,
            },
            y: RequiredShaper::SmoothZv {
                frequency_hz: 120.0,
            },
            z: AxisShaper::Passthrough,
        }
    }

    fn default_kernels() -> [Option<PlanShaper>; 4] {
        [
            Some(PlanShaper::SmoothZv {
                frequency_hz: 180.0,
            }),
            Some(PlanShaper::SmoothZv {
                frequency_hz: 120.0,
            }),
            Some(PlanShaper::Passthrough),
            None,
        ]
    }

    fn assert_nurbs_near_equal(a: &ScalarNurbs<f64>, b: &ScalarNurbs<f64>, label: &str) {
        assert_eq!(a.degree(), b.degree(), "{label}: degree differs");
        assert_eq!(a.knots().len(), b.knots().len(), "{label}: knot count differs");
        let max_knot_diff = a.knots().iter().zip(b.knots().iter())
            .map(|(ka, kb)| (ka - kb).abs()).fold(0.0_f64, f64::max);
        assert!(max_knot_diff < 1e-12, "{label}: knots differ by {max_knot_diff:.2e}");
        assert_eq!(a.control_points().len(), b.control_points().len(),
            "{label}: control point count differs");
        let max_cp_diff = a.control_points().iter().zip(b.control_points().iter())
            .map(|(ca, cb)| (ca - cb).abs()).fold(0.0_f64, f64::max);
        assert!(max_cp_diff < 1e-12, "{label}: control points differ by {max_cp_diff:.2e} mm");
        assert_eq!(a.weights().is_some(), b.weights().is_some(),
            "{label}: weight presence differs");
        if let (Some(wa), Some(wb)) = (a.weights(), b.weights()) {
            let max_w_diff = wa.iter().zip(wb.iter())
                .map(|(wa, wb)| (wa - wb).abs()).fold(0.0_f64, f64::max);
            assert!(max_w_diff < 1e-12, "{label}: weights differ by {max_w_diff:.2e}");
        }
    }

    /// **Byte-identity contract.** With an empty [`PerAxisHistory`] and the
    /// non-streaming right-pad behaviour, `plan_velocity + emit_shaped`
    /// must match `shape_batch`'s output byte-for-byte on the same input.
    #[test]
    fn empty_history_matches_shape_batch_byte_identical() {
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let plan_segs = [PlanSegment {
            temporal: temporal::multi::SegmentInput {
                curve: &curve,
                limits: default_limits(),
                trailing_junction_chord_tolerance_mm: 0.05,
            },
            e_mode: EMode::CoupledToXy,
            extrusion_per_xy_mm: 0.04,
            e_independent: None,
            feedrate_mm_s: 100.0,
        }];

        let plan_input = PlanInput {
            segments: &plan_segs,
            grid_strategy: temporal::multi::GridStrategy::Fixed(10),
            worker_threads: 1,
            kernels: default_kernels(),
            fit_tolerance_mm: 0.5,
            beta_max_iters: 5,
            beta_convergence_ratio: 1.02,
            e_limits: default_e_limits(),
            initial_v: 0.0,
            terminal_v: 0.0,
            safety_mode: SafetyMode::TerminalKnown,
        };
        let planned = plan_velocity(&plan_input).expect("plan_velocity should succeed");
        assert_eq!(planned.len(), 1);

        let kernels: [Option<PiecewisePolynomialKernel<f64>>; 4] = [
            Some(RequiredShaper::SmoothZv { frequency_hz: 180.0 }.to_kernel()),
            Some(RequiredShaper::SmoothZv { frequency_hz: 120.0 }.to_kernel()),
            None,
            None,
        ];
        let meta = [EmitSegmentMeta {
            e_mode: EMode::CoupledToXy,
            extrusion_per_xy_mm: 0.04,
        }];

        let batch_t_start = 0.0;
        let batch_t_end = planned[0].t_end;

        let emitted = emit_shaped(
            &planned,
            &meta,
            &kernels,
            &[],
            &PerAxisHistory::empty(),
            batch_t_start,
            batch_t_end,
        )
        .expect("emit_shaped should succeed");

        // Reference: `shape_batch` on the same input.
        let segs = [ShapeSegmentInput {
            temporal: plan_segs[0].temporal,
            e_mode: plan_segs[0].e_mode,
            extrusion_per_xy_mm: plan_segs[0].extrusion_per_xy_mm,
            e_independent: plan_segs[0].e_independent,
            feedrate_mm_s: plan_segs[0].feedrate_mm_s,
        }];
        let shape_input = ShapeBatchInput {
            segments: &segs,
            grid_strategy: temporal::multi::GridStrategy::Fixed(10),
            worker_threads: 1,
            shaper: default_shaper_config(),
            fit_tolerance_mm: 0.5,
            beta_max_iters: 5,
            beta_convergence_ratio: 1.02,
            e_limits: default_e_limits(),
            initial_v: 0.0,
            terminal_v: 0.0,
        };
        let reference = crate::shape_batch(&shape_input).expect("shape_batch should succeed");

        assert_eq!(emitted.len(), reference.segments.len());
        for (i, (a, b)) in emitted.iter().zip(reference.segments.iter()).enumerate() {
            assert_nurbs_near_equal(&a.axes[0], &b.axes[0], &format!("seg{i} X"));
            assert_nurbs_near_equal(&a.axes[1], &b.axes[1], &format!("seg{i} Y"));
            assert_nurbs_near_equal(&a.axes[2], &b.axes[2], &format!("seg{i} Z"));
            assert_eq!(a.e_mode, b.e_mode, "seg{i}: e_mode differs");
            assert!(
                (a.extrusion_per_xy_mm - b.extrusion_per_xy_mm).abs() < 1e-15,
                "seg{i}: extrusion_per_xy_mm differs",
            );
            #[allow(clippy::float_cmp)]
            {
                assert_eq!(a.t_start, b.t_start, "seg{i}: t_start differs");
                assert_eq!(a.t_end, b.t_end, "seg{i}: t_end differs");
            }
        }
    }

    /// **History-aware left-pad.** With a single known history piece
    /// ending at `batch_t_start`, the padded curve must read its tail
    /// value at the seam (rather than the constant `start_val` fallback
    /// produced by the no-history path).
    #[test]
    fn pad_segment_axis_with_history_seam_reads_history_tail() {
        // Single fitted segment on `t ∈ [1.0, 2.0]`, X linear from 10 → 30.
        let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![10.0, 20.0],
        }]);
        let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![0.0],
        }]);
        let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: 1.0,
            u_end: 2.0,
            coeffs: vec![0.0],
        }]);
        let fitted = vec![FittedSegment {
            axes: [x_nurbs, y_nurbs, z_nurbs],
            t_start: 1.0,
            t_end: 2.0,
        }];

        // History piece on `t ∈ [0.0, 1.0]`, X linear from 0 → 10. At
        // `t = 1.0` it evaluates to 10.0 — matching the segment's
        // start. The padded value sampled inside the history domain
        // (e.g. at `t = 0.8`) should be 8.0, not the constant fallback
        // (which would also be 10.0 for this contrived case but would
        // mis-represent the slope).
        let history_x = vec![BezierPiece {
            u_start: 0.0,
            u_end: 1.0,
            coeffs: vec![0.0, 10.0],
        }];

        let t_sm_half = 0.3; // pad target: t = 0.7
        let padded = pad_segment_axis_with_history(
            0,
            0,
            &fitted,
            &[],
            &history_x,
            t_sm_half,
            1.0,
            2.0,
        );

        let pieces = extract_bezier_pieces(&padded);
        // Padded must extend back to at least t = 0.7.
        assert!(
            pieces[0].u_start <= 0.7 + 1e-12,
            "padded must cover at least back to 0.7, got {}",
            pieces[0].u_start,
        );

        // Find the piece containing t = 0.8 and evaluate.
        let val_08 = pieces
            .iter()
            .find(|p| 0.8 >= p.u_start - 1e-12 && 0.8 <= p.u_end + 1e-12)
            .expect("padded curve should cover t = 0.8")
            .evaluate(0.8);
        // History X at t = 0.8: 0 + 10·0.8 = 8.0.
        assert!(
            (val_08 - 8.0).abs() < 1e-9,
            "expected 8.0 from history at t=0.8, got {val_08}",
        );

        // At the seam (t = 1.0), the value should also be 10.0 (matches
        // both segment-start and history-tail).
        let val_10 = pieces
            .iter()
            .find(|p| 1.0 >= p.u_start - 1e-12 && 1.0 <= p.u_end + 1e-12)
            .expect("padded curve should cover t = 1.0")
            .evaluate(1.0);
        assert!(
            (val_10 - 10.0).abs() < 1e-9,
            "expected 10.0 at seam, got {val_10}",
        );

        // ---- Sanity: the no-history call uses constant-extension and
        // therefore reads start_val (= 10.0 here) at t = 0.8, not 8.0. ----
        let padded_no_history =
            crate::pad::pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 1.0, 2.0);
        let pieces_no_history = extract_bezier_pieces(&padded_no_history);
        let val_08_no_history = pieces_no_history
            .iter()
            .find(|p| 0.8 >= p.u_start - 1e-12 && 0.8 <= p.u_end + 1e-12)
            .expect("padded curve should cover t = 0.8")
            .evaluate(0.8);
        assert!(
            (val_08_no_history - 10.0).abs() < 1e-9,
            "no-history path should read constant start_val (10.0) at t=0.8, got {val_08_no_history}",
        );
        // The two paths must disagree — that's the whole point of the
        // streaming-shaper history hook.
        assert!(
            (val_08 - val_08_no_history).abs() > 1.0,
            "history vs no-history must disagree at t=0.8 (history 8.0 vs constant 10.0)",
        );
    }

    /// `PerAxisHistory::empty()` must produce byte-identical pad output to
    /// the legacy `pad_segment_axis` wrapper (which itself passes `&[]`).
    #[test]
    fn empty_history_pad_matches_legacy() {
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
        let fitted = vec![FittedSegment {
            axes: [x_nurbs, y_nurbs, z_nurbs],
            t_start: 0.0,
            t_end: 1.0,
        }];

        let t_sm_half = 0.1;
        for axis in 0..3 {
            let with_history = pad_segment_axis_with_history(
                0,
                axis,
                &fitted,
                &[],
                &[],
                t_sm_half,
                0.0,
                1.0,
            );
            let legacy = crate::pad::pad_segment_axis(0, axis, &fitted, &[], t_sm_half, 0.0, 1.0);
            assert_nurbs_near_equal(&with_history, &legacy, &format!("axis {axis}"));
        }
    }
}
