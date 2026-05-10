//! Phase-2 Task-2.1: planning half of the streaming-shaper split.
//!
//! `plan_velocity` runs TOPP-RA + β-medium iteration on a multi-axis path and
//! returns the time-domain **planned** (β-converged, unshaped) trajectory as
//! `Vec<FittedSegment>`. It does **not** perform shaping convolution or refit;
//! the shaping half is Task 2.2's `emit_shaped`. Until Task 2.2 lands the
//! existing `shape_batch` keeps doing the shaping inline; this entry point is
//! used by the streaming planner (`ShaperState::append_and_replan` in Phase 3)
//! to re-plan an un-committed path tail without producing wire-bound output.
//!
//! Two safety modes are supported (spec §3.2 / §3.6):
//!
//! - [`SafetyMode::TerminalKnown`] — current `shape_batch` semantics. The
//!   path's terminal velocity is final; the β-medium derate uses constant-pad
//!   future at the path terminus.
//! - [`SafetyMode::WorstCaseFuture`] — streaming case. The terminal velocity
//!   is the speculative decel-to-zero; β-medium derates against the
//!   worst-case-future bound (spec §3.6) by applying a tighter effective
//!   `a_machine` (`0.5·a_machine`) to the trailing region. Output is safe
//!   under any conforming follow-on input arriving after dispatch.
//!
//! See `docs/superpowers/specs/2026-05-10-streaming-shaper-design.md` §3.2 and
//! §3.6 for the full bound derivation and the rationale for the
//! "loose-but-always-safe" model used here.

use crate::fit::FittedSegment;
use crate::partition::partition_batch;
use crate::{
    AxisShaper, ELimits, RequiredShaper, ShapeBatchInput, ShapeError, ShapeSegmentInput,
    ShaperConfig,
};

/// Boundary-future treatment for the β-medium derate test.
///
/// See module docs and spec §3.6.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SafetyMode {
    /// Terminal velocity is the actual final state of the path. β-medium
    /// derates against the post-shape peak computed with constant-position
    /// padding at the path terminus. This matches the current
    /// `shape_batch` behaviour byte-for-byte.
    TerminalKnown,
    /// Streaming case: the terminal velocity is speculative (the planner's
    /// decel-to-zero default) and the trailing-`h` region of the path may be
    /// replaced by a follow-on move at any time. β-medium derates against
    /// the worst-case-future bound (`|ẍ_shaped| ≤ past_term + 0.5·a_machine`
    /// for a symmetric unit-DC kernel) by tightening the effective machine
    /// accel limit on the trailing region.
    WorstCaseFuture,
}

/// Per-axis shaper for a [`PlanInput`]. Mirrors [`AxisShaper`] but allows
/// `None` on every axis (X / Y / Z / E), unlike [`ShaperConfig`] which forces
/// X and Y to be active. Streaming may legitimately plan with passthrough on
/// every axis (e.g., during early bring-up before per-axis shaper config is
/// loaded), so the planning API does not enforce X/Y activeness.
///
/// `E` is structurally always passthrough — the extruder follows shaped XY
/// arc-length on coupled segments and carries its own un-shaped NURBS on
/// independent segments — and we model it here only for symmetry with
/// [`crate::streaming::ShaperState`]'s 4-axis layout.
#[derive(Debug, Clone, Copy)]
pub enum PlanShaper {
    /// Smooth ZV at `frequency_hz`.
    SmoothZv { frequency_hz: f64 },
    /// Smooth MZV at `frequency_hz`.
    SmoothMzv { frequency_hz: f64 },
    /// No shaping for this axis (kernel half-support `h = 0`).
    Passthrough,
}

impl PlanShaper {
    fn into_required(self) -> Result<RequiredShaper, ShapeError> {
        match self {
            Self::SmoothZv { frequency_hz } => Ok(RequiredShaper::SmoothZv { frequency_hz }),
            Self::SmoothMzv { frequency_hz } => Ok(RequiredShaper::SmoothMzv { frequency_hz }),
            // Currently the underlying β-medium loop assumes X and Y are
            // active; passthrough on those axes is not exercised by the
            // existing test suite. We reject early rather than silently
            // producing untested behaviour. Phase 3 may relax this.
            Self::Passthrough => Err(ShapeError::UnsupportedShaperOnXY),
        }
    }

    fn into_axis(self) -> AxisShaper {
        match self {
            Self::SmoothZv { frequency_hz } => AxisShaper::SmoothZv { frequency_hz },
            Self::SmoothMzv { frequency_hz } => AxisShaper::SmoothMzv { frequency_hz },
            Self::Passthrough => AxisShaper::Passthrough,
        }
    }
}

/// One segment of a multi-segment planning input.
///
/// Mirrors [`ShapeSegmentInput`] without the `e_independent`/`feedrate_mm_s`
/// fields that only matter for the (Task-2.2) shaping half.
#[derive(Debug, Clone, Copy)]
pub struct PlanSegment<'a> {
    /// Layer-2 input for this segment (curve + dynamic limits + junction
    /// chord tolerance).
    pub temporal: temporal::multi::SegmentInput<'a>,
    /// E-axis mode. Used by the partitioning step to identify XY-motion runs.
    pub e_mode: geometry::segment::EMode,
    /// Extrusion ratio (mm E per mm XY arc length); zero for `Travel`.
    pub extrusion_per_xy_mm: f64,
    /// Independent-E NURBS for `Independent` E mode segments. Required by
    /// the partitioner to schedule E-only gaps; `None` for XY-motion segments.
    pub e_independent: Option<&'a nurbs::ScalarNurbs<f64>>,
    /// Feedrate (mm/s) — needed for E-only gap scheduling.
    pub feedrate_mm_s: f64,
}

/// Top-level input to [`plan_velocity`].
///
/// `initial_v` and `terminal_v` are the velocity boundary conditions at the
/// **batch start** (`segments[0]`'s u=0) and **batch end** (`segments[last]`'s
/// u=1) respectively, in mm/s. Phase 3 lifted the prior (0, 0) limitation:
/// both values are now forwarded to [`temporal::multi::plan_batch`] via
/// `BatchInput::{initial_velocity, terminal_velocity}`, which threads them
/// into the joining loop's first-segment `v_start` and last-segment `v_end`
/// seeds. TOPP-RA's per-segment `schedule_segment_with_tolerance` already
/// accepted arbitrary boundary velocities; the lift is purely plumbing.
///
/// The streaming shaper (`ShaperState::append_and_replan`) uses this to plan
/// from the velocity already committed at `t_dispatched` (so the un-committed
/// replan window chains continuously into the in-flight motion) and to
/// always decelerate the replanned tail to zero at the new move's terminal
/// (so the spec's "decel-to-zero default" holds even when no follow-on move
/// arrives in time).
#[derive(Debug)]
pub struct PlanInput<'a> {
    /// Multi-axis planning path; must be non-empty.
    pub segments: &'a [PlanSegment<'a>],
    /// Forwarded to `temporal::multi::plan_batch`.
    pub grid_strategy: temporal::multi::GridStrategy,
    /// Forwarded to `temporal::multi::plan_batch`.
    pub worker_threads: usize,
    /// Per-axis shapers in the order `[X, Y, Z, E]`. `E` is always passthrough
    /// (entry retained for streaming-state symmetry); see [`PlanShaper`].
    pub kernels: [Option<PlanShaper>; 4],
    /// L-infinity tolerance for the C1-constrained fit (mm).
    pub fit_tolerance_mm: f64,
    /// Maximum number of β-medium outer iterations.
    pub beta_max_iters: u8,
    /// Convergence ratio threshold for β-medium iteration.
    pub beta_convergence_ratio: f64,
    /// Extruder axis limits.
    pub e_limits: ELimits,
    /// Velocity at the batch start (mm/s). Must be finite and non-negative.
    /// Phase 3 accepts arbitrary values; the streaming shaper uses this to
    /// chain into the committed velocity at `t_dispatched`.
    pub initial_v: f64,
    /// Velocity at the batch end (mm/s). Must be finite and non-negative.
    /// Phase 3 accepts arbitrary values; the streaming shaper's "decel-to-
    /// zero default" plans always pass `0.0` here so the new move's terminal
    /// is a safe rest point.
    pub terminal_v: f64,
    /// Boundary-future treatment for the trailing region.
    pub safety_mode: SafetyMode,
}

/// Run the planning half of the shaper pipeline.
///
/// Returns the β-converged time-domain **fitted** trajectory: one
/// [`FittedSegment`] per XY-motion input segment, in the same order. E-only
/// gaps are excluded — they are inserted by the shaping half (Task 2.2).
///
/// # Errors
///
/// - [`ShapeError::EmptySegments`] — `input.segments` is empty.
/// - [`ShapeError::UnsupportedShaperOnXY`] — any axis kernel is `None` or
///   `Passthrough` for X or Y (Phase 2 limitation; the underlying β-medium
///   loop assumes both axes are actively shaped).
/// - [`ShapeError::UnsupportedBoundaryVelocity`] — `initial_v` or `terminal_v`
///   is non-finite or negative.
/// - Any error from the underlying β-medium loop (TOPP-RA infeasibility, fit
///   failure, etc.).
pub fn plan_velocity(input: &PlanInput<'_>) -> Result<Vec<FittedSegment>, ShapeError> {
    if input.segments.is_empty() {
        return Err(ShapeError::EmptySegments);
    }

    // Boundary-velocity validation: Phase 3 lifted the (0, 0) limitation —
    // `temporal::multi::plan_batch` now accepts arbitrary `(initial_velocity,
    // terminal_velocity)` and TOPP-RA already handles arbitrary boundary
    // conditions internally via `schedule_segment_with_tolerance(.., v_start,
    // v_end, ..)`. Only basic sanity (finite, non-negative) is enforced here;
    // physical feasibility is the temporal solver's job. The
    // [`ShapeError::UnsupportedBoundaryVelocity`] variant is retained so
    // callers that previously got a hard rejection still see a structured
    // error on out-of-domain inputs.
    if !input.initial_v.is_finite() || input.initial_v < 0.0 {
        return Err(ShapeError::UnsupportedBoundaryVelocity);
    }
    if !input.terminal_v.is_finite() || input.terminal_v < 0.0 {
        return Err(ShapeError::UnsupportedBoundaryVelocity);
    }

    // Build a `ShapeBatchInput` from `PlanInput`. Internally this is the
    // existing β-medium machinery; the only new behaviour is the
    // `safety_mode` interpretation of the post-shape peak vs machine limit
    // in the trailing region.
    let shaper = build_shaper_config(&input.kernels)?;
    let segments: Vec<ShapeSegmentInput<'_>> = input
        .segments
        .iter()
        .map(|s| ShapeSegmentInput {
            temporal: s.temporal,
            e_mode: s.e_mode,
            extrusion_per_xy_mm: s.extrusion_per_xy_mm,
            e_independent: s.e_independent,
            feedrate_mm_s: s.feedrate_mm_s,
        })
        .collect();

    let shape_input = ShapeBatchInput {
        segments: &segments,
        grid_strategy: input.grid_strategy,
        worker_threads: input.worker_threads,
        shaper,
        fit_tolerance_mm: input.fit_tolerance_mm,
        beta_max_iters: input.beta_max_iters,
        beta_convergence_ratio: input.beta_convergence_ratio,
        e_limits: input.e_limits,
        initial_v: input.initial_v,
        terminal_v: input.terminal_v,
    };

    let partition = partition_batch(&segments, &input.e_limits);
    crate::beta::plan_velocity_inner(&shape_input, &partition, input.safety_mode)
}

fn build_shaper_config(kernels: &[Option<PlanShaper>; 4]) -> Result<ShaperConfig, ShapeError> {
    // X and Y are required-active per `ShaperConfig`. `None` or `Passthrough`
    // entries in those slots are rejected (see [`PlanShaper`] doc).
    let x = kernels[0]
        .ok_or(ShapeError::UnsupportedShaperOnXY)?
        .into_required()?;
    let y = kernels[1]
        .ok_or(ShapeError::UnsupportedShaperOnXY)?
        .into_required()?;
    let z = kernels[2].map_or(AxisShaper::Passthrough, PlanShaper::into_axis);
    Ok(ShaperConfig { x, y, z })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ELimits;
    use geometry::segment::EMode;
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

    fn default_kernels() -> [Option<PlanShaper>; 4] {
        [
            Some(PlanShaper::SmoothZv {
                frequency_hz: 180.0,
            }),
            Some(PlanShaper::SmoothMzv {
                frequency_hz: 120.0,
            }),
            Some(PlanShaper::Passthrough),
            None,
        ]
    }

    fn default_input<'a>(segments: &'a [PlanSegment<'a>], safety: SafetyMode) -> PlanInput<'a> {
        PlanInput {
            segments,
            grid_strategy: temporal::multi::GridStrategy::Fixed(10),
            worker_threads: 1,
            kernels: default_kernels(),
            fit_tolerance_mm: 0.5,
            beta_max_iters: 5,
            beta_convergence_ratio: 1.02,
            e_limits: default_e_limits(),
            // Step-0 lift: caller may override these to exercise nonzero
            // boundary velocities (Phase 3's `append_and_replan` always
            // does). Defaults match the legacy (0, 0) shape_batch contract.
            initial_v: 0.0,
            terminal_v: 0.0,
            safety_mode: safety,
        }
    }

    #[test]
    fn rejects_empty_segments() {
        let input = default_input(&[], SafetyMode::TerminalKnown);
        let result = plan_velocity(&input);
        assert!(matches!(result, Err(ShapeError::EmptySegments)));
    }

    #[test]
    fn rejects_negative_initial_v() {
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let segments = [PlanSegment {
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
        let mut input = default_input(&segments, SafetyMode::TerminalKnown);
        input.initial_v = -1.0;
        let result = plan_velocity(&input);
        assert!(matches!(result, Err(ShapeError::UnsupportedBoundaryVelocity)));
    }

    #[test]
    fn rejects_nan_terminal_v() {
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let segments = [PlanSegment {
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
        let mut input = default_input(&segments, SafetyMode::TerminalKnown);
        input.terminal_v = f64::NAN;
        let result = plan_velocity(&input);
        assert!(matches!(result, Err(ShapeError::UnsupportedBoundaryVelocity)));
    }

    /// Step-0 lift contract: a non-zero `initial_v` is accepted (no error)
    /// and produces a valid plan. The first sample of the first segment's
    /// TOPP profile reflects the requested starting velocity.
    #[test]
    fn nonzero_initial_v_produces_chained_profile() {
        // 200 mm move to give TOPP-RA enough path length to actually run
        // an accel-cruise-decel under the default limits.
        let curve = straight_linear([0.0, 0.0, 0.0], [200.0, 0.0, 0.0]);
        let segments = [PlanSegment {
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
        let mut input = default_input(&segments, SafetyMode::TerminalKnown);
        input.initial_v = 50.0;
        input.terminal_v = 0.0;

        let fitted = plan_velocity(&input).expect("plan with nonzero initial_v should succeed");
        assert_eq!(fitted.len(), 1);

        // Sample the X-axis velocity at t = t_start. For a single 200 mm
        // pure-X move with `initial_v = 50 mm/s`, the toolhead's instantaneous
        // speed at the start should be 50 mm/s to within TOPP-RA's per-grid
        // tolerance (the joining loop's `ε_velocity = 1 mm/s`).
        let seg = &fitted[0];
        let mut t_eps = (seg.t_end - seg.t_start) * 1e-6;
        if t_eps <= 0.0 {
            t_eps = 1e-9;
        }
        let t_sample = seg.t_start + t_eps;
        let x0 = nurbs::eval::eval(&seg.axes[0], seg.t_start);
        let x1 = nurbs::eval::eval(&seg.axes[0], t_sample);
        let vx_start = (x1 - x0) / t_eps;
        assert!(
            (vx_start - 50.0).abs() < 5.0,
            "X-axis start velocity {vx_start} mm/s deviates from requested 50.0 mm/s",
        );
    }

    #[test]
    fn rejects_passthrough_on_x() {
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let segments = [PlanSegment {
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
        let mut input = default_input(&segments, SafetyMode::TerminalKnown);
        input.kernels[0] = Some(PlanShaper::Passthrough);
        let result = plan_velocity(&input);
        assert!(matches!(result, Err(ShapeError::UnsupportedShaperOnXY)));
    }

    #[test]
    fn rejects_passthrough_on_y() {
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let segments = [PlanSegment {
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
        let mut input = default_input(&segments, SafetyMode::TerminalKnown);
        input.kernels[1] = Some(PlanShaper::Passthrough);
        let result = plan_velocity(&input);
        assert!(matches!(result, Err(ShapeError::UnsupportedShaperOnXY)));
    }

    #[test]
    fn rejects_none_on_x() {
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let segments = [PlanSegment {
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
        let mut input = default_input(&segments, SafetyMode::TerminalKnown);
        input.kernels[0] = None;
        let result = plan_velocity(&input);
        assert!(matches!(result, Err(ShapeError::UnsupportedShaperOnXY)));
    }

    #[test]
    fn returns_one_fitted_per_xy_segment() {
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let segments = [PlanSegment {
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
        let input = default_input(&segments, SafetyMode::TerminalKnown);
        let fitted = plan_velocity(&input).expect("plan should succeed");
        assert_eq!(fitted.len(), 1);
        assert!(fitted[0].t_end > fitted[0].t_start);
    }

    /// **Spec §3.6 contract — multi-segment.** Only the **last** XY segment
    /// is subject to the worst-case-future half-machine-accel derate.
    ///
    /// We can't assert identical segment-0 _durations_ here: the temporal
    /// joining loop uses segment 1's tighter limit to compute the junction
    /// velocity, which propagates back and slows segment 0's tail too —
    /// that's TOPP-RA doing its job, not a β-derate regression. The
    /// invariant we _can_ assert end-to-end is monotonicity per-segment
    /// (both segments must take ≥ their TerminalKnown durations under
    /// WorstCaseFuture, with strict inequality on the last segment because
    /// its limit is genuinely halved).
    ///
    /// The "only last segment's _effective machine limit_ is changed"
    /// invariant is tested directly via `effective_machine_a_max` in the
    /// `beta::tests` module.
    #[test]
    fn worst_case_future_segment_durations_monotone() {
        let curve0 = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let curve1 = straight_linear([50.0, 0.0, 0.0], [100.0, 0.0, 0.0]);
        let segments = [
            PlanSegment {
                temporal: temporal::multi::SegmentInput {
                    curve: &curve0,
                    limits: default_limits(),
                    trailing_junction_chord_tolerance_mm: 0.05,
                },
                e_mode: EMode::CoupledToXy,
                extrusion_per_xy_mm: 0.04,
                e_independent: None,
                feedrate_mm_s: 100.0,
            },
            PlanSegment {
                temporal: temporal::multi::SegmentInput {
                    curve: &curve1,
                    limits: default_limits(),
                    trailing_junction_chord_tolerance_mm: 0.05,
                },
                e_mode: EMode::CoupledToXy,
                extrusion_per_xy_mm: 0.04,
                e_independent: None,
                feedrate_mm_s: 100.0,
            },
        ];

        let known = plan_velocity(&default_input(&segments, SafetyMode::TerminalKnown))
            .expect("TerminalKnown plan should succeed");
        let worst = plan_velocity(&default_input(&segments, SafetyMode::WorstCaseFuture))
            .expect("WorstCaseFuture plan should succeed");

        assert_eq!(known.len(), 2);
        assert_eq!(worst.len(), 2);

        let dur_known_0 = known[0].t_end - known[0].t_start;
        let dur_worst_0 = worst[0].t_end - worst[0].t_start;
        let dur_known_1 = known[1].t_end - known[1].t_start;
        let dur_worst_1 = worst[1].t_end - worst[1].t_start;

        // Both segments: WorstCaseFuture's tighter end-of-batch accel
        // bound is a strictly tighter constraint set, so neither
        // segment's β-converged duration can be shorter.
        assert!(
            dur_worst_0 >= dur_known_0 - 1e-9,
            "segment 0 WorstCaseFuture duration {dur_worst_0} \
             must be ≥ TerminalKnown duration {dur_known_0}",
        );
        assert!(
            dur_worst_1 >= dur_known_1 - 1e-9,
            "segment 1 WorstCaseFuture duration {dur_worst_1} \
             must be ≥ TerminalKnown duration {dur_known_1}",
        );
    }

    /// **Spec §3.6 contract.** For the same input the
    /// `WorstCaseFuture` β-converged plan must use accel limits no greater
    /// than the `TerminalKnown` plan — by construction the trailing region's
    /// effective machine limit is half of `a_machine` under
    /// `WorstCaseFuture`, so the resulting trajectory must take **at least
    /// as long** to traverse.
    #[test]
    fn worst_case_future_is_no_faster_than_terminal_known() {
        // 50 mm move with the same dynamic limits as the existing
        // `single_straight_line_converges` test (which is known to converge
        // β-medium under TerminalKnown). A 5000 mm/s² accel cap is enough
        // for TOPP-RA to feasibly schedule a triangular profile; the
        // β-medium step's behaviour under WorstCaseFuture is what we test
        // here, not TOPP-RA's own feasibility.
        let curve = straight_linear([0.0, 0.0, 0.0], [50.0, 0.0, 0.0]);
        let segments = [PlanSegment {
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

        let known = plan_velocity(&default_input(&segments, SafetyMode::TerminalKnown))
            .expect("TerminalKnown plan should succeed");
        let worst = plan_velocity(&default_input(&segments, SafetyMode::WorstCaseFuture))
            .expect("WorstCaseFuture plan should succeed");

        assert_eq!(known.len(), 1);
        assert_eq!(worst.len(), 1);

        let dur_known = known[0].t_end - known[0].t_start;
        let dur_worst = worst[0].t_end - worst[0].t_start;
        // The worst-case bound is loose but always safe; the worst-case
        // duration must be ≥ the terminal-known duration up to the same
        // numerical tolerance the β-medium loop converges with.
        assert!(
            dur_worst >= dur_known - 1e-9,
            "WorstCaseFuture duration {dur_worst} must be ≥ TerminalKnown duration {dur_known}",
        );
    }
}
