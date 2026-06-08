use std::collections::VecDeque;

use geometry::segment::{split_cubic_bezier, CubicSegment};
use nurbs::bezier::{extract_bezier_pieces, BezierPiece};

use std::time::Instant;

use super::decel_finder::find_decel_start_time;
use super::{AxisShaperQueue, ReplanContext, ReplanReport, ShaperState, UncommittedMove};
use crate::emit_shaped::EmitSegmentMeta;
use crate::fit::FittedSegment;
use crate::plan_velocity::{plan_velocity, PlanInput, PlanOutput, PlanSegment, PlanStats};
use crate::AxisShaper;
use crate::ShapeError;
use crate::ShapedSegment;

/// Sub-picosecond slack for absolute-time membership checks.
const TIME_LOOKUP_TOLERANCE: f64 = 1e-12;

/// Boundary slack for the `s_dispatched ∈ (0, 1)` interior check on the cubic
/// Bézier inverter result. Wider than [`TIME_LOOKUP_TOLERANCE`] because the
/// Newton iterate has more numerical wander than absolute-time arithmetic.
const SPLIT_BOUNDARY_TOLERANCE: f64 = 1e-9;

/// Y and Z control points must vanish within this tolerance to classify
/// a curve as pure-X for the closed-form `s` solve. One picometer is
/// well inside the planner's position resolution.
const PURE_AXIS_TOLERANCE: f64 = 1e-12;

/// Middle control points must sit within this distance of the (1/3, 2/3)
/// lerp of the endpoints to enable the analytic `s = (p − x0) / (x3 − x0)`
/// shortcut. One nanometer admits every collinear-cubic the planner emits.
const COLLINEAR_TOLERANCE: f64 = 1e-9;

const NEWTON_DENOMINATOR_FLOOR: f64 = 1e-18;

const NEWTON_PARAM_TOLERANCE: f64 = 1e-12;

const NEWTON_MAX_ITERS: usize = 12;

/// Residual budget for the Newton inverter's post-convergence position check.
/// 10× the C¹ refit's L∞ tolerance (5 µm) so a successful Newton converge on a
/// genuinely-on-curve target lands well inside the budget.
const NEWTON_RESIDUAL_MM: f64 = 0.05;

const NEWTON_SEED_CLAMP: f64 = 1e-6;

impl ShaperState {
    #[must_use]
    pub fn new(home_pos: [f64; 4], shapers: &[Option<AxisShaper>; 4]) -> Self {
        let axes: [AxisShaperQueue; 4] =
            std::array::from_fn(|i| build_axis_queue(home_pos[i], shapers[i]));

        Self {
            axes,
            uncommitted_moves: VecDeque::new(),
            t_appended: 0.0,
            t_decel_start: 0.0,
            t_shaped: 0.0,
            t_dispatched: 0.0,
            pending_dispatch: Vec::new(),
            planned_fitted: Vec::new(),
            planned_meta: Vec::new(),
        }
    }

    pub fn reset(&mut self, home_pos: [f64; 4]) {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            reseed_axis_queue(axis, home_pos[i]);
        }
        self.uncommitted_moves.clear();
        self.pending_dispatch.clear();
        self.planned_fitted.clear();
        self.planned_meta.clear();
        self.t_appended = 0.0;
        self.t_decel_start = 0.0;
        self.t_shaped = 0.0;
        self.t_dispatched = 0.0;
    }

    pub fn append_batch(&mut self, fitted: &FittedSegment) -> Result<(), nurbs::AlgebraError> {
        let shaped = shape_single_segment(fitted, &self.axes)?;
        self.pending_dispatch.push(shaped);
        Ok(())
    }

    pub fn drain_committed(&mut self) -> Vec<ShapedSegment> {
        std::mem::take(&mut self.pending_dispatch)
    }

    #[allow(clippy::too_many_lines)]
    pub fn append_and_replan(
        &mut self,
        new_segment: CubicSegment,
        ctx: &ReplanContext,
    ) -> Result<ReplanReport, ShapeError> {
        let initial_v = self.read_path_speed_at(self.t_dispatched, ctx.fallback_initial_v);
        let a_max_path = ctx.limits.a_max.iter().copied().fold(f64::MAX, f64::min);
        let (initial_a, start_d2_override) = if initial_v > 0.0 {
            let axis_accels = self.read_axis_accels_at(self.t_dispatched);
            let path_a = self
                .read_path_accel_at(self.t_dispatched, 0.0)
                .clamp(-a_max_path, a_max_path);
            (path_a, axis_accels)
        } else {
            (0.0, None)
        };

        let prior_uncommitted = self.uncommitted_moves.clone();
        let prior_t_appended = self.t_appended;
        let prior_t_decel_start = self.t_decel_start;
        let prior_planned_fitted = self.planned_fitted.clone();
        let prior_planned_meta = self.planned_meta.clone();

        let split_start = Instant::now();
        let partial_split = self.split_partially_committed_at_t_dispatched();

        self.uncommitted_moves
            .retain(|m| m.t_end > self.t_dispatched);

        match partial_split {
            Some(PartialCommitSplit::Replace { new_segment }) => {
                if let Some(front) = self.uncommitted_moves.front_mut() {
                    front.segment = *new_segment;
                    front.t_start = self.t_dispatched;
                }
            }
            Some(PartialCommitSplit::DropFromQueue) => {
                self.uncommitted_moves.pop_front();
            }
            None => {}
        }

        let pre_plan_t_start = self
            .uncommitted_moves
            .back()
            .map_or(self.t_dispatched, |m| m.t_end);
        self.uncommitted_moves.push_back(UncommittedMove {
            segment: new_segment,
            t_start: pre_plan_t_start,
            t_end: pre_plan_t_start,
        });
        let split_us = split_start.elapsed().as_micros() as u64;

        let window_segments = self.uncommitted_moves.len();

        let plan_segments: Vec<PlanSegment<'_>> = self
            .uncommitted_moves
            .iter()
            .map(|m| PlanSegment {
                temporal: temporal::multi::SegmentInput {
                    curve: &m.segment.xyz,
                    limits: per_segment_limits(&m.segment.xyz, ctx.limits, m.segment.feedrate_mm_s),
                    trailing_junction_chord_tolerance_mm: ctx.junction_chord_tolerance_mm,
                },
                e_mode: m.segment.e_mode,
                extrusion_per_xy_mm: m.segment.extrusion_per_xy_mm,
                e_independent: m.segment.e_independent.as_ref(),
                feedrate_mm_s: m.segment.feedrate_mm_s,
            })
            .collect();

        let plan_input = PlanInput {
            segments: &plan_segments,
            grid_strategy: ctx.grid_strategy,
            worker_threads: ctx.worker_threads,
            kernels: ctx.kernels,
            fit_tolerance_mm: ctx.fit_tolerance_mm,
            beta_max_iters: ctx.beta_max_iters,
            beta_convergence_ratio: ctx.beta_convergence_ratio,
            e_limits: ctx.e_limits,
            initial_v,
            initial_a,
            terminal_v: 0.0,
            safety_mode: ctx.safety_mode,
            start_d2_override,
        };

        let solve_start = Instant::now();
        let PlanOutput { fitted, stats } = match plan_velocity(&plan_input) {
            Ok(out) => out,
            Err(e) => {
                self.uncommitted_moves = prior_uncommitted;
                self.t_appended = prior_t_appended;
                self.t_decel_start = prior_t_decel_start;
                self.planned_fitted = prior_planned_fitted;
                self.planned_meta = prior_planned_meta;
                return Err(e);
            }
        };
        let solve_us = solve_start.elapsed().as_micros() as u64;

        let rebuild_start = Instant::now();
        let time_offset = self.t_dispatched;

        for axis_idx in 0..3 {
            self.replace_uncommitted_axis_pieces(axis_idx, time_offset, &fitted);
        }

        debug_assert_eq!(fitted.len(), self.uncommitted_moves.len());
        for (m, f) in self.uncommitted_moves.iter_mut().zip(fitted.iter()) {
            m.t_start = f.t_start + time_offset;
            m.t_end = f.t_end + time_offset;
        }

        let last = fitted
            .last()
            .expect("fitted non-empty by plan_velocity contract");
        self.t_appended = last.t_end + time_offset;

        self.t_decel_start = find_decel_start_time(&fitted) + time_offset;

        self.planned_fitted = fitted
            .into_iter()
            .map(|f| FittedSegment {
                axes: [
                    shift_nurbs_in_time(&f.axes[0], time_offset),
                    shift_nurbs_in_time(&f.axes[1], time_offset),
                    shift_nurbs_in_time(&f.axes[2], time_offset),
                ],
                t_start: f.t_start + time_offset,
                t_end: f.t_end + time_offset,
            })
            .collect();
        self.planned_meta = self
            .uncommitted_moves
            .iter()
            .map(|m| EmitSegmentMeta {
                e_mode: m.segment.e_mode,
                extrusion_per_xy_mm: m.segment.extrusion_per_xy_mm,
            })
            .collect();
        let rebuild_us = rebuild_start.elapsed().as_micros() as u64;

        Ok(ReplanReport {
            split_us,
            solve_us,
            rebuild_us,
            window_segments,
            plan: stats,
        })
    }
}

impl ShaperState {
    /// Tangential (path) acceleration of the PRE-SHAPE planned profile at time
    /// `t`: a_path = (v⃗·a⃗)/|v⃗| over the planned_fitted XY axes. This is the
    /// quantity the temporal SOCP's a_0 pin governs — the shaped axes include
    /// shaper transients and would pin the wrong layer. Below SPEED_FLOOR the
    /// tangential direction is undefined; standstill implies a_path = 0, so the
    /// fallback is returned.
    pub(crate) fn read_path_accel_at(&self, t: f64, fallback: f64) -> f64 {
        const SPEED_FLOOR: f64 = 1e-9;
        let Some(seg) = self
            .planned_fitted
            .iter()
            .find(|s| s.t_start <= t && t < s.t_end)
            .or_else(|| {
                self.planned_fitted
                    .last()
                    .filter(|s| (t - s.t_end).abs() <= TIME_LOOKUP_TOLERANCE)
            })
        else {
            return fallback;
        };
        let d = |axis: usize| -> (f64, f64) {
            if seg.axes[axis].degree() < 1 {
                return (0.0, 0.0);
            }
            let d1 = nurbs::eval::derivative(&seg.axes[axis]);
            if d1.degree() < 1 {
                return (nurbs::eval::eval(&d1, t), 0.0);
            }
            let d2 = nurbs::eval::derivative(&d1);
            (nurbs::eval::eval(&d1, t), nurbs::eval::eval(&d2, t))
        };
        let (vx, ax) = d(0);
        let (vy, ay) = d(1);
        let speed = (vx * vx + vy * vy).sqrt();
        if speed < SPEED_FLOOR {
            fallback
        } else {
            (vx * ax + vy * ay) / speed
        }
    }

    /// Axis-wise second derivatives `[d²x/dt², d²y/dt², d²z/dt²]` of the
    /// PRE-SHAPE planned_fitted polynomial at time `t`. Returns `None` when `t`
    /// is not covered by any fitted segment or when any axis polynomial has
    /// degree < 2.
    pub(crate) fn read_axis_accels_at(&self, t: f64) -> Option<[f64; 3]> {
        let seg = self
            .planned_fitted
            .iter()
            .find(|s| s.t_start <= t && t < s.t_end)
            .or_else(|| {
                self.planned_fitted
                    .last()
                    .filter(|s| (t - s.t_end).abs() <= TIME_LOOKUP_TOLERANCE)
            })?;
        let accel = std::array::from_fn(|axis| {
            if seg.axes[axis].degree() < 2 {
                return 0.0;
            }
            let d1 = nurbs::eval::derivative(&seg.axes[axis]);
            if d1.degree() < 1 {
                return 0.0;
            }
            let d2 = nurbs::eval::derivative(&d1);
            nurbs::eval::eval(&d2, t)
        });
        Some(accel)
    }

    pub(crate) fn read_path_speed_at(&self, t: f64, fallback: f64) -> f64 {
        let vx = self.axis_velocity_at(0, t);
        let vy = self.axis_velocity_at(1, t);
        match (vx, vy) {
            (Some(x), Some(y)) => (x * x + y * y).sqrt(),
            (Some(x), None) => x.abs(),
            (None, Some(y)) => y.abs(),
            (None, None) => fallback,
        }
    }

    fn axis_velocity_at(&self, axis_idx: usize, t: f64) -> Option<f64> {
        let pieces = &self.axes[axis_idx].pieces;
        if pieces.is_empty() {
            return None;
        }

        let last = pieces.back().unwrap();
        if t >= last.u_end && t <= last.u_end + TIME_LOOKUP_TOLERANCE {
            return Some(last.differentiate().evaluate(last.u_end));
        }

        for p in pieces {
            if p.u_start - TIME_LOOKUP_TOLERANCE <= t && t < p.u_end {
                return Some(p.differentiate().evaluate(t));
            }
        }
        None
    }

    #[must_use]
    pub fn current_position(&self) -> [f64; 4] {
        std::array::from_fn(|i| self.axis_position_at(i, self.t_appended).unwrap_or(0.0))
    }

    /// `t_dispatched` MUST advance with `t_appended`: leaving it behind causes -308 PieceStartInPast.
    pub fn advance_idle(&mut self, target_t: f64) {
        if target_t <= self.t_appended + 1e-12 {
            return;
        }
        debug_assert!(
            (self.t_dispatched - self.t_appended).abs() < 1e-9,
            "advance_idle requires fully-committed state: t_dispatched {} != t_appended {}",
            self.t_dispatched,
            self.t_appended,
        );
        debug_assert!(
            self.pending_dispatch.is_empty(),
            "advance_idle requires pending_dispatch drained before advancing",
        );
        let hold_start = self.t_appended;
        let hold_end = target_t;
        let end_pos: [f64; 4] =
            std::array::from_fn(|i| self.axis_position_at(i, hold_start).unwrap_or(0.0));

        for (i, axis) in self.axes.iter_mut().enumerate() {
            if axis.h > 0.0 {
                axis.pieces.push_back(BezierPiece {
                    u_start: hold_start,
                    u_end: hold_end,
                    coeffs: vec![end_pos[i]],
                });
            }
        }

        self.uncommitted_moves.clear();
        self.planned_fitted.clear();
        self.planned_meta.clear();
        self.t_appended = hold_end;
        self.t_decel_start = hold_end;
        self.t_dispatched = hold_end;
        self.t_shaped = hold_end;
    }

    fn axis_position_at(&self, axis_idx: usize, t: f64) -> Option<f64> {
        let pieces = &self.axes[axis_idx].pieces;
        if pieces.is_empty() {
            return None;
        }

        let last = pieces.back().unwrap();
        if t >= last.u_end && t <= last.u_end + TIME_LOOKUP_TOLERANCE {
            return Some(last.evaluate(last.u_end));
        }

        for p in pieces {
            if p.u_start - TIME_LOOKUP_TOLERANCE <= t && t < p.u_end {
                return Some(p.evaluate(t));
            }
        }
        None
    }

    fn split_partially_committed_at_t_dispatched(&self) -> Option<PartialCommitSplit> {
        let t_d = self.t_dispatched;
        let (idx, planned) = self
            .planned_fitted
            .iter()
            .enumerate()
            .find(|(_, f)| f.t_start - TIME_LOOKUP_TOLERANCE <= t_d && t_d < f.t_end)?;

        let move_ref = self.uncommitted_moves.get(idx)?;

        if matches!(
            move_ref.segment.e_mode,
            geometry::segment::EMode::Independent
        ) {
            return None;
        }

        let p_target = [
            nurbs::eval::eval(&planned.axes[0], t_d),
            nurbs::eval::eval(&planned.axes[1], t_d),
            nurbs::eval::eval(&planned.axes[2], t_d),
        ];

        let move_span_t = planned.t_end - planned.t_start;
        let s_seed = if move_span_t > TIME_LOOKUP_TOLERANCE {
            ((t_d - planned.t_start) / move_span_t)
                .clamp(NEWTON_SEED_CLAMP, 1.0 - NEWTON_SEED_CLAMP)
        } else {
            0.5
        };
        let s_dispatched =
            invert_cubic_bezier_xyz_to_param(&move_ref.segment.xyz, p_target, s_seed)?;

        if s_dispatched <= SPLIT_BOUNDARY_TOLERANCE {
            return None;
        }
        if s_dispatched >= 1.0 - SPLIT_BOUNDARY_TOLERANCE {
            return Some(PartialCommitSplit::DropFromQueue);
        }

        let (_left, right) = split_cubic_bezier(&move_ref.segment.xyz, s_dispatched);

        // `extrusion_per_xy_mm` and `feedrate_mm_s` are rates — unchanged on the shorter tail.
        let new_segment = CubicSegment::try_new(
            right,
            move_ref.segment.e_mode,
            move_ref.segment.extrusion_per_xy_mm,
            move_ref.segment.e_independent.clone(),
            move_ref.segment.feedrate_mm_s,
            move_ref.segment.source,
            move_ref.segment.split_info,
        )
        .expect("split_cubic_bezier output is a valid single-piece cubic Bézier");

        Some(PartialCommitSplit::Replace {
            new_segment: Box::new(new_segment),
        })
    }

    fn replace_uncommitted_axis_pieces(
        &mut self,
        axis_idx: usize,
        time_offset: f64,
        fitted: &[FittedSegment],
    ) {
        let t_keep_cutoff = self.t_dispatched;

        let pieces = &mut self.axes[axis_idx].pieces;
        while let Some(back) = pieces.back() {
            if back.u_start >= t_keep_cutoff - TIME_LOOKUP_TOLERANCE {
                pieces.pop_back();
            } else {
                break;
            }
        }

        for f in fitted {
            let axis_nurbs = &f.axes[axis_idx];
            let shifted = extract_bezier_pieces(axis_nurbs).into_iter().map(|mut p| {
                p.u_start += time_offset;
                p.u_end += time_offset;
                p
            });
            pieces.extend(shifted);
        }
    }
}

enum PartialCommitSplit {
    Replace { new_segment: Box<CubicSegment> },
    DropFromQueue,
}

#[allow(clippy::too_many_lines)]
fn invert_cubic_bezier_xyz_to_param(
    curve: &nurbs::VectorNurbs<f64, 3>,
    p_target: [f64; 3],
    s_seed: f64,
) -> Option<f64> {
    use nurbs::eval::vector_eval;

    let cps = curve.control_points();
    debug_assert_eq!(curve.degree(), 3);
    debug_assert_eq!(cps.len(), 4);

    let d1_cps: [[f64; 3]; 3] = [
        [
            3.0 * (cps[1][0] - cps[0][0]),
            3.0 * (cps[1][1] - cps[0][1]),
            3.0 * (cps[1][2] - cps[0][2]),
        ],
        [
            3.0 * (cps[2][0] - cps[1][0]),
            3.0 * (cps[2][1] - cps[1][1]),
            3.0 * (cps[2][2] - cps[1][2]),
        ],
        [
            3.0 * (cps[3][0] - cps[2][0]),
            3.0 * (cps[3][1] - cps[2][1]),
            3.0 * (cps[3][2] - cps[2][2]),
        ],
    ];
    let d2_cps: [[f64; 3]; 2] = [
        [
            2.0 * (d1_cps[1][0] - d1_cps[0][0]),
            2.0 * (d1_cps[1][1] - d1_cps[0][1]),
            2.0 * (d1_cps[1][2] - d1_cps[0][2]),
        ],
        [
            2.0 * (d1_cps[2][0] - d1_cps[1][0]),
            2.0 * (d1_cps[2][1] - d1_cps[1][1]),
            2.0 * (d1_cps[2][2] - d1_cps[1][2]),
        ],
    ];

    let eval_d1 = |s: f64| -> [f64; 3] {
        let one_minus = 1.0 - s;
        let b0 = one_minus * one_minus;
        let b1 = 2.0 * one_minus * s;
        let b2 = s * s;
        [
            b0 * d1_cps[0][0] + b1 * d1_cps[1][0] + b2 * d1_cps[2][0],
            b0 * d1_cps[0][1] + b1 * d1_cps[1][1] + b2 * d1_cps[2][1],
            b0 * d1_cps[0][2] + b1 * d1_cps[1][2] + b2 * d1_cps[2][2],
        ]
    };
    let eval_d2 = |s: f64| -> [f64; 3] {
        let one_minus = 1.0 - s;
        [
            one_minus * d2_cps[0][0] + s * d2_cps[1][0],
            one_minus * d2_cps[0][1] + s * d2_cps[1][1],
            one_minus * d2_cps[0][2] + s * d2_cps[1][2],
        ]
    };

    let mut s = s_seed.clamp(0.0, 1.0);

    let pure_x = cps
        .iter()
        .all(|p| p[1].abs() < PURE_AXIS_TOLERANCE && p[2].abs() < PURE_AXIS_TOLERANCE);
    if pure_x {
        let x0 = cps[0][0];
        let x3 = cps[3][0];
        let span = x3 - x0;
        let collinear_third = (cps[1][0] - (x0 + span / 3.0)).abs() < COLLINEAR_TOLERANCE
            && (cps[2][0] - (x0 + 2.0 * span / 3.0)).abs() < COLLINEAR_TOLERANCE;
        if collinear_third && span.abs() > PURE_AXIS_TOLERANCE {
            let s_closed = ((p_target[0] - x0) / span).clamp(0.0, 1.0);
            let xyz_s = vector_eval(curve, s_closed);
            let residual = ((xyz_s[0] - p_target[0]).powi(2)
                + (xyz_s[1] - p_target[1]).powi(2)
                + (xyz_s[2] - p_target[2]).powi(2))
            .sqrt();
            debug_assert!(
                residual < NEWTON_RESIDUAL_MM,
                "invert_cubic_bezier_xyz_to_param: pure-X collinear short-circuit \
                 produced residual {residual} mm > budget {NEWTON_RESIDUAL_MM} mm — \
                 the target point is not on the curve to within the planner's \
                 refit budget. Skipping split.",
            );
            if residual >= NEWTON_RESIDUAL_MM {
                return None;
            }
            return Some(s_closed);
        }
    }

    for _ in 0..NEWTON_MAX_ITERS {
        let xyz_s = vector_eval(curve, s);
        let d1 = eval_d1(s);
        let d2 = eval_d2(s);

        let dx = [
            xyz_s[0] - p_target[0],
            xyz_s[1] - p_target[1],
            xyz_s[2] - p_target[2],
        ];
        // f(s)   = (xyz − p_target) · xyz'
        // f'(s)  = xyz' · xyz' + (xyz − p_target) · xyz''
        let f = dx[0] * d1[0] + dx[1] * d1[1] + dx[2] * d1[2];
        let f_prime = d1[0] * d1[0]
            + d1[1] * d1[1]
            + d1[2] * d1[2]
            + dx[0] * d2[0]
            + dx[1] * d2[1]
            + dx[2] * d2[2];
        if !f.is_finite() || !f_prime.is_finite() || f_prime.abs() < NEWTON_DENOMINATOR_FLOOR {
            break;
        }
        let s_next = (s - f / f_prime).clamp(0.0, 1.0);
        if (s_next - s).abs() < NEWTON_PARAM_TOLERANCE {
            s = s_next;
            break;
        }
        s = s_next;
    }
    let s = s.clamp(0.0, 1.0);

    let xyz_final = vector_eval(curve, s);
    let residual = ((xyz_final[0] - p_target[0]).powi(2)
        + (xyz_final[1] - p_target[1]).powi(2)
        + (xyz_final[2] - p_target[2]).powi(2))
    .sqrt();
    debug_assert!(
        residual < NEWTON_RESIDUAL_MM,
        "invert_cubic_bezier_xyz_to_param: Newton converged to s = {s} but the \
         residual ||xyz(s) − p_target|| = {residual} mm exceeds the wrong-root \
         budget {NEWTON_RESIDUAL_MM} mm. This indicates a wrong-root convergence \
         (self-intersecting / highly-curved cubic, or stationary-point trap). \
         Falling back to no-split; the caller will skip the rewrite.",
    );
    if residual >= NEWTON_RESIDUAL_MM {
        return None;
    }
    Some(s)
}

fn shift_nurbs_in_time(curve: &nurbs::ScalarNurbs<f64>, dt: f64) -> nurbs::ScalarNurbs<f64> {
    use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces};
    let pieces: Vec<BezierPiece<f64>> = extract_bezier_pieces(curve)
        .into_iter()
        .map(|mut p| {
            p.u_start += dt;
            p.u_end += dt;
            p
        })
        .collect();
    bezier_pieces_to_nurbs(&pieces)
}

fn per_segment_limits(
    curve: &nurbs::VectorNurbs<f64, 3>,
    base: temporal::Limits,
    feedrate_mm_s: f64,
) -> temporal::Limits {
    const AXIS_INACTIVE_SPAN_EPS_MM: f64 = 1e-6;

    let cps = curve.control_points();

    let mut span = [0.0_f64; 3];
    for ax in 0..3 {
        let mut lo = f64::INFINITY;
        let mut hi = f64::NEG_INFINITY;
        for cp in cps {
            let v = cp[ax];
            if v < lo {
                lo = v;
            }
            if v > hi {
                hi = v;
            }
        }
        span[ax] = (hi - lo).max(0.0);
    }
    let chord_len = (span[0] * span[0] + span[1] * span[1] + span[2] * span[2]).sqrt();

    let max_active_j = (0..3)
        .filter_map(|ax| {
            if span[ax] > AXIS_INACTIVE_SPAN_EPS_MM {
                Some(base.j_max[ax])
            } else {
                None
            }
        })
        .fold(0.0_f64, f64::max);
    let mut j_max = base.j_max;
    if max_active_j > 0.0 {
        for ax in 0..3 {
            if span[ax] <= AXIS_INACTIVE_SPAN_EPS_MM {
                j_max[ax] = max_active_j;
            }
        }
    }

    let mut v_max = base.v_max;
    if feedrate_mm_s > 0.0 && chord_len > AXIS_INACTIVE_SPAN_EPS_MM {
        for ax in 0..3 {
            if span[ax] > AXIS_INACTIVE_SPAN_EPS_MM {
                let direction_fraction = span[ax] / chord_len;
                let feed_cap = feedrate_mm_s * direction_fraction;
                v_max[ax] = v_max[ax].min(feed_cap);
            }
        }
    }

    temporal::Limits::new(v_max, base.a_max, j_max, base.a_centripetal_max)
}

fn build_axis_queue(home_pos: f64, shaper: Option<AxisShaper>) -> AxisShaperQueue {
    let kernel = shaper.and_then(|s| s.to_kernel());
    let h = match shaper {
        Some(AxisShaper::SmoothZv { frequency_hz }) => 0.8025 / frequency_hz / 2.0,
        Some(AxisShaper::SmoothMzv { frequency_hz }) => 0.95625 / frequency_hz / 2.0,
        Some(AxisShaper::Passthrough) | None => 0.0,
    };

    let mut pieces = VecDeque::new();

    if h > 0.0 {
        let delta_safety = h;
        let total = h + delta_safety;
        pieces.push_back(BezierPiece {
            u_start: -total,
            u_end: 0.0,
            coeffs: vec![home_pos],
        });
    }

    AxisShaperQueue { pieces, kernel, h }
}

fn reseed_axis_queue(axis: &mut AxisShaperQueue, home_pos: f64) {
    axis.pieces.clear();
    if axis.h > 0.0 {
        let delta_safety = axis.h;
        let total = axis.h + delta_safety;
        axis.pieces.push_back(BezierPiece {
            u_start: -total,
            u_end: 0.0,
            coeffs: vec![home_pos],
        });
    }
}

fn shape_single_segment(
    fitted: &FittedSegment,
    axes: &[AxisShaperQueue; 4],
) -> Result<ShapedSegment, nurbs::AlgebraError> {
    use crate::pad::pad_segment_axis;
    use crate::refit::{refit_to_cubic, REFIT_TOLERANCE_MM};
    use crate::shaper::shape_axis;

    let t_start = fitted.t_start;
    let t_end = fitted.t_end;

    let fitted_slice = std::slice::from_ref(fitted);

    let mut shaped_axes: [Option<nurbs::ScalarNurbs<f64>>; 3] = [None, None, None];

    for axis in 0..3 {
        let q = &axes[axis];
        let axis_shaped = if let Some(kernel) = q.kernel.as_ref() {
            let padded = pad_segment_axis(0, axis, fitted_slice, &[], q.h, t_start, t_end);
            shape_axis(&padded, kernel, t_start, t_end)
        } else {
            fitted.axes[axis].clone()
        };

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
        e_mode: geometry::segment::EMode::CoupledToXy,
        extrusion_per_xy_mm: 0.0,
        e_independent: None,
        t_start,
        t_end,
    })
}
