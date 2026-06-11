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

const TIME_LOOKUP_TOLERANCE: f64 = 1e-12;

const SPLIT_BOUNDARY_TOLERANCE: f64 = 1e-9;

const PURE_AXIS_TOLERANCE: f64 = 1e-12;

const COLLINEAR_TOLERANCE: f64 = 1e-9;

const NEWTON_DENOMINATOR_FLOOR: f64 = 1e-18;

const NEWTON_PARAM_TOLERANCE: f64 = 1e-12;

const NEWTON_MAX_ITERS: usize = 12;

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
            planned_fitted: Vec::new(),
            planned_meta: Vec::new(),
            pending_freeze: Vec::new(),
        }
    }

    pub fn reset(&mut self, home_pos: [f64; 4]) {
        for (i, axis) in self.axes.iter_mut().enumerate() {
            reseed_axis_queue(axis, home_pos[i]);
        }
        self.uncommitted_moves.clear();
        self.planned_fitted.clear();
        self.planned_meta.clear();
        self.pending_freeze.clear();
        self.t_appended = 0.0;
        self.t_decel_start = 0.0;
        self.t_shaped = 0.0;
        self.t_dispatched = 0.0;
    }

    #[allow(clippy::too_many_lines)]
    pub fn append_and_replan(
        &mut self,
        new_segment: CubicSegment,
        ctx: &ReplanContext,
    ) -> Result<ReplanReport, ShapeError> {
        let max_h = self.axes.iter().map(|a| a.h).fold(0.0_f64, f64::max);

        let t_freeze =
            if self.t_dispatched > 0.0 && max_h > 0.0 && self.t_dispatched < self.t_appended {
                (self.t_dispatched + max_h).min(self.t_appended)
            } else {
                self.t_dispatched
            };

        let initial_v_raw = self.read_path_speed_at(t_freeze, ctx.fallback_initial_v);
        let a_max_path = ctx.limits.a_max.iter().copied().fold(f64::MAX, f64::min);
        let (initial_a, start_d2_override) = if initial_v_raw > 0.0 {
            let axis_accels = self.read_axis_accels_at(t_freeze);
            let path_a = self
                .read_path_accel_at(t_freeze, 0.0)
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

        let freeze_zone_active = t_freeze > self.t_dispatched + TIME_LOOKUP_TOLERANCE;

        let split_start = Instant::now();
        let partial_split = self.split_uncommitted_at_freeze(t_freeze);

        self.uncommitted_moves.retain(|m| m.t_end > t_freeze);

        let was_replace_split = matches!(partial_split, Some(PartialCommitSplit::Replace { .. }));

        match partial_split {
            Some(PartialCommitSplit::Replace { new_segment }) => {
                if let Some(front) = self.uncommitted_moves.front_mut() {
                    front.segment = *new_segment;
                    front.t_start = t_freeze;
                }
            }
            Some(PartialCommitSplit::DropFromQueue) => {
                self.uncommitted_moves.pop_front();
            }
            None => {}
        }

        let pre_plan_t_start = self.uncommitted_moves.back().map_or(t_freeze, |m| m.t_end);
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

        let boundary_v_cap = plan_segments
            .first()
            .map_or(f64::INFINITY, |s| boundary_path_speed_cap(s));
        let initial_v = initial_v_raw.min(boundary_v_cap);

        const REST_THRESHOLD_MM_S: f64 = 0.1;

        let (initial_v, initial_a, start_d2_override) = if initial_v < REST_THRESHOLD_MM_S {
            (0.0, 0.0, None)
        } else if initial_a > 0.0 {
            (initial_v, 0.0, start_d2_override)
        } else {
            (initial_v, initial_a, start_d2_override)
        };

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
        let (PlanOutput { fitted, stats }, time_offset, fallback_rung) =
            match plan_velocity(&plan_input) {
                Ok(out) => (out, t_freeze, 1u8),
                Err(rung1_err) => {
                    if let Some(rung2_out) =
                        try_rung2(was_replace_split, &prior_uncommitted, self, ctx, t_freeze)
                    {
                        let (out, offset) = rung2_out;
                        (out, offset, 2u8)
                    } else {
                        match try_rung3(
                            self,
                            &prior_uncommitted,
                            prior_t_appended,
                            prior_t_decel_start,
                            &prior_planned_fitted,
                            &prior_planned_meta,
                            ctx,
                        ) {
                            Ok((out, offset)) => (out, offset, 3u8),
                            Err(rung3_err) => {
                                self.uncommitted_moves = prior_uncommitted;
                                self.t_appended = prior_t_appended;
                                self.t_decel_start = prior_t_decel_start;
                                self.planned_fitted = prior_planned_fitted;
                                self.planned_meta = prior_planned_meta;
                                return Err(ShapeError::WitnessFallbackFailed {
                                    rung1: Box::new(rung1_err),
                                    rung3: Box::new(rung3_err),
                                });
                            }
                        }
                    }
                }
            };
        let solve_us = solve_start.elapsed().as_micros() as u64;

        let rebuild_start = Instant::now();

        for axis_idx in 0..3 {
            self.replace_uncommitted_axis_pieces(axis_idx, time_offset, time_offset, &fitted);
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

        let new_fitted: Vec<FittedSegment> = fitted
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

        let new_meta: Vec<EmitSegmentMeta> = self
            .uncommitted_moves
            .iter()
            .map(|m| EmitSegmentMeta {
                e_mode: m.segment.e_mode,
                extrusion_per_xy_mm: m.segment.extrusion_per_xy_mm,
            })
            .collect();

        self.planned_fitted = new_fitted;
        self.planned_meta = new_meta;

        if !freeze_zone_active {
            self.pending_freeze.clear();
        }

        let rebuild_us = rebuild_start.elapsed().as_micros() as u64;

        Ok(ReplanReport {
            split_us,
            solve_us,
            rebuild_us,
            window_segments,
            plan: stats,
            fallback_rung,
        })
    }
}

impl ShaperState {
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
        self.pending_freeze.clear();
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

    fn split_uncommitted_at_freeze(&self, t_freeze: f64) -> Option<PartialCommitSplit> {
        let (idx, planned) =
            self.planned_fitted.iter().enumerate().find(|(_, f)| {
                f.t_start - TIME_LOOKUP_TOLERANCE <= t_freeze && t_freeze < f.t_end
            })?;

        let move_ref = self.uncommitted_moves.get(idx)?;

        if matches!(
            move_ref.segment.e_mode,
            geometry::segment::EMode::Independent
        ) {
            return None;
        }

        let p_target = [
            nurbs::eval::eval(&planned.axes[0], t_freeze),
            nurbs::eval::eval(&planned.axes[1], t_freeze),
            nurbs::eval::eval(&planned.axes[2], t_freeze),
        ];

        let move_span_t = planned.t_end - planned.t_start;
        let s_seed = if move_span_t > TIME_LOOKUP_TOLERANCE {
            ((t_freeze - planned.t_start) / move_span_t)
                .clamp(NEWTON_SEED_CLAMP, 1.0 - NEWTON_SEED_CLAMP)
        } else {
            0.5
        };
        let s_freeze = invert_cubic_bezier_xyz_to_param(&move_ref.segment.xyz, p_target, s_seed)?;

        if s_freeze <= SPLIT_BOUNDARY_TOLERANCE {
            return None;
        }
        if s_freeze >= 1.0 - SPLIT_BOUNDARY_TOLERANCE {
            return Some(PartialCommitSplit::DropFromQueue);
        }

        let (_left, right) = split_cubic_bezier(&move_ref.segment.xyz, s_freeze);

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
        axis_cutoff_t: f64,
        fitted: &[FittedSegment],
    ) {
        let t_keep_cutoff = axis_cutoff_t;

        let pieces = &mut self.axes[axis_idx].pieces;
        while let Some(back) = pieces.back() {
            if back.u_start >= t_keep_cutoff - TIME_LOOKUP_TOLERANCE {
                pieces.pop_back();
            } else {
                break;
            }
        }

        if let Some(back) = pieces.back() {
            if back.u_end < time_offset - TIME_LOOKUP_TOLERANCE {
                let bridge_start = back.u_end;
                let bridge_val = back.evaluate(bridge_start);
                let degree = back.degree().max(1);
                let mut coeffs = vec![0.0f64; degree + 1];
                coeffs[0] = bridge_val;
                pieces.push_back(BezierPiece {
                    u_start: bridge_start,
                    u_end: time_offset,
                    coeffs,
                });
            }
        }

        debug_assert!(
            pieces
                .back()
                .map_or(true, |p| p.u_end >= time_offset - TIME_LOOKUP_TOLERANCE),
            "axis {} pieces deque ends at {:.12} but new plan starts at {:.12} — gap is unreachable after bridging",
            axis_idx,
            pieces.back().map_or(f64::NAN, |p| p.u_end),
            time_offset,
        );

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

fn try_rung2(
    was_replace_split: bool,
    prior_uncommitted: &std::collections::VecDeque<UncommittedMove>,
    state: &mut ShaperState,
    ctx: &ReplanContext,
    t_freeze: f64,
) -> Option<(crate::plan_velocity::PlanOutput, f64)> {
    if !was_replace_split {
        return None;
    }

    state.uncommitted_moves.pop_front();
    let retry_time_offset = prior_uncommitted[0].t_end.max(t_freeze);

    let retry_initial_v = state.read_path_speed_at(retry_time_offset, ctx.fallback_initial_v);

    let retry_segments: Vec<PlanSegment<'_>> = state
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

    let retry_boundary_v_cap = retry_segments
        .first()
        .map_or(f64::INFINITY, |s| boundary_path_speed_cap(s));
    let retry_iv = {
        const REST_THRESHOLD_MM_S: f64 = 0.1;
        let raw = retry_initial_v.min(retry_boundary_v_cap);
        if raw < REST_THRESHOLD_MM_S {
            0.0
        } else {
            raw
        }
    };

    let retry_input = PlanInput {
        segments: &retry_segments,
        grid_strategy: ctx.grid_strategy,
        worker_threads: ctx.worker_threads,
        kernels: ctx.kernels,
        fit_tolerance_mm: ctx.fit_tolerance_mm,
        beta_max_iters: ctx.beta_max_iters,
        beta_convergence_ratio: ctx.beta_convergence_ratio,
        e_limits: ctx.e_limits,
        initial_v: retry_iv,
        initial_a: 0.0,
        terminal_v: 0.0,
        safety_mode: ctx.safety_mode,
        start_d2_override: None,
    };

    plan_velocity(&retry_input)
        .ok()
        .map(|out| (out, retry_time_offset))
}

fn try_rung3(
    state: &mut ShaperState,
    _prior_uncommitted: &std::collections::VecDeque<UncommittedMove>,
    prior_t_appended: f64,
    prior_t_decel_start: f64,
    prior_planned_fitted: &[FittedSegment],
    prior_planned_meta: &[crate::emit_shaped::EmitSegmentMeta],
    ctx: &ReplanContext,
) -> Result<(crate::plan_velocity::PlanOutput, f64), ShapeError> {
    let new_move = state
        .uncommitted_moves
        .back()
        .expect("uncommitted_moves must be non-empty — new segment was just pushed")
        .clone();

    state.uncommitted_moves = {
        let mut single = std::collections::VecDeque::with_capacity(1);
        single.push_back(UncommittedMove {
            segment: new_move.segment.clone(),
            t_start: prior_t_appended,
            t_end: prior_t_appended,
        });
        single
    };

    state.t_appended = prior_t_appended;
    state.t_decel_start = prior_t_decel_start;
    state.planned_fitted = prior_planned_fitted.to_vec();
    state.planned_meta = prior_planned_meta.to_vec();

    let seg = state.uncommitted_moves.back().unwrap();
    let rung3_segments = [PlanSegment {
        temporal: temporal::multi::SegmentInput {
            curve: &seg.segment.xyz,
            limits: per_segment_limits(&seg.segment.xyz, ctx.limits, seg.segment.feedrate_mm_s),
            trailing_junction_chord_tolerance_mm: ctx.junction_chord_tolerance_mm,
        },
        e_mode: seg.segment.e_mode,
        extrusion_per_xy_mm: seg.segment.extrusion_per_xy_mm,
        e_independent: seg.segment.e_independent.as_ref(),
        feedrate_mm_s: seg.segment.feedrate_mm_s,
    }];

    let rung3_input = PlanInput {
        segments: &rung3_segments,
        grid_strategy: ctx.grid_strategy,
        worker_threads: ctx.worker_threads,
        kernels: ctx.kernels,
        fit_tolerance_mm: ctx.fit_tolerance_mm,
        beta_max_iters: ctx.beta_max_iters,
        beta_convergence_ratio: ctx.beta_convergence_ratio,
        e_limits: ctx.e_limits,
        initial_v: 0.0,
        initial_a: 0.0,
        terminal_v: 0.0,
        safety_mode: ctx.safety_mode,
        start_d2_override: None,
    };

    plan_velocity(&rung3_input).map(|out| (out, prior_t_appended))
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

fn boundary_path_speed_cap(seg: &PlanSegment<'_>) -> f64 {
    use nurbs::eval::{vector_derivative, vector_eval};
    let curve = seg.temporal.curve;
    let u0 = curve.knots()[0];
    let d1 = vector_derivative(curve);
    let tan = vector_eval(&d1.as_view(), u0);
    let mag = (tan[0] * tan[0] + tan[1] * tan[1] + tan[2] * tan[2]).sqrt();
    if mag < 1e-12 {
        return f64::INFINITY;
    }
    let v_max = seg.temporal.limits.v_max;
    let mut cap = f64::INFINITY;
    for ax in 0..3 {
        let dir = (tan[ax] / mag).abs();
        if dir > 1e-12 {
            cap = cap.min(v_max[ax] / dir);
        }
    }
    cap
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
