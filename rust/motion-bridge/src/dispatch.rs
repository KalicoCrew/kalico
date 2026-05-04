//! Per-MCU dispatch: maps a `ShapedSegment`'s per-axis NURBS curves onto the
//! configured MCU axis assignment, producing one [`McuPushPlan`] per MCU
//! that has at least one non-trivial curve to load.
//!
//! Trivially-constant curves (all control points equal within `EPS_CONST`)
//! are skipped — the MCU keeps the corresponding handle slot at
//! [`UNUSED_HANDLE`] and the per-sample evaluator treats it as "axis idle".

use kalico_host_rt::producer::{CurveLoadParams, SegmentPushParams};
use trajectory::ShapedSegment;

use crate::curve_chunker::{ChunkedAxisCurve, chunk_scalar_nurbs, split_chunked_at_breaks};

/// Sentinel "no curve loaded" handle value. The firmware checks
/// `handle == 0xFFFE_FFFE` to skip evaluating that axis for the segment.
pub const UNUSED_HANDLE: u32 = 0xFFFE_FFFE;

pub const AXIS_X: usize = 0;
pub const AXIS_Y: usize = 1;
pub const AXIS_Z: usize = 2;

/// Epsilon for the "all control points equal" trivial-constant test.
const EPS_CONST: f64 = 1e-12;

/// Per-MCU configuration: which `ShapedSegment` axes this MCU is responsible
/// for, plus the firmware kinematics tag.
#[derive(Debug, Clone)]
pub struct McuAxisConfig {
    pub mcu_id: u32,
    /// Indices into `ShapedSegment::axes` (0=X, 1=Y, 2=Z) that this MCU drives.
    pub axes: Vec<usize>,
    /// Kinematics tag forwarded to the MCU in `SegmentPushParams::kinematics`.
    pub kinematics: u8,
}

/// One MCU's slice of work for a single shaped segment: the curves it must
/// load (with the axis index they bind to) plus a partially-built
/// `SegmentPushParams` whose handle fields will be filled in once the
/// curve loads return packed handles.
#[derive(Debug, Clone)]
pub struct McuPushPlan {
    pub mcu_id: u32,
    /// `(axis_idx, curve)` pairs in the order the dispatcher discovered them.
    pub curves_to_load: Vec<(usize, CurveLoadParams)>,
    pub params: SegmentPushParams,
}

impl McuPushPlan {
    /// Fill the appropriate `*_handle_packed` field of `params` for the given
    /// shaped-segment axis index.
    pub fn set_handle(&mut self, axis_idx: usize, packed: u32) {
        match axis_idx {
            AXIS_X => self.params.x_handle_packed = packed,
            AXIS_Y => self.params.y_handle_packed = packed,
            AXIS_Z => self.params.z_handle_packed = packed,
            _ => {} // E lives in e_handle_packed and is dispatched separately
        }
    }
}

pub fn is_trivially_constant(curve: &nurbs::ScalarNurbs<f64>) -> bool {
    let cps = curve.control_points();
    if cps.is_empty() {
        return true;
    }
    let first = cps[0];
    cps.iter().all(|&v| (v - first).abs() <= EPS_CONST)
}

/// Build per-MCU push plans for a single shaped segment.
///
/// `t_start_clock` / `t_end_clock` are 64-bit MCU-clock values produced by
/// the temporal-to-clock conversion step (`planner::config::trajectory_to_clock`
/// or equivalent) — same value goes to every MCU for a given segment.
pub fn build_push_params(
    shaped: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    t_start_clock: u64,
    t_end_clock: u64,
) -> Vec<McuPushPlan> {
    let mut plans = Vec::with_capacity(mcu_configs.len());

    for cfg in mcu_configs {
        let mut curves_to_load: Vec<(usize, CurveLoadParams)> = Vec::new();
        for &axis_idx in &cfg.axes {
            if axis_idx >= shaped.axes.len() {
                continue;
            }
            let curve = &shaped.axes[axis_idx];
            if is_trivially_constant(curve) {
                continue;
            }
            curves_to_load.push((
                axis_idx,
                CurveLoadParams::from_scalar_nurbs_normalized(
                    curve,
                    shaped.t_start,
                    shaped.t_end,
                ),
            ));
        }

        if curves_to_load.is_empty() {
            continue;
        }

        let params = SegmentPushParams {
            id: 0,
            x_handle_packed: UNUSED_HANDLE,
            y_handle_packed: UNUSED_HANDLE,
            z_handle_packed: UNUSED_HANDLE,
            e_handle_packed: UNUSED_HANDLE,
            t_start: t_start_clock,
            t_end: t_end_clock,
            kinematics: cfg.kinematics,
            e_mode: 2, // Travel
            extrusion_ratio: 0.0,
        };

        plans.push(McuPushPlan {
            mcu_id: cfg.mcu_id,
            curves_to_load,
            params,
        });
    }

    plans
}

// ── Chunked dispatch (multi-piece-per-axis support) ─────────────────────
//
// See `docs/superpowers/specs/2026-05-04-multi-piece-dispatch-design.md`.
//
// `build_chunked_push_plans` is the new entry point that pre-chunks each
// axis curve to fit `kalico_load_curve`'s 63-f32-per-buffer wire cap, then
// emits one `McuChunkPlan` per (mcu, chunk-index) pair. The dispatch closure
// in `bridge.rs` walks these plans, allocating slots / loading / pushing
// per chunk so the MCU can begin executing chunk_0 while the host is still
// loading chunk_1.

/// One chunk's worth of curve loads + the segment time window it covers.
/// Constructed by `build_chunked_push_plans`; consumed by the dispatch
/// closure which fills in clock-domain `t_start`/`t_end` and per-chunk
/// `segment_id` before issuing `kalico_push_segment`.
#[derive(Debug, Clone)]
pub struct McuChunkPlan {
    /// `(axis_idx, curve)` pairs to load for this chunk.
    pub curves_to_load: Vec<(usize, CurveLoadParams)>,
    /// Chunk parametric window in the *source NURBS knot domain* (which
    /// equals seconds in trajectory's batch timeline). Note: per-chunk
    /// MCU-clock conversion derives `t_start_clock` / `t_end_clock` directly
    /// from these absolute-time values plus the per-MCU `entry.0` schedule
    /// base — the design memo's `seg.t_start + chunk.t_start_s` formulation
    /// would double-add the offset. See dispatch closure in `bridge.rs`.
    pub t_start_s: f64,
    pub t_end_s: f64,
}

/// All chunks for one MCU for one logical shaped segment.
#[derive(Debug, Clone)]
pub struct ChunkedMcuPlan {
    pub mcu_id: u32,
    pub kinematics: u8,
    pub e_mode: u8,
    pub extrusion_ratio: f32,
    pub chunks: Vec<McuChunkPlan>,
}

/// Build per-MCU chunked push plans for a single shaped segment.
///
/// Pre-chunks each non-trivial axis curve via `chunk_scalar_nurbs`, then
/// (defensively, for the multi-axis-non-trivial case) takes the union of
/// per-axis interior breakpoints and re-splits every axis at the union so
/// chunk indices line up across axes. For the MVP single-axis-X-move and
/// pure-Z-move paths exactly one axis is non-trivial, so the union collapses
/// to that axis's breakpoints — the union-split is then a no-op.
///
/// Per-chunk `CurveLoadParams::from_scalar_nurbs_normalized` calls remap
/// each chunk's knot domain to `[0, 1]` independently, so the MCU evaluates
/// each chunk on its own `[0, 1]` progress just like a single-piece move.
pub fn build_chunked_push_plans(
    shaped: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
) -> Vec<ChunkedMcuPlan> {
    let mut plans = Vec::with_capacity(mcu_configs.len());

    for cfg in mcu_configs {
        // 1) Chunk each non-trivial axis independently.
        let mut per_axis: Vec<(usize, ChunkedAxisCurve)> = Vec::new();
        for &axis_idx in &cfg.axes {
            if axis_idx >= shaped.axes.len() {
                continue;
            }
            let curve = &shaped.axes[axis_idx];
            if is_trivially_constant(curve) {
                continue;
            }
            let chunked = chunk_scalar_nurbs(curve);
            per_axis.push((axis_idx, chunked));
        }
        if per_axis.is_empty() {
            continue;
        }

        // 2) Build union of per-axis interior breakpoints. For the
        //    single-non-trivial-axis case this is just that axis's own
        //    breaks → split is a no-op.
        let mut union_breaks: Vec<f64> = Vec::new();
        for (_, chunked) in &per_axis {
            if chunked.windows.len() < 2 {
                continue;
            }
            for win in &chunked.windows[..chunked.windows.len() - 1] {
                union_breaks.push(win.1);
            }
        }
        union_breaks.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        union_breaks.dedup_by(|a, b| (*a - *b).abs() < 1e-12);

        // 3) Re-split each axis at every union breakpoint.
        let aligned: Vec<(usize, ChunkedAxisCurve)> = per_axis
            .into_iter()
            .map(|(idx, c)| (idx, split_chunked_at_breaks(c, &union_breaks)))
            .collect();

        // Sanity: after union-split every axis must have the same chunk
        // count and matching window boundaries. (For single-non-trivial-axis
        // this is trivially true.)
        let chunk_count = aligned[0].1.chunks.len();
        debug_assert!(
            aligned.iter().all(|(_, c)| c.chunks.len() == chunk_count),
            "post-union-split chunk-count mismatch across axes"
        );

        // 4) Compose `McuChunkPlan` per chunk index. Window time uses axis 0
        //    (all axes share by construction post-union-split).
        let windows: Vec<(f64, f64)> = aligned[0].1.windows.clone();
        let mut chunks: Vec<McuChunkPlan> = Vec::with_capacity(chunk_count);
        for chunk_idx in 0..chunk_count {
            let (u_start, u_end) = windows[chunk_idx];
            let mut curves_to_load: Vec<(usize, CurveLoadParams)> =
                Vec::with_capacity(aligned.len());
            for (axis_idx, chunked) in &aligned {
                let chunk_curve = &chunked.chunks[chunk_idx];
                // Each chunk's knots live in `[u_start, u_end]` of the source
                // NURBS knot domain (which is absolute time in seconds for
                // trajectory output). Normalize the chunk to its own [0, 1].
                curves_to_load.push((
                    *axis_idx,
                    CurveLoadParams::from_scalar_nurbs_normalized(
                        chunk_curve,
                        u_start,
                        u_end,
                    ),
                ));
            }
            chunks.push(McuChunkPlan {
                curves_to_load,
                t_start_s: u_start,
                t_end_s: u_end,
            });
        }

        plans.push(ChunkedMcuPlan {
            mcu_id: cfg.mcu_id,
            kinematics: cfg.kinematics,
            e_mode: 2, // Travel — matches build_push_params today
            extrusion_ratio: 0.0,
            chunks,
        });
    }

    plans
}

/// Build a `SegmentPushParams` skeleton for a chunk plan. Handles are filled
/// in after each chunk's `kalico_load_curve` returns; clock fields are filled
/// in by the dispatch closure once per-MCU clock conversion runs.
pub fn chunk_push_params_skeleton(plan: &ChunkedMcuPlan) -> SegmentPushParams {
    SegmentPushParams {
        id: 0,
        x_handle_packed: UNUSED_HANDLE,
        y_handle_packed: UNUSED_HANDLE,
        z_handle_packed: UNUSED_HANDLE,
        e_handle_packed: UNUSED_HANDLE,
        t_start: 0,
        t_end: 0,
        kinematics: plan.kinematics,
        e_mode: plan.e_mode,
        extrusion_ratio: plan.extrusion_ratio,
    }
}

/// Patch the appropriate `*_handle_packed` field of `params` for the given
/// shaped-segment axis index. Mirror of `McuPushPlan::set_handle` for the
/// chunked dispatch path (which doesn't carry an `McuPushPlan` per chunk).
pub fn set_axis_handle(params: &mut SegmentPushParams, axis_idx: usize, packed: u32) {
    match axis_idx {
        AXIS_X => params.x_handle_packed = packed,
        AXIS_Y => params.y_handle_packed = packed,
        AXIS_Z => params.z_handle_packed = packed,
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geometry::segment::EMode;
    use nurbs::ScalarNurbs;

    fn linear_curve(a: f64, b: f64) -> ScalarNurbs<f64> {
        // degree-3 Bézier with collinear cps a, lerp(1/3), lerp(2/3), b
        let cps = vec![a, a + (b - a) / 3.0, a + 2.0 * (b - a) / 3.0, b];
        ScalarNurbs::try_new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0], cps, None)
            .unwrap()
    }

    fn constant_curve(v: f64) -> ScalarNurbs<f64> {
        ScalarNurbs::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![v, v, v, v],
            None,
        )
        .unwrap()
    }

    fn shaped(axes: [ScalarNurbs<f64>; 3]) -> ShapedSegment {
        ShapedSegment {
            axes,
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        }
    }

    fn cfgs() -> Vec<McuAxisConfig> {
        vec![
            McuAxisConfig {
                mcu_id: 0,
                axes: vec![AXIS_X, AXIS_Y],
                kinematics: 1,
            },
            McuAxisConfig {
                mcu_id: 1,
                axes: vec![AXIS_Z],
                kinematics: 2,
            },
        ]
    }

    #[test]
    fn x_move_dispatches_to_octopus_only() {
        let seg = shaped([linear_curve(0.0, 10.0), constant_curve(0.0), constant_curve(0.0)]);
        let plans = build_push_params(&seg, &cfgs(), 1_000, 2_000);

        assert_eq!(plans.len(), 1, "only Octopus should get a plan");
        let plan = &plans[0];
        assert_eq!(plan.mcu_id, 0);
        assert_eq!(plan.curves_to_load.len(), 1);
        assert_eq!(plan.curves_to_load[0].0, AXIS_X);

        // All handles still UNUSED — caller fills them after load_curve returns.
        assert_eq!(plan.params.x_handle_packed, UNUSED_HANDLE);
        assert_eq!(plan.params.y_handle_packed, UNUSED_HANDLE);
        assert_eq!(plan.params.z_handle_packed, UNUSED_HANDLE);
        assert_eq!(plan.params.e_handle_packed, UNUSED_HANDLE);
        assert_eq!(plan.params.t_start, 1_000);
        assert_eq!(plan.params.t_end, 2_000);
        assert_eq!(plan.params.kinematics, 1);
        assert_eq!(plan.params.e_mode, 2);
    }

    #[test]
    fn z_move_dispatches_to_f446_only() {
        let seg = shaped([constant_curve(0.0), constant_curve(0.0), linear_curve(0.0, 5.0)]);
        let plans = build_push_params(&seg, &cfgs(), 1_000, 2_000);

        assert_eq!(plans.len(), 1, "only F446 should get a plan");
        let plan = &plans[0];
        assert_eq!(plan.mcu_id, 1);
        assert_eq!(plan.curves_to_load.len(), 1);
        assert_eq!(plan.curves_to_load[0].0, AXIS_Z);
        assert_eq!(plan.params.z_handle_packed, UNUSED_HANDLE);
        assert_eq!(plan.params.kinematics, 2);
    }

    #[test]
    fn set_handle_fills_correct_field() {
        let seg = shaped([linear_curve(0.0, 10.0), constant_curve(0.0), constant_curve(0.0)]);
        let mut plans = build_push_params(&seg, &cfgs(), 0, 100);
        plans[0].set_handle(AXIS_X, 0xCAFE);
        assert_eq!(plans[0].params.x_handle_packed, 0xCAFE);
        assert_eq!(plans[0].params.y_handle_packed, UNUSED_HANDLE);
    }

    /// Helper: build a synthetic piecewise-Bézier scalar NURBS of `n_pieces`
    /// pieces of `degree`, ramping linearly across `[0, n_pieces]` time.
    fn synth_piecewise_axis(degree: u8, n_pieces: usize) -> ScalarNurbs<f64> {
        let p = degree as usize;
        let mut knots: Vec<f64> = Vec::with_capacity((n_pieces + 1) * p + 2);
        for _ in 0..=p {
            knots.push(0.0);
        }
        for i in 1..n_pieces {
            for _ in 0..p {
                knots.push(i as f64);
            }
        }
        for _ in 0..=p {
            knots.push(n_pieces as f64);
        }
        let n_cps = p * n_pieces + 1;
        let cps: Vec<f64> = (0..n_cps).map(|i| i as f64).collect();
        ScalarNurbs::try_new(degree, knots, cps, None).unwrap()
    }

    fn shaped_with_axis(
        axis_idx: usize,
        axis: ScalarNurbs<f64>,
        t_start: f64,
        t_end: f64,
    ) -> ShapedSegment {
        let zero = constant_curve(0.0);
        let mut axes = [zero.clone(), zero.clone(), zero];
        axes[axis_idx] = axis;
        ShapedSegment {
            axes,
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start,
            t_end,
        }
    }

    /// Multi-chunk path: a degree-9 27-piece curve (well above the 5-piece
    /// cap) should produce 6 chunks per MCU. Mirrors integration test #10
    /// from the design memo §6 (`multi_chunk_x_move`); kept in-lib because
    /// the `tests/sim_motion.rs` integration target cannot compile on macOS
    /// today (pre-existing pyo3 lib-name mismatch).
    #[test]
    fn multi_chunk_x_move_emits_six_chunks() {
        let axis = synth_piecewise_axis(9, 27);
        let seg = shaped_with_axis(AXIS_X, axis, 0.0, 27.0);
        let cfg = vec![McuAxisConfig {
            mcu_id: 0,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: 0,
        }];
        let plans = build_chunked_push_plans(&seg, &cfg);
        assert_eq!(plans.len(), 1, "X-only move → only Octopus has work");
        let plan = &plans[0];
        assert_eq!(plan.chunks.len(), 6, "27 pieces / 5 per chunk = 6 chunks");

        // Per-chunk load list must contain exactly one (axis_idx=AXIS_X, ...).
        for (i, chunk) in plan.chunks.iter().enumerate() {
            assert_eq!(
                chunk.curves_to_load.len(),
                1,
                "chunk {i} should load exactly the X axis"
            );
            assert_eq!(chunk.curves_to_load[0].0, AXIS_X);
            // Wire-cap regression: every encoded chunk fits the 63-f32 cap.
            let load_params = &chunk.curves_to_load[0].1;
            assert!(
                load_params.knots_f32.len() <= 63,
                "chunk {i} knots {} > 63 wire cap",
                load_params.knots_f32.len()
            );
            assert!(
                load_params.cps_f32.len() <= 63,
                "chunk {i} cps {} > 63 wire cap",
                load_params.cps_f32.len()
            );
        }

        // Time windows must tile [0, 27] contiguously.
        let mut prev = 0.0;
        for chunk in &plan.chunks {
            assert_eq!(chunk.t_start_s, prev);
            assert!(chunk.t_end_s > chunk.t_start_s);
            prev = chunk.t_end_s;
        }
        assert_eq!(prev, 27.0);
    }

    /// Single-piece (1 Bézier piece, well under the 5-piece chunk cap)
    /// linear-cubic move must produce exactly one chunk per MCU and the
    /// chunk's `CurveLoadParams` must be byte-identical to what
    /// `build_push_params` emits today.
    #[test]
    fn build_chunked_push_plans_single_chunk_matches_today() {
        let seg = shaped([linear_curve(0.0, 10.0), constant_curve(0.0), constant_curve(0.0)]);
        let cfgs = cfgs();

        // Today's path
        let legacy = build_push_params(&seg, &cfgs, 1_000, 2_000);
        // New path
        let chunked = build_chunked_push_plans(&seg, &cfgs);

        assert_eq!(chunked.len(), legacy.len(), "plan-per-MCU count");
        assert_eq!(chunked.len(), 1, "X-only move → only Octopus has work");
        let plan = &chunked[0];
        assert_eq!(plan.mcu_id, legacy[0].mcu_id);
        assert_eq!(plan.kinematics, legacy[0].params.kinematics);
        assert_eq!(plan.chunks.len(), 1, "single piece → single chunk");

        let chunk = &plan.chunks[0];
        assert_eq!(chunk.curves_to_load.len(), legacy[0].curves_to_load.len());
        // Byte-identical comparison of the per-axis CurveLoadParams.
        for ((axis_a, params_a), (axis_b, params_b)) in
            chunk.curves_to_load.iter().zip(&legacy[0].curves_to_load)
        {
            assert_eq!(axis_a, axis_b);
            assert_eq!(params_a.degree, params_b.degree);
            assert_eq!(params_a.knots_f32, params_b.knots_f32);
            assert_eq!(params_a.cps_f32, params_b.cps_f32);
        }
        // Window covers the full segment (since seg.t_start=0, seg.t_end=1
        // and curve knots span [0, 1]).
        assert_eq!(chunk.t_start_s, 0.0);
        assert_eq!(chunk.t_end_s, 1.0);
    }
}
