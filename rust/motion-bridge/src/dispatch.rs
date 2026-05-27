//! Per-MCU dispatch: maps a `ShapedSegment`'s per-axis NURBS curves onto the
//! configured MCU axis assignment, producing one [`McuPushPlan`] per MCU
//! that has at least one non-trivial curve to load.
//!
//! For CoreXY MCUs (`kinematics == KINEMATICS_COREXY`) the logical X and Y
//! curves are combined into motor-frame A = X+Y and B = X-Y **here on the
//! host** before being serialised over the wire. The MCU therefore receives
//! motor-frame curves in its X/Y handle slots and has no CoreXY transform in
//! its hot path.

use crate::planner::DispatchError;
use kalico_host_rt::producer::{CurveLoadParams, SegmentPushParams};
use runtime::segment::KinematicTag;
use trajectory::ShapedSegment;

/// `McuAxisConfig::kinematics` tag: Octopus CoreXY, motors A (slot 0) + B (slot 1).
///
/// Derived from [`KinematicTag::CoreXyAndE`] so the wire-ABI discriminant has a
/// single source of truth. The `const _: ()` assertion below pins the mapping so
/// a renumbering of `KinematicTag` produces a compile-time error rather than a
/// silent wire mismatch.
pub const KINEMATICS_COREXY: u8 = KinematicTag::CoreXyAndE as u8;

const _: () = assert!(
    KinematicTag::CoreXyAndE as u8 == 0,
    "wire-ABI invariant broken: KinematicTag::CoreXyAndE discriminant must be 0 \
     (matches KINEMATICS_COREXY on the host and the MCU firmware's kinematics byte)",
);

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
    /// Per-MCU runtime sizing limits as reported by `QueryRuntimeCaps`
    /// (or `McuCaps::default()` for firmware that predates the message).
    pub caps: McuCaps,
}

/// Subset of `RuntimeCapsResponse` that the dispatcher needs to enforce
/// per-MCU sizing limits when planning a curve.
///
/// Simple-MCU-contract revision (2026-05-28): the MCU now reports a single
/// `total_piece_memory` (bytes). The host derives per-axis budgets from this
/// at `init_planner` time. Each piece is 32 bytes on the wire (`PushPieces`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuCaps {
    /// Total bytes available for piece storage across all per-axis rings,
    /// as reported by `RuntimeCapsResponse.total_piece_memory`.
    pub total_piece_memory: u32,
}

impl Default for McuCaps {
    /// Large-profile defaults used when the per-MCU `QueryRuntimeCaps`
    /// round-trip fails (e.g. transport timeout during attach). 62 KB
    /// matches the H7 `RUNTIME_TARGET_LARGE` total piece buffer.
    fn default() -> Self {
        Self {
            total_piece_memory: 62 * 1024,
        }
    }
}

impl From<kalico_protocol::messages::RuntimeCapsResponse> for McuCaps {
    fn from(r: kalico_protocol::messages::RuntimeCapsResponse) -> Self {
        Self {
            total_piece_memory: r.total_piece_memory,
        }
    }
}

impl McuCaps {
    /// Maximum number of 32-byte pieces the MCU can hold in total.
    pub fn total_pieces(&self) -> usize {
        self.total_piece_memory as usize / 32
    }
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

/// De Casteljau subdivision of a cubic Bernstein polynomial at parameter `t`.
/// Returns `(left_half, right_half)` — two sets of cubic Bernstein control
/// points covering `[0, t]` and `[t, 1]` respectively.
pub fn de_casteljau_split(bp: [f32; 4], t: f32) -> ([f32; 4], [f32; 4]) {
    let [b0, b1, b2, b3] = bp;
    let p01 = b0 + t * (b1 - b0);
    let p12 = b1 + t * (b2 - b1);
    let p23 = b2 + t * (b3 - b2);
    let p012 = p01 + t * (p12 - p01);
    let p123 = p12 + t * (p23 - p12);
    let p0123 = p012 + t * (p123 - p012);
    ([b0, p01, p012, p0123], [p0123, p123, p23, b3])
}

/// Extract the sub-curve covering time window `[win_start, win_end]` (seconds
/// relative to curve start). Pieces entirely within the window are included
/// as-is. Pieces straddling a boundary are subdivided via de Casteljau.
pub fn extract_time_window(
    curve: &CurveLoadParams,
    win_start: f64,
    win_end: f64,
) -> CurveLoadParams {
    let mut result_bp = Vec::new();
    let mut result_dur = Vec::new();
    let mut elapsed = 0.0_f64;

    for i in 0..curve.bp_per_piece.len() {
        let d = curve.duration_per_piece[i] as f64;
        let piece_start = elapsed;
        let piece_end = elapsed + d;
        elapsed = piece_end;

        if piece_end <= win_start + 1e-12 || piece_start >= win_end - 1e-12 {
            continue;
        }

        if piece_start >= win_start - 1e-12 && piece_end <= win_end + 1e-12 {
            result_bp.push(curve.bp_per_piece[i]);
            result_dur.push(curve.duration_per_piece[i]);
            continue;
        }

        let mut cur_bp = curve.bp_per_piece[i];
        let mut cur_dur = d;
        let mut cur_start = piece_start;

        if cur_start < win_start - 1e-12 {
            let t = ((win_start - cur_start) / cur_dur) as f32;
            let (_, right) = de_casteljau_split(cur_bp, t);
            cur_bp = right;
            cur_dur *= 1.0 - t as f64;
            cur_start = win_start;
        }

        if cur_start + cur_dur > win_end + 1e-12 {
            let t = ((win_end - cur_start) / cur_dur) as f32;
            let (left, _) = de_casteljau_split(cur_bp, t);
            cur_bp = left;
            cur_dur *= t as f64;
        }

        result_bp.push(cur_bp);
        result_dur.push(cur_dur as f32);
    }

    CurveLoadParams {
        bp_per_piece: result_bp,
        duration_per_piece: result_dur,
    }
}

/// Split an `McuPushPlan` into sub-plans where every axis has
/// `≤ max_pieces` pieces. Returns the plan unchanged if no axis exceeds
/// the limit. Uses time-domain splitting with de Casteljau subdivision
/// for pieces straddling chunk boundaries.
///
/// `freq` is the MCU clock frequency in Hz, used to convert piece durations
/// (seconds) to clock ticks for sub-plan timing.
pub fn split_plan_if_needed(
    plan: McuPushPlan,
    max_pieces: usize,
    freq: f64,
) -> Result<Vec<McuPushPlan>, DispatchError> {
    split_recursive(plan, max_pieces, freq, 0)
}

fn split_recursive(
    plan: McuPushPlan,
    max_pieces: usize,
    freq: f64,
    depth: usize,
) -> Result<Vec<McuPushPlan>, DispatchError> {
    let max_pc = plan
        .curves_to_load
        .iter()
        .map(|(_, c)| c.piece_count())
        .max()
        .unwrap_or(0);

    if max_pc <= max_pieces {
        return Ok(vec![plan]);
    }

    if max_pieces < 3 {
        return Err(DispatchError::CapsExceeded {
            mcu_id: plan.mcu_id,
            pieces: max_pc,
            max_pieces,
        });
    }

    if depth > 8 {
        return Err(DispatchError::CapsExceeded {
            mcu_id: plan.mcu_id,
            pieces: max_pc,
            max_pieces,
        });
    }

    let stride = max_pieces - 2;

    // Find bottleneck axis (most pieces)
    let bottleneck_idx = plan
        .curves_to_load
        .iter()
        .enumerate()
        .max_by_key(|(_, (_, c))| c.piece_count())
        .map(|(i, _)| i)
        .unwrap();
    let bottleneck = &plan.curves_to_load[bottleneck_idx].1;

    // Compute split times from bottleneck's piece boundaries
    let mut split_times = vec![0.0_f64];
    let mut elapsed = 0.0_f64;
    for (i, d) in bottleneck.duration_per_piece.iter().enumerate() {
        elapsed += *d as f64;
        if (i + 1) % stride == 0 && i + 1 < bottleneck.piece_count() {
            split_times.push(elapsed);
        }
    }
    split_times.push(elapsed);

    let t_start_clock = plan.params.t_start;
    let t_end_clock = plan.params.t_end;
    let n_chunks = split_times.len() - 1;
    let mut chunks = Vec::with_capacity(n_chunks);
    let mut chunk_start_clock = t_start_clock;

    for w in 0..n_chunks {
        let win_start = split_times[w];
        let win_end = split_times[w + 1];

        let chunk_end_clock = if w == n_chunks - 1 {
            t_end_clock
        } else {
            let dur_clocks = (win_end - win_start) * freq;
            chunk_start_clock + dur_clocks.round() as u64
        };

        let sub_curves: Vec<(usize, CurveLoadParams)> = plan
            .curves_to_load
            .iter()
            .map(|(axis_idx, curve)| (*axis_idx, extract_time_window(curve, win_start, win_end)))
            .collect();

        let mut sub_params = plan.params;
        sub_params.t_start = chunk_start_clock;
        sub_params.t_end = chunk_end_clock;
        sub_params.id = 0;
        sub_params.x_handle_packed = UNUSED_HANDLE;
        sub_params.y_handle_packed = UNUSED_HANDLE;
        sub_params.z_handle_packed = UNUSED_HANDLE;
        sub_params.e_handle_packed = UNUSED_HANDLE;

        chunks.push(McuPushPlan {
            mcu_id: plan.mcu_id,
            curves_to_load: sub_curves,
            params: sub_params,
        });

        chunk_start_clock = chunk_end_clock;
    }

    // Recursive validation: if any chunk still exceeds max_pieces
    // (e.g. non-bottleneck axis had high local density), re-split it.
    let mut result = Vec::new();
    for chunk in chunks {
        let sub = split_recursive(chunk, max_pieces, freq, depth + 1)?;
        result.extend(sub);
    }

    Ok(result)
}

/// Build per-MCU push plans for a single shaped segment.
///
/// `t_start_clock` / `t_end_clock` are 64-bit MCU-clock values produced by
/// the temporal-to-clock conversion step (`planner::config::trajectory_to_clock`
/// or equivalent) — same value goes to every MCU for a given segment.
///
/// **CoreXY transform:** when `cfg.kinematics == KINEMATICS_COREXY` and both
/// `AXIS_X` and `AXIS_Y` are in `cfg.axes`, the logical X and Y curves are
/// combined into motor-frame curves before serialisation:
///   - Motor-A curve (stored in `AXIS_X` slot) = X + Y
///   - Motor-B curve (stored in `AXIS_Y` slot) = X − Y
///
/// The slot indices (0 = AXIS_X, 1 = AXIS_Y) are unchanged — only the
/// *contents* differ. Knot vectors are aligned via exact Bézier-piece
/// union before the pointwise add — no approximation, no fit error.
/// After the union, `nurbs::algebra::add` is guaranteed to succeed; if it
/// returns `Err`, the function panics with "post-union add failed — bridge
/// invariant violated".
pub fn build_push_params(
    shaped: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    t_start_clock: u64,
    t_end_clock: u64,
) -> Vec<McuPushPlan> {
    let mut plans = Vec::with_capacity(mcu_configs.len());

    for cfg in mcu_configs {
        // For CoreXY MCUs that drive both AXIS_X and AXIS_Y, pre-compute the
        // motor-frame curves once per MCU. These replace the logical X/Y
        // curves in the curves_to_load list below.
        let corexy_motor_curves: Option<(nurbs::ScalarNurbs<f64>, nurbs::ScalarNurbs<f64>)> =
            if cfg.kinematics == KINEMATICS_COREXY
                && cfg.axes.contains(&AXIS_X)
                && cfg.axes.contains(&AXIS_Y)
                && AXIS_X < shaped.axes.len()
                && AXIS_Y < shaped.axes.len()
            {
                let x = &shaped.axes[AXIS_X];
                let y = &shaped.axes[AXIS_Y];
                // Align knot vectors via exact Bézier-piece union (no fit
                // error). After the union, add is guaranteed to succeed;
                // the expect below is the unreachable sentinel.
                let motor_a = nurbs::algebra::add_with_knot_union(x, y).unwrap_or_else(|e| {
                    panic!("post-union add failed — bridge invariant violated (motor-A): {e:?}")
                });
                let motor_b_neg_y = nurbs::algebra::scalar_multiply(y, -1.0_f64);
                let motor_b = nurbs::algebra::add_with_knot_union(x, &motor_b_neg_y)
                    .unwrap_or_else(|e| {
                        panic!("post-union add failed — bridge invariant violated (motor-B): {e:?}")
                    });
                Some((motor_a, motor_b))
            } else {
                None
            };

        let mut curves_to_load: Vec<(usize, CurveLoadParams)> = Vec::new();
        for &axis_idx in &cfg.axes {
            if axis_idx >= shaped.axes.len() {
                continue;
            }

            // Select the curve: for CoreXY MCUs, substitute motor-frame
            // curves in the X and Y slots; all other axes pass through.
            let curve_params = if let Some((ref motor_a, ref motor_b)) = corexy_motor_curves {
                match axis_idx {
                    AXIS_X => CurveLoadParams::from_scalar_nurbs_normalized(
                        motor_a,
                        shaped.t_start,
                        shaped.t_end,
                    ),
                    AXIS_Y => CurveLoadParams::from_scalar_nurbs_normalized(
                        motor_b,
                        shaped.t_start,
                        shaped.t_end,
                    ),
                    _ => CurveLoadParams::from_scalar_nurbs_normalized(
                        &shaped.axes[axis_idx],
                        shaped.t_start,
                        shaped.t_end,
                    ),
                }
            } else {
                CurveLoadParams::from_scalar_nurbs_normalized(
                    &shaped.axes[axis_idx],
                    shaped.t_start,
                    shaped.t_end,
                )
            };

            // 2026-05-11 fix — DO NOT skip "trivially constant" curves.
            // The previous optimization left the corresponding MCU handle
            // at UNUSED_SENTINEL, and the engine's UNUSED-handle semantic
            // (engine.rs::tick_with_current X/Y/Z branches) returned
            // (0.0, 0.0) for "axis at zero". That's wrong for absolute-
            // coordinate trajectory segments: an axis whose curve was
            // skipped one segment but sent the next (because refit-noise
            // pushed the constant-check just past 1e-12) produced phantom
            // position jumps equal to the actual axis position (e.g.,
            // 100 mm in Y when jogging on X), reliably tripping
            // STEP_BURST_EXCEEDED on the next segment activation.
            //
            // The architectural fix is twofold: (a) the engine now treats
            // UNUSED as "hold prev value" (engine.rs same commit), and
            // (b) the bridge sends every kinematic axis's curve every
            // segment — including constants — so the engine's hold value
            // is always anchored to klippy's current commanded position.
            // Slot-economy cost: every segment uses one slot per
            // kinematic axis (3 for X/Y/Z on a CoreXY+Z setup) instead of
            // 1 for pure-X jogs. With CURVE_POOL_N=16 and credit-flow
            // backpressure (producer.rs::push_segment_with_timeout), this
            // throttles in-flight depth to ~5 segments — fine for the
            // MVP; right-sized per-slot capacity is a future
            // optimization if production prints need more depth.
            curves_to_load.push((axis_idx, curve_params));
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

#[cfg(test)]
mod tests;

