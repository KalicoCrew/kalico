//! Per-axis curve and segment dispatch data types (Phase C-B).
//!
//! `CurveLoadParams` and `SegmentPushParams` are the host-side intermediate
//! representations shared between `dispatch::McuPushPlan` and the wire-send
//! layer. The old `load_curve` / `push_segment` / `reset_curve_pool`
//! functions (which targeted the now-removed `LoadCurveCubic` / `PushSegment`
//! / `ResetCurvePool` wire messages) have been deleted. Their callers in
//! `motion-bridge` are similarly removed.

/// Parameters for loading a single per-axis cubic-Bezier curve.
///
/// Per-piece layout: `bp_per_piece[i]` are the four Bernstein control points
/// of piece `i`; `duration_per_piece[i]` is the duration of that piece in
/// seconds. Length of both slices must match.
#[derive(Debug, Clone)]
pub struct CurveLoadParams {
    /// Per-piece Bernstein control points (length = `piece_count`, ≤ 255).
    pub bp_per_piece: Vec<[f32; 4]>,
    /// Per-piece duration in seconds (length matches `bp_per_piece`).
    pub duration_per_piece: Vec<f32>,
}

impl CurveLoadParams {
    /// Construct from a host-side cubic NURBS via `nurbs::bezier::extract_bezier_pieces`.
    ///
    /// `t_start_s` / `t_end_s` are accepted for API symmetry with legacy call
    /// sites but are unused — the input curve already carries the absolute-time
    /// domain in its knot vector.
    pub fn from_scalar_nurbs_normalized(
        curve: &nurbs::ScalarNurbs<f64>,
        _t_start_s: f64,
        _t_end_s: f64,
    ) -> Self {
        let pieces = nurbs::bezier::extract_bezier_pieces(curve);
        let mut bp_per_piece: Vec<[f32; 4]> = Vec::with_capacity(pieces.len());
        let mut duration_per_piece: Vec<f32> = Vec::with_capacity(pieces.len());
        for piece in &pieces {
            // Cubic invariant — Bernstein basis with 4 control points.
            // Higher-degree input would be a planner bug at this point in
            // the pipeline (refit guarantees cubic). Pad / truncate
            // defensively so an out-of-spec input still produces a
            // well-formed wire frame rather than panicking inside the
            // dispatch closure.
            let bern = piece.to_bernstein();
            let mut bp = [0.0_f32; 4];
            for k in 0..4.min(bern.len()) {
                bp[k] = bern[k] as f32;
            }
            // If degree < 3, hold the last CP (constant tail).
            if bern.len() < 4 && !bern.is_empty() {
                let last = bern[bern.len() - 1] as f32;
                for k in bern.len()..4 {
                    bp[k] = last;
                }
            }
            bp_per_piece.push(bp);
            let dur = (piece.u_end - piece.u_start) as f32;
            duration_per_piece.push(dur);
        }
        Self {
            bp_per_piece,
            duration_per_piece,
        }
    }

    /// Number of cubic-Bezier pieces in this curve.
    pub fn piece_count(&self) -> usize {
        self.bp_per_piece.len()
    }
}

/// Per-segment push parameters: timing window, handles, and kinematics tag.
///
/// `x_handle_packed` / `y_handle_packed` / `z_handle_packed` /
/// `e_handle_packed` are curve-pool packed handles (generation + slot index)
/// filled in by the dispatch closure after successful `load_curve` calls.
/// Timing fields (`t_start` / `t_end`) are in MCU clock ticks.
#[derive(Debug, Clone, Copy)]
pub struct SegmentPushParams {
    pub id: u32,
    pub x_handle_packed: u32,
    pub y_handle_packed: u32,
    pub z_handle_packed: u32,
    pub e_handle_packed: u32,
    pub t_start: u64,
    pub t_end: u64,
    pub kinematics: u8,
    pub e_mode: u8,
    pub extrusion_ratio: f32,
}
