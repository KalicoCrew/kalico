//! Per-segment enqueue adapter: flatten a `ShapedSegment` into absolute-timed
//! `PieceEntry` batches per `(mcu, axis)`. Replaces `dispatch::build_push_params`.
//!
//! See spec §3.2.
//!
//! # CoreXY transform
//!
//! When `cfg.kinematics == KINEMATICS_COREXY` and both `AXIS_X` and `AXIS_Y`
//! are in `cfg.axes`, the logical X and Y NURBS are combined into motor-frame
//! curves before flattening:
//!
//! - Motor-A (stored in `AXIS_X` slot) = X + Y
//! - Motor-B (stored in `AXIS_Y` slot) = X − Y
//!
//! All other axes pass through unchanged.
//!
//! # E axis
//!
//! E is intentionally not emitted here. The extruder is a follower of shaped
//! XY motion; `seg.extrusion_per_xy_mm` and `seg.e_independent` are handled
//! upstream. This adapter only produces pieces for the axes in `cfg.axes`.

use crate::dispatch::{McuAxisConfig, AXIS_X, AXIS_Y, KINEMATICS_COREXY};
use crate::pump::{AxisKey, EnqueueMsg};
use nurbs::ScalarNurbs;
use runtime::piece_ring::PieceEntry;
use trajectory::ShapedSegment;

/// Build per-`(mcu, axis)` enqueue messages for one shaped segment.
///
/// `project(mcu_id, host_secs) -> mcu_clock` converts a host-time instant to
/// that MCU's absolute clock (the router's `host_time_to_mcu_clock`). `t0` is
/// the shared anchor (host seconds); a piece whose planner-domain interval
/// starts at `u_start` has host time `t0 + u_start`. `fresh_stream` is
/// forwarded onto each [`EnqueueMsg`].
///
/// # Returns
///
/// One [`EnqueueMsg`] per `(mcu, axis)` pair that has at least one piece.
/// Empty axes (e.g. a static Z on a move with no Z component) still produce a
/// single constant-valued piece — the adapter never inspects piece content.
///
/// # Example
///
/// ```rust,ignore
/// let msgs = enqueue_segment(&seg, &mcu_configs, t0, false, |_mcu, hs| {
///     (hs * freq) as u64
/// });
/// ```
pub fn enqueue_segment<P>(
    seg: &ShapedSegment,
    mcu_configs: &[McuAxisConfig],
    t0: f64,
    fresh_stream: bool,
    project: P,
) -> Vec<EnqueueMsg>
where
    P: Fn(u32, f64) -> u64,
{
    let mut out = Vec::new();

    for cfg in mcu_configs {
        // CoreXY: pre-compute motor-frame A = X+Y, B = X−Y for MCUs that
        // drive both logical axes. The motor struct holds the owned results so
        // lifetime extends to the per-axis flatten step below.
        let corexy = cfg.kinematics == KINEMATICS_COREXY
            && cfg.axes.contains(&AXIS_X)
            && cfg.axes.contains(&AXIS_Y)
            && AXIS_X < seg.axes.len()
            && AXIS_Y < seg.axes.len();

        let motor: Option<(ScalarNurbs<f64>, ScalarNurbs<f64>)> = if corexy {
            let x = &seg.axes[AXIS_X];
            let y = &seg.axes[AXIS_Y];
            // scalar_multiply returns ScalarNurbs directly (no Result).
            let neg_y = nurbs::algebra::scalar_multiply(y, -1.0_f64);
            // add_with_knot_union returns Result; inputs share the same curve
            // origin so the knot union always succeeds.
            let a = nurbs::algebra::add_with_knot_union(x, y)
                .unwrap_or_else(|e| panic!("CoreXY motor-A knot-union add failed (invariant violation — all ShapedSegment axes share one time domain): {e:?}"));
            let b = nurbs::algebra::add_with_knot_union(x, &neg_y)
                .unwrap_or_else(|e| panic!("CoreXY motor-B knot-union add failed (invariant violation — all ShapedSegment axes share one time domain): {e:?}"));
            Some((a, b))
        } else {
            None
        };

        for &axis_idx in &cfg.axes {
            if axis_idx >= seg.axes.len() {
                continue;
            }

            // Select the curve to flatten: motor-frame for CoreXY X/Y slots,
            // pass-through for everything else.
            let curve: &ScalarNurbs<f64> = match (&motor, axis_idx) {
                (Some((a, _)), idx) if idx == AXIS_X => a,
                (Some((_, b)), idx) if idx == AXIS_Y => b,
                _ => &seg.axes[axis_idx],
            };

            let pieces = flatten_axis(curve, t0, cfg.mcu_id, &project);
            if !pieces.is_empty() {
                out.push(EnqueueMsg {
                    key: AxisKey {
                        mcu_id: cfg.mcu_id,
                        axis: axis_idx as u8,
                    },
                    pieces,
                    fresh_stream,
                });
            }
        }
    }

    out
}

/// Decompose `curve` into its constituent cubic Bézier pieces and produce one
/// [`PieceEntry`] per piece with Bernstein coefficients cast to `f32` and
/// `start_time` derived via `project`.
fn flatten_axis<P>(
    curve: &ScalarNurbs<f64>,
    t0: f64,
    mcu_id: u32,
    project: &P,
) -> Vec<PieceEntry>
where
    P: Fn(u32, f64) -> u64,
{
    let bps = nurbs::bezier::extract_bezier_pieces(curve);
    let mut out = Vec::with_capacity(bps.len());

    for bp in &bps {
        let bern = bp.to_bernstein();

        debug_assert_eq!(
            bern.len(),
            4,
            "expected cubic (degree-3) Bernstein coeffs; the pipeline is uniform-cubic per CLAUDE.md, got {}",
            bern.len()
        );
        let mut coeffs = [0.0_f32; 4];
        let n = bern.len().min(4);
        for k in 0..n {
            // f64→f32: the MCU wire/eval format is f32; ~1e-7 precision loss is acceptable.
            coeffs[k] = bern[k] as f32;
        }
        // Pipeline is uniform cubic (always 4 coeffs); debug_assert above enforces it.
        // The min()+pad is a release-mode safety net for a degenerate/malformed piece —
        // it is a constant-extension of the last coefficient, NOT a correct degree-elevation,
        // so it is only geometrically valid for the degree-0 case it should never hit.
        if n > 0 && n < 4 {
            let last = bern[n - 1] as f32;
            for k in n..4 {
                coeffs[k] = last;
            }
        }

        let start_time = project(mcu_id, t0 + bp.u_start);
        let duration = (bp.u_end - bp.u_start) as f32;

        out.push(PieceEntry {
            start_time,
            coeffs,
            duration,
            _reserved: 0,
        });
    }

    // SIPDIAG18 (revert): the -308 born-late is tick-rate-coupled (lateness
    // 1054µs@10kHz → 137µs@40kHz), i.e. a piece-drain backlog: the engine
    // advances <=1 piece/tick, so a burst of pieces shorter than the tick period
    // piles their start_times into the past. Log the per-call duration profile
    // for the H7 (mcu_id 0) so we can SEE the sub-tick pieces. REVERT after.
    if mcu_id == 0 && !out.is_empty() {
        use std::io::Write as _;
        if let Ok(mut fh) = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open("/home/dderg/printer_data/logs/piece-dur.log")
        {
            let durs_us: Vec<u32> = out.iter().map(|p| (p.duration * 1.0e6) as u32).collect();
            let min = durs_us.iter().copied().min().unwrap_or(0);
            let max = durs_us.iter().copied().max().unwrap_or(0);
            let lt25 = durs_us.iter().filter(|&&d| d < 25).count();
            let lt100 = durs_us.iter().filter(|&&d| d < 100).count();
            let head: Vec<u32> = durs_us.iter().take(30).copied().collect();
            let _ = writeln!(
                fh,
                "[piece-dur] n={} min_us={} max_us={} n_lt25us={} n_lt100us={} | first30_us={:?}",
                durs_us.len(), min, max, lt25, lt100, head,
            );
        }
    }

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dispatch::McuCaps;
    use geometry::segment::EMode;

    /// Build a linear scalar NURBS over `[0, 1]` from Bernstein control points.
    /// A linear move from `p0` to `p1` has Bernstein coefficients
    /// `[p0, p0+Δ/3, p0+2Δ/3, p1]` for a degree-3 curve.
    fn linear_axis(p0: f64, p1: f64) -> ScalarNurbs<f64> {
        let d = p1 - p0;
        let bern = [p0, p0 + d / 3.0, p0 + 2.0 * d / 3.0, p1];
        let piece = nurbs::bezier::BezierPiece::from_bernstein(&bern, 0.0_f64, 1.0_f64);
        nurbs::bezier::bezier_pieces_to_nurbs(&[piece])
    }

    /// A simple X-only travel segment: X moves 0→10 mm, Y and Z are stationary.
    fn seg_x_move() -> ShapedSegment {
        ShapedSegment {
            axes: [
                linear_axis(0.0, 10.0),
                linear_axis(0.0, 0.0),
                linear_axis(0.0, 0.0),
            ],
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        }
    }

    #[test]
    fn cartesian_x_axis_yields_pieces_with_projected_start_time() {
        let cfg = vec![McuAxisConfig {
            mcu_id: 7,
            axes: vec![AXIS_X, AXIS_Y, 2],
            kinematics: 1, // non-zero → Cartesian (not CoreXY)
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        }];

        let msgs = enqueue_segment(
            &seg_x_move(),
            &cfg,
            100.0,
            true,
            |_mcu, hs| (hs * 1_000.0) as u64,
        );

        // X axis must be present.
        let x = msgs
            .iter()
            .find(|m| m.key == AxisKey { mcu_id: 7, axis: 0 })
            .expect("X axis EnqueueMsg must be present");

        assert!(!x.pieces.is_empty(), "X must have at least one piece");
        assert_eq!(
            x.pieces[0].start_time, 100_000,
            "start_time = (t0=100) * 1000 = 100_000"
        );
        assert!(
            x.pieces.iter().all(|p| p.duration > 0.0),
            "all piece durations must be positive"
        );

        // Y (axis 1) and Z (axis 2) must also be emitted (stationary pieces).
        assert!(
            msgs.iter().any(|m| m.key == AxisKey { mcu_id: 7, axis: 1 }),
            "Y axis must be emitted"
        );
        assert!(
            msgs.iter().any(|m| m.key == AxisKey { mcu_id: 7, axis: 2 }),
            "Z axis must be emitted"
        );
    }

    #[test]
    fn corexy_x_slot_is_x_plus_y() {
        let cfg = vec![McuAxisConfig {
            mcu_id: 1,
            axes: vec![AXIS_X, AXIS_Y],
            kinematics: KINEMATICS_COREXY,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        }];

        // X moves 0→10, Y moves 0→4.
        // Motor-A = X+Y ends at 14; motor-B = X-Y ends at 6.
        let seg = ShapedSegment {
            axes: [
                linear_axis(0.0, 10.0),
                linear_axis(0.0, 4.0),
                linear_axis(0.0, 0.0),
            ],
            e_mode: EMode::Travel,
            extrusion_per_xy_mm: 0.0,
            e_independent: None,
            t_start: 0.0,
            t_end: 1.0,
        };

        let msgs = enqueue_segment(&seg, &cfg, 0.0, true, |_mcu, hs| (hs * 1_000.0) as u64);

        // Motor-A is in the AXIS_X slot (axis 0).
        let a = msgs
            .iter()
            .find(|m| m.key == AxisKey { mcu_id: 1, axis: 0 })
            .expect("motor-A (AXIS_X slot) must be present");

        // The last Bernstein coefficient of the last piece equals the curve's
        // endpoint value. Motor-A endpoint = 10 + 4 = 14.
        let last_coeff = a.pieces.last().unwrap().coeffs[3];
        assert!(
            (last_coeff - 14.0_f32).abs() < 1e-3,
            "motor-A endpoint coefficient expected ≈14, got {last_coeff}"
        );

        // Motor-B is in the AXIS_Y slot (axis 1); endpoint = 10 - 4 = 6.
        let b = msgs
            .iter()
            .find(|m| m.key == AxisKey { mcu_id: 1, axis: 1 })
            .expect("motor-B (AXIS_Y slot) must be present");

        let b_last = b.pieces.last().unwrap().coeffs[3];
        assert!(
            (b_last - 6.0_f32).abs() < 1e-3,
            "motor-B endpoint coefficient expected ≈6, got {b_last}"
        );
    }
}
