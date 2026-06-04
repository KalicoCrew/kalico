use crate::dispatch::{AXIS_X, AXIS_Y, McuAxisConfig, cfg_is_corexy};
use crate::pump::{AxisKey, EnqueueMsg};
use nurbs::ScalarNurbs;
use runtime::piece_ring::PieceEntry;
use trajectory::ShapedSegment;

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
        let corexy = cfg_is_corexy(cfg) && AXIS_X < seg.axes.len() && AXIS_Y < seg.axes.len();

        let motor: Option<(ScalarNurbs<f64>, ScalarNurbs<f64>)> = if corexy {
            let x = &seg.axes[AXIS_X];
            let y = &seg.axes[AXIS_Y];
            let neg_y = nurbs::algebra::scalar_multiply(y, -1.0_f64);
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

fn flatten_axis<P>(curve: &ScalarNurbs<f64>, t0: f64, mcu_id: u32, project: &P) -> Vec<PieceEntry>
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
            coeffs[k] = bern[k] as f32;
        }
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

    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::dispatch::{KINEMATICS_COREXY, McuCaps};
    use geometry::segment::EMode;

    fn linear_axis(p0: f64, p1: f64) -> ScalarNurbs<f64> {
        let d = p1 - p0;
        let bern = [p0, p0 + d / 3.0, p0 + 2.0 * d / 3.0, p1];
        let piece = nurbs::bezier::BezierPiece::from_bernstein(&bern, 0.0_f64, 1.0_f64);
        nurbs::bezier::bezier_pieces_to_nurbs(&[piece])
    }

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
            kinematics: 1,
            caps: McuCaps {
                total_piece_memory: 62 * 1024,
            },
        }];

        let msgs = enqueue_segment(&seg_x_move(), &cfg, 100.0, true, |_mcu, hs| {
            (hs * 1_000.0) as u64
        });

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

        let a = msgs
            .iter()
            .find(|m| m.key == AxisKey { mcu_id: 1, axis: 0 })
            .expect("motor-A (AXIS_X slot) must be present");

        let last_coeff = a.pieces.last().unwrap().coeffs[3];
        assert!(
            (last_coeff - 14.0_f32).abs() < 1e-3,
            "motor-A endpoint coefficient expected ≈14, got {last_coeff}"
        );

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
