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
    host_now: f64,
    lead_secs: f64,
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

            let pieces = flatten_axis(curve, t0, cfg.mcu_id, axis_idx, host_now, &project);
            if !pieces.is_empty() {
                out.push(EnqueueMsg {
                    key: AxisKey {
                        mcu_id: cfg.mcu_id,
                        axis: axis_idx as u8,
                    },
                    pieces,
                    fresh_stream,
                    lead_secs,
                });
            }
        }
    }

    out
}

fn flatten_axis<P>(
    curve: &ScalarNurbs<f64>,
    t0: f64,
    mcu_id: u32,
    axis_idx: usize,
    host_now: f64,
    project: &P,
) -> Vec<(PieceEntry, f64)>
where
    P: Fn(u32, f64) -> u64,
{
    let bps = nurbs::bezier::extract_bezier_pieces(curve);
    let mut out = Vec::with_capacity(bps.len());

    for (piece_idx, bp) in bps.iter().enumerate() {
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

        let host_secs = t0 + bp.u_start;
        let start_time = project(mcu_id, host_secs);
        let duration = (bp.u_end - bp.u_start) as f32;

        let margin_us = (host_secs - host_now) * 1e6;
        tracing::trace!(
            mcu_id,
            axis = axis_idx,
            piece_idx,
            u_start = bp.u_start,
            margin_us,
            start_ns = start_time,
            "[dispatch-margin]"
        );

        out.push((
            PieceEntry {
                start_time,
                coeffs,
                duration,
                _reserved: 0,
            },
            host_secs,
        ));
    }

    out
}

#[cfg(test)]
mod tests;
