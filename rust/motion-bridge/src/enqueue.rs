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
    max_piece_secs: Option<f64>,
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

            let key = AxisKey {
                mcu_id: cfg.mcu_id,
                axis: axis_idx as u8,
            };

            let pieces = flatten_axis(
                curve,
                t0,
                cfg.mcu_id,
                axis_idx,
                host_now,
                &project,
                max_piece_secs,
            );
            if !pieces.is_empty() {
                out.push(EnqueueMsg {
                    key,
                    pieces,
                    fresh_stream,
                    lead_secs,
                });
            }
        }
    }

    out
}

fn is_constant_piece(coeffs: &[f64; 4]) -> bool {
    coeffs[0].to_bits() == coeffs[1].to_bits()
        && coeffs[1].to_bits() == coeffs[2].to_bits()
        && coeffs[2].to_bits() == coeffs[3].to_bits()
}

fn flatten_axis<P>(
    curve: &ScalarNurbs<f64>,
    t0: f64,
    mcu_id: u32,
    axis_idx: usize,
    host_now: f64,
    project: &P,
    max_piece_secs: Option<f64>,
) -> Vec<(PieceEntry, f64)>
where
    P: Fn(u32, f64) -> u64,
{
    let bps = nurbs::bezier::extract_bezier_pieces(curve);
    // (coeffs, duration, u_start): u_start is bp.u_start in CURVE time — curves
    // are not 0-based (a dispatched stream's segments continue in trajectory
    // time), so emitted host times must be t0 + u_start, never an accumulator
    // restarted at zero. Getting this wrong shifts every piece of every
    // non-first segment into the MCU's past (bench: PieceStartInPast,
    // deficit saturated).
    let mut merged: Vec<([f64; 4], f64, f64)> = Vec::with_capacity(bps.len());

    for bp in bps.iter() {
        let bern = bp.to_bernstein();

        debug_assert_eq!(
            bern.len(),
            4,
            "expected cubic (degree-3) Bernstein coeffs; the pipeline is uniform-cubic per CLAUDE.md, got {}",
            bern.len()
        );

        let n = bern.len().min(4);
        let last_f64 = if n > 0 { bern[n - 1] } else { 0.0 };
        let mut coeffs_f64 = [last_f64; 4];
        for k in 0..n {
            coeffs_f64[k] = bern[k];
        }

        let duration = bp.u_end - bp.u_start;

        if is_constant_piece(&coeffs_f64) {
            if let Some(last) = merged.last_mut() {
                if is_constant_piece(&last.0) && last.0[0].to_bits() == coeffs_f64[0].to_bits() {
                    last.1 += duration;
                    continue;
                }
            }
        }
        merged.push((coeffs_f64, duration, bp.u_start));
    }

    let mut out = Vec::with_capacity(merged.len() * 8);

    for (piece_idx, (coeffs_f64, duration, u_start)) in merged.iter().enumerate() {
        // max_piece_secs is set only for homing (drip) enqueues, where
        // constants must subdivide like movers: a whole-move coalesced
        // constant never retires until the move ends, which both pins the
        // cohort's retirement watchdog at zero and exempts that axis from
        // the drip leash. Outside homing constants stay whole (the
        // coalescing exists to keep follower traffic off the wire).
        let subs: Vec<([f64; 4], f64)> = match max_piece_secs {
            Some(m) => subdivide_bernstein(*coeffs_f64, *duration, m),
            None => vec![(*coeffs_f64, *duration)],
        };

        let mut sub_offset = 0.0_f64;
        for (sub_idx, (sub_coeffs, sub_dur)) in subs.iter().enumerate() {
            let host_secs = t0 + u_start + sub_offset;
            let start_time = project(mcu_id, host_secs);

            let mut coeffs = [0.0_f32; 4];
            for k in 0..4 {
                coeffs[k] = sub_coeffs[k] as f32;
            }
            let duration_f32 = *sub_dur as f32;

            let margin_us = (host_secs - host_now) * 1e6;
            tracing::trace!(
                mcu_id,
                axis = axis_idx,
                piece_idx,
                sub_idx,
                u_start = host_secs - t0,
                margin_us,
                start_ns = start_time,
                "[dispatch-margin]"
            );

            out.push((
                PieceEntry {
                    start_time,
                    coeffs,
                    duration: duration_f32,
                    _reserved: 0,
                },
                host_secs,
            ));

            sub_offset += sub_dur;
        }
    }

    out
}

pub fn subdivide_bernstein(
    coeffs: [f64; 4],
    duration: f64,
    max_piece_secs: f64,
) -> Vec<([f64; 4], f64)> {
    if duration <= max_piece_secs {
        return vec![(coeffs, duration)];
    }
    let c0 = coeffs[0];
    if coeffs.iter().all(|&c| (c - c0).abs() <= 1e-9) {
        return vec![(coeffs, duration)];
    }
    let n = (duration / max_piece_secs).ceil() as usize;
    let sub = duration / n as f64;
    let mut out = Vec::with_capacity(n);
    let mut rest = coeffs;
    for i in 0..n - 1 {
        let u = sub / (duration - i as f64 * sub);
        let (left, right) = de_casteljau_split(rest, u);
        out.push((left, sub));
        rest = right;
    }
    out.push((rest, sub));
    out
}

fn de_casteljau_split(c: [f64; 4], u: f64) -> ([f64; 4], [f64; 4]) {
    let b01 = lerp(c[0], c[1], u);
    let b12 = lerp(c[1], c[2], u);
    let b23 = lerp(c[2], c[3], u);
    let b012 = lerp(b01, b12, u);
    let b123 = lerp(b12, b23, u);
    let b = lerp(b012, b123, u);
    ([c[0], b01, b012, b], [b, b123, b23, c[3]])
}

fn lerp(a: f64, b: f64, u: f64) -> f64 {
    a + (b - a) * u
}

#[cfg(test)]
mod tests;
