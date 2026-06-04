// Terminal decel-to-zero ramp localization for the streaming planner.
// `t_decel_start` returned here gates dispatch via `t_decel_start - max_h`
// in `emit_committed`.

use crate::fit::FittedSegment;

/// Returns the start time of the path's terminal decel-to-zero ramp (spec §3.2).
///
/// **Why not "time of peak path-speed":** on moves with a real cruise plateau,
/// argmax(v) returns end-of-accel, which holds the entire cruise + decel region
/// back from dispatch and measurably hurts throughput on long-cruise jogs.
///
/// Samples `||v_xy||²` at 32 points/piece and walks backward from the terminus
/// while velocity is strictly decreasing (`v_earlier > v_later + V_SQ_EPSILON`).
/// Stops at the first sample where the strict-decrease condition fails (cruise
/// or accel reached). On a pure triangle profile this degenerates to argmax(v) —
/// correct. Falls back to `fitted[0].t_start` if fewer than 2 samples exist.
pub(super) fn find_decel_start_time(fitted: &[FittedSegment]) -> f64 {
    const SAMPLES_PER_PIECE: usize = 32;
    /// Absolute tolerance on `v_sq` (mm²/s²) below which we treat two samples
    /// as "equal" for the monotonic-decrease check. Conservatively small —
    /// well below any post-shape feature on production trajectories.
    const V_SQ_EPSILON: f64 = 1e-6;

    // Sample v_sq (not v) to avoid a sqrt per sample; monotonic-decrease check
    // is sign-preserving under sqrt so the comparison is equivalent.
    let mut samples: Vec<(f64, f64)> = Vec::new();
    for f in fitted {
        let x_pieces = nurbs::bezier::extract_bezier_pieces(&f.axes[0]);
        let y_pieces = nurbs::bezier::extract_bezier_pieces(&f.axes[1]);
        for (xp, yp) in x_pieces.iter().zip(y_pieces.iter()) {
            let dx = xp.differentiate();
            let dy = yp.differentiate();
            let u0 = xp.u_start;
            let u1 = xp.u_end;
            // Skip the left endpoint on all but the first piece to avoid
            // duplicating boundary samples between adjacent pieces.
            let start_s = usize::from(!samples.is_empty());
            for s in start_s..=SAMPLES_PER_PIECE {
                let t = u0 + (u1 - u0) * (s as f64) / (SAMPLES_PER_PIECE as f64);
                let vx = dx.evaluate(t);
                let vy = dy.evaluate(t);
                let v_sq = vx * vx + vy * vy;
                samples.push((t, v_sq));
            }
        }
    }

    if samples.len() < 2 {
        return fitted[0].t_start;
    }

    let mut i = samples.len() - 1;
    while i > 0 {
        let v_later = samples[i].1;
        let v_earlier = samples[i - 1].1;
        if v_earlier > v_later + V_SQ_EPSILON {
            i -= 1;
        } else {
            break;
        }
    }
    samples[i].0
}
