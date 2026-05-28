// Terminal decel-to-zero ramp localization for the streaming planner.
//
// `find_decel_start_time` walks the velocity profile backward from the path
// terminus and returns the time at which the trailing decel ramp begins.
// Used by `ShaperState::append_and_replan` to compute the planner-state
// cursor `t_decel_start`, which in turn gates dispatch via the
// `t_decel_start − max_h` boundary in `emit_committed`.

use crate::fit::FittedSegment;

/// Find the START of the path's terminal decel-to-zero ramp by walking
/// backward from the path's terminus through dense `||v_xy(t)||` samples.
///
/// **Why this and not "time of global path-speed maximum":** the dispatch
/// boundary is `t_decel_start - max_h`. If we naively reported "time of peak
/// path-speed" we'd report end-of-accel / start-of-cruise on any move with
/// a real cruise plateau, holding back the entire cruise + decel region
/// from dispatch. Throughput suffers measurably on long-cruise jogs. Spec
/// §3.2 explicitly defines `t_decel_start` as "the start of the terminal
/// decel ramp" — we deliver that here.
///
/// **Algorithm.** Sample `||v_xy||` densely along every X/Y `BezierPiece`
/// in time order (32 samples / piece — same density as the legacy peak
/// finder, well below any post-shape feature size). Walk the sample list
/// backward from the terminus while the forward-direction velocity is
/// monotonically decreasing — i.e., while `v_earlier > v_later + epsilon`
/// at each step. When the strict-decrease condition fails (cruise plateau
/// or accel ramp reached), stop. The current sample time is the start of
/// the trailing decel ramp.
///
/// **Epsilon.** A small absolute tolerance (`1e-6` mm/s squared, in the
/// `v_sq` domain we compare on) absorbs floating-point noise on a true
/// constant-v cruise. TOPP-RA's per-segment velocity caps can produce tiny
/// numerical wiggles on a nominally-constant plateau; these are well
/// below the threshold and correctly read as "still on the plateau."
///
/// **Edge cases.**
/// - Pure accel-decel triangle (no cruise plateau): the walk stops at
///   the peak (where `v_earlier` is just-below-peak and `v_later` is
///   the peak, so `v_earlier > v_later + epsilon` is false). The peak
///   time IS the start of decel under that trajectory shape, so this
///   degenerates to the legacy "argmax velocity" answer — correct, and
///   identical to the previous implementation on short / no-cruise
///   moves where the bug had no observable effect.
/// - All-zero / fewer-than-2 samples (degenerate plan): fall back to
///   `fitted[0].t_start`.
/// - Multiple `FittedSegment`s chain naturally — we flatten all pieces
///   in time order before walking.
pub(super) fn find_decel_start_time(fitted: &[FittedSegment]) -> f64 {
    const SAMPLES_PER_PIECE: usize = 32;
    /// Absolute tolerance on `v_sq` (mm²/s²) below which we treat two samples
    /// as "equal" for the monotonic-decrease check. Conservatively small —
    /// well below any post-shape feature on production trajectories.
    const V_SQ_EPSILON: f64 = 1e-6;

    // Walk every piece in time order, accumulating (t, v_sq) samples.
    // We sample v_sq rather than v to avoid a `sqrt` per sample; the
    // monotonic-decrease check is sign-preserving under `sqrt` so working
    // in v_sq is equivalent.
    let mut samples: Vec<(f64, f64)> = Vec::new();
    for f in fitted {
        let x_pieces = nurbs::bezier::extract_bezier_pieces(&f.axes[0]);
        let y_pieces = nurbs::bezier::extract_bezier_pieces(&f.axes[1]);
        for (xp, yp) in x_pieces.iter().zip(y_pieces.iter()) {
            // X and Y pieces share the same time-domain partition (they
            // came out of the same C1-Hermite refit). Sample along X's
            // domain and combine the per-axis velocities for `||v||²`.
            let dx = xp.differentiate();
            let dy = yp.differentiate();
            let u0 = xp.u_start;
            let u1 = xp.u_end;
            // To avoid duplicating boundary samples across adjacent pieces
            // (which would inflate `samples.len()` without changing the
            // walk's outcome), only emit the right endpoint here; the next
            // piece's iteration will emit its left endpoint as its own
            // left endpoint. We still emit the very-first piece's left
            // endpoint by initializing the loop from `s = 0` on the first
            // piece only.
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

    // Degenerate cases — fall back to the path start.
    if samples.len() < 2 {
        return fitted[0].t_start;
    }

    // Walk backward. At each step compare v_sq[earlier] vs v_sq[later]:
    // if forward-decel monotonically decreasing (`v_sq[earlier] >
    // v_sq[later] + eps`), continue back; otherwise stop. The sample
    // index where we stopped is the start of the trailing decel ramp.
    let mut i = samples.len() - 1;
    while i > 0 {
        let v_later = samples[i].1;
        let v_earlier = samples[i - 1].1;
        if v_earlier > v_later + V_SQ_EPSILON {
            // Forward decel — `i-1 → i` showed velocity dropping.
            // Step further back; the decel ramp continues.
            i -= 1;
        } else {
            // Cruise plateau or accel ramp reached — `i-1 → i` did NOT
            // strictly decel. Stop. Sample `i` is the start of the
            // terminal decel ramp.
            break;
        }
    }
    samples[i].0
}
