// Stage 3a: variable-width neighbor padding + boundary extension.
//
// For each segment in a batch, collects neighbor fitted pieces so that
// convolution with the shaper kernel produces correct values near segment
// boundaries. Constant-position pieces fill E-gap intervals and batch-edge
// extensions.

use crate::fit::FittedSegment;
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};
use nurbs::ScalarNurbs;

/// The fitted data for all segments in a batch, plus E-gap halo context.
/// Used by the beta-medium loop (Stage 5) to drive per-segment padding.
#[allow(dead_code)]
pub struct BatchFittedData {
    /// Fitted segments from Stage 2, in batch order (XY motion only — E gaps
    /// are represented as `EHalo`s, not `FittedSegment`s).
    pub segments: Vec<FittedSegment>,
    /// Constant-XYZ pieces covering E-gap intervals between runs.
    pub e_halos: Vec<EHalo>,
}

/// A constant-position halo piece covering an E-gap interval.
#[derive(Debug, Clone)]
pub struct EHalo {
    /// XYZ hold position during the E-only move.
    pub xyz_position: [f64; 3],
    /// Start time of the E gap (batch-global seconds).
    pub t_start: f64,
    /// End time of the E gap (batch-global seconds).
    pub t_end: f64,
}

/// Pad segment `seg_idx` with neighbor data for a single axis, producing a
/// contiguous `ScalarNurbs<f64>` that extends at least `t_sm_half` beyond the
/// segment's time domain on each side.
///
/// Algorithm:
/// 1. Start with the segment's own fitted pieces for this axis.
/// 2. Scan backward (left padding): accumulate neighbor segments and E-gap halo
///    pieces until at least `t_sm_half` of extra time is covered. At batch
///    start, extend with a constant-position piece.
/// 3. Scan forward (right padding): same logic. At batch end, extend with a
///    constant-position piece.
/// 4. Concatenate left-pad + segment + right-pad into one `ScalarNurbs`.
pub fn pad_segment_axis(
    seg_idx: usize,
    axis: usize,
    fitted: &[FittedSegment],
    e_halos: &[EHalo],
    t_sm_half: f64,
    batch_t_start: f64,
    batch_t_end: f64,
) -> ScalarNurbs<f64> {
    pad_segment_axis_with_history(
        seg_idx,
        axis,
        fitted,
        e_halos,
        &[],
        t_sm_half,
        batch_t_start,
        batch_t_end,
    )
}

/// Variant of [`pad_segment_axis`] that consumes a per-axis `history` slice
/// (`BezierPiece`s in the absolute time domain, immediately preceding
/// `batch_t_start`) when the neighbour-segment scan exhausts before the
/// pad target is covered.
///
/// Streaming-shaper Phase-2 split: the streaming planner holds the
/// already-planned, β-converged pieces from prior `submit_move`s in
/// `ShaperState`. When the un-committed tail is replanned and shaped, the
/// left-pad must read from those prior pieces rather than fall back to a
/// constant-extension at the batch start (which would corrupt the convolution
/// at the seam — the original ~1 mm position-step bug v5 exists to fix).
///
/// Empty `history` reproduces [`pad_segment_axis`]'s behaviour byte-for-byte:
/// after the neighbour scan exhausts, fall back to constant-extension at
/// `batch_t_start`.
///
/// `history` is interpreted as the **left side** of the time line: pieces
/// preceding `batch_t_start`, in time order. The scan reads pieces from the
/// tail (largest `u_end` first) until the pad target is covered or the
/// history is exhausted; degree elevation matches the segment's fitted
/// degree, identical to the neighbour-segment branch.
#[allow(clippy::too_many_arguments)] // Mirrors `pad_segment_axis` plus one history slice; splitting hurts call-site readability.
pub fn pad_segment_axis_with_history(
    seg_idx: usize,
    axis: usize,
    fitted: &[FittedSegment],
    e_halos: &[EHalo],
    history: &[BezierPiece<f64>],
    t_sm_half: f64,
    batch_t_start: f64,
    batch_t_end: f64,
) -> ScalarNurbs<f64> {
    let seg = &fitted[seg_idx];
    let seg_pieces = extract_bezier_pieces(&seg.axes[axis]);
    let target_degree = seg_pieces[0].degree();

    // ---- left padding ----
    let mut left_pieces = collect_left_padding(
        seg_idx,
        axis,
        fitted,
        e_halos,
        history,
        t_sm_half,
        batch_t_start,
        target_degree,
    );

    // ---- right padding ----
    let mut right_pieces = collect_right_padding(
        seg_idx,
        axis,
        fitted,
        e_halos,
        t_sm_half,
        batch_t_end,
        target_degree,
    );

    // ---- concatenate ----
    let mut all_pieces = Vec::new();
    all_pieces.append(&mut left_pieces);
    all_pieces.extend(
        seg_pieces
            .into_iter()
            .map(|p| degree_elevate_to(p, target_degree)),
    );
    all_pieces.append(&mut right_pieces);

    bezier_pieces_to_nurbs(&all_pieces)
}

/// Collect left padding pieces, scanning backward from `seg_idx`.
///
/// `history` (if non-empty) supplies real prior planned `BezierPiece`s in
/// the absolute time domain immediately preceding `batch_t_start`. After the
/// neighbour scan exhausts, the history is consumed (tail-first) until the
/// pad target is covered. Only when both neighbours and history are
/// exhausted do we fall back to constant-extension at the batch start.
#[allow(clippy::too_many_arguments)] // Mirrors `collect_right_padding`'s signature; the new `history` slice is the streaming-shaper hook.
fn collect_left_padding(
    seg_idx: usize,
    axis: usize,
    fitted: &[FittedSegment],
    e_halos: &[EHalo],
    history: &[BezierPiece<f64>],
    t_sm_half: f64,
    batch_t_start: f64,
    target_degree: usize,
) -> Vec<BezierPiece<f64>> {
    let seg = &fitted[seg_idx];
    let pad_target = seg.t_start - t_sm_half;
    let mut pieces: Vec<BezierPiece<f64>> = Vec::new();
    let mut cursor = seg.t_start;

    // Scan backward through neighbors.
    if seg_idx > 0 {
        for i in (0..seg_idx).rev() {
            if cursor <= pad_target {
                break;
            }
            // Check for E-gap halos between segment i and segment i+1.
            let next_seg_start = if i + 1 < fitted.len() {
                fitted[i + 1].t_start
            } else {
                cursor
            };
            let neighbor_end = fitted[i].t_end;

            // Insert any E-gap halos that fall between neighbor[i].t_end and next_seg_start.
            let gap_halos = find_halos_in_range(e_halos, neighbor_end, next_seg_start);
            // Process gap halos in reverse time order (we're scanning backward).
            for halo in gap_halos.into_iter().rev() {
                if cursor <= pad_target {
                    break;
                }
                let h_start = halo.t_start.max(pad_target);
                let h_end = halo.t_end.min(cursor);
                if h_end > h_start {
                    pieces.push(constant_piece(
                        halo.xyz_position[axis],
                        h_start,
                        h_end,
                        target_degree,
                    ));
                    cursor = h_start;
                }
            }

            if cursor <= pad_target {
                break;
            }

            // Extract neighbor segment's pieces for this axis.
            let neighbor_pieces = extract_bezier_pieces(&fitted[i].axes[axis]);
            // Add pieces in reverse order, trimming as needed.
            for np in neighbor_pieces.into_iter().rev() {
                if cursor <= pad_target {
                    break;
                }
                let p_start = np.u_start.max(pad_target);
                let p_end = np.u_end.min(cursor);
                if p_end > p_start {
                    let trimmed = trim_piece(&np, p_start, p_end);
                    pieces.push(degree_elevate_to(trimmed, target_degree));
                    cursor = p_start;
                }
            }
        }
    }

    // Check for E-gap halos before the first segment.
    if cursor > pad_target {
        let first_seg_start = if seg_idx > 0 {
            fitted[0].t_start
        } else {
            seg.t_start
        };
        let gap_halos = find_halos_in_range(e_halos, batch_t_start, first_seg_start);
        for halo in gap_halos.into_iter().rev() {
            if cursor <= pad_target {
                break;
            }
            let h_start = halo.t_start.max(pad_target);
            let h_end = halo.t_end.min(cursor);
            if h_end > h_start {
                pieces.push(constant_piece(
                    halo.xyz_position[axis],
                    h_start,
                    h_end,
                    target_degree,
                ));
                cursor = h_start;
            }
        }
    }

    // Streaming-shaper hook: consume history pieces (tail-first) before
    // falling back to constant-extension. Each history piece is treated like
    // a neighbour-segment piece — trimmed to `[pad_target, cursor]` and
    // degree-elevated. The history slice is in time order, so we walk it
    // in reverse.
    if cursor > pad_target {
        for hp in history.iter().rev() {
            if cursor <= pad_target {
                break;
            }
            let p_start = hp.u_start.max(pad_target);
            let p_end = hp.u_end.min(cursor);
            if p_end > p_start {
                let trimmed = trim_piece(hp, p_start, p_end);
                pieces.push(degree_elevate_to(trimmed, target_degree));
                cursor = p_start;
            }
        }
    }

    // If we still need more padding, extend with constant at the batch start.
    if cursor > pad_target {
        let start_val = first_axis_value(seg_idx, axis, fitted);
        let ext_start = pad_target.max(batch_t_start - t_sm_half);
        if cursor > ext_start {
            pieces.push(constant_piece(start_val, ext_start, cursor, target_degree));
        }
    }

    // Reverse: we collected in reverse time order.
    pieces.reverse();
    pieces
}

/// Collect right padding pieces, scanning forward from `seg_idx`.
fn collect_right_padding(
    seg_idx: usize,
    axis: usize,
    fitted: &[FittedSegment],
    e_halos: &[EHalo],
    t_sm_half: f64,
    batch_t_end: f64,
    target_degree: usize,
) -> Vec<BezierPiece<f64>> {
    let seg = &fitted[seg_idx];
    let pad_target = seg.t_end + t_sm_half;
    let mut pieces: Vec<BezierPiece<f64>> = Vec::new();
    let mut cursor = seg.t_end;

    // Scan forward through neighbors.
    for i in (seg_idx + 1)..fitted.len() {
        if cursor >= pad_target {
            break;
        }
        // Check for E-gap halos between the previous segment and segment i.
        let prev_seg_end = if i > 0 { fitted[i - 1].t_end } else { cursor };
        let neighbor_start = fitted[i].t_start;

        let gap_halos = find_halos_in_range(e_halos, prev_seg_end, neighbor_start);
        for halo in &gap_halos {
            if cursor >= pad_target {
                break;
            }
            let h_start = halo.t_start.max(cursor);
            let h_end = halo.t_end.min(pad_target);
            if h_end > h_start {
                pieces.push(constant_piece(
                    halo.xyz_position[axis],
                    h_start,
                    h_end,
                    target_degree,
                ));
                cursor = h_end;
            }
        }

        if cursor >= pad_target {
            break;
        }

        // Extract neighbor segment's pieces for this axis.
        let neighbor_pieces = extract_bezier_pieces(&fitted[i].axes[axis]);
        for np in &neighbor_pieces {
            if cursor >= pad_target {
                break;
            }
            let p_start = np.u_start.max(cursor);
            let p_end = np.u_end.min(pad_target);
            if p_end > p_start {
                let trimmed = trim_piece(np, p_start, p_end);
                pieces.push(degree_elevate_to(trimmed, target_degree));
                cursor = p_end;
            }
        }
    }

    // Check for E-gap halos after the last segment.
    if cursor < pad_target {
        let last_seg_end = fitted.last().map_or(cursor, |s| s.t_end);
        let gap_halos = find_halos_in_range(e_halos, last_seg_end, batch_t_end);
        for halo in &gap_halos {
            if cursor >= pad_target {
                break;
            }
            let h_start = halo.t_start.max(cursor);
            let h_end = halo.t_end.min(pad_target);
            if h_end > h_start {
                pieces.push(constant_piece(
                    halo.xyz_position[axis],
                    h_start,
                    h_end,
                    target_degree,
                ));
                cursor = h_end;
            }
        }
    }

    // If we still need more padding, extend with constant at the batch end.
    if cursor < pad_target {
        let end_val = last_axis_value(seg_idx, axis, fitted);
        let ext_end = pad_target.min(batch_t_end + t_sm_half);
        if ext_end > cursor {
            pieces.push(constant_piece(end_val, cursor, ext_end, target_degree));
        }
    }

    pieces
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Create a constant-value `BezierPiece` at the target degree.
/// In the Pascal-shifted monomial basis, a constant `c` at degree `d` is
/// `coeffs = [c, 0, 0, ..., 0]`.
fn constant_piece(value: f64, t_start: f64, t_end: f64, degree: usize) -> BezierPiece<f64> {
    let mut coeffs = vec![0.0; degree + 1];
    coeffs[0] = value;
    BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs,
    }
}

/// Degree-elevate a `BezierPiece` to `target_degree` by padding with zero
/// coefficients. In the Pascal-shifted monomial basis, adding zero higher-order
/// coefficients preserves the polynomial identity.
fn degree_elevate_to(mut piece: BezierPiece<f64>, target_degree: usize) -> BezierPiece<f64> {
    while piece.degree() < target_degree {
        piece.coeffs.push(0.0);
    }
    piece
}

/// Trim a `BezierPiece` to `[t_lo, t_hi]` via `split_piece_at`.
/// If the piece already matches the requested range, returns a clone.
fn trim_piece(piece: &BezierPiece<f64>, t_lo: f64, t_hi: f64) -> BezierPiece<f64> {
    let mut p = piece.clone();

    // Trim left.
    if t_lo > p.u_start + 1e-15 && t_lo < p.u_end - 1e-15 {
        let (_, right) = nurbs::bezier::split_piece_at(&p, t_lo);
        p = right;
    }

    // Trim right.
    if t_hi < p.u_end - 1e-15 && t_hi > p.u_start + 1e-15 {
        let (left, _) = nurbs::bezier::split_piece_at(&p, t_hi);
        p = left;
    }

    p
}

/// Find E-gap halos whose time interval overlaps `[t_lo, t_hi)`.
fn find_halos_in_range(e_halos: &[EHalo], t_lo: f64, t_hi: f64) -> Vec<&EHalo> {
    e_halos
        .iter()
        .filter(|h| h.t_end > t_lo && h.t_start < t_hi)
        .collect()
}

/// Get the start-of-segment value for the given axis, walking backward to find
/// the first segment's starting position.
fn first_axis_value(seg_idx: usize, axis: usize, fitted: &[FittedSegment]) -> f64 {
    // Walk backward to find the earliest available value.
    for i in (0..=seg_idx).rev() {
        let pieces = extract_bezier_pieces(&fitted[i].axes[axis]);
        if let Some(first) = pieces.first() {
            return first.evaluate(first.u_start);
        }
    }
    0.0
}

/// Get the end-of-segment value for the given axis, walking forward to find
/// the last segment's ending position.
fn last_axis_value(seg_idx: usize, axis: usize, fitted: &[FittedSegment]) -> f64 {
    for i in seg_idx..fitted.len() {
        let pieces = extract_bezier_pieces(&fitted[i].axes[axis]);
        if let Some(last) = pieces.last() {
            return last.evaluate(last.u_end);
        }
    }
    0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use nurbs::bezier::BezierPiece;

    /// Build a simple `FittedSegment` with linear motion on axis 0 (X),
    /// constant on axes 1 and 2.
    fn linear_segment(x_start: f64, x_end: f64, t_start: f64, t_end: f64) -> FittedSegment {
        let dt = t_end - t_start;
        let slope = (x_end - x_start) / dt;
        // X axis: linear in Pascal-shifted basis.
        let x_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![x_start, slope],
        }]);
        // Y and Z: constant at 0.
        let y_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![0.0],
        }]);
        let z_nurbs = bezier_pieces_to_nurbs(&[BezierPiece {
            u_start: t_start,
            u_end: t_end,
            coeffs: vec![0.0],
        }]);
        FittedSegment {
            axes: [x_nurbs, y_nurbs, z_nurbs],
            t_start,
            t_end,
        }
    }

    #[test]
    fn pad_single_segment_extends_with_constants() {
        // Single segment from t=0 to t=1, X goes from 0 to 10.
        let fitted = vec![linear_segment(0.0, 10.0, 0.0, 1.0)];
        let t_sm_half = 0.1;

        let padded = pad_segment_axis(0, 0, &fitted, &[], t_sm_half, 0.0, 1.0);
        let pieces = extract_bezier_pieces(&padded);

        // Should have padding on both sides.
        assert!(
            pieces.len() >= 3,
            "expected at least 3 pieces, got {}",
            pieces.len()
        );

        // First piece should start before t=0.
        assert!(
            pieces[0].u_start < 0.0,
            "first piece should start before 0, starts at {}",
            pieces[0].u_start
        );

        // Last piece should end after t=1.
        assert!(
            pieces.last().unwrap().u_end > 1.0,
            "last piece should end after 1, ends at {}",
            pieces.last().unwrap().u_end
        );

        // Value at t=0 on the left pad should be the start value (0.0).
        let left_val = pieces[0].evaluate(pieces[0].u_start);
        assert!(
            left_val.abs() < 1e-10,
            "left pad should hold 0.0, got {left_val}"
        );

        // Value at the right pad should be the end value (10.0).
        let right_val = pieces
            .last()
            .unwrap()
            .evaluate(pieces.last().unwrap().u_end);
        assert!(
            (right_val - 10.0).abs() < 1e-10,
            "right pad should hold 10.0, got {right_val}"
        );
    }

    #[test]
    fn pad_middle_segment_uses_neighbors() {
        // Three segments:
        // seg 0: t=[0, 1], X=[0, 10]
        // seg 1: t=[1, 2], X=[10, 30]
        // seg 2: t=[2, 3], X=[30, 35]
        let fitted = vec![
            linear_segment(0.0, 10.0, 0.0, 1.0),
            linear_segment(10.0, 30.0, 1.0, 2.0),
            linear_segment(30.0, 35.0, 2.0, 3.0),
        ];
        let t_sm_half = 0.3;

        let padded = pad_segment_axis(1, 0, &fitted, &[], t_sm_half, 0.0, 3.0);
        let pieces = extract_bezier_pieces(&padded);

        // Padded curve should cover [1.0 - 0.3, 2.0 + 0.3] = [0.7, 2.3].
        let first = &pieces[0];
        let last = pieces.last().unwrap();
        assert!(
            (first.u_start - 0.7).abs() < 1e-10,
            "expected start ~0.7, got {}",
            first.u_start
        );
        assert!(
            (last.u_end - 2.3).abs() < 1e-10,
            "expected end ~2.3, got {}",
            last.u_end
        );

        // Value at t=0.7 should come from seg 0: x = 0 + 10*(0.7) = 7.0.
        assert!(
            (first.evaluate(0.7) - 7.0).abs() < 1e-6,
            "expected 7.0 at t=0.7, got {}",
            first.evaluate(0.7)
        );
    }

    #[test]
    fn pad_with_e_halo_gap() {
        // Two segments with an E-gap halo between them.
        // seg 0: t=[0, 1], X=[0, 10]
        // E-gap: t=[1, 1.5], xyz_position=[10, 0, 0]
        // seg 1: t=[1.5, 2.5], X=[10, 20]
        let fitted = vec![
            linear_segment(0.0, 10.0, 0.0, 1.0),
            linear_segment(10.0, 20.0, 1.5, 2.5),
        ];
        let e_halos = vec![EHalo {
            xyz_position: [10.0, 0.0, 0.0],
            t_start: 1.0,
            t_end: 1.5,
        }];
        let t_sm_half = 0.3;

        // Pad segment 1 — should pick up the E-gap halo.
        let padded = pad_segment_axis(1, 0, &fitted, &e_halos, t_sm_half, 0.0, 2.5);
        let pieces = extract_bezier_pieces(&padded);

        // Should start at 1.5 - 0.3 = 1.2, which is inside the E-gap.
        let first = &pieces[0];
        assert!(
            (first.u_start - 1.2).abs() < 1e-10,
            "expected start ~1.2, got {}",
            first.u_start
        );

        // Value at t=1.2 should be the halo value (10.0).
        assert!(
            (first.evaluate(1.2) - 10.0).abs() < 1e-6,
            "expected 10.0 at t=1.2 (halo), got {}",
            first.evaluate(1.2)
        );
    }

    #[test]
    fn padded_pieces_are_contiguous() {
        let fitted = vec![
            linear_segment(0.0, 5.0, 0.0, 0.5),
            linear_segment(5.0, 15.0, 0.5, 1.5),
            linear_segment(15.0, 18.0, 1.5, 2.0),
        ];
        let t_sm_half = 0.2;

        for seg_idx in 0..fitted.len() {
            let padded = pad_segment_axis(seg_idx, 0, &fitted, &[], t_sm_half, 0.0, 2.0);
            let pieces = extract_bezier_pieces(&padded);
            for w in pieces.windows(2) {
                assert!(
                    (w[0].u_end - w[1].u_start).abs() < 1e-12,
                    "non-contiguous pieces in segment {seg_idx}: {} vs {}",
                    w[0].u_end,
                    w[1].u_start
                );
            }
        }
    }
}
