use crate::fit::FittedSegment;
use nurbs::bezier::{bezier_pieces_to_nurbs, extract_bezier_pieces, BezierPiece};
use nurbs::ScalarNurbs;

#[allow(dead_code)]
pub struct BatchFittedData {
    pub segments: Vec<FittedSegment>,
    pub e_halos: Vec<EHalo>,
}

#[derive(Debug, Clone)]
pub struct EHalo {
    pub xyz_position: [f64; 3],
    pub t_start: f64,
    pub t_end: f64,
}

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

#[allow(clippy::too_many_arguments)]
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

    let mut right_pieces = collect_right_padding(
        seg_idx,
        axis,
        fitted,
        e_halos,
        t_sm_half,
        batch_t_end,
        target_degree,
    );

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

#[allow(clippy::too_many_arguments)]
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

    if seg_idx > 0 {
        for i in (0..seg_idx).rev() {
            if cursor <= pad_target {
                break;
            }
            let next_seg_start = if i + 1 < fitted.len() {
                fitted[i + 1].t_start
            } else {
                cursor
            };
            let neighbor_end = fitted[i].t_end;
            let gap_halos = find_halos_in_range(e_halos, neighbor_end, next_seg_start);
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

            let neighbor_pieces = extract_bezier_pieces(&fitted[i].axes[axis]);
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

    if cursor > pad_target {
        let start_val = first_axis_value(seg_idx, axis, fitted);
        let ext_start = pad_target.max(batch_t_start - t_sm_half);
        if cursor > ext_start {
            pieces.push(constant_piece(start_val, ext_start, cursor, target_degree));
        }
    }

    pieces.reverse();
    pieces
}

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

    for i in (seg_idx + 1)..fitted.len() {
        if cursor >= pad_target {
            break;
        }
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

    if cursor < pad_target {
        let end_val = last_axis_value(seg_idx, axis, fitted);
        let ext_end = pad_target.min(batch_t_end + t_sm_half);
        if ext_end > cursor {
            pieces.push(constant_piece(end_val, cursor, ext_end, target_degree));
        }
    }

    pieces
}

fn constant_piece(value: f64, t_start: f64, t_end: f64, degree: usize) -> BezierPiece<f64> {
    let mut coeffs = vec![0.0; degree + 1];
    coeffs[0] = value;
    BezierPiece {
        u_start: t_start,
        u_end: t_end,
        coeffs,
    }
}

fn degree_elevate_to(mut piece: BezierPiece<f64>, target_degree: usize) -> BezierPiece<f64> {
    while piece.degree() < target_degree {
        piece.coeffs.push(0.0);
    }
    piece
}

fn trim_piece(piece: &BezierPiece<f64>, t_lo: f64, t_hi: f64) -> BezierPiece<f64> {
    let mut p = piece.clone();

    if t_lo > p.u_start + 1e-15 && t_lo < p.u_end - 1e-15 {
        let (_, right) = nurbs::bezier::split_piece_at(&p, t_lo);
        p = right;
    }

    if t_hi < p.u_end - 1e-15 && t_hi > p.u_start + 1e-15 {
        let (left, _) = nurbs::bezier::split_piece_at(&p, t_hi);
        p = left;
    }

    p
}

fn find_halos_in_range(e_halos: &[EHalo], t_lo: f64, t_hi: f64) -> Vec<&EHalo> {
    e_halos
        .iter()
        .filter(|h| h.t_end > t_lo && h.t_start < t_hi)
        .collect()
}

fn first_axis_value(seg_idx: usize, axis: usize, fitted: &[FittedSegment]) -> f64 {
    for i in (0..=seg_idx).rev() {
        let pieces = extract_bezier_pieces(&fitted[i].axes[axis]);
        if let Some(first) = pieces.first() {
            return first.evaluate(first.u_start);
        }
    }
    0.0
}

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
mod tests;
