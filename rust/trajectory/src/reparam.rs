use nurbs::bezier::BezierPiece;

/// Velocity threshold below which a grid interval is treated as near-zero
/// (constant-position). Both endpoints must be below this threshold.
const NEAR_ZERO_V: f64 = 0.01;

#[derive(Debug, Clone)]
pub struct SOfTPieces {
    pub pieces: Vec<BezierPiece<f64>>,
    pub near_zero: Vec<bool>,
    pub t_start: f64,
    pub t_end: f64,
    #[allow(dead_code)]
    pub total_duration: f64,
}

pub fn build_s_of_t_pieces(profile: &temporal::TopProfile, t_global_offset: f64) -> SOfTPieces {
    let n = profile.samples.len();
    assert!(n >= 2, "TopProfile must have at least 2 samples");

    let mut pieces = Vec::with_capacity(n - 1);
    let mut near_zero = Vec::with_capacity(n - 1);
    let mut t_cursor = t_global_offset;

    for k in 0..n - 1 {
        let s_k = profile.samples[k].s;
        let s_k1 = profile.samples[k + 1].s;
        let v_k = profile.samples[k].v;
        let v_k1 = profile.samples[k + 1].v;
        let b_k = profile.samples[k].b;
        let b_k1 = profile.samples[k + 1].b;

        let ds = s_k1 - s_k;

        let is_near_zero = v_k < NEAR_ZERO_V && v_k1 < NEAR_ZERO_V;

        if is_near_zero {
            let dt = ds / NEAR_ZERO_V;
            let t_start = t_cursor;
            let t_end = t_cursor + dt;

            pieces.push(BezierPiece {
                u_start: t_start,
                u_end: t_end,
                coeffs: vec![s_k, 0.0, 0.0],
            });
            near_zero.push(true);
            t_cursor = t_end;
        } else {
            let v_sum = v_k + v_k1;
            let dt = if v_sum > 1e-12 {
                2.0 * ds / v_sum
            } else {
                ds / NEAR_ZERO_V
            };

            let a_k = if ds.abs() > 1e-15 {
                (b_k1 - b_k) / (2.0 * ds)
            } else {
                0.0
            };

            let t_start = t_cursor;
            let t_end = t_cursor + dt;

            pieces.push(BezierPiece {
                u_start: t_start,
                u_end: t_end,
                coeffs: vec![s_k, v_k, a_k / 2.0],
            });
            near_zero.push(false);
            t_cursor = t_end;
        }
    }

    let t_start = t_global_offset;
    let t_end = t_cursor;
    SOfTPieces {
        pieces,
        near_zero,
        t_start,
        t_end,
        total_duration: t_end - t_start,
    }
}

pub fn compose_segment(
    curve: &nurbs::VectorNurbs<f64, 3>,
    table: &nurbs::ArcLengthTableRef<'_, f64>,
    s_pieces: &SOfTPieces,
    fit_tolerance: f64,
) -> Result<Vec<[BezierPiece<f64>; 3]>, crate::ShapeError> {
    let mut result = Vec::with_capacity(s_pieces.pieces.len());

    for (k, s_piece) in s_pieces.pieces.iter().enumerate() {
        if s_pieces.near_zero[k] {
            let s_k = s_piece.coeffs[0];
            let u_k = nurbs::arc_length::param_from_arc_length(table, s_k);
            let pos = nurbs::eval::vector_eval(curve, u_k);

            let axes: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| BezierPiece {
                u_start: s_piece.u_start,
                u_end: s_piece.u_end,
                coeffs: vec![pos[axis]],
            });
            result.push(axes);
        } else {
            let s_lo = s_piece.evaluate(s_piece.u_start);
            let s_hi = s_piece.evaluate(s_piece.u_end);
            let s_hi_clamped = s_hi.min(table.s_max());
            let s_lo_safe = s_lo.max(0.0);

            if s_hi_clamped - s_lo_safe < 1e-15 {
                // Degenerate: arc-length span is essentially zero. Emit constant.
                let u_k = nurbs::arc_length::param_from_arc_length(table, s_lo_safe);
                let pos = nurbs::eval::vector_eval(curve, u_k);
                let axes: [BezierPiece<f64>; 3] = std::array::from_fn(|axis| BezierPiece {
                    u_start: s_piece.u_start,
                    u_end: s_piece.u_end,
                    coeffs: vec![pos[axis]],
                });
                result.push(axes);
                continue;
            }

            let x_of_s: [BezierPiece<f64>; 3] = nurbs::algebra::fit_x_to_arc_length_piece::<3>(
                curve,
                table,
                s_lo_safe,
                s_hi_clamped,
                3, // target_degree
                5, // max_degree
                fit_tolerance,
            )
            .map_err(|detail| crate::ShapeError::FitFailure { index: k, detail })?;

            let s_piece_adjusted =
                if (s_lo_safe - s_lo).abs() > 1e-15 || (s_hi_clamped - s_hi).abs() > 1e-15 {
                    let mut adj = s_piece.clone();
                    adj.coeffs[0] = s_lo_safe;
                    let dt = adj.u_end - adj.u_start;
                    if dt > 1e-15 && adj.coeffs.len() >= 3 {
                        let v_k = adj.coeffs[1];
                        adj.coeffs[2] = (s_hi_clamped - s_lo_safe - v_k * dt) / (dt * dt);
                    }
                    adj
                } else {
                    s_piece.clone()
                };

            let outer_refs: [&BezierPiece<f64>; 3] = [&x_of_s[0], &x_of_s[1], &x_of_s[2]];

            let composed =
                nurbs::algebra::compose_vector_piece::<3>(&outer_refs, &s_piece_adjusted)
                    .map_err(|detail| crate::ShapeError::Algebra { index: k, detail })?;

            result.push(composed);
        }
    }

    Ok(result)
}

#[cfg(test)]
mod tests;
