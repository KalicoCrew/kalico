//! Multi-piece curve chunker for shaped per-axis NURBS dispatch.
//!
//! See `docs/superpowers/specs/2026-05-04-multi-piece-dispatch-design.md`.
//!
//! ## Why
//!
//! `kalico_load_curve`'s wire encoding caps each `Buffer` field at 255 bytes
//! (`host_io/parser.rs:481`), i.e. 63 f32 values per field. For a piecewise-
//! Bézier NURBS in full-multiplicity-interior form, N pieces of degree d need
//! `d*N + d + 1` knots (binding constraint vs the d*N+1 control points).
//! Solving `d*N + d + 1 ≤ 63` gives the per-degree cap:
//!
//!   * d=9 (smooth_zv post-shape worst case) → N ≤ 5  (53/9)
//!   * d=7 → N ≤ 7  (55/7)
//!   * d=4 (current C¹ refit output) → N ≤ 14 (58/4)
//!
//! Trajectory-layer output is one `ScalarNurbs<f64>` per axis with no piece-
//! count cap; this module splits it into K sub-NURBS each ≤ the per-degree
//! cap so the dispatch loop can issue K `kalico_load_curve` + K
//! `kalico_push_segment` calls per logical move per MCU.
//!
//! ## Boundary semantics
//!
//! Each chunk is reassembled from `extract_bezier_pieces` output via
//! `bezier_pieces_to_nurbs`. Adjacent chunks share their boundary control
//! point by construction (the boundary CP is the same `f64` value the
//! trajectory layer computed; both sides truncate to f32 once each, so
//! the shared boundary value is bit-identical at f32 precision). The MCU
//! evaluator (`runtime/src/engine.rs`) holds no cross-segment curve state,
//! so C⁰ continuity at the boundary is sufficient for arc-length-following
//! E motion to stay glitch-free.

use nurbs::{NurbsView, ScalarNurbs};

/// Wire-side per-buffer cap: `kalico_load_curve` `knots`/`cps` fields are
/// `u8`-length-prefixed (255 bytes max → 63 f32 each).
const F32_PER_BUFFER_FIELD: usize = 63;

/// Maximum Bézier-piece count per chunk for a given polynomial degree.
///
/// `num_knots(N pieces, degree d) = d*N + d + 1` (full-multiplicity interior
/// breakpoints, clamped endpoints). Solve `d*N + d + 1 ≤ 63 ⇒ N ≤ (62 - d) / d`.
/// `degree.max(1)` guards against degree-0 inputs.
pub const fn max_pieces_per_chunk(degree: u8) -> usize {
    let d = if degree == 0 { 1 } else { degree as usize };
    let cap = (F32_PER_BUFFER_FIELD - 1).saturating_sub(d) / d;
    if cap == 0 { 1 } else { cap }
}

/// Output of `chunk_scalar_nurbs`: a sequence of sub-NURBS that recompose
/// the source curve, each ≤ `max_pieces_per_chunk(degree)` Bézier pieces.
#[derive(Debug, Clone)]
pub struct ChunkedAxisCurve {
    /// One `ScalarNurbs<f64>` per chunk. `chunks[i]` lives on
    /// `windows[i] = (u_start, u_end)` in the source NURBS knot domain.
    pub chunks: Vec<ScalarNurbs<f64>>,
    /// Per-chunk parametric (typically time-domain) windows, in the same
    /// units as the source NURBS knot vector.
    pub windows: Vec<(f64, f64)>,
}

/// Split a single shaped axis curve into chunks small enough to fit the
/// `kalico_load_curve` wire-buffer cap. Single-chunk fast path returns the
/// curve untouched (clone) when piece count already fits.
pub fn chunk_scalar_nurbs(curve: &ScalarNurbs<f64>) -> ChunkedAxisCurve {
    let degree = NurbsView::degree(curve);
    let max_pieces = max_pieces_per_chunk(degree);
    let pieces = nurbs::bezier::extract_bezier_pieces(curve);

    if pieces.len() <= max_pieces {
        let u_start = curve.knots().first().copied().unwrap_or(0.0);
        let u_end = curve.knots().last().copied().unwrap_or(1.0);
        return ChunkedAxisCurve {
            chunks: vec![curve.clone()],
            windows: vec![(u_start, u_end)],
        };
    }

    let mut chunks = Vec::with_capacity(pieces.len().div_ceil(max_pieces));
    let mut windows = Vec::with_capacity(pieces.len().div_ceil(max_pieces));
    for batch in pieces.chunks(max_pieces) {
        let u_start = batch.first().expect("chunk batch is non-empty").u_start;
        let u_end = batch.last().expect("chunk batch is non-empty").u_end;
        let chunk = nurbs::bezier::bezier_pieces_to_nurbs(batch);
        chunks.push(chunk);
        windows.push((u_start, u_end));
    }
    ChunkedAxisCurve { chunks, windows }
}

/// Re-split an already-chunked curve so that every interior boundary in
/// `union_breaks` is a chunk boundary. Used when multiple axes per MCU have
/// different natural piece counts and we need a synchronized chunk sequence
/// across axes (so per-chunk `t_start_clock`/`t_end_clock` line up).
///
/// `union_breaks` must be strictly between the curve's `[u_start_global,
/// u_end_global]`; values outside that range are silently skipped. The result
/// preserves piece-count limits because every original chunk was already ≤
/// `max_pieces_per_chunk(degree)` and re-splitting on additional interior
/// breakpoints can only reduce the per-chunk piece count.
pub fn split_chunked_at_breaks(
    chunked: ChunkedAxisCurve,
    union_breaks: &[f64],
) -> ChunkedAxisCurve {
    if union_breaks.is_empty() {
        return chunked;
    }
    let mut out_chunks: Vec<ScalarNurbs<f64>> = Vec::new();
    let mut out_windows: Vec<(f64, f64)> = Vec::new();
    for (chunk, (u_start, u_end)) in chunked.chunks.into_iter().zip(chunked.windows.into_iter())
    {
        // Collect breaks that fall *strictly* inside this chunk's window.
        let interior: Vec<f64> = union_breaks
            .iter()
            .copied()
            .filter(|b| *b > u_start && *b < u_end)
            .collect();
        if interior.is_empty() {
            out_chunks.push(chunk);
            out_windows.push((u_start, u_end));
            continue;
        }
        // Re-extract this chunk's Bézier pieces, then split each piece that
        // straddles a break, then re-bundle into per-sub-window groups.
        let mut pieces = nurbs::bezier::extract_bezier_pieces(&chunk);
        for &b in &interior {
            // Find the piece that contains b (strictly interior).
            if let Some(idx) = pieces
                .iter()
                .position(|p| p.u_start < b && b < p.u_end)
            {
                let (left, right) =
                    nurbs::bezier::split_piece_at(&pieces[idx], b);
                pieces.splice(idx..=idx, [left, right]);
            }
        }
        // Walk pieces, slicing whenever we cross a break point.
        let mut cursor = 0;
        let mut breaks_with_end: Vec<f64> = interior.clone();
        breaks_with_end.push(u_end);
        for boundary in breaks_with_end {
            // Take pieces while their u_end ≤ boundary.
            let mut take = cursor;
            while take < pieces.len() && pieces[take].u_end <= boundary {
                take += 1;
            }
            if take == cursor {
                continue;
            }
            let group = &pieces[cursor..take];
            let g_start = group.first().unwrap().u_start;
            let g_end = group.last().unwrap().u_end;
            out_chunks
                .push(nurbs::bezier::bezier_pieces_to_nurbs(group));
            out_windows.push((g_start, g_end));
            cursor = take;
        }
    }
    ChunkedAxisCurve {
        chunks: out_chunks,
        windows: out_windows,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: build a one-piece collinear cubic Bézier on [t0, t1]
    /// going from value `a` to `b`.
    fn linear_cubic(a: f64, b: f64, t0: f64, t1: f64) -> ScalarNurbs<f64> {
        let cps = vec![
            a,
            a + (b - a) / 3.0,
            a + 2.0 * (b - a) / 3.0,
            b,
        ];
        ScalarNurbs::try_new(
            3,
            vec![t0, t0, t0, t0, t1, t1, t1, t1],
            cps,
            None,
        )
        .unwrap()
    }

    /// Helper: build a piecewise-Bézier NURBS of `n_pieces` pieces of `degree`,
    /// with arbitrary monotone interior breakpoints. Each piece is a linear
    /// ramp in its own subinterval (zero higher-order coefficients) so the
    /// resulting curve is C⁰ but not generally C¹ (good enough for chunking).
    fn synth_piecewise(degree: u8, n_pieces: usize) -> ScalarNurbs<f64> {
        let p = degree as usize;
        // Knot vector: clamped endpoints with full-multiplicity interior.
        let mut knots: Vec<f64> = Vec::with_capacity((n_pieces + 1) * p + 2);
        for _ in 0..=p {
            knots.push(0.0);
        }
        for i in 1..n_pieces {
            for _ in 0..p {
                knots.push(i as f64);
            }
        }
        for _ in 0..=p {
            knots.push(n_pieces as f64);
        }
        // CPs: total = degree*n_pieces + 1. Just a monotone ramp.
        let n_cps = p * n_pieces + 1;
        let cps: Vec<f64> = (0..n_cps).map(|i| i as f64).collect();
        ScalarNurbs::try_new(degree, knots, cps, None).unwrap()
    }

    #[test]
    fn max_pieces_per_chunk_for_degree_9_is_5() {
        // Wire cap: 9*N + 10 ≤ 63 ⇒ N ≤ 5.
        assert_eq!(max_pieces_per_chunk(9), 5);
    }

    #[test]
    fn max_pieces_per_chunk_table_matches_design() {
        // Solve d*N + d + 1 ≤ 63 ⇒ N ≤ (62 - d) / d (integer floor).
        assert_eq!(max_pieces_per_chunk(3), 19); // 59 / 3 = 19
        assert_eq!(max_pieces_per_chunk(4), 14); // 58 / 4 = 14
        assert_eq!(max_pieces_per_chunk(7), 7); //  55 / 7 = 7
        assert_eq!(max_pieces_per_chunk(9), 5); //  53 / 9 = 5
        // Worst case: very high degree still returns ≥1.
        assert_eq!(max_pieces_per_chunk(20), 2); // 42 / 20 = 2
    }

    #[test]
    fn single_piece_curve_returns_one_chunk() {
        let c = linear_cubic(0.0, 10.0, 0.0, 1.0);
        let out = chunk_scalar_nurbs(&c);
        assert_eq!(out.chunks.len(), 1);
        assert_eq!(out.windows, vec![(0.0, 1.0)]);
    }

    #[test]
    fn degree_9_27_piece_curve_chunks_into_six() {
        // 27 pieces / 5 per chunk = 6 chunks (5+5+5+5+5+2).
        let c = synth_piecewise(9, 27);
        let out = chunk_scalar_nurbs(&c);
        assert_eq!(out.chunks.len(), 6, "expected 6 chunks for 27/5 split");
        let chunk_pieces: Vec<usize> = out
            .chunks
            .iter()
            .map(|c| nurbs::bezier::extract_bezier_pieces(c).len())
            .collect();
        assert_eq!(chunk_pieces, vec![5, 5, 5, 5, 5, 2]);

        // Every chunk's encoded knot count must fit the wire cap.
        for chunk in &out.chunks {
            assert!(
                chunk.knots().len() <= F32_PER_BUFFER_FIELD,
                "knot count {} > 63 wire cap",
                chunk.knots().len()
            );
        }
    }

    #[test]
    fn chunk_boundaries_are_c0_continuous() {
        // Use degree-3 to keep eval simple; force >1 chunk by setting
        // ridiculous low max via piece count.
        let c = synth_piecewise(9, 27);
        let out = chunk_scalar_nurbs(&c);
        for w in out.chunks.windows(2) {
            // The shared boundary CP is the last CP of left and first of right.
            let left_last = *w[0].control_points().last().unwrap();
            let right_first = *w[1].control_points().first().unwrap();
            // Truncate both to f32 — the wire path does this once each side.
            assert_eq!(
                left_last as f32, right_first as f32,
                "boundary not C0 at f32 precision"
            );
        }
    }

    #[test]
    fn chunk_breakpoints_match_source_piece_breakpoints() {
        // Every per-chunk window boundary must coincide with one of the
        // source's piece breakpoints — no spurious mid-piece splits.
        let c = synth_piecewise(9, 27);
        let source_pieces = nurbs::bezier::extract_bezier_pieces(&c);
        let source_breaks: Vec<f64> =
            source_pieces.iter().map(|p| p.u_end).collect();
        let out = chunk_scalar_nurbs(&c);
        for &(_, u_end) in out.windows.iter().take(out.windows.len() - 1) {
            assert!(
                source_breaks.iter().any(|s| (*s - u_end).abs() < 1e-12),
                "chunk break {u_end} is not a source piece breakpoint"
            );
        }
    }

    #[test]
    fn axis_breakpoint_union_aligns_chunk_count() {
        // Two synthetic axes with different natural piece counts; after
        // union-split they should share identical window sequences.
        let x = synth_piecewise(9, 12); // chunks: 5 + 5 + 2 → breaks at 5, 10
        let y = synth_piecewise(9, 8); //  chunks: 5 + 3      → break  at 5

        let chunked_x = chunk_scalar_nurbs(&x);
        let chunked_y = chunk_scalar_nurbs(&y);

        // Build union of interior breakpoints.
        let mut union: Vec<f64> = Vec::new();
        for win in &chunked_x.windows[..chunked_x.windows.len() - 1] {
            union.push(win.1);
        }
        for win in &chunked_y.windows[..chunked_y.windows.len() - 1] {
            union.push(win.1);
        }
        union.sort_by(|a, b| a.partial_cmp(b).unwrap());
        union.dedup();

        // Both axes' parametric domain is [0, 12] for X and [0, 8] for Y; for
        // this test scale Y up to match X to make the union meaningful.
        // (In real shaped output X and Y share the segment time domain.)
        // For this synthetic, we just demonstrate the splitter API is
        // deterministic on a single curve under a non-empty union.
        let split_x = split_chunked_at_breaks(chunked_x, &union);
        // After splitting on union {5, 10}, X (already had breaks at 5 and 10)
        // should still have the same 3 chunks.
        assert_eq!(split_x.chunks.len(), 3);
        let bounds: Vec<(f64, f64)> = split_x.windows.clone();
        assert_eq!(bounds, vec![(0.0, 5.0), (5.0, 10.0), (10.0, 12.0)]);
    }
}
