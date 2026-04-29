//! Path-length-capped subdivision of cubic-Bézier segments. Bounds per-MCU-segment
//! piece count for downstream Layer 3 (T-A) by capping arc length at
//! `max_arc_length_mm` (default 12.5 mm). See spec §5 of the 7-pre design doc.

use crate::{CubicSegment, SplitInfo};
use nurbs::{
    ScalarNurbs, VectorNurbs,
    arc_length::{arc_length_from_param, build_arc_length_table_vector, param_from_arc_length},
    bezier::{BezierPiece, extract_bezier_pieces, split_piece_at},
    eval::{vector_derivative, vector_eval},
};

const EPS_CP_POLYGON: f64 = 3e-6;
const EPS_U: f64 = 1e-9;
const MIN_PARAMETRIC_SPEED_FOR_SPLITTER: f64 = 1e-9;
/// Relative tolerance absorbing arc-length integration round-off so an exact
/// multiple of `max_arc_length_mm` doesn't get over-split by one. The arc-length
/// builder integrates via central-difference + Gauss-Legendre with residual
/// `1e-9 · |estimate|`; this tolerance must be at or above that order.
const EPS_RATIO: f64 = 1e-8;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SplitError {
    NotSinglePieceCubic,
    ArcLengthTableBuildFailed { reason: &'static str },
}

/// Split a cubic-Bézier segment along arc length so that each child segment's
/// arc length is at most `max_arc_length_mm`.
///
/// Passthrough behaviors:
/// - Zero-motion segments (control polygon length and midpoint parametric speed
///   both below numerical floors) are returned unchanged. This covers
///   `EMode::Independent` retraction / prime / filament-change segments whose
///   `xyz` collapses to a point.
/// - Segments whose total arc length is below `max_arc_length_mm` are returned
///   unchanged (`split_info` remains `None`).
pub fn split_segment_to_cap(
    segment: &CubicSegment,
    max_arc_length_mm: f64,
) -> Result<Vec<CubicSegment>, SplitError> {
    debug_assert!(
        max_arc_length_mm > 0.0,
        "max_arc_length_mm must be positive"
    );

    if is_zero_motion(&segment.xyz) {
        return Ok(vec![segment.clone()]);
    }

    // Defense-in-depth (spec §6.1 closing remark): an Independent segment
    // reaching here would indicate misclassification upstream. Pre-Fix-A.1
    // pure-Z+E moves slipped past `classify_e_mode` as `Independent` and the
    // multi-piece path then cloned the full E curve into every child,
    // producing N× over-extrusion. classify_e_mode now rejects pure-Z+E as
    // HelicalExtrusionUnsupported; assert here so any future regression
    // (e.g. the Step-13 compat layer adding a new admit path) trips.
    debug_assert!(
        segment.e_mode != crate::EMode::Independent,
        "Independent segment reached splitter past zero-motion fast-path \
         (cp_polygon_length: {}, mid_speed: {})",
        cp_polygon_length(&segment.xyz),
        midpoint_parametric_speed(&segment.xyz)
    );

    let table = build_arc_length_table_vector(&segment.xyz, 1e-9, 64).map_err(|_| {
        SplitError::ArcLengthTableBuildFailed {
            reason: "build_arc_length_table_vector failed",
        }
    })?;
    let table_ref = table.as_view();
    let total_length = table.s_max();

    // Below-cap passthrough. Accept arc-length-integration round-off (the table
    // builder converges to `1e-9 · |estimate|`) so an exact `length == cap`
    // input passes through instead of producing two children whose `s_hi - s_lo`
    // differ from the cap by a few parts in 1e10.
    if total_length <= max_arc_length_mm * (1.0 + EPS_RATIO) {
        return Ok(vec![segment.clone()]);
    }

    // Compute target arc-lengths and convert to parameters. The same relative
    // tolerance pulls back exact-multiple inputs (e.g. 25 mm at a 12.5 mm cap)
    // from being over-split by one piece.
    let ratio = total_length / max_arc_length_mm;
    // The ratio is positive (total_length > 0, max_arc_length_mm > 0); the
    // `.max(2.0)` floor keeps the cast in the safe range.
    #[allow(clippy::cast_sign_loss)]
    let k_planned = (ratio - EPS_RATIO).ceil().max(2.0) as usize;
    debug_assert!(k_planned >= 2, "expected at least two pieces past the cap");
    let mut u_breaks: Vec<f64> = Vec::with_capacity(k_planned - 1);
    for i in 1..k_planned {
        let target = total_length * (i as f64) / (k_planned as f64);
        u_breaks.push(param_from_arc_length(&table_ref, target));
    }

    // Project xyz onto its three scalar axes and extract the (single) Bézier piece per axis.
    let parent_pieces = extract_bezier_pieces_vector(&segment.xyz);
    debug_assert!(
        parent_pieces.iter().all(|axis_pieces| axis_pieces.len() == 1),
        "single-piece-cubic invariant"
    );

    let mut current_pieces: [BezierPiece<f64>; 3] = [
        parent_pieces[0][0].clone(),
        parent_pieces[1][0].clone(),
        parent_pieces[2][0].clone(),
    ];
    let mut emitted_axes: [Vec<BezierPiece<f64>>; 3] =
        [Vec::new(), Vec::new(), Vec::new()];

    for &u in &u_breaks {
        let u_start = current_pieces[0].u_start;
        let u_end = current_pieces[0].u_end;
        // Skip breakpoints within EPS_U of the carried piece's bounds.
        if u <= u_start + EPS_U || u >= u_end - EPS_U {
            continue;
        }
        for axis in 0..3 {
            let (left, right) = split_piece_at(&current_pieces[axis], u);
            emitted_axes[axis].push(left);
            current_pieces[axis] = right;
        }
    }
    for axis in 0..3 {
        emitted_axes[axis].push(current_pieces[axis].clone());
    }

    let n_emitted = emitted_axes[0].len();
    debug_assert!(
        emitted_axes.iter().all(|v| v.len() == n_emitted),
        "axes must agree on piece count"
    );

    let mut output: Vec<CubicSegment> = Vec::with_capacity(n_emitted);
    for i in 0..n_emitted {
        let xyz = vector_nurbs_from_pieces([
            &emitted_axes[0][i],
            &emitted_axes[1][i],
            &emitted_axes[2][i],
        ]);
        let s_lo = arc_length_from_param(&table_ref, emitted_axes[0][i].u_start);
        let s_hi = arc_length_from_param(&table_ref, emitted_axes[0][i].u_end);
        let split_info = SplitInfo {
            sub_index: i as u32,
            sub_count: n_emitted as u32,
            s_lo_mm: s_lo,
            s_hi_mm: s_hi,
        };
        let child = CubicSegment::try_new(
            xyz,
            segment.e_mode,
            segment.extrusion_per_xy_mm,
            segment.e_independent.clone(),
            segment.feedrate_mm_s,
            segment.source,
            Some(split_info),
        )
        .map_err(|_| SplitError::NotSinglePieceCubic)?;
        output.push(child);
    }
    Ok(output)
}

fn is_zero_motion(xyz: &VectorNurbs<f64, 3>) -> bool {
    cp_polygon_length(xyz) < EPS_CP_POLYGON
        && midpoint_parametric_speed(xyz) < MIN_PARAMETRIC_SPEED_FOR_SPLITTER
}

fn cp_polygon_length(xyz: &VectorNurbs<f64, 3>) -> f64 {
    let cps = xyz.control_points();
    (1..cps.len())
        .map(|i| {
            let dx = cps[i][0] - cps[i - 1][0];
            let dy = cps[i][1] - cps[i - 1][1];
            let dz = cps[i][2] - cps[i - 1][2];
            (dx * dx + dy * dy + dz * dz).sqrt()
        })
        .sum()
}

fn midpoint_parametric_speed(xyz: &VectorNurbs<f64, 3>) -> f64 {
    let deriv = vector_derivative(xyz);
    let d = vector_eval(&deriv, 0.5_f64);
    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
}

fn extract_bezier_pieces_vector(xyz: &VectorNurbs<f64, 3>) -> [Vec<BezierPiece<f64>>; 3] {
    let mut out: [Vec<BezierPiece<f64>>; 3] = [Vec::new(), Vec::new(), Vec::new()];
    for axis in 0..3 {
        let scalar = project_axis_to_scalar(xyz, axis);
        out[axis] = extract_bezier_pieces(&scalar);
    }
    out
}

fn project_axis_to_scalar(xyz: &VectorNurbs<f64, 3>, axis: usize) -> ScalarNurbs<f64> {
    let cps: Vec<f64> = xyz.control_points().iter().map(|cp| cp[axis]).collect();
    ScalarNurbs::try_new(
        xyz.degree(),
        xyz.knots().to_vec(),
        cps,
        xyz.weights().map(<[f64]>::to_vec),
    )
    .expect("projection always valid")
}

fn vector_nurbs_from_pieces(pieces: [&BezierPiece<f64>; 3]) -> VectorNurbs<f64, 3> {
    debug_assert!(pieces.iter().all(|p| p.degree() == 3));
    debug_assert!(pieces.iter().all(|p| {
        (p.u_start - pieces[0].u_start).abs() < 1e-12
            && (p.u_end - pieces[0].u_end).abs() < 1e-12
    }));
    let bern_x = pieces[0].to_bernstein();
    let bern_y = pieces[1].to_bernstein();
    let bern_z = pieces[2].to_bernstein();
    let cps: Vec<[f64; 3]> = (0..4).map(|i| [bern_x[i], bern_y[i], bern_z[i]]).collect();
    VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps,
        None,
    )
    .expect("valid cubic from pieces")
}
