//! Segment types — the product of the iterator. Layer 2 reads these.

use nurbs::{ScalarNurbs, VectorNurbs};

#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {
    /// Live-pipeline cubic Bézier segment with E-mode classification. Produced by
    /// `reduce.rs` from G5/G5.1 input.
    Cubic(CubicSegment),
    CornerBlend(CornerBlendSlot),
    Junction(JunctionDeviation),
}

#[derive(Debug, Clone, PartialEq)]
pub struct CornerBlendSlot {
    pub position: [f64; 3],
    pub t_in: [f64; 3],
    pub t_out: [f64; 3],
    pub seg_len_in: f64,
    pub seg_len_out: f64,
    pub tolerance_budget_mm: f64,
    pub default_family: BlendFamily,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

#[derive(Debug, Clone, PartialEq)]
pub struct JunctionDeviation {
    pub position: [f64; 3],
    pub angle_deg: f64,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
}

/// Live-pipeline cubic-Bézier segment. Single-piece cubic Bézier in `xyz` (degree 3,
/// 4 control points, no weights, clamped knot vector). E classification per `EMode`.
#[derive(Debug, Clone, PartialEq)]
pub struct CubicSegment {
    /// XYZ trajectory in u-domain. **Invariant** (enforced by `try_new`): single-piece
    /// cubic Bézier — degree 3, 4 control points, no weights, clamped knot vector.
    pub xyz: VectorNurbs<f64, 3>,
    pub e_mode: EMode,
    /// Valid when `e_mode == CoupledToXy`. Signed: negative for retract-during-XY-motion
    /// / wipe / coast. Zero when `e_mode == Travel`. Unused when `e_mode == Independent`
    /// (use `e_independent` instead).
    pub extrusion_per_xy_mm: f64,
    /// `Some(curve)` iff `e_mode == Independent`; carries the E trajectory for
    /// retraction / prime / filament-change segments.
    pub e_independent: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
    /// `None` on un-split segments; `Some` on splitter output.
    pub split_info: Option<SplitInfo>,
}

impl CubicSegment {
    /// Construct a `CubicSegment`, validating invariants. Returns `Err` on:
    /// - `NotSinglePieceCubic`: xyz is not single-piece cubic (degree != 3,
    ///   != 4 CPs, has weights, or knots are not clamped `[0,0,0,0,1,1,1,1]`).
    /// - `EModeInvariantViolation`: `e_mode` and the corresponding fields disagree.
    pub fn try_new(
        xyz: VectorNurbs<f64, 3>,
        e_mode: EMode,
        extrusion_per_xy_mm: f64,
        e_independent: Option<ScalarNurbs<f64>>,
        feedrate_mm_s: f64,
        source: SourceRange,
        split_info: Option<SplitInfo>,
    ) -> Result<Self, crate::GeometryError> {
        // xyz must be single-piece cubic Bézier.
        if xyz.degree() != 3 {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "degree != 3",
            });
        }
        if xyz.control_points().len() != 4 {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "control_points.len() != 4",
            });
        }
        if xyz.weights().is_some() {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "weights present (must be polynomial)",
            });
        }
        let expected_knots: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        if xyz.knots() != expected_knots.as_slice() {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "knot vector not clamped [0,0,0,0,1,1,1,1]",
            });
        }

        // EMode invariants.
        match e_mode {
            EMode::CoupledToXy => {
                if e_independent.is_some() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "CoupledToXy must have e_independent: None",
                    });
                }
            }
            EMode::Travel => {
                if extrusion_per_xy_mm != 0.0 {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Travel must have extrusion_per_xy_mm == 0.0",
                    });
                }
                if e_independent.is_some() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Travel must have e_independent: None",
                    });
                }
            }
            EMode::Independent => {
                if e_independent.is_none() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Independent must have e_independent: Some(_)",
                    });
                }
                if extrusion_per_xy_mm != 0.0 {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "Independent must have extrusion_per_xy_mm == 0.0",
                    });
                }
            }
        }

        // Defense-in-depth: callers bypassing the lexer (tests, Step-13 output
        // handlers, hand-built segments) must not be able to construct a
        // CubicSegment with non-finite values. Non-finite cps poison
        // xy_arc_length and downstream classification; non-finite feedrate /
        // extrusion ratio poisons TOPP-RA scheduling; non-finite e_independent
        // curve poisons E integration on the MCU.
        for cp in xyz.control_points() {
            for &v in cp {
                if !v.is_finite() {
                    return Err(crate::GeometryError::NotSinglePieceCubic {
                        reason: "control point contains non-finite value",
                    });
                }
            }
        }
        if !feedrate_mm_s.is_finite() {
            return Err(crate::GeometryError::EModeInvariantViolation {
                reason: "feedrate_mm_s must be finite",
            });
        }
        if !extrusion_per_xy_mm.is_finite() {
            return Err(crate::GeometryError::EModeInvariantViolation {
                reason: "extrusion_per_xy_mm must be finite",
            });
        }
        if let Some(curve) = &e_independent {
            for &v in curve.control_points() {
                if !v.is_finite() {
                    return Err(crate::GeometryError::EModeInvariantViolation {
                        reason: "e_independent control point contains non-finite value",
                    });
                }
            }
        }
        if let Some(info) = &split_info {
            if !info.s_lo_mm.is_finite() || !info.s_hi_mm.is_finite() {
                return Err(crate::GeometryError::EModeInvariantViolation {
                    reason: "split_info arc-length range contains non-finite value",
                });
            }
        }

        Ok(Self {
            xyz,
            e_mode,
            extrusion_per_xy_mm,
            e_independent,
            feedrate_mm_s,
            source,
            split_info,
        })
    }
}

/// Subdivide a single-piece cubic Bézier curve at parameter `s ∈ (0, 1)` using
/// the de Casteljau algorithm. Returns `(left, right)` — each a single-piece
/// cubic Bézier covering the input curve's `[0, s]` and `[s, 1]` respectively,
/// re-parameterized so each half's parameter domain is `[0, 1]` (clamped
/// `[0,0,0,0,1,1,1,1]` knot vector).
///
/// **Position continuity invariant.** Evaluating `left` at `u = 1` and `right`
/// at `u = 0` both reproduce `xyz(s)` to within floating-point round-off. The
/// caller is expected to use this property when chaining the right half back
/// into the planner (e.g., the streaming shaper's partial-commit replan,
/// where the toolhead's mid-move position must align with the new plan's
/// starting position).
///
/// # Panics
///
/// Panics if `xyz` is not a valid single-piece cubic Bézier (`CubicSegment`'s
/// `try_new` invariants — degree 3, 4 control points, no weights, clamped
/// knot vector). Panics if `s` is not strictly interior to `(0, 1)`.
#[must_use]
pub fn split_cubic_bezier(
    xyz: &VectorNurbs<f64, 3>,
    s: f64,
) -> (VectorNurbs<f64, 3>, VectorNurbs<f64, 3>) {
    assert_eq!(xyz.degree(), 3, "split_cubic_bezier: degree must be 3");
    let cps = xyz.control_points();
    assert_eq!(cps.len(), 4, "split_cubic_bezier: must have 4 control points");
    assert!(
        xyz.weights().is_none(),
        "split_cubic_bezier: weights must be absent (polynomial Bézier)",
    );
    let expected_knots: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    assert_eq!(
        xyz.knots(),
        expected_knots.as_slice(),
        "split_cubic_bezier: knot vector must be clamped [0,0,0,0,1,1,1,1]",
    );
    assert!(
        s > 0.0 && s < 1.0,
        "split_cubic_bezier: s = {s} must be strictly interior to (0, 1)",
    );

    let p0 = cps[0];
    let p1 = cps[1];
    let p2 = cps[2];
    let p3 = cps[3];

    // de Casteljau lerps. For a cubic Bézier:
    //   Q0 = lerp(P0, P1, s), Q1 = lerp(P1, P2, s), Q2 = lerp(P2, P3, s)
    //   R0 = lerp(Q0, Q1, s), R1 = lerp(Q1, Q2, s)
    //   S0 = lerp(R0, R1, s)
    // Left half (covering original [0, s]) is [P0, Q0, R0, S0]; right half
    // (covering original [s, 1]) is [S0, R1, Q2, P3]. Both are cubic Béziers
    // by the standard de Casteljau subdivision identity (Farin, "Curves and
    // Surfaces for CAGD", §6.3).
    let lerp = |a: [f64; 3], b: [f64; 3], t: f64| -> [f64; 3] {
        [
            a[0] + (b[0] - a[0]) * t,
            a[1] + (b[1] - a[1]) * t,
            a[2] + (b[2] - a[2]) * t,
        ]
    };
    let q0 = lerp(p0, p1, s);
    let q1 = lerp(p1, p2, s);
    let q2 = lerp(p2, p3, s);
    let r0 = lerp(q0, q1, s);
    let r1 = lerp(q1, q2, s);
    let s0 = lerp(r0, r1, s);

    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let left = VectorNurbs::<f64, 3>::try_new(3, knots.clone(), vec![p0, q0, r0, s0], None)
        .expect("split_cubic_bezier: left half is a valid single-piece cubic Bézier");
    let right = VectorNurbs::<f64, 3>::try_new(3, knots, vec![s0, r1, q2, p3], None)
        .expect("split_cubic_bezier: right half is a valid single-piece cubic Bézier");
    (left, right)
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlendFamily {
    CubicBezier,
}

/// E-axis classification per CLAUDE.md feature scope. `CubicSegment::try_new`
/// applies the §6.1 classification rules to derive this from raw `(ΔX, ΔY, ΔZ, ΔE)`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EMode {
    /// Extrusion proportional to actual XY shaped motion: `e_actual(t) = ratio × ∫|v_xy| dt`.
    /// `extrusion_per_xy_mm` is nonzero and signed (positive for normal extrusion;
    /// negative for retract-during-XY-motion / wipe / coast). Used for moves with
    /// `ΔXY > ε_xyz`, `ΔZ ≤ ε_z`, and `abs(ΔE) > ε_e`.
    CoupledToXy,
    /// Travel move: XY motion with no extrusion. Equivalent to `CoupledToXy` with
    /// `extrusion_per_xy_mm = 0`. Modeled distinctly for clarity in logs/telemetry
    /// and to allow a future plan layer to skip per-sample E integration when the
    /// ratio is definitionally zero.
    Travel,
    /// E motion not coupled to XY: own E NURBS carries the trajectory in time.
    /// In 7-pre's live pipeline, `Independent` always implies null `xyz` motion
    /// (`cp_polygon_length` and midpoint parametric speed both below thresholds).
    /// Helical extrusion (XYZ + E) is rejected upstream; never produces `Independent`
    /// in the live pipeline.
    Independent,
}

/// Sub-segment provenance, populated by `split_segment_to_cap` (`geometry::splitter`).
/// `None` when the segment was not split.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitInfo {
    /// 0-indexed position of this child within the parent's sub-segment sequence.
    pub sub_index: u32,
    /// Total sub-segments produced from the parent. May be < the originally-planned
    /// `k` if epsilon-filtering at splitter step 6 dropped near-boundary breakpoints.
    pub sub_count: u32,
    /// Arc-length range this sub-segment occupies in the parent's arc-length domain.
    /// Computed at split time by querying the parent's arc-length table at the child's
    /// `xyz.u_start` and `xyz.u_end`.
    pub s_lo_mm: f64,
    pub s_hi_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceRange {
    pub start_line: u32,
    pub end_line: u32,
}

#[cfg(test)]
mod tests {
    use super::*;
    use nurbs::VectorNurbs;

    #[test]
    fn cubic_variant_constructs() {
        let xyz = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
            ],
            None,
        )
        .expect("valid cubic");
        let cs = CubicSegment::try_new(
            xyz,
            EMode::Travel,
            0.0,
            None,
            100.0,
            SourceRange {
                start_line: 1,
                end_line: 1,
            },
            None,
        )
        .expect("valid travel");
        let seg: Segment = Segment::Cubic(cs);
        assert!(matches!(seg, Segment::Cubic(_)));
    }

    /// **Position continuity invariant** (the headline `split_cubic_bezier`
    /// guarantee): for any `s ∈ (0, 1)` the left half evaluated at `u = 1` and
    /// the right half evaluated at `u = 0` both reproduce the original curve's
    /// value at `s`, and the global curve traversed via left-then-right
    /// matches the original at every sampled point.
    #[test]
    fn split_cubic_bezier_preserves_position_at_split_and_along_curve() {
        use nurbs::eval::vector_eval;

        // A non-degenerate, non-collinear cubic Bézier in 3D. Z varies too so
        // we exercise all three coordinates.
        let xyz = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [10.0, 20.0, 1.0],
                [30.0, 5.0, 2.0],
                [40.0, 25.0, 3.0],
            ],
            None,
        )
        .unwrap();

        for &s in &[0.1_f64, 0.25, 0.4, 0.5, 0.6, 0.75, 0.9] {
            let (left, right) = split_cubic_bezier(&xyz, s);

            // Continuity at the split: left(1) == right(0) == xyz(s).
            let p_orig = vector_eval(&xyz, s);
            let p_left_end = vector_eval(&left, 1.0);
            let p_right_start = vector_eval(&right, 0.0);
            for axis in 0..3 {
                assert!(
                    (p_left_end[axis] - p_orig[axis]).abs() < 1e-12,
                    "s = {s}, axis {axis}: left(1) = {} vs xyz(s) = {}",
                    p_left_end[axis],
                    p_orig[axis],
                );
                assert!(
                    (p_right_start[axis] - p_orig[axis]).abs() < 1e-12,
                    "s = {s}, axis {axis}: right(0) = {} vs xyz(s) = {}",
                    p_right_start[axis],
                    p_orig[axis],
                );
            }

            // Endpoint continuity: left(0) == xyz(0), right(1) == xyz(1).
            let p0 = vector_eval(&xyz, 0.0);
            let p1 = vector_eval(&xyz, 1.0);
            let p_left_start = vector_eval(&left, 0.0);
            let p_right_end = vector_eval(&right, 1.0);
            for axis in 0..3 {
                assert!((p_left_start[axis] - p0[axis]).abs() < 1e-12);
                assert!((p_right_end[axis] - p1[axis]).abs() < 1e-12);
            }

            // Along-curve fidelity: sample both halves at a sweep of internal
            // u values and confirm the reconstructed traversal matches the
            // original curve at the matching s-domain location.
            //
            // Left covers original [0, s], right covers [s, 1]. Sample 21 u
            // values in [0, 1] for each half and remap to the original s-domain.
            for k in 0..=20 {
                let u_local = (k as f64) / 20.0;
                let u_left = u_local * s;
                let u_right = s + u_local * (1.0 - s);
                let lhs_left = vector_eval(&left, u_local);
                let lhs_right = vector_eval(&right, u_local);
                let rhs_left = vector_eval(&xyz, u_left);
                let rhs_right = vector_eval(&xyz, u_right);
                for axis in 0..3 {
                    assert!(
                        (lhs_left[axis] - rhs_left[axis]).abs() < 1e-10,
                        "s = {s}, u_local = {u_local}, axis {axis}: left mismatch",
                    );
                    assert!(
                        (lhs_right[axis] - rhs_right[axis]).abs() < 1e-10,
                        "s = {s}, u_local = {u_local}, axis {axis}: right mismatch",
                    );
                }
            }

            // Both halves are valid single-piece cubic Béziers — same
            // invariants `CubicSegment::try_new` enforces.
            assert_eq!(left.degree(), 3);
            assert_eq!(right.degree(), 3);
            assert_eq!(left.control_points().len(), 4);
            assert_eq!(right.control_points().len(), 4);
            let expected: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
            assert_eq!(left.knots(), expected.as_slice());
            assert_eq!(right.knots(), expected.as_slice());
            assert!(left.weights().is_none());
            assert!(right.weights().is_none());
        }
    }

    /// `split_cubic_bezier` panics if `s` is not strictly interior.
    #[test]
    #[should_panic(expected = "strictly interior")]
    fn split_cubic_bezier_panics_on_boundary_s() {
        let xyz = VectorNurbs::<f64, 3>::try_new(
            3,
            vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [2.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
            ],
            None,
        )
        .unwrap();
        let _ = split_cubic_bezier(&xyz, 0.0);
    }
}
