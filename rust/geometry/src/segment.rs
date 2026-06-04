use nurbs::{ScalarNurbs, VectorNurbs};

#[non_exhaustive]
#[derive(Debug, Clone, PartialEq)]
pub enum Segment {
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

#[derive(Debug, Clone, PartialEq)]
pub struct CubicSegment {
    pub xyz: VectorNurbs<f64, 3>,
    pub e_mode: EMode,
    pub extrusion_per_xy_mm: f64,
    pub e_independent: Option<ScalarNurbs<f64>>,
    pub feedrate_mm_s: f64,
    pub source: SourceRange,
    pub split_info: Option<SplitInfo>,
}

impl CubicSegment {
    pub fn try_new(
        xyz: VectorNurbs<f64, 3>,
        e_mode: EMode,
        extrusion_per_xy_mm: f64,
        e_independent: Option<ScalarNurbs<f64>>,
        feedrate_mm_s: f64,
        source: SourceRange,
        split_info: Option<SplitInfo>,
    ) -> Result<Self, crate::GeometryError> {
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
        let expected_knots: [f64; 8] = [0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
        if xyz.knots() != expected_knots.as_slice() {
            return Err(crate::GeometryError::NotSinglePieceCubic {
                reason: "knot vector not clamped [0,0,0,0,1,1,1,1]",
            });
        }

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

#[must_use]
pub fn split_cubic_bezier(
    xyz: &VectorNurbs<f64, 3>,
    s: f64,
) -> (VectorNurbs<f64, 3>, VectorNurbs<f64, 3>) {
    assert_eq!(xyz.degree(), 3, "split_cubic_bezier: degree must be 3");
    let cps = xyz.control_points();
    assert_eq!(
        cps.len(),
        4,
        "split_cubic_bezier: must have 4 control points"
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
    let left = VectorNurbs::<f64, 3>::try_new(3, knots.clone(), vec![p0, q0, r0, s0])
        .expect("split_cubic_bezier: left half is a valid single-piece cubic Bézier");
    let right = VectorNurbs::<f64, 3>::try_new(3, knots, vec![s0, r1, q2, p3])
        .expect("split_cubic_bezier: right half is a valid single-piece cubic Bézier");
    (left, right)
}

#[non_exhaustive]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BlendFamily {
    CubicBezier,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EMode {
    CoupledToXy,
    Travel,
    Independent,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SplitInfo {
    pub sub_index: u32,
    pub sub_count: u32,
    pub s_lo_mm: f64,
    pub s_hi_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SourceRange {
    pub start_line: u32,
    pub end_line: u32,
}

#[cfg(test)]
mod tests;
