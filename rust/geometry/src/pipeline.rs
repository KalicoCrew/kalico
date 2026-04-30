//! `GeometryPipeline`, `Segments`, `Item`. Drives reduce events into typed
//! segments. Only G5 / G5.1 produce motion segments and they emit as
//! `Segment::Cubic` (uniform single-piece cubic Bézier; G5.1 is
//! degree-elevated 2→3 exactly). G0/G1/G2/G3 are rejected at reduce time
//! and surface here as `Fatal::UnsupportedGcode`. Legacy G-code is
//! normalized offline by the Step-13 compatibility layer.

use crate::{
    CubicSegment, EMode, Fatal, FitterParams, GeometryError, JunctionDeviation, Recovery, Segment,
    SourceRange, TelemetryEvent,
    error::{InternalDetails, InternalKind},
    reduce::{CurveGeom, MotionMarkerKind, ParseErrorKind, ReduceEvent, reduce},
};
use gcode::lex;
use std::collections::VecDeque;

#[derive(Debug)]
pub struct GeometryPipeline {
    params: FitterParams,
}

impl GeometryPipeline {
    #[must_use]
    pub fn new(params: FitterParams) -> Self {
        debug_assert!(
            params.degree >= 1 && params.degree <= 5,
            "degree must be in [1, 5], got {}",
            params.degree
        );
        debug_assert!(
            params.theta_smooth_deg > 0.0
                && params.theta_smooth_deg < params.theta_hard_deg
                && params.theta_hard_deg < 180.0
        );
        debug_assert!(params.eps_chord_mm > 0.0);
        debug_assert!(params.max_window_vertices >= u32::from(params.degree) + 2);
        Self { params }
    }

    /// Process a complete G-code buffer. Returns a borrowing iterator over
    /// the segment stream. Sink receives observability events synchronously
    /// during processing.
    ///
    /// One-shot per file by convention.
    pub fn process<'a>(
        &'a mut self,
        text: &'a str,
        sink: &'a mut dyn FnMut(TelemetryEvent),
    ) -> Segments<'a> {
        Segments {
            params: &self.params,
            events: Box::new(reduce(lex(text))),
            queue: VecDeque::new(),
            sink,
            terminal: false,
            prev_g1_end: None,
            prev_g1_feedrate: None,
            prev_g1_dir: None,
        }
    }
}

#[derive(Debug)]
#[non_exhaustive]
pub enum Item {
    Segment(Segment),
    Recovered(Segment, Recovery),
    Fatal(Fatal),
}

/// Borrowing iterator over the segment stream produced by [`GeometryPipeline::process`].
///
/// `Debug` is not derived: `events` is a boxed trait object and `sink` is a
/// raw `&mut dyn Fn` pointer. A manual impl is not worthwhile while the struct
/// is still evolving across Tasks 18-22.
#[allow(missing_debug_implementations)]
pub struct Segments<'a> {
    #[allow(dead_code)] // consumed in Tasks 18-22
    params: &'a FitterParams,
    events: Box<dyn Iterator<Item = ReduceEvent> + 'a>,
    queue: VecDeque<Item>,
    sink: &'a mut dyn FnMut(TelemetryEvent),
    terminal: bool,
    /// End-position of the previous emitted G1 segment, for junction-deviation construction.
    /// In the live pipeline G1 never reaches `handle_curve`; retained for future use.
    prev_g1_end: Option<[f64; 3]>,
    /// Feedrate of the previous emitted G1, for junction-deviation construction.
    prev_g1_feedrate: Option<f64>,
    /// 3D unit direction of the previous emitted G1 segment, used to compute
    /// the junction angle when the next G1 arrives. Cleared at any marker break.
    prev_g1_dir: Option<[f64; 3]>,
}

const QUEUE_HARD_BOUND: usize = 8;

impl Iterator for Segments<'_> {
    type Item = Item;

    fn next(&mut self) -> Option<Item> {
        if self.terminal {
            return None;
        }
        loop {
            if let Some(item) = self.queue.pop_front() {
                if matches!(item, Item::Fatal(_)) {
                    self.terminal = true;
                }
                return Some(item);
            }
            // Drive the reduce iterator forward until something queues an item.
            let event = self.events.next()?;
            self.handle_event(event);
            debug_assert!(
                self.queue.len() <= QUEUE_HARD_BOUND,
                "queue grew beyond bound: {}",
                self.queue.len()
            );
        }
    }
}

impl Segments<'_> {
    fn handle_event(&mut self, event: ReduceEvent) {
        match event {
            ReduceEvent::Curve {
                geom,
                e_delta,
                feedrate_mm_s,
                line_no,
            } => {
                self.handle_curve(geom, e_delta, feedrate_mm_s, line_no);
            }
            ReduceEvent::CommentMarker { kind, line_no } => {
                // LayerType, EndOfPrint, and unknown markers have no Phase 1 telemetry mapping.
                if let gcode::MarkerKind::LayerChange { layer } = kind {
                    (self.sink)(TelemetryEvent::LayerChange { layer, line_no });
                }
                // Marker terminates G1 chain.
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
            ReduceEvent::Marker {
                kind,
                line_no,
                tool,
                e_delta_mm,
            } => {
                match kind {
                    MotionMarkerKind::T => {
                        if let Some(tool) = tool {
                            (self.sink)(TelemetryEvent::ToolChange { tool, line_no });
                        }
                    }
                    MotionMarkerKind::EOnly => {
                        if let Some(e_delta_mm) = e_delta_mm {
                            (self.sink)(TelemetryEvent::Retraction {
                                e_delta_mm,
                                line_no,
                            });
                        }
                    }
                    _ => {}
                }
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
            ReduceEvent::ParseError {
                line_no,
                kind,
                text,
            } => match kind {
                ParseErrorKind::UnsupportedGcode { kind } => {
                    // Live pipeline cannot continue safely: reduce-stage doesn't
                    // update modal state for rejected G0/G1/G2/G3, so any later
                    // G5 would emit cubic segments from stale position. Fail-closed;
                    // user must run Step-13 compat layer on the input first.
                    self.queue.push_back(Item::Fatal(Fatal::UnsupportedGcode {
                        line_no,
                        gcode_kind: kind,
                    }));
                }
                other_kind => {
                    let recovery = match other_kind {
                        ParseErrorKind::MalformedNumber
                        | ParseErrorKind::DuplicateParam
                        | ParseErrorKind::EmptyCommand
                        | ParseErrorKind::G5MalformedTangent => {
                            Recovery::MalformedParams { line_no, raw: text }
                        }
                        ParseErrorKind::UnrecognizedHead => Recovery::UnrecognizedCommand {
                            line_no,
                            head: text,
                        },
                        ParseErrorKind::G5MissingTangent => Recovery::G5MissingTangent { line_no },
                        ParseErrorKind::G5PlaneMismatch => {
                            let active_plane_g_code = text.parse::<u32>().expect(
                                "G5PlaneMismatch.text must be numeric per reduce-side contract (Task 18 emit format)"
                            );
                            Recovery::G5PlaneMismatch {
                                line_no,
                                active_plane_g_code,
                            }
                        }
                        ParseErrorKind::UnsupportedGcode { .. } => {
                            unreachable!("handled above")
                        }
                    };
                    // Dual-emit: sink fires first per §5.1 ordering contract.
                    (self.sink)(TelemetryEvent::Recovery(recovery.clone()));
                    // Synthetic zero-length junction at the previous position (or
                    // origin if none) so the consumer's segment stream sees
                    // Item::Recovered without losing the error.
                    let pos = self.prev_g1_end.unwrap_or([0.0, 0.0, 0.0]);
                    let jd = JunctionDeviation {
                        position: pos,
                        angle_deg: 0.0,
                        feedrate_mm_s: self.prev_g1_feedrate.unwrap_or(0.0),
                        source: SourceRange {
                            start_line: line_no,
                            end_line: line_no,
                        },
                    };
                    self.queue
                        .push_back(Item::Recovered(Segment::Junction(jd), recovery));
                }
            },
        }
    }

    #[allow(clippy::needless_pass_by_value)]
    fn handle_curve(
        &mut self,
        geom: CurveGeom,
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    ) {
        let source = SourceRange {
            start_line: line_no,
            end_line: line_no,
        };

        // Step 1: turn `geom` into a single-piece cubic Bézier xyz NURBS.
        // G5 → already cubic; G5.1 → exact degree-elevation 2→3.
        let xyz: nurbs::VectorNurbs<f64, 3> = match geom {
            CurveGeom::Cubic { cps } => nurbs_from_cubic(cps),
            CurveGeom::Quadratic { cps } => degree_elevate_2_to_3(&nurbs_from_quadratic(cps)),
        };

        // Step 2: classify E-mode and construct the segment.
        match classify_e_mode(&xyz, e_delta) {
            Ok((e_mode, extrusion_per_xy_mm, e_independent)) => {
                match CubicSegment::try_new(
                    xyz,
                    e_mode,
                    extrusion_per_xy_mm,
                    e_independent,
                    feedrate_mm_s,
                    source,
                    None,
                ) {
                    Ok(seg) => {
                        self.queue.push_back(Item::Segment(Segment::Cubic(seg)));
                    }
                    Err(
                        GeometryError::NotSinglePieceCubic { reason }
                        | GeometryError::EModeInvariantViolation { reason },
                    ) => {
                        self.queue.push_back(Item::Fatal(Fatal::Internal(Box::new(
                            InternalDetails {
                                kind: InternalKind::CubicSegmentInvariantViolation { reason },
                                context: format!("line_no={line_no}"),
                                backtrace: std::backtrace::Backtrace::capture(),
                            },
                        ))));
                    }
                    Err(_) => unreachable!(
                        "classify_e_mode would not have returned Ok if try_new could fail this way"
                    ),
                }
            }
            Err(GeometryError::ZeroMotion) => {
                // Drop zero-motion segments silently — no Recovery, no Fatal.
            }
            Err(GeometryError::HelicalExtrusionUnsupported) => {
                // Per design (CLAUDE.md feature scope), extrusion couples to XY
                // motion only — XY+Z+E and Z+E in one G5 segment are
                // design-rejected. Fail-closed because reduce-stage already
                // committed modal-state side effects (position, E, prev_g5_pq)
                // before classification reached this arm; continuation would
                // silently start subsequent G5s from the rejected endpoint.
                self.queue
                    .push_back(Item::Fatal(Fatal::HelicalExtrusionUnsupported { line_no }));
            }
            Err(_) => unreachable!("classify_e_mode return shape is exhaustive"),
        }

        // Cubic segments break the G1-tangent chain (curvature-continuity principle).
        self.prev_g1_end = None;
        self.prev_g1_feedrate = None;
        self.prev_g1_dir = None;
    }
}

/// Classify a cubic xyz NURBS plus its scalar `e_delta` into an `EMode` plus
/// the matching companion fields. See CLAUDE.md feature scope §"E-follows-XY".
///
/// Returns:
/// - `Ok((e_mode, extrusion_per_xy_mm, e_independent))` for valid segments.
/// - `Err(ZeroMotion)` when no motion threshold crosses (caller drops silently).
/// - `Err(HelicalExtrusionUnsupported)` for the XY+Z+E combination, which the
///   live MVP rejects.
///
/// **Why XY arc length, not endpoint chord:** a cubic with collinear or looping
/// control points can have zero endpoint chord but a real XY path. ROUND-1
/// review HIGH-1 fix.
fn classify_e_mode(
    xyz: &nurbs::VectorNurbs<f64, 3>,
    e_delta: Option<f64>,
) -> Result<(EMode, f64, Option<nurbs::ScalarNurbs<f64>>), GeometryError> {
    const EPS_XYZ: f64 = 1e-6;
    const EPS_Z: f64 = 1e-6;
    const EPS_E: f64 = 1e-6;

    let xy_len = nurbs::arc_length::xy_arc_length(xyz);

    // Z motion: endpoint delta on cps[3] - cps[0] (single-piece cubic Bézier).
    let cps = xyz.control_points();
    let dz = (cps[3][2] - cps[0][2]).abs();

    let de = e_delta.unwrap_or(0.0);
    let abs_de = de.abs();

    let xyz_motion = xy_len > EPS_XYZ;
    let z_motion = dz > EPS_Z;
    let e_motion = abs_de > EPS_E;

    match (xyz_motion, z_motion, e_motion) {
        // Helical extrusion (XY+Z+E) and pure-Z+E: both rejected. Extrusion is
        // meant to couple to XY motion only (CLAUDE.md feature scope), and the
        // splitter cannot safely subdivide an Independent segment with
        // non-trivial xyz motion (it would clone the full E curve into every
        // child). The (false, true, true) arm closes that pre-Fix-A.1 leak.
        (true | false, true, true) => Err(GeometryError::HelicalExtrusionUnsupported),
        // Coupled: real XY motion, no Z motion, real E motion. Signed ratio.
        (true, false, true) => Ok((EMode::CoupledToXy, de / xy_len, None)),
        // Travel: XY motion no E (Z optional), or pure-Z no E.
        (true, _, false) | (false, true, false) => Ok((EMode::Travel, 0.0, None)),
        // Pure E motion (no XY, no Z): Independent retraction/prime/filament-change.
        (false, false, true) => {
            let e_curve = build_linear_e_curve(de);
            Ok((EMode::Independent, 0.0, Some(e_curve)))
        }
        // No motion at all.
        (false, false, false) => Err(GeometryError::ZeroMotion),
    }
}

/// Build a degree-1 linear scalar NURBS for `Independent`-mode E motion:
/// `e(u) = (1−u)·0 + u·e_delta`, knots `[0,0,1,1]`.
fn build_linear_e_curve(e_delta: f64) -> nurbs::ScalarNurbs<f64> {
    nurbs::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, e_delta], None)
        .expect("linear E curve always valid")
}

/// Bernstein degree-elevation from a degree-2 Bézier polynomial NURBS to
/// degree-3, preserving the curve exactly (no fit error). For Bézier control
/// points `[Q_0, Q_1, Q_2]`, the equivalent degree-3 has CPs:
///
/// `[Q_0, (1/3)Q_0 + (2/3)Q_1, (2/3)Q_1 + (1/3)Q_2, Q_2]`
///
/// Per Piegl & Tiller §5.5. Used for G5.1 → G5 promotion.
#[must_use]
pub fn degree_elevate_2_to_3(quadratic: &nurbs::VectorNurbs<f64, 3>) -> nurbs::VectorNurbs<f64, 3> {
    debug_assert_eq!(quadratic.degree(), 2);
    debug_assert_eq!(quadratic.control_points().len(), 3);
    debug_assert!(quadratic.weights().is_none(), "G5.1 is non-rational");
    let q = quadratic.control_points();
    let p0 = q[0];
    let p1 = [
        (1.0 / 3.0) * q[0][0] + (2.0 / 3.0) * q[1][0],
        (1.0 / 3.0) * q[0][1] + (2.0 / 3.0) * q[1][1],
        (1.0 / 3.0) * q[0][2] + (2.0 / 3.0) * q[1][2],
    ];
    let p2 = [
        (2.0 / 3.0) * q[1][0] + (1.0 / 3.0) * q[2][0],
        (2.0 / 3.0) * q[1][1] + (1.0 / 3.0) * q[2][1],
        (2.0 / 3.0) * q[1][2] + (1.0 / 3.0) * q[2][2],
    ];
    let p3 = q[2];
    nurbs::VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        vec![p0, p1, p2, p3],
        None,
    )
    .expect("degree-elevation always valid")
}

fn nurbs_from_quadratic(cps: [[f64; 3]; 3]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], cps.to_vec(), None)
        .expect("non-rational quadratic Bézier with 3 CPs and clamped knots is always valid")
}

fn nurbs_from_cubic(cps: [[f64; 3]; 4]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
        None,
    )
    .expect("non-rational cubic Bézier with 4 CPs and clamped knots is always valid")
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Item, Recovery, Segment, TelemetryEvent};

    fn collect(text: &str) -> Vec<Item> {
        let mut p = GeometryPipeline::new(FitterParams::default());
        let mut sink = |_: crate::TelemetryEvent| {};
        p.process(text, &mut sink).collect()
    }

    #[test]
    fn empty_input_yields_no_items() {
        let mut p = GeometryPipeline::new(FitterParams::default());
        let mut sink = |_e: crate::TelemetryEvent| {};
        let items: Vec<_> = p.process("", &mut sink).collect();
        assert!(items.is_empty());
    }

    #[test]
    fn whitespace_input_yields_no_items() {
        let mut p = GeometryPipeline::new(FitterParams::default());
        let mut sink = |_e: crate::TelemetryEvent| {};
        let items: Vec<_> = p.process("\n\n   \n", &mut sink).collect();
        assert!(items.is_empty());
    }

    #[test]
    fn layer_change_marker_fires_telemetry() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let _items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process(";LAYER:5\n", &mut sink).collect()
        };
        assert!(matches!(
            events.as_slice(),
            [TelemetryEvent::LayerChange {
                layer: Some(5),
                line_no: 1
            }]
        ));
    }

    #[test]
    fn tool_change_fires_telemetry() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let _items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("T1\n", &mut sink).collect()
        };
        assert!(matches!(
            events.as_slice(),
            [TelemetryEvent::ToolChange {
                tool: 1,
                line_no: 1
            }]
        ));
    }

    #[test]
    fn g5_emits_cubic_segment() {
        // Pure G5: no preceding G1 (which would be UnsupportedGcode in live).
        // We need to seed `state.prev_g5_pq` for the implicit-tangent rule, so
        // chain two G5s: first one with explicit I,J and P,Q, second one
        // would inherit — but we just need one to test cubic emission.
        let items = collect("G5 X10 Y0 I3 J3 P-3 Q3\n");
        let cubic_seg = items.iter().find_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c),
            _ => None,
        });
        let c = cubic_seg.expect("expected a Segment::Cubic");
        assert_eq!(c.xyz.degree(), 3);
        let cps = c.xyz.control_points();
        assert_eq!(cps.len(), 4);
        let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
        assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 0.0));
        assert!(approx(cps[1][0], 3.0) && approx(cps[1][1], 3.0));
        assert!(approx(cps[2][0], 7.0) && approx(cps[2][1], 3.0));
        assert!(approx(cps[3][0], 10.0) && approx(cps[3][1], 0.0));
        // Knot vector [0,0,0,0,1,1,1,1].
        let knots = c.xyz.knots();
        assert_eq!(knots, &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
        // Non-rational.
        assert!(c.xyz.weights().is_none(), "G5 cubic must be non-rational");
        // No E delta → Travel.
        assert_eq!(c.e_mode, crate::EMode::Travel);
        // No JD before the G5 in the live pipeline.
        assert!(
            !items
                .iter()
                .any(|it| matches!(it, Item::Segment(Segment::Junction(_)))),
            "G5 must not emit a junction here, got {items:#?}"
        );
    }

    #[test]
    fn g5_1_emits_cubic_via_degree_elevation() {
        let items = collect("G5.1 X10 Y0 I3 J3\n");
        let cubic_seg = items.iter().find_map(|it| match it {
            Item::Segment(Segment::Cubic(c)) => Some(c),
            _ => None,
        });
        let c = cubic_seg.expect("expected a Segment::Cubic from G5.1");
        // Post-elevation invariants: degree 3, 4 CPs, non-rational.
        assert_eq!(c.xyz.degree(), 3);
        assert_eq!(c.xyz.control_points().len(), 4);
        assert!(
            c.xyz.weights().is_none(),
            "G5.1 → G5 must remain non-rational"
        );
        // Knot vector [0,0,0,0,1,1,1,1].
        let knots = c.xyz.knots();
        assert_eq!(knots, &[0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0]);
        // Endpoints preserved exactly: P0=(0,0,0), P3=(10,0,0).
        let cps = c.xyz.control_points();
        let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
        assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 0.0));
        assert!(approx(cps[3][0], 10.0) && approx(cps[3][1], 0.0));
        // No E delta → Travel.
        assert_eq!(c.e_mode, crate::EMode::Travel);
    }

    #[test]
    fn g5_1_outside_xy_plane_yields_recovered() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("G18\nG5.1 X10 Z1 I3 J3\n", &mut sink).collect()
        };
        let recovered = items.iter().find_map(|it| match it {
            Item::Recovered(
                _,
                Recovery::G5PlaneMismatch {
                    line_no: 2,
                    active_plane_g_code: 18,
                },
            ) => Some(()),
            _ => None,
        });
        assert!(
            recovered.is_some(),
            "expected G5PlaneMismatch, got {items:#?}"
        );
        assert!(matches!(
            events.last(),
            Some(TelemetryEvent::Recovery(Recovery::G5PlaneMismatch {
                line_no: 2,
                active_plane_g_code: 18
            }))
        ));
    }
}
