//! `GeometryPipeline`, `Segments`, `Item`. Drives reduce events into typed
//! segments. Phase 1 emits degree-1 NURBS for G1, 3D rational quadratic for
//! G2/G3, and `JunctionDeviation` at every G1-G1 transition.

use crate::{
    reduce::{reduce, CurveGeom, MotionMarkerKind, ParseErrorKind, ReduceEvent},
    ArcSegment, Fatal, FittedSegment, FitterParams, JunctionDeviation, Recovery, Segment,
    SourceRange, TelemetryEvent,
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
        debug_assert!(params.degree >= 1 && params.degree <= 5,
            "degree must be in [1, 5], got {}", params.degree);
        debug_assert!(params.theta_smooth_deg > 0.0
            && params.theta_smooth_deg < params.theta_hard_deg
            && params.theta_hard_deg < 180.0);
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
            debug_assert!(self.queue.len() <= QUEUE_HARD_BOUND,
                "queue grew beyond bound: {}", self.queue.len());
        }
    }
}

impl Segments<'_> {
    fn handle_event(&mut self, event: ReduceEvent) {
        match event {
            ReduceEvent::Curve { geom, e_delta: _, feedrate_mm_s, line_no } => {
                self.handle_curve(geom, feedrate_mm_s, line_no);
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
            ReduceEvent::Marker { kind, line_no, tool, e_delta_mm } => {
                match kind {
                    MotionMarkerKind::T => {
                        if let Some(tool) = tool {
                            (self.sink)(TelemetryEvent::ToolChange { tool, line_no });
                        }
                    }
                    MotionMarkerKind::EOnly => {
                        if let Some(e_delta_mm) = e_delta_mm {
                            (self.sink)(TelemetryEvent::Retraction { e_delta_mm, line_no });
                        }
                    }
                    _ => {}
                }
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
            ReduceEvent::ParseError { line_no, kind, text } => {
                let recovery = match kind {
                    ParseErrorKind::MalformedNumber
                    | ParseErrorKind::DuplicateParam
                    | ParseErrorKind::EmptyCommand
                    | ParseErrorKind::G5MalformedTangent => {
                        Recovery::MalformedParams { line_no, raw: text }
                    }
                    ParseErrorKind::UnrecognizedHead => {
                        Recovery::UnrecognizedCommand { line_no, head: text }
                    }
                    ParseErrorKind::G5MissingTangent => {
                        Recovery::G5MissingTangent { line_no }
                    }
                    ParseErrorKind::G5PlaneMismatch => {
                        // TODO(task-18): when G5.1 plane-mismatch reduce arm
                        // lands, add a round-trip test asserting text="18"
                        // → active_plane_g_code=18.
                        let active_plane_g_code = text.parse::<u32>().expect(
                            "G5PlaneMismatch.text must be numeric per reduce-side contract (Task 18 emit format)"
                        );
                        Recovery::G5PlaneMismatch { line_no, active_plane_g_code }
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
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Recovered(Segment::Junction(jd), recovery));
            }
        }
    }

    #[allow(clippy::needless_pass_by_value)]
    fn handle_curve(&mut self, geom: CurveGeom, feedrate_mm_s: f64, line_no: u32) {
        match geom {
            CurveGeom::Linear { cps } => {
                let from = cps[0];
                let to = cps[1];
                // Junction-deviation against previous G1 (if any).
                if let (Some(prev_dir), Some(prev_f)) =
                    (self.prev_g1_dir, self.prev_g1_feedrate)
                {
                    let cur_dir = unit([
                        to[0] - from[0], to[1] - from[1], to[2] - from[2],
                    ]);
                    let angle_deg = angle_between_deg(prev_dir, cur_dir);
                    let jd = JunctionDeviation {
                        position: from,
                        angle_deg,
                        feedrate_mm_s: prev_f.min(feedrate_mm_s),
                        source: SourceRange { start_line: line_no, end_line: line_no },
                    };
                    self.queue.push_back(Item::Segment(Segment::Junction(jd)));
                }
                let xyz = nurbs_from_linear(cps);
                let seg = FittedSegment {
                    xyz,
                    e: None,
                    feedrate_mm_s,
                    degree: 1,
                    max_residual_mm: 0.0,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Fitted(seg)));
                self.prev_g1_end = Some(to);
                self.prev_g1_feedrate = Some(feedrate_mm_s);
                self.prev_g1_dir = Some(unit([
                    to[0] - from[0], to[1] - from[1], to[2] - from[2],
                ]));
            }
            CurveGeom::RationalQuadratic { cps, weights } => {
                let xyz = nurbs_from_rational_quadratic(cps, weights);
                let seg = ArcSegment {
                    xyz,
                    e: None,
                    feedrate_mm_s,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Arc(seg)));
                // Arcs break the G1-junction chain (curvature-continuity principle).
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
            CurveGeom::Quadratic { cps } => {
                // Implemented in Task 18. For now, surface as Fatal so the
                // workspace compiles and any inadvertent emission is loud.
                // After Task 18 this arm constructs a degree-2 NURBS and emits
                // Segment::Fitted { degree: 2 }.
                self.emit_unimplemented_curve("Quadratic", line_no);
                let _ = cps;
                // Even when stubbed, break the G1 chain — curve endpoints are not G1 motion.
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
            CurveGeom::Cubic { cps } => {
                // Implemented in Task 17. For now, surface as Fatal so the
                // workspace compiles and any inadvertent emission is loud.
                // After Task 17 this arm constructs a degree-3 NURBS and emits
                // Segment::Fitted { degree: 3 }.
                self.emit_unimplemented_curve("Cubic", line_no);
                let _ = cps;
                // Even when stubbed, break the G1 chain — curve endpoints are not G1 motion.
                self.prev_g1_end = None;
                self.prev_g1_feedrate = None;
                self.prev_g1_dir = None;
            }
        }
    }

    #[allow(clippy::unused_self)]
    fn emit_unimplemented_curve(&mut self, kind: &'static str, _line_no: u32) {
        // Stub for Tasks 17 & 18. Production code never reaches here in
        // Tasks 6-12 because reduce does not yet emit Quadratic / Cubic.
        // debug_assert! lets tests catch a stray emission at developer time.
        debug_assert!(false, "CurveGeom::{kind} reached pipeline before Task 17/18 implementation");
    }
}

fn nurbs_from_linear(cps: [[f64; 3]; 2]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        cps.to_vec(),
        None,
    )
    .expect("degree-1 NURBS with 2 CPs is always valid")
}

fn nurbs_from_rational_quadratic(
    cps: [[f64; 3]; 3],
    weights: [f64; 3],
) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        2,
        vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
        Some(weights.to_vec()),
    )
    .expect("rational quadratic from reduce is always valid")
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
    if n < 1e-12 {
        [0.0, 0.0, 0.0]
    } else {
        [v[0]/n, v[1]/n, v[2]/n]
    }
}

fn angle_between_deg(a: [f64; 3], b: [f64; 3]) -> f64 {
    let dot = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2]).clamp(-1.0, 1.0);
    dot.acos().to_degrees()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Item, Recovery, Segment, FittedSegment, JunctionDeviation, TelemetryEvent};

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
    fn two_g1s_emit_fitted_junction_fitted() {
        let items = collect("G1 X10 F1500\nG1 X10 Y10\n");
        // First G1: Fitted only (no prev).
        // Second G1: Junction (between prev_g1_end and current from), then Fitted.
        assert_eq!(items.len(), 3, "expected 3 items, got {items:#?}");
        match &items[0] {
            Item::Segment(Segment::Fitted(_)) => {}
            other => panic!("[0] expected Fitted, got {other:?}"),
        }
        match &items[1] {
            Item::Segment(Segment::Junction(JunctionDeviation { position, angle_deg, feedrate_mm_s, .. })) => {
                #[allow(clippy::float_cmp)]
                { assert_eq!(*position, [10.0, 0.0, 0.0]); }
                // First leg goes (0,0)→(10,0), second leg (10,0)→(10,10): 90° turn.
                assert!((angle_deg - 90.0).abs() < 1e-6, "expected ~90°, got {angle_deg}");
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9);
            }
            other => panic!("[1] expected Junction, got {other:?}"),
        }
        match &items[2] {
            Item::Segment(Segment::Fitted(_)) => {}
            other => panic!("[2] expected Fitted, got {other:?}"),
        }
    }

    #[test]
    fn single_g1_emits_degree_1_fitted() {
        let items = collect("G1 X10 Y0 F1500\n");
        // First G1 from origin to (10,0): 1 FittedSegment (no preceding G1, so no junction).
        assert_eq!(items.len(), 1, "expected 1 item, got {items:#?}");
        match &items[0] {
            Item::Segment(Segment::Fitted(FittedSegment { xyz, degree, feedrate_mm_s, .. })) => {
                assert_eq!(*degree, 1);
                assert!((*feedrate_mm_s - 25.0).abs() < 1e-9);
                assert_eq!(xyz.degree(), 1);
                assert_eq!(xyz.control_points().len(), 2);
                // Control points are exact integral values set by us — bitwise equality is correct.
                #[allow(clippy::float_cmp)]
                {
                    assert_eq!(xyz.control_points()[0], [0.0_f64, 0.0, 0.0]);
                    assert_eq!(xyz.control_points()[1], [10.0_f64, 0.0, 0.0]);
                }
            }
            other => panic!("expected Fitted, got {other:?}"),
        }
    }

    #[test]
    fn g2_emits_arc_segment_with_3d_control_points() {
        // Quarter-circle from (1, 0, 0) to (0, 1, 0), center (0, 0, 0), CW (G2).
        let items = collect("G1 X1 F1500\nG2 X0 Y1 I-1 J0\n");
        // Expect: Fitted (G1) + ArcSegment.
        assert!(items.len() >= 2);
        let arc_seg = items.iter().find_map(|it| match it {
            Item::Segment(Segment::Arc(a)) => Some(a),
            _ => None,
        });
        let arc = arc_seg.expect("expected an ArcSegment");
        assert_eq!(arc.xyz.degree(), 2);
        // Rational quadratic uses 3 control points; weighted middle CP.
        assert_eq!(arc.xyz.control_points().len(), 3);
        assert!(arc.xyz.weights().is_some(), "rational arc must have weights");
        // For a 90° arc, the corner control point is at the corner of the
        // tangent extension — for arc center (0,0) start (1,0) end (0,1)
        // tangents extend to (1,1).
        let cps = arc.xyz.control_points();
        let approx_eq = |a: f64, b: f64| (a - b).abs() < 1e-9;
        assert!(approx_eq(cps[0][0], 1.0) && approx_eq(cps[0][1], 0.0));
        assert!(approx_eq(cps[1][0], 1.0) && approx_eq(cps[1][1], 1.0));
        assert!(approx_eq(cps[2][0], 0.0) && approx_eq(cps[2][1], 1.0));
        // Z constant.
        for cp in cps { assert!(approx_eq(cp[2], 0.0)); }
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
            [TelemetryEvent::LayerChange { layer: Some(5), line_no: 1 }]
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
            [TelemetryEvent::ToolChange { tool: 1, line_no: 1 }]
        ));
    }

    #[test]
    fn retraction_fires_telemetry() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let _items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("G1 E-1.5 F3000\n", &mut sink).collect()
        };
        assert_eq!(events.len(), 1);
        match &events[0] {
            TelemetryEvent::Retraction { e_delta_mm, line_no: 1 } => {
                assert!((e_delta_mm - (-1.5)).abs() < 1e-12);
            }
            other => panic!("expected Retraction, got {other:?}"),
        }
    }

    #[test]
    fn parse_error_yields_recovered() {
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("G1 X1.2.3\n", &mut sink).collect()
        };
        assert_eq!(items.len(), 1);
        match &items[0] {
            Item::Recovered(_, Recovery::MalformedParams { line_no: 1, .. }) => {}
            other => panic!("expected Recovered, got {other:?}"),
        }
        // Sink should also see Recovery (dual-emit).
        assert!(matches!(
            events.as_slice(),
            [TelemetryEvent::Recovery(Recovery::MalformedParams { line_no: 1, .. })]
        ));
    }

    #[test]
    fn g5_missing_tangent_yields_recovered() {
        // G1 followed directly by G5 with no I,J — chain has no prev G5.
        let mut events = vec![];
        let mut p = GeometryPipeline::new(FitterParams::default());
        let items: Vec<_> = {
            let mut sink = |e: TelemetryEvent| events.push(e);
            p.process("G1 X1 Y0 F1500\nG5 X10 Y0 P-1 Q-1\n", &mut sink).collect()
        };
        let recovered = items.iter().find_map(|it| match it {
            Item::Recovered(_, Recovery::G5MissingTangent { line_no: 2 }) => Some(()),
            _ => None,
        });
        assert!(recovered.is_some(), "expected G5MissingTangent recovery, got {items:#?}");
        assert!(matches!(
            events.last(),
            Some(TelemetryEvent::Recovery(Recovery::G5MissingTangent { line_no: 2 }))
        ));
    }

    #[test]
    fn g2_helical_yields_z_linear_control_points() {
        let items = collect("G1 X1 Z0 F1500\nG2 X0 Y1 Z0.5 I-1 J0\n");
        let arc = items.iter().find_map(|it| match it {
            Item::Segment(Segment::Arc(a)) => Some(a),
            _ => None,
        }).expect("ArcSegment expected");
        let cps = arc.xyz.control_points();
        // Z linear across CPs: 0.0, 0.25, 0.5
        let approx_eq = |a: f64, b: f64| (a - b).abs() < 1e-9;
        assert!(approx_eq(cps[0][2], 0.0));
        assert!(approx_eq(cps[1][2], 0.25));
        assert!(approx_eq(cps[2][2], 0.5));
    }
}
