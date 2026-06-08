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

#[allow(missing_debug_implementations)]
pub struct Segments<'a> {
    #[allow(dead_code)]
    params: &'a FitterParams,
    events: Box<dyn Iterator<Item = ReduceEvent> + 'a>,
    queue: VecDeque<Item>,
    sink: &'a mut dyn FnMut(TelemetryEvent),
    terminal: bool,
    prev_g1_end: Option<[f64; 3]>,
    prev_g1_feedrate: Option<f64>,
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
                if let gcode::MarkerKind::LayerChange { layer } = kind {
                    (self.sink)(TelemetryEvent::LayerChange { layer, line_no });
                }
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
                    (self.sink)(TelemetryEvent::Recovery(recovery.clone()));
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

        let xyz: nurbs::VectorNurbs<f64, 3> = match geom {
            CurveGeom::Cubic { cps } => nurbs_from_cubic(cps),
            CurveGeom::Quadratic { cps } => degree_elevate_2_to_3(&nurbs_from_quadratic(cps)),
        };

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
            Err(GeometryError::ZeroMotion) => {}
            Err(GeometryError::HelicalExtrusionUnsupported) => {
                self.queue
                    .push_back(Item::Fatal(Fatal::HelicalExtrusionUnsupported { line_no }));
            }
            Err(_) => unreachable!("classify_e_mode return shape is exhaustive"),
        }

        self.prev_g1_end = None;
        self.prev_g1_feedrate = None;
        self.prev_g1_dir = None;
    }
}

fn classify_e_mode(
    xyz: &nurbs::VectorNurbs<f64, 3>,
    e_delta: Option<f64>,
) -> Result<(EMode, f64, Option<nurbs::ScalarNurbs<f64>>), GeometryError> {
    const EPS_XYZ: f64 = 1e-6;
    const EPS_Z: f64 = 1e-6;
    const EPS_E: f64 = 1e-6;

    let xy_len = nurbs::arc_length::xy_arc_length(xyz);

    let cps = xyz.control_points();
    let dz = (cps[3][2] - cps[0][2]).abs();

    let de = e_delta.unwrap_or(0.0);
    let abs_de = de.abs();

    let xyz_motion = xy_len > EPS_XYZ;
    let z_motion = dz > EPS_Z;
    let e_motion = abs_de > EPS_E;

    match (xyz_motion, z_motion, e_motion) {
        (true | false, true, true) => Err(GeometryError::HelicalExtrusionUnsupported),
        (true, false, true) => Ok((EMode::CoupledToXy, de / xy_len, None)),
        (true, _, false) | (false, true, false) => Ok((EMode::Travel, 0.0, None)),
        (false, false, true) => {
            let e_curve = build_linear_e_curve(de);
            Ok((EMode::Independent, 0.0, Some(e_curve)))
        }
        (false, false, false) => Err(GeometryError::ZeroMotion),
    }
}

fn build_linear_e_curve(e_delta: f64) -> nurbs::ScalarNurbs<f64> {
    nurbs::ScalarNurbs::<f64>::try_new(1, vec![0.0, 0.0, 1.0, 1.0], vec![0.0, e_delta])
        .expect("linear E curve always valid")
}

#[must_use]
pub fn degree_elevate_2_to_3(quadratic: &nurbs::VectorNurbs<f64, 3>) -> nurbs::VectorNurbs<f64, 3> {
    debug_assert_eq!(quadratic.degree(), 2);
    debug_assert_eq!(quadratic.control_points().len(), 3);
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
    )
    .expect("degree-elevation always valid")
}

fn nurbs_from_quadratic(cps: [[f64; 3]; 3]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0], cps.to_vec())
        .expect("quadratic Bézier with 3 CPs and clamped knots is always valid")
}

fn nurbs_from_cubic(cps: [[f64; 3]; 4]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        3,
        vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        cps.to_vec(),
    )
    .expect("cubic Bézier with 4 CPs and clamped knots is always valid")
}

#[cfg(test)]
mod tests;
