//! `GeometryPipeline`, `Segments`, `Item`. Drives reduce events into typed
//! segments. Phase 1 emits degree-1 NURBS for G1, 3D rational quadratic for
//! G2/G3, and `JunctionDeviation` at every G1-G1 transition.

use crate::{
    reduce::{reduce, MotionMarkerKind, ReduceEvent},
    Fatal, FittedSegment, FitterParams, JunctionDeviation, Recovery, Segment, SourceRange,
    TelemetryEvent,
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
    #[allow(dead_code)] // consumed in Tasks 18-22
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
    #[allow(clippy::needless_pass_by_value)] // G1Move arm destructures and consumes; other arms handled in Tasks 19+
    fn handle_event(&mut self, event: ReduceEvent) {
        match event {
            ReduceEvent::G1Move { from, to, e_delta: _, feedrate_mm_s, line_no } => {
                // Emit a JunctionDeviation if we have a previous G1 direction.
                if let (Some(prev_dir), Some(prev_f)) = (self.prev_g1_dir, self.prev_g1_feedrate) {
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

                let xyz = degree_1_nurbs(from, to);
                let seg = FittedSegment {
                    xyz,
                    e: None, // Phase 1: E carried as marker-break or per-segment scalar; full E NURBS is Phase 2.
                    feedrate_mm_s,
                    degree: 1,
                    max_residual_mm: 0.0,
                    source: SourceRange { start_line: line_no, end_line: line_no },
                };
                self.queue.push_back(Item::Segment(Segment::Fitted(seg)));

                self.prev_g1_end = Some(to);
                self.prev_g1_feedrate = Some(feedrate_mm_s);
                self.prev_g1_dir = Some(unit([to[0] - from[0], to[1] - from[1], to[2] - from[2]]));
            }
            _ => {
                // Other event kinds handled in subsequent tasks.
                // Reference MotionMarkerKind to keep the import live until Tasks 19+.
                let _: Option<MotionMarkerKind> = None;
            }
        }
    }
}

fn degree_1_nurbs(from: [f64; 3], to: [f64; 3]) -> nurbs::VectorNurbs<f64, 3> {
    nurbs::VectorNurbs::<f64, 3>::try_new(
        1,
        vec![0.0, 0.0, 1.0, 1.0],
        vec![from, to],
        None,
    )
    .expect("degree-1 NURBS with 2 CPs is always valid")
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
    use crate::{Item, Segment, FittedSegment, JunctionDeviation};

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
}
