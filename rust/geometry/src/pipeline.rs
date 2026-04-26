//! `GeometryPipeline`, `Segments`, `Item`. Drives reduce events into typed
//! segments. Phase 1 emits degree-1 NURBS for G1, 3D rational quadratic for
//! G2/G3, and `JunctionDeviation` at every G1-G1 transition.

use crate::{
    reduce::{reduce, MotionMarkerKind, ReduceEvent},
    Fatal, FitterParams, Recovery, Segment, TelemetryEvent,
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
    #[allow(dead_code)] // consumed in Tasks 18-22
    prev_g1_end: Option<[f64; 3]>,
    /// Feedrate of the previous emitted G1, for junction-deviation construction.
    #[allow(dead_code)] // consumed in Tasks 18-22
    prev_g1_feedrate: Option<f64>,
    /// 3D unit direction of the previous emitted G1 segment, used to compute
    /// the junction angle when the next G1 arrives. Cleared at any marker break.
    #[allow(dead_code)] // consumed in Tasks 18-22
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
            self.handle_event(&event);
            debug_assert!(self.queue.len() <= QUEUE_HARD_BOUND,
                "queue grew beyond bound: {}", self.queue.len());
        }
    }
}

impl Segments<'_> {
    #[allow(clippy::unused_self)] // self.queue populated once Tasks 18-22 fill this in
    fn handle_event(&mut self, event: &ReduceEvent) {
        // Filled in across Tasks 18-22.
        // Reference MotionMarkerKind to keep the import live until Tasks 18-22.
        let _: Option<MotionMarkerKind> = None;
        let _ = event;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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
}
