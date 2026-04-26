//! Reduce: token stream → internal `ReduceEvent` stream. Pub(crate); tests
//! import via `#[cfg(test)] pub use`.
//! Phase 1 implementation is filled in across Tasks 13-17.

/// Modal state machine — accumulates the current position, feedrate, and tool
/// across the gcode stream, applying G1's modal "params absent → unchanged"
/// semantics.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub(crate) struct ModalState {
    pub position: [f64; 3],
    #[allow(dead_code)]
    pub e: f64,
    pub feedrate_mm_s: Option<f64>,
    pub tool: u32,
}

impl ModalState {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            e: 0.0,
            feedrate_mm_s: None,
            tool: 0,
        }
    }
}

/// Internal reduce-output events. `pipeline` consumes these.
#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub(crate) enum ReduceEvent {
    G1Move {
        from: [f64; 3],
        to: [f64; 3],
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    },
    Arc {
        start: [f64; 3],
        end: [f64; 3],
        center: [f64; 3],
        clockwise: bool,
        z_delta: f64,
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    },
    Marker {
        kind: MotionMarkerKind,
        line_no: u32,
        /// For T-codes, the tool number from the command's `major` field.
        tool: Option<u32>,
        /// For E-only G1 markers, the signed E delta (mm).
        e_delta_mm: Option<f64>,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(dead_code)]
pub(crate) enum MotionMarkerKind {
    /// G0 (rapid travel)
    G0,
    /// G1 with no X/Y change (Z-only or E-only move)
    ZOnly,
    /// G1 with E delta but no XY motion (retract / unretract)
    EOnly,
    /// G92 (set position)
    G92,
    /// M-code
    M,
    /// T-code (tool change)
    T,
    /// End of input
    EndOfFile,
}

#[cfg(test)]
#[allow(unused_imports)]
pub use tests::*;  // expose internal types to integration tests if needed

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modal_state_initializes_at_origin() {
        let st = ModalState::new();
        #[allow(clippy::float_cmp)]
        {
            assert_eq!(st.position, [0.0, 0.0, 0.0]);
        }
        assert_eq!(st.feedrate_mm_s, None);
        assert_eq!(st.tool, 0);
    }

    #[test]
    #[allow(clippy::no_effect_underscore_binding)]
    fn reduce_event_variants_construct() {
        let _e1 = ReduceEvent::G1Move {
            from: [0.0, 0.0, 0.0],
            to: [1.0, 0.0, 0.0],
            e_delta: Some(0.05),
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
        let _e2 = ReduceEvent::Arc {
            start: [0.0, 0.0, 0.0],
            end: [1.0, 0.0, 0.0],
            center: [0.5, -0.5, 0.0],
            clockwise: true,
            z_delta: 0.0,
            e_delta: Some(0.05),
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
        let _e3 = ReduceEvent::Marker {
            kind: MotionMarkerKind::ZOnly,
            line_no: 5,
            tool: None,
            e_delta_mm: None,
        };
    }
}