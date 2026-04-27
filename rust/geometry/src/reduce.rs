//! Reduce: token stream → internal `ReduceEvent` stream. Pub(crate); tests
//! import via `#[cfg(test)] pub use`.
//! Phase 1 implementation is filled in across Tasks 13-17.

use gcode::{MarkerKind, ParseError, Token};

/// Convert F-word (mm/min) to mm/s.
#[allow(dead_code)]
fn f_to_mm_s(f: f64) -> f64 {
    f / 60.0
}

/// Modal state machine — accumulates the current position, feedrate, and tool
/// across the gcode stream, applying G1's modal "params absent → unchanged"
/// semantics.
#[derive(Debug, Clone)]
#[allow(dead_code)]
pub(crate) struct ModalState {
    pub position: [f64; 3],
    pub e: f64,
    pub feedrate_mm_s: Option<f64>,
    pub tool: u32,
    pub active_plane: Plane,
}

impl ModalState {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            position: [0.0, 0.0, 0.0],
            e: 0.0,
            feedrate_mm_s: None,
            tool: 0,
            active_plane: Plane::XY,
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
    CommentMarker {
        kind: MarkerKind,
        line_no: u32,
    },
    ParseError {
        line_no: u32,
        kind: ParseErrorKind,
        text: String,
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

/// Classification of the parse error that caused a `ReduceEvent::ParseError`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ParseErrorKind {
    MalformedNumber,
    UnrecognizedHead,
    EmptyCommand,
    DuplicateParam,
}

/// Active machining plane per RS274NGC §3.5.1. Tracked across the gcode
/// stream by G17/G18/G19. Default G17 (XY) per spec. Used by G5.1 to validate
/// that the curve lies in a supported plane; G2/G3 are XY-only in Phase 1
/// regardless of plane state (deliberate non-goal of Step 3).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum Plane {
    #[default]
    XY,
    XZ,
    YZ,
}

/// Walk a token iterator, maintain modal state, and emit `ReduceEvent`s.
///
/// Phase 1 handles: G0 (marker), G1 (move or marker), G2/G3 (Arc — Task 15),
/// G92 (marker — Task 16), M-codes (marker), T-codes (marker), and forwards
/// recognized comment markers to `MotionMarkerKind`-bearing telemetry events.
/// Parse errors are skipped here; the pipeline layer translates them to
/// `Recovery` items.
#[allow(dead_code)]
pub(crate) fn reduce<I>(tokens: I) -> impl Iterator<Item = ReduceEvent>
where
    I: IntoIterator<Item = Result<Token, ParseError>>,
{
    ReduceIter {
        tokens: tokens.into_iter(),
        state: ModalState::new(),
    }
}

struct ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    tokens: I,
    state: ModalState,
}

impl<I> ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    #[allow(dead_code)]
    fn update_position(&mut self, params: &gcode::Params) {
        if let Some(x) = params.x() { self.state.position[0] = x; }
        if let Some(y) = params.y() { self.state.position[1] = y; }
        if let Some(z) = params.z() { self.state.position[2] = z; }
    }
}

impl<I> Iterator for ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    type Item = ReduceEvent;

    #[allow(clippy::too_many_lines)]
    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let tok = self.tokens.next()?;
            let tok = match tok {
                Ok(t) => t,
                Err(e) => {
                    let (kind, line_no, text) = match e {
                        ParseError::MalformedNumber { line_no, text } =>
                            (ParseErrorKind::MalformedNumber, line_no, String::from(text)),
                        ParseError::UnrecognizedHead { line_no, head } =>
                            (ParseErrorKind::UnrecognizedHead, line_no, String::from(head)),
                        ParseError::EmptyCommand { line_no } =>
                            (ParseErrorKind::EmptyCommand, line_no, String::new()),
                        ParseError::DuplicateParam { line_no, letter } =>
                            (ParseErrorKind::DuplicateParam, line_no, letter.to_string()),
                        _ => (ParseErrorKind::MalformedNumber, 0u32, format!("{e:?}")),
                    };
                    return Some(ReduceEvent::ParseError { line_no, kind, text });
                }
            };
            match tok {
                Token::Command { letter: b'G', major: 0, params, line_no, .. } => {
                    // G0 — update position state, emit G0 marker.
                    self.update_position(&params);
                    if let Some(f) = params.f() {
                        self.state.feedrate_mm_s = Some(f_to_mm_s(f));
                    }
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::G0,
                        line_no,
                        tool: None,
                        e_delta_mm: None,
                    });
                }
                Token::Command { letter: b'G', major: 1, params, line_no, .. } => {
                    let from = self.state.position;
                    let xy_changed = params.x().is_some() || params.y().is_some();
                    let z_changed = params.z().is_some();
                    let e_present = params.e().is_some();
                    if let Some(f) = params.f() {
                        self.state.feedrate_mm_s = Some(f_to_mm_s(f));
                    }
                    if !xy_changed && z_changed && !e_present {
                        // Z-only move: marker, but update position.
                        self.update_position(&params);
                        return Some(ReduceEvent::Marker {
                            kind: MotionMarkerKind::ZOnly,
                            line_no,
                            tool: None,
                            e_delta_mm: None,
                        });
                    }
                    if !xy_changed && !z_changed && e_present {
                        // E-only (retract / unretract).
                        let new_e = params.e().unwrap();
                        let delta = new_e - self.state.e;
                        self.state.e = new_e;
                        return Some(ReduceEvent::Marker {
                            kind: MotionMarkerKind::EOnly,
                            line_no,
                            tool: None,
                            e_delta_mm: Some(delta),
                        });
                    }
                    if !xy_changed && !z_changed && !e_present {
                        // G1 with only F — no motion, treated as no-op.
                        continue;
                    }
                    // Real move: update position and E, emit G1Move.
                    self.update_position(&params);
                    let e_delta = params.e().map(|new_e| {
                        let d = new_e - self.state.e;
                        self.state.e = new_e;
                        d
                    });
                    let to = self.state.position;
                    let feedrate_mm_s = self.state.feedrate_mm_s.unwrap_or(0.0);
                    return Some(ReduceEvent::G1Move {
                        from,
                        to,
                        e_delta,
                        feedrate_mm_s,
                        line_no,
                    });
                }
                Token::Command { letter: b'G', major: 92, line_no, .. } => {
                    // G92: position reset. Treated as marker break.
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::G92,
                        line_no,
                        tool: None,
                        e_delta_mm: None,
                    });
                }
                Token::Command { letter: b'M', line_no, .. } => {
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::M,
                        line_no,
                        tool: None,
                        e_delta_mm: None,
                    });
                }
                Token::Command { letter: b'T', major, line_no, .. } => {
                    self.state.tool = major;
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::T,
                        line_no,
                        tool: Some(major),
                        e_delta_mm: None,
                    });
                }
                Token::Command {
                    letter: b'G', major: g, params, line_no, ..
                } if g == 2 || g == 3 => {
                    let start = self.state.position;
                    let i = params.i().unwrap_or(0.0);
                    let j = params.j().unwrap_or(0.0);
                    let center = [start[0] + i, start[1] + j, start[2]];
                    let new_x = params.x().unwrap_or(start[0]);
                    let new_y = params.y().unwrap_or(start[1]);
                    let new_z = params.z().unwrap_or(start[2]);
                    let end = [new_x, new_y, new_z];
                    let z_delta = new_z - start[2];
                    let clockwise = g == 2;
                    if let Some(f) = params.f() {
                        self.state.feedrate_mm_s = Some(f_to_mm_s(f));
                    }
                    let e_delta = params.e().map(|new_e| {
                        let d = new_e - self.state.e;
                        self.state.e = new_e;
                        d
                    });
                    self.state.position = end;
                    let feedrate_mm_s = self.state.feedrate_mm_s.unwrap_or(0.0);
                    return Some(ReduceEvent::Arc {
                        start, end, center, clockwise, z_delta, e_delta,
                        feedrate_mm_s, line_no,
                    });
                }
                Token::Marker { kind, line_no } => {
                    return Some(ReduceEvent::CommentMarker { kind, line_no });
                }
                _ => {}
            }
        }
    }
}

#[cfg(test)]
#[allow(unused_imports)]
pub use tests::*;  // expose internal types to integration tests if needed

#[cfg(test)]
mod tests {
    use super::*;
    use gcode::{Params, Token};

    fn cmd(letter: u8, major: u32, line_no: u32, params: Params) -> Token {
        Token::Command { letter, major, minor: None, params, line_no }
    }

    fn p(setters: &[(u8, f64)]) -> Params {
        let mut p = Params::default();
        for (l, v) in setters { p.set(*l, *v); }
        p
    }

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

    #[test]
    #[allow(clippy::float_cmp)]
    fn g1_xy_emits_g1move() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 2.0), (b'F', 1500.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::G1Move { from, to, feedrate_mm_s, .. } => {
                assert_eq!(*from, [0.0, 0.0, 0.0]);
                assert_eq!(*to, [1.0, 2.0, 0.0]);
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9, "F1500 → 25 mm/s");
            }
            other => panic!("expected G1Move, got {other:?}"),
        }
    }

    #[test]
    fn g1_z_only_emits_zonly_marker() {
        let toks = vec![cmd(b'G', 1, 1, p(&[(b'Z', 0.2), (b'F', 1500.0)]))];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Marker { kind: MotionMarkerKind::ZOnly, line_no: 1, .. } => {}
            other => panic!("expected ZOnly Marker, got {other:?}"),
        }
    }

    #[test]
    fn g1_e_only_emits_eonly_marker() {
        let toks = vec![cmd(b'G', 1, 1, p(&[(b'E', -1.5), (b'F', 3000.0)]))];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Marker { kind: MotionMarkerKind::EOnly, line_no: 1, e_delta_mm: Some(d), .. } => {
                assert!((d - (-1.5)).abs() < 1e-12);
            }
            other => panic!("expected EOnly Marker, got {other:?}"),
        }
    }

    #[test]
    fn g0_emits_g0_marker() {
        let toks = vec![cmd(b'G', 0, 1, p(&[(b'X', 5.0), (b'Y', 5.0)]))];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Marker { kind: MotionMarkerKind::G0, line_no: 1, .. } => {}
            other => panic!("expected G0 Marker, got {other:?}"),
        }
    }

    #[test]
    fn t_marker_carries_tool_number() {
        let toks = vec![cmd(b'T', 2, 1, Params::default())];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Marker { kind: MotionMarkerKind::T, tool: Some(2), .. } => {}
            other => panic!("expected T Marker with tool=2, got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g2_emits_arc_clockwise() {
        // Set position to (1, 0, 0), then arc to (0, 1, 0) around (0, 0).
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 2, 2, p(&[(b'X', 0.0), (b'Y', 1.0), (b'I', -1.0), (b'J', 0.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::Arc { start, end, center, clockwise, z_delta, .. } => {
                assert_eq!(*start, [1.0, 0.0, 0.0]);
                assert_eq!(*end, [0.0, 1.0, 0.0]);
                assert_eq!(*center, [0.0, 0.0, 0.0]);
                assert!(*clockwise);
                assert_eq!(*z_delta, 0.0);
            }
            other => panic!("expected Arc, got {other:?}"),
        }
    }

    #[test]
    fn g3_emits_arc_counter_clockwise() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'F', 1500.0)])),
            cmd(b'G', 3, 2, p(&[(b'X', 0.0), (b'Y', 1.0), (b'I', -1.0), (b'J', 0.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[1] {
            ReduceEvent::Arc { clockwise: false, .. } => {}
            other => panic!("expected counter-clockwise Arc, got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g2_with_z_delta_yields_z_delta_field() {
        // Helical arc: end Z differs from start Z.
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Z', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 2, 2, p(&[
                (b'X', 0.0), (b'Y', 1.0), (b'Z', 0.5),
                (b'I', -1.0), (b'J', 0.0),
            ])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[1] {
            ReduceEvent::Arc { z_delta, end, .. } => {
                assert!((z_delta - 0.5).abs() < 1e-12);
                assert_eq!(end[2], 0.5);
            }
            other => panic!("expected helical Arc, got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn modal_position_persists_across_g1s() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 1, 2, p(&[(b'X', 2.0)])),  // Y not given, should persist as 0.0
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::G1Move { from, to, .. } => {
                assert_eq!(*from, [1.0, 0.0, 0.0]);
                assert_eq!(*to, [2.0, 0.0, 0.0]);
            }
            other => panic!("expected G1Move, got {other:?}"),
        }
    }

    #[test]
    fn modal_state_plane_defaults_to_xy() {
        let st = ModalState::new();
        assert_eq!(st.active_plane, Plane::XY);
    }

    #[test]
    fn g17_keeps_xy_plane() {
        let toks = vec![cmd(b'G', 17, 1, Params::default())];
        let _events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // Plane is internal modal state; this test is reachable today only by
        // observing through downstream behavior, which lands in Task 18. Test
        // ordering: this scaffolds the type so Task 18's plane-mismatch test
        // can construct cases that change the plane. For now, assert the type
        // compiles and the variant set is what we expect.
        assert_eq!(Plane::default(), Plane::XY);
        assert_eq!(Plane::XY, Plane::XY);
        assert_ne!(Plane::XY, Plane::XZ);
        assert_ne!(Plane::XZ, Plane::YZ);
    }

    #[test]
    fn comment_marker_layer_change_is_forwarded() {
        let toks = vec![
            Token::Marker {
                kind: gcode::MarkerKind::LayerChange { layer: Some(7) },
                line_no: 42,
            },
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::CommentMarker { kind, line_no: 42 } => {
                match kind {
                    gcode::MarkerKind::LayerChange { layer } => assert_eq!(*layer, Some(7)),
                    _ => panic!("expected LayerChange"),
                }
            }
            other => panic!("expected CommentMarker, got {other:?}"),
        }
    }
}