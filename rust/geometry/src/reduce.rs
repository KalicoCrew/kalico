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
    /// (P, Q) of the previous G5 segment, or `None` if the previous motion
    /// was not G5 (or no motion has occurred). Carried across an
    /// uninterrupted G5→G5 chain to support the RS274NGC §3.5.5 implicit
    /// next-tangent rule (I, J default to `−prev_pq` componentwise).
    /// **Cleared by every motion-producing g-code other than G5** (G0, G1,
    /// G2, G3, G5.1). Plane selects (G17/G18/G19), M-codes, and T-codes do
    /// **not** clear it — they don't move the machine.
    pub prev_g5_pq: Option<[f64; 2]>,
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
            prev_g5_pq: None,
        }
    }
}

/// Geometry payload of a `ReduceEvent::Curve`. Each variant carries its
/// control points as a fixed-size array — zero per-segment heap allocation,
/// type-level enforcement of the correct CP count for each variant.
///
/// **Variant choice is by source g-code semantics**, not by mathematical
/// class: G5.1 (`Quadratic`, non-rational) is distinct from G2/G3
/// (`RationalQuadratic`) at this layer, so consuming code that handles them
/// differently does not need to inspect `Option<weights>`.
///
/// Future G6.2 NURBS would add a single `Nurbs { cps: SmallVec<…>, weights:
/// Option<…>, knots: SmallVec<…>, degree: u8 }` variant; the outer
/// `ReduceEvent::Curve(_, _)` arm doesn't change.
#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub(crate) enum CurveGeom {
    /// Degree-2 non-rational Bézier. G5.1 lands here.
    Quadratic { cps: [[f64; 3]; 3] },
    /// Degree-3 non-rational Bézier. G5 lands here.
    Cubic { cps: [[f64; 3]; 4] },
}

/// Internal reduce-output events. `pipeline` consumes these.
#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub(crate) enum ReduceEvent {
    /// Any curve segment (line, conic, cubic, future NURBS). The geometry
    /// payload is in the `CurveGeom`; common motion-event fields are inline.
    Curve {
        geom: CurveGeom,
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
    /// G5 line missing both I,J with no previous G5 in modal chain.
    G5MissingTangent,
    /// G5.1 line with the active plane (G17/G18/G19) different from G17 (XY).
    /// The active plane is encoded in `text` as the literal G-code number
    /// ("18" or "19"); pipeline parses it back to populate Recovery.
    G5PlaneMismatch,
    /// G5/G5.1 with malformed I,J,P,Q (e.g. only I but not J, both zero on
    /// G5.1, etc.). Surfaced as `MalformedParams` equivalent but with G5
    /// context; pipeline maps to `Recovery::MalformedParams`.
    G5MalformedTangent,
    /// Live pipeline received G0/G1/G2/G3 — these are not part of the
    /// G5-only live pipeline. Caller (Step-13 compat layer) is expected
    /// to normalize them to G5 offline before feeding the live pipeline.
    /// `kind` is `"G0/G1"` or `"G2/G3"`.
    UnsupportedGcode {
        kind: &'static str,
    },
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

/// Test-only variant of `reduce` that takes a mutable `ModalState` reference,
/// allowing tests to inspect modal state after the iterator drains. Identical
/// to `reduce` otherwise; not exposed outside `#[cfg(test)]`.
#[cfg(test)]
#[allow(dead_code)]
pub(crate) fn reduce_with_state<'a, I>(
    state: &'a mut ModalState,
    tokens: I,
) -> impl Iterator<Item = ReduceEvent> + 'a
where
    I: IntoIterator<Item = Result<Token, ParseError>> + 'a,
    I::IntoIter: 'a,
{
    ReduceIterRef {
        tokens: tokens.into_iter(),
        state,
    }
}

#[cfg(test)]
struct ReduceIterRef<'a, I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    tokens: I,
    state: &'a mut ModalState,
}

#[cfg(test)]
impl<I> Iterator for ReduceIterRef<'_, I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    type Item = ReduceEvent;

    fn next(&mut self) -> Option<Self::Item> {
        next_event(&mut self.tokens, self.state)
    }
}

struct ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    tokens: I,
    state: ModalState,
}

impl<I> Iterator for ReduceIter<I>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    type Item = ReduceEvent;

    fn next(&mut self) -> Option<Self::Item> {
        next_event(&mut self.tokens, &mut self.state)
    }
}

/// Pull the next reduce-output event from the token stream, mutating modal
/// state in place. Shared between `ReduceIter` (production) and
/// `ReduceIterRef` (tests). Logic is identical to the original
/// `ReduceIter::next` body.
#[allow(clippy::too_many_lines, clippy::needless_continue)]
fn next_event<I>(tokens: &mut I, state: &mut ModalState) -> Option<ReduceEvent>
where
    I: Iterator<Item = Result<Token, ParseError>>,
{
    loop {
        let tok = tokens.next()?;
        let tok = match tok {
            Ok(t) => t,
            Err(e) => {
                let (kind, line_no, text) = match e {
                    ParseError::MalformedNumber { line_no, text } => {
                        (ParseErrorKind::MalformedNumber, line_no, String::from(text))
                    }
                    ParseError::UnrecognizedHead { line_no, head } => (
                        ParseErrorKind::UnrecognizedHead,
                        line_no,
                        String::from(head),
                    ),
                    ParseError::EmptyCommand { line_no } => {
                        (ParseErrorKind::EmptyCommand, line_no, String::new())
                    }
                    ParseError::DuplicateParam { line_no, letter } => {
                        (ParseErrorKind::DuplicateParam, line_no, letter.to_string())
                    }
                    _ => (ParseErrorKind::MalformedNumber, 0u32, format!("{e:?}")),
                };
                return Some(ReduceEvent::ParseError {
                    line_no,
                    kind,
                    text,
                });
            }
        };
        match tok {
            Token::Command {
                letter: b'G',
                major: 0,
                line_no,
                ..
            } => {
                // Live pipeline rejects G0; Step-13 compat layer must
                // normalize legacy g-code to G5 before it arrives here.
                state.prev_g5_pq = None;
                return Some(ReduceEvent::ParseError {
                    line_no,
                    kind: ParseErrorKind::UnsupportedGcode { kind: "G0/G1" },
                    text: String::new(),
                });
            }
            Token::Command {
                letter: b'G',
                major: 1,
                line_no,
                ..
            } => {
                // Live pipeline rejects G1 (Z-only / E-only / no-op /
                // real-XY-move all collapse to the same rejection here).
                state.prev_g5_pq = None;
                return Some(ReduceEvent::ParseError {
                    line_no,
                    kind: ParseErrorKind::UnsupportedGcode { kind: "G0/G1" },
                    text: String::new(),
                });
            }
            Token::Command {
                letter: b'G',
                major: 92,
                params,
                line_no,
                ..
            } => {
                // G92: position reset (RS274NGC §3.5.1 / RepRap). Sets the
                // current modal position/E to the specified words without
                // producing motion. Omitted axes are unchanged. Marker break:
                // the coordinate-frame redefinition makes any pending
                // `prev_g5_pq` deltas (expressed in the prior frame)
                // semantically stale — chain-clear per spec §3.5 clearing
                // discipline.
                if let Some(x) = params.x() {
                    state.position[0] = x;
                }
                if let Some(y) = params.y() {
                    state.position[1] = y;
                }
                if let Some(z) = params.z() {
                    state.position[2] = z;
                }
                if let Some(e) = params.e() {
                    state.e = e;
                }
                state.prev_g5_pq = None;
                return Some(ReduceEvent::Marker {
                    kind: MotionMarkerKind::G92,
                    line_no,
                    tool: None,
                    e_delta_mm: None,
                });
            }
            Token::Command {
                letter: b'M',
                line_no,
                ..
            } => {
                return Some(ReduceEvent::Marker {
                    kind: MotionMarkerKind::M,
                    line_no,
                    tool: None,
                    e_delta_mm: None,
                });
            }
            Token::Command {
                letter: b'T',
                major,
                line_no,
                ..
            } => {
                state.tool = major;
                return Some(ReduceEvent::Marker {
                    kind: MotionMarkerKind::T,
                    line_no,
                    tool: Some(major),
                    e_delta_mm: None,
                });
            }
            // G5: cubic Bézier with control points P0=current, P1=current+(I,J),
            // P2=end+(P,Q), P3=end. Per LinuxCNC RS274NGC §3.5.5.
            // Distinguished from G5.1 by the absence of `minor`.
            Token::Command {
                letter: b'G',
                major: 5,
                minor: None,
                params,
                line_no,
                ..
            } => {
                // Plane check (Phase 1: XY only). Defense-in-depth — modern
                // FDM slicers don't emit G18/G19, but a stray plane-select
                // followed by G5 would otherwise produce a curve interpreted
                // as XY when the user meant XZ/YZ. Mirror G5.1's behavior.
                if state.active_plane != Plane::XY {
                    let plane_g_code = match state.active_plane {
                        Plane::XY => 17,
                        Plane::XZ => 18,
                        Plane::YZ => 19,
                    };
                    state.prev_g5_pq = None;
                    return Some(ReduceEvent::ParseError {
                        line_no,
                        kind: ParseErrorKind::G5PlaneMismatch,
                        text: plane_g_code.to_string(),
                    });
                }

                let p0 = state.position;

                // Resolve I,J: explicit if present, modal-chain rule if both
                // absent and prev_g5_pq is set, error otherwise.
                let (i, j) = match (params.i(), params.j(), state.prev_g5_pq) {
                    (Some(i), Some(j), _) => (i, j),
                    (None, None, Some([prev_p, prev_q])) => (-prev_p, -prev_q),
                    (None, None, None) => {
                        state.prev_g5_pq = None; // already None, but explicit for symmetry
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MissingTangent,
                            text: String::new(),
                        });
                    }
                    _ => {
                        let i_present = params.i().is_some();
                        let j_present = params.j().is_some();
                        state.prev_g5_pq = None;
                        // Single I or single J specified — invalid per LinuxCNC.
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: format!(
                                "G5: I and J must both be specified or both omitted (i_present={i_present}, j_present={j_present})"
                            ),
                        });
                    }
                };

                // P, Q are required and explicit on every G5.
                let (Some(pp), Some(qq)) = (params.p(), params.q()) else {
                    state.prev_g5_pq = None;
                    return Some(ReduceEvent::ParseError {
                        line_no,
                        kind: ParseErrorKind::G5MalformedTangent,
                        text: format!(
                            "G5: P and Q are required (got p={:?}, q={:?})",
                            params.p(),
                            params.q()
                        ),
                    });
                };

                // End position: X/Y/Z modal — inherit from current position
                // for any axis not specified.
                let new_x = params.x().unwrap_or(p0[0]);
                let new_y = params.y().unwrap_or(p0[1]);
                let new_z = params.z().unwrap_or(p0[2]);
                let p3 = [new_x, new_y, new_z];

                // Z linearly interpolated across the four control points so
                // the curve remains exactly the planar cubic Bézier in XY
                // and linear in Z. Spacing 0, ⅓, ⅔, 1 along the parameter.
                let dz = p3[2] - p0[2];
                let p1 = [p0[0] + i, p0[1] + j, p0[2] + dz / 3.0];
                let p2 = [p3[0] + pp, p3[1] + qq, p0[2] + 2.0 * dz / 3.0];

                // Feedrate update.
                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                let e_delta = params.e().map(|new_e| {
                    let d = new_e - state.e;
                    state.e = new_e;
                    d
                });

                // State updates: position, prev_g5_pq for the next link.
                state.position = p3;
                state.prev_g5_pq = Some([pp, qq]);

                let feedrate_mm_s = state.feedrate_mm_s.unwrap_or(0.0);
                return Some(ReduceEvent::Curve {
                    geom: CurveGeom::Cubic {
                        cps: [p0, p1, p2, p3],
                    },
                    e_delta,
                    feedrate_mm_s,
                    line_no,
                });
            }
            // G5.1: quadratic Bézier with control points P0=current,
            // P1=current+(I,J), P2=end. Per LinuxCNC RS274NGC §G5.1.
            // Restricted to the active plane (G17/G18/G19); Phase 1 supports
            // only XY (G17). Both I and J must be specified and at least one
            // must be non-zero (a fully-zero tangent collapses to G1).
            Token::Command {
                letter: b'G',
                major: 5,
                minor: Some(1),
                params,
                line_no,
                ..
            } => {
                // Plane check (Phase 1: XY only).
                if state.active_plane != Plane::XY {
                    let plane_g_code = match state.active_plane {
                        Plane::XY => 17,
                        Plane::XZ => 18,
                        Plane::YZ => 19,
                    };
                    state.prev_g5_pq = None;
                    return Some(ReduceEvent::ParseError {
                        line_no,
                        kind: ParseErrorKind::G5PlaneMismatch,
                        text: plane_g_code.to_string(),
                    });
                }

                // I,J both required and at least one non-zero.
                let (i, j) = match (params.i(), params.j()) {
                    (Some(i), Some(j)) if i != 0.0 || j != 0.0 => (i, j),
                    (Some(_), Some(_)) => {
                        // Both zero — degenerate, equivalent to G1.
                        state.prev_g5_pq = None;
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: "G5.1: I and J both zero".to_string(),
                        });
                    }
                    _ => {
                        state.prev_g5_pq = None;
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: format!(
                                "G5.1: both I and J required (got i={:?}, j={:?})",
                                params.i(),
                                params.j()
                            ),
                        });
                    }
                };

                let p0 = state.position;
                let new_x = params.x().unwrap_or(p0[0]);
                let new_y = params.y().unwrap_or(p0[1]);
                let new_z = params.z().unwrap_or(p0[2]);
                let p2 = [new_x, new_y, new_z];

                // Z linearly interpolated across 3 control points: 0, ½, 1.
                let z1 = f64::midpoint(p0[2], p2[2]);
                let p1 = [p0[0] + i, p0[1] + j, z1];

                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                let e_delta = params.e().map(|new_e| {
                    let d = new_e - state.e;
                    state.e = new_e;
                    d
                });

                state.position = p2;
                // G5.1 is non-G5 motion: clear the modal chain.
                state.prev_g5_pq = None;

                let feedrate_mm_s = state.feedrate_mm_s.unwrap_or(0.0);
                return Some(ReduceEvent::Curve {
                    geom: CurveGeom::Quadratic { cps: [p0, p1, p2] },
                    e_delta,
                    feedrate_mm_s,
                    line_no,
                });
            }
            Token::Command {
                letter: b'G',
                major: g,
                line_no,
                ..
            } if g == 2 || g == 3 => {
                state.prev_g5_pq = None;
                return Some(ReduceEvent::ParseError {
                    line_no,
                    kind: ParseErrorKind::UnsupportedGcode { kind: "G2/G3" },
                    text: String::new(),
                });
            }
            Token::Command {
                letter: b'G',
                major: 17,
                ..
            } => {
                state.active_plane = Plane::XY;
                continue;
            }
            Token::Command {
                letter: b'G',
                major: 18,
                ..
            } => {
                state.active_plane = Plane::XZ;
                continue;
            }
            Token::Command {
                letter: b'G',
                major: 19,
                ..
            } => {
                state.active_plane = Plane::YZ;
                continue;
            }
            Token::Marker { kind, line_no } => {
                return Some(ReduceEvent::CommentMarker { kind, line_no });
            }
            _ => {}
        }
    }
}

#[cfg(test)]
#[allow(unused_imports)]
pub use tests::*; // expose internal types to integration tests if needed

#[cfg(test)]
mod tests;
