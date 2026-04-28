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
    /// Degree-1 line segment. G0 (when promoted) and G1 land here.
    Linear { cps: [[f64; 3]; 2] },
    /// Degree-2 non-rational Bézier. G5.1 lands here.
    Quadratic { cps: [[f64; 3]; 3] },
    /// Degree-2 rational Bézier (NURBS with weights). G2/G3 land here.
    RationalQuadratic {
        cps: [[f64; 3]; 3],
        weights: [f64; 3],
    },
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

#[allow(dead_code)]
fn update_position_in(state: &mut ModalState, params: &gcode::Params) {
    if let Some(x) = params.x() {
        state.position[0] = x;
    }
    if let Some(y) = params.y() {
        state.position[1] = y;
    }
    if let Some(z) = params.z() {
        state.position[2] = z;
    }
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
                params,
                line_no,
                ..
            } => {
                // G0 — update position state, emit G0 marker.
                update_position_in(state, &params);
                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                state.prev_g5_pq = None;
                return Some(ReduceEvent::Marker {
                    kind: MotionMarkerKind::G0,
                    line_no,
                    tool: None,
                    e_delta_mm: None,
                });
            }
            Token::Command {
                letter: b'G',
                major: 1,
                params,
                line_no,
                ..
            } => {
                let from = state.position;
                let xy_changed = params.x().is_some() || params.y().is_some();
                let z_changed = params.z().is_some();
                let e_present = params.e().is_some();
                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                if !xy_changed && z_changed && !e_present {
                    // Z-only move: marker, but update position.
                    update_position_in(state, &params);
                    state.prev_g5_pq = None;
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
                    let delta = new_e - state.e;
                    state.e = new_e;
                    state.prev_g5_pq = None;
                    return Some(ReduceEvent::Marker {
                        kind: MotionMarkerKind::EOnly,
                        line_no,
                        tool: None,
                        e_delta_mm: Some(delta),
                    });
                }
                if !xy_changed && !z_changed && !e_present {
                    // F-only no-op: no motion, no chain break.
                    continue;
                }
                // Real move: update position and E, emit Curve(Linear).
                update_position_in(state, &params);
                let e_delta = params.e().map(|new_e| {
                    let d = new_e - state.e;
                    state.e = new_e;
                    d
                });
                let to = state.position;
                let feedrate_mm_s = state.feedrate_mm_s.unwrap_or(0.0);
                // G1 clears the G5 modal-chain tangent — non-G5 motion.
                state.prev_g5_pq = None;
                return Some(ReduceEvent::Curve {
                    geom: CurveGeom::Linear { cps: [from, to] },
                    e_delta,
                    feedrate_mm_s,
                    line_no,
                });
            }
            Token::Command {
                letter: b'G',
                major: 92,
                line_no,
                ..
            } => {
                // G92: position reset. Treated as marker break. G92
                // redefines the coordinate frame, so any pending
                // `prev_g5_pq` deltas (expressed in the prior frame)
                // become semantically stale — clear the chain per
                // spec §3.5 clearing-discipline table.
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
                params,
                line_no,
                ..
            } if g == 2 || g == 3 => {
                let start = state.position;
                let i = params.i().unwrap_or(0.0);
                let j = params.j().unwrap_or(0.0);
                let center = [start[0] + i, start[1] + j, start[2]];
                let new_x = params.x().unwrap_or(start[0]);
                let new_y = params.y().unwrap_or(start[1]);
                let new_z = params.z().unwrap_or(start[2]);
                let end = [new_x, new_y, new_z];
                let clockwise = g == 2;
                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                let e_delta = params.e().map(|new_e| {
                    let d = new_e - state.e;
                    state.e = new_e;
                    d
                });
                state.position = end;
                state.prev_g5_pq = None; // arcs are non-G5 motion.
                let feedrate_mm_s = state.feedrate_mm_s.unwrap_or(0.0);
                return Some(ReduceEvent::Curve {
                    geom: build_arc_curve(start, end, center, clockwise),
                    e_delta,
                    feedrate_mm_s,
                    line_no,
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

/// Build the rational-quadratic-Bézier control points and weights for an arc
/// in 3D. Z is interpolated linearly across the 3 control points (helical
/// support); the rational-quadratic geometry follows Piegl & Tiller §7.2.
///
/// **Phase 1 limitation** (preserved from the original `pipeline::build_arc_nurbs`):
/// |sweep| < π required; sweeps ≥ π are clamped to (π − ε) so `cos(half_sweep)`
/// stays positive. Multi-piece exact representation for full circles is a
/// Phase 2 item.
fn build_arc_curve(start: [f64; 3], end: [f64; 3], center: [f64; 3], clockwise: bool) -> CurveGeom {
    const MAX_SWEEP: f64 = std::f64::consts::PI * (1.0 - 1e-9);

    let r_start = [start[0] - center[0], start[1] - center[1]];
    let radius = (r_start[0] * r_start[0] + r_start[1] * r_start[1]).sqrt();
    let start_angle = r_start[1].atan2(r_start[0]);
    let r_end = [end[0] - center[0], end[1] - center[1]];
    let end_angle = r_end[1].atan2(r_end[0]);

    let sweep = if clockwise {
        let mut s = end_angle - start_angle;
        if s < 0.0 {
            s += 2.0 * std::f64::consts::PI;
        }
        s
    } else {
        let mut s = start_angle - end_angle;
        if s < 0.0 {
            s += 2.0 * std::f64::consts::PI;
        }
        -s
    };
    let sweep = sweep.clamp(-MAX_SWEEP, MAX_SWEEP);

    let half = sweep / 2.0;
    let cos_half = half.cos();
    let mid_x = center[0] + radius * (start_angle + half).cos() / cos_half;
    let mid_y = center[1] + radius * (start_angle + half).sin() / cos_half;

    let z0 = start[2];
    let z2 = end[2];
    let z1 = f64::midpoint(z0, z2);

    CurveGeom::RationalQuadratic {
        cps: [start, [mid_x, mid_y, z1], end],
        weights: [1.0, cos_half, 1.0],
    }
}

#[cfg(test)]
#[allow(unused_imports)]
pub use tests::*; // expose internal types to integration tests if needed

#[cfg(test)]
mod tests {
    use super::*;
    use gcode::{Params, Token};

    fn cmd(letter: u8, major: u32, line_no: u32, params: Params) -> Token {
        Token::Command {
            letter,
            major,
            minor: None,
            params,
            line_no,
        }
    }

    fn cmd_with_minor(
        letter: u8,
        major: u32,
        minor: Option<u32>,
        line_no: u32,
        params: Params,
    ) -> Token {
        Token::Command {
            letter,
            major,
            minor,
            params,
            line_no,
        }
    }

    fn p(setters: &[(u8, f64)]) -> Params {
        let mut p = Params::default();
        for (l, v) in setters {
            p.set(*l, *v);
        }
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
        let _e1 = ReduceEvent::Curve {
            geom: CurveGeom::Linear {
                cps: [[0.0; 3], [1.0, 0.0, 0.0]],
            },
            e_delta: Some(0.05),
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
        let _e2 = ReduceEvent::Curve {
            geom: CurveGeom::RationalQuadratic {
                cps: [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
                weights: [1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
            },
            e_delta: None,
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
    fn g1_xy_emits_curve_linear() {
        let toks = vec![cmd(
            b'G',
            1,
            1,
            p(&[(b'X', 1.0), (b'Y', 2.0), (b'F', 1500.0)]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Linear { cps },
                feedrate_mm_s,
                line_no: 1,
                ..
            } => {
                assert_eq!(cps[0], [0.0, 0.0, 0.0]);
                assert_eq!(cps[1], [1.0, 2.0, 0.0]);
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9, "F1500 → 25 mm/s");
            }
            other => panic!("expected Curve(Linear), got {other:?}"),
        }
    }

    #[test]
    fn g1_z_only_emits_zonly_marker() {
        let toks = vec![cmd(b'G', 1, 1, p(&[(b'Z', 0.2), (b'F', 1500.0)]))];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Marker {
                kind: MotionMarkerKind::ZOnly,
                line_no: 1,
                ..
            } => {}
            other => panic!("expected ZOnly Marker, got {other:?}"),
        }
    }

    #[test]
    fn g1_e_only_emits_eonly_marker() {
        let toks = vec![cmd(b'G', 1, 1, p(&[(b'E', -1.5), (b'F', 3000.0)]))];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Marker {
                kind: MotionMarkerKind::EOnly,
                line_no: 1,
                e_delta_mm: Some(d),
                ..
            } => {
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
            ReduceEvent::Marker {
                kind: MotionMarkerKind::G0,
                line_no: 1,
                ..
            } => {}
            other => panic!("expected G0 Marker, got {other:?}"),
        }
    }

    #[test]
    fn t_marker_carries_tool_number() {
        let toks = vec![cmd(b'T', 2, 1, Params::default())];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Marker {
                kind: MotionMarkerKind::T,
                tool: Some(2),
                ..
            } => {}
            other => panic!("expected T Marker with tool=2, got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g2_emits_curve_rational_quadratic_clockwise() {
        // Quarter-circle from (1, 0, 0) to (0, 1, 0), center (0, 0, 0), CW (G2).
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 0.0), (b'F', 1500.0)])),
            cmd(
                b'G',
                2,
                2,
                p(&[(b'X', 0.0), (b'Y', 1.0), (b'I', -1.0), (b'J', 0.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::RationalQuadratic { cps, weights },
                line_no: 2,
                ..
            } => {
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-9;
                // Tangent intersection of (1,0)→(0,1) on unit circle = (1,1).
                assert!(approx(cps[0][0], 1.0) && approx(cps[0][1], 0.0));
                assert!(approx(cps[1][0], 1.0) && approx(cps[1][1], 1.0));
                assert!(approx(cps[2][0], 0.0) && approx(cps[2][1], 1.0));
                // Z constant.
                for cp in cps {
                    assert!(approx(cp[2], 0.0));
                }
                // Weight middle = cos(π/4) = √½.
                assert!(approx(weights[0], 1.0));
                assert!(approx(weights[1], std::f64::consts::FRAC_1_SQRT_2));
                assert!(approx(weights[2], 1.0));
            }
            other => panic!("expected Curve(RationalQuadratic), got {other:?}"),
        }
    }

    #[test]
    fn g3_emits_curve_rational_quadratic_counter_clockwise() {
        // CCW 90° from (0, 1) to (1, 0) around (0, 0). I = -0, J = -1 makes the
        // center at (0, 0) starting from (0, 1).
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 0.0), (b'Y', 1.0), (b'F', 1500.0)])),
            cmd(
                b'G',
                3,
                2,
                p(&[(b'X', 1.0), (b'Y', 0.0), (b'I', 0.0), (b'J', -1.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::RationalQuadratic { cps, weights },
                ..
            } => {
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-9;
                // CCW short way from (0,1) to (1,0): tangent intersection at (1,1).
                assert!(approx(cps[0][0], 0.0) && approx(cps[0][1], 1.0));
                assert!(approx(cps[1][0], 1.0) && approx(cps[1][1], 1.0));
                assert!(approx(cps[2][0], 1.0) && approx(cps[2][1], 0.0));
                assert!(approx(weights[1], std::f64::consts::FRAC_1_SQRT_2));
            }
            other => panic!("expected Curve(RationalQuadratic), got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g2_with_z_delta_yields_z_linear_control_points() {
        // Helical arc: end Z differs from start Z.
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Z', 0.0), (b'F', 1500.0)])),
            cmd(
                b'G',
                2,
                2,
                p(&[
                    (b'X', 0.0),
                    (b'Y', 1.0),
                    (b'Z', 0.5),
                    (b'I', -1.0),
                    (b'J', 0.0),
                ]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::RationalQuadratic { cps, .. },
                ..
            } => {
                // Z linear across CPs: 0.0, 0.25, 0.5
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-9;
                assert!(approx(cps[0][2], 0.0));
                assert!(approx(cps[1][2], 0.25));
                assert!(approx(cps[2][2], 0.5));
            }
            other => panic!("expected Curve(RationalQuadratic), got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn modal_position_persists_across_g1s() {
        let toks = vec![
            cmd(b'G', 1, 1, p(&[(b'X', 1.0), (b'Y', 0.0), (b'F', 1500.0)])),
            cmd(b'G', 1, 2, p(&[(b'X', 2.0)])),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::Linear { cps },
                ..
            } => {
                assert_eq!(cps[0], [1.0, 0.0, 0.0]);
                assert_eq!(cps[1], [2.0, 0.0, 0.0]);
            }
            other => panic!("expected Curve(Linear), got {other:?}"),
        }
    }

    #[test]
    fn modal_state_plane_defaults_to_xy() {
        let st = ModalState::new();
        assert_eq!(st.active_plane, Plane::XY);
    }

    #[test]
    fn modal_state_prev_g5_pq_defaults_to_none() {
        let st = ModalState::new();
        assert_eq!(st.prev_g5_pq, None);
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
    fn g17_sets_xy_plane() {
        let mut st = ModalState::new();
        let toks = vec![cmd(b'G', 17, 1, Params::default())];
        // Drive the iterator to consume the token; we observe the side-effect
        // by re-running with a follow-up G18 and checking that G18 wins.
        let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
        assert_eq!(st.active_plane, Plane::XY);
    }

    #[test]
    fn g18_sets_xz_plane() {
        let mut st = ModalState::new();
        let toks = vec![cmd(b'G', 18, 1, Params::default())];
        let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
        assert_eq!(st.active_plane, Plane::XZ);
    }

    #[test]
    fn g19_sets_yz_plane() {
        let mut st = ModalState::new();
        let toks = vec![cmd(b'G', 19, 1, Params::default())];
        let _events: Vec<_> = reduce_with_state(&mut st, toks.into_iter().map(Ok)).collect();
        assert_eq!(st.active_plane, Plane::YZ);
    }

    #[test]
    fn plane_select_emits_no_event() {
        let toks = vec![cmd(b'G', 17, 1, Params::default())];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // Plane selects update modal state silently — they're configuration,
        // not motion, and intentionally do not produce telemetry events.
        assert!(events.is_empty(), "expected no events, got {events:?}");
    }

    #[test]
    #[allow(clippy::no_effect_underscore_binding)]
    fn curve_geom_variants_construct() {
        let _linear = CurveGeom::Linear {
            cps: [[0.0; 3], [1.0, 0.0, 0.0]],
        };
        let _quad = CurveGeom::Quadratic {
            cps: [[0.0; 3], [1.0, 1.0, 0.0], [2.0, 0.0, 0.0]],
        };
        let _ratquad = CurveGeom::RationalQuadratic {
            cps: [[1.0, 0.0, 0.0], [1.0, 1.0, 0.0], [0.0, 1.0, 0.0]],
            weights: [1.0, std::f64::consts::FRAC_1_SQRT_2, 1.0],
        };
        let _cubic = CurveGeom::Cubic {
            cps: [[0.0; 3], [1.0, 1.0, 0.0], [2.0, 1.0, 0.0], [3.0, 0.0, 0.0]],
        };
    }

    #[test]
    #[allow(clippy::no_effect_underscore_binding)]
    fn reduce_event_curve_variant_constructs() {
        let _e = ReduceEvent::Curve {
            geom: CurveGeom::Linear {
                cps: [[0.0; 3], [1.0, 0.0, 0.0]],
            },
            e_delta: Some(0.1),
            feedrate_mm_s: 100.0,
            line_no: 1,
        };
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_with_explicit_ijpq_emits_curve_cubic() {
        // Position at origin, G5 to (10, 0) with tangent params I=3, J=3, P=-3, Q=3.
        // Expected control points:
        //   P0 = (0, 0, 0)
        //   P1 = (0+3, 0+3, 0) = (3, 3, 0)
        //   P2 = (10+(-3), 0+3, 0) = (7, 3, 0)
        //   P3 = (10, 0, 0)
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Cubic { cps },
                feedrate_mm_s,
                line_no: 1,
                ..
            } => {
                assert_eq!(cps[0], [0.0, 0.0, 0.0]);
                assert_eq!(cps[1], [3.0, 3.0, 0.0]);
                assert_eq!(cps[2], [7.0, 3.0, 0.0]);
                assert_eq!(cps[3], [10.0, 0.0, 0.0]);
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9);
            }
            other => panic!("expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    fn g5_error_path_clears_prev_g5_pq() {
        // First G5 succeeds and would normally extend the chain.
        // Second G5 errors (missing P) — must clear prev_g5_pq.
        // Third G5 has no I,J — must produce G5MissingTangent
        // (proves the second G5's error cleared the chain).
        let toks = vec![
            cmd_with_minor(
                b'G',
                5,
                None,
                1,
                p(&[
                    (b'X', 10.0),
                    (b'Y', 0.0),
                    (b'I', 3.0),
                    (b'J', 3.0),
                    (b'P', -3.0),
                    (b'Q', 3.0),
                    (b'F', 1500.0),
                ]),
            ),
            // Second G5: P omitted -> G5MalformedTangent.
            cmd_with_minor(
                b'G',
                5,
                None,
                2,
                p(&[
                    (b'X', 20.0),
                    (b'Y', 0.0),
                    (b'I', 3.0),
                    (b'J', 3.0),
                    (b'Q', 3.0),
                ]),
            ),
            // Third G5: no I,J. If the second G5 didn't clear, this would
            // silently link to the *first* G5's (P, Q) — wrong. Must error.
            cmd_with_minor(
                b'G',
                5,
                None,
                3,
                p(&[(b'X', 30.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 3);
        match &events[1] {
            ReduceEvent::ParseError {
                line_no: 2,
                kind: ParseErrorKind::G5MalformedTangent,
                ..
            } => {}
            other => panic!("[1] expected G5MalformedTangent, got {other:?}"),
        }
        match &events[2] {
            ReduceEvent::ParseError {
                line_no: 3,
                kind: ParseErrorKind::G5MissingTangent,
                ..
            } => {}
            other => {
                panic!("[2] expected G5MissingTangent (error path must clear chain), got {other:?}")
            }
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_chain_implicit_tangent_from_prev_pq() {
        // Three-G5 chain. Second and third have no I,J — should default to
        // -(prev P, prev Q).
        let toks = vec![
            cmd_with_minor(
                b'G',
                5,
                None,
                1,
                p(&[
                    (b'X', 10.0),
                    (b'Y', 0.0),
                    (b'I', 3.0),
                    (b'J', 3.0),
                    (b'P', -3.0),
                    (b'Q', 3.0),
                    (b'F', 1500.0),
                ]),
            ),
            // Second G5: I,J implicit. Should be -(P,Q) of prev = (3, -3).
            cmd_with_minor(
                b'G',
                5,
                None,
                2,
                p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
            ),
            // Third G5: I,J implicit. Should be -(P,Q) of second = (2, -2).
            cmd_with_minor(
                b'G',
                5,
                None,
                3,
                p(&[(b'X', 30.0), (b'Y', 0.0), (b'P', 0.0), (b'Q', 0.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 3);

        // Second G5: P0=(10,0,0), P1=(10+3, 0+(-3), 0)=(13, -3, 0).
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::Cubic { cps },
                ..
            } => {
                assert_eq!(cps[0], [10.0, 0.0, 0.0]);
                assert_eq!(cps[1], [13.0, -3.0, 0.0]);
                assert_eq!(cps[2], [20.0 + (-2.0), 0.0 + 2.0, 0.0]);
                assert_eq!(cps[3], [20.0, 0.0, 0.0]);
            }
            other => panic!("[1] expected Curve(Cubic), got {other:?}"),
        }

        // Third G5: P0=(20,0,0), P1=(20+2, 0+(-2), 0)=(22, -2, 0).
        match &events[2] {
            ReduceEvent::Curve {
                geom: CurveGeom::Cubic { cps },
                ..
            } => {
                assert_eq!(cps[0], [20.0, 0.0, 0.0]);
                assert_eq!(cps[1], [22.0, -2.0, 0.0]);
                assert_eq!(cps[2], [30.0 + 0.0, 0.0 + 0.0, 0.0]);
                assert_eq!(cps[3], [30.0, 0.0, 0.0]);
            }
            other => panic!("[2] expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    fn g5_chain_broken_by_g1_emits_recovery() {
        // G5 → G1 (breaks chain) → G5 with no I,J → expect ParseError.
        let toks = vec![
            cmd_with_minor(
                b'G',
                5,
                None,
                1,
                p(&[
                    (b'X', 10.0),
                    (b'Y', 0.0),
                    (b'I', 3.0),
                    (b'J', 3.0),
                    (b'P', -3.0),
                    (b'Q', 3.0),
                    (b'F', 1500.0),
                ]),
            ),
            cmd(b'G', 1, 2, p(&[(b'X', 11.0), (b'Y', 0.0)])),
            cmd_with_minor(
                b'G',
                5,
                None,
                3,
                p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 3);
        match &events[2] {
            ReduceEvent::ParseError {
                line_no: 3,
                kind: ParseErrorKind::G5MissingTangent,
                ..
            } => {}
            other => panic!("[2] expected G5MissingTangent ParseError, got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_chain_preserved_by_plane_select() {
        // G5 → G17 (no motion, doesn't break chain) → G5 with no I,J → uses prev_g5_pq.
        let toks = vec![
            cmd_with_minor(
                b'G',
                5,
                None,
                1,
                p(&[
                    (b'X', 10.0),
                    (b'Y', 0.0),
                    (b'I', 3.0),
                    (b'J', 3.0),
                    (b'P', -3.0),
                    (b'Q', 3.0),
                    (b'F', 1500.0),
                ]),
            ),
            cmd(b'G', 17, 2, Params::default()),
            cmd_with_minor(
                b'G',
                5,
                None,
                3,
                p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // G17 emits no event, so we have 2 events total (the two G5s).
        assert_eq!(events.len(), 2);
        match &events[1] {
            ReduceEvent::Curve {
                geom: CurveGeom::Cubic { cps },
                ..
            } => {
                // Modal-chain implicit I,J = -(prev P, prev Q) = (3, -3).
                assert_eq!(cps[1], [13.0, -3.0, 0.0]);
            }
            other => panic!("[1] expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_chain_preserved_by_m_and_t_codes() {
        // G5 → M104 → T0 → G5 with no I,J. M and T don't move; chain intact.
        let toks = vec![
            cmd_with_minor(
                b'G',
                5,
                None,
                1,
                p(&[
                    (b'X', 10.0),
                    (b'Y', 0.0),
                    (b'I', 3.0),
                    (b'J', 3.0),
                    (b'P', -3.0),
                    (b'Q', 3.0),
                    (b'F', 1500.0),
                ]),
            ),
            cmd(b'M', 104, 2, p(&[(b'S', 210.0)])),
            cmd(b'T', 0, 3, Params::default()),
            cmd_with_minor(
                b'G',
                5,
                None,
                4,
                p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // M and T emit Marker events; G5s emit Curve events; total = 4.
        assert_eq!(events.len(), 4);
        match &events[3] {
            ReduceEvent::Curve {
                geom: CurveGeom::Cubic { cps },
                ..
            } => {
                assert_eq!(cps[1], [13.0, -3.0, 0.0]);
            }
            other => panic!("[3] expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    fn g5_chain_broken_by_g92_emits_recovery() {
        // G5 → G92 (redefines coordinate frame; clears chain per spec §3.5)
        // → G5 with no I,J → expect ParseError::G5MissingTangent.
        // (G5 → G92 → G5(no IJ) → Recovery::G5MissingTangent — derived behavior.)
        let toks = vec![
            cmd_with_minor(
                b'G',
                5,
                None,
                1,
                p(&[
                    (b'X', 10.0),
                    (b'Y', 0.0),
                    (b'I', 3.0),
                    (b'J', 3.0),
                    (b'P', -3.0),
                    (b'Q', 3.0),
                    (b'F', 1500.0),
                ]),
            ),
            // G92 redefines the current position / coordinate frame; (P, Q)
            // become semantically stale because they are deltas in the prior
            // frame. Spec §3.5 chooses to clear conservatively.
            cmd(b'G', 92, 2, p(&[(b'X', 0.0), (b'Y', 0.0)])),
            cmd_with_minor(
                b'G',
                5,
                None,
                3,
                p(&[(b'X', 20.0), (b'Y', 0.0), (b'P', -2.0), (b'Q', 2.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // The trailing G5 must produce a ParseError, not silently link to
        // the pre-G92 G5's (P, Q).
        let last = events.last().expect("expected at least one event");
        match last {
            ReduceEvent::ParseError {
                line_no: 3,
                kind: ParseErrorKind::G5MissingTangent,
                ..
            } => {}
            other => panic!(
                "expected G5MissingTangent on trailing G5 (G92 must clear chain), got {other:?}"
            ),
        }
    }

    #[test]
    fn g5_single_i_only_is_malformed() {
        // I given but J omitted — invalid.
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError {
                line_no: 1,
                kind: ParseErrorKind::G5MalformedTangent,
                ..
            } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_missing_pq_is_malformed() {
        // P,Q absent on G5 — invalid (P,Q are required on every G5 line).
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'F', 1500.0),
            ]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError {
                line_no: 1,
                kind: ParseErrorKind::G5MalformedTangent,
                ..
            } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_with_z_delta_interpolates_z_at_thirds() {
        // From (0,0,0) to (10, 0, 0.3). Expected Z at CPs: 0, 0.1, 0.2, 0.3.
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            None,
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'Z', 0.3),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'P', -3.0),
                (b'Q', 3.0),
                (b'F', 1500.0),
            ]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Cubic { cps },
                ..
            } => {
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
                assert!(approx(cps[0][2], 0.0));
                assert!(approx(cps[1][2], 0.1));
                assert!(approx(cps[2][2], 0.2));
                assert!(approx(cps[3][2], 0.3));
            }
            other => panic!("expected Curve(Cubic), got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_1_with_z_delta_interpolates_z_at_midpoint() {
        // From (0,0,0) to (10, 0, 0.4). Expected Z at the three CPs:
        //   P0.z = 0, P1.z = 0.2 (midpoint), P2.z = 0.4.
        // Spec §6.2: "G5.1 with Z delta → control-point Z values at midpoint
        // (0, dz/2, dz)." Mirrors the cubic-at-thirds test above for G5.
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            Some(1),
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'Z', 0.4),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'F', 1500.0),
            ]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Quadratic { cps },
                ..
            } => {
                let approx = |a: f64, b: f64| (a - b).abs() < 1e-12;
                assert!(approx(cps[0][2], 0.0));
                assert!(approx(cps[1][2], 0.2));
                assert!(approx(cps[2][2], 0.4));
            }
            other => panic!("expected Curve(Quadratic), got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_1_with_explicit_ij_emits_curve_quadratic() {
        // From (0,0,0) to (10,0). I=3, J=3. Expected:
        //   P0 = (0, 0, 0), P1 = (3, 3, 0), P2 = (10, 0, 0).
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            Some(1),
            1,
            p(&[
                (b'X', 10.0),
                (b'Y', 0.0),
                (b'I', 3.0),
                (b'J', 3.0),
                (b'F', 1500.0),
            ]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Quadratic { cps },
                feedrate_mm_s,
                line_no: 1,
                ..
            } => {
                assert_eq!(cps[0], [0.0, 0.0, 0.0]);
                assert_eq!(cps[1], [3.0, 3.0, 0.0]);
                assert_eq!(cps[2], [10.0, 0.0, 0.0]);
                assert!((feedrate_mm_s - 25.0).abs() < 1e-9);
            }
            other => panic!("expected Curve(Quadratic), got {other:?}"),
        }
    }

    #[test]
    fn g5_1_outside_xy_plane_emits_recovery() {
        // G18 sets XZ plane; G5.1 should error.
        let toks = vec![
            cmd(b'G', 18, 1, Params::default()),
            cmd_with_minor(
                b'G',
                5,
                Some(1),
                2,
                p(&[(b'X', 10.0), (b'Z', 1.0), (b'I', 3.0), (b'J', 3.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // G18 emits no event, so we have 1 event total (the G5.1 ParseError).
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::ParseError {
                line_no: 2,
                kind: ParseErrorKind::G5PlaneMismatch,
                text,
            } => {
                assert_eq!(text, "18", "expected active plane G-code 18, got {text:?}");
            }
            other => panic!("expected G5PlaneMismatch, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_with_both_ij_zero_is_malformed() {
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            Some(1),
            1,
            p(&[(b'X', 10.0), (b'Y', 0.0), (b'I', 0.0), (b'J', 0.0)]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError {
                kind: ParseErrorKind::G5MalformedTangent,
                ..
            } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_missing_j_is_malformed() {
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            Some(1),
            1,
            p(&[(b'X', 10.0), (b'Y', 0.0), (b'I', 3.0)]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError {
                kind: ParseErrorKind::G5MalformedTangent,
                ..
            } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_missing_i_is_malformed() {
        // J specified but I omitted — invalid (G5.1 has no modal-chain rule;
        // both I and J are required). Symmetric to g5_1_missing_j_is_malformed.
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            Some(1),
            1,
            p(&[(b'X', 10.0), (b'Y', 0.0), (b'J', 3.0)]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError {
                kind: ParseErrorKind::G5MalformedTangent,
                ..
            } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_no_ij_is_malformed() {
        // Neither I nor J — G5.1 has no modal-chain rule, so this is invalid.
        // (Per spec §6.2: "G5.1 with no I, J → Recovery::MalformedParams.
        // No modal-chain rule for G5.1.")
        let toks = vec![cmd_with_minor(
            b'G',
            5,
            Some(1),
            1,
            p(&[(b'X', 10.0), (b'Y', 0.0)]),
        )];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        match &events[0] {
            ReduceEvent::ParseError {
                kind: ParseErrorKind::G5MalformedTangent,
                ..
            } => {}
            other => panic!("expected G5MalformedTangent, got {other:?}"),
        }
    }

    #[test]
    fn g5_1_outside_g19_plane_emits_recovery() {
        // G19 sets YZ plane; G5.1 should error.
        // Symmetric to g5_1_outside_xy_plane_emits_recovery (which uses G18).
        let toks = vec![
            cmd(b'G', 19, 1, Params::default()),
            cmd_with_minor(
                b'G',
                5,
                Some(1),
                2,
                p(&[(b'Y', 10.0), (b'Z', 1.0), (b'I', 3.0), (b'J', 3.0)]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // G19 emits no event, so we have 1 event total (the G5.1 ParseError).
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::ParseError {
                line_no: 2,
                kind: ParseErrorKind::G5PlaneMismatch,
                text,
            } => {
                assert_eq!(text, "19", "expected active plane G-code 19, got {text:?}");
            }
            other => panic!("expected G5PlaneMismatch, got {other:?}"),
        }
    }

    #[test]
    #[allow(clippy::float_cmp)]
    fn g5_1_after_g18_then_g17_succeeds() {
        // G18 (sets XZ — would error if G5.1 followed) → G17 (resets to XY)
        // → G5.1 should now succeed. Asserts the plane-mismatch error path
        // is not sticky and that G17 properly resets the active plane.
        let toks = vec![
            cmd(b'G', 18, 1, Params::default()),
            cmd(b'G', 17, 2, Params::default()),
            cmd_with_minor(
                b'G',
                5,
                Some(1),
                3,
                p(&[
                    (b'X', 10.0),
                    (b'Y', 0.0),
                    (b'I', 3.0),
                    (b'J', 3.0),
                    (b'F', 1500.0),
                ]),
            ),
        ];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        // G18 and G17 emit no events; G5.1 emits one Curve(Quadratic) event.
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::Curve {
                geom: CurveGeom::Quadratic { cps },
                line_no: 3,
                ..
            } => {
                assert_eq!(cps[0], [0.0, 0.0, 0.0]);
                assert_eq!(cps[1], [3.0, 3.0, 0.0]);
                assert_eq!(cps[2], [10.0, 0.0, 0.0]);
            }
            other => panic!("expected Curve(Quadratic) after G18→G17 reset, got {other:?}"),
        }
    }

    #[test]
    fn comment_marker_layer_change_is_forwarded() {
        let toks = vec![Token::Marker {
            kind: gcode::MarkerKind::LayerChange { layer: Some(7) },
            line_no: 42,
        }];
        let events = reduce(toks.into_iter().map(Ok)).collect::<Vec<_>>();
        assert_eq!(events.len(), 1);
        match &events[0] {
            ReduceEvent::CommentMarker { kind, line_no: 42 } => match kind {
                gcode::MarkerKind::LayerChange { layer } => assert_eq!(*layer, Some(7)),
                _ => panic!("expected LayerChange"),
            },
            other => panic!("expected CommentMarker, got {other:?}"),
        }
    }
}
