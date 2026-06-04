use gcode::{MarkerKind, ParseError, Token};

#[allow(dead_code)]
fn f_to_mm_s(f: f64) -> f64 {
    f / 60.0
}

#[derive(Debug, Clone)]
#[allow(dead_code)]
pub(crate) struct ModalState {
    pub position: [f64; 3],
    pub e: f64,
    pub feedrate_mm_s: Option<f64>,
    pub tool: u32,
    pub active_plane: Plane,
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

#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub(crate) enum CurveGeom {
    Quadratic { cps: [[f64; 3]; 3] },
    Cubic { cps: [[f64; 3]; 4] },
}

#[derive(Debug, Clone, PartialEq)]
#[allow(dead_code)]
pub(crate) enum ReduceEvent {
    Curve {
        geom: CurveGeom,
        e_delta: Option<f64>,
        feedrate_mm_s: f64,
        line_no: u32,
    },
    Marker {
        kind: MotionMarkerKind,
        line_no: u32,
        tool: Option<u32>,
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
    G0,
    ZOnly,
    EOnly,
    G92,
    M,
    T,
    EndOfFile,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ParseErrorKind {
    MalformedNumber,
    UnrecognizedHead,
    EmptyCommand,
    DuplicateParam,
    G5MissingTangent,
    // active_plane G-code number is encoded in `text`; pipeline parses it back to populate Recovery.
    G5PlaneMismatch,
    G5MalformedTangent,
    UnsupportedGcode { kind: &'static str },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub(crate) enum Plane {
    #[default]
    XY,
    XZ,
    YZ,
}

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
            Token::Command {
                letter: b'G',
                major: 5,
                minor: None,
                params,
                line_no,
                ..
            } => {
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
                        return Some(ReduceEvent::ParseError {
                            line_no,
                            kind: ParseErrorKind::G5MalformedTangent,
                            text: format!(
                                "G5: I and J must both be specified or both omitted (i_present={i_present}, j_present={j_present})"
                            ),
                        });
                    }
                };

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

                let new_x = params.x().unwrap_or(p0[0]);
                let new_y = params.y().unwrap_or(p0[1]);
                let new_z = params.z().unwrap_or(p0[2]);
                let p3 = [new_x, new_y, new_z];

                let dz = p3[2] - p0[2];
                let p1 = [p0[0] + i, p0[1] + j, p0[2] + dz / 3.0];
                let p2 = [p3[0] + pp, p3[1] + qq, p0[2] + 2.0 * dz / 3.0];

                if let Some(f) = params.f() {
                    state.feedrate_mm_s = Some(f_to_mm_s(f));
                }
                let e_delta = params.e().map(|new_e| {
                    let d = new_e - state.e;
                    state.e = new_e;
                    d
                });

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
            Token::Command {
                letter: b'G',
                major: 5,
                minor: Some(1),
                params,
                line_no,
                ..
            } => {
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

                let (i, j) = match (params.i(), params.j()) {
                    (Some(i), Some(j)) if i != 0.0 || j != 0.0 => (i, j),
                    (Some(_), Some(_)) => {
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
