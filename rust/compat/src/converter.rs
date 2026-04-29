//! Main converter pipeline: G-code text in, G5-only G-code text out.
//!
//! Orchestrates all the compat-layer building blocks (modal state, run
//! buffering, collinear/degree-elevation/arc/fitter reductions) into a single
//! `convert()` entry point that accepts raw G-code text and produces a
//! complete G5-only output string.

use std::fmt::{self, Write as FmtWrite};
use std::io::Cursor;

use gcode::{MarkerKind, Token};

use crate::arc::{self, ArcParams};
use crate::collinear::to_collinear_g5;
use crate::corner::{detect_corners, split_at_corners};
use crate::degree_elev::elevate_g51_to_g5;
use crate::emit::{write_preamble, G5Line};
use crate::fitter::fit_subrun;
use crate::g5_canon::canonicalize_g5;
use crate::modal::{ModalState, Plane};
use crate::run::{Run, Waypoint};

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Errors produced by the converter pipeline.
#[derive(Debug)]
pub enum ConvertError {
    /// A fatal, unrecoverable error (bad input that cannot be worked around).
    Fatal(String),
}

impl fmt::Display for ConvertError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ConvertError::Fatal(msg) => write!(f, "fatal: {msg}"),
        }
    }
}

impl std::error::Error for ConvertError {}

// ---------------------------------------------------------------------------
// Converter state (bundles the mutable context threaded through helpers)
// ---------------------------------------------------------------------------

struct Ctx {
    state: ModalState,
    run_buffer: Option<Run>,
    last_emitted_f: Option<f64>,
    out: Vec<u8>,
    tolerance_mm: f64,
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Convert a complete G-code file to G5-only output.
///
/// `input` is the raw G-code text. `input_name` is a human-readable label
/// (e.g. the filename) for the preamble header. `tolerance_um` is the
/// arc-to-Bezier tolerance in micrometres.
///
/// Returns the full output G-code as a `String`, or a `ConvertError` on
/// unrecoverable input.
pub fn convert(input: &str, input_name: &str, tolerance_um: f64) -> Result<String, ConvertError> {
    let tolerance_mm = tolerance_um / 1000.0;

    let mut ctx = Ctx {
        state: ModalState::new(),
        run_buffer: None,
        last_emitted_f: None,
        out: Vec::new(),
        tolerance_mm,
    };

    // Write preamble.
    {
        let mut cursor = Cursor::new(&mut ctx.out);
        write_preamble(&mut cursor, input_name, tolerance_um)
            .map_err(|e| ConvertError::Fatal(format!("write preamble: {e}")))?;
    }

    let tokens: Vec<Result<Token, gcode::ParseError>> = gcode::lex(input).collect();

    for (idx, token) in tokens.iter().enumerate() {
        match token {
            Err(e) => {
                eprintln!("kalico-compat: warning: {e}");
            }
            Ok(tok) => dispatch_token(&mut ctx, tok, &tokens, idx)?,
        }
    }

    // Flush any remaining run.
    flush_run(&mut ctx, None);

    String::from_utf8(ctx.out).map_err(|e| ConvertError::Fatal(format!("UTF-8 error: {e}")))
}

// ---------------------------------------------------------------------------
// Token dispatch
// ---------------------------------------------------------------------------

fn dispatch_token(
    ctx: &mut Ctx,
    tok: &Token,
    tokens: &[Result<Token, gcode::ParseError>],
    idx: usize,
) -> Result<(), ConvertError> {
    match tok {
        Token::Comment { text, .. } => {
            flush_run(ctx, None);
            writeln_out(&mut ctx.out, &format!("; {text}"));
        }
        Token::Marker { kind, .. } => {
            flush_run(ctx, None);
            writeln_out(&mut ctx.out, &reconstruct_marker(kind));
        }
        Token::Command {
            letter,
            major,
            minor,
            params,
            line_no,
        } => dispatch_command(ctx, *letter, *major, *minor, params, *line_no, (tokens, idx))?,
        _ => {} // Token is #[non_exhaustive]
    }
    Ok(())
}

fn dispatch_command(
    ctx: &mut Ctx,
    letter: u8,
    major: u32,
    minor: Option<u32>,
    params: &gcode::Params,
    line_no: u32,
    tokens_and_idx: (&[Result<Token, gcode::ParseError>], usize),
) -> Result<(), ConvertError> {
    match letter {
        b'G' => dispatch_g_code(ctx, major, minor, params, line_no, tokens_and_idx),
        b'M' => {
            flush_run(ctx, None);
            if major == 82 && minor.is_none() {
                ctx.state.absolute_e = true;
                return Ok(());
            } else if major == 83 && minor.is_none() {
                ctx.state.absolute_e = false;
                return Ok(());
            }
            writeln_out(&mut ctx.out, &reconstruct_command(b'M', major, minor, params));
            Ok(())
        }
        b'T' => {
            flush_run(ctx, None);
            writeln_out(&mut ctx.out, &format!("T{major}"));
            Ok(())
        }
        _ => {
            flush_run(ctx, None);
            writeln_out(
                &mut ctx.out,
                &reconstruct_command(letter, major, minor, params),
            );
            Ok(())
        }
    }
}

fn dispatch_g_code(
    ctx: &mut Ctx,
    major: u32,
    minor: Option<u32>,
    params: &gcode::Params,
    line_no: u32,
    tokens_and_idx: (&[Result<Token, gcode::ParseError>], usize),
) -> Result<(), ConvertError> {
    match (major, minor) {
        (0 | 1, None) => handle_g0_g1(ctx, major, params, line_no),
        (2 | 3, None) => {
            let (tokens, idx) = tokens_and_idx;
            handle_g2_g3(ctx, major, params, line_no, tokens, idx)
        }
        (5, None) => handle_g5(ctx, params, line_no),
        (5, Some(1)) => handle_g51(ctx, params, line_no),
        (90, None) => {
            ctx.state.absolute_xyz = true;
            Ok(())
        }
        (91, None) => {
            ctx.state.absolute_xyz = false;
            Ok(())
        }
        (17, None) => {
            ctx.state.active_plane = Plane::XY;
            writeln_out(&mut ctx.out, "G17");
            Ok(())
        }
        (18, None) => {
            ctx.state.active_plane = Plane::XZ;
            Ok(())
        }
        (19, None) => {
            ctx.state.active_plane = Plane::YZ;
            Ok(())
        }
        (92, None) => {
            flush_run(ctx, None);
            if let Some(x) = params.x() {
                ctx.state.position[0] = x;
            }
            if let Some(y) = params.y() {
                ctx.state.position[1] = y;
            }
            if let Some(z) = params.z() {
                ctx.state.position[2] = z;
            }
            if let Some(e) = params.e() {
                ctx.state.input_e = e;
                ctx.state.output_e = e;
            }
            ctx.state.prev_g5_pq = None;
            writeln_out(
                &mut ctx.out,
                &reconstruct_command(b'G', 92, None, params),
            );
            Ok(())
        }
        _ => {
            flush_run(ctx, None);
            writeln_out(
                &mut ctx.out,
                &reconstruct_command(b'G', major, minor, params),
            );
            Ok(())
        }
    }
}

// ---------------------------------------------------------------------------
// Per-command handlers
// ---------------------------------------------------------------------------

fn handle_g0_g1(
    ctx: &mut Ctx,
    major: u32,
    params: &gcode::Params,
    line_no: u32,
) -> Result<(), ConvertError> {
    if let Some(f) = params.f() {
        ctx.state.feedrate_mm_min = Some(f);
    }
    let feedrate = ctx.state.feedrate_mm_min.ok_or_else(|| {
        ConvertError::Fatal(format!(
            "line {line_no}: G{major} with no feedrate established"
        ))
    })?;

    let end_pos = ctx.state.resolve_position(params.x(), params.y(), params.z());
    let resolved_e = ctx.state.resolve_input_e(params.e());

    if ctx.state.has_xy_motion(&end_pos) {
        let e_input = resolved_e.unwrap_or(ctx.state.input_e);
        let e_delta = e_input - ctx.state.input_e;
        let dx = end_pos[0] - ctx.state.position[0];
        let dy = end_pos[1] - ctx.state.position[1];
        let xy_len = (dx * dx + dy * dy).sqrt();
        let this_e_ratio = if xy_len > 1e-12 { e_delta / xy_len } else { 0.0 };

        // Check for run breaks: feedrate change or E-ratio change.
        let should_flush = ctx.run_buffer.as_ref().is_some_and(|run| {
            if (run.feedrate_mm_min - feedrate).abs() > 1e-6 {
                return true;
            }
            if let Some(run_ratio) = run.e_ratio {
                let ratio_diff = (run_ratio - this_e_ratio).abs();
                let ratio_scale = run_ratio.abs().max(this_e_ratio.abs()).max(1e-9);
                ratio_diff / ratio_scale > 0.05
            } else {
                false
            }
        });
        if should_flush {
            flush_run(ctx, None);
        }

        if ctx.run_buffer.is_none() {
            let mut run = Run::new(
                Waypoint {
                    pos: ctx.state.position,
                    input_e: ctx.state.input_e,
                    line_no,
                },
                feedrate,
            );
            run.e_ratio = Some(this_e_ratio);
            run.start_tangent = ctx.state.prev_tangent;
            ctx.run_buffer = Some(run);
        }
        ctx.run_buffer.as_mut().unwrap().push(Waypoint {
            pos: end_pos,
            input_e: e_input,
            line_no,
        });
    } else {
        // E-only or Z-only move: flush run, emit collinear.
        flush_run(ctx, None);
        let e_abs = resolved_e.unwrap_or(ctx.state.output_e);
        let f_emit = f_if_changed(feedrate, &mut ctx.last_emitted_f);
        let line = to_collinear_g5(ctx.state.position, end_pos, e_abs, f_emit);
        writeln_out(&mut ctx.out, &line.to_string());
        ctx.state.output_e = e_abs;
    }

    ctx.state.position = end_pos;
    if let Some(e) = resolved_e {
        ctx.state.input_e = e;
    }
    ctx.state.prev_g5_pq = None;
    Ok(())
}

fn handle_g2_g3(
    ctx: &mut Ctx,
    major: u32,
    params: &gcode::Params,
    line_no: u32,
    tokens: &[Result<Token, gcode::ParseError>],
    idx: usize,
) -> Result<(), ConvertError> {
    // Peek for end tangent for the run flush.
    let end_tan = peek_tangent_for_flush(&tokens[idx + 1..], &ctx.state, ctx.tolerance_mm);
    flush_run(ctx, end_tan);

    if ctx.state.active_plane != Plane::XY {
        return Err(ConvertError::Fatal(format!(
            "line {line_no}: G{major} arcs only supported in XY plane (G17)"
        )));
    }

    if let Some(f) = params.f() {
        ctx.state.feedrate_mm_min = Some(f);
    }
    let feedrate = ctx.state.feedrate_mm_min.ok_or_else(|| {
        ConvertError::Fatal(format!(
            "line {line_no}: G{major} with no feedrate established"
        ))
    })?;

    if params.r().is_some() {
        return Err(ConvertError::Fatal(format!(
            "line {line_no}: R-format arcs not supported; use I/J center offsets"
        )));
    }
    let i_val = params.i().unwrap_or(0.0);
    let j_val = params.j().unwrap_or(0.0);
    if i_val.abs() < 1e-12 && j_val.abs() < 1e-12 {
        return Err(ConvertError::Fatal(format!(
            "line {line_no}: G{major} with I=J=0 (degenerate arc)"
        )));
    }

    let center = [
        ctx.state.position[0] + i_val,
        ctx.state.position[1] + j_val,
    ];

    let end_pos = ctx.state.resolve_position(params.x(), params.y(), params.z());
    let resolved_e = ctx.state.resolve_input_e(params.e());

    // Radius consistency check.
    let r_start = ((ctx.state.position[0] - center[0]).powi(2)
        + (ctx.state.position[1] - center[1]).powi(2))
    .sqrt();
    let r_end =
        ((end_pos[0] - center[0]).powi(2) + (end_pos[1] - center[1]).powi(2)).sqrt();
    let r_avg = f64::midpoint(r_start, r_end);
    if r_avg > 1e-12 && (r_start - r_end).abs() / r_avg > 0.001 {
        eprintln!(
            "kalico-compat: warning: line {line_no}: arc radius mismatch \
             (start={r_start:.4}, end={r_end:.4}), snapping endpoint"
        );
    }

    let arc_params = ArcParams {
        start: ctx.state.position,
        end: end_pos,
        center,
        clockwise: major == 2,
        tolerance_mm: ctx.tolerance_mm,
    };

    let mut pieces = arc::arc_to_g5(&arc_params);

    // Distribute E proportionally by chord length.
    let e_delta = resolved_e.map_or(0.0, |e| e - ctx.state.input_e);
    distribute_e_by_chord(&mut pieces, ctx.state.output_e, e_delta, ctx.state.position);

    // Set F on first piece if changed.
    if let Some(first) = pieces.first_mut() {
        first.f = f_if_changed(feedrate, &mut ctx.last_emitted_f);
    }

    for line in &pieces {
        writeln_out(&mut ctx.out, &line.to_string());
    }

    // Update state.
    let end_tangent = arc::arc_endpoint_tangent(&arc_params);
    ctx.state.prev_tangent = Some(end_tangent);
    if let Some(last) = pieces.last() {
        ctx.state.output_e = last.e;
    }
    ctx.state.position = end_pos;
    if let Some(e) = resolved_e {
        ctx.state.input_e = e;
    }
    ctx.state.prev_g5_pq = None;
    Ok(())
}

fn handle_g5(
    ctx: &mut Ctx,
    params: &gcode::Params,
    line_no: u32,
) -> Result<(), ConvertError> {
    flush_run(ctx, None);

    let (ci, cj, cp, cq) = canonicalize_g5(params, ctx.state.prev_g5_pq)
        .map_err(|e| ConvertError::Fatal(format!("line {line_no}: {e}")))?;

    if let Some(f) = params.f() {
        ctx.state.feedrate_mm_min = Some(f);
    }
    if ctx.state.feedrate_mm_min.is_none() {
        return Err(ConvertError::Fatal(format!(
            "line {line_no}: G5 with no feedrate established"
        )));
    }

    let end_pos = ctx.state.resolve_position(params.x(), params.y(), params.z());
    let resolved_e = ctx.state.resolve_input_e(params.e());
    let e_abs = resolved_e.unwrap_or(ctx.state.output_e);

    let f_emit = ctx
        .state
        .feedrate_mm_min
        .and_then(|f| f_if_changed(f, &mut ctx.last_emitted_f));

    let line = G5Line {
        x: end_pos[0],
        y: end_pos[1],
        z: end_pos[2],
        i: ci,
        j: cj,
        p: cp,
        q: cq,
        e: e_abs,
        f: f_emit,
    };
    writeln_out(&mut ctx.out, &line.to_string());

    ctx.state.prev_g5_pq = Some([cp, cq]);

    // Endpoint tangent: direction from CP2 to P3 = (-p, -q) normalized.
    let tx = -cp;
    let ty = -cq;
    let tlen = tx.hypot(ty);
    if tlen > 1e-12 {
        ctx.state.prev_tangent = Some([tx / tlen, ty / tlen]);
    }

    ctx.state.position = end_pos;
    if let Some(e) = resolved_e {
        ctx.state.input_e = e;
    }
    ctx.state.output_e = e_abs;
    Ok(())
}

fn handle_g51(
    ctx: &mut Ctx,
    params: &gcode::Params,
    line_no: u32,
) -> Result<(), ConvertError> {
    flush_run(ctx, None);

    let gi = params.i().ok_or_else(|| {
        ConvertError::Fatal(format!("line {line_no}: G5.1 missing I parameter"))
    })?;
    let gj = params.j().ok_or_else(|| {
        ConvertError::Fatal(format!("line {line_no}: G5.1 missing J parameter"))
    })?;

    if let Some(f) = params.f() {
        ctx.state.feedrate_mm_min = Some(f);
    }
    if ctx.state.feedrate_mm_min.is_none() {
        return Err(ConvertError::Fatal(format!(
            "line {line_no}: G5.1 with no feedrate established"
        )));
    }

    let end_pos = ctx.state.resolve_position(params.x(), params.y(), params.z());
    let resolved_e = ctx.state.resolve_input_e(params.e());
    let e_abs = resolved_e.unwrap_or(ctx.state.output_e);

    let p0 = ctx.state.position;
    let p1 = [
        p0[0] + gi,
        p0[1] + gj,
        f64::midpoint(p0[2], end_pos[2]),
    ];
    let p2 = end_pos;

    let f_emit = ctx
        .state
        .feedrate_mm_min
        .and_then(|f| f_if_changed(f, &mut ctx.last_emitted_f));

    let line = elevate_g51_to_g5(p0, p1, p2, e_abs, f_emit);
    writeln_out(&mut ctx.out, &line.to_string());

    // Endpoint tangent: direction P1 -> P2 normalized.
    let tx = p2[0] - p1[0];
    let ty = p2[1] - p1[1];
    let tlen = tx.hypot(ty);
    if tlen > 1e-12 {
        ctx.state.prev_tangent = Some([tx / tlen, ty / tlen]);
    }

    ctx.state.position = end_pos;
    if let Some(e) = resolved_e {
        ctx.state.input_e = e;
    }
    ctx.state.output_e = e_abs;
    ctx.state.prev_g5_pq = None;
    Ok(())
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Append a line (with trailing newline) to the output buffer.
fn writeln_out(buf: &mut Vec<u8>, line: &str) {
    buf.extend_from_slice(line.as_bytes());
    buf.push(b'\n');
}

/// Return `Some(feedrate)` if `feedrate` differs from `last_emitted_f`,
/// and update the tracking variable. Returns `None` if unchanged.
fn f_if_changed(feedrate: f64, last_emitted_f: &mut Option<f64>) -> Option<f64> {
    let changed = match *last_emitted_f {
        Some(prev) => (prev - feedrate).abs() > 1e-6,
        None => true,
    };
    if changed {
        *last_emitted_f = Some(feedrate);
        Some(feedrate)
    } else {
        None
    }
}

/// Reconstruct a G-code command line from parsed components.
fn reconstruct_command(
    letter: u8,
    major: u32,
    minor: Option<u32>,
    params: &gcode::Params,
) -> String {
    let mut s = String::new();
    s.push(letter as char);
    let _ = write!(s, "{major}");
    if let Some(m) = minor {
        s.push('.');
        let _ = write!(s, "{m}");
    }
    for ch in b'A'..=b'Z' {
        if let Some(v) = params.get(ch) {
            s.push(' ');
            s.push(ch as char);
            if v.fract().abs() < 1e-10 {
                let _ = write!(s, "{}", v as i64);
            } else {
                let _ = write!(s, "{v:.5}");
            }
        }
    }
    s
}

/// Reconstruct a marker comment from its `MarkerKind`.
fn reconstruct_marker(kind: &MarkerKind) -> String {
    match kind {
        MarkerKind::LayerChange { layer: Some(n) } => format!(";LAYER:{n}"),
        MarkerKind::LayerChange { layer: None } => ";LAYER_CHANGE".to_string(),
        MarkerKind::LayerType { name } => format!(";TYPE:{name}"),
        MarkerKind::EndOfPrint => ";END_OF_PRINT".to_string(),
        _ => "; unknown marker".to_string(),
    }
}

/// Flush the current G1-run buffer, fitting or emitting collinear G5s.
fn flush_run(ctx: &mut Ctx, end_tangent: Option<[f64; 2]>) {
    let Some(mut run) = ctx.run_buffer.take() else {
        return;
    };

    run.end_tangent = end_tangent;

    if run.waypoints.len() < 2 {
        return;
    }

    let positions = run.positions();
    let total_e_delta = run.total_e_delta();
    let feedrate = run.feedrate_mm_min;

    // Detect corners and split.
    let corners = detect_corners(&positions, ctx.tolerance_mm);
    let sub_runs = split_at_corners(&positions, &corners);

    let mut all_pieces: Vec<G5Line> = Vec::new();

    for (sub_idx, sub_pts) in sub_runs.iter().enumerate() {
        if sub_pts.len() < 2 {
            continue;
        }

        let st = if sub_idx == 0 {
            run.start_tangent
        } else {
            None
        };
        let et = if sub_idx == sub_runs.len() - 1 {
            run.end_tangent
        } else {
            None
        };

        all_pieces.extend(fit_subrun(sub_pts, ctx.tolerance_mm, st, et));
    }

    if all_pieces.is_empty() {
        return;
    }

    // Distribute E across all output pieces proportional to chord length.
    distribute_e_by_chord(
        &mut all_pieces,
        ctx.state.output_e,
        total_e_delta,
        positions[0],
    );

    // Set F on the first piece if changed.
    if let Some(first) = all_pieces.first_mut() {
        first.f = f_if_changed(feedrate, &mut ctx.last_emitted_f);
    }

    for line in &all_pieces {
        writeln_out(&mut ctx.out, &line.to_string());
    }

    // Update state from the last emitted piece.
    if let Some(last) = all_pieces.last() {
        ctx.state.output_e = last.e;

        // Tangent from the last piece: (-p, -q) normalized.
        let tx = -last.p;
        let ty = -last.q;
        let tlen = tx.hypot(ty);
        if tlen > 1e-12 {
            ctx.state.prev_tangent = Some([tx / tlen, ty / tlen]);
        }
    }
}

/// Distribute E across G5 pieces proportional to chord length.
fn distribute_e_by_chord(
    pieces: &mut [G5Line],
    start_e: f64,
    e_delta: f64,
    start_pos: [f64; 3],
) {
    if pieces.is_empty() {
        return;
    }

    let mut chords = Vec::with_capacity(pieces.len());
    let mut prev = start_pos;
    for p in pieces.iter() {
        let dx = p.x - prev[0];
        let dy = p.y - prev[1];
        let dz = p.z - prev[2];
        chords.push((dx * dx + dy * dy + dz * dz).sqrt());
        prev = [p.x, p.y, p.z];
    }

    let total_chord: f64 = chords.iter().sum();
    let mut e_acc = start_e;

    if total_chord < 1e-12 {
        let per_piece = e_delta / pieces.len() as f64;
        for p in pieces.iter_mut() {
            e_acc += per_piece;
            p.e = e_acc;
        }
    } else {
        for (p, chord) in pieces.iter_mut().zip(chords.iter()) {
            e_acc += e_delta * chord / total_chord;
            p.e = e_acc;
        }
    }
}

/// Peek at the next token to extract a tangent for the run flush end-tangent.
fn peek_tangent_for_flush(
    remaining: &[Result<Token, gcode::ParseError>],
    state: &ModalState,
    tolerance_mm: f64,
) -> Option<[f64; 2]> {
    let tok = remaining.first()?;
    match tok {
        Ok(Token::Command {
            letter: b'G',
            major: major @ (2 | 3),
            minor: None,
            params,
            ..
        }) => {
            let i_val = params.i().unwrap_or(0.0);
            let j_val = params.j().unwrap_or(0.0);
            if i_val.abs() < 1e-12 && j_val.abs() < 1e-12 {
                return None;
            }
            let end_pos = state.resolve_position(params.x(), params.y(), params.z());
            let center = [state.position[0] + i_val, state.position[1] + j_val];
            let arc_params = ArcParams {
                start: state.position,
                end: end_pos,
                center,
                clockwise: *major == 2,
                tolerance_mm,
            };
            Some(arc::arc_start_tangent(&arc_params))
        }
        Ok(Token::Command {
            letter: b'G',
            major: 5,
            minor: None,
            params,
            ..
        }) => {
            let (i, j, _, _) = canonicalize_g5(params, state.prev_g5_pq).ok()?;
            let len = i.hypot(j);
            if len > 1e-12 {
                Some([i / len, j / len])
            } else {
                None
            }
        }
        _ => None,
    }
}
