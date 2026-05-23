//! Segment producer (kalico-native, Phase C-B).
//!
//! Spec §7.3 / §7.4 / §15: each push acquires credit from a local
//! [`CreditCounter`], encodes the kalico-native command, sends via
//! [`KalicoHostIo::kalico_call`], and waits on the matching response.
//! Failures roll back the credit acquisition.
//!
//! The pre-Phase-C `producer::load_curve` (begin/chunk/finalize over
//! Klipper msgproto) and `producer::push_segment` (Klipper command
//! `kalico_push_segment`) retire — the MCU no longer accepts those
//! commands (commit `0b263982d`).

use std::time::Duration;

use kalico_protocol::{
    Decode, Encode, LoadCurveCubic, LoadCurveResponse, MessageKind, PushSegment,
    PushSegmentResponse, ResetCurvePool, ResetCurvePoolResponse,
};

/// Wire-format safety ceiling for `load_curve`'s piece_count argument.
/// The authoritative per-MCU cap is `RuntimeCapsResponse.max_pieces_per_curve`.
/// This guard short-circuits obviously-malformed uploads before they hit the
/// wire. Set to 255 (max u8) because `LoadCurveCubic.piece_count` is encoded
/// as `u8` on the wire — a value of 256 would overflow to 0.
/// Callers should validate against `caps.max_pieces_per_curve` (clamped to
/// 255 by the dispatch layer) rather than this constant.
pub const MAX_PIECES_PER_CURVE: usize = 255;

use crate::credit::CreditCounter;
use crate::host_io::KalicoHostIo;
use crate::transport::TransportError;

/// Default timeout for `LoadCurveResponse` (spec §7.4). The MCU should
/// reply within microseconds; 100 ms is loose by ~3 orders of magnitude
/// and only triggers on a host-side stall or a wire fault.
/// LoadCurveCubic is a small frame — the host ships at most 16 cubic-Bezier
/// pieces (4 + 16*20 = 324 bytes payload) and the MCU deserializes,
/// validates, and slots them into the curve pool before sending back the
/// response.
///
/// On silicon this completes in well under 50 ms. Under Renode's 1 µs
/// quantum the same dispatch routinely takes 20+ s wall because the
/// simulation drops to ~0.05× wall-clock under load (the comment in
/// `tools/sim/h723_sim.resc` advertises 0.2× but heavy `command_task`
/// activity drives it lower). 30 s gives ~1.5× headroom over the worst
/// observed sim time, while still surfacing a genuinely hung MCU on
/// silicon before the slot pool's in-flight occupancy stalls the planner.
pub const DEFAULT_LOAD_CURVE_TIMEOUT: Duration = Duration::from_millis(300_000);

/// Default timeout for `PushSegmentResponse`.
pub const DEFAULT_PUSH_RESPONSE_TIMEOUT: Duration = Duration::from_millis(2000);

/// Default timeout for the blocking credit acquire in `push_segment`.
/// The MCU's segment queue (`Q_N - 1 = 7`) drains as segments retire;
/// at a typical 40 kHz modulation rate with ~50 ms-per-segment moves,
/// a single retirement frees credit within ~50 ms.
///
/// 2026-05-18: bumped from 1 s to 60 s to ride out the H7 USB-CDC TX
/// wedge. Bench evidence (commit 83646bbd8): the H7 runtime engine
/// processes all segments correctly (cur=7 ret=7) but its USB-CDC TX
/// subsystem stalls for ~60 s after motion start, so the host doesn't
/// observe `retired_through_segment_id` advancing in time. With the
/// 1 s timeout the host gives up before the wedge clears; with 60 s
/// it rides through. The USB-CDC stall is the pre-existing H7 wedge
/// (project_h7_wedge_pre_existing.md); this is a host-side workaround
/// until the USB-CDC subsystem fix lands.
pub const DEFAULT_CREDIT_ACQUIRE_TIMEOUT: Duration = Duration::from_millis(60_000);

#[derive(Debug, Clone, Copy)]
pub struct PushedSegmentInfo {
    pub accepted_segment_id: u32,
    pub credit_epoch: u32,
}

#[derive(Debug)]
pub enum ProducerError {
    /// Blocking credit acquire timed out — the MCU's segment queue was
    /// full for the full timeout window, indicating either a stuck MCU
    /// (engine not retiring) or a dropped `kalico_credit_freed` event.
    NoCredit,
    /// Transport-layer failure (timeout, I/O, parse).
    Transport(TransportError),
    /// MCU rejected the command (`result != 0`).
    /// The negative `i32` is the spec §9 fault-code mapping.
    McuRejected(i32),
}

impl From<TransportError> for ProducerError {
    fn from(e: TransportError) -> Self {
        ProducerError::Transport(e)
    }
}

impl std::fmt::Display for ProducerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ProducerError::NoCredit => write!(
                f,
                "producer: timed out waiting for credit (MCU queue stayed full)"
            ),
            ProducerError::Transport(e) => write!(f, "producer transport: {e}"),
            ProducerError::McuRejected(r) => {
                write!(f, "producer: MCU rejected command (result={r})")
            }
        }
    }
}

impl std::error::Error for ProducerError {}

/// Per-segment push parameters for the 4-handle wire format.
#[derive(Debug, Clone, Copy)]
pub struct SegmentPushParams {
    pub id: u32,
    pub x_handle_packed: u32,
    pub y_handle_packed: u32,
    pub z_handle_packed: u32,
    pub e_handle_packed: u32,
    pub t_start: u64,
    pub t_end: u64,
    pub kinematics: u8,
    pub e_mode: u8,
    pub extrusion_ratio: f32,
}

/// Push a single segment to the MCU via kalico-native PushSegment.
pub fn push_segment(
    io: &KalicoHostIo,
    credit: &CreditCounter,
    params: &SegmentPushParams,
) -> Result<PushedSegmentInfo, ProducerError> {
    push_segment_with_timeout(io, credit, params, DEFAULT_PUSH_RESPONSE_TIMEOUT)
}

pub fn push_segment_with_timeout(
    io: &KalicoHostIo,
    credit: &CreditCounter,
    params: &SegmentPushParams,
    timeout: Duration,
) -> Result<PushedSegmentInfo, ProducerError> {
    // Spec §5 back-pressure: the MCU's segment queue is capped at
    // `Q_N - 1 = 7` in flight. Under rapid `submit_move` bursts the
    // host fills the queue and the next `kalico_credit_freed` event
    // arrives with `free_slots=0`, snapping `credit.available` to zero.
    // Block until the MCU retires a segment and emits the next
    // `credit_freed` (with non-zero free slots), or until `timeout`
    // elapses. Without this wait, the planner's dispatch closure would
    // fail on the first oversubscription, store a `Dispatch` error, and
    // stop — producing the bench-observed "every other jog dropped"
    // symptom (commit history: bench session 2026-05-11).
    credit
        .acquire_blocking(DEFAULT_CREDIT_ACQUIRE_TIMEOUT)
        .map_err(|()| ProducerError::NoCredit)?;

    let body = PushSegment {
        id: params.id,
        handle_x: params.x_handle_packed,
        handle_y: params.y_handle_packed,
        handle_z: params.z_handle_packed,
        handle_e: params.e_handle_packed,
        t_start: params.t_start,
        t_end: params.t_end,
        kinematics: params.kinematics,
        e_mode: params.e_mode,
        extrusion_ratio: params.extrusion_ratio,
    }
    .encoded_to_vec();

    let (kind, resp_body) = match io.kalico_call(MessageKind::PushSegment, body, timeout) {
        Ok(r) => r,
        Err(e) => {
            credit.release();
            return Err(ProducerError::Transport(e));
        }
    };
    if kind != MessageKind::PushSegmentResponse {
        credit.release();
        return Err(ProducerError::Transport(TransportError::Parse(format!(
            "expected PushSegmentResponse, got 0x{:04x}",
            kind.as_u16()
        ))));
    }
    let resp = match PushSegmentResponse::decode(&resp_body) {
        Ok(r) => r,
        Err(e) => {
            credit.release();
            return Err(ProducerError::Transport(TransportError::Parse(format!(
                "PushSegmentResponse decode failed: {e:?}"
            ))));
        }
    };
    if resp.result != 0 {
        credit.release();
        return Err(ProducerError::McuRejected(resp.result));
    }
    Ok(PushedSegmentInfo {
        accepted_segment_id: resp.accepted_segment_id,
        credit_epoch: resp.credit_epoch,
    })
}

/// Parameters for loading a single per-axis cubic-Bezier curve into an MCU's
/// curve pool.
///
/// Lays out the post-stepping-redesign-finish LoadCurveCubic body (spec
/// §3.2): `slot_idx: u16, axis_idx: u8, piece_count: u8, pieces[..]: 20 bytes
/// each` (4 Bernstein control points + duration, all f32 LE bit patterns).
///
/// Each piece carries its own `duration_s` in seconds; the firmware walks
/// the piece array via (curve_handle, piece_cursor) and uses per-piece
/// duration directly — no need to ship a knot vector or normalize-to-[0,1].
#[derive(Debug, Clone)]
pub struct CurveLoadParams {
    /// Per-piece Bernstein control points (length = `piece_count`, ≤ `MAX_PIECES_PER_CURVE`).
    pub bp_per_piece: Vec<[f32; 4]>,
    /// Per-piece duration in seconds (length matches `bp_per_piece`).
    pub duration_per_piece: Vec<f32>,
}

impl CurveLoadParams {
    /// Construct from a host-side cubic NURBS via `nurbs::bezier::extract_bezier_pieces`,
    /// scaling each piece's knot domain `[u_start, u_end]` (in seconds, host
    /// time) to a per-piece duration in seconds.
    ///
    /// `t_start_s` / `t_end_s` are unused here — the input curve already
    /// carries the absolute-time domain in its knot vector — but are
    /// accepted for symmetry with the legacy `from_scalar_nurbs_normalized`
    /// signature so dispatch-side callsites don't need to change shape.
    pub fn from_scalar_nurbs_normalized(
        curve: &nurbs::ScalarNurbs<f64>,
        _t_start_s: f64,
        _t_end_s: f64,
    ) -> Self {
        let pieces = nurbs::bezier::extract_bezier_pieces(curve);
        let mut bp_per_piece: Vec<[f32; 4]> = Vec::with_capacity(pieces.len());
        let mut duration_per_piece: Vec<f32> = Vec::with_capacity(pieces.len());
        for piece in &pieces {
            // Cubic invariant — Bernstein basis with 4 control points.
            // Higher-degree input would be a planner bug at this point in
            // the pipeline (refit guarantees cubic). Pad / truncate
            // defensively so an out-of-spec input still produces a
            // well-formed wire frame rather than panicking inside the
            // dispatch closure.
            let bern = piece.to_bernstein();
            let mut bp = [0.0_f32; 4];
            for k in 0..4.min(bern.len()) {
                bp[k] = bern[k] as f32;
            }
            // If degree < 3, hold the last CP (constant tail).
            if bern.len() < 4 && !bern.is_empty() {
                let last = bern[bern.len() - 1] as f32;
                for k in bern.len()..4 {
                    bp[k] = last;
                }
            }
            bp_per_piece.push(bp);
            let dur = (piece.u_end - piece.u_start) as f32;
            duration_per_piece.push(dur);
        }
        Self {
            bp_per_piece,
            duration_per_piece,
        }
    }

    /// Number of cubic-Bezier pieces in this curve.
    pub fn piece_count(&self) -> usize {
        self.bp_per_piece.len()
    }
}

/// Lay out the per-piece body (`piece_count * 20` bytes, little-endian).
///
/// Per-piece layout: `bp0:u32 | bp1:u32 | bp2:u32 | bp3:u32 | duration:u32`,
/// each u32 is an f32 bit pattern. Matches
/// `kalico_protocol::messages::LoadCurveCubic` and the firmware-side
/// `runtime::cubic_curve::populate_from_wire`.
fn build_pieces_wire_bytes(bp: &[[f32; 4]], durs: &[f32]) -> Vec<u8> {
    debug_assert_eq!(bp.len(), durs.len());
    let mut out = Vec::with_capacity(bp.len() * 20);
    for i in 0..bp.len() {
        for &cp in &bp[i] {
            out.extend_from_slice(&cp.to_bits().to_le_bytes());
        }
        out.extend_from_slice(&durs[i].to_bits().to_le_bytes());
    }
    out
}

/// Load a per-axis cubic-Bezier curve into the MCU's curve pool at the
/// caller-specified slot. Single-frame, atomic — the entire piece array
/// ships in one `LoadCurveCubic` (msg type 0x0010) and the MCU responds
/// with `LoadCurveResponse` carrying the packed handle.
///
/// On success returns the packed handle `(generation << 16) | slot_idx`.
/// On `result != 0` returns [`ProducerError::McuRejected`]. Host-side
/// rejection (piece count out of range) surfaces as
/// `ProducerError::Transport(TransportError::Parse(...))` to keep the
/// error surface single-channel for dispatch.
///
/// Spec: `docs/superpowers/specs/2026-05-20-stepping-redesign-finish-design.md` §3.2.
pub fn load_curve(
    io: &KalicoHostIo,
    slot: u16,
    axis_idx: u8,
    params: &CurveLoadParams,
    timeout: Duration,
) -> Result<u32, ProducerError> {
    let piece_count = params.piece_count();
    if piece_count == 0 || piece_count > MAX_PIECES_PER_CURVE {
        return Err(ProducerError::Transport(TransportError::Parse(format!(
            "load_curve: piece_count {piece_count} out of range [1, {MAX_PIECES_PER_CURVE}]"
        ))));
    }
    let body = LoadCurveCubic {
        slot_idx: slot,
        axis_idx,
        piece_count: piece_count as u8,
        pieces_bytes: build_pieces_wire_bytes(&params.bp_per_piece, &params.duration_per_piece),
    }
    .encoded_to_vec();

    eprintln!(
        "[host] producer::load_curve calling kalico_call (slot={slot}, axis={axis_idx}, pieces={piece_count}, body_len={})",
        body.len()
    );
    let (kind, resp_body) = io.kalico_call(MessageKind::LoadCurveCubic, body, timeout)?;
    eprintln!(
        "[host] producer::load_curve got response kind=0x{:04x}",
        kind.as_u16()
    );
    if kind != MessageKind::LoadCurveResponse {
        return Err(ProducerError::Transport(TransportError::Parse(format!(
            "expected LoadCurveResponse, got 0x{:04x}",
            kind.as_u16()
        ))));
    }
    let resp = LoadCurveResponse::decode(&resp_body).map_err(|e| {
        ProducerError::Transport(TransportError::Parse(format!(
            "LoadCurveResponse decode failed: {e:?}"
        )))
    })?;
    if resp.result != 0 {
        // Diagnostic 2026-05-12: when the MCU rejects load_curve with
        // INVALID_HANDLE due to SlotAlreadyLoaded, the FFI side encodes the
        // slot's (current_gen, last_retired_gen) into `curve_handle_packed`
        // as (kind << 30) | (cur << 16) | last.
        let diag_kind = (resp.curve_handle_packed >> 30) & 0x3;
        let mcu_cur = ((resp.curve_handle_packed >> 16) & 0xFFFF) as u16;
        let mcu_last = (resp.curve_handle_packed & 0xFFFF) as u16;
        let diag_detail = match diag_kind {
            1 => format!("slot busy: mcu_cur_gen={mcu_cur} mcu_last_retired_gen={mcu_last}"),
            2 => {
                let reason = resp.curve_handle_packed & 0x3FFF_FFFF;
                let reason_name = match reason {
                    1 => "piece_count out of range",
                    2 => "non-finite Bernstein control point",
                    3 => "non-positive duration",
                    _ => "unknown validation reason",
                };
                format!("invalid payload: {reason_name} ({reason})")
            }
            3 => {
                let requested = resp.curve_handle_packed & 0xFFFF;
                format!("slot out of bounds: requested={requested}")
            }
            _ => format!(
                "legacy/no diagnostic: packed=0x{:08x}",
                resp.curve_handle_packed
            ),
        };
        eprintln!(
            "[host] producer::load_curve rejected slot={} axis={} result={} diag_kind={} {}",
            slot, axis_idx, resp.result, diag_kind, diag_detail,
        );
        return Err(ProducerError::Transport(TransportError::Parse(format!(
            "MCU rejected LoadCurveCubic result={} diag_kind={} {}",
            resp.result, diag_kind, diag_detail
        ))));
    }
    Ok(resp.curve_handle_packed)
}

/// Default timeout for `ResetCurvePoolResponse`.
///
/// The MCU iterates over all pool slots (at most 64) with two atomic stores
/// each — sub-microsecond on silicon. 500 ms is generous.
pub const DEFAULT_RESET_CURVE_POOL_TIMEOUT: Duration = Duration::from_millis(500);

/// Flush stale generation counters from the MCU's curve pool.
///
/// Must be called during `init_planner` (after attach but before any
/// `load_curve`) whenever the MCU has not been power-cycled between klippy
/// sessions. Without this, the MCU's `CurvePool` slots may have
/// `current_gen != last_retired_gen` (curves loaded in the prior session
/// that were never retired because klippy died), causing every subsequent
/// `load_curve` to fail with "slot busy".
///
/// The MCU handler calls `CurvePool::reset_all_retired_to_current`, which
/// sets `last_retired_gen = current_gen` for every slot. After that, every
/// slot satisfies the alloc predicate (`cur == last`) and the next
/// `try_alloc_and_load` succeeds. The MCU still uses its pre-existing
/// `current_gen` values; the handle the MCU returns in the
/// `LoadCurveResponse` is always authoritative — the host's own generation
/// counter in `SlotPool` is advisory diagnostics only and is never sent to
/// the MCU.
pub fn reset_curve_pool(io: &KalicoHostIo, timeout: Duration) -> Result<(), ProducerError> {
    let body = ResetCurvePool.encoded_to_vec();
    let (kind, resp_body) = io.kalico_call(MessageKind::ResetCurvePool, body, timeout)?;
    if kind != MessageKind::ResetCurvePoolResponse {
        return Err(ProducerError::Transport(TransportError::Parse(format!(
            "reset_curve_pool: expected ResetCurvePoolResponse, got 0x{:04x}",
            kind.as_u16()
        ))));
    }
    let resp = ResetCurvePoolResponse::decode(&resp_body).map_err(|e| {
        ProducerError::Transport(TransportError::Parse(format!(
            "ResetCurvePoolResponse decode failed: {e:?}"
        )))
    })?;
    if resp.result != 0 {
        return Err(ProducerError::McuRejected(resp.result));
    }
    Ok(())
}
