//! `MessageKind` discriminants + per-message structs (spec §7).
//!
//! Each non-bootstrap message has a hand-written `Encode` and `Decode` impl.
//! Bootstrap messages (`Identify`, `IdentifyResponse`) live in
//! [`crate::bootstrap`] with a separate, fixed-forever byte layout.

use crate::codec::{
    Cursor, Decode, DecodeError, Encode, get_f32, get_i32, get_u8, get_u16, get_u32, get_u64,
    put_f32, put_i32, put_u8, put_u16, put_u32, put_u64,
};

/// Layer-4 message-type discriminants. Per spec §7.1.
///
/// Bootstrap (0x0001, 0x0002) is part of the catalog but never decoded
/// through the schema — they have a fixed-forever byte layout (spec §5).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum MessageKind {
    Identify = 0x0001,
    IdentifyResponse = 0x0002,
    LoadCurveCubic = 0x0010,
    LoadCurveResponse = 0x0011,
    PushSegment = 0x0020,
    PushSegmentResponse = 0x0021,
    ConfigureAxes = 0x0030,
    ConfigureAxesResponse = 0x0031,
    QueryRuntimeCaps = 0x0040,
    RuntimeCapsResponse = 0x0041,
    ResetCurvePool = 0x0050,
    ResetCurvePoolResponse = 0x0051,
    PushPieces = 0x0060,
    PushPiecesResponse = 0x0061,
    StatusEvent = 0x0080,
    CreditFreed = 0x0081,
    FaultEvent = 0x0082,
}

impl MessageKind {
    pub fn from_u16(v: u16) -> Option<Self> {
        Some(match v {
            0x0001 => Self::Identify,
            0x0002 => Self::IdentifyResponse,
            0x0010 => Self::LoadCurveCubic,
            0x0011 => Self::LoadCurveResponse,
            0x0020 => Self::PushSegment,
            0x0021 => Self::PushSegmentResponse,
            0x0030 => Self::ConfigureAxes,
            0x0031 => Self::ConfigureAxesResponse,
            0x0040 => Self::QueryRuntimeCaps,
            0x0041 => Self::RuntimeCapsResponse,
            0x0050 => Self::ResetCurvePool,
            0x0051 => Self::ResetCurvePoolResponse,
            0x0060 => Self::PushPieces,
            0x0061 => Self::PushPiecesResponse,
            0x0080 => Self::StatusEvent,
            0x0081 => Self::CreditFreed,
            0x0082 => Self::FaultEvent,
            _ => return None,
        })
    }

    pub fn as_u16(self) -> u16 {
        self as u16
    }

    /// True if this message is decoded via the schema. False for bootstrap
    /// (Identify / `IdentifyResponse`), which use [`crate::bootstrap`].
    pub fn is_schema_validated(self) -> bool {
        !matches!(self, Self::Identify | Self::IdentifyResponse)
    }

    /// True if this message belongs on the events channel (§7.1: tags
    /// `0x0080..=0x00BF`). False for control-channel messages (commands,
    /// responses, bootstrap).
    pub fn is_event(self) -> bool {
        let tag = self as u16;
        (0x0080..=0x00BF).contains(&tag)
    }
}

// =============================================================================
// LoadCurveCubic (0x0010) — spec §3.2 (cubic-piece wire format)
//
// Wire layout (little-endian):
//   slot_idx:    u16  (offset 0)
//   axis_idx:    u8   (offset 2)
//   piece_count: u8   (offset 3)
//   pieces:      piece_count × 20 bytes, each piece = 5 × u32 LE
//                  (bp0_bits, bp1_bits, bp2_bits, bp3_bits, duration_bits)
//
// Total body = 4 + piece_count * 20 bytes.
// The MCU dispatcher (`src/kalico_dispatch.c::handle_load_curve_cubic`)
// decodes precisely this layout — keep encoder + decoder in lock-step.
// =============================================================================

#[derive(Debug, Clone, PartialEq)]
pub struct LoadCurveCubic {
    pub slot_idx: u16,
    pub axis_idx: u8,
    pub piece_count: u8,
    /// Raw piece bytes: `piece_count * 20` bytes, each piece = 5 × u32 LE
    /// (`bp0_bits`, `bp1_bits`, `bp2_bits`, `bp3_bits`, `duration_bits`).
    pub pieces_bytes: Vec<u8>,
}

impl Encode for LoadCurveCubic {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u16(out, self.slot_idx);
        put_u8(out, self.axis_idx);
        put_u8(out, self.piece_count);
        out.extend_from_slice(&self.pieces_bytes);
    }
}

impl Decode for LoadCurveCubic {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let slot_idx = get_u16(c)?;
        let axis_idx = get_u8(c)?;
        let piece_count = get_u8(c)?;
        let pieces_len = (piece_count as usize).checked_mul(20).ok_or(
            DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(piece_count),
                available: c.remaining(),
            },
        )?;
        if pieces_len > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(piece_count),
                available: c.remaining(),
            });
        }
        let mut pieces_bytes = vec![0u8; pieces_len];
        for b in &mut pieces_bytes {
            *b = get_u8(c)?;
        }
        Ok(Self {
            slot_idx,
            axis_idx,
            piece_count,
            pieces_bytes,
        })
    }
}

// =============================================================================
// LoadCurveResponse (0x0011) — spec §7.4
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LoadCurveResponse {
    pub result: i32,
    pub curve_handle_packed: u32,
}

impl Encode for LoadCurveResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u32(out, self.curve_handle_packed);
    }
}

impl Decode for LoadCurveResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            curve_handle_packed: get_u32(c)?,
        })
    }
}

// =============================================================================
// PushSegment (0x0020) — spec §7.4
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PushSegment {
    pub id: u32,
    pub handle_x: u32,
    pub handle_y: u32,
    pub handle_z: u32,
    pub handle_e: u32,
    pub t_start: u64,
    pub t_end: u64,
    pub kinematics: u8,
    pub e_mode: u8,
    pub extrusion_ratio: f32,
}

impl Encode for PushSegment {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u32(out, self.id);
        put_u32(out, self.handle_x);
        put_u32(out, self.handle_y);
        put_u32(out, self.handle_z);
        put_u32(out, self.handle_e);
        put_u64(out, self.t_start);
        put_u64(out, self.t_end);
        put_u8(out, self.kinematics);
        put_u8(out, self.e_mode);
        put_f32(out, self.extrusion_ratio);
    }
}

impl Decode for PushSegment {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            id: get_u32(c)?,
            handle_x: get_u32(c)?,
            handle_y: get_u32(c)?,
            handle_z: get_u32(c)?,
            handle_e: get_u32(c)?,
            t_start: get_u64(c)?,
            t_end: get_u64(c)?,
            kinematics: get_u8(c)?,
            e_mode: get_u8(c)?,
            extrusion_ratio: get_f32(c)?,
        })
    }
}

// =============================================================================
// PushSegmentResponse (0x0021) — spec §7.4
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushSegmentResponse {
    pub result: i32,
    pub accepted_segment_id: u32,
    pub credit_epoch: u32,
}

impl Encode for PushSegmentResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u32(out, self.accepted_segment_id);
        put_u32(out, self.credit_epoch);
    }
}

impl Decode for PushSegmentResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            accepted_segment_id: get_u32(c)?,
            credit_epoch: get_u32(c)?,
        })
    }
}

// =============================================================================
// ConfigureAxes (0x0030)
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ConfigureAxes {
    pub kinematics: u8,
    pub present_mask: u8,
    pub awd_mask: u8,
    pub invert_mask: u8,
    pub steps_per_mm: [f32; 4],
}

impl Encode for ConfigureAxes {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.kinematics);
        put_u8(out, self.present_mask);
        put_u8(out, self.awd_mask);
        put_u8(out, self.invert_mask);
        for v in &self.steps_per_mm {
            put_f32(out, *v);
        }
    }
}

impl Decode for ConfigureAxes {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let kinematics = get_u8(c)?;
        let present_mask = get_u8(c)?;
        let awd_mask = get_u8(c)?;
        let invert_mask = get_u8(c)?;
        let steps_per_mm = [get_f32(c)?, get_f32(c)?, get_f32(c)?, get_f32(c)?];
        Ok(Self {
            kinematics,
            present_mask,
            awd_mask,
            invert_mask,
            steps_per_mm,
        })
    }
}

// =============================================================================
// ConfigureAxesResponse (0x0031)
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConfigureAxesResponse {
    pub result: i32,
}

impl Encode for ConfigureAxesResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for ConfigureAxesResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

// =============================================================================
// QueryRuntimeCaps (0x0040) — request body: empty.
// RuntimeCapsResponse (0x0041) — body layout:
//   0..2  curve_pool_n         : u16_le
//   2..4  max_pieces_per_curve : u16_le
// Total: 4 bytes.
//
// Cubic-only revision (2026-05-20 stepping redesign): the NURBS sizing fields
// (max_control_points / max_knot_vector_len / max_degree) were removed. The
// runtime now uses uniform cubic Bézier pieces; the only per-MCU sizing the
// host needs is the pool slot count and the per-curve piece ceiling.
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeCapsResponse {
    pub curve_pool_n: u16,
    pub max_pieces_per_curve: u16,
}

pub const RUNTIME_CAPS_RESPONSE_BODY_LEN: usize = 4;

impl Encode for RuntimeCapsResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u16(out, self.curve_pool_n);
        put_u16(out, self.max_pieces_per_curve);
    }
}

impl Decode for RuntimeCapsResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let curve_pool_n = get_u16(c)?;
        let max_pieces_per_curve = get_u16(c)?;
        Ok(Self {
            curve_pool_n,
            max_pieces_per_curve,
        })
    }
}

// =============================================================================
// ResetCurvePool (0x0050) — request body: empty.
// ResetCurvePoolResponse (0x0051) — body layout: result:i32_le (4 bytes).
//
// Sent by the host during `init_planner` to flush stale per-slot generation
// counters left over from a previous klippy session when the MCU was not
// power-cycled. The MCU calls `CurvePool::reset_all_retired_to_current` which
// sets `last_retired_gen = current_gen` for every slot, making every slot
// allocatable again. This fixes "slot busy" rejections (cur != last) that
// occur when the host starts a fresh `SlotPool` (gens at 0) after reconnect
// but the MCU's pool still carries session-old generations.
// =============================================================================

/// `ResetCurvePool` (0x0050) — zero-body request; no fields.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResetCurvePool;

impl Encode for ResetCurvePool {
    fn encode(&self, _out: &mut Vec<u8>) {}
}

impl Decode for ResetCurvePool {
    fn decode_from(_c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self)
    }
}

/// `ResetCurvePoolResponse` (0x0051).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResetCurvePoolResponse {
    pub result: i32,
}

impl Encode for ResetCurvePoolResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for ResetCurvePoolResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

// =============================================================================
// PushPieces (0x0060) — Host → MCU
//
// Wire layout (little-endian):
//   axis_idx:    u8  (offset 0)  — which axis ring to push to
//   piece_count: u8  (offset 1)  — number of 32-byte pieces in this message
//   pieces_bytes: piece_count × 32 bytes — raw piece data
//
// Total body = 2 + piece_count * 32 bytes.
//
// PushPiecesResponse (0x0061) — MCU → Host
//   result: i32  — 0 = OK, negative = error code
// =============================================================================

#[derive(Debug, Clone, PartialEq)]
pub struct PushPieces {
    pub axis_idx: u8,
    pub piece_count: u8,
    /// Raw piece bytes: `piece_count * 32` bytes.
    pub pieces_bytes: Vec<u8>,
}

impl Encode for PushPieces {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.axis_idx);
        put_u8(out, self.piece_count);
        out.extend_from_slice(&self.pieces_bytes);
    }
}

impl Decode for PushPieces {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let axis_idx = get_u8(c)?;
        let piece_count = get_u8(c)?;
        let pieces_len = (piece_count as usize).checked_mul(32).ok_or(
            DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(piece_count),
                available: c.remaining(),
            },
        )?;
        if pieces_len > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(piece_count),
                available: c.remaining(),
            });
        }
        let mut pieces_bytes = vec![0u8; pieces_len];
        for b in &mut pieces_bytes {
            *b = get_u8(c)?;
        }
        Ok(Self {
            axis_idx,
            piece_count,
            pieces_bytes,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushPiecesResponse {
    pub result: i32,
}

impl Encode for PushPiecesResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for PushPiecesResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

// =============================================================================
// StatusEvent (0x0080) — spec §7.4
//
// v2 (2026-05-17): added `retired_through_segment_id` as the last field so the
// 10 Hz periodic status frame is the load-bearing credit-flow signal. The
// standalone `CreditFreed` event (0x0081) is no longer required for
// correctness — the host advances its slot-pool watermark from this field on
// every status frame. CreditFreed remains in the schema as a redundant
// fast-path optimization but its loss under USB-TX congestion is no longer
// catastrophic.
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusEvent {
    pub engine_status: u8,
    pub queue_depth: u8,
    pub current_segment_id: u32,
    pub last_fault: i32,
    pub fault_detail: u32,
    pub reset_epoch: u32,
    pub retired_through_segment_id: u32,
}

impl Encode for StatusEvent {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.engine_status);
        put_u8(out, self.queue_depth);
        put_u32(out, self.current_segment_id);
        put_i32(out, self.last_fault);
        put_u32(out, self.fault_detail);
        put_u32(out, self.reset_epoch);
        put_u32(out, self.retired_through_segment_id);
    }
}

impl Decode for StatusEvent {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            engine_status: get_u8(c)?,
            queue_depth: get_u8(c)?,
            current_segment_id: get_u32(c)?,
            last_fault: get_i32(c)?,
            fault_detail: get_u32(c)?,
            reset_epoch: get_u32(c)?,
            retired_through_segment_id: get_u32(c)?,
        })
    }
}

// =============================================================================
// CreditFreed (0x0081) — spec §7.4
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CreditFreed {
    pub retired_through_segment_id: u32,
    pub free_slots: u8,
}

impl Encode for CreditFreed {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u32(out, self.retired_through_segment_id);
        put_u8(out, self.free_slots);
    }
}

impl Decode for CreditFreed {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            retired_through_segment_id: get_u32(c)?,
            free_slots: get_u8(c)?,
        })
    }
}

// =============================================================================
// FaultEvent (0x0082) — spec §7.4
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultEvent {
    pub fault_code: u16,
    pub fault_detail: u32,
    pub segment_id: u32,
}

impl Encode for FaultEvent {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u16(out, self.fault_code);
        put_u32(out, self.fault_detail);
        put_u32(out, self.segment_id);
    }
}

impl Decode for FaultEvent {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            fault_code: get_u16(c)?,
            fault_detail: get_u32(c)?,
            segment_id: get_u32(c)?,
        })
    }
}

#[cfg(test)]
mod tests;
