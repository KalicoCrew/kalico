//! `MessageKind` discriminants + per-message structs (spec §7).
//!
//! Each non-bootstrap message has a hand-written `Encode` and `Decode` impl.
//! Bootstrap messages (`Identify`, `IdentifyResponse`) live in
//! [`crate::bootstrap`] with a separate, fixed-forever byte layout.

use crate::codec::{
    Cursor, Decode, DecodeError, Encode, get_f32, get_i32, get_u8, get_u16, get_u32, put_f32,
    put_i32, put_u8, put_u16, put_u32,
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
    ConfigureAxes = 0x0030,
    ConfigureAxesResponse = 0x0031,
    QueryRuntimeCaps = 0x0040,
    RuntimeCapsResponse = 0x0041,
    PushPieces = 0x0060,
    PushPiecesResponse = 0x0061,
    FaultEvent = 0x0082,
    StatusHeartbeat = 0x0083,
}

impl MessageKind {
    pub fn from_u16(v: u16) -> Option<Self> {
        Some(match v {
            0x0001 => Self::Identify,
            0x0002 => Self::IdentifyResponse,
            0x0030 => Self::ConfigureAxes,
            0x0031 => Self::ConfigureAxesResponse,
            0x0040 => Self::QueryRuntimeCaps,
            0x0041 => Self::RuntimeCapsResponse,
            0x0060 => Self::PushPieces,
            0x0061 => Self::PushPiecesResponse,
            0x0082 => Self::FaultEvent,
            0x0083 => Self::StatusHeartbeat,
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
//   0..4  total_piece_memory : u32_le
// Total: 4 bytes.
//
// Simple-MCU-contract revision (2026-05-28): replaced the two-field layout
// (curve_pool_n: u16, max_pieces_per_curve: u16) with a single
// total_piece_memory: u32 representing the total bytes available for piece
// storage across all per-axis rings on the MCU. The host derives per-axis
// budgets from this figure at init_planner time.
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeCapsResponse {
    pub total_piece_memory: u32,
}

pub const RUNTIME_CAPS_RESPONSE_BODY_LEN: usize = 4;

impl Encode for RuntimeCapsResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u32(out, self.total_piece_memory);
    }
}

impl Decode for RuntimeCapsResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            total_piece_memory: get_u32(c)?,
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

// =============================================================================
// StatusHeartbeat (0x0083) — MCU → Host periodic status event.
//
// Wire layout (little-endian):
//   engine_state:    u8
//   fault_code:      u8
//   num_axes:        u8  — length of the consumed_counts array that follows
//   consumed_counts: num_axes × u32_le
//
// Total body = 3 + num_axes * 4 bytes.
//
// Sent by the MCU at the heartbeat rate (typically 10 Hz) so the host can
// track per-axis piece consumption without a separate query round-trip.
// =============================================================================

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatusHeartbeat {
    pub engine_state: u8,
    pub fault_code: u8,
    /// Per-axis consumed piece counts, one entry per configured axis.
    pub consumed_counts: Vec<u32>,
}

impl Encode for StatusHeartbeat {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.engine_state);
        put_u8(out, self.fault_code);
        let num_axes = self.consumed_counts.len() as u8;
        put_u8(out, num_axes);
        for &count in &self.consumed_counts {
            put_u32(out, count);
        }
    }
}

impl Decode for StatusHeartbeat {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let engine_state = get_u8(c)?;
        let fault_code = get_u8(c)?;
        let num_axes = get_u8(c)?;
        let counts_len =
            (num_axes as usize)
                .checked_mul(4)
                .ok_or(DecodeError::ArrayLengthExceedsBuffer {
                    claimed: u32::from(num_axes),
                    available: c.remaining(),
                })?;
        if counts_len > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(num_axes),
                available: c.remaining(),
            });
        }
        let mut consumed_counts = Vec::with_capacity(num_axes as usize);
        for _ in 0..num_axes {
            consumed_counts.push(get_u32(c)?);
        }
        Ok(Self {
            engine_state,
            fault_code,
            consumed_counts,
        })
    }
}

#[cfg(test)]
mod tests;
