//! MessageKind discriminants + per-message structs (spec §7).
//!
//! Each non-bootstrap message has a hand-written `Encode` and `Decode` impl.
//! Bootstrap messages (`Identify`, `IdentifyResponse`) live in
//! [`crate::bootstrap`] with a separate, fixed-forever byte layout.

use crate::codec::{
    Cursor, Decode, DecodeError, Encode, get_f32, get_i32, get_u16, get_u32, get_u64, get_u8,
    put_f32, put_i32, put_u16, put_u32, put_u64, put_u8,
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
    /// (Identify / IdentifyResponse), which use [`crate::bootstrap`].
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
    /// (bp0_bits, bp1_bits, bp2_bits, bp3_bits, duration_bits).
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
                claimed: piece_count as u32,
                available: c.remaining(),
            },
        )?;
        if pieces_len > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: piece_count as u32,
                available: c.remaining(),
            });
        }
        let mut pieces_bytes = vec![0u8; pieces_len];
        for b in &mut pieces_bytes {
            *b = get_u8(c)?;
        }
        Ok(Self { slot_idx, axis_idx, piece_count, pieces_bytes })
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
        Ok(Self { result: get_i32(c)?, curve_handle_packed: get_u32(c)? })
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
        Ok(Self { kinematics, present_mask, awd_mask, invert_mask, steps_per_mm })
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
        Ok(Self { result: get_i32(c)? })
    }
}

// =============================================================================
// QueryRuntimeCaps (0x0040) — request body: empty.
// RuntimeCapsResponse (0x0041) — body layout:
//   0..4   max_control_points  : u32_le
//   4..8   max_knot_vector_len : u32_le
//   8      max_degree          : u8
//   9..11  curve_pool_n        : u16_le
// Total: 11 bytes.
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeCapsResponse {
    pub max_control_points: u32,
    pub max_knot_vector_len: u32,
    pub max_degree: u8,
    pub curve_pool_n: u16,
}

pub const RUNTIME_CAPS_RESPONSE_BODY_LEN: usize = 11;

impl Encode for RuntimeCapsResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u32(out, self.max_control_points);
        put_u32(out, self.max_knot_vector_len);
        put_u8(out, self.max_degree);
        put_u16(out, self.curve_pool_n);
    }
}

impl Decode for RuntimeCapsResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let max_control_points = get_u32(c)?;
        let max_knot_vector_len = get_u32(c)?;
        let max_degree = get_u8(c)?;
        let curve_pool_n = get_u16(c)?;
        Ok(Self { max_control_points, max_knot_vector_len, max_degree, curve_pool_n })
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
mod tests {
    use super::*;

    fn roundtrip<T>(v: &T) -> T
    where
        T: Encode + Decode + PartialEq + std::fmt::Debug,
    {
        let bytes = v.encoded_to_vec();
        let decoded = T::decode(&bytes).expect("decode ok");
        decoded
    }

    #[test]
    fn message_kind_round_trips_via_u16() {
        for &k in &[
            MessageKind::Identify,
            MessageKind::IdentifyResponse,
            MessageKind::LoadCurveCubic,
            MessageKind::LoadCurveResponse,
            MessageKind::PushSegment,
            MessageKind::PushSegmentResponse,
            MessageKind::ConfigureAxes,
            MessageKind::ConfigureAxesResponse,
            MessageKind::QueryRuntimeCaps,
            MessageKind::RuntimeCapsResponse,
            MessageKind::StatusEvent,
            MessageKind::CreditFreed,
            MessageKind::FaultEvent,
        ] {
            assert_eq!(MessageKind::from_u16(k.as_u16()), Some(k));
        }
        assert_eq!(MessageKind::from_u16(0xFFFF), None);
    }

    #[test]
    fn load_curve_cubic_roundtrip_realistic() {
        // 3 cubic pieces (each 5 × u32 = 20 bytes).
        let piece_count = 3u8;
        let pieces_len = piece_count as usize * 20;
        // Deterministic pseudo-random fill via a simple LCG.
        let mut state: u32 = 0xC0FFEEEE;
        let next = |s: &mut u32| -> u8 {
            *s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (*s >> 24) as u8
        };
        let pieces_bytes: Vec<u8> = (0..pieces_len).map(|_| next(&mut state)).collect();
        let msg = LoadCurveCubic {
            slot_idx: 7,
            axis_idx: 1,
            piece_count,
            pieces_bytes: pieces_bytes.clone(),
        };
        let got = roundtrip(&msg);
        assert_eq!(got.slot_idx, 7);
        assert_eq!(got.axis_idx, 1);
        assert_eq!(got.piece_count, piece_count);
        assert_eq!(got.pieces_bytes, pieces_bytes);

        // Encoded body size: 2 (slot_idx) + 1 (axis_idx) + 1 (piece_count) + 3*20 = 64.
        let bytes = msg.encoded_to_vec();
        assert_eq!(bytes.len(), 4 + pieces_len);
    }

    #[test]
    fn load_curve_response_roundtrip() {
        let v = LoadCurveResponse { result: -3, curve_handle_packed: 0xDEAD_BEEF };
        assert_eq!(roundtrip(&v), v);
        assert_eq!(v.encoded_to_vec().len(), 8);
    }

    #[test]
    fn push_segment_roundtrip() {
        let v = PushSegment {
            id: 0x0102_0304,
            handle_x: 0x1111_2222,
            handle_y: 0x3333_4444,
            handle_z: 0x5555_6666,
            handle_e: 0x7777_8888,
            t_start: 0x0011_2233_4455_6677,
            t_end: 0x8899_AABB_CCDD_EEFF,
            kinematics: 0x05,
            e_mode: 0x01,
            extrusion_ratio: 0.0234567,
        };
        assert_eq!(roundtrip(&v), v);
        // 4*5 (ids+handles) + 8*2 (timestamps) + 1 + 1 + 4 = 42.
        assert_eq!(v.encoded_to_vec().len(), 42);
    }

    #[test]
    fn push_segment_response_roundtrip() {
        let v = PushSegmentResponse {
            result: 0,
            accepted_segment_id: 12345,
            credit_epoch: 67890,
        };
        assert_eq!(roundtrip(&v), v);
        assert_eq!(v.encoded_to_vec().len(), 12);
    }

    #[test]
    fn configure_axes_roundtrip() {
        let v = ConfigureAxes {
            kinematics: 0,
            present_mask: 0b1111,
            awd_mask: 0b0011,
            invert_mask: 0b0010,
            steps_per_mm: [80.0, 80.0, 400.0, 415.0],
        };
        assert_eq!(roundtrip(&v), v);
        // 4 (header bytes) + 16 (4×f32) = 20.
        assert_eq!(v.encoded_to_vec().len(), 20);
        let r = ConfigureAxesResponse { result: -7 };
        assert_eq!(roundtrip(&r), r);
        assert_eq!(r.encoded_to_vec().len(), 4);
    }

    #[test]
    fn status_event_roundtrip() {
        let v = StatusEvent {
            engine_status: 2,
            queue_depth: 7,
            current_segment_id: 999,
            last_fault: -42,
            fault_detail: 0xCAFE,
            reset_epoch: 0x1234_5678,
            retired_through_segment_id: 997,
        };
        assert_eq!(roundtrip(&v), v);
        // 1+1+4+4+4+4+4 = 22.
        assert_eq!(v.encoded_to_vec().len(), 22);
    }

    #[test]
    fn credit_freed_roundtrip() {
        let v = CreditFreed { retired_through_segment_id: 4242, free_slots: 14 };
        assert_eq!(roundtrip(&v), v);
        assert_eq!(v.encoded_to_vec().len(), 5);
    }

    #[test]
    fn fault_event_roundtrip() {
        let v = FaultEvent { fault_code: 0x0007, fault_detail: 0xBAAD_F00D, segment_id: 11 };
        assert_eq!(roundtrip(&v), v);
        assert_eq!(v.encoded_to_vec().len(), 10);
    }

    #[test]
    fn runtime_caps_roundtrip() {
        let original = RuntimeCapsResponse {
            max_control_points: 512,
            max_knot_vector_len: 524,
            max_degree: 10,
            curve_pool_n: 4,
        };
        let mut buf = Vec::new();
        original.encode(&mut buf);
        assert_eq!(buf.len(), RUNTIME_CAPS_RESPONSE_BODY_LEN);
        let mut c = Cursor::new(&buf);
        let decoded = RuntimeCapsResponse::decode_from(&mut c).unwrap();
        assert_eq!(decoded, original);
    }

    #[test]
    fn decode_rejects_truncated_load_curve_cubic() {
        // 3-byte truncation of the 4-byte header: triggers UnexpectedEof.
        let bytes = &[0x01u8, 0x00, 0x00]; // slot_idx LE + axis_idx, no piece_count
        assert!(matches!(
            LoadCurveCubic::decode(bytes),
            Err(DecodeError::UnexpectedEof | DecodeError::ArrayLengthExceedsBuffer { .. })
        ));
    }

    #[test]
    fn decode_rejects_oversized_piece_count() {
        // slot_idx u16 = 0, axis_idx u8 = 0, piece_count u8 = 0xFF (255 pieces = 5100
        // bytes), but no piece data follows. The decoder must reject without allocating.
        let bytes = &[0x00u8, 0x00, 0x00, 0xFF]; // header only, no pieces
        match LoadCurveCubic::decode(bytes) {
            Err(DecodeError::ArrayLengthExceedsBuffer { claimed, .. }) => {
                assert_eq!(claimed, 0xFF);
            }
            other => panic!("expected ArrayLengthExceedsBuffer, got {other:?}"),
        }
    }

    #[test]
    fn decode_rejects_trailing_bytes() {
        let v = FaultEvent { fault_code: 1, fault_detail: 2, segment_id: 3 };
        let mut bytes = v.encoded_to_vec();
        bytes.push(0xAA);
        match FaultEvent::decode(&bytes) {
            Err(DecodeError::TrailingBytes { remaining: 1 }) => {}
            other => panic!("expected TrailingBytes(1), got {other:?}"),
        }
    }
}
