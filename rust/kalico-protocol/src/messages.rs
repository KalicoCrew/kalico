//! MessageKind discriminants + per-message structs (spec §7).
//!
//! Each non-bootstrap message has a hand-written `Encode` and `Decode` impl.
//! Bootstrap messages (`Identify`, `IdentifyResponse`) live in
//! [`crate::bootstrap`] with a separate, fixed-forever byte layout.

use crate::codec::{
    Cursor, Decode, DecodeError, Encode, get_f32, get_f32_array, get_i32, get_u16, get_u32,
    get_u64, get_u8, put_f32, put_f32_array, put_i32, put_u16, put_u32, put_u64, put_u8,
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
    LoadCurve = 0x0010,
    LoadCurveResponse = 0x0011,
    PushSegment = 0x0020,
    PushSegmentResponse = 0x0021,
    StatusEvent = 0x0080,
    CreditFreed = 0x0081,
    FaultEvent = 0x0082,
}

impl MessageKind {
    pub fn from_u16(v: u16) -> Option<Self> {
        Some(match v {
            0x0001 => Self::Identify,
            0x0002 => Self::IdentifyResponse,
            0x0010 => Self::LoadCurve,
            0x0011 => Self::LoadCurveResponse,
            0x0020 => Self::PushSegment,
            0x0021 => Self::PushSegmentResponse,
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
// LoadCurve (0x0010) — spec §7.3
// =============================================================================

#[derive(Debug, Clone, PartialEq)]
pub struct LoadCurve {
    pub slot: u16,
    pub degree: u8,
    pub cps: Vec<f32>,
    pub knots: Vec<f32>,
}

impl Encode for LoadCurve {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u16(out, self.slot);
        put_u8(out, self.degree);
        // Per spec §7.3 the body lists `n_cps` and `n_knots` as separate
        // u32 fields *before* the array data. Our `put_f32_array` writes a
        // u32_le length prefix then the elements — matching the spec when
        // the two arrays appear in declaration order: cps-len, knots-len
        // would NOT be packed up front per the §7 rule "structs packed in
        // declaration order with no padding". Reading §7.3 strictly, the
        // explicit `n_cps` and `n_knots` fields ARE the length prefixes
        // that get_f32_array consumes — we keep them adjacent to their
        // arrays for canonical encoding.
        put_f32_array(out, &self.cps);
        put_f32_array(out, &self.knots);
    }
}

impl Decode for LoadCurve {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let slot = get_u16(c)?;
        let degree = get_u8(c)?;
        let cps = get_f32_array(c)?;
        let knots = get_f32_array(c)?;
        Ok(Self { slot, degree, cps, knots })
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
// StatusEvent (0x0080) — spec §7.4
// =============================================================================

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusEvent {
    pub engine_status: u8,
    pub queue_depth: u8,
    pub current_segment_id: u32,
    pub last_fault: i32,
    pub fault_detail: u32,
    pub reset_epoch: u32,
}

impl Encode for StatusEvent {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.engine_status);
        put_u8(out, self.queue_depth);
        put_u32(out, self.current_segment_id);
        put_i32(out, self.last_fault);
        put_u32(out, self.fault_detail);
        put_u32(out, self.reset_epoch);
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
            MessageKind::LoadCurve,
            MessageKind::LoadCurveResponse,
            MessageKind::PushSegment,
            MessageKind::PushSegmentResponse,
            MessageKind::StatusEvent,
            MessageKind::CreditFreed,
            MessageKind::FaultEvent,
        ] {
            assert_eq!(MessageKind::from_u16(k.as_u16()), Some(k));
        }
        assert_eq!(MessageKind::from_u16(0xFFFF), None);
    }

    #[test]
    fn load_curve_roundtrip_realistic() {
        // Spec §7.3 worst case: degree 9, large arrays.
        let degree = 9u8;
        let n_cps = 200usize;
        let n_knots = n_cps + degree as usize + 1; // = 210
        // Deterministic pseudo-random fill via a simple LCG so the test is
        // reproducible without pulling in `rand`.
        let mut state: u32 = 0xC0FFEEEE;
        let next = |s: &mut u32| -> f32 {
            *s = s.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            // Map to [-1000.0, 1000.0).
            ((*s as f32) / (u32::MAX as f32)) * 2000.0 - 1000.0
        };
        let cps: Vec<f32> = (0..n_cps).map(|_| next(&mut state)).collect();
        let knots: Vec<f32> = (0..n_knots).map(|_| next(&mut state)).collect();
        let msg = LoadCurve { slot: 7, degree, cps: cps.clone(), knots: knots.clone() };
        let got = roundtrip(&msg);
        assert_eq!(got.slot, 7);
        assert_eq!(got.degree, 9);
        assert_eq!(got.cps, cps);
        assert_eq!(got.knots, knots);

        // Encoded body size matches §7.3: 2 + 1 + 4 + 4 + 4*n_cps + 4*n_knots.
        let bytes = msg.encoded_to_vec();
        assert_eq!(bytes.len(), 2 + 1 + 4 + 4 + 4 * n_cps + 4 * n_knots);
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
    fn status_event_roundtrip() {
        let v = StatusEvent {
            engine_status: 2,
            queue_depth: 7,
            current_segment_id: 999,
            last_fault: -42,
            fault_detail: 0xCAFE,
            reset_epoch: 0x1234_5678,
        };
        assert_eq!(roundtrip(&v), v);
        assert_eq!(v.encoded_to_vec().len(), 18);
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
    fn decode_rejects_truncated_load_curve() {
        // First 6 bytes of a valid encoding, then EOF.
        let v = LoadCurve { slot: 1, degree: 3, cps: vec![1.0, 2.0], knots: vec![0.0] };
        let bytes = v.encoded_to_vec();
        let truncated = &bytes[..6];
        assert!(matches!(
            LoadCurve::decode(truncated),
            Err(DecodeError::UnexpectedEof | DecodeError::ArrayLengthExceedsBuffer { .. })
        ));
    }

    #[test]
    fn decode_rejects_oversized_array_claim() {
        // slot u16 = 0, degree u8 = 0, n_cps u32 = 0xFFFF_FFFF, no payload.
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&0u16.to_le_bytes());
        bytes.push(0);
        bytes.extend_from_slice(&0xFFFF_FFFFu32.to_le_bytes());
        match LoadCurve::decode(&bytes) {
            Err(DecodeError::ArrayLengthExceedsBuffer { claimed, .. }) => {
                assert_eq!(claimed, 0xFFFF_FFFF);
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
