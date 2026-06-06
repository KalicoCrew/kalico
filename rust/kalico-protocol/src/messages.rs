use crate::codec::{
    Cursor, Decode, DecodeError, Encode, get_f32, get_i32, get_u8, get_u16, get_u32, get_u64,
    put_f32, put_i32, put_u8, put_u16, put_u32, put_u64,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum MessageKind {
    Identify = 0x0001,
    IdentifyResponse = 0x0002,
    ConfigureAxes = 0x0030,
    ConfigureAxesResponse = 0x0031,
    QueryRuntimeCaps = 0x0040,
    RuntimeCapsResponse = 0x0041,
    ClaimHandshake = 0x0042,
    ClaimHandshakeReply = 0x0043,
    PushPieces = 0x0060,
    PushPiecesResponse = 0x0061,
    SetTorque = 0x0070,
    SetTorqueResponse = 0x0071,
    FaultEvent = 0x0082,
    StatusHeartbeat = 0x0083,
    McuLog = 0x0084,
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
            0x0042 => Self::ClaimHandshake,
            0x0043 => Self::ClaimHandshakeReply,
            0x0060 => Self::PushPieces,
            0x0061 => Self::PushPiecesResponse,
            0x0070 => Self::SetTorque,
            0x0071 => Self::SetTorqueResponse,
            0x0082 => Self::FaultEvent,
            0x0083 => Self::StatusHeartbeat,
            0x0084 => Self::McuLog,
            _ => return None,
        })
    }

    pub fn as_u16(self) -> u16 {
        self as u16
    }

    pub fn is_schema_validated(self) -> bool {
        !matches!(self, Self::Identify | Self::IdentifyResponse)
    }

    pub fn is_event(self) -> bool {
        let tag = self as u16;
        (0x0080..=0x00BF).contains(&tag)
    }
}

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

// PushPieces (0x0060) body: axis_idx:u8, piece_count:u8, start_slot:u16, new_head:u32, pieces_bytes[piece_count*32]
// PushPiecesResponse (0x0061) body: result:i32, arrival_clock:u64, front_start_time:u64  (20 bytes)
#[derive(Debug, Clone, PartialEq)]
pub struct PushPieces {
    pub axis_idx: u8,
    pub piece_count: u8,
    pub start_slot: u16,
    pub new_head: u32,
    pub pieces_bytes: Vec<u8>,
}

impl Encode for PushPieces {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.axis_idx);
        put_u8(out, self.piece_count);
        put_u16(out, self.start_slot);
        put_u32(out, self.new_head);
        out.extend_from_slice(&self.pieces_bytes);
    }
}

impl Decode for PushPieces {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let axis_idx = get_u8(c)?;
        let piece_count = get_u8(c)?;
        let start_slot = get_u16(c)?;
        let new_head = get_u32(c)?;
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
            start_slot,
            new_head,
            pieces_bytes,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PushPiecesResponse {
    pub result: i32,
    pub arrival_clock: u64,
    pub front_start_time: u64,
}

impl Encode for PushPiecesResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
        put_u64(out, self.arrival_clock);
        put_u64(out, self.front_start_time);
    }
}

impl Decode for PushPiecesResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
            arrival_clock: get_u64(c)?,
            front_start_time: get_u64(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetTorque {
    pub value: u8,
    pub execute_at_ns: u64,
}

impl Encode for SetTorque {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.value);
        put_u64(out, self.execute_at_ns);
    }
}

impl Decode for SetTorque {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            value: get_u8(c)?,
            execute_at_ns: get_u64(c)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SetTorqueResponse {
    pub result: i32,
}

impl Encode for SetTorqueResponse {
    fn encode(&self, out: &mut Vec<u8>) {
        put_i32(out, self.result);
    }
}

impl Decode for SetTorqueResponse {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            result: get_i32(c)?,
        })
    }
}

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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct StatusHeartbeat {
    pub engine_state: u8,
    pub fault_code: u8,
    pub retired_counts: Vec<u32>,
}

impl Encode for StatusHeartbeat {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.engine_state);
        put_u8(out, self.fault_code);
        let num_axes = self.retired_counts.len() as u8;
        put_u8(out, num_axes);
        for &count in &self.retired_counts {
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
        let mut retired_counts = Vec::with_capacity(num_axes as usize);
        for _ in 0..num_axes {
            retired_counts.push(get_u32(c)?);
        }
        Ok(Self {
            engine_state,
            fault_code,
            retired_counts,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuLog {
    pub mcu_tick: u64,
    pub level: u8,
    pub subsystem: u8,
    pub event: u16,
    pub code: u16,
    pub seq: u16,
    pub args: [u32; 2],
}

impl Encode for McuLog {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u64(out, self.mcu_tick);
        put_u8(out, self.level);
        put_u8(out, self.subsystem);
        put_u16(out, self.event);
        put_u16(out, self.code);
        put_u16(out, self.seq);
        put_u32(out, self.args[0]);
        put_u32(out, self.args[1]);
    }
}

impl Decode for McuLog {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        Ok(Self {
            mcu_tick: get_u64(c)?,
            level: get_u8(c)?,
            subsystem: get_u8(c)?,
            event: get_u16(c)?,
            code: get_u16(c)?,
            seq: get_u16(c)?,
            args: [get_u32(c)?, get_u32(c)?],
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
/// State of a single EtherCAT slave as reported in [`ClaimHandshakeReply`].
pub enum SlaveState {
    Ok = 0x00,
    Offline = 0x01,
    Fault = 0x02,
}

impl SlaveState {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::Ok),
            0x01 => Some(Self::Offline),
            0x02 => Some(Self::Fault),
            _ => None,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SlaveStatus {
    pub slave_idx: u8,
    pub state: SlaveState,
    pub fault_code: u16,
}

/// Sent once by the endpoint after bringup, before entering the DC loop.
/// Contains one entry per slave. Fail loudly: empty slave list or unknown
/// state byte are hard `DecodeError`s.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ClaimHandshakeReply {
    pub slave_statuses: Vec<SlaveStatus>,
}

impl Encode for ClaimHandshakeReply {
    fn encode(&self, out: &mut Vec<u8>) {
        put_u8(out, self.slave_statuses.len() as u8);
        for s in &self.slave_statuses {
            put_u8(out, s.slave_idx);
            put_u8(out, s.state as u8);
            put_u16(out, s.fault_code);
        }
    }
}

impl Decode for ClaimHandshakeReply {
    fn decode_from(c: &mut Cursor<'_>) -> Result<Self, DecodeError> {
        let count = get_u8(c)?;
        if count == 0 {
            return Err(DecodeError::EmptyArray {
                field: "slave_statuses",
            });
        }
        let entries_len =
            (count as usize)
                .checked_mul(4)
                .ok_or(DecodeError::ArrayLengthExceedsBuffer {
                    claimed: u32::from(count),
                    available: c.remaining(),
                })?;
        if entries_len > c.remaining() {
            return Err(DecodeError::ArrayLengthExceedsBuffer {
                claimed: u32::from(count),
                available: c.remaining(),
            });
        }
        let mut statuses = Vec::with_capacity(count as usize);
        for _ in 0..count {
            let slave_idx = get_u8(c)?;
            let state_raw = get_u8(c)?;
            let state = SlaveState::from_u8(state_raw).ok_or(DecodeError::BadDiscriminant {
                field: "SlaveState",
                raw: state_raw as u32,
            })?;
            let fault_code = get_u16(c)?;
            statuses.push(SlaveStatus {
                slave_idx,
                state,
                fault_code,
            });
        }
        Ok(Self {
            slave_statuses: statuses,
        })
    }
}

#[cfg(test)]
pub(super) fn roundtrip<T>(v: &T) -> T
where
    T: Encode + Decode + PartialEq + std::fmt::Debug,
{
    let bytes = v.encoded_to_vec();
    T::decode(&bytes).expect("decode ok")
}

#[cfg(test)]
mod claim_handshake_tests;
#[cfg(test)]
mod mcu_log_tests;
#[cfg(test)]
mod tests;
