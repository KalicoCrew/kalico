use core::time::Duration;
use std::fmt;

use crate::host_io::parser::FieldValue;
use crate::transport::{MessageParams, Transport, TransportError};

pub const SOURCE_RECORD_LEN: usize = 11;
pub const STEPPER_RECORD_LEN: usize = 1;
pub const MAX_SOURCES: usize = 4;
pub const MAX_STEPPERS: usize = 8;

pub const FMT_VERSION_V1: u8 = 1;

pub const DEFAULT_ARM_TIMEOUT: Duration = Duration::from_millis(100);
pub const DEFAULT_DISARM_TIMEOUT: Duration = Duration::from_millis(100);

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum SourceKind {
    Physical = 0,
    TmcDiag = 1,
    Software = 2,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ArmPolicy {
    TripImmediately = 0,
    WaitForClear = 1,
    IgnoreUntilMoving = 2,
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub struct SourceSpec {
    pub kind: SourceKind,
    pub gpio: u16,
    pub active_high: bool,
    pub policy: ArmPolicy,
    pub sample_n: u8,
    pub velocity_axis: u8,
    pub v_min_q16: u32,
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum ArmStatus {
    Armed = 0,
    AlreadyTripped = 1,
    Rejected = 2,
}

impl ArmStatus {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(ArmStatus::Armed),
            1 => Some(ArmStatus::AlreadyTripped),
            2 => Some(ArmStatus::Rejected),
            _ => None,
        }
    }
}

#[repr(u8)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum DisarmStatus {
    Disarmed = 0,
    AlreadyTripped = 1,
    Unknown = 2,
}

impl DisarmStatus {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(DisarmStatus::Disarmed),
            1 => Some(DisarmStatus::AlreadyTripped),
            2 => Some(DisarmStatus::Unknown),
            _ => None,
        }
    }
}

#[derive(Debug)]
pub enum EndstopError {
    Transport(TransportError),
    McuRejected(i32),
    MissingField(&'static str),
    InvalidStatus(u8),
    MalformedTripEvent(&'static str),
    TooManySources(usize),
    TooManySteppers(usize),
    UnsupportedFmtVersion(u8),
}

impl fmt::Display for EndstopError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Transport(e) => write!(f, "transport error: {e:?}"),
            Self::McuRejected(r) => write!(f, "MCU rejected command (result={r})"),
            Self::MissingField(name) => write!(f, "response missing field: {name}"),
            Self::InvalidStatus(b) => write!(f, "invalid status byte: {b}"),
            Self::MalformedTripEvent(reason) => {
                write!(f, "malformed trip-event payload: {reason}")
            }
            Self::TooManySources(n) => write!(f, "too many sources ({n} > {MAX_SOURCES} max)"),
            Self::TooManySteppers(n) => {
                write!(f, "too many steppers ({n} > {MAX_STEPPERS} max)")
            }
            Self::UnsupportedFmtVersion(v) => {
                write!(f, "unsupported trip-event fmt_version: {v}")
            }
        }
    }
}

impl std::error::Error for EndstopError {}

impl From<TransportError> for EndstopError {
    fn from(e: TransportError) -> Self {
        Self::Transport(e)
    }
}

pub fn encode_sources(sources: &[SourceSpec]) -> Result<Vec<u8>, EndstopError> {
    if sources.len() > MAX_SOURCES {
        return Err(EndstopError::TooManySources(sources.len()));
    }
    let mut buf = Vec::with_capacity(sources.len() * SOURCE_RECORD_LEN);
    for s in sources {
        buf.push(s.kind as u8);
        buf.extend_from_slice(&s.gpio.to_le_bytes());
        buf.push(if s.active_high { 1 } else { 0 });
        buf.push(s.policy as u8);
        buf.push(s.sample_n);
        buf.push(s.velocity_axis);
        buf.extend_from_slice(&s.v_min_q16.to_le_bytes());
    }
    Ok(buf)
}

pub fn encode_steppers(stepper_oids: &[u8]) -> Result<Vec<u8>, EndstopError> {
    if stepper_oids.len() > MAX_STEPPERS {
        return Err(EndstopError::TooManySteppers(stepper_oids.len()));
    }
    Ok(stepper_oids.to_vec())
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TripEventV1 {
    pub arm_id: u32,
    pub trip_clock: u64,
    pub trip_source_idx: u8,
    pub steppers: Vec<TripStepperRecord>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TripStepperRecord {
    pub oid: u8,
    pub step_count: i32,
}

pub fn decode_trip_event(params: &MessageParams) -> Result<TripEventV1, EndstopError> {
    let arm_id = params
        .try_get_u32("arm_id")
        .ok_or(EndstopError::MissingField("arm_id"))?;
    let lo = params
        .try_get_u32("trip_clock_lo")
        .ok_or(EndstopError::MissingField("trip_clock_lo"))?;
    let hi = params
        .try_get_u32("trip_clock_hi")
        .ok_or(EndstopError::MissingField("trip_clock_hi"))?;
    let trip_clock = (u64::from(hi) << 32) | u64::from(lo);
    let trip_source_idx = params
        .try_get_u32("trip_source_idx")
        .ok_or(EndstopError::MissingField("trip_source_idx"))? as u8;
    let fmt_version = params
        .try_get_u32("fmt_version")
        .ok_or(EndstopError::MissingField("fmt_version"))? as u8;
    if fmt_version != FMT_VERSION_V1 {
        return Err(EndstopError::UnsupportedFmtVersion(fmt_version));
    }
    let stepper_count = params
        .try_get_u32("stepper_count")
        .ok_or(EndstopError::MissingField("stepper_count"))? as u8;
    let blob = params
        .get_bytes("stepper_data")
        .ok_or(EndstopError::MissingField("stepper_data"))?;
    let expected_len = usize::from(stepper_count) * 5;
    if blob.len() != expected_len {
        return Err(EndstopError::MalformedTripEvent(
            "stepper_data length mismatch",
        ));
    }
    let mut steppers = Vec::with_capacity(usize::from(stepper_count));
    for i in 0..usize::from(stepper_count) {
        let off = i * 5;
        let oid = blob[off];
        let step_count =
            i32::from_le_bytes([blob[off + 1], blob[off + 2], blob[off + 3], blob[off + 4]]);
        steppers.push(TripStepperRecord { oid, step_count });
    }
    Ok(TripEventV1 {
        arm_id,
        trip_clock,
        trip_source_idx,
        steppers,
    })
}

pub fn arm_endstop<T: Transport>(
    io: &T,
    arm_id: u32,
    arm_clock: u64,
    sources: &[SourceSpec],
    stepper_oids: &[u8],
) -> Result<ArmStatus, EndstopError> {
    arm_endstop_with_timeout(
        io,
        arm_id,
        arm_clock,
        sources,
        stepper_oids,
        DEFAULT_ARM_TIMEOUT,
    )
}

pub fn arm_endstop_with_timeout<T: Transport>(
    io: &T,
    arm_id: u32,
    arm_clock: u64,
    sources: &[SourceSpec],
    stepper_oids: &[u8],
    timeout: Duration,
) -> Result<ArmStatus, EndstopError> {
    let sources_buf = encode_sources(sources)?;
    let steppers_buf = encode_steppers(stepper_oids)?;
    let resp = io.call_typed(
        "runtime_arm_endstop",
        &[
            ("arm_id", FieldValue::U32(arm_id)),
            ("arm_clock_lo", FieldValue::U32(arm_clock as u32)),
            ("arm_clock_hi", FieldValue::U32((arm_clock >> 32) as u32)),
            ("source_count", FieldValue::Byte(sources.len() as u8)),
            ("sources", FieldValue::Buffer(&sources_buf)),
            ("stepper_count", FieldValue::Byte(stepper_oids.len() as u8)),
            ("steppers", FieldValue::Buffer(&steppers_buf)),
        ],
        "kalico_arm_endstop_response",
        timeout,
    )?;
    let status_byte = resp
        .try_get_u32("status")
        .ok_or(EndstopError::MissingField("status"))? as u8;
    ArmStatus::from_u8(status_byte).ok_or(EndstopError::InvalidStatus(status_byte))
}

pub fn disarm_endstop<T: Transport>(io: &T, arm_id: u32) -> Result<DisarmStatus, EndstopError> {
    disarm_endstop_with_timeout(io, arm_id, DEFAULT_DISARM_TIMEOUT)
}

pub fn disarm_endstop_with_timeout<T: Transport>(
    io: &T,
    arm_id: u32,
    timeout: Duration,
) -> Result<DisarmStatus, EndstopError> {
    let resp = io.call_typed(
        "runtime_disarm_endstop",
        &[("arm_id", FieldValue::U32(arm_id))],
        "kalico_disarm_endstop_response",
        timeout,
    )?;
    let status_byte = resp
        .try_get_u32("status")
        .ok_or(EndstopError::MissingField("status"))? as u8;
    DisarmStatus::from_u8(status_byte).ok_or(EndstopError::InvalidStatus(status_byte))
}

#[cfg(test)]
mod tests;
