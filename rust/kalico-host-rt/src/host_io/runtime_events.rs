//! Layer B structured event extension. Spec §4.8.

use crate::transport::MessageParams;

#[derive(Debug, Clone)]
pub struct CreditFreedEvent {
    pub retired_through_segment_id: u32,
    pub free_slots:                 u8,
}

#[derive(Debug, Clone)]
pub struct FaultEvent {
    pub fault_code:    u16,
    pub fault_detail:  u32,
    pub segment_id:    u32,
    pub synthesized:   bool,
}

#[derive(Debug, Clone, Default)]
pub struct StatusEvent {
    pub engine_status:       u8,
    pub current_segment_id:  u32,
    pub last_fault:          u16,
    pub fault_detail:        u32,
}

#[derive(Debug, Clone)]
pub struct TraceEvent {
    pub count: u32,
    pub data:  Vec<u8>,
    pub flags: u32,
}

#[derive(Debug, Clone)]
pub enum RuntimeEvent {
    CreditFreed(CreditFreedEvent),
    Fault(FaultEvent),
    Status(StatusEvent),
    Trace(TraceEvent),
    UnknownOutput { format: String, msg: String },
}
