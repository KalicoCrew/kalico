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

impl RuntimeEvent {
    pub fn lift(name: &str, params: MessageParams) -> Self {
        match name {
            "kalico_credit_freed" => Self::CreditFreed(CreditFreedEvent {
                retired_through_segment_id: params.get_u32("retired_through_segment_id"),
                free_slots: params.get_u32("free_slots") as u8,
            }),
            "kalico_fault" => Self::Fault(FaultEvent {
                fault_code:   params.get_u32("fault_code") as u16,
                fault_detail: params.get_u32("fault_detail"),
                segment_id:   params.get_u32("segment_id"),
                synthesized:  false,
            }),
            "kalico_status_v6" => Self::Status(StatusEvent {
                engine_status:      params.get_u32("engine_status") as u8,
                current_segment_id: params.get_u32("current_segment_id"),
                last_fault:         params.get_u32("last_fault") as u16,
                fault_detail:       params.get_u32("fault_detail"),
            }),
            "kalico_trace" => Self::Trace(TraceEvent {
                count: params.get_u32("count"),
                data:  params.get_bytes("data").map(<[u8]>::to_vec).unwrap_or_default(),
                flags: 0,
            }),
            _ => {
                let msg = params.try_get_str("#msg").unwrap_or("").to_string();
                Self::UnknownOutput { format: name.to_string(), msg }
            }
        }
    }
}
