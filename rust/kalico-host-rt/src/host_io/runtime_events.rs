use std::time::Instant;

use crate::transport::MessageParams;

#[derive(Debug, Clone)]
pub struct CreditFreedEvent {
    pub retired_through_segment_id: u32,
    pub free_slots: u8,
}

#[derive(Debug, Clone)]
pub struct FaultEvent {
    pub fault_code: u16,
    pub fault_detail: u32,
    pub segment_id: u32,
    pub synthesized: bool,
}

#[derive(Debug, Clone, Default)]
pub struct StatusEvent {
    pub engine_status: u8,
    pub queue_depth: u8,
    pub current_segment_id: u32,
    pub last_fault: u16,
    pub fault_detail: u32,
    pub retired_through_segment_id: u32,
}

#[derive(Debug, Clone)]
pub struct TraceEvent {
    pub count: u32,
    pub data: Vec<u8>,
    pub flags: u32,
}

#[derive(Debug, Clone)]
pub struct McuLogEvent {
    pub mcu_tick: u64,
    pub level: u8,
    pub subsystem: u8,
    pub event: u16,
    pub code: u16,
    pub seq: u16,
    pub args: [u32; 2],
    pub host_recv: Instant,
}

#[derive(Debug, Clone)]
pub enum RuntimeEvent {
    CreditFreed(CreditFreedEvent),
    Fault(FaultEvent),
    Status(StatusEvent),
    Trace(TraceEvent),
    EndstopTripped(()),
    McuLog(McuLogEvent),
    Heartbeat { retired_counts: Vec<u32> },
    UnknownOutput { format: String, msg: String },
    PassthroughResponse { name: String, params: MessageParams },
}

impl RuntimeEvent {
    pub fn lift(name: &str, params: MessageParams) -> Self {
        match name {
            "kalico_credit_freed" => Self::CreditFreed(CreditFreedEvent {
                retired_through_segment_id: params.get_u32("retired_through_segment_id"),
                free_slots: params.get_u32("free_slots") as u8,
            }),
            "kalico_fault" => Self::Fault(FaultEvent {
                fault_code: params.get_u32("fault_code") as u16,
                fault_detail: params.get_u32("fault_detail"),
                segment_id: params.get_u32("segment_id"),
                synthesized: false,
            }),
            "kalico_status_v6" => Self::Status(StatusEvent {
                engine_status: params.get_u32("engine_status") as u8,
                queue_depth: params.get_u32("queue_depth") as u8,
                current_segment_id: params.get_u32("current_segment_id"),
                last_fault: params.get_u32("last_fault") as u16,
                fault_detail: params.get_u32("fault_detail"),
                retired_through_segment_id: params.get_u32("retired_through_segment_id"),
            }),
            "kalico_trace" => Self::Trace(TraceEvent {
                count: params.get_u32("count"),
                data: params
                    .get_bytes("data")
                    .map(<[u8]>::to_vec)
                    .unwrap_or_default(),
                flags: 0,
            }),
            "kalico_endstop_tripped" => Self::EndstopTripped(()),
            _ => {
                let msg = params.try_get_str("#msg").unwrap_or("").to_string();
                let format = params
                    .try_get_str("#format")
                    .map(str::to_string)
                    .unwrap_or_else(|| name.to_string());
                Self::UnknownOutput { format, msg }
            }
        }
    }
}

#[cfg(test)]
mod lift_tests;
